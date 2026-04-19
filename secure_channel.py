"""HMAC-authenticated envelope for the brain↔kernel TCP protocol.

This is the brain-side MVP wrapper around brain protocol packets.
Pairs with `crates/behavior/src/brain_authenticated.rs` on the kernel.

## Threat model

Without this layer the brain↔kernel TCP stream is plaintext on a presumed-
trusted LAN. Any peer on the segment can:
- Forge an ESTOP (`PKT_ESTOP=0x88`) and stop every robot.
- Replay an old "FORWARD 100" actuator command indefinitely.
- Inject malformed packets to crash the brain or kernel parsers.

## Wire format

```
Offset  Size  Field
0x00    8     Nonce (monotonically-increasing u64, big-endian)
0x08    16    HMAC-SHA-256 over (nonce || len || inner) truncated to 16 B
0x18    2     Inner-packet length (LE u16)
0x1A    N     Inner brain-protocol packet (the existing framed packet)
```

26-byte overhead per packet.

## Replay protection

Receiver tracks the highest nonce ever accepted. Any incoming nonce
≤ that high-water mark is silently discarded. After a brain restart,
the kernel's high-water resets to 0; the brain must persist its send
nonce to disk to avoid reusing one. For the MVP we re-derive an
ephemeral starting nonce from current time × 1000 — collision-resistant
within reason given the 64-bit space.

## Key management

Pre-shared symmetric key (32 bytes raw) loaded from:
- env var `ROBOT_BRAIN_LINK_KEY` as 64 hex chars (32 bytes)
- failing that, the channel runs UNAUTHENTICATED (legacy mode) — log a
  warning so operators see they're insecure.

The kernel loads the same key from `/fat/LINK.KEY` (32 bytes raw).
Both files / env vars MUST contain identical bytes; otherwise every
HMAC fails and the brain is silently disconnected.

This MVP intentionally does NOT encrypt the payload — encryption
without forward secrecy is bigger than the threat actually warrants
on a private LAN, and `crates/crypto/src/secure_channel.rs` already
provides a full X25519 + AES-CTR + HMAC handshake when needed.
"""

from __future__ import annotations

import hashlib
import hmac
import os
import struct
import time
from typing import Optional


# ── Constants ─────────────────────────────────────────────────────────────

NONCE_BYTES = 8
HMAC_BYTES  = 16
LEN_BYTES   = 2
ENVELOPE_OVERHEAD = NONCE_BYTES + HMAC_BYTES + LEN_BYTES   # 26

KEY_BYTES   = 32

#: Largest inner-packet we'll wrap. Must match kernel side.
MAX_INNER_BYTES = 8 * 1024


def load_link_key() -> Optional[bytes]:
    """Load the pre-shared link key.

    Returns 32 raw bytes, or `None` if no key is configured (legacy
    plaintext mode). Logs a warning when running unauthenticated.
    """
    hex_key = os.environ.get("ROBOT_BRAIN_LINK_KEY", "").strip()
    if not hex_key:
        return None
    try:
        raw = bytes.fromhex(hex_key)
    except ValueError:
        print("[SECCHAN] ERROR: ROBOT_BRAIN_LINK_KEY is not valid hex")
        return None
    if len(raw) != KEY_BYTES:
        print(f"[SECCHAN] ERROR: link key must be {KEY_BYTES} bytes, got {len(raw)}")
        return None
    return raw


# ── Send side ─────────────────────────────────────────────────────────────

class Sender:
    """Wraps outgoing brain-protocol packets with the HMAC envelope.

    Maintains a monotonically-increasing send-nonce. The starting
    nonce comes from `time.time_ns()` so two reconnects within the
    same nanosecond can't collide; nonces are then bumped by 1 per
    packet so the receiver's "highest-seen" check works.
    """

    def __init__(self, key: bytes):
        if len(key) != KEY_BYTES:
            raise ValueError(f"link key must be {KEY_BYTES} bytes")
        self._key = key
        self._nonce = int(time.time_ns()) & 0xFFFF_FFFF_FFFF_FFFF

    def wrap(self, inner: bytes) -> bytes:
        if len(inner) > MAX_INNER_BYTES:
            raise ValueError("inner packet too large for envelope")
        self._nonce = (self._nonce + 1) & 0xFFFF_FFFF_FFFF_FFFF
        nonce_b = self._nonce.to_bytes(NONCE_BYTES, "big")
        len_b   = len(inner).to_bytes(LEN_BYTES, "little")
        # HMAC over (nonce || len || inner) — the same canonical
        # ordering the kernel side computes.
        mac = hmac.new(self._key, nonce_b + len_b + inner, hashlib.sha256).digest()[:HMAC_BYTES]
        return nonce_b + mac + len_b + inner


# ── Receive side ──────────────────────────────────────────────────────────

class Receiver:
    """Verifies incoming envelopes and drops replays.

    Tracks the highest accepted nonce. A packet whose nonce is
    ≤ that high-water mark is rejected — this is the replay defence.
    Restart resets the high-water; the sender's clock-derived starting
    nonce makes accidental collisions astronomically unlikely.
    """

    def __init__(self, key: bytes):
        if len(key) != KEY_BYTES:
            raise ValueError(f"link key must be {KEY_BYTES} bytes")
        self._key = key
        self._highest_nonce = 0

    def unwrap(self, frame: bytes) -> Optional[bytes]:
        """Returns the inner packet if the envelope is valid + non-replay,
        otherwise `None`."""
        if len(frame) < ENVELOPE_OVERHEAD:
            return None
        nonce_b = frame[:NONCE_BYTES]
        mac_b   = frame[NONCE_BYTES:NONCE_BYTES + HMAC_BYTES]
        len_b   = frame[NONCE_BYTES + HMAC_BYTES:NONCE_BYTES + HMAC_BYTES + LEN_BYTES]
        n       = int.from_bytes(len_b, "little")
        if len(frame) < ENVELOPE_OVERHEAD + n:
            return None
        if n > MAX_INNER_BYTES:
            return None
        inner = frame[ENVELOPE_OVERHEAD:ENVELOPE_OVERHEAD + n]
        expected = hmac.new(self._key, nonce_b + len_b + inner,
                            hashlib.sha256).digest()[:HMAC_BYTES]
        # Constant-time comparison.
        if not hmac.compare_digest(expected, mac_b):
            return None
        nonce = int.from_bytes(nonce_b, "big")
        # Replay defence: strictly monotonic.
        if nonce <= self._highest_nonce:
            return None
        self._highest_nonce = nonce
        return inner
