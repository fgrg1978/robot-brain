"""HMAC-authenticated envelope for the brain↔kernel TCP protocol.

This is the brain-side MVP wrapper around brain protocol packets.
Pairs with `crates/behavior/src/auth_envelope.rs` on the kernel
(byte-for-byte identical wire format).

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
0x08    16    HMAC-SHA-256 over (dir || nonce || len || inner), truncated to 16 B
0x18    2     Inner-packet length (LE u16)
0x1A    N     Inner brain-protocol packet (the existing framed packet)
```

26-byte overhead per packet.

`dir` is a 3-byte direction label (`b"C2S"` brain→kernel, `b"S2C"`
kernel→brain — see `DIR_C2S`/`DIR_S2C` below). It is bound into the MAC
but NOT transmitted: each receiver knows which direction it is reading,
and the binding stops a frame from being reflected back at its sender.
The kernel computes the identical MAC input in
`crates/behavior/src/auth_envelope.rs`.

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
HMAC_BYTES = 16
LEN_BYTES = 2
ENVELOPE_OVERHEAD = NONCE_BYTES + HMAC_BYTES + LEN_BYTES  # 26

KEY_BYTES = 32

#: Largest inner-packet we'll wrap. Must match kernel side.
MAX_INNER_BYTES = 8 * 1024

# Replay-protection tolerance window.
# REPLAY_WINDOW = 0 → strict monotonic: any nonce ≤ rx_high_water is rejected.
# Out-of-order delivery is not tolerated in Phase 1 (TCP preserves order).
# If UDP transport is ever added, bump this to a small positive value and add
# a sliding-window bitmap check.
REPLAY_WINDOW: int = 0


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


# Direction labels bound into the KDF and into the envelope MAC.
#
# Both peers used to derive ONE enc/mac pair and use it in both directions, so
# a frame this side *sent* was cryptographically valid *inbound* here: the MAC
# proved "someone holding the key sent this", not "the peer sent this".  It was
# not exploitable only because the packet-type namespaces happen to be disjoint
# (kernel->brain 0x01-0x03, brain->kernel 0x80+) — a convention holding a
# cryptographic flaw closed.
#
# The kernel is the RESPONDER and the brain is the INITIATOR, so from here:
#   send    -> C2S  (brain -> kernel)
#   receive -> S2C  (kernel -> brain)
# The kernel mirrors this in crates/crypto/src/secure_channel.rs and
# crates/behavior/src/auth_envelope.rs.  Changing either side alone breaks the
# link; see newfeatures-free note in that crate's module header.
DIR_C2S = b"C2S"   # brain -> kernel  (this side transmits)
DIR_S2C = b"S2C"   # kernel -> brain  (this side receives)

# ── Send side ─────────────────────────────────────────────────────────────


class Sender:
    """Wraps outgoing brain-protocol packets with the HMAC envelope.

    Maintains a monotonically-increasing send-nonce. The starting
    nonce comes from `time.time_ns()` so two reconnects within the
    same nanosecond can't collide; nonces are then bumped by 1 per
    packet so the receiver's "highest-seen" check works.
    """

    def __init__(self, key: bytes, direction: bytes = DIR_C2S):
        """`direction` defaults to C2S — the brain transmits toward the kernel.

        It is a parameter only so a test can build the *kernel's* Sender
        (`direction=DIR_S2C`) and feed a real cross-direction frame to this
        module's Receiver.  Production code should never pass it.
        """
        if len(key) != KEY_BYTES:
            raise ValueError(f"link key must be {KEY_BYTES} bytes")
        self._key = key
        self._direction = direction
        self._nonce = int(time.time_ns()) & 0xFFFF_FFFF_FFFF_FFFF

    def wrap(self, inner: bytes) -> bytes:
        if len(inner) > MAX_INNER_BYTES:
            raise ValueError("inner packet too large for envelope")
        self._nonce = (self._nonce + 1) & 0xFFFF_FFFF_FFFF_FFFF
        nonce_b = self._nonce.to_bytes(NONCE_BYTES, "big")
        len_b = len(inner).to_bytes(LEN_BYTES, "little")
        # HMAC over (nonce || len || inner) — the same canonical
        # ordering the kernel side computes.
        # DIR_C2S: the label is hashed but NOT transmitted — the receiver
        # knows which direction it is reading.  Mirrors the kernel's
        # auth_envelope::wrap, which binds DIR_TX = S2C on its side.
        mac = hmac.new(
            self._key, self._direction + nonce_b + len_b + inner, hashlib.sha256
        ).digest()[:HMAC_BYTES]
        return nonce_b + mac + len_b + inner


# ── Receive side ──────────────────────────────────────────────────────────


class Receiver:
    """Verifies incoming envelopes and drops replays.

    Tracks the highest accepted nonce (rx_high_water).  A packet whose nonce
    is ≤ rx_high_water is rejected — this is the strict-monotonic replay
    defence governed by REPLAY_WINDOW = 0.  Restart resets the high-water;
    the sender's clock-derived starting nonce makes accidental collisions
    astronomically unlikely.
    """

    def __init__(self, key: bytes, direction: bytes = DIR_S2C):
        """`direction` defaults to S2C — the brain receives from the kernel.

        Parameterised for the same test reason as `Sender`; production code
        should never pass it.
        """
        if len(key) != KEY_BYTES:
            raise ValueError(f"link key must be {KEY_BYTES} bytes")
        self._key = key
        self._direction = direction
        # rx_high_water: highest nonce successfully accepted on this channel.
        # Initialized to 0; first valid packet must carry nonce > 0.
        self._rx_high_water: int = REPLAY_WINDOW  # == 0 for strict monotonic

        # Forensic drop counters — incremented on each rejected packet.
        # An outer caller (e.g. server.py) can read these periodically and log
        # them at WARNING level with peer context.  Keeping the counters here
        # (rather than calling logger directly) preserves the pure-function
        # contract of unwrap() and keeps this module unit-testable without log
        # capture.
        #
        # drops_hmac:   HMAC digest mismatch → forged or corrupted packet.
        # drops_replay: valid HMAC but nonce ≤ rx_high_water → replay attempt.
        self.drops_hmac: int = 0
        self.drops_replay: int = 0

    def unwrap(self, frame: bytes) -> Optional[bytes]:
        """Returns the inner packet if the envelope is valid + non-replay,
        otherwise `None`.

        On each rejection the appropriate drop counter is incremented:
          - `drops_hmac`   for HMAC mismatches (forgery / corruption).
          - `drops_replay` for valid-HMAC but stale-nonce packets (replay).
        Malformed frames (too short, inner_len overflow) are silently dropped
        without bumping either counter — they pre-date authentication.
        """
        if len(frame) < ENVELOPE_OVERHEAD:
            return None
        nonce_b = frame[:NONCE_BYTES]
        mac_b = frame[NONCE_BYTES : NONCE_BYTES + HMAC_BYTES]
        len_b = frame[NONCE_BYTES + HMAC_BYTES : NONCE_BYTES + HMAC_BYTES + LEN_BYTES]
        n = int.from_bytes(len_b, "little")
        if len(frame) < ENVELOPE_OVERHEAD + n:
            return None
        if n > MAX_INNER_BYTES:
            return None
        inner = frame[ENVELOPE_OVERHEAD : ENVELOPE_OVERHEAD + n]
        # DIR_S2C: we only ever accept frames travelling kernel -> brain, so a
        # frame we emitted cannot be reflected back at us and verify here.
        expected = hmac.new(
            self._key, self._direction + nonce_b + len_b + inner, hashlib.sha256
        ).digest()[
            :HMAC_BYTES
        ]
        # Constant-time comparison.
        if not hmac.compare_digest(expected, mac_b):
            self.drops_hmac += 1
            return None
        nonce = int.from_bytes(nonce_b, "big")
        # Replay defence: reject any nonce ≤ rx_high_water (strict monotonic,
        # REPLAY_WINDOW = 0).  A recorded+replayed packet will always have a
        # nonce ≤ the high-water seen from the original transmission.
        if nonce <= self._rx_high_water:
            self.drops_replay += 1
            return None
        self._rx_high_water = nonce
        return inner


# ── AEAD layer (RFC-0019) ─────────────────────────────────────────────────
#
# This is an OPT-IN encrypted layer that wraps the HMAC envelope above.
# Wire format matches `crates/crypto/src/secure_channel.rs` byte-for-byte:
#
#   0x00    12   Nonce (8B random + 4B AES-CTR counter, LE)
#   0x0C    2    Encrypted-payload length N (LE u16, MAX = 2048)
#   0x0E    N    Ciphertext (AES-128-CTR, enc_key)
#   0x0E+N  32   HMAC-SHA-256 over [nonce || length || ciphertext], mac_key
#
# Keys derived from X25519 shared secret using the existing crate's KDF:
#   enc_key = SHA-256(shared_secret || "ENC")[0..16]
#   mac_key = SHA-256(shared_secret || "MAC")[0..16]
#
# The plaintext is the inner HMAC envelope frame (8B nonce + 16B HMAC +
# 2B inner-len + N inner-bytes).  Both layers' replay defences apply
# independently — see RFC-0019 § "Encrypted frame format".

import os as _os

try:
    from cryptography.hazmat.primitives.asymmetric.x25519 import (
        X25519PrivateKey,
        X25519PublicKey,
    )
    from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
    from cryptography.hazmat.primitives import serialization

    _HAS_CRYPTOGRAPHY = True
except ImportError:  # pragma: no cover — only hit if requirements not installed
    _HAS_CRYPTOGRAPHY = False


# Mirror the crate constants — see crates/crypto/src/secure_channel.rs.
AEAD_NONCE_SIZE = 12
AEAD_LEN_SIZE = 2
AEAD_HMAC_SIZE = 32
AEAD_OVERHEAD = AEAD_NONCE_SIZE + AEAD_LEN_SIZE + AEAD_HMAC_SIZE  # 46
AEAD_MAX_PAYLOAD = 2048
AEAD_KEY_SIZE = 16  # AES-128

# Counter wrap guard — same as the Rust side.  Refuse encrypt() at u32::MAX
# so a repeated (random, counter) pair cannot leak plaintext via XOR.
_AEAD_COUNTER_MAX = 0xFFFFFFFF




def _kdf(shared_secret: bytes, label: bytes, direction: bytes) -> bytes:
    """Match the crate's KDF exactly:

        SHA-256(shared_secret || label || direction)[0..16]

    `label` and `direction` are both fixed-length 3-byte ASCII, so the
    concatenation is unambiguous without a length field — same reasoning as
    the Rust `kdf()`.
    """
    return hashlib.sha256(shared_secret + label + direction).digest()[:AEAD_KEY_SIZE]


def _hmac_sha256_16k(key: bytes, data: bytes) -> bytes:
    """HMAC-SHA-256 with a 16-byte key (matches the crate's `hmac_sha256`).

    RFC 2104 uses the hash's block size (64 B for SHA-256), so a 16-byte key
    is zero-padded to 64.  Python's `hmac.new` does that automatically given
    the raw key.
    """
    return hmac.new(key, data, hashlib.sha256).digest()


class SecureChannel:
    """X25519 ECDHE + AES-128-CTR + HMAC-SHA-256 channel (RFC-0019).

    Lifecycle:

        1. Create with the long-term PSK (same as `Sender`/`Receiver`).
        2. Call `start_handshake()` to get the bytes to send to the peer.
        3. Call `complete_handshake(peer_pub, peer_proof)` with the peer's
           reply.  Returns the local proof bytes to send back (initiator)
           or `None` (responder — proof was already in the HELLO reply).
        4. After established, use `encrypt(plaintext)` / `decrypt(frame)`.

    State machine matches RFC-0019 § "Handshake — Noise_XXpsk0 shape".

    This class is **deliberately separate** from `Sender`/`Receiver` so the
    HMAC envelope can continue to operate standalone.  The wiring in
    `protocol.py` chooses one layer or the other based on
    `ROBOT_BRAIN_ENCRYPT_LINK=1`.
    """

    # Handshake states
    INIT = 0
    AWAIT_PEER = 1  # sent HELLO, waiting for peer HELLO+proof
    AWAIT_CONFIRM = 2  # responder: sent HELLO+proof, waiting for CONFIRM
    ESTABLISHED = 3
    REJECTED = 4

    # Wire label bytes (RFC-0019 § "Wire framing").
    MODE_ENCRYPTED = 0x02
    LABEL_HELLO = 0x48  # 'H'
    LABEL_CONFIRM = 0x43  # 'C'
    LABEL_REJECT = 0x52  # 'R'
    # NOTE: there is deliberately NO rekey label or counter here. The kernel
    # has no rekey counterpart, and its SecureChannel rejected an rx-counter
    # precisely because rekey-resets-counter semantics would break strict
    # monotonicity (see crates/behavior/src/auth_envelope.rs, K-C5 note).
    # Rekey must be specified in an RFC covering BOTH repos (robot-os and
    # robot-brain) before any stub is reintroduced.

    def __init__(
        self, psk: bytes, is_initiator: bool = True, *, _testing_eph_priv: Optional[bytes] = None
    ):
        if not _HAS_CRYPTOGRAPHY:
            raise RuntimeError(
                "secure_channel.SecureChannel needs the 'cryptography' package — "
                "run `pip install -r requirements.txt`"
            )
        if len(psk) != KEY_BYTES:
            raise ValueError(f"PSK must be {KEY_BYTES} bytes")
        self._psk = psk
        self._is_initiator = is_initiator
        self._state = self.INIT

        # Ephemeral keypair — regenerated per channel, zeroed after handshake.
        # The `_testing_eph_priv` parameter is for cross-side compat vectors
        # ONLY — never wire it to a non-test code path.  Production always
        # uses fresh OS-RNG randomness.
        if _testing_eph_priv is not None:
            if len(_testing_eph_priv) != 32:
                raise ValueError("testing eph priv must be 32 bytes")
            self._eph_priv = X25519PrivateKey.from_private_bytes(_testing_eph_priv)
        else:
            self._eph_priv = X25519PrivateKey.generate()
        self._eph_pub_bytes: bytes = self._eph_priv.public_key().public_bytes(
            encoding=serialization.Encoding.Raw,
            format=serialization.PublicFormat.Raw,
        )
        self._peer_pub_bytes: Optional[bytes] = None

        # Derived keys (filled in on establish).
        self._tx_enc_key: Optional[bytes] = None
        self._tx_mac_key: Optional[bytes] = None
        self._rx_enc_key: Optional[bytes] = None
        self._rx_mac_key: Optional[bytes] = None

        # AES-CTR transmit counter — monotonic for the channel's lifetime
        # (no rekey exists; exhaustion forces a reconnect, see `encrypt`).
        self._tx_counter = 0

        # Drop counters parallel to Receiver's.
        self.drops_hmac = 0
        self.drops_decrypt = 0

    # ── Handshake ──────────────────────────────────────────────────────

    def start_handshake(self) -> bytes:
        """Initiator step 1 — returns the bytes to send to the kernel:

            [MODE_ENCRYPTED][LABEL_HELLO][eph_pub 32B]

        Total 34 bytes.  Move state to AWAIT_PEER.
        """
        if self._state != self.INIT:
            raise RuntimeError(f"start_handshake from invalid state {self._state}")
        self._state = self.AWAIT_PEER
        return bytes([self.MODE_ENCRYPTED, self.LABEL_HELLO]) + self._eph_pub_bytes

    def handle_peer_hello(self, frame: bytes) -> bytes:
        """Initiator step 2/3 — process kernel's HELLO+proof, send CONFIRM.

        Expected frame: `[MODE_ENCRYPTED][LABEL_HELLO][peer_pub 32B][proof 32B]`
        (66 bytes total).  Returns the CONFIRM bytes the initiator sends back.
        """
        if self._state != self.AWAIT_PEER:
            raise RuntimeError(f"handle_peer_hello from invalid state {self._state}")
        if len(frame) != 2 + 32 + 32:
            raise ValueError("HELLO frame wrong length")
        if frame[0] != self.MODE_ENCRYPTED or frame[1] != self.LABEL_HELLO:
            raise ValueError("HELLO frame wrong header")
        peer_pub = frame[2:34]
        peer_proof = frame[34:66]

        # Verify peer (responder) proof.
        expected = _hmac_sha256_16k(
            self._psk,
            b"RESP" + self._eph_pub_bytes + peer_pub,
        )
        if not hmac.compare_digest(expected, peer_proof):
            self._state = self.REJECTED
            raise PermissionError("peer proof_k failed — possible MITM or bad PSK")

        # Derive shared secret + keys.
        self._derive_keys(peer_pub)

        # Build our (initiator) proof for the CONFIRM.
        my_proof = _hmac_sha256_16k(
            self._psk,
            b"INIT" + peer_pub + self._eph_pub_bytes,
        )
        self._state = self.ESTABLISHED
        return bytes([self.MODE_ENCRYPTED, self.LABEL_CONFIRM]) + my_proof

    def handle_initiator_hello(self, frame: bytes) -> bytes:
        """Responder step 1 — process brain's HELLO, return HELLO+proof_k.

        Expected frame: `[MODE_ENCRYPTED][LABEL_HELLO][peer_pub 32B]`
        (34 bytes total).  Returns the responder's HELLO+proof reply.
        """
        if self._state != self.INIT:
            raise RuntimeError(f"handle_initiator_hello from invalid state {self._state}")
        if len(frame) != 2 + 32:
            raise ValueError("HELLO frame wrong length")
        if frame[0] != self.MODE_ENCRYPTED or frame[1] != self.LABEL_HELLO:
            raise ValueError("HELLO frame wrong header")
        peer_pub = frame[2:34]

        # Compute our proof_k = HMAC(PSK, "RESP" || peer_pub || my_pub).
        my_proof = _hmac_sha256_16k(
            self._psk,
            b"RESP" + peer_pub + self._eph_pub_bytes,
        )
        # Derive shared secret + keys at this point — we need them for the
        # subsequent encrypt path even though we still wait for CONFIRM.
        self._derive_keys(peer_pub)
        self._state = self.AWAIT_CONFIRM
        return bytes([self.MODE_ENCRYPTED, self.LABEL_HELLO]) + self._eph_pub_bytes + my_proof

    def handle_initiator_confirm(self, frame: bytes) -> None:
        """Responder step 3 — verify the initiator's CONFIRM proof."""
        if self._state != self.AWAIT_CONFIRM:
            raise RuntimeError(f"handle_initiator_confirm from invalid state {self._state}")
        if len(frame) != 2 + 32:
            raise ValueError("CONFIRM frame wrong length")
        if frame[0] != self.MODE_ENCRYPTED or frame[1] != self.LABEL_CONFIRM:
            raise ValueError("CONFIRM frame wrong header")
        peer_proof = frame[2:34]
        # Expected = HMAC(PSK, "INIT" || my_pub || peer_pub).
        assert self._peer_pub_bytes is not None
        expected = _hmac_sha256_16k(
            self._psk,
            b"INIT" + self._eph_pub_bytes + self._peer_pub_bytes,
        )
        if not hmac.compare_digest(expected, peer_proof):
            self._state = self.REJECTED
            raise PermissionError("initiator proof_b failed — possible MITM or bad PSK")
        self._state = self.ESTABLISHED

    def _derive_keys(self, peer_pub_bytes: bytes) -> None:
        """X25519 + KDF.  Stores `enc_key`, `mac_key`, `peer_pub_bytes`."""
        self._peer_pub_bytes = peer_pub_bytes
        peer_pub = X25519PublicKey.from_public_bytes(peer_pub_bytes)
        shared = self._eph_priv.exchange(peer_pub)
        # Four keys, two directions.  This side is the initiator, so it
        # transmits on C2S and receives on S2C.  Deriving all four (rather
        # than only the pair we need) keeps this function a literal mirror of
        # the Rust `handshake_directional`, which is what makes the two
        # implementations reviewable side by side.
        enc_c2s = _kdf(shared, b"ENC", DIR_C2S)
        mac_c2s = _kdf(shared, b"MAC", DIR_C2S)
        enc_s2c = _kdf(shared, b"ENC", DIR_S2C)
        mac_s2c = _kdf(shared, b"MAC", DIR_S2C)

        # Pick the mirrored pair by ROLE, exactly as the Rust
        # `handshake_directional` does with `Role::Initiator`/`Role::Responder`.
        # In production the brain is the initiator and the kernel the
        # responder, but this class is also driven with both roles in tests, so
        # keying off `_is_initiator` — not off a hardcoded direction — is what
        # keeps the two implementations symmetric.
        if self._is_initiator:
            self._tx_enc_key, self._tx_mac_key = enc_c2s, mac_c2s
            self._rx_enc_key, self._rx_mac_key = enc_s2c, mac_s2c
        else:
            self._tx_enc_key, self._tx_mac_key = enc_s2c, mac_s2c
            self._rx_enc_key, self._rx_mac_key = enc_c2s, mac_c2s
        # Zero the private key — forward secrecy.  cryptography lib doesn't
        # expose a destructor, but dropping the reference is the best Python
        # can do; the underlying OpenSSL EVP_PKEY_free runs on GC.
        self._eph_priv = None

    @property
    def is_established(self) -> bool:
        return self._state == self.ESTABLISHED

    # ── Encrypt / decrypt ──────────────────────────────────────────────

    def encrypt(self, plaintext: bytes, *, _testing_nonce_rand: Optional[bytes] = None) -> bytes:
        """Wrap `plaintext` in an AEAD frame.

        Plaintext is the existing HMAC-envelope frame from `Sender.wrap()`.
        Returns the wire bytes the peer feeds into `decrypt()`.

        `_testing_nonce_rand` injects a fixed 8-byte prefix instead of OS
        entropy — used ONLY by the cross-side compat vectors.
        """
        if not self.is_established:
            raise RuntimeError("encrypt before handshake established")
        if len(plaintext) > AEAD_MAX_PAYLOAD:
            raise ValueError(f"plaintext too large ({len(plaintext)} > {AEAD_MAX_PAYLOAD})")
        if self._tx_counter >= _AEAD_COUNTER_MAX:
            raise RuntimeError("tx_counter exhausted — reconnect for a fresh handshake")

        # Build nonce: 8 random + 4-byte LE counter.  This matches the Rust
        # side which takes 8B of randomness from the caller; we use os.urandom
        # so the test harness can also inject deterministic bytes.
        if _testing_nonce_rand is not None:
            if len(_testing_nonce_rand) != 8:
                raise ValueError("testing nonce rand must be 8 bytes")
            nonce_rand = _testing_nonce_rand
        else:
            nonce_rand = _os.urandom(8)
        nonce = nonce_rand + self._tx_counter.to_bytes(4, "little")
        self._tx_counter += 1

        # AES-128-CTR encrypt.  The Rust crate (crates/crypto/src/aes.rs
        # `ctr_encrypt`) builds the initial counter block as
        # `nonce(12) || counter_be32(=1)` and increments by 1 per block.
        # Python's `modes.CTR` expects the full 16B initial counter block;
        # we construct it explicitly to match the Rust side byte-for-byte.
        assert self._tx_enc_key is not None
        ctr_block = nonce + (1).to_bytes(4, "big")
        cipher = Cipher(algorithms.AES(self._tx_enc_key), modes.CTR(ctr_block))
        encryptor = cipher.encryptor()
        ciphertext: bytes = encryptor.update(plaintext) + encryptor.finalize()

        # HMAC over [nonce || length || ciphertext].
        len_bytes = len(plaintext).to_bytes(AEAD_LEN_SIZE, "little")
        assert self._tx_mac_key is not None
        mac = _hmac_sha256_16k(self._tx_mac_key, nonce + len_bytes + ciphertext)
        return nonce + len_bytes + ciphertext + mac

    def decrypt(self, frame: bytes) -> Optional[bytes]:
        """Verify + decrypt a frame.  Returns plaintext or None on rejection.

        Updates `drops_hmac` / `drops_decrypt` counters as appropriate so the
        caller can log forensic state.
        """
        if not self.is_established:
            raise RuntimeError("decrypt before handshake established")
        if len(frame) < AEAD_OVERHEAD:
            return None
        nonce = frame[:AEAD_NONCE_SIZE]
        n = int.from_bytes(frame[AEAD_NONCE_SIZE : AEAD_NONCE_SIZE + AEAD_LEN_SIZE], "little")
        if n > AEAD_MAX_PAYLOAD:
            return None
        ct_start = AEAD_NONCE_SIZE + AEAD_LEN_SIZE
        ct_end = ct_start + n
        if len(frame) < ct_end + AEAD_HMAC_SIZE:
            return None
        ciphertext = frame[ct_start:ct_end]
        mac_b = frame[ct_end : ct_end + AEAD_HMAC_SIZE]

        # Verify HMAC over [nonce || length || ciphertext].
        assert self._rx_mac_key is not None
        expected = _hmac_sha256_16k(self._rx_mac_key, frame[:ct_end])
        if not hmac.compare_digest(expected, mac_b):
            self.drops_hmac += 1
            return None

        # Decrypt.  Same 16B counter-block construction as encrypt.
        assert self._rx_enc_key is not None
        ctr_block = nonce + (1).to_bytes(4, "big")
        try:
            cipher = Cipher(algorithms.AES(self._rx_enc_key), modes.CTR(ctr_block))
            decryptor = cipher.decryptor()
            plaintext: bytes = decryptor.update(ciphertext) + decryptor.finalize()
        except Exception:
            self.drops_decrypt += 1
            return None
        return plaintext
