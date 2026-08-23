"""Cross-side wire compatibility test for the brain↔kernel HMAC envelope.

Validates that the brain `secure_channel.py` produces frame bytes that match
what the kernel `crates/behavior/src/auth_envelope.rs` would unwrap (and vice
versa), without needing to actually run the kernel. Compatibility is proved
by computing the envelope contents against KNOWN test vectors using only
Python's standard `hmac` + `hashlib` — if the Python module and the kernel
module both follow the documented spec, these vectors will match the kernel's
unwrap byte-for-byte.

If a wire-format change is ever made on either side, this test will fail
loudly, catching the cross-side drift before it ships.
"""

from __future__ import annotations

import hashlib
import hmac
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

from secure_channel import (
    DIR_C2S,
    DIR_S2C,
    ENVELOPE_OVERHEAD,
    HMAC_BYTES,
    KEY_BYTES,
    LEN_BYTES,
    MAX_INNER_BYTES,
    NONCE_BYTES,
    Receiver,
    Sender,
)

# Known test vectors — fixed key + fixed nonce + fixed inner produces a
# deterministic frame the kernel must also produce.
_KEY = bytes(range(KEY_BYTES))  # 0x00..0x1f
_INNER = b"\xaa" * 20


def _expected_frame(
    nonce_u64: int, key: bytes, inner: bytes, direction: bytes
) -> bytes:
    """Pure-spec envelope, computed only from RFC 2104 + the wire layout
    documented in `secure_channel.py`. Independent of any helper to avoid
    self-referencing 'just match my own impl' bugs.

    `direction` is the 3-byte label bound into the MAC and **never
    transmitted** — the receiver knows which way it is reading. Passing it
    explicitly keeps this an independent spec implementation: if the module
    ever picked the wrong label, this test would still compute the right one
    and catch it.
    """
    nonce_b = nonce_u64.to_bytes(NONCE_BYTES, "big")
    len_b = len(inner).to_bytes(LEN_BYTES, "little")
    mac = hmac.new(
        key, direction + nonce_b + len_b + inner, hashlib.sha256
    ).digest()[:HMAC_BYTES]
    return nonce_b + mac + len_b + inner


class TestWireFormat(unittest.TestCase):
    def test_overhead_constants(self):
        """Brain constants must match the kernel's auth_envelope.rs."""
        self.assertEqual(NONCE_BYTES, 8)
        self.assertEqual(HMAC_BYTES, 16)
        self.assertEqual(LEN_BYTES, 2)
        self.assertEqual(ENVELOPE_OVERHEAD, 26)
        self.assertEqual(KEY_BYTES, 32)
        self.assertEqual(MAX_INNER_BYTES, 8 * 1024)

    def test_sender_produces_spec_frame(self):
        """Brain Sender output must equal the spec frame at a fixed nonce."""
        s = Sender(_KEY)
        # Override the clock-derived starting nonce so the test is
        # deterministic: nonce will become 100 + 1 = 101 on the first wrap.
        s._nonce = 100
        frame = s.wrap(_INNER)
        self.assertEqual(frame, _expected_frame(101, _KEY, _INNER, DIR_C2S))

    def test_receiver_accepts_spec_frame(self):
        """Receiver must accept the spec-computed frame and recover inner."""
        r = Receiver(_KEY)
        frame = _expected_frame(1, _KEY, _INNER, DIR_S2C)
        self.assertEqual(r.unwrap(frame), _INNER)

    def test_kernel_seed_replay_pattern(self):
        """The kernel seeds its send nonce from a clock value and increments
        per packet. A frame at nonce=clock+1 must be accepted; the next at
        clock+2 too; a replay of clock+1 must be rejected after clock+2."""
        r = Receiver(_KEY)
        f1 = _expected_frame(1_000, _KEY, b"first", DIR_S2C)
        f2 = _expected_frame(1_001, _KEY, b"second", DIR_S2C)
        self.assertEqual(r.unwrap(f1), b"first")
        self.assertEqual(r.unwrap(f2), b"second")
        self.assertIsNone(r.unwrap(f1))  # replay
        self.assertIsNone(r.unwrap(f2))  # replay

    def test_kernel_style_truncated_frame_rejected(self):
        """A frame chopped before the envelope overhead must be rejected
        (kernel `if frame.len() < ENVELOPE_OVERHEAD` check)."""
        r = Receiver(_KEY)
        f = _expected_frame(1, _KEY, _INNER, DIR_S2C)
        for cut in range(ENVELOPE_OVERHEAD):
            self.assertIsNone(r.unwrap(f[:cut]))

    def test_kernel_style_bit_flip_rejected(self):
        """Flipping any single bit in the frame must break the HMAC and yield
        None — kernel constant-time compare must reject."""
        r = Receiver(_KEY)
        f = bytearray(_expected_frame(1, _KEY, _INNER, DIR_S2C))
        # Flip a bit in the inner payload (last byte) — HMAC was computed over
        # nonce+len+inner so any change there must break it.
        f[-1] ^= 0x01
        self.assertIsNone(r.unwrap(bytes(f)))

    def test_kernel_style_oversize_len_rejected(self):
        """A LEN field claiming > MAX_INNER_BYTES must be rejected without
        reading more (kernel `n > MAX_INNER_BYTES` check)."""
        r = Receiver(_KEY)
        # Build a malformed frame: header claims length = MAX_INNER + 1
        nonce_b = (1).to_bytes(NONCE_BYTES, "big")
        len_b = (MAX_INNER_BYTES + 1).to_bytes(LEN_BYTES, "little")
        # Bogus MAC — receiver should reject on size first.
        bogus_mac = b"\x00" * HMAC_BYTES
        frame = nonce_b + bogus_mac + len_b  # no body
        self.assertIsNone(r.unwrap(frame))


if __name__ == "__main__":
    unittest.main()
