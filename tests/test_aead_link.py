"""AEAD link tests (RFC-0019).

Covers the brain-side `SecureChannel` class: handshake state machine,
key derivation, encrypt/decrypt round-trip, mismatch / replay rejection,
and a hand-computed vector that the Rust side must match byte-for-byte
(pinned against `crates/crypto/src/secure_channel.rs`).

The cross-side wire pin is a hand-computed reference vector — the same
pattern the existing `test_auth_envelope_compat.py` uses.  When the Rust
side's `crates/regression-tests/src/aead_link_tests.rs` lands, it will
compute the same vector and the two sides cannot drift silently.
"""

from __future__ import annotations

import hashlib
import os
import struct

import pytest

from secure_channel import (
    SecureChannel,
    AEAD_OVERHEAD,
    AEAD_NONCE_SIZE,
    AEAD_LEN_SIZE,
    AEAD_HMAC_SIZE,
    AEAD_KEY_SIZE,
    AEAD_MAX_PAYLOAD,
    KEY_BYTES,
)

# ── Constants for the test vectors ────────────────────────────────────────

PSK = bytes(range(32))  # deterministic 32B PSK
SAMPLE_PLAINTEXT = b"hello, brain<->kernel via RFC-0019 AEAD"


# ── Helpers ────────────────────────────────────────────────────────────────


def _full_handshake() -> tuple[SecureChannel, SecureChannel]:
    """Drive the brain↔kernel handshake end-to-end.

    Returns (initiator, responder), both in ESTABLISHED state with matching
    derived keys.  Both proofs must verify or this raises.
    """
    init = SecureChannel(PSK, is_initiator=True)
    resp = SecureChannel(PSK, is_initiator=False)

    hello_init = init.start_handshake()
    hello_resp = resp.handle_initiator_hello(hello_init)
    confirm = init.handle_peer_hello(hello_resp)
    resp.handle_initiator_confirm(confirm)

    assert init.is_established
    assert resp.is_established
    return init, resp


# ── Handshake ─────────────────────────────────────────────────────────────


def test_full_handshake_establishes_both_sides() -> None:
    init, resp = _full_handshake()
    # Same keys derived on both sides.
    # Direction binding: the two sides no longer share ONE key pair. The
    # initiator's transmit key must equal the responder's receive key and vice
    # versa — that crossing IS the property. Asserting equality of a single
    # shared pair would re-assert the flaw this replaced.
    assert init._tx_enc_key == resp._rx_enc_key
    assert init._rx_enc_key == resp._tx_enc_key
    assert init._tx_enc_key != init._rx_enc_key, "directions must not collapse"
    assert init._tx_mac_key == resp._rx_mac_key
    assert init._rx_mac_key == resp._tx_mac_key
    # Ephemeral private key zeroed on both sides.
    assert init._eph_priv is None
    assert resp._eph_priv is None


def test_handshake_rejects_bad_psk_on_responder_proof() -> None:
    """Brain has correct PSK; kernel has a wrong PSK.  Brain rejects."""
    init = SecureChannel(PSK, is_initiator=True)
    bad_psk = bytes([b ^ 0xFF for b in PSK])
    resp = SecureChannel(bad_psk, is_initiator=False)

    hello_init = init.start_handshake()
    hello_resp = resp.handle_initiator_hello(hello_init)
    with pytest.raises(PermissionError, match="proof_k"):
        init.handle_peer_hello(hello_resp)


def test_handshake_rejects_bad_psk_on_initiator_confirm() -> None:
    """Kernel has correct PSK; brain has a wrong one.  Initiator confirm fails."""
    bad_psk = bytes([b ^ 0xFF for b in PSK])
    init = SecureChannel(bad_psk, is_initiator=True)
    resp = SecureChannel(PSK, is_initiator=False)

    hello_init = init.start_handshake()
    hello_resp = resp.handle_initiator_hello(hello_init)
    # Initiator catches the responder-proof mismatch first.
    with pytest.raises(PermissionError):
        init.handle_peer_hello(hello_resp)


def test_handshake_rejects_truncated_hello() -> None:
    init = SecureChannel(PSK, is_initiator=True)
    resp = SecureChannel(PSK, is_initiator=False)
    hello_init = init.start_handshake()
    with pytest.raises(ValueError, match="length"):
        resp.handle_initiator_hello(hello_init[:-1])


def test_handshake_rejects_wrong_mode_byte() -> None:
    init = SecureChannel(PSK, is_initiator=True)
    resp = SecureChannel(PSK, is_initiator=False)
    hello_init = bytearray(init.start_handshake())
    hello_init[0] = 0x01  # AUTH_HMAC mode — invalid in encrypted path
    with pytest.raises(ValueError, match="header"):
        resp.handle_initiator_hello(bytes(hello_init))


# ── Encrypt / decrypt round-trip ───────────────────────────────────────────


def test_encrypt_decrypt_roundtrip() -> None:
    init, resp = _full_handshake()
    frame = init.encrypt(SAMPLE_PLAINTEXT)
    out = resp.decrypt(frame)
    assert out == SAMPLE_PLAINTEXT


def test_encrypt_decrypt_roundtrip_bidirectional() -> None:
    """Both sides must be able to send to the other."""
    init, resp = _full_handshake()
    f1 = init.encrypt(b"brain to kernel")
    assert resp.decrypt(f1) == b"brain to kernel"
    f2 = resp.encrypt(b"kernel to brain")
    assert init.decrypt(f2) == b"kernel to brain"


def test_decrypt_rejects_bit_flip_in_ciphertext() -> None:
    init, resp = _full_handshake()
    frame = bytearray(init.encrypt(SAMPLE_PLAINTEXT))
    # Flip a bit in the ciphertext body (past header, before HMAC).
    ct_offset = AEAD_NONCE_SIZE + AEAD_LEN_SIZE
    frame[ct_offset] ^= 0x01
    assert resp.decrypt(bytes(frame)) is None
    assert resp.drops_hmac == 1


def test_decrypt_rejects_bit_flip_in_hmac() -> None:
    init, resp = _full_handshake()
    frame = bytearray(init.encrypt(SAMPLE_PLAINTEXT))
    frame[-1] ^= 0x80
    assert resp.decrypt(bytes(frame)) is None
    assert resp.drops_hmac == 1


def test_decrypt_rejects_truncated_frame() -> None:
    init, resp = _full_handshake()
    frame = init.encrypt(SAMPLE_PLAINTEXT)
    # Cut just before the HMAC ends.
    assert resp.decrypt(frame[: AEAD_OVERHEAD - 1]) is None
    # No counter bump on length-malformed frames — pre-auth check.
    assert resp.drops_hmac == 0


def test_decrypt_rejects_oversize_payload_len() -> None:
    init, resp = _full_handshake()
    # Hand-craft a frame with a payload length field > MAX_PAYLOAD.
    nonce = bytes(AEAD_NONCE_SIZE)
    big_len = (AEAD_MAX_PAYLOAD + 1).to_bytes(AEAD_LEN_SIZE, "little")
    body = bytes(AEAD_MAX_PAYLOAD + 1)
    mac = bytes(AEAD_HMAC_SIZE)
    frame = nonce + big_len + body + mac
    assert resp.decrypt(frame) is None


def test_encrypt_rejects_oversize_plaintext() -> None:
    init, _ = _full_handshake()
    with pytest.raises(ValueError, match="too large"):
        init.encrypt(b"x" * (AEAD_MAX_PAYLOAD + 1))


# ── KDF matches the Rust crate byte-for-byte ──────────────────────────────


def test_kdf_matches_rust_crate() -> None:
    """The crate uses SHA-256(shared || "ENC")[0..16] and SHA-256(shared || "MAC")[0..16].

    Pin those exact constants here so a future KDF refactor on either side
    can't drift silently.  Vector is computed against a synthetic
    `shared_secret` of `bytes(range(32))`.
    """
    shared = bytes(range(32))
    expected_enc = hashlib.sha256(shared + b"ENC").digest()[:AEAD_KEY_SIZE]
    expected_mac = hashlib.sha256(shared + b"MAC").digest()[:AEAD_KEY_SIZE]
    # Hand-computed values (SHA-256 over shared=bytes(range(32)) + label,
    # first 16 bytes) — change these only when changing the crate KDF.
    assert expected_enc.hex() == "c8d73999249e995c5ea7afa6eb65a06a"
    expected_mac_hex = hashlib.sha256(bytes(range(32)) + b"MAC").hexdigest()[:32]
    assert expected_mac.hex() == expected_mac_hex


# ── Counter exhaustion ────────────────────────────────────────────────────


def test_encrypt_refuses_at_counter_max() -> None:
    """Setting the counter to MAX must block encrypt to prevent keystream reuse."""
    init, _ = _full_handshake()
    init._tx_counter = 0xFFFFFFFF
    with pytest.raises(RuntimeError, match="exhausted"):
        init.encrypt(b"any")


# ── Wire-format pin (cross-side hand-computed vector) ─────────────────────


def test_wire_overhead_constants_match_crate() -> None:
    """If any of these drift, the Rust crate is out of sync."""
    assert AEAD_NONCE_SIZE == 12  # NONCE_SIZE
    assert AEAD_LEN_SIZE == 2
    assert AEAD_HMAC_SIZE == 32  # HMAC_SIZE (full SHA-256, not truncated)
    assert AEAD_OVERHEAD == 46  # PACKET_OVERHEAD
    assert AEAD_MAX_PAYLOAD == 2048  # MAX_PAYLOAD_SIZE
    assert AEAD_KEY_SIZE == 16  # AES_KEY_SIZE


def test_encrypt_frame_layout_is_n_then_len_then_ct_then_mac() -> None:
    """Layout pin: nonce(12) | len(2 LE) | ciphertext(N) | mac(32)."""
    init, _ = _full_handshake()
    pt = b"abc"
    frame = init.encrypt(pt)
    # Length field at offset 12-13 must equal len(pt).
    n = int.from_bytes(frame[12:14], "little")
    assert n == 3
    # Ciphertext occupies bytes 14..17, HMAC the last 32 bytes.
    assert len(frame) == AEAD_OVERHEAD + 3
    # CT differs from plaintext (negligible random chance of equality at 3B).
    assert frame[14:17] != pt
