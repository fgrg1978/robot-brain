"""Tests for the brain-side HMAC envelope (secure_channel.py)."""

import os
import secrets
from secure_channel import (
    DIR_S2C,
    Sender,
    Receiver,
    ENVELOPE_OVERHEAD,
    KEY_BYTES,
    MAX_INNER_BYTES,
    load_link_key,
)


def _new_pair():
    key = secrets.token_bytes(KEY_BYTES)
    # The Sender here stands in for the KERNEL: direction binding means a
    # frame is only valid in the direction it was minted for, so a Sender
    # built with the brain's own default (C2S) would — correctly — be
    # rejected by the brain's Receiver. Passing DIR_S2C models the real
    # kernel->brain path, which is what these tests are about.
    return Sender(key, direction=DIR_S2C), Receiver(key), key


def test_round_trip_inner_recovered():
    s, r, _ = _new_pair()
    inner = b"hello kernel"
    frame = s.wrap(inner)
    assert r.unwrap(frame) == inner


def test_envelope_overhead_is_26_bytes():
    s, _, _ = _new_pair()
    frame = s.wrap(b"")
    assert len(frame) == ENVELOPE_OVERHEAD


def test_replay_rejected():
    s, r, _ = _new_pair()
    frame1 = s.wrap(b"first")
    frame2 = s.wrap(b"second")
    assert r.unwrap(frame1) == b"first"
    assert r.unwrap(frame2) == b"second"
    # Replay either earlier frame.
    assert r.unwrap(frame1) is None
    assert r.unwrap(frame2) is None


def test_tampered_inner_rejected():
    s, r, _ = _new_pair()
    frame = bytearray(s.wrap(b"hello"))
    # Flip a byte in the inner section (after nonce + hmac + len = 26).
    frame[ENVELOPE_OVERHEAD] ^= 0xFF
    assert r.unwrap(bytes(frame)) is None


def test_tampered_nonce_rejected():
    s, r, _ = _new_pair()
    frame = bytearray(s.wrap(b"hello"))
    frame[0] ^= 0x01
    assert r.unwrap(bytes(frame)) is None


def test_tampered_hmac_rejected():
    s, r, _ = _new_pair()
    frame = bytearray(s.wrap(b"hello"))
    frame[8] ^= 0x01  # first hmac byte
    assert r.unwrap(bytes(frame)) is None


def test_truncated_frame_rejected():
    s, r, _ = _new_pair()
    frame = s.wrap(b"hello")
    assert r.unwrap(frame[:25]) is None
    assert r.unwrap(b"") is None


def test_oversize_inner_refused_at_send():
    s, _, _ = _new_pair()
    big = b"x" * (MAX_INNER_BYTES + 1)
    try:
        s.wrap(big)
    except ValueError:
        return
    raise AssertionError("oversize wrap should have raised")


def test_distinct_keys_dont_communicate():
    a_key = secrets.token_bytes(KEY_BYTES)
    b_key = secrets.token_bytes(KEY_BYTES)
    s = Sender(a_key, direction=DIR_S2C)
    r = Receiver(b_key)
    assert r.unwrap(s.wrap(b"hello")) is None


def test_load_link_key_missing_returns_none(monkeypatch):
    monkeypatch.delenv("ROBOT_BRAIN_LINK_KEY", raising=False)
    assert load_link_key() is None


def test_load_link_key_invalid_hex_returns_none(monkeypatch, capsys):
    monkeypatch.setenv("ROBOT_BRAIN_LINK_KEY", "zz")
    assert load_link_key() is None
    out = capsys.readouterr().out
    assert "not valid hex" in out


def test_load_link_key_wrong_length_returns_none(monkeypatch, capsys):
    monkeypatch.setenv("ROBOT_BRAIN_LINK_KEY", "00" * 31)
    assert load_link_key() is None
    out = capsys.readouterr().out
    assert "must be" in out


def test_load_link_key_valid_32_bytes(monkeypatch):
    hex32 = "00" * 32
    monkeypatch.setenv("ROBOT_BRAIN_LINK_KEY", hex32)
    key = load_link_key()
    assert key is not None
    assert len(key) == 32
