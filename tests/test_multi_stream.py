"""RFC-0021 multi-stream framing — wire-format pin + protocol.py wiring tests.

`multi_stream.py` is a byte-for-byte mirror of `crates/multi-stream`; the
wire-format vectors here must match the Rust crate's encoding. The wiring
tests prove the framing composes as the OUTERMOST layer in protocol.py
(plaintext and over the HMAC envelope).
"""

import asyncio

import pytest

import multi_stream as ms
import protocol

# ── Wire-format pins (must match crates/multi-stream byte layout) ────────────


def test_wire_format_matches_crate():
    # wrap(STREAM_CONTROL=0x00, [0x01,0x02,0x03]) -> 00 03 00 01 02 03
    assert ms.wrap(ms.STREAM_CONTROL, bytes([1, 2, 3])) == bytes(
        [0x00, 0x03, 0x00, 0x01, 0x02, 0x03]
    )
    # camera stream 0 = 0x10; len LE u16
    assert ms.wrap(ms.STREAM_CAMERA_BASE, b"\xaa" * 300)[:3] == bytes(
        [0x10, 0x2C, 0x01]
    )  # 300 = 0x012C LE
    assert ms.HEADER_LEN == 3
    assert ms.STREAM_CONTROL == 0x00
    assert (ms.STREAM_CAMERA_BASE, ms.STREAM_CAMERA_LAST) == (0x10, 0x1F)
    assert ms.STREAM_LIDAR == 0x20 and ms.STREAM_AUDIO == 0x21


def test_wrap_unwrap_roundtrip():
    for sid in (ms.STREAM_CONTROL, ms.STREAM_CAMERA_BASE, ms.STREAM_LIDAR):
        for payload in (b"", b"x", b"BR\x01\x00\x00\xab", b"\xff" * 1000):
            sid_out, p_out = ms.unwrap(ms.wrap(sid, payload))
            assert sid_out == sid and p_out == payload


def test_unwrap_rejects_short_and_truncated():
    assert ms.unwrap(b"") is None
    assert ms.unwrap(b"\x00\x03") is None  # < HEADER_LEN
    # LEN claims 5 but only 2 payload bytes present (length-extension/truncation)
    assert ms.unwrap(bytes([0x00, 0x05, 0x00, 0xAA, 0xBB])) is None


def test_payload_too_large_raises():
    with pytest.raises(ValueError):
        ms.wrap(ms.STREAM_CONTROL, b"\x00" * (ms.MAX_PAYLOAD_LEN + 1))


def test_camera_helpers():
    assert ms.camera_stream_id(0) == 0x10
    assert ms.camera_stream_id(15) == 0x1F
    assert ms.camera_stream_id(16) is None
    assert ms.is_camera_stream(0x10) and ms.is_camera_stream(0x1F)
    assert not ms.is_camera_stream(0x00) and not ms.is_camera_stream(0x20)


# ── protocol.py wiring (multi-stream as outermost layer) ─────────────────────


class _FakeWriter:
    def __init__(self) -> None:
        self.buf = bytearray()

    def write(self, b: bytes) -> None:
        self.buf += b

    async def drain(self) -> None:
        pass


@pytest.fixture(autouse=True)
def _reset_protocol_state():
    def _clear():
        protocol._link_sender = None
        protocol._link_receiver = None
        protocol._encrypt_psk = None
        protocol._secure_channel = None
        protocol._multi_stream_armed = False

    _clear()
    yield
    _clear()


def _feed(data: bytes) -> asyncio.StreamReader:
    r = asyncio.StreamReader()
    r.feed_data(data)
    r.feed_eof()
    return r


def test_enable_from_env(monkeypatch):
    monkeypatch.delenv("ROBOT_BRAIN_MULTI_STREAM", raising=False)
    assert protocol.enable_multi_stream() is False
    monkeypatch.setenv("ROBOT_BRAIN_MULTI_STREAM", "1")
    assert protocol.enable_multi_stream() is True
    assert protocol.multi_stream_armed() is True


def test_roundtrip_plaintext_multistream(monkeypatch):
    monkeypatch.setenv("ROBOT_BRAIN_MULTI_STREAM", "1")
    protocol.enable_multi_stream()

    async def run():
        w = _FakeWriter()
        await protocol.send_packet(w, protocol.STATUS, b"\x01\x02\x03")
        # Wire must be a multi-stream frame on STREAM_CONTROL.
        assert w.buf[0] == ms.STREAM_CONTROL
        pkt = await protocol.read_packet(_feed(bytes(w.buf)))
        return pkt

    pkt_type, payload = asyncio.run(run())
    assert pkt_type == protocol.STATUS and payload == b"\x01\x02\x03"


def test_roundtrip_multistream_over_hmac(monkeypatch):
    # Compose: multi_stream( HMAC_envelope( brain_frame ) ).
    monkeypatch.setenv("ROBOT_BRAIN_LINK_KEY", bytes(range(32)).hex())
    monkeypatch.setenv("ROBOT_BRAIN_MULTI_STREAM", "1")
    assert protocol.enable_auth_envelope() is True
    assert protocol.enable_multi_stream() is True

    async def run():
        w = _FakeWriter()
        await protocol.send_packet(w, protocol.SENSOR_PACKET, b"payload-bytes")
        assert w.buf[0] == ms.STREAM_CONTROL
        # Strip the outer multi-stream frame → inner must be the HMAC envelope
        # (not the bare brain frame), proving correct layering order.
        sid, inner = ms.unwrap(bytes(w.buf))
        assert sid == ms.STREAM_CONTROL
        assert inner[:2] != protocol.MAGIC  # inner is wrapped, not raw brain frame

        # The envelope MAC is direction-bound, so a frame this side WROTE is
        # not valid inbound here — that is the whole point of the binding. To
        # exercise the read path we have to re-mint the inner envelope in the
        # kernel's direction (S2C), which is what a real peer would send.
        # Reading back our own C2S frame must fail, and we assert that too.
        assert await protocol.read_packet(_feed(bytes(w.buf))) is None, \
            "a frame we sent must not verify inbound (direction binding)"

        from secure_channel import DIR_S2C, Sender as _S
        peer = _S(bytes(range(32)), direction=DIR_S2C)
        inner_brain = protocol.build_packet(protocol.SENSOR_PACKET, b"payload-bytes")
        wire = ms.wrap(ms.STREAM_CONTROL, peer.wrap(inner_brain))
        return await protocol.read_packet(_feed(bytes(wire)))

    pkt_type, payload = asyncio.run(run())
    assert pkt_type == protocol.SENSOR_PACKET and payload == b"payload-bytes"


def test_reader_skips_noncontrol_stream(monkeypatch):
    monkeypatch.setenv("ROBOT_BRAIN_MULTI_STREAM", "1")
    protocol.enable_multi_stream()

    async def run():
        # A camera-stream frame followed by a control packet: the control
        # reader must skip the camera frame and return the control one.
        cam = ms.wrap(ms.STREAM_CAMERA_BASE, b"\xde\xad\xbe\xef")
        w = _FakeWriter()
        await protocol.send_packet(w, protocol.STATUS, b"ok")
        return await protocol.read_packet(_feed(cam + bytes(w.buf)))

    pkt_type, payload = asyncio.run(run())
    assert pkt_type == protocol.STATUS and payload == b"ok"
