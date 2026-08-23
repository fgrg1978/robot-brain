"""RFC-0019 encrypted-link WIRING tests (protocol.py + server integration).

The crypto/state-machine itself is covered by `test_aead_link.py`. These
tests cover the *wiring* in `protocol.py`:

  - `enable_encrypt_link()` arming + the no-silent-fallback refusal,
  - `perform_handshake()` driving the brain (initiator) over real loopback
    streams against a mock kernel (responder),
  - the nested send/read path:  AEAD( HMAC_envelope( brain_frame ) ),
  - handshake failure (bad PSK) → connection refused.

Run: `python -m pytest tests/test_encrypt_link_wiring.py`
"""

import asyncio

import pytest

import protocol

# Skip the whole module if the optional crypto dep is missing.
sc = pytest.importorskip("secure_channel")
if not getattr(sc, "_HAS_CRYPTOGRAPHY", False):
    pytest.skip("cryptography package not installed", allow_module_level=True)

from secure_channel import (  # noqa: E402
    DIR_C2S,
    DIR_S2C,
    SecureChannel,
    Sender,
    Receiver,
    AEAD_NONCE_SIZE,
    AEAD_LEN_SIZE,
    AEAD_HMAC_SIZE,
)

PSK = bytes(range(32))  # 32-byte test key
HEXKEY = PSK.hex()  # 64 hex chars for ROBOT_BRAIN_LINK_KEY
_HELLO_INIT = 2 + 32  # 34
_CONFIRM = 2 + 32  # 34


@pytest.fixture(autouse=True)
def _reset_protocol_state():
    """protocol.py keeps module-level singletons for the single brain↔kernel
    connection. Reset them around every test so state doesn't leak."""

    def _clear():
        protocol._link_sender = None
        protocol._link_receiver = None
        protocol._encrypt_psk = None
        protocol._secure_channel = None

    _clear()
    yield
    _clear()


# ── enable_encrypt_link / arming ─────────────────────────────────────────────


def test_enable_off_when_flag_unset(monkeypatch):
    monkeypatch.delenv("ROBOT_BRAIN_ENCRYPT_LINK", raising=False)
    assert protocol.enable_encrypt_link() is False
    assert protocol.encrypt_link_armed() is False


def test_enable_refuses_when_flag_set_without_key(monkeypatch):
    # Flag on, but the HMAC envelope (PSK source / inner layer) is not active.
    monkeypatch.setenv("ROBOT_BRAIN_ENCRYPT_LINK", "1")
    monkeypatch.delenv("ROBOT_BRAIN_LINK_KEY", raising=False)
    with pytest.raises(RuntimeError, match="no silent fallback"):
        protocol.enable_encrypt_link()


def test_enable_arms_with_key(monkeypatch):
    monkeypatch.setenv("ROBOT_BRAIN_LINK_KEY", HEXKEY)
    monkeypatch.setenv("ROBOT_BRAIN_ENCRYPT_LINK", "1")
    assert protocol.enable_auth_envelope() is True  # inner HMAC layer
    assert protocol.enable_encrypt_link() is True  # outer AEAD armed
    assert protocol.encrypt_link_armed() is True


# ── Mock kernel (responder) over real loopback streams ───────────────────────


async def _mock_kernel(
    reader, writer, *, psk=PSK, capture=None, send_status=None, confirm_corrupt=False
):
    """Acts as the kernel side: responder handshake, then optionally read one
    encrypted packet (into `capture`) and/or send one (`send_status`)."""
    kc = SecureChannel(psk, is_initiator=False)
    # This mock plays the KERNEL, so its envelope directions are the mirror of
    # the brain's defaults: it transmits S2C and receives C2S. Using the
    # defaults here would make the mock a second brain, and every frame would
    # be correctly rejected by direction binding.
    ksend = Sender(psk, direction=DIR_S2C)
    krecv = Receiver(psk, direction=DIR_C2S)
    try:
        hello = await reader.readexactly(_HELLO_INIT)
        writer.write(kc.handle_initiator_hello(hello))
        await writer.drain()
        confirm = await reader.readexactly(_CONFIRM)
        if confirm_corrupt:
            return  # never verify — brain will see no further bytes
        kc.handle_initiator_confirm(confirm)

        if capture is not None:
            head = await reader.readexactly(AEAD_NONCE_SIZE + AEAD_LEN_SIZE)
            n = int.from_bytes(head[AEAD_NONCE_SIZE : AEAD_NONCE_SIZE + AEAD_LEN_SIZE], "little")
            body = await reader.readexactly(n + AEAD_HMAC_SIZE)
            envelope = kc.decrypt(head + body)
            inner = krecv.unwrap(envelope)
            capture["pkt"] = protocol.parse_packet(inner)

        if send_status is not None:
            pkt_type, payload = send_status
            frame = protocol.build_packet(pkt_type, payload)
            writer.write(kc.encrypt(ksend.wrap(frame)))
            await writer.drain()
            await asyncio.sleep(0.05)  # let the brain read before close
    except (asyncio.IncompleteReadError, ConnectionError):
        pass
    finally:
        writer.close()


async def _serve_once(handler):
    """Start a loopback server bound to an ephemeral port, return (server, port)."""
    server = await asyncio.start_server(handler, "127.0.0.1", 0)
    port = server.sockets[0].getsockname()[1]
    return server, port


def _arm_brain(monkeypatch):
    monkeypatch.setenv("ROBOT_BRAIN_LINK_KEY", HEXKEY)
    monkeypatch.setenv("ROBOT_BRAIN_ENCRYPT_LINK", "1")
    assert protocol.enable_auth_envelope() is True
    assert protocol.enable_encrypt_link() is True


def test_handshake_and_encrypted_roundtrip(monkeypatch):
    _arm_brain(monkeypatch)
    capture: dict = {}

    async def run():
        server, port = await _serve_once(
            lambda r, w: _mock_kernel(r, w, capture=capture, send_status=(0x03, b"\x01\x02"))
        )
        reader, writer = await asyncio.open_connection("127.0.0.1", port)
        ok = await protocol.perform_handshake(reader, writer)
        assert ok is True
        assert protocol._secure_channel is not None

        # Brain → kernel (encrypted): a sensor-shaped packet.
        await protocol.send_packet(writer, 0x01, b"hello-payload")
        # Kernel → brain (encrypted): the status packet the mock sent back.
        pkt = await protocol.read_packet(reader)

        writer.close()
        server.close()
        await server.wait_closed()
        return pkt

    pkt = asyncio.run(run())
    assert capture["pkt"] == (0x01, b"hello-payload")  # kernel decrypted ours
    assert pkt == (0x03, b"\x01\x02")  # we decrypted kernel's


def test_handshake_fails_on_bad_psk(monkeypatch):
    _arm_brain(monkeypatch)
    wrong_psk = bytes((b ^ 0xFF) for b in PSK)

    async def run():
        server, port = await _serve_once(lambda r, w: _mock_kernel(r, w, psk=wrong_psk))
        reader, writer = await asyncio.open_connection("127.0.0.1", port)
        ok = await protocol.perform_handshake(reader, writer)
        writer.close()
        server.close()
        await server.wait_closed()
        return ok

    ok = asyncio.run(run())
    assert ok is False  # bad proof_k → brain rejects
    assert protocol._secure_channel is None  # channel not established


def test_perform_handshake_noop_when_not_armed(monkeypatch):
    # Encryption not armed → perform_handshake is a pass-through True, and no
    # bytes are exchanged.
    monkeypatch.delenv("ROBOT_BRAIN_ENCRYPT_LINK", raising=False)
    assert protocol.encrypt_link_armed() is False

    async def run():
        # Dummy reader/writer that would fail if touched.
        reader = asyncio.StreamReader()
        # A real writer isn't needed since the function returns before I/O.
        return await protocol.perform_handshake(reader, None)  # type: ignore[arg-type]

    assert asyncio.run(run()) is True
