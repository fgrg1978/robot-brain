"""Self-test for tools/stub_brain.py — exercises the kernel↔brain wire
loop end-to-end in-process: spawn the stub on a free port, connect as
a fake robot, send N sensor packets, expect an ActuatorCmd back.

This is the Python-only half of the Phase 1 E2E story. The kernel-in-
QEMU half is a separate Makefile target (e2e-wheeled-qemu); both
exercise the same stub_brain.py so a green pytest run here implies
the wire-format glue is sound before involving QEMU.
"""

import asyncio
import os
import socket
import sys
import time
from contextlib import contextmanager
from typing import Optional

import pytest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.join(_ROOT, "tools"))

import protocol
from protocol import (
    build_packet,
    parse_packet,
    SensorPacket,
    SENSOR_PACKET,
    ACTUATOR_CMD,
)
from secure_channel import (
    DIR_C2S,
    DIR_S2C,
    ENVELOPE_OVERHEAD,
    KEY_BYTES,
    LEN_BYTES,
    Receiver,
    Sender,
)
from stub_brain import (
    ENVELOPE_LEN_OFFSET,
    EXIT_AUTH_FAILURE,
    EXIT_NO_LOOP,
    EXIT_OK,
    MAX_RESYNC_SKEW_BYTES,
    StubBrain,
    _corrupt_key_hex,
    arm_link_layer,
    compute_exit_code,
)


def _free_port() -> int:
    """Ask the kernel for an ephemeral port we can bind to. Avoids
    the test-flakiness of hardcoding 8080 on a dev box that might
    already have something listening there."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _make_sensor_packet(seq: int) -> bytes:
    """Build a minimal SensorPacket-shaped byte stream the stub
    will accept. Field values don't matter — we just want the
    wire format to validate."""
    pkt = SensorPacket(
        timestamp_ms=seq * 100,
        battery_mv=7400,
        accel_mg=(0, 0, 1000),
        gyro_mdps=(0, 0, 0),
        odom_dist_mm=seq * 10,
        odom_hdg_cdeg=0,
        encoder_l=seq,
        encoder_r=seq,
        range_front_mm=1500,
        range_right_mm=1500,
    )
    return build_packet(SENSOR_PACKET, pkt.to_bytes())


def test_loop_closes_after_n_sensors():
    """Sync wrapper that drives the loop via asyncio.run, so the
    test suite doesn't need pytest-asyncio installed."""
    port = _free_port()
    brain = StubBrain(
        duration_s=5,
        sensors_to_go=3,
        forward_pct=50,
        quiet=True,
    )

    async def run() -> "Optional[bytes]":
        server = await asyncio.start_server(brain.handle, host="127.0.0.1", port=port)

        async def fake_robot() -> "Optional[bytes]":
            reader, writer = await asyncio.open_connection("127.0.0.1", port)
            for seq in range(3):
                writer.write(_make_sensor_packet(seq))
                await writer.drain()
                await asyncio.sleep(0.05)
            buf = b""
            deadline = asyncio.get_event_loop().time() + 2.0
            actuator_payload = None  # Optional[bytes]
            while asyncio.get_event_loop().time() < deadline:
                try:
                    chunk = await asyncio.wait_for(reader.read(256), timeout=0.5)
                except asyncio.TimeoutError:
                    continue
                if not chunk:
                    break
                buf += chunk
                parsed = parse_packet(buf)
                if parsed is not None:
                    pkt_type, payload = parsed
                    if pkt_type == ACTUATOR_CMD:
                        actuator_payload = payload
                        break
            writer.close()
            try:
                await writer.wait_closed()
            except Exception:
                pass
            return actuator_payload

        async with server:
            return await asyncio.wait_for(fake_robot(), timeout=10)

    actuator_payload = asyncio.run(run())
    assert actuator_payload is not None, "brain never sent an ActuatorCmd"
    assert brain.pkt_counts.get(SENSOR_PACKET, 0) == 3
    assert brain.cmds_sent == 1


def test_summary_format_smoke():
    brain = StubBrain(duration_s=10, sensors_to_go=3, forward_pct=50, quiet=True)
    # Call _summary with empty state — must not blow up before
    # the first packet arrives.
    s = brain._summary()
    assert "rx[" in s and "tx[" in s


# ─────────────────────────────────────────────────────────────────────────────
# Auth-envelope coverage (RFC-0011)
#
# The stub used to read raw frames and resync past anything that was not
# MAGIC, so a 26-byte HMAC envelope was skipped as "slirp noise" and the
# harness reported OK while verifying nothing. These tests pin the
# opposite property: with a key armed the harness MUST reject.
#
# No listeners here — StubBrain.handle() is driven over a socketpair.
# ─────────────────────────────────────────────────────────────────────────────

#: Deterministic test keys (never used on a real link).
_KEY = bytes(range(KEY_BYTES))
_KEY_HEX = _KEY.hex()
_WRONG_KEY = bytes(bytearray(_KEY[:-1]) + bytes([_KEY[-1] ^ 0x01]))

_RX_CHUNK = 4096
_POLL_TIMEOUT_S = 0.25
#: Generous when a command is expected, short when one must NOT arrive.
_EXPECT_ACTUATOR_TIMEOUT_S = 3.0
_NO_ACTUATOR_TIMEOUT_S = 0.5
_JOIN_TIMEOUT_S = 5.0
_SESSION_DURATION_S = 10


@contextmanager
def _armed(key_hex):
    """Arm protocol.py's envelope singletons exactly as the CLI does,
    and tear them down so the rest of the suite stays plaintext."""
    prev = os.environ.get("ROBOT_BRAIN_LINK_KEY")
    os.environ["ROBOT_BRAIN_LINK_KEY"] = key_hex
    try:
        assert protocol.enable_auth_envelope() is True
        yield
    finally:
        protocol._link_sender = None
        protocol._link_receiver = None
        if prev is None:
            os.environ.pop("ROBOT_BRAIN_LINK_KEY", None)
        else:
            os.environ["ROBOT_BRAIN_LINK_KEY"] = prev


def _wrap_stream(frames, key, direction):
    """Bytes as the kernel would put them on the wire: one envelope per
    frame, MAC bound to `direction`."""
    sender = Sender(key, direction=direction)
    return b"".join(sender.wrap(f) for f in frames)


async def _read_actuator(reader, key, timeout_s):
    """Robot-side reader: unwrap (C2S — the brain's outbound label) and
    return the first ActuatorCmd payload, or None."""
    receiver = Receiver(key, direction=DIR_C2S) if key is not None else None
    buf = b""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            chunk = await asyncio.wait_for(reader.read(_RX_CHUNK), timeout=_POLL_TIMEOUT_S)
        except asyncio.TimeoutError:
            continue
        if not chunk:
            return None
        buf += chunk
        if receiver is not None:
            if len(buf) < ENVELOPE_OVERHEAD:
                continue
            lo = ENVELOPE_LEN_OFFSET
            n = int.from_bytes(buf[lo : lo + LEN_BYTES], "little")
            if len(buf) < ENVELOPE_OVERHEAD + n:
                continue
            inner = receiver.unwrap(buf[: ENVELOPE_OVERHEAD + n])
            buf = buf[ENVELOPE_OVERHEAD + n :]
            if inner is None:
                return None  # the brain's own outbound MAC did not verify
            parsed = parse_packet(inner)
        else:
            parsed = parse_packet(buf)
        if parsed is not None and parsed[0] == ACTUATOR_CMD:
            return parsed[1]
    return None


def _drive(brain, stream, actuator_key=None, read_timeout_s=_EXPECT_ACTUATOR_TIMEOUT_S):
    """Play the robot: hand `stream` to StubBrain.handle over a socketpair,
    optionally collect the ActuatorCmd, then EOF. Returns the payload."""

    async def run():
        sock_stub, sock_bot = socket.socketpair()
        r_stub, w_stub = await asyncio.open_connection(sock=sock_stub)
        r_bot, w_bot = await asyncio.open_connection(sock=sock_bot)
        task = asyncio.create_task(brain.handle(r_stub, w_stub))
        w_bot.write(stream)
        await w_bot.drain()
        payload = await _read_actuator(r_bot, actuator_key, read_timeout_s)
        w_bot.close()
        try:
            await w_bot.wait_closed()
        except Exception:
            pass
        await asyncio.wait_for(task, timeout=_JOIN_TIMEOUT_S)
        return payload

    return asyncio.run(run())


def _brain(sensors_to_go=3, expect_auth_failure=False):
    return StubBrain(
        duration_s=_SESSION_DURATION_S,
        sensors_to_go=sensors_to_go,
        forward_pct=50,
        quiet=True,
        expect_auth_failure=expect_auth_failure,
    )


def test_envelope_loop_closes_with_correct_key():
    """Positive: kernel wraps S2C, brain verifies, brain answers C2S."""
    with _armed(_KEY_HEX):
        brain = _brain()
        assert brain.envelope is True
        stream = _wrap_stream([_make_sensor_packet(i) for i in range(3)], _KEY, DIR_S2C)
        payload = _drive(brain, stream, actuator_key=_KEY)
        code = compute_exit_code(brain)

    assert payload is not None, "brain never sent a verifiable ActuatorCmd"
    assert brain.verified_frames == 3
    assert brain.auth_failures == 0
    assert brain.bad_inner_frames == 0
    assert brain.noise_bytes == 0
    assert brain.skew_bytes == 0
    assert brain.pkt_counts.get(SENSOR_PACKET, 0) == 3
    assert code == EXIT_OK


def test_envelope_rejects_wrong_key():
    """Negative: same frames, wrong key → nothing verifies, gate fails."""
    with _armed(_KEY_HEX):
        brain = _brain()
        stream = _wrap_stream(
            [_make_sensor_packet(i) for i in range(3)], _WRONG_KEY, DIR_S2C
        )
        payload = _drive(
            brain, stream, actuator_key=_KEY, read_timeout_s=_NO_ACTUATOR_TIMEOUT_S
        )
        code = compute_exit_code(brain)
        # Even --accept-tcp-only must not paper over a crypto failure.
        tcp_only_code = compute_exit_code(brain, accept_tcp_only=True)

    assert payload is None
    assert brain.verified_frames == 0
    assert brain.auth_failures == 3, "each mis-keyed envelope must be counted"
    assert brain.pkt_counts == {}
    assert brain.noise_bytes > 0
    assert code == EXIT_AUTH_FAILURE
    assert tcp_only_code == EXIT_AUTH_FAILURE


def test_envelope_rejects_wrong_direction_label():
    """The MAC is direction-bound: a frame wrapped with the brain's OWN
    outbound label (C2S) must not verify inbound, even with the right key.

    This is the property that just landed in both repos; without this
    test the E2E would ship blind to a direction-binding regression.
    """
    with _armed(_KEY_HEX):
        brain = _brain()
        stream = _wrap_stream([_make_sensor_packet(i) for i in range(3)], _KEY, DIR_C2S)
        payload = _drive(
            brain, stream, actuator_key=_KEY, read_timeout_s=_NO_ACTUATOR_TIMEOUT_S
        )
        code = compute_exit_code(brain)

    assert payload is None
    assert brain.verified_frames == 0
    assert brain.auth_failures == 3
    assert code == EXIT_AUTH_FAILURE


def test_expect_auth_failure_mode_passes_only_when_nothing_verifies():
    """`--expect-auth-failure`: exit 0 iff every frame was rejected."""
    with _armed(_KEY_HEX):
        rejected = _brain(expect_auth_failure=True)
        bad_stream = _wrap_stream(
            [_make_sensor_packet(i) for i in range(3)], _WRONG_KEY, DIR_S2C
        )
        _drive(rejected, bad_stream, read_timeout_s=_NO_ACTUATOR_TIMEOUT_S)
        rejected_code = compute_exit_code(rejected)

        accepted = _brain(expect_auth_failure=True)
        good_stream = _wrap_stream(
            [_make_sensor_packet(i) for i in range(3)], _KEY, DIR_S2C
        )
        _drive(accepted, good_stream, actuator_key=_KEY)
        accepted_code = compute_exit_code(accepted)

        silent = _brain(expect_auth_failure=True)
        _drive(silent, b"", read_timeout_s=_NO_ACTUATOR_TIMEOUT_S)
        silent_code = compute_exit_code(silent)

    assert rejected_code == EXIT_OK and rejected.auth_failures == 3
    # Anything verifying under a wrong key means the gate is not enforcing.
    assert accepted_code == EXIT_AUTH_FAILURE and accepted.verified_frames == 3
    # And a silent link proves nothing — it must not pass either.
    assert silent_code == EXIT_NO_LOOP


def test_slirp_first_byte_drop_still_tolerated_but_reported():
    """The slirp bug is real: losing byte 0 destroys the whole first
    envelope, so the resync must skip to the next frame boundary — and
    only because a MAC verified there, not because bytes looked odd."""
    frames = [_make_sensor_packet(i) for i in range(3)]
    first_frame_len = ENVELOPE_OVERHEAD + len(frames[0])
    assert first_frame_len - 1 <= MAX_RESYNC_SKEW_BYTES

    with _armed(_KEY_HEX):
        brain = _brain(sensors_to_go=2)
        stream = _wrap_stream(frames, _KEY, DIR_S2C)[1:]  # drop first byte
        payload = _drive(brain, stream, actuator_key=_KEY)
        code = compute_exit_code(brain)

    assert payload is not None
    assert brain.skew_bytes == first_frame_len - 1, "must skip exactly the lost frame"
    assert brain.verified_frames == 2, "the frame that lost its first byte is gone"
    assert brain.noise_bytes == 0
    assert brain.auth_failures == 0
    assert code == EXIT_OK


def test_envelope_junk_after_sync_is_fatal():
    """Once synced, unverifiable bytes are a failure, not noise."""
    with _armed(_KEY_HEX):
        brain = _brain(sensors_to_go=1)
        stream = _wrap_stream([_make_sensor_packet(0)], _KEY, DIR_S2C) + b"\xff" * 64
        _drive(brain, stream, actuator_key=_KEY)
        code = compute_exit_code(brain)

    assert brain.verified_frames == 1
    assert brain.noise_bytes > 0
    assert code == EXIT_AUTH_FAILURE


def test_stream_cut_mid_frame_is_not_an_auth_failure():
    """The kernel closing while a frame is in flight is ordinary TCP
    teardown. The gate must fail on cryptography, not on disconnects."""
    frames = [_make_sensor_packet(i) for i in range(3)]
    with _armed(_KEY_HEX):
        brain = _brain(sensors_to_go=2)
        full = _wrap_stream(frames, _KEY, DIR_S2C)
        # Cut the third envelope after its header + a few payload bytes.
        cut_at = 2 * (ENVELOPE_OVERHEAD + len(frames[0])) + ENVELOPE_OVERHEAD + 10
        payload = _drive(brain, full[:cut_at], actuator_key=_KEY)
        code = compute_exit_code(brain)

    assert payload is not None
    assert brain.verified_frames == 2
    assert brain.truncated_bytes == ENVELOPE_OVERHEAD + 10
    assert brain.noise_bytes == 0
    assert brain.auth_failures == 0
    assert code == EXIT_OK


def test_valid_mac_over_unparseable_inner_is_fatal():
    """A good MAC over a frame the parser rejects means the two repos
    disagree on the wire format — exactly what this harness must catch."""
    with _armed(_KEY_HEX):
        brain = _brain()
        stream = _wrap_stream([b"\x00" * 32], _KEY, DIR_S2C)
        _drive(brain, stream, read_timeout_s=_NO_ACTUATOR_TIMEOUT_S)
        code = compute_exit_code(brain)

    assert brain.verified_frames == 1
    assert brain.bad_inner_frames == 1
    assert brain.pkt_counts == {}
    assert code == EXIT_AUTH_FAILURE


def test_plaintext_mode_still_resyncs_and_stays_non_fatal():
    """No key → nothing to verify. Leading junk is counted and logged
    but must not turn currently-green keyless scenarios red."""
    brain = _brain()
    assert brain.envelope is False
    stream = b"\x99" * 7 + b"".join(_make_sensor_packet(i) for i in range(3))
    payload = _drive(brain, stream)
    code = compute_exit_code(brain)

    assert payload is not None
    assert brain.pkt_counts.get(SENSOR_PACKET, 0) == 3
    assert brain.skew_bytes == 7
    assert brain.noise_bytes == 0
    assert code == EXIT_OK


def test_arm_link_layer_never_downgrades_silently():
    prev = os.environ.get("ROBOT_BRAIN_LINK_KEY")
    try:
        # No key → plaintext, no error.
        os.environ.pop("ROBOT_BRAIN_LINK_KEY", None)
        assert arm_link_layer(quiet=True, expect_auth_failure=False) is None
        assert protocol._link_receiver is None

        # Malformed key → hard failure, NOT a quiet fallback to plaintext.
        os.environ["ROBOT_BRAIN_LINK_KEY"] = "not-hex"
        assert arm_link_layer(quiet=True, expect_auth_failure=False) == EXIT_AUTH_FAILURE
        assert protocol._link_receiver is None

        # Negative mode without a key is a misconfiguration, not a pass.
        os.environ.pop("ROBOT_BRAIN_LINK_KEY", None)
        assert arm_link_layer(quiet=True, expect_auth_failure=True) == EXIT_AUTH_FAILURE

        # Negative mode with a key: arms with a DIFFERENT (corrupted) key.
        os.environ["ROBOT_BRAIN_LINK_KEY"] = _KEY_HEX
        assert arm_link_layer(quiet=True, expect_auth_failure=True) is None
        assert protocol._link_receiver is not None
        assert os.environ["ROBOT_BRAIN_LINK_KEY"] != _KEY_HEX
        assert bytes.fromhex(os.environ["ROBOT_BRAIN_LINK_KEY"]) != _KEY
    finally:
        protocol._link_sender = None
        protocol._link_receiver = None
        if prev is None:
            os.environ.pop("ROBOT_BRAIN_LINK_KEY", None)
        else:
            os.environ["ROBOT_BRAIN_LINK_KEY"] = prev


def test_corrupt_key_hex_changes_exactly_one_bit():
    corrupted = bytes.fromhex(_corrupt_key_hex(_KEY_HEX))
    assert len(corrupted) == KEY_BYTES
    assert corrupted != _KEY
    diff = sum((a ^ b).bit_count() for a, b in zip(_KEY, corrupted))
    assert diff == 1
    with pytest.raises(ValueError):
        _corrupt_key_hex("00" * (KEY_BYTES - 1))
