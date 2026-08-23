"""Binary protocol for robot <-> brain communication.

Packet format:
  MAGIC (2B) | TYPE (1B) | LEN (2B LE) | PAYLOAD (0-1400B) | CRC8 (1B)

Robot -> Server types (0x01-0x7F):
  0x01  SENSOR_PACKET   wheeled: 62B, drone: 68B, humanoid: variable
  0x02  CAMERA_FRAME    variable
  0x03  STATUS          8 bytes (includes robot_type)
  0x04  OTA_ACK         3 bytes (status + slot + reserved)
  0x05  SENSOR_COMPACT  20 bytes (low-bandwidth: LoRa/RF)

Server -> Robot types (0x80-0xFF):
  0x80  ACTUATOR_CMD    3 + 2*N bytes (generic: type + channels)
  0x81  MODE_CMD        1 byte
  0x82  WAYPOINT_CMD    14 bytes
  0x83  CONFIG_CMD      40 bytes

Robot types:
  0  WHEELED    differential drive (2 channels: speed_l, speed_r)
  1  DRONE      quad rotor (4 channels: throttle, roll, pitch, yaw)
  2  HUMANOID   joint angles (N channels)
  3  ACKERMANN  car/tractor (2 channels: speed, steer_angle)
"""

import asyncio
import os
import struct
from dataclasses import dataclass, field
from typing import Optional, Union, TYPE_CHECKING

MAGIC = b"\x42\x52"  # "BR"

# Packet types — Robot -> Server
SENSOR_PACKET = 0x01
CAMERA_FRAME = 0x02
STATUS = 0x03
OTA_ACK = 0x04  # kernel ack for OTA chunks (matches kernel PKT_OTA_ACK)
SENSOR_COMPACT = 0x05  # low-bandwidth sensor frame (LoRa/RF)

# Packet types — Server -> Robot
ACTUATOR_CMD = 0x80
VELOCITY_CMD = 0x80  # alias (backward compat)
MODE_CMD = 0x81
WAYPOINT_CMD = 0x82
CONFIG_CMD = 0x83
OTA_BEGIN = 0x84
OTA_CHUNK = 0x85
OTA_END = 0x86
PAYLOAD_CMD = 0x87  # E04: payload control (spray, gripper, cam trigger)
ESTOP_CMD = 0x88  # remote emergency stop
PREDICT_CMD = 0x89  # RFC-0034: predicted next actuator command + confidence
DEGRADE_CMD = 0x8A  # RFC-0036: brain-triggered degraded mode (capability containment)
SEMANTIC_LEVEL_CMD = 0x8B  # RFC-0037: graded degrade-level index (0=FULL…3=CONTAINED)

# Robot types
ROBOT_WHEELED = 0
ROBOT_DRONE = 1
ROBOT_HUMANOID = 2
ROBOT_ACKERMANN = 3

# Config/CLI robot-type name ↔ ROBOT_* wire constant. Single source of truth:
# the mapping used to exist as two independent inline dicts (policy.get_
# translator and server._on_status) that had already drifted — the server-side
# copy was missing ackermann=3, so an ackermann chassis silently reported
# itself as "wheeled" to the experience/meta/task-planner stack. Type
# selection now feeds the safety profile (see server.BrainServer), so a
# disagreement here is a safety bug, not a cosmetic one.
ROBOT_TYPE_BY_NAME: dict[str, int] = {
    "wheeled": ROBOT_WHEELED,
    "drone": ROBOT_DRONE,
    "humanoid": ROBOT_HUMANOID,
    "ackermann": ROBOT_ACKERMANN,
}
ROBOT_TYPE_NAME_BY_ID: dict[int, str] = {v: k for k, v in ROBOT_TYPE_BY_NAME.items()}

#: Fallback when a config string / wire byte names no known robot type.
ROBOT_TYPE_DEFAULT_NAME: str = "wheeled"

# ActuatorCmd flags
FLAG_EMERGENCY = 0x01
FLAG_ALERT = 0x02
FLAG_LOW_CONFIDENCE = 0x04  # RFC-0035: kernel tightens the motor envelope

# Actuator types (for ActuatorCmd.actuator_type)
ACT_DIFF_DRIVE = 0
ACT_QUAD_ROTOR = 1
ACT_HUMANOID = 2
ACT_ACKERMANN = 3

# LED state codes (sent via CONFIG_CMD, config_key=LED_CONFIG_KEY)
LED_CONFIG_KEY = 0x10  # config key byte for LED commands
LED_OFF = 0x00
LED_GREEN = 0x01  # monitoring, all clear
LED_GREEN_BLINK = 0x02  # mapping perimeter
LED_YELLOW = 0x03  # possible detection (VLM analyzing)
LED_YELLOW_BLINK = 0x04  # investigating (navigating to zone)
LED_RED = 0x05  # confirmed detection, recording
LED_RED_BLINK = 0x06  # active tracking
LED_RED_STROBE = 0x07  # panic / deterrent
LED_BLUE = 0x08  # returning to dock
LED_BLUE_BLINK = 0x09  # low battery
LED_WHITE_FLASH = 0x0A  # photo taken (feedback)

# Buzzer config (sent via CONFIG_CMD, config_key=BUZZER_CONFIG_KEY)
BUZZER_CONFIG_KEY = 0x15
BUZZER_OFF = 0x00
BUZZER_BEEP = 0x01  # 3 short beeps (alert acknowledgement)
BUZZER_SIREN = 0x02  # continuous siren (deterrent)
BUZZER_CHIRP = 0x03  # single chirp (confirmation feedback)

# Deterrent hardware config keys (sent via CONFIG_CMD)
SIREN_CONFIG_KEY = 0x16  # siren module (12V via MOSFET)
SPOTLIGHT_CONFIG_KEY = 0x17  # LED 10W COB spotlight (via MOSFET)
LASER_CONFIG_KEY = 0x18  # green laser 532nm (via MOSFET)
SERVO_PAN_KEY = 0x19  # pan servo angle (0-180 degrees)
SERVO_TILT_KEY = 0x1A  # tilt servo angle (0-180 degrees)
SPEAKER_CONFIG_KEY = 0x1B  # PAM8403 amplifier + speaker

# Device on/off values (siren, spotlight, laser)
DEVICE_OFF = 0x00
DEVICE_ON = 0x01
SPOTLIGHT_STROBE = 0x02  # strobing mode for spotlight

# Speaker audio file IDs
SPEAKER_STOP = 0x00
SPEAKER_WARNING = 0x01  # "ATENCIÓN. ZONA VIGILADA."
SPEAKER_DOG_BARK = 0x02  # dog bark loop
SPEAKER_SIREN_FX = 0x03  # siren sound effect (software, not hardware siren)

# Digital sensor flags (bit flags in sensor_flags u16 field)
SENSOR_FLAG_PIR = 0x0001  # PIR motion detected
SENSOR_FLAG_SOUND = 0x0002  # Sound sensor triggered (glass break, impact)
SENSOR_FLAG_IR = 0x0004  # IR proximity triggered


def crc8(data: bytes) -> int:
    """CRC-8/MAXIM (polynomial 0x31)."""
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x31) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def build_packet(pkt_type: int, payload: bytes) -> bytes:
    """Build a wire-format packet."""
    header = MAGIC + struct.pack("<BH", pkt_type, len(payload))
    frame = header + payload
    return frame + bytes([crc8(frame)])


def parse_packet(data: bytes) -> Optional[tuple[int, bytes]]:
    """Parse a wire-format packet. Returns (type, payload) or None."""
    if len(data) < 6:
        return None
    if data[:2] != MAGIC:
        return None
    pkt_type, length = struct.unpack_from("<BH", data, 2)
    total = 5 + length + 1
    if len(data) < total:
        return None
    payload = data[5 : 5 + length]
    if data[5 + length] != crc8(data[: 5 + length]):
        return None
    return (pkt_type, payload)


#: Largest legitimate packet payload we will ever accept. Anything
#: bigger is treated as malformed/hostile so a corrupt or malicious
#: peer can't make us readexactly() up to 64 KiB on every read and
#: hang the connection (slowloris-style DoS).
MAX_PAYLOAD_BYTES = 4 * 1024


#
# ── Optional brain↔kernel auth envelope wiring ───────────────────────────────
#
# If a link key is configured (`ROBOT_BRAIN_LINK_KEY` env), `send_packet` and
# `read_packet` automatically wrap/unwrap each frame with the HMAC envelope
# from `secure_channel.py` (which pairs byte-for-byte with the kernel's
# `crates/behavior/src/auth_envelope.rs`). Without the env var the channel
# stays plaintext and these calls behave exactly as before.
#
# The module-level singletons reflect the single brain↔kernel TCP connection
# the server currently supports; if multi-connection support is added, move
# Sender/Receiver state per-connection.
#
if TYPE_CHECKING:
    from secure_channel import Sender, Receiver, SecureChannel

_link_sender: "Sender | None" = None
_link_receiver: "Receiver | None" = None


def enable_auth_envelope() -> bool:
    """Activate the auth envelope based on `ROBOT_BRAIN_LINK_KEY`.
    Returns True iff a valid key was loaded. Idempotent.
    Call once at server startup (after env is set). If no key, the calls
    below remain plaintext and the kernel side likewise falls back to
    identity (see `auth_envelope::is_authenticated`)."""
    global _link_sender, _link_receiver
    # Late import so test code that just imports protocol doesn't pay for
    # secure_channel's hashlib at import time.
    from secure_channel import load_link_key, Sender, Receiver

    key = load_link_key()
    if key is None:
        _link_sender = None
        _link_receiver = None
        return False
    _link_sender = Sender(key)
    # In HMAC-only keyed mode this Receiver deliberately lives for the whole
    # process: its rx_high_water watermark is the ONLY cross-reboot replay
    # defence for ESTOP/actuator frames, so it must survive kernel reconnects.
    # Consequence (accepted by design): the kernel seeds its envelope
    # SEND_NONCE from the CLINT timer, which restarts near zero on reboot, so
    # after a kernel reboot every kernel→brain nonce sits below this
    # watermark and is dropped as replay until the brain restarts. The exits
    # are kernel-side nonce persistence/RTC seeding, or enabling the RFC-0019
    # AEAD layer — with AEAD, `perform_handshake()` re-creates this Receiver
    # per connection (see the comment there for why that is sound).
    _link_receiver = Receiver(key)
    return True


#
# ── Optional RFC-0019 encrypted link (AEAD over the HMAC envelope) ────────────
#
# When `ROBOT_BRAIN_ENCRYPT_LINK=1` AND a link key is configured, the wire
# frame becomes  AEAD( HMAC_envelope( brain_protocol_frame ) ).  The nesting
# is intentional (RFC-0019 §"Encrypted frame format"): the outer AES-CTR nonce
# stops in-session replay; the inner envelope nonce stops cross-session replay.
# The handshake PSK *is* the link key — one provisioned secret, same as the
# kernel's `/fat/LINK.KEY`.
#
# `_secure_channel` is set per TCP connection by `perform_handshake()` once the
# RFC-0019 handshake reaches ESTABLISHED; it is None before/without encryption.
#
_encrypt_psk: "bytes | None" = None
_secure_channel: "SecureChannel | None" = None

#: RFC-0019 responder HELLO reply size: mode(1) + label(1) + pubkey(32) +
#: proof(32). The initiator (brain) reads exactly this from the kernel.
_HELLO_REPLY_BYTES = 2 + 32 + 32  # 66


def enable_encrypt_link() -> bool:
    """Arm RFC-0019 encryption. Idempotent.

    `ROBOT_BRAIN_ENCRYPT_LINK` semantics (default-on since 2026-08-23, in
    lockstep with the kernel's `link_encrypt=1` default — both sides MUST
    agree):

      * ``"0"``  — explicitly off (bring-up / debug).
      * ``"1"``  — explicitly on; raises RuntimeError if no link key is
        configured — RFC-0019 §"Opt-in flags (no silent fallback)" mandates
        a loud failure rather than quietly downgrading a secure deployment
        to plaintext.
      * unset — **armed iff a link key is configured** (HMAC envelope active
        and the key loads). A provisioned brain encrypts automatically; a
        keyless dev brain keeps working instead of refusing to start,
        because with no key there is no secure deployment to downgrade.

    The HMAC envelope (`enable_auth_envelope`) must already be active: it is
    the inner layer and the source of the PSK. Returns True iff encryption
    is armed.

    Arming only records intent + the PSK; the actual `SecureChannel` (with
    fresh per-connection ephemeral keys) is created in `perform_handshake`.
    """
    global _encrypt_psk
    flag = os.environ.get("ROBOT_BRAIN_ENCRYPT_LINK", "").strip()
    if flag == "0":
        _encrypt_psk = None
        return False
    explicit = flag == "1"
    if _link_sender is None or _link_receiver is None:
        if explicit:
            raise RuntimeError(
                "ROBOT_BRAIN_ENCRYPT_LINK=1 but the HMAC envelope is not active "
                "(set ROBOT_BRAIN_LINK_KEY) — refusing to start "
                "(RFC-0019: no silent fallback to plaintext)"
            )
        _encrypt_psk = None
        return False
    from secure_channel import load_link_key

    key = load_link_key()
    if key is None:
        if explicit:
            raise RuntimeError(
                "ROBOT_BRAIN_ENCRYPT_LINK=1 but ROBOT_BRAIN_LINK_KEY is unset/"
                "invalid — refusing to start (RFC-0019: no silent fallback)"
            )
        _encrypt_psk = None
        return False
    _encrypt_psk = key
    return True


def encrypt_link_armed() -> bool:
    """True iff RFC-0019 encryption is armed (a handshake must run per
    connection before the channel is usable)."""
    return _encrypt_psk is not None


def reset_secure_channel() -> None:
    """Drop the established channel — call on disconnect so the next
    connection performs a fresh handshake with new ephemeral keys.

    Deliberately does NOT touch `_link_receiver`: a connection loss must not
    reset the envelope replay watermark. The Receiver is only re-created on a
    *successful* AEAD handshake (see `perform_handshake`), where fresh session
    keys make that sound; in HMAC-only mode it persists for the process."""
    global _secure_channel
    _secure_channel = None


async def perform_handshake(
    reader: asyncio.StreamReader, writer: asyncio.StreamWriter, timeout: float = 10.0
) -> bool:
    """Brain-side (initiator) RFC-0019 handshake. Call once, immediately after
    the TCP connection is accepted and BEFORE any packet is sent.

    Returns True on success (channel ESTABLISHED) or when encryption is not
    armed (no-op pass-through). Returns False on any handshake failure — the
    caller MUST drop the connection (no silent plaintext fallback).

    `timeout` is deliberately generous: under QEMU TCG the kernel's X25519
    work is wall-clock slow (~830k cycles measured), so a tight deadline would
    make the live handshake flaky for the same reason the TCP handshake
    deadline had to move 500ms→2s.
    """
    global _secure_channel, _link_receiver
    if _encrypt_psk is None:
        return True  # encryption not armed → nothing to do
    from secure_channel import Receiver, SecureChannel

    sc = SecureChannel(_encrypt_psk, is_initiator=True)
    try:
        writer.write(sc.start_handshake())  # [0x02][HELLO][pub] 34B
        await writer.drain()
        reply = await asyncio.wait_for(reader.readexactly(_HELLO_REPLY_BYTES), timeout)  # 66B
        confirm = sc.handle_peer_hello(reply)  # [0x02][CONFIRM][proof] 34B
        writer.write(confirm)
        await writer.drain()
    except (
        asyncio.TimeoutError,
        asyncio.IncompleteReadError,
        ValueError,
        PermissionError,
        RuntimeError,
    ) as e:
        print(f"[BRAIN] secure_channel: RFC-0019 handshake failed: {e}")
        _secure_channel = None
        return False
    if not sc.is_established:
        _secure_channel = None
        return False
    _secure_channel = sc
    # AEAD handshake succeeded for THIS connection → re-create the inner
    # envelope Receiver so its replay watermark (rx_high_water) starts fresh.
    # Without this, the kernel's CLINT-seeded SEND_NONCE restarting near zero
    # after a reboot would leave every kernel→brain frame below the old
    # watermark, permanently dropped as replay until the brain restarts.
    # Resetting the watermark here reopens nothing: with AEAD armed,
    # cross-session/cross-reboot replay is already defeated by the fresh
    # per-connection session keys — an outer AEAD frame recorded from an old
    # session cannot decrypt under this session's keys, so no old inner
    # envelope can ever reach this Receiver. The reset happens ONLY on a
    # successful handshake; handshake failure and plain disconnects
    # (`reset_secure_channel`) leave the Receiver untouched, so HMAC-only
    # mode keeps its process-lifetime watermark.
    _link_receiver = Receiver(_encrypt_psk)
    print("[BRAIN] secure_channel: RFC-0019 encrypted link established")
    return True


#
# ── Optional RFC-0021 multi-stream framing (OUTERMOST layer) ──────────────────
#
# When `ROBOT_BRAIN_MULTI_STREAM=1`, every wire frame is prefixed with a
# 3-byte multi-stream header `[stream_id][len LE u16]` (multi_stream.py, mirror
# of crates/multi-stream). The control traffic (brain protocol, optionally
# HMAC/AEAD-wrapped) rides STREAM_CONTROL; camera/lidar/etc. get their own
# stream-ids (Phase D). Demux happens BEFORE decode, so unencrypted bulk
# streams (video) and the encrypted control stream coexist on one TCP conn.
# Independent of the encryption flags — composes outside them.
#
_multi_stream_armed: bool = False


def enable_multi_stream() -> bool:
    """Arm RFC-0021 multi-stream framing if `ROBOT_BRAIN_MULTI_STREAM=1`.
    Pure framing (no key); composes outside the HMAC/AEAD layers. Idempotent."""
    global _multi_stream_armed
    _multi_stream_armed = os.environ.get("ROBOT_BRAIN_MULTI_STREAM", "").strip() == "1"
    return _multi_stream_armed


def multi_stream_armed() -> bool:
    return _multi_stream_armed


def _decode_inner(frame: bytes) -> Optional[tuple[int, bytes]]:
    """Decode a COMPLETE inner frame (already de-multiplexed) into
    (pkt_type, payload), applying whichever lower layers are active:
    AEAD→HMAC→parse, or HMAC→parse, or plain parse. Returns None on any
    decrypt/HMAC/parse failure."""
    if _secure_channel is not None:
        assert _link_receiver is not None  # RFC-0019: AEAD ⟹ HMAC armed together
        envelope = _secure_channel.decrypt(frame)
        if envelope is None:
            return None
        inner = _link_receiver.unwrap(envelope)
        if inner is None:
            return None
        return parse_packet(inner)
    if _link_receiver is not None:
        inner = _link_receiver.unwrap(frame)
        if inner is None:
            return None
        return parse_packet(inner)
    return parse_packet(frame)


async def read_packet(reader: asyncio.StreamReader) -> Optional[tuple[int, bytes]]:
    """Read one packet from an asyncio StreamReader."""
    if _multi_stream_armed:
        # Outermost framing: read [stream_id][len][payload], demux by stream-id,
        # then decode the inner frame. Non-control streams (camera/etc.) are
        # drained and skipped here — the control reader only yields STREAM_CONTROL
        # packets (video routing is Phase D, a separate consumer).
        import multi_stream as _ms

        while True:
            head = await reader.readexactly(_ms.HEADER_LEN)
            stream_id = head[0]
            plen = int.from_bytes(head[_ms.STREAM_ID_BYTES : _ms.HEADER_LEN], "little")
            if plen > _ms.MAX_PAYLOAD_LEN:
                return None
            payload = await reader.readexactly(plen)
            if stream_id == _ms.STREAM_CONTROL:
                return _decode_inner(payload)
            # else: non-control stream — ignore in the control reader for now.
    if _secure_channel is not None:
        # Encrypted mode: read one AEAD frame, decrypt to the inner HMAC
        # envelope, unwrap that, then parse the now-plaintext brain frame.
        # AEAD wire layout: nonce(12) + len(2 LE) + ciphertext(N) + hmac(32).
        from secure_channel import AEAD_NONCE_SIZE, AEAD_LEN_SIZE, AEAD_HMAC_SIZE, AEAD_MAX_PAYLOAD

        head = await reader.readexactly(AEAD_NONCE_SIZE + AEAD_LEN_SIZE)
        n = int.from_bytes(head[AEAD_NONCE_SIZE : AEAD_NONCE_SIZE + AEAD_LEN_SIZE], "little")
        if n > AEAD_MAX_PAYLOAD:
            # Oversize ciphertext claim — drop (slowloris guard, mirrors the
            # plaintext-path length check below).
            return None
        body = await reader.readexactly(n + AEAD_HMAC_SIZE)
        assert _link_receiver is not None  # RFC-0019: AEAD ⟹ HMAC armed together
        envelope = _secure_channel.decrypt(head + body)
        if envelope is None:
            return None  # HMAC/decrypt failure — drop
        unwrapped = _link_receiver.unwrap(envelope)
        if unwrapped is None:
            return None  # inner envelope replay/HMAC failure — drop
        return parse_packet(unwrapped)
    if _link_receiver is not None:
        # Authenticated mode: read one envelope, unwrap to get the inner
        # brain-protocol frame, then parse that. ENVELOPE_OVERHEAD = 26
        # (8 nonce + 16 mac + 2 len) — same constants as the kernel.
        from secure_channel import ENVELOPE_OVERHEAD, NONCE_BYTES, HMAC_BYTES, LEN_BYTES

        head = await reader.readexactly(ENVELOPE_OVERHEAD)
        inner_len = int.from_bytes(
            head[NONCE_BYTES + HMAC_BYTES : NONCE_BYTES + HMAC_BYTES + LEN_BYTES],
            "little",
        )
        if inner_len > MAX_PAYLOAD_BYTES + 6:  # +header+CRC bytes
            return None
        inner = await reader.readexactly(inner_len)
        unwrapped = _link_receiver.unwrap(head + inner)
        if unwrapped is None:
            return None
        # Parse the now-plaintext brain-protocol frame.
        return parse_packet(unwrapped)
    # Plaintext path (unchanged).
    header = await reader.readexactly(5)
    if header[:2] != MAGIC:
        return None
    pkt_type, length = struct.unpack_from("<BH", header, 2)
    if length > MAX_PAYLOAD_BYTES:
        # Refuse oversize. Returning None signals the caller to drop
        # this connection; without this guard a peer claiming length=65535
        # makes us block readexactly(65536) forever even if they only
        # intend to send a few bytes.
        return None
    rest = await reader.readexactly(length + 1)
    payload = rest[:length]
    if crc8(header + payload) != rest[length]:
        return None
    return (pkt_type, payload)


async def send_packet(writer: asyncio.StreamWriter, pkt_type: int, payload: bytes) -> None:
    """Send one packet via an asyncio StreamWriter."""
    frame = build_packet(pkt_type, payload)
    # Lower layers: AEAD(HMAC(frame)) / HMAC(frame) / frame.
    if _secure_channel is not None:
        assert _link_sender is not None  # RFC-0019: AEAD ⟹ HMAC armed together
        inner = _secure_channel.encrypt(_link_sender.wrap(frame))
    elif _link_sender is not None:
        inner = _link_sender.wrap(frame)
    else:
        inner = frame
    # Outermost: multi-stream framing on STREAM_CONTROL when armed.
    if _multi_stream_armed:
        import multi_stream as _ms

        inner = _ms.wrap(_ms.STREAM_CONTROL, inner)
    writer.write(inner)
    await writer.drain()


# ── SensorPacket — common header + per-type payload ──────────────────────────

# Common header: timestamp(8) + battery(2) + accel(12) + gyro(12) = 34 bytes
_HDR_FMT = "<Q3i3iH"  # 34 bytes
_HDR_SIZE = struct.calcsize(_HDR_FMT)  # 34

# Wheeled extra: odom_dist(4) + odom_hdg(4) + enc_l(8) + enc_r(8)
#                + range_front(2) + range_right(2) = 28 bytes
# Extended: + sensor_flags(2) = 30 bytes
_WHL_FMT = "<2i2q2H"
_WHL_SIZE = struct.calcsize(_WHL_FMT)  # 28
_WHL_FLAGS_FMT = "<H"
_WHL_FLAGS_SIZE = struct.calcsize(_WHL_FLAGS_FMT)  # 2

# Drone extra: baro(4) + mag(6) + gps_lat(4) + gps_lon(4) + gps_alt(4) + sonar(2) = 24 bytes
_DRN_FMT = "<i3h3iH"
_DRN_SIZE = struct.calcsize(_DRN_FMT)  # 24 — but header already has battery


@dataclass
class SensorPacket:
    """Wheeled robot sensor packet (common header + wheeled payload).

    Extended format includes sensor_flags (u16 bit flags for PIR/sound/IR).
    Parser auto-detects legacy (62B) vs extended (64B) payloads.
    """

    # Common header
    timestamp_ms: int
    battery_mv: int
    accel_mg: tuple[int, int, int]
    gyro_mdps: tuple[int, int, int]
    # Wheeled payload
    odom_dist_mm: int
    odom_hdg_cdeg: int
    encoder_l: int
    encoder_r: int
    range_front_mm: int
    range_right_mm: int
    # Extended: digital sensor flags (PIR, sound, IR)
    sensor_flags: int = 0

    ROBOT_TYPE = ROBOT_WHEELED

    #: Minimum payload length (header + wheeled fields). `sensor_flags` is an
    #: optional extension beyond this — see the auto-detect below.
    MIN_SIZE = _HDR_SIZE + _WHL_SIZE

    @classmethod
    def from_bytes(cls, data: bytes) -> "SensorPacket":
        if len(data) < cls.MIN_SIZE:
            raise ValueError(f"wheeled sensor packet truncated: {len(data)} < {cls.MIN_SIZE} bytes")
        ts, ax, ay, az, gx, gy, gz, batt = struct.unpack_from(_HDR_FMT, data)
        od, oh, el, er, rf, rr = struct.unpack_from(_WHL_FMT, data, _HDR_SIZE)
        # Auto-detect extended format with sensor_flags
        flags = 0
        if len(data) >= _HDR_SIZE + _WHL_SIZE + _WHL_FLAGS_SIZE:
            (flags,) = struct.unpack_from(_WHL_FLAGS_FMT, data, _HDR_SIZE + _WHL_SIZE)
        return cls(
            timestamp_ms=ts,
            battery_mv=batt,
            accel_mg=(ax, ay, az),
            gyro_mdps=(gx, gy, gz),
            odom_dist_mm=od,
            odom_hdg_cdeg=oh,
            encoder_l=el,
            encoder_r=er,
            range_front_mm=rf,
            range_right_mm=rr,
            sensor_flags=flags,
        )

    def to_bytes(self) -> bytes:
        hdr = struct.pack(
            _HDR_FMT, self.timestamp_ms, *self.accel_mg, *self.gyro_mdps, self.battery_mv
        )
        whl = struct.pack(
            _WHL_FMT,
            self.odom_dist_mm,
            self.odom_hdg_cdeg,
            self.encoder_l,
            self.encoder_r,
            self.range_front_mm,
            self.range_right_mm,
        )
        flags = struct.pack(_WHL_FLAGS_FMT, self.sensor_flags)
        return hdr + whl + flags


@dataclass
class SensorPacketDrone:
    """Drone sensor packet (common header + drone payload)."""

    # Common header
    timestamp_ms: int
    battery_mv: int
    accel_mg: tuple[int, int, int]
    gyro_mdps: tuple[int, int, int]
    # Drone payload
    baro_pa: int
    mag_ut: tuple[int, int, int]
    gps_lat_deg7: int
    gps_lon_deg7: int
    gps_alt_cm: int
    sonar_down_mm: int

    ROBOT_TYPE = ROBOT_DRONE

    #: Minimum payload length (header + drone fields).
    MIN_SIZE = _HDR_SIZE + _DRN_SIZE

    @classmethod
    def from_bytes(cls, data: bytes) -> "SensorPacketDrone":
        if len(data) < cls.MIN_SIZE:
            raise ValueError(f"drone sensor packet truncated: {len(data)} < {cls.MIN_SIZE} bytes")
        ts, ax, ay, az, gx, gy, gz, batt = struct.unpack_from(_HDR_FMT, data)
        baro, mx, my, mz, lat, lon, alt, sonar = struct.unpack_from(_DRN_FMT, data, _HDR_SIZE)
        return cls(
            timestamp_ms=ts,
            battery_mv=batt,
            accel_mg=(ax, ay, az),
            gyro_mdps=(gx, gy, gz),
            baro_pa=baro,
            mag_ut=(mx, my, mz),
            gps_lat_deg7=lat,
            gps_lon_deg7=lon,
            gps_alt_cm=alt,
            sonar_down_mm=sonar,
        )

    def to_bytes(self) -> bytes:
        hdr = struct.pack(
            _HDR_FMT, self.timestamp_ms, *self.accel_mg, *self.gyro_mdps, self.battery_mv
        )
        drn = struct.pack(
            _DRN_FMT,
            self.baro_pa,
            *self.mag_ut,
            self.gps_lat_deg7,
            self.gps_lon_deg7,
            self.gps_alt_cm,
            self.sonar_down_mm,
        )
        return hdr + drn


@dataclass
class SensorPacketHumanoid:
    """Humanoid sensor packet (common header + joint angles + foot pressure)."""

    # Common header
    timestamp_ms: int
    battery_mv: int
    accel_mg: tuple[int, int, int]
    gyro_mdps: tuple[int, int, int]
    # Humanoid payload
    joint_angles: list[int]  # centidegrees, variable length
    foot_pressure_l: int  # mN
    foot_pressure_r: int  # mN

    ROBOT_TYPE = ROBOT_HUMANOID

    # Anti-DoS bound: real humanoids have <= 32 actuated joints. Reject
    # packets claiming more so a malformed kernel/man-in-the-middle can't
    # cause struct.unpack_from to either read past the buffer (raising
    # struct.error and spamming the asyncio handler) or — worse on
    # platforms where bytes are pre-padded — consume all available memory.
    MAX_JOINTS = 32

    @classmethod
    def from_bytes(cls, data: bytes) -> "SensorPacketHumanoid":
        if len(data) < _HDR_SIZE + 1:
            raise ValueError("humanoid sensor packet truncated (header)")
        ts, ax, ay, az, gx, gy, gz, batt = struct.unpack_from(_HDR_FMT, data)
        offset = _HDR_SIZE
        num_joints = data[offset]
        offset += 1
        if num_joints > cls.MAX_JOINTS:
            raise ValueError(
                f"humanoid sensor packet num_joints={num_joints} > MAX={cls.MAX_JOINTS}"
            )
        joints_bytes = num_joints * 2
        if len(data) < offset + joints_bytes + 4:
            raise ValueError("humanoid sensor packet truncated (joints+feet)")
        joints = list(struct.unpack_from(f"<{num_joints}h", data, offset))
        offset += joints_bytes
        fl, fr = struct.unpack_from("<HH", data, offset)
        return cls(
            timestamp_ms=ts,
            battery_mv=batt,
            accel_mg=(ax, ay, az),
            gyro_mdps=(gx, gy, gz),
            joint_angles=joints,
            foot_pressure_l=fl,
            foot_pressure_r=fr,
        )

    def to_bytes(self) -> bytes:
        n = len(self.joint_angles)
        hdr = struct.pack(
            _HDR_FMT, self.timestamp_ms, *self.accel_mg, *self.gyro_mdps, self.battery_mv
        )
        joints = struct.pack(f"<B{n}h", n, *self.joint_angles)
        foot = struct.pack("<HH", self.foot_pressure_l, self.foot_pressure_r)
        return hdr + joints + foot


def sensor_packet_from_bytes(
    robot_type: int, data: bytes
) -> Union["SensorPacket", "SensorPacketDrone", "SensorPacketHumanoid"]:
    """Parse a sensor packet according to robot type."""
    if robot_type == ROBOT_WHEELED:
        return SensorPacket.from_bytes(data)
    if robot_type == ROBOT_DRONE:
        return SensorPacketDrone.from_bytes(data)
    if robot_type == ROBOT_HUMANOID:
        return SensorPacketHumanoid.from_bytes(data)
    return SensorPacket.from_bytes(data)  # fallback to wheeled


# ── SensorCompact — low-bandwidth (LoRa/RF) ──────────────────────────────────


@dataclass
class SensorCompact:
    """Compact sensor packet for low-bandwidth links (20 bytes).

    NOTE: Digital sensor flags (PIR motion, IR proximity, sound detection)
    could be carried as bit flags in the 'mode' byte's upper bits or in a
    dedicated flags field if the format is extended. Currently unused bits
    in 'mode' (bits 3-7) are available for this purpose.
    """

    lat_deg7: int  # latitude × 1e7
    lon_deg7: int  # longitude × 1e7
    alt_cm: int  # altitude cm
    battery_mv: int
    mode: int
    gps_fix: int
    speed_cms: int
    heading_cdeg: int

    FORMAT = "<iiHHBBHH"  # 20 bytes

    @classmethod
    def from_bytes(cls, data: bytes) -> "SensorCompact":
        lat, lon, alt, batt, mode, fix, spd, hdg = struct.unpack(cls.FORMAT, data)
        return cls(
            lat_deg7=lat,
            lon_deg7=lon,
            alt_cm=alt,
            battery_mv=batt,
            mode=mode,
            gps_fix=fix,
            speed_cms=spd,
            heading_cdeg=hdg,
        )

    def to_bytes(self) -> bytes:
        return struct.pack(
            self.FORMAT,
            self.lat_deg7,
            self.lon_deg7,
            self.alt_cm,
            self.battery_mv,
            self.mode,
            self.gps_fix,
            self.speed_cms,
            self.heading_cdeg,
        )


# ── ActuatorCmd — generic (replaces VelocityCmd) ─────────────────────────────


@dataclass
class ActuatorCmd:
    """Generic actuator command for any robot type.

    actuator_type: 0=diff_drive, 1=quad_rotor, 2=humanoid, 3=ackermann
    channels:      list of i16 values (meaning depends on actuator_type)
      diff_drive:  [speed_l, speed_r]            (-100..100)
      quad_rotor:  [throttle, roll, pitch, yaw]  (PWM 1000-2000)
      humanoid:    [joint_0_cdeg, ..., joint_N]  (centidegrees)
      ackermann:   [speed, steer_angle_cdeg]
    flags: bit0=emergency_stop, bit1=alert
    """

    actuator_type: int
    channels: list[int]
    flags: int = 0

    def to_bytes(self) -> bytes:
        n = len(self.channels)
        return struct.pack(f"<BBB{n}h", self.actuator_type, n, self.flags, *self.channels)

    @classmethod
    def from_bytes(cls, data: bytes) -> "ActuatorCmd":
        act_type, n, flags = struct.unpack_from("<BBB", data)
        channels = list(struct.unpack_from(f"<{n}h", data, 3))
        return cls(actuator_type=act_type, channels=channels, flags=flags)

    @classmethod
    def stop(cls, actuator_type: int = ACT_DIFF_DRIVE, n_channels: int = 2) -> "ActuatorCmd":
        """Emergency stop for any robot type."""
        return cls(actuator_type=actuator_type, channels=[0] * n_channels, flags=FLAG_EMERGENCY)

    @classmethod
    def wheeled(cls, speed_l: int, speed_r: int, flags: int = 0) -> "ActuatorCmd":
        """Convenience constructor for differential drive."""
        return cls(actuator_type=ACT_DIFF_DRIVE, channels=[speed_l, speed_r], flags=flags)

    @classmethod
    def drone(cls, throttle: int, roll: int, pitch: int, yaw: int, flags: int = 0) -> "ActuatorCmd":
        """Convenience constructor for quad rotor."""
        return cls(actuator_type=ACT_QUAD_ROTOR, channels=[throttle, roll, pitch, yaw], flags=flags)


# ── PredictCmd — RFC-0034 speculative actuation ──────────────────────────────


@dataclass
class PredictCmd:
    """The brain's predicted NEXT actuator command, plus a confidence byte.

    Sent under PKT_PREDICT so the kernel can act on it ahead of the confirmed
    command (gated by the Fase-1 safety envelope + SPECULATIVE_ACTUATION, default
    off). Wire format mirrors the kernel `decode_predict_cmd`: the ActuatorCmd
    bytes followed by one trailing confidence byte (0..=255; 255 = certain, e.g.
    a deterministic scripted step).
    """

    cmd: ActuatorCmd
    confidence: int = 255  # 0..255

    def to_bytes(self) -> bytes:
        return self.cmd.to_bytes() + bytes([max(0, min(255, self.confidence))])

    @classmethod
    def from_bytes(cls, data: bytes) -> "PredictCmd":
        # Last byte is confidence; the rest is the ActuatorCmd.
        return cls(cmd=ActuatorCmd.from_bytes(data[:-1]), confidence=data[-1])


# ── VelocityCmd — kept for backward compatibility ────────────────────────────


@dataclass
class VelocityCmd:
    """Differential drive command (backward compat). Prefer ActuatorCmd.wheeled()."""

    speed_l: int
    speed_r: int
    flags: int = 0

    # Legacy wire format (i32 channels). The KERNEL only knows the ActuatorCmd
    # format (i16 channels) under PKT_ACTUATOR — sending raw `<iiB>` bytes would
    # silently corrupt the wire. `to_bytes()` therefore now emits the
    # ActuatorCmd encoding; the `_LEGACY_FORMAT` is kept only so `from_bytes`
    # can still decode any old recorded log frames.
    _LEGACY_FORMAT = "<iiB"  # 9 bytes — DO NOT use for new wire-level sends.

    def to_bytes(self) -> bytes:
        # Route through the ActuatorCmd encoding so any caller that still does
        # `vel.to_bytes()` (legacy path) produces a kernel-compatible frame.
        return self.to_actuator_cmd().to_bytes()

    @classmethod
    def from_bytes(cls, data: bytes) -> "VelocityCmd":
        sl, sr, flags = struct.unpack(cls._LEGACY_FORMAT, data)
        return cls(speed_l=sl, speed_r=sr, flags=flags)

    def to_actuator_cmd(self) -> ActuatorCmd:
        return ActuatorCmd.wheeled(self.speed_l, self.speed_r, self.flags)


# ── WaypointCmd (PKT_WAYPOINT = 0x82) ────────────────────────────────────────
#
# Server → Robot: a GPS waypoint to fly/drive to. Layout must byte-match the
# kernel's `decode_waypoint_cmd` in `crates/behavior/src/brain_protocol.rs`
# (14-byte payload, little-endian).

# Waypoint action codes (must match the kernel-side values).
WAYPOINT_ACT_GOTO: int = 0
WAYPOINT_ACT_LOITER: int = 1
WAYPOINT_ACT_LAND: int = 2
WAYPOINT_ACT_RTL: int = 3


@dataclass
class WaypointCmd:
    """GPS waypoint command. Kernel-compatible wire format."""

    lat_deg7: int  # latitude × 1e7 (i32)
    lon_deg7: int  # longitude × 1e7 (i32)
    alt_cm: int  # altitude in cm (u16, 0..65535)
    speed_cms: int  # target speed in cm/s (u16)
    action: int = WAYPOINT_ACT_GOTO  # u8
    flags: int = 0  # u8 (reserved / future)

    FORMAT = "<iiHHBB"  # 14 bytes, byte-identical to kernel

    def to_bytes(self) -> bytes:
        return struct.pack(
            self.FORMAT,
            self.lat_deg7,
            self.lon_deg7,
            self.alt_cm,
            self.speed_cms,
            self.action,
            self.flags,
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "WaypointCmd":
        lat, lon, alt, spd, act, flg = struct.unpack(cls.FORMAT, data[:14])
        return cls(lat_deg7=lat, lon_deg7=lon, alt_cm=alt, speed_cms=spd, action=act, flags=flg)


# ── EStopCmd (PKT_ESTOP = 0x88) ──────────────────────────────────────────────
#
# Server → Robot: emergency stop override-all-layers command. Single-byte
# payload carrying a reason code that the kernel logs and routes to the L0
# safety layer. Reason values must match `ESTOP_REASON_*` in `brain_protocol.rs`.

ESTOP_REASON_OPERATOR: int = 0  # human operator pressed the button
ESTOP_REASON_SAFETY: int = 1  # safety profile violation
ESTOP_REASON_GEOFENCE: int = 2  # geofence breach


@dataclass
class EStopCmd:
    """Emergency-stop command with reason code (1-byte payload)."""

    reason: int = ESTOP_REASON_OPERATOR

    FORMAT = "<B"  # 1 byte

    def to_bytes(self) -> bytes:
        return struct.pack(self.FORMAT, self.reason)

    @classmethod
    def from_bytes(cls, data: bytes) -> "EStopCmd":
        (r,) = struct.unpack(cls.FORMAT, data[:1])
        return cls(reason=r)


# ── DegradeCmd (PKT_DEGRADE = 0x8A) — RFC-0036 ───────────────────────────────
#
# Server → Robot: arm/clear degraded mode. The brain arms it when it detects a
# situational hazard only it can perceive (most concretely: perception has gone
# blind). In degraded mode the kernel contains the userspace blast radius (every
# write/actuation through a user-task capability is denied at the cap chokepoint)
# while the in-kernel control loop keeps safe-stopping. 1-byte payload = reason;
# reason 0 clears. Values must match `DEGRADE_*` in `brain_protocol.rs`.

DEGRADE_CLEAR: int = 0  # exit degraded mode (brain recovered)
DEGRADE_REASON_PERCEPTION_BLIND: int = 1  # perception failed N cycles in a row
DEGRADE_REASON_SENSOR_INCOHERENT: int = 2  # fused sensor state inconsistent
DEGRADE_REASON_UNMODELLED_HAZARD: int = 3  # situational anomaly

# ── SemanticLevelCmd level indices (PKT_SEMANTIC_LEVEL = 0x8B) — RFC-0037 ─────
#
# Ordered restriction level sent as a single byte. Higher = more restrictive.
# Values must match `DEGRADE_LEVEL_*` consts in `crates/ipc/src/cap.rs` and
# `SAFETY_DEGRADE_*_CAP_PCT` in `crates/behavior/src/safety.rs`.
SEMANTIC_LEVEL_FULL: int = 0  # no extra limit — normal operation
SEMANTIC_LEVEL_CAUTIOUS: int = 1  # speed ceiling 70 % of per-type maximum
SEMANTIC_LEVEL_SLOW: int = 2  # speed ceiling 30 % of per-type maximum
SEMANTIC_LEVEL_CONTAINED: int = 3  # speed ceiling 0 % (stop) + cap-denial


@dataclass
class DegradeCmd:
    """Degraded-mode trigger with reason code (1-byte payload). reason 0 clears."""

    reason: int = DEGRADE_REASON_PERCEPTION_BLIND

    FORMAT = "<B"  # 1 byte

    def to_bytes(self) -> bytes:
        return struct.pack(self.FORMAT, self.reason & 0xFF)

    @classmethod
    def from_bytes(cls, data: bytes) -> "DegradeCmd":
        (r,) = struct.unpack(cls.FORMAT, data[:1])
        return cls(reason=r)


# ── SemanticLevelCmd (PKT_SEMANTIC_LEVEL = 0x8B) — RFC-0037 ──────────────────
#
# Server → Robot: set the graded degrade level. 1-byte payload = level index
# (SEMANTIC_LEVEL_FULL … SEMANTIC_LEVEL_CONTAINED). The kernel clamps any
# out-of-range index to CONTAINED (fail-closed). The level is sticky; comms-loss
# safe-stop is handled by the motor watchdog (500 ms), not a TTL here — same
# pattern as DegradeCmd. Byte layout must match the kernel PKT_SEMANTIC_LEVEL
# handler in `kernel/src/main.rs` (both TCP and UART paths).


@dataclass
class SemanticLevelCmd:
    """Graded degrade-level command (1-byte payload). level 0 = FULL, 3 = CONTAINED."""

    level: int = SEMANTIC_LEVEL_FULL

    FORMAT = "<B"  # 1 byte — level index

    def to_bytes(self) -> bytes:
        return struct.pack(self.FORMAT, self.level & 0xFF)

    @classmethod
    def from_bytes(cls, data: bytes) -> "SemanticLevelCmd":
        (lv,) = struct.unpack(cls.FORMAT, data[:1])
        return cls(level=lv)


# ── StatusPacket ──────────────────────────────────────────────────────────────


@dataclass
class StatusPacket:
    """Status packet from robot. Includes robot_type for protocol negotiation."""

    mode: int
    tasks_ok: int
    canary_ok: int
    uptime_s: int
    robot_type: int = ROBOT_WHEELED  # optional — legacy packets don't have it

    FORMAT_V1 = "<BBBI"  # 7 bytes (legacy, no robot_type): mode, tasks_ok, canary_ok, uptime_s
    FORMAT_V2 = "<BBBIB"  # 8 bytes: mode, tasks_ok, canary_ok, uptime_s, robot_type

    def to_bytes(self) -> bytes:
        return struct.pack(
            self.FORMAT_V2, self.mode, self.tasks_ok, self.canary_ok, self.uptime_s, self.robot_type
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "StatusPacket":
        if len(data) == 7:
            # Legacy format (no robot_type field)
            m, t, c, u = struct.unpack(cls.FORMAT_V1, data)
            return cls(mode=m, tasks_ok=t, canary_ok=c, uptime_s=u)
        m, t, c, u, rt = struct.unpack(cls.FORMAT_V2, data)
        return cls(mode=m, tasks_ok=t, canary_ok=c, uptime_s=u, robot_type=rt)


# ── ConfigCmd (LED + generic config) ─────────────────────────────────────────


@dataclass
class ConfigCmd:
    """Config command to robot. Key-value: config_key(1B) + value(1B) + reserved(2B).

    Used for LED state, power mode, and other runtime configuration.
    """

    config_key: int
    value: int
    reserved: int = 0

    FORMAT = "<BBH"  # 4 bytes: key, value, reserved_u16

    def to_bytes(self) -> bytes:
        return struct.pack(self.FORMAT, self.config_key, self.value, self.reserved)

    @classmethod
    def from_bytes(cls, data: bytes) -> "ConfigCmd":
        k, v, r = struct.unpack(cls.FORMAT, data[:4])
        return cls(config_key=k, value=v, reserved=r)

    @classmethod
    def led(cls, state: int) -> "ConfigCmd":
        """Create a LED state command."""
        return cls(config_key=LED_CONFIG_KEY, value=state)

    @classmethod
    def buzzer(cls, pattern: int) -> "ConfigCmd":
        """Create a buzzer command."""
        return cls(config_key=BUZZER_CONFIG_KEY, value=pattern)


# ── PayloadCmd (E04) — spray pump, gripper servo, camera trigger ──────────────

# Payload types (payload_type field)
PAYLOAD_TYPE_SPRAY = 0  # spray pump (GPIO on/off)
PAYLOAD_TYPE_GRIPPER = 1  # gripper servo (0=closed … 100=open)
PAYLOAD_TYPE_CAM_TRIGGER = 2  # external camera shutter trigger (GPIO pulse)

# Generic on/off values
PAYLOAD_OFF = 0
PAYLOAD_ON = 1

# Gripper position shortcuts
GRIPPER_OPEN = 100
GRIPPER_CLOSED = 0


@dataclass
class PayloadCmd:
    """E04: payload control command (spray, gripper, external camera trigger).

    Wire format (5 bytes): payload_type(1) + channel(1) + value(1) + duration_ms(2 LE)

    payload_type: PAYLOAD_TYPE_SPRAY / GRIPPER / CAM_TRIGGER
    channel:      device index (0 = first of that type)
    value:        0/1 for on/off; 0-100 for gripper position
    duration_ms:  0 = indefinite / one-shot; >0 = auto-off after N ms
    """

    payload_type: int
    channel: int = 0
    value: int = 0
    duration_ms: int = 0

    FORMAT = "<BBBH"  # 5 bytes

    def to_bytes(self) -> bytes:
        return struct.pack(
            self.FORMAT, self.payload_type, self.channel, self.value, self.duration_ms
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "PayloadCmd":
        pt, ch, val, dur = struct.unpack(cls.FORMAT, data[:5])
        return cls(payload_type=pt, channel=ch, value=val, duration_ms=dur)

    @classmethod
    def spray(cls, on: bool, channel: int = 0, duration_ms: int = 0) -> "PayloadCmd":
        """Turn spray pump on or off."""
        return cls(
            payload_type=PAYLOAD_TYPE_SPRAY,
            channel=channel,
            value=PAYLOAD_ON if on else PAYLOAD_OFF,
            duration_ms=duration_ms,
        )

    @classmethod
    def gripper(cls, pos: int, channel: int = 0) -> "PayloadCmd":
        """Set gripper position (0=closed, 100=open)."""
        return cls(payload_type=PAYLOAD_TYPE_GRIPPER, channel=channel, value=max(0, min(100, pos)))

    @classmethod
    def cam_trigger(cls, channel: int = 0) -> "PayloadCmd":
        """Fire external camera shutter trigger pulse."""
        return cls(payload_type=PAYLOAD_TYPE_CAM_TRIGGER, channel=channel, value=1)
