"""Binary protocol for robot <-> brain communication.

Packet format:
  MAGIC (2B) | TYPE (1B) | LEN (2B LE) | PAYLOAD (0-1400B) | CRC8 (1B)

Robot -> Server types (0x01-0x7F):
  0x01  SENSOR_PACKET   wheeled: 62B, drone: 68B, humanoid: variable
  0x02  CAMERA_FRAME    variable
  0x03  STATUS          8 bytes (includes robot_type)
  0x04  SENSOR_COMPACT  20 bytes (low-bandwidth: LoRa/RF)

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

import struct
from dataclasses import dataclass, field
from typing import Optional

MAGIC = b"\x42\x52"  # "BR"

# Packet types — Robot -> Server
SENSOR_PACKET   = 0x01
CAMERA_FRAME    = 0x02
STATUS          = 0x03
SENSOR_COMPACT  = 0x04

# Packet types — Server -> Robot
ACTUATOR_CMD    = 0x80
VELOCITY_CMD    = 0x80  # alias (backward compat)
MODE_CMD        = 0x81
WAYPOINT_CMD    = 0x82
CONFIG_CMD      = 0x83

# Robot types
ROBOT_WHEELED   = 0
ROBOT_DRONE     = 1
ROBOT_HUMANOID  = 2
ROBOT_ACKERMANN = 3

# ActuatorCmd flags
FLAG_EMERGENCY  = 0x01
FLAG_ALERT      = 0x02

# Actuator types (for ActuatorCmd.actuator_type)
ACT_DIFF_DRIVE  = 0
ACT_QUAD_ROTOR  = 1
ACT_HUMANOID    = 2
ACT_ACKERMANN   = 3


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
    payload = data[5:5 + length]
    if data[5 + length] != crc8(data[:5 + length]):
        return None
    return (pkt_type, payload)


async def read_packet(reader) -> Optional[tuple[int, bytes]]:
    """Read one packet from an asyncio StreamReader."""
    header = await reader.readexactly(5)
    if header[:2] != MAGIC:
        return None
    pkt_type, length = struct.unpack_from("<BH", header, 2)
    rest = await reader.readexactly(length + 1)
    payload = rest[:length]
    if crc8(header + payload) != rest[length]:
        return None
    return (pkt_type, payload)


async def send_packet(writer, pkt_type: int, payload: bytes):
    """Send one packet via an asyncio StreamWriter."""
    writer.write(build_packet(pkt_type, payload))
    await writer.drain()


# ── SensorPacket — common header + per-type payload ──────────────────────────

# Common header: timestamp(8) + battery(2) + accel(12) + gyro(12) = 34 bytes
_HDR_FMT  = "<Q3i3iH"   # 34 bytes
_HDR_SIZE = struct.calcsize(_HDR_FMT)  # 34

# Wheeled extra: odom_dist(4) + odom_hdg(4) + enc_l(8) + enc_r(8)
#                + range_front(2) + range_right(2) = 28 bytes
_WHL_FMT  = "<2i2q2H"
_WHL_SIZE = struct.calcsize(_WHL_FMT)  # 28

# Drone extra: baro(4) + mag(6) + gps_lat(4) + gps_lon(4) + gps_alt(4) + sonar(2) = 24 bytes
_DRN_FMT  = "<i3h3iH"
_DRN_SIZE = struct.calcsize(_DRN_FMT)  # 24 — but header already has battery


@dataclass
class SensorPacket:
    """Wheeled robot sensor packet (common header + wheeled payload)."""
    # Common header
    timestamp_ms:   int
    battery_mv:     int
    accel_mg:       tuple[int, int, int]
    gyro_mdps:      tuple[int, int, int]
    # Wheeled payload
    odom_dist_mm:   int
    odom_hdg_cdeg:  int
    encoder_l:      int
    encoder_r:      int
    range_front_mm: int
    range_right_mm: int

    ROBOT_TYPE = ROBOT_WHEELED

    @classmethod
    def from_bytes(cls, data: bytes) -> "SensorPacket":
        ts, ax, ay, az, gx, gy, gz, batt = struct.unpack_from(_HDR_FMT, data)
        od, oh, el, er, rf, rr = struct.unpack_from(_WHL_FMT, data, _HDR_SIZE)
        return cls(
            timestamp_ms=ts, battery_mv=batt,
            accel_mg=(ax, ay, az), gyro_mdps=(gx, gy, gz),
            odom_dist_mm=od, odom_hdg_cdeg=oh,
            encoder_l=el, encoder_r=er,
            range_front_mm=rf, range_right_mm=rr,
        )

    def to_bytes(self) -> bytes:
        hdr = struct.pack(_HDR_FMT,
            self.timestamp_ms, *self.accel_mg, *self.gyro_mdps, self.battery_mv)
        whl = struct.pack(_WHL_FMT,
            self.odom_dist_mm, self.odom_hdg_cdeg,
            self.encoder_l, self.encoder_r,
            self.range_front_mm, self.range_right_mm)
        return hdr + whl


@dataclass
class SensorPacketDrone:
    """Drone sensor packet (common header + drone payload)."""
    # Common header
    timestamp_ms:  int
    battery_mv:    int
    accel_mg:      tuple[int, int, int]
    gyro_mdps:     tuple[int, int, int]
    # Drone payload
    baro_pa:       int
    mag_ut:        tuple[int, int, int]
    gps_lat_deg7:  int
    gps_lon_deg7:  int
    gps_alt_cm:    int
    sonar_down_mm: int

    ROBOT_TYPE = ROBOT_DRONE

    @classmethod
    def from_bytes(cls, data: bytes) -> "SensorPacketDrone":
        ts, ax, ay, az, gx, gy, gz, batt = struct.unpack_from(_HDR_FMT, data)
        baro, mx, my, mz, lat, lon, alt, sonar = struct.unpack_from(_DRN_FMT, data, _HDR_SIZE)
        return cls(
            timestamp_ms=ts, battery_mv=batt,
            accel_mg=(ax, ay, az), gyro_mdps=(gx, gy, gz),
            baro_pa=baro, mag_ut=(mx, my, mz),
            gps_lat_deg7=lat, gps_lon_deg7=lon, gps_alt_cm=alt,
            sonar_down_mm=sonar,
        )

    def to_bytes(self) -> bytes:
        hdr = struct.pack(_HDR_FMT,
            self.timestamp_ms, *self.accel_mg, *self.gyro_mdps, self.battery_mv)
        drn = struct.pack(_DRN_FMT,
            self.baro_pa, *self.mag_ut,
            self.gps_lat_deg7, self.gps_lon_deg7, self.gps_alt_cm,
            self.sonar_down_mm)
        return hdr + drn


@dataclass
class SensorPacketHumanoid:
    """Humanoid sensor packet (common header + joint angles + foot pressure)."""
    # Common header
    timestamp_ms:     int
    battery_mv:       int
    accel_mg:         tuple[int, int, int]
    gyro_mdps:        tuple[int, int, int]
    # Humanoid payload
    joint_angles:     list[int]   # centidegrees, variable length
    foot_pressure_l:  int         # mN
    foot_pressure_r:  int         # mN

    ROBOT_TYPE = ROBOT_HUMANOID

    @classmethod
    def from_bytes(cls, data: bytes) -> "SensorPacketHumanoid":
        ts, ax, ay, az, gx, gy, gz, batt = struct.unpack_from(_HDR_FMT, data)
        offset = _HDR_SIZE
        num_joints = data[offset]
        offset += 1
        joints = list(struct.unpack_from(f"<{num_joints}h", data, offset))
        offset += num_joints * 2
        fl, fr = struct.unpack_from("<HH", data, offset)
        return cls(
            timestamp_ms=ts, battery_mv=batt,
            accel_mg=(ax, ay, az), gyro_mdps=(gx, gy, gz),
            joint_angles=joints, foot_pressure_l=fl, foot_pressure_r=fr,
        )

    def to_bytes(self) -> bytes:
        n = len(self.joint_angles)
        hdr = struct.pack(_HDR_FMT,
            self.timestamp_ms, *self.accel_mg, *self.gyro_mdps, self.battery_mv)
        joints = struct.pack(f"<B{n}h", n, *self.joint_angles)
        foot = struct.pack("<HH", self.foot_pressure_l, self.foot_pressure_r)
        return hdr + joints + foot


def sensor_packet_from_bytes(robot_type: int, data: bytes):
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
    """Compact sensor packet for low-bandwidth links (20 bytes)."""
    lat_deg7:   int   # latitude × 1e7
    lon_deg7:   int   # longitude × 1e7
    alt_cm:     int   # altitude cm
    battery_mv: int
    mode:       int
    gps_fix:    int
    speed_cms:  int
    heading_cdeg: int

    FORMAT = "<iiHHBBHH"  # 20 bytes

    @classmethod
    def from_bytes(cls, data: bytes) -> "SensorCompact":
        lat, lon, alt, batt, mode, fix, spd, hdg = struct.unpack(cls.FORMAT, data)
        return cls(lat_deg7=lat, lon_deg7=lon, alt_cm=alt, battery_mv=batt,
                   mode=mode, gps_fix=fix, speed_cms=spd, heading_cdeg=hdg)

    def to_bytes(self) -> bytes:
        return struct.pack(self.FORMAT,
            self.lat_deg7, self.lon_deg7, self.alt_cm, self.battery_mv,
            self.mode, self.gps_fix, self.speed_cms, self.heading_cdeg)


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
    channels:      list[int]
    flags:         int = 0

    def to_bytes(self) -> bytes:
        n = len(self.channels)
        return struct.pack(f"<BBB{n}h",
            self.actuator_type, n, self.flags, *self.channels)

    @classmethod
    def from_bytes(cls, data: bytes) -> "ActuatorCmd":
        act_type, n, flags = struct.unpack_from("<BBB", data)
        channels = list(struct.unpack_from(f"<{n}h", data, 3))
        return cls(actuator_type=act_type, channels=channels, flags=flags)

    @classmethod
    def stop(cls, actuator_type: int = ACT_DIFF_DRIVE, n_channels: int = 2) -> "ActuatorCmd":
        """Emergency stop for any robot type."""
        return cls(actuator_type=actuator_type, channels=[0] * n_channels,
                   flags=FLAG_EMERGENCY)

    @classmethod
    def wheeled(cls, speed_l: int, speed_r: int, flags: int = 0) -> "ActuatorCmd":
        """Convenience constructor for differential drive."""
        return cls(actuator_type=ACT_DIFF_DRIVE, channels=[speed_l, speed_r], flags=flags)

    @classmethod
    def drone(cls, throttle: int, roll: int, pitch: int, yaw: int, flags: int = 0) -> "ActuatorCmd":
        """Convenience constructor for quad rotor."""
        return cls(actuator_type=ACT_QUAD_ROTOR,
                   channels=[throttle, roll, pitch, yaw], flags=flags)


# ── VelocityCmd — kept for backward compatibility ────────────────────────────

@dataclass
class VelocityCmd:
    """Differential drive command (backward compat). Prefer ActuatorCmd.wheeled()."""
    speed_l: int
    speed_r: int
    flags:   int = 0

    FORMAT = "<iiB"  # 9 bytes

    def to_bytes(self) -> bytes:
        return struct.pack(self.FORMAT, self.speed_l, self.speed_r, self.flags)

    @classmethod
    def from_bytes(cls, data: bytes) -> "VelocityCmd":
        sl, sr, flags = struct.unpack(cls.FORMAT, data)
        return cls(speed_l=sl, speed_r=sr, flags=flags)

    def to_actuator_cmd(self) -> ActuatorCmd:
        return ActuatorCmd.wheeled(self.speed_l, self.speed_r, self.flags)


# ── StatusPacket ──────────────────────────────────────────────────────────────

@dataclass
class StatusPacket:
    """Status packet from robot. Includes robot_type for protocol negotiation."""
    mode:       int
    tasks_ok:   int
    canary_ok:  int
    uptime_s:   int
    robot_type: int = ROBOT_WHEELED  # optional — legacy packets don't have it

    FORMAT_V1 = "<BBBI"   # 7 bytes (legacy, no robot_type): mode, tasks_ok, canary_ok, uptime_s
    FORMAT_V2 = "<BBBIB"  # 8 bytes: mode, tasks_ok, canary_ok, uptime_s, robot_type

    def to_bytes(self) -> bytes:
        return struct.pack(self.FORMAT_V2,
            self.mode, self.tasks_ok, self.canary_ok, self.uptime_s, self.robot_type)

    @classmethod
    def from_bytes(cls, data: bytes) -> "StatusPacket":
        if len(data) == 7:
            # Legacy format (no robot_type field)
            m, t, c, u = struct.unpack(cls.FORMAT_V1, data)
            return cls(mode=m, tasks_ok=t, canary_ok=c, uptime_s=u)
        m, t, c, u, rt = struct.unpack(cls.FORMAT_V2, data)
        return cls(mode=m, tasks_ok=t, canary_ok=c, uptime_s=u, robot_type=rt)
