"""Tests for the binary protocol."""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from protocol import (
    build_packet, parse_packet, crc8,
    SensorPacket, SensorPacketDrone, SensorPacketHumanoid,
    sensor_packet_from_bytes,
    ActuatorCmd, VelocityCmd, StatusPacket,
    SENSOR_PACKET, ACTUATOR_CMD, VELOCITY_CMD, STATUS,
    ROBOT_WHEELED, ROBOT_DRONE, ROBOT_HUMANOID,
    ACT_DIFF_DRIVE, ACT_QUAD_ROTOR, ACT_HUMANOID,
    FLAG_EMERGENCY, FLAG_ALERT,
)


# ── Core protocol ─────────────────────────────────────────────────────────────

def test_crc8():
    assert crc8(b"") == 0
    assert crc8(b"\x00") == 0
    c = crc8(b"RobotOS")
    assert 0 <= c <= 255


def test_roundtrip_packet():
    payload = b"\x01\x02\x03\x04"
    pkt = build_packet(0x42, payload)
    result = parse_packet(pkt)
    assert result is not None
    pkt_type, data = result
    assert pkt_type == 0x42
    assert data == payload


def test_bad_magic():
    pkt = b"\x00\x00\x42\x04\x00\x01\x02\x03\x04\x00"
    assert parse_packet(pkt) is None


def test_bad_crc():
    payload = b"\x01\x02\x03"
    pkt = build_packet(0x01, payload)
    corrupted = pkt[:-1] + bytes([(pkt[-1] + 1) & 0xFF])
    assert parse_packet(corrupted) is None


# ── SensorPacket (wheeled) ────────────────────────────────────────────────────

def test_sensor_packet_roundtrip():
    sp = SensorPacket(
        timestamp_ms=123456,
        battery_mv=7400,
        accel_mg=(100, -200, 1000),
        gyro_mdps=(50, -30, 10),
        odom_dist_mm=3200,
        odom_hdg_cdeg=4500,
        encoder_l=10000,
        encoder_r=10050,
        range_front_mm=500,
        range_right_mm=300,
    )
    data = sp.to_bytes()
    sp2 = SensorPacket.from_bytes(data)
    assert sp2.timestamp_ms == 123456
    assert sp2.accel_mg == (100, -200, 1000)
    assert sp2.encoder_l == 10000
    assert sp2.battery_mv == 7400
    assert sp2.range_front_mm == 500


def test_sensor_packet_from_bytes_dispatcher():
    sp = SensorPacket(
        timestamp_ms=1, battery_mv=8000, accel_mg=(0, 0, 1000),
        gyro_mdps=(0, 0, 0), odom_dist_mm=0, odom_hdg_cdeg=0,
        encoder_l=0, encoder_r=0, range_front_mm=1000, range_right_mm=1000,
    )
    data = sp.to_bytes()
    sp2 = sensor_packet_from_bytes(ROBOT_WHEELED, data)
    assert isinstance(sp2, SensorPacket)
    assert sp2.battery_mv == 8000


# ── SensorPacketDrone ─────────────────────────────────────────────────────────

def test_sensor_packet_drone_roundtrip():
    dp = SensorPacketDrone(
        timestamp_ms=999, battery_mv=11100,
        accel_mg=(10, -5, 980), gyro_mdps=(1, -2, 3),
        baro_pa=101325, mag_ut=(200, -100, 500),
        gps_lat_deg7=414123456, gps_lon_deg7=-2123456, gps_alt_cm=5000,
        sonar_down_mm=1500,
    )
    data = dp.to_bytes()
    dp2 = SensorPacketDrone.from_bytes(data)
    assert dp2.timestamp_ms == 999
    assert dp2.battery_mv == 11100
    assert dp2.gps_lat_deg7 == 414123456
    assert dp2.sonar_down_mm == 1500


# ── SensorPacketHumanoid ──────────────────────────────────────────────────────

def test_sensor_packet_humanoid_roundtrip():
    hp = SensorPacketHumanoid(
        timestamp_ms=42, battery_mv=7200,
        accel_mg=(0, 0, 1000), gyro_mdps=(10, 0, 0),
        joint_angles=[0, 0, -3000, 6000, -3000, 0, 0, 0, -3000, 6000, -3000, 0],
        foot_pressure_l=5000, foot_pressure_r=5200,
    )
    data = hp.to_bytes()
    hp2 = SensorPacketHumanoid.from_bytes(data)
    assert hp2.joint_angles == hp.joint_angles
    assert hp2.foot_pressure_l == 5000


# ── ActuatorCmd ───────────────────────────────────────────────────────────────

def test_actuator_cmd_wheeled_roundtrip():
    cmd = ActuatorCmd.wheeled(60, -40, flags=FLAG_ALERT)
    data = cmd.to_bytes()
    cmd2 = ActuatorCmd.from_bytes(data)
    assert cmd2.actuator_type == ACT_DIFF_DRIVE
    assert cmd2.channels == [60, -40]
    assert cmd2.flags == FLAG_ALERT


def test_actuator_cmd_drone_roundtrip():
    cmd = ActuatorCmd.drone(1450, 1500, 1500, 1520)
    data = cmd.to_bytes()
    cmd2 = ActuatorCmd.from_bytes(data)
    assert cmd2.actuator_type == ACT_QUAD_ROTOR
    assert cmd2.channels == [1450, 1500, 1500, 1520]


def test_actuator_cmd_humanoid_roundtrip():
    joints = [0, 0, -3000, 6000, -3000, 0, 0, 0, -3000, 6000, -3000, 0]
    cmd = ActuatorCmd(actuator_type=ACT_HUMANOID, channels=joints)
    data = cmd.to_bytes()
    cmd2 = ActuatorCmd.from_bytes(data)
    assert cmd2.channels == joints


def test_actuator_cmd_stop():
    cmd = ActuatorCmd.stop(n_channels=2)
    assert cmd.channels == [0, 0]
    assert cmd.flags == FLAG_EMERGENCY


def test_actuator_cmd_size_wheeled():
    cmd = ActuatorCmd.wheeled(60, 60)
    assert len(cmd.to_bytes()) == 3 + 2 * 2  # 7 bytes


def test_actuator_cmd_size_drone():
    cmd = ActuatorCmd.drone(1450, 1500, 1500, 1500)
    assert len(cmd.to_bytes()) == 3 + 4 * 2  # 11 bytes


# ── VelocityCmd (backward compat) ────────────────────────────────────────────

def test_velocity_cmd_roundtrip():
    cmd = VelocityCmd(speed_l=60, speed_r=-40, flags=0x02)
    data = cmd.to_bytes()
    cmd2 = VelocityCmd.from_bytes(data)
    assert cmd2.speed_l == 60
    assert cmd2.speed_r == -40
    assert cmd2.flags == 0x02


def test_velocity_cmd_to_actuator():
    vc = VelocityCmd(speed_l=50, speed_r=50)
    ac = vc.to_actuator_cmd()
    assert ac.actuator_type == ACT_DIFF_DRIVE
    assert ac.channels == [50, 50]


# ── StatusPacket ──────────────────────────────────────────────────────────────

def test_status_packet_roundtrip():
    st = StatusPacket(mode=1, tasks_ok=8, canary_ok=8, uptime_s=3600)
    data = st.to_bytes()
    st2 = StatusPacket.from_bytes(data)
    assert st2.mode == 1
    assert st2.uptime_s == 3600
    assert st2.robot_type == ROBOT_WHEELED  # default


def test_status_packet_with_robot_type():
    st = StatusPacket(mode=2, tasks_ok=4, canary_ok=4, uptime_s=120, robot_type=ROBOT_DRONE)
    data = st.to_bytes()
    st2 = StatusPacket.from_bytes(data)
    assert st2.robot_type == ROBOT_DRONE


def test_status_packet_legacy_7bytes():
    # Legacy 7-byte format (no robot_type) — should parse without error
    import struct
    data = struct.pack("<BBBI", 1, 8, 8, 3600)
    st = StatusPacket.from_bytes(data)
    assert st.mode == 1
    assert st.robot_type == ROBOT_WHEELED


if __name__ == "__main__":
    test_crc8()
    test_roundtrip_packet()
    test_bad_magic()
    test_bad_crc()
    test_sensor_packet_roundtrip()
    test_sensor_packet_from_bytes_dispatcher()
    test_sensor_packet_drone_roundtrip()
    test_sensor_packet_humanoid_roundtrip()
    test_actuator_cmd_wheeled_roundtrip()
    test_actuator_cmd_drone_roundtrip()
    test_actuator_cmd_humanoid_roundtrip()
    test_actuator_cmd_stop()
    test_actuator_cmd_size_wheeled()
    test_actuator_cmd_size_drone()
    test_velocity_cmd_roundtrip()
    test_velocity_cmd_to_actuator()
    test_status_packet_roundtrip()
    test_status_packet_with_robot_type()
    test_status_packet_legacy_7bytes()
    print("All protocol tests passed!")
