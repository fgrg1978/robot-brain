"""Tests for SITL physics and protocol integration (no network needed)."""

import sys
import os
import math
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from protocol import ActuatorCmd, ACT_DIFF_DRIVE, FLAG_EMERGENCY
from tools.sitl.sitl_wheeled import RobotSim, World, Rect


# ── World / raycast ────────────────────────────────────────────────────────────

def test_world_raycast_wall():
    world = World(obstacles=[], width_mm=5000, height_mm=5000)
    # Robot at center facing East — should hit east wall at ~2500 mm
    robot = RobotSim(world, start_x=2500, start_y=2500, start_hdg_deg=0)
    d = world.raycast(2500, 2500, 0)
    assert 2400 < d < 2700   # wall at x=5000 + 100 thick, from x=2500 → ~2500


def test_world_collision():
    obs = [Rect(1000, 1000, 500, 500)]
    world = World(obstacles=obs)
    assert world.collides(1200, 1200)
    assert not world.collides(200, 200)


def test_world_obstacle_raycast():
    obs = [Rect(1000, 500, 100, 1000)]  # vertical wall
    world = World(obstacles=obs, width_mm=5000, height_mm=5000)
    # From x=200, facing East (0°) — obstacle at x=1000
    d = world.raycast(200, 1000, 0)
    assert 750 < d < 850   # ~800 mm to x=1000


# ── RobotSim physics ───────────────────────────────────────────────────────────

def _make_robot(x=1000, y=1000, hdg=0) -> RobotSim:
    world = World(obstacles=[], width_mm=10000, height_mm=10000)
    return RobotSim(world, start_x=x, start_y=y, start_hdg_deg=hdg)


def test_robot_stop_by_default():
    robot = _make_robot()
    assert robot.speed_l == 0
    assert robot.speed_r == 0


def test_robot_apply_cmd_forward():
    robot = _make_robot()
    cmd = ActuatorCmd.wheeled(60, 60)
    robot.apply_cmd(cmd)
    assert robot.speed_l == 60
    assert robot.speed_r == 60


def test_robot_apply_cmd_emergency_stops():
    robot = _make_robot()
    cmd = ActuatorCmd.wheeled(60, 60)
    robot.apply_cmd(cmd)
    stop = ActuatorCmd.stop(n_channels=2)
    robot.apply_cmd(stop)
    assert robot.speed_l == 0
    assert robot.speed_r == 0


def test_robot_apply_cmd_clamps():
    robot = _make_robot()
    cmd = ActuatorCmd.wheeled(200, -200)
    robot.apply_cmd(cmd)
    assert robot.speed_l == 100
    assert robot.speed_r == -100


def test_robot_moves_forward():
    import time
    robot = _make_robot(x=5000, y=5000, hdg=0)  # facing East
    cmd = ActuatorCmd.wheeled(100, 100)
    robot.apply_cmd(cmd)

    # Step 1 second of physics at 100 Hz
    for _ in range(100):
        robot._last_tick -= 0.01   # fake 10ms elapsed
        robot.step()

    assert robot.x > 5200   # should have moved East


def test_robot_turns_right():
    import time
    robot = _make_robot(x=5000, y=5000, hdg=0)
    # Right turn: left faster than right → CW
    cmd = ActuatorCmd.wheeled(60, -60)
    robot.apply_cmd(cmd)

    for _ in range(50):
        robot._last_tick -= 0.01
        robot.step()

    assert robot.hdg_deg > 10   # heading increased (CW)


def test_robot_sensor_packet():
    robot = _make_robot()
    pkt = robot.sensor_packet()
    assert pkt.battery_mv > 0
    assert pkt.range_front_mm > 0
    from protocol import SensorPacket
    assert isinstance(pkt, SensorPacket)


def test_robot_sensor_packet_roundtrip():
    robot = _make_robot()
    pkt = robot.sensor_packet()
    data = pkt.to_bytes()
    from protocol import SensorPacket
    pkt2 = SensorPacket.from_bytes(data)
    assert pkt2.battery_mv == pkt.battery_mv
    assert pkt2.range_front_mm == pkt.range_front_mm


def test_robot_status_packet():
    robot = _make_robot()
    st = robot.status_packet()
    from protocol import ROBOT_WHEELED
    assert st.robot_type == ROBOT_WHEELED


def test_robot_collision_stops_motion():
    # Place robot right next to a wall
    obs = [Rect(1200, 0, 100, 5000)]   # vertical wall at x=1200
    world = World(obstacles=obs, width_mm=10000, height_mm=10000)
    robot = RobotSim(world, start_x=1000, start_y=2500, start_hdg_deg=0)
    cmd = ActuatorCmd.wheeled(100, 100)
    robot.apply_cmd(cmd)

    for _ in range(200):
        robot._last_tick -= 0.01
        robot.step()

    # Robot should not have passed through the wall
    assert robot.x < 1200


def test_robot_battery_drains():
    robot = _make_robot()
    initial_batt = robot.battery_mv
    cmd = ActuatorCmd.wheeled(100, 100)
    robot.apply_cmd(cmd)

    for _ in range(200):
        robot._last_tick -= 0.01
        robot.step()

    assert robot.battery_mv < initial_batt


# ── Scenario loading ───────────────────────────────────────────────────────────

def test_world_from_scenario_empty():
    scenario = {"obstacles": [], "width_mm": 8000, "height_mm": 6000}
    world = World.from_scenario(scenario)
    assert world.width == 8000
    assert world.height == 6000
    assert len(world.obstacles) == 0


def test_world_from_scenario_with_obstacles():
    scenario = {
        "obstacles": [{"x": 1000, "y": 1000, "w": 500, "h": 500}],
        "width_mm": 5000,
        "height_mm": 5000,
    }
    world = World.from_scenario(scenario)
    assert len(world.obstacles) == 1
    assert world.collides(1200, 1200)


if __name__ == "__main__":
    test_world_raycast_wall()
    test_world_collision()
    test_world_obstacle_raycast()
    test_robot_stop_by_default()
    test_robot_apply_cmd_forward()
    test_robot_apply_cmd_emergency_stops()
    test_robot_apply_cmd_clamps()
    test_robot_moves_forward()
    test_robot_turns_right()
    test_robot_sensor_packet()
    test_robot_sensor_packet_roundtrip()
    test_robot_status_packet()
    test_robot_collision_stops_motion()
    test_robot_battery_drains()
    test_world_from_scenario_empty()
    test_world_from_scenario_with_obstacles()
    print("All SITL tests passed!")
