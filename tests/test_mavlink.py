"""Tests for planner/mavlink_client.py — MAVLink bridge."""

import asyncio
from planner.mavlink_client import (
    MavlinkClient, MavTelemetry,
    MAVLINK_SYSTEM_ID, MAVLINK_TARGET_SYSTEM,
    MAV_CMD_NAV_TAKEOFF, MAV_CMD_NAV_LAND,
    MAV_CMD_COMPONENT_ARM_DISARM, MAV_CMD_NAV_RETURN_TO_LAUNCH,
    HEARTBEAT_INTERVAL_S, TELEMETRY_TIMEOUT_S,
)


class TestConstants:
    def test_system_ids(self):
        assert MAVLINK_SYSTEM_ID > 0
        assert MAVLINK_TARGET_SYSTEM > 0

    def test_command_ids(self):
        assert MAV_CMD_NAV_TAKEOFF == 22
        assert MAV_CMD_NAV_LAND == 21
        assert MAV_CMD_COMPONENT_ARM_DISARM == 400
        assert MAV_CMD_NAV_RETURN_TO_LAUNCH == 20

    def test_timeouts(self):
        assert HEARTBEAT_INTERVAL_S > 0
        assert TELEMETRY_TIMEOUT_S > 0


class TestMavTelemetry:
    def test_defaults(self):
        t = MavTelemetry()
        assert t.lat == 0.0
        assert t.lon == 0.0
        assert not t.armed
        assert not t.connected

    def test_connected_with_heartbeat(self):
        import time
        t = MavTelemetry()
        t.last_heartbeat = time.time()
        assert t.connected

    def test_not_connected_stale(self):
        import time
        t = MavTelemetry()
        t.last_heartbeat = time.time() - TELEMETRY_TIMEOUT_S - 1
        assert not t.connected


class TestMavlinkClient:
    def test_init(self):
        c = MavlinkClient("/dev/ttyUSB0", baud=57600)
        assert not c.connected
        assert c.telemetry.lat == 0.0

    def test_not_connected_initially(self):
        c = MavlinkClient()
        assert not c.connected

    def test_execute_skill_unknown(self):
        """Unknown skill should not crash."""
        async def _run():
            c = MavlinkClient()
            await c.execute_skill("NONEXISTENT")
        asyncio.run(_run())

    def test_telemetry_isolation(self):
        c1 = MavlinkClient()
        c2 = MavlinkClient()
        c1.telemetry.lat = 40.5
        assert c2.telemetry.lat == 0.0
