"""Robot Brain Server — receives sensor data from robot, sends motor commands.

Architecture:
  Robot --TCP--> server.py --API--> LM Studio (VLM + LLM)
                              |
                              +--> ModeManager ──> TaskPlanner ──> SkillRunner
                              |                                        |
                              +--> policy ─────────────────────> ActuatorCmd
                              |
                              +--> Notifier (Pushover/Telegram/Email/Webhook)
                              +--> TelegramBot (bidirectional commands)
                              +--> APIServer   (HTTP REST)
"""

import asyncio
import logging
import time
import yaml

import protocol
from protocol import (
    SensorPacket, sensor_packet_from_bytes,
    ActuatorCmd, VelocityCmd, StatusPacket,
    SENSOR_PACKET, CAMERA_FRAME, STATUS, ACTUATOR_CMD,
    ROBOT_WHEELED, ROBOT_DRONE, ROBOT_HUMANOID,
)
from perception.vision import VisionPerception
from planner.decide import Planner
from planner.modes import ModeManager
from planner.task_planner import TaskPlanner
from planner.skills import get_skills
from policy import get_translator
from policy.safety import SafetyProfile
from executor.skill_runner import SkillRunner
from notifications import Notifier
from telegram_bot import TelegramBot
from api import APIServer

logger = logging.getLogger("brain.server")


class RobotState:
    """Tracks the latest sensor data from the robot."""

    def __init__(self):
        self.sensors: dict = {}
        self.odom: dict = {"dist_mm": 0, "heading_cdeg": 0}
        self.last_image: bytes = b""
        self.last_sensor_time: float = 0
        self.last_image_time: float = 0
        self.status: dict = {}
        self.connected: bool = False

    def update_sensors(self, pkt):
        self.sensors = {
            "battery_mv":      pkt.battery_mv,
        }
        if hasattr(pkt, "range_front_mm"):
            self.sensors["range_front_mm"] = pkt.range_front_mm
            self.sensors["range_right_mm"] = pkt.range_right_mm
        if hasattr(pkt, "accel_mg"):
            self.sensors["accel_mg"]  = pkt.accel_mg
            self.sensors["gyro_mdps"] = pkt.gyro_mdps
        if hasattr(pkt, "odom_dist_mm"):
            self.odom = {
                "dist_mm":     pkt.odom_dist_mm,
                "heading_cdeg": pkt.odom_hdg_cdeg,
            }
        self.last_sensor_time = time.time()

    def update_image(self, image_data: bytes):
        self.last_image = image_data
        self.last_image_time = time.time()

    def update_status(self, pkt: StatusPacket):
        self.status = {
            "mode":      pkt.mode,
            "tasks_ok":  pkt.tasks_ok,
            "canary_ok": pkt.canary_ok,
            "uptime_s":  pkt.uptime_s,
        }


class BrainServer:
    """Main server — orchestrates perception, planning, execution, and notifications."""

    def __init__(self, config_path: str = "config.yaml"):
        with open(config_path) as f:
            self.config = yaml.safe_load(f)

        lm = self.config["lmstudio"]
        robot_type_str = self.config["robot"].get("type", "wheeled")

        # Perception + reactive planner (VLM/LLM)
        self.vision  = VisionPerception(lm["host"], lm["port"], lm["vlm_model"])
        self.planner = Planner(lm["host"], lm["port"], lm["llm_model"])

        # Task planner (LLM → skill plan)
        self.task_planner = TaskPlanner(lm["host"], lm["port"], lm["llm_model"],
                                        robot_type=robot_type_str)

        # Policy translator
        self.robot_type = ROBOT_WHEELED
        self.policy = get_translator(robot_type_str, self.config)

        # Mode manager
        self.mode_manager = ModeManager(self.config)

        # Executor
        self.runner: SkillRunner | None = None

        # Notifications
        self.notifier = Notifier(self.config.get("notifications", {}))

        # Robot state
        self.state       = RobotState()
        self.safety_profile = SafetyProfile.for_robot_type(self.robot_type, self.config)
        self._writer: asyncio.StreamWriter | None = None

        # Task queue (for /task API and Telegram /task command)
        self.task_queue: asyncio.Queue = asyncio.Queue()

        # Telegram bot (enabled only if configured)
        tg_cfg = self.config.get("notifications", {}).get("telegram", {})
        self.tg_bot = TelegramBot(tg_cfg, self) if tg_cfg.get("enabled") else None

        # HTTP API
        api_cfg = self.config.get("api", {})
        self.api = APIServer(self, port=api_cfg.get("port", 8080)) \
                   if api_cfg.get("enabled", True) else None

    # ── ActuatorCmd sender (used by SkillRunner) ──────────────────────────────

    async def _send_actuator_cmd(self, cmd: ActuatorCmd):
        if self._writer:
            await protocol.send_packet(self._writer, ACTUATOR_CMD, cmd.to_bytes())

    # ── Robot connection handler ──────────────────────────────────────────────

    async def handle_robot(self, reader: asyncio.StreamReader,
                           writer: asyncio.StreamWriter):
        addr = writer.get_extra_info("peername")
        print(f"[BRAIN] Robot connected from {addr}")
        self.state.connected = True
        self._writer = writer

        # Create SkillRunner bound to this connection
        self.runner = SkillRunner(self.policy, self._send_actuator_cmd)

        try:
            async for pkt_type, payload in self._packet_stream(reader):
                await self._dispatch(pkt_type, payload, writer)

        except asyncio.IncompleteReadError:
            print("[BRAIN] Robot disconnected")
        except Exception as e:
            print(f"[BRAIN] Error: {e}")
        finally:
            self.state.connected = False
            self._writer = None
            if self.runner:
                self.runner.interrupt("connection closed")
            writer.close()
            await writer.wait_closed()

    async def _packet_stream(self, reader: asyncio.StreamReader):
        """Async generator that yields (pkt_type, payload) packets."""
        while True:
            result = await protocol.read_packet(reader)
            if result is None:
                print("[BRAIN] Invalid packet")
                break
            yield result

    async def _dispatch(self, pkt_type: int, payload: bytes,
                        writer: asyncio.StreamWriter):
        """Route a received packet to the appropriate handler."""

        if pkt_type == SENSOR_PACKET:
            pkt = sensor_packet_from_bytes(self.robot_type, payload)
            self.state.update_sensors(pkt)
            await self._safety_check(pkt, writer)

        elif pkt_type == CAMERA_FRAME:
            image_data = self._decode_camera(payload)
            if image_data:
                self.state.update_image(image_data)
                await self._perception_cycle(image_data, writer)

        elif pkt_type == STATUS:
            pkt = StatusPacket.from_bytes(payload)
            self.state.update_status(pkt)
            await self._on_status(pkt)

    # ── Camera frame decoder ─────────────────────────────────────────────────

    # Camera payload format constants
    _CAM_HDR_SIZE = 5
    _CAM_FMT_GRAY8 = 0
    _CAM_FMT_JPEG = 1

    def _decode_camera(self, payload: bytes) -> bytes | None:
        """Decode a CAMERA_FRAME payload. Returns JPEG bytes for the VLM.

        Header: width(u16 LE) + height(u16 LE) + format(u8)
        If format is Gray8, converts raw grayscale to JPEG via Pillow.
        If format is JPEG, returns the bytes as-is.
        """
        if len(payload) < self._CAM_HDR_SIZE:
            return None
        import struct
        width, height, fmt = struct.unpack_from("<HHB", payload)
        image_data = payload[self._CAM_HDR_SIZE:]
        if not image_data:
            return None

        if fmt == self._CAM_FMT_JPEG:
            return image_data

        if fmt == self._CAM_FMT_GRAY8:
            expected = width * height
            if len(image_data) < expected:
                print(f"[BRAIN] Camera: Gray8 truncated ({len(image_data)}/{expected})")
                return None
            try:
                from PIL import Image
                import io
                img = Image.frombytes("L", (width, height), image_data[:expected])
                buf = io.BytesIO()
                img.save(buf, format="JPEG", quality=70)
                return buf.getvalue()
            except ImportError:
                print("[BRAIN] Camera: Pillow not installed, cannot convert Gray8")
                return None

        print(f"[BRAIN] Camera: unknown format {fmt}")
        return None

    # ── Safety ────────────────────────────────────────────────────────────────

    # Channel counts per actuator type for emergency stop
    _STOP_CHANNELS = {
        ROBOT_WHEELED: 2,
        ROBOT_DRONE: 4,
        ROBOT_HUMANOID: 12,
    }

    async def _safety_check(self, pkt, writer: asyncio.StreamWriter):
        """Hard safety stops — always evaluated, regardless of planner state.

        Uses SafetyProfile to run robot-type-specific checks.
        On any violation: emergency stop + interrupt runner + notify + log.
        """
        violations = self.safety_profile.check(pkt)

        if not violations:
            return

        # Log every violation
        for v in violations:
            logger.warning("[SAFETY] %s", v)
            print(f"[SAFETY] {v}")

        # Interrupt any running skill
        reason = violations[0]
        if self.runner:
            self.runner.interrupt(reason)

        # Determine emergency response based on robot type
        if self.robot_type == ROBOT_DRONE:
            await self._drone_emergency(pkt, writer, violations)
        else:
            # Wheeled / Humanoid / Ackermann: full stop
            n_channels = self._STOP_CHANNELS.get(self.robot_type, 2)
            cmd = ActuatorCmd.stop(n_channels=n_channels)
            await protocol.send_packet(writer, ACTUATOR_CMD, cmd.to_bytes())

        # Notify operator for battery violations
        battery_violations = [v for v in violations if "battery" in v.lower()]
        for bv in battery_violations:
            asyncio.create_task(
                self.notifier.alert(bv, title="Battery Alert")
            )

    async def _drone_emergency(self, pkt, writer: asyncio.StreamWriter,
                               violations: list[str]):
        """Handle drone-specific emergency actions.

        Priority order:
          1. Critical battery -> immediate LAND
          2. Low battery -> RTL (Return-To-Launch)
          3. Other violation -> HOVER (safest for drone)
        """
        action = self.safety_profile.drone_action(pkt)

        if action == "land":
            # Immediate landing — send LAND command via policy
            logger.critical("[SAFETY] Drone CRITICAL — forcing LAND")
            print("[SAFETY] Drone CRITICAL — forcing LAND")
            cmd = self.policy.translate("LAND")
            await protocol.send_packet(writer, ACTUATOR_CMD, cmd.to_bytes())
            asyncio.create_task(
                self.notifier.alert("DRONE CRITICAL BATTERY — LANDING NOW",
                                    title="DRONE EMERGENCY")
            )

        elif action == "rtl":
            # Return to launch
            logger.warning("[SAFETY] Drone low battery — RTL")
            print("[SAFETY] Drone low battery — RTL")
            cmd = self.policy.translate("RETURN_HOME")
            await protocol.send_packet(writer, ACTUATOR_CMD, cmd.to_bytes())
            asyncio.create_task(
                self.notifier.alert("Drone low battery — returning home",
                                    title="Drone RTL")
            )

        else:
            # Other violation — hover in place (safest for drone)
            logger.warning("[SAFETY] Drone violation — HOVER")
            print("[SAFETY] Drone violation — HOVER")
            cmd = self.policy.translate("HOVER")
            await protocol.send_packet(writer, ACTUATOR_CMD, cmd.to_bytes())

    # ── Perception cycle ──────────────────────────────────────────────────────

    async def _perception_cycle(self, image_data: bytes, writer: asyncio.StreamWriter):
        """Run VLM → LLM → policy → send command."""
        if self.mode_manager.uses_llm():
            # LLM reactive mode: VLM describes scene → LLM picks single action
            try:
                scene = self.vision.describe(image_data,
                                             context=self.mode_manager.current_name)
                print(f"[VLM] {scene}")

                action = self.planner.decide(
                    scene=scene,
                    sensors=self.state.sensors,
                    task=self.mode_manager.current_name,
                    odom=self.state.odom,
                )
                print(f"[LLM] {action}")

                cmd = self.policy.from_text(action)
                await protocol.send_packet(writer, ACTUATOR_CMD, cmd.to_bytes())

            except Exception as e:
                print(f"[BRAIN] AI error: {e} — STOP")
                await protocol.send_packet(writer, ACTUATOR_CMD,
                                           ActuatorCmd.stop(n_channels=2).to_bytes())

    # ── Status / type negotiation ─────────────────────────────────────────────

    async def _on_status(self, pkt: StatusPacket):
        if pkt.robot_type != self.robot_type:
            self.robot_type = pkt.robot_type
            robot_type_str  = {0: "wheeled", 1: "drone", 2: "humanoid"}.get(
                pkt.robot_type, "wheeled"
            )
            self.policy = get_translator(pkt.robot_type, self.config)
            self.safety_profile = SafetyProfile.for_robot_type(pkt.robot_type, self.config)
            self.task_planner.update_robot_type(robot_type_str)
            if self.runner:
                self.runner.policy = self.policy
            print(f"[BRAIN] Robot type changed to {robot_type_str}")
            print(f"[BRAIN] Safety profile: {robot_type_str}")

    # ── Task queue worker ─────────────────────────────────────────────────────

    async def _task_worker(self):
        """Background coroutine that processes queued tasks via TaskPlanner."""
        while True:
            task_desc = await self.task_queue.get()
            print(f"[BRAIN] Task: {task_desc!r}")
            try:
                plan = self.task_planner.plan(task_desc)
                print(f"[BRAIN] Plan: {plan}")
                if self.runner:
                    await self.runner.execute_plan(plan)
            except Exception as e:
                print(f"[BRAIN] Task error: {e}")
            finally:
                self.task_queue.task_done()

    # ── Server entry point ────────────────────────────────────────────────────

    async def run(self):
        port = self.config["robot"]["listen_port"]
        server = await asyncio.start_server(self.handle_robot, "0.0.0.0", port)
        print(f"[BRAIN] Listening on port {port}")
        print(f"[BRAIN] Robot type: {self.config['robot'].get('type', 'wheeled')}")
        print(f"[BRAIN] Mode: {self.mode_manager.current_name}")
        print(f"[BRAIN] VLM: {self.config['lmstudio']['vlm_model']}")
        print(f"[BRAIN] LLM: {self.config['lmstudio']['llm_model']}")

        tasks = [server.serve_forever(), self._task_worker()]

        if self.tg_bot:
            tasks.append(self.tg_bot.run())
            print("[BRAIN] Telegram bot enabled")

        if self.api:
            tasks.append(self.api.run())

        async with server:
            await asyncio.gather(*tasks)


def main():
    brain = BrainServer()
    asyncio.run(brain.run())


if __name__ == "__main__":
    main()
