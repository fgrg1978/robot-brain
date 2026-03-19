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
from perception.slam import SLAM, OccupancyGrid
from planner.path import PathPlanner
from planner.mapper import PerimeterMapper
from planner.patrol import PatrolController
from planner.led import LedController
from planner.power import PowerManager
from planner.sensors import SensorFusion
from planner.alert import AlertPipeline
from planner.deterrent import DeterrentManager, DeterrentLevel
from perception.rtsp_monitor import RtspMonitor, RtspEvent, cameras_from_config
from planner.docking import DockManager, DockState, DockInfo, dock_from_config
from planner.zones import ZoneManager
from planner.tracker import IntrusionTracker
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

        # SLAM + navigation
        slam_cfg = self.config.get("slam", {})
        grid = OccupancyGrid(
            resolution_mm=slam_cfg.get("map_resolution_mm", 50),
            size_cells=slam_cfg.get("map_size_cells", 1000),
        )
        self.slam = SLAM(grid)
        self.path_planner = PathPlanner(grid)
        self.mapper = PerimeterMapper(self.slam, self.path_planner)
        self.patrol: PatrolController | None = None  # created on connection

        # LED controller
        self.led = LedController(protocol.send_packet)

        # Power mode manager (ECO/ALERT)
        self.power = PowerManager(protocol.send_packet)

        # Multi-sensor fusion (PIR/sound/IR)
        self.sensor_fusion = SensorFusion()

        # Alert pipeline (buzzer, evidence, notifications)
        alert_cfg = self.config.get("alert", {})
        self.alert_pipeline = AlertPipeline(
            send_packet=protocol.send_packet,
            notifier=self.notifier,
            evidence_dir=alert_cfg.get("evidence_dir", "data/evidence"),
            cooldown_s=alert_cfg.get("cooldown_s", 30),
            evidence_frames=alert_cfg.get("evidence_frames", 10),
        )

        # Deterrent system (escalating response)
        self.deterrent = DeterrentManager(protocol.send_packet)

        # RTSP camera network (fixed surveillance cameras)
        rtsp_cameras = cameras_from_config(self.config)
        self.rtsp_monitor = RtspMonitor(
            cameras=rtsp_cameras,
            vision=self.vision,
            on_threat=self._on_rtsp_threat,
            detect_labels=self.config.get("modes", {}).get(
                "guardia", {}
            ).get("detect", ["person", "vehicle", "fire"]),
        ) if rtsp_cameras else None

        # Zones of interest manager
        self.zone_manager = ZoneManager()

        # Intruder tracker
        self.tracker = IntrusionTracker()

        # Auto-docking manager
        dock_info = dock_from_config(self.config)
        self.dock_manager = DockManager(
            dock=dock_info,
            on_dock_needed=self._on_dock_needed,
            on_undock_ready=self._on_undock_ready,
            on_critical=self._on_battery_critical,
        )

        # Robot state
        self.state       = RobotState()
        self.safety_profile = SafetyProfile.for_robot_type(self.robot_type, self.config)
        self._writer: asyncio.StreamWriter | None = None
        self._prev_odom: dict = {"dist_mm": 0, "heading_cdeg": 0}

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

        # Set LED to monitoring (green) on connect
        await self.led.set_state("monitoring", writer)

        # Create SkillRunner bound to this connection
        self.runner = SkillRunner(self.policy, self._send_actuator_cmd)

        # Create PatrolController bound to this connection
        self.patrol = PatrolController(
            slam=self.slam,
            path_planner=self.path_planner,
            mapper=self.mapper,
            send_cmd=self._send_actuator_cmd,
            policy=self.policy,
            on_waypoint_reached=self._on_waypoint_reached,
            on_detection=self._on_detection,
        )

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
            self._feed_slam(pkt)
            await self._safety_check(pkt, writer)
            # Check power mode timeout (ALERT → ECO after timeout)
            deescalated = await self.power.check_timeout(writer)
            if deescalated:
                await self.led.set_state("monitoring", writer)
            # Check deterrent safety timeout
            if self.deterrent.active:
                await self.deterrent.check_timeout(writer)
            # Battery monitoring → auto-dock
            new_dock_state = self.dock_manager.update_battery(pkt.battery_mv)
            if new_dock_state == DockState.LOW_BATTERY:
                await self._on_dock_needed()
            elif new_dock_state == DockState.CRITICAL:
                await self._on_battery_critical()
            elif new_dock_state == DockState.CHARGED:
                await self._on_undock_ready()
            # Process digital sensor triggers (PIR/sound/IR)
            if hasattr(pkt, "sensor_flags") and pkt.sensor_flags:
                await self._process_sensor_triggers(pkt.sensor_flags, writer)

        elif pkt_type == CAMERA_FRAME:
            image_data = self._decode_camera(payload)
            if image_data:
                self.state.update_image(image_data)
                await self._perception_cycle(image_data, writer)

        elif pkt_type == STATUS:
            pkt = StatusPacket.from_bytes(payload)
            self.state.update_status(pkt)
            await self._on_status(pkt)

    # ── Patrol callbacks ─────────────────────────────────────────────────────

    async def _on_waypoint_reached(self, wp, index: int):
        """Called when patrol reaches a waypoint — trigger VLM scan."""
        logger.info("[Patrol] Reached waypoint %d: (%d, %d) %s",
                    index, wp.x_mm, wp.y_mm, wp.label or "")

        # Skip scan if zone manager says not this pass (normal waypoints)
        if not self.zone_manager.should_scan(wp):
            return

        # LED: yellow while scanning
        await self.led.set_state("detecting", self._writer)

        # If we have a recent image, run VLM to check for detections
        if self.state.last_image:
            try:
                scene = self.vision.describe(
                    self.state.last_image,
                    context=f"patrol waypoint {index} ({wp.label})",
                )
                print(f"[VLM@wp{index}] {scene}")

                # Zone tagging (during mapping) and state change detection
                if not wp.zone_type:
                    self.zone_manager.tag_waypoint(wp, scene)
                else:
                    changed, change_desc = self.zone_manager.check_state(
                        wp, scene,
                    )
                    if changed:
                        print(f"[ZONE CHANGE] {change_desc}")
                        actions = self.mode_manager.on_detect_actions() \
                            if self.mode_manager.current else []
                        await self.alert_pipeline.raise_alert(
                            trigger_label=f"zone_change_wp{index}",
                            detection_label=f"zone_{wp.zone_type}",
                            vlm_description=change_desc,
                            image_data=self.state.last_image,
                            writer=self._writer,
                            actions=actions,
                        )

                # Check for detections
                mode = self.mode_manager.current
                if mode:
                    detected = False
                    actions = self.mode_manager.on_detect_actions()
                    for label in mode.detect:
                        if label.lower() in scene.lower():
                            print(f"[DETECT] '{label}' at waypoint {index}")
                            detected = True
                            await self.led.set_state("confirmed", self._writer)
                            await self.power.report_threat(self._writer)

                            # Use alert pipeline for full response
                            await self.alert_pipeline.raise_alert(
                                trigger_label=f"patrol_wp{index}",
                                detection_label=label,
                                vlm_description=scene,
                                image_data=self.state.last_image,
                                writer=self._writer,
                                actions=actions,
                            )

                            # Start deterrent if configured
                            if "deterrent" in actions:
                                await self.deterrent.start(self._writer)
                                await self.led.set_state("panic", self._writer)

                    # If VLM says clear, report to power/deterrent for de-escalation
                    if not detected and "clear" in scene.lower():
                        await self.power.report_clear(self._writer)
                        if self.deterrent.active:
                            stood_down = await self.deterrent.report_clear(
                                self._writer
                            )
                            if stood_down:
                                await self.led.set_state("monitoring",
                                                        self._writer)

            except Exception as e:
                logger.error("[Patrol] VLM error at waypoint %d: %s", index, e)

        # LED: back to green (monitoring)
        await self.led.set_state("monitoring", self._writer)

    async def _on_detection(self, label: str, image_data: bytes):
        """Called on a confirmed detection during patrol."""
        await self.led.set_state("confirmed", self._writer)
        asyncio.create_task(
            self.notifier.alert(f"Detection: {label}", title="Patrol Alert",
                                image_data=image_data)
        )

    # ── Auto-docking callbacks ──────────────────────────────────────────

    async def _on_dock_needed(self):
        """Battery low — interrupt patrol and navigate to dock."""
        logger.warning("[Dock] Battery low — returning to dock")
        if self._writer:
            await self.led.set_state("low_battery", self._writer)
        if self.patrol:
            self.patrol.stop()
        await self.task_queue.put("RETURN_TO_DOCK")
        asyncio.create_task(
            self.notifier.alert(
                f"Battery low ({self.dock_manager.battery_mv}mV) — returning to dock",
                title="Low Battery",
            )
        )

    async def _on_undock_ready(self):
        """Battery full — undock and resume patrol."""
        logger.info("[Dock] Battery full — undocking")
        await self.task_queue.put("UNDOCK")

    async def _on_battery_critical(self):
        """Battery critical — emergency dock."""
        logger.critical("[Dock] Battery CRITICAL — emergency dock")
        if self._writer:
            await self.led.set_state("low_battery", self._writer)
        if self.patrol:
            self.patrol.stop()
        if self.runner:
            self.runner.interrupt("battery critical")
        await self.task_queue.put("RETURN_TO_DOCK")
        asyncio.create_task(
            self.notifier.alert(
                f"CRITICAL battery ({self.dock_manager.battery_mv}mV) — emergency dock!",
                title="BATTERY CRITICAL",
            )
        )

    # ── RTSP camera threat callback ─────────────────────────────────────

    async def _on_rtsp_threat(self, event: RtspEvent):
        """Called when an RTSP camera confirms a threat — dispatch robot."""
        logger.info(
            "[RTSP] Threat on '%s': %s (%s)",
            event.camera_name, event.detection_label, event.vlm_description,
        )
        # Escalate to ALERT mode
        if self._writer:
            await self.power.trigger_alert(
                f"rtsp_{event.camera_name}", self._writer,
            )
            await self.led.set_state("detecting", self._writer)

        # Alert pipeline (buzzer + evidence + notifications)
        mode = self.mode_manager.current
        actions = self.mode_manager.on_detect_actions() if mode else []
        if self._writer:
            await self.alert_pipeline.raise_alert(
                trigger_label=f"rtsp_{event.camera_name}",
                detection_label=event.detection_label,
                vlm_description=event.vlm_description,
                image_data=event.image_data,
                writer=self._writer,
                actions=actions,
            )

        # Dispatch robot to investigate zone
        if event.zone_waypoint:
            await self.task_queue.put(
                f"INVESTIGATE_ZONE {event.zone_waypoint}"
            )

    # ── Sensor triggers (PIR/sound/IR) ────────────────────────────────────

    async def _process_sensor_triggers(self, sensor_flags: int,
                                       writer: asyncio.StreamWriter):
        """Process digital sensor triggers — escalate to ALERT + VLM confirm."""
        triggers = self.sensor_fusion.process_flags(sensor_flags)
        if not triggers:
            return

        # Escalate to ALERT mode
        first = triggers[0]
        await self.power.trigger_alert(first.label, writer)
        await self.led.set_state("detecting", writer)

        # Run VLM to confirm detection (if we have an image)
        if self.state.last_image:
            try:
                trigger_names = ", ".join(t.label for t in triggers)
                scene = self.vision.describe(
                    self.state.last_image,
                    context=f"sensor trigger: {trigger_names}",
                )
                print(f"[VLM@sensor] {scene}")

                # Check for actual threats
                mode = self.mode_manager.current
                actions = self.mode_manager.on_detect_actions() if mode else []
                detected = False

                if mode:
                    for label in mode.detect:
                        if label.lower() in scene.lower():
                            detected = True
                            await self.led.set_state("confirmed", writer)

                            # Raise alert through pipeline
                            event = await self.alert_pipeline.raise_alert(
                                trigger_label=first.label,
                                detection_label=label,
                                vlm_description=scene,
                                image_data=self.state.last_image,
                                writer=writer,
                                actions=actions,
                            )
                            if event:
                                self.sensor_fusion.mark_all_alerted(triggers)
                                # Start deterrent if configured
                                if "deterrent" in actions:
                                    await self.deterrent.start(writer)
                                    await self.led.set_state("panic", writer)
                            break

                # VLM says clear — report to power/deterrent
                if not detected:
                    if "clear" in scene.lower():
                        await self.power.report_clear(writer)
                        if self.deterrent.active:
                            stood_down = await self.deterrent.report_clear(
                                writer
                            )
                            if stood_down:
                                await self.led.set_state("monitoring", writer)
                    await self.led.set_state("monitoring", writer)

            except Exception as e:
                logger.error("[Sensor] VLM confirmation error: %s", e)
                await self.led.set_state("monitoring", writer)
        else:
            # No image available — still trigger alert, mark for VLM on next frame
            self.sensor_fusion.mark_all_alerted(triggers)

    # ── SLAM feed ──────────────────────────────────────────────────────────

    def _feed_slam(self, pkt):
        """Extract odometry deltas from sensor packet and feed SLAM."""
        if not hasattr(pkt, "odom_dist_mm"):
            return

        # compute deltas from previous odometry
        curr_dist = pkt.odom_dist_mm
        curr_hdg = pkt.odom_hdg_cdeg
        prev_dist = self._prev_odom.get("dist_mm", 0)
        prev_hdg = self._prev_odom.get("heading_cdeg", 0)

        d_dist = curr_dist - prev_dist
        d_hdg = curr_hdg - prev_hdg
        # wrap heading delta
        if d_hdg > 18000:
            d_hdg -= 36000
        elif d_hdg < -18000:
            d_hdg += 36000

        self._prev_odom = {"dist_mm": curr_dist, "heading_cdeg": curr_hdg}

        # forward distance as dx (robot frame), no lateral movement for diff drive
        if self.patrol and d_dist != 0:
            # empty scan for now — real LiDAR packets would feed this
            self.patrol.feed_sensors(d_dist, 0, d_hdg, [])

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

        # LED feedback for violations
        battery_violations = [v for v in violations if "battery" in v.lower()]
        if battery_violations:
            await self.led.set_state("low_battery", writer)
        else:
            await self.led.set_state("confirmed", writer)

        # Notify operator for battery violations
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
        """Run VLM → LLM → policy → send command. Skipped in ECO mode."""
        # In ECO mode, camera is OFF — skip VLM processing
        if self.power.is_eco:
            return

        # If actively recording evidence, save frame
        if self.alert_pipeline.active_evidence:
            await self.alert_pipeline.save_evidence_frame(image_data)

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
                # Handle patrol/mapping commands directly
                if task_desc.strip().upper() == "MAP":
                    if self.patrol:
                        await self.led.set_state("mapping", self._writer)
                        await self.patrol.run_mapping()
                        await self.led.set_state("monitoring", self._writer)
                elif task_desc.strip().upper() == "PATROL":
                    if self.patrol:
                        await self.led.set_state("monitoring", self._writer)
                        await self.patrol.run_patrol(loop=True)
                elif task_desc.strip().upper() == "PATROL_ONCE":
                    if self.patrol:
                        await self.led.set_state("monitoring", self._writer)
                        await self.patrol.run_patrol(loop=False)
                        await self.led.set_state("monitoring", self._writer)
                elif task_desc.strip().upper() == "STOP_PATROL":
                    if self.patrol:
                        self.patrol.stop()
                        await self.led.set_state("monitoring", self._writer)
                elif task_desc.strip().upper() == "DETERRENT":
                    await self.deterrent.start(self._writer)
                    await self.led.set_state("panic", self._writer)
                elif task_desc.strip().upper() == "STAND_DOWN":
                    await self.deterrent.stand_down(self._writer)
                    await self.led.set_state("monitoring", self._writer)
                elif task_desc.strip().upper() == "SILENCE":
                    await self.deterrent.silence(self._writer)
                elif task_desc.strip().upper() == "PANIC":
                    await self.deterrent.start(self._writer)
                    # Fast-escalate to max level
                    for _ in range(DeterrentLevel.AGGRESSIVE):
                        await self.deterrent.escalate(self._writer)
                    await self.led.set_state("panic", self._writer)
                elif task_desc.strip().upper() == "RETURN_TO_DOCK":
                    if self.patrol:
                        await self.led.set_state("docking", self._writer)
                        dock = self.dock_manager.dock_info
                        from planner.mapper import Waypoint
                        wp = Waypoint(
                            x_mm=dock.x_mm, y_mm=dock.y_mm,
                            heading_cdeg=dock.heading_cdeg, label="dock",
                        )
                        self.dock_manager.state = DockState.NAVIGATING
                        reached = await self.patrol._navigate_to(wp)
                        if reached:
                            self.dock_manager.report_arrived_at_dock()
                            # IR homing would happen here in real hardware
                            self.dock_manager.report_docked()
                            self.dock_manager.report_charging()
                            await self.led.set_state("docking", self._writer)
                        else:
                            self.dock_manager.abort()
                            await self.led.set_state("monitoring",
                                                     self._writer)
                elif task_desc.strip().upper() == "UNDOCK":
                    self.dock_manager.start_undock_sequence()
                    if self.runner:
                        await self.runner.execute_one("UNDOCK")
                    self.dock_manager.report_undocked()
                    await self.led.set_state("monitoring", self._writer)
                    # Resume patrol
                    await self.task_queue.put("PATROL")
                elif task_desc.strip().upper() == "DOCK":
                    # Manual dock command (Telegram /dock)
                    self.dock_manager.start_dock_sequence()
                    await self.task_queue.put("RETURN_TO_DOCK")
                elif task_desc.strip().upper().startswith("TRACK_INTRUDER"):
                    parts = task_desc.strip().split(maxsplit=1)
                    target = parts[1] if len(parts) > 1 else "person"
                    self.tracker.start(target)
                    await self.led.set_state("tracking", self._writer)
                elif task_desc.strip().upper().startswith("INVESTIGATE_ZONE"):
                    parts = task_desc.strip().split(maxsplit=1)
                    zone = parts[1] if len(parts) > 1 else ""
                    if self.patrol and zone:
                        await self.led.set_state("detecting", self._writer)
                        # Navigate to zone waypoint, scan, report
                        wp = self._mapper.get_waypoint_by_label(zone)
                        if wp:
                            from planner.mapper import Waypoint
                            await self.patrol._navigate_to(wp)
                            await self.patrol._rotate_360()
                            if self._on_waypoint_reached:
                                await self._on_waypoint_reached(wp, 0)
                        else:
                            logger.warning(
                                "[Task] Zone '%s' not found in map", zone,
                            )
                        await self.led.set_state("monitoring", self._writer)
                else:
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

        if self.rtsp_monitor and self.rtsp_monitor.camera_count > 0:
            tasks.append(self.rtsp_monitor.start())
            print(f"[BRAIN] RTSP monitoring: {self.rtsp_monitor.camera_count} cameras")

        async with server:
            await asyncio.gather(*tasks)


def main():
    brain = BrainServer()
    asyncio.run(brain.run())


if __name__ == "__main__":
    main()
