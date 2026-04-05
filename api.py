"""Minimal HTTP API for the Robot Brain.

Pure asyncio — no extra dependencies. Implements a subset of HTTP/1.1
sufficient for REST calls from scripts, curl, and dashboards.

Endpoints:
  GET  /health          — 200 OK, {"status": "ok"}
  GET  /status          — full robot state JSON
  GET  /mode            — current mode name
  POST /mode            — {"mode": "patrulla"} — switch mode
  POST /stop            — emergency stop
  POST /task            — {"task": "patrol room A then return"} — queue task
  POST /cmd             — {"skill": "FORWARD", "args": {"speed": 60}} — run one skill
  GET  /topics          — list available data topics
  GET  /topics/{name}   — latest data for a topic
  GET  /config/{key}    — read a config value (dot notation)
  POST /config/{key}    — set a config value (dot notation)

Usage:
    api = APIServer(brain, port=8080)
    asyncio.create_task(api.run())
"""

import asyncio
import json
import time
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from server import BrainServer


# ── HTTP helpers ──────────────────────────────────────────────────────────────

def _response(writer: asyncio.StreamWriter, status: int, body: dict | str,
              status_text: str = ""):
    _STATUS_TEXTS = {200: "OK", 201: "Created", 400: "Bad Request",
                     404: "Not Found", 405: "Method Not Allowed",
                     500: "Internal Server Error"}
    if not status_text:
        status_text = _STATUS_TEXTS.get(status, "Unknown")

    if isinstance(body, dict):
        payload = json.dumps(body).encode()
        ctype   = "application/json"
    else:
        payload = body.encode()
        ctype   = "text/plain"

    header = (
        f"HTTP/1.1 {status} {status_text}\r\n"
        f"Content-Type: {ctype}\r\n"
        f"Content-Length: {len(payload)}\r\n"
        f"Connection: close\r\n"
        f"\r\n"
    ).encode()
    writer.write(header + payload)


def _parse_request(raw: bytes) -> tuple[str, str, dict, bytes]:
    """Returns (method, path, headers, body)."""
    header_end = raw.find(b"\r\n\r\n")
    if header_end == -1:
        return "", "", {}, b""
    header_raw = raw[:header_end].decode(errors="replace")
    body = raw[header_end + 4:]

    lines = header_raw.split("\r\n")
    if not lines:
        return "", "", {}, body

    parts = lines[0].split(" ", 2)
    method = parts[0].upper() if len(parts) > 0 else ""
    path   = parts[1] if len(parts) > 1 else "/"

    headers = {}
    for line in lines[1:]:
        if ":" in line:
            k, v = line.split(":", 1)
            headers[k.strip().lower()] = v.strip()

    return method, path, headers, body


# ── Router ────────────────────────────────────────────────────────────────────

class APIServer:
    """Async HTTP API server."""

    def __init__(self, brain: "BrainServer", port: int = 8080):
        self.brain  = brain
        self.port   = port
        self._start = time.time()

    async def run(self):
        server = await asyncio.start_server(self._handle, "0.0.0.0", self.port)
        print(f"[API] Listening on port {self.port}")
        async with server:
            await server.serve_forever()

    async def _handle(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        try:
            raw = await asyncio.wait_for(reader.read(8192), timeout=5)
            method, path, headers, body = _parse_request(raw)
            if not method:
                _response(writer, 400, {"error": "bad request"})
                return
            await self._route(method, path, body, writer)
        except asyncio.TimeoutError:
            _response(writer, 400, {"error": "request timeout"})
        except Exception as e:
            _response(writer, 500, {"error": str(e)})
        finally:
            await writer.drain()
            writer.close()

    async def _route(self, method: str, path: str, body: bytes,
                     writer: asyncio.StreamWriter):
        # Strip query string
        path = path.split("?")[0].rstrip("/") or "/"

        # ── GET routes ─────────────────────────────────────────────────────────
        if method == "GET":
            if path in ("/health", ""):
                _response(writer, 200, {
                    "status": "ok",
                    "uptime_s": round(time.time() - self._start, 1),
                })
            elif path == "/status":
                _response(writer, 200, self._full_status())
            elif path == "/mode":
                _response(writer, 200, {"mode": self._current_mode()})
            elif path == "/topics":
                _response(writer, 200, self._list_topics())
            elif path.startswith("/topics/"):
                topic_name = path[len("/topics/"):]
                _response(writer, 200, self._topic_data(topic_name))
            elif path.startswith("/config/"):
                key = path[len("/config/"):]
                _response(writer, 200, self._config_get(key))
            else:
                _response(writer, 404, {"error": "not found"})

        # ── POST routes ────────────────────────────────────────────────────────
        elif method == "POST":
            try:
                payload = json.loads(body) if body.strip() else {}
            except json.JSONDecodeError:
                _response(writer, 400, {"error": "invalid JSON"})
                return

            if path == "/stop":
                await self._do_stop()
                _response(writer, 200, {"status": "stop sent"})

            elif path == "/mode":
                name = payload.get("mode", "")
                if not name:
                    _response(writer, 400, {"error": "missing 'mode' field"})
                    return
                ok = self._set_mode(name)
                if ok:
                    _response(writer, 200, {"mode": name})
                else:
                    modes = self._available_modes()
                    _response(writer, 400, {
                        "error": f"unknown mode '{name}'",
                        "available": modes,
                    })

            elif path == "/task":
                task = payload.get("task", "")
                if not task:
                    _response(writer, 400, {"error": "missing 'task' field"})
                    return
                await self._queue_task(task)
                _response(writer, 201, {"task": task, "status": "queued"})

            elif path == "/cmd":
                skill = payload.get("skill", "")
                args  = payload.get("args", {})
                if not skill:
                    _response(writer, 400, {"error": "missing 'skill' field"})
                    return
                result = await self._execute_skill(skill, args)
                _response(writer, 200, result)

            elif path.startswith("/config/"):
                key = path[len("/config/"):]
                _response(writer, 200, self._config_set(key, payload))

            else:
                _response(writer, 404, {"error": "not found"})

        else:
            _response(writer, 405, {"error": "method not allowed"})

    # ── Brain interactions ─────────────────────────────────────────────────────

    def _full_status(self) -> dict:
        b = self.brain
        s = b.state
        return {
            "connected":     s.connected,
            "robot_type":    b.robot_type,
            "mode":          self._current_mode(),
            "sensors":       s.sensors,
            "odom":          s.odom,
            "status":        s.status,
            "last_sensor_age_s": round(time.time() - s.last_sensor_time, 2)
                                  if s.last_sensor_time else None,
            "has_image":     len(s.last_image) > 0,
        }

    def _current_mode(self) -> str:
        mm = getattr(self.brain, "mode_manager", None)
        return mm.current_name if mm else "unknown"

    def _available_modes(self) -> list[str]:
        mm = getattr(self.brain, "mode_manager", None)
        return list(mm.modes.keys()) if mm else []

    def _set_mode(self, name: str) -> bool:
        mm = getattr(self.brain, "mode_manager", None)
        if mm:
            return mm.set_mode(name)
        return False

    async def _do_stop(self):
        from protocol import ActuatorCmd
        runner = getattr(self.brain, "runner", None)
        if runner:
            runner.interrupt("API /stop")
        writer = getattr(self.brain, "_writer", None)
        if writer:
            import protocol
            cmd = ActuatorCmd.stop(n_channels=2)
            await protocol.send_packet(writer, 0x80, cmd.to_bytes())

    async def _queue_task(self, task: str):
        q = getattr(self.brain, "task_queue", None)
        if q:
            await q.put(task)

    async def _execute_skill(self, skill: str, args: dict) -> dict:
        runner = getattr(self.brain, "runner", None)
        if runner is None:
            return {"error": "skill runner not available"}
        try:
            cmd = await runner.execute_one(skill, args)
            return {
                "skill":        skill,
                "args":         args,
                "actuator_type": cmd.actuator_type,
                "channels":     cmd.channels,
                "flags":        cmd.flags,
            }
        except Exception as e:
            return {"error": str(e)}

    # ── Topics ────────────────────────────────────────────────────────────────

    DEFAULT_SENSOR_RATE_HZ = 20
    DEFAULT_CAMERA_RATE_HZ = 2
    STATUS_RATE_HZ = 1
    CMD_MOTOR_RATE_HZ = 0       # on-demand, not periodic

    def _list_topics(self) -> list[dict]:
        """Return available data topics with their update rates."""
        robot_cfg = self.brain.config.get("robot", {})
        return [
            {"name": "/sensors/imu",
             "rate_hz": robot_cfg.get("sensor_rate_hz",
                                      self.DEFAULT_SENSOR_RATE_HZ)},
            {"name": "/sensors/camera",
             "rate_hz": robot_cfg.get("camera_rate_hz",
                                      self.DEFAULT_CAMERA_RATE_HZ)},
            {"name": "/cmd/motor",
             "rate_hz": self.CMD_MOTOR_RATE_HZ},
            {"name": "/status",
             "rate_hz": self.STATUS_RATE_HZ},
        ]

    ZERO_ACCEL = [0, 0, 0]
    ZERO_GYRO = [0, 0, 0]

    def _topic_data(self, topic_name: str) -> dict:
        """Return latest data for a given topic."""
        state = self.brain.state
        if topic_name == "sensors/imu" and state.sensors:
            return {
                "accel_mg":  state.sensors.get("accel_mg", self.ZERO_ACCEL),
                "gyro_mdps": state.sensors.get("gyro_mdps", self.ZERO_GYRO),
                "battery_mv": state.sensors.get("battery_mv", 0),
            }
        elif topic_name == "sensors/camera":
            return {
                "has_image": len(state.last_image) > 0,
                "image_age_s": round(time.time() - state.last_image_time, 2)
                               if state.last_image_time else None,
            }
        elif topic_name == "cmd/motor":
            return {"info": "write-only topic, POST /cmd to send commands"}
        elif topic_name == "status":
            return self._full_status()
        return {"error": f"unknown topic: {topic_name}"}

    # ── Config read/write ─────────────────────────────────────────────────────

    CONFIG_SEPARATOR = "."

    def _config_get(self, key: str) -> dict:
        """Get a config value by dot-notation key (e.g. 'robot.type')."""
        parts = key.split(self.CONFIG_SEPARATOR)
        node = self.brain.config
        for part in parts:
            if isinstance(node, dict) and part in node:
                node = node[part]
            else:
                return {"error": f"key not found: {key}"}
        return {"key": key, "value": node}

    def _config_set(self, key: str, body: dict) -> dict:
        """Set a config value by dot-notation key. Body must have 'value'."""
        if "value" not in body:
            return {"error": "missing 'value' field in body"}
        parts = key.split(self.CONFIG_SEPARATOR)
        node = self.brain.config
        for part in parts[:-1]:
            if isinstance(node, dict) and part in node:
                node = node[part]
            else:
                return {"error": f"key not found: {key}"}
        last = parts[-1]
        if not isinstance(node, dict):
            return {"error": f"cannot set on non-dict: {key}"}
        old_value = node.get(last)
        node[last] = body["value"]
        return {"key": key, "old_value": old_value, "value": body["value"]}
