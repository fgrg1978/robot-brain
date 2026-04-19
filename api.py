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
  GET  /fleet/robots    — list all registered robots + aggregated status
  POST /fleet/command   — {"id": "bot_1", "pkt_type": 128, "payload_hex": ".."}
  POST /fleet/broadcast — {"pkt_type": 128, "payload_hex": "..", "type": 0}
  GET  /dashboard[/...] — static dashboard files (B02)

Usage:
    api = APIServer(brain, port=8080)
    asyncio.create_task(api.run())
"""

import asyncio
import json
import os
import time
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from server import BrainServer


# ── Dashboard static files ────────────────────────────────────────────────────
# B02 Fleet Dashboard: vanilla HTML/JS/CSS served from ./dashboard/.

DASHBOARD_DIR_NAME = "dashboard"
DASHBOARD_ROUTE_PREFIX = "/dashboard"
DASHBOARD_INDEX_FILE = "index.html"

# Content-type mapping for static files served under /dashboard.
_STATIC_MIME_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".js":   "application/javascript; charset=utf-8",
    ".css":  "text/css; charset=utf-8",
    ".json": "application/json",
    ".svg":  "image/svg+xml",
    ".png":  "image/png",
    ".jpg":  "image/jpeg",
    ".jpeg": "image/jpeg",
    ".ico":  "image/x-icon",
    ".txt":  "text/plain; charset=utf-8",
}
_DEFAULT_STATIC_MIME = "application/octet-stream"

# Absolute path to the dashboard directory (resolved once at import time).
DASHBOARD_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), DASHBOARD_DIR_NAME)
)


def _guess_content_type(path: str) -> str:
    """Return a content-type header value for a static file path."""
    _, ext = os.path.splitext(path)
    return _STATIC_MIME_TYPES.get(ext.lower(), _DEFAULT_STATIC_MIME)


def _resolve_dashboard_path(url_path: str) -> Optional[str]:
    """Translate a /dashboard[/...] URL to an absolute file path.

    Returns the absolute path if it is a readable file inside DASHBOARD_ROOT,
    or None if the path is missing / escapes the dashboard root / is a dir.
    Index file is served for the bare /dashboard prefix.
    """
    if not url_path.startswith(DASHBOARD_ROUTE_PREFIX):
        return None

    rel = url_path[len(DASHBOARD_ROUTE_PREFIX):]
    if rel in ("", "/"):
        rel = "/" + DASHBOARD_INDEX_FILE
    # Strip leading slash before joining so os.path.join doesn't reset root.
    rel = rel.lstrip("/")

    candidate = os.path.abspath(os.path.join(DASHBOARD_ROOT, rel))

    # Guard against path traversal: must stay inside DASHBOARD_ROOT.
    if os.path.commonpath([candidate, DASHBOARD_ROOT]) != DASHBOARD_ROOT:
        return None
    if not os.path.isfile(candidate):
        return None
    return candidate


def _serve_static(writer: asyncio.StreamWriter, file_path: str) -> None:
    """Write a 200 response with the file body and appropriate content-type."""
    with open(file_path, "rb") as f:
        payload = f.read()
    ctype = _guess_content_type(file_path)
    header = (
        f"HTTP/1.1 200 OK\r\n"
        f"Content-Type: {ctype}\r\n"
        f"Content-Length: {len(payload)}\r\n"
        f"Cache-Control: no-cache\r\n"
        f"Connection: close\r\n"
        f"\r\n"
    ).encode()
    writer.write(header + payload)


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

    # ── Authentication ────────────────────────────────────────────────────
    # The HTTP API was designed for a single-user hobby + private LAN. To
    # avoid trivial tampering on a less-trusted network, accept an API key
    # from either:
    #   - env var `ROBOT_BRAIN_API_KEY`
    #   - constructor `api_key=` argument (e.g. from server.yaml)
    # When set, every request that isn't on `_PUBLIC_ROUTES` must include a
    # matching `Authorization: Bearer <key>` header. When unset (the
    # default), the API stays open as before — explicit opt-in.

    #: Routes that are reachable without auth (health checks, dashboard).
    _PUBLIC_ROUTES = {"/health", "/", ""}

    def __init__(self, brain: "BrainServer", port: int = 8080,
                 api_key: Optional[str] = None):
        self.brain   = brain
        self.port    = port
        self._start  = time.time()
        # Env wins over constructor arg so deployment can override config.
        self.api_key = os.environ.get("ROBOT_BRAIN_API_KEY") or api_key or None

    async def run(self):
        server = await asyncio.start_server(self._handle, "0.0.0.0", self.port)
        if self.api_key:
            print(f"[API] Listening on port {self.port} (auth: API key required)")
        else:
            print(f"[API] Listening on port {self.port} "
                  "(WARNING: no API key set — set ROBOT_BRAIN_API_KEY)")
        async with server:
            await server.serve_forever()

    def _is_authorised(self, path: str, headers: dict) -> bool:
        """Authorise a request: public routes always pass; everything else
        must carry a matching bearer token if `api_key` is configured."""
        if self.api_key is None:
            return True  # explicit opt-out
        # Strip query string + trailing slash for the public-route check.
        bare = path.split("?")[0].rstrip("/") or "/"
        if bare in self._PUBLIC_ROUTES or bare.startswith(DASHBOARD_ROUTE_PREFIX):
            return True
        auth = headers.get("authorization", "")
        if not auth.lower().startswith("bearer "):
            return False
        provided = auth[7:].strip()
        # Constant-time comparison to avoid timing-attack key recovery.
        import hmac
        return hmac.compare_digest(provided, self.api_key)

    async def _handle(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        try:
            raw = await asyncio.wait_for(reader.read(8192), timeout=5)
            method, path, headers, body = _parse_request(raw)
            if not method:
                _response(writer, 400, {"error": "bad request"})
                return
            if not self._is_authorised(path, headers):
                _response(writer, 401, {"error": "unauthorised"})
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
            # Dashboard static files — handled before the normal routes so
            # that /dashboard/app.js etc. don't fall through to 404.
            if path == DASHBOARD_ROUTE_PREFIX or path.startswith(
                    DASHBOARD_ROUTE_PREFIX + "/"):
                file_path = _resolve_dashboard_path(path)
                if file_path is None:
                    _response(writer, 404, {"error": "dashboard file not found"})
                else:
                    _serve_static(writer, file_path)
                return

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
            elif path == "/fleet/robots":
                _response(writer, 200, self._fleet_status())
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

            elif path == "/fleet/command":
                result, http_status = await self._fleet_command(payload)
                _response(writer, http_status, result)

            elif path == "/fleet/broadcast":
                result, http_status = await self._fleet_broadcast(payload)
                _response(writer, http_status, result)

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

    # ── Fleet endpoints ───────────────────────────────────────────────────────

    # HTTP status codes — no magic numbers in route handlers
    HTTP_OK = 200
    HTTP_BAD_REQUEST = 400
    HTTP_NOT_FOUND = 404
    HTTP_SERVICE_UNAVAILABLE = 503

    # Field names accepted by fleet POST bodies
    FLEET_FIELD_ID = "id"
    FLEET_FIELD_PKT_TYPE = "pkt_type"
    FLEET_FIELD_PAYLOAD_HEX = "payload_hex"
    FLEET_FIELD_ROBOT_TYPE = "type"

    def _fleet_manager(self):
        """Return the brain's FleetManager or None."""
        return getattr(self.brain, "fleet_manager", None)

    def _fleet_status(self) -> dict:
        fm = self._fleet_manager()
        if fm is None:
            return {"error": "fleet manager not enabled"}
        # Run timeout sweep before serving status so data is fresh.
        fm.check_timeouts()
        return fm.get_fleet_status()

    @staticmethod
    def _decode_payload_hex(payload_hex: str) -> bytes:
        """Decode hex payload. Accepts empty string as empty bytes."""
        if not payload_hex:
            return b""
        return bytes.fromhex(payload_hex)

    async def _fleet_command(self, body: dict) -> tuple[dict, int]:
        fm = self._fleet_manager()
        if fm is None:
            return {"error": "fleet manager not enabled"}, self.HTTP_SERVICE_UNAVAILABLE

        robot_id = body.get(self.FLEET_FIELD_ID, "")
        pkt_type = body.get(self.FLEET_FIELD_PKT_TYPE)
        payload_hex = body.get(self.FLEET_FIELD_PAYLOAD_HEX, "")

        if not robot_id:
            return {"error": f"missing '{self.FLEET_FIELD_ID}' field"}, self.HTTP_BAD_REQUEST
        if pkt_type is None:
            return {"error": f"missing '{self.FLEET_FIELD_PKT_TYPE}' field"}, self.HTTP_BAD_REQUEST
        try:
            payload = self._decode_payload_hex(payload_hex)
        except ValueError:
            return {"error": "invalid hex in 'payload_hex'"}, self.HTTP_BAD_REQUEST

        ok = await fm.send_targeted(robot_id, int(pkt_type), payload)
        return {
            "id":        robot_id,
            "pkt_type":  int(pkt_type),
            "delivered": ok,
        }, self.HTTP_OK if ok else self.HTTP_NOT_FOUND

    async def _fleet_broadcast(self, body: dict) -> tuple[dict, int]:
        fm = self._fleet_manager()
        if fm is None:
            return {"error": "fleet manager not enabled"}, self.HTTP_SERVICE_UNAVAILABLE

        pkt_type = body.get(self.FLEET_FIELD_PKT_TYPE)
        payload_hex = body.get(self.FLEET_FIELD_PAYLOAD_HEX, "")
        robot_type = body.get(self.FLEET_FIELD_ROBOT_TYPE)

        if pkt_type is None:
            return {"error": f"missing '{self.FLEET_FIELD_PKT_TYPE}' field"}, self.HTTP_BAD_REQUEST
        try:
            payload = self._decode_payload_hex(payload_hex)
        except ValueError:
            return {"error": "invalid hex in 'payload_hex'"}, self.HTTP_BAD_REQUEST

        results = await fm.broadcast(
            int(pkt_type), payload,
            robot_type=int(robot_type) if robot_type is not None else None,
        )
        return {
            "pkt_type":  int(pkt_type),
            "delivered": sum(1 for ok in results.values() if ok),
            "attempted": len(results),
            "results":   results,
        }, self.HTTP_OK

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
