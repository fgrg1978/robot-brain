"""Data-plane TCP server.

Accepts kernel connections on a configurable port.  For each new connection:
  1. Reads the first STATUS or SENSOR packet to extract ``robot_id``.
  2. Calls ``OwnershipChecker.check_ownership(robot_id)``.
  3. If NOT owner → sends an error response and closes the connection.
  4. If owner → hands the connection to the pipeline (perception / planner /
     policy).  In single-process mode this delegates to the existing
     ``BrainServer`` connection handler.  In stub mode (multi-process without
     a wired pipeline), the robot_id is logged and the connection is left
     open until the kernel disconnects.

Security status (READ THIS BEFORE WIRING A REAL PIPELINE):
  There is no peer authentication here.  ``robot_id`` is whatever the peer put
  in its own first packet, and ``check_ownership`` only decides which shard
  should handle that id — it is routing, not identity.  Nothing on this path
  proves the connection belongs to the robot it names.  The exposure is
  currently bounded by two things and neither is a substitute for
  authentication: the listener binds loopback by default
  (``DP_DEFAULT_BIND_HOST``), and ``pipeline_fn`` is never wired to anything
  but ``_pipeline_stub``, so no actuation hangs off it.  Wiring a real
  ``pipeline_fn`` — especially ``BrainServer.handle_robot`` — makes this an
  actuation surface, and an authentication step (shared link key, mTLS, or a
  per-worker token) must land first.

Pipeline integration:
  The existing ``BrainServer`` logic in ``server.py`` is NOT imported at
  module load time.  It is imported lazily inside ``_pipeline_stub`` so that
  unit tests for the data plane never trigger LM Studio / YAML / vision
  imports.  When a real pipeline is wired (full multi-process deployment),
  supply a ``pipeline_fn`` coroutine to the ``DataPlaneServer`` constructor.
"""

from __future__ import annotations

import asyncio
import json
import logging
import struct
from typing import Any, Callable, Coroutine, Optional

from data_plane.worker import OwnershipChecker
from data_plane import CP_INPROCESS_SENTINEL
from control_plane.discovery import ShardCoordinator

logger = logging.getLogger("brain.data_plane")

# ---------------------------------------------------------------------------
# Constants — no magic numbers
# ---------------------------------------------------------------------------

#: Default TCP port for data-plane kernel connections.
DP_DEFAULT_PORT: int = 9100

#: Maximum bytes for the first packet read (used to extract robot_id).
DP_FIRST_PACKET_MAX_BYTES: int = 512

#: Timeout in seconds waiting for the first packet from a new kernel connection.
DP_FIRST_PACKET_TIMEOUT_S: float = 5.0

#: Magic bytes at the start of every brain protocol packet ("BR").
_PROTO_MAGIC: bytes = b"BR"

#: Minimum packet header length: 2 magic + 1 type + 2 length LE = 5 bytes.
_PROTO_HEADER_LEN: int = 5

#: Packet type byte for STATUS (0x03) — carries robot_id.
_PKT_STATUS: int = 0x03

#: Packet type byte for SENSOR (0x01) — may carry robot_id in payload.
_PKT_SENSOR: int = 0x01

#: JSON key in a STATUS payload that carries the robot_id.
_STATUS_ROBOT_ID_KEY: str = "robot_id"

#: Fallback robot_id when the first packet does not carry one.
_FALLBACK_ROBOT_ID_PREFIX: str = "unknown_"

#: Default bind address.
#:
#: B-A16 (partial fix): this server performs NO authentication of the peer.
#: ``_handle`` takes ``robot_id`` straight out of the first packet's JSON, and
#: ``OwnershipChecker.check_ownership`` is shard routing, not authentication —
#: it answers "is this robot mine?", never "is this really that robot?".
#: Binding that to 0.0.0.0, as this did unconditionally, lets any LAN peer
#: claim any robot_id.  Until a real peer-authentication step exists (see the
#: module docstring), keep the socket on loopback so the exposure is bounded
#: to this host; pass ``bind_host`` explicitly if the deployment terminates
#: authentication in front of this process.
DP_DEFAULT_BIND_HOST: str = "127.0.0.1"


# Type alias for an optional custom pipeline coroutine.
PipelineFn = Callable[
    [str, asyncio.StreamReader, asyncio.StreamWriter],
    Coroutine[Any, Any, None],
]


class DataPlaneServer:
    """Async TCP server that gates kernel connections by ownership.

    Args:
        self_node:      This worker's own address string (e.g. "localhost:9100").
                        Use ``CP_INPROCESS_SENTINEL`` for single-process mode.
        coordinator:    Local ``ShardCoordinator`` (single-process mode).
        cp_address:     Control-plane address for HTTP ownership queries
                        (multi-process mode).
        cp_api_key:     Bearer token for control-plane ownership queries.
                        Defaults to ``ROBOT_BRAIN_CP_API_KEY`` from the
                        environment (see ``OwnershipChecker``).
        pipeline_fn:    Optional coroutine called with (robot_id, reader, writer)
                        after ownership is confirmed.  Defaults to a stub that
                        drains the connection.
        port:           TCP port to listen on.
        bind_host:      Interface to bind.  Defaults to loopback — see
                        ``DP_DEFAULT_BIND_HOST`` for why.
    """

    def __init__(
        self,
        *,
        self_node: str = CP_INPROCESS_SENTINEL,
        coordinator: Optional[ShardCoordinator] = None,
        cp_address: Optional[str] = None,
        cp_api_key: Optional[str] = None,
        pipeline_fn: Optional[PipelineFn] = None,
        port: int = DP_DEFAULT_PORT,
        bind_host: str = DP_DEFAULT_BIND_HOST,
    ) -> None:
        self._checker = OwnershipChecker(
            self_node,
            coordinator=coordinator,
            cp_address=cp_address,
            cp_api_key=cp_api_key,
        )
        self._pipeline_fn = pipeline_fn or _pipeline_stub
        self._port = port
        self._bind_host = bind_host
        self._self_node = self_node

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    async def run(self) -> None:
        server = await asyncio.start_server(self._handle, self._bind_host, self._port)
        logger.warning(
            "[DataPlane] node=%s listening on %s:%d — connections are NOT "
            "authenticated; robot_id is taken from the peer's own first packet",
            self._self_node,
            self._bind_host,
            self._port,
        )
        async with server:
            await server.serve_forever()

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    async def _handle(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
    ) -> None:
        peer = writer.get_extra_info("peername", ("?", 0))
        logger.debug("[DataPlane] new connection from %s:%s", peer[0], peer[1])
        try:
            raw = await asyncio.wait_for(
                reader.read(DP_FIRST_PACKET_MAX_BYTES),
                timeout=DP_FIRST_PACKET_TIMEOUT_S,
            )
            # NOTE: robot_id here is self-declared by the peer and unverified;
            # check_ownership below is shard routing, not authentication.
            # See the "Security status" section of the module docstring.
            robot_id = _extract_robot_id(raw, peer)
            owns = await self._checker.check_ownership(robot_id)
            if not owns:
                logger.info("[DataPlane] refusing robot_id=%s (not our shard)", robot_id)
                _send_reject(writer, robot_id)
                return
            logger.info("[DataPlane] accepted robot_id=%s", robot_id)
            # Prepend the already-read bytes back for the pipeline.
            combined_reader = _PrefixReader(raw, reader)
            await self._pipeline_fn(robot_id, combined_reader, writer)  # type: ignore[arg-type]
        except asyncio.TimeoutError:
            logger.warning("[DataPlane] timeout waiting for first packet from %s", peer)
        except Exception as exc:
            logger.error("[DataPlane] connection error from %s: %s", peer, exc)
        finally:
            try:
                writer.close()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _extract_robot_id(
    raw: bytes,
    peer: tuple[str, int],
) -> str:
    """Best-effort extraction of robot_id from the first packet.

    Tries to parse a STATUS (0x03) or SENSOR (0x01) packet with a JSON body
    that contains ``"robot_id"`` key.  Falls back to a peer-address-based ID
    when the packet is too short or unparseable.
    """
    if len(raw) >= _PROTO_HEADER_LEN and raw[:2] == _PROTO_MAGIC:
        pkt_type = raw[2]
        payload_len = struct.unpack_from("<H", raw, 3)[0]
        payload = raw[_PROTO_HEADER_LEN : _PROTO_HEADER_LEN + payload_len]
        if pkt_type in (_PKT_STATUS, _PKT_SENSOR):
            try:
                data: dict[str, Any] = json.loads(payload)
                rid = data.get(_STATUS_ROBOT_ID_KEY)
                if isinstance(rid, str) and rid:
                    return rid
            except (json.JSONDecodeError, UnicodeDecodeError):
                pass
    # Fallback: derive from peer address so the ID is at least unique.
    return f"{_FALLBACK_ROBOT_ID_PREFIX}{peer[0]}_{peer[1]}"


def _send_reject(writer: asyncio.StreamWriter, robot_id: str) -> None:
    """Send a minimal JSON error over the wire before closing."""
    body = json.dumps({"error": "not_owner", "robot_id": robot_id}).encode()
    writer.write(body)


async def _pipeline_stub(
    robot_id: str,
    reader: asyncio.StreamReader,  # type: ignore[type-arg]
    writer: asyncio.StreamWriter,
) -> None:
    """Default pipeline: drain the connection and log.

    Replace with a real pipeline coroutine in production deployments by
    supplying ``pipeline_fn`` to ``DataPlaneServer``.

    In single-process mode the caller wires the existing ``BrainServer``
    connection handler here so the full perception/planner/policy stack runs.
    """
    logger.info("[DataPlane/stub] pipeline for robot_id=%s (draining)", robot_id)
    try:
        while True:
            chunk = await reader.read(4096)
            if not chunk:
                break
    except Exception:
        pass


class _PrefixReader:
    """Wraps an asyncio.StreamReader, prepending already-read bytes.

    This lets us hand the pipeline the full byte stream including the first
    packet we already consumed for robot_id extraction.
    """

    def __init__(self, prefix: bytes, inner: asyncio.StreamReader) -> None:
        self._buf = bytearray(prefix)
        self._inner = inner

    async def read(self, n: int = -1) -> bytes:
        if self._buf:
            if n > 0:
                chunk = bytes(self._buf[:n])
                del self._buf[:n]
            else:
                chunk = bytes(self._buf)
                del self._buf[:]
            return chunk
        return await self._inner.read(n)

    # Proxy other StreamReader methods the pipeline may need.
    async def readexactly(self, n: int) -> bytes:
        result = bytearray()
        while len(result) < n:
            need = n - len(result)
            chunk = await self.read(need)
            if not chunk:
                raise asyncio.IncompleteReadError(bytes(result), n)
            result.extend(chunk)
        return bytes(result)


# ---------------------------------------------------------------------------
# Standalone entry-point (``python -m data_plane.main``)
# ---------------------------------------------------------------------------


def _build_arg_parser() -> "argparse.ArgumentParser":  # type: ignore[name-defined]
    import argparse

    p = argparse.ArgumentParser(description="PHANES brain — data plane worker")
    p.add_argument(
        "--port", type=int, default=DP_DEFAULT_PORT, help="TCP port for kernel connections"
    )
    p.add_argument(
        "--self-node",
        type=str,
        default=CP_INPROCESS_SENTINEL,
        help="This worker's address (host:port)",
    )
    p.add_argument(
        "--cp-address",
        type=str,
        default=None,
        help="Control-plane address (host:port) for ownership queries",
    )
    p.add_argument(
        "--bind-host",
        type=str,
        default=DP_DEFAULT_BIND_HOST,
        help=(
            "Interface to bind (default: %(default)s). Connections are NOT "
            "authenticated — only widen this if something in front of this "
            "process authenticates the peer."
        ),
    )
    return p


def main() -> None:
    import argparse

    args = _build_arg_parser().parse_args()
    coord: Optional[ShardCoordinator] = None
    if args.self_node == CP_INPROCESS_SENTINEL:
        coord = ShardCoordinator()
    server = DataPlaneServer(
        self_node=args.self_node,
        coordinator=coord,
        cp_address=args.cp_address,
        port=args.port,
        bind_host=args.bind_host,
    )
    asyncio.run(server.run())


if __name__ == "__main__":
    main()
