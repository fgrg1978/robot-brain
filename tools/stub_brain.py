#!/usr/bin/env python3
"""Headless stub brain for Phase 1 E2E with a real kernel in QEMU.

Goal: drive the kernel↔brain TCP loop end-to-end without depending on
LM Studio, openai, or any vision/LLM stack. `server.py` is the full
production brain (vision → planner → policy → notifications); this
stub is for the minimum-path scenario in MEMORY.md:

    "robot wheeled funcional E2E en QEMU antes de expandir plan"

What it does:

    1. Listens on TCP 0.0.0.0:9000 (`DEFAULT_PORT`, matching
       `build/CONFIG.INI`'s `behavior_server_port`; QEMU forwards it
       from the host, which the guest reaches as 10.0.2.2).
    2. Accepts ONE robot connection (the kernel boots, reads
       CONFIG.INI, and connects out to 10.0.2.2:9000 = QEMU host).
    3. Decodes every packet using `protocol.parse_packet` so we get
       the full kernel-side wire-format coverage. Counts each kind.
    4. After receiving `--sensors-to-go` sensor packets (default 3),
       sends `ActuatorCmd.wheeled(50, 50)` — forward at 50 %.
    5. After `--duration-s` seconds total, sends a stop command and
       closes the connection cleanly.
    6. Exits 0 iff we received ≥ 1 sensor packet AND sent ≥ 1
       actuator command; exits 1 otherwise (lets a CI harness gate
       on "the loop actually closed").

Auth envelope (RFC-0011)
------------------------

When `ROBOT_BRAIN_LINK_KEY` is exported, the stub arms the HMAC
envelope exactly like `server.py` does — `protocol.enable_auth_envelope()`
— and every wire frame becomes `envelope(brain_frame)`, with the MAC
bound to the direction label (`C2S` outbound, `S2C` inbound; hashed,
never transmitted). A configured key that fails to arm is a hard error,
never a silent downgrade to plaintext.

With the envelope armed, bytes that do not unwrap are a VERIFICATION
FAILURE, not noise: they are counted, logged, and they fail the final
verdict (exit `EXIT_AUTH_FAILURE`). The only tolerated discard is the
QEMU-slirp first-byte drop, and only before the first MAC-verified
frame — and even then the resync is confirmed by a MAC, so it cannot
swallow a forged or mis-keyed stream.

Without a key the channel stays plaintext (most scenarios run this
way) and the MAGIC resync behaves as before — counted and logged, but
not fatal, because there is no cryptography to verify.

Usage:

    # Run in foreground:
    python3 tools/stub_brain.py

    # In another terminal (or use scripts/e2e_wheeled_qemu.sh):
    make qemu-full-smp     # in robot-os
    # → kernel boots, connects out, you see SENSOR packets here.

    # CI / scripted use:
    python3 tools/stub_brain.py --duration-s 20 --quiet | tee brain.log

    # Authenticated link (the kernel needs the same /fat/LINK.KEY):
    export ROBOT_BRAIN_LINK_KEY=$(xxd -p -c64 build/LINK.KEY)
    python3 tools/stub_brain.py --duration-s 20

    # NEGATIVE gate — proves the envelope actually rejects:
    #   corrupts the key on purpose; exits 0 ONLY if nothing verified.
    python3 tools/stub_brain.py --duration-s 10 --expect-auth-failure
"""

import argparse
import asyncio
import os
import struct
import sys
import time
from typing import Optional

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT_DIR = os.path.dirname(_THIS_DIR)
sys.path.insert(0, _ROOT_DIR)

import protocol
from protocol import (
    parse_packet,
    SensorPacket,
    SensorPacketDrone,
    SensorPacketHumanoid,
    SensorCompact,
    StatusPacket,
    ActuatorCmd,
    SENSOR_PACKET,
    CAMERA_FRAME,
    STATUS,
    OTA_ACK,
    SENSOR_COMPACT,
    ROBOT_WHEELED,
)
from secure_channel import (
    ENVELOPE_OVERHEAD,
    HMAC_BYTES,
    KEY_BYTES,
    LEN_BYTES,
    MAX_INNER_BYTES,
    NONCE_BYTES,
)

# ── Defaults (named constants — no magic numbers) ────────────────────────────

DEFAULT_HOST = "0.0.0.0"
# Default port matches `build/CONFIG.INI`'s `behavior_server_port`.
# In QEMU user-mode networking the guest reaches the host at
# 10.0.2.2, so the kernel dials 10.0.2.2:9000 → our listener.
DEFAULT_PORT = 9000
DEFAULT_DURATION_S = 30
DEFAULT_SENSORS_TO_GO = 3
DEFAULT_FORWARD_PCT = 50
RX_BUF_SIZE = 64 * 1024
EXIT_OK = 0
EXIT_NO_LOOP = 1
EXIT_TIMEOUT = 2
#: Envelope verification failed (or was expected to fail and did not).
EXIT_AUTH_FAILURE = 3

# ── Frame geometry (derived from protocol.py, never hardcoded) ───────────────

#: MAGIC(2) + TYPE(1) + LEN(2)
FRAME_HEADER_BYTES = len(protocol.MAGIC) + 1 + 2
FRAME_CRC_BYTES = 1
FRAME_OVERHEAD_BYTES = FRAME_HEADER_BYTES + FRAME_CRC_BYTES
#: Largest brain-protocol frame that can ride inside one envelope.
MAX_FRAME_BYTES = FRAME_OVERHEAD_BYTES + protocol.MAX_PAYLOAD_BYTES
#: Offset of the (plaintext) inner-length field inside an envelope.
ENVELOPE_LEN_OFFSET = NONCE_BYTES + HMAC_BYTES

#: Upper bound on bytes we may discard while hunting for the FIRST
#: MAC-verified frame. QEMU slirp drops the first byte of the first
#: application segment; losing byte 0 destroys the whole first envelope,
#: so the next frame boundary can be up to one maximum-size frame away.
#: Past this budget we stop looking and report the bytes as unverified.
MAX_RESYNC_SKEW_BYTES = ENVELOPE_OVERHEAD + MAX_FRAME_BYTES

#: Cap on per-event resync/verification log lines so a mis-keyed peer
#: cannot flood the CI log; the final summary always carries the totals.
MAX_NOISE_LOG_LINES = 5


def log(quiet: bool, msg: str) -> None:
    if not quiet:
        print(msg, flush=True)


def _corrupt_key_hex(key_hex: str) -> str:
    """Flip the low bit of the key's last byte — used by
    `--expect-auth-failure` so the negative gate needs no second key."""
    raw = bytearray(bytes.fromhex(key_hex))
    if len(raw) != KEY_BYTES:
        raise ValueError(f"link key must be {KEY_BYTES} bytes, got {len(raw)}")
    raw[-1] ^= 0x01
    return bytes(raw).hex()


class StubBrain:
    """A single-connection headless brain.

    Per-connection state lives on the instance so a future fork to
    handle multiple kernels would just spawn one per accept().
    """

    def __init__(
        self,
        duration_s: int,
        sensors_to_go: int,
        forward_pct: int,
        quiet: bool,
        expect_auth_failure: bool = False,
    ):
        self.duration_s = duration_s
        self.sensors_to_go = sensors_to_go
        self.forward_pct = forward_pct
        self.quiet = quiet
        self.expect_auth_failure = expect_auth_failure
        self.pkt_counts: dict[int, int] = {}
        self.cmds_sent = 0
        self.first_seen_at: Optional[float] = None
        self.start_t: Optional[float] = None
        self.connected = False

        # ── Link accounting ──────────────────────────────────────────
        #: True when the HMAC envelope is armed for this run.
        self.envelope = protocol._link_receiver is not None
        #: Every byte read off the socket (verified or not).
        self.bytes_rx = 0
        #: Envelopes whose MAC verified (and were not replays).
        self.verified_frames = 0
        #: Envelopes with intact framing whose MAC did NOT verify —
        #: forged, mis-keyed, or wrong-direction. Always fatal.
        self.auth_failures = 0
        #: MAC verified but the inner frame does not parse → the two
        #: repos disagree on the wire format. Always fatal.
        self.bad_inner_frames = 0
        #: Bytes discarded before the first verified/parsed frame and
        #: within `MAX_RESYNC_SKEW_BYTES` (the slirp drop). Tolerated,
        #: but reported.
        self.skew_bytes = 0
        #: Bytes discarded that never verified/parsed. Fatal when the
        #: envelope is armed; informational in plaintext mode.
        self.noise_bytes = 0
        #: Bytes of a frame still in flight when the peer closed. Ordinary
        #: TCP teardown, NOT a verification failure — never fatal.
        self.truncated_bytes = 0

        self._synced = False
        self._sync_abandoned = False
        self._noise_logs = 0

    # ── Logging helpers ──────────────────────────────────────────────

    def _note(self, msg: str) -> None:
        """Bounded per-event logging (totals always land in the summary)."""
        if self._noise_logs >= MAX_NOISE_LOG_LINES:
            return
        self._noise_logs += 1
        log(self.quiet, msg)
        if self._noise_logs == MAX_NOISE_LOG_LINES:
            log(self.quiet, "[STUB] (further resync/verification events suppressed)")

    def _label(self, pkt_type: int) -> str:
        return {
            SENSOR_PACKET: "SENSOR",
            CAMERA_FRAME: "CAMERA",
            STATUS: "STATUS",
            OTA_ACK: "OTA_ACK",
            SENSOR_COMPACT: "SENSOR_COMPACT",
        }.get(pkt_type, f"0x{pkt_type:02x}")

    def _summary(self) -> str:
        # NOTE: the `rx[...] tx[ACTUATOR=N] duration=Ns` prefix is parsed by
        # tools/bench_e2e_collect.py (_RE_STUB_SUMMARY) — append, never reshape.
        kinds = ", ".join(f"{self._label(k)}={v}" for k, v in sorted(self.pkt_counts.items()))
        out = (
            f"rx[{kinds or 'none'}] tx[ACTUATOR={self.cmds_sent}] "
            f"duration={int(time.time() - (self.start_t or time.time()))}s"
        )
        if self.envelope:
            out += (
                f" auth[verified={self.verified_frames}, failed={self.auth_failures}, "
                f"bad_inner={self.bad_inner_frames}, skew={self.skew_bytes}B, "
                f"unverified={self.noise_bytes}B, truncated={self.truncated_bytes}B]"
            )
        elif self.skew_bytes or self.noise_bytes:
            out += f" resync[skew={self.skew_bytes}B, dropped={self.noise_bytes}B]"
        return out

    # ── Byte accounting ──────────────────────────────────────────────

    def _discard(self, count: int, why: str) -> None:
        """Account for `count` bytes we are about to throw away.

        Before the first good frame, and only within the slirp budget,
        the loss is tolerated (`skew_bytes`). Everything else lands in
        `noise_bytes` — fatal when the envelope is armed.
        """
        if count <= 0:
            return
        if not self._synced and self.skew_bytes + count <= MAX_RESYNC_SKEW_BYTES:
            self.skew_bytes += count
            self._note(f"[STUB] resync: tolerating {count}B before first frame ({why})")
            return
        self.noise_bytes += count
        self._note(f"[STUB] DISCARD {count}B that never verified/parsed ({why})")

    def _note_eof_remainder(self, buf: bytes) -> None:
        """Classify whatever is left in the buffer when the peer closes.

        A stream cut mid-frame is ordinary TCP teardown (the kernel closes
        while a frame is in flight), not a forgery — counting it as a
        verification failure would make the gate flake on disconnects
        instead of on cryptography. Anything that is NOT a plausible
        partial frame still counts as unverified.
        """
        if not buf:
            return
        partial = False
        if not self.envelope:
            partial = True  # plaintext: never fatal anyway
        elif len(buf) < ENVELOPE_OVERHEAD:
            partial = True  # not even a full envelope header arrived
        else:
            n = int.from_bytes(
                buf[ENVELOPE_LEN_OFFSET : ENVELOPE_LEN_OFFSET + LEN_BYTES], "little"
            )
            partial = 0 < n <= MAX_INNER_BYTES and len(buf) < ENVELOPE_OVERHEAD + n
        if partial:
            self.truncated_bytes += len(buf)
            self._note(
                f"[STUB] EOF with {len(buf)}B of an incomplete frame "
                f"(connection cut mid-frame — not a verification failure)"
            )
            return
        self._discard(len(buf), "unusable bytes left at EOF")

    # ── Envelope framing (ROBOT_BRAIN_LINK_KEY armed) ────────────────

    def _accept_inner(self, inner: bytes, out: list[tuple[int, bytes]]) -> None:
        """Record one MAC-verified envelope and parse its inner frame."""
        self._synced = True
        self.verified_frames += 1
        parsed = parse_packet(inner)
        if parsed is None:
            self.bad_inner_frames += 1
            self._note(
                f"[STUB] MAC OK but inner frame does not parse ({len(inner)}B) — "
                f"wire-format drift between robot-os and robot-brain"
            )
            return
        out.append(parsed)

    def _scan_for_sync(self, buf: bytes) -> Optional[tuple[int, bytes, int]]:
        """Hunt for the first offset > 0 whose envelope MAC verifies.

        Verification-gated: an offset only counts as a resync if the key
        proves it, so this can never swallow a forged or mis-keyed
        stream. Returns `(offset, inner, frame_len)` or None.
        """
        receiver = protocol._link_receiver
        assert receiver is not None
        limit = min(MAX_RESYNC_SKEW_BYTES, len(buf) - ENVELOPE_OVERHEAD)
        for off in range(1, limit + 1):
            lo = off + ENVELOPE_LEN_OFFSET
            n = int.from_bytes(buf[lo : lo + LEN_BYTES], "little")
            if not 0 < n <= MAX_INNER_BYTES:
                continue
            total = ENVELOPE_OVERHEAD + n
            if len(buf) < off + total:
                continue
            inner = receiver.unwrap(buf[off : off + total])
            if inner is not None:
                return (off, inner, total)
        return None

    def _drain_envelope(self, buf: bytes) -> tuple[bytes, list[tuple[int, bytes]]]:
        """Unwrap as many whole envelopes as `buf` holds.

        The inner-length field is plaintext, so intact framing with a bad
        MAC is unambiguous: wrong key / wrong direction / forgery, counted
        immediately. Only garbage framing (a lost byte) engages the
        bounded, MAC-confirmed resync scan.
        """
        receiver = protocol._link_receiver
        assert receiver is not None
        out: list[tuple[int, bytes]] = []
        while True:
            if len(buf) < ENVELOPE_OVERHEAD:
                break
            n = int.from_bytes(
                buf[ENVELOPE_LEN_OFFSET : ENVELOPE_LEN_OFFSET + LEN_BYTES], "little"
            )
            if 0 < n <= MAX_INNER_BYTES:
                total = ENVELOPE_OVERHEAD + n
                if len(buf) < total:
                    break  # frame still arriving
                inner = receiver.unwrap(buf[:total])
                buf = buf[total:]
                if inner is not None:
                    self._accept_inner(inner, out)
                    continue
                # Framing intact, MAC rejected → wrong key, wrong
                # direction label, replay, or forgery. Never "noise".
                self.auth_failures += 1
                self.noise_bytes += total
                self._note(
                    f"[STUB] AUTH FAILURE: envelope of {total}B failed to verify "
                    f"(hmac_drops={receiver.drops_hmac} replay_drops={receiver.drops_replay})"
                )
                continue
            # Length field is implausible → we are not on a frame boundary.
            if self._synced or self._sync_abandoned:
                self._discard(len(buf), "lost envelope framing mid-stream")
                buf = b""
                break
            hit = self._scan_for_sync(buf)
            if hit is None:
                if len(buf) - ENVELOPE_OVERHEAD > MAX_RESYNC_SKEW_BYTES:
                    # Whole tolerated window scanned, with bytes to spare.
                    self._sync_abandoned = True
                    self._discard(len(buf), "no MAC-verifying frame within resync budget")
                    buf = b""
                break  # otherwise: wait for more bytes
            off, inner, total = hit
            self._discard(off, "slirp first-byte drop, resync confirmed by MAC")
            self._accept_inner(inner, out)
            buf = buf[off + total :]
        return buf, out

    # ── Plaintext framing (no key configured) ────────────────────────

    def _drain_plain(self, buf: bytes) -> tuple[bytes, list[tuple[int, bytes]]]:
        """Drain whole packets, resyncing on MAGIC.

        Defensive resync: QEMU slirp NAT has a reproducible bug that
        drops the FIRST byte of the FIRST application-data segment after
        the TCP handshake. We compensate by scanning forward to the next
        MAGIC instead of requiring it at offset 0. On real hardware
        (Ethernet, no slirp) the scan is a no-op. Discards are counted
        and logged; with no key there is nothing to verify, so they are
        reported but not fatal.
        """
        out: list[tuple[int, bytes]] = []
        while True:
            idx = buf.find(protocol.MAGIC)
            if idx == -1:
                # Keep the last byte in case MAGIC straddles two chunks.
                keep = buf[-1:]
                self._discard(len(buf) - len(keep), "no MAGIC in buffer")
                buf = keep
                break
            if idx > 0:
                self._discard(idx, "bytes before MAGIC")
                buf = buf[idx:]
            parsed = parse_packet(buf)
            if parsed is None:
                if len(buf) >= FRAME_HEADER_BYTES:
                    (length,) = struct.unpack_from("<H", buf, len(protocol.MAGIC) + 1)
                    if (
                        length <= protocol.MAX_PAYLOAD_BYTES
                        and len(buf) >= FRAME_OVERHEAD_BYTES + length
                    ):
                        # Complete frame that failed CRC → skip this MAGIC.
                        self._discard(len(protocol.MAGIC), "CRC mismatch at MAGIC")
                        buf = buf[len(protocol.MAGIC) :]
                        continue
                break  # incomplete — wait for more bytes
            pkt_type, payload = parsed
            buf = buf[FRAME_OVERHEAD_BYTES + len(payload) :]
            self._synced = True
            out.append((pkt_type, payload))
        return buf, out

    # ── Connection handling ──────────────────────────────────────────

    async def _send(self, writer: asyncio.StreamWriter, pkt_type: int, payload: bytes) -> None:
        # protocol.send_packet applies whichever link layers are armed
        # (HMAC envelope when a key is configured), so the stub emits
        # byte-identical frames to the production server.
        await protocol.send_packet(writer, pkt_type, payload)

    async def handle(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
    ) -> None:
        peer = writer.get_extra_info("peername")
        self.envelope = protocol._link_receiver is not None
        log(self.quiet, f"[STUB] robot connected from {peer}")
        log(
            self.quiet,
            "[STUB] link: HMAC envelope ACTIVE (direction-bound MAC)"
            if self.envelope
            else "[STUB] link: PLAINTEXT (no ROBOT_BRAIN_LINK_KEY)",
        )
        self.connected = True
        self.start_t = time.time()
        sensors_seen = 0
        buf = b""

        try:
            while True:
                # Watchdog on overall duration.
                elapsed = time.time() - self.start_t
                if elapsed >= self.duration_s:
                    log(self.quiet, "[STUB] duration reached, sending stop")
                    await self._send(
                        writer,
                        protocol.ACTUATOR_CMD,
                        ActuatorCmd.wheeled(0, 0).to_bytes(),
                    )
                    self.cmds_sent += 1
                    return

                # Read whatever's available, bounded by remaining time.
                try:
                    chunk = await asyncio.wait_for(
                        reader.read(RX_BUF_SIZE),
                        timeout=max(0.5, self.duration_s - elapsed),
                    )
                except asyncio.TimeoutError:
                    continue
                if not chunk:
                    log(self.quiet, "[STUB] EOF from robot")
                    self._note_eof_remainder(buf)
                    buf = b""
                    return
                buf += chunk
                self.bytes_rx += len(chunk)

                # Drain as many whole packets as the buffer holds.
                if self.envelope:
                    buf, packets = self._drain_envelope(buf)
                else:
                    buf, packets = self._drain_plain(buf)

                for pkt_type, payload in packets:
                    self.pkt_counts[pkt_type] = self.pkt_counts.get(pkt_type, 0) + 1
                    if self.first_seen_at is None:
                        self.first_seen_at = time.time()
                        log(
                            self.quiet,
                            f"[STUB] first packet: type={self._label(pkt_type)} "
                            f"len={len(payload)}",
                        )

                    if pkt_type == SENSOR_PACKET:
                        sensors_seen += 1
                        if sensors_seen == self.sensors_to_go:
                            log(
                                self.quiet,
                                f"[STUB] {sensors_seen} sensors received → "
                                f"sending wheeled forward {self.forward_pct}%",
                            )
                            await self._send(
                                writer,
                                protocol.ACTUATOR_CMD,
                                ActuatorCmd.wheeled(
                                    self.forward_pct, self.forward_pct
                                ).to_bytes(),
                            )
                            self.cmds_sent += 1

        except (ConnectionResetError, BrokenPipeError) as e:
            log(self.quiet, f"[STUB] connection error: {e}")
        finally:
            try:
                writer.close()
                await writer.wait_closed()
            except Exception:
                pass
            log(self.quiet, f"[STUB] disconnected — {self._summary()}")


def compute_exit_code(brain: StubBrain, accept_tcp_only: bool = False) -> int:
    """The verdict, as a pure function of the run's accounting.

    Kept separate from `amain` so the gate itself is unit-testable — a
    gate nobody has watched reject something is trusted on faith.
    """
    quiet = brain.quiet
    if not brain.connected:
        log(quiet, "[STUB] no robot ever connected — kernel didn't dial us")
        return EXIT_TIMEOUT

    if brain.expect_auth_failure:
        # Negative gate: the key is wrong ON PURPOSE. Success means the
        # envelope rejected everything that arrived.
        if brain.bytes_rx == 0:
            log(
                quiet,
                "[STUB] FAIL (negative gate inconclusive): not one byte arrived, "
                "so nothing was ever offered for verification",
            )
            return EXIT_NO_LOOP
        if brain.verified_frames or brain.pkt_counts:
            log(
                quiet,
                f"[STUB] FAIL: {brain.verified_frames} frame(s) VERIFIED under a "
                f"deliberately WRONG key — the envelope is NOT being enforced. "
                f"{brain._summary()}",
            )
            return EXIT_AUTH_FAILURE
        log(
            quiet,
            f"[STUB] OK (expected auth failure): rejected {brain.auth_failures} frame(s) / "
            f"{brain.bytes_rx}B, verified none. {brain._summary()}",
        )
        return EXIT_OK

    if brain.envelope:
        if brain.auth_failures or brain.bad_inner_frames or brain.noise_bytes:
            log(
                quiet,
                f"[STUB] FAIL: auth envelope verification failed — "
                f"{brain.auth_failures} bad MAC, {brain.bad_inner_frames} unparseable inner, "
                f"{brain.noise_bytes}B unverified. {brain._summary()}",
            )
            return EXIT_AUTH_FAILURE
        if brain.bytes_rx and brain.verified_frames == 0:
            log(
                quiet,
                f"[STUB] FAIL: received {brain.bytes_rx}B but not one envelope verified. "
                f"{brain._summary()}",
            )
            return EXIT_AUTH_FAILURE

    # `--accept-tcp-only` reports SUCCESS on a TCP handshake alone,
    # without requiring inbound sensor packets. Useful under QEMU
    # TCG on macOS where the host can't emulate the kernel timer
    # in real time — sensor-task `task_block(Timer(+100ms))` becomes
    # a multi-second wait, so the brain never sees a packet in the
    # test window even though the kernel TCP path is healthy.
    # It never masks a crypto failure: those are checked above.
    if accept_tcp_only:
        log(quiet, f"[STUB] OK (tcp-only): {brain._summary()}")
        return EXIT_OK
    if not (sum(brain.pkt_counts.values()) > 0 and brain.cmds_sent > 0):
        log(quiet, f"[STUB] loop incomplete: {brain._summary()}")
        return EXIT_NO_LOOP
    log(quiet, f"[STUB] OK: {brain._summary()}")
    return EXIT_OK


def arm_link_layer(quiet: bool, expect_auth_failure: bool) -> Optional[int]:
    """Arm the HMAC envelope from the environment.

    Returns None on success, or an exit code on a fatal misconfiguration.
    Never downgrades silently: if a key is configured it MUST arm.
    """
    key_hex = os.environ.get("ROBOT_BRAIN_LINK_KEY", "").strip()

    if expect_auth_failure:
        if not key_hex:
            print(
                "[STUB] FATAL: --expect-auth-failure needs ROBOT_BRAIN_LINK_KEY set "
                "(it corrupts that key on purpose); with no key there is no "
                "envelope to reject anything",
                flush=True,
            )
            return EXIT_AUTH_FAILURE
        try:
            key_hex = _corrupt_key_hex(key_hex)
        except ValueError as e:
            print(f"[STUB] FATAL: --expect-auth-failure: {e}", flush=True)
            return EXIT_AUTH_FAILURE
        os.environ["ROBOT_BRAIN_LINK_KEY"] = key_hex
        print(
            "[STUB] --expect-auth-failure: link key deliberately CORRUPTED "
            "(last byte bit-flipped) — NOTHING must verify; exit 0 only if "
            "every frame is rejected",
            flush=True,
        )

    if not key_hex:
        log(quiet, "[STUB] link: PLAINTEXT (ROBOT_BRAIN_LINK_KEY unset)")
        return None

    if not protocol.enable_auth_envelope():
        # Loud even under --quiet: a configured key that fails to load is
        # a security regression, not a log-level decision.
        print(
            "[STUB] FATAL: ROBOT_BRAIN_LINK_KEY is set but the auth envelope "
            "refused to arm (bad hex or wrong length) — refusing to run "
            "unauthenticated",
            flush=True,
        )
        return EXIT_AUTH_FAILURE
    log(
        quiet,
        "[STUB] link: HMAC envelope ARMED from ROBOT_BRAIN_LINK_KEY "
        "(direction-bound MAC: C2S out, S2C in)",
    )
    return None


async def amain(args: argparse.Namespace) -> int:
    armed = arm_link_layer(args.quiet, args.expect_auth_failure)
    if armed is not None:
        return armed

    brain = StubBrain(
        duration_s=args.duration_s,
        sensors_to_go=args.sensors_to_go,
        forward_pct=args.forward_pct,
        quiet=args.quiet,
        expect_auth_failure=args.expect_auth_failure,
    )

    server = await asyncio.start_server(brain.handle, host=args.host, port=args.port)
    addrs = ", ".join(str(s.getsockname()) for s in server.sockets)
    log(args.quiet, f"[STUB] listening on {addrs}")

    # Accept one connection's lifecycle, then return.
    async with server:
        try:
            await asyncio.wait_for(
                server.serve_forever(),
                timeout=args.duration_s + args.accept_grace_s,
            )
        except asyncio.TimeoutError:
            pass

    return compute_exit_code(brain, accept_tcp_only=args.accept_tcp_only)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--duration-s", type=int, default=DEFAULT_DURATION_S)
    ap.add_argument("--sensors-to-go", type=int, default=DEFAULT_SENSORS_TO_GO)
    ap.add_argument("--forward-pct", type=int, default=DEFAULT_FORWARD_PCT)
    ap.add_argument(
        "--accept-grace-s",
        type=int,
        default=5,
        help="extra seconds to wait for the kernel to dial us in",
    )
    ap.add_argument(
        "--accept-tcp-only",
        action="store_true",
        help="report success on TCP handshake alone (no rx needed) — "
        "use under QEMU TCG where the host can't keep up with "
        "the kernel timer in real time. Crypto failures still fail.",
    )
    ap.add_argument(
        "--expect-auth-failure",
        action="store_true",
        help="NEGATIVE gate: corrupt ROBOT_BRAIN_LINK_KEY on purpose and "
        "exit 0 only if nothing verifies (proves the envelope rejects)",
    )
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()
    return asyncio.run(amain(args))


if __name__ == "__main__":
    sys.exit(main())
