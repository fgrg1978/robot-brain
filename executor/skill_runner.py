"""Skill executor — runs a typed skill plan as a state machine.

A plan is a list of steps from TaskPlanner or ModeManager:
  [{"skill": "FORWARD", "args": {"speed": 60}}, ...]

Each step:
  1. Translates to ActuatorCmd via the policy translator.
  2. Sends the command via send_cmd(cmd).
  3. Waits for the step duration (time-based; nav feedback is a future phase).

Interrupts are cooperative: the runner checks _stop_flag between steps
and during sleeps (every POLL_INTERVAL seconds).

Usage:
    runner = SkillRunner(policy, send_cmd=server.send_actuator_cmd)
    await runner.execute_plan(plan)

    # From another coroutine:
    runner.interrupt("obstacle detected")
"""

import asyncio
import enum
import time
from typing import Callable, Awaitable, Optional

from protocol import ActuatorCmd, FLAG_EMERGENCY


# ── State ─────────────────────────────────────────────────────────────────────

class RunnerState(enum.Enum):
    IDLE        = "idle"
    RUNNING     = "running"
    INTERRUPTED = "interrupted"
    DONE        = "done"
    ERROR       = "error"


# ── Step duration table (seconds) ─────────────────────────────────────────────
# Skills that have explicit timing from args use those.
# Everything else falls back to the values here.

_INSTANT = 0.1   # send-and-done (STOP, EMERGENCY, ALERT, REPORT)

_DEFAULT_STEP_S: dict[str, float] = {
    # Universal
    "STOP":        _INSTANT,
    "EMERGENCY":   _INSTANT,
    "ALERT":       _INSTANT,
    "REPORT":      _INSTANT,
    "SCAN_360":    8.0,      # rotate 360° at low speed
    "INVESTIGATE": 3.0,
    "NAVIGATE_TO": 10.0,    # placeholder until nav module exists
    # Wheeled
    "FORWARD":     2.0,
    "BACKWARD":    2.0,
    "TURN_LEFT":   1.5,
    "TURN_RIGHT":  1.5,
    "FOLLOW_WALL": 5.0,
    "TRACK":       5.0,
    # Drone
    "HOVER":       5.0,
    "TAKEOFF":     4.0,
    "LAND":        4.0,
    "ASCEND":      2.0,
    "DESCEND":     2.0,
    "YAW_LEFT":    1.5,
    "YAW_RIGHT":   1.5,
    "FLY_FORWARD": 2.0,
    "RETURN_HOME": 12.0,
    # Humanoid
    "STAND":       1.0,
    "CROUCH":      1.0,
    "SIT":         1.5,
    "WALK_FORWARD": 3.0,
    "WALK_BACKWARD": 3.0,
    "WAVE":        2.0,
    "NOD":         1.5,
    "SHAKE_HEAD":  1.5,
}

POLL_INTERVAL = 0.05   # seconds between interrupt checks during sleep


# ── SkillRunner ───────────────────────────────────────────────────────────────

class SkillRunner:
    """Async skill-plan executor."""

    def __init__(self,
                 policy,
                 send_cmd: Callable[[ActuatorCmd], Awaitable[None]],
                 on_step_done: Optional[Callable[[str, dict, ActuatorCmd], None]] = None):
        """
        Args:
            policy:       WheeledPolicy | DronePolicy | HumanoidPolicy
            send_cmd:     async callable that sends an ActuatorCmd to the robot
            on_step_done: optional callback(skill, args, cmd) after each step
        """
        self.policy      = policy
        self.send_cmd    = send_cmd
        self.on_step_done = on_step_done

        self.state       = RunnerState.IDLE
        self.current_plan: list[dict] = []
        self.current_step = 0
        self._stop_event: asyncio.Event | None = None   # created lazily per run
        self._interrupt_reason = ""

        # Metrics
        self.steps_executed = 0
        self.last_error     = ""

    # ── Public API ─────────────────────────────────────────────────────────────

    def interrupt(self, reason: str = ""):
        """Request interrupt. Current step finishes, then execution stops."""
        if self._stop_event:
            self._stop_event.set()
        self._interrupt_reason = reason
        if reason:
            print(f"[Runner] Interrupt requested: {reason}")

    def clear(self):
        """Reset to IDLE. Creates a fresh Event bound to the current running loop."""
        self._stop_event      = asyncio.Event()
        self._interrupt_reason = ""
        self.state          = RunnerState.IDLE
        self.current_plan   = []
        self.current_step   = 0
        self.steps_executed = 0

    async def execute_plan(self, plan: list[dict]) -> RunnerState:
        """Execute a plan sequentially. Returns final state.

        Safe to call from a single coroutine; use asyncio.create_task for
        background execution with interrupt() from outside.
        """
        self.clear()
        self.current_plan = plan
        self.state = RunnerState.RUNNING
        print(f"[Runner] Starting plan: {len(plan)} steps")

        for i, step in enumerate(plan):
            if self._stop_event and self._stop_event.is_set():
                self.state = RunnerState.INTERRUPTED
                print(f"[Runner] Interrupted at step {i}: {self._interrupt_reason}")
                break

            self.current_step = i
            skill = step.get("skill", "STOP")
            args  = step.get("args", {}) or {}

            try:
                await self._execute_step(skill, args)
                self.steps_executed += 1
            except Exception as e:
                self.last_error = str(e)
                self.state = RunnerState.ERROR
                print(f"[Runner] Error in step {i} ({skill}): {e}")
                # Emergency stop on error
                await self._send_stop()
                return self.state

        if self.state == RunnerState.RUNNING:
            self.state = RunnerState.DONE
            print(f"[Runner] Plan complete ({self.steps_executed} steps)")

        return self.state

    async def execute_one(self, skill: str, args: dict | None = None) -> ActuatorCmd:
        """Execute a single skill step immediately, bypassing the plan queue."""
        return await self._execute_step(skill, args or {})

    # ── Internal ───────────────────────────────────────────────────────────────

    async def _execute_step(self, skill: str, args: dict) -> ActuatorCmd:
        """Translate, send, and wait for one step."""
        cmd = self.policy.translate(skill, args)
        await self.send_cmd(cmd)
        print(f"[Runner] → {skill} {args}  ch={cmd.channels} flags={cmd.flags:#04x}")

        duration = self._duration(skill, args)
        await self._interruptible_sleep(duration)

        if self.on_step_done:
            self.on_step_done(skill, args, cmd)

        return cmd

    def _duration(self, skill: str, args: dict) -> float:
        """Compute step duration in seconds."""
        # WAIT uses its own args
        if skill == "WAIT":
            return float(args.get("seconds", 1.0))

        # Skills where duration scales with args
        if skill in ("TURN_LEFT", "TURN_RIGHT", "YAW_LEFT", "YAW_RIGHT"):
            # ~2 deg/s at default speed → degrees / 60 (rough)
            return float(args.get("degrees", 90)) / 60.0

        if skill in ("ASCEND", "DESCEND"):
            return float(args.get("meters", 1.0)) * 2.0   # ~0.5 m/s

        if skill in ("WALK_FORWARD", "WALK_BACKWARD"):
            return float(args.get("steps", 5)) * 0.4      # ~0.4 s/step

        if skill == "HOVER":
            secs = args.get("seconds", 0)
            if secs and float(secs) > 0:
                return float(secs)

        return _DEFAULT_STEP_S.get(skill, 2.0)

    async def _interruptible_sleep(self, duration: float):
        """Sleep for duration, but wake early on interrupt."""
        if duration <= 0:
            return
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if self._stop_event and self._stop_event.is_set():
                break
            remaining = deadline - time.monotonic()
            await asyncio.sleep(min(POLL_INTERVAL, remaining))

    async def _send_stop(self):
        """Send an emergency stop."""
        try:
            n = len(self.policy.translate("STOP").channels)
        except Exception:
            n = 2
        cmd = ActuatorCmd.stop(n_channels=n)
        await self.send_cmd(cmd)

    # ── Repr ───────────────────────────────────────────────────────────────────

    def __repr__(self) -> str:
        return (f"SkillRunner(state={self.state.value}, "
                f"step={self.current_step}/{len(self.current_plan)}, "
                f"executed={self.steps_executed})")
