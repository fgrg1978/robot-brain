# Robot Brain

Python brain server for autonomous robots. Pairs with [`robot-os`](https://github.com/fgrg1978/robot-os) — a bare-metal RISC-V Rust kernel — to deliver the full perception → planning → actuation loop for an autonomous ground / air robot.

Runs on macOS/Linux hosts with [LM Studio](https://lmstudio.ai/) (VLM + LLM) locally, communicating with the robot over TCP (WiFi) or UART (ESP32 bridge) using a compact binary protocol.

## Why this exists

Commercial robotic stacks (ROS 2, PX4 ground stations, fleet dashboards) are
either too heavy for a single-person hobby project or lock you into a specific
hardware ecosystem. This repo is the counter-proposal:

- **All-in-one brain for small fleets** — VLM, LLM, fleet manager, MAVLink
  bridge, motion detection, SITL, web dashboard, REST API, Telegram bot — in a
  single ~15k-line Python server with no external services required.
- **Latency you can measure** — the companion kernel keeps the safety/control
  loop under 10 µs; the brain handles cognition (perception / planning) that
  doesn't need hard real-time.
- **Works offline** — the robot has its own autonomy layer (`E05`); the brain
  can cache VLM/LLM responses and serve canned fallbacks when LM Studio is
  unreachable.
- **Hackable end-to-end** — every module is 200-800 lines, no magic, `pytest`
  covers the core.

## Architecture

```
                 ┌───────────────┐
    IP cameras ──┤ RTSP monitor  │ (motion-triggered VLM dispatch)
                 │  (GMM + blob) │
                 └───────┬───────┘
                         │
        ┌────────────────▼─────────────────┐
        │       Brain Server (this repo)   │
        │  ┌────────────┐  ┌────────────┐  │
        │  │ Perception │→│  Planner    │  │
        │  │   (VLM)    │  │ (LLM+skill)│  │
        │  └────────────┘  └────────────┘  │
        │        │               │          │
        │  ┌─────▼───────┐ ┌─────▼────────┐ │
        │  │  Policy     │ │  Executor    │ │
        │  │ (per robot) │ │ (skill run)  │ │
        │  └─────────────┘ └──────────────┘ │
        │           Fleet / API / UI        │
        └─────────────────┬─────────────────┘
                          │   binary TCP / UART
                    ┌─────▼──────┐
                    │  robot-os  │ ←→ sensors / actuators
                    │  (kernel)  │
                    └────────────┘
```

## What's included

### Core server
- **server.py** — TCP listener, packet dispatch, safety enforcement, connection registry
- **protocol.py** — Binary protocol (`BR` magic + type + payload + CRC8), packet types synced with `robot-os`
- **api.py** — HTTP REST API (asyncio, no framework): `/status`, `/mode`, `/fleet/*`, `/dashboard`, `/ota/*`

### Perception
- **perception/vision.py** — VLM via LM Studio (SmolVLM)
- **perception/motion_detect.py** — GMM background model (B04), blob tracker, day/night/IR profiles
- **perception/rtsp_monitor.py** — Multi-IP-camera monitor with motion-triggered VLM
- **perception/surround_view.py** — Bird's-eye-view fusion from 4 cameras (homography blend)

### Planning & policy
- **planner/decide.py** — LLM single-action decision (reactive mode)
- **planner/modes.py** — ModeManager (idle / patrol / investigate / charge)
- **planner/skills.py** — Skill catalogue per robot type
- **planner/task_planner.py** — Free-text → JSON skill plan decomposer
- **planner/mapper.py** — Perimeter waypoint mapping
- **planner/fleet.py** — Fleet-level planner (zone dispatch)
- **policy/{wheeled,drone,humanoid,ackermann}.py** — Skill → actuator translation per type
- **policy/safety.py** — Brain-side safety validation

### Fleet & deployment
- **fleet.py** — FleetManager (registry, heartbeat, broadcast, targeted commands, REST endpoints)
- **dashboard/** — Vanilla JS web UI (no frameworks), served at `/dashboard`
- **mavlink_client.py** — MAVLink v1 bridge to PX4/ArduPilot SITL (pure Python, no pymavlink required)
- **transport.py** — Multi-link client (WiFi/LoRa/RF) with priority failover + failback

### Extras
- **sitl.py** — Multi-type Software-In-The-Loop (wheeled/drone/ackermann/humanoid) with scenarios
- **offline_cache.py** — LRU + TTL cache for VLM/LLM with JSON persistence + canned fallbacks
- **executor/skill_runner.py** — Async state-machine for plan execution
- **notifications.py** — Pushover / Telegram / Email / Webhook alerts
- **telegram_bot.py** — Remote control via Telegram
- **tools/sitl/** — Standalone 2D-physics wheeled SITL with raycast camera

## State (2026-04)

All master-plan phases implemented:
- V / X / Y (modes, skills, task planner, notifications, telegram, API, multi-robot)
- E03 (GPS missions + geofence), E04 (payload abstraction)
- E07 (fleet management), E08 (MAVLink bridge)
- B01 (SITL multi-type), B02 (fleet dashboard), B03 (offline cache), B04 (GMM motion)
- Full multi-link transport client (E02 brain side)

**Tests**: 1115/1115 passing (pytest).

Companion kernel (`robot-os`) implements all core phases (safety, FS, tracing,
secure boot, UEFI scaffolding, COW fork, demand paging, userspace driver
framework). End-to-end smoke passes: kernel boots in QEMU + brain listens +
TCP pipe established.

## Hardware

Three build tiers (indoor / tracked-outdoor / production) documented in:
- **docs/SHOPPING_LIST.md** — BOM + wiring diagram + sensor map (3 tiers, 17-480 EUR)
- **docs/HARDWARE_BUILD.md** — step-by-step assembly + first-boot + troubleshooting

Target robots (hardware arriving July 2026):
- **Tier 1 (~17 €)** — 4WD indoor dev chassis with encoders + MPU6050
- **Tier 2 (~330 €)** — Tracked chassis + LiDAR LD19 + 3S LiPo + IP54 enclosure
- **Tier 3 (+36 €)** — Siren, spotlight, pan/tilt camera, INA219, speaker, LoRa

## Quick start

Install:
```bash
python3.12 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

Run:
```bash
# Edit config.yaml (robot type, LM Studio endpoint, ports)
python -m server
```

Standalone SITL (no hardware required):
```bash
python sitl.py --type wheeled --scenario forward_10m
```

End-to-end test against the kernel in QEMU:
```bash
# From robot-os repo
./tools/test_e2e_auto.sh
```

## Tests

```bash
python3.12 -m pytest tests/              # 1115 tests
python3.12 -m pytest tests/test_protocol.py  # just the kernel protocol sync tests
```

## Companion repo

- **[robot-os](../robot-os)** — RISC-V Rust kernel (this brain's pair)

## License

MIT
