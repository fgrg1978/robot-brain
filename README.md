# Robot Brain

Python brain server for autonomous robots. Runs on macOS with [LM Studio](https://lmstudio.ai/) (VLM + LLM), communicates with a bare-metal RISC-V kernel ([robot-os](https://github.com/fgrg1978/robot-os)) via custom binary protocol over TCP.

## How it works

```
Robot (VisionFive 2) ──sensors/camera──> Brain Server ──> VLM ──> LLM ──> Policy
                      <──actuator cmds──                                    │
                                                                    skill/action
```

The robot sends sensor data and camera frames. The brain sees through a VLM (SmolVLM), decides with an LLM (Llama 3.2), translates decisions into motor commands, and sends them back. Safety checks are always active.

## What's here

- **server.py** — TCP listener, packet dispatch, safety enforcement
- **protocol.py** — Binary protocol (`BR` magic + type + payload + CRC8), shared with robot-os
- **perception/** — VLM interface (LM Studio API)
- **planner/** — LLM decisions, behavior modes, skill catalog, task decomposer
- **policy/** — Translate skills to motor commands (wheeled, drone, humanoid, ackermann)
- **policy/safety.py** — Per-robot-type safety profiles
- **executor/** — Async skill runner (plan to sequential execution)
- **tools/sitl/** — Software-in-the-loop simulator (2D physics, raycasting camera)
- **notifications.py** — Pushover / Telegram / Email / Webhook alerts
- **api.py** — HTTP REST API
- **telegram_bot.py** — Remote control via Telegram

## Sensors

The robot supports a full sensor suite via the kernel's driver layer:

- **Vision** — RPi Camera (CSI MIPI) fed to SmolVLM for scene understanding
- **IMU** — MPU6050 (I2C) for tilt safety and heading
- **Rangefinders** — HC-SR04 ultrasonic (front), laser rangefinder (precise distance)
- **ADC** — ADS1115 16-bit 4-channel (I2C) for battery voltage and analog inputs
- **Digital sensors** — PIR (motion detection), IR (proximity/line following), sound sensor
- **Encoders** — Wheel encoders (GPIO interrupt) for odometry and PID feedback
- **Buzzer** — Passive piezo (PWM) for audio alerts and status tones

See `docs/SHOPPING_LIST.md` for the full hardware BOM, wiring diagram, and sensor integration map.

## What's pending

- **Phase T** — Real CSI camera streaming from kernel
- **Phase Z** — Multi-link transport (WiFi/LoRa/RF/4G)
- **Phase AA** — GPS missions + geofencing
- **Phase AB** — Payload abstraction (spray/gripper/PTO)
- **Phase AC** — Offline autonomy (no brain server)
- **Phase AD** — Logging, replay, analytics
- **Phase AE** — Fleet management
- **Phase AF** — MAVLink bridge
- **Phases AH-AN** — Drone-critical (EKF, SITL/HITL, 3D path, motor mixing, terrain, SLAM, CI)

## Quick start

```bash
pip install -r requirements.txt
# Edit config.yaml (robot type, LM Studio endpoint)
python server.py
# Or run the simulator:
python tools/sitl/sitl_wheeled.py
```

## Tests

```bash
python -m pytest tests/ -v    # 138 tests
```

## License

MIT
