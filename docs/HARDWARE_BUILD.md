# Hardware Build Guide — Robot Wheeled v1

Step-by-step assembly for a wheeled autonomous robot using the VisionFive 2
(RISC-V) as the main computer running `robot-os` kernel, paired with the brain
server (`robot-brain`) on a host over WiFi.

See [`SHOPPING_LIST.md`](./SHOPPING_LIST.md) for the full BOM.

---

## 1. Pre-build checklist

Before unboxing anything:

- [ ] All BOM items present
- [ ] VisionFive 2 U-Boot flashed to SPI flash (see board manual)
- [ ] SD card ≥16 GB, FAT32-formatted
- [ ] Host machine with macOS / Linux + `cargo` toolchain
- [ ] ESP32 flashed with `uart_bridge` firmware (see `docs/ESP32_BRIDGE.md`)
- [ ] Multimeter available
- [ ] Soldering iron + solder + heat-shrink tubing

---

## 2. Assembly order

Build in this order so each step is testable before committing to the next.

### 2.1 Chassis (stand-alone)

1. Assemble the 4WD chassis per kit instructions.
2. Mount the 4 gear motors to the lower plate. Verify each spins freely by hand.
3. Snap the speed encoder discs onto the motor shafts (if the kit provides them).
4. Route encoder wires through the plate to the upper deck.

### 2.2 Power stage (bench test first)

> **Never connect the VisionFive 2 before verifying the 5 V rail is stable.**

1. Wire the 18650 2S3P pack (2 cells series, 3 packs parallel → 7.4 V nominal, 3.6 Ah).
2. Connect the 2S pack to the DD05CVSA buck converter input.
3. Adjust the trimpot so the output reads `5.00 V ± 0.05 V` under a 500 mA load (use a dummy load or the motor driver with motors attached).
4. Confirm voltage doesn't sag below 4.9 V when motors stall.
5. **Add a fuse** (inline, 3 A automotive blade) between the battery + terminal and the buck converter.

### 2.3 Motor driver (TB6612FNG)

1. Mount the TB6612FNG on the upper deck.
2. Connect motor pairs: `M_LEFT_FRONT + M_LEFT_REAR → A1/A2`, `M_RIGHT_FRONT + M_RIGHT_REAR → B1/B2`.
3. Wire `VM ← 7.4 V battery`, `VCC ← 5 V rail`, `GND ← common ground`.
4. Do **not** connect PWM/direction pins to VF2 yet — bench-test the board first by manually driving AIN1/AIN2/BIN1/BIN2/PWMA/PWMB with a 3.3 V source through a push-button.
5. Confirm motors spin forward/reverse with expected polarity. Swap wires if any motor is reversed.

### 2.4 VisionFive 2 integration

1. Mount VF2 on the upper deck with M3 standoffs.
2. Connect the DD05CVSA 5 V output to the VF2 barrel jack (or GPIO 2/4 power rails — check board revision).
3. Do **not** power on yet.
4. Wire GPIO per the pinout below (see [SHOPPING_LIST.md § Wiring Diagram](./SHOPPING_LIST.md#wiring-diagram)).
5. Double-check with a multimeter that:
   - 5 V rail is stable.
   - No shorts between GPIO and VCC.
   - HC-SR04 ECHO has a voltage divider (2 kΩ + 1 kΩ) to step 5 V → 3.3 V.

### 2.5 I²C devices

1. Mount the MPU6050 IMU near the centre of mass, Z axis pointing up.
2. Mount the ADS1115 next to the battery for monitoring.
3. Mount the INA219 **in series** with the battery + terminal (main power path through its shunt).
4. Wire `SDA/SCL` from VF2 I²C bus 1 to all three devices in parallel.
5. Add 4.7 kΩ pull-ups on SDA and SCL to 3.3 V (often already present on breakout boards — check with multimeter, only add if missing).

### 2.6 CSI camera

1. Align the 15-pin FPC cable: blue tab faces the black clip on the VF2, silver contacts face the PCB.
2. Route the cable so it can't catch on moving wheels.
3. Mount the RPi camera on a pan/tilt bracket (future) or a fixed angle (~15° down).

### 2.7 UART links

- **UART0** → LD19 LiDAR (when installed). Cross TX↔RX. LiDAR has its own 5 V input; share GND with VF2.
- **UART1** → ESP32 bridge. TX↔RX, GND common, ESP32 powered from 5 V.

### 2.8 Sensor peripherals

Mount these last so wires are already routed:

| Sensor | Location | Notes |
|--------|----------|-------|
| HC-SR04 | Front bumper, aimed 5° down | Detects low obstacles (glass, curbs) |
| PIR | Top deck, tilted 10° down | 120° field of view |
| IR sensor | Front, flush with chassis | Line following / dock homing |
| Sound | Top deck, near camera | Event detection |
| Status LEDs (×3) | Visible from outside | Red/Yellow/Green stacked |
| Buzzer | Inside chassis | Muffled audio feedback |

---

## 3. First power-on checklist

Before inserting the SD card:

- [ ] All GPIO wires routed correctly per wiring diagram
- [ ] 5 V rail measured 5.0 V ± 0.05 V at VF2 input
- [ ] Battery voltage ≥ 7.0 V (not deep-discharged)
- [ ] No LEDs shorting to VCC (use multimeter continuity mode)
- [ ] Motors disconnected (for safety during first boot)
- [ ] Fuse installed in battery + line

### First-boot sequence

1. **SD card only, no battery**: power VF2 via USB-C (developer convenience).
2. Connect UART-to-USB adapter to VF2 console pins.
3. Open a serial terminal at 115200 baud.
4. Insert SD card, power on.
5. Watch for `[KERNEL] robot-os ready` banner.
6. At the shell prompt, run `status` and `sensors`. Verify IMU reads plausible values.
7. Power down. **Now** connect battery.
8. Full power-on with motors still disconnected. Retest `sensors`.
9. Connect motor driver, run `motor test` (safe test — low PWM, short duration).
10. If all OK, put wheels on and do a tethered test drive.

---

## 4. Mounting and weight distribution

- Battery pack: **low and centred** (lowest plate, over the wheelbase centre).
- VF2: upper plate, centred.
- Camera: front upper, as high as practical for field of view.
- LiDAR (future): top mast, ≥10 cm above all obstructions.

Target centre of mass: < 8 cm from the floor to avoid tipping.

---

## 5. Cable management

- Power wires (battery, 5 V rail): **thick, separate bundle**, twisted pair to reduce EMI.
- Signal wires (GPIO, I²C, UART): **separate bundle** from power.
- Run motor wires **away** from signal lines.
- Use spiral wrap for the main cable bundles; heat-shrink all battery terminations.

---

## 6. Troubleshooting

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| VF2 won't boot | SD card not FAT32 or SPL missing | Reflash U-Boot, re-format SD |
| VF2 reboots under motor load | Voltage sag on 5 V rail | Add 1000 µF bulk cap at VF2 input; verify buck can supply 2 A |
| Motors run reverse | Wire polarity | Swap A1/A2 (or B1/B2) at TB6612 |
| No I²C devices found | Missing pull-ups or wrong bus | Check SDA/SCL with scope; add 4.7 kΩ pull-ups |
| `sensors` shows bogus IMU | MPU6050 loose / not calibrated | Reseat; run `imu calibrate` from shell |
| HC-SR04 always reads max | ECHO not level-shifted | Verify voltage divider (should read 3.3 V max on ECHO pin) |
| VF2 serial noise | Ground loop between VF2 and ESP32 | Share GND at ONE point only |

---

## 7. First autonomous test (bench)

Before putting the robot on the floor, do a bench test with wheels off the ground:

1. Boot the robot.
2. Connect the brain server on the host (`python3 server.py`).
3. Flash the robot's WiFi credentials into ESP32 bridge config.
4. Watch the brain log for `Robot connected`.
5. From the shell, run `skills list` to verify the skill catalogue.
6. Issue `mode idle` → `mode autonomous` transitions.
7. Verify motors do **not** drive unexpectedly (L0 safety layer should be holding them).
8. Issue a test waypoint `0,500` (500 mm forward). Motors should spin briefly.
9. Verify `safety` layer reports no violations.

Only after all of the above passes should the robot be put on the floor.

---

## 8. Deployment readiness gate

Before the first real-world test:

- [ ] All bench tests pass
- [ ] Watchdog (`F11`) verified by deliberately hanging a task — robot must reboot.
- [ ] Battery failsafe (`E09`) verified by lowering the threshold temporarily.
- [ ] Geofence (`E03`) configured to a small safe area.
- [ ] Logging (`E06`) writes to SD card — check file appears in `/LOG/`.
- [ ] OTA path (`F17`) validated — flash an updated kernel via the brain server.
- [ ] Emergency stop button wired (physical, breaks motor power rail).

---

See also:
- [SHOPPING_LIST.md](./SHOPPING_LIST.md) — full BOM + wiring diagram
- [FLASH_PROCEDURE.md](../../robot-os/docs/FLASH_PROCEDURE.md) — kernel flash procedure (in robot-os repo)
- [DEPLOY.md](../../robot-os/docs/DEPLOY.md) — deploy / OTA workflow
