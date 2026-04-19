# Hardware BOM

## Stock (already owned)

### Core
- VisionFive 2 (JH7110, RISC-V SBC) — main computer
- Raspberry Pi Camera (CSI MIPI) — vision
- ESP32 (WiFi bridge UART↔TCP) — wireless link
- macOS with LM Studio — brain server (VLM + LLM)
- 4× RTSP IP cameras — fixed perimeter surveillance

### Motor Control
- TB6612FNG motor driver — dual H-bridge, 2 channels

### Sensors
- HC-SR04 ultrasonic rangefinder ×1 — low obstacle detection (complements LiDAR)
- ADS1115 16-bit ADC 4-channel — battery voltage, analog sensors
- PIR sensor — passive infrared motion detection
- IR sensor — infrared proximity / line following / dock homing
- Laser rangefinder — precise distance measurement
- Sound sensor / microphone — audio event detection (glass break, impact)
- Various Arduino sensor kits
- Colored LEDs (red, green, yellow/blue) — status indicators

### Power
- 18650 cells 1200mAh × 30 units — robot packs + dock spares
- 18650 holders (2S series = 7.4V)
- TP4056 charger modules — Li-ion charging (robot + dock)
- DD05CVSA — 5V boost/buck converter for VF2
- Recommended pack: 2S3P (6 cells, 7.4V 3.6Ah, ~2.5h autonomy per charge)

### Misc
- Resistors, capacitors, diodes — all values
- Dupont cables, breadboards, protoboards
- M3 nylon standoffs
- ESP8266 modules (spare WiFi options)
- Stepper motors ×2 (not used for wheeled robot, future projects)

## Build tiers

Three tiers depending on the target environment.  All three run the **same
kernel + brain** — only the hardware differs.

### Tier 1 — Indoor dev / bring-up (~17-19 EUR)

Minimum kit to validate the full software stack on a real chassis. Ideal as
the first build before committing to outdoor hardware.

| # | Component | Search on AliExpress | ~EUR |
|---|---|---|---|
| 1 | 4WD chassis with encoders | [4WD 2-layer chassis with speed encoder](https://es.aliexpress.com/item/1005007626442188.html) | 13 |
| 2 | IMU (accelerometer + gyro) | "GY-521 MPU6050 module" | 1-2 |
| 3 | Piezo buzzer (3.3V passive) | "passive buzzer module 3.3V" (or use from Arduino kit) | 1 |
| 4 | 18650 holder 2S3P | "18650 holder 2S3P" (or wire 3× 2S holders in parallel) | 2-3 |

**Tier 1 total: ~17-19 EUR**

> Indoor-only. Plastic chassis, small motors, no weather-proofing. Good for
> software validation and shelf demos.

---

### Tier 2 — Outdoor / tracked / autonomous (~310-410 EUR)

Upgrade path to match commercial tracked platforms (e.g. Waveshare UGV Beast)
mechanically, while keeping our software stack. The **differential-drive
protocol (2 channels: speed_l, speed_r) is unchanged** — tracked chassis is
transparent to the software layer.

**Buy this tier when moving the robot outdoors (garden, patio, parking).**

| # | Component | Why / search terms | ~EUR |
|---|---|---|---|
| 1 | Tracked chassis with aluminium frame + rubber tracks | Off-road, slopes, uneven terrain. Search: "tracked robot chassis aluminium tank" (Waveshare WAVE ROVER/UGV Rover, DFRobot Rover 5, SZDoit TS100 are known-good models) | 150-180 |
| 2 | LD19 2D LiDAR 360° | Real SLAM + outdoor navigation. Search: "LD19 LD-06 LiDAR 360" | 70-90 |
| 3 | LiPo 3S 5000 mAh + BMS + XT60 connector | 4-6 h autonomy vs 1-2 h of 18650 2S3P | 30-40 |
| 4 | IP54 ABS enclosure (printed or off-shelf) | Dust + light rain resistance | 30-50 |
| 5 | MPU9250 (9-axis with magnetometer) | Dead-reckoning without encoders on rough terrain | 5-10 |
| 6 | IR-capable night-vision camera + IR LED ring | 24/7 surveillance. Search: "Raspberry Pi NoIR camera + 940nm IR LED ring" | 25-40 |

**Tier 2 total: ~310-410 EUR**

> After installing Tier 2 HW, our stack is functionally equivalent to UGV Beast
> or similar commercial tracked robots — with the advantage that we control
> every layer of the software and our latency is measurably lower.

---

### Tier 3 — Advanced perception / deterrent / production (additional ~36-50 EUR)

Extras beyond tracked autonomy — useful for actual security/deterrent use.

| # | Component | Search on AliExpress | ~EUR |
|---|---|---|---|
| 1 | Micro servo SG90 ×2 | "SG90 micro servo 9g" | 2-3 |
| 2 | Pan-tilt bracket for SG90 | "SG90 pan tilt bracket kit" | 2-3 |
| 3 | Green laser module 5 mW | "laser module 532nm 5mW 3.3V" | 2-3 |
| 4 | Siren module 12 V | "active buzzer siren module 12V" | 3 |
| 5 | LED 10W COB white + MOSFET | "10W COB LED module" + "IRF520 MOSFET module" | 4 |
| 6 | PAM8403 amplifier + 3W speaker | "PAM8403 amplifier module" + "3W 4ohm speaker" | 5 |
| 7 | Pogo pins (spring-loaded) | "pogo pin spring loaded 2mm" ×4 | 2 |
| 8 | INA219 current/voltage sensor | "INA219 I2C current sensor module" | 1-2 |
| 9 | SX1262 LoRa module (E02) | Long-range WiFi fallback. Search: "SX1262 LoRa 868MHz module" | 10-15 |

**Tier 3 additional: ~36-50 EUR**

---

## Build recommendation

| Your goal | Buy tier(s) | Total |
|-----------|-------------|-------|
| Indoor demo / learning | **Tier 1** | 17-19 EUR |
| Garden / patio surveillance | **Tier 1 + Tier 2** | 330-430 EUR |
| Production / deterrent / harsh outdoor | **Tier 1 + Tier 2 + Tier 3** | 365-480 EUR |

> Start with Tier 1 to validate the complete software stack on physical
> hardware, then decide if the outdoor use-case warrants the Tier 2 upgrade.
> The software requires **no changes** between tiers.

## Wiring Diagram

```
18650 ×2 series (7.4V) ──┬── TB6612 VM (4 motors, pairs in parallel)
                         └── DD05CVSA 5V ──┬── VF2 5V power
                                           └── HC-SR04 VCC

VF2 GPIO (PWM):
  PWM0 ──── TB6612 PWMA (left motor pair)
  PWM1 ──── TB6612 PWMB (right motor pair)
  PWM2 ──── Buzzer (audio feedback)

VF2 GPIO (digital output):
  GPIO A ── TB6612 AIN1, AIN2 (left direction)
  GPIO B ── TB6612 BIN1, BIN2 (right direction)
  GPIO C ── LED green (status: monitoring)
  GPIO D ── LED yellow (status: possible detection)
  GPIO E ── LED red (status: confirmed detection)

VF2 GPIO (digital input):
  GPIO F ── HC-SR04 TRIG
  GPIO G ── HC-SR04 ECHO (voltage divider 5V→3.3V: 2kΩ + 1kΩ)
  GPIO H ── PIR sensor OUT (3.3V digital)
  GPIO I ── IR sensor OUT (3.3V digital)
  GPIO J ── Sound sensor DO (digital out)

VF2 I2C bus 1:
  SDA/SCL ── MPU6050 (IMU, addr 0x68)
  SDA/SCL ── ADS1115 (ADC, addr 0x48) ← battery voltage, analog sensors
  SDA/SCL ── INA219 (current/voltage, addr 0x40) ← in series with battery

VF2 CSI:
  15-pin flat cable ── RPi Camera

VF2 UART0:
  TX/RX ── LD19 LiDAR (115200 baud, 3.3V)

VF2 UART1:
  TX/RX ── ESP32 (WiFi bridge → macOS brain server)

Encoder signals (from chassis kit):
  ENC_L ── VF2 GPIO (interrupt-driven tick counting)
  ENC_R ── VF2 GPIO (interrupt-driven tick counting)
```

## Sensor Integration Map

| Sensor | Interface | Kernel Driver | Brain Use |
|---|---|---|---|
| LD19 LiDAR 2D | UART | lidar.rs | SLAM mapping, localization, path planning |
| RPi Camera | CSI MIPI | csi.rs | VLM perception (SmolVLM) |
| MPU6050 | I2C | imu (external crate) | Tilt safety, heading |
| HC-SR04 | GPIO | rangefinder.rs | Low obstacles below LiDAR plane, glass detection |
| ADS1115 | I2C | ads1115.rs | Battery voltage, analog inputs |
| PIR | GPIO (digital) | gpio.rs | Fast motion trigger → wake VLM |
| IR | GPIO (digital) | gpio.rs | Proximity, dock homing beacon |
| Sound sensor | GPIO (digital) | gpio.rs | Glass break, impact detection |
| Laser | GPIO/UART | rangefinder.rs | Precise distance |
| Encoders | GPIO (interrupt) | encoder (robot crate) | Odometry, PID feedback |
| Buzzer | PWM | buzzer.rs | Audio alerts, deterrent siren |
| Status LEDs | GPIO ×3 | status_led.rs | Green/yellow/red state indicator |
| INA219 | I2C | ina219.rs (future) | Current sensing, mAh counting, voltage sag, capacity % |
| RTSP cameras ×4 | Network (WiFi) | N/A (brain only) | Fixed surveillance, motion detect → dispatch robot |
