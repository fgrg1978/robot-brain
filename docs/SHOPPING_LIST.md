# Lista de Compra — Robot Terrestre v1

Robot autónomo con VisionFive 2 (RISC-V). Objetivo: plataforma mínima funcional
para desarrollar y probar el sistema (movimiento, odometría, visión, brain link).

---

## Chassis + Motores

| # | Componente | Especificaciones | Precio aprox | Link |
|---|---|---|---|---|
| 1 | Yahboom Mini Smart Car Chassis 4WD | Aluminio, 14.2cm ancho, muchos agujeros montaje | ~$35 | [Yahboom](https://category.yahboom.net/products/yahboom-mini-car-chassis) |

**Incluye:**
- 4× Motor 310 DC con encoder Hall cuadratura (señal A/B), reducción 1:20
- 4× Ruedas goma
- Chassis aluminio + tornillería completa

**Por qué este:** encoders Hall reales (no ópticos) → odometría precisa. Aluminio aguanta peso VF2 + batería. Agujeros estándar para montar todo.

---

## Electrónica

| # | Componente | Especificaciones | Precio aprox | Link |
|---|---|---|---|---|
| 2 | L298N Dual H-Bridge | 2 canales, hasta 2A/canal, 5-35V | ~$3 | Amazon/AliExpress "L298N motor driver" |
| 3 | Step-down DC-DC 5V/3A | LM2596 o MP1584, input 7-28V → 5V fijo | ~$3 | Amazon/AliExpress "LM2596 5V step down" |
| 4 | Cables dupont M-F y F-F | Pack 40 cables 20cm, ambos tipos | ~$3 | Amazon/AliExpress "dupont cable kit" |
| 5 | Conector XT60 macho + cable | Para conectar LiPo al sistema | ~$2 | Amazon/AliExpress "XT60 pigtail" |

**Nota sobre L298N:** Controla los 4 motores en pares (2 izquierda, 2 derecha) — configuración diferencial. Cada canal controla un lado. Si necesitas control individual de 4 motores, usa 2× L298N o 1× L298N + 1× TB6612FNG.

---

## Alimentación

| # | Componente | Especificaciones | Precio aprox | Link |
|---|---|---|---|---|
| 6 | LiPo 3S 5000mAh 25C | 11.1V, conector XT60, ~140×45×30mm | ~$30 | Amazon/AliExpress "3S 5000mAh 25C LiPo" |
| 7 | Cargador balance LiPo | B3 Pro (solo 2S/3S) o iMAX B6 (universal) | ~$12 | Amazon/AliExpress "B3 LiPo charger" |
| 8 | Alarma voltaje LiPo (opcional) | Avisa con buzzer cuando voltaje baja | ~$2 | Amazon/AliExpress "LiPo voltage alarm" |

**Autonomía estimada:**
- Consumo normal: ~1.6A (VF2 + cámara + motores marcha)
- Con 5000mAh: **~3 horas** de operación continua
- Con picos (stall motores): batería 25C soporta 125A — sin problema

**Distribución eléctrica:**
```
LiPo 3S 11.1V 5000mAh
    │
    ├── XT60 → Step-down DC-DC → 5V/3A → VisionFive 2 (USB-C) + USB camera
    │
    └── XT60 → L298N (VCC motor) → 4× motores 310
```

---

## Visión

| # | Componente | Especificaciones | Precio aprox | Link |
|---|---|---|---|---|
| 9 | USB Camera 720p/1080p | UVC compatible, ángulo amplio (>90°), con soporte | ~$12 | Amazon "USB camera wide angle 720p" |

**Requisitos:** Que sea UVC estándar (plug & play, sin driver propietario). Ángulo amplio preferible para navegación. No necesita ser alta resolución — 720p sobra para SmolVLM.

---

## Montaje

| # | Componente | Especificaciones | Precio aprox | Link |
|---|---|---|---|---|
| 10 | Separadores M3 nylon (pack) | Para montar VF2 sobre chassis, varios largos | ~$3 | Amazon/AliExpress "M3 nylon standoff kit" |
| 11 | Bridas plástico (zip ties) | Para sujetar cables y batería | ~$2 | Cualquier ferretería |

---

## Resumen

| Categoría | Subtotal |
|---|---|
| Chassis + motores + encoders | $35 |
| Electrónica (driver + step-down + cables) | $11 |
| Batería + cargador + alarma | $44 |
| Cámara USB | $12 |
| Montaje (separadores + bridas) | $5 |
| **TOTAL** | **~$107** |

---

## Ya tienes (no comprar)

- VisionFive 2 (RISC-V SBC)
- Cable USB-C (alimentación VF2)
- microSD con el kernel
- PC/Mac con LM Studio (brain server)

---

## Conexiones VF2 → Hardware

```
VF2 GPIO 40-pin header
│
├── GPIO OUT → L298N IN1, IN2 (motor izquierdo)
├── GPIO OUT → L298N IN3, IN4 (motor derecho)
├── GPIO PWM → L298N ENA, ENB (velocidad)
│
├── GPIO IN  ← Encoder A/B motor izq (interrupt)
├── GPIO IN  ← Encoder A/B motor der (interrupt)
│
├── USB port ← USB camera (UVC)
│
└── Ethernet/WiFi ←→ macOS brain server (TCP)
```

**Nota WiFi:** La VF2 no tiene WiFi integrado. Fase 1 = cable Ethernet. Fase 2 = USB WiFi dongle o ESP32 bridge (ver plan fases W/W-alt).

---

## Fases de montaje

1. **Montar chassis** — ensamblar motores + ruedas + chassis (viene con instrucciones)
2. **Montar VF2** — separadores M3 sobre el chassis, atornillar VF2
3. **Cableado motores** — motores → L298N → GPIO VF2
4. **Cableado encoders** — encoder A/B → GPIO VF2 (pines con interrupt)
5. **Alimentación** — LiPo → XT60 split → step-down 5V (VF2) + L298N (motores)
6. **Cámara** — USB camera al puerto USB de VF2
7. **Test** — boot kernel, probar motores via shell (`motor` cmd), leer encoders (`odom`), capturar cámara

---

## Opcional (futuro)

| Componente | Para qué | Precio |
|---|---|---|
| HC-SR04 ultrasonido ×2 | Rangefinder frontal + lateral | $3 |
| MPU-6050 breakout | IMU real (ya tienes driver) | $3 |
| ESP32-C3 devkit | WiFi bridge (fase W-alt) | $5 |
| USB WiFi dongle RTL8188 | WiFi directo (fase W) | $8 |
| GPS NEO-6M | Navegación exterior | $10 |
| RPLIDAR A1 (usado) | SLAM / mapeo | $50-80 |
