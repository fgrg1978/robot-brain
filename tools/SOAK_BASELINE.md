# Fleet Soak Baseline — 2026-05-24

## N=100 run (fleet_brain_stub loopback, 20 s)

```
[SOAK] N=100 duration=20.0s tx=876pkt/s rx=871pkt/s p99=10.0ms drops=0 disconnects=0 conn_err=0
```

Per-interval CSV excerpt (5-second windows):

| t(s) | active | tx_pkt | rx_pkt | p50_ms | p95_ms | p99_ms | drops |
|------|--------|--------|--------|--------|--------|--------|-------|
|  5.0 |  99    |  2 599 |  2 500 |    1.0 |    4.0 |   11.0 |     0 |
| 10.0 | 100    |  7 601 |  7 500 |    1.0 |    6.0 |   12.0 |     0 |
| 15.0 | 100    | 12 605 | 12 504 |    1.0 |    7.0 |   11.0 |     0 |
| 20.0 | 100    | 17 600 | 17 500 |    1.0 |    6.0 |   10.0 |     0 |

## Saturation sweep result

Sweep: N = 10 → 20 → 40 → 80 → 160 → 320 → 640 → 1280 (doubles every 30 s)

| N    | tx pkt/s | p99_ms | drops |
|------|----------|--------|-------|
|   10 |       91 |   10.0 |     0 |
|   20 |      182 |   10.0 |     0 |
|   40 |      366 |    7.0 |     0 |
|   80 |      732 |    5.0 |     0 |
|  160 |    1 466 |    2.0 |     0 |
|  320 |    2 924 |    1.0 |     0 |
|  640 |    5 857 |    1.0 |     0 |
| 1280 |   11 695 |  152.0 |     0 |

**Inflection N = 1280** — p99 jumps from 1 ms (N=640) to 152 ms (N=1280).
Formal breakdown threshold (p99 > 500 ms) not reached through N=2000.

## Methodology

- Harness: `fleet_brain_stub.py` (single asyncio event loop, 1:1 ack per SensorPacket)
- Load generator: `fleet_soak.py --sensor-hz 10 --ramp-s 5`
- Host: macOS Darwin 25.5.0, loopback 127.0.0.1
- "Drop" = SensorPacket sent but no ActuatorCmd ack within 2 s (none observed)
