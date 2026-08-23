# RFC-CONTROL-DATA-SPLIT — Control-Plane / Data-Plane Split for PHANES Brain

**Status**: Draft  
**Author**: PHANES brain team  
**Date**: 2026-05-24  
**Scope**: `robot-brain` / `phanes-brain` (Python layer only; no kernel changes)

---

## 1. Motivation

The monolithic `python -m server` process handles everything: robot TCP
connections, perception (VLM calls), planning (LLM calls), policy execution,
fleet registry, OTA coordination, and the operator REST API — all in one
asyncio event loop.

This works for a single robot but has two structural problems at fleet scale:

1. **Coupling of latency domains.** A slow VLM call on robot A blocks policy
   updates for robot B running in the same event loop.
2. **No independent scaling.** Control-path concerns (auth, discovery, OTA
   orchestration) must scale in lockstep with data-path concerns (perception,
   planning, policy) even though their resource profiles are very different.

---

## 2. Architecture

```
Operators / dashboards
        │  HTTP
        ▼
┌────────────────────────┐
│    Control Plane       │   stateless; scales independently
│                        │
│  POST /v1/robots/*/route  ← returns owning DP address
│  GET  /v1/fleet           ← fleet-wide read
│  POST /v1/ota             ← fan-out to data planes
│                        │
│  BearerAuth            │
│  ShardCoordinator      │   consistent-hash ring
│  OtaCoordinator        │   fan-out + job tracking
└──────────┬─────────────┘
           │  HTTP (ownership queries + OTA dispatch)
    ┌──────┴──────┐
    │             │
    ▼             ▼
┌──────────┐  ┌──────────┐     ...  N data-plane workers
│ DP-0     │  │ DP-1     │
│ port 9100│  │ port 9101│
│          │  │          │
│ owns:    │  │ owns:    │
│ bot_A    │  │ bot_B    │
│ bot_C    │  │ bot_D    │
│          │  │          │
│ perception│ │ perception│
│ planner  │  │ planner  │
│ policy   │  │ policy   │
└──────────┘  └──────────┘
     │  TCP          │  TCP
     ▼               ▼
 Robot kernels   Robot kernels
```

**Single-process default** (``--data-planes 0``): the control plane and one
data-plane worker run in the same asyncio event loop.  The `ShardCoordinator`
is shared in-memory; no HTTP round-trips occur.  `python -m server` is
unaffected and continues to work unchanged.

---

## 3. Where State Lives

| State kind            | Today (In-Memory)       | Future (Redis)              |
|-----------------------|-------------------------|-----------------------------|
| Robot registry        | `FleetManager` dict     | Redis hash `fleet:{id}`     |
| Heartbeat timestamps  | `FleetManager` dict     | Redis sorted set            |
| OTA job status        | `OtaCoordinator` dict   | Redis hash `ota_job:{id}`   |
| Shard ring membership | `ShardCoordinator` list | Redis sorted set (gossip)   |

In single-process mode (`--data-planes 0`) the in-memory backend is correct
because there is exactly one Python process.  In multi-process mode each
process has its own heap; state is NOT shared — the `start_split.py` launcher
refuses to start multi-process mode unless `--allow-no-shared-state` is
explicitly passed.

**TODO**: Implement a `RedisBackend` that satisfies the same interface as
`InMemoryBackend` and wire it into `FleetManager`, `OtaCoordinator`, and
`ShardCoordinator`.  The `start_split.py` guard should then be converted to
`--redis-url` requirement instead of a blanket refusal.

---

## 4. Failure Modes

### 4.1 Data-plane worker crashes

- The control plane's `ShardCoordinator` still holds the dead node in its
  ring.  Robots assigned to the dead node will receive no pipeline processing
  until the ring is updated.
- **Recovery path**: operator calls `POST /v1/ring/remove` (future endpoint)
  to remove the dead node; the ring redistributes.  Affected robots reconnect
  (kernel-side reconnect loop, configurable via `CONFIG_TCP_RECONNECT_MS`) and
  the new owner data plane accepts them.
- **Eventual consistency**: there is no split-brain here because only one data
  plane can win ownership for a given `robot_id` at any instant (the ring is
  deterministic).

### 4.2 Control plane crashes

- Data planes continue to serve their currently-connected robots using cached
  ownership knowledge.  New robot connections cannot be validated without the
  control plane; `OwnershipChecker` fails closed (returns `False`) after
  `OWNERSHIP_QUERY_TIMEOUT_S`.
- **Recovery path**: restart the control plane.  It rebuilds ring state from
  the registered data-plane address list (static config or future gossip).

### 4.3 Network partition between control plane and data plane

- Same as §4.2 from the data plane's perspective.  The control plane may
  mistakenly believe data planes are alive; OTA fan-out will time out and
  mark affected data planes as `FAILED` in the job record.

---

## 5. Open Questions

1. **Hot-key mitigation**: A robot that generates very high sensor rates
   (e.g. a high-FPS camera) will saturate its owning data-plane worker
   regardless of shard count.  Options: per-robot rate limiting at the data
   plane; dedicated high-bandwidth shard for camera-heavy robots; async
   perception queue with back-pressure.

2. **Control-plane HA**: a single control-plane process is a SPOF.  Options:
   active/standby with shared Redis; Raft-based ring membership (overkill for
   hobbyist deployment); simple restart-on-crash via systemd.

3. **Deployment topology**: for the initial hobbyist use case (one macOS
   laptop + 1–4 physical robots arriving in July 2026) `--data-planes 0`
   (single-process) is the correct default.  The multi-process path is
   scaffolded now so the architecture is proven before it is needed at scale.

4. **Metrics wiring**: once the S5 metrics agent lands, each data plane should
   push its `phanes_brain_*` metrics to a Prometheus push-gateway or expose
   them on a per-worker `/metrics` endpoint scraped by the control plane.

5. **Security**: the internal `/internal/ota` endpoint on data planes is
   currently unauthenticated.  It should require a shared internal token
   (different from the operator-facing `ROBOT_BRAIN_CP_API_KEY`) so that a
   compromised robot kernel cannot trigger OTA pushes by impersonating the
   control plane.
