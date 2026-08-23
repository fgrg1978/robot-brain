# RFC-SHARDING — Horizontal Brain Sharding via Consistent Hashing

**Status**: Draft  
**Component**: phanes-brain (S3 — Sharding Control Plane)  
**Author**: PHANES contributors  
**Date**: 2026-05-24  

---

## 1. Problem Statement

A single `FleetManager` instance holds all robot state in memory and processes
every incoming connection.  At the 1000-robot scale a single brain node becomes
a bottleneck: CPU-bound perception tasks, heartbeat processing, and REST queries
all compete on one event loop.

The goal of S3 is to let N brain data-plane nodes share the load by each
owning a disjoint subset of robot_ids, without a centralised router.

---

## 2. Two-Tier Architecture

```
                    ┌─────────────────────────────────┐
                    │       Control Plane (S3)         │
                    │  ConsistentHashRing  (stateless) │
                    │  ShardCoordinator    (per node)  │
                    └──────────────┬──────────────────┘
                                   │  is_local / forward_target
              ┌────────────────────┼────────────────────┐
              ▼                    ▼                    ▼
       brain-0:5000         brain-1:5000         brain-2:5000
       FleetManager         FleetManager         FleetManager
       (data plane)         (data plane)         (data plane)
       robots 0-332         robots 333-665       robots 666-999
```

Each brain node:
1. Constructs a `ConsistentHashRing` seeded with the same node list.
2. Wraps it in a `ShardCoordinator(ring, self_node="brain-N:port")`.
3. Passes the coordinator to `FleetManager(shard=coord)`.
4. On every incoming `register()` call, the manager routes local vs. foreign.

Node membership is currently distributed via static config (e.g. `brain.toml`
`[sharding]` section).  Dynamic membership (see §6) is future work.

---

## 3. Why Consistent Hashing

| Approach | Churn on scale event | State lookup | Hot-key risk |
|---|---|---|---|
| Modulo (robot_id % N) | ALL keys remapped | None | Low |
| Random + state DB | 0 keys move | DB round-trip | Low |
| **Consistent hash** | ~1/N keys move | None | Low |

Consistent hashing moves only ~1/N of keys when a node is added or removed,
making rolling deployments and node failures cheap.  No external state lookup
is required on the hot path — `route()` is a pure in-process hash lookup.

The ring uses SHA-256 as the key hash function: deterministic, uniform, and
available in stdlib without dependencies.  128 virtual nodes per physical node
produces ±5% load variance across nodes for any realistic fleet size.

---

## 4. How Redis Would Slot In

The `BackendProtocol` (structural typing.Protocol) decouples the ring logic
from the storage backend:

```python
class BackendProtocol(Protocol):
    async def register_robot(self, robot_id: str, meta: dict) -> None: ...
    async def update_heartbeat(self, robot_id: str, ts: float) -> None: ...
    async def query_robot(self, robot_id: str) -> dict | None: ...
    async def list_robots(self, filter_fn=None) -> list[dict]: ...
```

Today: `InMemoryBackend` — asyncio.Lock-protected dict, single process.

To add Redis:

```python
# future: fleet_sharding_redis.py
import aioredis

class RedisBackend:                          # satisfies BackendProtocol
    def __init__(self, url: str) -> None:
        self._r = aioredis.from_url(url)

    async def register_robot(self, robot_id: str, meta: dict) -> None:
        await self._r.hset(f"robot:{robot_id}", mapping=meta)

    async def update_heartbeat(self, robot_id: str, ts: float) -> None:
        await self._r.hset(f"robot:{robot_id}", "last_heartbeat", ts)

    async def query_robot(self, robot_id: str) -> dict | None:
        return await self._r.hgetall(f"robot:{robot_id}") or None

    async def list_robots(self, filter_fn=None) -> list[dict]:
        ...
```

The `FleetManager` and `ShardCoordinator` are unchanged — only the
`InMemoryBackend` instance is swapped for `RedisBackend`.

---

## 5. Redistribution Cost Helper

`ConsistentHashRing.redistribution_cost(old_ring, new_ring, keys)` accepts a
key universe and returns a `dict[robot_id, 1]` for every robot that would move.

Example capacity planning:

```python
keys = fleet_manager.all_robot_ids()   # current fleet
ring3 = ConsistentHashRing(["brain-0", "brain-1", "brain-2"])
ring4 = ConsistentHashRing(["brain-0", "brain-1", "brain-2", "brain-3"])
moved = ConsistentHashRing.redistribution_cost(ring3, ring4, keys)
print(f"Scaling 3→4 nodes moves {len(moved)}/{len(keys)} robots")
# Expected: ~25% of fleet
```

The `keys` parameter is explicit (not implicitly derived from the ring) because
the ring is stateless — it does not store which keys are currently active.

---

## 6. Open Questions

### Q1: Rebalancing during scale events

When a new node joins, robots routed to it need to have their live TCP
connections migrated or re-established.  Current proposal: robots reconnect on
the next heartbeat timeout (30 s); the brain that receives the re-connect
rejects with an HTTP-style "301 Moved" sentinel packet containing the new
node's address.

Exact packet format TBD — likely a `PKT_REDIRECT (0x88)` brain-to-robot
packet carrying the target host:port as ASCII.

### Q2: Hot-key mitigation

A single high-frequency robot (e.g. a camera drone with 30 fps VLM calls)
can saturate one node.  Mitigation options:
- Robot-level throttling already in `safety.py` (sensor rate cap).
- Explicit pinning override (see Q3).
- Per-robot virtual-node weight (future `add_node(node, weight=N)` API).

### Q3: Robot pinning override

Some robots may need to be pinned to a specific node regardless of the hash
(e.g. hardware in the same rack for latency reasons).  Proposal: an optional
`pin_map: dict[str, str]` in `ShardCoordinator` that short-circuits `route()`
for listed robot_ids.

### Q4: Dynamic membership discovery

Currently node list is static config.  A gossip protocol or a lightweight
etcd/consul watch would allow nodes to join/leave without restarting peers.
Out of scope for S3; tracked as S4.

### Q5: Split-brain during node failure

If a node fails and the ring is not updated, its robots become unreachable.
A health-check + ring update mechanism is needed.  Tracked as S4.
