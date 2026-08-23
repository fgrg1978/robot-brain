# bench/ — End-to-End Benchmark Results

This directory stores structured JSON benchmark results produced by
`scripts/bench_e2e.sh` (in the `robot-os` repo) + `tools/bench_e2e_collect.py`.

## Directory layout

```
bench/
  README.md              this file
  results/
    .gitkeep             keeps the directory tracked in git
    <sha12>.json         one file per kernel git SHA (first 12 chars)
```

## Running the harness

From the `robot-os` repo root:

```sh
scripts/bench_e2e.sh            # default: 3 runs × 30s each
N_RUNS=1 SCENARIO_DURATION_S=15 scripts/bench_e2e.sh   # faster iteration
SKIP_BUILD=1 scripts/bench_e2e.sh   # skip rebuild, reuse existing ELF
```

The script writes `bench/results/<sha>.json` (in this repo) and prints a
one-line summary to stdout:

```
[BENCH] rtt_p99=nullms boot=4823ms tx=1.2pkt/s footprint=2559KiB
```

## JSON schema

Top-level keys are **stable** so a dashboard can diff across SHAs.

| Key | Description |
|-----|-------------|
| `meta` | Build metadata: sha, qemu_version, host, timestamp, n_runs |
| `rtt_ms` | TCP RTT distribution: p50/p95/p99/stddev/n_samples (ms) |
| `throughput` | steady_msgs_per_s + burst_peak_msgs_per_s |
| `boot_ms` | QEMU launch → first `[NET] Stack ready` (ms) |
| `wcet_us` | Per-point WCET: min/max/avg/p99/violations (µs) |
| `jitter_ns` | Timer ISR jitter: min/max (ns) |
| `footprint` | text/rodata/data/bss/total (bytes) |

All numeric fields may be `null` when a metric couldn't be captured in
that run.  Consumers **must** tolerate nulls.

## Comparing two SHAs

```python
import json, sys

with open(f"bench/results/{sys.argv[1]}.json") as f:
    a = json.load(f)
with open(f"bench/results/{sys.argv[2]}.json") as f:
    b = json.load(f)

for key in ("boot_ms", "wcet_us", "footprint"):
    print(f"{key}: {a.get(key)} → {b.get(key)}")
```

## QEMU TCG noise caveats

- `rdcycle` under `-smp 4` TCG counts host wall-time, not per-hart work-time.
  This inflates WCET numbers significantly (see `crates/drivers/src/wcet.rs`
  comments).  WCET values from QEMU are directionally useful but not cycle-
  accurate.  Real numbers come from VF2/K1 hardware.
- RTT is expected `null` under QEMU TCG because the kernel sensor pump
  (`task_block(Timer(+100ms))`) can stall for seconds under TCG-SMP (issue #39).
- Boot time (`boot_ms`) is measured from QEMU process launch to the kernel's
  `[NET] Stack ready` log line; it includes OpenSBI startup (~300ms) and is
  stable to ±50ms across runs.
- The harness runs N_RUNS=3 and takes medians to dampen single-run noise.
  The goal is to detect **>= 10% regressions**, not sub-percent differences.
- `wcet_us.timer_isr` is populated only on the **cold-JIT first boot** after a
  fresh `make build`.  The `probe!` macro emits `[ISR-WCET]` only when ISR
  subcomponents exceed 1 ms; once TCG translation is warm (run 2+), all
  subcomponents finish in <1 ms and no lines appear.  If the harness is invoked
  with `SKIP_BUILD=1` against an already-warm binary, `wcet_us` will be `{}`.
  The CI gate only requires `wcet_us.timer_isr.max` not null, so it must run
  with `SKIP_BUILD=0` (default).

## CI gating (Phase A6)

### What `bench/baselines.json` is

`bench/baselines.json` is the single committed baseline that every PR is
compared against.  It has the same schema as `bench/results/<sha>.json` plus
a top-level `_meta` block:

```json
"_meta": {
  "baseline_sha": "<12-char SHA of the run that produced these numbers>",
  "baseline_date": "<ISO date>",
  "harness_version": "1.0"
}
```

The comparison logic lives in `tools/bench_compare.py`.  For each leaf
numeric metric it computes the percentage change vs the baseline and applies
a **5% regression threshold**.  Direction is metric-specific: latency/size
metrics are good-when-smaller; throughput metrics are good-when-larger.

### How to update the baseline

1. Run the harness locally or let CI produce a `bench/results/<sha>.json`.
2. Copy it over `bench/baselines.json` and update the `_meta` block with the
   new SHA and date.
3. Open a PR.  The commit message body **must** explain why the new numbers
   are the new expected baseline (e.g. "footprint grows because of RFC-0027
   adding the AEAD-link crate").

### How to waive a metric for a one-off PR

If a single PR intentionally regresses one or more metrics (e.g. a planned
size increase for a new feature), add this line anywhere in the commit
message body:

```
BENCH-WAIVER: <comma-separated dot-path list>
```

Example:

```
feat: add AEAD link crate (~50 KiB footprint increase)

BENCH-WAIVER: footprint.total_bytes,footprint.bss_bytes
```

The CI step extracts the waiver line with `grep -o 'BENCH-WAIVER:.*'` and
passes it to `bench_compare.py --waiver-text`.  Waived regressions are still
displayed in the PR comment (yellow section) so reviewers can see them, but
they do not fail the build.

### How to read the PR comment report

The `bench-e2e` CI job posts (or updates) a comment on every PR with the
marker `<!-- BENCH-REPORT -->`.  The report has these sections:

| Section | Emoji | Meaning |
|---------|-------|---------|
| Regressions (unwaived) | ❌ | Metric moved in the bad direction by ≥ 5%.  Blocks CI once `continue-on-error` is removed. |
| Regressions (waived) | ⚠️ | Same regression but covered by `BENCH-WAIVER:` in the commit body. |
| Improvements | ✅ | Metric improved by ≥ 5% — informational only. |
| New metrics | ℹ️ | Metric present in the result but not in the baseline. |
| Missing metrics | ⚠️ | Metric present in the baseline but absent from this run (possible harness regression). |
| Stable metrics | (collapsed) | All other metrics within the ±5% band. |

### Noise floor and hard-gate promotion

The `bench-e2e` job currently runs with `continue-on-error: true`.  GitHub
Actions shared runners have higher timing variance than dedicated hardware, so
the first ~10 PRs are used to establish a stable noise floor.  Once results
converge (no spurious regressions across 10 PRs), remove `continue-on-error`
(or flip it to `false`) to promote the gate to a hard CI failure.
