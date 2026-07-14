# Phase 0 Performance and Footprint Baseline

Captured on 2026-07-14 after the Phase 0 geometry fixes. These values are the
comparison point for later architectural phases; they are measurements, not
hard-coded test thresholds.

## Reference machine

- OS: Microsoft Windows 11 Home 10.0.26200
- CPU: AMD Ryzen 7 7840HS with Radeon 780M Graphics
- RAM: 16,309,932,032 bytes
- Rust: `rustc 1.94.0 (4a4ef493e 2026-03-02)`
- Release profile: LTO, one codegen unit, `opt-level = 3`, stripped

## Application footprint

| Measurement | Phase 0 baseline |
|---|---:|
| Stripped `zerocad-gui.exe` | 21,829,120 bytes (20.82 MiB) |
| Cold process start to ready window | 1,022.4 ms |
| Idle working set, three seconds after ready | 130,306,048 bytes (124.27 MiB) |
| Peak startup working set | 154,341,376 bytes (147.19 MiB) |

The cold-window result is 22.4 ms above the eventual sub-one-second target. It
is retained as-is so later work is compared against the real starting point.

## Kernel and document workloads

The reproducible runner is:

```text
cargo run --release -p zerocad-core --example phase0_baseline
```

Standalone analytic-cylinder tessellation produced 192 triangles in 1.800 ms.

| Frozen corpus | Features | Bodies | Triangles | Cold rebuild | Warm rebuild | Compact save | Compact open | `.zcad` bytes | Hydrated bytes |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Small drilled part | 4 | 1 | 4,816 | 183.802 ms | 58.257 ms | 2.555 ms | 0.349 ms | 586 | 138,383 |
| Dependent history | 100 | 1 | 12 | 42.222 ms | 1.291 ms | 2.737 ms | 0.443 ms | 1,350 | 17,894 |
| Dependent history | 500 | 1 | 12 | 212.891 ms | 31.073 ms | 4.268 ms | 1.712 ms | 4,193 | 22,665 |
| Imported STEP box | 1 | 1 | 12 | 1.208 ms | 0.170 ms | 3.303 ms | 0.157 ms | 2,055 | 5,608 |
| Difficult through-hole | 4 | 1 | 6,928 | 219.346 ms | 70.467 ms | 2.356 ms | 0.101 ms | 608 | 180,385 |

## Frozen corpus contract

The corpus constructors live in
`zerocad-core/benches/support/phase0_corpus.rs`. Their names, defining
dimensions, and feature counts are pinned by
`zerocad-core/tests/phase0_corpora.rs`:

1. `small_part`
2. `dependent_100_feature`
3. `dependent_500_feature`
4. `imported_step_box`
5. `difficult_through_hole`

The Criterion benchmark in `zerocad-core/benches/modeling_pipeline.rs` consumes
the same constructors, preventing benchmark and regression workloads from
silently drifting apart.

## Measurement notes

- GUI startup was measured from process creation until Windows exposed a main
  window handle. The process was then left idle for three seconds before memory
  capture and terminated normally by the measurement harness.
- Rebuild timings include evaluator construction and display tessellation.
- Warm timings are immediate checkpoint-backed reevaluations of the same graph.
- Compact files contain the authoritative recipe without mesh/evaluator caches.
- Hydrated sizes include the current mesh and evaluation checkpoint cache.
