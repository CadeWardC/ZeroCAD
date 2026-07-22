# Qualified Wave 5 modeling comparison

Recorded on 2026-07-21/22 on the reference Lenovo 82XU, on AC power with the
Windows Best performance overlay. Every profiler invocation passed the runner's
environment checks and used five warmups plus 31 serial recorded samples.

- Immutable post-Wave-4B baseline source: `3994a35f7e33744ab0e10a4735fe1b61fb7ab1ab`.
- Wave 5 working-tree HEAD: `5613654876d4def7fac78e027eb262964df69a06`.
- The measured Wave 5 source was intentionally uncommitted. Its tracked source
  diff Git object hash was `9a262a08a04afa61c65915b4c6735467bec16fb5`.
- Rust: `rustc 1.94.0 (4a4ef493e 2026-03-02)`.

| Run | Trailing p50 | Trailing p95 | Rebuild p50 | Rebuild p95 |
|---|---:|---:|---:|---:|
| Post-4B baseline | 14.550 ms | 16.164 ms | 75.403 ms | 91.083 ms |
| First post-build run | 13.677 ms | 16.594 ms | 108.306 ms | 119.518 ms |
| Hot repeat | 15.180 ms | 17.956 ms | 79.550 ms | 99.451 ms |
| Hot confirmation | 15.326 ms | 18.533 ms | 77.710 ms | 95.482 ms |
| Absolute budget | < 50 ms | < 50 ms | < 250 ms | < 250 ms |

All runs pass both absolute performance budgets. The first post-build rebuild
run triggered the 10% investigation threshold, but did not reproduce after the
release executable and operating-system caches were warm: the two confirmation
runs put rebuild p50 at +5.5% and +3.1%, and p95 at +9.2% and +4.8%.

Trailing-edit p50 remained within +5.3%. Its two hot p95 values were 1.79 ms
and 2.37 ms above baseline (+11.1% and +14.7%), so the investigation threshold
was acknowledged rather than hidden. Alternating edit directions had nearly
identical means, and the 500-feature confirmation did not show a corresponding
regression. This is classified as small Windows tail-latency variance, not a
deterministic modeling-path regression. The absolute p95 remains below 19 ms,
with more than 31 ms of headroom against the 50 ms release budget.

Raw qualified evidence:

- `modeling-wave5-comparison.json`
- `modeling-wave5-comparison-rerun.json`
- `modeling-wave5-comparison-confirmation.json`

The post-Wave-4B `modeling-baseline.json` was not modified.
