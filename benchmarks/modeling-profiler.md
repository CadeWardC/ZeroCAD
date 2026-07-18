# Release-mode modeling profiler

Run the Foundation 1 modeling gate on the reference Lenovo 82XU while connected
to AC power, with Windows power mode set to **Best performance** and unrelated
workloads closed:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File benchmarks/profile-modeling.ps1 -ConfirmBestPerformanceAndIdle
```

The runner writes `target/modeling-profile.json`. It performs five unrecorded
warmups and 31 recorded samples for each workload, then reports nearest-rank p50
and p95 (`sorted[ceil(p * n) - 1]`). Both percentiles must remain below their
budget:

| Workload | Corpus | Budget |
|---|---|---:|
| Trailing edit | Toggle and commit the last feature of the dependent 100-feature corpus, then rebuild and tessellate | < 50 ms |
| Representative rebuild | Evaluate and tessellate a newly constructed, cache-free dependent 500-feature corpus | < 250 ms |

The reference is Lenovo 82XU, Ryzen 7 7840HS, 16 GiB, Windows 11, Rust 1.94.
`profile-modeling.ps1` verifies AC power and the effective **Best performance**
overlay, and requires explicit confirmation that unrelated workloads are closed.
`-SkipEnvironmentChecks` exists only for compilation and harness smoke checks on
non-reference CI hosts; its output is explicitly marked `smoke_only` and is not
release evidence.

The workspace and nested OpenRCAD release profiles deliberately retain
`panic = "unwind"`; profiler work must not weaken guarded Boolean recovery.
