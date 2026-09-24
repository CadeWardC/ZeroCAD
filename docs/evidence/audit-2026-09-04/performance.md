# Local mechanical-part timings

2026-09-04, optimized build, two Rayon workers. Ten samples per workload, one
second warmup and two seconds measurement. Values below are Criterion median
estimates in milliseconds. This is a single-machine reference, not a qualified
regression threshold or a pre-change speedup measurement. See `environment.json`
and `performance.json` for machine, source fingerprint, estimates, and raw samples.

| Part | Cold evaluation | Cached evaluation | Trailing move edit |
|---|---:|---:|---:|
| curved_step_import | 8.0526 | 0.0203 | 0.2081 |
| engraved_plate | 277.3723 | 4.7850 | 138.9882 |
| filleted_block | 4.6898 | 0.0195 | 0.2857 |
| open_enclosure | 5.9765 | 0.0290 | 0.4874 |
| patterned_plate | 69.4079 | 0.0648 | 2.0096 |

The workloads verify different modeling paths; times should not be compared as
equivalent complexity. Export, startup, peak memory, assembly performance, and
large upstream edits are outside this timing capture.
