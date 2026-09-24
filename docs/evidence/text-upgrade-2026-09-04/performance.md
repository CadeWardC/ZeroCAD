# Local optimized performance comparison

Ten samples; same frozen parts, machine, two Rayon workers, one-second warmup,
two-second requested measurement. Criterion extends slow cases as needed.
Numbers are median estimates in milliseconds. Earlier and current captures are
separate sessions, so small changes may reflect machine conditions. Cached
workloads also changed despite no intended optimization to their path. This is
not a controlled regression threshold or a viewport/typing-latency measurement.

| Workload | Before (ms) | After (ms) | Before / after |
|---|---:|---:|---:|
| curved_step_import/cached | 0.0203 | 0.0141 | 1.44x |
| curved_step_import/cold | 8.0526 | 7.0264 | 1.15x |
| curved_step_import/trailing_edit | 0.2081 | 0.1036 | 2.01x |
| engraved_plate/cached | 4.7850 | 4.1321 | 1.16x |
| engraved_plate/cold | 277.3723 | 249.4896 | 1.11x |
| engraved_plate/trailing_edit | 138.9882 | 14.6041 | 9.52x |
| filleted_block/cached | 0.0195 | 0.0136 | 1.43x |
| filleted_block/cold | 4.6898 | 3.5686 | 1.31x |
| filleted_block/trailing_edit | 0.2857 | 0.1250 | 2.28x |
| open_enclosure/cached | 0.0290 | 0.0225 | 1.29x |
| open_enclosure/cold | 5.9765 | 4.2804 | 1.40x |
| open_enclosure/trailing_edit | 0.4874 | 0.2068 | 2.36x |
| patterned_plate/cached | 0.0648 | 0.0653 | 0.99x |
| patterned_plate/cold | 69.4079 | 64.8524 | 1.07x |
| patterned_plate/trailing_edit | 2.0096 | 0.7814 | 2.57x |
