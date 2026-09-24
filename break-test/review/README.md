# Review evidence, 2026-09-11

Read the [verdicts and implementation plan](../../docs/break-test-review-and-implementation-plan.md) first.

`review_probes.rs` contains five review-only observations derived from existing tests. It prints warnings, body ownership, displayed bounds/volume, and `inspect_body` results. Its inherited characterization assertions deliberately reproduce the old tests; their passing does not validate the descriptions in FINDINGS.md. The additional observations disprove several descriptions.

The probe is archived outside Cargo's automatic test discovery. To reproduce from the repository root in PowerShell:

```powershell
if (Test-Path 'break-test/tests/review_probes.rs') { throw 'A probe target already exists; do not overwrite it.' }
Copy-Item -LiteralPath 'break-test/review/review_probes.rs' -Destination 'break-test/tests/review_probes.rs'
cargo test -p break-test --test review_probes -- --nocapture --test-threads=1
```

Afterward, remove only the temporary `break-test/tests/review_probes.rs` copy you created. Keep the archived source. Thread inspection took substantially longer than the other observations.

Logs are in the parent `break-test` directory:

- `review-ignored-run.log`: explicit reproduction of six ignored panic tests.
- `review-probes-run.log`: five additional geometry/ownership observations.
- `review-characterizations.log`: focused characterization rerun.
- `review-rim-controls.log`: existing circular/annular rim controls.
- `review-thread-cutter.log`: explicit threaded-cutter rejection reproduction.
- `review-test-run.log`: original broad break-suite run; manually interrupted during the 100-pocket case.
- `review-workspace-test.log`: broader workspace run with that case excluded; manually interrupted during the 36-hole workload, after 157 tests passed. `workspace-interruption.json` records the interrupted process. This is not a full suite pass.
- `review-fmt.log`, `review-check.log`, `review-clippy.log`: repository checks.

The workspace was already dirty and new test files appeared during this review. No production source was changed by this review. Results describe the tested working tree, not a clean commit or release qualification.
