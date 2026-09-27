# Sketch responsiveness repair

Date: 2026-09-27. Local implementation and regression evidence; not a release qualification.

## Reproductions

The two original Downloads documents are preserved byte-for-byte in regression fixtures:

| Fixture | SHA-256 |
|---|---|
| `tests/fixtures/lag-fix.zcad` | `cd0d4983414ee90413aebb3d798d8d9436bb19de35351230e297b0596d055c19` |
| `tests/fixtures/more-lag.zcad` | `c5a285b32fc8c6d40e02b8889858cb17d7950faa2467bf6509cebeb10239a680` |

Paths in this table are relative to `zerocad-core`. Hashes match the Downloads originals.

The first sketch has 38 points, 35 entities, and 68 constraints. The second has
34 points, 31 entities, and 59 constraints. The original solver reported conflicts
114 and 15 respectively. These identifiers describe a constraint whose removal
lets the numerical search succeed; they do not prove the named constraint is
uniquely wrong. In particular, constraint 15 in the second file fixes a point.

A probe linked to the pre-change optimized library measured:

| File | One solve | Cold evaluation | Repeated evaluation |
|---|---:|---:|---:|
| Lag_Fix | 30-38 ms | 1,123 ms | 1,100 ms |
| more_lag_and-issue | 9.9-11.0 ms | 307 ms | 291 ms |

These exploratory samples came from `evaluate_request` with final quality.
File parsing was about 1-2 ms. The expensive work was repeatedly solving the
same sketch, including a leave-one-out diagnostic search, not reading the file.

## Implementation

- A bounded per-thread solver cache shares successful and failed immutable
  results between whole-sketch baking, per-entity extraction, diagnostics, and
  repeated evaluation. Inputs are compared exactly, including expressions and
  variable values. Individual entity extraction no longer clones the entire
  solver model or re-solves it. Cache entries are disposable, outside documents
  and undo, with an entry limit and serialized-payload accounting budget.
- Drawing, picking, measurements, feature properties, typography masks, and the
  constraints panel request/reuse background results rather than solving inside
  a read-only UI call. Worker completions are published at frame boundaries so
  those consumers see the same snapshot throughout a frame.
- Live edits get a small cancellable immediate budget; work exceeding it runs on
  one persistent sketch worker. New live inputs cancel superseded requests and
  preempt unrelated display solves. Pending edits and completions are bounded;
  cancelled work never publishes a result. Exact input identity prevents stale
  writeback. Finish Sketch waits for a pending edit while the interface remains
  responsive. Dimension acceptance remains transactional, including named
  variables; cancel and undo discard pending acceptance.
- The model evaluator now retains one running request, one replaceable pending
  document, and one completion, rather than an unbounded channel of snapshots.
- Cancellation reaches system construction, nonlinear iterations, damping,
  normal-matrix construction, Cholesky, QR rank analysis, and diagnostic trials.
  Leave-one-out attribution is capped at 64 trials; an unestablished culprit
  remains unknown instead of being guessed.
- Point-distance residuals now measure actual length error in mm, with the
  corresponding analytic derivative. Explicitly linear rows have a final
  acceptance bound of 1e-6 mm (one nanometre), independent of world position.
  The optimizer still attempts the stricter target before accepting a stationary
  result. Every row must pass its own threshold and geometry must be finite.
  Other nonlinear/angular residuals retain their existing stricter thresholds;
  they have not received a blanket tolerance increase.
- Disposable geometry cache ABIs advance from 15 to 16 so hydrated documents
  rebuild with the corrected solver. Authoritative `.zcad` payloads and stored
  values are unchanged. No constraints are deleted and no decimal intent is
  guessed from already-rounded literals.

## Regression evidence

Core regressions cover both frozen files, independently measured distance
errors, real 0.01 mm contradictions, 0.0001 mm contradictions at multiple scales
and offsets, non-finite inputs, cancellation, exact cache reuse, variable
invalidation, cold/warm evaluation, and save/reopen. Evaluation does not rewrite
the authoritative document. The second file still produces one body without
warnings. The first retains its separate invalid Join diagnostic: there is no
existing solid for that Join to fuse with.

Worker regressions cover result reuse, request bursts, superseded inputs,
selection of an existing request while cancelling another, and the model
mailbox's latest-generation behavior. GUI coverage includes waiting to finish,
discarded sessions, undo and named dimensions, plus 60 pointer/redraw/measurement
frames per fixture after settling. Unchanged frames schedule zero new solves.

An unoptimized CPU-viewport smoke run measured about 2.6 ms p50 and 2.8 ms p95
for those frames. This does not measure native input-to-display latency, the
GPU driver, or 60-second idle CPU. The test asserts work reuse, not wall time.

## Reproduction commands

```
cargo test -p zerocad-core --test sketch_responsiveness
cargo test -p zerocad-gui --bin zerocad-gui app::sketch_resolution -- --nocapture
cargo run --release -p zerocad-core --example sketch_profile -- zerocad-core/tests/fixtures/lag-fix.zcad zerocad-core/tests/fixtures/more-lag.zcad
```

Local hardware: AMD Ryzen 7 7840HS, 8 cores/16 logical processors, approximately
16 GB installed RAM, Windows. Compiler: rustc 1.94.0 (4a4ef493e 2026-03-02).
Base commit: `4e37ce27e6fdfd101335290f95ef8ad6e1400389`, with this working-tree
repair. Repository build/test parallelism remains two workers.

## Scope relative to the implementation plan

This delivers solver reuse, interface scheduling, bounded workers and targeted
physical-distance acceptance for the reproduced failures. Full authoritative
f64 migration remains the separate versioned-format project described in
`precision-and-performance-plan.md`; it is not performed by this repair.
Likewise, extending normalized residuals to every specialized nonlinear
constraint needs its own geometry evidence. Native frame/idle qualification and
large-sketch memory scaling remain distinct from the local smoke measurements.
