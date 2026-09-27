# Sketch responsiveness and numerical robustness implementation plan

Status: responsiveness repair implemented; see [implementation and evidence](sketch-responsiveness-repair.md). Full precision migration and native performance qualification remain separate work.
Date: 2026-09-26
Reproducer: `C:/Users/cadel/Downloads/Lag_Fix.zcad`

## Objective

Keep the entire ZeroCAD interface responsive while opening, viewing, selecting,
editing, or evaluating a constrained sketch, including sketches that cannot
solve. Eliminate repeated work before adding concurrency. Handle numerical
rounding without accepting materially contradictory constraints or changing
saved geometry silently.

Follow `docs/competitive-development-plan.md` and
`docs/precision-and-performance-plan.md`. Use shared core behavior for GUI and
headless callers. Keep public persisted data stable in the performance work.

## Evidence and remaining uncertainty

The reproducer is a 2,738-byte compact document with one sketch (38 points,
35 line entities, 68 constraints), variables, and one Join extrusion.
An ad hoc optimized probe linked against the existing local release library
measured file loading at 1.8 ms, individual solves at 30-38 ms, region detection
at 0.32 ms, and cold/repeated model evaluation at approximately 1.1 seconds.
These are exploratory measurements, not qualified release benchmarks or an
end-to-end GUI profile. Record source/build fingerprints before formal comparison.

The solver reports Conflicting, residual approximately 3.32e-8, and identifies
constraint 114 as a constraint whose removal allows convergence. That does not
establish that constraint 114 is the uniquely wrong dimension: leave-one-out
attribution may identify one of several ways to relax an inconsistent loop.

The connected 18.8 + 0.8 + 0.8 and 20.4 dimensions have a stored-value mismatch
of approximately 3.58e-7 mm. Numerical inconsistency is a strong hypothesis to
isolate, not yet a proved single cause of every observed stall.

Confirmed code paths:

- `sketch.rs`: Dimension stores f32, and resolve casts expression results to f32.
- `sketch/solve.rs`: one fixed residual threshold covers mixed units; a conflict
  invokes repeated leave-one-out solves.
- `parametric/eval.rs::sketch_region_cache`: resolves a whole sketch, separately
  solves for diagnostics, then calls entity_curves_solved for each durable ID.
  Each entity call can solve the entire sketch again.
- GUI rendering, picking, selection measurements, and the constraints panel can
  call solving synchronously while building a frame.
- `app/sketch.rs::solve_live_sketch` solves and then invokes geometry rebuilding
  that can solve the same variable-driven model again.
- The existing model worker collapses queued requests, but its channel is
  unbounded. Copying that implementation would not bound pending memory.

The Join extrusion also fails because no existing solid is available to fuse.
Preserve and report that independent failure; do not count disappearance of its
warning as a performance improvement or silently convert the operation.

## Stage 1: Freeze the reproducer and establish a baseline

1. Copy the original into a regression-fixture location; record its hash and
   expected load/diagnostic behavior. Never overwrite the Downloads original.
2. Add a focused profiling harness that reads the fixture through the normal
   document API and records cold/warm evaluation, sketch solves, diagnostic
   sub-solves, region construction, and entity extraction separately.
3. Add lightweight counters or an evaluation trace for structural assertions:
   solves per sketch revision, diagnostic attempts, cache hits, and cancelled
   work. Do not make functional tests depend on elapsed time.
4. Measure release-build UI behavior for open, pointer movement, orbit/pan,
   sketch editing, dimension changes, constraints panel on/off, and 60-second
   idle. Record CPU, memory, input/frame latency, and render settings.
5. Create controlled in-memory variants: original, an exactly consistent small
   rectangle loop, and deliberately inconsistent loops. Compare normalized
   residuals and actual dimensions; do not delete constraints in the user's file.

Exit: a reproducible baseline separates frame work, solver work, and geometry
work. The numeric trigger is isolated or recorded as unresolved.

## Stage 2: Build one reusable result per sketch revision

Primary files: `zerocad-core/src/sketch.rs`, `sketch/solve.rs`,
`sketch/constraints.rs`, `parametric/eval.rs`, and cache types as needed.

1. Introduce an internal immutable resolved-sketch result with solved/fallback
   geometry, solve outcome, diagnostics, and entity/provenance mappings. Use
   shared ownership for consumers instead of cloning a full model per entity.
2. Separate solving from baking curves. Baking whole-sketch and individual
   entity curves must accept an already-resolved model and never invoke solving.
3. Populate evaluator SketchEval from that single result, including diagnostics.
   Preserve construction filtering, derived_from identity, text, mirrors,
   offsets, patterns, region ordering, and attached-face boundaries.
4. Cache successful and failed results. A conflicting unchanged sketch must not
   repeat diagnosis simply because another feature consumes it.
5. Define dependencies explicitly: model geometry/constraints, expression
   source and resolved variables, shapes/modifiers, projection dependencies,
   and solver-policy version. Keep local solve results separate from frame or
   attached-boundary changes when these do not affect solving.
6. Use dirty revisions in the GUI and canonical content/dependency keys for
   headless evaluation. Never serialize/hash the whole sketch on every frame.
   Start with conservative variable invalidation; narrow dependency tracking
   only when tests establish correctness.
7. Bound storage: one current result per sketch plus results still held by
   in-flight consumers. Evict deleted/replaced sketches and clear on document
   replacement. Derived results do not belong in undo or document serialization.

Exit: one primary solve per changed sketch input; zero primary solves for an
unchanged warm evaluation. Individual entity extraction performs zero solves.
Geometry and existing failure behavior remain equivalent before numeric changes.

## Stage 3: Remove solving from interface rendering and picking

Primary files: `render.rs`, `app/picking.rs`, `app/ui/viewport.rs`,
`app/ui/status_bar.rs`, `app/ui/constraints_panel.rs`, `geom2d.rs`,
`extrude.rs`, `app/sketch.rs`, and finished-sketch cache owners.

1. Audit every effective_curves_solved/solve_model caller; classify as a
   mutation/evaluation boundary or a read-only display consumer.
2. Supply all read-only consumers with the same resolved sketch snapshot.
   Drawing, picking, dimensions, and diagnostics must agree on one revision.
3. Remove the per-frame constraints-panel solve and the duplicate live-sketch
   solve/rebuild sequence. Reuse the same report for DOF and conflict badges.
4. A cache miss schedules work; rendering must never hide a synchronous solve
   inside a lazy cache accessor. Show the last valid snapshot with a pending
   indication. If no snapshot exists, show an explicit loading state.
5. Permit inspection of the displayed snapshot while pending, but defer or
   revalidate geometry-changing commands that need the latest revision.
6. Audit repaint requests so an unchanged idle scene does not continually solve,
   regenerate geometry, upload meshes, or redraw without a reason.

Exit: pointer movement, redraw, panel opening, and selection measurement cause
zero solves on an unchanged sketch. Picking matches displayed geometry.

## Stage 4: Make edits and diagnosis cancellable background work

Primary files: core solver, `evaluation_worker.rs`, `app/sketch.rs`,
`app/update.rs`, and a focused sketch-worker module if needed.

Use the existing generation/cancellation conventions, but keep live sketch
requests from waiting behind a long solid evaluation. Prefer one persistent
sketch worker with one running request and one replaceable pending request;
never spawn a worker per frame. Bound completion storage too. Measure the
additional worker's memory/CPU impact and keep existing concurrency limits.

1. Tag requests/results with document/session identity, sketch identity, edit
   revision, and variable revision. Discard every stale completion.
2. Check cancellation inside solver iterations, damping attempts, and between
   diagnostic trials; add checks inside expensive matrix work if measurements
   show the cancellation target is missed.
3. Separate primary solving from detailed conflict attribution. Interactive
   work has a bounded deterministic iteration/diagnostic budget. More detailed
   attribution is cached and runs after the current request settles.
4. Distinguish converged, conflicting, non-converged, pending, and cancelled.
   Cancellation or exhausted diagnostic budget must never imply convergence
   or identify an unverified culprit. Keep new state internal where possible;
   audit callers before changing public result contracts.
5. Preserve transactional edits: stage a candidate, validate it asynchronously,
   and commit only an accepted current result. Preserve current policy for
   invalid dimensions and the previous valid document/geometry on failure.
6. Dragging displays immediate handles and last-valid geometry while solving.
   One completed gesture produces one undo transaction; mouse release requests
   the final revision. Escape, undo, close, and document replacement invalidate
   pending work. Never let a late result resurrect a discarded edit.
7. Save/export/Finish Sketch must await or explicitly reject a pending current
   validation, without blocking the interface or saving stale solved geometry.
   Do not add provisional results to recovery as accepted state.
8. Workers wake the UI on useful completions; avoid continuous polling repaint.

Exit: deliberately slow or impossible sketches cannot block menus or pointer
feedback, and request bursts cannot grow queues or memory without bound.

## Stage 5: Establish a numerically consistent solver policy

Primary files: `sketch/solve.rs`, `sketch/linalg.rs`, `sketch.rs`, and focused
solver regression tests. Complete the reproducer isolation before changing
acceptance criteria.

1. Inventory every residual and Jacobian: distance, coincidence, fixed points,
   radius, orientation/angle, parallel/perpendicular, and special constraints.
   Record their units, behavior near degeneracy, and parameter scaling.
2. Normalize optimization rows and derivatives consistently. Define acceptance
   using physical distance errors and angular errors rather than comparing
   squared-distance and linear residuals to the same number.
3. Choose physical tolerances from supported precision and geometry tests.
   Keep bounded absolute/relative terms tied to local geometry, not distance
   from the world origin. Test exact boundary behavior and small features.
4. Validate every constraint against the final candidate independently of the
   optimizer's stopping condition. Low gradient, small step, or poor rank alone
   does not establish success. Check finiteness and degenerate entities.
5. Keep DOF/rank thresholds separate from acceptance tolerances; make rank
   analysis consistent with row/parameter scaling.
6. Add a solver-facing f64 expression resolution path without changing the
   persisted Dimension layout. Stored f32 literals can only be widened exactly;
   they cannot recover lost decimal intent. Test expression-vs-literal behavior
   before switching the production path, since partial migration can introduce
   new inconsistencies between expression and legacy values.
7. Version the solver policy in derived caches and invalidate incompatible
   hydrated results as appropriate. Do not rewrite legacy dimensions, remove
   constraints, or silently snap stored geometry to guessed decimal values.

Exit: the intended small-rectangle relationships pass justified geometric
checks; deliberately contradictory constraints still fail; supported results
remain deterministic across repeated solve and save/reopen workflows.

## Stage 6: Separate authoritative precision migration

This is a separate, explicit schema project under the existing precision plan,
not a prerequisite for eliminating interface stalls.

Inventory dimensions, expression entry, shape coordinates, solver points,
frames, baked geometry, and selection references. Introduce versioned f64
payloads through the decoder registry, preserve exactly the values in older
files, and convert to f32 only at display boundaries. Test migration, old/new
readability rules, undo, export, file size, and large-offset small-detail cases.
Do not perform a broad f32-to-f64 replacement in the performance patch.

## Verification and acceptance

Functional matrix:

- Frozen Lag_Fix, isolated rectangle loops, 0.01/0.8/1/100/1000 mm features,
  translated/rotated variants, and mm/inch display modes.
- Equivalent arithmetic expressions, variable edits, genuine conflicts,
  redundant constraints, under-constrained sketches, degeneracy and non-finite
  input. Verify actual geometry, not only solver outcome labels.
- Cache invalidation for every authoritative edit, modifiers, projections,
  variable changes, suppression, undo/redo, deletion, open, and save/reopen.
- Cancelled edits, rapid request replacement, document switches, stale
  completions, worker failures, and final commit while a solve is pending.
- Text, offsets, patterns, mirrors, region identity, and downstream extrusions.

Structural tests assert zero solves in rendering/picking, one primary solve per
changed sketch, zero unchanged warm solves, bounded pending requests, no stale
writeback, and preservation of last-valid geometry after rejection.

Performance goals follow the competitive plan: p95 ordinary frames <=16.7 ms,
input feedback <=33 ms, common settled edits <=100 ms, cancellation <=50 ms,
and idle CPU <1% averaged over 60 seconds on specified reference hardware.
Treat these as qualification goals, not promises or noisy unit-test assertions.
Publish raw samples, hardware/build fingerprints, solve counts, CPU/memory,
and cold/warm timings. Explain any remaining misses rather than weakening gates.

Run required repository checks for implementation changes:

- `cargo fmt --all -- --check`
- `cargo check`
- `cargo test`
- `cargo clippy -p zerocad-core --all-targets`

Retain repository limits of two build/test workers. Geometry behavior changes
need focused core regressions in addition to full gates. Apply OpenRCAD release
checks if kernel code changes; this plan should not require kernel changes.

## Suggested delivery order

1. Baseline fixture, instrumentation, and controlled numeric reproductions.
2. Shared resolved result and evaluator reuse, with unchanged solver semantics.
3. Read-only GUI consumers and cache invalidation.
4. Cancellable bounded sketch worker and transactional edit integration.
5. Normalized numerical acceptance and validated expression precision handling.
6. Separate versioned precision migration, if approved as its own schema scope.

Each implementation change carries its relevant regression tests and measured
before/after evidence. Re-run the original file after every stage, keeping its
independent invalid Join behavior visible. Claim the lag fixed only after the
end-to-end open/edit/view/idle acceptance run, not from headless timing alone.
