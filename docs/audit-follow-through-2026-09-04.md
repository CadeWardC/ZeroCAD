# Audit follow-through — 2026-09-04

This records implementation following `project-audit-2026-09-04.md`. The audit
describes the original `9b1290f1` snapshot; this delivery is an uncommitted change
set and does not assert released or hosted-CI qualification.

The first-batch verification below is historical. The subsequent text optimization,
nested-contour repair, cavity export, and remaining limitations are recorded in
[text-and-interchange-2026-09-04.md](text-and-interchange-2026-09-04.md).
Assembly follow-through has its own [qualification record](assembly-qualification-2026-09-04.md).

## Implemented

- Authoritative single- and multi-body Boolean paths propagate surface
  intersection failures with a shared work budget. Recursive intersection and
  face-curve trimming observe cancellation. Regression tests force exhaustion,
  check unchanged operands and retry, and exercise recursive cancellation.
  Legacy convenience intersection wrappers remain available; Boolean authority
  no longer uses their error-discarding behavior.
- Asset-bound V2 replay cases verify SHA-256 content before loading actual STEP
  data or rebuilding a saved document body. V1 identities remain compatible;
  mislabeled primitive corpus stages are corrected. Observations compare body
  count, volume, surface area, and centroid, including a test that rejects a
  healthy but geometrically wrong result. Valid empty Boolean results remain
  representable. The supported fuzz lane also checks an analytic volume/area/
  centroid oracle.
- The File > Export menu includes part STEP export. A background worker captures
  the authoritative document, visibility, project generation, and revision.
  Export evaluates final B-Reps, rejects unresolved/mesh-only output, and writes
  through an adjacent temporary file before replacement. The STEP writer emits
  named products, every independent body, and explicit millimetre units. Unicode
  and apostrophes in names are escaped. No display mesh is used as export truth.
- Named cylinder/plane straight intersections receive a support-frame branch
  discriminator. Existing durable identities are preserved. The previously
  ignored test now checks opposite branches, radius changes, a rigid transform,
  saved consumer references, and refusal to retarget a missing branch. Exact
  bounded B-Rep line support takes precedence over erroneous display circle
  hints for these edges. This is a targeted support-pair implementation, not a
  universal topological naming solution.
- Five frozen mechanical parts are shared between lifecycle tests and the
  Criterion performance suite. Lifecycle checks cover fresh/cached evaluation,
  save/reopen, and STEP export. Benchmarks include cached evaluation and trailing
  moves as well as cold evaluation.
- CI retains fuzz logs, reproducers, and evolved corpus seeds. A separate
  scheduled/manual verification job compares identical canonical inputs against
  an external kernel and validates exported STEP geometry. Release publication
  depends on the reusable validation workflow for the tagged source. A Clippy
  fingerprint baseline rejects additional warning occurrences in both Cargo
  workspaces without suppressing the existing warnings.

## Runtime boundary

OpenRCAD remains the sole shipped geometry kernel. OpenCASCADE/OCP is optional
Python verification tooling installed only on test machines, never linked into
ZeroCAD or included in release packaging. Neither the application nor kernel Cargo lockfile gained a package.
The stale, separate fuzz lockfile was refreshed to match the current core and
its production dependency versions. The existing Rust `tempfile` package is now used by core export
for atomic destination replacement.

Multi-body STEP output is verified with an independent reader. The current
native STEP importer still selects one solid; multi-body import and assembly
STEP export need their own implementation and tests.

## Additional defects found while verifying

1. `ZeroCAD 07` engraved with the frozen Hack font creates nested hole contours
   on a native planar face; the independent reader rejects that arrangement.
   A native export guard now rejects it and preserves an existing destination.
   The regression retains this exact input. The supported benchmark uses
   `ZeroCAD 17`. **Repair of the source text/Boolean arrangement remains open.**
   The guard samples curved boundaries and does not prove arbitrary arrangement
   validity. Expand native face validation and fix this source operation next.
2. The writer previously selected only the first shell of a solid. Closed
   interior cavities now fail explicitly instead of silently losing that shell.
   **Interior-shell STEP representation remains unsupported.** Open enclosures
   and multiple independent single-shell solids are covered separately.
3. A downstream fillet on the cylinder/slab branch is still outside the verified
   blend capability. Reference persistence and reattachment pass; that does not
   claim the blend itself succeeds. Expand curved-support blends from concrete
   reproducers rather than weakening candidate validation.

## Verification and reproduction

Native gates passed: formatting in both workspaces; application check; 926
application tests; 575 kernel tests; and `python scripts/check-clippy.py`. No
tests were ignored. Both fuzz targets compile and all three warning-gate
negative-control tests pass. The scheduled fuzz campaign and hosted CI were
not run locally, and the GUI menu was not exercised interactively. The baseline contains 139
distinct fingerprints / 452 compiler warning occurrences, including repeated
library/test diagnostics. It is a reviewed backlog, not warning-free code.

Optional independent checks (separate environment with
`cadquery-ocp==7.8.1.1.post1`):

```text
cargo build --locked -p zerocad-core --example boolean_observation
python tools/boolean_occt_replay.py --rust-executable <built example> --output <report.json>
```

Set `ZEROCAD_INTERCHANGE_EVIDENCE_DIR` to an absolute directory and run the
`mechanical_workflows` and `step_export` integration tests, then:

```text
python tools/verify_step_exports.py <fixture directory> --output <report.json>
```

The reports are preserved under `docs/evidence/audit-2026-09-04/`, with input
identities, STEP file hashes, tolerances, and environment metadata.

The local independent checks pass for all four V1 corpus inputs and all six
supported STEP outputs. They check exact body counts, independent shape validity
for STEP, and volume within the documented mesh-measurement tolerance. Canonical
comparison also checks area and centroid. This is bounded evidence, not general
STEP conformance certification or a complete NURBS capability claim.

A local optimized timing run completed for all 15 workload/mode combinations.
See [the timing table](evidence/audit-2026-09-04/performance.md) and raw samples.
These are initial measurements, not a pre-change speedup comparison or qualified
performance budgets. The engraved plate was the slowest tested part: about
277 ms cold and 139 ms for a trailing move rebuild, versus about 4.8 ms cached.
Profile this rebuild path first; these are backend evaluation measurements, not
measurements of viewport frame latency.

## Next delivery order

1. Repair the reproduced nested text arrangement and add interior-shell STEP
   support, each with native and independent regression evidence.
2. Expand the five-part corpus to 20–30 actual mechanical designs and exercise
   upstream edits, suppression, undo/redo, and assemblies. Keep unsupported
   outcomes separate from incorrect results and timeouts.
3. Qualify repeatable latency, memory, startup, and package-size baselines, then
   optimize the measured hot paths. No speedup against the pre-audit version has
   been established by these changes.
4. Implement the staged precision migration and general intersection research
   described in `precision-and-performance-plan.md`. These remain substantive
   projects; they were not replaced with a mechanical float-type rewrite or an
   unproven general geometry algorithm.
