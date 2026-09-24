# Native precision and performance plan

Status: design proposal, 2026-09-04. No document schema or authoritative float
representation was changed by the audit follow-through.

## Product constraint

ZeroCAD should remain a compact native CAD application powered by OpenRCAD.
External kernels may be used as independent test oracles on verification
machines; they must not become runtime dependencies or packaged components.
Optimize common mechanical modeling paths before adding general machinery.

## Precision work

The application uses `f32` in sketch coordinates, coordinate systems, feature
dimensions, and selection geometry. OpenRCAD operates in `f64`. At 1,000,000 mm,
adjacent positive `f32` coordinates are 0.0625 mm apart; converting those values
to `f64` afterward cannot restore small detail. These facts do not establish a
supported modeling envelope; that requires full workflow tests.

Implement the migration in these independently reviewable steps:

1. Freeze old `.zcad` fixtures and document their expected geometry and units.
   Inventory authoritative numeric fields separately from render buffers,
   thumbnails, approximate bounding boxes, and legacy selection hints.
2. Add double-precision authoritative coordinate/frame types. Keep GPU vertex
   buffers as floats, and use camera-relative display transforms so rendering
   does not dictate model precision. Preserve IDs and topological identity.
3. Introduce explicit feature payload schema versions through the existing
   `(kind_id, payload_schema)` decoder registry. Old decoders preserve exactly
   the values present in the old document; new writers do not silently claim
   backward readability. Cover sketches, baked text, transforms, expressions,
   and assembly placements, rather than changing only primitive dimensions.
4. Move geometry computation and selection reattachment to the authoritative
   types. Convert only at display boundaries. Measure cache memory, copy costs,
   serialization size, and cold/warm evaluation before expanding the rollout.
5. Publish the supported envelope only after the acceptance matrix passes.

Acceptance matrix: 0.01, 1, 100, and 1,000 mm features at offsets of 0, 1,000,
100,000, and 1,000,000 mm, with several rotations and mm/inch display units.
For every qualified cell, check intended dimensions, Boolean volume, edge
identity, upstream edits, undo/redo, save/reopen, and STEP export. Set tolerances
in physical units; do not hide coordinate loss behind a percentage of a large
world-space coordinate. Record safe rejection separately from wrong geometry.

## Performance work

`mechanical_parts_v1` now benchmarks drilled plates, open enclosures, filleted
blocks, engraved plates, and curved STEP imports. It separates fresh evaluation,
cached evaluation, and a trailing body move. It supplements the existing
synthetic 100/500-node histories; five parts are not yet a representative
production corpus.

The initial local results are in `evidence/audit-2026-09-04/performance.md`.
The engraved plate (277 ms cold, 139 ms trailing move rebuild) is the first
concrete profiling candidate. The follow-up now removes quadratic conservative
history and redundant repair/merge work from already-valid placements; full
validation remains. See [the implementation record](text-and-interchange-2026-09-04.md)
for measurements and limits.

Next expand to 20–30 frozen parts, including shafts, brackets, patterned
features, upstream sketch edits, multi-loop lofts, sweeps, and small assemblies.
Use identical input artifacts and outcome checks for baseline and candidate.

For each qualified run record commit plus working-tree diff hash, lockfile hash,
compiler, OS, CPU, RAM, build profile, worker count, and corpus version. Run on
an idle machine, retain raw samples, and report median and tail latency. Measure
peak memory separately; a time-only benchmark is insufficient. Establish
budgets from these results rather than inventing latency promises.

Prioritize work in this order:

1. Prevent unnecessary geometry rebuilds and repeated tessellation after local
   edits; prove cache invalidation remains correct.
2. Profile Boolean intersections, blend candidate searches, and selection
   metadata generation on actual slow parts. Keep analytic fast paths and
   avoid work for support types an operation cannot use.
3. Measure cold start, idle memory, opening a representative design, and release
   package size. Track the shipped dependency graph independently of test tools.
4. Add a regression gate only after measurement variability is understood.
   No Fusion/FreeCAD performance comparison is claimed by this implementation.

## General intersection research

Error propagation and cooperative cancellation are implemented. They do not
prove intersection completeness. The generic fallback still substitutes finite
parameter intervals for unbounded surfaces. Replace that assumption using
trimmed-face domains and operand bounds, with fixtures for translated/scaled
equivalents, small closed loops, tangencies, seams, and imported spline patches.
Require branch counts, residual bounds, and material measurements, not merely
watertight output. An unresolved search must remain a failure rather than an
empty intersection. Follow the existing reproducer-based strategy before
replacing the face split/arrangement engine.
