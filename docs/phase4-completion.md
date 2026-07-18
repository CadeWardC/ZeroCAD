# Phase 4 completion ledger

Phase 4 is closed by the cross-cutting gate in
`zerocad-core/tests/phase4_gate.rs` plus the focused sketch, standards,
inspection, rendering, persistence, and GUI tests. This ledger names the
deliberate v1 boundaries so they cannot be mistaken for unimplemented promises.

## Closed finish items

- **Projected sketch edges.** Project Edge stores the source body and durable
  topology edge reference, creates locked construction geometry, registers the
  body-to-sketch dependency, and refreshes the projection after upstream body
  evaluation. Lines and coplanar circular edges remain analytic. An oblique
  circle is sampled deterministically at points lying on its exact orthogonal
  projection.
- **Reference dimensions.** Every dimensional sketch constraint can toggle
  between driving and reference modes without replacing its durable id.
  Reference dimensions report current geometry and contribute no solver rows or
  degrees-of-freedom reduction.
- **Standards immutability.** Hole and thread features persist library identity,
  version, selection, class/fit, and frozen resolved geometry. Evaluation reads
  the feature's numeric fields. The Phase 4 gate simulates a future library
  version/table change and proves an existing feature's geometry is unchanged.
- **Inspection gate.** Density-derived mass, arbitrary world/datum/selected-face
  section planes, and exact Common-based interference are covered by the Phase 4
  gate. Sections remain presentation-only and never mutate model topology.

## Phase 6 removal record

Every scheduled Phase 4 limitation is closed:

- Standards data is append-only and provenance-labelled. Pack v2 expands the
  bundled range to ISO M3-M24 and ANSI #2-1/2 inch while pack v1 remains
  identifiable. Features still persist their selected pack version and frozen
  dimensions, so catalog updates cannot change an existing model.
- `SketchEntity::Ellipse` stores an analytic ellipse center, major/minor vectors,
  trim range, and source association. Oblique circular projections now remain
  one exact associative ellipse through rebuild and `.zcad` save/load; faceting
  occurs only at the legacy `SketchCurves` consumption boundary. Parabola and
  hyperbola sketch entities are not part of this implementation.
- General edge length is integrated from stored kernel curves. Pairwise closest
  distance and tangent angle use a deterministic 32x32 parameter grid plus
  bounded refinement after accepting a kernel edge within a 5% endpoint-match
  threshold. The UI labels those pair measurements as approximations; viewport
  tessellation quality does not affect them.
- `SplineTangent` and `SplineCurvature` are durable native constraints with
  solver residuals/Jacobians, GUI creation/editing, and serialization coverage.
  The v1 curvature constraint is labelled approximate because it uses the first
  three control/fit-point handles as a finite difference rather than exact
  NURBS curvature.
- The GPU shader clips arbitrary world-space planes. Capped sections are real
  GPU mesh layers built from the same contour extraction and nested-loop
  triangulation used by the CPU fallback. Shader image tests prove half-space
  clipping, and geometry-equivalence tests keep cap area and plane placement
  backend-independent.

The bundled standards tables remain a redistributable engineering subset, not
a claim to reproduce every licensed standards publication. Further reviewed
packs can be appended without reopening Phase 4 or changing existing documents.

## Public-alpha milestone

Phase 4 completed the feature surface required by public alpha. Actual alpha
distribution begins after the corrected Phase 6 runtime, persistence,
interchange, frozen-corpus, and performance gate passes, then runs during and
alongside Phase 7. The v1 limits above must appear in alpha release notes; none
may be represented as complete 1.0 behavior.
