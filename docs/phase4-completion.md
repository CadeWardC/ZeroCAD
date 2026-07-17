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

## Deliberate v1 limits and removal phases

- The bundled standards data is a curated ISO/ANSI engineering subset rather
  than a complete licensed standards catalog. Phase 6 expands coverage through
  versioned data packs and provenance-reviewed fixtures without changing old
  documents.
- Oblique circular projection uses a deterministic exact-point polyline because
  the sketch model has no analytic ellipse entity. Phase 6 adds an associative
  ellipse/conic entity, then migrates these projections behind save/load
  equivalence tests.
- Point distances are exact and analytic circular edge measurements use stored
  curve metadata, but general edge distances/angles still consume selectable
  display-edge geometry. Phase 6 routes these through kernel curve extrema and
  tangent evaluators.
- Spline handles are editable, but geometric constraints attach to their points,
  not native spline tangents/curvature/control polygons. Phase 6 adds native
  spline constraint targets and solver Jacobians.
- Arbitrary section planes are rendered correctly by the CPU clipping path. The
  GPU viewport is intentionally disabled while a section is active. Phase 6
  adds native GPU clip planes and cap presentation; CPU/GPU image-equivalence
  tests are the removal gate for the fallback.

## Public-alpha milestone

The end of Phase 4 is the first public-alpha milestone. An alpha build may be
published only when the Phase 4 cross-cutting gate, both workspace gates, the
frozen correctness corpus, and the relative performance comparison all pass.
The v1 limits above must appear in alpha release notes; none may be represented
as complete 1.0 behavior.
