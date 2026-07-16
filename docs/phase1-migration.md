# Phase 1 representation migration ledger

Phase 3 closes the representation debt recorded here. The architecture test in
`zerocad-core/tests/phase1_api_boundaries.rs` keeps the production allowlist
empty and rejects any return to metadata-discarding kernel calls.

## Closed production boundary

- Every ZeroCAD primitive, import, boolean, blend, shell, sweep/loft, pattern,
  mirror, thread, and transform path enters through a policy-aware structured
  operation or the single outcome adapter.
- Production display and candidate validation use strict stored-pcurve
  tessellation. Projection-based compatibility tessellation is not called by
  ZeroCAD production code.
- Prism/extrusion and full/partial revolve build native pcurves. Unary operations
  repair and validate pcurves before commit, then normalize safe coplanar and
  cocylindrical same-domain faces.
- Booleans use face arrangements, split inherited pcurves, fit and validate
  intersection boundaries, return per-body history, and fail atomically when
  strict output cannot be produced. No application-level axis-aligned cut
  approximation remains.
- Enclosed cuts are represented by an outer shell plus independently oriented
  void shells. Severing cuts return structured bodies with per-body face
  history rather than a disconnected shell.
- Fillet, chamfer, shell, prism, revolve, skin/loft, transform, native
  primitives, STEP imports, and booleans all pass the cross-cutting Phase 3
  operation gate: health, watertightness, pcurve consistency, history coverage,
  and strict tessellation.
- Legacy STEP reconstruction is explicit. It reports every rebuilt pcurve and
  any policy-capped imported-edge tolerance promotion; strict import never
  performs that recovery implicitly.

## Compatibility status

The Phase 3 production compatibility allowlist is empty. Deprecated OpenRCAD
source-compatibility entry points remain callable for downstream consumers, but
they are not used by ZeroCAD production and are not an alternate geometry
engine. Each delegates to the canonical operation and documents the metadata or
strictness it discards.

No fillet, chamfer, pocket, thread, sequential-blend, enclosed-void, or boolean
regression is ignored for Phase 3. The only remaining ignored topology
reattachment case is the explicitly Phase 4 half-space discriminator.
