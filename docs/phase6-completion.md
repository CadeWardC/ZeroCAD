# Phase 6 completion record

Phase 6 closes the scheduled exchange, architecture, representation, rendering,
and profiling work behind `zerocad-core/tests/phase6_gate.rs` and the complete
ZeroCAD/OpenRCAD workspace suites. This record distinguishes shipped behavior
from the narrow topology-surgery exceptions owned by Phase 7.

## Architecture and compatibility closure

- Persisted `DocumentSemantics::body_outputs` is the sole source of semantic ownership.
  The historical `feature::body:N` parser and edit/load repair convention are
  deleted. The architecture gate now constructs, evaluates, saves, and reloads
  a split-body document, proving table-based ownership behavior without source
  or documentation text searches.
- `Document` remains the application container and `FeatureRecord` remains the
  authoritative feature record. The registry-driven resolve, invoke, validate,
  record, commit, and cache transaction is unchanged and mechanically gated.
- The Phase 1 production compatibility allowlist is empty. ZeroCAD production
  uses canonical document and kernel operation APIs. Public deprecated
  source-compatibility functions remain thin downstream ABI adapters only; they
  are neither scheduled migration debt nor an alternate production engine.

## Phase 4 representation closure

- Versioned standards pack v2 expands the bundled, provenance-labelled ISO/ANSI
  range without changing the frozen numeric geometry stored by old features.
- An analytic ellipse sketch entity preserves oblique circular edge projections
  as one associative curve across rebuild and canonical save/load. General
  parabola and hyperbola entities are not implemented.
- Native spline tangent and curvature constraints have durable payloads,
  solver rows and Jacobians, variable-bound dimensions, and GUI editing. The v1
  curvature row is explicitly approximate: it uses a finite difference of the
  first three spline handles and a numerical Jacobian, not exact NURBS
  curvature.
- Edge length and midpoint/tangent inspection integrate stored kernel curves.
  Pairwise closest distance and angle are explicitly approximate: curve
  resolution uses a 5% endpoint-match threshold and the closest-point search is
  a deterministic 32x32 parameter grid followed by bounded refinement. Neither
  path depends on viewport tessellation.

## Exchange and direct topology work

- Self-generated strict STEP round trips cover planar, cylindrical, conical,
  spherical, truncated, and apex-cone cases plus seam pcurves and deterministic
  canonical rewrite. A byte-frozen FreeCAD AP214 cube from Mayo is the successful
  third-party interoperability fixture; the byte-frozen NIST AP203 bracket is a
  deterministic unsupported-`INTERSECTION_CURVE` failure oracle. The writer now
  represents curve-less apex seams with their real endpoint chord instead of
  emitting a zero-length placeholder.
- Tangential Move Face is supported for imported six-planar-face blocks.
  Delete Face handles both internal cylindrical holes and external cylindrical
  bosses. Conical and spherical offset/thicken paths build exact pcurve-complete
  replacements. Split Body accepts strict bounded solid tools in addition to
  its established plane and infinite-cylinder semantics.
- Every component replacement, including Thicken and bounded Split Body, enters
  one transactional runtime gate. Watertightness, topology health, pcurve
  coverage, strict validation, and checked tessellation pass before source
  replacement. Operation-specific conservation remains an additional bounded-
  split precondition; it is not claimed as a generic direct-edit invariant.

## Rendering and performance work

- Arbitrary section planes are GPU-native shader clip planes. Capped sections
  are GPU mesh layers, while both viewport paths share contour extraction and
  nested-loop triangulation; the CPU renderer remains the automatic hardware
  fallback. Offscreen shader tests and cap-plane/area tests prevent backend
  drift.
- Startup no longer loads preferences twice or probes the fallback GL backend
  before the primary backend, and normal builds no longer enable debug logging.
  `benchmarks/profile-startup.ps1` performs the required seven-sample startup
  run and records median, p95, working set, and peak working set.
- `zerocad-core/benches/modeling_pipeline.rs` profiles Common booleans, checked
  tessellation, GPU render-buffer preparation, cold/warm rebuild, canonical
  500-feature save/open, and hydrated first edit. The benchmark itself now uses
  the canonical document APIs rather than a legacy container adapter.
- The frozen Phase 0 corpus and committed JSON baseline remain unchanged. The
  comparison reports topology counts separately from timing/footprint data so
  schema-era document costs stay visible and no later regression can hide
  behind them.

## Public-alpha entry

Public alpha begins only after the corrected Phase 6 runtime, persistence,
interchange, frozen-corpus, and performance gates pass. It runs during and
alongside Phase 7 so real imported models and direct edits can expose defects
while contracts may still change. Part Design 1.0 remains the Phase 7 exit, not
the first external test of the product.

## Phase 7 handoff

The owned exceptions, including general neighborhood Move Face, general
patch-and-sew Delete Face, non-analytic offset/thicken, difficult B-spline
bounded splits, and provenance-bearing STL repair, are listed with reason, user
impact, owner, and removal trigger in
[`phase5-completion.md`](phase5-completion.md). No Phase 6 compatibility
allowlist is carried forward.

One release-engineering blocker is also explicit: the first execution of a
freshly linked, unsigned Windows artifact measured 1,661.338 ms even though the
same settled artifact measured 61.554 ms median and 151.351 ms p95. The observed
one-time cost is consistent with host artifact scanning and graphics-driver
warm-up rather than repeated application initialization. Packaging/release
engineering owns it; the user impact is a slow first launch after installation;
the removal trigger is a signed packaged build that includes the first launch in
its seven-sample run and records both median and p95 below 1 second on the
reference machine.

## Verification record

- The complete ZeroCAD test suite passes, including the Phase 0–6 architecture,
  operation, persistence, GUI, and frozen-corpus gates. The workspace compiles
  without deprecated-call warnings outside the explicitly documented `.zcad`
  source-compatibility adapter suite.
- The complete OpenRCAD workspace format, check, test, and all-target clippy
  gates pass, including strict STEP apex/truncated-cone round trips and the GPU
  offscreen arbitrary-plane clip test.
- The stripped release executable is 22,761,984 bytes (21.71 MiB), below the
  30 MiB release ceiling and 4.27% above the immutable Phase 0 measurement.
- The settled-artifact seven-process startup profile passes at 61.554 ms median
  and 151.351 ms p95. The just-linked first-launch result remains visible as the
  Phase 7 blocker above; it is not removed as an outlier or treated as passing.
- `target/phase6-phase0-current.json` preserves every frozen feature, body,
  triangle, and tessellation count. Its application measurement is 185.335 ms
  cold start, 84,373,504-byte idle/peak working set, and 1.0122 ms analytic
  cylinder tessellation. The enforced comparison exits zero.
- `benchmarks/phase0-baseline.json` remains unchanged. The v5 document-only
  reference is SHA-256-bound to that exact baseline and applies solely to
  save/open time and compact/hydrated byte counts; topology, rebuild, binary,
  startup, memory, and tessellation remain compared directly with Phase 0.

Failures cannot be converted into passing results by refreshing either
reference: the harness refuses to overwrite them and verifies their binding.
