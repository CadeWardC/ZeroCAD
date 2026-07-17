# ZeroCAD and OpenRCAD Part-Design Master Plan

This is the canonical, tracked roadmap for ZeroCAD Part Design and the OpenRCAD
kernel work that supports it. It supersedes the original planning attachment;
that attachment remains unchanged as a historical source artifact. This copy
uses UTF-8 text, reflects work completed through Phase 5, and incorporates the
Phase 3.5 amendment and the revised Phases 5–7.

## Goal and architecture

ZeroCAD is a history-based parametric CAD application built on the in-tree,
pure-Rust OpenRCAD boundary-representation kernel. The project is developed by
renovating the running architecture rather than replacing it wholesale:

- OpenRCAD owns exact topology, surface geometry, pcurves, validation, healing,
  booleans, topology history, and checked tessellation.
- `zerocad-core` owns documents, semantic bodies and timelines, stable feature
  identities and DTOs, evaluator contracts, provenance, and face naming.
- `zerocad-gui` owns feature creation and editing, selection, previews,
  diagnostics, the feature tree, and viewport presentation.
- `.zcad` remains backward compatible. New feature payloads are append-only and
  versioned, and persisted meaning must not depend on display labels or inferred
  identifier prefixes.

Every body-producing operation follows the same contract:

1. resolve semantic inputs and selectors;
2. invoke a policy-aware kernel or shared body service;
3. require strict topology, pcurve, history, and tessellation validity;
4. record diagnostics, recovery, provenance, and stable naming;
5. commit all outputs atomically;
6. cache only a successfully committed result.

The OpenRCAD operation result is authoritative. Compatibility geometry may keep
old documents usable, but cannot be counted as passing a strict phase gate.

## Release invariants

All release-bound body operations must preserve these invariants:

- healthy, connected, watertight solids with positive volume;
- valid stored pcurves on every surface-backed coedge;
- deterministic topology history with complete required coverage;
- stable semantic ownership and explicit output provenance;
- atomic failure without partial document mutation;
- deterministic save/load, reorder, suppression, undo, and redo behavior;
- crack-free checked tessellation with consistent shared boundaries;
- no unexplained correctness drift in the frozen Phase 0 corpus;
- no greater-than-10% performance regression against its committed baseline.

## Phase 0 — Baseline and regression control (complete)

Phase 0 froze the behavior that later kernel and document changes must preserve.
It established deterministic correctness corpus tests, a byte-frozen external
NIST STEP fixture, and a committed JSON performance baseline with an enforceable
10% comparison harness. The through-hole and Join/Thread cases are permanent
kernel regressions rather than one-time debugging examples.

Gate: the frozen corpus, fixture, and performance comparison are repeatable and
fail on unexplained topology, serialization, or performance drift.

## Phase 1 — OpenRCAD representation foundation (complete)

Phase 1 made stored coedge pcurves, tolerance policy, structured operation
results, validation, recovery, diagnostics, and topology history first-class.
Native primitives, STEP import and export, current booleans, transforms,
healing, and checked tessellation share those contracts. Exact pcurves are
created or rebuilt across construction and topology-changing paths, and strict
tessellation consumes stored pcurves.

The migration ledger is in [`phase1-migration.md`](phase1-migration.md).

Gate: canonical primitives, STEP round trips, booleans, healing cases, and
tessellation pass strict topology and pcurve validation without hidden boundary
projection.

## Phase 2 — Document semantics and evaluator contracts (complete)

Phase 2 introduced stable feature registry identities, versioned numeric-field
DTOs, semantic selectors, explicit feature inputs, suppression, independent
sequence keys, body timelines, body outputs, provenance, semantic validation,
and the shared evaluator outcome boundary. The semantic document is embedded in
the existing graph during migration so established picking, preview, undo, and
rendering paths remain stable.

The architectural debt and removal gates are recorded in
[`phase2-document-architecture.md`](phase2-document-architecture.md).

Gate: semantic contracts drive evaluation and survive deterministic save/load,
reorder, suppression, and undo/redo behavior.

## Phase 3 — Shared Part Design operations (complete)

Phase 3 routed body-producing Part Design features through shared New, Join,
Cut, and operation-outcome services, hardened boolean and blend behavior,
completed native blend pcurves, removed obsolete compatibility paths, and
integrated per-operation topology history directly into downstream face naming.
The unused generic history composition/chain API was deleted rather than
retained as a fictional abstraction. Phase 3 also added the operation and
equivalence matrices required to keep feature ordering and serialization
behavior stable.

The official boolean strategy and its arrangement-engine trigger are documented
in [`phase3-boolean-strategy.md`](phase3-boolean-strategy.md).

Gate: both workspaces pass formatting, checks, complete tests, and all-target
clippy, including the frozen Phase 0 suite.

## Phase 3.5 — Operation completeness (complete)

Phase 3.5 closes the remaining Combine and body-transform gaps before the
professional Part Design UI is built on top of them.

### Intersect

- Add the append-only `BodyIntersect { target, tool, keep_tool }` feature with a
  stable registry identity and version-1 DTO.
- Dispatch `BooleanOp::Common` through the existing shared boolean adapter and
  evaluator pipeline.
- Evaluate every positive-volume overlapping target/tool part pair and commit
  only after every result passes strict validation.
- Reject empty, disjoint, tangent, and contact-only results atomically.
- Consume the target and consume or retain the tool according to `keep_tool`.
- Preserve exact history where available and use the conservative multipart
  naming fallback otherwise.

### Planar Split Body

- Add `BodySplit { target, plane, face }` and the canonical OpenRCAD
  `split_solid_by_plane_operation[_with_policy]` returning
  `OperationResult<PlaneSplitBodies>`.
- Accept origin planes, datum planes, and selected planar body faces.
- Intersect the target with deterministic, policy-sized positive and negative
  half-space tools whose construction is pcurve-complete.
- Reject outside, tangent, incomplete, overlapping, or non-conservative splits.
- Require two positive-volume, strictly valid outputs and commit them as the
  primary feature body and `body_output_id(feature, 1)`.
- Persist both output producers explicitly so UI and downstream evaluation never
  infer ownership from identifiers.
- Defer curved and bounded-face splitting to Phase 5 direct modeling.

### Scale Body

- Add `BodyScale { source, factor, factor_expr, center }` as a distinct feature.
- Accept only finite positive uniform factors; expressions use the existing
  variables service.
- Persist an explicit XYZ pivot, initially the current body bounding-box center,
  with editable fields and a reset action.
- Use an immediate GUI-side transform of the current display mesh for the live
  ghost; committing still invokes the shared evaluator and strict kernel
  operation. The scale preview is not described as a worker-generated B-Rep.
- Transform all source parts transactionally and consume the source only after
  complete validation.
- Scale local topology tolerances by the absolute factor and map stored pcurves
  directly when the carrying surface has an exact UV transform. Unsupported
  parameterizations use the explicit, validated reconstruction fallback with
  recorded recovery. Strict validation remains mandatory before commit.

### Phase 3.5 gate

- Intersect tests cover overlap, disjoint/tangent failure, containment,
  box/cylinder, multipart, and native/imported combinations.
- Split tests cover centered, offset, rotated datum, selected planar face,
  periodic seams, volume conservation, stable dual outputs, and independent
  downstream operations.
- Scale tests cover factors 0.1, 0.5, 2, and 100 on primitives, imports,
  booleans, and blends; invalid factors fail without mutation; area and volume
  scale by factor squared and cubed.
- DTO, semantic, reorder, suppression, undo/redo, save/load, GUI, history,
  pcurve, watertightness, and checked-tessellation coverage passes.
- Both workspaces pass all formatting, checking, test, and all-target clippy
  gates, followed by the frozen Phase 0 correctness and performance harness.

Verification on 2026-07-16 preserved every frozen corpus feature, body, and
triangle count. The settled Phase 3.5 report passed the unchanged 10% gate
against the retained Phase 3 format-era report. A direct comparison with the
original Phase 0 report continues to show only the already-recorded v4-to-v5
document size and canonical save/open costs documented in
[`phase2-document-architecture.md`](phase2-document-architecture.md); the
committed Phase 0 baseline was not rewritten or relaxed.

## Phase 4 — Professional sketching, standards, and inspection (complete)

Phase 4 turns the operation foundation into an efficient engineering workflow.
It added native control-point and fit-point splines, the remaining driving and
geometric sketch constraints, construction/reference geometry, edge projection,
ASCII DXF-to-sketch import, versioned engineering standards, a dependency-aware
parameters table, strict measurement, visual section views, and exact
Common-based interference checking.

The finish pass added associative, curve-aware edge projection, driven/reference
dimensions, density-derived mass, user-selectable thread classes, arbitrary
origin/datum/selected-face section planes, and explicit section/interference
coverage in the cross-cutting gate. The intentional v1 boundaries and their
removal phases are tracked in [`phase4-completion.md`](phase4-completion.md).

### Sketch and profile creation

- Treat control-point and fit-point splines as a dedicated work item, including
  stable edit handles, constraints, continuity controls, trimming, and robust
  region participation.
- After spline support is reliable, import planar ASCII DXF entities into
  sketches: LINE, ARC, CIRCLE, ELLIPSE, LWPOLYLINE/POLYLINE including bulges,
  and SPLINE.
- Preserve layer names and unit information, report unsupported entities, and
  defer blocks, text, hatch, and drawing dimensions.
- Complete the remaining professional sketch constraints, diagnostics,
  projection/reference geometry, construction geometry, and direct arc/spline
  editing needed by Part Design features.

### Standards-driven features

- Add versioned ISO and ANSI hole and thread tables for clearance, tapped,
  counterbore, countersink, and supported thread forms.
- Persist the standards-library identifier and version alongside the fully
  resolved numeric geometry. Updating a library must never silently change an
  existing document.
- Expose standards, size, class, fit, depth, tip, and cosmetic/manufacturing
  metadata in the feature UI while retaining explicit overrides.

### Parameters and inspection

- Add a centralized parameters-table UI over the current variable and expression
  engine, with units, dependencies, validation, rename safety, and diagnostics.
- Expand Measure to exact distances, angles, radii/diameters, areas, volumes,
  mass properties, and selectable coordinate readouts.
- Add visual section views with adjustable planes and correct capped/uncapped
  presentation without mutating model topology.
- Add exact Common-based interference checking with pair reporting, selectable
  results, and positive-volume thresholds.

Gate: professional sketches and standards features remain editable and stable
through rebuild/save/load; standards-library changes cannot alter frozen feature
geometry; reference dimensions do not constrain the solver; and measurement,
arbitrary section, and exact interference results agree with model data.

Verification on 2026-07-16 passed the complete ZeroCAD and OpenRCAD workspace
tests, formatting, checks, and all-target clippy commands. The cross-cutting
Phase 4 gate preserves DXF units/layers and spline records, standards identity
and resolved dimensions, manufacturing metadata, safe parameter renames,
deterministic save/load, equivalent rebuild geometry, and strict inspection
results. Relative to a detached clean pre-Phase-4 `HEAD`, the frozen corpus
retained every feature, body, triangle, and compact file size; hydrated sizes
also remained within the 10% limit.

The immutable legacy Phase 0 comparison continues to report the already-recorded
v4-to-v5 document-size/timing differences. A paired Phase 4/pre-Phase-4 run also
showed that the single-sample timer can exceed 10% on short rows when repeated
with identical code; the Phase 4 timing comparison has therefore not yet
produced the settled zero-exit result required for an alpha build. This is a
release blocker, not permission to relax or rewrite the committed baseline.

The latest Phase 4 release sample produced a 23,698,944-byte executable,
129,314,816-byte idle working set, and 153,538,560-byte peak working set, all
within the current absolute footprint ceilings. Its 1.440-second cold start
remains above the Phase 7 sub-one-second release invariant and is retained as
measured stabilization work rather than being normalized away.

Milestone: the Phase 4 finish gate defines the first public alpha. Alpha builds
must publish the v1 limits in the completion ledger and pass both workspace
gates, the frozen corpus, and the relative performance comparison.

## Phase 5 — Imported-part editing and mesh intake (complete)

Phase 5 makes external parts useful rather than merely viewable. The exact
shipped surface set and deliberate structured-failure boundaries are recorded
in [`phase5-completion.md`](phase5-completion.md).

### Direct editing of STEP B-Reps

- Add history features for Press/Pull or Offset Face, Move Face, and Delete Face
  on imported STEP bodies with no original feature tree.
- Add Thicken and Offset Face for explicitly supported planar and analytic
  surfaces; unsupported blends, offsets, or topology changes return structured
  failures rather than suspicious geometry.
- Use exact analytic construction, structured booleans, pcurve rebuilding,
  healing, strict validation, and per-operation topology history so direct edits
  participate in naming and downstream Part Design operations.
- Close the Phase 3.5 curved-split deferral for analytic cylindrical faces.
  General bounded-face and non-analytic curved split tools remain structured
  failures until the local topology-surgery work in Phase 6.

### STL mesh bodies

- Import ASCII and binary STL into a validated mesh-body type.
- Support display, selection, transform, measurement, sectioning, and export.
- Diagnose non-manifold, degenerate, inconsistent-winding, and open input.
- Do not pretend mesh bodies are B-Reps and do not allow B-Rep booleans on them.

Direct STEP editing and useful STL intake are Part Design 1.0 requirements.
General surface-recognizing mesh-to-BRep conversion is explicitly deferred until
after 1.0.

Gate (closed for the exact supported set): a curated matrix of strict STEP-imported
planar and analytic cylindrical parts survives every supported direct-edit
feature, strict validation, deterministic save/load, and subsequent Part Design
operations. The external NIST recovery fixture also proves that an unsupported
legacy boundary fails atomically. Imported STL bodies pass manifold and winding
diagnostics plus display, selection, transform, measurement, section, and export
tests without being treated as B-Reps.

## Phase 6 — Exchange, performance, and architecture cleanup

- Broaden STEP interoperability and round-trip fixtures without weakening the
  strict internal representation.
- Complete remaining evaluator/container migration only behind equivalence and
  file-compatibility tests.
- Remove temporary ledgers, allowlists, wrappers, and dual conventions once
  their named replacement gates pass.
- Profile rebuild, boolean, tessellation, rendering, and large-document paths;
  use deterministic caches and cancellation without changing results.
- Close every item in the Phase 4 completion ledger: expand versioned standards
  packs, add analytic projected ellipse/conic geometry, route general edge
  inspection through exact kernel curves, and add spline-native constraints.
- Extend direct topology surgery beyond the Phase 5 exact set: tangential face
  moves, external/general Delete Face healing, conical and spherical offsets and
  thickening, and bounded or non-analytic curved Split Body tools.
- Add GPU-native arbitrary section clip planes and caps. CPU/GPU image
  equivalence is required before removing the correct CPU fallback.
- Maintain the Phase 0 performance comparison and investigate every material
  regression rather than refreshing the baseline to hide it.

Gate: every scheduled migration ledger, compatibility allowlist, obsolete
wrapper, and dual ownership/container convention is empty. STEP round-trip and
cross-version fixtures pass, the frozen correctness corpus is unchanged, the
relative performance harness is green, and the remaining distance to the Phase
7 absolute budgets is measured. Any unavoidable release exception moves to
Phase 7 with an owner, reason, user impact, and removal trigger.

## Phase 7 — Stabilization and Part Design 1.0

- Run long-form randomized, scale, imported-part, save/load, undo/redo, and
  cross-platform stress suites.
- Run a curated OCCT differential-oracle suite for primitives, imports,
  booleans, direct edits, mass properties, and failure classification. OCCT is
  test-oracle-only and is never linked into or shipped with ZeroCAD/OpenRCAD.
- Finish user-facing diagnostics, recovery explanations, accessibility,
  documentation, examples, migration notes, and crash-safe persistence.
- Triage all compatibility and strict-validation exceptions; every retained
  exception needs an owner, reason, and removal trigger.
- Ship only when both workspaces are warning-reviewed, all gates are green, and
  the frozen corpus and performance harness pass unchanged.

### Absolute performance and footprint release invariants

The relative Phase 0 comparison prevents sudden regressions; these absolute
limits prevent several individually acceptable changes from accumulating into a
slow release. Measurements use the Phase 0 reference machine and procedure, or
a formally documented replacement with both old and new results captured for
normalization.

- Stripped release executable remains under 30 MiB.
- Compressed distribution targets 20–25 MiB; installed footprint remains under
  50 MiB.
- Cold start remains under 1 second and warm start under 500 ms.
- Idle working set targets 120–150 MiB and must not exceed 150 MiB without an
  approved, measured justification.
- A typical trailing feature edit completes in under 50 ms; a representative
  full rebuild completes in under 250 ms.
- The viewport sustains 60 FPS around one million displayed triangles on the
  reference scene and hardware.
- Compact recipe save and open each remain under 100 ms for 100 features and
  under 250 ms for 500 features, excluding geometry rebuild.
- A hydrated `.zcadh` document displays its validated mesh within 250 ms and its
  first edit takes no more than twice the equivalent already-warm edit.
- Streaming `.zcadh` save/open never retains the complete file plus every
  uncompressed section simultaneously.
- No time, footprint, or file-size regression above 10% is accepted without a
  documented correctness justification, even when the absolute ceiling still
  passes.

Gate: all absolute limits above, the relative 10% comparison, long-form stress
suites, compatibility checks, and both workspace gates pass on a release build.

## Required handoff gates

Run these at each operation-completeness and release handoff:

```text
# ZeroCAD workspace
cargo fmt --all -- --check
cargo check
cargo test
cargo clippy -p zerocad-core --all-targets

# OpenRCAD workspace
cargo fmt --all -- --check
cargo check --workspace
cargo test --workspace
cargo clippy --workspace --all-targets
```

Then run the frozen Phase 0 corpus and JSON performance comparison. Correctness
or topology drift is a hard failure. A performance change greater than 10% is a
hard failure until explained and resolved; the committed baseline remains
unchanged.
