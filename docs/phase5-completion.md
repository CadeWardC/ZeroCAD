# Phase 5 completion ledger

Phase 5 is closed by the direct-edit and STL matrices in
`zerocad-core/src/parametric/tests/phase5.rs`, the stable DTO and complete
feature-payload corpus, the `.zcad` binary-asset tests, and the GUI command
tests. This ledger states the exact supported set so a structured failure is
not mistaken for missing validation or an unsafe approximation.

## Closed direct-edit set

- **Press/Pull and Offset Face.** Planar faces use their exact trimmed face to
  build a pcurve-complete prism and then add or remove material transactionally.
  Analytic cylindrical walls use exact radius reconstruction or a revolved
  annular tool. Strict topology and checked tessellation gate the replacement.
- **Move Face.** A planar face can move along its normal. Phase 6 additionally
  rebuilds six-planar-face imported blocks for finite tangential translations,
  preserving strict topology and named-face reattachment transactionally.
- **Thicken.** Planar faces create exact prismatic bodies, cylindrical and
  conical faces create exact revolved annular bodies, and spherical faces create
  exact nested-shell bodies. Thickness must remain finite, positive, and
  non-collapsing.
- **Delete Face.** Internal cylindrical walls are healed by filling their exact
  trimmed axial span; external cylindrical bosses are removed with their exact
  analytic tool. Regressions prove through-hole fill and boss removal restore
  the host volume without introducing caps or cavities.
- **Curved Split Body.** Analytic cylindrical faces retain their infinite
  divider semantics. Other selected strict solids act as bounded tools, which
  closes the locally bounded split contract and is frozen by a conical-tool
  regression. Both results must be positive, strict, watertight,
  pcurve-complete, non-overlapping, and volume-conserving before either is
  committed.
- **History and document behavior.** Exact boolean history is consumed by the
  existing naming adapter when available; conservative geometric naming covers
  analytic reconstruction. Direct edits have stable v1 DTOs, semantic inputs,
  explicit provenance, expressions where applicable, reorder and suppression
  behavior, deterministic save/load, and GUI undo/redo.
- **External compatibility safety.** The byte-frozen NIST recovery fixture is
  included as a negative direct-edit case: when its selected legacy boundary
  cannot produce a strict replacement, the imported body remains live and the
  failure is diagnosed instead of committing suspicious geometry.

## Closed STL mesh-body set

- ASCII and binary STL parse into a validated mesh-only body. Original bytes are
  stored as content-addressed `.zcad` assets and identical assets deduplicate.
- Validation reports discarded degenerate facets, open boundaries,
  non-manifold edges, inconsistent shared-edge winding, and disconnected
  components. Malformed, empty, non-finite, or entirely unusable input fails.
- Mesh bodies support display and triangle selection, body translation and
  uniform scale, measurement and density-derived mass for closed input,
  arbitrary-plane sectioning, save/load, and binary STL export.
- A mesh body cannot enter a B-Rep boolean or face edit. Rejection is explicit
  and atomic; Phase 5 never silently promotes triangle soup into a kernel solid.

## Post-completion hardening

- A working imported STEP plus direct-edit graph now passes deterministic binary
  `.zcad` save, load, semantic validation, exact mesh comparison, and strict
  kernel reconstruction. The payload-only corpus remains as a separate ABI gate.
- Reorder, suppression, restoration, provenance, and strict-output checks cover
  Face Offset, Move Face, Delete Face, and Thicken independently.
- Open meshes report enclosed volume and density-derived mass as unavailable;
  the inspection UI no longer displays a misleading numeric zero.
- Direct-edit boolean tools overshoot selected faces using model bounds and the
  kernel tolerance policy instead of an absolute millimetre constant.
- Dedicated STL fixtures assert both non-manifold-edge and disconnected-component
  counts and their user-facing diagnostics.

## Phase 7 owned exceptions

These are structured capability boundaries, not silent fallback paths. Each is
owned and has an explicit removal trigger:

- **General tangential Move Face.** Owner: OpenRCAD local-topology surgery.
  Reason: arbitrary neighborhoods require extending/re-intersecting adjacent
  surfaces rather than translating one face. User impact: Phase 6 supports the
  common imported six-plane block; other neighborhoods fail atomically with a
  diagnostic. Trigger: a strict imported-part fixture matrix covering analytic,
  blend-adjacent, concave, and multi-loop neighborhoods with stable naming.
- **General Delete Face patch-and-sew.** Owner: OpenRCAD healing/sewing. Reason:
  planar exterior patches, blends, and mixed-surface gaps need deterministic
  surface extension and trimming. User impact: internal cylindrical holes and
  external cylindrical bosses are supported; other selections remain unchanged
  and diagnosed. Trigger: patch history, pcurve reconstruction, and strict
  watertight tests for planar, filleted, conical, and multi-face deletions.
- **Non-analytic offset/thicken.** Owner: OpenRCAD surface-offset algorithms.
  Reason: toroidal, spline, and mixed neighborhoods need exact intersection and
  self-intersection classification. User impact: planar, cylindrical, conical,
  and spherical surfaces are supported; the rest fail explicitly. Trigger: an
  exact offset/intersection service passing scale, seam, and collapse matrices.
- **Difficult non-analytic bounded splits.** Owner: OpenRCAD boolean
  arrangements. Reason: the bounded-tool contract accepts any strict solid, but
  success for B-spline-heavy tools is limited by the current intersection
  engine. User impact: analytic bounded tools work; an unsupported tool fails
  before commit. Trigger: curated STEP B-spline tool fixtures pass split,
  conservation, pcurve, and downstream-operation gates.
- **STL interpretation and repair.** Owner: post-1.0 mesh operations. Reason:
  STL encodes no units and automatic repair/decimation changes source geometry.
  User impact: input is interpreted in document units and diagnosed exactly as
  supplied. Trigger: explicit provenance-bearing repair/decimation features;
  surface-recognizing mesh-to-BRep remains outside Part Design 1.0.

## Verification record

On 2026-07-17, formatting, check, complete tests, and all-target clippy exited
zero in both the ZeroCAD and OpenRCAD workspaces. ZeroCAD's run included 292
passing core unit tests, every integration suite, 35 document-container tests,
and 65 GUI tests; the one pre-existing half-space naming test remains explicitly
ignored under its named N4 trigger.

The frozen Phase 0 structural corpus remained byte-manifest compatible: feature,
body, and triangle counts did not move. The retained Phase 5 measurement reports
23,859,712 binary bytes and a 66,920,448-byte idle working set, within the
absolute size and idle-memory ceilings. Phase 4-to-Phase-5 compact corpus sizes
are identical and the release binary grew 1.1%.

The unmodified historical 10% comparison still exits nonzero. It reports the
already-recorded v4-to-v5 compact-size differences, and the first Phase 5 sample
also flags cold startup and three short timers relative to the retained Phase 4
sample. An immediate rerun varied substantially in unrelated sub-millisecond and
short serialization rows, confirming that the one-shot timing comparison is not
settled. Cold startup measured 1.747 seconds and remains above the Phase 7
sub-one-second invariant. These are release blockers; the committed baseline was
not refreshed or relaxed.

The startup blocker is now executable work rather than a passive note:
`benchmarks/profile-startup.ps1` builds once, launches seven fresh GUI processes,
records every sample plus median and p95, and exits nonzero while either is at
or above one second. The first seven-sample run measured a 61 ms median but a
1.245-second p95, so the blocker remains open rather than being hidden by warm
launches. Phase 6 owns profiling initialization until both gates are green; it
does not permit refreshing the frozen Phase 0 baseline.
