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
- **Move Face.** A planar face can move along its normal. A translation with a
  tangential component fails with a diagnostic and leaves the source body
  untouched.
- **Thicken.** Planar faces create exact prismatic bodies and cylindrical faces
  create exact revolved annular bodies. Thickness must remain finite, positive,
  and non-collapsing.
- **Delete Face.** An internal cylindrical wall is healed by filling its exact
  trimmed axial span. The regression proves that healing a through-hole restores
  the original volume without creating cap bosses.
- **Curved Split Body.** A selected analytic cylindrical face divides a target
  into deterministic inside and outside bodies. Both bodies must be positive,
  strict, watertight, pcurve-complete, and volume-conserving before either is
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

## Deliberate exact-set limits and removal phase

- General tangential Move Face needs neighborhood surgery rather than a face
  offset. Phase 6 adds it behind imported-part reattachment fixtures.
- Delete Face currently heals internal analytic cylindrical holes. External
  walls, planar patch deletion, blends, and general surface gaps require a
  deterministic patch-and-sew service in Phase 6.
- Direct offset and thickening support planar and cylindrical surfaces. Conical,
  spherical, toroidal, spline, and mixed-surface neighborhoods fail explicitly
  until Phase 6 supplies exact local replacement and intersection curves.
- Split Body accepts infinite planes and analytic cylindrical dividing
  surfaces. A selected planar face currently contributes its plane rather than
  acting as a locally bounded knife; general bounded or non-analytic curved
  tools move to Phase 6.
- STL units are not encoded by the format and are interpreted in document
  units. Automatic repair, decimation, and surface-recognizing mesh-to-BRep stay
  out of Part Design 1.0 unless introduced as explicit operations with their own
  validation and provenance contracts.

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
