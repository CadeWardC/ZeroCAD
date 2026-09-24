# break-test FINDINGS — errors found and proposed fixes

Date: 2026-09-10. Suites: `degenerate_params.rs`, `tangency_sweeps.rs`,
`complex_parts.rs`, `feature_edge_cases.rs`, `extrude_on_faces.rs`,
`edge_blend_break.rs`. Every item was recorded either as an `#[ignore]`d
known-break test or a `characterization_*` test.

> **Status 2026-09-11** — every finding below was audited and the repairs
> sequenced in
> [docs/break-test-review-and-implementation-plan.md](../docs/break-test-review-and-implementation-plan.md).
> That review CORRECTS several entries here (E9, E11-box, E15 cause, E18,
> E19, E3's fit limit, A3's overflow site) — do not implement this file
> verbatim. As of that date the A1–A6 panics, E10, E11, E12, E15, E19, and
> E21 are FIXED with active regressions in this suite; E16's kernel half is
> pinned by `OpenRCAD/crates/openrcad-algo/tests/cut_coincident_cap.rs`.

Failure taxonomy reconstructed from real CAD sources:
[Why Fillet Fails on Some Edges in Fusion 360](https://cadin360.com/blog/fusion-360/why-fillet-fails-on-some-edges-in-fusion-360/),
[FreeCAD: cannot fillet interior edge at 45°](https://forum.freecad.org/viewtopic.php?t=5315),
[AutoCAD: fillet radius too large](https://www.quora.com/Why-does-it-say-the-fillet-radius-is-too-large-in-AutoCAD-What-do-I-need-to-adjust-to-fillet-the-edges),
[McNeel forum: fillet edge situations](https://discourse.mcneel.com/t/how-to-fillet-edges-in-this-situation/150974).

## A. Hard panics (evaluator crashes — `#[ignore]`d in degenerate_params.rs)

### A1. Box primitive panics on non-positive dimensions
- **Error**: `panicked at zerocad-core/src/mock_kernel/primitives.rs:9` —
  `"positive box dimensions must produce a valid solid: box extents must be positive"`.
  Triggered by w/h/d of 0, negative, or NaN.
- **Impact**: a corrupt `.zcad` file, or a variable-driven dimension that
  resolves to ≤ 0, crashes evaluation (and would crash the GUI's background
  worker). Cylinders already handle the identical inputs gracefully.
- **Proposed fix (safe, local)**: in `ParametricGraph` evaluation, before
  calling `box_solid`, validate `w.is_finite() && w > 0` (same for h, d).
  On failure, mark the feature `ResolutionState::Unresolved` with reason
  `"box dimensions must be positive and finite (w={}, h={}, d={})"` and emit
  `DiagnosticCode::PARAMETER_INVALID` — exactly the path cylinders already
  use. Optionally also replace the `.expect()` in `box_solid` with a
  `Result` return so no future caller can reintroduce the panic; the call
  site in eval already handles `Option`/`Result` for cylinders, so the shape
  change is contained to `primitives.rs` + its single caller.
- **Test**: `degenerate_box_dimensions_panic_known_break` (ignored).

### A2. NaN / ±Infinity extrude depth overflows the kernel arena
- **Error**: `panicked at OpenRCAD/crates/openrcad-topo/src/arena.rs:233` —
  `"attempt to add with overflow"`. Triggered by `Extrude.depth` of NaN,
  INFINITY, or NEG_INFINITY (finite depths of any magnitude are fine).
- **Proposed fix (safe, local)**: guard at the top of the extrude apply path
  in `zerocad-core/src/parametric/eval.rs` (where `depth` is read, before any
  kernel call): `if !depth.is_finite() { return unresolved("extrude depth must be finite") }`.
  This mirrors the existing "must be positive" guards that already run for
  zero-depth (which correctly warn). One guard covers NewBody/Join/Cut since
  they share the entry point; add the same check where `depth_expr` resolves
  so a variable producing NaN fails the same way.
- **Test**: `nan_and_infinite_extrude_depths_panic_known_break` (ignored).

### A3. Huge / Infinite box dimensions overflow the same arena
- **Error**: same `arena.rs:233` overflow with w/h/d of 1e9 or INFINITY
  (1e-6 and normal sizes are fine; 1e9 is a plausible unit mistake, e.g.
  meters entered as millimeters).
- **Proposed fix**: extend the A1 validation to also reject non-finite
  dimensions, and clamp-reject absurd magnitudes. A conservative upper bound
  is safe because everything downstream is f32 geometry: reject
  `w > 1e7` (10 km in mm) with `PARAMETER_INVALID`. The bound must be a
  documented constant, not per-call-site magic.
- **Test**: `huge_box_dimensions_overflow_known_break` (ignored).

### A4. NaN linear-pattern spacing panics in transform validation
- **Error**: `panicked at zerocad-core/src/mock_kernel/primitives.rs:323` —
  `"a rigid transform of a valid solid must remain valid"` with
  `NonFiniteVertex` and `max_deviation: inf` in the validation report.
  Triggered by `PatternKind::Linear { spacing: NaN }` (spacing 0 / negative /
  tiny are handled fine).
- **Proposed fix**: validate `spacing.is_finite()` when the pattern feature
  is applied (same eval-stage guard pattern as A2), reporting
  `PARAMETER_INVALID` / unresolved. Independent of — and simpler than —
  hardening the transform path, because 0 and negative spacing are already
  accepted, so only non-finite needs the guard.
- **Test**: `pattern_with_nan_spacing_panics_known_break` (ignored).

### A5. NaN / Infinity thread parameters panic in band construction
- **Error**: `panicked at zerocad-core/src/mock_kernel/primitives.rs:3053` —
  `called Option::unwrap() on a None value`. Triggered by Thread pitch or
  depth of NaN, or pitch of INFINITY (finite-but-absurd values — 0, negative,
  1e9, depth beyond radius, 1000 starts — all degrade gracefully).
- **Proposed fix**: same guard family as A1–A4 — validate
  `pitch.is_finite() && pitch > 0` and `depth.is_finite() && depth >= 0` at
  the Thread apply path (plus `angle_deg.is_finite()`), reporting
  `PARAMETER_INVALID`/unresolved instead of reaching the band builder.
- **Test**: `thread_with_nan_parameters_panics_known_break` (ignored, in
  `thread_torture.rs`).

## B. Fillet / chamfer gaps and gray zones (characterization tests in edge_blend_break.rs)

### E1. Cylinder rim fillet unsupported — silently a no-op with warning
- **Behavior**: filleting the top rim of a `Cylinder` primitive (r=10, h=20,
  r_fillet=2 — one of the most common fillets in mechanical CAD) always fails:
  `"rolling ball: selected edge was not found on an adjacent face"` → body
  left unchanged with a warning. Confirmed with correctly anchored edge
  references (cylinders run along +Y, base at origin).
- **Root cause**: the rim is a circle, not a straight segment; the EdgeMod
  geometric matcher (`p0/p1/n1/n2`) cannot match a curved edge, and the
  `curve: None` hint gives the blender nothing to work with.
- **Proposed fix**: populate `EdgeRef.curve` with a circle hint
  (`EdgeCurveHint` circle: axis point, axis direction, radius) for circular
  edges when the GUI picks them, and teach the native edge-blend edge matcher
  to resolve a circular edge hint against the body's circular edges by
  (axis-line distance, radius) tolerance matching — the same
  geometric-resolution strategy `FaceRef` already uses for faces
  (centroid+normal). The failure path is already graceful, so this is purely
  additive capability.
- **Test**: `characterization_cylinder_rim_fillet_is_unsupported`.

### E2. Box↔cylinder junction (boss root) fillet unsupported
- **Behavior**: the single most common fillet in real parts — the circular
  edge where a cylindrical boss meets a plate — fails with the same
  "selected edge was not found on an adjacent face" and leaves the body
  unchanged with a warning.
- **Proposed fix**: same as E1 (circular-edge hint + matcher). Note
  `zerocad-core` already has passing rim-blend tests (`annular_rim_blend.rs`,
  `circular_rim_blend.rs`) that go through topology-named references from
  real picks; the gap is specifically the geometric (p0/p1) fallback path
  used by legacy references and scripted construction.
- **Test**: `characterization_box_cylinder_junction_fillet_is_unsupported`.

### E3. Oversize fillet gray zone: r slightly above the maximum succeeds silently
- **Behavior**: on a 20×20×20 box the largest geometrically fitting edge
  fillet is r = 10. r = 10.001 and presumably the band up to ~20 are
  **silently accepted** and cut *something* (r=10.001 → volume 7563.07 vs
  7563.16 for exact-fit r=10), while r = 20 correctly warns. So an invalid
  fillet can report success with geometry that cannot be a true rolling-ball
  fillet.
- **Root cause**: EdgeMod is implemented as a guarded boolean subtraction of
  an edge-aligned cutter, so any radius that produces a *boolean-valid*
  cutter is accepted — the boolean never asks whether the fillet fits the
  adjacent faces.
- **Proposed fix (precise, local)**: compute the fit bound at EdgeMod apply
  time. For a straight edge with adjacent face normals n1, n2 on a box-like
  body, the maximum radius is half the smallest extent of the two adjacent
  faces along the directions perpendicular to the edge. Cheaper and still
  correct for the common case: compare `dist` against half the body's
  smallest AABB extent perpendicular to the edge direction (edge dir =
  normalize(p1 - p0)); reject `dist > bound` with
  `PARAMETER_INVALID` + "radius too large" reason. The kernel-side
  `rolling-ball` path already enforces this strictly (see the thin-plate
  case E5) — the fix only extends the same bound to the boolean-cutter
  fallback. Safest exact variant: when `EdgeBlend` native fails and the
  boolean fallback is about to run, require `2*dist ≤ min(adjacent face
  extents)` which the code already computes for cutter construction.
- **Test**: `characterization_oversize_fillet_gray_zone`,
  plus `fillet_radius_sweep_up_to_overflow` (no-panic sweep).

### E4. Tiny positive fillet radius fails noisily
- **Behavior**: `r = 1e-7` (positive, finite) fails the native path
  ("rolling ball" construction error) and warns, leaving the body unchanged.
  The mathematical limit r→0 is the identity operation, so the correct
  behavior is either success (no visible change) or a silent no-op.
- **Proposed fix**: treat `0 < dist < eps` (eps = e.g. 1e-4 mm, matching the
  tessellation quantization already used in `mass_properties`) as a
  successful no-op — return the input body unchanged with no warning,
  mirroring how `dist == 0` is already handled ("distance must be positive"
  is itself arguably wrong for 0; Fusion treats r=0 as remove-fillet).
  One-line bound check where the existing `dist > 0` guard lives.
- **Test**: `characterization_tiny_fillet_radius_warns_noisily`.

### E5. (Expected, pinned) Fillet radius larger than thin-plate thickness
- Correct current behavior: r=5 on a 2 mm-thick plate fails the rolling-ball
  construction, warns, leaves the body intact. Pinned as a regression guard;
  no fix needed. Test: `characterization_fillet_larger_than_thickness_warns_cleanly`.

### E6. Crossing ribs on a shared base plate fail to fuse (very common construction)
- **Behavior**: two perpendicular rib walls, both extruded (Join) from the
  plate's top face so that they cross each other, cannot be fused. The second
  Join fails: `"Join 'rib_2' could not produce one valid fused solid; the
  feature was not applied and its input bodies were left unchanged."` This is
  the classic ribbed-plate / I-beam-cross construction used everywhere in
  mechanical design.
- **Isolation** (tests `crossed_prism_bars_fuse_correctly` and the two
  characterization tests in `complex_extrusions.rs`):
  - Crossing prism bars WITHOUT a plate fuse correctly (analytic volume
    exact) — both as separate features and as two regions of one sketch.
  - A single rib joining a plate works (exact volume).
  - Two DISJOINT ribs on a plate in one feature work (exact volume).
  - The failure requires the crossing AND both prisms sitting coplanar on the
    same base face — i.e. the boolean must merge a T-crossing of side walls
    while re-stitching the shared coplanar top-face region of the plate.
  - Fails identically as two sequential features and as one multi-tool
    feature; in both cases the atomic rollback is CORRECT (the body is
    restored to its exact pre-feature volume — verified numerically).
- **Locus**: `zerocad-core/src/parametric/join.rs` — `apply_join` /
  `join_tool_into_body`: all three union variants (smooth / exact / dipped)
  fail validation for this configuration.
- **Proposed fix**: add a deterministic reconstruction path before the
  general boolean, in the same family as the existing
  `try_prismatic_profile_join`: when the body is a prism over a base plane
  (or a stack of coplanar-based prisms) and the Join tool is a prism based on
  the same plane, compute the union analytically as a 2-D footprint problem —
  union the tool footprint with each prism layer's footprint using the
  existing sketch arrangement code, then rebuild stepped prisms and sew once.
  This removes the boolean (and its coplanar re-stitching) entirely for the
  exact configuration that fails, and it is how the already-working
  multi-region single-plane cases succeed today. A smaller alternative: a
  cut-then-fuse fallback (fuse(cut(body, tool), tool)) which eliminates the
  coincident-wall topology the splitter chokes on — cheaper to add but less
  deterministic; both must stay behind the existing strict one-solid
  validation so a wrong reconstruction can never commit.
- **Tests**: `characterization_crossing_rib_join_fails_but_rolls_back_correctly`,
  `characterization_multi_tool_crossing_join_fails_and_rolls_back`,
  `crossed_prism_bars_fuse_correctly` (control), plus the positive
  `ribbed_mounting_plate` (ribs offset so they don't cross — full analytic
  volume).

### E9. FaceMove silently drops the tangential component
- **Behavior**: `FaceMove` with translation `[5, 0, 5]` on a box's top face
  produces 20×20×15 with NO warning — the 5 mm normal component was applied,
  the 5 mm tangential component was silently discarded. The feature's own doc
  comment says "a tangential component is rejected instead of approximated".
- **Proposed fix**: in the FaceMove apply path, compare the requested vector
  to the resolved face normal: if the perpendicular residual
  `|t − (t·n)n|` exceeds a small tolerance (e.g. 1e-4 mm), report
  `PARAMETER_INVALID`/unresolved and leave the body unchanged — the exact
  rejection the documentation already promises. Test:
  `characterization_face_move_silently_drops_tangential_component`.

### E10. FaceThicken consumes the source body
- **Behavior**: after thickening a box's top face by 4 mm, only the 1600 mm³
  plate remains as the feature's own body — the 4000 mm³ source box is gone,
  silently. The feature's doc comment says "The source remains in the
  document and the new feature owns the thickened result."
- **Proposed fix**: the FaceThicken apply path must not replace the target
  body's parts with the thickened plate; it must keep the target body's
  output untouched (like FaceOffset does — verified exact) and emit the plate
  as a new body owned by the thicken node. If the current consume behavior is
  intended, the doc comment needs updating instead — one of the two is wrong.
  Test: `characterization_face_thicken_consumes_source_body`.

### E11. Circular FeaturePattern always rolls back
- **Behavior**: a circular `FeaturePattern` replicating a hole fails with
  `"instance 1 failed (the instance missed the target or made no material
  change); the entire pattern was rolled back"` — on a clean bolt circle on a
  flat box cap (4 × r=2 holes at radius 10, all with material to remove) and
  on a cylinder cap bolt ring (6 × r=1.5 at radius 13 on a r=20 cylinder).
  Only the source hole applies. Linear patterns of the same holes work with
  exact volumes.
- **Root-cause hint**: "made no material change" suggests instance 1 is being
  re-applied at the SOURCE position (rotation transform not applied to the
  hole's captured position), so its cut intersects the already-cut source
  hole and removes nothing. Worth checking the circular instance transform
  construction in the FeaturePattern evaluator (`Instance 1` = first rotated
  copy).
- **Proposed fix**: apply the circular instance transform (rotation about the
  resolved axis) to the hole's captured position AND direction before
  rebuilding the instance cut; the "no material change" guard itself is good
  and should stay. Tests:
  `characterization_circular_feature_pattern_always_rolls_back_box`,
  `characterization_circular_feature_pattern_rolls_back_on_cylinder` (both
  flip to full-ring volumes when fixed).

### E12. Negative FaceOffset removes 1 mm more than requested
- **Behavior**: `FaceOffset` with distance −1 on a 10 mm-tall box yields
  height 8 (not 9); −4 yields 5 (not 6); −6 yields 3 (not 4) — a constant,
  silent +1 mm over-removal at every probed magnitude. Positive offsets are
  exact (verified +3 → 13).
- **Root-cause hint**: a fixed overshoot/grow constant (likely a 1 mm cutter
  growth used to avoid coplanar boolean caps) is being applied on the
  negative side where it changes the result instead of only the tool.
- **Proposed fix**: after the boolean, the rebuilt solid's moved face must
  sit at `original_face_plane + distance` exactly — verify the offset solid
  construction clamps its cutter cap to the requested plane rather than one
  overshoot past it. Test:
  `characterization_negative_face_offset_over_removes_by_one_mm`.

### E13. (Design observation) Bare geometric FaceRef rejects the whole document
- **Behavior**: a face selection carrying only centroid+normal (no durable
  topology name, no producer feature) is rejected by the document-level
  semantic gate with a hard `Err("Invalid semantic document: ... invalid
  semantic input 'face'")` — the entire document fails to evaluate, not just
  the one feature. Face selections captured from evaluated meshes (GUI path)
  carry topology names and pass.
- **Assessment**: fail-loud for corrupt data is defensible, but one malformed
  face ref anywhere takes down every body in the document, where the
  per-feature path (mark this feature Unresolved, keep everything else) is
  both available and consistent with how missing bodies are handled.
- **Proposed change**: treat a face selection with neither topology id nor
  producer feature as an unresolved feature (REFERENCE_MISSING diagnostic)
  rather than a document-level rejection; keep the document-level gate for
  structural corruption (duplicate ids, dependency cycles).
  Test: `characterization_bare_face_ref_rejects_whole_document`.

### E14. Circle cut tangent to all four box walls fails
- **Behavior**: a through-cut of a circle exactly inscribed in a box's top
  face (r = 20 in a 40×40 box — tangent to all four side walls) fails with
  `"the overlapping cut could not produce a valid solid. The original
  material was preserved"` and removes NOTHING. r = 19.9 removes the expected
  12 433 mm³ cleanly (verified). The resulting part (a square frame) is a
  completely ordinary geometry.
- **Root cause**: same exact-tangency boolean weakness as the tangency sweeps
  in `tangency_sweeps.rs`, now on the cutter boundary coincident with four
  target faces at once.
- **Proposed fix**: reuse the CUT_OVERSHOOT machinery that already exists for
  join (`join.rs::overshoot_cs`) on the cut side: grow the cutting prism's
  side walls by a sub-tolerance epsilon (e.g. 1e-3 mm) before the boolean so
  no cutter wall is exactly coincident with a target face, then the frame
  result falls out of the ordinary difference. The epsilon must stay below
  the mesh quantization so displayed geometry is unchanged.
- **Tests**: `characterization_inscribed_circle_cut_fails_on_tangency`,
  control `near_inscribed_circle_cut_applies`.

### E15. BodyCut's overlap precheck tests the tool's base plane, not the swept prism
- **Behavior**: `BodyCut` rejects a box tool whose SKETCH PLANE lies outside
  the target's AABB with `"the target and cutting body must overlap; both
  original bodies were left unchanged"` — even though the extruded prism
  crosses the rod with a ~785 mm³ overlap (tool sketched at z = −15, extruded
  to z = −5, against a rod spanning z −10..10). Isolation matrix:
  - tool plane inside the target's z-range, extending beyond it in x/y:
    **works** (even flush at y = 0);
  - tool plane below the target (z = −15): **rejected as disjoint**;
  - tool fully containing the target: rejected with a different, misleading
    `"the solver could not subtract the overlapping body"` instead of a
    "target fully consumed" outcome.
- **Root cause**: the overlap precheck intersects the tool's sketch plane
  (or base) with the target's AABB instead of the tool's swept volume —
  any miter-style cutter sketched outside the part's extent is falsely
  rejected.
- **Proposed fix**: compute the tool's actual AABB from its evaluated solid
  (`mock_kernel::solid_aabb` is already used elsewhere) and precheck
  AABB-vs-AABB with a small positive tolerance; or drop the precheck and let
  the guarded boolean decide the disjoint case (it already returns the
  target unchanged). Separately, detect the fully-contained case and report
  "cut consumes the entire target" explicitly.
- **Tests**: `characterization_body_cut_rejects_tool_based_below_target`,
  control `body_cut_with_plane_inside_target_applies`,
  `characterization_body_cut_consuming_target_misreports`.

### E16. A second overlapping cut removes nothing (silently for extrudes)
- **Behavior**: two overlapping r=4 through-cuts 5 mm apart should remove
  875 mm³; only ~503 mm³ (the first circle) is removed. The crescent of new
  material between the circles survives. The Hole variant at least warns
  ("the overlapping cut could not produce a valid solid"); the cut-EXTRUDE
  variant does the same thing with NO warning at all — a silent data-loss
  failure. Disjoint cuts both apply exactly (control test).
- **Root-cause hint**: cutting where a void already exists — the cutter
  prism's boolean difference intersects the previously cut region, and the
  guarded boolean's validity check rejects the partial-overlap difference
  (or the "no material change" fast path misfires because the first fraction
  of the sweep is already void).
- **Proposed fix**: the cut prism's difference against a body with an
  existing void must still commit the crescent. Verify the guarded boolean's
  precondition isn't "cutter fully inside material"; the boolean itself
  (OpenRCAD fuse/difference) handles partial voids — this looks like a
  precheck in the cut apply path. At minimum the extrude variant MUST warn
  like the hole variant does (the silent path is the worst part).
- **Test**: `characterization_overlapping_second_cut_removes_nothing` (both
  variants), control `disjoint_circle_cuts_both_apply`.

### E17. (Verified correct — pinned as regression) BodyCut/BodyIntersect result bodies are owned by the feature
- After a BodyCut or BodyIntersect, the result body's id becomes the
  *feature's* id (`cut_3`, `isect_3`) — later features referencing the
  original body id get "target body no longer exists" warnings. This is
  consistent ownership semantics, but callers must chain on the feature id.
  Pinned by the chained-cut and chained-intersect tests in
  `body_booleans_stress.rs`.

### E18. Flush coaxial plug does not fuse (silently)
- **Behavior**: a plug with EXACTLY the bore's radius (coincident cylindrical
  walls) joined into a tube leaves two separate volumes with NO warning —
  the union should be the solid rod. A plug with a 0.1 mm clearance gap is
  handled correctly (volumes conserved) though it also does not need to fuse.
- **Proposed fix**: this is the coincident-curved-face fusion gap (same
  family as the E1/E2 circular-edge matching). In the BodyJoin apply path,
  detect the coaxial same-radius case geometrically (axis lines coincide and
  |r1 − r2| < tol) and rebuild as a single cylinder solid analytically rather
  than boolean-fusing. At minimum, emit a warning (the current silence is
  the worst part).
- **Test**: `characterization_flush_coaxial_plug_join_does_not_fuse`,
  control `clearance_plug_join_conserves_volume`.

### E19. Thread with an unmatched face is a silent no-op
- **Behavior**: a Thread whose FaceRef matches no cylindrical face of the
  target (far-away centroid) leaves the body unchanged with NO warning — the
  documented "cosmetic fallback" warning path is not reached for unmatched
  faces. The user believes threads were modeled.
- **Proposed fix**: in the thread apply path, when the face reference fails
  to resolve against any cylindrical face, emit the cosmetic-fallback warning
  (the message text already exists for other fallback cases).
- **Test**: `thread_on_ghost_target_and_far_face_is_graceful` (thread_torture.rs).

### E20. A threaded body cannot be used as a boolean tool
- **Behavior**: `BodyCut` with a threaded rod as the tool against a plate it
  pierces fails with "the solver could not subtract the overlapping body;
  both original bodies were left unchanged" — warning present, volumes
  preserved. Imprinting threads by subtraction is therefore impossible.
- **Root-cause hint**: the threaded wall is a band solid; the boolean
  difference against the banded surface likely fails the same strict
  validation as the other E15b containment/complex-subtraction cases.
- **Proposed fix**: short-term, keep the graceful rejection (already
  correct) and document "threaded bodies cannot be cutters"; long-term the
  OpenRCAD difference needs to handle the helical band faces — same kernel
  work as E1/E2.
- **Test**: `threaded_rod_used_as_body_cut_tool` (thread_torture.rs).

### E21. One Extrude with two sketch parents silently drops the second sketch
- **Behavior**: an `Extrude` feature with dependency edges to TWO sketches
  evaluates successfully but extrudes only ONE of them — the body grows by
  exactly one boss (16800 mm³ instead of 17600), with NO warning and no
  document-level error. The second sketch is silently discarded.
- **Expected semantics**: one extrude should drive every sketch parent,
  extruding each sketch's regions from its own plane and combining per
  `mode` — exactly how a Loft already treats its multi-sketch `sections`
  (pinned legal by `one_loft_two_sketch_sections_is_legal`). Until that is
  implemented, the extra parent must at least fail loudly (semantic-gate
  rejection or an unresolved-feature warning), because silent data loss is
  the worst outcome.
- **Proposed fix** (two options, either is safe):
  1. **Capability**: in the extrude apply path, iterate every sketch-parent
     of the node instead of taking the single parent — build one JoinTool /
     cut prism per sketch (each with its own plane and regions) and apply
     them under the feature's existing atomic transaction (`apply_join`
     already accepts `Vec<JoinTool>`; the cut path likewise processes
     region lists). Update `feature_inputs_for_runtime` to register each
     sketch as a semantic input, mirroring Loft's section handling.
  2. **Guard (minimal)**: in the Extrude semantic contract, require exactly
     one sketch parent; extra sketch parents produce the standard
     "semantic inputs disagree" document error or a per-feature unresolved
     warning instead of silent drop.
- **Tests**: `characterization_one_extrude_two_sketches_drops_one_silently`
  (pins the silent drop), control `two_sketches_two_extrudes_works_exactly`,
  contrast `one_loft_two_sketch_sections_is_legal`, graceful variants
  `extrude_with_zero_or_non_sketch_parents_is_graceful`
  (`multi_sketch_extrude.rs`).

### E22. Crossing-line sketches cannot be cut (internal wire-splitter panics, caught)
- **Behavior**: a sketch of crossing lines (8 vertical × 8 horizontal forming
  a 7×7 grid of cell regions) fails to cut: the OpenRCAD wire splitter panics
  internally — `openrcad-topo/src/builder.rs:449/453: "split_face: V_A/V_B
  not found on outer loop"` — at every crossing vertex. The guarded boolean
  catches each panic (no crash — the `panic = "unwind"` + `catch_unwind`
  machinery works as designed), but every cell cut is then rejected with
  "the overlapping cut could not produce a valid solid" and NO material is
  removed. Closed line loops (rectangles, triangles, L-profiles) all work;
  the mid-segment crossing junctions are the trigger.
- **Impact**: any hatched / grid / mesh-like profile (vent grills cut as line
  grids rather than 49 explicit rectangles) cannot be extruded.
- **Proposed fix**: the splitter assumes both split vertices lie on the
  face's outer loop; when a crossing vertex lands on an INTERIOR wire (or
  the loop has already been split at that vertex), it must walk the wire set
  instead of only the outer loop. Short of kernel work: pre-split the input
  segments at every intersection point in the sketch arrangement
  (`openrcad-sketch` arrangement already computes these intersections) so
  the builder never sees mid-segment crossings.
- **Test**: `characterization` block inside `crossing_line_grid_regions`
  (`sketch_expr_torture.rs`).

### E23. Out-of-range region_indices are silently ignored
- **Behavior**: region selection itself WORKS (verified: `[0]` extrudes
  region 0 exactly, duplicate indices dedup, `[0]`/`[1]` on a two-circle
  sketch pick the right discs; an earlier probe suggesting selection was a
  no-op was the probe's fault — its sketch sat at z = 0 inside the box where
  a Join prism adds nothing). The genuine gap: indices that DON'T exist —
  `[1]` on a one-region sketch, `[1,7,99]` — apply nothing for the invalid
  entries, and an all-invalid list skips the whole feature with NO warning.
- **Proposed fix**: validate each index against the detected region count at
  extrude-apply time; emit a warning naming the skipped indices (or reject
  the feature as unresolved when NO index is valid), mirroring the
  "region N could not be built" messages that already exist for unbuildable
  regions.
- **Tests**: `region_selection_and_duplicates_work_exactly` (positive),
  `characterization_out_of_range_region_indices_are_silent` (the gap),
  `selective_region_extrude_extrudes_less_than_all` in
  `complex_extrusions.rs` (now asserts the real one-disc volume).

## C. Behavior verified correct (no action)

- **Extrude-on-face workflow** (19 tests): bosses, pockets, side-face
  extrudes, through-cuts, pocket-in-pocket chains, sketch-on-boss-top
  through-holes, overhanging sketches, floating planes, midplane cuts,
  sliver depth 1e-3, zero-floor pocket → clean through-cut, 10-deep nested
  pocket chains, draft on face bosses, open profiles, out-of-range region
  indices — all finite, deterministic, and analytically correct volumes.
- **Fillet/chamfer on straight box edges**: correct volumes (fillet r=3 →
  7960.45 vs analytic 7961.2; chamfer s=3 → 7910.0 exact), correct rejection
  of r ≤ 0 / NaN / r = 20, chamfer s = 40 rejected with warning.
- **3-edge corner networks (EdgeBlend)**: both r=3 and oversize r=10 succeed
  with sane volumes; empty/duplicate edge selections handled gracefully.
- **Stale edge references** (edge far from body, ghost target, NaN/zero
  length/garbage normals in EdgeRef): warn and leave the body untouched —
  the reattachment fallback works exactly as documented.
- **Sequenced fillets** (fillet-then-fillet, fillet-then-chamfer on adjacent
  / same edges, fillet after pocket, fillet after hole, fillet whose radius
  grows into an existing hole): all finite and deterministic.


## D. Research-pass findings (2026-09-11, `research_*_torture.rs`)

Second pass driven by an online survey of documented kernel failures (OCCT
boolean/fuzzy-boolean specs and tickets 0025771/0029034/0023025, FreeCAD
t=1067/t=21969/t=99516 and issues #12119/#22721/#29401, OpenSCAD
#1591/#4039/#6116 and PR #5012, Blender T67744, SolidWorks shell/chamfer
diagnostics docs, Onshape shell/fillet threads, Hoffmann's "Robustness in
Geometric Computations", CGAL PMP repair taxonomy, bincode/serde format
spec). New suites: `research_boolean_metamorphic.rs`,
`research_blend_shell_torture.rs`, `research_variables_history_torture.rs`,
`research_exchange_torture.rs`.

### R1. Through-cut fails in a narrow band right before breakthrough (graceful)
- **Behavior**: a circle cut from the top of a 10 mm plate at depth 9.9,
  9.99, 9.999, and 9.9999 removes ≈785 mm³ correctly — but depth **9.99995**
  (5e-5 short of the far face) applies NOTHING with the warning "the
  overlapping cut could not produce a valid solid. The original material was
  preserved". The full through-cut (10.0) works. This is the local analogue
  of OCCT's fuzzy-boolean Example 4.1 (near-coincident back faces leaving a
  5e-5 membrane).
- **Assessment**: the failure is graceful (loud warning + exact rollback) and
  the band is narrow, but a user extruding "almost through" by a
  mis-typed expression gets a silently intact plate minus one warning.
- **Proposed fix**: same family as E14 — grow the cutter's far cap past the
  target's far plane by a sub-tolerance epsilon when the cut ends within it,
  or clamp cut depths within 1e-4 of breakthrough to the through case.
- **Test**: `through_cut_fails_only_in_the_5e5_breakthrough_band`
  (`research_boolean_metamorphic.rs`).

### R2. BodyCut that fully contains the target preserves BOTH bodies
- **Behavior**: a BodyCut whose tool encloses the target reports the
  overlapping-subtraction failure and rolls back with both bodies present —
  total 9000 = target 1000 + tool 8000 — even though `keep_tool: false`. The
  rollback message matches E15c; this pins the volume side.
- **Assessment**: graceful and deterministic, but "consumed the target" is
  misreported as a solver failure and the tool leaks back into the document.
- **Proposed fix**: detect the containment case and either consume
  (explicit "target fully consumed" outcome) or keep the tool only when
  `keep_tool` is set; never report a containment as a generic subtraction
  failure.
- **Test**: `body_cut_consuming_entire_target_stays_graceful`.

### R3. Curved-tool union deviates from the volume law by ~3.6 mm³ (0.06%)
- **Behavior**: for box ∪ cylinder the cut and intersect partition the box
  EXACTLY (3609.611 + 390.389 = 4000.000), but the fused volume misses
  `A + B − I` by ≈3.6 mm³ — within curved-wall chord re-segmentation noise
  (the fused cylinder wall is re-tessellated differently than the standalone
  cylinder), so this is characterized with a 0.5%-of-tool tolerance rather
  than pinned as a bug. Prismatic tools satisfy all three volume laws to
  <0.5 mm³ (see `metamorphic_volume_laws_union_intersect_cut`).
- **Test**: `metamorphic_union_intersect_cut_curved_tool`.

### R4. Behavior verified correct (no action) — research pass
- **Metamorphic volume laws** for prismatic tools (join/intersect/cut),
  three-body union order-independence, volume cubing under uniform scale
  (0.25×–4×), 45° frame invariance, and cut-volume invariance at 1e5 mm from
  the origin.
- **Cutting nothing**: a cut tool fully inside a shelled cavity is a
  clean no-op; near-miss cuts 1e-5 off a face remove nothing (no "pin"
  protrusion — OCCT fuzzy Example 4.3 class); vertex-touching three-cube
  chains conserve material.
- **Variables/expressions**: circular (`a=b+1, b=a+1`) and self-referential
  variables terminate without hanging; division-by-zero expressions are
  rejected at the variable layer (the NaN never reaches the kernel — the A2
  class is not reachable through expressions); dangling identifiers warn;
  a 200-deep variable chain resolves exactly.
- **Shell/offset/thread boundaries**: shell thickness above the cylinder
  radius, at the box half-min-extent, all-six-open-faces, two-opposite-face
  openings — all graceful and deterministic; face offset −50 beyond the body
  is graceful; counterbore diameter == bore, blind depth == plate thickness,
  thread pitch below band depth, partial-turn thread ends, `starts` 0/1/4/64,
  internal thread bands beyond the host hole, and a 300-turn fine-pitch
  thread all stay finite and deterministic.
- **Exchange**: the STL reader rejects NaN/inf vertices, discards degenerate
  facets with diagnostics, flags duplicate-face and shared-wall
  non-manifold edges and inconsistent winding, parses binary STLs with
  "solid" headers via the size formula, and types every truncation/empty
  case; garbage/header-only/unknown-schema STEP warns cleanly and never
  takes down unrelated bodies.
- **Document format**: `.zcad` round-trips bit-identically with unicode and
  250/251-char ids; 64 truncations + 200 deterministic single-byte bit-flips
  never panic on load or evaluation (corrupt-but-parseable payloads land in
  the same parameter guards as direct input); the empty document round-trips.

## Implementation notes for the fixes

All four A-fixes follow one pattern — a finite/positive-parameter guard at
feature-apply time in `parametric/eval.rs` that converts a would-be panic
into `ResolutionState::Unresolved` + `PARAMETER_INVALID`. The codebase
already has this machinery ("must be positive" guards for fillet distance,
zero-depth extrude warnings), so each fix is a few lines at the existing
guard sites and cannot change behavior of valid inputs. The E3/E4 fixes are
bound checks in the same EdgeMod apply path. E1/E2 are the only genuinely
new work (circular-edge hint matching) and are additive: the failure path
already degrades gracefully, so landing the matcher later changes no current
behavior.

## Catalog campaign (2026-09-13, plan §6 categories A–J)

New suites `catalog_a_numeric.rs` … `catalog_j_patterns.rs` implement the
adversarial plan's scenario catalog against the real APIs. Categories K–P
(reference/lifecycle matrices, cancellation lanes, byte-level container
corruption, exchange, assemblies, viewport) are still open — see README.

### Fixed during the campaign

- **A03 sub-tolerance box panic (FIXED)** — `box_solid` panicked on
  positive-but-sub-tolerance dimensions (`10×10×1e-7`, subnormals): the
  kernel's tolerance model collapses the edges (`DegenerateEdge`) and the
  constructor `.expect()`ed validity. `box_solid` now returns `Option`
  (like `build_cylinder_solid`), the Box feature arm reports a typed
  `parameter.invalid` ("dimensions are below the modeling tolerance"), and
  `MockMesh::make_box` falls back to an empty mesh.
  Regression: `a03_sub_tolerance_magnitudes_are_exact_or_rejected`,
  `a03_subnormal_box_dimension_terminates_classified`.

### New tracked defects (`#[ignore]`d known-breaks)

- **A-display-range** — huge-but-finite bodies (Box 10e37-wide, BodyScale
  ×1e37) tessellate into f32 `MockMesh`es with non-finite vertices; the
  f64 B-Rep is valid, the display boundary has no finiteness gate.
- **E-overlap** — two PARTIALLY OVERLAPPING bore circles (0 < s < 2r)
  silently remove only the base circle (~502 of ~1005 mm³), even with every
  region explicitly selected; disjoint and concentric pairs are exact.
  Overlap-cluster tool-lens rules in `boolean_region_plan`.
- **E-tangency** — two bores at EXACT tangency drop one bore of the pair
  with no diagnostic (same cluster family).
- **I-tangent-hole** — a hole EXACTLY tangent to the plate edge removes
  NOTHING, silently (4.001 cuts the full bore, 3.999 breaks out correctly).
- **G-fillet-shell** — shelling a filleted box warns "couldn't hollow this
  body" while the body IS hollowed: failure diagnostic + changed geometry
  (partial-transaction shape). Related capability note: hollowing supports
  only pristine box/cylinder solids — any prior boolean makes it refuse
  (attributed, safe).
- **G-tangential** — a tangential `FaceMove` is a silent no-op; the
  documented contract says tangential components are "rejected", and a
  rejection needs a diagnostic.
- **H-phantom-loft** — coincident loft sections build a PHANTOM body (an
  evaluated body entry with no solid mass properties) with no warning.
- **H-closed-sweep** — a sweep around a closed square path loses ~one leg
  of the loop (≈709 of ≈1005 mm³) with no diagnostic.
- **J-mirror-chain** — a second, perpendicular mirror pattern on a mirrored
  body does not double the material (2000 instead of 4000) with no
  diagnostic.

### Behavioral pins (characterizations, active tests)

- Draft beyond ~45° flips the taper direction (removal grows again).
- Whole-sketch cuts of overlapping circles follow tool-lens semantics
  (base circle only); nested circles extrude the full disk ("island").
- Pattern-derived overflow (count×spacing > f32) rolls back atomically with
  the source preserved, classified `operation.failed` rather than
  `parameter.invalid` (J-face-budget also notes the display-budget rejection
  is an untyped plain warning).
- Coincident circular pattern instances at 0°/360° double-count in
  part-based volume.
- Island removal via BodyCut on a multicomponent body is an attributed safe
  rejection (capability gap, E-island); sliver cuts of 1e-3 are exact and
  sub-tolerance slivers stay within the ~±0.1 mm³ tessellation noise floor.
