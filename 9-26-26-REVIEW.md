# ZeroCAD Project Review — 2026-09-26

Scope: the whole repository at `4e37ce27` plus the working tree (ZeroCAD core, GUI,
break-test, and the vendored OpenRCAD kernel). The review focuses on **Join and Cut
reliability**, because that is where you are running into problems. Every other area
is also rated, and each area is compared with how **Autodesk Fusion 360** does the same job.

Method: static reading only. No code was edited, and nothing was built or run. Each
finding is labeled:

- **CONFIRMED**: I read the code path and the defect or behavior follows directly from it.
- **LIKELY**: the code strongly implies it, but a short repro test should confirm it
  before anyone acts on it.
- **OBSERVATION**: a design or quality point, not a defect.

Scale: 1–10, using the same rubric as `rating.md` (1 = nothing demonstrated,
5 = functional foundation with reliability limits, 7 = strong with identifiable gaps,
9 = professional-grade).

Codebase size at review time: zerocad-core ≈ 85k lines of Rust (plus 17k in integration
tests), zerocad-gui ≈ 45k, OpenRCAD ≈ 82k, break-test ≈ 19k. There are about 2,000 `#[test]`
functions and only 1 `#[ignore]`.

---

## 1. Executive summary

ZeroCAD is an unusually ambitious and unusually *honest* project. It has its own Rust
B-Rep kernel, atomic feature transactions, typed diagnostics, and fixture-driven
regressions. Its evidence documents state their limits instead of claiming parity. None
of that is common in hobby or early-stage CAD.

The Join and Cut problems you keep hitting are not random. They come from **five
structural causes**, and each fix doc in `docs/` from the last month is a local patch
over one of them:

1. **Mixed precision.** The application layer (sketch regions, coordinate frames,
   depths) is `f32`. The kernel is `f64`, with *absolute* tolerances of 1e-5 mm for
   intersection and sewing. Above roughly 128 mm from the origin, a single f32 rounding
   step (1.5e-5) is larger than the kernel's sewing tolerance. Your own fix docs describe
   this failure mode repeatedly: "radius differs by a few float32 ULPs", "pcurve
   deviation 1.9e-6", "frame translation is calculated in f64", "rounding the translated
   origin to f32 can separate coincident profile edges by several microns".
2. **Coincident geometry has no general solution in the kernel,** so the app layer grew
   a **7-stage fallback ladder** for Join and a **5-stage ladder plus a reverse direction**
   for Cut. The ladders rely on scale-blind 0.1 mm nudges.
3. **The robust paths only apply to fresh sketch prisms.** The robust "sectional" fast
   paths need a retained `sketch_source`. Fillet, chamfer, shell, draft, direct edit,
   pattern, and body operations all clear it. After any of those, every later Join or
   Cut falls through to the fragile general Boolean. In practice: *it works on a fresh
   block and fails after you fillet something.*
4. **Derived state is invalidated by hand.** The retained `sketch_source` has to be
   cleared manually at every mutation site. At least one site, threading, does not clear
   it. The sectional paths then **rebuild the body from those stale prisms** and discard
   the real B-Rep, with no check that the prisms still describe the solid.
5. **Missing Fusion-style controls push users into fragile configurations.** Extrude has
   no Through All, To Object, Symmetric, Two Sides, or Start Offset, and no
   participating-bodies list. Users therefore model cuts as "exact depth flush with a
   face", which is the hardest case for the Boolean. The only other choice is
   origin-plane sketches that hit every nearby body.

### Headline scores

| Area | Score |
|---|---:|
| **Join/Cut reliability (overall)** | **4.5** |
| Kernel Boolean core | 5 |
| Precision & tolerance architecture | 3 |
| Failure safety (atomic rollback, never corrupt) | 8.5 |
| Diagnostics quality | 7 |
| Code architecture & maintainability | 5 |
| Test suite breadth | 8 |
| Test suite *feedback loop* (speed, isolation) | 3 |
| Engineering process (commits, branching) | 3 |
| Documentation & evidence honesty | 8 |
| Feature breadth vs Fusion 360 (part design) | 4 |
| Runtime footprint / native performance vs Fusion | 8 |
| **Overall project** | **5.5** |

---

## 2. Join and Cut: detailed findings

### 2.1 How a Join currently executes

`apply_join` (`zerocad-core/src/parametric/join.rs:242`) tries the following, in order:

1. `merge_adjacent_join_profiles` (join.rs:311) merges selected regions, but only if
   every tool shares the same frame **and** the same depth.
2. `try_prismatic_profile_join` (join.rs:844): the whole-body sectional path, for a boss
   on the far cap of a single-region sketch prism.
3. `profile_join::try_profile_join` (profile_join.rs:118): rebuilds the whole body from
   the retained prisms.
4. Per tool, `try_prismatic_boundary_join` (join.rs:1027): two prisms sharing a profile
   boundary.
5. `profile_join::try_cap_join` (profile_join.rs:301).
6. The general Boolean with the `smooth` analytic cylinder variant.
7. The general Boolean with the `exact` prism.
8. The general Boolean with the `dipped` prism (near cap pushed 0.1 mm into the body),
   accepted only with a recovery certificate.

Cut (`cut.rs:1168`) follows the same pattern: `profile_cut::try_profile_cut` →
`try_prismatic_profile_cut` → per part, `cap_pocket::cut` → `smooth` → `exact` →
`external-only box` → `expanded`. Each of those can also run in the **reverse direction**.

**Fusion 360 comparison.** Fusion calls the Autodesk Shape Manager (ASM) kernel once per
participating body. ASM has **tolerant edges and vertices**, where each entity carries its
own tolerance, and it resolves coincident faces natively. Fusion has no nudge variants
and no reversed-direction retries. When ASM fails, the timeline item turns red with a
specific error. Fusion never quietly substitutes a slightly different tool.

### 2.2 Findings

#### J1. Threading leaves a stale `sketch_source`, and later sectional Join/Cut can erase the thread (LIKELY, high severity)

- `thread_one` replaces a part at `eval.rs:7211` (`body.parts[pi] = threaded.solid`) and
  never clears `body.sketch_source`.
- Shell clears it explicitly and explains why (`eval.rs:6703-6705`): *"Reusing the
  original solid prism for a subsequent sectional cut/join would silently fill the cavity."*
  Threading has the same hazard.
- Both `profile_cut::try_profile_cut` (profile_cut.rs:114) and
  `profile_join::try_profile_join` (profile_join.rs:118) rebuild the result **from the
  retained prisms**. They never read the body's actual faces. Nothing checks that those
  prisms still equal the current solid.
- Why no test catches it: the thread tests (`parametric/tests/thread_join_head.rs:10`)
  build the rod with the `Cylinder` primitive, which carries `sketch_source: None`
  (`eval.rs:2732`). A **sketch-extruded** circular rod keeps its source.
- Repro to write: sketch a circle → extrude → thread the wall → sketch on the rod's end
  face → Cut a small pocket (or Join a head). Expected: the threads survive. Suspected:
  a valid, watertight, *unthreaded* rod with the pocket.
- Fix: clear `sketch_source` in `thread_one` today. For the longer term, see A2
  (fingerprint the source against the solid it describes).

#### J2. Retained-source invalidation is manual and spread across about 20 sites (CONFIRMED, design risk)

`sketch_source = None` is written by hand in edge_mod.rs (5 sites), eval.rs (8), join.rs,
cut.rs, draft.rs, body_ops.rs, and direct_edit.rs. Any future feature that mutates `parts`
and forgets that line reproduces J1. The type system gives no help: `parts` and
`sketch_source` are independent public fields on `LiveBody`.

#### J3. After any fillet, chamfer, shell, draft, or direct edit, all later Joins and Cuts lose the robust path (CONFIRMED, explains "works sometimes")

Every blend clears the source (`edge_mod.rs:209, 376, 1340, 1427`). From then on, every
Join or Cut on that body goes to the general 3D Boolean. That is exactly the path the
fix docs describe as failing on coincident faces, pcurve deviation, and Euler errors. The
fast paths therefore make the *first* operations reliable and leave later ones fragile.
This matches a "sometimes it works, sometimes it doesn't" experience.

Fusion does not work this way: every operation reaches the same kernel code, so
reliability does not depend on history order.

#### J4. Cut rolls back the whole feature if any bystander body's AABB is within 0.05 mm (CONFIRMED)

In `apply_cut` (cut.rs:1347-1356), when a part's cut returns `None`,
`failed_on_overlap` is set if *either* direction's AABB overlaps that part with a 0.05 mm
margin (`aabbs_overlap(p, t, 0.05)`). The whole cut then reverts for **all** bodies
(cut.rs:1397-1404).

`cut_part_one_dir` returns `None` for a tangent or near-miss tool (cut.rs:328-341), which
is a legitimate miss. The outer loop still counts it as an overlap failure.

Scenario: an origin-plane sketch (target `None` → every body is a candidate) cuts body
A, while body B sits flush against the tool, or within 0.05 mm of it. Body B returns
`None`, and the cut on A is discarded with *"the overlapping cut could not produce a
valid solid"*.

Fix: decide "genuine failure" with the same exact-overlap test that `cut_part_one_dir`
uses. Better, run a real volume-overlap check (Common or signed-distance probes), not
AABBs.

Fusion comparison: the preview fills in "Objects to Cut". A body merely touching the tool
is not listed, and one body failing never cancels the cut on another.

#### J5. Face-attached sketches can only Join or Cut the body they sit on (CONFIRMED, UX gap causing "cut does nothing")

The GUI (`zerocad-gui/src/extrude.rs:1152-1163` and `:717-724`) sets `target` to the
body that owns the sketch face. The inspector shows the target read-only. Two common
workflows are therefore impossible:

- Sketch on body A's top face and cut **through into body B** beneath it.
- Sketch on body A and join a boss that should fuse into **body B**.

Origin-plane sketches have the opposite problem: they hit *every* overlapping body.

Fusion's Extrude has an **Objects to Cut / Participating Bodies** list, auto-filled from
the preview and editable with checkboxes.

#### J6. A Join that bridges two separate bodies fuses into only one of them (LIKELY)

`apply_join`'s general loop (join.rs:279-294) merges each tool into the **first** body
that accepts it and then `break`s. `union_variant_into_parts` merges only the parts
*within* that body. If a join block bridges body A and body B with no target (an
origin-plane sketch), you get A∪tool as one body and B still overlapping it, unfused.
That is the "overlapping, unfused solids" state the comment at join.rs:275-277 says it
wants to prevent.

Fusion joins the tool with **all** participating bodies into one body.

#### J7. The `expanded` cut fallback is not a real offset, and its certificate can't detect that (CONFIRMED mechanism, LIKELY symptom)

- `grow_loop` (join.rs:183) *scales* the loop about the **vertex-average** centroid. It
  does not offset it. For convex profiles that is roughly an offset. For concave profiles
  (U, L, slots, text) the inner walls move **away from where they belong**. Scaling a U
  by f > 1 widens the gap between its arms, so the expanded tool does **not** contain the
  exact tool near those inner walls. Holes are shrunk about their own centroids, so
  islands lose material non-uniformly.
- The vertex-average centroid is biased toward densely sampled arcs.
- `certify_expanded_cut` (recovery_certificate.rs:132-198) checks containment with an
  **AABB**, **one probe point**, and a **total volume computed from tessellation**. A
  U-shaped expanded tool passes all three. Tessellation-derived cavity volume is the same
  measurement `rating.md` flags as untrustworthy (37,248 vs 15,744 mm³).
- Likely symptom: a thin 0–0.1 mm skin left on an inner pocket wall, or a later fillet
  failing on that wall.
- Fix: build `expanded` with the sketch offset you already have (`sketch/offset`), make
  the displacement relative to model size, and certify with exact B-Rep volume and
  several probes along every tool wall. Alternatively, delete `expanded` once coincident
  handling in the kernel improves.

#### J8. `cap_pocket::cut` returns an unvalidated solid (CONFIRMED)

`cap_pocket.rs:143` returns `Some(KernelSolid::new(shell))` without checking watertight,
health, strict validation, or containment. `cut_part_one_dir` accepts it immediately
(cut.rs:343-351). Every other fast path validates. The acceptance gates are inconsistent
between paths:

| Path | Watertight | Healthy | Strict validate | Connected | Containment | Volume check |
|---|:-:|:-:|:-:|:-:|:-:|:-:|
| `try_prismatic_profile_join` | ✓ | ✓ | ✓ | ✓ | AABB | ✗ |
| `try_prismatic_boundary_join` | ✓ | ✓ | ✗ | ✓ | AABB | ✗ |
| general join (`valid_union_result`) | ✓ | ✓ | ✗ (kernel-side only) | ✓ | AABB | ✗ |
| `profile_cut` | ✓ | ✓ | ✓ | split | ✗ | ✗ |
| `cap_pocket` | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ |
| general cut | kernel | kernel | kernel | split | ✗ | ✗ (mesh-signature "changed") |

Fix: route every candidate through one function, `accept_boolean_candidate(op, inputs,
result)`, that enforces all of these, including an exact-volume inequality:

- Join: `max(Va, Vb) ≤ V ≤ Va + Vb`
- Cut: `Va − Vb ≤ V < Va`

This one gate would catch J1, J7, and several failures that are still undiscovered.

#### J9. "Did the cut change anything?" is decided by tessellating and hashing (CONFIRMED, perf + correctness)

`cut_parts_changed` (cut.rs:513-550) builds a full `MockMesh` of the input *and* of every
result, then compares vertex counts, bounds, and quantized sums. Problems:

1. It costs two full tessellations per candidate on the hot preview path.
2. Retessellation noise can register as "changed" (the comment at cut.rs:1336-1339
   admits this happened with a flush reverse cap).
3. A real cut that happens to keep the same counts and quantized sums reads as unchanged.

An exact B-Rep volume delta, or the kernel's own face-history "tool faces kept > 0",
would be both cheaper and correct.

#### J10. Cut silently reverses direction (CONFIRMED, intent risk)

If the drawn direction's AABB has zero overlap and the reverse has some, the reverse is
used (cut.rs:1307-1308). The saved document keeps the original signed depth.

After an upstream edit (the body moves or gets thicker), the same feature can flip which
side it cuts **without the user knowing**. That is non-deterministic intent. The profile
path uses a slightly different rule (z-range overlap, `profile_cut.rs:150-167`), so the
two paths can disagree on direction for the same input.

Fusion never flips. The preview arrow shows the direction, and a cut into air produces
"No target body found to cut".

Fix: flip once in the *GUI* at creation time, and store the resolved direction. At
evaluation time, a miss should be a clear diagnostic, not a flip.

#### J11. Scale-blind constants throughout the Boolean glue (CONFIRMED)

| Constant | Where | Problem |
|---|---|---|
| `CUT_OVERSHOOT = 0.1 mm` | join.rs:149 | 20% of a 0.5 mm feature; invisible on a 2 m part, but also too small to break coplanarity there after f32 rounding |
| `CUT_WALL_GROW = 0.1 mm` | join.rs:153 | same |
| AABB overlap margin `0.05` | cut.rs:317, :1280, join.rs:1141, :1206 | absolute, and drives J4 |
| `SECTION_TOLERANCE 1e-4` / `1e-5` | join.rs:845, :1069 | two different values for the same concept in the same file |
| `frames_match` 1e-5 | join.rs:806 | an f32 frame far from the origin can never match at 1e-5 |
| profile_join tolerance | profile_join.rs:151-160 | `scale·ε·8`, capped at sewing 1e-5, so it saturates at about 10 mm model size |

Fusion (ASM) works internally in double-precision centimetres with one modeling
resolution, plus tolerant entities. Nothing at the feature level hard-codes millimetre
constants.

#### J12. The f32 application layer is the recurring root cause (CONFIRMED, architectural)

- `geometry::Vec3` and `CoordinateSystem` are f32 (`zerocad-core/src/geometry.rs:2,81`).
- `Region.boundary` and holes are `Vec<(f32, f32)>` (`sketch.rs:1495-1501`).
- `Extrude.depth` is `f32` (`types.rs:733`).
- Kernel `TolerancePolicy::STANDARD` uses **absolute** values: intersection, sewing, and
  classification are all 1e-5 (`OpenRCAD/crates/openrcad-foundation/src/tolerance.rs:51-64`).

| Distance from origin | f32 ULP | vs 1e-5 sewing tolerance |
|---|---|---|
| 10 mm | 9.5e-7 | 10× margin |
| 100 mm | 7.6e-6 | ~1.3× margin (one `unproject` = 3–4 roundings, which already exceeds it) |
| 200 mm | 1.5e-5 | **exceeds tolerance with one rounding** |
| 1000 mm | 6.1e-5 | 6× over |

`cs.unproject(u, v)` computes `origin + u·U + v·V` in f32, so every point picks up several
roundings. Parts modeled away from the origin, or large parts, are exactly the "some
scenarios" pattern. The kernel has `snap_relative`/`snap_max`, but those only snap
*planar tool faces* in the pre-snap (boolean.rs:780-789). They do not fix edge and pcurve
mismatches.

Fusion is double precision end to end.

#### J13. Two representations of every profile (CONFIRMED, design)

Each `Region` carries both a sampled f32 polyline (`boundary`) and an optional analytic
f64 description (`analytic`). Code paths choose one or the other:

- `source_region` (join.rs:773) rebuilds analytic data from 4-point polylines.
- `loop_to_wire` refits circles out of polylines (README: "refits co-circular runs").
- `merged-polygon-join-fix.md` exists because one path *discarded* the analytic loops.

Whenever the two representations disagree, the sectional paths and the general Boolean
see different geometry. Fusion's sketch profiles are exact curves only; tessellation is
display-only.

#### J14. Kernel coincident-face classification uses one sample per fragment (CONFIRMED, OBSERVATION)

`boolean_impl` (`OpenRCAD/crates/openrcad-algo/src/boolean.rs:760`, 586 lines) splits
edges and faces, then classifies **each fragment by one point** (`point_on_face`):

- For coplanar faces, it tests whether that single point lies inside the other face's
  trimming loops (boolean.rs:1128-1152, 1215-1229).
- Otherwise it runs 5-direction ray-parity voting (boolean.rs:2242-2294).

This is the classic design, and it is correct *only if the splitting was complete*. If
one intersection curve is missed, a whole fragment is misclassified. The result is then
non-watertight, the Boolean is rejected, and the app layer runs the next fallback. That
is the mechanism behind the long chain of "Euler −15 / free edge" fix docs.

Standard improvements:

1. **Classification propagation.** Classify one seed fragment per connected region, then
   flood-fill across edges that are *not* intersection edges. That is O(1) ray casts per
   region instead of per face, and it is self-consistent by construction.
2. When propagation hits a contradiction, treat it as a *splitting* bug and report which
   edge caused it. Don't fall through silently.
3. Use exact or adaptive orientation predicates for the planar–planar case, which covers
   most mechanical parts.

Also noted: `f_obj.surface().unwrap()` (boolean.rs:880-881) and 136 `unwrap`s plus 101
`expect`s in openrcad-algo. Panics are caught with `catch_unwind` and used as a recovery
path, which works but hides the real reason for a failure.

#### J15. Failure diagnostics are good but costly (OBSERVATION)

`join_failure_detail` (join.rs:379) runs up to 3 extra Booleans per body (union, Common,
face-area overlap) *after* a failure. That work happens inside the evaluation, and so also
inside previews. Messages like *"touches the body only along an edge"* are better than
Fusion's generic errors, which is a real strength. Consider computing the details lazily,
only when the user opens the warning.

#### J16. Dead or misleading code in the Boolean path (CONFIRMED, low severity)

- `directional_cut` (cut.rs:288) is now the identity function `(*cs, depth)`.
- `quiet_panic` (boolean.rs:62) is a no-op wrapper. The README still documents it as the
  catch-unwind boundary.
- `solids_overlap_with_volume` (cut.rs:261) is an **AABB** test, and it returns `true`
  when the AABB is unknown.
- The root `Cargo.toml` comment still says catch_unwind is needed for "truck's
  boolean-solver panics". The kernel is OpenRCAD now.
- `FeatureType::EdgeMod`'s doc (types.rs:758-761) still says "applied as a guarded boolean
  subtraction of an edge-aligned cutter". It is now native rolling-ball.

### 2.3 Join/Cut sub-scores

| Sub-category | Score | Fusion (est.) | Notes |
|---|---:|---:|---|
| Atomicity / never corrupting the model | 9 | 8 | Atomic rollback is excellent; Fusion also leaves failed features in place |
| Failure messages | 7 | 6 | Better than Fusion's generic errors (J15) |
| Fresh sketch-prism Join/Cut | 7 | 9 | Sectional paths are strong here |
| Join/Cut after fillet/chamfer/shell | 3 | 9 | J3 |
| Coincident/flush faces (general path) | 4 | 9 | J14, the fallback ladder |
| Models far from the origin / large parts | 3 | 9 | J12 |
| Very small parts (<1 mm features) | 3 | 8 | J11 |
| Concave or complex cut profiles | 4 | 9 | J7 |
| Multi-body targeting | 3 | 9 | J4, J5, J6 |
| Direction/intent determinism | 4 | 10 | J10 |
| Derived-state safety | 3 | n/a | J1, J2 |
| Acceptance-gate consistency | 4 | n/a | J8 |
| Boolean path performance | 5 | 8 | J9, J15, and up to 6 tool variants per region |
| Regression evidence for fixed cases | 8 | ? | Saved fixtures, cold/warm/reopen, volume checks |
| **Overall Join/Cut** | **4.5** | **9** | |

---

## 3. Kernel (OpenRCAD)

| Category | Score | Notes |
|---|---:|---|
| Topology data model (arena B-Rep, builder) | 7 | Clear arena model; orientation-aware accessors are a known trap (see your edge-orientation memory note), so the API has sharp edges |
| Analytic surface coverage | 6 | Plane, cylinder, cone, torus, sphere, ruled, BSpline, helix: good for mechanical parts |
| Boolean engine | 5 | See J14. 3.2k lines, one 586-line function |
| Sewing & healing | 6 | Validated T-junction healer, recently made boundary-only (a good fix); still reactive |
| Blends (rolling-ball) | 5 | `rolling_ball.rs` is **9,742 lines** in one file; functions of 472, 374, and 340 lines. Lots of capability, very hard to reason about |
| Tolerance model | 3 | Absolute-only `STANDARD` policy; no per-model scaling; no tolerant entities |
| Validation (strict, Euler, pcurve) | 8 | Strong and consistently used inside the kernel |
| Panic hygiene | 4 | Recovers through catch_unwind; about 240 unwrap/expect in algo+topo |
| Test coverage | 7 | 589 tests in the kernel workspace per the docs |

**Fusion (ASM) comparison:** decades-mature kernel with tolerant modeling, exact
intersection curves, persistent entity IDs, and a separate healing stage for imports.
ZeroCAD's kernel is roughly where early ACIS or Parasolid releases were. It is sound in
structure, and a long tail of degenerate cases remains to be closed.

**Top kernel recommendations**

1. Add a `TolerancePolicy::for_model(bbox)` that scales `intersection`, `sewing`, and
   `classification` with model size, with a floor tied to f64 resolution. Use it at every
   entry point.
2. Classification propagation (J14).
3. Split `rolling_ball.rs` into cohesive modules (spine, contact, corner, trim, flow)
   **only** behind the existing geometry tests. The AGENTS.md guidance on hotspots still
   applies.
4. Replace `unwrap()` on `surface()` and `pcurve()` in the Boolean with typed errors, so
   failures explain themselves instead of becoming generic "rejected" results.

---

## 4. Architecture and code quality

| Category | Score | Notes |
|---|---:|---|
| Crate boundaries (core has no GUI deps; kernel separate; render isolated) | 8 | Genuinely good separation |
| Module map & AGENTS.md navigation | 8 | Useful for humans and agents |
| Function size / complexity | 3 | `apply_extrude` **1,218 lines** (eval.rs); `invoke_registered_feature` 606; `build_live_with_cancel_traced` 410; `apply_pattern` 388 |
| File size | 4 | eval.rs 8.8k, primitives.rs 4.5k, zcad_format.rs 4.1k, sketch.rs 3.5k lines |
| Derived-state invariants | 3 | J2. Also `node_map` serde-skip desync (your memory notes it) is the same class of bug |
| Representation duplication | 4 | J13 (polyline plus analytic) |
| Naming/comment accuracy | 6 | Comments are generous and mostly right; stale ones listed in J16 |
| Error handling | 5 | Diagnostics are typed at the feature level; 580 unwrap + 505 expect in zerocad-core (many are in tests) |
| Numeric types | 3 | J12 |
| Serialization / format evolution | 7 | Versioned `.zcad` container with separate recipe and cache ABIs. The cache ABI moved 5→15 in one month, which is fine by design (derived data), but shows how often geometry semantics change |

### Recommended refactors (in order of payoff)

1. **Make `LiveBody` enforce its own invariant.** Make `parts` private. Expose
   `replace_parts(parts, SourcePolicy)` where `SourcePolicy::{Invalidate,
   KeepVerified(SketchExtrudeSource)}`. `KeepVerified` stores a cheap fingerprint of the
   solid (exact volume, face count, bbox) and is rechecked before any sectional rebuild.
   That removes J1 and J2 as a class.
2. **Split `apply_extrude`** into:
   - `ExtrudeRequest` (resolved profile, frame, extent, direction, participating bodies)
   - `ToolFactory` (builds only the variants actually needed, lazily; many already are)
   - `BooleanPlanner`, which selects a strategy and **records which strategy succeeded**
     in the diagnostics. Today you can't tell from a saved model whether a join went
     through "sectional", "cap", "smooth", or "dipped". That information would make field
     bug reports much faster to triage.
   - `CommitGate` (J8)
3. **Consider moving the app layer to f64.** `Vec3`, `CoordinateSystem`, `Region`, and
   depths. This is a schema-affecting change: serde can read existing f32 CBOR values into
   f64 fields, but plan it as an explicit format bump with round-trip tests. The GUI can
   keep f32 for rendering only.
4. **Collapse `Region.boundary` into a view of `analytic`.** Sample polylines on demand for
   display and picking. Never use them as geometry input.

---

## 5. Features and product: comparison with Fusion 360

### 5.1 Extrude (the root of most Join/Cut workflows)

| Capability | ZeroCAD | Fusion 360 |
|---|---|---|
| Operations | New Body, Join, Cut | New Body, **New Component**, Join, Cut, **Intersect** |
| Extent | Signed distance only (`depth: f32`) | Distance, **To Object** (face/body/vertex), **All** (Through All) |
| Direction | One side (signed) | One Side, **Two Sides** (independent), **Symmetric** |
| Start | Profile plane | Profile Plane, **Offset**, **From Object** |
| Taper | ✓ (`draft_angle_deg`) | ✓ (per side) |
| Participating bodies | Implicit: sketch body, or all overlapping | **Explicit list**, auto-filled, editable |
| Profile types | Sketch regions, body faces | Sketch profiles, planar faces, **text**, projected geometry |
| Silent direction flip | Yes (J10) | Never |
| Edit after failure | Feature stays, ⚠ Unresolved | Feature stays, red, with the error |

Why this matters for your Join/Cut problems: without **Through All** and **To Object**,
users set a cut depth *equal* to the body thickness. That creates the exact coincident
far-cap configuration the Boolean struggles with, and it breaks again when the body gets
thicker upstream. Fusion's "All" extent builds the tool well past the body, so
coplanarity never arises. **Adding Through All is probably the single cheapest
reliability win available**, because it sidesteps the hardest kernel case for the most
common cut. Symmetric and To Object remove most of the other "flush" setups.

Score: **3/10** vs Fusion 9.

### 5.2 Timeline and history

| Capability | ZeroCAD | Fusion |
|---|---|---|
| Ordered history, editable features | ✓ | ✓ |
| Suppress | ✓ (`ResolutionState::Suppressed`) | ✓ |
| **Rollback marker** (insert features earlier in history) | ✗ (none found) | ✓ |
| Failed features stay in place | ✓ (⚠ in the tree) | ✓ |
| Topological naming robustness | 5 (history-based, targeted) | 8 |
| Capture design history on/off (direct modeling mode) | partial (direct edits exist) | ✓ |

Score: **5/10** vs 9.

### 5.3 Whole-product category ratings

The confidence column shows how much I inspected. "Low" means I rated from the docs and
`rating.md` rather than reading the code in depth this pass.

| Category | ZeroCAD | Fusion | Confidence | Key gap |
|---|---:|---:|---|---|
| Sketch creation | 7 | 9 | medium | — |
| Constraint solver | 6 | 9 | low | Damped Gauss-Newton with DOF analysis; unknown robustness on hard sketches |
| Sketch → region extraction | 6 | 9 | medium | f32 arrangement; several tangency fixes recorded in memory |
| Extrude options | 3 | 9 | high | 5.1 |
| Join/Cut | 4.5 | 9 | high | Section 2 |
| Revolve/loft/sweep | 5 | 9 | low | — |
| Fillet/chamfer | 5 | 9 | medium | Heavy special-casing in rolling_ball; many fix docs |
| Shell | 4 | 8 | low | Documented cavity volume defect |
| Holes & threads | 6 | 9 | medium | Analytic helical threads are impressive; J1 risk |
| Patterns/mirror | 6 | 9 | low | — |
| Direct editing | 5 | 9 | low | — |
| Assemblies | 4 | 9 | low | No nested subassemblies (per rating.md); interactive solve limits |
| 2D drawings | 1 | 8 | high | Not present |
| CAM / simulation / rendering | 1 | 9 | high | Out of scope today |
| STEP import/export | 6 | 9 | medium | OCCT oracle gate is a great idea |
| Precision across scales | 3 | 9 | high | J11, J12 |
| Safe failure | 8.5 | 8 | high | Clear strength |
| Performance / footprint | 8 | 5 | medium | Native Rust, compact binary, offline, GPU viewport. Fusion is heavy and cloud-bound |
| Offline / licensing / privacy | 10 | 4 | high | Real differentiator; lead with it |
| Onboarding / UX polish | 5 | 8 | low | Start page and onboarding exist; not assessed interactively |

---

## 6. Testing and quality assurance

| Category | Score | Notes |
|---|---:|---|
| Breadth (≈2,000 tests, 1 ignored) | 8 | Excellent for a project this size |
| Fixture-driven user regressions | 9 | Saved `.zcad` repros with cold/warm/save-reopen and analytic volume checks. Best practice |
| Metamorphic/property tests | 6 | `property_geometry.rs`, `degeneracy_stability.rs`; could be much wider (see below) |
| Cross-feature interaction coverage | 4 | J1 slipped through; thread tests use primitives only. Need a *feature-pair matrix* |
| Scale/translation coverage | 5 | Some fixes test "three scales/translations"; not systematic across all Boolean tests |
| Feedback-loop speed | 3 | Full `cargo test` ran more than 40 minutes and was stopped (merged-polygon doc). Several runs failed with LNK1104 because concurrent sessions shared `target/` |
| Fuzzing | 4 | One fuzz crate and a boolean-fuzz workflow; small |
| CI | 7 | Windows, Linux, and macOS; separate kernel workspace; fmt and clippy gates |
| Lint debt | 5 | 450 reviewed baseline Clippy warnings, gated from growing (good) but not shrinking |

### Highest-value test additions

1. **Feature-pair matrix.** For each of {extrude-prism, primitive, revolve, loft, fillet,
   chamfer, shell, thread, hole, pattern, direct-edit, import}, then each of {Join, Cut}
   from {same face, opposite face, side face, origin plane}: assert the exact-volume
   inequality (J8) and that the prior feature survives (the thread still has helical
   faces, the fillet still has its torus). This would have caught J1.
2. **Global translation/scale metamorphic property.** Every existing Boolean fixture should
   pass when the whole model is translated by (±250, ±1000) mm and scaled ×0.01 and ×100.
   Today this is the biggest hidden-failure class (J11, J12).
3. **Concave profile cuts** (U, L, E, text) that force the `expanded` fallback, with
   probes along the inner walls (J7).
4. **Multi-body scenarios:** bystander flush body (J4), bridging join (J6), cut through
   sketch body into another body (J5).
5. **Test tiers.** A `fast` tier under 5 minutes, run on every change. Stress and threaded
   suites move to nightly or `ZEROCAD_LONG_STRESS`. Use a separate `CARGO_TARGET_DIR` per
   concurrent session or worktree to end LNK1104.

---

## 7. Process and project hygiene

| Category | Score | Notes |
|---|---:|---|
| Commit granularity & messages | 2 | Recent history contains `push` (×3), `ASsembly + cosmetics`, and one 42-file, +2,353-line commit; a month-long gap (Aug 24 → Sep 23) of uncommitted work. Boolean regressions can't be bisected |
| Branching | 3 | Everything lands on `openrcad` directly; no PR trail |
| Concurrent edits in one tree | 3 | Several fix docs report "kernel and GUI files were being edited concurrently" and runs interrupted by other processes. Use worktrees |
| Release engineering | 7 | Pinned toolchain, reproducible builds, Debian packaging, signing guide |
| Documentation volume | 7 | 34 docs plus README (793 lines), rating.md, AGENTS.md. Thorough |
| Documentation organisation | 5 | Many dated one-off fix docs at the top level; no index; the README duplicates some invariants that are now stale |
| Evidence honesty | 9 | Docs consistently state scope and what was *not* verified. Keep this |

Recommendations:

1. One logical change per commit, with a message that says *what and why*. For geometry
   fixes, name the fixture.
2. Move dated fix write-ups into `docs/fixes/YYYY-MM-DD-*.md` and add a `docs/README.md`
   index.
3. Keep README invariants current, or link them to the code as the single source of truth
   (J16 items).
4. One worktree and one target dir per concurrent agent or session.

---

## 8. Prioritised roadmap

### Now (days): targeted, low-risk fixes

1. Clear `sketch_source` in `thread_one`, and add the sketch-rod → thread → cut/join
   regression (J1).
2. Add a single `accept_boolean_candidate` gate with an exact-volume inequality and apply
   it to every fast path, including `cap_pocket` (J8).
3. Fix `failed_on_overlap` so it only counts genuine volume overlap (J4).
4. Remove the silent reverse at evaluation time; resolve direction once in the GUI and
   store it (J10).
5. Replace `cut_parts_changed` tessellation hashing with an exact volume delta (J9).
6. Clean up J16: stale comments, no-op wrappers, misnamed functions.

### Next (weeks): Fusion-parity extrude, which also fixes reliability

7. Add the **Through All** extent, then **To Object**, **Symmetric**, and **Two Sides**
   (Section 5.1).
8. Add a **participating bodies** list for Join and Cut, auto-filled from the preview and
   editable in the inspector (J5). Join all participating bodies into one (J6).
9. Replace `grow_loop` with a true offset, or drop `expanded` (J7). Make the remaining
   nudges relative to model size (J11).
10. Record the winning Boolean strategy in diagnostics.

### Later (months): structural

11. Scale-aware `TolerancePolicy::for_model` in the kernel.
12. Move the core application geometry to f64 (J12), with a planned format bump.
13. Make exact analytic profiles the only geometry input (J13).
14. Kernel classification propagation and exact planar predicates (J14). Once these land,
    measure how many ladder steps still get used, and delete the ones that don't.
15. Enforce the `LiveBody` invariant in the type system (refactor 1).
16. Split `apply_extrude`, `boolean_impl`, and `rolling_ball.rs`, behind the full geometry
    suite.
17. Add a timeline rollback marker (Section 5.2).

**Suggested success metric:** a nightly "translated/scaled feature-pair matrix" pass rate.
Today it would likely be well below 100% for anything after a blend. Track it per week,
and treat each new user-supplied `.zcad` as a matrix row instead of a one-off fix doc.

---

## 9. What is genuinely excellent (keep doing it)

- **Atomic features.** A failed Join or Cut never leaves a half-fused or overlapping body,
  and never quietly drops material. That is better discipline than many commercial tools
  show.
- **Diagnostics that explain geometry** ("touches only along an edge, which is
  non-manifold; make the profiles overlap by area"). This is friendlier than Fusion.
- **User-model regressions** with cold, warm, save/reopen, analytic volume, and point
  membership checks.
- **Evidence culture.** The docs say what was *not* proven.
- **An OCCT differential oracle**, pinned and never linked. A smart way to get a second
  opinion without shipping OCCT.
- **Native, offline, compact.** A real product differentiator from Fusion's cloud-bound,
  heavy client.
- **Ambitious geometry done properly in places:** analytic multi-start helical threads,
  Gregory corner patches, and native circular rim blends.

---

*This is a static review; nothing was built or executed. The items marked LIKELY (J1, J6,
J7 symptom) should each get a short repro test before anyone treats them as confirmed.*
