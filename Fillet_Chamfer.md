# Fillets and Chamfers: ZeroCAD vs Fusion 360

*Written 2026-09-27. Read-only analysis: no code was changed while writing this.*

## TL;DR

ZeroCAD's fillets and chamfers are **accurate and safe but narrow**. Every result
goes through strict validation (watertight, healthy, subtractive, local, cracks
checked) and a failure leaves the body unchanged. So the system rarely produces
broken geometry. But the set of edges it *can* blend is a hand-built list of
geometric cases, not a general algorithm. That is why it feels "non-general".

Fusion 360 runs on Autodesk Shape Manager (ASM, an ACIS descendant). ASM treats a
fillet as one general problem: a ball of radius *r* touching two **arbitrary**
surfaces, solved numerically along an **arbitrary** edge curve. It then handles
ends, corners and overflow with generic tools. ZeroCAD/OpenRCAD solves a fillet
**in closed form for a short list of surface pairs**. After that it closes each
end with one of about nine special-case routines, tried in a fixed order.

The root causes, with evidence in §5:

1. **Supporting-surface whitelist.** The rolling-ball solver only handles
   plane+plane and plane+cylinder (in two specific orientations). Every other
   pair returns `UnsupportedSurfacePair`. That includes cone, sphere, torus,
   B-spline, cylinder+cylinder and an oblique plane on a cylinder. There is no
   numeric (marching) solver, even though the error enum and the kernel's
   `CLAUDE.md` both describe one.
2. **Chamfers are narrower still.** They support plane+plane, plus circular
   rims through a cone-frustum path.
3. **Edge-curve whitelist.** Edges are persisted as `Line` or `Circle`, and the
   kernel locates edges by straight-line endpoint matching. Ellipse, B-spline and
   helix edges cannot be addressed properly.
4. **End caps and corners are a set of special cases**, not a general trim.
   A blend end that meets more than one distinct cap face is rejected. Corner
   networks are limited to equal-radius, all-planar, 3–6 edge vertices. `Setback`
   is stored in the file format but always rejected by the kernel.
5. **Only one constant value is supported.** There is no variable radius, chord
   length, asymmetric fillet, two-distance chamfer, distance-and-angle chamfer or
   G2 fillet. `BlendLaw::Variable` exists only as a placeholder.
6. **Results can depend on how the body was built.** The app layer retries with
   rebuilt "canonical" solids from the sketch (e.g. "box-cylinder sketch"), so a
   fillet can succeed on a body made one way and fail on the same shape made
   another way.
7. **UX gaps compared with Fusion.** A straight edge does not carry its tangent
   chain into the kernel. You cannot select faces or features, or use rules. A
   feature has one radius (no selection sets). Values have a hard 0.2 mm floor.
   You cannot flip a chamfer's direction.

The recommended path (§7) is a **general blend core**:

- a surface-pair-agnostic contact solver, with the existing analytic cases as
  fast paths;
- a trim step based on imprinting rather than per-case wire surgery;
- capping by boolean trimming as the general fallback;
- always-simultaneous network solving;
- the missing value laws.

---

## 1. How fillets and chamfers are supposed to work (theory)

### 1.1 Vocabulary

| Term | Meaning |
|---|---|
| **Spine** | The sharp edge being blended (or, more precisely, the path of the ball's center). |
| **Supports** | The two faces that meet at the edge (face A, face B). |
| **Contact curves** (also called spring curves or rails) | Where the blend surface touches each support tangentially. The supports get trimmed back to these curves. |
| **Cross-section** | The profile in the plane normal to the spine: a circular arc for a fillet, a straight segment for a chamfer, or a conic/G2 curve for a curvature-continuous blend. |
| **Convex / concave** | A convex edge (a "round") removes material. A concave edge (a "fillet" in the strict sense, e.g. an inside pocket corner) adds material. |
| **End condition** | What happens where the blend stops: it caps against a face, runs out onto a tangent face, overflows onto a neighbouring face, or meets another blend. |
| **Vertex blend** | The patch where three or more blended edges meet: a rolling-ball sphere, a setback patch, or a miter. |
| **Chain** | A sequence of edges joined tangentially (G1). CAD systems blend a chain as one continuous band. |

### 1.2 The rolling-ball definition (constant radius)

A ball of radius *r* rolls along the edge while staying tangent to both supports.

- **Ball center path:** the intersection curve of the two supports, each offset
  by *r* toward the material side: `C(t) = Offset(A, r) ∩ Offset(B, r)`.
- **Contacts:** the foot points of `C(t)` on A and on B.
- **Blend surface:** the envelope of the moving ball, i.e. a pipe surface of
  radius *r* around `C(t)`, cut down to the arc between the two contacts.
- **Analytic special cases**, which kernels detect for exact output:

| Support pair | Blend surface |
|---|---|
| plane + plane | cylinder |
| plane + coaxial cylinder (rim edge) | torus |
| plane + cylinder generator (plane parallel to the axis) | cylinder |
| plane + cone (rim) | torus |
| sphere + coaxial cylinder or plane | torus |
| **anything else** (oblique plane on cylinder, cylinder+cylinder, B-spline, …) | a **general swept surface**, usually represented as a NURBS approximation with a tolerance, or a procedural "blend surface" with exact evaluation |

- **General solve:** production kernels (ASM, Parasolid, OCCT's
  `BRepFilletAPI` / `ChFi3d`) march along the edge. At each step they solve four
  unknowns (u, v on A and u, v on B) from four equations: the ball center lies at
  distance *r* along each surface normal, and the two centers coincide. A
  predictor–corrector continuation handles this, with step control based on
  curvature. Spine curvature tighter than *r*, or a support whose curvature
  radius is below *r*, makes the problem ill-posed. That is a genuine
  "radius too large" error.

### 1.3 Chamfers

A chamfer is a ruled surface between two rails on the supports.

- **Equal distance:** each rail is offset *d* from the edge, measured **along
  each face** and perpendicular to the edge. On curved faces, "distance" means a
  geodesic or cross-section distance, depending on the kernel.
- **Two distances:** different offsets on A and B. The user can pick which side
  gets which value ("flip").
- **Distance and angle:** offset *d* on one face, then a bevel plane at angle θ
  to that face.
- On a plane+plane edge the chamfer face is planar. On a plane+cylinder rim it
  is a cone frustum. In general it is a ruled surface between two computed rails.

### 1.4 End conditions

This is where most real-world difficulty lives.

1. **Cap:** the blend ends on a third face. That face is trimmed by the end
   cross-section, or by the true intersection of the blend with that face if it
   is not perpendicular to the spine.
2. **Runout:** the edge fades into a tangent face, so the blend narrows to
   nothing.
3. **Overflow / roll-over:** a contact curve runs off the edge of its support
   (radius larger than the face). The kernel must continue the contact onto the
   next face ("roll along the edge"), or keep the edge sharp and trim the blend
   against it ("cliff edge" / "notch").
4. **Meets another blend:** handled as a vertex blend or a miter.

### 1.5 Vertex (corner) blends

- **Rolling ball (sphere corner):** when three equal-radius convex fillets meet
  at a trihedral vertex, the corner is a spherical triangle. For unequal radii or
  other valences the patch is a general N-sided patch.
- **Setback:** the band ends are pulled back from the vertex by setback
  distances, and an N-sided patch (Gregory, Coons, or a subdivision-based patch)
  joins them smoothly. Designers prefer it for visibly unequal radii.
- **Miter:** two bands are trimmed along their mutual intersection seam, leaving
  a crease. This is the default when only two of three edges at a corner are
  selected.
- **Chamfer corners:** "Chamfer" (a flat triangular corner facet), "Miter"
  (bevels meet at a point), or "Blend" (smooth into adjacent faces).

### 1.6 Networks and ordering

Blending edges one at a time changes the result: filleting A then B differs
from B then A at a shared vertex. Good systems do two things:

- **(a)** solve every edge selected in one feature **simultaneously** against
  the original body, then build corners from the complete incidence picture;
- **(b)** propagate selection along tangent chains so a chain becomes one band.

### 1.7 Value laws

- **Constant.**
- **Chord length:** the width of the blend between its contacts is fixed, so the
  radius varies with the dihedral angle. Useful on draft or tapered faces.
- **Variable radius:** a radius law along the spine, set by values at points and
  interpolated.
- **Asymmetric / elliptical:** different setbacks on A and B, giving a conic
  cross-section.
- **Continuity:** a G1 circular cross-section, or G2 curvature-continuous
  (conic, or a quintic with a tangency weight).

---

## 2. What Fusion 360 offers

From the Autodesk Fusion help (Fillet and Chamfer references) and the usual
workflow:

### 2.1 Fillet command

| Area | Fusion capability |
|---|---|
| Types | **Fillet** (edges / faces / features), **Rule Fillet**, **Full Round Fillet** (replaces a centre face with a round tangent to two side faces). |
| Selection | Edges; **faces** (all boundary edges of the face); **features** (all edges created by a feature); multiple **selection sets** in one feature, each with its own radius. **Tangent Chain** is on by default. |
| Radius type | **Constant**, **Chord Length**, **Variable** (points along the edge, each with a radius), **Asymmetric** (two distances, conic-like). |
| Continuity | **Tangent (G1)** or **Curvature (G2)**, plus a **Tangency Weight** from 0.1 to 2 that shapes the G2 profile. |
| Corner type | **Rolling Ball** or **Setback**. |
| Rule fillet | Rules: **All Edges** or **Between Faces/Features**. Topology: **Rounds and Fillets / Rounds Only / Fillets Only** (convex only, concave only, or both). |
| Supports | Any B-rep surfaces ASM supports: planes, quadrics, tori, NURBS, T-spline-converted bodies, surface bodies. |
| Overflow / ends | Handled automatically by ASM. Fusion does not expose the overflow knobs that Inventor has ("roll along sharp edges", "rolling ball where possible", "preserve all features"), but ASM still rolls over, caps and trims generically. |
| Failure | The feature fails as a whole and is marked in the timeline. There is no partial silent success, which matches ZeroCAD's atomic philosophy. |
| Interaction | Drag arrow for the radius, pre-selection, match the radius by hovering an existing fillet, Ctrl to add or remove edges, and later edits can change the edge set. |

### 2.2 Chamfer command

| Area | Fusion capability |
|---|---|
| Types | **Equal Distance**, **Two Distances**, **Distance and Angle**. |
| Direction | **Flip** swaps which face gets distance 1 or the angle reference. |
| Corner type | **Chamfer**, **Miter**, **Blend**. |
| Selection | Edges, faces, multiple selection sets, **tangent chain**. |
| Supports | Arbitrary (ASM). |

### 2.3 What users experience as "almost any edge"

In Fusion, any manifold edge between two faces is a valid candidate, whatever
the surface types, whatever the edge curve, and whatever is at its ends. The
only common failures are geometric infeasibility (radius larger than local
curvature or thickness) and pathological self-intersections. Fusion's error
message then names the offending edges.

---

## 3. How ZeroCAD implements fillets and chamfers today

### 3.1 Pipeline (GUI → document → evaluator → kernel)

```
GUI  zerocad-gui/src/edgemod.rs
  begin_edge_mod(kind)                   selection → EdgeRef list
    tangent_edge_chain()                 (app/picking.rs) display-only unless arc-led
  begin_all_edge_mod(kind)               "Fillet All"/"Chamfer All" → durable name set
  show_edge_mod_dialog()                 inline size box, Fillet↔Chamfer, corner mode
  commit_edge_mod()                      only after exact worker preview has 0 warnings
        │  FeatureType::EdgeBlend { target, edges, dist, dist_expr, kind, corner_mode }
        ▼
Core  zerocad-core/src/parametric/edge_mod.rs
  apply_edge_blend()                     resolve each EdgeRef (topology name → geometry)
                                         preflight (dist>0, edge length>1e-4)
                                         per part: mock_kernel::blend_edge_network
  apply_edge_mod()  (legacy EdgeMod)     single edge / strict all-edge selector
    apply_fillet / apply_chamfer         native → alternates rebuilt from sketch source
    edge_mod_accept_candidate_*          containment / locality / crack / nonmanifold gates
  named_edge_mod_result_mesh()           propagate face names, stamp generated faces
        ▼
Facade zerocad-core/src/mock_kernel/blend.rs
  blend_edge_network()                   EdgeRef → kernel Edge (snap f32 endpoints)
                                         Circle hint → circle_edge_requests
      on native failure: boolean "cutter" fallbacks (straight, orthogonal only)
  fillet_all_edges_strict / chamfer_all_edges_strict
        ▼
Kernel OpenRCAD/crates/openrcad-algo/src/
  contour.rs      apply_blend_network_with_policy → apply_blend_contour_with_policy
                  dispatch: mixed line+arc tangent chain | Circle hint | generic
  rolling_ball.rs (9.7k lines)  fillet_edges_with_policy
                  ├─ closed planar loop  → fillet_closed_planar_edge_loop
                  ├─ ≥3 edges at a vertex → fillet_planar_corner_network (corner_network.rs)
                  └─ otherwise SEQUENTIAL: fillet_planar_edge per edge
                        overflow probe → fillet_planar_overflow (planar prism only)
                        fillet_planar_edge_inner: try_* end-closure cascade, trim, sew, validate
                  fillet_circular_edge_chain / fillet_tangent_edge_chain
  chamfer.rs      chamfer_edges_with_policy (planar-planar), closed-loop miter
  all_edges.rs    strict whole-solid wrapper (box/cylinder fast paths + per-edge)
```

### 3.2 Persisted data model

- `FeatureType::EdgeBlend` (`parametric/types.rs:782`) stores the target, a
  canonical `Vec<EdgeRef>`, one `dist`, an optional `dist_expr`, `kind`
  (`Fillet | Chamfer`) and `corner_mode` (`Auto | Miter | RollingBall | Setback`).
- `FeatureType::EdgeMod` is the legacy single-edge form. It also carries the
  "all edges" selector, encoded into `TopologyEdgeRef::edge_id`.
- `EdgeRef` (`types.rs:142`) holds two world-space f32 endpoints and two
  adjacent face normals. It also has an optional `EdgeCurveHint`, which is
  **only `Line` or `Circle`** (`mock_kernel/types.rs:17`), and an optional
  durable topology name.
- There is **one scalar value per feature**. There is no per-side distance,
  angle, law, continuity or setback distance.

### 3.3 Kernel capability matrix (what actually works)

| Case | Fillet | Chamfer | Where |
|---|---|---|---|
| Straight edge, plane+plane, convex, any dihedral | ✅ exact cylinder | ✅ plane | `planar_blend`, `planar_chamfer` |
| Straight edge, plane+plane, **concave** | ✅ | ✅ | `planar_blend_concave`, `concave` flag |
| Circular rim, plane ⟂ cylinder axis (boss top, bored hole) | ✅ torus (open and closed arcs) | ✅ cone frustum | `rolling_ball_between_curved_faces`, `fillet/chamfer_circular_edge_chain` |
| Straight generator, plane ∥ cylinder axis | ✅ cylinder | ❌ (planar only) | `rolling_ball_plane_perp_cylinder` |
| Mixed tangent chain of lines + co-circular arcs | ✅ simultaneous | ✅ | `fillet/chamfer_tangent_edge_chain` |
| Closed planar edge loop (e.g. top rim of a prism) | ✅ simultaneous | ✅ simultaneous miter | `fillet_closed_planar_edge_loop`, `chamfer_closed_planar_edge_loop` |
| Trihedral and higher corner, all edges selected, equal r, **all planar**, valence 3–6 | ✅ exact common sphere | (via all-edge path) | `corner_network.rs` |
| Two fillets at a corner, third edge sharp | ✅ miter (ellipse seam) | ✅ | `try_corner_miter`, unequal/mixed/circular-band variants |
| Blend end meets a concave cut cylinder | ✅ flush trim | ✅ | `try_corner_cut`, chamfer `plane_cylinder_trim_edge` |
| Blend end meets an oblique planar cap | ✅ ellipse flush trim | ✅ | `try_oblique_planar_cap` |
| Contact overflows a face (radius > face width) | ✅ **only** convex, plane+plane, straight prism | ❌ | `fillet_planar_overflow_with_policy` |
| Plane + cylinder at an **oblique** angle (ellipse edge) | ❌ `UnsupportedSurfacePair` | ❌ | — |
| Cylinder + cylinder (boss on a pipe, cross-drilled holes) | ❌ | ❌ | — |
| Any cone / sphere / torus support (drafted walls, revolves, **edges of a previous fillet**) | ❌ | ❌ | — |
| B-spline / ruled / loft / sweep / imported STEP NURBS faces | ❌ | ❌ | — |
| Ellipse, B-spline or helix edge curve | ❌ (cannot even be addressed) | ❌ | `EdgeCurveHint` |
| Edge end touching ≥2 distinct cap faces (partial corner selection) | ❌ `CornerNetwork::IncompleteSelection` | ❌ `UnsupportedTrimTopology` | `handle_corner_endpoint`, `close_chamfer_endpoint` |
| Unequal-radius or non-planar corner networks | ❌ | — | `corner_network.rs` |
| Setback corners | ❌ `UnsupportedCornerMode("setback")` | ❌ | `contour.rs:176` |
| Variable radius / chord length / asymmetric / G2 | ❌ (`BlendLaw::Variable` placeholder) | — | `contour.rs:30` |
| Two-distance / distance-and-angle chamfer | — | ❌ | — |

The validation around this is a real strength and should be kept. Each result
must pass all of the following:

- sewn watertight, with a healthy topology report;
- strict pcurves;
- containment against the pre-blend body (subtractive, unless concave);
- locality;
- mesh crack and non-manifold checks;
- a circular-bite locality check.

Failures are atomic, and the body is left unchanged with a specific warning.

---

## 4. Side-by-side comparison

| Capability | Fusion 360 | ZeroCAD today | Gap |
|---|---|---|---|
| Supporting-surface types | Any | Plane, cylinder (2 orientations) | **Critical** |
| Edge-curve types | Any | Line, circle | **Critical** |
| Constant radius | ✅ | ✅ | — |
| Chord length | ✅ | ❌ | High |
| Variable radius | ✅ | ❌ (placeholder) | High |
| Asymmetric fillet | ✅ | ❌ | Medium |
| G2 continuity + tangency weight | ✅ | ❌ | Medium (cosmetic and industrial design) |
| Rolling-ball corners | ✅ general | ✅ equal-r planar 3–6 only | High |
| Setback corners | ✅ | ❌ (persisted, kernel rejects) | Medium |
| Partial corner selection (2 of 3 edges) | ✅ | ✅ miter paths (planar/cyl only) | Medium |
| Overflow / roll-over | ✅ automatic | ✅ planar straight prisms only | High |
| Simultaneous multi-edge solve | ✅ | Partial: loops, common vertex and tangent chains; otherwise sequential | Medium |
| Tangent-chain selection | ✅ default | Displayed; persisted only for arc-led chains | Medium |
| Face / feature selection | ✅ | ❌ (whole-body "All" only) | Medium |
| Rule fillet (rounds / fillets only) | ✅ | ❌ | Low–Medium |
| Full round fillet | ✅ | ❌ | Low–Medium |
| Multiple radii in one feature | ✅ selection sets | ❌ one value | Medium |
| Chamfer: equal distance | ✅ | ✅ | — |
| Chamfer: two distances, distance-and-angle, flip | ✅ | ❌ | High (manufacturing) |
| Chamfer corner types | Chamfer / Miter / Blend | Auto / Miter | Medium |
| Minimum size | Tolerance-limited | Hard 0.2 mm floor (`commit_edge_mod`, inspector slider 0.2–40) | Low, but surprising |
| Atomic failure with diagnostics | ✅ | ✅ (arguably stricter) | ZeroCAD strength |
| Live exact preview | ✅ | ✅ (exact worker preview, commit gated on it) | Parity |
| Persistent edge references | ✅ | ✅ durable names + geometric fallback | Parity-ish |

---

## 5. Why it feels non-general: root causes with evidence

### 5.1 The rolling-ball solver is a whitelist, not a solver

`rolling_ball_fillet_edge_with_policy` (`rolling_ball.rs:7303`) branches like this:

- if both faces are planar, use `planar_blend` or `planar_blend_concave`;
- otherwise, use `rolling_ball_between_curved_faces_with_policy`.

That function (`:7375`) demands **one plane and one cylinder**. It accepts only
two configurations:

- a circular rim with the plane normal parallel to the axis, giving a torus;
- a straight generator with the plane normal perpendicular to the axis, giving
  a cylinder.

Everything else returns
`UnsolvableAdjacency { reason: UnsupportedSurfacePair }`.

`AdjacencyReason::NewtonFailed` and `RollingBallError::NewtonDiverged` exist in
the enums, but **nothing emits them**. The only matches are in tests and in the
error mapping in `chamfer.rs`. `OpenRCAD/CLAUDE.md` Phase 5 claims "curved–curved
edge adjacency with a Gauss-Newton solver". That is documentation drift: there
is no marching or Newton contact solver.

**Consequence:** after one fillet, the edges bounding the new band (torus or
cylinder against a cylinder or torus) are generally not filletable. Drafted
extrudes (cones), revolves (cones, spheres, tori), lofts and sweeps (ruled or
B-spline), cross-drilled holes (cylinder+cylinder) and any imported NURBS all
fall outside the solver. This is exactly the class of "any edge" that Fusion
users expect to work.

### 5.2 Chamfers are even narrower

`chamfer_planar_blend` (`chamfer.rs:465`) requires both faces to be planes;
otherwise it returns `ChamferError::UnsupportedSurfacePair`. The only non-planar
chamfer is the circular-rim cone path. A longitudinal plane+cylinder edge,
which *does* fillet, cannot be chamfered.

### 5.3 Edge identity assumes lines and circles

`EdgeCurveHint` has two variants. For everything that is not a circle, the
facade builds `Edge::between_points(p0, p1)` (`mock_kernel/blend.rs:24`). The
kernel then matches that request with `same_undirected_edge` (endpoints only)
or `edge_contains_requested_span` (a **collinear** test)
(`rolling_ball.rs:8035–8068`).

- An elliptical or B-spline edge whose endpoints coincide with another edge's
  endpoints is ambiguous.
- A closed non-circular edge (one vertex) cannot be addressed at all.
- The f32 snap in `snap_point_to_topology` exists because the persisted
  geometry is a tessellation cast, not a kernel reference.

### 5.4 End conditions are a cascade of special cases

`fillet_planar_edge_inner` (`rolling_ball.rs:898`) tries these routines, in
this order, at each end:

1. `try_corner_cut`
2. `try_corner_sphere_two_caps`
3. `try_corner_miter`
4. `try_corner_circular_band_miter`
5. `try_tangent_curved_wall_runout`
6. `try_oblique_planar_cap`
7. `try_perpendicular_planar_setback`
8. `handle_corner_endpoint` (single planar cap, or a prior-blend sphere)

The file also contains `try_corner_mixed_side_miter` and
`try_corner_unequal_miter`. There is also a retry with `use_sphere = false`.

Each routine recognizes one specific geometric pattern and edits the wire by
hand. Anything unrecognized becomes `UnsupportedTrimTopology`. When an endpoint
touches several distinct cap planes, the result is
`CornerNetworkError::IncompleteSelection` (`:1496`).

Every recent fix follows this same pattern: a new recognizer plus a new
regression test. Examples are `docs/fillet-boss-junction-2026-09-13.md`
("General corners involving five distinct surfaces are not added") and the
memory notes on cut-fillet flush trim, the oblique cap, and circular bite. That
is why coverage grows edge case by edge case instead of generally.

### 5.5 Multi-edge features are only partly simultaneous

`fillet_edges_with_policy` (`:5423`) uses a simultaneous solve only for:

- a closed planar loop;
- three or more edges sharing **one** common vertex (equal-r planar network,
  valence ≤ 6).

Every other network is applied **sequentially**, re-locating each edge in the
evolving body. An open L- or U-chain, or two separate corners in one selection,
is therefore order-dependent and relies on the miter recognizers above. The
chamfer path behaves the same way (`chamfer.rs:191`).

### 5.6 Corner networks and overflow are narrow by construction

- `CornerTangentSphere::new` only solves equal-offset **planes**, valence 3–6
  (`corner_network.rs`). Mixed-radius and non-planar corners are typed
  rejections.
- Overflow requires plane+plane supports, a convex edge and a straight prism
  profile (`fillet_planar_overflow_with_policy`, `:437`). Otherwise it returns
  `FilletOverflowError::UnsupportedSelectedSupports`, `ConcaveUnsupported` or
  `NonPrismaticSource`.
- `BlendCornerMode::Setback` always returns `UnsupportedCornerMode("setback")`
  (`contour.rs:176`), although it is part of the persisted `EdgeCornerMode`.

### 5.7 App-layer heuristics depend on construction history

`edge_mod_native_fillet_all_parts` and `edge_mod_native_chamfer_all_parts`
(`edge_mod.rs:1071`, `:1172`) retry, on native failure, against
`sketch_source_alternate_parts`. These are solids **rebuilt from the originating
sketch**: a "box-cylinder sketch" recognizer (`rect_circle`,
`rect_minus_circle_region_solid`) and a faceted re-extrusion. Circular-bite
locality checks are also keyed to the sketch region.

So the same geometry can blend or fail depending on whether it came from a
recognized sketch extrude, a boolean, a STEP import or a direct edit.
`body.sketch_source` is cleared after the first edge mod, so a second blend on
the same body loses the fallback.

`blend.rs` also has two **boolean cutter** fallbacks, which require straight
edges and (for fillets) **orthogonal** supports. They cover networks where the
native path fails, but only in the easiest geometry.

### 5.8 A single scalar value

`BlendContour.law` is `Constant(f64)` or `Variable` (unimplemented). The
document schema has one `dist`. Chord length, variable radius, asymmetric
fillets, G2 fillets, two-distance chamfers, distance-and-angle chamfers and
per-selection-set radii would all need schema growth. This is additive
`#[serde(default)]` work, but still work.

### 5.9 UX differences that add to the feeling

- **Tangent chain:** `begin_edge_mod` (`edgemod.rs:1142`) persists the
  propagated chain **only when it is arc-led**. "A straight seed deliberately
  stays a single edge." The overlay shows the chain, but the kernel receives one
  edge. Fusion blends the whole chain.
- **Selection types:** edges or whole body only. There is no face selection
  ("fillet this face's boundary"), feature selection, rule fillet or full round.
- **One size per feature:** Fusion's selection sets allow several radii in one
  feature.
- **Size floor:** 0.2 mm on commit (`dist.max(0.2)`), and the inspector slider
  is limited to 0.2–40 (`feature_properties.rs:1719`, though an expression can
  exceed it). The inline field clamps to 0.05–300, which does not match the
  commit floor.
- **Chamfer controls:** there is no flip or asymmetric chamfer. Corner-mode
  buttons appear only when more than one edge is selected, and Setback is
  hidden (correctly, since it is unsupported).
- **Stale doc comment:** `FeatureType::EdgeMod` still says "Applied as a guarded
  boolean subtraction of an edge-aligned cutter". The native path replaced that
  approach; only the network fallback still uses cutters.
- **Absolute tolerance:** `EDGE_MOD_CONTAINMENT_TOL = 0.25` is an absolute
  length in mm (`edge_mod.rs:9`). It is permissive on tiny parts and tight on
  large ones, where a relative or policy-derived tolerance would scale.

---

## 6. What ZeroCAD already does well (keep this)

- **Atomic, validated commits.** Watertight, healthy, strict-pcurve,
  containment, locality, crack and non-manifold gates. Preview and commit use
  the same exact operation, and commit is refused until the exact preview
  passes with no warnings. This is stricter than Fusion.
- **Exact analytic geometry** where it applies (cylinder, torus, cone, sphere
  corners), with no NURBS approximation error in those cases.
- **Durable edge identity** with a deliberate geometric fallback, and failure
  (not a silent jump) when a named edge vanishes (break-test F06).
- **Concave blends** that add material correctly.
- **Well-covered regression tests** (`break-test/tests/catalog_f_blends.rs`,
  `edge_blend_break.rs`, `openrcad-algo/tests/*fillet*`, `corner_flow.rs`,
  volume oracles). This makes a rewrite of the core safe to attempt.

---

## 7. Recommended path to Fusion-grade generality

The goal is to replace the *whitelist + recognizer* architecture with a
*general solve + general trim*, and keep today's analytic builders as **fast
paths**, not as the whole capability.

### Phase A: general contact solver (largest single win)

- Implement `solve_contacts(face_a, face_b, spine, r, law)` for **any two
  surfaces** that implement the `Surface` trait (`point`, `d1`, `normal`).
  - Solve for the unknowns (u_a, v_a, u_b, v_b) such that
    `S_a(u_a,v_a) + r·N_a = S_b(u_b,v_b) + r·N_b`, projected to the spine's
    normal plane at parameter *t* (four equations, four unknowns).
  - Use predictor–corrector continuation along *t*, with adaptive steps (chord
    and angle error) and seam and periodicity handling.
  - Raise real `NewtonDiverged` and `RadiusTooLarge`, detected when r exceeds
    the local principal curvature radius.
- Output: the contact curves as B-splines fitted to the sampled points, with
  pcurves on each support. The blend surface is either a pipe/rolling-ball
  surface or a NURBS fit within tolerance.
- **Analytic recognition afterwards:** if the samples fit a cylinder, torus,
  cone or sphere within tolerance, emit the exact analytic surface. Existing
  planar and cylindrical cases keep bit-identical output.
- Chamfer from the same machinery: rails at distance d₁ / d₂ (or d plus angle
  θ) in the cross-section plane, joined by a ruled surface.
- **Acceptance:** property sweeps over surface-pair × dihedral × radius ×
  scale × rotation. Cover plane+cone, plane+sphere, cylinder+cylinder (equal
  and unequal radii, crossing), plane+torus (edge of a prior fillet), and
  plane+B-spline (loft side). Check a volume oracle and contact tangency
  residual below the policy tolerance.

### Phase B: general spine addressing

- Extend `EdgeCurveHint`, or better, persist a kernel-side edge reference: the
  curve kind, curve parameters, and a mid-parameter sample point. Then
  `relocate_edge` can match any curve by (endpoints, midpoint, tangent), not by
  collinearity.
- Keep the durable topology name as the primary key, with this as the fallback.

### Phase C: general trimming and capping

- Trim supports along contact curves with the existing
  **imprint / `partition_face`** machinery (`imprint.rs`, `BRepBuilder`),
  instead of per-case wire surgery.
- **General end-cap fallback (hybrid approach):** build the blend as a *local
  tool solid*. This is the crescent swept past both ends: the region between
  the sharp edge, both contacts and the blend surface. Extend it by a margin
  beyond each end, then **boolean-subtract** it (or boolean-add it for a
  concave edge) from the body. The boolean computes the correct cap, runout,
  oblique, multi-face or curved intersection automatically.
  - The existing `fillet_straight_edge_network_with_cutters` is a narrow
    version of this idea. Generalize it to use the Phase A blend surface
    instead of an exact cylinder, and drop the orthogonality requirement.
  - Keep the `try_*` recognizers as fast, exact paths in front of this
    fallback. They stop being the limit of what can be blended.
- Overflow: continue the contact solve across support-face boundaries (a
  face-hopping march). Alternatively, rely on the boolean-tool fallback, which
  handles most overflow implicitly, for the "cliff edge" result.

### Phase D: always-simultaneous networks and general vertex blends

- Solve every band of a feature against the **original** body first. Then
  classify each vertex by the incident selected and unselected edges:
  - valence 1: cap (Phase C);
  - two bands: miter via band∩band intersection;
  - three or more bands: sphere when equal r and planar, otherwise an
    **N-sided Gregory setback patch**. `GregorySurface` already exists; it is
    4-sided today and needs N-sided generalization.
- Implement `Setback` for real, then expose it in the GUI.
- Remove the sequential fallback in `fillet_edges_with_policy`, or keep it
  only as a last resort behind the simultaneous solve.

### Phase E: value laws and schema

Add optional fields with `#[serde(default)]` so `.zcad` files stay compatible:

- `law: BlendLaw` = `Constant | ChordLength | Variable(Vec<(param, r)>) |
  Asymmetric(d1, d2)`;
- `continuity: G1 | G2 { weight }`;
- chamfer `ChamferSpec` = `Equal(d) | TwoDistances(d1, d2) |
  DistanceAngle(d, θ)` plus a `flip` flag;
- per-feature `selection_sets: Vec<{ edges, value }>`, so one feature can carry
  several radii like Fusion.

Implement `BlendLaw::Variable` in the Phase A solver: the radius becomes r(t).

### Phase F: UX parity

- Persist tangent chains for straight seeds as well, controlled by a Tangent
  Chain toggle (default on, as in Fusion).
- Face selection (boundary edges), feature selection (edges created by a
  feature), and **Rule fillet** (All / Between faces; Rounds / Fillets /
  Both, using the existing convexity probe `edge_wedge_is_concave`).
- **Full round fillet**: solve r from the three-face tangency (centre face
  removed).
- Chamfer flip and two-distance handles; corner type Chamfer / Miter / Blend.
- Replace the 0.2 mm floor with a policy-derived minimum (a multiple of
  `TolerancePolicy.linear`) and align the inline, inspector and commit limits.
- Error surfacing like Fusion: highlight the blocking edges in the viewport
  (`AllEdgeBlocker` already carries them).
- Edit an existing feature's edge set, not only its value.

### Phase G: cleanups (small and independent)

- Fix the stale `FeatureType::EdgeMod` doc comment, and `OpenRCAD/CLAUDE.md`'s
  "Gauss-Newton curved–curved" claim.
- Make `EDGE_MOD_CONTAINMENT_TOL` relative to the part size or tolerance policy.
- Stop coupling blend success to `sketch_source`. Once Phases A–C land, the
  "box-cylinder sketch" alternates should be deletable, and a blend should
  depend only on the current B-rep.

### Suggested order and effort

| Order | Phase | Why first |
|---|---|---|
| 1 | A (solver) + C (boolean-tool cap fallback) | Together they turn "unsupported surface pair" and "unsupported trim topology", the two dominant failure classes, into successes on most real parts. |
| 2 | B (spine addressing) | Needed so A can reach ellipse, spline and helix edges. |
| 3 | E (chamfer two-distance / angle) + F (tangent chain, flip) | High user-visible value, low geometric risk (the chamfer is ruled between rails). |
| 4 | D (simultaneous networks, setback, N-sided patches) | Quality of corners; builds on A. |
| 5 | E (variable / chord / G2), F (rule / full round) | Breadth features on top of a general core. |

The existing validation and property-sweep discipline should gate every phase.
The project memory warns that boolean and blend fixes regress at scales that
were not swept, so each phase needs the full scale and rotation matrix, not
single fixtures.

---

## Appendix A: key files

| Concern | File |
|---|---|
| Persisted feature and edge reference | `zerocad-core/src/parametric/types.rs` (`EdgeRef`, `EdgeCornerMode`, `FeatureType::EdgeMod` / `EdgeBlend`) |
| Evaluator, resolution, validation gates | `zerocad-core/src/parametric/edge_mod.rs` |
| Facade to OpenRCAD, cutter fallbacks | `zerocad-core/src/mock_kernel/blend.rs` |
| Contour dispatch, law and corner-mode types | `OpenRCAD/crates/openrcad-algo/src/contour.rs` |
| Rolling-ball solver, end-closure cascade, circular chains | `OpenRCAD/crates/openrcad-algo/src/rolling_ball.rs` |
| Selected-edge chamfer | `OpenRCAD/crates/openrcad-algo/src/chamfer.rs` |
| Corner sphere networks | `OpenRCAD/crates/openrcad-algo/src/corner_network.rs` |
| Whole-solid strict ops | `OpenRCAD/crates/openrcad-algo/src/all_edges.rs` |
| Primitive box/cylinder builders | `OpenRCAD/crates/openrcad-algo/src/fillet.rs`, `blend.rs` |
| GUI tool, dialog, preview, commit | `zerocad-gui/src/edgemod.rs` |
| Tangent chain / edge picking | `zerocad-gui/src/app/picking.rs` |
| Inspector editing | `zerocad-gui/src/app/ui/feature_properties.rs` |
| Sketch (2D) corner fillet/chamfer | `zerocad-core/src/sketch.rs` (`CornerMod`, `apply_corner_mod`) |
| Break tests | `break-test/tests/catalog_f_blends.rs`, `edge_blend_break.rs` |

## Appendix B: failure messages and their root causes

| Message users see (abridged) | Root cause | Fixed by phase |
|---|---|---|
| "…pair of surface types has no supported solver path" / `UnsupportedSurfacePair` | §5.1 whitelist | A |
| "chamfer: native selected-edge chamfer currently supports planar-planar edges" | §5.2 | A |
| "selected edge was not found on the current body" / `SpineNotOnFace` | §5.3 addressing | B |
| "endpoint trim topology is not supported" / `UnsupportedTrimTopology` | §5.4 cascade | C |
| `CornerNetwork::IncompleteSelection` / `UnsupportedValence` | §5.4–5.6 | C, D |
| "fillet overflow does not support selected … surfaces" / `NonPrismaticSource` | §5.6 | A, C |
| "setback corners are not supported" | §5.6 | D |
| "variable radius/distance laws are not implemented" | §5.8 | E |

## Sources (Fusion behavior)

- Autodesk Fusion Help, Fillet reference: https://help.autodesk.com/cloudhelp/ENU/Fusion-Model/files/SLD-REF-FILLET.htm
- Autodesk Fusion Help, Create a fillet: https://help.autodesk.com/view/fusion360/ENU/?guid=SLD-FILLET-SOLID
- Autodesk Fusion Help, Chamfer reference: https://help.autodesk.com/cloudhelp/ENU/Fusion-Model/files/SLD-REF-CHAMFER.htm
- Autodesk Fusion Help, Create a chamfer: https://help.autodesk.com/cloudhelp/ENU/Fusion-Model/files/SLD-CHAMFER-SOLID.htm
- CADED LLC, "Fusion Fillets: A deep-dive into how they work": https://www.cadedllc.com/post/fusion-fillets-a-deep-dive-into-how-they-work
- Autodesk blog, "Get Smart With Fusion 360 Part 5: Fillet Best Practices": https://www.autodesk.com/products/fusion-360/blog/get-smart-with-fusion-360-fillet-best-practices/
