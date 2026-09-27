# Fillet and Chamfer Plan: General Blending

*Companion to [Fillet_Chamfer.md](Fillet_Chamfer.md), which is the diagnosis.
Written 2026-09-27. This is a plan only; no code has been changed.*

## 0. Goal, definition of done, and ground rules

### Goal

Any manifold edge between two faces can be filleted or chamfered, whatever the
two surface types, the edge's curve type, or what lies at its ends. The only
failures left are geometrically infeasible requests, and those fail atomically
with a precise diagnostic. ZeroCAD's current guarantees must survive: validated,
all-or-nothing commits; an exact preview that is the same operation as the
commit; durable references; and exact analytic output wherever the geometry
allows it.

### Definition of done (program level)

1. **Coverage.** At least 97% of *feasible* cases in the Blend Coverage Matrix
   (§2) succeed. 100% of *infeasible* cases reject atomically, leave the source
   unchanged, and name a specific cause.
2. **Accuracy.**
   - Volume within 1e-4 relative of the OCCT frozen oracle, or an analytic
     oracle, on every matrix case that both systems accept.
   - Contact tangency residual ≤ `policy.angular`.
   - Contact position error ≤ `policy.linear`.
3. **Fusion option parity:**
   - Fillet: constant, chord length, variable, asymmetric, G1/G2 with tangency
     weight, rolling-ball and setback corners, tangent chain, selection by
     edge/face/feature, rule fillet, full round, multiple selection sets.
   - Chamfer: equal distance, two distances, distance and angle, flip, and
     chamfer/miter/blend corners.
4. **No regressions.** Every existing blend test passes unchanged: the
   `openrcad-algo` blend tests, `zerocad-core` blend tests, `break-test`
   catalog F plus edge_blend_break and research_blend_shell_torture, and GUI
   edgemod tests. Existing analytic cases produce **identical face types**
   (cylinder, torus, cone, sphere) as before.
5. **Performance.** A single straight planar edge is no slower than today
   (±10%). A general curved-support edge takes ≤ 40 ms per 100 mm of spine at
   default tolerance on the reference machine. Preview stays interactive:
   cancellable and budgeted.
6. **Documents.** Documents written by old builds load and evaluate
   byte-identically, apart from bumped cache ABIs. New fields are additive.

### Ground rules

These come from `AGENTS.md`, the kernel `CLAUDE.md`, and lessons in the
project's memory notes.

- **Fast paths stay in front.** Today's analytic builders (`planar_blend`,
  torus rim, cone-frustum chamfer, corner sphere, and the `try_*` closures)
  remain the first choice. The general engine runs **only when they decline**.
  Each phase therefore starts at zero regressions and only grows coverage.
- **One transaction path for GUI and headless.** Blend creation moves to a core
  command builder used by the GUI, tests and scripting (§9).
- **Property sweeps are mandatory.** No fix is accepted from a single fixture.
  Every geometry change runs the scale × rotation × far-origin sweep.
- **Rust constraints:** `#![forbid(unsafe_code)]`, owned-enum geometry, and no
  new dependencies in the kernel crates.
- **Required checks per step:** `cargo fmt --all -- --check`, `cargo check`,
  `cargo test`, and `cargo clippy -p zerocad-core --all-targets`, plus the
  OpenRCAD clippy gate.
- **Hotspots:** `edge_mod.rs` and `rolling_ball.rs` are large and fragile. New
  engine code goes into **new modules** (§1.2). Old files are touched only to
  delegate.

---

## 1. Target architecture

### 1.1 Pipeline after the rework

```
GUI / headless ──► core::blend_command::EdgeBlendBuilder   (validate, canonicalize, persist)
                          │ FeatureType::EdgeBlend (schema v2: sets, laws, chamfer spec)
                          ▼
parametric::edge_mod::apply_edge_blend
   resolve selections (edges | faces | features | rules) → EdgeQuery set
                          ▼
mock_kernel::blend::blend_edge_network  ──►  openrcad::algo::blend_engine::blend(request)
                                                 │
     ┌───────────────────────────────────────────┴───────────────────────────────┐
     │ 1. locate      EdgeQuery → kernel Edge (any curve)          [spine.rs]     │
     │ 2. chain       group into G1 contours, order, orient        [chain.rs]     │
     │ 3. plan        per contour: supports, convexity, law        [plan.rs]      │
     │ 4. solve       contacts: ANALYTIC fast path else MARCH      [contact.rs]   │
     │                (face-hopping overflow, feasibility checks)                  │
     │ 5. surface     band surface: analytic recognize else fit    [band.rs]      │
     │ 6. vertices    classify every vertex → cap/miter/sphere/     [vertex.rs]    │
     │                setback patch (simultaneous, all bands known)                │
     │ 7. trim        imprint contacts + end curves on supports    [trim.rs]      │
     │                (generic SSI-based; try_* closures as fast paths)            │
     │ 8. assemble    replace strips, sew, pcurves, merge           [assemble.rs]  │
     │ 9. validate    native gates (tangency, G1, volume sign,     [certify.rs]   │
     │                locality, self-intersection, strict topology)                │
     │ 10. fallback   boolean tool-body recut if 7–9 fail          [tool_recut.rs]│
     └────────────────────────────────────────────────────────────────────────────┘
                          ▼
   OperationResult { solid, history (face provenance), diagnostics, certificate }
```

### 1.2 New kernel modules

All live under `OpenRCAD/crates/openrcad-algo/src/blend_engine/`:

| Module | Responsibility | Replaces or absorbs |
|---|---|---|
| `mod.rs` | `BlendRequest`, `BlendSpec`, `BlendLaw`, `ChamferSpec`, `CornerPolicy`, entry `blend_operation_with_policy_and_cancel` | `contour.rs` dispatch (kept as a thin shim) |
| `spine.rs` | `EdgeQuery` → `Edge` for any `GeomCurve`; spine parameterization by arc length | `relocate_edge`, `Edge::between_points` requests, `circle_edge_requests` |
| `chain.rs` | G1 chain discovery, contour ordering, closed-loop detection | `is_mixed_tangent_chain`, `circular_spine_from_chain`, `order_closed_*_loop` |
| `plan.rs` | Support faces, material side, convex/concave, value law per station | `planar_edge_material_wedge_is_concave` (reused) |
| `contact.rs` | `ContactSolver` trait; `AnalyticContacts` (existing closed forms) and `MarchingContacts` (new) | curved-face whitelist in `rolling_ball_between_curved_faces` |
| `band.rs` | Band surface construction: analytic recognition, canal-surface B-spline fit, ruled chamfer band, G2 cross-sections | `RollingBallBlend.blend_face` construction |
| `vertex.rs` | Vertex classification and corner patches (cap, miter, sphere, setback N-sided) | `handle_corner_endpoint`, corner cascade, `corner_network.rs` (reused) |
| `trim.rs` | Generic support trimming and end capping via SSI and imprinting | `trim_face_along_spine*`, `trim_face_at_corner` (kept as fast paths) |
| `assemble.rs` | Face replacement, sewing, pcurve completion, coplanar/cocylindrical merge | duplicated assembly tails in `fillet_planar_edge_inner` / `chamfer_planar_edge` |
| `certify.rs` | Native validation and the `BlendCertificate` | scattered watertight/health checks |
| `tool_recut.rs` | General boolean fallback | `fillet/chamfer_straight_edge_network_with_cutters` (ZeroCAD facade) |

`rolling_ball.rs` and `chamfer.rs` keep their public APIs. Internally they
become "fast-path providers" called from `contact.rs`, `band.rs`, `trim.rs` and
`vertex.rs`.

---

## 2. Phase 0: measure, freeze, prepare (foundation, ~1 week)

Nothing may change behaviour in this phase. It builds the yardstick every later
phase is judged against.

### 0.1 Blend Coverage Matrix (census harness)

New file: `zerocad-core/tests/blend_coverage_matrix.rs`, plus a
`--ignored`-free fast subset and a `ZEROCAD_BLEND_CENSUS=full` long mode that
writes `target/blend-census.json`.

**Fixture generators** (a support pair plus an end context):

| ID | Body | Edge under test | Support pair |
|---|---|---|---|
| M01 | box | top edge | plane/plane 90° |
| M02 | wedge 30°/120°/150° | ridge | plane/plane oblique |
| M03 | pocket | inner corner | plane/plane concave |
| M04 | cylinder boss on plate | rim | plane/cyl (rim) |
| M05 | D-shaft | flat-to-round edge | plane/cyl (generator) |
| M06 | cylinder cut by 30° plane | ellipse edge | plane/cyl oblique |
| M07 | boss on pipe (equal / unequal R, offset) | saddle curve | cyl/cyl |
| M08 | cross-drilled holes | saddle (concave) | cyl/cyl concave |
| M09 | drafted round extrude | top rim | plane/cone |
| M10 | drafted polygon extrude | vertical edges | plane/plane (drafted) |
| M11 | revolve with spherical cap | cap rim | sphere/cyl, sphere/plane |
| M12 | filleted box (prior fillet) | edge between band and side face | torus/plane, cyl/plane |
| M13 | loft rectangle→circle | loft/cap edge | B-spline/plane |
| M14 | sweep along arc | sweep side edges | ruled or B-spline/plane |
| M15 | FreeCAD STEP fixture (existing) | every edge | mixed NURBS |
| M16 | helical thread flank | crest edge | ruled/helix (expected infeasible or chamfer only) |

**Axes per fixture:**

- radius or distance at {0.1, 0.3, 0.6, 0.95, 1.2} × the local feasible
  maximum;
- scale {1e-2, 1, 1e3};
- rotation {identity, (17°, 31°, 47°)};
- origin {0, (1e4, −3e3, 7e3)};
- kind {fillet, chamfer};
- end context {free ends to caps, meets fillet, partial corner, overflow}.

**Recorded per case:**

- outcome (ok, or reject with its error class);
- wall time;
- native volume against the oracle;
- tangency residual and face-type census.

This gives the **baseline success rate per cell**. It is committed as
`docs/evidence/blend-census-baseline.json`.

### 0.2 Oracles

- Extend `tools/phase7_occt_oracle.py` into `tools/blend_occt_oracle.py`, using
  `BRepFilletAPI_MakeFillet` and `BRepFilletAPI_MakeChamfer` (including
  `AddDA` and two-distance). Generate frozen volume, area and centroid for
  every matrix case. Same rules as today: the input is SHA-pinned and OCCT is
  never linked.
- **Analytic oracles** where closed forms exist (plane/plane, rim tori, cones),
  following the pattern of `docs/fillet-boss-junction-2026-09-13.md`.
- **Native volume for all surface types.** Extend the boundary-integration
  volume (currently plane and cylinder) to cone, sphere, torus, B-spline,
  ruled and Gregory surfaces, using Gauss–Legendre quadrature over trimmed
  pcurve domains. This replaces display-mesh volume, which the junction doc
  showed is unreliable.

### 0.3 Kernel prerequisites (no behaviour change)

| Item | Where | Why |
|---|---|---|
| `GeomSurface::d2(u,v)` (and `normal_d1`) for every variant | `openrcad-geom` (analytic closed forms; B-spline via `bspline_derivatives.rs`; Offset/Ruled/Gregory derived) | The marching Jacobian needs normal derivatives, and the curvature feasibility check needs the Weingarten map |
| `principal_curvatures(u,v)` helper | `openrcad-geom/src/surface.rs` | Radius feasibility (§3.4) |
| Curve fitting: `BSplineCurve::interpolate(points, params, degree, end_tangents)` and `approximate(points, tol)` | `openrcad-geom/src/bspline_curve.rs` (reuse `smooth_skin::interpolation_knots`) | Contact curves and centre curve. Today only degree-1 `polyline_to_bspline` exists in `intersect.rs` |
| Surface fitting from cross-section curves | promote `smooth_skin::interpolate_surface` to `openrcad-geom` | Band surfaces |
| **Adaptive tessellation** for BSpline, Gregory, Offset and Ruled | `openrcad-mesh/src/triangulate.rs:970` (currently a fixed 10×10 grid) | General bands must render smoothly and pass the crack and sag gates |
| SSI bounds | `intersect.rs` clamps parameters to ±100 | Use face parameter bounds plus margin, so large parts are not clipped |
| Doc fixes | `FeatureType::EdgeMod` doc comment; `OpenRCAD/CLAUDE.md` "Gauss-Newton curved–curved" claim | Stop the drift |

**Exit criteria:**

- The census baseline is committed.
- The OCCT oracle JSON is committed.
- `d2` and curvature have unit tests against finite differences for every
  variant.
- The fitting round-trips sample data within tolerance.
- Adaptive tessellation passes the existing mesh tests.
- Zero behaviour change: the census baseline is identical before and after.

---

## 3. Phase A: general contact solver (the core unlock)

### 3.1 Spine and support model (`plan.rs`)

`ContourPlan` holds:

- `spine: SpineCurve` (arc-length parameterized, any `GeomCurve`);
- `stations`, created lazily;
- for each side: `support_faces: [Vec<FaceId>; 2]` (a list, to allow face
  hopping), the material-side sign, and `convex: bool`;
- `law: ValueLaw` (a constant in this phase).

Convexity comes from the existing `edge_material_wedge_is_concave`, generalized
to curved supports: probe at the spine midpoint using the surface normals and a
material ray test.

### 3.2 Marching solver (`contact.rs::MarchingContacts`)

For a spine parameter `s`, the unknowns are `x = (u_a, v_a, u_b, v_b)`.
Let `P_x`, `N_x` be point and outward normal, `σ = −1` for a convex blend
(ball inside the material wedge offset inward) and `+1` for concave, and
`T(s)` the spine tangent.

```
F1..3:  P_a + σ r(s) N_a  −  (P_b + σ r(s) N_b)  = 0          (common ball centre)
F4:     (C(x) − spine(s)) · T(s)                  = 0          (cross-section plane)
        where C = P_a + σ r N_a
```

- **Jacobian:** analytic, from `d1` and the normal derivatives (from `d2`).
- **Correction:** damped Newton with line search and trust-region clamping in
  parameter space. Seam and periodicity handling uses the existing
  `SurfacePeriodicity`.
- **Seed:** solve the 2D cross-section problem at `s0`. Intersect the two
  support curves of the section plane (linearized with local tangents), offset
  by r, then refine.
- **Continuation:** predictor using the tangent of the solution curve, from the
  nullspace of the 4×5 extended Jacobian. Adaptive step control on:
  - chord deviation of the centre curve ≤ `policy.approximation`;
  - angular change of the contact normals ≤ `policy.angular`;
  - a halving rejection loop.
- **Stops when:**
  - the spine end is reached, which gives the end stations to Phase C;
  - a contact leaves its support face's trimmed domain (an **overflow event**,
    recorded with the crossing boundary edge, see Phase D);
  - Newton diverges, giving `RollingBallError::NewtonDiverged` for real;
  - feasibility is violated (§3.4).
- **Output:** a `ContactSolution` holding stations `(s, x, C, r)`, fitted
  contact curves (`BSplineCurve::approximate`), pcurves on both supports, and
  the centre curve.
- **Plumbing:** uses `GeometryWorkBudget` (new stage `BlendMarch`) and
  `CancellationProbe` throughout, so the preview worker can abort.

### 3.3 Analytic-first dispatch

```
contact(plan) =
    AnalyticContacts::try(plan)          // existing closed forms, unchanged output
    .or_else(|| MarchingContacts::solve(plan))
```

`AnalyticContacts` wraps today's `planar_blend`, `planar_blend_concave`, the
rim torus, `rolling_ball_plane_perp_cylinder` and the circular-chain cases. It
**adds** further closed forms as cheap wins:

- plane/cone rim (torus);
- sphere/plane and sphere/coaxial cylinder (torus);
- equal-radius cylinder/cylinder with intersecting axes and the fillet in the
  symmetry plane (a known canal surface; left to the fit).

### 3.4 Feasibility checks (typed rejections, never broken bodies)

- **Convex support curvature.** At every station, r must be less than the
  principal radius of curvature on each support in the cross-section direction
  wherever that support is concave toward the ball. Otherwise the result is
  `RadiusTooLarge { station, limit }`.
- **Centre-curve curvature.** The centre curve's radius of curvature must
  exceed r, otherwise the pipe self-intersects:
  `BandSelfIntersection { station }`.
- **Dihedral.** A contact angle below `policy.angular` means the faces are
  tangent and there is no edge to blend: `InvalidDihedral`.
- Every error carries **the maximum feasible value** (found by bisection on r at
  the failing station). The GUI shows it: "max ≈ 2.37 mm here".

### 3.5 Band surfaces (`band.rs`)

- **Analytic recognition first.** Fit the station data to a cylinder, torus,
  cone or sphere. If every station lies within `policy.linear`, emit the exact
  surface with exact circular or line contacts. This extends today's analytic
  behaviour automatically to newly solved pairs whose result is a quadric
  (e.g. plane/cone rim).
- **Otherwise, a canal-surface B-spline.** Each cross-section is an **exact
  rational quadratic arc** from contact A to contact B about `C(s)`. Loft the
  sections along `s` with the promoted `interpolate_surface`, using degree 3
  along the spine.
  - Refine the knots until the deviation from a true sphere-sweep, sampled
    between stations, is ≤ `policy.approximation`.
  - The boundary curves **are** the contact B-splines, with shared control
    points, so the edges are identical and sewing is exact.
- **Chamfer band.** `RuledSurface` between the two rails, or a plane when the
  rails are coplanar lines (unchanged fast path).
- **Orientation.** The band normal points out of the material. This is checked
  by the certifier; the project memory records orientation bugs here before.

### 3.6 Chamfer on general supports

A rail is the curve on the support at a *cross-section distance* d from the
spine: the intersection of the support with the cross-section plane, walked a
distance d. That walk is solved with the same station machinery, using one
unknown `(u,v)` per side plus the distance constraint. Fusion measures along
the face, and the matrix records this convention explicitly.

### 3.7 Wiring

- In `rolling_ball_fillet_edge_with_policy`, replace the terminal
  `UnsupportedSurfacePair` with a call into
  `blend_engine::contact::solve_general`. Adapt the `ContactSolution` into the
  existing `RollingBallBlend` shape (spine, faces, contacts, arcs, face), so
  **every downstream closure keeps working**.
- In `chamfer_planar_blend`, replace the plane-only gate the same way,
  producing a `ChamferBlend`.
- Circular-hint and tangent-chain paths still take their existing fast paths
  first.

### 3.8 Tests and exit criteria

- **Unit tests:** marching reproduces the analytic torus and cylinder bands on
  the M01/M04/M05 cases within 1e-9 when analytic dispatch is forced off.
- **Matrix cells** M06–M09 and M11–M14 with *free or simple planar caps* go
  from 0% to ≥95% feasible success, and their volumes match the OCCT oracle
  within 1e-4.
- The infeasible column (1.2× max) is 100% typed rejection with
  `max_feasible` reported.
- Existing tests are unchanged, and face types are identical on
  pre-existing-supported cells.
- Performance budget met (§0).

---

## 4. Phase B: general edge addressing

### 4.1 Persisted reference (additive)

Add these optional fields to `EdgeRef`, all `#[serde(default)]`, so old files
still load:

```rust
pub mid: Option<[f64; 3]>,          // point at curve mid-parameter (f64, not f32)
pub mid_tangent: Option<[f32; 3]>,
pub p0_f64: Option<[f64; 3]>, pub p1_f64: Option<[f64; 3]>,  // exact endpoints
pub closed: bool,
```

- `EdgeCurveHint` gains `Ellipse {..}` and `Other { kind: String }`. Existing
  `Line` and `Circle` are unchanged.
- The `MeshEdgeRef` producer (`populate_edge_adjacent_face_names`, and the
  tessellation that emits `edge_refs`) fills these from the **B-rep edge**, not
  the mesh. This removes the f32 snap hack.

### 4.2 Kernel locator (`spine.rs::locate`)

`EdgeQuery { p0, p1, mid, tangent, curve_kind, closed }` is resolved in this
order:

1. exact endpoints plus a midpoint on the curve, within `policy.linear`;
2. a closed edge by its midpoint and tangent;
3. a surviving sub-span after earlier trims, generalized from collinear lines
   to **any curve**: project the query points onto candidate curves and keep
   the largest parameter overlap.

Ambiguity (two candidates within tolerance) is an **error**, never a first
match. This is consistent with break-test F06.

- `mock_kernel/blend.rs` stops constructing `Edge::between_points` requests.
  `circle_edge_requests` becomes one case of `locate`.

### 4.3 Exit criteria

- M06, M07 and M16 edges (ellipse, saddle B-spline, helix) are selectable,
  persist, reload, and resolve after an upstream dimension edit, with the same
  reattachment matrix as `reattachment_matrix.rs`.
- Old documents are unchanged.

---

## 5. Phase C: general trimming and end capping

### 5.1 Support trimming (`trim.rs::trim_support`)

- Imprint the contact curve (with its pcurve from Phase A) onto the support
  using `imprint_curve_on_face` and `BRepBuilder::partition_face`. Discard the
  sub-face that touches the spine, classified by a point on the spine side.
- Existing `trim_face_along_spine*` stay as the fast path for straight
  contacts on planes.

### 5.2 Generic end capping (`trim.rs::cap_end`)

This replaces the "recognizer or reject" behaviour. At a spine endpoint `V`
whose blend is not continued by another selected band:

1. **Extend.** Continue the band surface and both contact curves past `V` by
   `δ = r · k` (k = 2), using parameter-space extension. That is exact for
   analytic bands; B-spline bands get a tangent-continuous end extension.
2. **Gather the face fan at V.** This is every face incident to `V` other than
   the two supports, plus faces reachable within `δ` along the incident edges.
   It gives a list, so **several distinct cap faces are allowed**. That removes
   the `IncompleteSelection` rejection for plain partial corners.
3. **Intersect.** Use `surface_surface_curves_with_budget_and_cancel` between
   the extended band and each fan face, and between each extended contact
   support and the fan faces. This yields the **end boundary curve chain**:
   band∩cap arcs joined at contact∩cap points.
4. **Classify and trim.** Split each fan face along its intersection curve and
   remove the sub-region between `V`, the contacts and the band. Trim the band
   at the chain.
5. **Validate the local patch.** The loop must be closed, oriented, and have
   G0 continuity at the chain vertices.

Today's `try_corner_cut`, `try_oblique_planar_cap`,
`try_perpendicular_planar_setback`, `try_tangent_curved_wall_runout` and the
planar `handle_corner_endpoint` all become **fast-path specializations** of
`cap_end`, run first because they are exact and cheap. The generic path runs
when they decline. The correctness contract is that fast path and generic path
agree within tolerance on all their shared cells. A test runs both and
compares.

### 5.3 Runout

If the support becomes tangent to the neighbouring face at `V` (dihedral → 0),
the contact solution converges to the spine and the band narrows naturally.
The marcher stops at `width ≤ policy.linear`, and the band ends in a
degenerate (collapsed) edge. Faces of this kind are already tolerated by the
torus and horn handling; the certifier accepts the collapse only if it is
geometric.

### 5.4 Boolean tool fallback (`tool_recut.rs`)

This is the last resort before rejection, generalizing today's orthogonal-only
cutters.

- **Tool solid (convex case):** the region bounded by the band, the two
  support strips between contact and spine, and extended end planes normal to
  the spine at ±δ. This is the removed "crescent", built as a closed solid
  from exact faces.
- **Tangency hazard.** The tool's band meets the supports tangentially at the
  contacts. Mitigate this by **imprinting the contact curves onto the body
  first** (§5.1 without removal). The boolean then meets existing edges, not a
  tangential surface intersection. This is the same principle that made the
  existing box-minus-cylinder cutter robust.
- **Concave case:** the tool is the added wedge, applied by union.
- The result must pass the same `certify` gates. There is no weaker acceptance
  for the fallback.

### 5.5 Exit criteria

- Matrix end-context axis: "free end", "oblique cap", "cap is curved (cylinder,
  cone)", "cap fan of 2–3 distinct faces" and "end at a cut cylinder" all
  reach ≥95% feasible success on every support pair from Phase A.
- F02 (partial chains at three-way corners) goes from "terminate or reject" to
  "terminate cleanly" for every subset.
- Every `try_*` closure has an agreement test against the generic `cap_end`.

---

## 6. Phase D: networks, vertices and overflow

### 6.1 Always-simultaneous solve

`blend_engine::blend` solves **every contour against the original body**
before any trimming. Vertices are then planned with complete knowledge.

- Remove the sequential loop in `fillet_edges_with_policy` and
  `chamfer_edges_with_policy`. Keep it behind a
  `#[cfg(feature = "legacy-sequential")]` flag for one release, then delete it.
- **Determinism:** contours and vertices are processed in canonical order (by
  durable names, then geometry), never by selection order.

### 6.2 Vertex classification (`vertex.rs`)

For each vertex `V` touched by the selection:

| Selected bands at V | Unselected edges at V | Construction (Auto) |
|---|---|---|
| 1 | any | **cap** (Phase C `cap_end`) |
| 2, G1-continuous | – | **chain joint**: no patch; bands share a cross-section |
| 2 | ≥1 | **miter**: band∩band SSI seam, both bands trimmed (a generalized `try_corner_miter`) |
| ≥3, equal r, all planar | – | **sphere**: `CornerTangentSphere` fast path (unchanged) |
| ≥3, otherwise | – | **setback patch** (§6.3) |
| ≥2 | ≥1 and mixed | **mixed**: miter the selected pair where it is feasible, else setback |

`CornerPolicy` (renamed from `EdgeCornerMode`, with the serde name kept):

- `Auto` follows the table.
- `Miter` forces a miter, rejecting where that is impossible.
- `RollingBall` forces a sphere or rolling-ball patch.
- `Setback { distance: Option<f64> }` forces a setback. `Setback` alone stays
  deserializable.

### 6.3 Setback N-sided patch

- **Setback distance** is per vertex, defaulting to `1.5 × max(r_i)`. Each
  incident band is trimmed back to a cross-section at that distance from `V`.
- **Boundary:** N band-end arcs, alternating with N support-face trim curves
  where unselected edges meet. This is a 2N- or N-sided hole.
- **Fill:** split into N quadrilateral sub-patches around a centroid point.
  Each is a 4-sided `GregorySurface` (it already exists and is G1 at its
  boundary). Internal boundary cross-derivatives are matched by the standard
  Gregory/Charrot–Gregory construction.
  - The centroid position and normal are the average of the boundary data,
    projected.
  - G1 across the outer boundary uses the band and support tangent planes.
- **Certify:** the G1 angle along every internal and external boundary must be
  ≤ `policy.angular`.
- `RollingBall` with unequal radii or non-planar supports uses the same filler,
  with setback 0 at the ball contacts. The rolling-ball corner is then the
  limit case.

### 6.4 Overflow and face hopping

When the marcher reports an overflow event (a contact crossing a boundary edge
`e` of support A):

1. **Roll over (default).** Switch the support to the face `A'` across `e` and
   continue marching.
   - If `A` and `A'` are G1 across `e`, the continuation is smooth.
   - Otherwise it is a kink: split the band at that station, and the contact
     gets a vertex.
2. **Cliff edge.** If the roll-over is infeasible (a sharp convex `e` makes the
   ball jump), keep `e` sharp and trim the band against `A'` using
   `cap_end`-style SSI along the whole overflow span.
3. **Policy:** `OverflowPolicy { Auto, RollOver, CliffEdge }` is persisted and
   defaults to `Auto` (roll over when feasible, else cliff).

The existing planar prism overflow reconstruction remains the exact fast path
for its cell.

### 6.5 Exit criteria

- Networks: open L/U/Z chains, two separate corners in one feature, trihedral
  corners with unequal radii, mixed plane/cylinder corners, and 4- and 5-valent
  corners all reach ≥95% feasible success.
- Setback G1 is certified.
- Results are independent of selection order (hash-identical solids under
  permutation).
- Overflow: plane/plane, plane/cylinder and cylinder/cylinder with r greater
  than the face width all roll over or cliff correctly, with volumes matching
  the oracle.

---

## 7. Phase E: value laws, continuity and chamfer types

### 7.1 Kernel types (`blend_engine/mod.rs`)

```rust
pub enum ValueLaw {
    Constant(f64),
    ChordLength(f64),                         // fixed contact-to-contact width
    Variable(Vec<LawPoint>),                  // LawPoint { s: f64 /*0..1 arclength*/, value: f64 }
    Asymmetric { a: f64, b: f64 },            // setbacks on side A / side B (conic section)
}
pub enum Continuity { G1, G2 { weight: f64 /* 0.1..=2.0 */ } }
pub enum ChamferSpec {
    Equal(f64),
    TwoDistances { a: f64, b: f64 },
    DistanceAngle { distance: f64, angle_deg: f64 },
}
pub enum ChamferCorner { Chamfer, Miter, Blend }
```

### 7.2 Semantics

- **ChordLength.** At each station, r follows from the chord width `c` and the
  local contact dihedral φ: `r = c / (2·cos(φ/2))`. That needs a nested solve
  per station (r depends on φ, which depends on the contacts). The analytic
  plane/plane case has constant φ and so a constant cylinder.
- **Variable.** Monotone cubic (Fritsch–Carlson) interpolation of r over spine
  arc length. On plane/plane with a linear law the band is **exactly a cone**
  (centre line straight, radius linear); detect that analytically. Otherwise it
  is a canal B-spline.
- **Asymmetric.** Contacts at setbacks a and b. The cross-section is a conic
  (a rational quadratic with a computed weight), which gives an elliptical
  cylinder on plane/plane. That shape is a B-spline or an exact
  elliptic-section ruled sweep.
- **G2.** The cross-section is a quintic or rational quartic that matches the
  support curvature at each contact, with the tangency weight scaling the
  inner control points (Fusion's 0.1–2 range). The band is always a B-spline.
  The certifier additionally checks curvature continuity across contacts
  (relative curvature jump ≤ 1e-3).
- **TwoDistances.** Rails at a (side A) and b (side B). "Flip" swaps the sides.
  Side A is **persisted by durable face identity** (see §8), not by the
  arbitrary `n1`/`n2` order. The project memory records that normal order is
  arbitrary.
- **DistanceAngle.** Rail A at distance d, and the bevel leaves at angle θ from
  face A in the cross-section. Rail B is the ray∩support-B solve per station.
  On plane/plane this is `b = d·sinθ / sin(φ−θ)`, which is closed form.
- **Chamfer corners:**
  - `Chamfer` gives a planar or ruled triangular corner face.
  - `Miter` intersects the bevel faces at a point or seam.
  - `Blend` uses a Gregory fill as in §6.3, but G0 or G1 to the bevels.

### 7.3 Exit criteria

- Each law has matrix cells on plane/plane (analytic oracle) and cyl/plane,
  cone/plane and B-spline/plane (OCCT oracle: `SetRadius` law for variable,
  `AddDA` and two-distance for chamfers).
- G2 is certified by curvature sampling.
- The `BlendLaw::Variable` placeholder is removed, so
  `VariableLawUnsupported` disappears.

---

## 8. Phase F: document schema and selection model

### 8.1 `FeatureType::EdgeBlend` payload schema v2

Numeric field slots follow `feature_dto.rs` and are additive.

| Slot | Field | Default for older payloads |
|---|---|---|
| 0–5 | existing: target, edges, dist, dist_expr, kind, corner_mode | – |
| 6 | `law: ValueLawDto` (with `*_expr` per value) | `Constant(dist)` |
| 7 | `continuity: Continuity` | `G1` |
| 8 | `chamfer: Option<ChamferSpecDto>` (+exprs) | `Equal(dist)` when kind = Chamfer |
| 9 | `extra_sets: Vec<BlendSetDto>` (each with its own edges/selection plus law) | empty |
| 10 | `selection: SelectionSpec` (see §8.2) | `Edges` (implied by slot 1) |
| 11 | `tangent_chain: bool` | `false` for old payloads (preserves old results); `true` for new GUI features |
| 12 | `overflow: OverflowPolicy` | `Auto` |
| 13 | `setbacks: Vec<(VertexRef, f64)>` | empty |
| 14 | `reference_faces: Vec<FaceRef>` (side A per edge, used for flip, two-distance and angle) | empty, meaning the legacy side rule |

- Bump the `part.edge_blend` payload schema and register the decoder. The v1
  decoder maps to v2 defaults.
- Bump `OPENRCAD_CACHE_ABI` and `MESH_CACHE_ABI` when geometry output changes.
- Serialization tests: v1 fixtures round-trip, v2 round-trips, and unknown
  future slots are rejected loudly (the existing behaviour).
- The legacy `EdgeMod` stays readable. The evaluator converts it into a
  one-edge `EdgeBlend` request internally, so there is one engine path.

### 8.2 Selection model (associative, like Fusion)

```rust
pub enum SelectionSpec {
    Edges,                                         // slot 1 edges (+tangent chain)
    Faces(Vec<FaceRef>),                           // all boundary edges of these faces
    Features(Vec<FeatureId>),                      // edges whose producer_feature_id matches
    Rule { rule: RuleKind, topology: RuleTopology }, // AllEdges | BetweenFaces{a,b} × Rounds|Fillets|Both
    AllEdgesStrict(AllEdgeSelector),               // today's strict selector, unchanged semantics
}
```

- Expansion happens **at evaluation time** against the current body, using
  durable names. Faces and features are associative by design, like Fusion: an
  upstream edit that adds an edge to a selected face includes that edge. The
  evaluator emits an **informational diagnostic** listing added and removed
  edges, so the change is visible.
- Convexity filtering (Rounds, Fillets, Both) uses the Phase A `plan.rs`
  convexity classifier.
- The strict all-edge selector keeps its refuse-on-change semantics. Users can
  choose between "Strict" and "Associative" in the dialog.
- **Tangent chain** expansion happens at evaluation time too. The persisted
  seeds plus the flag guarantee that chains follow upstream edits.

### 8.3 Full round fillet

This is a separate `SelectionSpec`-like payload on the same feature:
`FullRound { center: FaceRef, side_a: FaceRef, side_b: FaceRef }`.

- **Kernel:** the unknown r(s) is solved so the ball is tangent to *three*
  surfaces: side A, side B and the removed centre face's replacement
  constraint. The marcher gets a fifth unknown r, with the tangency to both
  sides and a centre-face contact equation. The centre face is removed and the
  band replaces it.
- **Exit:** a rib top becomes a semicylinder (analytic oracle), and tapered
  ribs give a variable-radius band (OCCT oracle via a variable fillet).

---

## 9. Phase G: core command API and GUI

### 9.1 Core command (shared by GUI and headless)

New file `zerocad-core/src/parametric/blend_command.rs` with
`EdgeBlendBuilder`:

- `new(target)`
- `.edges(refs)`, `.faces(..)`, `.features(..)`, `.rule(..)`
- `.fillet(law)` / `.chamfer(spec)`, `.continuity(..)`, `.corner(..)`,
  `.overflow(..)`, `.tangent_chain(bool)`
- `.add_set(..)`
- `.build(&Document) -> Result<FeatureNode, BlendBuildError>`

`build` does the canonicalization, value validation against the policy floor,
and expression binding. It returns the feature, which the caller adds within
its transaction.

- `commit_edge_mod` and `begin_all_edge_mod` in `zerocad-gui/src/edgemod.rs`
  use the builder. There are no more ad-hoc `FeatureType::EdgeBlend { .. }`
  literals outside core.
- `preview_blend(document, builder) -> PreviewResult` runs the same eval path.
  The worker uses it.

### 9.2 Dialog (Fusion-equivalent, keeping the inline-first feel)

Layout: a compact inline box as today, expandable into a panel.

| Control | Fillet | Chamfer |
|---|---|---|
| Type | Fillet / Rule / Full Round | Equal / Two distances / Distance-Angle |
| Size | value + expression; radius type Constant / Chord / Variable / Asymmetric | d, or (a, b), or (d, θ) + **Flip** |
| Variable | click points on the spine to add radius handles (drag handles) | – |
| Continuity | G1 / G2 + weight slider | – |
| Corners | Auto / Rolling ball / Setback (+distance) / Miter | Chamfer / Miter / Blend |
| Selection | Tangent chain ☑ (default on); Edges / Faces / Features picker; **+ Add set** | same |
| Overflow | Auto / Roll over / Cliff | Auto |
| Rule | All edges / Between faces; Rounds / Fillets / Both | same |

- **Value limits.** Remove the hard `0.2` floor in `commit_edge_mod` and the
  `0.2..=40` inspector slider. The minimum becomes `k · policy.linear` via the
  builder, and the slider range derives from the body's bounding-box diagonal.
  The inline field, inspector and commit all use the same validation.
- **Feedback on failure.** Use `AllEdgeBlocker` and the new
  `RadiusTooLarge { max_feasible, station }` to highlight blocking edges in red
  in the viewport and show "max ≈ X mm" next to the field. Fusion's equivalent
  is the red edge plus warning.
- **Edit feature.** Double-clicking an `EdgeBlend` reopens the dialog
  pre-filled, including re-selecting edges. Today the inspector only edits
  value, kind and corner.
- **Radius matching.** Hovering an existing band during the tool copies its
  radius (a Fusion convenience).
- The exact preview gating stays: commit is refused until the exact preview
  passes with no warnings.

### 9.3 Legacy cleanup at the application layer (`edge_mod.rs`)

- Delete `sketch_source_alternate_parts` usage in the fillet and chamfer paths
  (the "box-cylinder sketch" and "faceted sketch" retries) **once Phase C's
  census shows zero cases depending on them**. Keep a census flag to prove it.
  After that, blend results depend only on the current B-rep.
- Replace the mesh-based containment and locality gates with `certify.rs`
  native gates, keeping the mesh crack gate for display.
  `EDGE_MOD_CONTAINMENT_TOL` becomes policy-relative.
- The circular-bite locality code is removed with the alternates. Its
  regression tests must still pass through the native certifier.

---

## 10. Certification (`certify.rs`), applied to every result and every path

A `BlendCertificate` must pass **all** of these before commit:

1. **Topology:** watertight, strict validation, healthy, and pcurves complete
   (existing checks).
2. **Tangency:** at N samples along each contact, the band and support normals
   agree within `policy.angular` (G1). For G2, the curvature jump is also ≤
   1e-3 relative.
3. **Position:** contacts lie on their supports within `policy.linear`, and
   band sections match r(s) within `policy.approximation`.
4. **Material sign:** the native volume change has the expected sign
   (convex removes, concave adds), and its magnitude lies within the
   cross-section integral bounds computed from the station data (a cheap
   independent estimate).
5. **Locality:** every new face lies within the blend envelope, i.e. the
   distance to the spine is ≤ `r · (1 + 1/sin(φ_min/2)) + tol`. Untouched faces
   are bit-identical except where they were trimmed.
6. **No self-intersection:** no band self-intersection, and no band∩unrelated
   face intersection outside the recorded trims. This is a BVH-pruned SSI
   spot-check.
7. **History:** every output face maps to either a source face (trimmed) or a
   generated role (band, corner patch, cap trim), with the operation id. The
   feeds `named_edge_mod_result_mesh` naming, and complete history is required
   (as in the all-edge contract).

A failure returns the source unchanged, the failing gate, and the station or
face involved.

---

## 11. Test strategy (summary)

| Layer | Tests |
|---|---|
| Geometry units | `d2` and curvature against finite differences for every surface; curve and surface fitting round-trip; Gregory N-sided G1 |
| Solver units | marching reproduces analytic bands; seam crossing; periodic supports; divergence detection; feasibility bisection |
| Engine integration | per-module tests in `openrcad-algo/tests/blend_engine_*.rs`; fast-path vs generic agreement tests for each `try_*` |
| Coverage matrix | `blend_coverage_matrix.rs` quick subset in CI; full census nightly with the success-rate ratchet (it can only go up; a drop fails CI) |
| Oracles | OCCT frozen JSON plus analytic oracles; native volume on every accepted case |
| Metamorphic | selection-order permutation invariance; rigid-motion invariance; scale invariance with a scaled radius; fillet(r1)→edit(r2) equals a direct fillet(r2) |
| Documents | v1/v2 payload round-trip; old `.zcad` fixtures in `zerocad-core/tests/fixtures` evaluate identically; reattachment after upstream edits for edge, face and feature selections |
| GUI | edgemod dialog tests: builder parity, the preview gate, limit handling, blocker highlighting |
| Performance | extend the `perf_*` harnesses: per-edge planar (±10% of baseline), marching budget, preview cancellation latency < 50 ms |
| Break-test | catalog F expanded with the new cells, and `edge_blend_break` gains curved-support and network torture |

---

## 12. Sequencing, milestones and effort

Estimates assume one engineer familiar with the codebase and include tests.

| Milestone | Contents | Est. | User-visible result |
|---|---|---|---|
| **M0** | Phase 0: census, oracles, `d2`/curvature, fitting, adaptive tessellation, doc fixes | 1–1.5 wk | none (baseline published) |
| **M1** | Phase A: marching contacts, band fit, general chamfer rails, feasibility with max-value | 3–4 wk | Curved-support edges with simple ends start working: cones, cyl/cyl, oblique cuts, prior fillets, lofts, STEP |
| **M2** | Phase B: EdgeQuery persistence and locator | 1 wk | Ellipse and spline edges selectable and stable |
| **M3** | Phase C: generic trim, `cap_end`, runout, boolean fallback | 3 wk | "Unsupported trim topology" mostly disappears; partial corners and odd caps work |
| **M4** | Phase G (part): builder API; tangent chain for straight seeds; floor fix; blocker highlighting; edit-feature | 1.5 wk | Fusion-like selection feel; clearer failures |
| **M5** | Phase D: simultaneous networks, vertex classifier, setback N-sided patches, overflow hopping | 4 wk | Complex corners, unequal radii, radius larger than face |
| **M6** | Phase E + F: laws, G2, chamfer types, schema v2, selection sets, face/feature/rule selection | 3–4 wk | Chord, variable, asymmetric, G2; two-distance, angle and flip chamfers; rule fillets |
| **M7** | Full round; remove application-layer alternates; native certifier replaces mesh gates; legacy sequential path deleted | 1.5 wk | Full round; results independent of construction history |

Total is roughly 18–21 weeks. M1–M3 deliver most of the "almost any edge"
feel (about 8 weeks). M4 can run in parallel with M3 because it is GUI and
core only.

**Release gating.** Each milestone ships behind the census ratchet. A milestone
is accepted only when:

- its target cells reach ≥95% of feasible cases;
- there is zero regression on previously green cells;
- the oracle agreement holds;
- all required checks pass.

---

## 13. Risks and mitigations

| Risk | Mitigation |
|---|---|
| Marching fragility (divergence, branch jumping at near-tangent supports) | Trust-region Newton; step halving; branch continuity check (the contact normal angle may not jump); fallback to the boolean tool recut; typed failure with the station |
| B-spline bands harm boolean and shell robustness downstream | Analytic recognition first; exact shared boundary curves; the Phase 0 SSI bounds fix; add B-spline bands to the shell and boolean torture tests in M1 |
| Tangential boolean in the tool-recut fallback | Imprint contacts before the boolean (§5.4); the fallback must pass the same certifier |
| Regressions in the fragile existing closures | Fast paths stay first and are unmodified; agreement tests; census ratchet |
| Performance of general marching in the live preview | `GeometryWorkBudget` plus cancellation; preview at coarse `policy.approximation`, commit at full; the existing LRU of solved sizes |
| Schema growth breaks `.zcad` compatibility | Additive numeric slots; a v1 decoder; fixture round-trip tests; cache ABI bumps only |
| Associative face/feature selections change results silently after edits | The added/removed-edge diagnostic is always shown; a Strict option is available |
| The fixed-grid tessellation of new surface types produces cracks | Phase 0 adaptive tessellation is a hard prerequisite for M1 |
| Scope creep toward "prove every corner" | The census defines done: ≥97% of feasible cases, not 100% of the imaginable |

---

## 14. What stays exactly as is

- Atomic commit semantics and the rule that commit requires an exact, warning-free
  preview.
- Durable naming and the reattachment contract, including a deliberate failure
  instead of a silent jump.
- Today's analytic fast paths and their exact output types.
- The `RollingBallError` and `ChamferError` public variants. New variants are
  added; none are removed except `VariableLawUnsupported`, whose placeholder
  goes away.
- Release profile `panic=unwind` (booleans rely on `catch_unwind`).

## Appendix: first concrete tasks (M0 kickoff checklist)

1. Create `zerocad-core/tests/blend_coverage_matrix.rs` with generators
   M01–M05 (currently green) and M06–M09 (currently red). Record the baseline.
2. Create `tools/blend_occt_oracle.py`, freeze the results, and commit the JSON
   plus SHA pins.
3. Add `GeomSurface::d2` and `principal_curvatures`, with finite-difference
   tests for all 9 variants.
4. Add `BSplineCurve::interpolate` and `approximate`, and promote
   `interpolate_surface` into `openrcad-geom`.
5. Add adaptive tessellation for BSpline, Gregory, Offset and Ruled faces,
   replacing the fixed 10×10 grid in `triangulate.rs:970`.
6. Replace the ±100 parameter clamp in `intersect.rs` SSI with face bounds.
7. Extend native volume integration to all surface types.
8. Fix the two stale docs (the `FeatureType::EdgeMod` comment, and
   `OpenRCAD/CLAUDE.md` Phase 5).
