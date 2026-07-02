# OpenRCAD Development Guide

## Project Overview
OpenRCAD is an optimized, lightweight, pure Rust rewrite of OpenCASCADE, informed by the `truck` CAD kernel. It targets proprietary-grade robustness in a minimal, zero-dependency (beyond `serde`) codebase that compiles natively and to WebAssembly.

## Project Structure
- `crates/`: The Rust workspace implementing the kernel:
  - `openrcad-foundation`: Core math (`gp`-style `Pnt`/`Vec`/`Dir`/`Trsf`), tolerances, bounding boxes. No internal deps.
  - `openrcad-geom2d`: 2D curves (lines, conics, B-splines).
  - `openrcad-geom`: 3D curves and surfaces (lines, conics, NURBS, planes).
  - `openrcad-topo`: B-Rep topology (Vertex, Edge, Wire, Face, Shell, Solid).
  - `openrcad-primitives`: Solid builders (box, cylinder, cone, sphere, wedge).
  - `openrcad-algo`: Boolean operations, fillets, chamfers, offsets, sewing.
  - `openrcad-mesh`: B-Rep to triangle mesh tessellation (+ `GpuMesh` render buffers).
  - `openrcad-exchange`: STEP (AP242) read/write and STL write.
  - `openrcad-sketch`: 2D sketches / closed profiles (parametric intent).
  - `openrcad-document`: parametric document history (sketches + features + recompute) and the `.zcad` document format.
  - `openrcad-render`: interactive `wgpu`/`winit` viewer (orbit/pan/zoom, MSAA, edge wireframe, click-select). **Not** in the `openrcad` facade — GPU deps stay out of the kernel.
  - `openrcad-truck-compat`: bidirectional `truck_topology` interop (separate crate, not a `topo` feature).
- `OpenCASCADE Source Code/src/`: Upstream C++ reference (headers `.hxx`, implementations `.cxx`).
- `truck Source Code/`: Reference Rust CAD kernel (`truck-topology`, `truck-geometry`, `truck-modeling`, etc.).

## Reference Guidelines
- When implementing any feature, cross-reference the C++ logic in `OpenCASCADE Source Code/src/` and the idiomatic Rust patterns in `truck Source Code/`.
- Use search/grep inside both folders (e.g., C++ `gp_Pnt` ↔ truck's points, `TopoDS_Shape` ↔ truck's topology).
- Translate to clean, safe Rust. Replace C++ pointer graphs with arena indices. Replace class hierarchies with enums + traits.

## Build and Test Commands
- `cargo build` — build workspace
- `cargo test` — run all tests
- `cargo build --release` — optimized release build
- `cargo clippy --workspace --all-targets` — lint

## Hard Rules
These constraints are non-negotiable across the entire workspace:

- **`#![forbid(unsafe_code)]`** — Every crate uses this. No exceptions. All data structures and algorithms must be implemented in safe Rust.
- **Geometry is owned enums, not `Box<dyn Trait>`** — `GeomCurve` and `GeomSurface` are enums (`Line | Circle | Ellipse | BSplineCurve | …`), stored **by value** inside topology. This means topology is `Clone`, `Serialize`, and lifetime-free — no `Box<dyn>`, no `&'a` references in stored data. Algorithms that need to be generic over curves/surfaces use `Curve`/`Surface` traits, which the enums implement by delegation.
- **Minimal dependencies** — Only `serde` (with `derive`) is allowed as an external dependency for core crates. Performance crates like `slotmap` and `rayon` are permitted in `openrcad-topo` and `openrcad-mesh` respectively. No C/C++ dependencies anywhere.

---

## Architectural Principles

These are the core design decisions that differentiate OpenRCAD from legacy kernels and `truck`. Every implementation must follow these principles.

### 1. Arena-Based B-Rep (Lock-Free, Cache-Friendly Topology)
B-Rep graphs are inherently cyclic (Face → Loop → Edge → Vertex → …). Rust's ownership model forbids pointer cycles without `Rc`/`Arc` + locks, which destroy performance and cache locality.

**OpenRCAD's approach:**
- Store all topological entities (Vertex, Edge, Loop, Face, Shell, Solid) in flat **generational arenas** (e.g., `slotmap`). Each entity is referenced by a lightweight, `Copy`-able index handle (`VertexId`, `EdgeId`, etc.).
- Topological adjacency is stored as arrays of these indices, not pointers.
- This gives O(1) access, contiguous memory layout for cache efficiency, and enables safe concurrent reads via `rayon` with zero mutex contention.

### 2. BVH Spatial Acceleration
Every geometric query (intersection, closest point, containment) must be accelerated by a **Bounding Volume Hierarchy (BVH)** built over B-Rep faces and edges.

**Implementation specifics:**
- Use axis-aligned bounding boxes (AABBs) as bounding volumes.
- Build with the **Surface Area Heuristic (SAH)** for optimal split quality.
- The BVH is lazily constructed and invalidated on topology changes.
- This reduces intersection and boolean complexity from O(n²) to O(n log n).

### 3. Robust Numerics: Exact Predicates + Adaptive Intervals
Floating-point errors are the #1 cause of boolean and fillet failures in legacy kernels. Tiny rounding differences create "splinter" faces or open gaps.

**OpenRCAD's approach:**
- Geometric predicates (point-on-plane, orientation tests) use **Shewchuk-style adaptive exact arithmetic** — fast floating-point with automatic promotion to exact computation only when the result is ambiguous.
- Intersection solvers use **adaptive subdivision with interval bounding** to guarantee that intersection curves are topologically correct before committing them to the B-Rep.

### 4. Tolerant Modeling (Per-Entity Precision)
Imported CAD data frequently has small gaps between surfaces. A single global tolerance is too rigid.

**OpenRCAD's approach:**
- Each `Vertex` and `Edge` stores its own local tolerance value (the radius of its uncertainty sphere/tube).
- All adjacency checks and sewing operations respect per-entity tolerances rather than a single global constant.
- This allows the kernel to robustly process "dirty" imported geometry from STEP, IGES, and other kernels.

### 5. NURBS: de Boor Evaluation + Boehm Knot Insertion
NURBS curves and surfaces are the mathematical backbone of the kernel. Use the proven, numerically stable algorithms:

- **Evaluation:** de Boor's algorithm (local support, O(k²) per point where k = degree). Never use naive basis function summation.
- **Knot insertion:** Boehm's algorithm for single-knot insertion (corner-cutting). Oslo algorithm for bulk multi-knot refinement.
- **Derivative computation:** Derive from de Boor directly (no finite differences).

### 6. Local Euler Operators for Topology Edits
Avoid global boolean sweeps for local modifications (fillets, chamfers, face deletions).

**OpenRCAD's approach:**
- Implement atomic **Euler operators** (`MEV`, `MEF`, `KEV`, `KEF`, `MEKR`, `KEMR`) that surgically modify the B-Rep graph in the local neighborhood of an edit.
- Each operator preserves the Euler-Poincaré invariant: $V - E + F = 2(S - G) + H$.
- Operators are composable into higher-level transactions (e.g., "insert fillet face along edge").

### 7. Fusion 360-Grade Filleting & Chamfering
The blending system must handle the cases that crash `truck` and produce poor results in OpenCASCADE:

- **Edge blends:** Rolling ball sweep along the edge guide curve. Support constant and variable radius.
- **Corner blends (n-valent vertices):** When 3+ blended edges meet at a vertex, generate an **N-sided Gregory patch** (not degenerate quad NURBS). This guarantees G1/G2 continuity without mathematical singularities.
- **Face overflow:** If the fillet radius exceeds an adjacent face, the solver must automatically trace across the face boundary onto the next face, trim it, and continue the blend.
- **Offset surfaces / shelling:** Offset each face along its normal, detect and resolve self-intersections in concave regions, and stitch the result into a valid shell.

### 8. Topology Sewing & Model Healing
Robust import/export requires a sewing engine that repairs fragmented geometry:

- Identify free (unshared) edges on open shells.
- Match free edges within tolerance and merge them, stitching adjacent faces together.
- Validate the repaired shell for manifoldness, orientation consistency, and watertightness.

---

## Implementation Roadmap

A 5-phase plan. Each phase produces a testable, self-contained deliverable. Phases are sequenced by dependency — each builds directly on the outputs of the previous phase.

### Phase 1: Geometry & Parallel Topology Foundation
*Deliverable: Complete curve/surface math and a thread-safe B-Rep container.*

> **Note:** `openrcad-foundation` (points, vectors, directions, transforms, bounding boxes, tolerances) is already implemented and tested. Do not rewrite it — extend it where needed.

| Crate | Work |
|---|---|
| `openrcad-geom2d` | Extend existing line/circle/ellipse with remaining conics (parabola, hyperbola). Add 2D B-spline/NURBS curves via de Boor evaluation. |
| `openrcad-geom` | Extend existing line/circle/ellipse/plane with 3D B-spline/NURBS curves and surfaces. Implement Boehm knot insertion. Add surface evaluators (point, normal, derivatives). All new curve/surface types are added as enum variants to `GeomCurve`/`GeomSurface`. |
| `openrcad-topo` | Replace stub with generational arena storage (`slotmap`). Define `VertexId`/`EdgeId`/`FaceId` handles. Implement topology traversals. Geometry is stored by value (owned enums) inside each topological entity. |

**Tests:** NURBS evaluation matches analytical results. Concurrent multi-threaded topology traversals complete without data races.

### Phase 2: Intersection Engine & BVH
*Deliverable: A robust intersection solver accelerated by spatial indexing.*

| Crate | Work |
|---|---|
| `openrcad-foundation` | Add interval arithmetic types and Shewchuk exact predicates. |
| `openrcad-algo` | Implement curve-curve, curve-surface, and surface-surface intersection via adaptive subdivision. Build a BVH over B-Rep faces using SAH. |

**Tests:** Intersections of tangent cylinders, coincident planes, and complex NURBS surfaces all produce watertight, gap-free intersection curves. BVH accelerates brute-force by >10×.

### Phase 3: Booleans, Euler Operators & Primitives
*Deliverable: Working boolean operations and primitive shape builders.*

| Crate | Work |
|---|---|
| `openrcad-algo` | Implement Euler operators (`MEV`, `MEF`, `KEV`, `KEF`). Implement boolean operations (union, subtract, intersect) using the intersection engine + face classification. |
| `openrcad-primitives` | Build Box, Cylinder, Cone, Sphere, Wedge using Euler operators on top of the geometry layer. |

**Current Status & Implementation Details:**
- **Step 1: Mutable Topological Staging (`openrcad-topo`)** - Completed `BRepBuilder` in [builder.rs](file:///c:/Users/cadel/Documents/Coding_Projects/OpenRCAD/crates/openrcad-topo/src/builder.rs) supporting edge-splitting and face-partitioning via 2D Jordan curve point-in-polygon containment.
- **Step 2: Intersection & BVH** - Completed `overlapping_pairs` BVH dual-tree traversal in [bvh.rs](file:///c:/Users/cadel/Documents/Coding_Projects/OpenRCAD/crates/openrcad-algo/src/bvh.rs) and `ray_face` intersections on trimmed faces in [intersect.rs](file:///c:/Users/cadel/Documents/Coding_Projects/OpenRCAD/crates/openrcad-algo/src/intersect.rs).
- **Step 3: Gap Healing & Sewing (`openrcad-algo`)** - Implemented tolerant `sew` in [sew.rs](file:///c:/Users/cadel/Documents/Coding_Projects/OpenRCAD/crates/openrcad-algo/src/sew.rs) to merge vertices within a custom tolerance and clean up redundant topological entities.
- **Step 4: Boolean Engine (`openrcad-algo`)** - Implemented the full B-Rep Boolean engine in [boolean.rs](file:///c:/Users/cadel/Documents/Coding_Projects/OpenRCAD/crates/openrcad-algo/src/boolean.rs) supporting Union, Intersection, and Difference:
  - Intersection/coplanar passes to split edges and faces.
  - Robust Point-on-Surface (PoS) interior sampling.
  - Ray-cast classification with boundary collision retry/perturbation.
  - Watertight assembly utilizing `sew`.
- **Step 5: Verification** - Added integration tests for Cube Union, Intersection, and Difference in [boolean.rs](file:///c:/Users/cadel/Documents/Coding_Projects/OpenRCAD/crates/openrcad-algo/src/boolean.rs#L505). These pass quickly: the subdivision solvers gained rigorous interval pruning and depth caps, and `curve_curve` now takes a closed-form fast path for analytic pairs (line∩line, line∩circle, circle∩circle) instead of subdividing the clamped ±100 line domain — box∩box dropped from ~13.8 ms to ~1.3 ms (≈10×); box−cylinder runs in ~1.4 ms. The classification pass in `boolean()` is BVH-pruned (`box_overlap` per split face) so the coplanar pre-check is ≈O(n·log m) instead of O(n·m). Remaining perf headroom: parallelising the per-face classification (`rayon`, behind the `parallel` feature) and a triangle BVH for render picking. All four previously-partial-imprint cases (through side-drill, corner-overlap union, blind pocket, rotated partial cut) are now watertight as of 2026-06-19 — see Phase 5 status for details.
- **Step 6: Boolean robustness & result quality (2026-06-20)** — hardened the engine so `boolean_checked` accepts the everyday cases a CAD app generates (thin plates, off-axis bodies, coplanar joins), not just clean axis-aligned cubes:
  - **Loop re-threading ([sew.rs](file:///c:/Users/cadel/Documents/Coding_Projects/OpenRCAD/crates/openrcad-algo/src/sew.rs), step "6.5")** — the coplanar-split/imprint path could emit a closed loop with one co-edge's orientation flipped (`LoopNotContiguous` → `health_report` unhealthy → `boolean_checked` rejected the whole result, silently breaking joins). `rethread_loop` re-orders/re-orients each loop's co-edges into one contiguous chain; a no-op on already-contiguous loops, so winding is preserved.
  - **Coplanar + collinear merge ([merge.rs](file:///c:/Users/cadel/Documents/Coding_Projects/OpenRCAD/crates/openrcad-algo/src/merge.rs), called at the end of `boolean()`)** — `merge_coplanar_faces` cancels interior seam edges to merge coplanar adjacent faces (a 2-box union is now a clean 6-face box, not 14), then `merge_collinear_edges` collapses the leftover collinear sub-edges (back to 8 vertices / 12 edges). Outer-vs-hole is classified by containment nesting (a box plane's `Ax3` can be left-handed about its normal, so signed area is unreliable). The whole pass is wrapped in a hard safety net: it returns the original solid unless the merged result is watertight, healthy, and smaller — so it can only improve a result.
  - **Cylinder boss-union ([boolean.rs](file:///c:/Users/cadel/Documents/Coding_Projects/OpenRCAD/crates/openrcad-algo/src/boolean.rs))** — when a coplanar face's boundary is a full circle (a cylinder cap rim) inside the opposite face (`wire_full_circle` + `circle_inside_face`), the split imprints the *whole* circle once so `imprint::cut_hole` bores it as a hole (3 arcs at thirds, matching `make_cylinder`'s wall). A box + cylinder boss now fuses watertight; native (smooth) cylinder cuts and boss unions all pass `boolean_checked`.
  - Regression gates: [`tests/repro_screenshots.rs`](file:///c:/Users/cadel/Documents/Coding_Projects/OpenRCAD/crates/openrcad-algo/tests/repro_screenshots.rs), [`repro_fillet.rs`](file:///c:/Users/cadel/Documents/Coding_Projects/OpenRCAD/crates/openrcad-algo/tests/repro_fillet.rs), [`repro_cylinder.rs`](file:///c:/Users/cadel/Documents/Coding_Projects/OpenRCAD/crates/openrcad-algo/tests/repro_cylinder.rs).

**Tests:** Euler-Poincaré invariant holds after every operation. Boolean of a box minus a cylinder produces a valid 10-face solid. Primitives round-trip through serialize/deserialize.

### Phase 4: Tessellation, Sewing & Data Exchange
*Deliverable: Parallel mesh generation, model healing, and file I/O.*

| Crate | Work |
|---|---|
| `openrcad-mesh` | Adaptive face tessellation (chordal error + angular deflection). Parallelize across faces via `rayon`. |
| `openrcad-algo` | Implement topology sewing (free-edge matching within tolerance) and shell healing (orientation, manifoldness). Add tolerant modeling (per-entity tolerance storage). |
| `openrcad-exchange` | STEP AP242 reader/writer (incl. toroidal surfaces). Binary and ASCII STL writers (STL is write-only). |

**Tests:** Import STEP reference models, sew/heal, tessellate in parallel, export to watertight STL. Round-trip STEP → OpenRCAD → STEP preserves topology.

**Current Status & Implementation Details:**
- **Validation (`openrcad-topo`)** — [validate.rs](file:///c:/Users/cadel/Documents/Coding_Projects/OpenRCAD/crates/openrcad-topo/src/validate.rs) adds `Solid::manifold_report` / `is_watertight` / `is_manifold`: it matches undirected boundary edges by quantized endpoint position (independent arena edges at one location merge) and counts face usage — a closed two-manifold solid shares every edge exactly twice. `health_report` warns on non-manifold edges. Boolean integration tests now assert `is_watertight()` on their output (box∩box, box∪box, box−box, and the cylinder-drilled box all pass).
- **Sewing (`openrcad-algo`)** — [sew.rs](file:///c:/Users/cadel/Documents/Coding_Projects/OpenRCAD/crates/openrcad-algo/src/sew.rs) merges coincident vertices/edges within per-entity tolerance and re-threads each loop into a contiguous, consistently-oriented co-edge chain (see Phase 3 Step 6), so sewn results pass `validate()` and `is_healthy()` rather than just `is_watertight()`.
- **Viewer (`openrcad-render`)** — interactive: orbit/pan/zoom, click-to-select with a shader face highlight, **4× MSAA**, and a **topological-edge wireframe overlay** (edges between triangles of different source `face_id`s + open boundaries, extracted in [edges.rs](file:///c:/Users/cadel/Documents/Coding_Projects/OpenRCAD/crates/openrcad-render/src/edges.rs), drawn with a depth-biased line pipeline). Still no materials/multi-object scene or photoreal path.

### Phase 5: Advanced Blending & Offset
*Deliverable: Production-quality fillets, chamfers, and shelling.*

| Crate | Work |
|---|---|
| `openrcad-algo` | Rolling ball fillet (constant + variable radius). N-sided Gregory patch corner blends. Chamfer (distance + angle modes). Offset surfaces and shell generation with self-intersection resolution. |

**Tests:** Fillet a box with 3 different radii meeting at a corner — verify G2 continuity. Shell a complex solid — verify watertight result. Fillet radius larger than adjacent face — verify successful face overflow.

**Current Status & Implementation Details (as of 2026-06-19):**
- **Geometry (`openrcad-geom`)** — `RuledSurface` ([ruled.rs](file:///c:/Users/cadel/Documents/Coding_Projects/OpenRCAD/crates/openrcad-geom/src/ruled.rs)), `GregorySurface` (4-sided, G1 boundary, [gregory.rs](file:///c:/Users/cadel/Documents/Coding_Projects/OpenRCAD/crates/openrcad-geom/src/gregory.rs)), `OffsetSurface` ([offset.rs](file:///c:/Users/cadel/Documents/Coding_Projects/OpenRCAD/crates/openrcad-geom/src/offset.rs)), and `ToroidalSurface` ([torus.rs](file:///c:/Users/cadel/Documents/Coding_Projects/OpenRCAD/crates/openrcad-geom/src/torus.rs)) are `GeomSurface` enum variants with `point`/`d1`/`bounds`/`transformed` delegation, each with a point+derivative unit test. The torus is also wired through `interval_bounds`, the BVH, intersection `uv_of`, mesh tessellation, and STEP read/write (`TOROIDAL_SURFACE`).
- **Blend API** — `fillet`, `chamfer`, and `shell_solid` return `Result<Solid, BlendError>` (`BlendError::{UnsupportedShape, ParameterTooLarge}`). Shared detection + cylinder builders live in [blend.rs](file:///c:/Users/cadel/Documents/Coding_Projects/OpenRCAD/crates/openrcad-algo/src/blend.rs); the box paths stay in fillet/chamfer/offset.
- **Filleting** — box (via `detect_box`): 6 trimmed planes + 12 cylindrical faces + 8 spherical octant corners = 26 faces. Cylinder (via `detect_cylinder`): 2 caps + 3 wall + 6 **torus** rolling-ball faces, 11 faces, χ=2.
- **Chamfering** — box: 14 planes + 12 ruled faces. Cylinder: 2 caps + 3 wall + 6 conical (45° frustum) faces.
- **Shelling** — `shell_solid` box + cylinder: inner cavity via `OffsetSurface`, planar rim faces around removed `open_faces`.
- **General rolling-ball sweep (`openrcad-algo/rolling_ball.rs`)** — `rolling_ball_fillet_edge`, `fillet_planar_edge`, `rolling_ball_between_planar_faces`, and `fillet_edges` are now public via the `openrcad` prelude. Handles planar–planar, planar–cylindrical, and planar–analytic edge adjacency with a Gauss-Newton Newton solver; emits `RollingBallError` variants (`InvalidRadius`, `DegenerateSpine`, `EdgeAdjacency`, `UnsolvableAdjacency`, `InvalidDihedral`, `SpineNotOnFace`, `UnsupportedTrimTopology`, `NewtonDiverged`, `BlendSurfaceBuild`) for unsupported or degenerate inputs. **`fillet_edges` works on boolean results** — the boolean's collinear-edge merge (Phase 3 Step 6) restores full-span edges, so endpoint-based edge selection finds them instead of failing with `SpineNotOnFace`. `fillet_planar_edge` now also rejects a blend that didn't close (radius larger than half the local thickness → degenerate edges) with `InvalidRadius` instead of returning a broken solid. **`planar_blend` refactor (2026-06-21):** `rolling_ball_fillet_edge` and `rolling_ball_between_planar_faces` share a factored-out `planar_blend(edge, face_a, face_b, n_a, n_b, radius)` core that takes already-outward normals explicitly. This relies on `sew` canonicalizing every sewn shell so each planar face's stored orientation/winding agrees with an outward normal — so the fillet reads `planar_outward_normal` directly (no solid-interior re-derivation) and a sewn prism cap with a once-inward stored orientation no longer mis-seeds the bisector side (pairs with the prism cap-orientation fix below).
- **Selected-edge blend façade (`openrcad-algo/contour.rs`)** — `apply_blend_contour(&solid, &BlendContour)` is a deliberately thin public façade over the per-edge solvers: it routes a `BlendContour { edges, kind: BlendKind::{Fillet|Chamfer}, law: BlendLaw::Constant(_), hint: Option<BlendCurveHint::{Line|Circle}> }` to `fillet_edges`, `fillet_circular_edge_chain` (for a co-circular contour), or `chamfer_edges`, and returns `Result<Solid, BlendContourError>` (`EmptyContour`, `VariableLawUnsupported`, `Fillet(..)`, `Chamfer(..)`). `BlendLaw::Variable` is a reserved placeholder. These types plus `RollingBallError`/`ChamferError` are re-exported from `lib.rs`, so a consumer that calls the per-edge API imports them directly.
- **Prism/extrusion sweeping (`openrcad-algo/prism.rs`)** — `prism(face, vector)` and `sweep_prism` are public. Straight edges → planar laterals; circular arcs axially aligned → cylindrical laterals; all other curves → `RuledSurface` laterals. Reports `SweepError::{DegenerateVector, MissingOuterWire, OpenWire}`. **Cap orientation fix (2026-06-21):** the cap built from `normal.reversed()` now has its loop winding reversed (`reversed_wire`) so winding stays consistent with the cap's outward plane normal. Without this, `sew`'s winding-based orientation propagation resolved the winding/normal conflict by flipping the cap's *orientation flag*, leaving an inward *effective* normal (`orientation × plane normal`) — invisible to the watertight/health checks but fatal to back-face culling (the renderer's *"extruded box's top disappears"* artifact) and to the rolling-ball fillet (wrong bisector side). Regression test: `prism_caps_are_oriented_outward`.
- **Partial-imprint boolean solver (as of 2026-06-19):** The boolean split pass now correctly handles the four previously-failing partial-imprint cases. Key changes:
  - **`imprint.rs`** — new shared primitive extracted from the boolean engine; `imprint_curve_on_face` handles clean cross-cuts and closed hole-drills; non-trivial topologies are left unchanged and tolerated by the pipeline.
  - **`intersect.rs`** — `surface_surface_curves(face1, face2, tol)` replaces the raw `surface_surface` call in the split pass; it trims intersection curves to the actual face boundaries by intersecting with all boundary edges, then keeps only segments whose midpoint lies inside both faces. `trim_curve_to_face` likewise trims edge-curves before imprinting them onto the opposing face.
  - **`boolean.rs`** — the split pass now collects `splitting_edges` per face and defers to `BRepBuilder::partition_face` (see below) rather than calling `split_tracked` immediately; this decouples edge insertion from face partitioning.
  - **`openrcad-topo/builder.rs`** — `BRepBuilder::partition_face(face_id, &[EdgeId])` partitions a face along an arbitrary edge network, distributes inner loops (holes) to the correct new sub-face, and uses a Gauss-Newton `search_nearest_parameter` to project 3D points into parameter space for containment decisions. The old floating-point Jordan-curve test is replaced throughout by the canonical `containment::point_in_polygon_2d`.
  - **`openrcad-topo/containment.rs`** — canonical home for `point_in_polygon_2d` (robust exact-predicate crossing-number test). Replaces three previous copies in `builder.rs`, `intersect.rs`, and `triangulate.rs`; all three now import from here.
  - **`robustness.rs`** — the four previously-`#[ignore]`d partial-imprint goal-tests (`through_side_drill_should_be_closed`, `corner_overlap_union_should_be_watertight`, `blind_pocket_cut_should_be_watertight`, `rotated_tool_partial_cut_should_be_watertight`) now pass without `#[ignore]`. `checked_boolean_rejects_known_side_drill_failure` is renamed `checked_boolean_accepts_side_drill` and flipped to assert success.
- **Known gaps / next work:** the *whole-solid* `fillet`/`chamfer`/`shell_solid` still recognise only a single box or cylinder (`UnsupportedShape` for arbitrary B-Reps) — but the *per-edge* `fillet_edges` rolling-ball blend works on arbitrary planar/analytic edges, including boolean results. N-valent Gregory corner blends, fillet face overflow, and concave-offset self-intersection resolution are not yet implemented. A boolean cut that *severs* a body is returned by `boolean` as one solid (Euler=4); use `boolean_bodies` / `boolean_checked_bodies` (or `Solid::split_disconnected`) to get one solid per connected component.
- **Build/lint:** `cargo build`/`cargo test --workspace` pass with **zero compiler warnings**, and `cargo clippy --workspace --all-targets` is **clean (0 warnings)**. The pervasive `needless_range_loop` lint in the NURBS/BVH/sew numeric kernels is suppressed by a justified crate-level `#![allow(clippy::needless_range_loop)]`; intentional many-arg builders and the large `GeomSurface` variant carry targeted `#[allow]`s.

---

## Future Directions (Post-v1)
These are valuable but not required for a functional kernel. Implement after the 5 core phases are stable:

- **Convergent Modeling:** Allow B-Rep faces backed by discrete polygon meshes, enabling hybrid NURBS/mesh booleans.
- **Direct Modeling:** Push-pull face editing, feature recognition (detect holes, pockets, fillets), and defeaturing.
- **Sweep with RMF:** Rotation Minimizing Frames (Double Reflection Method) for twist-free profile sweeps along complex 3D paths.
- **WebAssembly optimization:** Zero-copy serialization (bincode/flatbuffers) for browser-side evaluation and rendering.
