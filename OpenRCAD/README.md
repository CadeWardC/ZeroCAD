# OpenRCAD

**A pure-Rust CAD kernel — a refactor of [OpenCASCADE]'s architecture, informed by [truck].**

OpenRCAD is an early-stage, dependency-light geometric modeling kernel written
entirely in Rust. Its goal is to provide the foundation a CAD *application*
needs — points, vectors, curves, surfaces, boundary-representation (B-Rep)
topology, and primitive builders — without depending on any C++ kernel.

It is organized as a deliberate, Rust-idiomatic port of [OpenCASCADE
Technology (OCCT)][OpenCASCADE]'s module structure, with [truck][truck] used as
a reference for how that structure reads in modern Rust. The motivation is
direct: the author's other project, [ZeroCAD][ZeroCAD], is a parametric CAD app
built on `truck`, and years of working around `truck`'s boolean-solver panics,
missing fillet/chamfer support, and smooth-face failures make the case for a
kernel whose entire stack you own and can fix.

> **Status:** pre-alpha, but end-to-end across the main layers. Foundation,
> geometry, topology, primitives, the intersection engine + BVH, booleans,
> tessellation, sewing, STEP/STL/3MF exchange, solids of revolution and
> skin/loft sweeps, plane splitting and solid healing, an interactive wgpu
> viewer (also embeddable into host apps such as ZeroCAD), and a parametric
> document layer (`.zcad`) are implemented and tested. Booleans are
> watertight **and** health-validated across the everyday cases — thin plates,
> off-axis bodies, coplanar joins, through/blind cylinder cuts and bosses — with
> coplanar faces merged back to clean topology. Whole-solid Shell retains its
> box/cylinder fast paths and also handles verified plane/cylinder/cone/sphere
> networks plus regular torus fillet bands; per-edge rolling-ball fillets work
> on arbitrary edges — including boolean results and curved–curved face
> adjacency — with equal-radius corner networks solved exactly for valences
> 3–6. This
> README is the architectural map; for the browsable version see [`index.html`](index.html).
> For the current status and verification notes, see [`status.html`](status.html).

[OpenCASCADE]: https://dev.opencascade.org/
[truck]: https://github.com/ricosjp/truck
[ZeroCAD]: https://github.com/CadeWardC/ZeroCAD

---

## Why another kernel?

`truck` proved a pure-Rust CAD kernel is possible, and it is the right starting
point. But it is incomplete in ways that matter to an application:

- Its boolean solver **panics** on some configurations (a true cylinder meeting
  a box) and returns degenerate results on coplanar faces.
- It has **no fillet/chamfer builder**.
- Smooth (NURBS) faces break the boolean path, forcing applications to facet
  everything.

OpenCASCADE is the most mature open-source kernel in existence (decades of
industrial use, powering FreeCAD and many others). Its architecture — Foundation
classes, Modeling Data, Modeling Algorithms, Data Exchange, Visualization — is
battle-tested. OpenRCAD adopts that architecture, because the *shape* of OCCT is
the thing worth copying, and re-implements each layer in safe Rust.

OpenRCAD contains **no OCCT or truck source**. Both are design references only
(see the parent repo's [`THIRD_PARTY_NOTICES/`](../THIRD_PARTY_NOTICES/) directory).

---

## What's implemented

| OCCT module (toolkits) | OpenRCAD crate | Status |
|---|---|---|
| Foundation — `TKernel`, `TKMath`, `gp`, `Precision`, `Bnd` | [`openrcad-foundation`](crates/openrcad-foundation) | ✅ math, transforms, bnd boxes, interval arithmetic, exact predicates + tested |
| Modeling Data — 2D geometry (`TKG2d`) | [`openrcad-geom2d`](crates/openrcad-geom2d) | ✅ lines, conics, NURBS + tested |
| Modeling Data — 3D geometry (`TKG3d`, `TKGeomBase`) | [`openrcad-geom`](crates/openrcad-geom) | ✅ plane/cyl/cone/sphere/torus, helix, NURBS with exact rational second derivatives, Gregory/offset/ruled + tested |
| Modeling Data — topology / B-Rep (`TKBRep`) | [`openrcad-topo`](crates/openrcad-topo) | ✅ arena B-Rep, per-entity tolerance, validate/manifold/watertight checks + tested |
| Modeling Algorithms — primitives (`TKPrim`) | [`openrcad-primitives`](crates/openrcad-primitives) | ✅ box, cylinder, cone, sphere, wedge + tested |
| Modeling Algorithms (`TKBool`, `TKGeomAlgo`, `TKFillet`, …) | [`openrcad-algo`](crates/openrcad-algo) | ✅ intersection engine, SAH BVH, Euler ops, booleans (coplanar/collinear merge → clean topology), sew, `prism`/sweep + `revolve`, skin/loft skinning, plane split, heal, per-edge rolling-ball fillet/chamfer on arbitrary edges (incl. curved–curved adjacency and exact corner networks), an `apply_blend_contour` façade, and the `*_operation*` / `*_with_policy` / `*_with_cancel` contract layer returning `OperationResult` diagnostics; 🟡 verified analytic Shell matrix through regular torus fillet bands |
| Meshing / tessellation (`TKMesh`) | [`openrcad-mesh`](crates/openrcad-mesh) | ✅ adaptive parallel tessellation, GPU buffers + tested |
| Data Exchange (`TKSTEP`, `TKSTL`, …) | [`openrcad-exchange`](crates/openrcad-exchange) | ✅ STEP read/write, STL write, 3MF write (incl. assemblies) + tested |
| Visualization (`TKV3d`, `TKOpenGl`) | [`openrcad-render`](crates/openrcad-render) | 🟡 interactive wgpu viewer: orbit/pan/zoom, MSAA, edge wireframe, click-select; plus an embeddable `RenderCore` for host apps (not in facade) |
| 2D sketches / profiles | [`openrcad-sketch`](crates/openrcad-sketch) | ✅ rectangles, circles, lines → closed profiles; analytic planar `arrangement` (DCEL faces) and analytic `offset` + tested |
| Parametric document history (`.zcad`) | [`openrcad-document`](crates/openrcad-document) | ✅ sketches + features + recompute, serde document format + tested |
| truck interop | [`openrcad-truck-compat`](crates/openrcad-truck-compat) | 🟡 bidirectional `truck_topology` conversion |
| (facade) | [`openrcad`](src/lib.rs) | ✅ re-exports the kernel layers |

The proof that the architecture hangs together: `openrcad_primitives::make_box`
builds a solid from the foundation up — eight `Pnt`s become `Vertex`s, twelve
`Line` curves become `Edge`s, six `Planar` surfaces become `Face`s, and the six
faces become a closed `Shell` wrapped in a `Solid`. That end-to-end path is
asserted by crate-local tests and the workspace test suite, and the result is
verified **watertight** (`Solid::is_watertight`) and structurally valid
(`Solid::validate`).

---

## Workspace layout

```
OpenRCAD/
├── Cargo.toml          # `openrcad` facade package + workspace root
├── src/lib.rs          # facade: re-exports the kernel crates
├── crates/
│   ├── openrcad-foundation/    # gp, Precision, Bnd, Trsf, intervals, exact predicates
│   ├── openrcad-geom2d/        # 2D curves                (depends on foundation)
│   ├── openrcad-geom/          # 3D curves + surfaces     (depends on foundation)
│   ├── openrcad-topo/          # arena B-Rep + validation (depends on geom)
│   ├── openrcad-sketch/        # 2D sketches / profiles   (depends on foundation)
│   ├── openrcad-primitives/    # box, cyl, cone, sphere…  (depends on topo)
│   ├── openrcad-algo/          # intersection, BVH, booleans, blends, sew
│   ├── openrcad-document/      # parametric history (.zcad)  (depends on algo+sketch)
│   ├── openrcad-mesh/          # adaptive parallel tessellation
│   ├── openrcad-exchange/      # STEP + STL I/O
│   ├── openrcad-render/        # interactive wgpu viewer  (NOT in the facade)
│   └── openrcad-truck-compat/  # truck_topology interop   (separate crate)
├── index.html          # browsable docs (open in a browser)
├── architecture.html, crates.html, getting-started.html, status.html
├── tests/              # cross-cutting facade gate (strict_representation_gate.rs)
├── INTEGRATION.md      # host-application integration guide (ZeroCAD)
├── KERNEL_HARDENING_PLAN.md / CLAUDE.md   # hardening checklist / agent notes
└── docs.css            # shared stylesheet for the docs pages
```

The dependency DAG is strictly layered and points one way (the kernel core; the
viewer and truck-compat sit outside the facade so GPU/interop deps never reach a
core crate). Internal dependencies, per each crate's `Cargo.toml`:

```
openrcad-foundation    (no internal deps — the `gp` layer)
openrcad-geom2d        → foundation
openrcad-geom          → foundation
openrcad-topo          → foundation, geom, geom2d
openrcad-sketch        → foundation, geom2d
openrcad-primitives    → foundation, geom, geom2d, topo
openrcad-mesh          → foundation, geom, geom2d, topo, primitives
openrcad-algo          → foundation, geom, geom2d, topo, primitives, mesh
openrcad-exchange      → foundation, geom, geom2d, topo, primitives, mesh
openrcad-document      → foundation, topo, primitives, sketch, algo
openrcad-render        → foundation, topo, primitives, mesh      (outside the facade)
openrcad-truck-compat  → foundation, geom, topo                  (outside the facade)
```

---

## Quick start

Requires a recent stable Rust toolchain ([rustup]).

```bash
cargo build --workspace      # build everything
cargo test --workspace       # run the full suite
```

This workspace ships no `.cargo/config.toml` of its own; note that when you
build from inside the parent ZeroCAD checkout, the parent repo's
`.cargo/config.toml` applies and caps concurrent jobs at two.

[rustup]: https://rustup.rs

### Use it as a library

```toml
[dependencies]
openrcad = { version = "0.1" }
```

```rust
use openrcad::foundation::{Pnt, Dir, Vec as GeomVec, Ax1, Trsf};
use openrcad::primitives::make_box;

// A 10×20×30 mm box sitting on the origin.
let solid = make_box(&Pnt::new(0.0, 0.0, 0.0), 10.0, 20.0, 30.0);

assert_eq!(solid.shell().faces().len(), 6);
assert_eq!(solid.vertex_count(), 8);

// Rotate it 90° about the Z axis and read the moved AABB.
let rot = Trsf::rotation(&Ax1::new(Pnt::origin(), Dir::dz()), std::f64::consts::FRAC_PI_2);
let moved = solid.transformed(&rot);
let (min, max) = moved.bounding_box().corners().unwrap();
assert!((min.x() - (-20.0)).abs() < 1e-9);
```

---

## Design decisions

A few choices look unusual until you see the bug they prevent.

### Arena-Based B-Rep (Lock-Free, Cache-Friendly Topology)
B-Rep graphs are inherently cyclic (Face → Wire → Edge → Vertex). Safe Rust cannot represent pointer cycles without locks, which degrades multithreaded performance.
OpenRCAD stores all topological entities in flat, generational arenas (`slotmap`) inside a central `BRep` container. Topological entity handles wrap a shared `Arc<BRep>` and their index ID, enabling lock-free concurrent traversals (via `rayon`) with zero lock contention.

### Geometry is owned `enum`s, not `Box<dyn Trait>`
`GeomCurve` and `GeomSurface` are enums (`Line`/`Circle`/`Ellipse`/`BSplineCurve`, …). This is deliberate: topology stores geometry *by value*, which means it is `Clone`, `Serialize`, and never holds a `Box<dyn>` or a lifetime. Algorithms that need to be generic over curves take the `Curve`/`Surface` traits, which the enums implement by delegating. Owned + serializable data is what lets a whole model round-trip to a single document blob (the foundation of ZeroCAD's `.zcad` format).

### `truck` Compatibility (`openrcad-truck-compat`)
To let projects built on the `truck` CAD kernel integrate with OpenRCAD, the standalone `openrcad-truck-compat` crate provides bidirectional translation contexts (`TruckToOpenRcadContext`, `OpenRcadToTruckContext`) that map topological structures while maintaining correct entity sharing (deduplicating coincident vertices and shared edges). It is a *separate* crate, not a feature of `openrcad-topo`, so the core topology crate stays dependency-minimal and only projects that actually bridge to `truck` pull `truck-topology` into their graph.

### The `gp_Trsf` model
`Trsf` stores a `scale`, a 3×3 `matrix`, a translation `loc`, and a `form` (identity / translation / rotation / mirror / scale / compound). Every primitive transform is built so that the unified apply rule holds:

```
transform_point(P)  =  scale * (matrix · P) + loc
transform_vector(v) =  scale * (matrix · v)        // vectors ignore translation
transform_dir(d)    =  normalize(matrix · d)       // directions ignore scale
```

and composition is `scale = s1·s2`, `matrix = M1·M2`, `loc = s1·(M1·l2) + l1`. This is OCCT's exact convention, ported faithfully so future algorithmic code behaves identically.

### Honest scope limits
The higher layers are real but not yet fully general.

- **Whole-solid blends** retain optimized box/cylinder builders at any position/orientation. General Shell additionally supports the published plane/cylinder/cone/sphere matrix and regular torus fillet bands bounded by planes/cylinders; unsupported cone/torus, torus/torus, sphere/torus, concave, or ambiguous networks reject with `BlendError`. The **per-edge** rolling-ball fillet (`fillet_edges`) works on arbitrary planar/analytic edges, including boolean results. Complete equal-radius convex planar corner selections from valence three through six are solved simultaneously with an exact spherical patch and verified across scale/rotation/far-origin sweeps. Exact contact-curve overflow is verified for convex straight all-planar prisms, including one or multiple successive planar faces; curved successors, non-prismatic sources, concave corners, and profile-consuming radii reject explicitly. Partial, higher-valence, mixed-radius, and non-planar corner networks also reject explicitly. General curved-support overflow rejects explicitly. Concave offsets are accepted when self-intersection evidence is verified — `certify_concave_shell_with_policy` returns a `ConcaveShellCertificate` with wall-thickness evidence — while branching and NURBS-backed concave cases reject explicitly.
- **Booleans** are watertight and **health-validated** (`is_healthy()` — contiguous loops, no degenerate edges, manifold) across through-cuts, face-flush and corner-overlap unions, blind pockets, enclosed voids, partial and rotated cuts, and through/blind cylinder cuts and bosses — all locked in by [`robustness.rs`](crates/openrcad-algo/tests/robustness.rs) and the `repro_*` suites (no `#[ignore]`d boolean goal-tests remain). Coplanar adjacent faces are merged back to clean topology (a 2-box union is a 6-face box). The remaining edge case: a cut that *severs* a body is returned as one solid (Euler=4) rather than split into separate bodies — `boolean_bodies` (or `Solid::split_disconnected`) recovers one `Solid` per connected component.

Where a path is not yet implemented the code returns an explicit error rather than silently producing garbage, and modeling results can be self-checked with `Solid::is_watertight()` / `Solid::validate()`.

---

## Documentation

- **Browsable:** open [`index.html`](index.html) in any browser (static, no build step) — overview, architecture, per-crate reference, and a getting started guide.
- **Rust API:** `cargo doc --workspace --open`.
- **Current status:** [`status.html`](status.html) records what is verified, what remains limited, and the roadmap state.
- **This file** is the architectural map and the source of truth for the non-obvious decisions.

---

## Roadmap (rough order)

1. **Phase 1: Geometry & Parallel Topology Foundation** — *Status: **Complete***. All conics (parabola, hyperbola); B-spline / NURBS curves and surfaces; `openrcad-topo` on generational slotmap arenas.
2. **Phase 2: Intersection Engine & BVH** — *Status: **Complete***. Curve-curve, curve-surface, and surface-surface intersection via adaptive interval subdivision, with closed-form fast paths for line/circle pairs; SAH BVH with dual-tree overlap traversal. (The analytic `curve_curve` path cut box∩box booleans ~10×, from ~13.8 ms to ~1.3 ms.)
3. **Phase 3: Booleans, Euler Operators & Primitives** — *Status: **Complete***. Euler operators (`MEV`, `MEF`, `KEV`, `KEF`); fuse/cut/common with BVH-pruned classification; box, cylinder, cone, sphere, wedge. Results verified watertight *and* healthy (loop re-threading in `sew`), with coplanar/collinear merge for clean topology and a coplanar circular-hole imprint that closes cylinder boss unions. Partial-imprint cases are resolved; the only open item is splitting a severed cut into separate bodies.
4. **Phase 4: Tessellation, Sewing & Data Exchange** — *Status: **Complete***. Parallel meshing (`tessellate`) with chord/angular tolerances, topology sewing (`sew`), STEP read/write and STL write. Plus an interactive wgpu viewer (`openrcad-render`).
5. **Phase 5: Advanced Blending & Offset** — *Status: **In Progress***. `RuledSurface`, `GregorySurface`, `OffsetSurface`, and `ToroidalSurface` landed in `openrcad-geom`. Fillet and chamfer retain their box/cylinder fast paths; Shell also covers verified mixed plane/cylinder/cone/sphere networks and regular torus fillet bands with complete pcurves/history. The per-edge rolling-ball blends (`fillet_edges` / `chamfer_edges`, plus the thin `apply_blend_contour` façade that treats a co-circular run of edges as one logical contour) handle arbitrary planar/analytic edges, including boolean results, and reject an over-large radius rather than emitting a degenerate solid. Equal-radius convex planar corner networks are verified for consecutive valences three through six. Exact contact curves continue across successive planar faces on convex straight all-planar prisms; general curved-support overflow and self-intersection resolution for concave offsets remain.
6. **Operation contracts & expanded modeling** — *Status: **implemented; hardening continues*** (see [`KERNEL_HARDENING_PLAN.md`](KERNEL_HARDENING_PLAN.md)). The classic wrappers (`boolean`, `boolean_checked`, `fillet`, `chamfer`, `shell_solid`, `prism`, `tessellate`, `read_step`, `sew`) are now `#[deprecated]` in favor of `*_operation` / `*_with_policy` / `*_with_cancel` entry points that return `OperationResult` with typed `Diagnostic`s instead of panicking or discarding metadata; explicit `TolerancePolicy`/`ToleranceContext` plumbing replaced ambient tolerances. This wave also landed solids of revolution (`revolve*`), skin/loft sweeps (`skin*`, `smooth_skin*`, incl. smooth analytic lofts), plane splitting, solid healing, the `SolidExt` fluent façade and `openrcad::prelude`, helix curves, sketch `arrangement`/`offset`, curved–curved rolling-ball support, tangent-edge-chain blends, and 3MF export including assembly models.

Beyond the kernel: `openrcad-sketch` (2D profiles) and `openrcad-document` (parametric history + the `.zcad` document format) provide a Fusion/FreeCAD-style modeling spine on top of the kernel.

---

## Contributing & license

Build/test/style conventions live in the parent repository's
[`CONTRIBUTING.md`](../CONTRIBUTING.md). CI (GitHub Actions, defined in the
parent ZeroCAD repo) builds and tests this workspace on Ubuntu, Windows, and
macOS on every push and PR, with `rustfmt` and `clippy` as gates.

OpenRCAD is licensed under either of [MIT](LICENSE-MIT) or
[Apache-2.0](LICENSE-APACHE) at your option. Unless you state otherwise, any
contribution you submit is dual-licensed under the same terms.
