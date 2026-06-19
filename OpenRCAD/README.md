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
> tessellation, sewing, STEP/STL exchange, an interactive wgpu viewer, and a
> parametric document layer (`.zcad`) are implemented and tested. Booleans are
> watertight on a wide range of configurations (with a documented "partial
> imprint" frontier); fillet/chamfer/shell handle a single box or cylinder at
> **any orientation** (the general sweep is still open). This README is the
> architectural map; for the browsable version see [`index.html`](index.html).
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
(see [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md)).

---

## What's implemented

| OCCT module (toolkits) | OpenRCAD crate | Status |
|---|---|---|
| Foundation — `TKernel`, `TKMath`, `gp`, `Precision`, `Bnd` | [`openrcad-foundation`](crates/openrcad-foundation) | ✅ math, transforms, bnd boxes, interval arithmetic, exact predicates + tested |
| Modeling Data — 2D geometry (`TKG2d`) | [`openrcad-geom2d`](crates/openrcad-geom2d) | ✅ lines, conics, NURBS + tested |
| Modeling Data — 3D geometry (`TKG3d`, `TKGeomBase`) | [`openrcad-geom`](crates/openrcad-geom) | ✅ plane/cyl/cone/sphere/torus, NURBS, Gregory/offset/ruled + tested |
| Modeling Data — topology / B-Rep (`TKBRep`) | [`openrcad-topo`](crates/openrcad-topo) | ✅ arena B-Rep, per-entity tolerance, validate/manifold/watertight checks + tested |
| Modeling Algorithms — primitives (`TKPrim`) | [`openrcad-primitives`](crates/openrcad-primitives) | ✅ box, cylinder, cone, sphere, wedge + tested |
| Modeling Algorithms (`TKBool`, `TKGeomAlgo`, `TKFillet`, …) | [`openrcad-algo`](crates/openrcad-algo) | ✅ intersection engine, SAH BVH, Euler ops, booleans, sew; 🟡 blends for box/cylinder (any orientation) + tested |
| Meshing / tessellation (`TKMesh`) | [`openrcad-mesh`](crates/openrcad-mesh) | ✅ adaptive parallel tessellation, GPU buffers + tested |
| Data Exchange (`TKSTEP`, `TKSTL`, …) | [`openrcad-exchange`](crates/openrcad-exchange) | ✅ STEP read/write, STL write + tested |
| Visualization (`TKV3d`, `TKOpenGl`) | [`openrcad-render`](crates/openrcad-render) | 🟡 interactive wgpu viewer: orbit/pan/zoom, MSAA, edge wireframe, click-select (not in facade) |
| 2D sketches / profiles | [`openrcad-sketch`](crates/openrcad-sketch) | ✅ rectangles, circles, lines → closed profiles + tested |
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
├── INTEGRATION.md
└── docs.css            # shared stylesheet for the docs pages
```

The dependency DAG is strictly layered and points one way (the kernel core; the
viewer and truck-compat sit outside the facade so GPU/interop deps never reach a
core crate):

```
                       openrcad  (facade, re-exports the kernel)
                           │
        ┌──────────────────┼───────────────┬──────────────┐
   openrcad-primitives  openrcad-exchange  mesh          algo
        │                   │               │             │
        └──────► openrcad-topo ◄────────────┴─────────────┘
                       │        ◄── openrcad-geom, openrcad-geom2d
                       │
                openrcad-foundation   (no internal deps — the `gp` layer)

   openrcad-document ──► openrcad-algo + openrcad-sketch   (parametric layer)
   openrcad-render   ──► openrcad-mesh + openrcad-topo      (viewer, + wgpu/winit)
```

---

## Quick start

Requires a recent stable Rust toolchain ([rustup]).

```bash
cargo build --workspace      # build everything
cargo test --workspace       # run the full suite
```

Or use the aliases in [`.cargo/config.toml`](.cargo/config.toml):

```bash
cargo t        # == cargo test --workspace
cargo bx       # == cargo build -p openrcad-primitives
cargo ck       # == cargo clippy --workspace --all-targets
```

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

- **Blends** (`fillet`/`chamfer`/`shell_solid`) detect a single **box or cylinder primitive at any position/orientation** (`detect_box()`/`detect_cylinder()` recover the local frame from geometry) and construct the result directly; any other solid — including a boolean result — returns `BlendError::UnsupportedShape`. The general rolling-ball sweep on arbitrary edges, N-valent Gregory corner blends, face overflow, and concave-offset self-intersection resolution are not yet implemented.
- **Booleans** are watertight and structurally valid on a wide range of inputs (through-cuts, face-flush unions, enclosed voids, corner-overlap intersections, and all rotated placements — locked in by [`crates/openrcad-algo/tests/robustness.rs`](crates/openrcad-algo/tests/robustness.rs)). They are **not** yet watertight on *partial-imprint* cases (corner-overlap unions, blind pockets, partial rotated cuts), where an intersection curve only partly crosses a face. Those are captured as `#[ignore]`d goal-tests.

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
3. **Phase 3: Booleans, Euler Operators & Primitives** — *Status: **Complete (for scope)***. Euler operators (`MEV`, `MEF`, `KEV`, `KEF`); fuse/cut/common with BVH-pruned classification; box, cylinder, cone, sphere, wedge. Results verified watertight; the partial-imprint frontier is the open robustness item.
4. **Phase 4: Tessellation, Sewing & Data Exchange** — *Status: **Complete***. Parallel meshing (`tessellate`) with chord/angular tolerances, topology sewing (`sew`), STEP read/write and STL write. Plus an interactive wgpu viewer (`openrcad-render`).
5. **Phase 5: Advanced Blending & Offset** — *Status: **In Progress***. `RuledSurface`, `GregorySurface`, `OffsetSurface`, and `ToroidalSurface` landed in `openrcad-geom`. Fillet, chamfer, and shelling cover a box or cylinder at **any orientation** (cylinder fillets use rolling-ball **tori**, chamfers use 45° conical frustums), each watertight via `sew` and returning `Result<Solid, BlendError>`. Still to do: the general rolling-ball sweep on arbitrary edges, N-valent Gregory corner blends, face overflow, and self-intersection resolution for concave offsets.

Beyond the kernel: `openrcad-sketch` (2D profiles) and `openrcad-document` (parametric history + the `.zcad` document format) provide a Fusion/FreeCAD-style modeling spine on top of the kernel.

---

## Contributing & license

See [`CONTRIBUTING.md`](CONTRIBUTING.md) for build/test/style conventions.
CI (GitHub Actions) builds and tests the workspace on Ubuntu and Windows on
every push and PR, with `rustfmt` and `clippy` as gates.

OpenRCAD is licensed under either of [MIT](LICENSE-MIT) or
[Apache-2.0](LICENSE-APACHE) at your option. Unless you state otherwise, any
contribution you submit is dual-licensed under the same terms.
