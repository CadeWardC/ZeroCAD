# Integrating OpenRCAD into your project

This guide shows how to add OpenRCAD to a Rust project and drive every layer of
the kernel — from building primitives to booleans, blends, meshing, and file
exchange. It is written to be copy-paste practical; for the conceptual *why*, see
[`architecture.html`](architecture.html) and [`CLAUDE.md`](CLAUDE.md).

> **Status note.** OpenRCAD is pre-alpha. The foundation, geometry, topology,
> primitives, intersection engine + BVH, booleans, tessellation, sewing, and
> STEP/STL layers are real and tested, plus an interactive viewer
> (`openrcad-render`) and a parametric document layer (`openrcad-document`). The
> whole-solid fillet/chamfer/shell builders handle a single **box** or
> **cylinder** primitive at **any orientation** and return a typed error for
> anything else; the per-edge rolling-ball fillet (`fillet_edges`) works on
> arbitrary edges including boolean results. Booleans are watertight **and**
> health-validated across the everyday cases (partial-imprint, coplanar joins,
> cylinder cuts and bosses). See [Limitations](#limitations).

---

## 1. Add the dependency

Depend on the **facade** crate, which re-exports every layer as a submodule:

```toml
# Cargo.toml
[dependencies]
openrcad = { version = "0.1" }
# or, from a checkout:
# openrcad = { path = "../OpenRCAD" }
```

```rust
use openrcad::foundation::{Pnt, Dir, Vec as GeomVec, Ax1, Ax2, Ax3, Trsf};
use openrcad::primitives::{make_box, make_cylinder, make_cone, make_sphere};
use openrcad::algo::{boolean, fillet, chamfer, shell_solid, BooleanOp, BlendError};
use openrcad::mesh::tessellate;
use openrcad::exchange::{write_step, read_step, write_stl_ascii, write_stl_binary};
```

Want a thinner build? Depend on individual crates (`openrcad-foundation`,
`openrcad-topo`, …) directly — the layering guarantees you pull in only what you
use. The submodule paths above map 1:1 to crate names (`openrcad::geom` ↔
`openrcad-geom`).

**No system dependencies.** Pure Rust, `#![forbid(unsafe_code)]`, compiles
natively and to `wasm32`.

---

## 2. The core types

| Type | Module | What it is |
|---|---|---|
| `Pnt`, `Vec`, `Dir` | `foundation` | Point, free vector, unit direction (the `gp` layer). |
| `Ax1`, `Ax2`, `Ax3` | `foundation` | Axis, coordinate frame with one ref dir, full frame. |
| `Trsf` | `foundation` | Rigid/affine transform (`transform_point`, composition). |
| `GeomCurve`, `GeomSurface` | `geom` | Owned curve/surface enums stored *by value* in topology. |
| `Solid`, `Shell`, `Face`, `Edge`, `Vertex` | `topo` | The B-Rep entities. |
| `BooleanOp` | `algo` | `Fuse` \| `Cut` \| `Common`. |
| `BlendError` | `algo` | Error from `fillet`/`chamfer`/`shell_solid`. |
| `TriangleMesh` | `mesh` | Tessellation output. |

Every one of these is `Clone + Serialize + Deserialize` (see [§8](#8-persist-a-model-with-serde)).

---

## 3. Build primitives

```rust
use openrcad::foundation::{Pnt, Dir, Ax2};
use openrcad::primitives::{make_box, make_cylinder, make_cone, make_sphere, make_wedge};

// Box: base corner + three side lengths.
let b = make_box(&Pnt::new(0.0, 0.0, 0.0), 10.0, 20.0, 30.0);

// Cylinder: a frame (base point + axis direction), radius, height.
let c = make_cylinder(&Ax2::new(Pnt::origin(), Dir::dz()), 5.0, 12.0);

// Cone: bottom radius, top radius, height. (r2 = 0.0 gives a point.)
let k = make_cone(&Ax2::new(Pnt::origin(), Dir::dz()), 5.0, 2.0, 8.0);

// Sphere: centre + radius.
let s = make_sphere(&Pnt::origin(), 4.0);

assert_eq!(b.vertex_count(), 8);
assert_eq!(b.shell().faces().len(), 6);
```

Traverse the topology with the entity API: `solid.shell().faces()`,
`face.surface()`, `solid.vertex_count() / edge_count() / face_count()`.

---

## 4. Transform

`Trsf` follows OCCT's `gp_Trsf` convention exactly:

```rust
use openrcad::foundation::{Ax1, Dir, Pnt, Trsf};

let rot = Trsf::rotation(&Ax1::new(Pnt::origin(), Dir::dz()), std::f64::consts::FRAC_PI_2);
let moved = solid.transformed(&rot);
let (min, max) = moved.bounding_box().corners().unwrap();
```

Transforms compose; `transform_point(P) = scale·(matrix·P) + loc`.

---

## 5. Boolean operations

```rust
use openrcad::algo::{boolean, BooleanOp};

let union     = boolean(&a, &b, BooleanOp::Fuse);   // A ∪ B
let common    = boolean(&a, &b, BooleanOp::Common); // A ∩ B
let difference = boolean(&a, &b, BooleanOp::Cut);   // A − B
```

Booleans run over a BVH-accelerated intersection engine (with closed-form
line/circle intersection fast paths) and tolerant, ray-cast face classification,
finished by a watertight sewing pass. The result is a `Solid` you can feed
straight back into more operations.

**Validate what you get back.** Every modeling result should be a closed,
two-manifold solid — check it directly:

```rust
let cut = boolean(&a, &b, BooleanOp::Cut);
assert!(cut.is_watertight());          // every edge shared by exactly two faces
assert!(cut.validate().is_ok());       // every boundary loop is closed + contiguous
let m = cut.manifold_report();         // { total_edges, free_edges, nonmanifold_edges }
```

`is_watertight` (edge pairing) and `validate` (loop contiguity) are
complementary; together they are the cheapest catch for a malformed solid.
Booleans are watertight **and** healthy on through-cuts, face-flush and
corner-overlap unions, blind pockets, enclosed voids, partial and rotated cuts,
and cylinder cuts and bosses; coplanar adjacent faces are merged to clean
topology (see [Limitations](#limitations)).

---

## 6. Fillet, chamfer, and shell

These return `Result<Solid, BlendError>`. Handle both error cases:

```rust
use openrcad::algo::{fillet, chamfer, shell_solid, BlendError};

let cyl = make_cylinder(&Ax2::new(Pnt::origin(), Dir::dz()), 5.0, 12.0);

// Roll a 0.8 fillet along both circular rims (torus rolling-ball surfaces).
match fillet(&cyl, 0.8) {
    Ok(rounded)                                  => use_it(rounded),
    Err(BlendError::UnsupportedShape)            => { /* not a box/cylinder primitive */ }
    Err(BlendError::ParameterTooLarge { max, .. }) => { /* radius too big; `max` fits */ }
}

// Chamfer: a 45° conical frustum on each cylinder rim.
let beveled = chamfer(&cyl, 0.8)?;

// Shell: hollow to a wall thickness, leaving chosen faces open.
let top = cyl.shell().faces()
    .into_iter()
    .find(|f| matches!(f.surface(),
        Some(openrcad::geom::GeomSurface::Plane(p)) if p.normal().z() > 0.9))
    .unwrap();
let cup = shell_solid(&cyl, 0.3, &[top])?;   // open-topped tube
```

`?` works anywhere your function returns `Result<_, BlendError>` (it implements
`std::error::Error` + `Display`).

**What you get back is watertight.** Each builder sews its faces into one shell;
the result satisfies the Euler characteristic `V − E + F = 2`.

| Operation | On a box | On a cylinder |
|---|---|---|
| `fillet` | cylindrical edge faces + spherical octant corners | torus faces on both rims |
| `chamfer` | ruled bevel faces + triangular corners | 45° conical frustums |
| `shell_solid` | offset-surface cavity + planar rims | offset-surface cavity + planar rims |

---

## 7. Tessellate for rendering

```rust
use openrcad::mesh::tessellate;

// chordal error (max sag), angular error (radians).
let mesh = tessellate(&part, 0.05, 0.5);

println!("{} verts, {} tris", mesh.vertex_count(), mesh.triangle_count());

// Flat f64 buffer (x,y,z,x,y,z,…) ready for a GPU vertex buffer.
let positions: Vec<f64> = mesh.flat_positions();
```

Tessellation is adaptive (smaller chordal error → denser mesh) and parallelized
across faces with `rayon`. For a GPU, `mesh.gpu_mesh()` returns flat-shaded
`f32` position/normal buffers plus a per-triangle source-`face_id` buffer for
picking.

### Show it in a window (`openrcad-render`)

The viewer is a **separate crate** (not re-exported by the `openrcad` facade), so
its `wgpu`/`winit` dependencies never reach a core kernel crate. Add it directly:

```toml
openrcad-render = { version = "0.1" }
```

```rust
// Opens an interactive window: left-drag orbit, middle/right-drag pan, scroll
// zoom, click a face to select it. 4× MSAA + a topological-edge wireframe.
openrcad_render::run_solid(&part, 0.02); // (solid, chord error) — blocks until closed
```

---

## 8. Persist a model with serde

Because geometry is owned (no `Box<dyn>`, no lifetimes), any entity serializes:

```rust
let json = serde_json::to_string(&solid)?;
let back: openrcad::topo::Solid = serde_json::from_str(&json)?;
assert_eq!(back.face_count(), solid.face_count());
```

This is the basis for storing an entire model as a single document blob (use
`bincode` for a compact binary form).

---

## 9. Read and write files

```rust
use openrcad::exchange::{write_step, read_step, write_stl_ascii, write_stl_binary};
use std::fs::File;

// STEP (AP242 B-Rep) — round-trips solids, including toroidal surfaces.
write_step(&part, "part.step")?;
let reloaded = read_step("part.step")?;

// STL — write a tessellated mesh.
let mesh = tessellate(&part, 0.05, 0.5);
let mut f = File::create("part.stl")?;
write_stl_binary(&mesh, &mut f)?;
// or: write_stl_ascii(&mesh, "part", &mut f)?;
```

---

## 10. WebAssembly

Nothing in the kernel touches the filesystem except the `exchange` file helpers,
so for `wasm32` targets build with the geometry/topology/algo/mesh layers and do
STEP/STL I/O in memory (the STL writers take any `Write`; pass a `Vec<u8>`).
Owned, serializable data means you can evaluate a model in the browser and ship
the result back as a serde blob with zero pointer fix-ups.

---

## Limitations

Know these before you wire OpenRCAD into a production path:

- **Whole-solid blends are box/cylinder-only.** `fillet`, `chamfer`, and
  `shell_solid` detect a single box or cylinder primitive (at **any
  position/orientation** — the frame is recovered from geometry) and construct the
  result directly; any other *whole solid* yields `BlendError::UnsupportedShape`.
  The **per-edge** `fillet_edges` (rolling ball) does handle arbitrary
  planar/analytic edges, including boolean results — and rejects an over-large
  radius rather than emitting a degenerate solid. N-valent Gregory corner blends,
  face overflow, and concave-offset self-intersection resolution are not
  implemented yet.
- **Booleans: severed cuts stay one body.** Watertight and health-validated on
  through-cuts, face-flush and corner-overlap unions, blind pockets, enclosed
  voids, partial and rotated cuts, and cylinder cuts and bosses (the former
  partial-imprint goal-tests now pass and are un-`#[ignore]`d in
  `crates/openrcad-algo/tests/robustness.rs`). The remaining edge case: a cut that
  *severs* a body is returned as a single solid (Euler=4) rather than split into
  separate bodies.
- **Intersection subdivision** can be deep on tangential NURBS configurations;
  prefer analytic primitives (line/circle pairs take a closed-form fast path).
- **STL is write-only**; STEP is read + write.

When in doubt, call `solid.is_watertight()` and `solid.validate()` on results in
your own tests — together they are the cheapest catch for a malformed solid.

---

## Quick reference

```rust
// Primitives
make_box(&corner, dx, dy, dz) -> Solid
make_cylinder(&ax2, radius, height) -> Solid
make_cone(&ax2, r_bottom, r_top, height) -> Solid
make_sphere(&center, radius) -> Solid
make_wedge(dx, dy, dz, ltx) -> Solid

// Algorithms
boolean(&object, &tool, BooleanOp::{Fuse|Cut|Common}) -> Solid
fillet(&solid, radius) -> Result<Solid, BlendError>
chamfer(&solid, distance) -> Result<Solid, BlendError>
shell_solid(&solid, thickness, &open_faces) -> Result<Solid, BlendError>

// Mesh & exchange
tessellate(&solid, chord_err, angle_err) -> TriangleMesh
write_step(&solid, path) -> io::Result<()>
read_step(path) -> io::Result<Solid>
write_stl_ascii(&mesh, name, &mut writer) -> io::Result<()>
write_stl_binary(&mesh, &mut writer) -> io::Result<()>
```
