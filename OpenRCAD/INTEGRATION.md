# Integrating OpenRCAD into your project

This guide shows how to add OpenRCAD to a Rust project and drive every layer of
the kernel — from building primitives to booleans, blends, meshing, and file
exchange. It is written to be copy-paste practical; for the conceptual *why*, see
[`architecture.html`](architecture.html) and [`CLAUDE.md`](CLAUDE.md).

> **Status note.** OpenRCAD is pre-alpha. The foundation, geometry, topology,
> primitives, intersection engine + BVH, booleans, tessellation, sewing, and
> STEP/STL/3MF layers are real and tested, plus an interactive viewer
> (`openrcad-render`) and a parametric document layer (`openrcad-document`). The
> primitive fillet/chamfer fast paths handle a single **box** or
> **cylinder** primitive at **any orientation**. Shell additionally handles the
> verified planar/cylinder/cone/sphere/regular-torus analytic network and
> topology-preserving concave subset. Strict all-edge Fillet/Chamfer
> adapters preserve those fast paths and extend to arbitrary solids whenever
> every edge passes the selected-edge machinery; otherwise they return the
> complete typed blocker set atomically. The per-edge rolling-ball fillet
> (`fillet_edges`) works on arbitrary edges including boolean results and
> curved–curved face adjacency. Booleans are watertight **and**
> health-validated across the everyday cases (partial-imprint, coplanar joins,
> cylinder cuts and bosses). See [Limitations](#limitations).
>
> **API generation note.** The classic wrappers (`boolean`, `boolean_checked`,
> `fillet`, `chamfer`, `shell_solid`, `prism`, `tessellate`, `read_step`, `sew`)
> are `#[deprecated]`: `boolean` discards metadata **and panics on failure**.
> New code should use the `*_operation` / `*_with_policy` / `*_with_cancel`
> entry points shown below — they return `OperationResult<T>` (value + topology
> history + diagnostics + recovery/validation reports) and never panic.

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
use openrcad::foundation::{Pnt, Dir, Vec as GeomVec, Ax1, Ax2, Ax3, Trsf, TolerancePolicy};
use openrcad::primitives::{make_box, make_cylinder, make_cone, make_sphere};
use openrcad::algo::{boolean_operation, fillet_with_policy, chamfer_with_policy,
                     shell_solid_with_policy, BooleanOp, BlendError};
use openrcad::mesh::tessellate_checked;
use openrcad::exchange::{write_step, read_step_operation, write_stl_ascii, write_stl_binary};
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
| `OperationResult<T>` | `topo` | Value + topology history + diagnostics + recovery/validation reports — what every `*_operation` entry point returns (`.value` holds the `T`). |
| `TolerancePolicy` | `foundation` | Explicit tolerance set (start from `TolerancePolicy::STANDARD`); threaded through every `*_with_policy` call. |
| `BlendError` | `algo` | Error from `fillet_with_policy`/`chamfer_with_policy`/`shell_solid_with_policy`. |
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
use openrcad::algo::{boolean_operation, BooleanOp};

let union     = boolean_operation(&a, &b, BooleanOp::Fuse)?.value;   // A ∪ B
let common    = boolean_operation(&a, &b, BooleanOp::Common)?.value; // A ∩ B
let difference = boolean_operation(&a, &b, BooleanOp::Cut)?.value;   // A − B
```

Every modeling entry point follows one **operation contract**: the
`*_operation` forms return `Result<OperationResult<T>, TypedError>`, where
`OperationResult<T>` carries the `.value` plus a topology `history`, typed
`diagnostics`, a `recovery` report, and a `validation` report. The
`*_with_policy` variants take an explicit `&TolerancePolicy` (start from
`TolerancePolicy::STANDARD`), and the `*_with_cancel` variants additionally
accept a cancellation token for cooperative cancellation in interactive hosts.
The legacy wrappers (`boolean`, `boolean_checked`, …) are `#[deprecated]` —
`boolean` in particular **panics on failure**; don't use it in new code.

Booleans run over a BVH-accelerated intersection engine (with closed-form
line/circle intersection fast paths) and tolerant, ray-cast face classification,
finished by a watertight sewing pass. The result is a `Solid` you can feed
straight back into more operations.

**Validate what you get back.** Every modeling result should be a closed,
two-manifold solid — check it directly:

```rust
let cut = boolean_operation(&a, &b, BooleanOp::Cut)?.value;
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

These return `Result<Solid, BlendError>` and take an explicit tolerance
policy. Handle both error cases:

```rust
use openrcad::algo::{fillet_with_policy, chamfer_with_policy, shell_solid_with_policy, BlendError};
use openrcad::foundation::TolerancePolicy;

let cyl = make_cylinder(&Ax2::new(Pnt::origin(), Dir::dz()), 5.0, 12.0);
let policy = TolerancePolicy::STANDARD;

// Roll a 0.8 fillet along both circular rims (torus rolling-ball surfaces).
match fillet_with_policy(&cyl, 0.8, &policy) {
    Ok(rounded)                                    => use_it(rounded),
    Err(BlendError::UnsupportedShape)              => { /* not a box/cylinder primitive */ }
    Err(BlendError::ParameterTooLarge { max, .. }) => { /* radius too big; `max` fits */ }
}

// Chamfer: a 45° conical frustum on each cylinder rim.
let beveled = chamfer_with_policy(&cyl, 0.8, &policy)?;

// Shell: hollow to a wall thickness, leaving chosen faces open.
let top = cyl.shell().faces()
    .into_iter()
    .find(|f| matches!(f.surface(),
        Some(openrcad::geom::GeomSurface::Plane(p)) if p.normal().z() > 0.9))
    .unwrap();
let cup = shell_solid_with_policy(&cyl, 0.3, &[top], &policy)?;   // open-topped tube
```

`?` works anywhere your function returns `Result<_, BlendError>` (it implements
`std::error::Error` + `Display`).

**What you get back is watertight.** Each builder sews its faces into one shell;
the result satisfies the Euler characteristic `V − E + F = 2`.

| Operation | On a box | On a cylinder |
|---|---|---|
| `fillet_with_policy` | cylindrical edge faces + spherical octant corners | torus faces on both rims |
| `chamfer_with_policy` | ruled bevel faces + triangular corners | 45° conical frustums |
| `shell_solid_with_policy` | offset-surface cavity + planar rims | offset-surface cavity + planar rims |

General Shell candidates use checked material-outward normals, exact analytic
support intersections, construction-time pcurves, and shared-edge
discretization. Concave sources receive an additional immutable envelope
certificate through `certify_concave_shell_with_policy`: it measures every
retained inward cell (with the opening set supplied explicitly), broad-phases
non-adjacent cells, intersects surviving pairs exactly, and verifies
material-side classification. A true unresolved
self-intersection or local-clearance collapse returns
`BlendError::ConcaveShell(ConcaveShellError::...)` without modifying the input.
The verified concave release subset preserves one or more exact cells per
retained analytic source support. Branching non-adjacent intersections and
arbitrary NURBS Shell are explicit rejections, not approximate topology.

### Per-edge blends (any solid, including boolean results)

The primitive whole-solid builders above recognise only a box or cylinder. To
blend selected edges of an **arbitrary** solid — including a boolean result —
use the per-edge rolling-ball API. It handles planar–planar, planar–cylindrical,
planar–analytic, and curved–curved edge adjacency, and rejects an over-large
radius with a typed error rather than emitting a degenerate solid.

```rust
use openrcad::algo::{fillet_edges, chamfer_edges};

// Pick the edges you want (e.g. the four vertical edges of a box).
let edges: Vec<_> = solid.edges().into_iter().filter(is_vertical).collect();

let rounded  = fillet_edges(&solid, &edges, 2.0)?;   // -> Result<Solid, RollingBallError>
let beveled  = chamfer_edges(&solid, &edges, 2.0)?;  // -> Result<Solid, ChamferError>
```

For a **contour** of edges that should be treated as one logical blend (e.g. a run
of co-circular fragments left by a boolean), the thin `apply_blend_contour` façade
routes the request to the right specialized solver based on kind and spine hint:

```rust
use openrcad::algo::{apply_blend_contour, BlendContour, BlendKind, BlendCurveHint, BlendLaw};

let contour = BlendContour {
    edges,                                // ordered edges forming the contour
    kind: BlendKind::Fillet,              // or BlendKind::Chamfer
    law: BlendLaw::Constant(2.0),         // Variable laws are not implemented yet
    curve_hint: Some(BlendCurveHint::Circle), // co-circular → one logical arc contour
};
let blended = apply_blend_contour(&solid, &contour)?; // -> Result<Solid, BlendContourError>
```

`BlendContourError` distinguishes an empty contour, an unsupported variable law,
an incompatible corner construction, and an underlying fillet/chamfer failure.
For a **network** of contours that must blend together (with corner modes
`BlendCornerMode::{Auto, Miter, RollingBall, Setback}`), use
`apply_blend_network_with_policy`.

### Strict all-edge convenience

Use the strict adapters when skipping an unsupported edge would be incorrect:

```rust
use openrcad::algo::{
    fillet_all_edges_strict,
    chamfer_all_edges_strict,
    AllEdgeBlendError,
};

let rounded = fillet_all_edges_strict(&solid, 2.0)?;
let beveled = chamfer_all_edges_strict(&solid, 2.0)?;
```

Edges are ordered canonically. Box/cylinder inputs use their simultaneous
primitive solution; other solids use the grouped selected-edge engines. On
failure, `AllEdgeBlendError::BlockingEdges` carries the deterministic source
edges and typed reasons found by diagnostic replay. The input solid is immutable
and no partly blended candidate is returned.

### Sweep a profile into a solid (`prism_operation`, `revolve_operation`)

`prism_operation(&face, vector)` extrudes a planar face along a vector into a
solid (`prism` is the deprecated plain-`Solid` form; `sweep_prism` is the
lower-level form). Straight profile edges become planar laterals, axially-aligned
circular arcs become cylindrical laterals, and other curves become ruled
laterals. It reports `SweepError::{DegenerateVector, MissingOuterWire, OpenWire}`.

```rust
use openrcad::algo::prism_operation;
use openrcad::foundation::Vec as GeomVec;

let solid = prism_operation(&profile_face, GeomVec::new(0.0, 0.0, 10.0))?.value;
```

The companion `revolve_operation*` builds solids of revolution from a profile
face and an axis, and the `skin*` / `smooth_skin*` family lofts section loops
into solids (including an exact analytic smooth-loft mode).

---

## 7. Tessellate for rendering

```rust
use openrcad::mesh::tessellate_checked;

// chordal error (max sag), angular error (radians).
// Checked tessellation validates the result instead of trusting it.
let mesh = tessellate_checked(&part, 0.05, 0.5)?;

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
use openrcad::exchange::{write_step, read_step_operation, write_stl_ascii, write_stl_binary};
use std::fs::File;

// STEP (AP242 B-Rep) — round-trips solids, including toroidal surfaces.
write_step(&part, "part.step")?;
let reloaded = read_step_operation("part.step")?.value; // + validation/diagnostics

// STL — write a tessellated mesh.
let mesh = tessellate_checked(&part, 0.05, 0.5)?;
let mut f = File::create("part.stl")?;
write_stl_binary(&mesh, &mut f)?;
// or: write_stl_ascii(&mesh, "part", &mut f)?;

// 3MF — zipped mesh packages, including multi-definition assembly models
// (write_3mf / to_3mf_bytes / to_3mf_assembly_bytes).
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

- **Whole-solid Fillet/Chamfer fast paths are box/cylinder-only.** `fillet_with_policy` and
  `chamfer_with_policy` detect a single box or cylinder primitive (at **any
  position/orientation** — the frame is recovered from geometry) and construct the
  result directly. The strict all-edge adapters extend beyond those primitives
  only when every edge passes. `shell_solid_with_policy` has a wider verified analytic
  network, including topology-preserving concave planar sources with exact
  envelope evidence.
  The **per-edge** `fillet_edges` (rolling ball) does handle arbitrary
  planar/analytic edges, including boolean results — and rejects an over-large
  radius rather than emitting a degenerate solid. Equal-radius convex planar
  corner networks are verified for complete selections of valence three through
  six. Partial, higher-valence, mixed-radius, and non-planar networks reject
  explicitly. Contact-curve overflow is exact for convex straight all-planar
  prisms and may cross multiple successive planar faces. Curved successor
  supports, non-prismatic sources, concave corners, and profile-consuming radii
  reject explicitly. General curved-support overflow remains unsupported.
  Concave Shell accepts only a uniquely classifiable topology-preserving
  envelope; periodic or imprinted supports may own multiple exact cells.
  Genuinely branching non-adjacent offset intersections and arbitrary NURBS
  supports reject atomically with typed errors.
- **Booleans: severed cuts stay one body.** Watertight and health-validated on
  through-cuts, face-flush and corner-overlap unions, blind pockets, enclosed
  voids, partial and rotated cuts, and cylinder cuts and bosses (the former
  partial-imprint goal-tests now pass and are un-`#[ignore]`d in
  `crates/openrcad-algo/tests/robustness.rs`). The remaining edge case: a cut that
  *severs* a body is returned as a single solid (Euler=4) rather than split into
  separate bodies — call `Solid::split_disconnected()` on the result to get one
  `Solid` per connected component.
- **Intersection subdivision** can be deep on tangential NURBS configurations;
  prefer analytic primitives (line/circle pairs take a closed-form fast path).
- **STL and 3MF are write-only**; STEP is read + write.

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

// Algorithms — prefer the *_operation / *_with_policy / *_with_cancel forms;
// the bare legacy names (boolean, fillet, chamfer, shell_solid, prism,
// tessellate, read_step, sew) are #[deprecated].
boolean_operation(&object, &tool, BooleanOp::{Fuse|Cut|Common})
    -> Result<OperationResult<Solid>, BooleanError>
fillet_with_policy(&solid, radius, &policy) -> Result<Solid, BlendError>   // whole box/cylinder
chamfer_with_policy(&solid, distance, &policy) -> Result<Solid, BlendError>
shell_solid_with_policy(&solid, thickness, &open_faces, &policy) -> Result<Solid, BlendError>
fillet_edges(&solid, &edges, radius) -> Result<Solid, RollingBallError>   // any solid
chamfer_edges(&solid, &edges, distance) -> Result<Solid, ChamferError>    // any solid
apply_blend_contour(&solid, &BlendContour) -> Result<Solid, BlendContourError>
prism_operation(&face, vector) -> Result<OperationResult<Solid>, SweepError> // extrude a profile
revolve_operation(&face, …) -> Result<OperationResult<Solid>, _>            // solid of revolution

// Mesh & exchange
tessellate_checked(&solid, chord_err, angle_err) -> Result<TriangleMesh, TessellationError>
write_step(&solid, path) -> io::Result<()>
read_step_operation(path) -> Result<OperationResult<Solid>, StepImportError>
write_stl_ascii(&mesh, name, &mut writer) -> io::Result<()>
write_stl_binary(&mesh, &mut writer) -> io::Result<()>
write_3mf / to_3mf_bytes / to_3mf_assembly_bytes                          // 3MF export
```
