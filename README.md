# ZeroCAD

A parametric 3D CAD application written in Rust. Sketch 2D profiles on planes,
extrude them into solids, and combine solids with boolean join/cut — all driven
by an editable feature history.

This document is the architectural map. It exists so that a new contributor —
human or agent — does not have to reconstruct the non-obvious design decisions
from comments scattered across the source. Read it before changing the geometry
engine.

---

## Workspace layout

```
ZeroCAD/
├── zerocad-core/        # Pure geometry + parametric engine. No UI, no GPU.
│   ├── geometry.rs      # Vec3, CoordinateSystem (project/unproject a plane).
│   ├── sketch.rs        # 2D curves + detect_regions() (planar arrangement).
│   ├── parametric.rs    # Feature graph (the history tree) + evaluator.
│   ├── mock_kernel.rs   # Thin wrapper over `truck` B-Rep + tessellation.
│   └── units.rs         # mm / inch / meter conversions (base unit = mm).
└── zerocad-gui/         # egui/eframe + wgpu front end.
    ├── main.rs          # App state (ZeroCadApp) + event loop + browser UI.
    ├── render.rs        # CPU-projected viewport (painter's algorithm).
    ├── extrude.rs       # Extrude tool + live preview + default-mode logic.
    ├── sketch_ui.rs     # Fusion-style inline dimension dialogs.
    ├── expr.rs          # Expression evaluator + variable-autocomplete field.
    └── geom2d.rs        # 2D sketch drawing helpers.
```

`zerocad-core` knows nothing about the GUI. The GUI mutates a
`ParametricGraph`, calls `evaluate_bodies*`, and renders the resulting meshes.
Everything in core is `Serialize`/`Deserialize`, so the whole document is one
JSON blob (the foundation for save/load and undo/redo, neither wired up yet).

## Running

```
cargo run --release      # release strongly recommended — the truck solver is CPU-heavy
cargo test --workspace
```

---

## The evaluation pipeline (`parametric.rs`)

`ParametricGraph` is a `petgraph::DiGraph<FeatureNode, ()>`. Each node is a
`FeatureType` (`Origin`, `Box`, `Cylinder`, `Sketch`, `Extrude`, `VariableSet`).
Edges are dependencies (an `Extrude` depends on its `Sketch`).

`evaluate_bodies_with_warnings()` is the entry point. It is intentionally split
into small pieces — extend the matching piece, don't grow one function:

| Function | Responsibility |
|---|---|
| `evaluate_bodies_with_warnings` | Orchestrator: cycle check → assemble → tessellate. |
| `sketch_region_cache` / `cached_regions` | Region detection, memoized (see below). |
| `body_nodes_in_creation_order` | The solid-producing nodes, sorted by creation key. |
| `apply_extrude` | One Extrude node → tool solids → dispatch to a mode. |
| `apply_join` / `apply_cut` | The boolean assemblers (free functions). |
| `tessellate_bodies` | `LiveBody` list → `(id, MockMesh)` list. |

`evaluate_bodies()` is a thin wrapper that drops the warnings; tests and the
preview path use it.

### Adding a new feature type — the checklist

1. Add a variant to `FeatureType` in `parametric.rs`.
2. If it produces a solid, add it to the `matches!` in
   `body_nodes_in_creation_order` and a match arm in
   `evaluate_bodies_with_warnings`.
3. Add a solid builder in `mock_kernel.rs` returning `KernelSolid`. **Read the
   invariants below first** — orientation and handedness will bite you.
4. Add a regression test to `tests/realistic_modes.rs` that asserts the geometry
   actually changed, not just the triangle count.
5. Add the GUI affordance in `main.rs` / a tool module if it's user-facing.

---

## Non-obvious invariants — read before touching geometry

These are the decisions that look wrong until you hit the bug they prevent.

### Booleans are fragile — always go through the guarded wrappers
`truck`'s boolean solver panics outright on some configurations (notably a true
cylinder meeting a box) and silently returns degenerate results on coplanar
faces. **Never call `truck_shapeops::or`/`and` directly.** Use
`mock_kernel::union` / `difference`, which route through `guarded_boolean()`:
it catches panics, silences the panic hook (so a drag frame doesn't spam the
log), and hands back `None`. Callers degrade gracefully from `None`.

### Cylinders are 48-gon prisms, except for display
`cylinder_solid()` builds a 48-sided prism, not a smooth cylinder, because
smooth cylinders make the boolean solver panic. The smooth
`oriented_cylinder_solid()` (4 NURBS quarter-arcs) is **display only** — never
feed it to a boolean. `MockMesh::make_cylinder` tessellates the display form.

### The 0.1 mm coplanarity overshoot
truck cannot resolve a boolean where the tool's cap is coplanar with a body
face. Every join/cut tool is therefore built in two forms:

- **Join**: `exact` (drawn geometry) + `dipped` (near cap nudged
  `CUT_OVERSHOOT = 0.1 mm` *into* the body, via `overshoot_cs`). The dip is
  swallowed by the joined body and leaves no artifact.
- **Cut**: `exact` + `expanded` (end caps pushed clear via `directional_cut`,
  side walls grown `CUT_WALL_GROW = 0.1 mm` past the body face via `grow_loop`).

`exact` is always tried first so the result keeps the dimensions the user drew;
the fallback only runs when the solver rejects `exact`. 0.1 mm is comfortably
above the solver tolerance yet invisible at part scale.

### Creation order, not topological order
Bodies are assembled in **creation order** (the trailing numeric suffix of the
node id, via `creation_key`), *not* topological order. A cut/join extrude acts
on whatever bodies already exist at its point in history, and there is no
dependency edge between, say, a `Box` and a later cut `Extrude`. Sketch → extrude
order is still respected because a sketch's id is always allocated before the
extrude that consumes it.

### Solid orientation / winding handedness
truck booleans require outward-facing solids. The origin-plane consts (XZ/YZ)
are left-handed, and a negative extrude depth also flips winding.
`build_extrusion_solid` XORs `left_handed ^ negative_depth` to reverse the
winding exactly when needed, and `enforce_outward_normals` re-signs mesh normals
after tessellation as a backstop (one centroid–normal dot test per shell).

### Join must never delete material
`a ∪ b` always contains `a`, but truck can hand back a degenerate union (e.g. an
inverted tool that *subtracts*). `apply_join` guards every union with an
AABB-containment check (`aabb_contains`) and rejects any result that no longer
encloses the original body. A cut that fails on both tool variants likewise
keeps the original part intact rather than dropping a valid body.

---

## Warnings — surfacing silent boolean failures

A boolean that doesn't do what the user asked used to fail silently. Evaluation
now returns a `Vec<String>` of **non-fatal** warnings alongside the meshes:

- A **Cut** whose solver fails on a body it overlaps (material left intact).
- A **Join** that overlapped nothing and became a separate body.

Successful coplanarity fallbacks (the 0.1 mm dip/expand) are **not** warned
about — they produce exactly the geometry the user drew, so a warning would be
noise. The GUI shows warnings in the status bar via `reevaluate_geometry`.

---

## Dimension input — expressions + variable autocomplete

Every length box (sketch dimensions and the extrude distance) is the same widget:
`expr::autocomplete_field`. Users can type a number, a variable name, or an
arithmetic expression (`width / 2 + 3`, with `+ - * /` and parentheses).

- **Evaluation** is `zerocad_core::expr::eval`, a pure recursive-descent parser
  (unit-tested in core). It lives in **core** so the parametric engine and the UI
  share one grammar. Everything resolves in the **base unit (mm)**: a variable
  contributes `Variable::value_in_base()`, and literals are mm — matching how the
  geometry consumes dimensions.
- **Autocomplete**: typing an identifier prefix pops a suggestion list of matching
  variable names (from the visible variable sets); ↑/↓ move the selection,
  Enter/Tab or a click accept it. The widget *consumes* those keys
  (`input_mut().consume_key`) when its popup is open, so the host dialog must read
  its own Enter/Escape **after** the field renders — otherwise an Enter meant to
  accept a suggestion would also commit the dialog.

### Live parametric dimensions

Both **extrude depth** and **sketch dimensions** can follow variables, and both
re-resolve on every build against `ParametricGraph::variable_map()` (all
variables, in mm) — so editing a variable updates every dimension bound to it.

- **Extrude depth**: a distance box holding a variable expression stores the raw
  text in `FeatureType::Extrude::depth_expr` (detected via
  `expr::references_variable`); `evaluate_bodies` resolves it each build. `depth`
  is the resolved fallback. The Extrude panel shows/edits the expression; dragging
  the depth slider converts it back to a literal.
- **Sketch dimensions**: a sketch stores a parametric **shape list**
  (`FeatureType::Sketch::shapes`, a `Vec<SketchShape>`) — each shape keeps its
  anchor points and its dimensions as `Dimension { value, expr }`. The live
  geometry is rebuilt from the shapes via `sketch::effective_curves` (→
  `build_sketch_curves`) against the current variables wherever a sketch is
  consumed (region detection in `sketch_region_cache`, the renderer, face
  picking, the extrude tool). Point-driven tools (3-point shapes, ellipses) have
  no dimension fields and are stored pre-built as `SketchShape::Raw`. Legacy
  documents with an empty `shapes` fall back to the baked `curves`.

In the GUI, drawing records a `SketchShape` per shape (`shape_record_from_points`,
capturing the dimension dialog's expressions); `shape_from_points` is now a thin
wrapper that resolves the record so preview and committed geometry share one path.
A sketch dimension's variable binding is set **at draw time** (type a variable
into the dimension box); the sketch property panel surfaces which dimensions are
bound. Re-editing a binding without redrawing is the natural next step.

## Sketch tools — modes, flyouts, and the multi-click state machine

`SketchTool` enumerates every drawing *mode*; each toolbar button is a
`ToolFamily` (Line / Rectangle / Circle / Corner) whose flyout lists its modes
(first = default). Re-clicking an already-armed Rectangle/Circle/Corner button —
or right-clicking it — opens the flyout (`egui::popup_below_widget`).

- **Rectangle**: corner-to-corner (default), center, 3-point (rotated).
- **Circle**: center (default), 3-point, **ellipse**, 3-point ellipse.
- **Corner**: Fillet (default) / Chamfer — one button, like the 3D edge tool.

**Input.** `SketchTool::point_count()` is 2 or 3. The viewport click handler pushes
each snapped click into `sketch_points`; the shape finalizes on the click that
completes the count. 2-point tools open the inline dimension dialog on the first
click; 3-point tools draw by clicking with a live preview (Escape aborts a
half-placed shape).

**One geometry source.** `ZeroCadApp::shape_from_points(last)` builds the
in-progress shape as a fresh `SketchCurves` from the placed points plus the cursor
(folding in typed dimensions for 2-point tools). Both the live preview
(`render.rs`) and `finalize_shape` call it, so preview and committed geometry can
never diverge.

**No new kernel primitives.** Ellipses are faceted into a 48-segment closed
polyline (`SketchCurves::add_ellipse`) — the same polygon-approximation the
48-gon cylinder uses — so region detection, extrusion, and serialization need no
changes. 3-point circles resolve to a real `Circle` (`geom2d::circumcircle`);
rotated/centered rectangles are plain line segments.

## Sketch fillet & chamfer; selecting body geometry while sketching

- **Corner fillet/chamfer.** A sketch carries `FeatureType::Sketch::corner_mods`
  (`Vec<CornerMod>`), applied by `effective_curves` *after* the shapes are built —
  so the parametric shapes stay intact. `CornerMod { at, radius, kind }` snaps `at`
  to the nearest shared vertex, trims the two segments meeting there, and inserts a
  smoothly-tessellated arc (fillet) or a straight bevel (chamfer). `radius` is a
  `Dimension`, so a fillet radius can follow a variable too. The arc is faceted
  adaptively by [`arc_segments`](zerocad-core/src/sketch.rs) — ~3.6°/segment, finer
  for large radii (chord tolerance 0.01 mm) — so it reads as a smooth curve rather
  than a few visible flats, at a cost the planar region-detection can absorb.
- **Staged, multi-corner workflow with live preview.** In the GUI the
  **Fillet**/**Chamfer** tools take a radius/distance from the toolbar (shown with
  the document's unit — `mm`/`in`/`m` — and accepting a number *or* a variable
  expression). Clicking near a corner **stages** it: it previews rounded/beveled
  immediately at the current radius (the pending corners are folded into the live
  curves by `rebuild_active_sketch_curves`, and marked with orange dots). You can
  click several corners and re-tune the radius — the whole preview updates live —
  and nothing is committed until you press **Enter** or click the green **✓ OK**
  button. **Esc** discards the staged corners; finishing the sketch bakes them.
  Pending corners live in `ZeroCadApp::pending_corners`; `commit_pending_corners`
  moves them into `sketch_corner_mods` capturing the radius on each. A Fusion-style
  floating **size box** (`show_corner_radius_box`) tracks the cursor / last staged
  corner so the radius can be typed right at the geometry, and a **drag handle**
  (`drag_corner_radius_handle`) sits on the staged corner's bisector — drag it to
  pull the radius in/out. Box, handle, and toolbar field all edit the same value
  and re-preview the staged corners live. The handle's bisector comes from the
  un-rounded geometry (`corner_bisector`), and its screen axis carries px-per-mm so
  the drag maps 1:1 to millimetres in the sketch plane.
- **Body selection while sketching.** The 3D face/edge/vertex picker (`BodyPick`)
  now also runs in sketch mode when no drawing tool is armed — the **Select**
  toolbar button (or Esc) enters that state. The drawing handler only runs when a
  tool is armed, so the two never conflict.

## 3D edge fillet & chamfer

Select a single body **edge** in the viewport (normal modeling mode) and a
**Modify Edge** panel appears with a single **Fillet ▾** button: a left-click
starts a fillet (the default), and a right-click (or the ▾) opens a flyout to pick
Fillet or Chamfer — the same convention the Rectangle/Circle sketch tools use.
Either way it starts a **live preview** (Fusion-style): the viewport shows the
actual rounded/beveled body in real time. You can set the size two ways — both edit
the same value live (and the inline dialog can still toggle Fillet ↔ Chamfer):

- **Drag manipulator** — a handle on the edge (joined to it by a guide line),
  dragged along the edge's outward bisector to pull the radius in/out. The drag's
  pixels are converted back to millimetres via the projected axis's px-per-mm
  (`drag_edge_mod_handle`), so it tracks the cursor 1:1 at any zoom/orientation.
- **Floating size box** — type a value (`mm`/`in`/`m`, or a variable expression)
  and toggle Fillet ↔ Chamfer.

**Enter / ✓ OK** commits, **Esc / Cancel** aborts.
The preview re-evaluates the model with a temporary node exactly like the
Cut/Join extrude preview ([`preview_edge_mod_bodies`](zerocad-gui/src/edgemod.rs),
mirroring [`preview_extrude_bodies`](zerocad-gui/src/extrude.rs)).

Committing adds a `FeatureType::EdgeMod { target, edge, dist, dist_expr, kind }` to
the history; its distance and type stay editable in the property panel and the
distance can follow a variable.

The truck kernel has no fillet/chamfer builder, so it's done with a **boolean
cut**: [`edge_corner_cutter`](zerocad-core/src/mock_kernel.rs) sweeps the corner
cross-section perpendicular to the edge — a right triangle (chamfer) or that
triangle minus a faceted circular segment (fillet) — along the edge using the
same tested `extruded_region_solid` prism path, then `apply_edge_mod` subtracts it
through the guarded `difference`. The edge's endpoints and two adjacent face
normals are read straight from the body wireframe (`MockMesh::edge_vertices` /
`edge_face_normals`) and frozen in world space on the `EdgeRef`.

Two robustness offsets dodge truck's coplanar/tangent-face boolean failures (the
same hazards the extrude cut fights): the cutter overshoots both ends of the edge
to clear the perpendicular faces, and the fallback cutter inflates its
cross-section outward (`EDGE_MOD_GROW`, comfortably past `BOOL_TOL`) so its tangent
edges lift *off* the body faces and the cut is transversal rather than tangent.
That inflation is a proper **per-edge polygon offset** (`offset_polygon_outward`),
not a radial scale about the centroid — the fillet's cross-section is *concave*
(its arc bulges toward the corner), and a radial scale folds the near-centroid arc
vertices over the legs, which produced garbled filleted bodies. The evaluator also
rejects any boolean result that extends *beyond* the original part (a subtraction
can only remove material), so a tangent/inverted boolean that flares out is caught
and the robust cutter (or an intact body + warning) is used instead.

**One smooth face.** The fillet B-rep is necessarily *faceted* — truck's boolean
solver fails on the smooth cylindrical faces a true round would introduce (the same
reason the app facets all its cylinders for kernel ops). So the round is made to
look like one smooth face three ways:

1. The cutter tessellates the arc adaptively (~3.6°/segment, up to
   `EDGE_FILLET_SEGS`) for a smooth silhouette.
2. [`smooth_vertex_normals`](zerocad-core/src/mock_kernel.rs) blends the facet
   normals across shallow creases (`SHADE_CREASE_COS`, ~30°) so the round carries a
   continuous normal field, and the renderer shades it **Gouraud** (one
   interpolated color per vertex) instead of flat-per-facet — so there's no shading
   banding. Sharp features (90° box corners, 45° chamfers) diverge past the crease
   angle and keep per-face normals, so they stay crisply shaded.
3. [`mesh_feature_edges`](zerocad-core/src/mock_kernel.rs) **suppresses the
   facet-boundary wireframe lines** — a crease below ~18° is a tessellation seam,
   not a design edge, so it isn't drawn.

Together the round (and any boolean'd / many-sided extruded cylinder wall) reads as
one continuous curved face, with no seams and no bands, while genuine edges still
draw. The smoothing runs on every solid mesh but is a no-op for flat faces, so a
plain box stays perfectly crisp.

> **Caveats (honest scope).** This is the tractable slice of a fragile problem.
> It targets **convex** edges of plain box/extrude bodies; a concave edge (where a
> fillet would *add* material) degrades to "no change + warning". The `EdgeRef` is
> frozen in world space, so an `EdgeMod` does **not** follow an upstream dimension
> change — resize the body via a variable and the modifier still cuts at the
> captured location (the guarded boolean copes gracefully if it no longer meets
> material). The fallback cutter trades up to `EDGE_MOD_GROW`mm of exactness for a
> boolean that resolves at all.

## Smart default extrude mode

A sketch records whether it was placed on an origin plane or a body face
(`FeatureType::Sketch::on_face`, set at sketch creation in the GUI). The extrude
tool seeds its mode from that (`extrude::default_extrude_mode`): a plane sketch →
**New Body**; a face sketch pulled **outward** (depth ≥ 0, along the outward face
normal) → **Join**; pushed **inward** (depth < 0) → **Cut**. The default
re-evaluates live as the user drags the depth, until they click a mode button,
which sets `mode_user_set` and freezes their choice.

## Region cache — `detect_regions` memoization

`detect_regions()` (the half-edge DCEL planar arrangement in `sketch.rs`) is a
pure, O(n²) function of a sketch's curves. It is memoized in
`ParametricGraph::region_cache`, keyed by a bit-pattern hash of the curves
(`hash_curves`). The cache:

- is `#[serde(skip)]` (never persisted) and carried by `Clone`, so the
  per-frame graph clone in the extrude-drag preview path starts **warm** — the
  hot path that re-evaluates the whole model every frame no longer re-runs
  planar arrangement for sketches that haven't changed;
- is a transparent accelerator: identical curves always yield identical regions,
  so dropping it (it self-clears at `REGION_CACHE_CAP` entries) is always safe.

This is the safe, correct slice of incremental evaluation. Full per-body boolean
memoization is **deliberately not done**: under whole-model boolean semantics a
body's final mesh depends on every later cut/join that touches it, so naive
per-node mesh caching would be incorrect. That remains future work.

---

## Rendering (`render.rs`)

The viewport is **CPU-projected** — despite the `wgpu` dependency, projection,
depth sorting (painter's algorithm), back-face culling, and hidden-line removal
all run on the CPU. Pristine primitive bodies carry analytic wireframes with
precomputed face normals; boolean results derive feature edges from the
tessellation (`mesh_feature_edges`) using per-face ids. This is correct but does
not scale to large models — moving to a GPU z-buffer is the main performance
lever and a near-total rewrite of this file.

---

## Tests

- `zerocad-core/src/*` inline `#[cfg(test)]` modules — `sketch::tests` covers
  `detect_regions`; `parametric::extrude_mode_tests` covers the boolean modes,
  warnings, and the region cache.
- `zerocad-core/tests/realistic_modes.rs` — the regression suite. Tests assert
  geometry actually changed (e.g. a cut adds hole-wall triangles), not just
  counts. This is the executable spec; add to it when you add a feature.
- `tests/bool_matrix.rs`, `tests/cylinder_tests.rs`, `tests/parametric_tests.rs`.

Gaps worth filling: serialization round-trips, performance benchmarks, and
property-based tests over random profiles.
