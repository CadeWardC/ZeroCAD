# ZeroCAD

A parametric 3D CAD application written in Rust. Sketch 2D profiles on planes,
extrude them into solids, and combine solids with boolean join/cut — all driven
by an editable feature history, and all built on **[OpenRCAD](OpenRCAD/)**, an
in-tree pure-Rust B-Rep geometry kernel.

This document is the architectural map. It exists so that a new contributor —
human or agent — does not have to reconstruct the non-obvious design decisions
from comments scattered across the source. Read it before changing the geometry
engine.

The authoritative delivery roadmap is
[`docs/part-design-master-plan.md`](docs/part-design-master-plan.md). It records
completed phase gates, current work, deferred scope, and the acceptance criteria
for Part Design 1.0.

---

## Workspace layout

```
ZeroCAD/
├── zerocad-core/        # Pure geometry + parametric engine. No UI, no GPU.
│   ├── geometry.rs      # Vec3, CoordinateSystem (project/unproject a plane).
│   ├── sketch.rs        # 2D curves + detect_regions() (planar arrangement).
│   ├── expr.rs          # Recursive-descent expression evaluator (shared with the UI).
│   ├── units.rs         # mm / inch / meter conversions (base unit = mm).
│   ├── stl.rs           # Validated ASCII/binary STL mesh import and binary export.
│   ├── document.rs      # Semantic bodies, timelines, inputs, selectors, feature registry.
│   ├── feature_dto.rs   # Stable numeric-field persistence DTOs for every feature kind.
│   ├── zcad_format.rs   # The binary `.zcad` document container (read/write).
│   ├── parametric/      # Feature graph, evaluator, extrude/join/cut/edge-mod logic.
│   │   ├── types.rs     # FeatureType/FeatureNode, ParametricGraph, EdgeRef/FaceRef,
│   │   │                #   FeatureStatus, the eval/region caches.
│   │   ├── eval.rs      # Orchestrates evaluation + the incremental checkpoint cache.
│   │   ├── extrude.rs   # Extrude → tool solids → dispatch to a mode; LiveBody.
│   │   ├── join.rs / cut.rs   # The boolean assemblers + coplanarity fallbacks.
│   │   └── edge_mod.rs  # 3D fillet/chamfer: native rolling-ball + reattachment.
│   └── mock_kernel/     # Thin façade over the OpenRCAD B-Rep kernel + tessellation.
│       ├── primitives.rs / boolean.rs / blend.rs / edge_ops.rs
│       ├── tessellation.rs / mesh_topology.rs / wireframe.rs / arc_display.rs
│       ├── history.rs   # Face naming + boolean face history + part identity.
│       └── types.rs     # MockMesh + selectable edge/face metadata.
└── zerocad-gui/         # egui/eframe + wgpu front end.
    ├── main.rs          # The ZeroCadApp state struct + process entrypoint.
    ├── render.rs        # CPU-projected viewport (painter's algorithm + HLR).
    ├── extrude.rs / edgemod.rs   # The Extrude and Edge-Mod tools (+ live preview).
    ├── sketch_ui.rs / expr.rs / geom2d.rs   # Dimension dialogs, autocomplete, 2D helpers.
    ├── icons.rs / theme.rs / settings.rs / shortcuts.rs / thumbnail.rs
    └── app/             # ZeroCadApp methods, split by concern.
        ├── base.rs      # Constructor / initial state.
        ├── update.rs    # eframe::App::update — the per-frame orchestrator.
        ├── eval.rs      # Shortcut dispatch + reevaluate_geometry.
        ├── editing.rs / sketch.rs / picking.rs / io.rs   # Sketch camera, region
        │                #   detection, hit-testing, and .zcad/STL file I/O.
        └── ui/          # egui panels: viewport, top_bar*, feature_tree,
                         #   feature_properties, extrude_panel, status_bar, …
```

`zerocad-core` knows nothing about the GUI. The GUI mutates a
`ParametricGraph`, calls `evaluate_bodies*`, and renders the resulting meshes.
The geometry itself is produced by **OpenRCAD** (`openrcad`, a path dependency on
the sibling `OpenRCAD/` workspace): `mock_kernel` builds real
`openrcad::topo::Solid`s, tessellates them with `openrcad::mesh`, and flattens
the result into the interleaved position+normal buffer the egui painter expects.

The runtime graph is paired with a semantic document sidecar that owns explicit
bodies, timelines, feature inputs, sequence keys, suppression, units, and
visibility. Save/Load persists that stable semantic recipe rather than petgraph
arena indices or Rust enum field names (see
[The `.zcad` document format](#the-zcad-document-format)). Snapshot
**Undo/Redo** captures both recipe and presentation state. The same tessellated meshes feed
**binary STL export** (`zerocad_core::stl`) for handing models to slicers and
mesh tools.

## Running

```
cargo run --release      # release strongly recommended — the geometry kernel is CPU-heavy
cargo test --workspace
```

## Keyboard shortcuts

Common actions are bound to keys and **rebindable** under *Settings → Shortcuts*
(persisted to the OS config dir). Defaults: New `Ctrl+N`, Open `Ctrl+O`, Save
`Ctrl+S`, Export STL `Ctrl+E`, Undo `Ctrl+Z`, Redo `Ctrl+Y`, Delete `Delete`,
Toggle Dark Mode `Ctrl+D`, Settings `Ctrl+,`.

---

## The geometry kernel — OpenRCAD

ZeroCAD's solids are built by **[OpenRCAD](OpenRCAD/README.md)**, a pure-Rust
B-Rep CAD kernel that lives in this tree as its own cargo workspace and is
consumed only through the `openrcad` façade crate. It provides points/vectors,
NURBS curves and surfaces, arena B-Rep topology, primitive builders (`make_box`,
`make_cylinder`, …), BVH-accelerated booleans (`boolean_checked`), rolling-ball
fillets (`fillet_edges`), and parallel tessellation (`tessellate`).

`zerocad-core::mock_kernel` is a thin façade over that kernel. The `MockMesh`
name and field layout are preserved from ZeroCAD's earlier (pre-OpenRCAD)
geometry backend so the parametric and rendering code kept working unchanged;
internally every constructor now drives a real OpenRCAD solid.

> **History note.** ZeroCAD was originally built on the [`truck`] kernel.
> `truck`'s boolean solver panicked on some configurations, had no fillet/chamfer
> builder, and failed on smooth faces — which is precisely why OpenRCAD was
> written. Comments in the source that still say "truck" are describing the
> boolean/tessellation path in the abstract; the actual calls all go to
> OpenRCAD now. If you touch one of those, please update the wording.

[`truck`]: https://github.com/ricosjp/truck

---

## The evaluation pipeline (`zerocad-core/src/parametric/`)

`ParametricGraph` is a `petgraph::DiGraph<FeatureNode, ()>` paired with
`DocumentSemantics`. Each node has a stable feature-kind id and payload version;
the semantic sidecar owns its explicit inputs, sequence key, suppression state,
and body timeline membership. Graph edges remain the runtime dependency index.

`evaluate_bodies_with_warnings()` is the entry point. It is intentionally split
into small pieces — extend the matching piece, don't grow one function:

| Function | Responsibility |
|---|---|
| `evaluate_bodies_with_warnings` | Public orchestrator: build the live bodies, then tessellate them. |
| `evaluate_bodies_with_status` | Same, but also returns a per-feature `FeatureStatus` list (see [Warnings](#warnings--surfacing-silent-boolean-failures)). |
| `evaluate_bodies_draft` | Live-preview variant. Historically used a faster faceted fillet; now identical output (native fillets need no draft/commit split). |
| `build_live` / `evaluate_bodies_inner` | Assemble the `LiveBody` list in creation order, with the incremental checkpoint cache. |
| `sketch_region_cache` / `cached_regions` | Region detection, memoized (see [Region cache](#caches--region-detection-and-incremental-evaluation)). |
| `body_nodes_in_creation_order` | The solid-producing nodes, sorted by creation key. |
| `apply_extrude` | One Extrude node → tool solids → dispatch to a mode. |
| `apply_join` / `apply_cut` | The boolean assemblers (free functions in `join.rs` / `cut.rs`). |
| `apply_edge_mod` | One EdgeMod node → native rolling-ball fillet/chamfer, reattaching its edge. |
| `tessellate_bodies` | `LiveBody` list → `(id, MockMesh)` list. |

`evaluate_bodies()` is a thin wrapper that drops the warnings; tests and the
preview path use it.

### Adding a new feature type — the checklist

1. Add a variant to `FeatureType` in `zerocad-core/src/parametric/types.rs`.
2. Register its stable kind id, payload version, evaluator kind, editor group,
   and intrinsic inputs in `zerocad-core/src/document.rs`.
3. Add its numeric-field encode/decode contract in
   `zerocad-core/src/feature_dto.rs`. Never persist Rust field or enum names as
   the feature ABI.
4. If it produces a solid, add it to the `matches!` in
   `body_nodes_in_creation_order` and a match arm in the evaluator.
5. Add a solid builder under `zerocad-core/src/mock_kernel/` returning a
   `KernelSolid` (an `openrcad::topo::Solid`). **Read the invariants below
   first** — orientation and handedness will bite you.
6. Add a regression test to `tests/realistic_modes.rs` that asserts the geometry
   actually changed, not just the triangle count.
7. Add the GUI affordance under `zerocad-gui/src/app/` (state + a `ui/` panel) or
   the relevant tool module if it's user-facing.

---

## Non-obvious invariants — read before touching geometry

These are the decisions that look wrong until you hit the bug they prevent.

### Always go through the guarded boolean wrappers
OpenRCAD's `boolean_checked` returns a `Result` and can *reject* a configuration
it cannot resolve watertightly (and, defensively, its own catch-unwind guards a
panic in the solver). **Never call `openrcad::algo::boolean*` directly.** Use
`mock_kernel::union` / `difference`, which wrap it in `quiet_panic()` (silences
the panic hook so a degraded drag frame doesn't spam the console) and hand back
an `Option` — `None` on any failure or non-watertight output. Callers degrade
gracefully from `None` (keep the original body intact, raise a warning). This is
also why the release profile keeps `panic = "unwind"` (see the note in the root
`Cargo.toml`): a catch-unwind in the boolean path depends on it.

`difference_bodies` is the multi-body variant: a cut that *severs* a part comes
back from the kernel as one watertight shell holding both lumps, and this splits
it into one `KernelSolid` per connected component (sorted by a stable position
key so the part list is deterministic across rebuilds).

### Cylinders are native and smooth — faceting is a retired fallback
Circular geometry is now built as **true analytic cylinders** (OpenRCAD
`make_cylinder`, and native circular join/cut/boss tools), so a Ø-boss or a
bored hole reads perfectly round. The old "always facet a cylinder into a 48-gon
prism" rule — a workaround for the since-retired `truck` panic — survives only as
a *defensive fallback* should a native build ever fail.

A **sketched** circle or arc is drawn as a many-segment polyline, so an extruded
circle would naively become a faceted prism. `loop_to_wire` (in `mock_kernel`)
refits co-circular runs of that polyline back into real `Circle` arc edges, so
the prism builds smooth cylindrical walls. Geometry the user genuinely wants
faceted — an octagon, an ellipse — turns too sharply per vertex to be mistaken
for an arc, so it stays a crisp polygon. (`arc_reconstruction_tests` in
`mock_kernel/mod.rs` pins both directions.) `CIRCLE_SEGS = 48` in
`zerocad-core/src/lib.rs` is the single source of truth for the segment count,
shared by sketch arrangement, ellipse faceting, and cylinder wireframes.

### The 0.1 mm coplanarity fallback
OpenRCAD merges coplanar adjacent faces and closes coplanar boss/hole imprints,
so the everyday join/cut cases resolve on the geometry the user drew. As a
robustness fallback for the cases the solver still rejects, every join/cut tool
is built in ordered variants and tried in turn:

- **Join** (`JoinTool`): `smooth` (analytic cylinder for a circular boss) →
  `exact` (faceted prism, perfect dimensions) → `dipped` (near cap nudged
  `CUT_OVERSHOOT = 0.1 mm` *into* the body to break coplanarity; the dip is
  swallowed by the joined body).
- **Cut** (`CutTool`): `smooth` → `exact` → `expanded` (end caps pushed clear via
  `directional_cut`, side walls grown `CUT_WALL_GROW = 0.1 mm` past the body face
  via `grow_loop`). Each also has a **reversed-direction** variant so a cut whose
  drawn direction sweeps into empty air still bites material (fixes "cut works
  once, then does nothing").

`smooth`/`exact` are always preferred so the result keeps the dimensions the user
drew; a fallback runs only when the solver rejects the earlier variant. 0.1 mm is
comfortably above the solver tolerance yet invisible at part scale.

### Explicit sequence, not graph arena or identifier order
Bodies are assembled by the semantic `SequenceKey`, *not* petgraph insertion
order or a feature id suffix. A cut/join extrude or edge mod acts on the bodies
that exist at its point in the explicit timeline. The old numeric-suffix rule is
only a compatibility fallback while loading in-memory graphs that predate the
semantic sidecar; normalization immediately records explicit sequence keys.

### Solid orientation / winding handedness
Kernel booleans require outward-facing solids. The origin-plane consts (XZ/YZ)
are left-handed, and a negative extrude depth also flips winding.
`build_extrusion_solid` XORs `left_handed ^ negative_depth` to reverse the
winding exactly when needed, and `enforce_outward_normals` re-signs mesh normals
after tessellation as a backstop (one centroid–normal dot test per shell).

### Join is a transactional solid operation
`a ∪ b` always contains both inputs and, for a modeling Join, must produce one
valid solid. `apply_join` accepts a result only when it encloses both operands,
is healthy and watertight, and has one connected component. It then repeatedly
merges any additional body parts that the new result touches, so a bridge feature
cannot leave an order-dependent collection of overlapping parts behind.

A multi-region Join is atomic: if any region cannot produce a valid fused solid,
the entire feature is rolled back and marked unresolved. It never appends the
failed tool as a visually coincident second part. A cut that fails on every tool
variant likewise keeps the original part intact rather than dropping a valid
body; the edge-mod path similarly rejects any "fillet" result that adds material
or extends beyond the pre-edit body.

---

## Warnings — surfacing silent boolean failures

A boolean that doesn't do what the user asked used to fail silently. Evaluation
now returns a `Vec<String>` of **non-fatal** warnings alongside the meshes:

- A **Cut** whose solver fails on a body it overlaps (material left intact).
- A **Join** that could not fuse every tool region into one valid solid (the
  entire feature is rolled back).
- An **EdgeMod** whose fillet/chamfer could not be applied (edge not found on the
  rebuilt body, radius infeasible, result not subtractive).

Successful coplanarity fallbacks (the 0.1 mm dip/expand) are **not** warned
about — they produce exactly the geometry the user drew, so a warning would be
noise. The GUI shows warnings in the status bar via `reevaluate_geometry`.

`evaluate_bodies_with_status` returns the same information in structured form: a
`Vec<FeatureStatus>` (one per feature, in creation order), each `Resolved` or
`Unresolved(reason)`. That lets the history tree flag *which* node failed with a
red marker, rather than only showing a global count — the "fail loud and
attributable" contract of history reattachment (see below).

---

## Persistent topological naming — history reattachment

When an upstream dimension changes, a downstream feature (a fillet on a specific
edge, a sketch placed on a specific face) must still find *the same* edge or face
on the rebuilt body. ZeroCAD is moving from purely geometric re-selection toward
**identity-based** naming, with geometry kept as a fallback.

- **Faces are the primary named entity.** Through a boolean, `mock_kernel::history`
  maps each result face back to an input face by its **supporting-surface
  signature** (quantized plane normal+offset, or cylinder axis+radius), with the
  earlier feature winning ties (`boolean_face_history`), then carries the durable
  name onto the tessellated mesh (`propagate_face_names`). A face name looks like
  `sketch:extrude_3:region:0:face:top`.
- **Edges derive identity from the pair of faces they separate.** An `EdgeMod`
  stores a `TopologyEdgeRef` (body id, edge id, adjacent-face names) alongside the
  captured world-space `EdgeRef`. On rebuild, `resolve_edge_ref_by_topology`
  tries an exact edge-id match, then a face-owner-pair match disambiguated by
  geometry, and only falls back to the raw captured endpoints+normals for legacy
  documents or genuinely changed topology.
- **A sketch on a body face** stores a `FaceRef`/`TopologyFaceRef` in
  `ParametricGraph::sketch_face_refs`; on rebuild the sketch's plane is re-derived
  from wherever that face now is, so a sketch-on-face follows the body.
- **Component identity** is stamped onto every selectable face from a stable,
  quantized solid key. Face references therefore identify both their owning body
  and their solid component. Downstream features such as Thread resolve that
  component first instead of depending on whichever internal part happens to be
  visited first. If an edit changes the component's bounds, durable face naming
  and nearest-geometry matching provide the controlled reattachment fallback.
- **Fail loud.** A feature that cannot reattach its reference is marked
  `Unresolved` (surfaced in the tree and status bar) instead of silently applying
  to the wrong entity.

This is active work; the `reattachment_matrix.rs` test suite is the executable
spec for what currently reattaches. Non-analytic (spline/NURBS) surfaces are not
yet named, and edge reattachment through a cut can still fall back to geometry.

---

## Caches — region detection and incremental evaluation

Two `#[serde(skip)]` caches on `ParametricGraph` keep the per-frame re-evaluation
in the preview paths fast. Both are transparent accelerators — carried by `Clone`
(so the per-frame graph clone starts warm) and always safe to drop.

- **`region_cache`** memoizes `detect_regions()` — the half-edge DCEL planar
  arrangement in `sketch.rs`, a pure O(n²) function of a sketch's curves — keyed
  by a content hash of the curves (`hash_curves`). Identical curves always yield
  identical regions, so this only accelerates; it self-clears at
  `REGION_CACHE_CAP` entries.
- **`eval_cache`** holds per-node **checkpoints** of the assembled bodies, one per
  body node in creation order, keyed by a cumulative content hash of every input
  that node's geometry depends on (variables, feature content, upstream nodes,
  hidden state). When an edit changes only a trailing node — dragging a fillet
  radius, tweaking an extrude depth — the prefix hashes still match, so the
  expensive upstream booleans are restored from the checkpoint instead of
  recomputed every frame. Each checkpoint also caches that prefix's warnings and
  `FeatureStatus` list.

Full per-body boolean memoization across the *whole* model is deliberately not
done: under whole-model boolean semantics a body's final mesh depends on every
later cut/join that touches it, so naive per-node output caching would be
incorrect. The prefix-checkpoint scheme is the correct, conservative slice.

Checkpoint meshes are held behind `Arc`, so snapshots of a long unchanged
prefix share the same vertex/index allocation instead of copying it once per
feature. The GUI also owns one persistent, latest-wins model evaluator: direct
manipulation draws a lightweight ghost immediately, waits 100 ms for the input
to settle, and then schedules an interactive B-Rep preview. A newer request
cooperatively cancels the obsolete one; committed/final evaluation has priority.

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
  consumed (region detection, the renderer, face picking, the extrude tool).
  Point-driven tools (3-point shapes, ellipses) have no dimension fields and are
  stored pre-built as `SketchShape::Raw`. Legacy documents with an empty `shapes`
  fall back to the baked `curves`.
- **EdgeMod distance** follows the same pattern (`dist_expr`).

In the GUI, drawing records a `SketchShape` per shape (`shape_record_from_points`,
capturing the dimension dialog's expressions); `shape_from_points` is now a thin
wrapper that resolves the record so preview and committed geometry share one path.

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
cylinder fallback uses — so region detection, extrusion, and serialization need no
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
  also runs in sketch mode when no drawing tool is armed — the **Select**
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

**Enter / ✓ OK** commits, **Esc / Cancel** aborts. The preview uses an immediate
lightweight overlay, then submits a temporary-node evaluation to the shared
background worker and caches the exact result (`cached_preview_edge_mod_bodies`,
mirroring `cached_preview_extrude_bodies`). Committing adds a
`FeatureType::EdgeMod { target, edge, dist, dist_expr, kind, replay }` to the
history; its distance and type stay editable in the property panel and the
distance can follow a variable.

**Native rolling-ball blends.** OpenRCAD *does* have a blend builder, so the fillet
is a **true geometric round**, not a boolean approximation.
[`apply_edge_mod`](zerocad-core/src/parametric/edge_mod.rs) locates the captured
edge in each part's B-Rep (by its endpoints, then by the reattachment path above)
and calls OpenRCAD's `fillet_edges` (via `mock_kernel::fillet_edge`) to replace it
with a real cylindrical fillet face — or `chamfer_edge` for a bevel. There is no
edge-mod draft/commit split: the background preview and committed model use the
same operation. Radius feasibility is delegated to the exact kernel solve rather
than a conservative app-side estimate, and every candidate is validated to be
**local and subtractive** — a result that refills a
cut void or adds visible material is rejected and the body left unchanged with a
warning.

**Filleting through earlier cuts.** An `EdgeModReplayIntent` lets a fillet on an
edge that a later cut passes through be reconstructed correctly: imprint the fillet
before the cut, or re-apply the cut history after the fillet, whichever validates.
Circular rim edges use native-only mode (no construction replay).

**One smooth face.** The native fillet is a single analytic cylindrical/toroidal
face, so it is smooth by construction. Where another curved face is tessellated
into facets (for example, a many-sided extruded wall), three mechanisms
make it read as one continuous surface: adaptive arc tessellation for a smooth
silhouette; [`smooth_vertex_normals`](zerocad-core/src/mock_kernel/tessellation.rs)
blending facet normals across shallow creases (`SHADE_CREASE_COS`, ~30°) for
Gouraud shading with no banding; and
[`mesh_feature_edges`](zerocad-core/src/mock_kernel/mesh_topology.rs) suppressing
sub-~18° facet-boundary wireframe lines (tessellation seams, not design edges).
Sharp features (90° box corners, 45° chamfers) diverge past the crease angle and
stay crisp. The smoothing runs on every solid mesh but is a no-op for flat faces,
so a plain box stays perfectly crisp.

> **Caveats (honest scope).** This targets edges of box/extrude/boolean bodies.
> The `EdgeRef` is captured in world space and reattached by the naming path
> above; where reattachment falls back to geometry, an `EdgeMod` may not perfectly
> follow a large upstream dimension change. The fallback cutter trades up to
> `EDGE_MOD_GROW`mm of exactness for a boolean that resolves at all.

## Smart default extrude mode

A sketch records whether it was placed on an origin plane or a body face
(`FeatureType::Sketch::on_face`, set at sketch creation in the GUI). The extrude
tool seeds its mode from that (`extrude::default_extrude_mode`): a plane sketch →
**New Body**; a face sketch pulled **outward** (depth ≥ 0, along the outward face
normal) → **Join**; pushed **inward** (depth < 0) → **Cut**. The default
re-evaluates live as the user drags the depth, until they click a mode button,
which sets `mode_user_set` and freezes their choice.

---

## Rendering (`render.rs`)

The default viewport embeds `openrcad-render` into eframe's wgpu device and
composites its depth-tested offscreen texture below egui's sketch/dimension
overlays. Geometry uploads are per-body and content-fingerprinted; unchanged
scenes reuse their rendered texture. Face hover uses a three-buffer asynchronous
GPU readback ring, so pointer motion never waits for `device.poll(Wait)`. The CPU
projector remains the automatic fallback when wgpu is unavailable or disabled.

---

## The `.zcad` document format

A saved model is a binary `.zcad` v5 container
(`zerocad-core/src/zcad_format.rs`), not plain JSON. Its fixed 32-byte header is
followed by 48-byte section-table entries and independently stored or
zstd-compressed payloads. Header and section integrity use truncated BLAKE3
digests, checked lengths, non-overlap rules, bounded decompression, and explicit
required/disposable flags.

- The authoritative `DocumentRecipeV3` uses deterministic CBOR maps with numeric
  keys. Every feature record carries a stable kind id, payload version, numeric
  field map, explicit inputs, sequence, suppression, and body membership. Large
  STEP data is a required content-addressed asset, deduplicated by BLAKE3.
- `write_document[_to_vec|_file]` and
  `read_document[_from_slice|_file]` are the canonical APIs. The reader and
  writer process one section at a time; file APIs do not stage a second complete
  container in memory. Deprecated `write_zcad`/`read_zcad` wrappers remain only
  for the frozen Phase 0 and explicit compatibility tests.
- `SaveProfile::Compact` stores the recipe, required assets, manifest, and an
  optional preview capped at 32 KiB. It never stores meshes, B-Rep checkpoints,
  or a large preview. `SaveProfile::Hydrated` may add those disposable
  accelerators within a 64/128/256/512 MiB budget, dropping mesh, then
  checkpoints, then the large preview as needed.
- The manifest records container, recipe, feature-payload, required-asset,
  OpenRCAD, tessellation, and tolerance ABIs. A model hash binds the canonical
  recipe and sorted required assets; a separate presentation hash binds units
  and visibility. Visibility never invalidates geometry caches.
- Optional accelerators are independently digest-checked, ABI-bound,
  model-bound, and structurally validated. Missing, stale, corrupt, or removable
  optional sections produce diagnostics and fall back to a deterministic
  rebuild; they cannot invalidate the authoritative recipe.
- File writes use a synced sibling temporary file plus backup replacement so a
  failure leaves either the complete previous document or the complete new one.
  The loader enforces section-count and decoded-size ceilings before allocation.

The full contract and migration rules are documented in
[`docs/phase2-document-architecture.md`](docs/phase2-document-architecture.md).

The GUI's **Recent projects** onboarding screen shows each file's thumbnail. The
thumbnail is a small CPU-rasterized 3/4-isometric preview of the evaluated meshes
(`zerocad-gui/src/thumbnail.rs`) — z-buffered, flat-shaded, no GPU — PNG-encoded
and embedded in the file's thumbnail section.

---

## Tests

- `zerocad-core/src/*` inline `#[cfg(test)]` modules — `sketch::tests` covers
  `detect_regions`; `mock_kernel` covers wireframe grouping and arc
  reconstruction; `parametric::tests` covers the boolean modes, warnings, the
  caches, edge mods, and the reattachment matrix.
- `zerocad-core/tests/realistic_modes.rs` — the regression suite. Tests assert
  geometry actually changed (e.g. a cut adds hole-wall triangles), not just
  counts. This is the executable spec; add to it when you add a feature.
- Targeted repro suites: `repro_fillet_then_cut.rs`, `repro_fillet_then_fillet.rs`,
  `repro_cutout_fillet_mesh.rs`, `repro_miter_render.rs`, `smooth_cylinder.rs`,
  `sketch_fillet_extrude.rs`, `primitive_equals_extrude.rs`, plus
  `bool_matrix.rs`, `cylinder_tests.rs`, `parametric_tests.rs`.
- `tests/serialization.rs` and `tests/zcad_format.rs` — `.zcad` binary
  round-trip for every feature kind, byte determinism, save-profile enforcement,
  streaming, atomic replacement, bounded malformed-input handling, and
  independent optional-cache corruption recovery.
  `stl::tests` covers binary STL export.

`benches/modeling_pipeline.rs` tracks cold, warm, and hydrated-open long-history
evaluation with Criterion. `tests/property_geometry.rs` generates bounded mechanical through-hole
cases and asserts healthy, watertight, finite, repeatable results.

---

## Contributing & license

See [CONTRIBUTING.md](CONTRIBUTING.md) for build/test/style conventions and
[AGENTS.md](AGENTS.md) for a navigation map of the source. CI (GitHub Actions)
builds and tests the full workspace on Windows and the core engine on Linux on
every push and PR, with `cargo fmt --all -- --check` as a required gate and
clippy advisory.

ZeroCAD is licensed under either of [MIT](LICENSE-MIT) or
[Apache-2.0](LICENSE-APACHE) at your option. Unless you state otherwise, any
contribution you submit is dual-licensed under the same terms.
