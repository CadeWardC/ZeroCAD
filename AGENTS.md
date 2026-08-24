# ZeroCAD Agent Guide

Use this as the quick map before editing. The longer architecture notes live in
`README.md`; this file is for navigation and safe handoff.

## First Stops

- Core parametric behavior: `zerocad-core/src/parametric/`
  - `mod.rs` is the map.
  - `types.rs` owns public feature graph data.
  - `eval.rs` orchestrates evaluation (and owns the revolve/loft/sweep/hole/
    thread/pattern apply paths).
  - `extrude.rs`, `join.rs`, `cut.rs`, and `edge_mod.rs` own operation logic.
  - Per-feature logic lives alongside them: `body_ops.rs`, `datum.rs`,
    `direct_edit.rs`, `draft.rs`, `inspection.rs`, `standards.rs`
    (hole/thread standards library), `thread.rs`, `topo_name.rs`,
    `diagnostics.rs`, `recovery_certificate.rs`.
- Assemblies: `zerocad-core/src/assembly.rs` (documents, occurrence trees,
  transactional commands), `assembly_mates.rs` (mate schema), `assembly_ops.rs`,
  `assembly_solver.rs` (deterministic V2 mate solver), `assembly_release*.rs`.
- Sketch text: `zerocad-core/src/text.rs` (font discovery, shaping, baked glyph
  outlines); GUI dialog in `zerocad-gui/src/text_ui.rs`.
- Sketch solver: `zerocad-core/src/sketch/` — `constraints.rs` (constraint
  model), `solve.rs` (damped Gauss-Newton + DOF analysis), plus offset,
  pattern, trim, and projection.
- Kernel/tessellation behavior: `zerocad-core/src/mock_kernel/`
  - `mod.rs` is the map. This is a thin façade over the **OpenRCAD** B-Rep kernel
    (`openrcad`, a path dep on the sibling `OpenRCAD/` workspace); actual geometry
    work on the kernel itself happens in `OpenRCAD/`.
  - `types.rs` owns `MockMesh` and selectable edge metadata.
  - `tessellation.rs` and `mesh_topology.rs` are the main display-mesh paths.
  - `boolean.rs` owns the guarded booleans; `blend.rs` the native fillet/chamfer.
  - `history.rs` owns face naming, boolean face history, and part identity.
- Document format: `zerocad-core/src/zcad_format.rs` owns the binary `.zcad`
  container (read/write); `parametric/types.rs` owns the `ParametricGraph` it stores.
- GUI behavior: `zerocad-gui/src/app/`
  - `update.rs` is frame orchestration only.
  - `ui/viewport.rs` owns viewport input, picking, and overlays.
  - `ui/top_bar*.rs` owns toolbar actions; `ui/assembly.rs` the assembly panel;
    `ui/constraints_panel.rs` the sketch-constraint UI.
  - `ui/feature_tree.rs` owns the browser; `ui/feature_properties.rs` owns the inspector.
  - Per-feature dialogs live in `zerocad-gui/src/*_ui.rs` (revolve, loft_sweep,
    shell, draft, hole, thread, pattern, move, combine, body_ops, direct_edit,
    inspection, parameters, dxf, text).
  - `evaluation_worker.rs` / `document_worker.rs` own background evaluation and file IO.

## Boundaries

- Keep public data shape stable unless the task is explicitly a file-format or
  schema change. `.zcad` loading depends on serde compatibility.
- Geometry changes need regression tests. Prefer focused tests in
  `zerocad-core/tests/` or the relevant `zerocad-core/src/parametric/tests/`
  module.

## Known Hotspots

- `zerocad-core/src/parametric/edge_mod.rs` is intentionally still large. It
  mixes selection reattachment, native fillet/chamfer attempts, guarded fallbacks,
  and candidate validation. Split only in a dedicated geometry refactor.
- `zerocad-core/src/mock_kernel/tessellation.rs` is intentionally still large.
  It mixes wire construction, mesh flattening, normal repair, and B-Rep edge
  restoration. Split only with full geometry tests.
- Boolean and tessellation code is fragile around coplanar faces, cylinders, and
  orientation. Read the README invariants before changing those paths.

## Checks

Run these before handing back changes:

```bash
cargo fmt --all -- --check
cargo check
cargo test
cargo clippy -p zerocad-core --all-targets
```

Clippy is a required CI gate (the "Clippy (release gate)" job also covers the
OpenRCAD workspace) — fix warnings or annotate them deliberately. Rustfmt,
check, and tests should always pass.
