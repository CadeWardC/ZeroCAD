# ZeroCAD Agent Guide

Use this as the quick map before editing. The longer architecture notes live in
`README.md`; this file is for navigation and safe handoff.

## First Stops

- Core parametric behavior: `zerocad-core/src/parametric/`
  - `mod.rs` is the map.
  - `types.rs` owns public feature graph data.
  - `eval.rs` orchestrates evaluation.
  - `extrude.rs`, `join.rs`, `cut.rs`, and `edge_mod.rs` own operation logic.
- Kernel/tessellation behavior: `zerocad-core/src/mock_kernel/`
  - `mod.rs` is the map. This is a thin façade over the **OpenRCAD** B-Rep kernel
    (`openrcad`, a path dep on the sibling `OpenRCAD/` workspace); actual geometry
    work on the kernel itself happens in `OpenRCAD/`.
  - `types.rs` owns `MockMesh` and selectable edge metadata.
  - `tessellation.rs` and `mesh_topology.rs` are the main display-mesh paths.
  - `history.rs` owns face naming, boolean face history, and part identity.
- Document format: `zerocad-core/src/zcad_format.rs` owns the binary `.zcad`
  container (read/write); `parametric/types.rs` owns the `ParametricGraph` it stores.
- GUI behavior: `zerocad-gui/src/app/`
  - `update.rs` is frame orchestration only.
  - `ui/viewport.rs` owns viewport input, picking, and overlays.
  - `ui/top_bar*.rs` owns toolbar actions.
  - `ui/feature_tree.rs` owns the browser; `ui/feature_properties.rs` owns the inspector.

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

Clippy is advisory; rustfmt, check, and tests should pass for cleanup work.
