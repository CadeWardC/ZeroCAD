# Merged polygon profile joins

The saved `Fit_GPU.zcad` model reproduced an unresolved `extrude_7` join:
the general Boolean fuse returned invalid Euler topology and a free edge.
Its base extrusion combines two selected arrangement regions into a concave
12-sided outline. The subsequent join uses another region of that sketch at
a different height.

The shared profile preparation code deliberately discarded analytic data for
all-line merged outlines to avoid collinear face divisions. That also discarded
the exact source needed by sectional joins and cuts. Only four-sided outlines
could recover that source later, so this model fell through to the failing
general Boolean path.

Merged all-line profiles now retain analytic loops, including holes. Consecutive
collinear spans are coalesced using the existing profile endpoint tolerance,
preserving their line supports and avoiding redundant side faces. The existing
sectional operation can then assemble the exposed boundary at each height.
Watertightness, strict validation, connectedness, and containment gates remain
in place. No file names, feature IDs, or particular dimensions enter the fix.

Derived geometry cache versions are advanced; the authoritative document recipe
and public data formats remain unchanged. Older files rebuild their caches.

Regression coverage includes the saved model, cold/warm evaluation,
save/reopen, changed heights, negative extrusion, rotated/translated frames,
volume and material/void comparison against independently extruded regions,
and a synthetic split rectangular ring with a retained four-sided hole.

Scope: merged straight-sided sketch profiles eligible for the existing parallel
prism join/cut paths. This does not claim to repair every general 3D Boolean
failure; unsupported geometry still fails atomically.

Validation on 2026-09-26: the saved-model/edited-model regressions, synthetic
ring regression, existing overhanging joins, formatting, `cargo check`, and
the Clippy baseline gate for both workspaces passed. The complete `cargo test`
run was stopped after approximately 40 minutes in unrelated threaded-solid
stress tests; no assertion failures had been reported before that stop. That
run is incomplete, not a full-suite pass. A separate core/GUI run passed all
545 core unit tests, then stopped at four `circular_rim_blend` integration
failures (open bite-arc fillet/chamfer cases). Two construct their operands
directly through OpenRCAD primitives/booleans and never call the modified
profile preparation code. Kernel and GUI files were being edited concurrently
in this workspace; those edits are outside this fix. Details are recorded in
`target/fit-gpu-core-gui-test.log`. No full-suite success is claimed.
The final targeted rerun passed both saved-model tests and all three existing
overhanging-join tests against the rebuilt workspace. A separate GUI run passed
all 169 tests (`target/fit-gpu-gui-test.log`).

The running Windows application locked the normal executable destination.
The newly linked debug executable was copied to
`target/debug/ZeroCAD-GPU-Fit.exe` without closing the open application.
