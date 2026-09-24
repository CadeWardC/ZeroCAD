# Overhanging face Join repair

The saved `Curve.zcad` model reproduced `extrude_5` failing with an invalid
Euler characteristic of -15 and a free edge. Its two selected regions extend
beyond a concave supporting cap. The contained-boss fast path could not handle
that footprint, and the general 3D fuse failed while partitioning the interface.

Parallel, undrafted sketch prisms now have an exact sectional union path. It
arranges the line/arc footprints in a common frame, unions occupied cells in
each depth interval, and assembles only exposed side walls and caps. All selected
regions participate together. The existing contained-boss path remains first.
Other solids and unsupported curves continue through the guarded boolean paths.

Public kernel sewing now invokes the existing strictly validated T-junction
repair before rejecting a shell with differently subdivided coincident edges.
The operation records that repair in its recovery report. It cannot close an
actually missing face. Joins still require one healthy, watertight solid; failed
features remain atomic. Drafted tools do not carry a plain-prism profile hint.
Derived geometry cache ABIs advance to 7; authoritative document data is unchanged.

Regression coverage:

- `overhanging_face_join.rs` replays the saved model, compares volume and material
  samples against separately extruded inputs, checks strict topology, repeats warm
  evaluation, and saves/reopens the document.
- The same regression changes join depth, reverses both sweeps, and rotates and
  translates the support frames.
- `canonical_sew_heals_mismatched_boundary_subdivisions` splits one face's edge,
  verifies validated sewing and repair metadata, and rejects a missing-face shell.

This supports parallel line/arc sketch prisms; it does not establish general
freeform boolean robustness.

Validation: all three document regressions, all 125 kernel algorithm unit tests,
and 43 existing geometry/history/document-format regressions passed. `cargo check`
and the Clippy warning-baseline gate passed (no new warnings). The full `cargo test`
was attempted but Windows rejected linking the core test executable with LNK1104
because another test process in this shared workspace still held it open.
