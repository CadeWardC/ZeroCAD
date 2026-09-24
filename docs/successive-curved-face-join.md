# Successive curved face Join

The September 13 autosave of Curve reproduced the reported `extrude_10`
failure exactly: a pcurve deviation of 1.9073486328125e-6 and Euler
characteristic -1. The rejected action selected regions 0 and 1 of `sketch_9`
and extruded 10 mm away from the opposite cap of the previously joined body.
The fixture captures the document before that uncommitted action.

Validated profile joins now retain their constituent prism sections for later
parallel joins. Compound sections are separate from the per-part sources used
by Draft and edge editing. Other topology edits invalidate both together.
Serialized checkpoints omit these analytic inputs; partial evaluation therefore
rebuilds a prefix that lacks them, while unchanged finished results remain
eligible for cache reuse. Derived cache ABIs advance to 10; document recipes
and public feature schemas are unchanged.

The sectional arrangement uses the existing f32 sketch endpoint resolution.
Axis-aligned line supports within that resolution and the kernel sewing budget share coordinates, including
overhang endpoints beyond the original edge. Only endpoints incident to those
straight supports participate; analytic curves are preserved. Known straight
section boundaries bypass the legacy circular-arc refitter. Frame translation
is calculated in f64. Final acceptance still requires healthy, strictly valid,
watertight, connected topology and bounds covering all operands.

`curved_face_join.rs` checks the reproduced action against separately built
material, including volume and point containment, cold/warm evaluation,
save/reopen, depth edits, and an edit after serializing the evaluation cache.
Existing overhanging joins cover reverse sweeps and alternate world frames.
This repair supports retained parallel sketch prisms; it does not establish
general freeform fuse robustness.
