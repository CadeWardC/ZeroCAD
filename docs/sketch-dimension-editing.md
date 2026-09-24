# Sketch dimension editing

The Dimension tool edits the existing driver for a circle, line, or constrained
rectangle instead of adding a conflicting second size constraint.

1. Create a sketch or edit the source sketch of an existing body.
2. Choose Dimension, then select a circle outline, a line, or two parallel lines.
3. Click away from the geometry to place the value, type a size or expression,
   and press Enter. Circles use diameter; arcs use radius.
4. Finish Sketch commits the source sketch in place and rebuilds dependent bodies.

Either edge of a rectangle edits the corresponding width or height. Selecting
two opposite sides edits their separation. Repeating the picks reopens the
existing driver; it does not accumulate size constraints. Escape restores the
pre-edit sketch, including solved positions, and Undo reverses the whole edit.
Invalid or conflicting typed values preserve the last valid model.

Driver matching lives in `zerocad-core/src/sketch/dimension.rs`. Rectangle
equivalence follows horizontal, vertical, and coincident constraints, not
coordinate proximity or shape names. Rotated or arbitrarily constrained
quadrilaterals are not inferred as equivalent rectangle dimensions. No document
schema or runtime dependency was added. Annotation placement remains session
state; saved dimensions can be reopened by picking the geometry again.

Regression coverage includes circle diameter and all four rectangle edges,
both opposite-side pairs, atomic undo, invalid lengths, extruded volume changes,
warm evaluation, save/reopen, expression preservation, and independent overlapping
shapes. The native interaction check was interrupted by the user's Escape key;
interactive usability and latency remain unmeasured. Matching builds coordinate
components once per pick; there is no persistent cache or background task.

Validation: formatting and compilation pass; all 162 GUI tests and both new
core driver tests pass. Core Clippy completes with existing warnings and no
diagnostics in the new dimension module. The full test command stops after
540 core tests pass and two geometry tests fail:
`delete_external_cylindrical_boss_restores_the_supporting_body` and
`cut_pocket_into_threaded_shaft_removes_material`. Neither calls the new driver
resolver. The former fails to heal a boss; the latter leaves pocket volume
unchanged. These failures remain outside this sketch-tool change.
