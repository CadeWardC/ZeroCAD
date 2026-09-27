# Cut, circular chamfer and face-join repairs

Reproductions: `zerocad-core/tests/fixtures/cut-notwork.zcad` and
`chamfer-notwork.zcad`, supplied from Downloads on 2026-09-26.

The cut sketch includes the existing bore boundary, so the selected cutting
region is an annulus. Circular inner cap wires previously bypassed winding
normalization. Cylindrical lateral construction also mixed material-side
winding with the intrinsic surface normal. A subsequent Boolean could not join
the resulting shoulder consistently. The prism now normalizes circular holes
and lets sewing assign the material side to intrinsically oriented walls.

Closed imprints now transfer enclosed existing holes to the newly enclosed
patch. Boundary healing checks strict topology, aligns mismatched subdivisions,
and sews the aligned edges before accepting a repair. All ordinary validation
gates remain in place.

Circular blends now distinguish a pocket mouth from its floor using the
supporting wall's axial extent. At a concave shoulder both contacts move into
the corner void. Closed-ring contacts must remain within the finite supporting
faces; open chains retain their separate endpoint/runout trimming path.

The saved chamfer distance is about 2.733 mm. The radial shoulder width is
3.925 - 2.2 = 1.725 mm, so that value crosses the smaller bore and is rejected
with a clearance diagnostic. The supported repair is a local shoulder bevel;
continuing an oversized bevel through the smaller bore is not implemented.

`repro_download_geometry` checks the original counterbore, concave chamfers at
0.5 and 1 mm, and atomic rejection of the saved oversized chamfer. It checks
strict solid validity, connectivity, finely tessellated volume against the
analytic change, material/void probes, warm reevaluation, and save/reopen.
Derived solid and display-cache ABIs are incremented; document recipes and the
original Downloads files are unchanged.

Validation includes 159 OpenRCAD algorithm, Boolean, circular-blend, and
concave-blend tests. Formatting, `cargo check`, core Clippy and algorithm Clippy
complete successfully. Clippy still reports existing warnings outside the new
logic. Run logs are under `target/counterbore-chamfer-validation/`.

## Reused face sketches and face joins

Added `cut-notwork-2.zcad` and `join-notwork.zcad`. The second cut goes across
an empty bracket opening into the opposite wall. A projected native circular
rim consists of three arcs; its radius differs from the drawn circle by a few
float32 ULPs. The old projection filter handled sampled chords only. These
arcs created artificial crescent regions and an extra failing cut tool.
`SketchCurves::extend_face_boundary` now removes projected circles/arcs already
covered by a drawn circle within the existing coordinate-rounding tolerance.
Distinct concentric boundaries remain intact.

The initial join save ends at Sketch_4; the failed `extrude_29` preview is not persisted.
Regression tests reconstruct a 5 mm positive extrusion. Region 0 (the rim) and
regions 0+1 (the complete rectangle) fuse to one strictly valid solid. Region 1
alone fills the opening above the bracket and meets the old body only along
edges; it remains an atomic rejection. This is a real non-manifold selection,
not a reason to disable solid validation.

A face's sampled centroid can change after retessellation even when its actual
outline does not. Attached sketches now retain their captured in-plane placement
when the old and current linear outlines cover each other completely. The
comparison supports different edge subdivisions and traversal directions.
Changes to the support plane's normal position still move the sketch with it;
changed outlines retain the existing face-following behavior. This prevents the
saved rectangle from drifting by several millimeters because a mesh centroid
changed. Extrude and other sketch consumers share this placement rule.

New regressions verify the opposite-wall cut's analytic volume and preserve the
starting counterbore/bore; valid joins preserve exact extent, existing holes,
and material through warm evaluation, face reattachment, and save/reopen.
A separate test covers native circular rim projection at three scales and both
orientations while retaining genuine nearby circles.

## Updated committed join

The user replaced the join download at 17:13 with a save that includes
`extrude_27`, depth 5.868814 mm, selected regions 1 and 3. It is retained as
`join-notwork-committed.zcad`. This is a valid combination of a U-shaped cap
patch and the adjacent center; it must not be treated as the initial save's
center-only selection.

Joins now merge adjacent selected analytic profiles before entering the solid
operation. For a prism extending outward from a planar extremity, a shared cap
repair arranges the source cap and tool footprint in 2D, removes only their
shared interface, and sews exposed cap patches to all unchanged side walls.
The enclosing-box test conservatively proves the source lies behind the plane;
positive shared area and strict single-solid validation are mandatory. This
supports partial face contact and overhangs on bodies with cross-axis holes,
without reconstructing or filling those holes and without dipping new material
into the void underneath the join. Other configurations retain existing paths.

Failure diagnostics also test shared-face area independently of a failed Common
operation, so a solver failure cannot hide obvious face contact behind the
edge-only explanation.

The committed reproduction now verifies the exact selected rectangle's added
volume, unchanged 5.868814 mm depth and side extents, no extra material beneath
the bridge, both earlier bores, cold/warm evaluation, reattachment and reopening.
On the final source, all eight download/boundary regressions passed in 44.78
seconds, all 545 core unit tests passed, and all 169 GUI tests passed. The GUI
debug executable was rebuilt successfully. Focused join/lifecycle matrices also
passed all 131 cases before the final conservative tolerance restriction.

The final complete `cargo test` run succeeded: 1,497 tests passed across 100
test binaries/doc-test groups, with zero failures and zero ignored tests.
This includes the break-test workspace, core unit/integration tests and GUI.
Its log is `target/counterbore-chamfer-validation/final-workspace-complete.log`.
Final formatting, `cargo check`, core Clippy, and `git diff --check` succeeded;
the separate OpenRCAD checks above cover the changed kernel implementation.
