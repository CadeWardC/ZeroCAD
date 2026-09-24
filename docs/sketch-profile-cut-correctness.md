# Sketch regions and parallel profile cuts

The `overlapping-face.zcad` and `overlapping-face-cut.zcad` regression fixtures
capture a plate with an intersecting circle/rounded rectangle and a larger
rounded recess sketched on its face.

The failures had several distinct causes:

- The viewport reclassified assigned holes from a boundary vertex and bridged
  adjacent holes, producing overlapping highlight triangles.
- The planar arrangement retained open strokes as zero-width boundary detours
  and assigned adjacent bounded cells as separate touching holes.
- Face-attached sketches projected display-mesh chords instead of native arcs.
  Intersecting those approximations with the original circles created slivers.
- Shared profile boundaries require opposite outer/hole winding to cancel.
  Keeping both copies left internal walls in a sectional reconstruction.

Region filling now uses the region's assigned holes. The arrangement removes
graph bridges from face boundaries, assigns each disconnected component's
exterior loop as one hole, and preserves the original bounded-face ordering.
Supported planar B-Rep outlines preserve lines and circular arcs in both GUI
capture and evaluator re-derivation. Unsupported curves retain the prior
projection path.

Parallel cuts of one source sketch prism now arrange the body and tool
footprints in a common frame. Each depth interval sweeps the selected material
cells, and only exposed caps and walls are sewn into the output. This covers
blind and through cuts, either entry face, rotated/reflected in-plane frames,
existing holes, and cuts crossing outside edges. Candidate solids must pass
watertightness, health, and strict topology checks before replacing the body.
Dimensions are not expanded or nudged. Curves outside the supported line/arc
family and bodies without a retained single-prism source use the existing
guarded boolean paths; this is not a universal boolean-parity claim.

Sectional results retain existing face names and recover generated cut-wall
names only from unambiguous analytic supports. Repeated supports do not select
an arbitrary owner. The existing cut-wall reattachment regression checks that
an upstream width edit preserves the captured pocket wall.

Regression evidence is in `zerocad-core/tests/overlapping_face.rs`, the GUI
`geom2d` tests, and OpenRCAD's arrangement tests. It checks fill versus picking,
duplicate coverage, stable region order, the original outer extrusion,
2 mm/6.63 mm/through recesses, retained/removed material probes, strict solid
validation, approximate display volume against analytic profile area, and
cold/warm/save/reopen behavior. Display-volume comparisons allow the existing
arc chord approximation; they are not exact mass-property qualification.

Derived kernel and mesh cache ABIs advance to 6. Authoritative file schemas and
feature recipes are unchanged. All invalid/unhandled candidates retain the
previous model rather than presenting a partial operation as success.
