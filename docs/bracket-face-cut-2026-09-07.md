# Face-attached bracket cuts

The frozen `zerocad-core/tests/fixtures/bracket-face-cut.zcad` reproduces a
circle cut from an inner support face across a gap into a sloping brace. Its
second cut selected the surrounding profile; the user confirmed both cuts
should select the circle. The original fixture remains unchanged.

Corrections:

- Merge projected face outlines through `SketchCurves::extend_face_boundary`
  in both GUI and core. A complete sampled rim coincident with a drawn circle
  no longer creates sliver regions or redirects the circle selection.
- Keep cut start/end planes at their specified positions, preserving supplied
  plane handedness. The former 0.1 mm axial expansion scarred the support
  behind an outward cut and increased blind-pocket depth.
- Prefer the requested cut direction over the larger bounding-box overlap.
  The existing reverse-direction recovery remains available.
- Preserve captured face centroids across changes within the f32 capture
  envelope, avoiding microscopic sketch movement after re-tessellation.
- Orient planar circular imprint disks in the surface's parameter frame.
  Raw loops must agree with the surface; the face orientation is applied
  separately. An inverted retained disk corrupted display volume and subsequent
  cuts despite a successful first boolean.

Disposable geometry/mesh cache ABIs are now 5; authoritative schemas are
unchanged. Reopen rebuilds old cached geometry.

Regression evidence includes the supplied bracket with each intended cut,
cold/warm evaluation, save/reopen, point membership, analytic expected volume,
scaled projected outlines, and a rotated/translated U-shaped kernel fixture.
The independent OCCT check of the two-hole STEP gives a valid solid with volume
14456.738691 mm³; fine native tessellation is within 0.6 mm³ of the analytic
expectation. This is bounded evidence for these workflows, not a claim of
general boolean completeness. Resource/performance impact is not benchmarked.
