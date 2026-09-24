# Split-boss junction fillet repair — 2026-09-13

The radius-3 fillet on the triangular prism top edge now terminates on the
radius-6 boss without the previous unsupported five-band junction diagnostic.
Three incident caps were patches of one cylinder, split by the Boolean seam and
prism top plane. Treating them as independent endpoint supports caused the P2
capability failure; the old operation rejected safely and retained its source.

The kernel now joins connected, consistently oriented cylindrical patches before
using the existing cylinder/cylinder intersection trim. Seam cancellation checks
both endpoints and curve midpoint; all patches must connect through shared
edges. Different cylinders, opposite orientations, point-only contact and inner
loops do not enter this path. Existing sewing, pcurve and candidate acceptance
checks remain. General corners involving five distinct surfaces are not added.
Derived native and display cache ABI values are 9, so saved cached geometry is
rebuilt after this change; the document schema is unchanged.

## Independent geometry checks

The starting union has volume 12000 + 720π mm³. Native planar and cylindrical
surface boundary integration checks this to 0.02 mm³ before measuring the result.
For the selected edge, inward distance q runs from 0 to 3. Removed height is
3 − sqrt(9 − (q − 3)²); the span from the oblique end plane to the boss is
25 − sqrt(36 − q²) − 4q/3. Their product integrates to 35.09667 mm³.
The regression requires native removed volume within 0.02 mm³ of this independent
cross-section oracle. Face-interior samples projected to their native supports
must stay outside the retained boss interior. Strict tessellation succeeds and
the actual saved document recipe reloads and evaluates without warnings.

Display-mesh signed volume was not a reliable oracle: chord/angle settings
0.002/0.03 and 0.0002/0.01 produced removed-volume estimates of 35.7811 and
15.3389 mm³ respectively. The latter failed after 292.8 seconds. This remaining
tessellation/measurement discrepancy is explicitly not certified by the native
geometry repair. Projecting mesh samples to their supports prevents cylinder
chord sag from being mistaken for native material removal; this is not a claim
that those display meshes have accurate mass properties at every resolution.

## Verification

- Broad core/GUI run: 952 tests passed in completed targets; the stopped target passed all 6 tests on final rerun.
- Final prism/boss regression target: 6 passed, including save/reload and material checks.
- OpenRCAD algorithm crate: 306 unit, integration and documentation tests passed.
- Edge-blend break tests: 29 passed; feature torture: 27 passed.
- Formatting and cargo check passed.
- Both-workspace Clippy baseline gate passed, unchanged 450-warning baseline.

The broad run excluded `break-test`. Its stale experimental prism/boss binary
was stopped after the final focused regression passed; that target was then
rebuilt and rerun under the broad run's feature selection. This is a composite
verification, not an uninterrupted all-workspace test pass.

Logs are under `target/junction-*.log`; source hashes are in
`break-test/review/fillet-junction-fingerprints-2026-09-13.json`.
The full break-test workspace is not requalified: the prior 100-slot workload
exceeded its ten-minute run budget. No general performance or GUI verification
claim follows from these checks.
