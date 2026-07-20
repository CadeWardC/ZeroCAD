# Wave 4 Shell feasibility gate

This is the entry contract for curved analytic Shell. The executable source of
truth is `openrcad_algo::shell_feasibility::SHELL_FEASIBILITY_V1`; this document
explains the staged boundary frozen before the Wave 4 geometry work began.

## Current offset readiness

| Support | Required by | Current state |
|---|---|---|
| Plane | Baseline | Verified in the general planar Shell path. |
| Cylinder | 4A | Verified in mixed plane/cylinder networks; primitive fast paths remain reachable. |
| Cone | 4B | Verified for truncated cones and plane/cone/cylinder countersink networks. |
| Sphere | 4B | Verified for spherical domes joined to planar bodies. |
| Torus | 4C | Missing; requires a separate research review. |
| NURBS | Deferred | Explicitly unsupported in Wave 4. |

## Intersection readiness

The matrix is symmetric. “Kernel analytic” means an analytic intersection is
available but Shell does not yet own trimming, imprinting, pcurve construction,
or complete history for the result.

| Pair | Required by | Current state |
|---|---|---|
| Plane / Plane | Baseline | Verified in planar Shell. |
| Plane / Cylinder | 4A | Verified in Shell. |
| Cylinder / Cylinder | 4A | Kernel analytic only. |
| Plane / Cone | 4B | Verified in Shell. |
| Cylinder / Cone | 4B | Verified in Shell for the coaxial countersink transition. |
| Cone / Cone | 4B | Generic controlled fallback only. |
| Plane / Sphere | 4B | Exact circle construction and Shell ownership verified. |
| Cylinder or Cone / Sphere; Sphere / Sphere | 4B | Generic controlled fallback only. |
| Any supported analytic surface / Torus; Torus / Torus | 4C | Generic controlled fallback only. |
| Any pair containing NURBS | Deferred | Unsupported. |

Generic subdivision/refinement is research evidence, not completion evidence for
an analytic Shell milestone.

## Ownership and history

- A retained or trimmed outer face is owned by its source face and is Modified.
- Its inner offset face is Generated from that source face.
- A removed outer face is Deleted.
- An intersection edge is Generated from a canonical ordered support pair.
- A rim bridge is Generated from the removed-face boundary.

Generated coedges receive pcurves during construction. Unchanged outer coedges
preserve their pcurves. Bounded repair is allowed only for legacy/imported input,
and incomplete output is rejected before commit.

## Provisional self-intersection policy

The feasibility contract records face-bounds broad phase, exact intersection
for supported pairs, and wall-thickness/collapse checks. For the supported
convex/analytic 4A/4B fixtures, strict topology validation and watertightness
are fail-closed acceptance gates. They do not constitute a general proof that
every possible self-intersection is absent. Any ambiguous or unresolved case is
rejected atomically. Wave 5 owns explicit concave intersection, imprinting,
classification, and resewing; Wave 4 does not silently heal those cases.

Stages 4A and 4B are implemented and verified. Stage 4C remains stopped and
always requires an explicit go/no-go research review after the band-transition
prerequisites are green.

The verified 4A/4B boundary is deliberate: removed opening faces remain planar,
distinct cylinder/cylinder, cone/cone, and sphere-pair intersections are not
claimed, and torus/fillet-band shelling is not enabled. Scale sweeps cover
`1e-3..1e3` plus far-origin cone and sphere fixtures. The compound countersink
fixture is topology-verified across the scale sweep and display-verified at the
reference scale; extreme-scale cone/cylinder lens stitching and a microscopic
countersink translated to `1e9` remain recorded limitations rather than being
hidden behind a tolerance increase.

The conical collapse guard samples the supported face's boundary vertices. That
is sufficient for the current analytic solid trims, whose relevant radial
extrema occur at those vertices. A future degenerate or nonstandard trim with a
smaller interior radius must add an analytic interior-extrema query before it
can enter the supported boundary.

The current 4C review is mechanically blocked. Cone- and torus-banded recuts
must complete deterministically rather than merely return through the guarded
failure path; fillet overflow plus deterministic transition ownership are still
missing. Cylinder-seam imprinting is now verified by the mixed-network Shell
regression that crosses a rotated primitive-cylinder seam.
These prerequisites are tracked in `SHELL_FEASIBILITY_V1` during 4A and 4B so
they cannot first appear as surprises at the 4C review.
