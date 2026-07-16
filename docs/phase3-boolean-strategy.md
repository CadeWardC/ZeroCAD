# Phase 3 boolean strategy

## Official engine

Phase 3 standardizes on OpenRCAD's split/imprint boolean engine. Every boolean
uses the same policy-aware transaction:

1. validate both operands;
2. intersect candidate face pairs;
3. split inherited edges and attach face-local pcurves to new boundaries;
4. queue all imprints for each face and partition the complete split graph;
5. classify every resulting face region with deterministic multi-direction
   parity, including coincident and same-domain cases;
6. sew, heal T-junctions, normalize safe same-domain faces, and validate;
7. return structured diagnostics, recovery, and complete topology history.

Analytic intersection and same-domain paths are optimizations inside this
transaction. They are not alternate boolean engines and must satisfy the same
strict output contract.

## Deferred general face-arrangement replacement

The master plan's fully general face-local 2D arrangement pipeline is not a
second Phase 3 implementation. The current imprint partitioner is the official
engine until its replacement trigger is met. A replacement becomes mandatory
when a minimized, valid operand pair within the supported surface/curve set
cannot be represented by the complete per-face split graph without a
case-specific topology reconstruction, or when the frozen boolean corpus finds
an N-way intersection, overlapping trim, periodic-seam crossing, or nested-loop
case that the split/imprint engine cannot classify and return strictly.

Meeting that trigger requires a design that constructs one UV arrangement per
face from all intersection and boundary curves, extracts bounded regions, and
feeds those regions into the existing classification, sewing, history, and
validation stages. A failure must remain an explicit operation error until that
replacement lands; ZeroCAD may not add a geometry-specific fallback.

## Change rules

- New boolean cases are added to the OpenRCAD robustness matrix before a fix.
- No successful result may rely on debug-only validation.
- New boundaries carry pcurves before face construction completes.
- Classification changes must preserve deterministic topology counts across
  repeated runs and orientations.
- ZeroCAD consumes only canonical structured boolean results.

