# Cap join after nearly tangent cuts

The supplied `join_failed.zcad` reproduces an unresolved `extrude_45` after
rectangular and circular cuts. The saved depth is 3 mm; the reported preview
shows 7.776 mm. Both selected regions form the outward cap extrusion.

Cap replacement correctly removes the shared interface, but its independently
built walls require matching boundary subdivisions. OpenRCAD's T-junction
healer scanned every edge and vertex, including already shared edges below
the cap. It imprinted a nearby circular-cut vertex onto an unrelated closed
edge near the rectangular notch. That joined separate face fans into a pinch
point; the candidate failed pcurve/Euler validation. The subsequent general
Boolean attempts also failed, so ZeroCAD correctly rolled the join back.

The healer now identifies edges with one active face use and their boundary
vertices. Only those edges and vertices participate in imprinting, and split
children remain eligible for further boundary subdivision. Traversal stays
in arena order. Existing closed topology remains unchanged; strict pcurve,
health, watertightness, and connectedness checks remain enabled. No global
tolerance is increased and no model-specific identifiers enter the repair.

Regression coverage:

- The supplied document at 3 mm and 7.776 mm, cold/warm and save/reopen.
- Joined volume and material/void samples compared with separately evaluated
  original body and extrusion, retaining all four cuts.
- A synthetic prism with a narrow notch near a closed edge and a mismatched
  boundary subdivision elsewhere, plus existing sewing regressions.

The disposable B-Rep and mesh cache ABIs advance to 15, so older geometry is
rebuilt. The authoritative document format and user's original save are unchanged.

Validation on 2026-09-26: the saved-model regression passed at both depths,
including cold/warm and save/reopen checks. All nine sewing tests and four
merge tests passed. Formatting checks for both workspaces, `cargo check`,
core all-target Clippy, and the two-workspace Clippy baseline gate passed
(450 reviewed existing warnings; no new warnings). The optimized Windows
build passed and was copied unchanged to
`C:\Users\cadel\Downloads\ZeroCAD-Join-Fix.exe`.

The complete `cargo test -p zerocad-core -p zerocad-gui` run passed: 983
tests across 65 harnesses, including all 545 core unit tests and the saved
join/cut regressions. Logs are in `target/join-failed-tests.log`.
An overlapping unfiltered `cargo test` invocation hit Windows linker error
LNK1104 while the core test executable was running. That invocation is not
a full-workspace runtime pass; the separate break-test stress suite is not
included in the 983-test result.
After the active tests finished, `cargo test --no-run` succeeded for the full
workspace, including break-test. This resolved the executable-lock build
failure; it does not claim an additional full stress-suite runtime pass.
