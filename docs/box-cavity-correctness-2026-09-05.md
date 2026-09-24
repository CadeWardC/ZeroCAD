# Box-cavity correctness

Status: implemented and locally verified. This is the first code
delivery under the [competitive development plan](competitive-development-plan.md).

## Reproduced problem

The closed 60 × 40 × 20 mm enclosure with 2 mm walls should contain
`60*40*20 - 56*36*16 = 15,744 mm³` of material, centered at (30, 20, 10) mm.
A new assertion against strict native tessellation reproduced the existing defect:
37,248 mm³ and centroid approximately (25.6701, 17.1134, 12.8866) mm.

The four box side-wall loops ran against their supporting surface normals.
Sewing can reconcile a plane's normal with its winding, but the inner offset
surfaces retain their intrinsic base parameter sense. That disagreement left
mixed effective face senses. Sewing also normalizes each closed shell outward,
whereas an enclosed void needs inward orientation relative to the material.

## Change

- Correct the four side-wall boundary directions before sewing. Preserve the
  explicit offset/base relationship used to verify wall thickness, the boundary
  geometry, and the surface representation.
- Reverse the sewn closed inner shell before assembling the material solid.
- Advance disposable kernel and mesh cache ABIs from 3 to 4 so old generated
  geometry is rebuilt. The authoritative recipe, feature schemas, IDs, and
  `.zcad` container version are unchanged.

The fix is in `OpenRCAD/crates/openrcad-algo/src/offset.rs`; it does not add a
runtime dependency, general mesh-repair pass, or new persisted field. It also
corrects side-wall winding in open box shells, so existing open-shell and
wall-thickness regressions are part of validation. Broader surface shelling and
native cavity STEP import are not added by this change.

## Regression evidence

- Kernel: two box sizes, original and rotated/translated placements; expected
  material volume, surface area, centroid, exterior/void triangle directions,
  and opposite traversal of shared mesh edges.
- Application: strict native volume and centroid, inspection, cold/warm display
  evaluation, compact save/reopen, and STEP export.
- Persistence: a valid container marked with pre-fix cache ABIs discards both
  disposable accelerators, retains its recipe, and rebuilds correct geometry.
- Frozen fillet, concave-shell, torus-band-shell, and loft fixtures remain
  unchanged and hash-checked. Their recipe comparisons now load and write the
  frozen document through the current canonical writer, accommodating the
  deliberate cache-metadata version change without replacing historical inputs.

These focused regressions pass. The original volume assertion was observed
failing before the implementation change and passing afterward.

## Verification record

- Application: **934 tests passed**, zero ignored.
- OpenRCAD workspace: **576 tests passed**, zero ignored.
- Formatting in both workspaces, application `cargo check`, and core all-target
  Clippy passed. The two-workspace Clippy baseline gate passed with no new
  warnings; the reviewed backlog remains 452 warning occurrences.
- Independent OCCT 7.8.1.1 validation: **11 STEP exports passed**. The cavity
  is a valid one-solid shape with volume 15,744.000000000146 mm³ and center
  (30, 20, 10) mm to floating-point precision.
- Whitespace check passed. Frozen modeling fixture files and their hashes were
  preserved; compatibility comparisons use the current canonical writer.

The [verification summary and selected source hashes](evidence/cavity-2026-09-05/verification.json)
and [independent STEP report](evidence/cavity-2026-09-05/step-validation.json)
are retained. Full local logs are in `target/competitive-checks/`.
No end-to-end latency, memory, package-size, interactive UI, or hosted-CI claim
is made by this correctness delivery. This is an uncommitted working-tree
delivery, not a released artifact or completion of the broader product plan.

## Focused reproduction

```text
cargo test -p zerocad-core --test mechanical_workflows closed_cavity
cargo test -p zerocad-core --lib pre_cavity_fix_cache
cargo test --manifest-path OpenRCAD/Cargo.toml -p openrcad-algo --test closed_box_shell --target-dir target
```
