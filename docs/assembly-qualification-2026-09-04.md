# Assembly qualification — 2026-09-04

**Release qualification: failed / incomplete.** This work adds reproducible local
measurements, workflow regressions, and STEP assembly export. It does not certify
interactive performance or add assembly STEP import.

## Measurement scope

`zerocad-core/examples/assembly_measure.rs` performs one warmup and 31 measured
runs per case in the standard optimized release profile. The fixtures contain
four plates for dense-SVD calibration and 1,000 plates for sparse-PCG stress.
The sparse chain has 999 distance mates; the closed-loop graph has 1,997 mates.
The latter is a locally dense constraint graph, not an all-to-all graph.
Parts are real serialized and evaluated 10 × 8 × 2 mm solids, spaced 12 mm apart
along Z. Origin-based distance mates reproduce their spacing after reopening.

Every committed solve starts from a fresh snapshot with a 0.01 mm displacement
of the last plate along Z. Interactive calls start from the satisfied assembly
with a temporary 0.01 mm Y target on the last plate. Samples include full solver
calls, not just linear algebra. Convergence is recorded independently of time.
These are translation cases; broader angular calibration remains desirable.

The raw files are in [the evidence directory](evidence/assembly-2026-09-04/):

- `solver.json`: calibration residuals, iterations, and stress timings.
- `convergence.json`: committed and interactive convergence for every sample.
- `cpu-open.json`: in-memory decode, definition hydration, and CPU placement
  conversion timings, plus the existing V2 validator's findings.
- `environment.json`, `cpu.json`: source and executable fingerprints, toolchain,
  machine, and sample count. The checkout has uncommitted work, so the base commit
  alone is not a release identity.

The open measurement excludes disk access, mate solving, GPU upload, and actual
frame presentation. It therefore deliberately does **not** produce purported V1
hydrated-open/manipulation-frame release evidence. Full viewport qualification
is still outstanding.

## Findings and next work

Final local release run on an AMD Ryzen 7 7840HS (31 measured samples):

| Case | Committed solve median | Interactive call median | Interactive convergence |
| --- | ---: | ---: | ---: |
| 1,000-plate sparse chain | 74.37 ms | 15.19 ms | 0 / 31 |
| 1,000-plate closed loops | 103.06 ms | 25.15 ms | 0 / 31 |

CPU-only open/hydration median: **5.75 ms**, with the exclusions above.
These are local observations, not a certified reference-machine baseline.

All measured committed solves converged. Large interactive cases did not converge
within their deadline. The V2 validator rejected the four-plate translation
calibration: ten times the measured residual exceeds the currently committed
tolerance. Tolerances were not loosened to make the check pass.

The interactive solver checks its deadline around whole linearizations, while
`linearize` scans mate membership for each free degree of freedom and copies the
variable vector. Work within that phase can exceed the 4 ms deadline. Profiling
and reducing that work, then checking deadlines inside bounded chunks, is the
next performance task. A timing-only pass would still be insufficient when the
interactive solve does not converge.

## Workflow coverage

- `assembly_workflows.rs`: replacement with a changed plate dimension, identity
  and mate preservation, unresolved resolution context, suppression/restoration,
  transactional snapshots, rejected edits, and binary save/reopen.
- GUI assembly tests: actual undo/redo for replacement, visibility and mate
  suppression; missing face references survive save/reopen, report unresolved,
  and can be repaired with undo/redo of the repair.

This is a small plate-array corpus, not broad mechanical-assembly certification.
No new mate types or motion features were added.

## STEP export

The assembly file menu now exposes **Export STEP assembly…**. Export runs against
a captured snapshot on a background worker. It writes shared part definitions,
component names, root assembly relationships, millimetre units, and each
component's resolved translation and rotation. Hidden components/bodies are
excluded. Unsolved active mates and GUI-reported broken/conflicting mates are
rejected. A failed export preserves an existing destination.

Independent OCCT 7.8 verification checks valid solids, volume and component
centres. XCAF separately verifies one root assembly, three named occurrences and
one shared definition, including a translated, rotated occurrence. Reports:
`occt.json` and `occt-hierarchy.json` in the evidence directory.

Supported hierarchy matches the current document model: assembly → part
instances. Nested subassemblies, assembly import, colors/materials, and mate
semantics in STEP are not implemented. Existing exact-B-Rep export limitations
still apply; the exporter does not substitute a mesh.

## Reproduce

Validation completed: workspace `cargo test` (930 passed), final focused core
assembly tests (4 passed), GUI assembly tests (4 passed), and OpenRCAD exchange
tests (17 passed). Formatting and `cargo check` passed. Core all-target Clippy
and the two-workspace warning-baseline gate passed with no new warnings.
Logs are stored as `docs/evidence/assembly-checks-*.log`.

Run from the repository root:

```powershell
cargo run -p zerocad-core --release --example assembly_measure -- docs/evidence/assembly-2026-09-04
$env:ZEROCAD_INTERCHANGE_EVIDENCE_DIR = (Resolve-Path docs/evidence/assembly-2026-09-04).Path
cargo test -p zerocad-core --test assembly_workflows --test assembly_step
cargo test -p zerocad-gui app::ui::assembly::tests
python tools/verify_step_exports.py docs/evidence/assembly-2026-09-04 --output docs/evidence/assembly-2026-09-04/occt.json
python tools/verify_assembly_step.py docs/evidence/assembly-2026-09-04/assembly.step --output docs/evidence/assembly-2026-09-04/occt-hierarchy.json
```

The independent Python checks require the existing optional `cadquery-ocp`
verification environment. OCCT is not a shipped application dependency.
