# Phase 7 stabilization and release record

Phase 7's engineering contracts are implemented. Part Design 1.0 is **not yet
declared released**: signed-package, public-alpha, and reference-hardware facts
remain evidence gates and cannot be replaced by local unit tests.

## Implemented stabilization infrastructure

- The GUI keeps a revision-aware compact recovery snapshot, writes debounced
  autosaves off the interface thread through the canonical atomic `.zcad`
  writer, and captures the latest main-thread document plus a backtrace when an
  application-boundary panic occurs. A save completion clears recovery only if
  it corresponds to the current document revision.
- File → Recover Autosave is enabled when a validated local recovery exists. A
  startup status message points to it instead of silently leaving the file on
  disk.
- File → Export Bug Report creates one deterministic, uncompressed ZIP containing
  the current authoritative `.zcad`, session log, latest panic text, the last
  evaluator error plus the structured unresolved-feature map, renderer/adapter
  identity, platform, application version, and git build hash. It is fully
  offline and runs only after an explicit user action.
- File → About ZeroCAD displays the same version, build, platform, and renderer
  identity embedded in reports.
- The five retained 1.0 capability boundaries are executable data in
  `zerocad_core::PHASE7_COMPATIBILITY_EXCEPTIONS`. The behavioral gate rejects
  duplicate IDs or an empty owner, reason, user impact, or removal trigger.
  They are structured atomic failures, not production compatibility allowlists.
- Seeded scale, boolean, persistence, semantic reorder/suppression, and GUI
  undo/redo stress loops run sequentially. Repository-level Cargo and test
  scheduling is capped at two jobs/threads to bound machine load. Set
  `ZEROCAD_LONG_STRESS_ITERATIONS` up to 1000 for longer release/CI sampling;
  the GUI test keeps a fixed eight-entry undo stack and varies repeated
  undo/redo cycles so longer runs do not grow memory without bound.
- CI runs the core suite on Linux and macOS, the complete application on Windows,
  and the OpenRCAD workspace on Windows, Linux, and macOS. Static-analysis command
  failures are no longer advisory. Retained advisory lint families and their
  stabilization rationale are recorded in
  [`phase7-warning-review.md`](phase7-warning-review.md).

## OCCT differential oracle

`tools/phase7_occt_oracle.py` uses the external OCP bindings for OCCT 7.8.1.1 to
generate the committed `phase7-oracle-v1.json`. The nine cases cover primitives,
fuse/cut/common, the externally authored Mayo/FreeCAD STEP cube, uniform-scale
direct editing, mass properties, centroids, and disjoint/tangent failure
classification. The generator records the exact OCCT version and the STEP
fixture SHA-256.

The Rust gate rebuilds the same cases using OpenRCAD and compares classification,
volume, area, and centroid. OCCT/OCP remains test-oracle-only: it is not a Cargo
dependency, is never linked, and is not shipped.

## Release evidence contract

`zerocad_core::validate_phase7_release_evidence` is the authoritative executable
check for the absolute Part Design 1.0 limits and external evidence. Copy
`benchmarks/phase7-release-evidence.template.json` to
`target/phase7-release-evidence.json`, replace every placeholder with measured
evidence, then run:

```text
cargo run --release -p zerocad-core --example release_evidence_check -- target/phase7-release-evidence.json
```

The command reports every failure in one run. It requires a signed artifact, an
installed-first-launch seven-sample profile, absolute size/startup/memory/edit/
viewport/document limits, both workspaces, the long stress and OCCT gates, the
frozen Phase 0 comparison, warning review, public-alpha sessions and foreign
models, a frozen regression for every accepted alpha defect, and zero unresolved
release blockers.

## Open release evidence

The previous freshly linked unsigned first launch exceeded one second. No signed
installed artifact has yet supplied the required first-launch median and p95.
The repository also contains no genuine public-alpha session count or curated
customer-model result, and no new one-million-triangle reference-hardware frame
profile. These remain visible release blockers. Filling a template with invented
or zero-duration values does not pass the validator.

The signed installed startup measurement is collected without rebuilding or
replacing the artifact:

```text
powershell -File benchmarks/profile-startup.ps1 -SkipBuild -ExecutablePath <installed-zerocad-gui.exe> -SignedPackage -InstalledArtifact -Phase7ReleaseGate
```

The implementation may be used for public alpha now. Part Design 1.0 may be
declared only after the evidence validator and all handoff commands exit zero.

## Migration notes

Phase 7 does not increment the `.zcad` container or feature-payload schemas.
Current v5 documents remain byte-contract compatible and recovery documents are
ordinary compact v5 files. Exact-version rejection of pre-v5 prototypes remains
intentional: keep the original file, open it with the build that created it, and
export a supported STEP or resave through an explicitly documented migrator.
There is no silent JSON/graph fallback. New recovery files, session logs, and bug
bundles live outside the project and never alter its semantic recipe.
