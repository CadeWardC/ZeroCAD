# ZeroCAD and OpenRCAD project audit — 2026-09-04

Audited checkout: `9b1290f1` (initial working tree clean). This is a source and verification audit, not a claim of production readiness or an interactive usability test. Findings below distinguish observed implementation gaps from work that needs an experimental reproducer. No geometry or application behavior was changed.

## Assessment

ZeroCAD has a substantial modeling foundation: a separate geometry kernel, strict operation-result validation, a semantic document format, revision-aware evaluation, transactional assembly edits, recovery, and extensive regression suites. Preserve these investments.

The next milestone should make ordinary mechanical-part workflows dependable from creation through upstream edits and exchange. The highest return is closing correctness-contract gaps and improving the evidence behind reliability claims, followed by STEP export and targeted blend expansion. A wholesale kernel rewrite or another broad feature wave would be premature.

## Findings in priority order

### 1. High: Boolean intersection failures can be interpreted as no intersection

**Observed:** `OpenRCAD/crates/openrcad-algo/src/boolean.rs:920` calls `surface_surface_curves`. In `intersect.rs:2376`, that wrapper invokes the budgeted intersection implementation and uses `unwrap_or_default()`. An exhausted intersection budget therefore becomes an empty curve list. The analogous public surface/surface wrapper at `intersect.rs:1706` also discards errors. The checked Boolean entry point reaches this code through `boolean_impl`; its internal error type currently represents cancellation only.

**Impact:** the Boolean can continue without required split curves. Later validation may reject the result, but watertight topology does not establish that the requested material operation was computed correctly. This audit establishes the error-loss path; it does not claim to have reproduced a specific incorrect successful solid.

**Next:** propagate a typed intersection failure through the Boolean transaction. Share an operation-level work budget and thread cancellation through expensive intersection work. Keep “empty intersection,” “unsupported,” “exhausted,” and “cancelled” distinct.

**Acceptance:** an injected exhausted budget returns a typed failure and preserves the previous body; disjoint geometry remains a legitimate empty-intersection case; no authoritative Boolean path calls an error-discarding intersection wrapper.

### 2. High: the promoted Boolean corpus overstates operand diversity

**Observed:** `zerocad-core/src/boolean_case.rs:105` permits only primitive operands plus rigid transforms. Its replay builds those primitives. Nevertheless, `tests/boolean_cases/imported_round_union.json` is labeled `imported_brep` and constructs a cylinder and sphere, while `history_through_cut.json` is labeled `history_extrusion` and constructs a box and cylinder. The stage enum in `tests/boolean_replay.rs` checks ordering; it does not change operand construction.

The promoted replay list has four cases. The safety fuzz target accepts structured rejections and verifies identity; the resolvable target covers overlapping equal-sized box unions. These lanes have value, but they do not exercise the imported/history-bearing operand classes their labels imply. Other regression suites and two external STEP fixtures exist; this finding is specifically about the canonical fuzz/replay lane.

**Next:** introduce a versioned test-case format with primitive, feature-recipe, and content-addressed imported-asset operands. Preserve V1 identities. Generate actual extrusion/Boolean/blend histories and actual imported B-Reps. Add volume, area, centroid, component count, and point-membership expectations to success cases.

**Acceptance:** a case labeled imported demonstrably reads its frozen asset; a history case reconstructs its feature sequence; identity binds every operand asset/recipe; a healthy but semantically wrong result fails the oracle.

### 3. High-value product gap: no STEP export in the application

**Observed:** `zerocad-gui/src/app/ui/top_bar_file_menu.rs:108` exposes STL, 3MF, and OBJ exports. `app/io.rs:1966` onward implements these mesh export paths. OpenRCAD already has a STEP writer, exercised by `zerocad-core/tests/step_interchange_gate.rs`. The file menu's import/export section is also restricted to Part projects.

**Impact:** users can bring precise CAD geometry in but cannot send their modeled B-Reps back out through the application. Mesh exchange does not substitute for editable CAD surfaces and topology.

**Next:** expose a revision-bound, validated B-Rep export service and a STEP export action. Start with parts and multiple bodies, with explicit units and names; define separate scope for assembly hierarchy, placements, and metadata. Reject unsupported mesh-only bodies explicitly. Use atomic file replacement and a background job.

**Acceptance:** exported modeled parts reopen in an independent CAD reader with matching units, dimensions, body count, and mass properties. Cover cuts, fillets, shells, a hole-bearing loft, and mixed analytic surfaces. Test pending/failed evaluation and output-write failure.

### 4. High: finish the known edge-identity ambiguity before broader editing claims

**Observed:** `zerocad-core/src/parametric/tests/reattachment_matrix.rs:546` explicitly ignores the half-space discriminator test. Its documented case has multiple edges with the same adjacent face-owner pair and falls back to geometry. `tests/exact_geometry_boundaries.rs` exempts the entire reattachment test file from its ignored-test prohibition.

**Next:** reproduce a real ambiguous edge selection, then implement a stable discriminator based on proven lineage and side/parameter information. Preserve the current safe unresolved behavior when identity cannot be established. Narrow the ignored-test exception to the exact known test until it is removed.

**Acceptance:** a downstream fillet and attached sketch retain the intended side through large dimension edits, rotations, save/reload, and cold/warm rebuilds; deleting the selected topology suspends its consumer instead of retargeting it.

### 5. Medium: performance evidence needs richer workloads and a current baseline

**Observed:** `zerocad-core/benches/support/benchmark_corpus.rs:74` constructs the 100/500-feature histories from one box followed by repeated body translations. These measure scheduling, transforms, and cache behavior well; they are weak proxies for a 500-feature mechanical design. Criterion also includes a drilled part, import, and kernel hotspots, so performance coverage is not absent.

Qualified historical measurements exist in `benchmarks/modeling-wave5-comparison.md`; the roadmap's earlier “no committed reference baseline” statement is stale relative to later files. Those measurements predate the current text/embossing and assembly integration state.

**Next:** retain the move-chain benchmarks and add frozen histories for a patterned mounting bracket, filleted/shelled enclosure, text-heavy engraving, imported curved part, constrained sketch, and assembly. Measure early and trailing edits, cold rebuild, cancellation latency, peak memory, document open, and viewport upload/frame cost.

**Acceptance:** current-commit reference runs record p50/p95 and memory on named hardware, with workload-specific budgets and semantic equivalence checks. Keep qualified measurements separate from shared-runner smoke tests. Do not extrapolate move-chain results to arbitrary feature histories.

### 6. Medium: close the fuzzing and release automation loop

**Observed:** `.github/workflows/boolean-fuzz.yml` runs two five-minute fuzz targets weekly, but has no artifact upload or persistent corpus step and no live OCP differential step. `tools/phase7_occt_oracle.py` generates nine fixed cases; it does not consume the canonical `BooleanCaseV1` input described in the newer roadmap. Static replay of that oracle is useful but is a different capability.

CI invokes Clippy without denying new warnings. Thus command execution is gated, while newly introduced warning-level lints can still pass. `.github/workflows/release.yml` builds draft release packages from tags without a dependency on tests/evidence for that exact tag. Draft publication limits immediate exposure, but it is not a verified release gate.

**Next:** always upload fuzz failures and logs, retain seed corpora, implement canonical-case OCP comparison outside the fuzz hot loop, and promote minimized findings into normal replay tests. Deny newly introduced warnings through explicit reviewed allowances or a baseline policy. Require validation of the exact release commit before promotion from draft.

**Acceptance:** an intentional fuzz failure leaves a downloadable reproducer; both engines consume the same case; a new warning fails the chosen policy; failing tests prevent release promotion. Define the active vNext evidence requirements rather than silently reinstating superseded 1.0 gates.

### 7. Medium architectural risk: application precision can limit kernel precision

**Observed:** `zerocad-core/src/geometry.rs:1` stores coordinates and sketch frames in `f32`; `parametric/types.rs:665` and subsequent features store many dimensions in `f32`. The kernel uses `f64`. At a coordinate magnitude of 1,000,000 mm, adjacent `f32` values are 0.0625 mm apart. Conversion to `f64` cannot recover already-lost detail.

**Next:** define a supported size/precision envelope and test small features far from the origin through the complete app-to-kernel path. Plan double-precision authoritative coordinates/local modeling frames, retaining float render buffers. Treat persisted field changes as an explicit schema/migration project; do not mechanically replace float types.

**Acceptance:** selected precision cases survive sketch placement, edits, save/reload, and STEP exchange within a documented tolerance. Existing document fixtures remain compatible under deliberate migration rules.

### 8. Medium kernel research risk: general intersections need completeness evidence

**Observed:** the generic surface intersection fallback in `intersect.rs:1724` substitutes ±100 for unbounded parameter domains. Its subdivision leaf target is derived from domain width (roughly 1/24 per axis), followed by local refinement and point chaining. Face trimming also starts unbounded curves at ±100 (`intersect.rs:2409` vicinity).

**Impact:** the existence of B-spline/NURBS representations does not establish general, complete intersection support. Small loops, tangencies, seam crossings, and far-domain intersections need targeted proof. This is a source-based risk, not a reproduced failing model in this audit.

**Next:** derive search domains from trimmed faces and operand bounds, test branch completeness and residuals, and reject unresolved cases explicitly. Prioritize paired analytic/NURBS fixtures from actual imports. Follow `docs/phase3-boolean-strategy.md`: replace the split/imprint engine with a face-local arrangement only when the documented reproducer-based trigger is met.

**Acceptance:** translated/scaled equivalents preserve intersection branches and Boolean semantics; closed intersection loops, tangencies, periodic seams, and tiny trims have regression cases with independent expected results.

## Architecture to preserve and improve gradually

- Preserve the core/GUI separation, the OpenRCAD façade, strict operation results, semantic document recipe, revision-bound writebacks, and transactional edit model.
- Distinguish complete topology-history coverage from proven lineage. `boolean.rs` explicitly fills unattributed results as generated; this is honest metadata but cannot alone guarantee durable reattachment.
- Extract feature-family evaluation from the large `parametric/eval.rs` behind the existing candidate-result contracts. Keep orchestration and commit authority centralized.
- Split `rolling_ball.rs` by support solving, trimming, corner networks, and validation only as dedicated refactors with full geometry regression coverage. Avoid mixing feature expansion with structural rewrites.
- Keep numerical geometry, display cleanup, and selection identity separate. A visually repaired mesh must never become proof that the underlying solid is correct.
- Expand blends based on frequency in the real-part corpus. Prioritize common partial selections, runouts, mixed radii, and curved support transitions before unrestricted high-valence/Gregory work. Existing unsupported cases should continue to reject atomically.

## Recommended next delivery sequence

| Milestone | Work | Exit condition |
|---|---|---|
| A — Trustworthy kernel results | Propagate intersection errors; make corpus stages real; archive fuzz failures; reproduce and close edge ambiguity | Forced failures roll back; geometry oracle detects wrong successes; representative edit identities remain stable |
| B — Complete mechanical-part workflow | Part STEP export; 20–30 frozen real parts; scripted create/edit/save/reopen/export scenarios; visible actionable failure explanations | Named workflows pass end to end and independent CAD readers accept output |
| C — Measured robustness expansion | Current realistic performance baseline; support-pair-driven blend fixes; general intersection research; precision migration design | Published capability matrix, measured budgets, and regression-linked support expansion |

The proposed 20–30-part corpus is a starting target, not an existing result. Include drilled brackets, pocketed plates, thin enclosures, shafts, patterned holes, text engraving, multi-loop lofts, curved imports, and small assemblies. For each, test upstream edits, suppression, undo/redo, save/reopen, cold/warm evaluation, and export. Classify success, safe unsupported failure, incorrect geometry, crash, and timeout separately.

After these milestones, choose subsequent features from observed workflow failures. Drawings, CAM, broad surface modeling, and more assembly mate types should compete against measured user needs rather than be assumed next priorities.

## Documentation corrections

The capability roadmap still describes several committed changes as working-tree-only. The kernel hardening checklist still lists disconnected Boolean splitting as open although multi-body operations exist. The main README describes panic-hook suppression while `quiet_panic` deliberately does not alter the global hook. Reconcile these records against code and tests and distinguish implemented, verified, and released status.

## Verification record

- Initial Git working tree: clean.
- ZeroCAD formatting: passed.
- OpenRCAD formatting: passed.
- `cargo check --locked`: passed.
- `cargo test --workspace --locked` (ZeroCAD): passed; 917 tests passed, one known edge-discriminator test ignored.
- `cargo clippy -p zerocad-core --all-targets --locked`: completed successfully with warnings (43 library warnings plus warnings in test targets). No source was changed to suppress them.
- `cargo clippy --workspace --all-targets --locked` (OpenRCAD): completed successfully with warnings.
- `cargo test --workspace --locked` (OpenRCAD): passed; 572 tests passed, none ignored. The nested workspace uses its own lockfile; this audit reused the root target directory for build artifacts.
- No interactive GUI session, external CAD round-trip, qualified performance capture, hosted CI inspection, or public-alpha evidence collection was performed. Release-readiness conclusions cannot be inferred from this source review alone.
