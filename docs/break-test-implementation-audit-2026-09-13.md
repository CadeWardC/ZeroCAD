# Break-test implementation audit — 2026-09-13

## Implementation follow-through

The sections below this update record the **pre-repair audit**, including its
original failures and line references. The subsequent repair pass changed:

| Finding | Repair and supported scope |
|---|---|
| 1 — P1, wrong-face threading | Named picks must resolve to a cylindrical face. Its sampled support selects a unique connected native wall group, and replacement is restricted to that group's face indices. Deleted names and planar caps cannot fall back to a nearby wall. Legacy picks enumerate all wall groups and reject ambiguity. Positive named-wall and negative cap/deleted-name regressions are active. |
| 2 — P1, underestimated bounds | Gregory and bounded Offset/Ruled surface domains use interval enclosures. Missing/unbounded domains return explicit unknown bounds. Broad-phase callers cannot reject overlap using unknown bounds; finite through-all/split tools reject unresolved sizing. Conversion to display precision rounds outward. Active regressions cover the Gregory interior, unknown offset trims, and rounding. |
| 3 — P2, bore oracle | Corrected removed volume to the union of both discs and added a monotonic-removal assertion. The kernel test still accepts an explicit unsupported-operation failure; it does not certify general overlapping-bore support. |
| 4 — P1, body-pattern exhaustion/partial output | Pre-allocation limits are 4,096 instances and 262,144 face instances; staged display buffers also have limits. Per-instance cancellation and transform failure roll back the entire pattern. Huge linear/circular counts have typed-diagnostic and source-preservation tests. These limits apply to body patterns. |
| 5 — P2, invalid containment claim | Candidate surface samples are checked against the source's enclosing box. Unknown source bounds reject the candidate. This is explicitly a rejection guard, not a mathematical material-containment proof. Full material-containment certification remains future kernel work. |
| 6 — P2, weak test contracts | Missing mass properties now fail tests; determinism tests rebuild a document without evaluation caches. Orphan extrusion and selected invalid-parameter cases assert feature state and/or typed diagnostics. Broader thread measurement and independent CAD qualification remain open. |

Two additional failing core regressions were repaired. Deleting an isolated
capped cylindrical boss now removes its connected wall and cap and heals the
matching support-face ring, then uses strict sewing and normal transaction
validation. A polygonal blind pocket entirely inside the solid core of an
analytic threaded shaft now preserves the thread walls and constructs the exact
pocket boundary. The latter regression requires 48 mm³ removal for a 4×4×3 mm
pocket, both at the original position and after translation, and compares a cold
rebuild. Hollow/interrupted shafts, pockets intersecting the thread walls, and
general freeform cases are outside this specialized path.

The E16 wrong-success path was also repaired: a solver failure no longer retries
the opposite cut direction and reports a successful air cut. Both overlapping
hole and cut-extrude regressions now require an attributed warning and preserved
source material. General overlapping-circle subtraction remains unsupported.
Persisted geometry and mesh cache ABI values were incremented so old cached
outputs cannot hide these repairs.

Strengthening the coaxial-tube test exposed a further P1 residual: the input
face-history mapper only understood planes, so boring a cylinder discarded its
outer wall's durable name. The mapper now identifies cylindrical owners by their
complete radial/axial support before passing names through the kernel's exact
history. The tube regression requires an actual helical outer wall, no warnings,
and a retained radius-6 cylindrical bore; the old volume-only check could accept
a cosmetic thread. This test now passes.

The broader integration run found two more repairable paths. A second rectangular
pocket touching the body's edge now uses a rectangular cutter extension only in
space proven outside the source's bounds. The regression checks exact remaining
volumes for four successive-cut scenarios, rather than warning text alone.
Single-component face resolution no longer requires a finite spatial box once
the durable face has been found in that body's mesh. This restores the retained
face datum after shelling a filleted body with unknown curved bounds. The shell
fixture was regenerated after verifying that its previous version still loads
and evaluates; its manifest checksum was updated, and save/reload passes.

### Boss-junction fillet follow-through

The remaining P2 junction rejection was caused by counting three patches of the
same cylindrical boss as separate endpoint supports. The repair merges only
seam-connected patches on the same, consistently oriented cylinder and reuses
the native cylinder-intersection trim. Shared edges must match both endpoints
and their curve midpoint. Point-only contact, different supports, reversed
patches, and inner loops are rejected. Sewing and transaction guards remain.
This is support for a split cylindrical wall, not general five-surface corners.

The regression now checks native-surface volume against an independent
cross-section integral (35.09667 mm³ removed, tolerance 0.02 mm³), verifies the
starting solid against its analytic volume, samples retained boss surfaces, and
saves and reloads the recipe. Derived geometry/mesh cache ABI values are now 9.
Final evidence and the display-mesh measurement limitation are recorded in
[the junction repair report](fillet-boss-junction-2026-09-13.md).

General overlapping-circle cuts, full material-containment certification,
independent thread measurement qualification, and the expensive-workload
performance work also remain outside the supported repairs above.

### Repair validation

| Check | Result and evidence |
|---|---|
| Core unit tests | 544 passed, zero failures (`target/audit-fix-core-final3.log`) |
| Audit regressions | 6 passed; same log |
| Second-cut scenarios | All four volume oracles pass; 6 GUI reproduction tests pass in the same log |
| Shell/datum save/reload | Passed; old fixture rebuild also passed before regeneration (`target/audit-fix-shell-fixture-update.log`, `target/audit-fix-shell-datum-final3.log`) |
| GUI unit tests | 166 passed (`target/audit-fix-gui-final.log`) |
| Kernel geometry/topology/algorithm tests | 393 passed (`target/audit-fix-kernel-final.log`); final topology rerun: 47 passed |
| Selected break-test suites | 158 passed across parameter, extrusion-cardinality, thread, sweep/hole, body-boolean, blend, edge-case, and feature controls (`target/audit-fix-targets-final.log`, `target/audit-fix-controls-final.log`) |
| Core integration suite | **247 passed, 1 failed** in the final rerun (`target/audit-fix-integrations-final2.log`); the sole failure is the five-way fillet above |
| Formatting and compilation | Passed (`target/audit-fix-fmt-final4.log`, `target/audit-fix-check-final4.log`); kernel formatting also passed |
| Clippy release gate | Passed both workspaces, no warnings beyond the 450-occurrence reviewed baseline (`target/audit-fix-clippy-final4.log`) |
| Full workspace test command | Incomplete: stopped after the complex-extrusions target exceeded 10 minutes with `hundred_region_vent_plate` unfinished; its other 23 cases had passed. This is a timeout, not a pass (`target/audit-fix-workspace-timeout.log`) |

Source fingerprints are in
`break-test/review/repair-fingerprints-2026-09-13.json`. No interactive GUI or
independent CAD-oracle qualification is claimed. Existing unrelated working-tree
changes were retained.

## Verdict and scope

**The fixes are partly correct, but the implementation is not ready to be signed off.** The six original crash paths have targeted guards, the negative-offset algebra is corrected, thickening now retains its source, and multiple-sketch extrusion is rejected. However, the claims that selection safety and conservative bounds are complete are false. Two additional review probes fail, including a silent wrong-face operation. The future overlapping-bore success oracle is also mathematically wrong.

Reviewed the break-test implementation claims and their production paths in the current uncommitted tree at HEAD `9b1290f14f25daa188fb6e8bcd62a5664fcdd9a0`. This is not a review of every unrelated working-tree change. No production fixes were made. The prior [implementation plan](break-test-review-and-implementation-plan.md) supplies the original A/E/R identifiers; this audit supersedes its completion claims where stated below.

Severity: **P1/high** means prioritize before trusting the affected modeling path (silent wrong geometry, unsafe rejection tests, or resource exhaustion). **P2/medium** means a significant correctness/coverage limitation or incorrect regression oracle. No P0 catastrophe was established. Severity describes impact, not implementation effort or the competence of its author.

## Findings requiring repair

### 1. P1 — Thread selection still silently targets the wrong face

`zerocad-core/src/parametric/eval.rs:6962–7041`; helper `mock_kernel/geom_utils.rs:164–249`.

The durable resolver's answer is reduced to a component index. The actual selected face and its surface type are discarded, then `cylinder_face_near` chooses another face by distance. That helper returns only one cylinder per component, so the new ambiguity test cannot detect multiple plausible walls in the same solid. Failed named resolution can also enter the legacy fallback.

**Reproduced:** capture the named planar end cap of a radius-6, height-10 cylinder, then request an external thread. Evaluation emits **no warnings**, and the mesh vertex buffer changes from 13,824 to 101,484 entries. The cap is silently redirected to the cylindrical wall. The new far-away-reference test does not exercise this case.

**Repair:** carry the resolved face identity through to wall replacement; reject named noncylindrical/deleted faces. Only use legacy recovery for genuinely legacy references, enumerate individual trimmed cylindrical faces, and require a unique match. Preserve support-face trims rather than measuring a whole component's matching-radius vertices. Test planar caps, deleted names, coaxial inner/outer walls, equal-radius walls with distinct axial spans, and upstream edits. Require no warning and the intended face change on supported picks; require unchanged source geometry and attributed diagnostics otherwise.

### 2. P1 — The new conservative bounding box does not enclose every supported surface

`OpenRCAD/crates/openrcad-topo/src/solid.rs:198–336` and `zerocad-core/src/mock_kernel/geom_utils.rs:15`.

The function skips surface interiors for Offset, Ruled, and Gregory variants. A boundary box cannot generally enclose a Gregory or offset patch's interior. This box is now used to reject overlap and size through-all/splitting tools, where an underestimate changes behavior.

**Reproduced:** a Gregory patch with planar square boundary has center z=**5.625**, but this function returns z bounds **[0, 0]**. The probe uses a single-face shell to isolate the bounding contract, like the existing cylinder-wall unit test; it does not establish an end-to-end closed-solid boolean failure. The incorrect enclosure guarantee itself is directly demonstrated.

Additionally, nearest f64-to-f32 rounding in `solid_aabb` can round an enclosing minimum upward or maximum downward. The documented flush-touch direction workaround does not restore the enclosure guarantee.

**Repair:** bound each surface family with a proven enclosure, or return an explicit unknown-bound result and disable rejection for it. Gregory control-point hulls are a candidate approach; validate parameter-domain assumptions. Separate contact/direction selection from broad-phase rejection, retain f64 bounds or round outward, and test translated/scaled/sliver cases. Keep existing cylinder seam and large-fillet controls.

### 3. P2 — The overlapping-bore success oracle subtracts the intersection instead of the union

`OpenRCAD/crates/openrcad-algo/tests/cut_coincident_cap.rs:113–124`.

For two radius-4 discs 5 mm apart, `lens` is their intersection area. Both bores together remove `2*pi*r*r - lens`, not `lens`. The current oracle expects **8869.5122 mm³**; the correct remaining volume is **8125.1781 mm³**. It even expects more material than remains after the first bore (8497.3452 mm³).

The test currently passes through its `Err(InvalidOutput)` arm, so this bad formula is dormant. A correct kernel repair would activate it and fail the test; an incorrect material result could satisfy it.

**Repair first, before boolean implementation:** use the union formula, assert monotonic material removal, verify remaining volume and local material occupancy/connectivity, and distinguish known unsupported rejection from supported success. Keep safe-rejection coverage separately so a green characterization is never counted as a completed repair.

### 4. P1 — Pattern resource limits required by the plan remain absent

`zerocad-core/src/parametric/eval.rs:7801–7811,7855–7857`; cancellation is checked before entry at `2236–2240`.

The body-pattern branches eagerly collect `1..count` transforms with no upper work/memory bound. `count` is u32. A persisted very large count can request billions of transforms before instance processing, and no cancellation check occurs inside this construction. The finite-spacing guard fixes A4's specific panic but does not complete Step 2's explicit bounded-work requirement.

**Source-confirmed; no intentional out-of-memory execution.** This is an existing residual weakness, not proof the guard introduced it.

**Repair:** validate checked instance/component/work budgets before allocation, pass cancellation through transform generation and per-instance geometry work, and stage outputs atomically. The new transform-error branch skips instances and can retain partial output; define failure as rollback unless partial patterns are an explicit product contract. Test budget rejection without allocating a huge collection, cancellation, signed spacing, and a failed middle instance.

### 5. P2 — Vertex-only containment is not a sound replacement for curved bounds

`zerocad-core/src/parametric/edge_mod.rs:3003–3027`.

The new comment calls vertex boxes “subset-monotone” and says this gate proves containment. Neither is true for curved solids: a cut can create vertices on a retained curved surface beyond the original vertices' box; conversely, a bulging surface can escape while all vertices stay inside. The existing volume floor does not prove containment either.

**Source/mathematical finding, not a newly reproduced fillet failure.** Existing circular-rim and large-radius controls must continue to pass.

**Repair:** use a sound outer bound of the source to reject clearly impossible candidate points, then validate candidate material against the source where a proof is required. Treat loose bounds as inconclusive. Add partial-cylinder/seam-changing candidates and surface-interior escape cases. Do not simply restore the previous conservative-box-to-conservative-box comparison; that also produces false rejections.

### 6. P2 — Test completeness is still overstated

`break-test/tests/common/mod.rs:186–200`, `multi_sketch_extrude.rs:145–173`, `thread_torture.rs:629–680`, and crash-regression assertions.

`total_volume` silently omits bodies with unavailable mass properties; repeated evaluations can reuse the same cache; the zero-parent extrusion test discards its result; thread-on-tube coverage allows an 8% volume difference without checking which wall changed. Several tests named “parameter diagnostic” assert warning text, not the typed code, feature attribution, or state. These tests are useful crash smoke checks, but do not prove their stronger stated contracts.

**Repair:** require mass availability where a solid-volume oracle is used, assert body identity/count and typed diagnostics, compare cold/warm/reopened graphs, and check affected material locally. Preserve deliberate unsupported tests under clearly labeled expectations. Complete GUI and independent geometry qualification separately.

## Disposition of every original finding

| Finding | Current verdict |
|---|---|
| A1 invalid box | Guard correctly placed before construction; fresh dimension tests pass. Numeric-range qualification remains broader than these samples. |
| A2 nonfinite extrusion | Effective f32 depth checked after expression conversion; supplied regressions pass. Add explicit expression-overflow and typed-diagnostic assertions. |
| A3 huge box | Overflow-safe arena key band is a reasonable local arithmetic fix; huge-box and topology tests pass. This is not general resource/precision qualification. |
| A4/A6 pattern spacing/mirror offset | Specific crash guards pass. Completion is partial because of Finding 4. |
| A5 thread numbers | Nonfinite parameter regression passes; supplied finite/lead checks are correctly before construction. Does not certify thread geometry or measurement. |
| E1/E2 rim/boss-root | Incomplete legacy references are honestly rejected. Captured concave boss-root capability still needs a dedicated proof; no blanket capability claim. |
| E3 large single-edge radius | Correct analytic fillet oracle and valid radii covered by passing blend suite. Finding 5 remains separate. |
| E4/E5 tiny/too-thick fillets | Honest unresolved behavior retained; passing controls. |
| E6 crossing ribs | Still unsupported with rollback; fresh characterization passes. This remains incomplete capability work, not a newly broken fix. |
| E7/E8 | Not present in the original findings inventory; no invented verdict. |
| E9 tangential FaceMove | Existing shear behavior correctly retained; corrected geometry assertions pass. |
| E10 FaceThicken | Create-new-body route used by planar/cylinder/cone/sphere branches. Source-plus-plate regression and imported-surface tests pass; no new ownership defect established. |
| E11 circular FeaturePattern | Corrected box and cylinder bolt-circle regressions pass. |
| E12 negative FaceOffset | Endpoint algebra is correct; requested-plane regression passes. |
| E13 bare FaceRef | Strict document validation intentionally retained; separate headless recovery design remains. |
| E14 tangent inscribed cut | Supported-component topology expansion remains open; do not perturb radius to force success. |
| E15 curved BodyCut | Cylinder-specific improvement passes current body-boolean coverage; general bounds completion denied by Finding 2. |
| E15b full consumption/island removal | Still a recorded unsupported/rollback case; whole-island characterization passes, which does not mean removal works. |
| E16 overlapping cuts | Still incomplete; kernel regression accepts guarded rejection and has the incorrect success oracle in Finding 3. Fresh façade characterization passes, reproducing the silent no-op cut rather than a correct subtraction. Treat the wrong-success path as P1. |
| E17 ownership after body booleans | Intended resulting-feature ownership; retain this contract. |
| E18 plug join | Correct protruding-plug material expectation retained; body-boolean suite passes. |
| E19 thread selection | **Not fixed generally**: Finding 1. Far-reference rejection alone is insufficient. |
| E20 threaded cutters | Remains limited/unsupported; no general support claim. Long thread interactions are not qualified by this review. |
| E21 extrusion cardinality | Exactly-one-sketch implementation is appropriate; four-test suite passes. Zero-parent assertions need strengthening. |
| R1 bounds | Partially fixed; Findings 2 and 5. |
| R2 thread measurement | Still open; no fix or fresh independent measurement qualification established. |
| R3 tests | Still partial; Findings 3 and 6. |
| R4 expensive workloads | Still open. Concurrent debug test duration is not a performance baseline. |

## Repair order and acceptance

1. **Correct the bore oracle and promote the two archived probes to active failing regressions.** Add typed-diagnostic and local-material checks. Keep the archived probes outside automatic discovery until implementing the fixes, so this review alone does not add deliberate CI failures.
2. **Repair thread identity and unknown-bound handling.** These are the highest demonstrated correctness risks. Keep previous bodies on rejection; require each counterexample plus the existing positive controls to pass.
3. **Finish pattern work budgets/cancellation and repair blend validation.** Add bounded adversarial tests and atomic-output assertions.
4. **Resolve existing boolean/thread failures in separate changes.** First eliminate wrong-success E16 outcomes, then implement exact crescent/empty-component subtraction; investigate R2 representation disagreement. Crossing ribs, tangent components, and broader blends remain capability work with exact material tests.
5. **Qualify the final tree.** Run formatting, check, full workspace tests, the actual Clippy baseline gate in both workspaces, and bounded long suites. Verify cold/warm/save-reopen/upstream-edit/GUI behavior and independent geometry for newly supported operations. Record working-tree fingerprints and explicit timeouts. Do not classify old failures as unrelated merely because reverting the bounds migration alone leaves them failing.

Estimated relative effort: oracle and guard-test corrections are small; face-identity plumbing and bounded pattern work are moderate; general surface bounds, material containment, thread measurement, and boolean topology are substantial kernel work. No defensible calendar estimate follows from this review alone.

## Evidence

Review probes: [archived source](../break-test/review/implementation_audit_probes.rs), [failure output](../break-test/review-2026-09-13-probes.log). To reproduce, copy the source to `zerocad-core/tests/implementation_audit_probes.rs`, run `cargo test -p zerocad-core --test implementation_audit_probes -- --nocapture`, then remove only that temporary copy. Both tests intentionally assert the desired behavior and currently fail. No production source was modified.

The following are fresh results, not copied from the previous report:

| Check | Result |
|---|---|
| Box/extrusion degenerates | 17 passed |
| Edge blends | 29 passed |
| Feature edge cases | 19 passed |
| Multi-sketch extrusion | 4 passed |
| Body booleans | 15 passed |
| Direct-feature torture | 27 passed |
| Nonfinite thread guard | 1 passed |
| Crossing-rib characterization | 1 passed; verifies rollback |
| OpenRCAD topology tests | 47 passed |
| Kernel coincident-cap tests | 3 passed; overlapping-bore case permits rejection |
| Imported face edits | 14 passed, **1 failed**: external cylindrical boss deletion returns invalid topology and retains the source. The expected resulting body is absent. |
| Full thread-torture target | 16 passed in 382.42 seconds; completed before the attempted timeout check, so no process was interrupted. Broad tolerances/characterizations limit what these passes prove. |
| Threaded-shaft pocket regression | **1 failed** in 47.24 seconds: remaining volume 1051.29 equals the threaded baseline 1051.29; the pocket removes no material. This and the boss failure were also recorded in the previous plan; introduction is not attributed by this audit. |
| Overlapping second-cut characterization | 1 passed, reproducing the known Hole rejection and silent cut-extrude no-op |
| Additional review probes | **2 failed**, as described above |
| `cargo fmt --all -- --check` | Passed initially and again after archiving the separately formatted probes |
| `cargo check` | Passed |
| `cargo clippy -p zerocad-core --all-targets` | Completed with warnings; this alone is not the repository's Clippy baseline gate |
| `python scripts/check-clippy.py` | Passed both-workspace baseline gate: no warnings beyond 450 reviewed occurrences |

Logs are `break-test/review-2026-09-13-*.log`; [source fingerprints](../break-test/review/implementation-audit-fingerprints-2026-09-13.json) identify six central reviewed files at final capture. Full workspace tests, interactive GUI verification, independent CAD-oracle qualification, and a release/performance baseline are **not** established by this review. The 100-pocket/36-hole long suites were not rerun. All test processes launched by this review completed. The two existing regression failures alone prevent a clean test-gate claim.
