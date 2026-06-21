# OpenRCAD Kernel Hardening Plan

OpenRCAD's path to beating Truck is trust first: every operation used by a CAD
application's feature timeline must either return a healthy, watertight solid or
a structured error that the application can surface without poisoning downstream
features. ZeroCAD is one intended consumer of this contract, but the plan is for
the public OpenRCAD kernel.

## Current Contract

- Low-level experiments can still call `openrcad_algo::boolean`.
- Application code should prefer `openrcad_algo::boolean_checked` or the
  `openrcad-document` feature APIs.
- `Document` recompute only caches healthy, watertight solids.
- Failed document features are rolled back, leaving prior feature results intact.

## Priority 1: Boolean Robustness

Goal: make common CAD modeling operations close correctly.

**Status: substantially done (2026-06-20).** Every boolean goal-test is fixed and
un-`#[ignore]`d; `robustness.rs` and the `repro_*` suites are green.

1. ✅ Fix the partial-imprint regressions (`through_side_drill_should_be_closed`,
   `blind_pocket_cut_should_be_watertight`, `rotated_tool_partial_cut_should_be_watertight`,
   `corner_overlap_union_should_be_watertight`) and un-ignore each.
2. ✅ **Healthy, not just watertight** — `sew` re-threads each loop into a
   contiguous, consistently-oriented co-edge chain (`rethread_loop`), so results
   pass `is_healthy()` / `validate()`. This was the bug that made `boolean_checked`
   silently reject ordinary joins on thin/off-axis/coplanar geometry.
3. ✅ **Clean topology** — `merge.rs` merges coplanar adjacent faces and collapses
   collinear sub-edges (a 2-box union → a 6-face box), behind a watertight+healthy
   safety net. The collinear merge also restores full-span edges so edge selection
   (fillet/chamfer) finds them on a boolean result.
4. ✅ **Cylinder cuts and boss unions** — native (smooth) cylinder cuts (drill /
   pocket, tool or object) and the boss union (cap coplanar with the top, bored as
   a circular hole) all pass `boolean_checked`.
5. ⬜ Split a boolean cut that *severs* a body into separate bodies (currently one
   solid, Euler=4).

Acceptance:

```text
cargo test -p openrcad-algo --test robustness
cargo test -p openrcad-algo --test repro_screenshots --test repro_fillet --test repro_cylinder
cargo test --workspace
```

No common boolean should return `Ok` with invalid or non-watertight topology.

## Priority 2: Recoverable Algorithm Errors

Goal: applications should not need to wrap OpenRCAD in panic recovery.

1. Keep extending checked APIs around modeling operations.
2. Convert non-test algorithm `unwrap`/`expect` sites into structured errors.
3. Add `TessellationError` and checked mesh export paths.
4. Add `IntersectionError` where intersection failure is expected input behavior.
5. Preserve raw/internal helpers only where tests and benchmarks need them.

Acceptance:

```text
rg "unwrap\(|expect\(|panic!" crates/openrcad-algo crates/openrcad-mesh crates/openrcad-exchange
```

Remaining hits should be tests, examples, or documented invariants.

## Priority 3: Torture Corpus

Goal: build a public reliability story stronger than Truck's.

1. Add procedural boolean cases for face-touch, edge-touch, corner-touch,
   partial overlap, side drill, blind pocket, nested void, and rotated tools.
2. Add randomized primitive combinations with health gates.
3. Add application-generated regression models as downstream users find them.
4. Store small STEP regression files for import/export failures.
5. Promote `examples/bench_kernel.rs` to Criterion when timing stability matters.

Acceptance: every fixed bug has a named regression test.

## Priority 4: General Blends

Goal: move beyond whole-box and whole-cylinder special cases.

1. ✅ Selected-edge chamfer on planar solids (ZeroCAD uses a faceted cutter +
   boolean; the boolean robustness above makes it reliable).
2. ✅ Selected-edge fillet on planar solids — `fillet_edges` (rolling ball) works
   on box/plate edges and on boolean-result edges (collinear merge restores the
   selectable full-span edge). Over-large radius is rejected, not silently broken.
3. ✅ Constant-radius rolling-ball fillet on analytic surfaces (planar–planar,
   planar–cylindrical, planar–analytic).
4. ⬜ Three-valent corner patches.
5. ⬜ N-valent/Gregory corner handling.
6. 🟡 Self-intersection / non-closing detection — the per-edge fillet now returns
   an error instead of a degenerate solid; concave-offset self-intersection
   resolution is still open.

Acceptance: applications can fillet/chamfer selected edges of ordinary
mechanical parts without first converting them to special primitive cases.

## Priority 5: Data Exchange

Goal: preserve enough CAD intent for real application workflows.

1. STEP units.
2. STEP names and product labels.
3. STEP assemblies.
4. STEP colors/materials.
5. Round-trip boolean and blend results.
6. Keep `.zcad` as a richer native parametric recipe format for applications
   that want it.

Acceptance: import/export practical mechanical STEP files without losing
hierarchy or basic metadata.

## Priority 6: Viewport as Debugger

Goal: make failures understandable inside any CAD application.

1. Selection highlighting.
2. Face/edge/vertex pick modes.
3. Free-edge and non-manifold overlays.
4. Face-normal and bad-loop overlays.
5. Grid, axes, fit view, and view cube.
6. Per-face materials/colors.
7. WASM/WebGPU demo path.

Acceptance: when a boolean fails, the viewport can show why.
