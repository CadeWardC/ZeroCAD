# break-test

Adversarial and real-world stress tests for the ZeroCAD part builder
(`zerocad-core` parametric evaluation + OpenRCAD kernel).

Run with:

```bash
cargo test -p break-test
```

The September 17–23 expansion adds 119 test functions across
`join_contract_matrix.rs`, `join_feature_lifecycle.rs`,
`import_attack_matrix.rs`, `expression_work_limits.rs`,
`closed_sweep_material.rs`, `joined_hole_lifecycle.rs`,
`cylinder_slab_cut.rs`, and `boolean_face_selection.rs`. Parameterized loops also exercise hundreds of
corrupt files and additional geometry variants. These are ordinary Cargo tests;
none of the break-test cases are disabled with `#[ignore]`.

See [the September 17 repair record](review/2026-09-17-repairs.md) for current
fixes, supported boundaries, test oracles, and verification. Earlier findings
below are historical observations and may describe behavior since repaired.

Run the new focused cases with:

```bash
cargo test -p break-test --test join_contract_matrix --test join_feature_lifecycle --test joined_hole_lifecycle --test import_attack_matrix --test expression_work_limits --test closed_sweep_material --test cylinder_slab_cut --test boolean_face_selection
```

## What "break" means here

Every test enforces the three universal invariants:

1. **No panics** — bad input produces a structured error
   (`evaluate_bodies_with_warnings` returns `Err`) or an unresolved-feature
   warning, never a crash.
2. **Finite geometry** — any produced mesh has finite vertices, in-range
   indices, and no repeated-index triangles.
3. **Determinism** — evaluating the same graph twice yields bit-identical
   volumes (guards against HashMap-ordering / parallel-eval races).

## Suites

| File | Focus |
|---|---|
| `degenerate_params.rs` | Zero / negative / NaN / huge / inf dimensions, depths, hole parameters, draft angles; ghost dependencies; duplicate feature ids. |
| `tangency_sweeps.rs` | Booleans swept through exact tangency — coplanar faces, flush side walls, tangent cylinders, coincident bodies, sliver-thin residue. |
| `complex_parts.rs` | Realistic parts: L-bracket, revolved hub, shelled housing, lofted duct, swept pipe elbow, body boolean workflows, patterns, long edit chains — with analytic volume checks. |
| `feature_edge_cases.rs` | Misused features: single-section lofts, disconnected sweep paths, extreme twist, thread on a flat face, consumed-tool reuse, empty joins, deep cut chains, nested/overlapping sketch regions. |
| `extrude_on_faces.rs` | The draw-on-a-face workflow: bosses/pockets on top and side faces, overhanging and floating sketches, midplane cuts, pocket-in-pocket chains, sliver depths, region-index misuse, 10-deep face-edit histories. |
| `complex_extrusions.rs` | Complex profiles (I-beam, L, combs, triangles from bare lines), region-grid cuts (25 and 100 pockets), one sketch feeding several extrudes, targeted cut/join between overlapping bodies, multi-body documents, ziggurats, interleaved 24-node histories, pseudo-random pocket fields, and the crossing-rib characterization tests (FINDINGS.md E6). |
| `edge_blend_break.rs` | Fillet/chamfer (EdgeMod + EdgeBlend): radius-overflow sweeps, corner networks, cylinder rims, box↔cylinder junctions, acute dihedral edges, chained fillets, stale references — plus `characterization_*` tests pinning the current capability gaps. |
| `torture_features.rs` | Draft, BodySplit, direct face modeling (FaceOffset/Move/Delete/Thicken with GUI-style captured face refs), FeaturePattern, mirror patterns, datum planes/chains/cycles — plus characterization tests for FINDINGS E9–E13. |
| `torture_sweeps_holes.rs` | Revolve/loft/sweep abuse (on-axis and axis-crossing profiles, angle continuum, Pappus check, zero-height lofts, torus sweeps, self-intersecting profiles, zigzags), hole edge/tangency torture, cylinder boolean torture — plus characterization tests for FINDINGS E14–E16. |
| `join_cut_stress.rs` | Join/cut contact topology: face/edge/vertex-only contact, sliver overlaps, 3-body chains, containment, tilted prisms, tangent rods, island splits, C-cuts, identity round-trips, thin webs, drafted pockets, 20-feature alternating chains, untargeted cuts through three bodies. |
| `body_booleans_stress.rs` | Full-body booleans: keep-tool reuse, intersect overlap/disjoint/contained/tangent, split midplane + join-back identity, boundary-tangent splits, ghost/duplicate join sources, island-body cuts, coaxial plug joins — with characterizations for FINDINGS E15b/E18 and the E17 body-ownership pin. |
| `sketch_expr_torture.rs` | Sketch-geometry torture (bowtie self-intersections, duplicate geometry, nesting, crossing line grids — FINDINGS E22 — region-selection semantics and E23, far-offset/micro coordinates, 50 concentric circles) plus the variable/expression system (exact math, missing vars, syntax errors, w/0, negative depths, variable chains, unit semantics, draft + pattern expressions). |
| `misc_torture.rs` | STL mesh bodies (import, garbage bytes, exclusion from solid booleans), FeaturePattern of targeted join/cut extrudes, self-referential BodyCut/Intersect, transform-copy and scale chains, 36-hole swiss cheese, diagonal cuts, bosses floating inside pocket voids, annulus-on-rod joins, a 100-feature document. |
| `multi_sketch_extrude.rs` | Single-feature / multi-sketch pins: one Extrude driving two sketches (FINDINGS E21 — the second sketch is silently dropped today), the two-extrude control, the two-section Loft contrast, and zero/non-sketch parent variants. |
| `thread_torture.rs` | Threads: external (partial, flipped, left-hand, multi-start), internal/tapped holes, bolt-in-plate assemblies, cross-drills through threads, cutting the threaded end off, collars on threaded ends, threaded bodies as cutters, threads after turn-down/on tubes/on revolved hubs, degenerate + NaN parameters (A5). |

| `research_boolean_metamorphic.rs` | Research-pass booleans + metamorphic laws: volume algebra (join/intersect/cut, prismatic and curved tools), union order-independence, cutting air inside a void, consumed-target rollback, near-miss/near-breakthrough cut bands, micro-jitter union chains, scale/rotation/far-translation invariance, torus-plane tangency, axis-crossing revolve. |
| `research_blend_shell_torture.rs` | Research-pass feature boundaries: shell thickness vs curvature radius (cylinder/torus/box), all-faces-open, two-opposite-faces-open, counterbore==bore, blind depth==thickness, draft collapse angle, inward offset beyond the body, thread pitch<depth, partial-turn end trims, multi-start counts, internal band past the host hole, 300-turn fine pitch. |
| `research_variables_history_torture.rs` | Research-pass parametrics: circular/self-referential variables, div-by-zero expressions, 200-deep chains, dangling references, 1e6-mm-from-origin precision, diamond dependency branches, `.zcad` round-trip with unicode/boundary ids, truncation+bitflip corruption fuzz. |
| `research_exchange_torture.rs` | Research-pass exchange: STL "solid"-header binaries, NaN/inf facets, degenerate/duplicate/non-manifold/winding defects, truncated/empty/lying-count files, ASCII number formats, single-facet open shells; garbage/header-only/unknown-schema STEP imports. |

## Catalog campaign suites (adversarial plan §6, categories A–J)

Implements the 128-scenario catalog from the adversarial testing plan against
the real APIs. Each file maps to one catalog letter; scenarios already pinned
by the earlier suites are noted in the file headers instead of being
duplicated.

| File | Catalog coverage |
|---|---|
| `catalog_a_numeric.rs` | Numeric inputs & expressions: conversion overflow via expressions (A02), sub-tolerance/subnormal magnitudes (A03 — found and fixed the box-panic), derived-product overflow (pattern count×spacing, thread lead, body scale; A04), empty/trailing-garbage expressions (A05), mm/inch unit paths (A07), variable rename/delete/recreate + save/reopen/undo lifecycle (A08). |
| `catalog_b_constraints.rs` | Sketch constraints: conflicting dimensions with leave-one-out attribution and last-valid-geometry preservation (B01), redundancy vs inconsistency dof stability (B02), near-collinear degeneracy sweep (B03), mixed micro/macro scales (B04), mirrored-branch stability (B05), dense equal/horizontal networks (B06), insertion-order invariance (B07). |
| `catalog_c_regions_text.rs` | Regions & text: nested material/void stacks with per-ring extrusion + occupancy (C01), duplicate/reversed/zero-length normalization (C02), endpoint gap-closure tolerance sweep (C03), multi-intersection-at-a-point sectors (C04), offset collapse atomicity (C05), projected-edge rebuild + kind-change refusal (C06), glyph counters/overlap fill (C07), baked-text reopen without the font (C08). |
| `catalog_d_extrude.rs` | Extrusion & region selection: all construction planes ± depth with occupancy (D01), zero-depth crossing under Join/Cut auto-direction (D02), annulus hole survival (D03), region-selection stability under insertion (D05), unfusable-region join rollback (D06), explicit vs implicit multi-body cut targeting (D07), draft frustum sweeps through collapse (D08). |
| `catalog_e_boolean.rs` | Boolean material: exact analytic union/intersect/subtract with occupancy probes (E01), overlapping-bore removal vs the union area (E03), full-consumption contract + island removal (E05), direct removed-material checks for sliver cuts (E08). Two tracked defects: overlapping and exactly-tangent bore pairs silently drop one bore. |
| `catalog_f_blends.rs` | Fillets/chamfers: partial chains at 3-way corners (F02), fillet/cut order-dependence with analytic answers both ways (F04), intersecting blends (F05), upstream edits that shrink/split/lookalike the selected edge (F06), blend placement invariance (F08). |
| `catalog_g_faces.rs` | Shell/draft/direct edits: shell + downstream pocket edits through the floor (G03), face-offset through zero remaining thickness (G04), normal vs tangential face moves (G05), boss-face vs hole-face deletion (G06), face thickening ownership (G07). Two tracked defects: shell-on-fillet warns but hollows; tangential moves are silent no-ops. |
| `catalog_h_sweeps.rs` | Revolves/lofts/sweeps: 360° seam closure with Pappus volumes (H01), section reorder invariance (H03), ring-section lofts with continuous cavities (H04), coincident sections (H05 — phantom-body defect), tight-bend and degenerate-path sweeps (H06/H07), closed-path sweeps (H08 — dropped-leg defect). |
| `catalog_i_holes_threads.rs` | Holes/threads: edge tangency/breakout swept against the analytic circle∩plate patch (I01 — exact-tangency no-op defect), blind depth ≡ through-all (I02), invalid counterbore/countersink orderings (I03), tube outer-wall vs inner-bore threads with wall-localized occupancy (I05). |
| `catalog_j_patterns.rs` | Patterns/mirrors/transforms: count boundaries incl. the 4096 instance cap (J01), work-budget bounding (J02), spacing extremes (J03), near-full-turn duplicate instances (J04), mid-pattern cancellation atomicity incl. a live cross-thread cancel (J06), mirror joining (J07 — chained-mirror defect), inverse-transform/scale drift (J08). |

**Still open from the catalog**: K (history/reference/caching matrices),
L (preview/commit/cancel lanes beyond the pattern cancellation in J06),
M (byte-level `.zcad` corruption beyond the existing truncation/bitflip
fuzz), N (STEP/DXF exchange families), O (assemblies and mate solving), and
P (viewport — interactive-only). Shared catalog helpers live in
`tests/common/mod.rs`: `assert_meshes_valid` (zero-area triangles, finite
normals, positive orientation), `occupancy`/`assert_occupancy` (ray-parity
material oracle), `close`/`assert_close` (abs+rel tolerance policy),
`body_volume` (per-body checks), `capture_face` (GUI-style face refs).

Note: `complex_extrusions.rs` runs the guarded booleans hundreds of times in
a debug build — expect it to take several minutes on its own.

Errors found by these suites — panics, capability gaps, and proposed fixes —
are written up in [FINDINGS.md](FINDINGS.md). A follow-up review audited
every finding and sequenced the repairs:
[break-test review and implementation plan](../docs/break-test-review-and-implementation-plan.md).

## Known breaks (found 2026-09-10; FIXED 2026-09-11)

Six real evaluator panics were found on first run, recorded as `#[ignore]`d
`..._known_break` tests, and fixed as structured regressions (the plan's
Step 2). All six now resolve to typed `PARAMETER_INVALID` diagnostics with
the affected feature unresolved and every body unchanged:

1. **Non-positive / non-finite Box dimensions** — validated before either
   constructor (`mock_kernel/primitives.rs:9`'s `.expect()` is no longer
   reachable from untrusted input).
2. **NaN / ±Infinity Extrude depth** — the effective depth (after expression
   evaluation and the f64→f32 conversion) is checked before any geometry
   work, for every mode.
3. **Huge / Infinite Box dimensions** — the OpenRCAD arena's spatial-grid
   neighbour arithmetic saturates into an overflow-safe key band
   (`openrcad-topo/src/arena.rs`); finite magnitudes have no invented cap,
   non-finite dimensions are parameter-rejected.
4. **NaN linear body-pattern spacing** — finite-value guard on the resolved
   spacing (valid negative spacing keeps its semantics); the pattern instance
   transform boundary reports failures instead of panicking.
5. **NaN / Infinity thread parameters** — pitch, depth, angle, derived lead,
   and optional length validated before band construction; the runout face
   sort is NaN-tolerant.
6. **NaN mirror offset** — same guard family as (4), including
   offset-expression results and circular body-pattern angles.

## Adding cases

Put new cases in the matching suite (or add a new `tests/*.rs` file that does
`mod common;`). Helpers live in `tests/common/mod.rs` — `add_box`,
`add_cylinder`, `add_sketch`, `add_extrude`, `add_hole`,
`assert_part_sane` (the full invariant gate, returns total volume), and
`tangency_sweep` (runs your builder across an offset array). When a test finds
a panic or NaN, fix the kernel, then keep the case here as a regression.
