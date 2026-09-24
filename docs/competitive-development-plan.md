# ZeroCAD competitive development plan

Date: 2026-09-05. Status: active product direction; most implementation milestones below remain open.

## Product objective

Make ZeroCAD a dependable, fast, compact mechanical CAD application with rich sketching, parametric parts, assemblies, and interchange. Common work should feel direct: select geometry, invoke a relevant tool, see a responsive preview, enter a dimension, commit, and revise later without rebuilding the design manually.

The user priorities are optimization, lightweightness, ease of use, feature depth, and accessibility to coding agents. Here, accessibility to agents means a repository that an agent can navigate and modify safely, plus a reusable command surface for future CAD automation. It does not imply adding an embedded model, a cloud dependency, or a new service.

This plan sets product priorities over the existing [capability roadmap](capability-robustness-roadmap.md). That roadmap retains detailed technical contracts and the vNext baseline. The [precision plan](precision-and-performance-plan.md) retains migration requirements. [rating.md](../rating.md) is an assessment, not an implementation checklist or evidence of completion. Historical release gates remain historical; this plan does not declare a release.

## Non-negotiable engineering rules

- Correct material, dimensions, topology, and reference identity take precedence over a fast incorrect result. Unsupported work must preserve the previous valid model and explain the limitation.
- Optimize measured work: avoid unnecessary rebuilds, mesh regeneration, uploads, cloning, and allocation before adding concurrency or complex caches.
- Keep authoritative geometry precise. Viewport approximations, adaptive preview quality, and float GPU buffers must not change final geometry or measurement.
- Keep operation cost proportional to changed geometry where possible. Shared assembly definitions should not duplicate immutable geometry per occurrence.
- Bound preview queues, cache memory, undo memory, cancellation work, imports, and decompression. A new feature must account for its lifetime and resource use.
- Keep the shipped runtime native and local. New dependencies need a concrete benefit and measured package/startup/memory impact. Test-only independent CAD tools stay outside release packages.
- Preserve stable feature IDs and document compatibility. Derived-cache incompatibility should trigger rebuilds, not invalidate the model recipe. Authoritative schema changes require explicit migration tests.
- Keep routine tools discoverable and consistent. Advanced controls should expand when relevant rather than make every basic operation complicated.

## What “Fusion-like ease” means here

These are interaction requirements, not a promise to reproduce another application's entire interface:

1. Selection drives available commands. Faces, edges, profiles, and bodies have clear hover and selection feedback and persistent selections through compatible tool changes.
2. Common dimensions can be typed at the geometry; units and expressions work consistently. Drag handles and numeric inputs update the same staged operation.
3. Enter commits, Escape backs out safely, and Undo reverses one meaningful action. Preview dragging must not fill the history with hundreds of entries.
4. A preview reacts immediately to input, displays when exact calculation is pending, and never presents an old calculation as the current result.
5. Sketch constraint feedback identifies what is free, conflicting, or redundant and offers a useful correction. Do not require users to interpret numerical solver output.
6. Upstream edits retain intended references or visibly suspend the affected feature. Recovery should lead users to the failing selection or parameter.
7. Commands have consistent selection rules, defaults, validation, help, and keyboard access. A searchable command entry point should use the same operation implementation as the toolbar.
8. Save/open/import/export remain responsive, show useful progress for long jobs, and report actionable failures without discarding work.

Validate these with actual create/edit workflows and observed users. Source review and a screenshot cannot establish ease of use.

## Measurement and lightweightness contract

First record current behavior on the existing reference laptop and frozen release builds. Retain hardware, power mode, compiler, lockfiles, source plus working-tree fingerprint, worker count, corpus version, raw samples, p50/p95, and success/convergence outcomes. Shared-runner smoke results must be labeled separately.

The following are **initial design targets**, not achieved measurements or active CI thresholds. Ratify them against an uncontended baseline before enforcing them. Report absolute values and baseline deltas; do not silently weaken a target to pass a gate.

| Metric | Initial target / decision rule |
|---|---|
| Warm pointer/selection/drag feedback | p95 input-to-present <= 33 ms on the qualified small-part and sketch workloads |
| Ordinary interactive frame | p95 <= 16.7 ms on the qualified scene; publish triangle/occurrence counts and render settings |
| Common sketch dimension edit | p95 visible settled response <= 100 ms on a named constrained sketch |
| Stale expensive preview cancellation | p95 acknowledgement <= 50 ms on the frozen heavy workload; no stale writeback |
| Common small-part exact preview | Aim for p95 <= 100 ms; otherwise retain immediate handles and show pending calculation |
| Idle CPU | No continuous geometry, tessellation, or upload work; target < 1% process CPU averaged over 60 s after settling |
| Startup, resident memory, package size | Measure signed installed cold/warm startup, idle and peak working set, installed bytes, and compressed bytes separately; set absolute budgets from baseline |
| Growth per feature/occurrence | Record memory and allocation scaling at several sizes; reject unexplained quadratic growth in routine workloads |
| Performance regressions | A reproducible > 5% timing or > 10% memory/package increase triggers investigation, subject to measured noise and a documented feature tradeoff |
| Speed leadership | Require identical tasks, equivalent accuracy, hardware, and outcomes against competitor builds before claiming leadership |

Use short, deterministic checks during development and longer measurements for qualification. Never make a functional test depend on a fragile wall-clock threshold on a busy machine.

## Delivery sequence

Each milestone is a vertical delivery: core behavior, usable interface where relevant, persistence, regression evidence, performance impact, and a short implementation record. Completing source code alone is insufficient.

### 1. Trustworthy geometry and a representative baseline

First implementation slice: repair the reproduced box-cavity orientation defect. Native inspection, display output, cold/warm evaluation, save/reopen, and independent STEP validation must agree on material volume. Invalidate disposable caches containing old generated geometry. Do not enable cavity import solely because export passes.

Then expand the five-part corpus to 20–30 frozen designs: plates, brackets, blind and through pockets, shafts, threaded parts, open/closed enclosures, text counters, multi-loop lofts, swept profiles, curved imports, and small assemblies. Include large upstream dimension edits and selection deletion.

Exit evidence: zero wrong-success results in the qualified corpus; separate supported success, unsupported rejection, crash, timeout, and incorrect geometry. Record startup/memory/package and end-to-end UI baselines, not only evaluator timings.

### 2. Fast, predictable sketching and history editing

| Work | Acceptance |
|---|---|
| Profile solver recomputation and region rebuilding | Unchanged geometry is reused; dimensional edit latency and memory are measured on small and dense constrained sketches |
| Improve constraint diagnosis and direct dimension editing | User can identify the conflict and edit/remove its cause; drags, typed values, expressions, and undo agree |
| Standardize tool state and selection behavior | Keyboard, toolbar, and contextual tools share selection and staged-commit semantics |
| Harden projections, trims, offsets, and spline edits | Identity and constraints survive supported edits, rotations, save/reopen, and cold/warm evaluation |
| Improve reference repair | Missing topology suspends consumers; repair selects a deliberate replacement and is undoable |
| Add searchable commands through a registry | Search invokes existing operation paths and honors their enablement and selection rules |

Primary code: `zerocad-core/src/sketch/`, `parametric/topo_name.rs`, `parametric/diagnostics.rs`, `zerocad-gui/src/sketch_ui.rs`, `app/sketch.rs`, and `app/ui/viewport.rs`.

Exit evidence: a user can sketch, constrain, extrude, modify an early dimension, repair a deleted reference, undo, and reopen without hidden manual recovery. Measure interaction latency separately from the solver.

### 3. Broader reliable solids without runtime bloat

| Work | Acceptance |
|---|---|
| Common fillet/chamfer support expansion | Frozen failures for partial chains, curved supports, runouts, and interacting blends become correct successes; infeasible radii remain atomic failures |
| Shell/draft/thicken robustness | Wall thickness, normals, material, selected openings, and identities agree after edits and exchange |
| Trim-bounded intersections | Translated/scaled variants, periodic seams, tangencies, small loops, and spline imports preserve branches and material or return explicit unresolved failures |
| Loft/sweep depth | Supported multi-loop profiles, guides, orientation, and end conditions have regression-backed user workflows |
| Authoritative precision migration | Versioned payload decoders preserve existing documents; high-precision coordinates reach evaluation and measurement, with float conversion restricted to display |
| Targeted performance optimization | Profiles identify removed work; baseline/candidate results are geometrically equivalent and memory impact is recorded |

Use the existing per-feature files and kernel façade. Do not split fragile geometry hotspots during a feature fix; schedule dedicated refactors with frozen behavior and full geometry gates.

Exit evidence: every newly supported operation passes upstream-edit, cancellation, persistence, and independent geometry checks. NURBS data structures alone do not count as general freeform support.

### 4. Useful assemblies and dependable exchange

| Work | Acceptance |
|---|---|
| Reduce solver linearization cost | Remove repeated whole-mate scans and variable copies where profiling supports it; verify residuals, convergence, and allocation scaling |
| Bound interactive work | Deadline checks occur inside expensive phases; committed and drag solves meet separate quality contracts |
| Shared definition rendering | Transform-only occurrence edits avoid geometry rebuild and redundant GPU uploads; visibility and replacement invalidate only affected state |
| Broaden mate workflows | Add mate types from actual mechanical cases, with overconstraint diagnosis, undo, replacement, and persistence |
| Multi-solid and cavity STEP import | Import all supported material with correct units and orientation; reject unsupported structures without silently dropping solids or voids |
| Assembly STEP import and nesting | Define and test hierarchy, shared definitions, transforms, naming, cycles, and export equivalence before advertising support |
| Clear exchange UI | Show scope/units, progress, and actionable unsupported-content errors; retain atomic destination writes |

Primary code: `assembly*.rs`, `assembly_solver.rs`, `step_export.rs`, kernel exchange, GUI assembly panel, and document workers.

Exit evidence: named small real mechanisms and the 1,000-occurrence stress cases pass convergence and residual limits, plus measured disk-to-present opening and interactive frame cost. Fast calls that never converge do not pass.

### 5. Make development and automation easy to extend

- Keep `AGENTS.md` a short navigation map with feature owners, invariants, and the fastest relevant test. Put detailed specifications alongside the implementation.
- Add or extend features through stable IDs, typed inputs/results, and the existing transactional evaluator. Core operations must remain callable without the GUI or GPU.
- Give reproducible bug cases one invocation that names its input and expected result. Reuse the same frozen artifact in tests, profiles, and external verification.
- Expose structured diagnostics and capability boundaries. Tools should not need to scrape English messages or duplicate geometry logic.
- Build any future headless command interface over the same validation, transaction, revision, and persistence paths used by the application. Version commands and include deterministic result/diagnostic output.
- Prefer feature-sized modules and explicit interfaces. Extract orchestration separately from algorithms only when it makes the ownership clearer and has behavior-preserving tests.
- Track agent change quality using concrete maintenance tasks: time to locate the owner, files required for a change, targeted-check time, regressions, and completeness of the handoff. File count alone is not a quality metric.

Exit evidence: a new contributor or coding agent can add a small feature, reproduce a geometry defect, and run a headless modeling workflow from documented entry points without reverse-engineering the entire application.

## Definition of done

For each delivery record the problem, visible result, source owner, supported boundaries, regression case, checks, and measured resource impact. Mark unmeasured impacts explicitly. Keep `rating.md` scores evidence-based; finishing one defect does not automatically raise an entire category.

Required checks remain those in `AGENTS.md`, plus nested-kernel checks for kernel changes and the warning-baseline gate. UI-affecting changes require an actual interaction check. Geometry changes require semantic assertions and appropriate independent comparisons. Preserve schema compatibility and invalidate affected disposable cache ABIs.

The first slice is recorded in [box-cavity correctness](box-cavity-correctness-2026-09-05.md). Subsequent priorities are baseline capture and assembly solver profiling, followed by a complete sketch dimension-edit workflow. Broad parity and performance leadership remain objectives, not current claims.
