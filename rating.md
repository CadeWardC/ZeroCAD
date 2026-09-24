# ZeroCAD competitive rating

Assessment date: 2026-09-04. Scale: 1–10.

Follow-up, 2026-09-05: the [competitive development plan](docs/competitive-development-plan.md)
sets the improvement priorities. The [box-cavity correction](docs/box-cavity-correctness-2026-09-05.md)
addresses the specific defect cited below; these original assessment scores are
retained pending broader qualification, not automatically raised for one fix.

**Verdict: ZeroCAD is a credible early contender for focused mechanical-part modeling, but is not yet a general replacement for FreeCAD or Fusion 360.** My current ratings are **6/10 for focused part modeling**, **4/10 as a broad FreeCAD alternative**, and **3/10 as a broad Fusion alternative**. These are editorial judgments about current usefulness and evidence, not percentages of feature parity.

No existing `rating.md` was found in this checkout, so this document establishes the rubric. It evaluates the current working tree, including existing uncommitted improvements. It does not rate only the last commit or imply those improvements have shipped.

## How to read the scores

| Score | Meaning |
|---|---|
| 1 | No demonstrated user workflow in the reviewed scope |
| 2 | Infrastructure or very limited partial capability |
| 3 | Early implementation with major practical limitations |
| 4 | Usable in restricted cases; substantial gaps |
| 5 | Functional foundation; reliability or breadth limits regular use |
| 6 | Credible for common, bounded workflows |
| 7 | Strong capability with identifiable remaining gaps |
| 8 | Mature and broadly useful |
| 9 | Excellent professional capability |
| 10 | Exceptional breadth, dependability, and evidence in this category |

The tables score all three products on the same scale. Competitor scores are informed estimates from official capability documentation, not results of a controlled three-product benchmark. Fusion means its broader commercial offering, including relevant paid extensions; FreeCAD includes its established add-on ecosystem where explicitly stated. No price or entitlement equivalence is implied. A 1 means no demonstrated workflow, not proof that no experimental code exists anywhere.

ZeroCAD scores combine implemented functionality, known correctness limits, and verification maturity. Source structure earns architectural credit but does not prove usability or performance. UI and competitor scores have lower confidence than directly documented ZeroCAD defects. Summary scores are use-case judgments, not arithmetic averages: missing drawings or CAM can block an entire workflow even when sketching is good.

## Sketching and parametric design

| Area | ZeroCAD | FreeCAD | Fusion | ZeroCAD basis and main limitation |
|---|---:|---:|---:|---|
| Basic sketch creation | 7 | 8 | 9 | Lines, circles, arcs, ellipses, profiles, and interactive tools exist; no fresh usability session here. |
| Constraints and solver | 6 | 8 | 9 | Stable entity IDs, numerical solving, and DOF analysis; broad difficult-sketch qualification remains limited. |
| Dimensions, expressions, units | 7 | 9 | 9 | Expression-driven features and unit handling; application precision remains mixed. |
| Splines and advanced curves | 5 | 8 | 9 | Spline entities and curve controls exist; representation alone does not establish downstream robustness. |
| Trim, offset, sketch patterns | 6 | 8 | 9 | Dedicated implementations; needs more combined editing workflows. |
| Projection and references | 6 | 8 | 9 | Associative projected geometry and provenance; reference survival is not universally proven. |
| Sketch text and engraving | 7 | 7 | 8 | Shaped, baked glyphs, counters, engraving, and recent nested-contour fixes. |
| History and upstream editing | 6 | 8 | 9 | Explicit semantic ordering, suppression, and editable features; complex edit resilience remains bounded. |
| Topological naming | 5 | 7 | 8 | History-based identity and a targeted branch discriminator; not a universal solution. |
| Datums and construction geometry | 6 | 8 | 9 | Datum planes, axes, and points have dedicated schema and implementation. |

Evidence: [feature schema](zerocad-core/src/parametric/types.rs), [sketch entities and constraints](zerocad-core/src/sketch/constraints.rs), [solver](zerocad-core/src/sketch/solve.rs), [text follow-through](docs/text-and-interchange-2026-09-04.md), and [reference improvements](docs/audit-follow-through-2026-09-04.md).

## Solids, kernel, and correctness

| Area | ZeroCAD | FreeCAD | Fusion | ZeroCAD basis and main limitation |
|---|---:|---:|---:|---|
| Extrude and revolve | 7 | 9 | 9 | Real B-Rep operations and regression coverage; representative customer histories remain scarce. |
| Loft and sweep | 6 | 8 | 9 | Dedicated paths and explicit support boundaries; general freeform behavior is not established. |
| Boolean join/cut/intersect | 5 | 8 | 9 | Typed failures, budgets, validation, and replay evidence; general intersection completeness remains open. |
| Fillets and chamfers | 5 | 7 | 9 | Native blends and guarded results; curved-support and complex blend coverage remains limited. |
| Shell and thicken | 4 | 7 | 8 | Implemented, but a documented enclosed-cavity tessellation defect limits confidence. |
| Draft | 5 | 8 | 9 | Implemented with supported-surface restrictions. |
| Holes and threads | 6 | 8 | 9 | Standards library and dedicated feature paths; broad manufacturing qualification not shown. |
| Patterns and mirror workflows | 6 | 8 | 9 | Sketch/feature patterns and transform/history work; difficult patterned histories need wider coverage. |
| Multiple bodies and body operations | 6 | 8 | 9 | Join, cut, intersect, split, scale, and transform support. |
| Direct face editing | 5 | 6 | 9 | Offset, move, delete, and thicken; constrained geometric scope. |
| Freeform surface modeling | 3 | 7 | 9 | Kernel NURBS infrastructure is not a complete user-facing surface-design workspace. |
| Precision across scales | 4 | 8 | 9 | Kernel and sketch solver use doubles, but authoritative application coordinates and many dimensions still use floats. |
| Measurement and mass properties | 4 | 8 | 9 | Measurement and interference tools exist; cavity orientation affects trust in tessellation-derived volume. |
| Safe failure and diagnostics | 7 | 7 | 8 | Transactional rejection, warnings, cancellation, and preserved prior bodies are substantial strengths. |

Evidence: [architecture and invariants](README.md), [inspection implementation](zerocad-core/src/parametric/inspection.rs), [application coordinates](zerocad-core/src/geometry.rs), [capability boundaries](docs/capability-robustness-roadmap.md), and [latest defect record](docs/text-and-interchange-2026-09-04.md).

The cavity example is material: the latest record reports **37,248 mm³ from native checked tessellation versus 15,744 mm³ expected**, while the exported B-Rep passes independent validation. Correct STEP export therefore does not qualify native cavity measurements. This is a documented existing result, not a reproduction performed for this rating.

## Assemblies and interchange

| Area | ZeroCAD | FreeCAD | Fusion | ZeroCAD basis and main limitation |
|---|---:|---:|---:|---|
| Assembly structure and editing | 6 | 7 | 9 | Definitions, occurrences, replacement, visibility, grounding, and transactional edits; nested subassemblies are not implemented. |
| Mates and committed solving | 5 | 7 | 9 | Deterministic solver and convergent stress cases; calibration qualification still fails. |
| Interactive assembly manipulation | 3 | 6 | 8 | Measured large interactive cases did not converge within their deadlines. |
| Large-assembly readiness | 3 | 6 | 8 | Thousand-part backend evidence exists; full viewport qualification does not. |
| STEP part import | 4 | 9 | 9 | Useful single-solid path; cavities and multiple solids explicitly unsupported by the current native importer. |
| STEP part export | 6 | 9 | 9 | Revision-bound B-Rep export, independent checks, multiple bodies, and cavity export. |
| STEP assembly exchange | 4 | 7 | 9 | Named instances and placements export; assembly STEP import remains absent. |
| STL/OBJ/3MF workflows | 6 | 8 | 9 | STL import and mesh export paths; not a full mesh-repair or manufacturing workspace. |
| DXF and broader format coverage | 4 | 8 | 8 | Planar ASCII DXF import; narrower demonstrated exchange coverage. |

Evidence: [assembly qualification](docs/assembly-qualification-2026-09-04.md), [STEP export service](zerocad-core/src/step_export.rs), [current importer limitations](docs/text-and-interchange-2026-09-04.md), and [file workflow map](README.md).

The recorded 1,000-plate sparse and loop cases had committed medians of **74.37 ms and 103.06 ms**. Interactive calls had **0/31 convergent samples in each case**. These translation workloads do not establish general assembly performance, and fast CPU hydration excludes disk, solving, GPU upload, and presentation.

## Complete engineering workflows

| Area | ZeroCAD | FreeCAD | Fusion | ZeroCAD basis and main limitation |
|---|---:|---:|---:|---|
| Manufacturing drawings and GD&T | 1 | 8 | 9 | No demonstrated associative production-drawing workspace. |
| Sheet metal and flat patterns | 1 | 7 | 9 | No demonstrated sheet-metal workflow; FreeCAD score includes add-ons. |
| CAM and CNC toolpaths | 1 | 7 | 9 | Mesh export does not create machining toolpaths or postprocessed programs. |
| FEA and engineering simulation | 1 | 8 | 8 | Geometric inspection is not a structural or thermal solver. |
| Mechanism simulation | 2 | 6 | 8 | Assembly placement/constraints offer a foundation, not qualified motion studies. |
| Electronics and PCB integration | 1 | 5 | 9 | No demonstrated PCB design integration; FreeCAD score includes ecosystem bridges. |
| Photorealistic rendering | 2 | 5 | 8 | Engineering viewport exists; no demonstrated production rendering workflow. |
| BOM and production documentation | 5 | 7 | 8 | Assembly BOM and CSV export exist; associative drawing and comprehensive production deliverables remain gaps. |
| End-user scripting and extensions | 2 | 10 | 8 | Rust library modularity is useful to developers; no comparable user automation ecosystem demonstrated. |
| Team collaboration and data management | 2 | 4 | 9 | Local documents and exchange; shared revision/review/permission workflows not demonstrated. |

These are scope gaps, not necessarily immediate roadmap commitments. FreeCAD officially documents parametric modeling, exchange, drawings, analysis, and extensibility in its [feature overview](https://www.freecad.org/features.php?lang=us). Its [Assembly documentation](https://github.com/FreeCAD/FreeCAD-documentation/blob/main/wiki/Assembly_Workbench.md) describes its built-in assembly workspace. Fusion documents integrated design, simulation, manufacturing, and drawings in its [workspace guide](https://help.autodesk.com/view/fusion360/ENU/?contextId=GS-WORKSPACES), and broader commercial capabilities in its [feature overview](https://www.autodesk.com/products/fusion-360/features). These sources establish capability scope; the numerical scores remain my judgment.

## Product quality and engineering foundation

| Area | ZeroCAD | FreeCAD | Fusion | ZeroCAD basis and main limitation |
|---|---:|---:|---:|---|
| Modeling interface and discoverability | 6 | 6 | 9 | Live previews, handles, contextual tools, and shortcuts; provisional source-based score. |
| Viewport architecture | 7 | 7 | 9 | GPU depth testing, reuse, asynchronous picking, sectioning, and CPU fallback. |
| Demonstrated end-to-end performance | 5 | 6 | 8 | Useful local measurements; no matched competitor benchmark or complete startup/memory/frame qualification. |
| Save integrity and recovery | 8 | 8 | 8 | Atomic writes, integrity checks, bounded decoding, optional-cache isolation, and autosave recovery. |
| Long-term file compatibility | 5 | 8 | 8 | Explicit versioned contracts; prototype compatibility and migration remain bounded. |
| Automated verification | 7 | 8 | 8 | Substantial native tests, external checks, fuzz/replay infrastructure; limited real-part breadth. Competitor internals not audited. |
| Architecture and maintainability | 7 | 7 | 8 | Core/UI separation and semantic contracts; large geometry hotspots and existing lint backlog. Competitor scores have low confidence. |
| User documentation and onboarding | 4 | 8 | 9 | Good engineering documentation; less demonstrated user training and workflow material. |
| Distribution and release qualification | 3 | 9 | 9 | Historical release gates remain open; current assembly qualification is failed/incomplete. |
| Community and support ecosystem | 2 | 9 | 9 | No comparable ecosystem demonstrated in the reviewed repository. |
| Source freedom and local ownership | 9 | 10 | 4 | Permissive source licensing and local documents are strong advantages. |

Evidence: [README](README.md), [document format](zerocad-core/src/zcad_format.rs), [stabilization record](docs/phase7-completion.md), and [current performance evidence](docs/evidence/text-upgrade-2026-09-04/performance.md). The historical Part Design 1.0 gate is not treated as the active vNext release specification; neither roadmap completion nor a test pass proves a qualified release.

The latest recorded text timings improved a trailing move from **138.99 ms to 14.60 ms**, approximately **9.5×**, on the documented local workload. Cold evaluation improved from 277.37 ms to 249.49 ms. This is a targeted improvement, not evidence that ZeroCAD is faster than either competitor.

## What would materially raise the rating

1. **Fix and independently verify cavity orientation and volume.** Check native tessellation, inspection, edits, persistence, and export together.
2. **Expand to 20–30 representative mechanical designs.** Exercise creation, upstream edits, suppression, undo/redo, reopen, and independent exchange. Track wrong geometry separately from safe unsupported outcomes.
3. **Qualify assembly interaction.** Meet convergence and residual criteria as well as latency; measure actual viewport work.
4. **Expand blends and intersections from real failures.** Preserve atomic rejection while adding support-pair regressions and imported curved geometry.
5. **Complete the precision design and migration.** Test small geometry far from the origin through the whole application pipeline.
6. **Choose a product scope and complete it.** For a mechanical CAD replacement, associative drawings and practical assembly exchange matter more than adding many isolated feature buttons. CAM would be a separate major investment.
7. **Collect release and user evidence.** Repeatable installed-build measurements and outside-user workflows would justify stronger usability and readiness scores.

The most plausible near-term position is an open, local mechanical-part modeler for bounded maker and engineering workflows. Broad FreeCAD or Fusion replacement requires both wider capabilities and stronger evidence that ordinary edits remain correct.

## Assessment limits and verification

This assessment inspected the architecture, feature and sketch schemas, precision and inspection paths, and current audit/qualification records. It did not interactively operate any of the three applications or run a shared modeling benchmark. Existing implementation changes were preserved; this assessment adds only `rating.md`.

Historical evidence reports 933 application tests, 575 kernel tests, and 11 independent STEP checks passing. Those are attributed to the latest follow-through record, not claimed as freshly reproduced here. A passing suite can coexist with the documented cavity defect and failed assembly qualification.

Fresh assessment checks: ran `cargo fmt --all -- --check`, `cargo check`, `cargo test`, and `cargo clippy -p zerocad-core --all-targets`. The command sequence completed; observed test results passed, including all 540 core unit tests and the mechanical workflow tests. Clippy completed with warnings. No fresh warning-baseline comparison, nested OpenRCAD suite, or independent external STEP verification was performed for this documentation-only change. `git diff --check` found no whitespace errors.
