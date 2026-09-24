# Text performance and interchange follow-through

This is an uncommitted follow-up to the original audit. OpenRCAD remains the
shipped kernel. No OpenCASCADE runtime, new crate, or document schema was added.

## Implemented

Text-heavy body placement now retains the arena's exact one-to-one topology
history. The old conservative history attached every source entity of a kind
to every result of that kind, creating quadratic time and storage. Canonical
vertex/edge representatives are checked before positional correspondence is
used; changes caused by geometric quantization retain the conservative fallback.

A placement that passes both strict validation and the full validation report
returns immediately with its exact history. Invalid candidates still enter the
existing repair/normalization path. Reflections retain their existing sewing
path. Validation, tessellation accuracy, glyph curves, and model detail were not
weakened. This accelerates moves of any detailed body, including engraved text;
it does not promise a corresponding improvement to typing or every cold rebuild.

The prismatic text-cut path assigns cutouts to their smallest enclosing material
island. A dotted zero's dot therefore cuts its counter island rather than
creating an invalid nested hole on the plate face. Regression fixtures include
`ZeroCAD 07`, `0080`, and `B8@0` with the frozen Hack font. The original benchmark
still uses `ZeroCAD 17`, preserving a comparable workload. Ambiguous partial
containment declines the fast path, and the export validity guard remains.

The STEP writer emits `BREP_WITH_VOIDS` and reversed uses of oriented closed
shells for enclosed cavities. Independent validation of the 60 x 40 x 20 mm
box with a 56 x 36 x 16 mm void reports a valid solid and 15,744 mm³ material.
The one-solid native importer now explicitly rejects multiple solids and cavity
records rather than silently returning an arbitrary solid or the outer shell.

## Remaining work and the discovered cavity defect

Update 2026-09-05: the [box-cavity correction](box-cavity-correctness-2026-09-05.md)
addresses the construction/orientation defect described below. Retain these
measurements as the pre-fix record; see that document for current verification.

Native cavity import is deliberately unsupported. Investigation found the
native checked tessellation of the source closed enclosure itself reports
37,248 mm³ instead of 15,744 mm³: inner cap and wall triangle senses disagree.
The exported B-Rep passes independent shape/volume validation. This discrepancy
needs a focused shell-orientation/tessellation repair, native and independent
mass checks, and only then native cavity import. Export success is not evidence
that native cavity mass properties are qualified.

The full audit roadmap remains unfinished: authoritative double precision and
versioned payload migration, trim-bounded general intersections, broader curved
blends, a representative mechanical corpus, and measured startup/memory/package
budgets. Those require separate compatibility and geometry evidence. This change
must not be described as Fusion/FreeCAD feature parity or production qualification.

## Reproduction

Run `cargo run --release -p zerocad-core --example text_pipeline_profile` to
inspect placement stages on the frozen text part. The legacy stages are measured
individually alongside the canonical operation; they are explanatory timings,
not a replacement for the end-to-end Criterion comparison.

Run `mechanical_workflows` and `step_export` with
`ZEROCAD_INTERCHANGE_EVIDENCE_DIR` set, then use `tools/verify_step_exports.py`.
OCP is a separate optional test installation and is never shipped with ZeroCAD.

## Verified results

| Engraved plate | Earlier median | Current median |
|---|---:|---:|
| Cold evaluation | 277.37 ms | 249.49 ms |
| Cached evaluation | 4.79 ms | 4.13 ms |
| Trailing move edit | 138.99 ms | 14.60 ms |

The move is approximately 9.5 times faster in these local captures. Ten samples
used the same machine, two workers, frozen inputs, compiler, and optimized build
settings. Separate sessions also show shifts in unaffected cached workloads;
small differences must not be attributed entirely to this change. In five
same-process explanatory profiles, the canonical placement median was
3.68 ms, versus 104.55 ms for the summed legacy stages. This confirms
the removed history/cleanup cost independently of the earlier timing session.

[All 15 timings and caveats](evidence/text-upgrade-2026-09-04/performance.md),
[raw estimates/samples](evidence/text-upgrade-2026-09-04/performance.json),
[placement profiles](evidence/text-upgrade-2026-09-04/placement-stages.json), and
[source/environment fingerprint](evidence/text-upgrade-2026-09-04/environment.json)
are retained.

Formatting passes in both workspaces, application check passes, all 933
application tests and 575 kernel tests pass with zero ignored tests, and the
Clippy gate passes without new warnings (452 existing occurrences). The updated
through-cut fixture was also rerun after its final expansion. All 11 independent
STEP checks pass, including dotted counters, a through-cut with separate islands,
and cavity export. Native cavity import is tested as an explicit unsupported
outcome. No interactive UI test, hosted CI run, or memory measurement is claimed.
