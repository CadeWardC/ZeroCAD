# Phase 7 compiler and clippy warning review

Both workspaces' clippy commands exit successfully. Phase 7 does not hide output
or use `continue-on-error` in CI. The current advisory warnings were reviewed by
family rather than mechanically rewriting geometry hotspots during stabilization:

- **Type complexity / too many arguments.** Concentrated in the documented
  tessellation, topology, edge-modification, and viewport hotspots. Splitting
  these signatures without their complete geometry matrices is a higher-risk
  refactor than retaining explicit local tuples at the 1.0 gate.
- **Trait-like Vec3 method names.** The established `add/sub/mul` API predates
  Phase 7. Renaming is public churn; implementing operators is a post-freeze API
  improvement and does not change numerical behavior.
- **Iterator/style suggestions.** `map_or`, range-loop, `len() > 0`, and similar
  diagnostics are readability suggestions with no identified correctness or
  safety consequence. They are not suppressed globally and remain visible.
- **Test-only warnings.** Redundant clones, borrows, and intentionally simple
  fixture code occur outside production paths. They remain visible so normal
  cleanup may remove them without coupling that work to kernel stabilization.
- **Items after test modules.** Several older large GUI modules place focused
  tests beside the first relevant implementation. Moving large blocks solely to
  satisfy ordering is deferred until those files receive dedicated refactors.

New Phase 7 recovery, report-bundle, release-contract, oracle, and stress code is
clippy-clean. Any new warning in those files is a regression. A clippy non-zero
exit is a hard CI and handoff failure even while the reviewed advisory families
above remain printed.
