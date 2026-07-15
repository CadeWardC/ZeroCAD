# Phase 1 representation migration ledger

This ledger is the mechanical boundary between the strict Phase 1 production
path and operations intentionally deferred to Phase 3. The integration test in
`zerocad-core/tests/phase1_api_boundaries.rs` rejects new legacy OpenRCAD calls
unless both the file and API are listed below.

## Migrated production paths

- Box, cylinder, cone, prism/extrusion, and revolve construction consume
  `OperationResult<Solid>` through `mock_kernel::outcome::consume_operation`.
- STEP feature import uses the strict `read_step_str_operation` entry point.
- Single-body fuse, cut, and common operations use policy-aware canonical
  boolean operations, including cancellation and owner-aware face history.
- Kernel diagnostics and recovery actions are converted into evaluator
  diagnostics by the outcome adapter. Shared topology history is mapped into
  ZeroCAD's current face-naming representation at that same boundary.
- Display tessellation crosses one explicitly named compatibility adapter. It
  is a no-op for Phase 1 solids with complete pcurves, and attaches and validates
  pcurves only for allowlisted Phase 3 results before strict tessellation.

## Phase 3 compatibility allowlist

| Production file | Temporary API | Reason and removal gate |
| --- | --- | --- |
| `zerocad-core/src/mock_kernel/boolean.rs` | `boolean_checked_with_history[_cancel]` | A severing cut returns multiple connected solids while preserving the combined pre-split face-index map. Replace with a structured multi-body operation result and tests for per-body history coverage. |
| `zerocad-core/src/mock_kernel/tessellation.rs` | `tessellate_compatibility_for_display_with_policy_and_cancel` | Fillet, chamfer, shell/offset, generalized sweep/loft/skin, pattern, and mirror results are Phase 3. Remove after each producer stores valid pcurves and its replacement tests pass. |
| `zerocad-core/src/parametric/edge_mod.rs` | `tessellate_compatibility_for_display_with_policy_and_cancel` | Phase 3 edge-mod candidate volume scoring uses the explicit, fallible compatibility adapter. Reconstruction failure rejects the candidate with a user-visible diagnostic; it never silently drops a fillet or panics. Remove after native fillet/chamfer trim builders store pcurves. |

No fillet, chamfer, or sequential-blend regression is ignored. Phase 3 frontier
cases run on every test pass with a Phase 1 safety contract: either the operation
returns a healthy, watertight result that passes standard validation, or it
returns a concrete diagnostic while leaving the healthy source body unchanged.
The valid native-kernel cases retain geometry assertions; expensive app-level
duplicates use bounded oversized-radius rejection cases so the gate cannot hang.

The removal gate remains a native fillet/chamfer trim builder that stores valid
pcurves at construction time. Relaxing strict validation or restoring implicit
projection inside strict tessellation is not an acceptable workaround.

The enclosed-void boolean probe is also deferred: its legacy result packs outer
and cavity faces into one disconnected shell. Its removal gate is representing
the cavity as a separately oriented inner shell of one solid; shell-connectivity
validation must remain strict.

The compatibility wrappers remain deprecated. Delete them in Phase 3 only
after the corresponding producer is migrated, strict representation tests pass,
and this allowlist entry can be removed.
