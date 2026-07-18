//! Stable-platform replay gate for cases promoted out of nightly fuzzing.

use serde::Deserialize;
use std::collections::HashSet;
use zerocad_core::{BooleanCaseV1, BooleanReplayOutcome};

#[derive(Debug, Deserialize)]
struct PromotedCase {
    stage: CorpusStage,
    case: BooleanCaseV1,
    expected: BooleanReplayOutcome,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Deserialize)]
#[serde(rename_all = "snake_case")]
enum CorpusStage {
    Primitive,
    HistoryExtrusion,
    ImportedBrep,
    MixedTopology,
}

const PROMOTED: &[(&str, &str)] = &[
    (
        "primitive_box_union",
        include_str!("boolean_cases/primitive_box_union.json"),
    ),
    (
        "history_through_cut",
        include_str!("boolean_cases/history_through_cut.json"),
    ),
    (
        "imported_round_union",
        include_str!("boolean_cases/imported_round_union.json"),
    ),
    (
        "mixed_rotated_intersection",
        include_str!("boolean_cases/mixed_rotated_intersection.json"),
    ),
];

#[test]
fn promoted_boolean_cases_replay_in_stage_order_with_unique_identity() {
    let mut previous_stage = CorpusStage::Primitive;
    let mut identities = HashSet::new();
    for (name, json) in PROMOTED {
        let promoted: PromotedCase = serde_json::from_str(json)
            .unwrap_or_else(|error| panic!("promoted case {name} is invalid: {error}"));
        assert!(
            promoted.stage >= previous_stage,
            "primitive cases must precede history/import/mixed cases"
        );
        previous_stage = promoted.stage;
        let identity = promoted.case.identity().unwrap().hex();
        assert!(
            identities.insert(identity.clone()),
            "duplicate case {identity}"
        );
        let replay = promoted
            .case
            .replay()
            .unwrap_or_else(|error| panic!("promoted case {name} could not replay: {error}"));
        assert_eq!(replay.identity, identity);
        assert_eq!(
            replay.outcome, promoted.expected,
            "promoted case {name}: {:?}",
            replay.diagnostic
        );
        if replay.outcome == BooleanReplayOutcome::Resolved {
            assert!(replay.body_count > 0, "resolved case {name} has no body");
            assert!(replay.vertex_count > 0 && replay.edge_count > 0 && replay.face_count > 0);
        }
    }
}
