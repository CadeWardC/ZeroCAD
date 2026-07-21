use std::collections::HashSet;

use openrcad_algo::shell_feasibility::{
    shell_stage_entry_decision, stage4c_prerequisites_are_verified, ShellArtifact,
    ShellCapabilityState, ShellHistoryDisposition, ShellMilestone, ShellPcurveRule,
    ShellPrerequisiteState, ShellSelfIntersectionStep, ShellStage4CPrerequisite,
    ShellStageEntryDecision, ShellSupportKind, SHELL_FEASIBILITY_V1,
};

const SUPPORTS: [ShellSupportKind; 6] = [
    ShellSupportKind::Plane,
    ShellSupportKind::Cylinder,
    ShellSupportKind::Cone,
    ShellSupportKind::Sphere,
    ShellSupportKind::Torus,
    ShellSupportKind::Nurbs,
];

#[test]
fn wave4_support_matrices_are_complete_and_record_the_verified_4c_subset() {
    assert_eq!(SHELL_FEASIBILITY_V1.version, 1);
    assert_eq!(SHELL_FEASIBILITY_V1.offsets.len(), SUPPORTS.len());

    let offsets: HashSet<_> = SHELL_FEASIBILITY_V1
        .offsets
        .iter()
        .map(|entry| entry.support)
        .collect();
    assert_eq!(offsets, SUPPORTS.into_iter().collect());

    let expected_pairs = SUPPORTS.len() * (SUPPORTS.len() + 1) / 2;
    assert_eq!(SHELL_FEASIBILITY_V1.intersections.len(), expected_pairs);
    let actual_pairs: HashSet<_> = SHELL_FEASIBILITY_V1
        .intersections
        .iter()
        .map(|entry| {
            assert!(entry.first <= entry.second, "matrix pair is not canonical");
            (entry.first, entry.second)
        })
        .collect();
    assert_eq!(actual_pairs.len(), expected_pairs);
    for (index, first) in SUPPORTS.iter().enumerate() {
        for second in &SUPPORTS[index..] {
            assert!(actual_pairs.contains(&(*first, *second)));
        }
    }

    let cylinder = SHELL_FEASIBILITY_V1
        .offsets
        .iter()
        .find(|entry| entry.support == ShellSupportKind::Cylinder)
        .unwrap();
    assert_eq!(cylinder.state, ShellCapabilityState::VerifiedInShell);
    for support in [
        ShellSupportKind::Cone,
        ShellSupportKind::Sphere,
        ShellSupportKind::Torus,
    ] {
        assert_eq!(
            SHELL_FEASIBILITY_V1
                .offsets
                .iter()
                .find(|entry| entry.support == support)
                .unwrap()
                .state,
            ShellCapabilityState::VerifiedInShell
        );
    }
    let state = |first, second| {
        SHELL_FEASIBILITY_V1
            .intersections
            .iter()
            .find(|entry| entry.first == first && entry.second == second)
            .unwrap()
            .state
    };
    assert_eq!(
        state(ShellSupportKind::Plane, ShellSupportKind::Torus),
        ShellCapabilityState::VerifiedInShell
    );
    assert_eq!(
        state(ShellSupportKind::Cylinder, ShellSupportKind::Torus),
        ShellCapabilityState::VerifiedInShell
    );
    assert_eq!(
        state(ShellSupportKind::Cone, ShellSupportKind::Torus),
        ShellCapabilityState::AnalyticKernelOnly
    );
    assert_eq!(
        state(ShellSupportKind::Sphere, ShellSupportKind::Torus),
        ShellCapabilityState::GenericFallbackOnly
    );
    assert_eq!(
        state(ShellSupportKind::Torus, ShellSupportKind::Torus),
        ShellCapabilityState::AnalyticKernelOnly
    );
}

#[test]
fn wave4_topology_pcurve_and_self_intersection_contracts_are_explicit() {
    let artifacts: HashSet<_> = SHELL_FEASIBILITY_V1
        .ownership
        .iter()
        .map(|rule| rule.artifact)
        .collect();
    assert_eq!(
        artifacts,
        [
            ShellArtifact::RetainedOuterFace,
            ShellArtifact::OffsetInnerFace,
            ShellArtifact::RemovedOuterFace,
            ShellArtifact::TrimmedSupportFace,
            ShellArtifact::SupportIntersectionEdge,
            ShellArtifact::RemovedFaceRim,
        ]
        .into_iter()
        .collect()
    );
    let history: HashSet<_> = SHELL_FEASIBILITY_V1
        .ownership
        .iter()
        .map(|rule| rule.history)
        .collect();
    assert_eq!(
        history,
        [
            ShellHistoryDisposition::Modified,
            ShellHistoryDisposition::Generated,
            ShellHistoryDisposition::Deleted,
        ]
        .into_iter()
        .collect()
    );

    assert!(SHELL_FEASIBILITY_V1
        .pcurves
        .contains(&ShellPcurveRule::BuildGeneratedCoedgesDuringConstruction));
    assert!(SHELL_FEASIBILITY_V1
        .pcurves
        .contains(&ShellPcurveRule::RejectIncompleteOutput));
    assert_eq!(
        SHELL_FEASIBILITY_V1.self_intersection,
        [
            ShellSelfIntersectionStep::BroadPhaseFaceBounds,
            ShellSelfIntersectionStep::ExactSupportedPairIntersection,
            ShellSelfIntersectionStep::WallThicknessAndCollapseCheck,
            ShellSelfIntersectionStep::RejectAmbiguousOrUnresolved,
        ]
    );
}

#[test]
fn stage4c_retains_its_explicit_review_and_all_prerequisites_are_green() {
    assert_eq!(
        shell_stage_entry_decision(ShellMilestone::Stage4A),
        ShellStageEntryDecision::EnterImplementation
    );
    assert_eq!(
        shell_stage_entry_decision(ShellMilestone::Stage4B),
        ShellStageEntryDecision::EnterImplementation
    );
    assert_eq!(
        shell_stage_entry_decision(ShellMilestone::Stage4C),
        ShellStageEntryDecision::ResearchReviewRequired
    );
    assert!(SHELL_FEASIBILITY_V1.stage4c_review_required);
    assert!(stage4c_prerequisites_are_verified());

    let statuses: HashSet<_> = SHELL_FEASIBILITY_V1
        .stage4c_prerequisites
        .iter()
        .map(|entry| (entry.prerequisite, entry.state))
        .collect();
    for prerequisite in [
        ShellStage4CPrerequisite::ConeBandRecutCompletes,
        ShellStage4CPrerequisite::TorusBandRecutCompletes,
        ShellStage4CPrerequisite::CylinderSeamImprinting,
        ShellStage4CPrerequisite::FilletBandOverflow,
        ShellStage4CPrerequisite::DeterministicBandOwnership,
    ] {
        assert!(statuses.contains(&(prerequisite, ShellPrerequisiteState::Verified)));
    }
}
