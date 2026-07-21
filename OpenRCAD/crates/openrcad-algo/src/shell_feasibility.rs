//! Executable Wave 4 Shell feasibility contract.
//!
//! A surface enum or a generic surface/surface solver is not sufficient to
//! claim curved Shell support. This module records the complete offset and
//! intersection matrix together with the topology-ownership, pcurve, history,
//! and self-intersection rules that each staged implementation must satisfy.

/// Analytic support families considered by the Wave 4 Shell contract.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum ShellSupportKind {
    Plane,
    Cylinder,
    Cone,
    Sphere,
    Torus,
    Nurbs,
}

/// The milestone that first requires a capability.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum ShellMilestone {
    Baseline,
    Stage4A,
    Stage4B,
    Stage4C,
    Deferred,
}

/// Readiness of a support or intersection inside the Shell pipeline.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShellCapabilityState {
    /// Construction, trimming, pcurves, validation, and history are tested in
    /// the general Shell pipeline.
    VerifiedInShell,
    /// A recognized primitive has a verified optimized builder, but mixed face
    /// networks cannot use it yet.
    PrimitiveFastPathOnly,
    /// The kernel has an analytic construction/intersection, but Shell does not
    /// yet own trimming, imprinting, pcurves, and history for it.
    AnalyticKernelOnly,
    /// Only the controlled subdivision/refinement solver is available. This is
    /// useful for research, but cannot close an analytic Shell milestone.
    GenericFallbackOnly,
    /// The required capability has no implementation yet.
    Missing,
    /// This support is intentionally outside the Wave 4 contract.
    Unsupported,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ShellOffsetCapability {
    pub support: ShellSupportKind,
    pub required_by: ShellMilestone,
    pub state: ShellCapabilityState,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ShellIntersectionCapability {
    pub first: ShellSupportKind,
    pub second: ShellSupportKind,
    pub required_by: ShellMilestone,
    pub state: ShellCapabilityState,
}

/// Result artifacts whose ownership and history must be unambiguous.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ShellArtifact {
    RetainedOuterFace,
    OffsetInnerFace,
    RemovedOuterFace,
    TrimmedSupportFace,
    SupportIntersectionEdge,
    RemovedFaceRim,
}

/// Required topology-history disposition for a Shell artifact.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ShellHistoryDisposition {
    Modified,
    Generated,
    Deleted,
}

/// Stable ownership source used for topology naming and history attribution.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ShellArtifactOwner {
    SourceFace,
    OrderedSupportPair,
    RemovedFaceBoundary,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ShellOwnershipRule {
    pub artifact: ShellArtifact,
    pub owner: ShellArtifactOwner,
    pub history: ShellHistoryDisposition,
}

/// Pcurve rules that every stage must satisfy before validation and commit.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ShellPcurveRule {
    PreserveUnchangedOuterCoedges,
    BuildGeneratedCoedgesDuringConstruction,
    AllowBoundedRepairForLegacyInputsOnly,
    RejectIncompleteOutput,
}

/// Provisional fail-closed self-intersection contract for Wave 4.
///
/// For the supported 4A/4B analytic boundary, thickness/collapse checks plus
/// strict topology validation and watertightness are atomic acceptance gates;
/// they are not a comprehensive proof that every possible self-intersection is
/// absent. Explicit concave intersection, imprinting, classification, and
/// resewing remain Wave 5 work.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ShellSelfIntersectionStep {
    BroadPhaseFaceBounds,
    ExactSupportedPairIntersection,
    WallThicknessAndCollapseCheck,
    RejectAmbiguousOrUnresolved,
}

/// Whether a stage may enter implementation under the current contract.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShellStageEntryDecision {
    EnterImplementation,
    BlockedOnPreviousStage,
    ResearchReviewRequired,
    Unsupported,
}

/// Kernel work that must be proven before the Stage 4C research review can
/// return a go decision. A guarded failure is deliberately not `Verified`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ShellStage4CPrerequisite {
    ConeBandRecutCompletes,
    TorusBandRecutCompletes,
    CylinderSeamImprinting,
    FilletBandOverflow,
    DeterministicBandOwnership,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ShellPrerequisiteState {
    Verified,
    Investigating,
    Blocked,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ShellPrerequisiteStatus {
    pub prerequisite: ShellStage4CPrerequisite,
    pub state: ShellPrerequisiteState,
}

/// Versioned feasibility data. This is an engineering/test contract, not a
/// persisted document schema.
#[derive(Clone, Copy, Debug)]
pub struct ShellFeasibilityContract {
    pub version: u8,
    pub offsets: &'static [ShellOffsetCapability],
    pub intersections: &'static [ShellIntersectionCapability],
    pub ownership: &'static [ShellOwnershipRule],
    pub pcurves: &'static [ShellPcurveRule],
    pub self_intersection: &'static [ShellSelfIntersectionStep],
    pub stage4c_prerequisites: &'static [ShellPrerequisiteStatus],
    pub stage4c_review_required: bool,
}

const OFFSETS: &[ShellOffsetCapability] = &[
    ShellOffsetCapability {
        support: ShellSupportKind::Plane,
        required_by: ShellMilestone::Baseline,
        state: ShellCapabilityState::VerifiedInShell,
    },
    ShellOffsetCapability {
        support: ShellSupportKind::Cylinder,
        required_by: ShellMilestone::Stage4A,
        state: ShellCapabilityState::VerifiedInShell,
    },
    ShellOffsetCapability {
        support: ShellSupportKind::Cone,
        required_by: ShellMilestone::Stage4B,
        state: ShellCapabilityState::VerifiedInShell,
    },
    ShellOffsetCapability {
        support: ShellSupportKind::Sphere,
        required_by: ShellMilestone::Stage4B,
        state: ShellCapabilityState::VerifiedInShell,
    },
    ShellOffsetCapability {
        support: ShellSupportKind::Torus,
        required_by: ShellMilestone::Stage4C,
        state: ShellCapabilityState::VerifiedInShell,
    },
    ShellOffsetCapability {
        support: ShellSupportKind::Nurbs,
        required_by: ShellMilestone::Deferred,
        state: ShellCapabilityState::Unsupported,
    },
];

const INTERSECTIONS: &[ShellIntersectionCapability] = &[
    intersection(
        ShellSupportKind::Plane,
        ShellSupportKind::Plane,
        ShellMilestone::Baseline,
        ShellCapabilityState::VerifiedInShell,
    ),
    intersection(
        ShellSupportKind::Plane,
        ShellSupportKind::Cylinder,
        ShellMilestone::Stage4A,
        ShellCapabilityState::VerifiedInShell,
    ),
    intersection(
        ShellSupportKind::Plane,
        ShellSupportKind::Cone,
        ShellMilestone::Stage4B,
        ShellCapabilityState::VerifiedInShell,
    ),
    intersection(
        ShellSupportKind::Plane,
        ShellSupportKind::Sphere,
        ShellMilestone::Stage4B,
        ShellCapabilityState::VerifiedInShell,
    ),
    intersection(
        ShellSupportKind::Plane,
        ShellSupportKind::Torus,
        ShellMilestone::Stage4C,
        ShellCapabilityState::VerifiedInShell,
    ),
    intersection(
        ShellSupportKind::Plane,
        ShellSupportKind::Nurbs,
        ShellMilestone::Deferred,
        ShellCapabilityState::Unsupported,
    ),
    intersection(
        ShellSupportKind::Cylinder,
        ShellSupportKind::Cylinder,
        ShellMilestone::Stage4A,
        ShellCapabilityState::AnalyticKernelOnly,
    ),
    intersection(
        ShellSupportKind::Cylinder,
        ShellSupportKind::Cone,
        ShellMilestone::Stage4B,
        ShellCapabilityState::VerifiedInShell,
    ),
    intersection(
        ShellSupportKind::Cylinder,
        ShellSupportKind::Sphere,
        ShellMilestone::Stage4B,
        ShellCapabilityState::GenericFallbackOnly,
    ),
    intersection(
        ShellSupportKind::Cylinder,
        ShellSupportKind::Torus,
        ShellMilestone::Stage4C,
        ShellCapabilityState::VerifiedInShell,
    ),
    intersection(
        ShellSupportKind::Cylinder,
        ShellSupportKind::Nurbs,
        ShellMilestone::Deferred,
        ShellCapabilityState::Unsupported,
    ),
    intersection(
        ShellSupportKind::Cone,
        ShellSupportKind::Cone,
        ShellMilestone::Stage4B,
        ShellCapabilityState::GenericFallbackOnly,
    ),
    intersection(
        ShellSupportKind::Cone,
        ShellSupportKind::Sphere,
        ShellMilestone::Stage4B,
        ShellCapabilityState::GenericFallbackOnly,
    ),
    intersection(
        ShellSupportKind::Cone,
        ShellSupportKind::Torus,
        ShellMilestone::Stage4C,
        ShellCapabilityState::AnalyticKernelOnly,
    ),
    intersection(
        ShellSupportKind::Cone,
        ShellSupportKind::Nurbs,
        ShellMilestone::Deferred,
        ShellCapabilityState::Unsupported,
    ),
    intersection(
        ShellSupportKind::Sphere,
        ShellSupportKind::Sphere,
        ShellMilestone::Stage4B,
        ShellCapabilityState::GenericFallbackOnly,
    ),
    intersection(
        ShellSupportKind::Sphere,
        ShellSupportKind::Torus,
        ShellMilestone::Stage4C,
        ShellCapabilityState::GenericFallbackOnly,
    ),
    intersection(
        ShellSupportKind::Sphere,
        ShellSupportKind::Nurbs,
        ShellMilestone::Deferred,
        ShellCapabilityState::Unsupported,
    ),
    intersection(
        ShellSupportKind::Torus,
        ShellSupportKind::Torus,
        ShellMilestone::Stage4C,
        ShellCapabilityState::AnalyticKernelOnly,
    ),
    intersection(
        ShellSupportKind::Torus,
        ShellSupportKind::Nurbs,
        ShellMilestone::Deferred,
        ShellCapabilityState::Unsupported,
    ),
    intersection(
        ShellSupportKind::Nurbs,
        ShellSupportKind::Nurbs,
        ShellMilestone::Deferred,
        ShellCapabilityState::Unsupported,
    ),
];

const fn intersection(
    first: ShellSupportKind,
    second: ShellSupportKind,
    required_by: ShellMilestone,
    state: ShellCapabilityState,
) -> ShellIntersectionCapability {
    ShellIntersectionCapability {
        first,
        second,
        required_by,
        state,
    }
}

const OWNERSHIP: &[ShellOwnershipRule] = &[
    ownership(
        ShellArtifact::RetainedOuterFace,
        ShellArtifactOwner::SourceFace,
        ShellHistoryDisposition::Modified,
    ),
    ownership(
        ShellArtifact::OffsetInnerFace,
        ShellArtifactOwner::SourceFace,
        ShellHistoryDisposition::Generated,
    ),
    ownership(
        ShellArtifact::RemovedOuterFace,
        ShellArtifactOwner::SourceFace,
        ShellHistoryDisposition::Deleted,
    ),
    ownership(
        ShellArtifact::TrimmedSupportFace,
        ShellArtifactOwner::SourceFace,
        ShellHistoryDisposition::Modified,
    ),
    ownership(
        ShellArtifact::SupportIntersectionEdge,
        ShellArtifactOwner::OrderedSupportPair,
        ShellHistoryDisposition::Generated,
    ),
    ownership(
        ShellArtifact::RemovedFaceRim,
        ShellArtifactOwner::RemovedFaceBoundary,
        ShellHistoryDisposition::Generated,
    ),
];

const fn ownership(
    artifact: ShellArtifact,
    owner: ShellArtifactOwner,
    history: ShellHistoryDisposition,
) -> ShellOwnershipRule {
    ShellOwnershipRule {
        artifact,
        owner,
        history,
    }
}

const PCURVES: &[ShellPcurveRule] = &[
    ShellPcurveRule::PreserveUnchangedOuterCoedges,
    ShellPcurveRule::BuildGeneratedCoedgesDuringConstruction,
    ShellPcurveRule::AllowBoundedRepairForLegacyInputsOnly,
    ShellPcurveRule::RejectIncompleteOutput,
];

const SELF_INTERSECTION: &[ShellSelfIntersectionStep] = &[
    ShellSelfIntersectionStep::BroadPhaseFaceBounds,
    ShellSelfIntersectionStep::ExactSupportedPairIntersection,
    ShellSelfIntersectionStep::WallThicknessAndCollapseCheck,
    ShellSelfIntersectionStep::RejectAmbiguousOrUnresolved,
];

const STAGE4C_PREREQUISITES: &[ShellPrerequisiteStatus] = &[
    prerequisite(
        ShellStage4CPrerequisite::ConeBandRecutCompletes,
        ShellPrerequisiteState::Verified,
    ),
    prerequisite(
        ShellStage4CPrerequisite::TorusBandRecutCompletes,
        ShellPrerequisiteState::Verified,
    ),
    prerequisite(
        ShellStage4CPrerequisite::CylinderSeamImprinting,
        ShellPrerequisiteState::Verified,
    ),
    prerequisite(
        ShellStage4CPrerequisite::FilletBandOverflow,
        ShellPrerequisiteState::Verified,
    ),
    prerequisite(
        ShellStage4CPrerequisite::DeterministicBandOwnership,
        ShellPrerequisiteState::Verified,
    ),
];

const fn prerequisite(
    prerequisite: ShellStage4CPrerequisite,
    state: ShellPrerequisiteState,
) -> ShellPrerequisiteStatus {
    ShellPrerequisiteStatus {
        prerequisite,
        state,
    }
}

pub const SHELL_FEASIBILITY_V1: ShellFeasibilityContract = ShellFeasibilityContract {
    version: 1,
    offsets: OFFSETS,
    intersections: INTERSECTIONS,
    ownership: OWNERSHIP,
    pcurves: PCURVES,
    self_intersection: SELF_INTERSECTION,
    stage4c_prerequisites: STAGE4C_PREREQUISITES,
    stage4c_review_required: true,
};

/// Stage 4C stays blocked until every prerequisite is verified. This check is
/// intentionally data-driven so replacing a stall with a guard cannot
/// accidentally satisfy the release gate.
pub fn stage4c_prerequisites_are_verified() -> bool {
    SHELL_FEASIBILITY_V1
        .stage4c_prerequisites
        .iter()
        .all(|entry| entry.state == ShellPrerequisiteState::Verified)
}

/// Entry decisions deliberately describe permission to begin implementation,
/// not completion. Stage 4A and 4B have verified staged entry contracts; Stage
/// 4C is always an explicit research review.
pub const fn shell_stage_entry_decision(stage: ShellMilestone) -> ShellStageEntryDecision {
    match stage {
        ShellMilestone::Baseline | ShellMilestone::Stage4A | ShellMilestone::Stage4B => {
            ShellStageEntryDecision::EnterImplementation
        }
        ShellMilestone::Stage4C => ShellStageEntryDecision::ResearchReviewRequired,
        ShellMilestone::Deferred => ShellStageEntryDecision::Unsupported,
    }
}
