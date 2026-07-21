#![forbid(unsafe_code)]
#![cfg_attr(test, allow(deprecated))]
// The BVH builder and the sewing engine walk parallel index arrays (vertex/edge
// key tables, SAH bins); index loops read more directly than iterator chains.
#![allow(clippy::needless_range_loop)]
//! Modeling algorithms for OpenRCAD (OCCT `TKBool` / `TKGeomAlgo` / `TKFillet`).

pub mod band_topology;
pub mod blend;
pub mod bvh;
pub mod euler;
pub mod facade;
pub mod heal;
pub mod imprint;
pub mod intersect;
pub mod merge;
mod native_pcurve;
pub mod operation;
pub mod plane_split;
pub mod shell_feasibility;

pub use band_topology::{
    BandBoundary, BandOwnerKey, BandSource, BandSupportKind, BandTopologyError, GeometryWorkBudget,
    GeometryWorkStage,
};
pub use blend::BlendError;
pub use facade::SolidExt;
pub use heal::{heal_solid_operation, heal_solid_operation_with_policy, HealingError};
pub use operation::{
    Diagnostic, DiagnosticSeverity, OperationResult, RecoveryAction, RecoveryReport,
    ValidationReport,
};
pub use plane_split::{
    split_solid_by_plane_operation, split_solid_by_plane_operation_with_policy, PlaneSplitBodies,
    PlaneSplitError,
};
pub use sew::SewError;
pub use shell_feasibility::{
    ShellArtifact, ShellCapabilityState, ShellHistoryDisposition, ShellMilestone,
    ShellOwnershipRule, ShellPcurveRule, ShellPrerequisiteState, ShellPrerequisiteStatus,
    ShellSelfIntersectionStep, ShellStage4CPrerequisite, ShellStageEntryDecision, ShellSupportKind,
    SHELL_FEASIBILITY_V1,
};

use serde::{Deserialize, Serialize};

use openrcad_topo::{Edge, Face, Shell, Solid};

/// A boolean operation between two solids (OCCT `BRepAlgoAPI_BooleanOperation`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum BooleanOp {
    /// Intersection: keep only the overlapping volume (`Common`).
    Common,
    /// Union: combine both volumes (`Fuse`).
    Fuse,
    /// Difference: subtract `tool` from `object` (`Cut`).
    Cut,
}

pub mod boolean;
#[allow(deprecated)]
pub use boolean::{
    boolean_bodies_operation_with_classes_policy_and_cancel, boolean_checked_bodies_with_policy,
    boolean_checked_with_cancel, boolean_checked_with_policy,
    boolean_checked_with_policy_and_cancel, boolean_operation,
    boolean_operation_with_classes_policy_and_cancel, boolean_operation_with_policy,
    boolean_operation_with_policy_and_cancel, BooleanBodies, BooleanError, BooleanFaceHistory,
    BooleanFaceSource, BooleanInput,
};
pub mod contour;
pub use contour::{
    apply_blend_contour, apply_blend_contour_with_policy, BlendContour, BlendContourError,
    BlendCurveHint, BlendKind, BlendLaw,
};
pub mod corner_network;
pub use corner_network::{
    CornerIncidentBand, CornerNetworkCertificate, CornerNetworkError, CornerNetworkPlan,
    CornerTangentSphere,
};

/// Apply `op` between `object` and `tool` (OCCT `BRepAlgoAPI_Fuse/Cut/Common`).
///
/// ```
/// use openrcad_primitives::make_box;
/// use openrcad_foundation::Pnt;
/// use openrcad_algo::{boolean, BooleanOp};
///
/// let a = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
/// let b = make_box(&Pnt::new(5.0, 0.0, 0.0), 10.0, 10.0, 10.0);
/// // The shared half-volume of two boxes overlapping along X.
/// let common = boolean(&a, &b, BooleanOp::Common);
/// assert!(common.face_count() > 0);
/// ```
#[deprecated(note = "use boolean_operation; this wrapper discards metadata and panics on failure")]
pub fn boolean(object: &Solid, tool: &Solid, op: BooleanOp) -> Solid {
    boolean::boolean_operation(object, tool, op)
        .map(|result| result.value)
        .unwrap_or_else(|error| panic!("boolean: {error}"))
}

/// Checked boolean operation for applications that need recoverable failures.
///
/// This preflights both operands, catches panics from the current boolean
/// engine, and rejects invalid or non-watertight output instead of returning a
/// body that downstream features might cache.
#[deprecated(note = "use boolean_operation; this wrapper discards operation metadata")]
pub fn boolean_checked(object: &Solid, tool: &Solid, op: BooleanOp) -> Result<Solid, BooleanError> {
    boolean::boolean_operation(object, tool, op).map(|result| result.value)
}

/// Apply `op`, then split a severed result into one solid per connected body
/// (see [`Solid::split_disconnected`]). Use this when a cut may slice a body in
/// two; the common single-body case returns a one-element vector.
#[allow(deprecated)]
#[deprecated(
    note = "Phase 3 compatibility API for multi-body results; this wrapper discards operation metadata"
)]
pub fn boolean_bodies(object: &Solid, tool: &Solid, op: BooleanOp) -> Vec<Solid> {
    boolean::boolean_bodies(object, tool, op)
}

/// Checked multi-body boolean: splits a severed result and validates each body.
#[allow(deprecated)]
#[deprecated(
    note = "Phase 3 compatibility API for multi-body results; this wrapper discards operation metadata"
)]
pub fn boolean_checked_bodies(
    object: &Solid,
    tool: &Solid,
    op: BooleanOp,
) -> Result<Vec<Solid>, BooleanError> {
    boolean::boolean_checked_bodies(object, tool, op)
}

pub mod chamfer;
pub mod fillet;
pub mod offset;
pub mod prism;
pub mod rolling_ball;
pub use chamfer::{chamfer_edges, chamfer_edges_with_policy, ChamferError};

/// Roll a constant-`radius` fillet along every edge of `solid`
/// (OCCT `BRepFilletAPI_MakeFillet`).
///
/// Handles a single box or cylinder primitive at **any position/orientation**;
/// returns [`BlendError::UnsupportedShape`] for arbitrary B-Reps (including
/// boolean results) and [`BlendError::ParameterTooLarge`] when the radius
/// exceeds the geometry.
///
/// ```
/// use openrcad_primitives::make_box;
/// use openrcad_foundation::Pnt;
///
/// let cube = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
/// let rounded = openrcad_algo::fillet(&cube, 1.0)?;
/// // 6 trimmed planes + 12 cylindrical edges + 8 spherical corners.
/// assert_eq!(rounded.face_count(), 26);
///
/// // A radius that would meet itself in the middle is rejected, with the limit.
/// match openrcad_algo::fillet(&cube, 5.0) {
///     Err(openrcad_algo::BlendError::ParameterTooLarge { max, .. }) => assert_eq!(max, 5.0),
///     other => panic!("expected ParameterTooLarge, got {other:?}"),
/// }
/// # Ok::<(), openrcad_algo::BlendError>(())
/// ```
#[deprecated(note = "use fillet_with_policy")]
pub fn fillet(solid: &Solid, radius: f64) -> Result<Solid, BlendError> {
    fillet::fillet_with_policy(
        solid,
        radius,
        &openrcad_foundation::TolerancePolicy::STANDARD,
    )
}

/// Roll a constant-radius fillet along every edge using one validated document
/// tolerance policy throughout construction and sewing.
pub fn fillet_with_policy(
    solid: &Solid,
    radius: f64,
    policy: &openrcad_foundation::TolerancePolicy,
) -> Result<Solid, BlendError> {
    fillet::fillet_with_policy(solid, radius, policy)
}

/// Chamfer every edge of `solid` by `distance` (OCCT `BRepFilletAPI_MakeChamfer`).
///
/// Handles a single box or cylinder primitive at any orientation; see [`fillet`]
/// for the error cases.
#[deprecated(note = "use chamfer_with_policy")]
pub fn chamfer(solid: &Solid, distance: f64) -> Result<Solid, BlendError> {
    chamfer::chamfer_with_policy(
        solid,
        distance,
        &openrcad_foundation::TolerancePolicy::STANDARD,
    )
}

pub fn chamfer_with_policy(
    solid: &Solid,
    distance: f64,
    policy: &openrcad_foundation::TolerancePolicy,
) -> Result<Solid, BlendError> {
    chamfer::chamfer_with_policy(solid, distance, policy)
}

/// Hollow out `solid` by `thickness` while leaving `open_faces` removed
/// (OCCT `BRepOffsetAPI_MakeThickSolid`).
///
/// Handles a single box or cylinder primitive at any orientation; see [`fillet`]
/// for the error cases.
#[deprecated(note = "use shell_solid_with_policy")]
pub fn shell_solid(
    solid: &Solid,
    thickness: f64,
    open_faces: &[Face],
) -> Result<Solid, BlendError> {
    offset::shell_solid_with_policy(
        solid,
        thickness,
        open_faces,
        &openrcad_foundation::TolerancePolicy::STANDARD,
    )
}

pub fn shell_solid_with_policy(
    solid: &Solid,
    thickness: f64,
    open_faces: &[Face],
    policy: &openrcad_foundation::TolerancePolicy,
) -> Result<Solid, BlendError> {
    offset::shell_solid_with_policy(solid, thickness, open_faces, policy)
}

pub mod revolve;
pub mod sew;
pub mod skin;
#[allow(deprecated)]
pub use prism::{prism, prism_operation, prism_operation_with_policy, sweep_prism, SweepError};
#[allow(deprecated)]
pub use revolve::{revolve, revolve_operation, revolve_operation_with_policy, RevolveError};
pub use rolling_ball::{
    chamfer_circular_edge_chain, chamfer_circular_edge_chain_with_policy,
    chamfer_tangent_edge_chain, chamfer_tangent_edge_chain_with_policy,
    edge_material_wedge_is_concave, fillet_circular_edge_chain,
    fillet_circular_edge_chain_with_policy, fillet_edges, fillet_edges_with_policy,
    fillet_planar_edge, fillet_planar_edge_with_policy, fillet_tangent_edge_chain,
    fillet_tangent_edge_chain_with_policy, rolling_ball_between_curved_faces,
    rolling_ball_between_curved_faces_with_policy, rolling_ball_between_planar_faces,
    rolling_ball_between_planar_faces_with_policy, rolling_ball_fillet_edge,
    rolling_ball_fillet_edge_with_policy, RollingBallBlend, RollingBallError,
};
#[allow(deprecated)]
pub use skin::{
    skin_ordered_section_loops_with_policy, skin_polygon_rings, skin_polygon_rings_with_policy,
    skin_section_loops_with_policy, SectionLoops, SkinError,
};

/// Error shared by canonical unary modeling-operation entry points.
#[derive(Clone, Debug, PartialEq)]
pub enum ModelingOperationError {
    Build(String),
    PcurveBuild(String),
    InvalidOutput(openrcad_topo::ValidationReport),
}

impl core::fmt::Display for ModelingOperationError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Build(reason) => write!(f, "operation construction failed: {reason}"),
            Self::PcurveBuild(reason) => {
                write!(f, "operation pcurve construction failed: {reason}")
            }
            Self::InvalidOutput(report) => {
                write!(f, "operation returned invalid topology: {report:?}")
            }
        }
    }
}

impl std::error::Error for ModelingOperationError {}

fn finish_unary_operation(
    input: &Solid,
    value: Solid,
    policy: &openrcad_foundation::TolerancePolicy,
) -> Result<openrcad_topo::OperationResult<Solid>, ModelingOperationError> {
    policy
        .validate()
        .map_err(|error| ModelingOperationError::Build(error.to_string()))?;
    let (value, reconstructed) = value
        .repair_pcurves(policy)
        .map_err(|error| ModelingOperationError::PcurveBuild(error.to_string()))?;
    let (value, mut recovery) = normalize_operation_result(value, policy);
    let validation = openrcad_topo::ValidationReport::for_solid(&value, policy);
    if !validation.is_valid() || value.validate_strict_with_policy(policy).is_err() {
        return Err(ModelingOperationError::InvalidOutput(validation));
    }
    if reconstructed > 0 {
        recovery
            .actions
            .push(openrcad_topo::RecoveryAction::ReconstructPcurves {
                count: reconstructed,
            });
    }
    Ok(openrcad_topo::OperationResult {
        history: openrcad_topo::TopologyHistory::conservative_unary(input, &value),
        value,
        diagnostics: Vec::new(),
        recovery,
        validation,
    })
}

fn finish_generated_operation(
    value: Solid,
    policy: &openrcad_foundation::TolerancePolicy,
) -> Result<openrcad_topo::OperationResult<Solid>, ModelingOperationError> {
    policy
        .validate()
        .map_err(|error| ModelingOperationError::Build(error.to_string()))?;
    let (value, reconstructed) = value
        .repair_pcurves(policy)
        .map_err(|error| ModelingOperationError::PcurveBuild(error.to_string()))?;
    let (value, mut recovery) = normalize_operation_result(value, policy);
    let validation = openrcad_topo::ValidationReport::for_solid(&value, policy);
    if !validation.is_valid() || value.validate_strict_with_policy(policy).is_err() {
        return Err(ModelingOperationError::InvalidOutput(validation));
    }
    if reconstructed > 0 {
        recovery
            .actions
            .push(openrcad_topo::RecoveryAction::ReconstructPcurves {
                count: reconstructed,
            });
    }
    Ok(openrcad_topo::OperationResult {
        history: openrcad_topo::TopologyHistory::generated_solid(&value),
        value,
        diagnostics: Vec::new(),
        recovery,
        validation,
    })
}

fn normalize_operation_result(
    value: Solid,
    policy: &openrcad_foundation::TolerancePolicy,
) -> (Solid, openrcad_topo::RecoveryReport) {
    let mut recovery = openrcad_topo::RecoveryReport::default();
    let before = value;
    let before_count = before.face_count();
    let value = merge::merge_coplanar_faces_classed_with_policy(&before, None, policy);
    if !std::sync::Arc::ptr_eq(before.brep(), value.brep()) {
        recovery
            .actions
            .push(openrcad_topo::RecoveryAction::MergeCoplanarFaces {
                removed_faces: before_count.saturating_sub(value.face_count()),
            });
    }
    let before = value;
    let before_count = before.face_count();
    let value = merge::merge_cocylindrical_faces_classed_with_policy(&before, None, policy);
    if !std::sync::Arc::ptr_eq(before.brep(), value.brep()) {
        recovery
            .actions
            .push(openrcad_topo::RecoveryAction::MergeCocylindricalFaces {
                removed_faces: before_count.saturating_sub(value.face_count()),
            });
    }
    (value, recovery)
}

pub fn fillet_operation_with_policy(
    solid: &Solid,
    radius: f64,
    policy: &openrcad_foundation::TolerancePolicy,
) -> Result<openrcad_topo::OperationResult<Solid>, ModelingOperationError> {
    let value = fillet_with_policy(solid, radius, policy)
        .map_err(|error| ModelingOperationError::Build(error.to_string()))?;
    finish_unary_operation(solid, value, policy)
}

/// Canonical operation result for a selected-edge rolling-ball fillet.
///
/// The builder remains atomic and returns typed [`RollingBallError`] values at
/// its direct API boundary. This operation adapter adds strict validation,
/// recovery evidence, and complete conservative topology lineage for evaluator
/// consumers that need an immutable candidate rather than a bare solid.
pub fn fillet_edges_operation_with_policy(
    solid: &Solid,
    edges: &[Edge],
    radius: f64,
    policy: &openrcad_foundation::TolerancePolicy,
) -> Result<openrcad_topo::OperationResult<Solid>, ModelingOperationError> {
    let value = rolling_ball::fillet_edges_with_policy(solid, edges, radius, policy)
        .map_err(|error| ModelingOperationError::Build(error.to_string()))?;
    finish_unary_operation(solid, value, policy)
}

pub fn chamfer_operation_with_policy(
    solid: &Solid,
    distance: f64,
    policy: &openrcad_foundation::TolerancePolicy,
) -> Result<openrcad_topo::OperationResult<Solid>, ModelingOperationError> {
    let value = chamfer_with_policy(solid, distance, policy)
        .map_err(|error| ModelingOperationError::Build(error.to_string()))?;
    finish_unary_operation(solid, value, policy)
}

pub fn shell_solid_operation_with_policy(
    solid: &Solid,
    thickness: f64,
    open_faces: &[Face],
    policy: &openrcad_foundation::TolerancePolicy,
) -> Result<openrcad_topo::OperationResult<Solid>, ModelingOperationError> {
    let value = shell_solid_with_policy(solid, thickness, open_faces, policy)
        .map_err(|error| ModelingOperationError::Build(error.to_string()))?;
    let mut result = finish_unary_operation(solid, value, policy)?;
    result.history = offset::shell_topology_history(solid, &result.value, open_faces, policy);
    Ok(result)
}

pub fn blend_contour_operation_with_policy(
    solid: &Solid,
    contour: &BlendContour,
    policy: &openrcad_foundation::TolerancePolicy,
) -> Result<openrcad_topo::OperationResult<Solid>, ModelingOperationError> {
    let value = apply_blend_contour_with_policy(solid, contour, policy)
        .map_err(|error| ModelingOperationError::Build(error.to_string()))?;
    finish_unary_operation(solid, value, policy)
}

pub fn skin_polygon_rings_operation_with_policy(
    rings: &[Vec<openrcad_foundation::Pnt>],
    policy: &openrcad_foundation::TolerancePolicy,
) -> Result<openrcad_topo::OperationResult<Solid>, ModelingOperationError> {
    let value = skin_polygon_rings_with_policy(rings, policy)
        .map_err(|error| ModelingOperationError::Build(error.to_string()))?;
    finish_generated_operation(value, policy)
}

pub fn skin_section_loops_operation_with_policy(
    sections: &[SectionLoops],
    policy: &openrcad_foundation::TolerancePolicy,
) -> Result<openrcad_topo::OperationResult<Solid>, ModelingOperationError> {
    let value = skin_section_loops_with_policy(sections, policy)
        .map_err(|error| ModelingOperationError::Build(error.to_string()))?;
    finish_generated_operation(value, policy)
}

pub fn skin_ordered_section_loops_operation_with_policy(
    sections: &[SectionLoops],
    closed: bool,
    policy: &openrcad_foundation::TolerancePolicy,
) -> Result<openrcad_topo::OperationResult<Solid>, ModelingOperationError> {
    let value = skin_ordered_section_loops_with_policy(sections, closed, policy)
        .map_err(|error| ModelingOperationError::Build(error.to_string()))?;
    finish_generated_operation(value, policy)
}

pub fn transform_operation_with_policy(
    solid: &Solid,
    transform: &openrcad_foundation::Trsf,
    is_reflection: bool,
    policy: &openrcad_foundation::TolerancePolicy,
) -> Result<openrcad_topo::OperationResult<Solid>, ModelingOperationError> {
    let scale = transform.scale_factor();
    if !scale.is_finite() || scale.abs() <= policy.resolution {
        return Err(ModelingOperationError::Build(
            "transform scale must be finite and greater than policy resolution".into(),
        ));
    }
    let reverses_orientation = transform.reverses_orientation();
    if is_reflection != reverses_orientation {
        return Err(ModelingOperationError::Build(format!(
            "transform orientation flag ({is_reflection}) disagrees with its linear determinant ({})",
            transform.linear_determinant()
        )));
    }
    let value = if reverses_orientation {
        let shells = solid
            .shells()
            .into_iter()
            .map(|shell| {
                let faces: Vec<Face> = shell
                    .faces()
                    .iter()
                    .map(|face| face.transformed(transform))
                    .collect();
                sew::sew_shell_with_policy(&faces, policy)
                    .map_err(|error| ModelingOperationError::Build(error.to_string()))
            })
            .collect::<Result<Vec<_>, _>>()?;
        Solid::from_shells(shells)
            .ok_or_else(|| ModelingOperationError::Build("reflection lost all shells".into()))?
    } else {
        solid.transformed(transform)
    };
    finish_unary_operation(solid, value, policy)
}

/// Sew a collection of faces into a single shell, joining edges within `tol`
/// (OCCT `BRepBuilderAPI_Sewing`).
#[deprecated(
    note = "use sew_with_policy; this wrapper supplies STANDARD for non-sewing tolerances"
)]
#[allow(deprecated)]
pub fn sew(faces: &[Face], tol: f64) -> Shell {
    sew::sew(faces, tol)
}

/// Sew a collection of faces using one validated document tolerance policy.
pub fn sew_with_policy(
    faces: &[Face],
    policy: &openrcad_foundation::TolerancePolicy,
) -> Result<openrcad_topo::OperationResult<Shell>, SewError> {
    sew::sew_with_policy(faces, policy)
}
