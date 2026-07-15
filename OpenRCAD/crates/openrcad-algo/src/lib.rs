#![forbid(unsafe_code)]
#![cfg_attr(test, allow(deprecated))]
// The BVH builder and the sewing engine walk parallel index arrays (vertex/edge
// key tables, SAH bins); index loops read more directly than iterator chains.
#![allow(clippy::needless_range_loop)]
//! Modeling algorithms for OpenRCAD (OCCT `TKBool` / `TKGeomAlgo` / `TKFillet`).

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

pub use blend::BlendError;
pub use facade::SolidExt;
pub use heal::{heal_solid_operation, heal_solid_operation_with_policy, HealingError};
pub use operation::{
    Diagnostic, DiagnosticSeverity, OperationResult, RecoveryAction, RecoveryReport,
    ValidationReport,
};

use serde::{Deserialize, Serialize};

use openrcad_topo::{Face, Shell, Solid};

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
    boolean_checked_bodies_with_policy, boolean_checked_with_cancel, boolean_checked_with_history,
    boolean_checked_with_history_and_policy, boolean_checked_with_history_cancel,
    boolean_checked_with_history_policy_and_cancel, boolean_checked_with_policy,
    boolean_checked_with_policy_and_cancel, boolean_operation,
    boolean_operation_with_classes_policy_and_cancel, boolean_operation_with_policy,
    boolean_operation_with_policy_and_cancel, boolean_with_history, BooleanError,
    BooleanFaceHistory, BooleanFaceSource, BooleanInput,
};
pub mod contour;
pub use contour::{
    apply_blend_contour, apply_blend_contour_with_policy, BlendContour, BlendContourError,
    BlendCurveHint, BlendKind, BlendLaw,
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
pub fn chamfer(solid: &Solid, distance: f64) -> Result<Solid, BlendError> {
    chamfer::chamfer(solid, distance)
}

/// Hollow out `solid` by `thickness` while leaving `open_faces` removed
/// (OCCT `BRepOffsetAPI_MakeThickSolid`).
///
/// Handles a single box or cylinder primitive at any orientation; see [`fillet`]
/// for the error cases.
pub fn shell_solid(
    solid: &Solid,
    thickness: f64,
    open_faces: &[Face],
) -> Result<Solid, BlendError> {
    offset::shell_solid(solid, thickness, open_faces)
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
pub use skin::{skin_polygon_rings, SkinError};

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
) -> Result<Shell, openrcad_foundation::TolerancePolicyError> {
    sew::sew_with_policy(faces, policy)
}
