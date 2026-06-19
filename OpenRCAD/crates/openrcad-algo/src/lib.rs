#![forbid(unsafe_code)]
// The BVH builder and the sewing engine walk parallel index arrays (vertex/edge
// key tables, SAH bins); index loops read more directly than iterator chains.
#![allow(clippy::needless_range_loop)]
//! Modeling algorithms for OpenRCAD (OCCT `TKBool` / `TKGeomAlgo` / `TKFillet`).

pub mod blend;
pub mod bvh;
pub mod euler;
pub mod facade;
pub mod intersect;

pub use blend::BlendError;
pub use facade::SolidExt;

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
pub use boolean::{BooleanError, BooleanInput};

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
pub fn boolean(object: &Solid, tool: &Solid, op: BooleanOp) -> Solid {
    boolean::boolean(object, tool, op)
}

/// Checked boolean operation for applications that need recoverable failures.
///
/// This preflights both operands, catches panics from the current boolean
/// engine, and rejects invalid or non-watertight output instead of returning a
/// body that downstream features might cache.
pub fn boolean_checked(object: &Solid, tool: &Solid, op: BooleanOp) -> Result<Solid, BooleanError> {
    boolean::boolean_checked(object, tool, op)
}

pub mod chamfer;
pub mod fillet;
pub mod offset;

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
pub fn fillet(solid: &Solid, radius: f64) -> Result<Solid, BlendError> {
    fillet::fillet(solid, radius)
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

pub mod sew;

/// Sew a collection of faces into a single shell, joining edges within `tol`
/// (OCCT `BRepBuilderAPI_Sewing`).
pub fn sew(faces: &[Face], tol: f64) -> Shell {
    sew::sew(faces, tol)
}
