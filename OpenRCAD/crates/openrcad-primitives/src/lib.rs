#![forbid(unsafe_code)]
//! Primitive solid builders for OpenRCAD (OCCT `TKPrim`).
//!
//! Each builder turns a handful of parameters into a watertight [`Solid`] built
//! from analytic [`openrcad_geom::GeomSurface`]s and [`openrcad_topo::Edge`]s /
//! [`openrcad_topo::Wire`]s:
//!
//! - [`make_box`] / [`make_wedge`] — planar prisms (6 planar faces).
//! - [`make_cylinder`] — cylindrical wall + two planar caps.
//! - [`make_cone`] — conical wall + cap(s), with a sharp-apex degenerate case.
//! - [`make_sphere`] — a single spherical surface split into eight faces.
//!
//! Curved primitives split their circular rims into arcs so that no two edges
//! share both endpoints (the endpoint-based deduplication in [`openrcad_topo::Solid`]
//! would otherwise collapse them). Every primitive satisfies the Euler–Poincaré
//! invariant V − E + F = 2.

mod common;

pub mod box_solid;
pub mod cone;
pub mod cylinder;
pub mod sphere;
pub mod wedge;

use openrcad_foundation::{Ax2, Pnt, TolerancePolicy, TolerancePolicyError};
use openrcad_topo::{
    HealthReport, OperationResult, RecoveryReport, Solid, TopologyHistory, ValidationReport,
};

/// Failure from a validated primitive operation.
#[derive(Clone, Debug, PartialEq)]
pub enum PrimitiveError {
    InvalidParameter(&'static str),
    InvalidTolerancePolicy(TolerancePolicyError),
    InvalidOutput {
        report: HealthReport,
        watertight: bool,
        pcurves_complete: bool,
    },
}

impl core::fmt::Display for PrimitiveError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidParameter(message) => f.write_str(message),
            Self::InvalidTolerancePolicy(error) => write!(f, "invalid tolerance policy: {error}"),
            Self::InvalidOutput {
                report,
                watertight,
                pcurves_complete,
            } => write!(
                f,
                "primitive output is invalid (watertight={watertight}, pcurves_complete={pcurves_complete}): {report:?}"
            ),
        }
    }
}

impl std::error::Error for PrimitiveError {}

fn finish_primitive(
    solid: Solid,
    policy: &TolerancePolicy,
) -> Result<OperationResult<Solid>, PrimitiveError> {
    policy
        .validate()
        .map_err(PrimitiveError::InvalidTolerancePolicy)?;
    let validation = ValidationReport::for_solid(&solid, policy);
    if !validation.is_valid() || solid.validate_strict_with_policy(policy).is_err() {
        return Err(PrimitiveError::InvalidOutput {
            report: validation.health,
            watertight: validation.watertight,
            pcurves_complete: validation.pcurves_complete,
        });
    }
    let history = TopologyHistory::generated_solid(&solid);
    debug_assert!(history.coverage_for_solid(&solid).is_complete());
    Ok(OperationResult {
        value: solid,
        history,
        diagnostics: Vec::new(),
        recovery: RecoveryReport::default(),
        validation,
    })
}

pub fn make_box_operation(
    corner: &Pnt,
    dx: f64,
    dy: f64,
    dz: f64,
) -> Result<OperationResult<Solid>, PrimitiveError> {
    make_box_operation_with_policy(corner, dx, dy, dz, &TolerancePolicy::STANDARD)
}

pub fn make_box_operation_with_policy(
    corner: &Pnt,
    dx: f64,
    dy: f64,
    dz: f64,
    policy: &TolerancePolicy,
) -> Result<OperationResult<Solid>, PrimitiveError> {
    if !(dx > 0.0 && dy > 0.0 && dz > 0.0) {
        return Err(PrimitiveError::InvalidParameter(
            "box extents must be positive",
        ));
    }
    finish_primitive(box_solid::build_box(corner, dx, dy, dz), policy)
}

#[deprecated(note = "use make_box_operation; this wrapper discards operation metadata")]
pub fn make_box(corner: &Pnt, dx: f64, dy: f64, dz: f64) -> Solid {
    make_box_operation(corner, dx, dy, dz)
        .unwrap_or_else(|error| panic!("make_box: {error}"))
        .value
}

pub fn make_cylinder_operation(
    axis: &Ax2,
    radius: f64,
    height: f64,
) -> Result<OperationResult<Solid>, PrimitiveError> {
    make_cylinder_operation_with_policy(axis, radius, height, &TolerancePolicy::STANDARD)
}

pub fn make_cylinder_operation_with_policy(
    axis: &Ax2,
    radius: f64,
    height: f64,
    policy: &TolerancePolicy,
) -> Result<OperationResult<Solid>, PrimitiveError> {
    if !(radius > 0.0 && height > 0.0) {
        return Err(PrimitiveError::InvalidParameter(
            "cylinder radius and height must be positive",
        ));
    }
    finish_primitive(cylinder::build_cylinder(axis, radius, height), policy)
}

#[deprecated(note = "use make_cylinder_operation; this wrapper discards operation metadata")]
pub fn make_cylinder(axis: &Ax2, radius: f64, height: f64) -> Solid {
    make_cylinder_operation(axis, radius, height)
        .unwrap_or_else(|error| panic!("make_cylinder: {error}"))
        .value
}

pub fn make_cone_operation(
    axis: &Ax2,
    r1: f64,
    r2: f64,
    height: f64,
) -> Result<OperationResult<Solid>, PrimitiveError> {
    make_cone_operation_with_policy(axis, r1, r2, height, &TolerancePolicy::STANDARD)
}

pub fn make_cone_operation_with_policy(
    axis: &Ax2,
    r1: f64,
    r2: f64,
    height: f64,
    policy: &TolerancePolicy,
) -> Result<OperationResult<Solid>, PrimitiveError> {
    if !(r1 > 0.0 && r2 >= 0.0 && height > 0.0) {
        return Err(PrimitiveError::InvalidParameter(
            "cone radii must be non-negative with a positive base and height",
        ));
    }
    finish_primitive(cone::build_cone(axis, r1, r2, height), policy)
}

#[deprecated(note = "use make_cone_operation; this wrapper discards operation metadata")]
pub fn make_cone(axis: &Ax2, r1: f64, r2: f64, height: f64) -> Solid {
    make_cone_operation(axis, r1, r2, height)
        .unwrap_or_else(|error| panic!("make_cone: {error}"))
        .value
}

pub fn make_sphere_operation(
    center: &Pnt,
    radius: f64,
) -> Result<OperationResult<Solid>, PrimitiveError> {
    make_sphere_operation_with_policy(center, radius, &TolerancePolicy::STANDARD)
}

pub fn make_sphere_operation_with_policy(
    center: &Pnt,
    radius: f64,
    policy: &TolerancePolicy,
) -> Result<OperationResult<Solid>, PrimitiveError> {
    if radius <= 0.0 {
        return Err(PrimitiveError::InvalidParameter(
            "sphere radius must be positive",
        ));
    }
    finish_primitive(sphere::build_sphere(center, radius), policy)
}

#[deprecated(note = "use make_sphere_operation; this wrapper discards operation metadata")]
pub fn make_sphere(center: &Pnt, radius: f64) -> Solid {
    make_sphere_operation(center, radius)
        .unwrap_or_else(|error| panic!("make_sphere: {error}"))
        .value
}

pub fn make_wedge_operation(
    dx: f64,
    dy: f64,
    dz: f64,
    ltx: f64,
) -> Result<OperationResult<Solid>, PrimitiveError> {
    make_wedge_operation_with_policy(dx, dy, dz, ltx, &TolerancePolicy::STANDARD)
}

pub fn make_wedge_operation_with_policy(
    dx: f64,
    dy: f64,
    dz: f64,
    ltx: f64,
    policy: &TolerancePolicy,
) -> Result<OperationResult<Solid>, PrimitiveError> {
    if !(dx > 0.0 && dy > 0.0 && dz > 0.0 && ltx > 0.0 && ltx <= dx) {
        return Err(PrimitiveError::InvalidParameter(
            "wedge requires positive extents and 0 < ltx <= dx",
        ));
    }
    finish_primitive(wedge::build_wedge(dx, dy, dz, ltx), policy)
}

#[deprecated(note = "use make_wedge_operation; this wrapper discards operation metadata")]
pub fn make_wedge(dx: f64, dy: f64, dz: f64, ltx: f64) -> Solid {
    make_wedge_operation(dx, dy, dz, ltx)
        .unwrap_or_else(|error| panic!("make_wedge: {error}"))
        .value
}

#[cfg(test)]
mod operation_tests {
    use openrcad_foundation::{Ax2, Dir, Pnt, TolerancePolicy};

    use super::*;

    #[test]
    fn native_primitive_matrix_is_strictly_valid_and_history_complete() {
        let axis = Ax2::new(Pnt::origin(), Dir::dz());
        let results = vec![
            make_box_operation(&Pnt::origin(), 2.0, 3.0, 4.0).unwrap(),
            make_wedge_operation(4.0, 3.0, 2.0, 1.5).unwrap(),
            make_cylinder_operation(&axis, 2.0, 5.0).unwrap(),
            make_cone_operation(&axis, 3.0, 1.5, 4.0).unwrap(),
            make_cone_operation(&axis, 3.0, 0.0, 4.0).unwrap(),
            make_sphere_operation(&Pnt::origin(), 2.5).unwrap(),
        ];
        for result in results {
            assert!(result.validation.is_valid());
            assert!(result
                .value
                .validate_strict_with_policy(&TolerancePolicy::STANDARD)
                .is_ok());
            assert!(result.history.coverage_for_solid(&result.value).is_complete());
        }
    }

    #[test]
    fn canonical_primitive_rejects_bad_parameters_without_panicking() {
        assert!(matches!(
            make_box_operation(&Pnt::origin(), 0.0, 1.0, 1.0),
            Err(PrimitiveError::InvalidParameter(_))
        ));
        assert!(matches!(
            make_sphere_operation(&Pnt::origin(), -1.0),
            Err(PrimitiveError::InvalidParameter(_))
        ));
    }
}
