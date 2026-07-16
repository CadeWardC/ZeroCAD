//! Strict planar solid splitting built on the canonical boolean engine.

use openrcad_foundation::{NeverCancelled, Pnt, TolerancePolicy, Vec as GeomVec};
use openrcad_geom::Plane;
use openrcad_topo::{
    Edge, InputTopologyRef, OperationResult, Orientation, RecoveryReport, Solid, TopologyHistory,
    TopologyKind, TopologyRef, ValidationReport, Wire,
};

use crate::boolean::{
    boolean_bodies_operation_with_classes_policy_and_cancel, BooleanFaceHistory, BooleanFaceSource,
};
use crate::native_pcurve::planar_face_with_pcurves;
use crate::BooleanOp;

/// Connected solids on the two sides of a splitting plane.
#[derive(Clone, Debug, PartialEq)]
pub struct PlaneSplitBodies {
    pub negative: Vec<Solid>,
    pub positive: Vec<Solid>,
    pub negative_face_history: Vec<BooleanFaceHistory>,
    pub positive_face_history: Vec<BooleanFaceHistory>,
}

impl PlaneSplitBodies {
    pub fn solids(&self) -> impl Iterator<Item = &Solid> {
        self.negative.iter().chain(&self.positive)
    }
}

/// A planar split never returns a one-sided or suspicious partial result.
#[derive(Clone, Debug, PartialEq)]
pub enum PlaneSplitError {
    InvalidTolerancePolicy(String),
    InvalidPlaneTool(String),
    NegativeSide(String),
    PositiveSide(String),
    DoesNotSplit,
    InvalidOutput(String),
    VolumeMismatch {
        input: f64,
        outputs: f64,
        relative_error: f64,
    },
}

impl core::fmt::Display for PlaneSplitError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidTolerancePolicy(reason) => write!(f, "invalid tolerance policy: {reason}"),
            Self::InvalidPlaneTool(reason) => write!(f, "could not construct split tool: {reason}"),
            Self::NegativeSide(reason) => write!(f, "negative split side failed: {reason}"),
            Self::PositiveSide(reason) => write!(f, "positive split side failed: {reason}"),
            Self::DoesNotSplit => f.write_str("the plane does not divide the solid into two volumes"),
            Self::InvalidOutput(reason) => write!(f, "split output is invalid: {reason}"),
            Self::VolumeMismatch {
                input,
                outputs,
                relative_error,
            } => write!(
                f,
                "split does not conserve volume ({input} input, {outputs} output, relative error {relative_error})"
            ),
        }
    }
}

impl std::error::Error for PlaneSplitError {}

/// Split using the standard document tolerance policy.
pub fn split_solid_by_plane_operation(
    solid: &Solid,
    plane: &Plane,
) -> Result<OperationResult<PlaneSplitBodies>, PlaneSplitError> {
    split_solid_by_plane_operation_with_policy(solid, plane, &TolerancePolicy::STANDARD)
}

/// Split a solid into its negative- and positive-plane half-spaces.
///
/// The two finite tools are exact planar prisms sized from the input bounds.
/// Each Common is handled by the regular boolean operation, including strict
/// pcurve validation and face history. Both sides must succeed before this
/// function returns anything.
pub fn split_solid_by_plane_operation_with_policy(
    solid: &Solid,
    plane: &Plane,
    policy: &TolerancePolicy,
) -> Result<OperationResult<PlaneSplitBodies>, PlaneSplitError> {
    policy
        .validate()
        .map_err(|error| PlaneSplitError::InvalidTolerancePolicy(error.to_string()))?;
    solid
        .validate_strict_with_policy(policy)
        .map_err(|error| PlaneSplitError::InvalidOutput(error.to_string()))?;

    let (negative_tool, positive_tool) = half_space_tools(solid, plane, policy)?;
    let negative = boolean_bodies_operation_with_classes_policy_and_cancel(
        solid,
        &negative_tool,
        BooleanOp::Common,
        None,
        None,
        policy,
        &NeverCancelled,
    )
    .map_err(|error| PlaneSplitError::NegativeSide(error.to_string()))?;
    let positive = boolean_bodies_operation_with_classes_policy_and_cancel(
        solid,
        &positive_tool,
        BooleanOp::Common,
        None,
        None,
        policy,
        &NeverCancelled,
    )
    .map_err(|error| PlaneSplitError::PositiveSide(error.to_string()))?;

    if negative.value.bodies.is_empty() || positive.value.bodies.is_empty() {
        return Err(PlaneSplitError::DoesNotSplit);
    }
    let all = negative
        .value
        .bodies
        .iter()
        .chain(&positive.value.bodies)
        .cloned()
        .collect::<Vec<_>>();
    if all.iter().any(|body| {
        !body.is_watertight_with_policy(policy) || body.validate_strict_with_policy(policy).is_err()
    }) {
        return Err(PlaneSplitError::InvalidOutput(
            "one or more connected results failed strict validation".into(),
        ));
    }

    verify_side_classification(
        &negative.value.bodies,
        &positive.value.bodies,
        plane,
        policy,
    )?;
    verify_volume_conservation(solid, &all, policy)?;

    let value = PlaneSplitBodies {
        negative: negative.value.bodies,
        positive: positive.value.bodies,
        negative_face_history: negative.value.face_history,
        positive_face_history: positive.value.face_history,
    };
    let history = split_history(solid, &value);
    history
        .validate()
        .map_err(|error| PlaneSplitError::InvalidOutput(error.to_string()))?;
    let coverage = history.coverage_for_solids(&all);
    if !coverage.is_complete() {
        return Err(PlaneSplitError::InvalidOutput(format!(
            "topology history omitted {:?}",
            coverage.missing_results
        )));
    }

    let mut validation = ValidationReport::for_solid(&all[0], policy);
    for body in all.iter().skip(1) {
        let report = ValidationReport::for_solid(body, policy);
        validation.health.errors.extend(report.health.errors);
        validation.health.warnings.extend(report.health.warnings);
        validation.watertight &= report.watertight;
        validation.pcurves_complete &= report.pcurves_complete;
    }
    if !validation.is_valid() {
        return Err(PlaneSplitError::InvalidOutput(format!(
            "{:?}",
            validation.health
        )));
    }

    let mut recovery = RecoveryReport::default();
    recovery.actions.extend(negative.recovery.actions);
    recovery.actions.extend(positive.recovery.actions);
    let mut diagnostics = negative.diagnostics;
    diagnostics.extend(positive.diagnostics);
    Ok(OperationResult {
        value,
        history,
        diagnostics,
        recovery,
        validation,
    })
}

fn half_space_tools(
    solid: &Solid,
    plane: &Plane,
    policy: &TolerancePolicy,
) -> Result<(Solid, Solid), PlaneSplitError> {
    let (lo, hi) = solid.bounding_box().corners().ok_or_else(|| {
        PlaneSplitError::InvalidPlaneTool("the input solid has no finite bounds".into())
    })?;
    let origin = plane.location();
    let u = GeomVec::from_dir(plane.position().x_direction());
    let v = GeomVec::from_dir(plane.position().y_direction());
    let n = GeomVec::from_dir(plane.normal());
    let mut u_min = f64::INFINITY;
    let mut u_max = f64::NEG_INFINITY;
    let mut v_min = f64::INFINITY;
    let mut v_max = f64::NEG_INFINITY;
    let mut n_min = f64::INFINITY;
    let mut n_max = f64::NEG_INFINITY;
    for x in [lo.x(), hi.x()] {
        for y in [lo.y(), hi.y()] {
            for z in [lo.z(), hi.z()] {
                let offset = Pnt::new(x, y, z) - origin;
                let pu = offset.dot(&u);
                let pv = offset.dot(&v);
                let pn = offset.dot(&n);
                u_min = u_min.min(pu);
                u_max = u_max.max(pu);
                v_min = v_min.min(pv);
                v_max = v_max.max(pv);
                n_min = n_min.min(pn);
                n_max = n_max.max(pn);
            }
        }
    }
    if n_min >= -policy.classification || n_max <= policy.classification {
        return Err(PlaneSplitError::DoesNotSplit);
    }
    let diagonal =
        ((hi.x() - lo.x()).powi(2) + (hi.y() - lo.y()).powi(2) + (hi.z() - lo.z()).powi(2))
            .sqrt()
            .max(1.0);
    let margin = (diagonal * 0.25).max(policy.snap_max * 16.0);
    let corners = [
        plane_point(plane, u_min - margin, v_min - margin),
        plane_point(plane, u_max + margin, v_min - margin),
        plane_point(plane, u_max + margin, v_max + margin),
        plane_point(plane, u_min - margin, v_max + margin),
    ];
    let wire = Wire::from_edges(
        (0..4).map(|index| Edge::between_points(corners[index], corners[(index + 1) % 4])),
    );
    let face = planar_face_with_pcurves(*plane, Some(wire), Vec::new(), Orientation::Forward)
        .map_err(|error| PlaneSplitError::InvalidPlaneTool(error.to_string()))?;
    let positive_depth = n_max + margin;
    let negative_depth = -n_min + margin;
    let positive =
        crate::prism::prism_operation_with_policy(&face, n.multiplied(positive_depth), policy)
            .map_err(|error| PlaneSplitError::InvalidPlaneTool(error.to_string()))?
            .value;
    let negative =
        crate::prism::prism_operation_with_policy(&face, n.multiplied(-negative_depth), policy)
            .map_err(|error| PlaneSplitError::InvalidPlaneTool(error.to_string()))?
            .value;
    Ok((negative, positive))
}

fn plane_point(plane: &Plane, u: f64, v: f64) -> Pnt {
    plane.location()
        + GeomVec::from_dir(plane.position().x_direction()).multiplied(u)
        + GeomVec::from_dir(plane.position().y_direction()).multiplied(v)
}

fn verify_side_classification(
    negative: &[Solid],
    positive: &[Solid],
    plane: &Plane,
    policy: &TolerancePolicy,
) -> Result<(), PlaneSplitError> {
    let n = GeomVec::from_dir(plane.normal());
    let signed = |point: Pnt| (point - plane.location()).dot(&n);
    let tolerance = policy.classification.max(policy.pcurve_consistency) * 8.0;
    if negative
        .iter()
        .flat_map(Solid::vertices)
        .any(|vertex| signed(vertex.point()) > tolerance)
        || positive
            .iter()
            .flat_map(Solid::vertices)
            .any(|vertex| signed(vertex.point()) < -tolerance)
    {
        return Err(PlaneSplitError::InvalidOutput(
            "a result crossed to the wrong side of the splitting plane".into(),
        ));
    }
    Ok(())
}

fn verify_volume_conservation(
    input: &Solid,
    outputs: &[Solid],
    policy: &TolerancePolicy,
) -> Result<(), PlaneSplitError> {
    let volume = |solid: &Solid| -> Result<f64, PlaneSplitError> {
        let (lo, hi) = solid.bounding_box().corners().ok_or_else(|| {
            PlaneSplitError::InvalidOutput("a result has no finite bounds".into())
        })?;
        let diagonal =
            ((hi.x() - lo.x()).powi(2) + (hi.y() - lo.y()).powi(2) + (hi.z() - lo.z()).powi(2))
                .sqrt()
                .max(1.0);
        let chord = (diagonal * 1.0e-4)
            .max(policy.approximation * 4.0)
            .min(0.05);
        let mesh = openrcad_mesh::tessellate_checked_with_policy(solid, chord, 0.1, policy)
            .map_err(|error| PlaneSplitError::InvalidOutput(error.to_string()))?;
        openrcad_mesh::mass_properties(&mesh)
            .map(|properties| properties.volume)
            .ok_or_else(|| PlaneSplitError::InvalidOutput("could not measure solid volume".into()))
    };
    let input_volume = volume(input)?;
    let output_volume = outputs
        .iter()
        .map(volume)
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .sum::<f64>();
    let relative_error = (input_volume - output_volume).abs() / input_volume.max(policy.resolution);
    // This is a tessellation-derived conservation check. Its tolerance is
    // intentionally tighter than the display mesh but wider than geometric
    // classification so curved caps do not fail on chordal integration noise.
    if relative_error > 5.0e-3 {
        return Err(PlaneSplitError::VolumeMismatch {
            input: input_volume,
            outputs: output_volume,
            relative_error,
        });
    }
    Ok(())
}

fn split_history(source: &Solid, value: &PlaneSplitBodies) -> TopologyHistory {
    let bodies = value.solids().cloned().collect::<Vec<_>>();
    let mut history = TopologyHistory::default();
    let source_counts = topology_counts(core::slice::from_ref(source));
    let result_counts = topology_counts(&bodies);
    for kind in [
        TopologyKind::Vertex,
        TopologyKind::Edge,
        TopologyKind::Wire,
        TopologyKind::Shell,
        TopologyKind::Solid,
    ] {
        let sources = (0..source_counts[kind_index(kind)])
            .map(|index| InputTopologyRef::new(0, TopologyRef::new(kind, index)))
            .collect::<Vec<_>>();
        for index in 0..result_counts[kind_index(kind)] {
            history.generated(sources.iter().copied(), TopologyRef::new(kind, index));
        }
    }

    let mut result_face = 0usize;
    for per_body in value
        .negative_face_history
        .iter()
        .chain(&value.positive_face_history)
    {
        for source in &per_body.face_source {
            let result = TopologyRef::face(result_face);
            match source {
                Some(BooleanFaceSource::Object(index)) => {
                    history.modified(InputTopologyRef::face(0, *index), result)
                }
                // Half-space tool faces are split-generated caps, not document
                // inputs, so they deliberately carry no input operand lineage.
                Some(BooleanFaceSource::Tool(_)) | None => history.generated([], result),
            }
            result_face += 1;
        }
    }
    history
}

fn topology_counts(solids: &[Solid]) -> [usize; 6] {
    let vertices = solids.iter().map(Solid::vertex_count).sum();
    let edges = solids.iter().map(Solid::edge_count).sum();
    let wires = solids
        .iter()
        .flat_map(Solid::shells)
        .flat_map(|shell| shell.faces().to_vec())
        .map(|face| face.wires().len())
        .sum();
    let faces = solids.iter().map(Solid::face_count).sum();
    let shells = solids.iter().map(|solid| solid.shells().len()).sum();
    [vertices, edges, wires, faces, shells, solids.len()]
}

const fn kind_index(kind: TopologyKind) -> usize {
    match kind {
        TopologyKind::Vertex => 0,
        TopologyKind::Edge => 1,
        TopologyKind::Wire => 2,
        TopologyKind::Face => 3,
        TopologyKind::Shell => 4,
        TopologyKind::Solid => 5,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::{Ax2, Dir};

    #[test]
    fn centered_box_split_is_strict_and_conservative() {
        let source = openrcad_primitives::make_box_operation(&Pnt::origin(), 10.0, 8.0, 6.0)
            .unwrap()
            .value;
        let plane = Plane::from_point_normal(Pnt::new(5.0, 0.0, 0.0), Dir::dx());
        let result = split_solid_by_plane_operation(&source, &plane).unwrap();
        assert_eq!(result.value.negative.len(), 1);
        assert_eq!(result.value.positive.len(), 1);
        assert!(result.validation.is_valid());
        assert!(result
            .history
            .coverage_for_solids(&result.value.solids().cloned().collect::<Vec<_>>())
            .is_complete());
    }

    #[test]
    fn outside_plane_is_rejected() {
        let source = openrcad_primitives::make_box_operation(&Pnt::origin(), 10.0, 8.0, 6.0)
            .unwrap()
            .value;
        let plane = Plane::from_point_normal(Pnt::new(20.0, 0.0, 0.0), Dir::dx());
        assert_eq!(
            split_solid_by_plane_operation(&source, &plane),
            Err(PlaneSplitError::DoesNotSplit)
        );
    }

    #[test]
    fn axial_cylinder_split_keeps_periodic_boundaries_strict() {
        let source = openrcad_primitives::make_cylinder_operation(
            &Ax2::new(Pnt::origin(), Dir::dz()),
            3.0,
            6.0,
        )
        .unwrap()
        .value;
        let plane = Plane::from_point_normal(Pnt::origin(), Dir::dx());
        let result = split_solid_by_plane_operation(&source, &plane).unwrap();
        assert_eq!(result.value.negative.len(), 1);
        assert_eq!(result.value.positive.len(), 1);
        for body in result.value.solids() {
            body.validate_strict_with_policy(&TolerancePolicy::STANDARD)
                .unwrap();
            assert!(body.has_complete_pcurves());
            openrcad_mesh::tessellate_checked(body, 0.04, 0.25).unwrap();
        }
    }
}
