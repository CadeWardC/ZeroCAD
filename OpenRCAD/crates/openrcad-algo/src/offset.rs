use openrcad_foundation::{Dir, Pnt, ToleranceContext, TolerancePolicy, Trsf, Vec as GeomVec};
use openrcad_geom::{
    ConicalSurface, Curve, CylindricalSurface, GeomCurve, GeomSurface, OffsetSurface, Plane,
    SphericalSurface, Surface,
};
use openrcad_topo::{
    Edge, Face, InputTopologyRef, Solid, TopologyChange, TopologyHistory, TopologyKind,
    TopologyRef, Vertex, Wire,
};
use std::collections::HashMap;

use crate::blend::{detect_cylinder_with_policy, shell_cylinder_with_policy, BlendError};
use crate::sew::sew_shell_with_policy as sew_with_policy;

/// Dispatch path selected before building a Shell candidate. Keeping this
/// observable pins the primitive optimizations while mixed analytic networks
/// are added incrementally.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShellBuildPath {
    BoxPrimitive,
    CylinderPrimitive,
    PlanarNetwork,
    MixedAnalyticNetwork,
}

pub fn shell_build_path(solid: &Solid) -> ShellBuildPath {
    shell_build_path_with_policy(solid, &TolerancePolicy::STANDARD)
}

pub fn shell_build_path_with_policy(solid: &Solid, policy: &TolerancePolicy) -> ShellBuildPath {
    if detect_box(solid, policy).is_some() {
        return ShellBuildPath::BoxPrimitive;
    }
    if detect_cylinder_with_policy(solid, policy).is_some() {
        return ShellBuildPath::CylinderPrimitive;
    }
    if solid
        .shell()
        .faces()
        .iter()
        .all(|face| matches!(face.surface(), Some(GeomSurface::Plane(_))))
    {
        ShellBuildPath::PlanarNetwork
    } else {
        ShellBuildPath::MixedAnalyticNetwork
    }
}

/// Material-checked planar support normal used by the staged Shell builders.
/// CUT-derived faces can carry an inverted stored orientation, so Shell must
/// not derive its offset direction from `Face::orientation()` alone.
pub fn shell_planar_outward_normal_with_policy(
    solid: &Solid,
    face: &Face,
    policy: &TolerancePolicy,
) -> Result<Dir, crate::rolling_ball::RollingBallError> {
    crate::rolling_ball::planar_outward_normal_checked_with_policy(solid, face, policy)
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ShellWallThicknessEvidence {
    pub expected: f64,
    pub measured_min: f64,
    pub measured_max: f64,
    pub samples: usize,
    pub tolerance: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ShellWallThicknessError {
    NoOffsetSupports,
    NonFiniteMeasurement,
    OutsideTolerance(ShellWallThicknessEvidence),
}

impl core::fmt::Display for ShellWallThicknessError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::NoOffsetSupports => f.write_str("shell has no measurable offset supports"),
            Self::NonFiniteMeasurement => {
                f.write_str("shell wall-thickness measurement was non-finite")
            }
            Self::OutsideTolerance(evidence) => write!(
                f,
                "shell wall thickness [{}, {}] differs from {} by more than {}",
                evidence.measured_min, evidence.measured_max, evidence.expected, evidence.tolerance
            ),
        }
    }
}

impl std::error::Error for ShellWallThicknessError {}

/// Measure actual point-to-point separation between every explicit offset
/// support and its analytic base. The sampling distances come from the
/// operation-local tolerance context; no unit-scale floor is introduced.
pub fn measure_shell_wall_thickness(
    solid: &Solid,
    expected: f64,
    policy: &TolerancePolicy,
) -> Result<ShellWallThicknessEvidence, ShellWallThicknessError> {
    let context =
        ToleranceContext::derive(policy, &[solid.bounding_box()], Some(expected.abs()), 1.0)
            .map_err(|_| ShellWallThicknessError::NonFiniteMeasurement)?;
    let fallback_span = context.local_feature_size;
    let sample_axis = |min: f64, max: f64| {
        if min.is_finite() && max.is_finite() && max >= min {
            [min, min + (max - min) * 0.5, max]
        } else {
            [-fallback_span * 0.25, 0.0, fallback_span * 0.25]
        }
    };

    let mut measured_min = f64::INFINITY;
    let mut measured_max = f64::NEG_INFINITY;
    let mut samples = 0usize;
    for face in solid.shell().faces() {
        let Some(GeomSurface::Offset(offset)) = face.surface() else {
            continue;
        };
        let (u_min, u_max, v_min, v_max) = offset.bounds();
        for u in sample_axis(u_min, u_max) {
            for v in sample_axis(v_min, v_max) {
                let distance = offset.base.point(u, v).distance(&offset.point(u, v));
                if !distance.is_finite() {
                    return Err(ShellWallThicknessError::NonFiniteMeasurement);
                }
                measured_min = measured_min.min(distance);
                measured_max = measured_max.max(distance);
                samples += 1;
            }
        }
    }
    if samples == 0 {
        return Err(ShellWallThicknessError::NoOffsetSupports);
    }

    let tolerance = context.policy.classification.max(context.convergence) * 8.0;
    let evidence = ShellWallThicknessEvidence {
        expected,
        measured_min,
        measured_max,
        samples,
        tolerance,
    };
    if (measured_min - expected.abs()).abs() > tolerance
        || (measured_max - expected.abs()).abs() > tolerance
    {
        return Err(ShellWallThicknessError::OutsideTolerance(evidence));
    }
    Ok(evidence)
}

/// Shell a solid by `thickness`, removing `open_faces`.
#[deprecated(note = "use shell_solid_with_policy")]
pub fn shell_solid(
    solid: &Solid,
    thickness: f64,
    open_faces: &[Face],
) -> Result<Solid, BlendError> {
    shell_solid_with_policy(solid, thickness, open_faces, &TolerancePolicy::STANDARD)
}

pub fn shell_solid_with_policy(
    solid: &Solid,
    thickness: f64,
    open_faces: &[Face],
    policy: &TolerancePolicy,
) -> Result<Solid, BlendError> {
    policy
        .validate()
        .map_err(|error| BlendError::InvalidTolerancePolicy(error.to_string()))?;
    let tolerance_context =
        ToleranceContext::derive(policy, &[solid.bounding_box()], Some(thickness.abs()), 1.0)
            .map_err(|error| BlendError::InvalidTolerancePolicy(error.to_string()))?;
    let policy = &tolerance_context.policy;
    if thickness.abs() <= policy.linear {
        return Ok(solid.clone());
    }
    match shell_build_path_with_policy(solid, policy) {
        ShellBuildPath::BoxPrimitive => {
            let (p0, ex, ey, ez, dx, dy, dz) =
                detect_box(solid, policy).expect("box path was classified by the same detector");
            return Ok(shell_box(
                p0, ex, ey, ez, dx, dy, dz, thickness, open_faces, policy,
            ));
        }
        ShellBuildPath::CylinderPrimitive => {
            let cylinder = detect_cylinder_with_policy(solid, policy)
                .expect("cylinder path was classified by the same detector");
            return shell_cylinder_with_policy(&cylinder, thickness, open_faces, policy);
        }
        ShellBuildPath::PlanarNetwork => {}
        ShellBuildPath::MixedAnalyticNetwork => {
            return shell_analytic_general(solid, thickness, open_faces, policy);
        }
    }
    shell_planar_general(solid, thickness, open_faces, policy)
}

/// Build complete Shell-specific history rather than accepting conservative
/// all-to-all unary lineage. Removed faces are Deleted, retained supports are
/// Modified, offset/rim faces are Generated, and lower-dimensional topology
/// retains conservative complete coverage until the staged builders own it.
pub(crate) fn shell_topology_history(
    source: &Solid,
    result: &Solid,
    open_faces: &[Face],
    policy: &TolerancePolicy,
) -> TopologyHistory {
    let source_faces = source.shell().faces();
    let result_faces = result.shell().faces();
    let removed: Vec<bool> = source_faces
        .iter()
        .map(|face| open_faces.iter().any(|open| open == face))
        .collect();

    let mut history = TopologyHistory::conservative_unary(source, result);
    history.changes.retain(|change| match change {
        TopologyChange::Generated { result, .. }
        | TopologyChange::Modified { result, .. }
        | TopologyChange::Merged { result, .. } => result.kind != TopologyKind::Face,
        TopologyChange::Split { results, .. } => results
            .iter()
            .all(|result| result.kind != TopologyKind::Face),
        TopologyChange::Deleted { source } => source.entity.kind != TopologyKind::Face,
    });

    let same_domain = |first: &GeomSurface, second: &GeomSurface| {
        crate::boolean::surfaces_are_same_domain(
            first,
            second,
            policy.classification.max(policy.intersection),
        )
    };
    let mut source_has_descendant = vec![false; source_faces.len()];
    for (result_index, result_face) in result_faces.iter().enumerate() {
        let result_ref = TopologyRef::face(result_index);
        let result_surface = result_face.surface();

        let retained = result_surface.and_then(|surface| {
            source_faces
                .iter()
                .enumerate()
                .position(|(index, source_face)| {
                    !removed[index]
                        && source_face
                            .surface()
                            .is_some_and(|source_surface| same_domain(surface, source_surface))
                })
        });
        if let Some(source_index) = retained {
            history.modified(InputTopologyRef::face(0, source_index), result_ref);
            source_has_descendant[source_index] = true;
            continue;
        }

        let offset_source = match result_surface {
            Some(GeomSurface::Offset(offset)) => {
                source_faces
                    .iter()
                    .enumerate()
                    .position(|(_, source_face)| {
                        source_face
                            .surface()
                            .is_some_and(|surface| same_domain(&offset.base, surface))
                    })
            }
            _ => None,
        };
        if let Some(source_index) = offset_source {
            history.generated([InputTopologyRef::face(0, source_index)], result_ref);
            source_has_descendant[source_index] = true;
            continue;
        }

        let rim_sources: Vec<_> = source_faces
            .iter()
            .enumerate()
            .filter(|(index, source_face)| {
                removed[*index]
                    && result_surface.zip(source_face.surface()).is_some_and(
                        |(result_surface, source_surface)| {
                            same_domain(result_surface, source_surface)
                        },
                    )
            })
            .map(|(index, _)| InputTopologyRef::face(0, index))
            .collect();
        let sources = if rim_sources.is_empty() {
            (0..source_faces.len())
                .map(|index| InputTopologyRef::face(0, index))
                .collect()
        } else {
            rim_sources
        };
        for source in &sources {
            source_has_descendant[source.entity.index] = true;
        }
        history.generated(sources, result_ref);
    }

    for (source_index, was_removed) in removed.into_iter().enumerate() {
        if was_removed || !source_has_descendant[source_index] {
            history.deleted(InputTopologyRef::face(0, source_index));
        }
    }
    history
}

#[derive(Clone)]
struct AnalyticShellSupport {
    display: GeomSurface,
    analytic: GeomSurface,
    checked_outward: Dir,
    base_normal: Dir,
}

fn checked_face_normal(
    solid: &Solid,
    face: &Face,
    surface: &GeomSurface,
    context: &ToleranceContext,
) -> Result<(Pnt, Dir, Dir), BlendError> {
    let sample = crate::boolean::point_on_face(face);
    let (u, v) = crate::intersect::uv_of(surface, &sample);
    let (_, du, dv) = surface.d1(u, v);
    let base_normal = du
        .cross(&dv)
        .normalized()
        .ok_or(BlendError::UnsupportedShape)?;
    let mut candidate = base_normal;
    if face.orientation() == openrcad_topo::Orientation::Reversed {
        candidate = candidate.reversed();
    }
    let maximum_probe = (context.local_feature_size * 0.05).max(context.arithmetic_floor);
    let probe = context
        .convergence
        .max(context.policy.classification * 4.0)
        .max(context.arithmetic_floor)
        .min(maximum_probe);
    let positive = sample + GeomVec::from_dir(base_normal) * probe;
    let negative = sample - GeomVec::from_dir(base_normal) * probe;
    match (
        crate::boolean::point_in_solid(&positive, solid),
        crate::boolean::point_in_solid(&negative, solid),
    ) {
        (false, true) => candidate = base_normal,
        (true, false) => candidate = base_normal.reversed(),
        _ => {
            let outward = sample + GeomVec::from_dir(candidate) * probe;
            let inward = sample - GeomVec::from_dir(candidate) * probe;
            if crate::boolean::point_in_solid(&outward, solid)
                && !crate::boolean::point_in_solid(&inward, solid)
            {
                candidate = candidate.reversed();
            }
        }
    }
    Ok((sample, candidate, base_normal))
}

fn analytic_offset_support(
    solid: &Solid,
    face: &Face,
    thickness: f64,
    is_open: bool,
    context: &ToleranceContext,
) -> Result<AnalyticShellSupport, BlendError> {
    let surface = face
        .surface()
        .cloned()
        .ok_or(BlendError::UnsupportedShape)?;
    let (_, checked_outward, base_normal) = checked_face_normal(solid, face, &surface, context)?;
    if is_open {
        return Ok(AnalyticShellSupport {
            display: surface.clone(),
            analytic: surface,
            checked_outward,
            base_normal,
        });
    }

    let signed_distance = if checked_outward.dot(&base_normal) >= 0.0 {
        -thickness
    } else {
        thickness
    };
    let analytic = match &surface {
        GeomSurface::Plane(plane) => GeomSurface::plane(Plane::from_point_normal(
            plane.location() + GeomVec::from_dir(base_normal) * signed_distance,
            plane.normal(),
        )),
        GeomSurface::Cylinder(cylinder) => {
            let radius = cylinder.radius() + signed_distance;
            if radius <= context.policy.linear {
                return Err(BlendError::ParameterTooLarge {
                    requested: thickness,
                    max: cylinder.radius(),
                });
            }
            GeomSurface::cylinder(CylindricalSurface::new(cylinder.position(), radius))
        }
        GeomSurface::Cone(cone) => {
            let cosine = cone.semi_angle().cos();
            if cosine.abs() <= context.policy.angular {
                return Err(BlendError::UnsupportedShape);
            }
            let axis = cone.position();
            let direction = GeomVec::from_dir(axis.direction());
            // Wave 4's supported conical solids are bounded by analytic edges
            // whose relevant radial extrema occur at boundary vertices. This
            // is intentionally not a general trimmed-surface extrema solver:
            // a future degenerate or nonstandard trim whose interior can
            // approach the apex more closely must add an analytic extrema
            // query and fail closed until that query is available.
            let minimum_trimmed_radius = face
                .wires()
                .into_iter()
                .flat_map(|wire| wire.edges().into_iter())
                .flat_map(|edge| [edge.source().point(), edge.target().point()])
                .map(|point| {
                    let delta = point - axis.location();
                    (delta - direction * delta.dot(&direction)).magnitude()
                })
                .fold(f64::INFINITY, f64::min);
            let minimum_offset_radius = minimum_trimmed_radius + signed_distance * cosine;
            if !minimum_offset_radius.is_finite() || minimum_offset_radius <= context.policy.linear
            {
                return Err(BlendError::ParameterTooLarge {
                    requested: thickness,
                    max: minimum_trimmed_radius / cosine.abs(),
                });
            }
            let radius = cone.ref_radius() + signed_distance / cosine;
            if radius <= context.policy.linear {
                return Err(BlendError::ParameterTooLarge {
                    requested: thickness,
                    max: cone.ref_radius() * cosine.abs(),
                });
            }
            GeomSurface::cone(ConicalSurface::new(
                cone.position(),
                radius,
                cone.semi_angle(),
            ))
        }
        GeomSurface::Sphere(sphere) => {
            let radius = sphere.radius() + signed_distance;
            if radius <= context.policy.linear {
                return Err(BlendError::ParameterTooLarge {
                    requested: thickness,
                    max: sphere.radius(),
                });
            }
            GeomSurface::sphere(SphericalSurface::new(sphere.position(), radius))
        }
        _ => return Err(BlendError::UnsupportedShape),
    };
    Ok(AnalyticShellSupport {
        display: GeomSurface::offset(OffsetSurface::new(surface, signed_distance)),
        analytic,
        checked_outward,
        base_normal,
    })
}

fn support_residual_gradient(surface: &GeomSurface, point: Pnt) -> Option<(f64, GeomVec)> {
    match surface {
        GeomSurface::Plane(plane) => {
            let normal = GeomVec::from_dir(plane.normal());
            Some(((point - plane.location()).dot(&normal), normal))
        }
        GeomSurface::Cylinder(cylinder) => {
            let axis = cylinder.position();
            let direction = GeomVec::from_dir(axis.direction());
            let delta = point - axis.location();
            let radial = delta - direction * delta.dot(&direction);
            let magnitude = radial.magnitude();
            let gradient = radial.normalized().map(GeomVec::from_dir)?;
            Some((magnitude - cylinder.radius(), gradient))
        }
        GeomSurface::Cone(cone) => {
            let axis = cone.position();
            let direction = GeomVec::from_dir(axis.direction());
            let delta = point - axis.location();
            let axial = delta.dot(&direction);
            let radial = delta - direction * axial;
            let magnitude = radial.magnitude();
            let radial_direction = radial.normalized().map(GeomVec::from_dir)?;
            let cosine = cone.semi_angle().cos();
            let sine = cone.semi_angle().sin();
            let residual =
                (magnitude - cone.ref_radius() - axial * cone.semi_angle().tan()) * cosine;
            Some((residual, radial_direction * cosine - direction * sine))
        }
        GeomSurface::Sphere(sphere) => {
            let radial = point - sphere.center();
            let magnitude = radial.magnitude();
            let gradient = radial.normalized().map(GeomVec::from_dir)?;
            Some((magnitude - sphere.radius(), gradient))
        }
        _ => None,
    }
}

fn independent_support_rows(
    supports: &[&GeomSurface],
    point: Pnt,
    original: Pnt,
    angular_tolerance: f64,
) -> Option<([[f64; 3]; 3], [f64; 3])> {
    let evaluated: Vec<_> = supports
        .iter()
        .map(|support| support_residual_gradient(support, point))
        .collect::<Option<_>>()?;
    if evaluated.len() < 2 {
        return None;
    }

    let mut best: Option<(f64, [usize; 3])> = None;
    if evaluated.len() >= 3 {
        for first in 0..evaluated.len() - 2 {
            for second in first + 1..evaluated.len() - 1 {
                for third in second + 1..evaluated.len() {
                    let rows = [
                        vec_row(evaluated[first].1),
                        vec_row(evaluated[second].1),
                        vec_row(evaluated[third].1),
                    ];
                    let determinant = det3(&rows).abs();
                    if best.is_none_or(|(current, _)| determinant > current) {
                        best = Some((determinant, [first, second, third]));
                    }
                }
            }
        }
    }
    if let Some((determinant, indices)) = best {
        if determinant > angular_tolerance {
            let rows = indices.map(|index| vec_row(evaluated[index].1));
            let rhs = indices.map(|index| -evaluated[index].0);
            return Some((rows, rhs));
        }
    }

    let first = evaluated[0];
    let second = evaluated.iter().copied().skip(1).max_by(|left, right| {
        first
            .1
            .cross(&left.1)
            .magnitude()
            .total_cmp(&first.1.cross(&right.1).magnitude())
    })?;
    let tangent = first.1.cross(&second.1).normalized()?;
    let tangent = GeomVec::from_dir(tangent);
    let rows = [vec_row(first.1), vec_row(second.1), vec_row(tangent)];
    let rhs = [-first.0, -second.0, -(point - original).dot(&tangent)];
    (det3(&rows).abs() > angular_tolerance).then_some((rows, rhs))
}

fn vec_row(vector: GeomVec) -> [f64; 3] {
    [vector.x(), vector.y(), vector.z()]
}

// A valid local offset intersection can move more than one wall thickness near
// shallow support angles. Twenty thicknesses is a deliberately loose,
// scale-relative divergence guard for the Newton solve, not an accuracy
// tolerance; policy-derived residual and validation gates remain authoritative.
// The Wave 4 scale/aspect/far-origin sweeps pin this heuristic's behavior.
const MAX_VERTEX_TRAVEL_THICKNESSES: f64 = 20.0;

fn remap_analytic_vertex(
    original: Pnt,
    supports: &[&GeomSurface],
    thickness: f64,
    context: &ToleranceContext,
) -> Result<Pnt, BlendError> {
    let mut point = original;
    if supports.len() == 1 {
        for _ in 0..8 {
            let (residual, gradient) = support_residual_gradient(supports[0], point)
                .ok_or(BlendError::UnsupportedShape)?;
            point = point - gradient * residual;
            if residual.abs() <= context.convergence {
                break;
            }
        }
        if original.distance(&point) > thickness.abs() * MAX_VERTEX_TRAVEL_THICKNESSES {
            return Err(BlendError::ParameterTooLarge {
                requested: thickness,
                max: original.distance(&point) / MAX_VERTEX_TRAVEL_THICKNESSES,
            });
        }
        return Ok(point);
    }
    for _ in 0..24 {
        let Some((rows, rhs)) =
            independent_support_rows(supports, point, original, context.policy.angular)
        else {
            return Err(BlendError::UnsupportedShape);
        };
        let determinant = det3(&rows);
        if determinant.abs() <= context.policy.angular {
            return Err(BlendError::UnsupportedShape);
        }
        let delta = solve3(&rows, &rhs, determinant);
        let step = GeomVec::new(delta[0], delta[1], delta[2]);
        point = point + step;
        if step.magnitude() <= context.convergence {
            break;
        }
    }
    let residual = supports
        .iter()
        .filter_map(|support| support_residual_gradient(support, point))
        .map(|(residual, _)| residual.abs())
        .fold(0.0, f64::max);
    if residual > context.policy.classification * 8.0 {
        return Err(BlendError::UnsupportedShape);
    }
    if original.distance(&point) > thickness.abs() * MAX_VERTEX_TRAVEL_THICKNESSES {
        return Err(BlendError::ParameterTooLarge {
            requested: thickness,
            max: original.distance(&point) / MAX_VERTEX_TRAVEL_THICKNESSES,
        });
    }
    Ok(point)
}

fn distinct_support_indices(
    indices: &[usize],
    supports: &[AnalyticShellSupport],
    policy: &TolerancePolicy,
) -> Vec<usize> {
    let mut distinct: Vec<usize> = Vec::new();
    for &index in indices {
        if distinct.iter().all(|&existing| {
            !crate::boolean::surfaces_are_same_domain(
                &supports[index].analytic,
                &supports[existing].analytic,
                policy.classification.max(policy.intersection),
            )
        }) {
            distinct.push(index);
        }
    }
    distinct
}

/// Stages 4A/4B mixed analytic shell. It preserves the input face graph,
/// offsets each analytic support, re-solves shared vertices against those
/// supports, rebuilds intersection edges, and constructs opening rims before
/// the normal strict operation boundary repairs pcurves and validates atomically.
fn shell_analytic_general(
    solid: &Solid,
    thickness: f64,
    open_faces: &[Face],
    policy: &TolerancePolicy,
) -> Result<Solid, BlendError> {
    if open_faces.is_empty() {
        return Err(BlendError::UnsupportedShape);
    }
    let faces = solid.shell().faces();
    let context =
        ToleranceContext::derive(policy, &[solid.bounding_box()], Some(thickness.abs()), 1.0)
            .map_err(|error| BlendError::InvalidTolerancePolicy(error.to_string()))?;
    let quantum = context.quantization;
    let grid = 1.0 / quantum;
    let quant = |point: &Pnt| {
        (
            (point.x() * grid).round() as i64,
            (point.y() * grid).round() as i64,
            (point.z() * grid).round() as i64,
        )
    };
    let is_open: Vec<bool> = faces
        .iter()
        .map(|face| open_faces.iter().any(|open| open == face))
        .collect();
    if is_open.iter().enumerate().any(|(index, open)| {
        *open && !matches!(faces[index].surface(), Some(GeomSurface::Plane(_)))
    }) {
        return Err(BlendError::UnsupportedShape);
    }
    let supports: Vec<_> = faces
        .iter()
        .enumerate()
        .map(|(index, face)| {
            analytic_offset_support(solid, face, thickness, is_open[index], &context)
        })
        .collect::<Result<_, _>>()?;

    let mut vertex_faces: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
    let mut vertex_points: HashMap<(i64, i64, i64), Pnt> = HashMap::new();
    let mut edge_faces: HashMap<((i64, i64, i64), (i64, i64, i64)), Vec<usize>> = HashMap::new();
    let edge_key = |edge: &Edge| {
        let first = quant(&edge.source().point());
        let second = quant(&edge.target().point());
        if first <= second {
            (first, second)
        } else {
            (second, first)
        }
    };
    for (face_index, face) in faces.iter().enumerate() {
        for wire in face.wires() {
            for edge in wire.edges() {
                for point in [edge.source().point(), edge.target().point()] {
                    let key = quant(&point);
                    vertex_points.entry(key).or_insert(point);
                    let adjacent = vertex_faces.entry(key).or_default();
                    if !adjacent.contains(&face_index) {
                        adjacent.push(face_index);
                    }
                }
                let adjacent = edge_faces.entry(edge_key(&edge)).or_default();
                if !adjacent.contains(&face_index) {
                    adjacent.push(face_index);
                }
            }
        }
    }

    let mut offset_points = HashMap::new();
    for (key, adjacent) in &vertex_faces {
        let distinct = distinct_support_indices(adjacent, &supports, &context.policy);
        let analytic: Vec<_> = distinct
            .iter()
            .map(|index| &supports[*index].analytic)
            .collect();
        let original = vertex_points[key];
        offset_points.insert(
            *key,
            remap_analytic_vertex(original, &analytic, thickness, &context)?,
        );
    }

    let rebuild_edge = |edge: &Edge| -> Result<Edge, BlendError> {
        let start = offset_points[&quant(&edge.source().point())];
        let end = offset_points[&quant(&edge.target().point())];
        if matches!(edge.curve(), None | Some(GeomCurve::Line(_))) {
            return Ok(Edge::between_points(start, end));
        }
        let adjacent = edge_faces
            .get(&edge_key(edge))
            .ok_or(BlendError::UnsupportedShape)?;
        let distinct = distinct_support_indices(adjacent, &supports, &context.policy);
        if distinct.len() == 1 {
            let support = &supports[distinct[0]];
            if let (
                GeomSurface::Offset(offset),
                GeomSurface::Sphere(analytic_sphere),
                Some(curve),
            ) = (&support.display, &support.analytic, edge.curve())
            {
                if let GeomSurface::Sphere(base_sphere) = offset.base.as_ref() {
                    let factor = analytic_sphere.radius() / base_sphere.radius();
                    let mut transform = Trsf::new();
                    transform.set_scale(&base_sphere.center(), factor);
                    let transformed = curve.transformed(&transform);
                    let (source_first, source_last) =
                        if edge.orientation() == openrcad_topo::Orientation::Reversed {
                            (edge.last(), edge.first())
                        } else {
                            (edge.first(), edge.last())
                        };
                    let (curve_min, curve_max) = transformed.bounds();
                    let extent = context.model_scale.max(context.local_feature_size) * 2.0;
                    let (search_min, search_max) = if transformed.is_periodic() {
                        let period = transformed.period();
                        (
                            source_first.min(source_last) - period,
                            source_first.max(source_last) + period,
                        )
                    } else if curve_min.is_finite() && curve_max.is_finite() {
                        (curve_min, curve_max)
                    } else {
                        (-extent, extent)
                    };
                    let project = |point: Pnt, near: f64| match &transformed {
                        GeomCurve::Circle(circle) => {
                            let delta = point - circle.center();
                            let x = delta.dot(&GeomVec::from_dir(circle.position().x_direction()));
                            let y = delta.dot(&GeomVec::from_dir(circle.position().y_direction()));
                            let angle = y.atan2(x).rem_euclid(core::f64::consts::TAU);
                            angle
                                + ((near - angle) / core::f64::consts::TAU).round()
                                    * core::f64::consts::TAU
                        }
                        _ => crate::boolean::project_point_on_curve(
                            &point,
                            &transformed,
                            search_min,
                            search_max,
                        ),
                    };
                    let first = project(start, source_first);
                    let mut last = project(end, source_last);
                    if transformed.is_periodic() {
                        let period = transformed.period();
                        let direction = source_last - source_first;
                        last += ((first + direction - last) / period).round() * period;
                        if (last - first).abs() <= context.policy.angular {
                            last += period.copysign(direction);
                        }
                    }
                    return Ok(Edge::new_with_tolerance(
                        Some(transformed),
                        first,
                        last,
                        Vertex::new(start),
                        Vertex::new(end),
                        context.policy.linear,
                    ));
                }
            }
            return Err(BlendError::UnsupportedShape);
        }
        if distinct.len() < 2 {
            return Err(BlendError::UnsupportedShape);
        }
        let candidates = crate::intersect::surface_surface(
            &supports[distinct[0]].analytic,
            &supports[distinct[1]].analytic,
            context.policy.intersection,
        );
        let original_midpoint = edge
            .curve()
            .map(|curve| curve.point((edge.first() + edge.last()) * 0.5))
            .unwrap_or_else(|| edge.source().point().midpoint(&edge.target().point()));
        let mut best: Option<(f64, GeomCurve, f64, f64)> = None;
        for curve in candidates {
            let (curve_min, curve_max) = curve.bounds();
            let extent = context.model_scale.max(context.local_feature_size) * 2.0;
            let (search_min, search_max) = if curve.is_periodic() {
                let period = curve.period();
                (edge.first() - period, edge.last() + period)
            } else if curve_min.is_finite() && curve_max.is_finite() {
                (curve_min, curve_max)
            } else {
                (-extent, extent)
            };
            let start_parameter =
                crate::boolean::project_point_on_curve(&start, &curve, search_min, search_max);
            let mut end_parameter =
                crate::boolean::project_point_on_curve(&end, &curve, search_min, search_max);
            if curve.is_periodic() {
                let period = curve.period();
                let direction = if edge.orientation() == openrcad_topo::Orientation::Reversed {
                    edge.first() - edge.last()
                } else {
                    edge.last() - edge.first()
                };
                let target_span = direction.abs();
                end_parameter +=
                    ((start_parameter + direction - end_parameter) / period).round() * period;
                if (end_parameter - start_parameter).abs() <= context.policy.angular {
                    end_parameter += period.copysign(direction);
                }
                if target_span < period && (end_parameter - start_parameter).abs() > period * 0.75 {
                    end_parameter -= period.copysign(end_parameter - start_parameter);
                }
            }
            let start_error = curve.point(start_parameter).distance(&start);
            let end_error = curve.point(end_parameter).distance(&end);
            if start_error > context.policy.classification * 8.0
                || end_error > context.policy.classification * 8.0
            {
                continue;
            }
            let midpoint = curve.point((start_parameter + end_parameter) * 0.5);
            let score = midpoint.distance(&original_midpoint) + start_error + end_error;
            if best.as_ref().is_none_or(|(current, ..)| score < *current) {
                best = Some((score, curve, start_parameter, end_parameter));
            }
        }
        let (_, curve, first, last) = best.ok_or(BlendError::UnsupportedShape)?;
        Ok(Edge::new_with_tolerance(
            Some(curve),
            first,
            last,
            Vertex::new(start),
            Vertex::new(end),
            context.policy.linear,
        ))
    };

    let rebuild_wire = |wire: &Wire| -> Result<Wire, BlendError> {
        wire.edges()
            .iter()
            .map(&rebuild_edge)
            .collect::<Result<Vec<_>, _>>()
            .map(Wire::from_edges)
    };

    let mut result = Vec::new();
    for (face_index, face) in faces.iter().enumerate() {
        if is_open[face_index] {
            let rim_surface = face
                .surface()
                .cloned()
                .ok_or(BlendError::UnsupportedShape)?;
            for wire in face.wires() {
                for edge in wire.edges() {
                    let inner = rebuild_edge(&edge)?;
                    let outer_start = edge.source().point();
                    let outer_end = edge.target().point();
                    let inner_start = inner.source().point();
                    let inner_end = inner.target().point();
                    result.push(Face::new(
                        Some(rim_surface.clone()),
                        Wire::from_edges([
                            edge.clone(),
                            Edge::between_points(outer_end, inner_end),
                            inner.reversed(),
                            Edge::between_points(inner_start, outer_start),
                        ]),
                    ));
                }
            }
            continue;
        }
        result.push(face.clone());
        let outer = face
            .outer_wire()
            .map(|wire| rebuild_wire(&wire))
            .transpose()?
            .ok_or(BlendError::UnsupportedShape)?;
        let inners = face
            .inner_wires()
            .iter()
            .map(&rebuild_wire)
            .collect::<Result<Vec<_>, _>>()?;
        let desired = supports[face_index].checked_outward.reversed();
        let orientation = if desired.dot(&supports[face_index].base_normal) >= 0.0 {
            openrcad_topo::Orientation::Forward
        } else {
            openrcad_topo::Orientation::Reversed
        };
        result.push(Face::with_wires(
            Some(supports[face_index].display.clone()),
            Some(outer),
            inners,
            orientation,
        ));
    }

    let shell = sew_with_policy(&result, &context.policy)
        .map_err(|error| BlendError::InvalidTolerancePolicy(error.to_string()))?;
    let candidate = Solid::new(shell);
    if !candidate.is_watertight_with_policy(&context.policy) {
        return Err(BlendError::UnsupportedShape);
    }
    Ok(candidate)
}

/// General shell for arbitrary PLANAR-faced solids with straight edges (any
/// extruded profile, wedges, chamfered blocks — everything the box/cylinder
/// recognisers above don't match). Each face's plane is offset inward by
/// `thickness`; every vertex is re-solved as the intersection of its three
/// adjacent offset planes; the inner shell reuses the outer topology with the
/// remapped vertices; removed faces get rim quads bridging outer → inner.
///
/// Limits (fail-loud with [`BlendError::UnsupportedShape`]): curved faces or
/// curved edges, vertices with ≠ 3 distinct adjacent face planes, and
/// thickness large enough to invert a vertex (offset planes meeting on the
/// wrong side). `open_faces` must name at least one face — a fully closed
/// hollow (a void) needs two-shell solids, which the arena represents but
/// this path does not emit yet.
fn shell_planar_general(
    solid: &Solid,
    thickness: f64,
    open_faces: &[Face],
    policy: &TolerancePolicy,
) -> Result<Solid, BlendError> {
    if open_faces.is_empty() {
        return Err(BlendError::UnsupportedShape);
    }
    let faces = solid.shell().faces();
    let context =
        ToleranceContext::derive(policy, &[solid.bounding_box()], Some(thickness.abs()), 1.0)
            .map_err(|error| BlendError::InvalidTolerancePolicy(error.to_string()))?;
    let quantum = context.quantization;
    let grid = 1.0 / quantum;
    let quant = |p: &Pnt| {
        (
            (p.x() * grid).round() as i64,
            (p.y() * grid).round() as i64,
            (p.z() * grid).round() as i64,
        )
    };

    // Effective OUTWARD plane per face (normal ⊗ orientation), keyed by index.
    let mut planes: Vec<(Pnt, Dir)> = Vec::with_capacity(faces.len());
    for face in &faces {
        let Some(GeomSurface::Plane(pl)) = face.surface() else {
            return Err(BlendError::UnsupportedShape);
        };
        let mut n = pl.normal();
        if face.orientation() == openrcad_topo::Orientation::Reversed {
            n = n.reversed();
        }
        // Anchor the plane at an actual boundary point (the stored plane
        // location may sit anywhere).
        let anchor = face
            .outer_wire()
            .and_then(|w| w.edges().first().map(|e| e.source().point()))
            .ok_or(BlendError::UnsupportedShape)?;
        planes.push((anchor, n));
    }

    // Vertex → set of adjacent face indices (by quantized position).
    let mut vertex_faces: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
    for (fi, face) in faces.iter().enumerate() {
        for wire in face.wires() {
            for edge in wire.edges() {
                if edge.curve().is_some() && !matches!(edge.curve(), Some(GeomCurve::Line(_))) {
                    return Err(BlendError::UnsupportedShape);
                }
                for p in [edge.source().point(), edge.target().point()] {
                    let entry = vertex_faces.entry(quant(&p)).or_default();
                    if !entry.contains(&fi) {
                        entry.push(fi);
                    }
                }
            }
        }
    }

    // Offset every vertex: intersection of its three adjacent offset planes.
    // (Coplanar duplicates — split faces on one plane — collapse first.)
    let mut offset_of: HashMap<(i64, i64, i64), Pnt> = HashMap::new();
    for (key, fis) in &vertex_faces {
        let mut distinct: Vec<usize> = Vec::new();
        for &fi in fis {
            let (_, n) = planes[fi];
            let dup = distinct.iter().any(|&fj| {
                let (pj, nj) = planes[fj];
                n.dot(&nj).abs() > 1.0 - context.policy.angular
                    && ((planes[fi].0 - pj).dot(&GeomVec::from_dir(nj))).abs()
                        < context.policy.classification
            });
            if !dup {
                distinct.push(fi);
            }
        }
        if distinct.len() != 3 {
            return Err(BlendError::UnsupportedShape);
        }
        let mut rows = [[0.0f64; 3]; 3];
        let mut rhs = [0.0f64; 3];
        for (r, &fi) in distinct.iter().enumerate() {
            let (p, n) = planes[fi];
            rows[r] = [n.x(), n.y(), n.z()];
            // Inward offset: the plane moves −thickness along its OUTWARD normal.
            rhs[r] = n.x() * p.x() + n.y() * p.y() + n.z() * p.z() - thickness;
        }
        let det = det3(&rows);
        if det.abs() < context.policy.angular {
            return Err(BlendError::UnsupportedShape);
        }
        let x = solve3(&rows, &rhs, det);
        offset_of.insert(*key, Pnt::new(x[0], x[1], x[2]));
    }

    // Guard against inversion: every offset vertex must have moved less than
    // a few thicknesses (a vertex shooting far away means the offset planes
    // meet on the wrong side — thickness ≥ the local feature size).
    for (key, off) in &offset_of {
        let orig = Pnt::new(
            key.0 as f64 * quantum,
            key.1 as f64 * quantum,
            key.2 as f64 * quantum,
        );
        if orig.distance(off) > thickness.abs() * 10.0 {
            return Err(BlendError::ParameterTooLarge {
                requested: thickness,
                max: orig.distance(off) / 10.0,
            });
        }
    }

    let is_open = |fi: usize| -> bool {
        open_faces.iter().any(|of| {
            let a = of
                .outer_wire()
                .and_then(|w| w.edges().first().map(|e| e.source().point()));
            let b = faces[fi]
                .outer_wire()
                .and_then(|w| w.edges().first().map(|e| e.source().point()));
            // Same face iff same plane and same anchor vertex (faces come from
            // this very solid, so anchor identity is exact).
            match (a, b) {
                (Some(a), Some(b)) => quant(&a) == quant(&b),
                _ => false,
            }
        })
    };

    let remap_wire = |wire: &Wire| -> Option<Wire> {
        let mut edges = Vec::new();
        for edge in wire.edges() {
            let a = *offset_of.get(&quant(&edge.source().point()))?;
            let b = *offset_of.get(&quant(&edge.target().point()))?;
            edges.push(Edge::between_points(a, b));
        }
        Some(Wire::from_edges(edges))
    };

    let mut result: Vec<Face> = Vec::new();
    for (fi, face) in faces.iter().enumerate() {
        if is_open(fi) {
            // Rim: one quad per boundary edge of the opening, bridging the
            // outer edge to its inner (offset) twin.
            if let Some(wire) = face.outer_wire() {
                for edge in wire.edges() {
                    let p0 = edge.source().point();
                    let p1 = edge.target().point();
                    let (Some(&q0), Some(&q1)) =
                        (offset_of.get(&quant(&p0)), offset_of.get(&quant(&p1)))
                    else {
                        return Err(BlendError::UnsupportedShape);
                    };
                    let n = ((p1 - p0).cross(&(q0 - p0))).normalized();
                    let Some(n) = n else { continue };
                    result.push(Face::new(
                        Some(GeomSurface::plane(Plane::from_point_normal(p0, n))),
                        Wire::from_edges([
                            Edge::between_points(p0, p1),
                            Edge::between_points(p1, q1),
                            Edge::between_points(q1, q0),
                            Edge::between_points(q0, p0),
                        ]),
                    ));
                }
            }
            continue;
        }
        // Outer face kept verbatim; inner face is its offset twin. Windings
        // stay as-built — sew's planar canonicalization + global outward pass
        // orient the closed result.
        result.push(face.clone());
        let (anchor, n) = planes[fi];
        let inner_anchor = anchor + GeomVec::from_dir(n) * (-thickness);
        let outer = face.outer_wire().and_then(|w| remap_wire(&w));
        let inners: Vec<Wire> = face
            .inner_wires()
            .into_iter()
            .filter_map(|w| remap_wire(&w))
            .collect();
        let Some(outer) = outer else {
            return Err(BlendError::UnsupportedShape);
        };
        result.push(Face::with_wires(
            Some(GeomSurface::plane(Plane::from_point_normal(
                inner_anchor,
                n.reversed(),
            ))),
            Some(outer),
            inners,
            openrcad_topo::Orientation::Forward,
        ));
    }

    let shelled = Solid::new(
        sew_with_policy(&result, policy)
            .map_err(|error| BlendError::InvalidTolerancePolicy(error.to_string()))?,
    );
    if !shelled.is_watertight_with_policy(policy) {
        return Err(BlendError::UnsupportedShape);
    }
    Ok(shelled)
}

fn det3(m: &[[f64; 3]; 3]) -> f64 {
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
}

/// Cramer's rule for the 3×3 system `m·x = b` with precomputed determinant.
fn solve3(m: &[[f64; 3]; 3], b: &[f64; 3], det: f64) -> [f64; 3] {
    let mut x = [0.0f64; 3];
    for c in 0..3 {
        let mut mc = *m;
        for r in 0..3 {
            mc[r][c] = b[r];
        }
        x[c] = det3(&mc) / det;
    }
    x
}

struct LocalFrame {
    p0: Pnt,
    ex: GeomVec,
    ey: GeomVec,
    ez: GeomVec,
}

impl LocalFrame {
    fn to_world(&self, u: f64, v: f64, w: f64) -> Pnt {
        self.p0 + self.ex * u + self.ey * v + self.ez * w
    }

    fn to_world_dir(&self, du: f64, dv: f64, dw: f64) -> Dir {
        let v = self.ex * du + self.ey * dv + self.ez * dw;
        Dir::new(v.x(), v.y(), v.z())
    }
}

fn detect_box(
    solid: &Solid,
    policy: &TolerancePolicy,
) -> Option<(Pnt, GeomVec, GeomVec, GeomVec, f64, f64, f64)> {
    let vertices = solid.vertices();
    if vertices.len() != 8 {
        return None;
    }
    let faces = solid.shell().faces();
    if faces.len() != 6 {
        return None;
    }

    let context = ToleranceContext::derive(policy, &[solid.bounding_box()], None, 1.0).ok()?;
    let point_tolerance = context.policy.classification.max(context.arithmetic_floor);
    let p0 = vertices[0].point();
    let mut adjacent = Vec::new();
    for edge in solid.edges() {
        let start = edge.start().point();
        let end = edge.end().point();
        if start.distance(&p0) <= point_tolerance {
            adjacent.push(end);
        } else if end.distance(&p0) <= point_tolerance {
            adjacent.push(start);
        }
    }
    adjacent.dedup_by(|first, second| first.distance(second) <= point_tolerance);
    if adjacent.len() != 3 {
        return None;
    }

    let p_x = adjacent[0];
    let p_y = adjacent[1];
    let p_z = adjacent[2];

    let v_x = p_x - p0;
    let v_y = p_y - p0;
    let v_z = p_z - p0;

    let dx = v_x.magnitude();
    let dy = v_y.magnitude();
    let dz = v_z.magnitude();

    if dx <= context.policy.linear || dy <= context.policy.linear || dz <= context.policy.linear {
        return None;
    }

    let ex = GeomVec::from_dir(v_x.normalized().unwrap());
    let ey = GeomVec::from_dir(v_y.normalized().unwrap());
    let ez = GeomVec::from_dir(v_z.normalized().unwrap());

    // Check orthogonality
    if ex.dot(&ey).abs() > context.policy.angular
        || ey.dot(&ez).abs() > context.policy.angular
        || ez.dot(&ex).abs() > context.policy.angular
    {
        return None;
    }

    // Verify combinations
    let tol = point_tolerance;
    let expected = [
        p0 + ex * dx + ey * dy,
        p0 + ey * dy + ez * dz,
        p0 + ex * dx + ez * dz,
        p0 + ex * dx + ey * dy + ez * dz,
    ];

    for &exp in &expected {
        if !vertices
            .iter()
            .any(|vertex| vertex.point().distance(&exp) <= tol)
        {
            return None;
        }
    }

    Some((p0, ex, ey, ez, dx, dy, dz))
}

fn make_straight_edge(v1: &Vertex, v2: &Vertex, policy: &TolerancePolicy) -> Edge {
    let p1 = v1.point();
    let p2 = v2.point();
    let disp = p2 - p1;
    let len = disp.magnitude();
    if len <= policy.linear {
        Edge::new_with_tolerance(None, 0.0, 0.0, v1.clone(), v2.clone(), v1.tolerance())
    } else {
        let dir = disp.normalized().unwrap();
        let line = GeomCurve::line(openrcad_geom::Line::from_point_dir(
            p1,
            Dir::new(dir.x(), dir.y(), dir.z()),
        ));
        Edge::new_with_tolerance(Some(line), 0.0, len, v1.clone(), v2.clone(), policy.linear)
    }
}

#[allow(clippy::too_many_arguments)]
fn shell_box(
    p0: Pnt,
    ex: GeomVec,
    ey: GeomVec,
    ez: GeomVec,
    dx: f64,
    dy: f64,
    dz: f64,
    thickness: f64,
    open_faces: &[Face],
    policy: &TolerancePolicy,
) -> Solid {
    let frame = LocalFrame { p0, ex, ey, ez };

    // Identify which faces are open.
    // Face indices: 0:Bottom, 1:Top, 2:Front, 3:Back, 4:Left, 5:Right.
    let mut is_open = [false; 6];

    let check_face_open = |face_center: Pnt| -> bool {
        for open_f in open_faces {
            if let Some(surf) = open_f.surface() {
                // Check if the face center lies on the open face surface.
                let (u, v) = openrcad_algo_intersect::uv_of(surf, &face_center);
                let p_on_surf = surf.point(u, v);
                if p_on_surf.distance(&face_center) <= policy.classification {
                    // Check if it's within the face trimming loops
                    if openrcad_algo_intersect::is_inside_trimming_loops(u, v, open_f) {
                        return true;
                    }
                }
            }
        }
        false
    };

    is_open[0] = check_face_open(frame.to_world(dx / 2.0, dy / 2.0, 0.0));
    is_open[1] = check_face_open(frame.to_world(dx / 2.0, dy / 2.0, dz));
    is_open[2] = check_face_open(frame.to_world(dx / 2.0, 0.0, dz / 2.0));
    is_open[3] = check_face_open(frame.to_world(dx / 2.0, dy, dz / 2.0));
    is_open[4] = check_face_open(frame.to_world(0.0, dy / 2.0, dz / 2.0));
    is_open[5] = check_face_open(frame.to_world(dx, dy / 2.0, dz / 2.0));

    // Outer shell bounds
    let min_x_out = 0.0;
    let max_x_out = dx;
    let min_y_out = 0.0;
    let max_y_out = dy;
    let min_z_out = 0.0;
    let max_z_out = dz;

    // Inner shell bounds, adjusted by whether the face is open or closed
    let min_x_in = if is_open[4] { 0.0 } else { thickness };
    let max_x_in = if is_open[5] { dx } else { dx - thickness };
    let min_y_in = if is_open[2] { 0.0 } else { thickness };
    let max_y_in = if is_open[3] { dy } else { dy - thickness };
    let min_z_in = if is_open[0] { 0.0 } else { thickness };
    let max_z_in = if is_open[1] { dz } else { dz - thickness };

    // 1. Create the 8 outer and 8 inner vertices
    let mut v_out = HashMap::new();
    let mut v_in = HashMap::new();
    for i in 0..=1 {
        for j in 0..=1 {
            for k in 0..=1 {
                let x_o = if i == 0 { min_x_out } else { max_x_out };
                let y_o = if j == 0 { min_y_out } else { max_y_out };
                let z_o = if k == 0 { min_z_out } else { max_z_out };
                v_out.insert((i, j, k), Vertex::new(frame.to_world(x_o, y_o, z_o)));

                let x_i = if i == 0 { min_x_in } else { max_x_in };
                let y_i = if j == 0 { min_y_in } else { max_y_in };
                let z_i = if k == 0 { min_z_in } else { max_z_in };
                v_in.insert((i, j, k), Vertex::new(frame.to_world(x_i, y_i, z_i)));
            }
        }
    }

    let mut faces = Vec::new();

    // 2. Helper to construct outer & inner faces
    let mut add_flat_face = |face_idx: usize,
                             v_out_coords: [(usize, usize, usize); 4],
                             v_in_coords: [(usize, usize, usize); 4],
                             surf_origin: Pnt,
                             surf_normal: Dir| {
        if is_open[face_idx] {
            return;
        }

        // Outer Face
        let a_o = v_out[&v_out_coords[0]].clone();
        let b_o = v_out[&v_out_coords[1]].clone();
        let c_o = v_out[&v_out_coords[2]].clone();
        let d_o = v_out[&v_out_coords[3]].clone();
        let w_o = Wire::from_edges([
            make_straight_edge(&a_o, &b_o, policy),
            make_straight_edge(&b_o, &c_o, policy),
            make_straight_edge(&c_o, &d_o, policy),
            make_straight_edge(&d_o, &a_o, policy),
        ]);
        let outer_surf = Plane::from_point_normal(surf_origin, surf_normal);
        let f_outer = Face::new(Some(GeomSurface::plane(outer_surf)), w_o);
        faces.push(f_outer);

        // Inner Face (Reversed & offset)
        let a_i = v_in[&v_in_coords[0]].clone();
        let b_i = v_in[&v_in_coords[1]].clone();
        let c_i = v_in[&v_in_coords[2]].clone();
        let d_i = v_in[&v_in_coords[3]].clone();
        let w_i = Wire::from_edges([
            make_straight_edge(&a_i, &b_i, policy),
            make_straight_edge(&b_i, &c_i, policy),
            make_straight_edge(&c_i, &d_i, policy),
            make_straight_edge(&d_i, &a_i, policy),
        ]);
        let base_surf = GeomSurface::plane(outer_surf);
        let offset_surf = GeomSurface::offset(OffsetSurface::new(base_surf, -thickness));
        let f_inner = Face::new(Some(offset_surf), w_i).reversed();
        faces.push(f_inner);
    };

    // Bottom (Z=0, index 0)
    add_flat_face(
        0,
        [(0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 0, 0)],
        [(0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 0, 0)],
        frame.to_world(0.0, 0.0, 0.0),
        frame.to_world_dir(0.0, 0.0, -1.0),
    );

    // Top (Z=dz, index 1)
    add_flat_face(
        1,
        [(0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)],
        [(0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)],
        frame.to_world(0.0, 0.0, dz),
        frame.to_world_dir(0.0, 0.0, 1.0),
    );

    // Front (Y=0, index 2)
    add_flat_face(
        2,
        [(0, 0, 0), (0, 0, 1), (1, 0, 1), (1, 0, 0)],
        [(0, 0, 0), (0, 0, 1), (1, 0, 1), (1, 0, 0)],
        frame.to_world(0.0, 0.0, 0.0),
        frame.to_world_dir(0.0, -1.0, 0.0),
    );

    // Back (Y=dy, index 3)
    add_flat_face(
        3,
        [(0, 1, 0), (1, 1, 0), (1, 1, 1), (0, 1, 1)],
        [(0, 1, 0), (1, 1, 0), (1, 1, 1), (0, 1, 1)],
        frame.to_world(0.0, dy, 0.0),
        frame.to_world_dir(0.0, 1.0, 0.0),
    );

    // Left (X=0, index 4)
    add_flat_face(
        4,
        [(0, 0, 0), (0, 1, 0), (0, 1, 1), (0, 0, 1)],
        [(0, 0, 0), (0, 1, 0), (0, 1, 1), (0, 0, 1)],
        frame.to_world(0.0, 0.0, 0.0),
        frame.to_world_dir(-1.0, 0.0, 0.0),
    );

    // Right (X=dx, index 5)
    add_flat_face(
        5,
        [(1, 0, 0), (1, 0, 1), (1, 1, 1), (1, 1, 0)],
        [(1, 0, 0), (1, 0, 1), (1, 1, 1), (1, 1, 0)],
        frame.to_world(dx, 0.0, 0.0),
        frame.to_world_dir(1.0, 0.0, 0.0),
    );

    // 3. Add Rim stitching faces around the open face loops
    // If a face is open, we add rim faces connecting its boundary segments.
    // For a rectangular box, each open face has 4 boundary segments.
    // Let's specify the 4 segments for each of the 6 possible open faces.
    // Each segment connects an outer vertex pair to an inner vertex pair.
    // Winding: Outer_A -> Inner_A -> Inner_B -> Outer_B -> Outer_A.
    let mut add_rim_segment = |v_out_a: (usize, usize, usize),
                               v_out_b: (usize, usize, usize),
                               v_in_a: (usize, usize, usize),
                               v_in_b: (usize, usize, usize),
                               rim_normal: Dir| {
        let a_o = v_out[&v_out_a].clone();
        let b_o = v_out[&v_out_b].clone();
        let a_i = v_in[&v_in_a].clone();
        let b_i = v_in[&v_in_b].clone();

        let e1 = make_straight_edge(&a_o, &a_i, policy);
        let e2 = make_straight_edge(&a_i, &b_i, policy);
        let e3 = make_straight_edge(&b_i, &b_o, policy);
        let e4 = make_straight_edge(&b_o, &a_o, policy);

        let w = Wire::from_edges([e1, e2, e3, e4]);
        let surf = GeomSurface::plane(Plane::from_point_normal(a_o.point(), rim_normal));
        let f = Face::new(Some(surf), w);
        faces.push(f);
    };

    // Bottom (index 0) open: Z=0
    if is_open[0] {
        let n = frame.to_world_dir(0.0, 0.0, -1.0);
        add_rim_segment((0, 0, 0), (0, 1, 0), (0, 0, 0), (0, 1, 0), n);
        add_rim_segment((0, 1, 0), (1, 1, 0), (0, 1, 0), (1, 1, 0), n);
        add_rim_segment((1, 1, 0), (1, 0, 0), (1, 1, 0), (1, 0, 0), n);
        add_rim_segment((1, 0, 0), (0, 0, 0), (1, 0, 0), (0, 0, 0), n);
    }

    // Top (index 1) open: Z=dz
    if is_open[1] {
        let n = frame.to_world_dir(0.0, 0.0, 1.0);
        add_rim_segment((0, 0, 1), (1, 0, 1), (0, 0, 1), (1, 0, 1), n);
        add_rim_segment((1, 0, 1), (1, 1, 1), (1, 0, 1), (1, 1, 1), n);
        add_rim_segment((1, 1, 1), (0, 1, 1), (1, 1, 1), (0, 1, 1), n);
        add_rim_segment((0, 1, 1), (0, 0, 1), (0, 1, 1), (0, 0, 1), n);
    }

    // Front (index 2) open: Y=0
    if is_open[2] {
        let n = frame.to_world_dir(0.0, -1.0, 0.0);
        add_rim_segment((0, 0, 0), (1, 0, 0), (0, 0, 0), (1, 0, 0), n);
        add_rim_segment((1, 0, 0), (1, 0, 1), (1, 0, 0), (1, 0, 1), n);
        add_rim_segment((1, 0, 1), (0, 0, 1), (1, 0, 1), (0, 0, 1), n);
        add_rim_segment((0, 0, 1), (0, 0, 0), (0, 0, 1), (0, 0, 0), n);
    }

    // Back (index 3) open: Y=dy
    if is_open[3] {
        let n = frame.to_world_dir(0.0, 1.0, 0.0);
        add_rim_segment((0, 1, 0), (0, 1, 1), (0, 1, 0), (0, 1, 1), n);
        add_rim_segment((0, 1, 1), (1, 1, 1), (0, 1, 1), (1, 1, 1), n);
        add_rim_segment((1, 1, 1), (1, 1, 0), (1, 1, 1), (1, 1, 0), n);
        add_rim_segment((1, 1, 0), (0, 1, 0), (1, 1, 0), (0, 1, 0), n);
    }

    // Left (index 4) open: X=0
    if is_open[4] {
        let n = frame.to_world_dir(-1.0, 0.0, 0.0);
        add_rim_segment((0, 0, 0), (0, 1, 0), (0, 0, 0), (0, 1, 0), n);
        add_rim_segment((0, 1, 0), (0, 1, 1), (0, 1, 0), (0, 1, 1), n);
        add_rim_segment((0, 1, 1), (0, 0, 1), (0, 1, 1), (0, 0, 1), n);
        add_rim_segment((0, 0, 1), (0, 0, 0), (0, 0, 1), (0, 0, 0), n);
    }

    // Right (index 5) open: X=dx
    if is_open[5] {
        let n = frame.to_world_dir(1.0, 0.0, 0.0);
        add_rim_segment((1, 0, 0), (1, 0, 1), (1, 0, 0), (1, 0, 1), n);
        add_rim_segment((1, 0, 1), (1, 1, 1), (1, 0, 1), (1, 1, 1), n);
        add_rim_segment((1, 1, 1), (1, 1, 0), (1, 1, 1), (1, 1, 0), n);
        add_rim_segment((1, 1, 0), (1, 0, 0), (1, 1, 0), (1, 0, 0), n);
    }

    // Sew the collection of faces into a watertight shell
    let shell =
        sew_with_policy(&faces, policy).expect("policy was validated by shell_solid_with_policy");
    Solid::new(shell)
}

mod openrcad_algo_intersect {
    use super::*;
    pub fn uv_of(s: &GeomSurface, p: &Pnt) -> (f64, f64) {
        crate::intersect::uv_of(s, p)
    }
    pub fn is_inside_trimming_loops(u: f64, v: f64, face: &Face) -> bool {
        crate::intersect::is_inside_trimming_loops(u, v, face)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::Pnt;
    use openrcad_primitives::make_box;

    /// A right-triangle prism (legs `a`, height `h`) built through `prism` —
    /// NOT the box/cylinder recognisers' territory, so it exercises the
    /// general planar shell path.
    fn triangle_prism(a: f64, h: f64) -> Solid {
        use openrcad_geom::Plane as GPlane;
        let face = Face::new(
            Some(GeomSurface::plane(GPlane::from_point_normal(
                Pnt::origin(),
                Dir::dz(),
            ))),
            Wire::from_edges([
                Edge::between_points(Pnt::origin(), Pnt::new(a, 0.0, 0.0)),
                Edge::between_points(Pnt::new(a, 0.0, 0.0), Pnt::new(0.0, a, 0.0)),
                Edge::between_points(Pnt::new(0.0, a, 0.0), Pnt::origin()),
            ]),
        );
        crate::prism::prism(&face, GeomVec::new(0.0, 0.0, h)).unwrap()
    }

    #[test]
    fn general_shell_triangle_prism_open_top() {
        let a = 10.0f64;
        let h = 10.0f64;
        let t = 1.0f64;
        let solid = triangle_prism(a, h);
        // The top face: plane z = h with outward +z normal.
        let faces = solid.shell().faces();
        let top: Vec<Face> = faces
            .iter()
            .filter(|f| {
                f.outer_wire()
                    .and_then(|w| w.edges().first().map(|e| e.source().point().z()))
                    .map(|z| (z - h).abs() < 1e-6)
                    .unwrap_or(false)
                    && matches!(f.surface(), Some(GeomSurface::Plane(p)) if p.normal().z().abs() > 0.9)
            })
            .cloned()
            .collect();
        assert_eq!(top.len(), 1, "expected exactly one top cap");

        let cup = shell_solid(&solid, t, &top).expect("general shell");
        assert!(cup.is_watertight(), "shelled prism not watertight");

        // Expected material volume. The inner triangle is the outer inset by t
        // per edge: legs shrink to a − t·(2 + √2). The void is a prism of the
        // inner triangle from z=t to h−t, topped by a linear transition to the
        // full outer triangle at z=h (the slanted rim quads). The transition's
        // cross-section area is quadratic in z, so Simpson integrates exactly.
        let s2 = 2.0f64.sqrt();
        let leg_inner = a - t * (2.0 + s2);
        let area = |leg: f64| leg * leg / 2.0;
        let a_outer = area(a);
        let a_inner = area(leg_inner);
        let a_mid = area((a + leg_inner) / 2.0);
        let transition = t / 6.0 * (a_inner + 4.0 * a_mid + a_outer);
        let void = a_inner * (h - 2.0 * t) + transition;
        let expected = a_outer * h - void;

        let mesh = openrcad_mesh::tessellate(&cup, 0.002, 0.05);
        let mp = openrcad_mesh::mass_properties(&mesh).expect("closed shelled mesh");
        assert!(
            (mp.volume - expected).abs() / expected < 0.01,
            "shelled volume {} vs expected {expected}",
            mp.volume
        );
    }

    #[test]
    fn general_shell_rejects_closed_hollow_and_curved() {
        let solid = triangle_prism(10.0, 10.0);
        // No open faces → not supported (needs two-shell voids).
        assert!(matches!(
            shell_solid(&solid, 1.0, &[]),
            Err(BlendError::UnsupportedShape)
        ));
    }

    #[test]
    fn test_solid_shelling() {
        let cube = make_box(&Pnt::origin(), 1.0, 1.0, 1.0);

        // Find the top face (Z = 1.0) to remove
        let faces = cube.shell().faces();
        let mut open_faces = Vec::new();
        for face in &faces {
            if let Some(GeomSurface::Plane(plane)) = face.surface() {
                if (plane.location().z() - 1.0).abs() < 1e-4 && plane.normal().z() > 0.9 {
                    open_faces.push(face.clone());
                    break;
                }
            }
        }
        assert_eq!(open_faces.len(), 1);

        let cup = shell_solid(&cube, 0.1, &open_faces).unwrap();

        // A shelled box with 1 open face has:
        // 5 outer faces + 5 inner faces + 4 rim faces = 14 faces.
        // 16 vertices.
        // 28 edges.
        assert_eq!(cup.face_count(), 14);
        assert_eq!(cup.vertex_count(), 16);
        assert_eq!(cup.edge_count(), 28);

        // Verify Euler characteristic
        let v = cup.vertex_count() as i32;
        let e = cup.edge_count() as i32;
        let f = cup.face_count() as i32;
        assert_eq!(v - e + f, 2);

        // Verify face surface types:
        // 5 Planes (outer) + 5 Offset surfaces (inner) + 4 Planes (rims) = 9 Planes, 5 Offsets
        let mut planes_count = 0;
        let mut offsets_count = 0;

        for face in cup.shell().faces() {
            if let Some(surf) = face.surface() {
                match surf {
                    GeomSurface::Plane(_) => planes_count += 1,
                    GeomSurface::Offset(_) => offsets_count += 1,
                    _ => {}
                }
            }
        }

        assert_eq!(planes_count, 9);
        assert_eq!(offsets_count, 5);
    }
}
