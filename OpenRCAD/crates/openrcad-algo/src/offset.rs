use openrcad_foundation::{
    Dir, Pnt, Pnt2d, ToleranceContext, TolerancePolicy, Trsf, Vec as GeomVec,
};
use openrcad_geom::{
    ConicalSurface, Curve, CylindricalSurface, GeomCurve, GeomSurface, OffsetSurface, Plane,
    ReparametrizedCurve, RuledSurface, SphericalSurface, Surface, ToroidalSurface,
    TorusSurfaceCurve,
};
use openrcad_topo::{
    Edge, Face, InputTopologyRef, Solid, TopologyChange, TopologyHistory, TopologyKind,
    TopologyRef, Vertex, Wire,
};
use std::{
    collections::{HashMap, HashSet},
    sync::OnceLock,
};

use crate::blend::{detect_cylinder_with_policy, shell_cylinder_with_policy, BlendError};
use crate::sew::sew_shell_with_policy as sew_with_policy;

fn shell_trace_enabled() -> bool {
    static ENABLED: OnceLock<bool> = OnceLock::new();
    *ENABLED.get_or_init(|| std::env::var_os("OPENRCAD_SHELL_DEBUG").is_some())
}

fn shell_trace(arguments: core::fmt::Arguments<'_>) {
    if shell_trace_enabled() {
        eprintln!("[openrcad-shell] {arguments}");
    }
}

macro_rules! shell_trace {
    ($($argument:tt)*) => {
        shell_trace(format_args!($($argument)*))
    };
}

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

/// Deterministic evidence that a concave Shell candidate represents one
/// unambiguous material envelope. The topology-preserving analytic sheller
/// has already imprinted adjacent offset supports; this certificate broad-
/// phases every non-adjacent cell pair, runs exact trimmed intersections for
/// the survivors, and refuses to commit an unresolved branch.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ConcaveShellCertificate {
    pub concave_edges: usize,
    pub candidate_pairs: usize,
    pub intersection_curves: usize,
    pub imprinted_cells: usize,
    pub kept_cells: usize,
    pub discarded_cells: usize,
    pub minimum_thickness: f64,
    pub maximum_thickness: f64,
    pub unresolved_branches: usize,
    pub work_units: u64,
}

impl ConcaveShellCertificate {
    pub fn summary(&self) -> String {
        format!(
            "concave_edges={} candidate_pairs={} intersection_curves={} cells={} kept={} discarded={} thickness=[{}, {}] unresolved={} work_units={}",
            self.concave_edges,
            self.candidate_pairs,
            self.intersection_curves,
            self.imprinted_cells,
            self.kept_cells,
            self.discarded_cells,
            self.minimum_thickness,
            self.maximum_thickness,
            self.unresolved_branches,
            self.work_units,
        )
    }
}

/// Typed, fail-closed reasons a concave Shell envelope cannot be certified.
#[derive(Clone, Debug, PartialEq)]
pub enum ConcaveShellError {
    InvalidTolerancePolicy(String),
    UnsupportedSupport {
        source_face: usize,
        support: crate::band_topology::BandSupportKind,
    },
    MissingOffsetCell {
        source_face: usize,
    },
    AmbiguousOffsetCell {
        source_face: usize,
        matches: usize,
    },
    OffsetCollapse {
        requested: f64,
        max: f64,
    },
    AmbiguousMaterialClassification {
        cell: usize,
    },
    AmbiguousSelfIntersection {
        candidate_pairs: usize,
        intersection_curves: usize,
    },
    BandTopology(crate::band_topology::BandTopologyError),
}

impl ConcaveShellError {
    pub const fn diagnostic_code(&self) -> &'static str {
        match self {
            Self::InvalidTolerancePolicy(_) => "parameter.invalid",
            Self::UnsupportedSupport { .. } => "shell.unsupported_support",
            Self::MissingOffsetCell { .. }
            | Self::AmbiguousOffsetCell { .. }
            | Self::OffsetCollapse { .. } => "shell.offset_collapse",
            Self::AmbiguousMaterialClassification { .. }
            | Self::AmbiguousSelfIntersection { .. } => "shell.ambiguous_self_intersection",
            Self::BandTopology(error) => error.diagnostic_code(),
        }
    }
}

impl core::fmt::Display for ConcaveShellError {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidTolerancePolicy(reason) => {
                write!(formatter, "invalid Shell tolerance policy: {reason}")
            }
            Self::UnsupportedSupport {
                source_face,
                support,
            } => write!(
                formatter,
                "concave Shell source face {source_face} has unsupported {support:?} support"
            ),
            Self::MissingOffsetCell { source_face } => write!(
                formatter,
                "concave Shell source face {source_face} has no unique offset cell"
            ),
            Self::AmbiguousOffsetCell {
                source_face,
                matches,
            } => write!(
                formatter,
                "concave Shell source face {source_face} resolves to {matches} offset cells"
            ),
            Self::OffsetCollapse { requested, max } => write!(
                formatter,
                "concave Shell thickness {requested} reaches the conservative local-clearance limit {max}"
            ),
            Self::AmbiguousMaterialClassification { cell } => write!(
                formatter,
                "concave Shell cell {cell} has ambiguous material-side classification"
            ),
            Self::AmbiguousSelfIntersection {
                candidate_pairs,
                intersection_curves,
            } => write!(
                formatter,
                "concave Shell has {intersection_curves} unresolved intersection curves across {candidate_pairs} candidate pairs"
            ),
            Self::BandTopology(error) => error.fmt(formatter),
        }
    }
}

impl std::error::Error for ConcaveShellError {}

impl From<crate::band_topology::BandTopologyError> for ConcaveShellError {
    fn from(value: crate::band_topology::BandTopologyError) -> Self {
        Self::BandTopology(value)
    }
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

fn planar_support_separation(
    source: &GeomSurface,
    candidate: &GeomSurface,
    policy: &TolerancePolicy,
) -> Option<f64> {
    let (GeomSurface::Plane(source), GeomSurface::Plane(candidate)) = (source, candidate) else {
        return None;
    };
    if source.normal().dot(&candidate.normal()).abs() < 1.0 - policy.angular * 8.0 {
        return None;
    }
    Some(
        (candidate.location() - source.location())
            .dot(&GeomVec::from_dir(source.normal()))
            .abs(),
    )
}

fn source_offset_separation(
    source: &GeomSurface,
    candidate: &GeomSurface,
    policy: &TolerancePolicy,
) -> Option<f64> {
    match candidate {
        GeomSurface::Offset(offset)
            if crate::boolean::surfaces_are_same_domain(
                source,
                &offset.base,
                policy.classification.max(policy.intersection),
            ) =>
        {
            Some(offset.distance.abs())
        }
        _ => planar_support_separation(source, candidate, policy),
    }
}

fn faces_share_edge(first: &Face, second: &Face) -> bool {
    let first_edges: HashSet<_> = first
        .wires()
        .into_iter()
        .flat_map(|wire| wire.edges())
        .map(|edge| edge.id())
        .collect();
    second
        .wires()
        .into_iter()
        .flat_map(|wire| wire.edges())
        .any(|edge| first_edges.contains(&edge.id()))
}

fn point_segment_distance(point: Pnt, first: Pnt, second: Pnt) -> f64 {
    let direction = second - first;
    let denominator = direction.dot(&direction);
    if denominator <= f64::MIN_POSITIVE {
        return point.distance(&first);
    }
    let parameter = ((point - first).dot(&direction) / denominator).clamp(0.0, 1.0);
    point.distance(&(first + direction * parameter))
}

pub(crate) fn segment_distance_for_certificate(
    first_start: Pnt,
    first_end: Pnt,
    second_start: Pnt,
    second_end: Pnt,
) -> f64 {
    // The endpoint probes cover parallel and degenerate segments. The
    // interior/interior solve covers the skew case without a unit-scale
    // epsilon: the operation-local angular policy decides parallelism.
    let first_direction = first_end - first_start;
    let second_direction = second_end - second_start;
    let cross = first_direction.cross(&second_direction);
    let cross_squared = cross.dot(&cross);
    let mut distance = [
        point_segment_distance(first_start, second_start, second_end),
        point_segment_distance(first_end, second_start, second_end),
        point_segment_distance(second_start, first_start, first_end),
        point_segment_distance(second_end, first_start, first_end),
    ]
    .into_iter()
    .fold(f64::INFINITY, f64::min);
    if cross_squared > f64::MIN_POSITIVE {
        let between = second_start - first_start;
        let first_parameter = between.cross(&second_direction).dot(&cross) / cross_squared;
        let second_parameter = between.cross(&first_direction).dot(&cross) / cross_squared;
        if (0.0..=1.0).contains(&first_parameter) && (0.0..=1.0).contains(&second_parameter) {
            let first_point = first_start + first_direction * first_parameter;
            let second_point = second_start + second_direction * second_parameter;
            distance = distance.min(first_point.distance(&second_point));
        }
    }
    distance
}

fn planar_reflex_vertex_count(source: &Solid, policy: &TolerancePolicy) -> usize {
    let mut reflex_vertices = 0usize;
    for face in source.shell().faces() {
        let Some(GeomSurface::Plane(plane)) = face.surface() else {
            continue;
        };
        let Some(wire) = face.outer_wire() else {
            continue;
        };
        let points = wire
            .edges()
            .iter()
            .map(|edge| {
                let point = edge.source().point();
                let (u, v) = crate::intersect::uv_of(&GeomSurface::plane(*plane), &point);
                (u, v)
            })
            .collect::<Vec<_>>();
        if points.len() < 4 {
            continue;
        }
        let signed_twice_area = (0..points.len())
            .map(|index| {
                let first = points[index];
                let second = points[(index + 1) % points.len()];
                first.0 * second.1 - second.0 * first.1
            })
            .sum::<f64>();
        if signed_twice_area.abs() <= policy.resolution * policy.resolution {
            continue;
        }
        let winding = signed_twice_area.signum();
        for index in 0..points.len() {
            let previous = points[(index + points.len() - 1) % points.len()];
            let current = points[index];
            let next = points[(index + 1) % points.len()];
            let incoming = (current.0 - previous.0, current.1 - previous.1);
            let outgoing = (next.0 - current.0, next.1 - current.1);
            let cross = incoming.0 * outgoing.1 - incoming.1 * outgoing.0;
            let scale = incoming.0.hypot(incoming.1) * outgoing.0.hypot(outgoing.1);
            if cross * winding < -policy.angular * scale {
                reflex_vertices += 1;
            }
        }
    }
    reflex_vertices
}

fn preflight_concave_shell_with_policy(
    source: &Solid,
    requested_thickness: f64,
    policy: &TolerancePolicy,
) -> Result<usize, ConcaveShellError> {
    let context = ToleranceContext::derive(
        policy,
        &[source.bounding_box()],
        Some(requested_thickness.abs()),
        1.0,
    )
    .map_err(|error| ConcaveShellError::InvalidTolerancePolicy(error.to_string()))?;
    let probed_concave_edges = source
        .edges()
        .iter()
        .filter(|edge| {
            crate::rolling_ball::edge_material_wedge_is_concave(source, edge) == Some(true)
        })
        .count();
    // Extremely high aspect ratios can make the containment probe too local
    // to distinguish a reflex wedge. A planar loop's signed turns are exact
    // under rigid transforms and provide an independent analytic classifier.
    let analytic_concave_edges = planar_reflex_vertex_count(source, &context.policy).div_ceil(2);
    let concave_edges = probed_concave_edges.max(analytic_concave_edges);
    if concave_edges == 0 {
        return Ok(0);
    }
    for (source_face, face) in source.shell().faces().iter().enumerate() {
        let support = face
            .surface()
            .map(crate::band_topology::BandSupportKind::of)
            .unwrap_or(crate::band_topology::BandSupportKind::Other);
        if support == crate::band_topology::BandSupportKind::Other {
            return Err(ConcaveShellError::UnsupportedSupport {
                source_face,
                support,
            });
        }
    }

    // The straight-edge clearance certificate is exact for the planar 5E
    // subset. Mixed analytic networks retain their surface-specific collapse
    // checks (notably the established torus minor-radius rejection), so this
    // conservative planar bound cannot mask a more precise legacy diagnostic.
    if source
        .shell()
        .faces()
        .iter()
        .all(|face| matches!(face.surface(), Some(GeomSurface::Plane(_))))
    {
        let straight_edges = source
            .edges()
            .into_iter()
            .filter(|edge| matches!(edge.curve(), None | Some(GeomCurve::Line(_))))
            .collect::<Vec<_>>();
        let mut minimum_clearance = f64::INFINITY;
        for first in 0..straight_edges.len() {
            for second in first + 1..straight_edges.len() {
                let first_start = straight_edges[first].source().point();
                let first_end = straight_edges[first].target().point();
                let second_start = straight_edges[second].source().point();
                let second_end = straight_edges[second].target().point();
                if [first_start, first_end].into_iter().any(|first_point| {
                    [second_start, second_end].into_iter().any(|second_point| {
                        first_point.distance(&second_point) <= context.policy.linear
                    })
                }) {
                    continue;
                }
                let distance = segment_distance_for_certificate(
                    first_start,
                    first_end,
                    second_start,
                    second_end,
                );
                if distance > context.policy.linear {
                    minimum_clearance = minimum_clearance.min(distance);
                }
            }
        }
        if minimum_clearance.is_finite() {
            let maximum = minimum_clearance * 0.5;
            if requested_thickness.abs() >= maximum - context.policy.classification {
                return Err(ConcaveShellError::OffsetCollapse {
                    requested: requested_thickness,
                    max: maximum,
                });
            }
        }
    }
    Ok(concave_edges)
}

/// Certify the supported concave Shell envelope without mutating either
/// operand. `Ok(None)` means the source has no material-concave analytic edge
/// and therefore does not require the Wave 5E certificate.
///
/// The current accepted matrix is analytic topology whose inward offset keeps
/// one cell per retained source face. Adjacent cells are already imprinted by
/// the Shell builder. If non-adjacent cells intersect, exact face/face curves
/// are recorded and the candidate is rejected atomically; approximation is
/// never substituted for unresolved cell partitioning.
pub fn certify_concave_shell_with_policy(
    source: &Solid,
    candidate: &Solid,
    expected_thickness: f64,
    open_faces: &[Face],
    policy: &TolerancePolicy,
) -> Result<Option<ConcaveShellCertificate>, ConcaveShellError> {
    policy
        .validate()
        .map_err(|error| ConcaveShellError::InvalidTolerancePolicy(error.to_string()))?;
    let context = ToleranceContext::derive(
        policy,
        &[source.bounding_box(), candidate.bounding_box()],
        Some(expected_thickness.abs()),
        1.0,
    )
    .map_err(|error| ConcaveShellError::InvalidTolerancePolicy(error.to_string()))?;
    let concave_edges =
        preflight_concave_shell_with_policy(source, expected_thickness, &context.policy)?;
    if concave_edges == 0 {
        return Ok(None);
    }

    let source_faces = source.shell().faces();
    let result_faces = candidate.shell().faces();
    let domain_tolerance = context
        .policy
        .classification
        .max(context.policy.intersection);
    let thickness_tolerance = (context.policy.linear * 10.0)
        .max(expected_thickness.abs() * 0.005)
        .max(context.arithmetic_floor * 8.0);
    let mut inner_cells: HashMap<usize, f64> = HashMap::new();

    for (source_index, source_face) in source_faces.iter().enumerate() {
        if open_faces.iter().any(|open| open == source_face) {
            continue;
        }
        let Some(source_surface) = source_face.surface() else {
            return Err(ConcaveShellError::UnsupportedSupport {
                source_face: source_index,
                support: crate::band_topology::BandSupportKind::Other,
            });
        };
        let support = crate::band_topology::BandSupportKind::of(source_surface);
        if support == crate::band_topology::BandSupportKind::Other {
            return Err(ConcaveShellError::UnsupportedSupport {
                source_face: source_index,
                support,
            });
        }

        // Removed opening faces have no retained same-domain outer face and no
        // corresponding inner cap. Every retained source face must have one
        // uniquely measurable inward cell.
        let retained = result_faces.iter().any(|face| {
            face.surface().is_some_and(|surface| {
                crate::boolean::surfaces_are_same_domain(source_surface, surface, domain_tolerance)
            })
        });
        if !retained {
            continue;
        }

        let mut matches = result_faces
            .iter()
            .enumerate()
            .filter_map(|(result_index, result_face)| {
                let separation = source_offset_separation(
                    source_surface,
                    result_face.surface()?,
                    &context.policy,
                )?;
                (separation > thickness_tolerance
                    && (separation - expected_thickness.abs()).abs() <= thickness_tolerance)
                    .then_some((result_index, separation))
            })
            .collect::<Vec<_>>();
        matches.sort_by(|left, right| {
            (left.1 - expected_thickness.abs())
                .abs()
                .total_cmp(&(right.1 - expected_thickness.abs()).abs())
                .then_with(|| left.0.cmp(&right.0))
        });
        let Some(_) = matches.first() else {
            return Err(ConcaveShellError::MissingOffsetCell {
                source_face: source_index,
            });
        };
        // Periodic/imprinted supports may be split on either side: one source
        // face can own several result cells, and coplanar or co-toroidal source
        // partitions can share one exact offset cell. Uniqueness is therefore
        // geometric (result-cell identity plus requested separation), not a
        // one-to-one face-index mapping. The 4C torus-band fixture exercises
        // both forms of partitioning.
        for (result_index, separation) in matches {
            inner_cells.entry(result_index).or_insert(separation);
        }
    }
    if inner_cells.is_empty() {
        return Err(ConcaveShellError::MissingOffsetCell { source_face: 0 });
    }

    let mut kept_cells = 0usize;
    let mut ordered_cells = inner_cells.keys().copied().collect::<Vec<_>>();
    ordered_cells.sort_unstable();
    let mut budget = crate::band_topology::GeometryWorkBudget::intersection_default();
    for &cell in &ordered_cells {
        budget.charge(crate::band_topology::GeometryWorkStage::Classification, 1)?;
        let sample = crate::boolean::point_on_face(&result_faces[cell]);
        if !crate::boolean::point_in_solid(&sample, source) {
            return Err(ConcaveShellError::AmbiguousMaterialClassification { cell });
        }
        kept_cells += 1;
    }

    let mut edge_uses = HashMap::new();
    for &cell in &ordered_cells {
        for edge in result_faces[cell]
            .wires()
            .into_iter()
            .flat_map(|wire| wire.edges())
        {
            *edge_uses.entry(edge.id()).or_insert(0usize) += 1;
        }
    }
    let imprinted_boundaries = edge_uses.values().filter(|uses| **uses > 1).count();

    let mut candidate_pairs = 0usize;
    let mut intersection_curves = 0usize;
    for first_position in 0..ordered_cells.len() {
        for second_position in first_position + 1..ordered_cells.len() {
            let first = &result_faces[ordered_cells[first_position]];
            let second = &result_faces[ordered_cells[second_position]];
            if faces_share_edge(first, second) {
                continue;
            }
            let mut first_bounds = crate::bvh::compute_face_bounds(first);
            let mut second_bounds = crate::bvh::compute_face_bounds(second);
            first_bounds.enlarge(context.policy.classification);
            second_bounds.enlarge(context.policy.classification);
            if first_bounds.is_out_box(&second_bounds) {
                continue;
            }
            candidate_pairs += 1;
            budget.charge(crate::band_topology::GeometryWorkStage::Intersection, 1)?;
            intersection_curves += crate::intersect::surface_surface_curves_with_budget(
                first,
                second,
                context.policy.intersection,
                &mut budget,
            )?
            .len();
        }
    }
    if intersection_curves > 0 {
        return Err(ConcaveShellError::AmbiguousSelfIntersection {
            candidate_pairs,
            intersection_curves,
        });
    }

    let minimum_thickness = inner_cells.values().copied().fold(f64::INFINITY, f64::min);
    let maximum_thickness = inner_cells
        .values()
        .copied()
        .fold(f64::NEG_INFINITY, f64::max);
    Ok(Some(ConcaveShellCertificate {
        concave_edges,
        candidate_pairs,
        intersection_curves,
        imprinted_cells: kept_cells + imprinted_boundaries,
        kept_cells,
        discarded_cells: 0,
        minimum_thickness,
        maximum_thickness,
        unresolved_branches: 0,
        work_units: budget.consumed(),
    }))
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
    preflight_concave_shell_with_policy(solid, thickness, policy)?;
    let candidate = match shell_build_path_with_policy(solid, policy) {
        ShellBuildPath::BoxPrimitive => {
            let (p0, ex, ey, ez, dx, dy, dz) =
                detect_box(solid, policy).expect("box path was classified by the same detector");
            shell_box(p0, ex, ey, ez, dx, dy, dz, thickness, open_faces, policy)
        }
        ShellBuildPath::CylinderPrimitive => {
            let cylinder = detect_cylinder_with_policy(solid, policy)
                .expect("cylinder path was classified by the same detector");
            shell_cylinder_with_policy(&cylinder, thickness, open_faces, policy)?
        }
        ShellBuildPath::PlanarNetwork => {
            shell_planar_general(solid, thickness, open_faces, policy)?
        }
        ShellBuildPath::MixedAnalyticNetwork => {
            shell_analytic_general(solid, thickness, open_faces, policy)?
        }
    };
    // The candidate remains immutable until its material envelope is proven.
    // Convex and primitive sources return `None` cheaply; concave analytic
    // sources must pass the exact broad-phase/intersection certificate.
    certify_concave_shell_with_policy(solid, &candidate, thickness, open_faces, policy)?;
    Ok(candidate)
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

fn trace_shell_failure(stage: &'static str, error: BlendError) -> BlendError {
    shell_trace!("stage {stage} rejected: {error}");
    error
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
        GeomSurface::Torus(torus) => {
            let minor_radius = torus.minor_radius() + signed_distance;
            let regular_limit = torus.major_radius() - context.policy.linear;
            if minor_radius <= context.policy.linear || minor_radius >= regular_limit {
                let max = if signed_distance < 0.0 {
                    torus.minor_radius()
                } else {
                    (torus.major_radius() - torus.minor_radius()).max(0.0)
                };
                return Err(BlendError::ParameterTooLarge {
                    requested: thickness,
                    max,
                });
            }
            GeomSurface::torus(ToroidalSurface::new(
                torus.position(),
                torus.major_radius(),
                minor_radius,
            ))
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
        GeomSurface::Torus(torus) => {
            let frame = torus.position();
            let axis = GeomVec::from_dir(frame.direction());
            let delta = point - frame.location();
            let axial = delta.dot(&axis);
            let radial = delta - axis * axial;
            let radial_magnitude = radial.magnitude();
            let radial_direction = radial.normalized().map(GeomVec::from_dir)?;
            let meridian_radial = radial_magnitude - torus.major_radius();
            let tube_distance = meridian_radial.hypot(axial);
            if tube_distance <= f64::MIN_POSITIVE {
                return None;
            }
            let gradient = radial_direction * (meridian_radial / tube_distance)
                + axis * (axial / tube_distance);
            Some((tube_distance - torus.minor_radius(), gradient))
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
                    if best.map_or(true, |(current, _)| determinant > current) {
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
    // Leave accuracy headroom for the independent pcurve certificate. The
    // looser classification tolerance is suitable for containment, but not
    // for committing a shared analytic endpoint.
    let target_residual = context.policy.pcurve_consistency * 0.25;
    if supports.len() == 1 {
        for _ in 0..8 {
            let (residual, gradient) = support_residual_gradient(supports[0], point)
                .ok_or(BlendError::UnsupportedShape)?;
            point = point - gradient * residual;
            if support_residual_gradient(supports[0], point)
                .is_some_and(|(residual, _)| residual.abs() <= target_residual)
            {
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
    let initial = supports
        .iter()
        .map(|support| support_residual_gradient(support, point))
        .collect::<Option<Vec<_>>>()
        .ok_or(BlendError::UnsupportedShape)?;
    let reference_gradient = initial[0].1;
    let tangent_network = initial.iter().skip(1).all(|(_, gradient)| {
        reference_gradient.cross(gradient).magnitude() <= context.policy.angular * 8.0
    });
    if tangent_network {
        // G1 support transitions (notably cylinder/torus fillet contact) have
        // dependent normals by construction. Their offset intersection is a
        // curve, so preserve the source parameter along that curve and solve
        // only the shared normal displacement. Reversing an implicit gradient
        // also reverses its residual before the consistency comparison.
        for _ in 0..12 {
            let evaluated = supports
                .iter()
                .map(|support| support_residual_gradient(support, point))
                .collect::<Option<Vec<_>>>()
                .ok_or(BlendError::UnsupportedShape)?;
            let base_gradient = evaluated[0].1;
            let aligned: Vec<f64> = evaluated
                .iter()
                .map(|(residual, gradient)| {
                    if base_gradient.dot(gradient) >= 0.0 {
                        *residual
                    } else {
                        -*residual
                    }
                })
                .collect();
            let minimum = aligned.iter().copied().fold(f64::INFINITY, f64::min);
            let maximum = aligned.iter().copied().fold(f64::NEG_INFINITY, f64::max);
            if maximum - minimum > context.policy.classification * 8.0 {
                shell_trace!(
                    "tangent support residual disagreement at source={original:?} result={point:?}: {aligned:?}"
                );
                return Err(BlendError::UnsupportedShape);
            }
            let residual = aligned.iter().sum::<f64>() / aligned.len() as f64;
            point = point - base_gradient * residual;
            let maximum_residual = supports
                .iter()
                .filter_map(|support| support_residual_gradient(support, point))
                .map(|(residual, _)| residual.abs())
                .fold(0.0, f64::max);
            if maximum_residual <= target_residual {
                break;
            }
        }
        let residual = supports
            .iter()
            .filter_map(|support| support_residual_gradient(support, point))
            .map(|(residual, _)| residual.abs())
            .fold(0.0, f64::max);
        if residual > context.policy.classification * 8.0 {
            shell_trace!(
                "tangent remap residual {residual} at source={original:?} result={point:?}"
            );
            return Err(BlendError::UnsupportedShape);
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
            shell_trace!("remap could not select independent support rows at {point:?}");
            return Err(BlendError::UnsupportedShape);
        };
        let determinant = det3(&rows);
        if determinant.abs() <= context.policy.angular {
            shell_trace!("remap determinant {determinant} at {point:?}");
            return Err(BlendError::UnsupportedShape);
        }
        let delta = solve3(&rows, &rhs, determinant);
        let step = GeomVec::new(delta[0], delta[1], delta[2]);
        point = point + step;
        let maximum_residual = supports
            .iter()
            .filter_map(|support| support_residual_gradient(support, point))
            .map(|(residual, _)| residual.abs())
            .fold(0.0, f64::max);
        if maximum_residual <= target_residual {
            break;
        }
    }
    let residual = supports
        .iter()
        .filter_map(|support| support_residual_gradient(support, point))
        .map(|(residual, _)| residual.abs())
        .fold(0.0, f64::max);
    if residual > context.policy.classification * 8.0 {
        if shell_trace_enabled() {
            let residuals = supports
                .iter()
                .filter_map(|support| support_residual_gradient(support, point))
                .collect::<Vec<_>>();
            shell_trace!(
                "remap residual {residual} at source={original:?} result={point:?}: {residuals:?}"
            );
        }
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

/// Exact image of a circular isocurve when a regular torus changes only its
/// minor radius.  Fillet bands are commonly split into several faces on one
/// support, so their shared seam has only one distinct support and cannot be
/// recovered through a two-surface intersection.
fn remap_torus_isocurve(
    edge: &Edge,
    curve: &GeomCurve,
    base: &ToroidalSurface,
    offset: &ToroidalSurface,
    offset_start: Pnt,
    offset_end: Pnt,
    context: &ToleranceContext,
) -> Option<GeomCurve> {
    if !matches!(curve, GeomCurve::Circle(_)) {
        return None;
    }
    let (domain_first, domain_last) = if edge.orientation() == openrcad_topo::Orientation::Reversed
    {
        (edge.last(), edge.first())
    } else {
        (edge.first(), edge.last())
    };
    let point = |fraction: f64| curve.point(domain_first + (domain_last - domain_first) * fraction);
    let start = crate::intersect::uv_of(&GeomSurface::torus(*base), &point(0.0));
    let middle = crate::intersect::uv_of(&GeomSurface::torus(*base), &point(0.5));
    let end = crate::intersect::uv_of(&GeomSurface::torus(*base), &point(1.0));
    let unwrap = |value: f64, reference: f64| {
        crate::native_pcurve::unwrap_near(value, reference, Some(core::f64::consts::TAU))
    };
    let middle = (unwrap(middle.0, start.0), unwrap(middle.1, start.1));
    let predicted_end = (2.0 * middle.0 - start.0, 2.0 * middle.1 - start.1);
    let end = (
        unwrap(end.0, predicted_end.0),
        unwrap(end.1, predicted_end.1),
    );
    let u_span = (start.0 - middle.0).abs().max((middle.0 - end.0).abs());
    let v_span = (start.1 - middle.1).abs().max((middle.1 - end.1).abs());
    if u_span > context.policy.angular * 8.0 && v_span > context.policy.angular * 8.0 {
        return None;
    }

    // Neighboring offset faces can move the two endpoints by different
    // amounts along the torus. The shared split seam then ceases to be a
    // circle, even though any exact curve on the common torus is a valid seam.
    // Preserve the source winding and connect those constraints with an exact
    // affine curve in torus parameter space.
    let target_start = crate::intersect::uv_of(&GeomSurface::torus(*offset), &offset_start);
    let target_end = crate::intersect::uv_of(&GeomSurface::torus(*offset), &offset_end);
    let target_start = (
        unwrap(target_start.0, start.0),
        unwrap(target_start.1, start.1),
    );
    let predicted_target_end = (
        target_start.0 + (end.0 - start.0),
        target_start.1 + (end.1 - start.1),
    );
    let target_end = (
        unwrap(target_end.0, predicted_target_end.0),
        unwrap(target_end.1, predicted_target_end.1),
    );
    TorusSurfaceCurve::new(
        *offset,
        (domain_first, domain_last),
        (target_start.0, target_end.0),
        (target_start.1, target_end.1),
    )
    .map(GeomCurve::torus_surface_curve)
}

/// Make an all-linear cylinder loop share one exact UV vertex per topological
/// vertex. At extreme translation/scale ratios, deriving a generator angle
/// from its world-space location loses more bits than deriving an adjacent
/// circular boundary from its analytic frame. Selecting the candidate that
/// lifts closest to the shared 3D vertex preserves the stronger construction
/// without introducing a tessellated or approximate B-Rep curve.
fn canonicalize_cylinder_wire_pcurves(
    surface: &GeomSurface,
    edges: &[Edge],
    pcurves: &mut [openrcad_topo::PcurveData],
) {
    let is_cylinder = matches!(surface, GeomSurface::Cylinder(_))
        || matches!(
            surface,
            GeomSurface::Offset(offset)
                if matches!(offset.base.as_ref(), GeomSurface::Cylinder(_))
        );
    if !is_cylinder
        || edges.len() < 2
        || edges.len() != pcurves.len()
        || pcurves
            .iter()
            .any(|pcurve| !matches!(&pcurve.curve, openrcad_geom2d::GeomCurve2d::Line(_)))
    {
        return;
    }

    let periodicity = pcurves[0].periodicity;
    let align = |point: Pnt2d, reference: Pnt2d| {
        Pnt2d::new(
            crate::native_pcurve::unwrap_near(point.x(), reference.x(), periodicity.u_period),
            crate::native_pcurve::unwrap_near(point.y(), reference.y(), periodicity.v_period),
        )
    };
    let mut starts = Vec::with_capacity(edges.len());
    let mut ends = Vec::with_capacity(edges.len());
    for (edge, pcurve) in edges.iter().zip(pcurves.iter()) {
        let (start, end) = if edge.orientation() == openrcad_topo::Orientation::Reversed {
            (pcurve.point_at_fraction(1.0), pcurve.point_at_fraction(0.0))
        } else {
            (pcurve.point_at_fraction(0.0), pcurve.point_at_fraction(1.0))
        };
        if let Some(previous) = ends.last().copied() {
            let aligned = align(start, previous);
            let shift = aligned - start;
            starts.push(aligned);
            ends.push(end + shift);
        } else {
            starts.push(start);
            ends.push(end);
        }
    }

    let mut vertices = Vec::with_capacity(edges.len());
    for index in 0..edges.len() {
        let current = starts[index];
        let previous = if index == 0 {
            align(ends[edges.len() - 1], current)
        } else {
            ends[index - 1]
        };
        let point = edges[index].source().point();
        let previous_error = surface.point(previous.x(), previous.y()).distance(&point);
        let current_error = surface.point(current.x(), current.y()).distance(&point);
        vertices.push(if previous_error < current_error {
            previous
        } else {
            current
        });
    }

    for index in 0..edges.len() {
        let traversal_start = vertices[index];
        let traversal_end = if index + 1 == edges.len() {
            align(vertices[0], traversal_start)
        } else {
            vertices[index + 1]
        };
        let (natural_start, natural_end) =
            if edges[index].orientation() == openrcad_topo::Orientation::Reversed {
                (traversal_end, traversal_start)
            } else {
                (traversal_start, traversal_end)
            };
        pcurves[index] = crate::native_pcurve::uv_line(natural_start, natural_end, periodicity);
    }
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
                .map_err(|error| trace_shell_failure("offset-support", error))
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
        // A removed opening is not an inner-shell constraint. Ordinary planar
        // walls happen to remain in the opening plane after offset, but a
        // tangent fillet band moves away from it and needs a ruled rim between
        // the original and offset boundaries.
        let closed_adjacent: Vec<_> = adjacent
            .iter()
            .copied()
            .filter(|index| !is_open[*index])
            .collect();
        let distinct = distinct_support_indices(&closed_adjacent, &supports, &context.policy);
        if distinct.is_empty() {
            return Err(BlendError::UnsupportedShape);
        }
        let mut analytic_owned: Vec<_> = distinct
            .iter()
            .map(|index| supports[*index].analytic.clone())
            .collect();
        let original = vertex_points[key];
        let opening_indices: Vec<_> = adjacent
            .iter()
            .copied()
            .filter(|index| is_open[*index])
            .collect();
        if let Some(&opening_index) = opening_indices.first() {
            let opening_plane = match &supports[opening_index].analytic {
                GeomSurface::Plane(plane) => plane,
                _ => return Err(BlendError::UnsupportedShape),
            };
            if opening_indices.iter().skip(1).any(|index| {
                !crate::boolean::surfaces_are_same_domain(
                    &supports[*index].analytic,
                    &supports[opening_index].analytic,
                    context
                        .policy
                        .classification
                        .max(context.policy.intersection),
                )
            }) {
                return Err(BlendError::UnsupportedShape);
            }
            let boundary_support = analytic_owned
                .iter()
                .find(|support| !matches!(support, GeomSurface::Plane(_)))
                .or_else(|| analytic_owned.first())
                .ok_or(BlendError::UnsupportedShape)?;
            let projected =
                remap_analytic_vertex(original, &[boundary_support], thickness, &context)?;
            let derived_opening_constraint =
                GeomSurface::plane(Plane::from_point_normal(projected, opening_plane.normal()));
            if analytic_owned.iter().all(|support| {
                !crate::boolean::surfaces_are_same_domain(
                    support,
                    &derived_opening_constraint,
                    context
                        .policy
                        .classification
                        .max(context.policy.intersection),
                )
            }) {
                analytic_owned.push(derived_opening_constraint);
            }
        }
        let analytic: Vec<_> = analytic_owned.iter().collect();
        if shell_trace_enabled() {
            shell_trace!(
                "remap vertex {original:?} supports={:?}",
                analytic
                    .iter()
                    .map(|support| crate::band_topology::BandSupportKind::of(support))
                    .collect::<Vec<_>>()
            );
        }
        offset_points.insert(
            *key,
            remap_analytic_vertex(original, &analytic, thickness, &context)
                .map_err(|error| trace_shell_failure("remap-vertex", error))?,
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
        let closed_adjacent: Vec<_> = adjacent
            .iter()
            .copied()
            .filter(|index| !is_open[*index])
            .collect();
        let distinct = distinct_support_indices(&closed_adjacent, &supports, &context.policy);
        // Opening boundaries need the retained support plus a plane parallel
        // to the removed face through the remapped edge. For ordinary
        // plane/cylinder and plane/cone rims this is the original plane; for a
        // shifted fillet band it is the exact parallel plane containing the
        // inner boundary. Keeping this constraint local to the edge avoids
        // incorrectly pinning every opening vertex to the removed face.
        let derived_opening_index = adjacent.iter().copied().find(|index| is_open[*index]);
        let derived_opening_constraint = derived_opening_index
            .and_then(|index| match &supports[index].analytic {
                GeomSurface::Plane(opening_plane) => Some(GeomSurface::plane(
                    Plane::from_point_normal(start, opening_plane.normal()),
                )),
                _ => None,
            })
            .filter(|plane| {
                support_residual_gradient(plane, end).is_some_and(|(residual, _)| {
                    residual.abs() <= context.policy.classification * 8.0
                })
            });
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
            if let (
                GeomSurface::Offset(offset_surface),
                GeomSurface::Torus(offset_torus),
                Some(curve),
            ) = (&support.display, &support.analytic, edge.curve())
            {
                if let GeomSurface::Torus(base_torus) = offset_surface.base.as_ref() {
                    let transformed = remap_torus_isocurve(
                        edge,
                        curve,
                        base_torus,
                        offset_torus,
                        start,
                        end,
                        &context,
                    )
                    .ok_or_else(|| {
                        shell_trace!("torus edge is not a supported exact isocurve");
                        BlendError::UnsupportedShape
                    })?;
                    let (start_parameter, end_parameter) = transformed.bounds();
                    if transformed.point(start_parameter).distance(&start)
                        > context.policy.classification * 8.0
                        || transformed.point(end_parameter).distance(&end)
                            > context.policy.classification * 8.0
                    {
                        if shell_trace_enabled() {
                            let start_uv =
                                crate::intersect::uv_of(&GeomSurface::torus(*offset_torus), &start);
                            let end_uv =
                                crate::intersect::uv_of(&GeomSurface::torus(*offset_torus), &end);
                            shell_trace!(
                                "torus edge projection mismatch start={} end={} parameters=({start_parameter}, {end_parameter}) uv=({start_uv:?}, {end_uv:?})",
                                transformed.point(start_parameter).distance(&start),
                                transformed.point(end_parameter).distance(&end)
                            );
                        }
                        return Err(BlendError::UnsupportedShape);
                    }
                    return Ok(Edge::new_with_tolerance(
                        Some(transformed),
                        start_parameter,
                        end_parameter,
                        Vertex::new(start),
                        Vertex::new(end),
                        context.policy.linear,
                    ));
                }
            }
        }
        let (first_support, second_support, first_source_index, second_source_index) =
            if distinct.len() >= 2 {
                (
                    &supports[distinct[0]].analytic,
                    &supports[distinct[1]].analytic,
                    distinct[0],
                    distinct[1],
                )
            } else if distinct.len() == 1 {
                (
                    &supports[distinct[0]].analytic,
                    derived_opening_constraint
                        .as_ref()
                        .ok_or(BlendError::UnsupportedShape)?,
                    distinct[0],
                    derived_opening_index.ok_or(BlendError::UnsupportedShape)?,
                )
            } else {
                return Err(BlendError::UnsupportedShape);
            };
        let contains_torus = matches!(first_support, GeomSurface::Torus(_))
            || matches!(second_support, GeomSurface::Torus(_));
        let mut boundary_budget = crate::band_topology::GeometryWorkBudget::intersection_default();
        let candidates = if contains_torus {
            boundary_budget.charge(crate::band_topology::GeometryWorkStage::Intersection, 1)?;
            crate::intersect::analytic_surface_surface(
                first_support,
                second_support,
                context.policy.intersection,
            )
            .ok_or_else(|| {
                BlendError::BandTopology(
                    crate::band_topology::BandTopologyError::UnsupportedTransition {
                        left: crate::band_topology::BandSupportKind::of(first_support),
                        right: crate::band_topology::BandSupportKind::of(second_support),
                    },
                )
            })?
        } else {
            crate::intersect::surface_surface_with_budget(
                first_support,
                second_support,
                context.policy.intersection,
                &mut boundary_budget,
            )?
        };
        let original_midpoint = edge
            .curve()
            .map(|curve| curve.point((edge.first() + edge.last()) * 0.5))
            .unwrap_or_else(|| edge.source().point().midpoint(&edge.target().point()));
        let mut best: Option<(f64, u32, GeomCurve, f64, f64)> = None;
        for (branch, curve) in candidates.into_iter().enumerate() {
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
            let end_parameter =
                crate::boolean::project_point_on_curve(&end, &curve, search_min, search_max);
            let end_parameters: Vec<_> = if curve.is_periodic() {
                let period = curve.period();
                let central_branch = ((start_parameter - end_parameter) / period).round();
                [-1.0, 0.0, 1.0]
                    .into_iter()
                    .map(|delta| end_parameter + (central_branch + delta) * period)
                    .collect()
            } else {
                vec![end_parameter]
            };
            for end_parameter in end_parameters {
                let start_error = curve.point(start_parameter).distance(&start);
                let end_error = curve.point(end_parameter).distance(&end);
                if start_error > context.policy.classification * 8.0
                    || end_error > context.policy.classification * 8.0
                {
                    continue;
                }
                // The source midpoint disambiguates the two complementary arcs
                // without relying on raw first/last ordering. Reversed coedges
                // therefore select the same geometric arc in opposite senses.
                let midpoint = curve.point((start_parameter + end_parameter) * 0.5);
                let score = midpoint.distance(&original_midpoint) + start_error + end_error;
                if best.as_ref().map_or(true, |(current, ..)| score < *current) {
                    best = Some((
                        score,
                        branch as u32,
                        curve.clone(),
                        start_parameter,
                        end_parameter,
                    ));
                }
            }
        }
        let (_, branch, curve, first, last) = best.ok_or(BlendError::UnsupportedShape)?;
        let boundary = crate::band_topology::build_two_sided_boundary(
            curve,
            first,
            last,
            start,
            end,
            first_support,
            second_support,
            InputTopologyRef::face(0, first_source_index).into(),
            InputTopologyRef::face(0, second_source_index).into(),
            branch,
            &context,
            &mut boundary_budget,
        )?;
        Ok(boundary.edge)
    };

    let rebuild_wire = |wire: &Wire,
                        surface: &GeomSurface|
     -> Result<(Wire, Vec<openrcad_topo::PcurveData>), BlendError> {
        let edges = wire
            .edges()
            .iter()
            .map(|edge| {
                rebuild_edge(edge).map_err(|error| {
                    if shell_trace_enabled() {
                        shell_trace!(
                            "edge rebuild failed curve={:?} source={:?} target={:?}",
                            edge.curve(),
                            edge.source().point(),
                            edge.target().point()
                        );
                    }
                    error
                })
            })
            .collect::<Result<Vec<_>, _>>()?;
        let mut pcurves: Vec<openrcad_topo::PcurveData> = edges
            .iter()
            .map(|edge| {
                openrcad_topo::pcurve_build::exact_pcurve_for_edge(surface, edge, &context.policy)
                    .unwrap_or_else(|| crate::native_pcurve::analytic_line_pcurve(surface, edge))
            })
            .collect();
        canonicalize_cylinder_wire_pcurves(surface, &edges, &mut pcurves);
        Ok((Wire::from_edges(edges), pcurves))
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
                    let inner = rebuild_edge(&edge)
                        .map_err(|error| trace_shell_failure("opening-rim-edge", error))?;
                    let outer_start = edge.source().point();
                    let outer_end = edge.target().point();
                    let inner_start = inner.source().point();
                    let inner_end = inner.target().point();
                    let inner_is_coplanar = [inner_start, inner_end].into_iter().all(|point| {
                        support_residual_gradient(&rim_surface, point).is_some_and(
                            |(residual, _)| residual.abs() <= context.policy.classification * 8.0,
                        )
                    });
                    let (wall_surface, inner_boundary) = if inner_is_coplanar {
                        (rim_surface.clone(), inner.reversed())
                    } else {
                        let outer_curve =
                            edge.curve().cloned().ok_or(BlendError::UnsupportedShape)?;
                        let inner_curve =
                            inner.curve().cloned().ok_or(BlendError::UnsupportedShape)?;
                        // A ruled surface requires both rails to describe the
                        // corresponding points at the same `u`. Offset
                        // intersections can move an endpoint along the inner
                        // rail, so its native interval is generally different.
                        // Align it exactly with an affine parameter map instead
                        // of approximating the rail or weakening pcurve checks.
                        let reversed = edge.orientation() == openrcad_topo::Orientation::Reversed;
                        let (target_first, target_last, aligned_start, aligned_end) = if reversed {
                            (inner.last(), inner.first(), inner.end(), inner.start())
                        } else {
                            (inner.first(), inner.last(), inner.start(), inner.end())
                        };
                        let aligned_curve = GeomCurve::reparametrized(
                            ReparametrizedCurve::new(
                                inner_curve,
                                edge.first(),
                                edge.last(),
                                target_first,
                                target_last,
                            )
                            .ok_or(BlendError::UnsupportedShape)?,
                        );
                        let aligned_inner = Edge::new_with_tolerance(
                            Some(aligned_curve.clone()),
                            edge.first(),
                            edge.last(),
                            aligned_start,
                            aligned_end,
                            context.policy.linear,
                        );
                        let inner_boundary = if reversed {
                            aligned_inner
                        } else {
                            aligned_inner.reversed()
                        };
                        (
                            GeomSurface::ruled(RuledSurface::new(outer_curve, aligned_curve)),
                            inner_boundary,
                        )
                    };
                    let end_connector =
                        Edge::between_points(outer_end, inner_boundary.source().point());
                    let start_connector =
                        Edge::between_points(inner_boundary.target().point(), outer_start);
                    let rim_wire = Wire::from_edges([
                        edge.clone(),
                        end_connector,
                        inner_boundary,
                        start_connector,
                    ]);
                    if matches!(wall_surface, GeomSurface::Ruled(_)) {
                        let source_parameter =
                            if edge.orientation() == openrcad_topo::Orientation::Reversed {
                                edge.last()
                            } else {
                                edge.first()
                            };
                        let target_parameter =
                            if edge.orientation() == openrcad_topo::Orientation::Reversed {
                                edge.first()
                            } else {
                                edge.last()
                            };
                        let pcurves = vec![
                            crate::native_pcurve::uv_line(
                                Pnt2d::new(edge.first(), 0.0),
                                Pnt2d::new(edge.last(), 0.0),
                                openrcad_topo::SurfacePeriodicity::NONE,
                            ),
                            crate::native_pcurve::uv_line(
                                Pnt2d::new(target_parameter, 0.0),
                                Pnt2d::new(target_parameter, 1.0),
                                openrcad_topo::SurfacePeriodicity::NONE,
                            ),
                            crate::native_pcurve::uv_line(
                                Pnt2d::new(edge.first(), 1.0),
                                Pnt2d::new(edge.last(), 1.0),
                                openrcad_topo::SurfacePeriodicity::NONE,
                            ),
                            crate::native_pcurve::uv_line(
                                Pnt2d::new(source_parameter, 1.0),
                                Pnt2d::new(source_parameter, 0.0),
                                openrcad_topo::SurfacePeriodicity::NONE,
                            ),
                        ];
                        result.push(Face::with_pcurves(wall_surface, rim_wire, pcurves).map_err(
                            |error| BlendError::InvalidTolerancePolicy(error.to_string()),
                        )?);
                    } else {
                        result.push(
                            crate::native_pcurve::analytic_face_with_pcurves(
                                wall_surface,
                                rim_wire,
                                openrcad_topo::Orientation::Forward,
                            )
                            .map_err(|error| {
                                BlendError::InvalidTolerancePolicy(error.to_string())
                            })?,
                        );
                    }
                }
            }
            continue;
        }
        result.push(face.clone());
        let (outer, outer_pcurves) = face
            .outer_wire()
            .map(|wire| {
                rebuild_wire(&wire, &supports[face_index].display)
                    .map_err(|error| trace_shell_failure("offset-outer-wire", error))
            })
            .transpose()?
            .ok_or(BlendError::UnsupportedShape)?;
        let inners = face
            .inner_wires()
            .iter()
            .map(|wire| {
                rebuild_wire(wire, &supports[face_index].display)
                    .map_err(|error| trace_shell_failure("offset-inner-wire", error))
            })
            .collect::<Result<Vec<_>, _>>()?;
        let desired = supports[face_index].checked_outward.reversed();
        let orientation = if desired.dot(&supports[face_index].base_normal) >= 0.0 {
            openrcad_topo::Orientation::Forward
        } else {
            openrcad_topo::Orientation::Reversed
        };
        result.push(
            Face::with_wires_and_pcurves(
                Some(supports[face_index].display.clone()),
                Some((outer, outer_pcurves)),
                inners,
                orientation,
            )
            .map_err(|error| BlendError::InvalidTolerancePolicy(error.to_string()))?,
        );
    }

    shell_trace!("sewing {} candidate faces", result.len());
    let shell = sew_with_policy(&result, &context.policy).map_err(|error| {
        trace_shell_failure(
            "sewing",
            BlendError::InvalidTolerancePolicy(error.to_string()),
        )
    })?;
    let candidate = Solid::new(shell);
    if !candidate.is_watertight_with_policy(&context.policy) {
        if shell_trace_enabled() {
            shell_trace!(
                "candidate is not watertight: health={:?} manifold={:?} counts=({}, {}, {})",
                candidate.health_report_with_policy(&context.policy),
                candidate.manifold_report_with_policy(&context.policy),
                candidate.vertex_count(),
                candidate.edge_count(),
                candidate.face_count()
            );
            let grid = 1.0 / context.policy.approximation;
            let quant = |point: Pnt| {
                (
                    (point.x() * grid).round() as i64,
                    (point.y() * grid).round() as i64,
                    (point.z() * grid).round() as i64,
                )
            };
            let mut uses = std::collections::HashMap::new();
            for (face_index, face) in candidate.faces().iter().enumerate() {
                for edge in face.wires().into_iter().flat_map(|wire| wire.edges()) {
                    let start = edge.start().point();
                    let end = edge.end().point();
                    let middle = edge.curve().map_or_else(
                        || start.midpoint(&end),
                        |curve| curve.point((edge.first() + edge.last()) * 0.5),
                    );
                    let (first, second) = (quant(start), quant(end));
                    let key = if first <= second {
                        (first, second, quant(middle))
                    } else {
                        (second, first, quant(middle))
                    };
                    let kind = match edge.curve() {
                        Some(GeomCurve::Line(_)) => "line",
                        Some(GeomCurve::Circle(_)) => "circle",
                        Some(GeomCurve::Ellipse(_)) => "ellipse",
                        Some(GeomCurve::Parabola(_)) => "parabola",
                        Some(GeomCurve::Hyperbola(_)) => "hyperbola",
                        Some(GeomCurve::BSpline(_)) => "bspline",
                        Some(GeomCurve::Helix(_)) => "helix",
                        Some(GeomCurve::TorusPlaneSection(_)) => "torus-plane",
                        Some(GeomCurve::Reparametrized(_)) => "reparametrized",
                        Some(GeomCurve::TorusSurfaceCurve(_)) => "torus-surface",
                        None => "none",
                    };
                    uses.entry(key)
                        .or_insert_with(Vec::new)
                        .push((face_index, kind, start, end, middle));
                }
            }
            for entries in uses.values().filter(|entries| entries.len() == 1) {
                shell_trace!("free edge: {:?}", entries[0]);
            }
        }
        return Err(BlendError::UnsupportedShape);
    }
    if shell_trace_enabled() {
        for (face_index, face) in candidate.faces().iter().enumerate() {
            let surface = match face.surface() {
                Some(GeomSurface::Plane(_)) => "plane",
                Some(GeomSurface::Cylinder(_)) => "cylinder",
                Some(GeomSurface::Cone(_)) => "cone",
                Some(GeomSurface::Sphere(_)) => "sphere",
                Some(GeomSurface::Torus(_)) => "torus",
                Some(GeomSurface::BSpline(_)) => "bspline",
                Some(GeomSurface::Gregory(_)) => "gregory",
                Some(GeomSurface::Offset(_)) => "offset",
                Some(GeomSurface::Ruled(_)) => "ruled",
                None => "none",
            };
            for (wire_index, wire) in face.wires().iter().enumerate() {
                for edge_index in 0..wire.len() {
                    if wire.pcurve(edge_index).is_none() {
                        shell_trace!(
                            "missing pcurve face_index={face_index} face_id={:?} surface={surface} wire={wire_index} edge={edge_index}",
                            face.id()
                        );
                    }
                }
            }
        }
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
