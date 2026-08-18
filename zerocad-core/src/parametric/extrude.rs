use super::*;

#[derive(Debug, Clone, PartialEq)]
pub enum ExtrudeDraftError {
    InvalidAngle(f32),
    UnsupportedCurves,
    Offset(openrcad::sketch::OffsetError),
    ProfileCollapse,
    ChangedEdgeCorrespondence,
    RegionCollision,
    KernelFailure,
}

impl std::fmt::Display for ExtrudeDraftError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidAngle(angle) => write!(
                formatter,
                "draft angle {angle:.3}° is invalid; it must be finite and strictly between -89° and 89°"
            ),
            Self::UnsupportedCurves => formatter.write_str(
                "draft v1 supports only planar straight-edged regions; circular, arc, ellipse, and spline boundaries remain unresolved",
            ),
            Self::Offset(error) => write!(formatter, "draft profile offset failed: {error}"),
            Self::ProfileCollapse => formatter.write_str(
                "draft collapses or inverts a material boundary before the far section",
            ),
            Self::ChangedEdgeCorrespondence => formatter.write_str(
                "draft changed edge correspondence between the base and far sections",
            ),
            Self::RegionCollision => formatter.write_str(
                "drafted far sections collide or touch; the multi-region result is ambiguous",
            ),
            Self::KernelFailure => {
                formatter.write_str("the kernel could not build a closed drafted solid")
            }
        }
    }
}

impl std::error::Error for ExtrudeDraftError {}

#[derive(Debug, Clone)]
pub(crate) struct DraftedRegionLoops {
    pub(crate) top_boundary: Vec<(f32, f32)>,
    pub(crate) top_holes: Vec<Vec<(f32, f32)>>,
}

#[derive(Debug)]
pub(crate) enum ExactFaceExtrudeError {
    MissingOwner,
    TargetMismatch { target: String, owner: String },
    MissingBody(String),
    Unresolved,
    Ambiguous,
    UnsupportedSurface,
    InvalidDepth,
    Prism(String),
}

impl std::fmt::Display for ExactFaceExtrudeError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::MissingOwner => formatter.write_str(
                "the selected face has no owning body; reselect the face before retrying",
            ),
            Self::TargetMismatch { target, owner } => write!(
                formatter,
                "the selected face belongs to body '{owner}', but the feature targets '{target}'"
            ),
            Self::MissingBody(body) => {
                write!(formatter, "the selected face's body '{body}' no longer exists")
            }
            Self::Unresolved => formatter.write_str(
                "the selected face could not be resolved to one exact B-Rep face",
            ),
            Self::Ambiguous => formatter.write_str(
                "the legacy face reference matches more than one exact B-Rep face; reselect it to record a durable face identity",
            ),
            Self::UnsupportedSurface => {
                formatter.write_str("the selected face is not planar")
            }
            Self::InvalidDepth => {
                formatter.write_str("the extrusion distance must be finite and non-zero")
            }
            Self::Prism(reason) => {
                write!(formatter, "OpenRCAD could not construct the exact face prism ({reason})")
            }
        }
    }
}

struct ExactFaceCandidate {
    component_index: usize,
    face: openrcad::topo::Face,
    centroid: [f32; 3],
    normal: [f32; 3],
}

#[derive(Debug)]
pub(crate) struct ResolvedExactPlanarFace {
    pub(crate) body_id: String,
    pub(crate) component_index: usize,
    pub(crate) face: openrcad::topo::Face,
    pub(crate) normal: Vec3,
}

fn capture_ulp(value: f32) -> f32 {
    let value = value.abs();
    if !value.is_finite() {
        return f32::INFINITY;
    }
    if value == 0.0 {
        return f32::from_bits(1);
    }
    let next = f32::from_bits(value.to_bits().saturating_add(1));
    (next - value).abs()
}

fn legacy_face_match_tolerance(body: &LiveBody, reference: &FaceRef) -> f32 {
    let model_scale = body
        .parts
        .iter()
        .filter_map(crate::mock_kernel::solid_aabb)
        .map(|(min, max)| {
            (0..3)
                .map(|axis| (max[axis] - min[axis]).abs())
                .fold(0.0_f32, f32::max)
        })
        .fold(0.0_f32, f32::max);
    let coordinate_ulp = reference
        .centroid
        .iter()
        .copied()
        .map(capture_ulp)
        .fold(0.0_f32, f32::max);
    let policy = openrcad::foundation::TolerancePolicy::STANDARD;
    let policy_floor = policy
        .classification
        .max(policy.intersection)
        .max(policy.sewing) as f32
        * 32.0;
    policy_floor
        .max(coordinate_ulp * 8.0)
        .max((model_scale * 1.0e-4).min(0.1))
}

fn exact_face_candidates(
    body: &LiveBody,
    reference: &FaceRef,
    component_filter: Option<usize>,
    requested_name: Option<&str>,
) -> Vec<ExactFaceCandidate> {
    let reference_mesh = body
        .pristine
        .as_deref()
        .cloned()
        .unwrap_or_else(|| edge_mod_reference_mesh(body));
    let reference_normal = Vec3::new(
        reference.normal[0],
        reference.normal[1],
        reference.normal[2],
    )
    .normalize();
    let mut candidates = Vec::new();
    for (component_index, component) in body.parts.iter().enumerate() {
        if component_filter.is_some_and(|filter| filter != component_index) {
            continue;
        }
        let shell_faces = component.shell().faces();
        let names = requested_name
            .map(|_| crate::mock_kernel::input_shell_face_names(&reference_mesh, component));
        let component_mesh = MockMesh::from_solid(component);
        let mut seen = std::collections::HashSet::new();
        for face_ref in &component_mesh.face_refs {
            let shell_index = face_ref.face_id as usize;
            if !seen.insert(shell_index) {
                continue;
            }
            let Some(face) = shell_faces.get(shell_index) else {
                continue;
            };
            if !matches!(face.surface(), Some(openrcad::geom::GeomSurface::Plane(_))) {
                continue;
            }
            if requested_name.is_some_and(|requested| {
                names
                    .as_ref()
                    .and_then(|names| names.get(shell_index))
                    .and_then(|name| name.as_deref())
                    != Some(requested)
            }) {
                continue;
            }
            let normal =
                Vec3::new(face_ref.normal[0], face_ref.normal[1], face_ref.normal[2]).normalize();
            if normal.dot(reference_normal) < 0.999 {
                continue;
            }
            candidates.push(ExactFaceCandidate {
                component_index,
                face: face.clone(),
                centroid: face_ref.centroid,
                normal: face_ref.normal,
            });
        }
    }
    candidates.sort_by(|left, right| {
        crate::mock_kernel::dist3(left.centroid, reference.centroid).total_cmp(
            &crate::mock_kernel::dist3(right.centroid, reference.centroid),
        )
    });
    candidates
}

/// Resolve a display-mesh face capture back to exactly one planar kernel face.
/// Durable names win. Legacy unnamed captures are accepted only when one exact
/// candidate lies within the f32 capture envelope; ambiguous captures never
/// silently choose whichever shell face happens to enumerate first.
pub(crate) fn resolve_exact_planar_face(
    live: &[LiveBody],
    reference: &FaceRef,
    boolean_target: Option<&str>,
) -> Result<ResolvedExactPlanarFace, ExactFaceExtrudeError> {
    let captured_owner = reference
        .topology
        .as_ref()
        .and_then(|topology| topology.body_id.as_deref());
    if let (Some(target), Some(owner)) = (boolean_target, captured_owner) {
        if target != owner {
            return Err(ExactFaceExtrudeError::TargetMismatch {
                target: target.to_string(),
                owner: owner.to_string(),
            });
        }
    }
    let body_id = boolean_target
        .or(captured_owner)
        .ok_or(ExactFaceExtrudeError::MissingOwner)?;
    let body = live
        .iter()
        .find(|body| body.id == body_id)
        .ok_or_else(|| ExactFaceExtrudeError::MissingBody(body_id.to_string()))?;
    let resolved =
        resolve_face_on_body(body, reference).ok_or(ExactFaceExtrudeError::Unresolved)?;
    let requested_name = reference
        .topology
        .as_ref()
        .and_then(|topology| topology.face_id.as_deref());
    let has_component_name = reference
        .topology
        .as_ref()
        .and_then(|topology| topology.component_id.as_deref())
        .is_some();
    let component_filter =
        (requested_name.is_some() || has_component_name).then_some(resolved.component_index);
    let candidates = exact_face_candidates(body, reference, component_filter, requested_name);

    let candidate = if requested_name.is_some() {
        match candidates.as_slice() {
            [candidate] => candidate,
            [] => return Err(ExactFaceExtrudeError::Unresolved),
            _ => {
                let tolerance = legacy_face_match_tolerance(body, reference);
                let mut close = candidates.iter().filter(|candidate| {
                    crate::mock_kernel::dist3(candidate.centroid, reference.centroid) <= tolerance
                });
                let Some(candidate) = close.next() else {
                    return Err(ExactFaceExtrudeError::Ambiguous);
                };
                if close.next().is_some() {
                    return Err(ExactFaceExtrudeError::Ambiguous);
                }
                candidate
            }
        }
    } else {
        let tolerance = legacy_face_match_tolerance(body, reference);
        let mut close = candidates.iter().filter(|candidate| {
            crate::mock_kernel::dist3(candidate.centroid, reference.centroid) <= tolerance
        });
        let Some(candidate) = close.next() else {
            return Err(ExactFaceExtrudeError::Unresolved);
        };
        if close.next().is_some() {
            return Err(ExactFaceExtrudeError::Ambiguous);
        }
        candidate
    };

    let normal = Vec3::new(
        candidate.normal[0],
        candidate.normal[1],
        candidate.normal[2],
    )
    .normalize();
    if normal == Vec3::ZERO {
        return Err(ExactFaceExtrudeError::UnsupportedSurface);
    }
    Ok(ResolvedExactPlanarFace {
        body_id: body_id.to_string(),
        component_index: candidate.component_index,
        face: candidate.face.clone(),
        normal,
    })
}

fn exact_face_recovery_overlap(component: &KernelSolid) -> f32 {
    let model_scale = crate::mock_kernel::solid_aabb(component)
        .map(|(min, max)| {
            (0..3)
                .map(|axis| (max[axis] - min[axis]).abs())
                .fold(0.0_f32, f32::max)
        })
        .unwrap_or(0.0);
    let policy = openrcad::foundation::TolerancePolicy::STANDARD;
    let policy_floor = policy
        .classification
        .max(policy.intersection)
        .max(policy.sewing) as f32
        * 32.0;
    policy_floor.max(model_scale * 1.0e-5).min(0.1)
}

fn exact_face_prism(
    face: &openrcad::topo::Face,
    normal: Vec3,
    start_offset: f32,
    end_offset: f32,
) -> Result<KernelSolid, ExactFaceExtrudeError> {
    let start = if start_offset.abs() <= f32::EPSILON {
        face.clone()
    } else {
        face.transformed(&openrcad::foundation::Trsf::translation(
            openrcad::foundation::Vec::new(
                f64::from(normal.x * start_offset),
                f64::from(normal.y * start_offset),
                f64::from(normal.z * start_offset),
            ),
        ))
    };
    let sweep = end_offset - start_offset;
    let vector = openrcad::foundation::Vec::new(
        f64::from(normal.x * sweep),
        f64::from(normal.y * sweep),
        f64::from(normal.z * sweep),
    );
    crate::mock_kernel::consume_operation(
        "exact selected-face prism",
        openrcad::algo::prism_operation_with_policy(
            &start,
            vector,
            &openrcad::foundation::TolerancePolicy::STANDARD,
        ),
    )
    .map(|outcome| outcome.solid)
    .map_err(|reason| ExactFaceExtrudeError::Prism(reason.to_string()))
}

fn stamp_direct_face_extrude_refs(mesh: &mut MockMesh, body_id: &str) {
    let quant = |value: f32| (f64::from(value) * 1.0e3).round() as i64;
    let mut order: Vec<usize> = (0..mesh.face_refs.len()).collect();
    order.sort_by_key(|&index| {
        let centroid = mesh.face_refs[index].centroid;
        (quant(centroid[0]), quant(centroid[1]), quant(centroid[2]))
    });
    for (face_index, index) in order.into_iter().enumerate() {
        mesh.face_refs[index].topology = Some(crate::mock_kernel::MeshTopologyFaceRef {
            body_id: Some(body_id.to_string()),
            component_id: None,
            topology_version: Some(0),
            face_id: Some(format!("face-extrude:{body_id}:face:{face_index}")),
            surface_kind: None,
            producer_feature_id: Some(body_id.to_string()),
            source_entity_id: None,
        });
    }
    crate::mock_kernel::populate_edge_adjacent_face_names(mesh);
}

/// Execute an outline-only face extrusion from the selected face's exact
/// OpenRCAD wires. The coincident prism is always attempted first. Recovery
/// extends only the hidden/entry end of the boolean tool, so the requested far
/// plane remains exact and no tolerance adjustment enters model dimensions.
#[allow(clippy::too_many_arguments)]
pub(crate) fn apply_exact_face_extrude(
    node_id: &str,
    depth: f32,
    mode: ExtrudeMode,
    boolean_target: Option<&str>,
    reference: &FaceRef,
    draft: bool,
    live: &mut Vec<LiveBody>,
    warnings: &mut Vec<String>,
) {
    if !depth.is_finite() || depth.abs() <= 1.0e-5 {
        warnings.push(format!(
            "Extrude '{node_id}': {}; the feature was not applied.",
            ExactFaceExtrudeError::InvalidDepth
        ));
        return;
    }
    let resolved = match resolve_exact_planar_face(live, reference, boolean_target) {
        Ok(resolved) => resolved,
        Err(error) => {
            warnings.push(format!(
                "Extrude '{node_id}': {error}; the feature was not applied."
            ));
            return;
        }
    };
    let Some(component) = live
        .iter()
        .find(|body| body.id == resolved.body_id)
        .and_then(|body| body.parts.get(resolved.component_index))
    else {
        warnings.push(format!(
            "Extrude '{node_id}': the resolved body component no longer exists; the feature was not applied."
        ));
        return;
    };
    let overlap = exact_face_recovery_overlap(component);
    let exact = match exact_face_prism(&resolved.face, resolved.normal, 0.0, depth) {
        Ok(exact) => exact,
        Err(error) => {
            warnings.push(format!(
                "Extrude '{node_id}': {error}; the feature was not applied."
            ));
            return;
        }
    };

    match mode {
        ExtrudeMode::NewBody => {
            let mut mesh = MockMesh::from_solid(&exact);
            stamp_direct_face_extrude_refs(&mut mesh, node_id);
            apply_new(
                live,
                LiveBody {
                    id: node_id.into(),
                    parts: vec![exact],
                    pristine: (!mesh.indices.is_empty()).then(|| std::sync::Arc::new(mesh)),
                    sketch_source: None,
                },
            );
        }
        ExtrudeMode::Join => {
            let dipped = exact_face_prism(&resolved.face, resolved.normal, -overlap, depth).ok();
            apply_join(
                live,
                node_id,
                vec![JoinTool {
                    smooth: None,
                    exact: Some(exact),
                    dipped,
                    profile: None,
                }],
                Some(&resolved.body_id),
                draft,
                warnings,
            );
        }
        ExtrudeMode::Cut => {
            let expanded = exact_face_prism(&resolved.face, resolved.normal, overlap, depth).ok();
            let exact_rev = exact_face_prism(&resolved.face, resolved.normal, 0.0, -depth).ok();
            let expanded_rev =
                exact_face_prism(&resolved.face, resolved.normal, overlap, -depth).ok();
            apply_cut(
                live,
                node_id,
                vec![CutTool {
                    smooth: None,
                    exact: Some(exact),
                    expanded,
                    smooth_rev: None,
                    exact_rev,
                    expanded_rev,
                    circle: None,
                }],
                Some(&resolved.body_id),
                draft,
                warnings,
            );
        }
    }
}

fn signed_area(points: &[(f32, f32)]) -> f64 {
    points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .take(points.len())
        .map(|(&(ax, ay), &(bx, by))| f64::from(ax) * f64::from(by) - f64::from(bx) * f64::from(ay))
        .sum::<f64>()
        * 0.5
}

fn line_loop(
    points: &[(f32, f32)],
    loop_index: usize,
) -> Result<openrcad::sketch::ArrangementLoop<(usize, usize)>, ExtrudeDraftError> {
    use openrcad::foundation::{Dir2d, Pnt2d};
    use openrcad::geom2d::{CurveSpan, GeomCurve2d, Line2d};

    if points.len() < 3 || !signed_area(points).is_finite() {
        return Err(ExtrudeDraftError::ChangedEdgeCorrespondence);
    }
    let mut spans = Vec::with_capacity(points.len());
    for (edge_index, (&start, &end)) in points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .take(points.len())
        .enumerate()
    {
        let dx = f64::from(end.0 - start.0);
        let dy = f64::from(end.1 - start.1);
        let length = dx.hypot(dy);
        if !length.is_finite() || length <= 1.0e-7 {
            return Err(ExtrudeDraftError::ChangedEdgeCorrespondence);
        }
        spans.push(CurveSpan::new(
            GeomCurve2d::line(Line2d::from_point_dir(
                Pnt2d::new(f64::from(start.0), f64::from(start.1)),
                Dir2d::try_new(dx / length, dy / length)
                    .ok_or(ExtrudeDraftError::ChangedEdgeCorrespondence)?,
            )),
            0.0,
            length,
            (loop_index, edge_index),
        ));
    }
    Ok(openrcad::sketch::ArrangementLoop {
        spans,
        signed_area: signed_area(points),
    })
}

fn span_loop_points<P>(loop_: &openrcad::sketch::ArrangementLoop<P>) -> Vec<(f32, f32)> {
    loop_
        .spans
        .iter()
        .map(|span| {
            let point = span.start();
            (point.x() as f32, point.y() as f32)
        })
        .collect()
}

fn correspondence_is_unchanged(
    input: &openrcad::sketch::ArrangementRegion<(usize, usize)>,
    output: &openrcad::sketch::ArrangementRegion<(usize, usize)>,
) -> bool {
    let provenance = |loop_: &openrcad::sketch::ArrangementLoop<(usize, usize)>| {
        let mut ids: Vec<_> = loop_.spans.iter().map(|span| span.provenance).collect();
        ids.sort_unstable();
        ids
    };
    if provenance(&input.outer) != provenance(&output.outer)
        || input.holes.len() != output.holes.len()
    {
        return false;
    }
    let mut input_holes: Vec<_> = input.holes.iter().map(provenance).collect();
    let mut output_holes: Vec<_> = output.holes.iter().map(provenance).collect();
    input_holes.sort();
    output_holes.sort();
    input_holes == output_holes
}

pub(crate) fn drafted_region_loops(
    boundary: &[(f32, f32)],
    holes: &[Vec<(f32, f32)>],
    depth: f32,
    angle_deg: f32,
) -> Result<DraftedRegionLoops, ExtrudeDraftError> {
    if !angle_deg.is_finite() || angle_deg.abs() >= 89.0 {
        return Err(ExtrudeDraftError::InvalidAngle(angle_deg));
    }
    if angle_deg.abs() <= f32::EPSILON {
        return Ok(DraftedRegionLoops {
            top_boundary: boundary.to_vec(),
            top_holes: holes.to_vec(),
        });
    }
    let outer = line_loop(boundary, 0)?;
    let hole_loops = holes
        .iter()
        .enumerate()
        .map(|(index, hole)| line_loop(hole, index + 1))
        .collect::<Result<Vec<_>, _>>()?;
    let area = outer.area() - hole_loops.iter().map(|hole| hole.area()).sum::<f64>();
    let input = openrcad::sketch::ArrangementRegion {
        outer,
        holes: hole_loops,
        area,
    };
    // Product convention: positive draft removes material away from the sketch
    // plane (far section contracts); negative draft adds material (expands).
    let distance = -f64::from(depth.abs()) * f64::from(angle_deg).to_radians().tan();
    let output = openrcad::sketch::offset_material_region(
        &input,
        distance,
        openrcad::sketch::OffsetOptions { tolerance: 1.0e-6 },
    )
    .map_err(ExtrudeDraftError::Offset)?;
    if !correspondence_is_unchanged(&input, &output) {
        return Err(ExtrudeDraftError::ChangedEdgeCorrespondence);
    }
    let area_tolerance = 1.0e-8;
    let outer_moved_correctly = if distance > 0.0 {
        output.outer.area() > input.outer.area() + area_tolerance
    } else {
        output.outer.area() + area_tolerance < input.outer.area()
    };
    let holes_moved_correctly = input.holes.iter().all(|input_hole| {
        output
            .holes
            .iter()
            .find(|output_hole| {
                let mut input_ids: Vec<_> = input_hole
                    .spans
                    .iter()
                    .map(|span| span.provenance)
                    .collect();
                let mut output_ids: Vec<_> = output_hole
                    .spans
                    .iter()
                    .map(|span| span.provenance)
                    .collect();
                input_ids.sort_unstable();
                output_ids.sort_unstable();
                input_ids == output_ids
            })
            .is_some_and(|output_hole| {
                if distance > 0.0 {
                    output_hole.area() + area_tolerance < input_hole.area()
                } else {
                    output_hole.area() > input_hole.area() + area_tolerance
                }
            })
    });
    if !outer_moved_correctly || !holes_moved_correctly || output.area <= area_tolerance {
        return Err(ExtrudeDraftError::ProfileCollapse);
    }
    Ok(DraftedRegionLoops {
        top_boundary: span_loop_points(&output.outer),
        top_holes: output.holes.iter().map(span_loop_points).collect(),
    })
}

fn orientation(a: (f32, f32), b: (f32, f32), c: (f32, f32)) -> f64 {
    f64::from(b.0 - a.0) * f64::from(c.1 - a.1) - f64::from(b.1 - a.1) * f64::from(c.0 - a.0)
}

fn segments_touch(a: (f32, f32), b: (f32, f32), c: (f32, f32), d: (f32, f32)) -> bool {
    let (ab_c, ab_d) = (orientation(a, b, c), orientation(a, b, d));
    let (cd_a, cd_b) = (orientation(c, d, a), orientation(c, d, b));
    let tolerance = 1.0e-8;
    if ab_c.abs() <= tolerance
        || ab_d.abs() <= tolerance
        || cd_a.abs() <= tolerance
        || cd_b.abs() <= tolerance
    {
        let overlaps = |a: f32, b: f32, c: f32| c >= a.min(b) - 1.0e-6 && c <= a.max(b) + 1.0e-6;
        return (ab_c.abs() <= tolerance && overlaps(a.0, b.0, c.0) && overlaps(a.1, b.1, c.1))
            || (ab_d.abs() <= tolerance && overlaps(a.0, b.0, d.0) && overlaps(a.1, b.1, d.1))
            || (cd_a.abs() <= tolerance && overlaps(c.0, d.0, a.0) && overlaps(c.1, d.1, a.1))
            || (cd_b.abs() <= tolerance && overlaps(c.0, d.0, b.0) && overlaps(c.1, d.1, b.1));
    }
    (ab_c > 0.0) != (ab_d > 0.0) && (cd_a > 0.0) != (cd_b > 0.0)
}

fn loop_edges(points: &[(f32, f32)]) -> impl Iterator<Item = ((f32, f32), (f32, f32))> + '_ {
    points
        .iter()
        .copied()
        .zip(points.iter().copied().cycle().skip(1))
        .take(points.len())
}

fn point_in_loop(point: (f32, f32), polygon: &[(f32, f32)]) -> bool {
    let mut inside = false;
    for (a, b) in loop_edges(polygon) {
        if (a.1 > point.1) != (b.1 > point.1)
            && point.0 < (b.0 - a.0) * (point.1 - a.1) / (b.1 - a.1) + a.0
        {
            inside = !inside;
        }
    }
    inside
}

fn point_in_material(point: (f32, f32), region: &DraftedRegionLoops) -> bool {
    point_in_loop(point, &region.top_boundary)
        && !region
            .top_holes
            .iter()
            .any(|hole| point_in_loop(point, hole))
}

pub(crate) fn validate_drafted_region_set(
    regions: &[DraftedRegionLoops],
) -> Result<(), ExtrudeDraftError> {
    for first in 0..regions.len() {
        for second in first + 1..regions.len() {
            let first_loops = std::iter::once(&regions[first].top_boundary)
                .chain(regions[first].top_holes.iter());
            let second_loops: Vec<_> = std::iter::once(&regions[second].top_boundary)
                .chain(regions[second].top_holes.iter())
                .collect();
            for first_loop in first_loops {
                for second_loop in &second_loops {
                    if loop_edges(first_loop).any(|(a, b)| {
                        loop_edges(second_loop).any(|(c, d)| segments_touch(a, b, c, d))
                    }) {
                        return Err(ExtrudeDraftError::RegionCollision);
                    }
                }
            }
            if regions[first]
                .top_boundary
                .first()
                .is_some_and(|point| point_in_material(*point, &regions[second]))
                || regions[second]
                    .top_boundary
                    .first()
                    .is_some_and(|point| point_in_material(*point, &regions[first]))
            {
                return Err(ExtrudeDraftError::RegionCollision);
            }
        }
    }
    Ok(())
}

pub fn drafted_region_solid(
    boundary: &[(f32, f32)],
    holes: &[Vec<(f32, f32)>],
    depth: f32,
    cs: &CoordinateSystem,
    angle_deg: f32,
) -> Result<KernelSolid, ExtrudeDraftError> {
    let drafted = drafted_region_loops(boundary, holes, depth, angle_deg)?;
    let top_cs = cs.with_origin(cs.origin.add(cs.n.mul(depth)));
    let solid = crate::mock_kernel::lofted_solid(&[
        (*cs, boundary.to_vec(), holes.to_vec()),
        (top_cs, drafted.top_boundary, drafted.top_holes),
    ])
    .ok_or(ExtrudeDraftError::KernelFailure)?;
    let expected_faces = 2 + boundary.len() + holes.iter().map(Vec::len).sum::<usize>();
    if solid.shell().faces().len() != expected_faces {
        return Err(ExtrudeDraftError::ChangedEdgeCorrespondence);
    }
    Ok(solid)
}

fn shifted_loop_with_selected_edges(
    points: &[(f32, f32)],
    selected: &[bool],
    shift_into_material: f32,
    outer: bool,
) -> Result<Vec<(f32, f32)>, ExtrudeDraftError> {
    if points.len() < 3 || selected.len() != points.len() {
        return Err(ExtrudeDraftError::ChangedEdgeCorrespondence);
    }
    let orientation_sign = if signed_area(points) >= 0.0 {
        1.0
    } else {
        -1.0
    };
    let material_sign = if outer {
        orientation_sign
    } else {
        -orientation_sign
    };
    let mut lines = Vec::with_capacity(points.len());
    for (index, (&start, &end)) in points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .take(points.len())
        .enumerate()
    {
        let direction = (end.0 - start.0, end.1 - start.1);
        let length = direction.0.hypot(direction.1);
        if !length.is_finite() || length <= 1.0e-7 {
            return Err(ExtrudeDraftError::ChangedEdgeCorrespondence);
        }
        let amount = if selected[index] {
            shift_into_material
        } else {
            0.0
        };
        let offset = (
            -direction.1 / length * material_sign * amount,
            direction.0 / length * material_sign * amount,
        );
        lines.push((
            (start.0 + offset.0, start.1 + offset.1),
            (end.0 + offset.0, end.1 + offset.1),
        ));
    }
    let mut output = Vec::with_capacity(points.len());
    for index in 0..points.len() {
        let previous = lines[(index + points.len() - 1) % points.len()];
        let current = lines[index];
        let a = (previous.1 .0 - previous.0 .0, previous.1 .1 - previous.0 .1);
        let b = (current.1 .0 - current.0 .0, current.1 .1 - current.0 .1);
        let denominator = a.0 * b.1 - a.1 * b.0;
        if denominator.abs() <= 1.0e-8 {
            return Err(ExtrudeDraftError::ChangedEdgeCorrespondence);
        }
        let delta = (current.0 .0 - previous.0 .0, current.0 .1 - previous.0 .1);
        let t = (delta.0 * b.1 - delta.1 * b.0) / denominator;
        let point = (previous.0 .0 + a.0 * t, previous.0 .1 + a.1 * t);
        if !point.0.is_finite() || !point.1.is_finite() {
            return Err(ExtrudeDraftError::ProfileCollapse);
        }
        output.push(point);
    }
    if signed_area(&output).signum() != signed_area(points).signum()
        || signed_area(&output).abs() <= 1.0e-8
    {
        return Err(ExtrudeDraftError::ProfileCollapse);
    }
    Ok(output)
}

pub(crate) fn drafted_region_solid_selected(
    boundary: &[(f32, f32)],
    holes: &[Vec<(f32, f32)>],
    depth: f32,
    cs: &CoordinateSystem,
    angle_deg: f32,
    selected_outer: &[bool],
    selected_holes: &[Vec<bool>],
) -> Result<KernelSolid, ExtrudeDraftError> {
    if !angle_deg.is_finite() || angle_deg.abs() >= 89.0 {
        return Err(ExtrudeDraftError::InvalidAngle(angle_deg));
    }
    if selected_outer.len() != boundary.len()
        || selected_holes.len() != holes.len()
        || holes
            .iter()
            .zip(selected_holes)
            .any(|(hole, selected)| hole.len() != selected.len())
    {
        return Err(ExtrudeDraftError::ChangedEdgeCorrespondence);
    }
    let shift = depth.abs() * angle_deg.to_radians().tan();
    let top_boundary = shifted_loop_with_selected_edges(boundary, selected_outer, shift, true)?;
    let top_holes = holes
        .iter()
        .zip(selected_holes)
        .map(|(hole, selected)| shifted_loop_with_selected_edges(hole, selected, shift, false))
        .collect::<Result<Vec<_>, _>>()?;
    let drafted = DraftedRegionLoops {
        top_boundary: top_boundary.clone(),
        top_holes: top_holes.clone(),
    };
    validate_drafted_region_set(std::slice::from_ref(&drafted))?;
    let top_cs = cs.with_origin(cs.origin.add(cs.n.mul(depth)));
    let solid = crate::mock_kernel::lofted_solid(&[
        (*cs, boundary.to_vec(), holes.to_vec()),
        (top_cs, top_boundary, top_holes),
    ])
    .ok_or(ExtrudeDraftError::KernelFailure)?;
    let expected_faces = 2 + boundary.len() + holes.iter().map(Vec::len).sum::<usize>();
    if solid.shell().faces().len() != expected_faces {
        return Err(ExtrudeDraftError::ChangedEdgeCorrespondence);
    }
    Ok(solid)
}

#[cfg(test)]
mod draft_tests {
    use super::*;

    fn rectangle(min: f32, max: f32) -> Vec<(f32, f32)> {
        vec![(min, min), (max, min), (max, max), (min, max)]
    }

    #[test]
    fn signed_draft_offsets_the_far_section_analytically() {
        let boundary = rectangle(0.0, 10.0);
        let contracted = drafted_region_loops(&boundary, &[], 10.0, 5.0).unwrap();
        let expanded = drafted_region_loops(&boundary, &[], 10.0, -5.0).unwrap();
        let offset = 10.0 * 5.0_f32.to_radians().tan();
        let expanded_min = expanded
            .top_boundary
            .iter()
            .map(|point| point.0)
            .fold(f32::INFINITY, f32::min);
        let contracted_min = contracted
            .top_boundary
            .iter()
            .map(|point| point.0)
            .fold(f32::INFINITY, f32::min);
        assert!((contracted_min - offset).abs() < 1.0e-4);
        assert!((expanded_min + offset).abs() < 1.0e-4);
    }

    #[test]
    fn draft_preserves_holes_and_edge_correspondence() {
        let boundary = rectangle(0.0, 20.0);
        let hole = rectangle(7.0, 13.0);
        let drafted = drafted_region_loops(&boundary, std::slice::from_ref(&hole), 5.0, 3.0)
            .expect("straight holed region should draft");
        assert_eq!(drafted.top_boundary.len(), boundary.len());
        assert_eq!(drafted.top_holes.len(), 1);
        assert_eq!(drafted.top_holes[0].len(), hole.len());
        let solid = drafted_region_solid(&boundary, &[hole], 5.0, &CoordinateSystem::XY, 3.0)
            .expect("drafted holed solid");
        assert_eq!(solid.shell().faces().len(), 10);
    }

    #[test]
    fn invalid_and_collapsing_drafts_reject_atomically() {
        let boundary = rectangle(0.0, 2.0);
        assert!(matches!(
            drafted_region_loops(&boundary, &[], 2.0, 89.0),
            Err(ExtrudeDraftError::InvalidAngle(_))
        ));
        assert!(matches!(
            drafted_region_loops(&boundary, &[], 10.0, 45.0),
            Err(ExtrudeDraftError::Offset(_) | ExtrudeDraftError::ProfileCollapse)
        ));
    }

    #[test]
    fn expanding_far_sections_reject_region_collisions() {
        let first = drafted_region_loops(&rectangle(0.0, 4.0), &[], 5.0, -10.0).unwrap();
        let second_boundary = vec![(5.0, 0.0), (9.0, 0.0), (9.0, 4.0), (5.0, 4.0)];
        let second = drafted_region_loops(&second_boundary, &[], 5.0, -10.0).unwrap();
        assert_eq!(
            validate_drafted_region_set(&[first, second]),
            Err(ExtrudeDraftError::RegionCollision)
        );
    }
}

pub(crate) fn stamp_sketch_extrude_edge_refs(
    mesh: &mut MockMesh,
    body_id: &str,
    region_index: usize,
    provenance: Option<&RegionProvenance>,
    cs: &CoordinateSystem,
    depth: f32,
) {
    let Some(provenance) = provenance else {
        return;
    };
    if provenance.fragments.is_empty() {
        return;
    }

    // Occurrence numbers disambiguate edges that share a base id (a boolean can
    // split one design edge into several fragments). Assign them in GEOMETRIC
    // order — sorted by quantized endpoints — never in enumeration order:
    // edge_refs order varies with the per-process hash seed and with the
    // tessellation, and an occ id that names a different fragment each run
    // breaks captured references.
    let quant = |v: f32| (v as f64 * 1.0e3).round() as i64;
    let edge_sort_key = |p0: [f32; 3], p1: [f32; 3]| {
        let a = (quant(p0[0]), quant(p0[1]), quant(p0[2]));
        let b = (quant(p1[0]), quant(p1[1]), quant(p1[2]));
        if a <= b {
            (a, b)
        } else {
            (b, a)
        }
    };
    let mut by_base: HashMap<String, Vec<usize>> = HashMap::new();
    let mut base_ids: Vec<String> = Vec::with_capacity(mesh.edge_refs.len());
    for (i, edge_ref) in mesh.edge_refs.iter().enumerate() {
        let role = sketch_extrude_edge_role(edge_ref.p0, edge_ref.p1, cs, depth);
        let fragment_id = sketch_extrude_edge_fragment_id(edge_ref, provenance, cs)
            .unwrap_or_else(|| "unknown".to_string());
        let base_id =
            format!("sketch:{body_id}:region:{region_index}:fragment:{fragment_id}:role:{role}");
        by_base.entry(base_id.clone()).or_default().push(i);
        base_ids.push(base_id);
    }
    let mut assigned: Vec<Option<String>> = vec![None; mesh.edge_refs.len()];
    for (base_id, mut indices) in by_base {
        indices.sort_by_key(|&i| edge_sort_key(mesh.edge_refs[i].p0, mesh.edge_refs[i].p1));
        for (occurrence, &i) in indices.iter().enumerate() {
            assigned[i] = Some(if occurrence == 0 {
                base_id.clone()
            } else {
                format!("{base_id}:occ:{occurrence}")
            });
        }
    }
    for (i, edge_ref) in mesh.edge_refs.iter_mut().enumerate() {
        let edge_id = assigned[i].take().unwrap_or_else(|| base_ids[i].clone());
        let curve_kind = match edge_ref.curve {
            Some(EdgeCurveHint::Circle { .. }) => Some("circle".to_string()),
            Some(EdgeCurveHint::Line) => Some("line".to_string()),
            None => None,
        };
        edge_ref.topology = Some(MeshTopologyEdgeRef {
            body_id: Some(body_id.to_string()),
            topology_version: Some(0),
            edge_id: Some(edge_id),
            curve_kind,
            adjacent_surface_kinds: Vec::new(),
            adjacent_face_ids: Vec::new(),
            producer_feature_id: Some(body_id.to_string()),
            source_entity_id: None,
        });
    }
}

/// Stamp durable names onto an extruded region's **faces**, the face analogue of
/// [`stamp_sketch_extrude_edge_refs`]. Each cap becomes
/// `sketch:{body}:region:{i}:face:{top|bottom}`. Drafted walls supplied with
/// provenance are owned by the source curve fragment; compatibility calls
/// retain `sketch:{body}:region:{i}:face:side[:occ:n]`. Faces are the primary named
/// entity — an edge's identity derives from the pair of faces it separates — so
/// this is what a sketch-on-face placement or a cut/join target pins to.
pub(crate) fn stamp_sketch_extrude_face_refs(
    mesh: &mut MockMesh,
    body_id: &str,
    region_index: usize,
    provenance: Option<&RegionProvenance>,
    cs: &CoordinateSystem,
    depth: f32,
) {
    // Same geometric occurrence numbering as the edge stamp: side walls share
    // one base id, and which wall is `occ:N` must not depend on the hash seed
    // or the tessellation order. Sort by quantized centroid.
    let quant = |v: f32| (v as f64 * 1.0e3).round() as i64;
    let mut by_base: HashMap<String, Vec<usize>> = HashMap::new();
    for (i, face_ref) in mesh.face_refs.iter().enumerate() {
        let role = sketch_extrude_face_role(face_ref.normal, cs, depth);
        let fragment = if role == "side" {
            provenance.and_then(|provenance| {
                drafted_face_base_edge(mesh, face_ref.face_id, cs, depth)
                    .and_then(|edge| sketch_extrude_edge_fragment_id(&edge, provenance, cs))
            })
        } else {
            None
        };
        let base_id = fragment.map_or_else(
            || format!("sketch:{body_id}:region:{region_index}:face:{role}"),
            |fragment| {
                format!("sketch:{body_id}:region:{region_index}:face:wall:fragment:{fragment}")
            },
        );
        by_base.entry(base_id).or_default().push(i);
    }
    let mut assigned: Vec<Option<String>> = vec![None; mesh.face_refs.len()];
    for (base_id, mut indices) in by_base {
        indices.sort_by_key(|&i| {
            let c = mesh.face_refs[i].centroid;
            (quant(c[0]), quant(c[1]), quant(c[2]))
        });
        for (occurrence, &i) in indices.iter().enumerate() {
            assigned[i] = Some(if occurrence == 0 {
                base_id.clone()
            } else {
                format!("{base_id}:occ:{occurrence}")
            });
        }
    }
    for (i, face_ref) in mesh.face_refs.iter_mut().enumerate() {
        face_ref.topology = Some(crate::mock_kernel::MeshTopologyFaceRef {
            body_id: Some(body_id.to_string()),
            component_id: None,
            topology_version: Some(0),
            face_id: assigned[i].take(),
            surface_kind: None,
            producer_feature_id: Some(body_id.to_string()),
            source_entity_id: None,
        });
    }
}

pub(crate) fn drafted_face_base_edge(
    mesh: &MockMesh,
    face_id: u32,
    cs: &CoordinateSystem,
    depth: f32,
) -> Option<crate::mock_kernel::MeshEdgeRef> {
    let axis = cs.n.normalize();
    let tolerance = (depth.abs() * 1.0e-5).max(1.0e-6);
    let mut base_vertices: Vec<[f32; 3]> = Vec::new();
    for (triangle, triangle_face_id) in mesh
        .indices
        .chunks_exact(3)
        .zip(mesh.face_ids.iter().copied())
    {
        if triangle_face_id != face_id {
            continue;
        }
        for index in triangle {
            let offset = *index as usize * 6;
            let vertex = mesh.vertices.get(offset..offset + 3)?;
            let position = [vertex[0], vertex[1], vertex[2]];
            let plane_offset = Vec3::new(position[0], position[1], position[2])
                .sub(cs.origin)
                .dot(axis);
            if plane_offset.abs() <= tolerance
                && !base_vertices.iter().any(|candidate| {
                    (candidate[0] - position[0]).powi(2)
                        + (candidate[1] - position[1]).powi(2)
                        + (candidate[2] - position[2]).powi(2)
                        <= tolerance * tolerance
                })
            {
                base_vertices.push(position);
            }
        }
    }
    let mut pair = None;
    let mut greatest_distance = 0.0_f32;
    for first in 0..base_vertices.len() {
        for second in first + 1..base_vertices.len() {
            let distance = (base_vertices[first][0] - base_vertices[second][0]).powi(2)
                + (base_vertices[first][1] - base_vertices[second][1]).powi(2)
                + (base_vertices[first][2] - base_vertices[second][2]).powi(2);
            if distance > greatest_distance {
                greatest_distance = distance;
                pair = Some((base_vertices[first], base_vertices[second]));
            }
        }
    }
    let (p0, p1) = pair?;
    Some(crate::mock_kernel::MeshEdgeRef {
        group: 0,
        p0,
        p1,
        n1: [0.0; 3],
        n2: [0.0; 3],
        curve: Some(EdgeCurveHint::Line),
        topology: None,
    })
}

/// Stamp durable PRIMITIVE names onto a box body's faces:
/// `box_{node}:face:{+x|-x|+y|-y|+z|-z}` by dominant outward normal. Primitive
/// faces were the last unnamed creation path — without these a fillet or a
/// sketch placed on a primitive box could only reattach geometrically.
pub(crate) fn stamp_box_face_refs(mesh: &mut MockMesh, body_id: &str) {
    for face_ref in mesh.face_refs.iter_mut() {
        let n = face_ref.normal;
        let (ax, ay, az) = (n[0].abs(), n[1].abs(), n[2].abs());
        let role = if ax >= ay && ax >= az {
            if n[0] >= 0.0 {
                "+x"
            } else {
                "-x"
            }
        } else if ay >= az {
            if n[1] >= 0.0 {
                "+y"
            } else {
                "-y"
            }
        } else if n[2] >= 0.0 {
            "+z"
        } else {
            "-z"
        };
        face_ref.topology = Some(crate::mock_kernel::MeshTopologyFaceRef {
            body_id: Some(body_id.to_string()),
            component_id: None,
            topology_version: Some(0),
            face_id: Some(format!("box_{body_id}:face:{role}")),
            surface_kind: Some("plane".to_string()),
            producer_feature_id: Some(body_id.to_string()),
            source_entity_id: None,
        });
    }
}

/// Stamp durable names onto a STEP-imported body's faces:
/// `import:{node}:face:{k}` with `k` assigned in quantized-centroid order, so
/// the same file yields the same names every load (mesh face order can vary
/// with the per-process hash seed; geometry cannot).
pub(crate) fn stamp_import_face_refs(mesh: &mut MockMesh, body_id: &str) {
    let quant = |v: f32| (v as f64 * 1.0e3).round() as i64;
    let mut order: Vec<usize> = (0..mesh.face_refs.len()).collect();
    order.sort_by_key(|&i| {
        let c = mesh.face_refs[i].centroid;
        (quant(c[0]), quant(c[1]), quant(c[2]))
    });
    for (k, &i) in order.iter().enumerate() {
        mesh.face_refs[i].topology = Some(crate::mock_kernel::MeshTopologyFaceRef {
            body_id: Some(body_id.to_string()),
            component_id: None,
            topology_version: Some(0),
            face_id: Some(format!("import:{body_id}:face:{k}")),
            surface_kind: None,
            producer_feature_id: Some(body_id.to_string()),
            source_entity_id: None,
        });
    }
}

/// Stamp durable names onto a revolved body's faces:
/// `revolve:{node}:region:{i}:face:{k}` with `k` in quantized-centroid order
/// (stable across runs — see [`stamp_import_face_refs`]).
pub(crate) fn stamp_revolve_face_refs(mesh: &mut MockMesh, body_id: &str, region_index: usize) {
    let quant = |v: f32| (v as f64 * 1.0e3).round() as i64;
    let unstamped: Vec<usize> = (0..mesh.face_refs.len())
        .filter(|&i| mesh.face_refs[i].topology.is_none())
        .collect();
    let mut order = unstamped;
    order.sort_by_key(|&i| {
        let c = mesh.face_refs[i].centroid;
        (quant(c[0]), quant(c[1]), quant(c[2]))
    });
    for (k, &i) in order.iter().enumerate() {
        mesh.face_refs[i].topology = Some(crate::mock_kernel::MeshTopologyFaceRef {
            body_id: Some(body_id.to_string()),
            component_id: None,
            topology_version: Some(0),
            face_id: Some(format!("revolve:{body_id}:region:{region_index}:face:{k}")),
            surface_kind: None,
            producer_feature_id: Some(body_id.to_string()),
            source_entity_id: None,
        });
    }
}

/// Stamp durable names onto a pattern instance's faces:
/// `pattern:{node}:inst:{k}:face:{j}` in quantized-centroid order per instance.
pub(crate) fn stamp_pattern_face_refs(mesh: &mut MockMesh, body_id: &str, instance: usize) {
    let quant = |v: f32| (v as f64 * 1.0e3).round() as i64;
    let mut order: Vec<usize> = (0..mesh.face_refs.len())
        .filter(|&i| mesh.face_refs[i].topology.is_none())
        .collect();
    order.sort_by_key(|&i| {
        let c = mesh.face_refs[i].centroid;
        (quant(c[0]), quant(c[1]), quant(c[2]))
    });
    for (j, &i) in order.iter().enumerate() {
        mesh.face_refs[i].topology = Some(crate::mock_kernel::MeshTopologyFaceRef {
            body_id: Some(body_id.to_string()),
            component_id: None,
            topology_version: Some(0),
            face_id: Some(format!("pattern:{body_id}:inst:{instance}:face:{j}")),
            surface_kind: None,
            producer_feature_id: Some(body_id.to_string()),
            source_entity_id: None,
        });
    }
}

/// Give previously unnamed faces created by a joined pattern a durable owner
/// while keeping the body identity of the modified source. Source faces that
/// survived the join are named before this runs and remain untouched.
pub(crate) fn stamp_joined_pattern_face_refs(mesh: &mut MockMesh, body_id: &str, pattern_id: &str) {
    let quant = |value: f32| (f64::from(value) * 1.0e3).round() as i64;
    let mut order: Vec<usize> = (0..mesh.face_refs.len())
        .filter(|&index| mesh.face_refs[index].topology.is_none())
        .collect();
    order.sort_by_key(|&index| {
        let centroid = mesh.face_refs[index].centroid;
        (quant(centroid[0]), quant(centroid[1]), quant(centroid[2]))
    });
    for (face_index, index) in order.into_iter().enumerate() {
        let face_id = format!("pattern:{pattern_id}:joined:face:{face_index}");
        mesh.face_refs[index].topology = Some(crate::mock_kernel::MeshTopologyFaceRef {
            body_id: Some(body_id.to_string()),
            component_id: None,
            topology_version: Some(0),
            face_id: Some(face_id),
            surface_kind: None,
            producer_feature_id: Some(pattern_id.to_string()),
            source_entity_id: None,
        });
    }
    crate::mock_kernel::populate_edge_adjacent_face_names(mesh);
}

/// Stamp durable PRIMITIVE names onto a cylinder body's faces:
/// `cyl_{node}:face:{lateral|top|bottom}`. The cylinder primitive stands along
/// +Y with its base at the origin; the wall's area-weighted average normal is
/// near zero (it wraps all the way around), so anything that isn't clearly a
/// ±Y cap is the lateral. The mesh's cylinder-arc canonicalization has already
/// collapsed the wall to one face id, so the lateral is a single face ref.
pub(crate) fn stamp_cylinder_face_refs(mesh: &mut MockMesh, body_id: &str) {
    for face_ref in mesh.face_refs.iter_mut() {
        let n = Vec3::new(face_ref.normal[0], face_ref.normal[1], face_ref.normal[2]);
        let len = n.dot(n).sqrt();
        let ny = if len > 1.0e-6 { n.y / len } else { 0.0 };
        let (role, kind) = if ny > 0.7 {
            ("top", "plane")
        } else if ny < -0.7 {
            ("bottom", "plane")
        } else {
            ("lateral", "cylinder")
        };
        face_ref.topology = Some(crate::mock_kernel::MeshTopologyFaceRef {
            body_id: Some(body_id.to_string()),
            component_id: None,
            topology_version: Some(0),
            face_id: Some(format!("cyl_{body_id}:face:{role}")),
            surface_kind: Some(kind.to_string()),
            producer_feature_id: Some(body_id.to_string()),
            source_entity_id: None,
        });
    }
}

/// Classify an extruded face as `top` (far cap), `bottom` (base), or `side`
/// (wall) from its outward normal relative to the sketch-plane normal `cs.n`.
pub(crate) fn sketch_extrude_face_role(
    normal: [f32; 3],
    cs: &CoordinateSystem,
    depth: f32,
) -> &'static str {
    let n = Vec3::new(normal[0], normal[1], normal[2]);
    let nlen = n.dot(n).sqrt();
    if nlen <= 1.0e-6 {
        return "side";
    }
    let axis = cs.n.normalize();
    // Positive `depth` sweeps along +axis, so a face whose outward normal points
    // the swept way is the far cap; the opposite is the base. `depth.signum()`
    // keeps the roles stable when the sketch is extruded in the −axis direction.
    let d = (n.dot(axis) / nlen) * depth.signum();
    if d > 0.5 {
        "top"
    } else if d < -0.5 {
        "bottom"
    } else {
        "side"
    }
}

pub(crate) fn sketch_extrude_edge_role(
    p0: [f32; 3],
    p1: [f32; 3],
    cs: &CoordinateSystem,
    depth: f32,
) -> &'static str {
    let offset = |p: [f32; 3]| {
        Vec3::new(p[0], p[1], p[2])
            .sub(cs.origin)
            .dot(cs.n.normalize())
    };
    let a = offset(p0);
    let b = offset(p1);
    let tol = (depth.abs() * 1.0e-3).max(0.02);
    if a.abs() <= tol && b.abs() <= tol {
        "bottom"
    } else if (a - depth).abs() <= tol && (b - depth).abs() <= tol {
        "top"
    } else {
        "side"
    }
}

pub(crate) fn sketch_extrude_edge_fragment_id(
    edge_ref: &crate::mock_kernel::MeshEdgeRef,
    provenance: &RegionProvenance,
    cs: &CoordinateSystem,
) -> Option<String> {
    match edge_ref.curve {
        Some(EdgeCurveHint::Circle { center, radius, .. }) => {
            let center_2d = cs.project(Vec3::new(center[0], center[1], center[2]));
            provenance
                .fragments
                .iter()
                .enumerate()
                .find_map(|(i, fragment)| match fragment {
                    RegionProvenanceFragment::CircleArc {
                        shape_id,
                        center,
                        radius: source_radius,
                    } if (center.0 - center_2d.0).hypot(center.1 - center_2d.1) <= 0.05
                        && (*source_radius - radius).abs() <= 0.05 =>
                    {
                        Some(provenance_fragment_stable_id(i, fragment, *shape_id))
                    }
                    _ => None,
                })
        }
        _ => sketch_extrude_linear_fragment_id(edge_ref, provenance, cs),
    }
}

pub(crate) fn sketch_extrude_linear_fragment_id(
    edge_ref: &crate::mock_kernel::MeshEdgeRef,
    provenance: &RegionProvenance,
    cs: &CoordinateSystem,
) -> Option<String> {
    let mid = [
        (edge_ref.p0[0] + edge_ref.p1[0]) * 0.5,
        (edge_ref.p0[1] + edge_ref.p1[1]) * 0.5,
        (edge_ref.p0[2] + edge_ref.p1[2]) * 0.5,
    ];
    let mid_2d = cs.project(Vec3::new(mid[0], mid[1], mid[2]));

    let mut best_rect: Option<(f32, String)> = None;
    let mut best_circle: Option<(f32, String)> = None;
    let mut raw: Option<String> = None;
    for (i, fragment) in provenance.fragments.iter().enumerate() {
        match fragment {
            RegionProvenanceFragment::RectangleEdge {
                shape_id,
                edge_index,
                rect_min,
                rect_max,
            } => {
                let dist = distance_to_rect_edge(mid_2d, *edge_index, *rect_min, *rect_max);
                if best_rect.as_ref().map_or(true, |(best, _)| dist < *best) {
                    best_rect = Some((dist, provenance_fragment_stable_id(i, fragment, *shape_id)));
                }
            }
            RegionProvenanceFragment::CircleArc {
                shape_id,
                center,
                radius,
            } => {
                let dist = ((mid_2d.0 - center.0).hypot(mid_2d.1 - center.1) - radius).abs();
                if best_circle.as_ref().map_or(true, |(best, _)| dist < *best) {
                    best_circle =
                        Some((dist, provenance_fragment_stable_id(i, fragment, *shape_id)));
                }
            }
            RegionProvenanceFragment::RawPolyline { shape_id }
            | RegionProvenanceFragment::SketchFilletArc { shape_id }
            | RegionProvenanceFragment::SketchChamferEdge { shape_id }
            | RegionProvenanceFragment::Slot { shape_id, .. }
            | RegionProvenanceFragment::RoundedRectangle { shape_id } => {
                raw.get_or_insert_with(|| provenance_fragment_stable_id(i, fragment, *shape_id));
            }
        }
    }

    if let Some((dist, id)) = best_circle {
        if dist <= 0.08 {
            return Some(id);
        }
    }
    if let Some((dist, id)) = best_rect {
        if dist <= 0.08 {
            return Some(id);
        }
    }
    raw
}

pub(crate) fn distance_to_rect_edge(
    p: (f32, f32),
    edge_index: usize,
    rect_min: (f32, f32),
    rect_max: (f32, f32),
) -> f32 {
    match edge_index {
        0 => {
            let x = p.0.clamp(rect_min.0, rect_max.0);
            (p.0 - x).hypot(p.1 - rect_min.1)
        }
        1 => {
            let y = p.1.clamp(rect_min.1, rect_max.1);
            (p.0 - rect_max.0).hypot(p.1 - y)
        }
        2 => {
            let x = p.0.clamp(rect_min.0, rect_max.0);
            (p.0 - x).hypot(p.1 - rect_max.1)
        }
        _ => {
            let y = p.1.clamp(rect_min.1, rect_max.1);
            (p.0 - rect_min.0).hypot(p.1 - y)
        }
    }
}

pub(crate) fn provenance_fragment_stable_id(
    fallback_index: usize,
    fragment: &RegionProvenanceFragment,
    shape_id: Option<usize>,
) -> String {
    let owner = shape_id
        .map(|id| format!("shape:{id}"))
        .unwrap_or_else(|| format!("fragment:{fallback_index}"));
    match fragment {
        RegionProvenanceFragment::RectangleEdge { edge_index, .. } => {
            format!("{owner}:rectangle-edge:{edge_index}")
        }
        RegionProvenanceFragment::CircleArc { .. } => format!("{owner}:circle"),
        RegionProvenanceFragment::SketchFilletArc { .. } => format!("{owner}:sketch-fillet"),
        RegionProvenanceFragment::SketchChamferEdge { .. } => format!("{owner}:sketch-chamfer"),
        RegionProvenanceFragment::Slot { boundary, .. } => {
            let boundary = match boundary {
                crate::sketch::SlotBoundary::LeftSide => "side-left",
                crate::sketch::SlotBoundary::EndCap => "end",
                crate::sketch::SlotBoundary::RightSide => "side-right",
                crate::sketch::SlotBoundary::StartCap => "start",
            };
            format!("{owner}:slot:{boundary}")
        }
        RegionProvenanceFragment::RoundedRectangle { .. } => {
            format!("{owner}:rounded-rectangle")
        }
        RegionProvenanceFragment::RawPolyline { .. } => format!("{owner}:raw-polyline"),
    }
}

/// A body being assembled during evaluation. `parts` are the kernel solids that
/// make it up (more than one only when disjoint lumps share a node); `pristine`
/// is the current named display-mesh cache. Initially it holds the analytic
/// sketch mesh; operations that can preserve durable face ownership replace it
/// with a freshly named result mesh, while other topology changes clear it and
/// force tessellation from `parts`.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub(crate) struct LiveBody {
    pub(crate) id: crate::document::BodyId,
    pub(crate) parts: Vec<KernelSolid>,
    pub(crate) pristine: Option<std::sync::Arc<MockMesh>>,
    pub(crate) sketch_source: Option<SketchExtrudeSource>,
}

// Phase 3 evaluates booleans against the current body; no feature replay state
// is stored on a live body.

/// Fully resolved inputs for one native thread operation.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub(crate) struct ThreadParameters {
    pub(crate) face: FaceRef,
    pub(crate) internal: bool,
    pub(crate) pitch: f32,
    pub(crate) depth: f32,
    pub(crate) angle_deg: f32,
    pub(crate) right_handed: bool,
    pub(crate) starts: u32,
    pub(crate) length: Option<f32>,
    pub(crate) flip: bool,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub(crate) struct SketchExtrudeSource {
    pub(crate) regions: Vec<SketchExtrudeRegionSource>,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub(crate) struct SketchExtrudeRegionSource {
    pub(crate) boundary: Vec<(f32, f32)>,
    pub(crate) holes: Vec<Vec<(f32, f32)>>,
    pub(crate) depth: f32,
    pub(crate) cs: CoordinateSystem,
    pub(crate) rect_circle: Option<RectCircleCanonicalSource>,
    /// Exact runtime arrangement for guarded prismatic reconstruction. Cached
    /// body payloads may omit it; evaluation always rebuilds it from the
    /// authoritative sketch before a new operation can use the fallback.
    #[serde(skip)]
    pub(crate) analytic: Option<crate::sketch::AnalyticSketchRegion>,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub(crate) struct RectCircleCanonicalSource {
    pub(crate) base: KernelSolid,
    pub(crate) cutter: KernelSolid,
    pub(crate) body: Option<KernelSolid>,
}

pub(crate) fn rect_circle_region_base_and_cutter_from_sketch(
    curves: &SketchCurves,
    region: &Region,
    depth: f32,
    cs: &CoordinateSystem,
    radius_grow: f32,
) -> Option<(KernelSolid, KernelSolid)> {
    if region.holes.len() > 0 {
        return None;
    }
    let (rect_min, rect_max) = rectangle_bounds_from_source_curves(curves)?;
    let circle = curves.circles.first().copied()?;
    if curves.circles.len() != 1 {
        return None;
    }
    if !circle_intersects_rect_boundary(rect_min, rect_max, circle.center, circle.radius) {
        return None;
    }
    if !region_is_rect_minus_circle_material(
        region,
        rect_min,
        rect_max,
        circle.center,
        circle.radius,
    ) {
        return None;
    }
    crate::mock_kernel::rect_circle_base_and_cutter_from_primitives(
        rect_min,
        rect_max,
        circle.center,
        circle.radius,
        depth,
        cs,
        radius_grow,
    )
}

pub(crate) fn rect_circle_region_base_and_cutter_from_provenance(
    provenance: &RegionProvenance,
    region: &Region,
    depth: f32,
    cs: &CoordinateSystem,
    radius_grow: f32,
) -> Option<(KernelSolid, KernelSolid)> {
    if !region.holes.is_empty() {
        return None;
    }
    let mut rect: Option<((f32, f32), (f32, f32))> = None;
    let mut circle: Option<((f32, f32), f32)> = None;
    for fragment in &provenance.fragments {
        match fragment {
            RegionProvenanceFragment::RectangleEdge {
                rect_min, rect_max, ..
            } => {
                let candidate = (*rect_min, *rect_max);
                if rect.is_none() {
                    rect = Some(candidate);
                } else if rect != Some(candidate) {
                    return None;
                }
            }
            RegionProvenanceFragment::CircleArc { center, radius, .. } => {
                let candidate = (*center, *radius);
                if circle.is_none() {
                    circle = Some(candidate);
                } else if circle != Some(candidate) {
                    return None;
                }
            }
            RegionProvenanceFragment::RawPolyline { .. } => return None,
            _ => {}
        }
    }
    let (rect_min, rect_max) = rect?;
    let (circle_center, circle_radius) = circle?;
    if !circle_intersects_rect_boundary(rect_min, rect_max, circle_center, circle_radius) {
        return None;
    }
    if !region_is_rect_minus_circle_material(
        region,
        rect_min,
        rect_max,
        circle_center,
        circle_radius,
    ) {
        return None;
    }
    crate::mock_kernel::rect_circle_base_and_cutter_from_primitives(
        rect_min,
        rect_max,
        circle_center,
        circle_radius,
        depth,
        cs,
        radius_grow,
    )
}

pub(crate) fn sketch_source_after_circle_cut(
    source: &SketchExtrudeSource,
    circle: Circle,
) -> Option<SketchExtrudeSource> {
    let mut next = source.clone();
    let mut any = false;
    for region in &mut next.regions {
        if !region.holes.is_empty() || region.rect_circle.is_some() {
            continue;
        }
        let (rect_min, rect_max) = loop_bounds_2d(&region.boundary)?;
        if !circle_intersects_rect_boundary(rect_min, rect_max, circle.center, circle.radius) {
            continue;
        }
        let mut provenance_curves = SketchCurves::new();
        provenance_curves.add_rectangle(rect_min, rect_max);
        provenance_curves.add_circle(circle.center, circle.radius);
        if let Some(material_region) = detect_regions(&provenance_curves).into_iter().find(|r| {
            region_is_rect_minus_circle_material(
                r,
                rect_min,
                rect_max,
                circle.center,
                circle.radius,
            )
        }) {
            region.boundary = material_region.boundary;
            region.holes = material_region.holes;
        }
        let Some((base, cutter)) = crate::mock_kernel::rect_circle_base_and_cutter_from_primitives(
            rect_min,
            rect_max,
            circle.center,
            circle.radius,
            region.depth,
            &region.cs,
            0.0,
        ) else {
            continue;
        };
        region.rect_circle = Some(RectCircleCanonicalSource {
            body: crate::mock_kernel::difference(&base, &cutter),
            base,
            cutter,
        });
        any = true;
    }
    any.then_some(next)
}

pub(crate) fn rectangle_bounds_from_source_curves(
    curves: &SketchCurves,
) -> Option<((f32, f32), (f32, f32))> {
    if curves.segments.len() != 4 {
        return None;
    }
    let mut pts: Vec<(f32, f32)> = Vec::new();
    for seg in &curves.segments {
        for p in [seg.a, seg.b] {
            if !pts.iter().any(|q| (q.0 - p.0).hypot(q.1 - p.1) <= 1.0e-4) {
                pts.push(p);
            }
        }
    }
    if pts.len() != 4 {
        return None;
    }

    let (mut min_x, mut min_y) = (f32::INFINITY, f32::INFINITY);
    let (mut max_x, mut max_y) = (f32::NEG_INFINITY, f32::NEG_INFINITY);
    for (x, y) in pts {
        min_x = min_x.min(x);
        min_y = min_y.min(y);
        max_x = max_x.max(x);
        max_y = max_y.max(y);
    }
    if max_x - min_x <= 1.0e-3 || max_y - min_y <= 1.0e-3 {
        return None;
    }

    let has_corner = |p: (f32, f32)| {
        curves
            .segments
            .iter()
            .flat_map(|s| [s.a, s.b])
            .any(|q| (q.0 - p.0).abs() <= 1.0e-4 && (q.1 - p.1).abs() <= 1.0e-4)
    };
    for corner in [
        (min_x, min_y),
        (max_x, min_y),
        (max_x, max_y),
        (min_x, max_y),
    ] {
        if !has_corner(corner) {
            return None;
        }
    }

    let mut sides = [false; 4];
    for seg in &curves.segments {
        let horizontal = (seg.a.1 - seg.b.1).abs() <= 1.0e-4;
        let vertical = (seg.a.0 - seg.b.0).abs() <= 1.0e-4;
        if horizontal {
            if (seg.a.1 - min_y).abs() <= 1.0e-4 {
                sides[0] = true;
            } else if (seg.a.1 - max_y).abs() <= 1.0e-4 {
                sides[2] = true;
            } else {
                return None;
            }
        } else if vertical {
            if (seg.a.0 - max_x).abs() <= 1.0e-4 {
                sides[1] = true;
            } else if (seg.a.0 - min_x).abs() <= 1.0e-4 {
                sides[3] = true;
            } else {
                return None;
            }
        } else {
            return None;
        }
    }
    sides
        .iter()
        .all(|s| *s)
        .then_some(((min_x, min_y), (max_x, max_y)))
}

pub(crate) fn circle_intersects_rect_boundary(
    rect_min: (f32, f32),
    rect_max: (f32, f32),
    center: (f32, f32),
    radius: f32,
) -> bool {
    let ((min_x, min_y), (max_x, max_y)) = ordered_rect(rect_min, rect_max);
    let mut hits = 0usize;
    let eps = 1.0e-4;
    for y in [min_y, max_y] {
        let dy = y - center.1;
        if dy.abs() <= radius + eps {
            let dx2 = radius * radius - dy * dy;
            if dx2 >= -eps {
                let dx = dx2.max(0.0).sqrt();
                for x in [center.0 - dx, center.0 + dx] {
                    if x >= min_x - eps && x <= max_x + eps {
                        hits += 1;
                    }
                }
            }
        }
    }
    for x in [min_x, max_x] {
        let dx = x - center.0;
        if dx.abs() <= radius + eps {
            let dy2 = radius * radius - dx * dx;
            if dy2 >= -eps {
                let dy = dy2.max(0.0).sqrt();
                for y in [center.1 - dy, center.1 + dy] {
                    if y >= min_y - eps && y <= max_y + eps {
                        hits += 1;
                    }
                }
            }
        }
    }
    hits >= 2
}

pub(crate) fn region_is_rect_minus_circle_material(
    region: &Region,
    rect_min: (f32, f32),
    rect_max: (f32, f32),
    center: (f32, f32),
    radius: f32,
) -> bool {
    if region.contains(center) {
        return false;
    }
    let ((min_x, min_y), (max_x, max_y)) = ordered_rect(rect_min, rect_max);
    let rect_area = (max_x - min_x) * (max_y - min_y);
    if region.area <= 1.0e-3 || region.area >= rect_area - 1.0e-3 {
        return false;
    }

    let mut has_material_sample = false;
    let mut has_removed_circle_sample = false;
    for ix in 1..5 {
        for iy in 1..5 {
            let x = min_x + (max_x - min_x) * (ix as f32 / 5.0);
            let y = min_y + (max_y - min_y) * (iy as f32 / 5.0);
            let inside_circle = (x - center.0).hypot(y - center.1) < radius - 0.05;
            let inside_region = region.contains((x, y));
            if inside_region && !inside_circle {
                has_material_sample = true;
            }
            if inside_region && inside_circle {
                has_removed_circle_sample = true;
            }
        }
    }
    has_material_sample && !has_removed_circle_sample
}

pub(crate) fn ordered_rect(a: (f32, f32), b: (f32, f32)) -> ((f32, f32), (f32, f32)) {
    ((a.0.min(b.0), a.1.min(b.1)), (a.0.max(b.0), a.1.max(b.1)))
}

// ---------------------------------------------------------------------------
// Boolean extrude of overlapping sketch shapes
// ---------------------------------------------------------------------------

/// Absolute area of a closed 2D loop (shoelace).
fn loop_area(poly: &[(f32, f32)]) -> f32 {
    let n = poly.len();
    if n < 3 {
        return 0.0;
    }
    let mut s = 0.0f32;
    for i in 0..n {
        let (x0, y0) = poly[i];
        let (x1, y1) = poly[(i + 1) % n];
        s += x0 * y1 - x1 * y0;
    }
    (s * 0.5).abs()
}

/// A point inside a region's actual material — inside the outer boundary and not
/// in any hole. `polygon_interior_point` only sees the outer boundary, so for a
/// holed region (an annulus) its ear-centroid can land in the hole; this falls
/// back to boundary-edge midpoints nudged inward until one is truly inside.
pub(crate) fn region_material_point(region: &Region) -> (f32, f32) {
    let p = crate::sketch::polygon_interior_point(&region.boundary);
    if region.holes.is_empty() || region.contains(p) {
        return p;
    }
    let b = &region.boundary;
    let n = b.len();
    let (mut cx, mut cy) = (0.0f32, 0.0f32);
    for &(x, y) in b {
        cx += x;
        cy += y;
    }
    cx /= n as f32;
    cy /= n as f32;
    for i in 0..n {
        let a = b[i];
        let c = b[(i + 1) % n];
        let mid = ((a.0 + c.0) * 0.5, (a.1 + c.1) * 0.5);
        let q = (mid.0 + (cx - mid.0) * 0.02, mid.1 + (cy - mid.1) * 0.02);
        if region.contains(q) {
            return q;
        }
    }
    p
}

/// Drawn circles whose every atomic arrangement region is selected for the
/// current extrusion. Open/projected lines can split one circle into multiple
/// regions; callers use this to rebuild the original analytic cylinder instead
/// of previewing/evaluating touching semicylinders with construction seams.
pub fn complete_selected_circles(
    circles: &[crate::sketch::Circle],
    regions: &[Region],
    process: &[bool],
) -> Vec<(crate::sketch::Circle, Vec<usize>)> {
    circles
        .iter()
        .filter_map(|circle| {
            let r2 = circle.radius * circle.radius;
            let inside: Vec<usize> = regions
                .iter()
                .enumerate()
                .filter_map(|(i, region)| {
                    let p = region_material_point(region);
                    let dx = p.0 - circle.center.0;
                    let dy = p.1 - circle.center.1;
                    (dx * dx + dy * dy < r2 * 1.0001).then_some(i)
                })
                .collect();
            (inside.len() >= 2
                && inside
                    .iter()
                    .all(|&i| process.get(i).copied().unwrap_or(false)))
            .then_some((*circle, inside))
        })
        .collect()
}

/// Every shape loop whose boundary contains `interior` (all shapes the region
/// belongs to; >1 for an overlap region / lens).
pub(crate) fn region_containing_shapes(interior: (f32, f32), loops: &[ShapeLoop]) -> Vec<usize> {
    loops
        .iter()
        .enumerate()
        .filter(|(_, l)| crate::sketch::point_in_polygon(interior, &l.boundary))
        .map(|(s, _)| s)
        .collect()
}

/// Which shapes the user kept (the base material). Two passes so a shared overlap
/// region doesn't accidentally flip the base onto the wrong (smaller) shape:
///
/// 1. A selected region lying in exactly one shape marks that shape as base.
/// 2. A selected region shared by several shapes marks the **smallest** of them
///    only when none of its shapes is already a base — so selecting the
///    rectangle (whose body region is exclusive) keeps the rectangle the base
///    even if the small overlap lens is also picked, while selecting the inner
///    disk of a circle-in-rectangle still makes the circle the base.
///
/// An empty `region_indices` (whole-sketch extrude) selects every region.
pub(crate) fn selected_shape_mask(
    regions: &[Region],
    region_indices: &[usize],
    loops: &[ShapeLoop],
) -> Vec<bool> {
    let mut mask = vec![false; loops.len()];
    let take_all = region_indices.is_empty();
    let mut shared: Vec<Vec<usize>> = Vec::new();
    for (i, r) in regions.iter().enumerate() {
        if !take_all && !region_indices.contains(&i) {
            continue;
        }
        let interior = region_material_point(r);
        let containing = region_containing_shapes(interior, loops);
        match containing.len() {
            0 => {}
            1 => mask[containing[0]] = true,
            _ => shared.push(containing),
        }
    }
    for containing in shared {
        if containing.iter().any(|&s| mask[s]) {
            continue;
        }
        if let Some(&s) = containing.iter().min_by(|&&a, &&b| {
            loop_area(&loops[a].boundary)
                .partial_cmp(&loop_area(&loops[b].boundary))
                .unwrap_or(std::cmp::Ordering::Equal)
        }) {
            mask[s] = true;
        }
    }
    mask
}

/// Which regions of a sketch to build as material, resolving overlapping-shapes-
/// as-boolean the same way the extrude evaluator does. Shared so the live
/// extrude ghost keeps exactly the regions a commit would, instead of drawing
/// the tool-lens regions a boolean drops (e.g. the inner disc of a circle drawn
/// inside a rectangle, which must read as a hole).
///
/// Returns one entry per region: `process` (build it) and `is_boolean` (it
/// belongs to a multi-shape overlap cluster, so the shape selection — not the
/// raw `region_indices` — decides whether it is kept). Mirrors the per-region
/// classification in `apply_extrude`.
pub struct BooleanRegionPlan {
    pub process: Vec<bool>,
    pub is_boolean: Vec<bool>,
}

/// Resolve which regions to build (see [`BooleanRegionPlan`]). `loops` are the
/// sketch's shape outlines ([`crate::sketch::shape_loops`]); an empty `loops`
/// (legacy sketch / sketch corner-mods) leaves every region on the normal
/// `take_all || region_indices.contains(i)` selection path.
pub fn boolean_region_plan(
    loops: &[ShapeLoop],
    regions: &[Region],
    region_indices: &[usize],
) -> BooleanRegionPlan {
    let take_all = region_indices.is_empty();
    // A single viewport click selects one detected face, not its originating
    // sketch primitive. Overlap detection can split one rectangle/circle into
    // several faces; expanding that one pick back to the whole source shape
    // silently extruded unselected faces. Multi-face and whole-sketch requests
    // retain the shape-level boolean behavior below.
    if region_indices.len() == 1 {
        let selected = region_indices[0];
        return BooleanRegionPlan {
            process: (0..regions.len()).map(|i| i == selected).collect(),
            is_boolean: vec![false; regions.len()],
        };
    }
    let clusters = if loops.is_empty() {
        Vec::new()
    } else {
        crate::sketch::overlap_clusters(loops)
    };
    let mut shape_cluster = vec![usize::MAX; loops.len()];
    for (ci, c) in clusters.iter().enumerate() {
        for &s in c {
            shape_cluster[s] = ci;
        }
    }
    let cluster_is_multi: Vec<bool> = clusters.iter().map(|c| c.len() >= 2).collect();
    let selected_mask = selected_shape_mask(regions, region_indices, loops);
    let mut is_boolean = vec![false; regions.len()];
    let mut process = vec![false; regions.len()];
    for (i, r) in regions.iter().enumerate() {
        let interior = region_material_point(r);
        let containing = region_containing_shapes(interior, loops);
        let in_multi = containing
            .iter()
            .any(|&s| cluster_is_multi[shape_cluster[s]]);
        if in_multi {
            let in_base = containing.iter().any(|&s| selected_mask[s]);
            let in_tool = containing.iter().any(|&s| !selected_mask[s]);
            is_boolean[i] = true;
            // Explicit region selection wins over the base∩tool "drop the lens"
            // rule. Picking a region that lies inside another shape — the inner
            // rectangle of a nested pair, or an overlap lens — means the user
            // wants THAT material, so build it. The tool-lens drop
            // (`in_base && !in_tool`) still governs regions the user did NOT
            // single out, which is what keeps "extrude the whole sketch" of a
            // circle-in-rectangle yielding a rect-with-hole rather than a filled
            // slab. `take_all` (whole-sketch) has no explicit picks, so it is
            // unaffected.
            let explicitly_selected = !take_all && region_indices.contains(&i);
            process[i] = explicitly_selected || (in_base && !in_tool);
        } else {
            process[i] = take_all || region_indices.contains(&i);
        }
    }
    BooleanRegionPlan {
        process,
        is_boolean,
    }
}

/// One connected 2D material region prepared for a single extrusion. Adjacent
/// arrangement faces are merged before entering the 3D kernel, so their shared
/// sketch edges never become coplanar B-Rep seams that a later boolean must
/// repair.
#[derive(Debug, Clone)]
pub(crate) struct PreparedExtrudeRegion {
    pub(crate) region: Region,
    pub(crate) source_indices: Vec<usize>,
}

type RegionVertexKey = usize;

#[derive(Clone, Copy)]
struct RegionBoundaryEdge {
    from: RegionVertexKey,
    to: RegionVertexKey,
}

fn signed_loop_area(points: &[(f32, f32)]) -> f32 {
    points
        .iter()
        .enumerate()
        .map(|(index, point)| {
            let next = points[(index + 1) % points.len()];
            point.0 * next.1 - point.1 * next.0
        })
        .sum::<f32>()
        * 0.5
}

fn simplify_collinear_loop(mut points: Vec<(f32, f32)>, tolerance: f32) -> Vec<(f32, f32)> {
    loop {
        if points.len() <= 3 {
            return points;
        }
        let remove = (0..points.len()).find(|&index| {
            let previous = points[(index + points.len() - 1) % points.len()];
            let current = points[index];
            let next = points[(index + 1) % points.len()];
            let incoming = (current.0 - previous.0, current.1 - previous.1);
            let outgoing = (next.0 - current.0, next.1 - current.1);
            let incoming_len = incoming.0.hypot(incoming.1);
            let outgoing_len = outgoing.0.hypot(outgoing.1);
            let cross = incoming.0 * outgoing.1 - incoming.1 * outgoing.0;
            let dot = incoming.0 * outgoing.0 + incoming.1 * outgoing.1;
            dot > 0.0 && cross.abs() <= tolerance * (incoming_len + outgoing_len)
        });
        let Some(index) = remove else {
            return points;
        };
        points.remove(index);
    }
}

fn selected_region_tolerance(regions: &[Region], selected: &[usize]) -> f32 {
    let scale = selected
        .iter()
        .flat_map(|index| {
            let region = &regions[*index];
            region.boundary.iter().chain(region.holes.iter().flatten())
        })
        .flat_map(|point| [f64::from(point.0.abs()), f64::from(point.1.abs())])
        .fold(0.0_f64, f64::max);
    (scale * f64::from(f32::EPSILON) * 8.0).max(f64::EPSILON * 64.0) as f32
}

fn region_loops(region: &Region) -> impl Iterator<Item = &[(f32, f32)]> {
    std::iter::once(region.boundary.as_slice()).chain(region.holes.iter().map(Vec::as_slice))
}

#[derive(Clone)]
struct AnalyticBoundarySpan {
    from: usize,
    to: usize,
    span: openrcad::geom2d::CurveSpan<crate::sketch::SketchCurveProvenance>,
}

fn analytic_spans_match_reversed(
    first: &openrcad::geom2d::CurveSpan<crate::sketch::SketchCurveProvenance>,
    second: &openrcad::geom2d::CurveSpan<crate::sketch::SketchCurveProvenance>,
    tolerance: f64,
) -> bool {
    use openrcad::geom2d::Curve2d;

    first.kind() == second.kind()
        && [0.25, 0.5, 0.75].into_iter().all(|fraction| {
            let first_parameter = first.first + (first.last - first.first) * fraction;
            let second_parameter = second.last + (second.first - second.last) * fraction;
            first
                .curve
                .point(first_parameter)
                .distance(&second.curve.point(second_parameter))
                <= tolerance
        })
}

fn analytic_loop_signed_area(
    spans: &[openrcad::geom2d::CurveSpan<crate::sketch::SketchCurveProvenance>],
) -> f64 {
    use openrcad::geom2d::{Curve2d, CurveKind2d};

    let mut points = Vec::new();
    for span in spans {
        let steps = match span.kind() {
            CurveKind2d::Line => 1,
            CurveKind2d::Circle | CurveKind2d::Ellipse => {
                ((span.parameter_length() / std::f64::consts::TAU * 128.0).ceil() as usize).max(2)
            }
            _ => 32,
        };
        for step in 0..steps {
            let parameter = span.first + (span.last - span.first) * step as f64 / steps as f64;
            let point = span.curve.point(parameter);
            points.push((point.x(), point.y()));
        }
    }
    points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .take(points.len())
        .map(|(&(ax, ay), &(bx, by))| ax * by - bx * ay)
        .sum::<f64>()
        * 0.5
}

fn reverse_analytic_loop(
    loop_: &mut openrcad::sketch::ArrangementLoop<crate::sketch::SketchCurveProvenance>,
) {
    loop_.spans = loop_
        .spans
        .drain(..)
        .rev()
        .map(openrcad::geom2d::CurveSpan::reversed)
        .collect();
    loop_.signed_area = -loop_.signed_area;
}

/// Exact counterpart to the sampled boundary cancellation below. Every
/// detected region retains the arrangement's analytic line/arc spans; carry
/// those through the union so a circular notch and the circle that later fills
/// it use the identical cylindrical support. Re-fitting the merged point loop
/// shifts a 0.4 mm arc by about 1e-3 mm, which is enough to create micro-edges
/// when a partial-height Join splits that wall.
fn merge_analytic_regions(
    regions: &[Region],
    source_indices: &[usize],
    tolerance: f32,
) -> Option<crate::sketch::AnalyticSketchRegion> {
    let analytic_regions: Vec<_> = source_indices
        .iter()
        .map(|index| regions[*index].analytic.as_ref())
        .collect::<Option<_>>()?;
    let tolerance = f64::from(tolerance);
    let mut vertices = Vec::new();
    let mut vertex_key = |point: openrcad::foundation::Pnt2d| {
        if let Some(index) = vertices
            .iter()
            .position(|existing: &openrcad::foundation::Pnt2d| {
                existing.distance(&point) <= tolerance
            })
        {
            index
        } else {
            vertices.push(point);
            vertices.len() - 1
        }
    };
    let mut unmatched: std::collections::HashMap<(usize, usize), Vec<AnalyticBoundarySpan>> =
        std::collections::HashMap::new();
    for region in analytic_regions {
        for loop_ in std::iter::once(&region.outer).chain(region.holes.iter()) {
            for span in &loop_.spans {
                let (from, to) = (vertex_key(span.start()), vertex_key(span.end()));
                let reverse = (to, from);
                let match_index = unmatched.get(&reverse).and_then(|candidates| {
                    candidates.iter().position(|candidate| {
                        analytic_spans_match_reversed(&candidate.span, span, tolerance)
                    })
                });
                if let Some(match_index) = match_index {
                    let candidates = unmatched
                        .get_mut(&reverse)
                        .expect("matched analytic edge list");
                    candidates.swap_remove(match_index);
                    if candidates.is_empty() {
                        unmatched.remove(&reverse);
                    }
                } else {
                    unmatched
                        .entry((from, to))
                        .or_default()
                        .push(AnalyticBoundarySpan {
                            from,
                            to,
                            span: span.clone(),
                        });
                }
            }
        }
    }

    let edges: Vec<AnalyticBoundarySpan> = unmatched.into_values().flatten().collect();
    let mut outgoing: std::collections::HashMap<usize, Vec<usize>> =
        std::collections::HashMap::new();
    let mut incoming: std::collections::HashMap<usize, usize> = std::collections::HashMap::new();
    for (index, edge) in edges.iter().enumerate() {
        outgoing.entry(edge.from).or_default().push(index);
        *incoming.entry(edge.to).or_default() += 1;
    }
    if outgoing.values().any(|edges| edges.len() != 1) || incoming.values().any(|count| *count != 1)
    {
        return None;
    }

    let mut used = vec![false; edges.len()];
    let mut loops = Vec::new();
    for start in 0..edges.len() {
        if used[start] {
            continue;
        }
        let first = edges[start].from;
        let mut current = start;
        let mut spans = Vec::new();
        loop {
            if used[current] {
                return None;
            }
            used[current] = true;
            let edge = &edges[current];
            spans.push(edge.span.clone());
            if edge.to == first {
                break;
            }
            current = *outgoing.get(&edge.to)?.first()?;
        }
        let signed_area = analytic_loop_signed_area(&spans);
        if spans.len() < 2 || signed_area.abs() <= tolerance * tolerance {
            return None;
        }
        loops.push(openrcad::sketch::ArrangementLoop { spans, signed_area });
    }
    if used.iter().any(|used| !used) || loops.is_empty() {
        return None;
    }

    loops.sort_by(|left, right| right.area().total_cmp(&left.area()));
    let mut outer = loops.remove(0);
    if outer.signed_area < 0.0 {
        reverse_analytic_loop(&mut outer);
    }
    for hole in &mut loops {
        if hole.signed_area > 0.0 {
            reverse_analytic_loop(hole);
        }
    }
    // A straight-only union is represented more cleanly by the simplified
    // polygon assembled below. Keeping every arrangement subspan here would
    // reintroduce collinear face divisions (for example, two overlapping
    // rectangles would tessellate as 28 triangles instead of the canonical
    // box-like 12). Curves still need the analytic loop so a later shared-wall
    // Join sees the exact same arc/cylinder rather than a sampled refit.
    if std::iter::once(&outer)
        .chain(loops.iter())
        .flat_map(|loop_| loop_.spans.iter())
        .all(|span| span.kind() == openrcad::geom2d::CurveKind2d::Line)
    {
        return None;
    }
    let area = outer.area() - loops.iter().map(|hole| hole.area()).sum::<f64>();
    (area > tolerance * tolerance).then_some(openrcad::sketch::ArrangementRegion {
        outer,
        holes: loops,
        area,
    })
}

/// Merge selected arrangement tiles that share a complete boundary edge.
/// Point/line tangency without a shared 2D edge is deliberately not considered
/// connectivity: extruding that contact would create a non-manifold solid.
pub(crate) fn prepare_extrude_regions(
    regions: &[Region],
    process: &[bool],
) -> Vec<PreparedExtrudeRegion> {
    let selected: Vec<usize> = process
        .iter()
        .enumerate()
        .filter_map(|(index, selected)| (*selected).then_some(index))
        .collect();
    if selected.len() <= 1 {
        return selected
            .into_iter()
            .map(|index| PreparedExtrudeRegion {
                region: regions[index].clone(),
                source_indices: vec![index],
            })
            .collect();
    }

    let tolerance = selected_region_tolerance(regions, &selected);
    // Assign tolerance-close endpoints to one canonical vertex. Rounding to a
    // fixed grid is insufficient: two points can be much closer than the
    // tolerance yet fall on opposite sides of a cell boundary (the exact
    // BugCase1 coordinates do). Searching the current cell and its neighbours
    // makes the equivalence depend on physical distance instead.
    let cell_for = |point: (f32, f32)| {
        (
            (point.0 / tolerance).floor() as i64,
            (point.1 / tolerance).floor() as i64,
        )
    };
    let mut canonical_positions: Vec<(f32, f32)> = Vec::new();
    let mut canonical_cells: std::collections::HashMap<(i64, i64), Vec<RegionVertexKey>> =
        std::collections::HashMap::new();
    let mut point_keys: std::collections::HashMap<(u32, u32), RegionVertexKey> =
        std::collections::HashMap::new();
    for &region_index in &selected {
        for loop_ in region_loops(&regions[region_index]) {
            for &point in loop_ {
                let bits = (point.0.to_bits(), point.1.to_bits());
                if point_keys.contains_key(&bits) {
                    continue;
                }
                let cell = cell_for(point);
                let mut key = None;
                'neighbours: for dx in -1..=1 {
                    for dy in -1..=1 {
                        let neighbour = (cell.0 + dx, cell.1 + dy);
                        for &candidate in canonical_cells.get(&neighbour).into_iter().flatten() {
                            let existing = canonical_positions[candidate];
                            if (existing.0 - point.0).hypot(existing.1 - point.1) <= tolerance {
                                key = Some(candidate);
                                break 'neighbours;
                            }
                        }
                    }
                }
                let key = key.unwrap_or_else(|| {
                    let key = canonical_positions.len();
                    canonical_positions.push(point);
                    canonical_cells.entry(cell).or_default().push(key);
                    key
                });
                point_keys.insert(bits, key);
            }
        }
    }
    let vertex_key = |point: (f32, f32)| -> RegionVertexKey {
        point_keys[&(point.0.to_bits(), point.1.to_bits())]
    };
    let edge_key = |a: RegionVertexKey, b: RegionVertexKey| {
        if a <= b {
            (a, b)
        } else {
            (b, a)
        }
    };

    let mut parent: Vec<usize> = (0..selected.len()).collect();
    fn find(parent: &mut [usize], index: usize) -> usize {
        if parent[index] != index {
            parent[index] = find(parent, parent[index]);
        }
        parent[index]
    }
    fn union(parent: &mut [usize], a: usize, b: usize) {
        let (a, b) = (find(parent, a), find(parent, b));
        if a != b {
            parent[b] = a;
        }
    }

    let mut edge_owner: std::collections::HashMap<(RegionVertexKey, RegionVertexKey), usize> =
        std::collections::HashMap::new();
    for (local_index, &region_index) in selected.iter().enumerate() {
        for loop_ in region_loops(&regions[region_index]) {
            for edge in loop_
                .iter()
                .copied()
                .zip(loop_.iter().copied().cycle().skip(1))
                .take(loop_.len())
            {
                let key = edge_key(vertex_key(edge.0), vertex_key(edge.1));
                if let Some(&other) = edge_owner.get(&key) {
                    union(&mut parent, local_index, other);
                } else {
                    edge_owner.insert(key, local_index);
                }
            }
        }
    }

    let mut groups: std::collections::BTreeMap<usize, Vec<usize>> =
        std::collections::BTreeMap::new();
    for (local_index, region_index) in selected.into_iter().enumerate() {
        let root = find(&mut parent, local_index);
        groups.entry(root).or_default().push(region_index);
    }

    let mut prepared = Vec::new();
    for mut source_indices in groups.into_values() {
        source_indices.sort_unstable();
        if source_indices.len() == 1 {
            prepared.push(PreparedExtrudeRegion {
                region: regions[source_indices[0]].clone(),
                source_indices,
            });
            continue;
        }

        let mut positions: std::collections::HashMap<RegionVertexKey, ([f64; 2], usize)> =
            std::collections::HashMap::new();
        let mut unmatched: std::collections::HashMap<
            (RegionVertexKey, RegionVertexKey),
            Vec<RegionBoundaryEdge>,
        > = std::collections::HashMap::new();
        for &region_index in &source_indices {
            for loop_ in region_loops(&regions[region_index]) {
                for (a, b) in loop_
                    .iter()
                    .copied()
                    .zip(loop_.iter().copied().cycle().skip(1))
                    .take(loop_.len())
                {
                    let (from, to) = (vertex_key(a), vertex_key(b));
                    for (key, point) in [(from, a), (to, b)] {
                        let entry = positions.entry(key).or_insert(([0.0, 0.0], 0));
                        entry.0[0] += f64::from(point.0);
                        entry.0[1] += f64::from(point.1);
                        entry.1 += 1;
                    }
                    let reverse = (to, from);
                    let mut cancelled = false;
                    if let Some(edges) = unmatched.get_mut(&reverse) {
                        if edges.pop().is_some() {
                            cancelled = true;
                        }
                        if edges.is_empty() {
                            unmatched.remove(&reverse);
                        }
                    }
                    if !cancelled {
                        unmatched
                            .entry((from, to))
                            .or_default()
                            .push(RegionBoundaryEdge { from, to });
                    }
                }
            }
        }

        let edges: Vec<RegionBoundaryEdge> = unmatched.into_values().flatten().collect();
        let mut outgoing: std::collections::HashMap<RegionVertexKey, Vec<usize>> =
            std::collections::HashMap::new();
        let mut incoming: std::collections::HashMap<RegionVertexKey, usize> =
            std::collections::HashMap::new();
        for (index, edge) in edges.iter().enumerate() {
            outgoing.entry(edge.from).or_default().push(index);
            *incoming.entry(edge.to).or_default() += 1;
        }
        let manifold = outgoing.values().all(|edges| edges.len() == 1)
            && incoming.values().all(|count| *count == 1);
        if !manifold {
            prepared.extend(
                source_indices
                    .into_iter()
                    .map(|index| PreparedExtrudeRegion {
                        region: regions[index].clone(),
                        source_indices: vec![index],
                    }),
            );
            continue;
        }

        let point_for = |key: RegionVertexKey| {
            let (sum, count) = positions[&key];
            (
                (sum[0] / count as f64) as f32,
                (sum[1] / count as f64) as f32,
            )
        };
        let mut used = vec![false; edges.len()];
        let mut loops = Vec::new();
        for start in 0..edges.len() {
            if used[start] {
                continue;
            }
            let first = edges[start].from;
            let mut current = start;
            let mut loop_ = Vec::new();
            loop {
                if used[current] {
                    break;
                }
                used[current] = true;
                let edge = edges[current];
                loop_.push(point_for(edge.from));
                if edge.to == first {
                    break;
                }
                let Some(next) = outgoing.get(&edge.to).and_then(|next| next.first()) else {
                    loop_.clear();
                    break;
                };
                current = *next;
            }
            if loop_.len() >= 3 && signed_loop_area(&loop_).abs() > tolerance * tolerance {
                loops.push(simplify_collinear_loop(loop_, tolerance));
            }
        }
        if loops.is_empty() || used.iter().any(|used| !used) {
            prepared.extend(
                source_indices
                    .into_iter()
                    .map(|index| PreparedExtrudeRegion {
                        region: regions[index].clone(),
                        source_indices: vec![index],
                    }),
            );
            continue;
        }

        loops.sort_by(|left, right| {
            signed_loop_area(right)
                .abs()
                .total_cmp(&signed_loop_area(left).abs())
        });
        let mut outer = loops.remove(0);
        if signed_loop_area(&outer) < 0.0 {
            outer.reverse();
        }
        let mut holes = Vec::new();
        let mut valid = true;
        for mut loop_ in loops {
            let interior = crate::sketch::polygon_interior_point(&loop_);
            if !crate::sketch::point_in_polygon(interior, &outer) {
                valid = false;
                break;
            }
            if signed_loop_area(&loop_) > 0.0 {
                loop_.reverse();
            }
            holes.push(loop_);
        }
        if !valid {
            prepared.extend(
                source_indices
                    .into_iter()
                    .map(|index| PreparedExtrudeRegion {
                        region: regions[index].clone(),
                        source_indices: vec![index],
                    }),
            );
            continue;
        }
        let area = signed_loop_area(&outer).abs()
            - holes
                .iter()
                .map(|hole| signed_loop_area(hole).abs())
                .sum::<f32>();
        let analytic = merge_analytic_regions(regions, &source_indices, tolerance);
        prepared.push(PreparedExtrudeRegion {
            region: Region {
                boundary: outer,
                holes,
                area,
                analytic,
            },
            source_indices,
        });
    }
    prepared.sort_by_key(|region| region.source_indices[0]);
    prepared
}

/// Fuse a body's parts so that adjacent/overlapping kept regions of a boolean
/// cluster read as one solid (a unioned cluster), while genuinely disjoint lumps
/// stay separate. A union that fails or that would drop material is skipped —
/// the parts simply stay separate (visually identical for a New Body).
pub(crate) fn fuse_overlapping_solids(parts: Vec<KernelSolid>) -> Vec<KernelSolid> {
    let mut out: Vec<KernelSolid> = Vec::new();
    for part in parts {
        let mut merged = false;
        for existing in out.iter_mut() {
            // Only fuse parts whose AABBs touch — disjoint lumps must not be
            // folded into one (false) solid.
            let touch = match (
                crate::mock_kernel::solid_aabb(existing),
                crate::mock_kernel::solid_aabb(&part),
            ) {
                (Some(a), Some(b)) => crate::mock_kernel::aabbs_overlap(&a, &b, 0.05),
                _ => false,
            };
            if touch {
                if let Some(u) = crate::mock_kernel::union(existing, &part) {
                    // Nested AABBs do not imply connected material (two
                    // concentric rings are the common case). Some kernels
                    // legitimately return that union as one `Solid` with two
                    // disconnected shells. Keep the original inputs as separate
                    // body parts unless the boolean actually fused them into one
                    // connected component.
                    let components = u.split_disconnected();
                    if crate::mock_kernel::components_form_connected_material(&components)
                        && u.validate_strict_with_policy(
                            &openrcad::foundation::TolerancePolicy::STANDARD,
                        )
                        .is_ok()
                    {
                        *existing = u;
                        merged = true;
                        break;
                    }
                }
            }
        }
        if !merged {
            out.push(part);
        }
    }
    out
}

/// Group independently valid solids that form one connected piece of material
/// without packing them into an invalid disconnected shell. This is the honest
/// fallback for tangent profile partitions that the boolean solver cannot sew.
pub(crate) fn connected_material_groups(parts: Vec<KernelSolid>) -> Vec<Vec<KernelSolid>> {
    let mut groups: Vec<Vec<KernelSolid>> = Vec::new();
    for part in parts {
        let touching: Vec<usize> = groups
            .iter()
            .enumerate()
            .filter_map(|(index, group)| {
                group
                    .iter()
                    .any(|member| {
                        crate::mock_kernel::components_form_connected_material(&[
                            member.clone(),
                            part.clone(),
                        ])
                    })
                    .then_some(index)
            })
            .collect();
        let Some((&first, rest)) = touching.split_first() else {
            groups.push(vec![part]);
            continue;
        };
        groups[first].push(part);
        for &index in rest.iter().rev() {
            let merged = groups.remove(index);
            groups[first].extend(merged);
        }
    }
    groups
}
