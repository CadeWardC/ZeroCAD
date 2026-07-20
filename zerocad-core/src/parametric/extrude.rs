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
                Dir2d::new(dx / length, dy / length),
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
/// holds the analytic mesh while the body is untouched by any boolean, so plain
/// bodies keep their nice hidden-line wireframes. A boolean clears it, forcing a
/// fresh tessellation from `parts`.
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
