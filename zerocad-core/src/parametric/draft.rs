use super::*;

#[derive(Clone)]
struct RegionDraftSelection {
    outer: Vec<bool>,
    holes: Vec<Vec<bool>>,
}

fn project_point(cs: &CoordinateSystem, point: [f32; 3]) -> (f32, f32) {
    let relative = Vec3::new(point[0], point[1], point[2]).sub(cs.origin);
    (relative.dot(cs.u), relative.dot(cs.v))
}

fn segment_match(
    first: (f32, f32),
    second: (f32, f32),
    a: (f32, f32),
    b: (f32, f32),
    tolerance: f32,
) -> bool {
    let distance = |left: (f32, f32), right: (f32, f32)| (left.0 - right.0).hypot(left.1 - right.1);
    (distance(first, a) + distance(second, b)).min(distance(first, b) + distance(second, a))
        <= tolerance
}

fn resolved_mesh_face<'a>(
    mesh: &'a MockMesh,
    resolved: &FaceRef,
) -> Option<&'a crate::mock_kernel::MeshFaceRef> {
    let durable_name = resolved
        .topology
        .as_ref()
        .and_then(|topology| topology.face_id.as_deref());
    let mut candidates: Vec<_> = mesh
        .face_refs
        .iter()
        .filter(|face| {
            durable_name.is_none_or(|name| {
                face.topology
                    .as_ref()
                    .and_then(|topology| topology.face_id.as_deref())
                    == Some(name)
            })
        })
        .collect();
    candidates.sort_by(|left, right| {
        let distance = |face: &crate::mock_kernel::MeshFaceRef| {
            Vec3::new(
                face.centroid[0] - resolved.centroid[0],
                face.centroid[1] - resolved.centroid[1],
                face.centroid[2] - resolved.centroid[2],
            )
            .length()
        };
        distance(left).total_cmp(&distance(right))
    });
    candidates.into_iter().next()
}

fn selected_region_edge(
    mesh: &MockMesh,
    face: &crate::mock_kernel::MeshFaceRef,
    sources: &[SketchExtrudeRegionSource],
) -> Option<(usize, Option<usize>, usize)> {
    let mut matches = Vec::new();
    for (region_index, source) in sources.iter().enumerate() {
        let Some(edge) = drafted_face_base_edge(mesh, face.face_id, &source.cs, source.depth)
        else {
            continue;
        };
        let first = project_point(&source.cs, edge.p0);
        let second = project_point(&source.cs, edge.p1);
        let scale = source
            .boundary
            .iter()
            .chain(source.holes.iter().flatten())
            .flat_map(|point| [point.0.abs(), point.1.abs()])
            .fold(1.0_f32, f32::max);
        let tolerance = scale * 1.0e-4 + 1.0e-4;
        for edge_index in 0..source.boundary.len() {
            if segment_match(
                first,
                second,
                source.boundary[edge_index],
                source.boundary[(edge_index + 1) % source.boundary.len()],
                tolerance,
            ) {
                matches.push((region_index, None, edge_index));
            }
        }
        for (hole_index, hole) in source.holes.iter().enumerate() {
            for edge_index in 0..hole.len() {
                if segment_match(
                    first,
                    second,
                    hole[edge_index],
                    hole[(edge_index + 1) % hole.len()],
                    tolerance,
                ) {
                    matches.push((region_index, Some(hole_index), edge_index));
                }
            }
        }
    }
    matches.sort_unstable();
    matches.dedup();
    (matches.len() == 1).then(|| matches[0])
}

fn neutral_section(
    neutral: &DraftNeutral,
    body: &LiveBody,
    datums: &HashMap<String, DatumValue>,
    source: &SketchExtrudeRegionSource,
) -> Result<bool, String> {
    let (origin, normal) = match neutral {
        DraftNeutral::Face(face) => {
            if face
                .topology
                .as_ref()
                .and_then(|topology| topology.face_id.as_deref())
                .is_none()
            {
                return Err("the neutral face must carry a durable topology name".into());
            }
            let resolved = resolve_face_ref_by_topology(body, face)
                .ok_or_else(|| "the named neutral face no longer resolves".to_string())?;
            (
                Vec3::new(
                    resolved.centroid[0],
                    resolved.centroid[1],
                    resolved.centroid[2],
                ),
                Vec3::new(resolved.normal[0], resolved.normal[1], resolved.normal[2]).normalize(),
            )
        }
        DraftNeutral::Datum(id) => match datums.get(id) {
            Some(DatumValue::Plane(cs)) => (cs.origin, cs.n.normalize()),
            _ => {
                return Err(format!(
                    "neutral datum '{id}' no longer resolves to a plane"
                ))
            }
        },
    };
    let axis = source.cs.n.normalize();
    if normal.dot(axis).abs() < 0.999 {
        return Err(
            "the neutral reference is not perpendicular to the prism pull direction".into(),
        );
    }
    let offset = origin.sub(source.cs.origin).dot(axis);
    let tolerance = source.depth.abs().max(1.0) * 1.0e-4;
    if offset.abs() <= tolerance {
        Ok(false)
    } else if (offset - source.depth).abs() <= tolerance {
        Ok(true)
    } else {
        Err("the neutral reference is not one of the prism end sections".into())
    }
}

fn stamp_draft_faces(mesh: &mut MockMesh, target: &str, draft_id: &str, region_index: usize) {
    let quantize = |value: f32| (f64::from(value) * 1.0e4).round() as i64;
    let mut order: Vec<_> = (0..mesh.face_refs.len()).collect();
    order.sort_by_key(|index| {
        let centroid = mesh.face_refs[*index].centroid;
        (
            quantize(centroid[0]),
            quantize(centroid[1]),
            quantize(centroid[2]),
        )
    });
    for (face_index, index) in order.into_iter().enumerate() {
        mesh.face_refs[index].topology = Some(crate::mock_kernel::MeshTopologyFaceRef {
            body_id: Some(target.to_string()),
            component_id: None,
            topology_version: Some(0),
            face_id: Some(format!(
                "draft:{draft_id}:region:{region_index}:face:{face_index}"
            )),
            surface_kind: Some("plane".to_string()),
            producer_feature_id: Some(draft_id.to_string()),
            source_entity_id: None,
        });
    }
    crate::mock_kernel::populate_edge_adjacent_face_names(mesh);
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn apply_standalone_draft(
    draft_id: &str,
    target: &str,
    faces: &[FaceRef],
    neutral: &DraftNeutral,
    angle_deg: f32,
    flip_pull: bool,
    datums: &HashMap<String, DatumValue>,
    live: &mut [LiveBody],
) -> Result<(), String> {
    if faces.is_empty() {
        return Err("select at least one planar side face".into());
    }
    if !angle_deg.is_finite() || angle_deg.abs() >= 89.0 {
        return Err(format!(
            "angle {angle_deg:.3}° must be finite and strictly between -89° and 89°"
        ));
    }
    let body_index = live
        .iter()
        .position(|body| body.id == target)
        .ok_or_else(|| format!("target body '{target}' no longer exists"))?;
    let body = &live[body_index];
    let source = body.sketch_source.as_ref().ok_or_else(|| {
        "v1 supports only planar prismatic bodies with retained sections".to_string()
    })?;
    if source.regions.is_empty()
        || source
            .regions
            .iter()
            .any(|region| region.rect_circle.is_some())
    {
        return Err("v1 supports only straight-edged planar prism regions".into());
    }
    let reference = source
        .regions
        .first()
        .ok_or_else(|| "the prism has no source section".to_string())?;
    let common_frame = source.regions.iter().all(|region| {
        region
            .cs
            .n
            .normalize()
            .dot(reference.cs.n.normalize())
            .abs()
            > 0.999
            && (region.depth - reference.depth).abs() <= reference.depth.abs().max(1.0) * 1.0e-5
    });
    if !common_frame {
        return Err("the target regions do not share one prismatic pull direction".into());
    }
    let neutral_is_far = neutral_section(neutral, body, datums, reference)?;
    let mesh = body
        .pristine
        .as_deref()
        .ok_or_else(|| "v1 requires an unchanged planar-prism topology snapshot".to_string())?;
    let mut selections: Vec<_> = source
        .regions
        .iter()
        .map(|region| RegionDraftSelection {
            outer: vec![false; region.boundary.len()],
            holes: region
                .holes
                .iter()
                .map(|hole| vec![false; hole.len()])
                .collect(),
        })
        .collect();
    for face in faces {
        if face
            .topology
            .as_ref()
            .and_then(|topology| topology.face_id.as_deref())
            .is_none()
        {
            return Err("every selected side face must carry a durable topology name".into());
        }
        let resolved = resolve_face_ref_by_topology(body, face)
            .ok_or_else(|| "a selected named side face no longer resolves".to_string())?;
        let face_normal =
            Vec3::new(resolved.normal[0], resolved.normal[1], resolved.normal[2]).normalize();
        if face_normal.dot(reference.cs.n.normalize()).abs() > 0.1 {
            return Err("selected Draft faces must be prism side faces".into());
        }
        let planar = body.parts.iter().any(|part| {
            !crate::mock_kernel::kernel_faces_matching(part, resolved.centroid, resolved.normal)
                .is_empty()
        });
        if !planar {
            return Err("curved Draft faces are deferred beyond v1".into());
        }
        let mesh_face = resolved_mesh_face(mesh, &resolved)
            .ok_or_else(|| "the selected side face has no historical mesh face".to_string())?;
        let (region, hole, edge) = selected_region_edge(mesh, mesh_face, &source.regions)
            .ok_or_else(|| {
                "the selected side face is ambiguous in the source section".to_string()
            })?;
        if let Some(hole) = hole {
            selections[region].holes[hole][edge] = true;
        } else {
            selections[region].outer[edge] = true;
        }
    }
    if angle_deg.abs() <= f32::EPSILON {
        return Ok(());
    }
    let effective_angle = if flip_pull { -angle_deg } else { angle_deg };
    let mut parts = Vec::with_capacity(source.regions.len());
    let mut pristine = MockMesh::empty();
    for (region_index, (region, selected)) in source.regions.iter().zip(&selections).enumerate() {
        let (section, depth) = if neutral_is_far {
            (
                region
                    .cs
                    .with_origin(region.cs.origin.add(region.cs.n.mul(region.depth))),
                -region.depth,
            )
        } else {
            (region.cs, region.depth)
        };
        let solid = drafted_region_solid_selected(
            &region.boundary,
            &region.holes,
            depth,
            &section,
            effective_angle,
            &selected.outer,
            &selected.holes,
        )
        .map_err(|error| error.to_string())?;
        let mut part_mesh = MockMesh::try_from_solid(&solid)
            .map_err(|error| format!("the drafted candidate cannot be displayed: {error}"))?;
        stamp_draft_faces(&mut part_mesh, target, draft_id, region_index);
        pristine.append(part_mesh);
        parts.push(solid);
    }
    let candidate = &mut live[body_index];
    candidate.parts = parts;
    candidate.pristine = Some(std::sync::Arc::new(pristine));
    candidate.sketch_source = None;
    Ok(())
}
