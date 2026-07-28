//! Display-mesh cleanup used by evaluator families.
//!
//! These helpers operate only on derived display data. They never decide
//! feature scheduling, mutate the document graph, or commit candidate bodies.

use crate::geometry::Vec3;
use crate::mock_kernel::{KernelSolid, MockMesh};

fn named_pattern_part_meshes(
    parts: &[KernelSolid],
    source_names: Option<&MockMesh>,
    body_id: &str,
) -> Vec<MockMesh> {
    parts
        .iter()
        .map(|part| {
            source_names.map_or_else(
                || MockMesh::from_solid(part),
                |source| crate::mock_kernel::propagate_face_names(source, part, body_id),
            )
        })
        .collect()
}

/// Name every face of a joined pattern result, preserving source owners where
/// geometry continued and assigning the operation as owner of genuinely new
/// faces. Used when no mirror-plane display cleanup is necessary.
pub(crate) fn joined_pattern_display_mesh(
    node_id: &str,
    parts: &[KernelSolid],
    source_names: Option<&MockMesh>,
    body_id: &str,
) -> MockMesh {
    let mut combined = MockMesh::empty();
    for mesh in named_pattern_part_meshes(parts, source_names, body_id) {
        combined.append(mesh);
    }
    super::stamp_joined_pattern_face_refs(&mut combined, body_id, node_id);
    combined
}

/// Display mesh for Mirror+Join. Guarded boolean fallbacks can retain coincident
/// caps and distinct face ids at the mirror plane even though they are one logical
/// body. Remove those internal caps and merge continuous coplanar faces before
/// suppressing the construction edge at the interface.
pub(crate) fn mirror_join_display_mesh(
    node_id: &str,
    parts: &[KernelSolid],
    origin: Vec3,
    normal: Vec3,
    source_names: Option<&MockMesh>,
    body_id: &str,
) -> MockMesh {
    let mut combined = MockMesh::empty();
    let mut removed_plane_triangles = 0usize;
    let mut removed_plane_edges = 0usize;
    let mut removed_overlap_edges = 0usize;
    let raw_meshes = named_pattern_part_meshes(parts, source_names, body_id);
    for (part_index, mut mesh) in raw_meshes.iter().cloned().enumerate() {
        let triangles_before = mesh.indices.len() / 3;
        suppress_faces_on_plane(&mut mesh, origin, normal);
        removed_plane_triangles += triangles_before - mesh.indices.len() / 3;
        let edges_before = mesh.edge_indices.len() / 2;
        suppress_edges_on_plane(&mut mesh, origin, normal);
        removed_plane_edges += edges_before - mesh.edge_indices.len() / 2;
        let edges_before = mesh.edge_indices.len() / 2;
        suppress_edges_inside_other_parts(&mut mesh, part_index, parts, &raw_meshes);
        removed_overlap_edges += edges_before - mesh.edge_indices.len() / 2;
        combined.append(mesh);
    }
    let faces_before = combined.face_refs.len();
    merge_coplanar_faces_across_plane(&mut combined, origin, normal);
    let edge_segments_before_regroup = combined.edge_indices.len() / 2;
    let edge_groups_before_regroup = combined
        .edge_groups
        .iter()
        .copied()
        .collect::<std::collections::HashSet<_>>()
        .len();
    regroup_joined_mirror_edges(&mut combined);
    super::stamp_joined_pattern_face_refs(&mut combined, body_id, node_id);
    log::debug!(
        "[mirror_join:{node_id}] display cleanup parts={} removed_plane_triangles={} removed_plane_edges={} removed_overlap_edges={} merged_faces={} edge_segments={}->{} edge_groups={}->{} final_triangles={} final_faces={}",
        parts.len(),
        removed_plane_triangles,
        removed_plane_edges,
        removed_overlap_edges,
        faces_before.saturating_sub(combined.face_refs.len()),
        edge_segments_before_regroup,
        combined.edge_indices.len() / 2,
        edge_groups_before_regroup,
        combined.edge_refs.len(),
        combined.indices.len() / 3,
        combined.face_refs.len(),
    );
    combined
}

/// Rebuild edge ownership after all joined-mirror parts have been combined.
/// `MockMesh::append` deliberately keeps each input's edge groups separate; that
/// is correct for separate bodies but leaves a continuous mirrored boundary as
/// two selectable half-edges. Here tangent-connected and collinear-overlapping
/// segments are unioned globally. Straight runs are collapsed to one segment;
/// curved runs retain their chords so circle/arc fitting remains analytic.
fn regroup_joined_mirror_edges(mesh: &mut MockMesh) {
    let segment_count = mesh.edge_indices.len() / 2;
    if segment_count == 0 {
        mesh.edge_groups.clear();
        mesh.edge_refs.clear();
        return;
    }

    let initial =
        crate::mock_kernel::group_edge_segments(&mesh.edge_vertices, &mesh.edge_indices, None);
    let mut parent: Vec<usize> = (0..segment_count).collect();
    fn find(parent: &mut [usize], mut value: usize) -> usize {
        while parent[value] != value {
            parent[value] = parent[parent[value]];
            value = parent[value];
        }
        value
    }
    fn union(parent: &mut [usize], a: usize, b: usize) {
        let (a, b) = (find(parent, a), find(parent, b));
        if a != b {
            parent[a.max(b)] = a.min(b);
        }
    }
    for a in 0..segment_count {
        for b in (a + 1)..segment_count {
            if initial[a] == initial[b] || collinear_segments_overlap(mesh, a, b) {
                union(&mut parent, a, b);
            }
        }
    }

    let mut dense = std::collections::HashMap::new();
    let mut groups = vec![0u32; segment_count];
    let mut next = 0u32;
    for (segment, group) in groups.iter_mut().enumerate() {
        let root = find(&mut parent, segment);
        *group = *dense.entry(root).or_insert_with(|| {
            let value = next;
            next += 1;
            value
        });
    }

    let mut members: std::collections::BTreeMap<u32, Vec<usize>> =
        std::collections::BTreeMap::new();
    for (segment, group) in groups.iter().copied().enumerate() {
        members.entry(group).or_default().push(segment);
    }
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut normals = Vec::new();
    let mut rebuilt_groups = Vec::new();
    for (group, segments) in members {
        if let Some((start, end)) = collapsed_collinear_run(mesh, &segments) {
            let base = (vertices.len() / 3) as u32;
            vertices.extend_from_slice(&start);
            vertices.extend_from_slice(&end);
            indices.extend_from_slice(&[base, base + 1]);
            let first = segments[0];
            if mesh.edge_face_normals.len() >= (first + 1) * 6 {
                normals.extend_from_slice(&mesh.edge_face_normals[first * 6..first * 6 + 6]);
            }
            rebuilt_groups.push(group);
        } else {
            for segment in segments {
                let base = (vertices.len() / 3) as u32;
                for &vertex in &mesh.edge_indices[segment * 2..segment * 2 + 2] {
                    let offset = vertex as usize * 3;
                    vertices.extend_from_slice(&mesh.edge_vertices[offset..offset + 3]);
                }
                indices.extend_from_slice(&[base, base + 1]);
                if mesh.edge_face_normals.len() >= (segment + 1) * 6 {
                    normals
                        .extend_from_slice(&mesh.edge_face_normals[segment * 6..segment * 6 + 6]);
                }
                rebuilt_groups.push(group);
            }
        }
    }
    mesh.edge_vertices = vertices;
    mesh.edge_indices = indices;
    mesh.edge_face_normals = normals;
    mesh.edge_groups = rebuilt_groups;
    mesh.edge_refs = crate::mock_kernel::mesh_edge_refs_from_groups(
        &mesh.vertices,
        &mesh.indices,
        &mesh.edge_vertices,
        &mesh.edge_indices,
        &mesh.edge_face_normals,
        &mesh.edge_groups,
    );
    crate::mock_kernel::populate_edge_adjacent_face_names(mesh);
}

fn edge_segment_points(mesh: &MockMesh, segment: usize) -> ([f32; 3], [f32; 3]) {
    let read = |vertex: u32| {
        let offset = vertex as usize * 3;
        [
            mesh.edge_vertices[offset],
            mesh.edge_vertices[offset + 1],
            mesh.edge_vertices[offset + 2],
        ]
    };
    (
        read(mesh.edge_indices[segment * 2]),
        read(mesh.edge_indices[segment * 2 + 1]),
    )
}

fn collinear_segments_overlap(mesh: &MockMesh, a: usize, b: usize) -> bool {
    let ((a0, a1), (b0, b1)) = (edge_segment_points(mesh, a), edge_segment_points(mesh, b));
    collinear_edge_points_overlap(a0, a1, b0, b1)
}

fn collinear_edge_points_overlap(a0: [f32; 3], a1: [f32; 3], b0: [f32; 3], b1: [f32; 3]) -> bool {
    let direction = [a1[0] - a0[0], a1[1] - a0[1], a1[2] - a0[2]];
    let length =
        (direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2])
            .sqrt();
    if length < 1.0e-6 {
        return false;
    }
    let unit = [
        direction[0] / length,
        direction[1] / length,
        direction[2] / length,
    ];
    let projection = |point: [f32; 3]| {
        (point[0] - a0[0]) * unit[0] + (point[1] - a0[1]) * unit[1] + (point[2] - a0[2]) * unit[2]
    };
    let distance_to_line = |point: [f32; 3]| {
        let t = projection(point);
        let nearest = [
            a0[0] + unit[0] * t,
            a0[1] + unit[1] * t,
            a0[2] + unit[2] * t,
        ];
        crate::mock_kernel::dist3(point, nearest)
    };
    if distance_to_line(b0) > 1.0e-3 || distance_to_line(b1) > 1.0e-3 {
        return false;
    }
    let (b0, b1) = (projection(b0), projection(b1));
    let (b_lo, b_hi) = (b0.min(b1), b0.max(b1));
    b_hi >= -1.0e-3 && b_lo <= length + 1.0e-3
}

fn collapsed_collinear_run(mesh: &MockMesh, segments: &[usize]) -> Option<([f32; 3], [f32; 3])> {
    let (origin, first_end) = edge_segment_points(mesh, *segments.first()?);
    let direction = [
        first_end[0] - origin[0],
        first_end[1] - origin[1],
        first_end[2] - origin[2],
    ];
    let length =
        (direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2])
            .sqrt();
    if length < 1.0e-6 {
        return None;
    }
    let unit = [
        direction[0] / length,
        direction[1] / length,
        direction[2] / length,
    ];
    let mut lo = 0.0f32;
    let mut hi = length;
    for &segment in segments {
        let (a, b) = edge_segment_points(mesh, segment);
        for point in [a, b] {
            let t = (point[0] - origin[0]) * unit[0]
                + (point[1] - origin[1]) * unit[1]
                + (point[2] - origin[2]) * unit[2];
            let nearest = [
                origin[0] + unit[0] * t,
                origin[1] + unit[1] * t,
                origin[2] + unit[2] * t,
            ];
            if crate::mock_kernel::dist3(point, nearest) > 1.0e-3 {
                return None;
            }
            lo = lo.min(t);
            hi = hi.max(t);
        }
    }
    Some((
        [
            origin[0] + unit[0] * lo,
            origin[1] + unit[1] * lo,
            origin[2] + unit[2] * lo,
        ],
        [
            origin[0] + unit[0] * hi,
            origin[1] + unit[1] * hi,
            origin[2] + unit[2] * hi,
        ],
    ))
}

fn plane_distance(p: [f32; 3], origin: Vec3, normal: Vec3) -> f32 {
    (p[0] - origin.x) * normal.x + (p[1] - origin.y) * normal.y + (p[2] - origin.z) * normal.z
}

fn suppress_faces_on_plane(mesh: &mut MockMesh, origin: Vec3, normal: Vec3) {
    let mut indices = Vec::with_capacity(mesh.indices.len());
    let mut face_ids = Vec::with_capacity(mesh.face_ids.len());
    for (triangle, tri) in mesh.indices.chunks_exact(3).enumerate() {
        let on_plane = tri.iter().all(|&vertex| {
            let base = vertex as usize * 6;
            plane_distance(
                [
                    mesh.vertices[base],
                    mesh.vertices[base + 1],
                    mesh.vertices[base + 2],
                ],
                origin,
                normal,
            )
            .abs()
                < 1.0e-3
        });
        if !on_plane {
            indices.extend_from_slice(tri);
            face_ids.push(mesh.face_ids.get(triangle).copied().unwrap_or(0));
        }
    }
    mesh.indices = indices;
    mesh.face_ids = face_ids;
    let topology: std::collections::HashMap<_, _> = mesh
        .face_refs
        .iter()
        .map(|face| (face.face_id, face.topology.clone()))
        .collect();
    mesh.face_refs =
        crate::mock_kernel::mesh_face_refs(&mesh.vertices, &mesh.indices, &mesh.face_ids);
    for face in &mut mesh.face_refs {
        face.topology = topology.get(&face.face_id).cloned().flatten();
    }
}

fn merge_coplanar_faces_across_plane(mesh: &mut MockMesh, origin: Vec3, normal: Vec3) {
    use std::collections::{HashMap, HashSet};

    let mut plane_points: HashMap<u32, HashSet<(i64, i64, i64)>> = HashMap::new();
    for (triangle, tri) in mesh.indices.chunks_exact(3).enumerate() {
        let fid = mesh.face_ids.get(triangle).copied().unwrap_or(0);
        for &vertex in tri {
            let base = vertex as usize * 6;
            let p = [
                mesh.vertices[base],
                mesh.vertices[base + 1],
                mesh.vertices[base + 2],
            ];
            if plane_distance(p, origin, normal).abs() < 1.0e-3 {
                let q = |v: f32| (v as f64 * 10_000.0).round() as i64;
                plane_points
                    .entry(fid)
                    .or_default()
                    .insert((q(p[0]), q(p[1]), q(p[2])));
            }
        }
    }

    let mut face_bounds: HashMap<u32, ([f32; 3], [f32; 3])> = HashMap::new();
    for (triangle, tri) in mesh.indices.chunks_exact(3).enumerate() {
        let fid = mesh.face_ids.get(triangle).copied().unwrap_or(0);
        let bounds = face_bounds
            .entry(fid)
            .or_insert(([f32::INFINITY; 3], [f32::NEG_INFINITY; 3]));
        for &vertex in tri {
            let base = vertex as usize * 6;
            for axis in 0..3 {
                bounds.0[axis] = bounds.0[axis].min(mesh.vertices[base + axis]);
                bounds.1[axis] = bounds.1[axis].max(mesh.vertices[base + axis]);
            }
        }
    }

    let mut remap: HashMap<u32, u32> = mesh
        .face_refs
        .iter()
        .map(|face| (face.face_id, face.face_id))
        .collect();
    for (i, a) in mesh.face_refs.iter().enumerate() {
        for b in mesh.face_refs.iter().skip(i + 1) {
            let dot =
                a.normal[0] * b.normal[0] + a.normal[1] * b.normal[1] + a.normal[2] * b.normal[2];
            let plane_delta = (a.normal[0] * (a.centroid[0] - b.centroid[0])
                + a.normal[1] * (a.centroid[1] - b.centroid[1])
                + a.normal[2] * (a.centroid[2] - b.centroid[2]))
                .abs();
            let shared = plane_points.get(&a.face_id).is_some_and(|points| {
                plane_points
                    .get(&b.face_id)
                    .is_some_and(|other| points.intersection(other).take(2).count() >= 2)
            });
            let overlapping_footprints = face_bounds
                .get(&a.face_id)
                .zip(face_bounds.get(&b.face_id))
                .is_some_and(|(a, b)| {
                    // Coplanar faces have one near-zero local dimension. Their
                    // remaining footprint must overlap or touch on both axes;
                    // this joins overlapping mirror faces without grouping
                    // unrelated coplanar faces elsewhere in the body.
                    (0..3)
                        .filter(|&axis| {
                            a.1[axis].min(b.1[axis]) >= a.0[axis].max(b.0[axis]) - 1.0e-3
                        })
                        .count()
                        >= 2
                });
            if dot > 0.999 && plane_delta < 1.0e-3 && (shared || overlapping_footprints) {
                let canonical = remap[&a.face_id].min(remap[&b.face_id]);
                let old_a = remap[&a.face_id];
                let old_b = remap[&b.face_id];
                for value in remap.values_mut() {
                    if *value == old_a || *value == old_b {
                        *value = canonical;
                    }
                }
            }
        }
    }
    for fid in &mut mesh.face_ids {
        *fid = remap.get(fid).copied().unwrap_or(*fid);
    }
    let topology: HashMap<_, _> = mesh
        .face_refs
        .iter()
        .filter_map(|face| {
            face.topology
                .clone()
                .map(|topology| (remap[&face.face_id], topology))
        })
        .collect();
    mesh.face_refs =
        crate::mock_kernel::mesh_face_refs(&mesh.vertices, &mesh.indices, &mesh.face_ids);
    for face in &mut mesh.face_refs {
        face.topology = topology.get(&face.face_id).cloned();
    }
}

fn suppress_edges_inside_other_parts(
    mesh: &mut MockMesh,
    owner: usize,
    parts: &[KernelSolid],
    raw_meshes: &[MockMesh],
) {
    let mut hidden_segments = std::collections::HashSet::new();
    for (segment, pair) in mesh.edge_indices.chunks_exact(2).enumerate() {
        let read = |vertex: u32| {
            let base = vertex as usize * 3;
            [
                mesh.edge_vertices[base],
                mesh.edge_vertices[base + 1],
                mesh.edge_vertices[base + 2],
            ]
        };
        let a = read(pair[0]);
        let b = read(pair[1]);
        let midpoint = [
            (a[0] + b[0]) * 0.5,
            (a[1] + b[1]) * 0.5,
            (a[2] + b[2]) * 0.5,
        ];
        let mut probes = vec![midpoint];
        if let Some(normals) = mesh.edge_face_normals.get(segment * 6..segment * 6 + 6) {
            for normal in [
                [normals[0], normals[1], normals[2]],
                [normals[3], normals[4], normals[5]],
            ] {
                probes.push([
                    midpoint[0] - normal[0] * 1.0e-3,
                    midpoint[1] - normal[1] * 1.0e-3,
                    midpoint[2] - normal[2] * 1.0e-3,
                ]);
            }
        }
        let inside_other = parts.iter().enumerate().any(|(index, part)| {
            index != owner
                && probes.iter().any(|probe| {
                    openrcad::prelude::boolean::point_in_solid(
                        &openrcad::foundation::Pnt::new(
                            probe[0] as f64,
                            probe[1] as f64,
                            probe[2] as f64,
                        ),
                        part,
                    )
                })
        });
        // A coincident exterior boundary is present in both input meshes. Keep
        // both spans for now: the global regrouping below unions/collapses them
        // into one full edge. An internal cross-boundary has no collinear mate
        // in the other mesh and is correctly removed here.
        let has_coincident_exterior = raw_meshes.iter().enumerate().any(|(index, other)| {
            index != owner
                && (0..other.edge_indices.len() / 2).any(|other_segment| {
                    let (c, d) = edge_segment_points(other, other_segment);
                    collinear_edge_points_overlap(a, b, c, d)
                })
        });
        if inside_other && !has_coincident_exterior {
            hidden_segments.insert(segment);
        }
    }
    if hidden_segments.is_empty() {
        return;
    }
    retain_edge_segments(mesh, |segment, _| !hidden_segments.contains(&segment));
}

fn retain_edge_segments(mesh: &mut MockMesh, keep: impl Fn(usize, u32) -> bool) {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut normals = Vec::new();
    let mut groups = Vec::new();
    for (segment, pair) in mesh.edge_indices.chunks_exact(2).enumerate() {
        let group = mesh
            .edge_groups
            .get(segment)
            .copied()
            .unwrap_or(segment as u32);
        if !keep(segment, group) {
            continue;
        }
        let base = (vertices.len() / 3) as u32;
        for &vertex in pair {
            let offset = vertex as usize * 3;
            vertices.extend_from_slice(&mesh.edge_vertices[offset..offset + 3]);
        }
        indices.extend_from_slice(&[base, base + 1]);
        if mesh.edge_face_normals.len() >= (segment + 1) * 6 {
            normals.extend_from_slice(&mesh.edge_face_normals[segment * 6..segment * 6 + 6]);
        }
        groups.push(group);
    }
    let kept: std::collections::HashSet<_> = groups.iter().copied().collect();
    mesh.edge_refs.retain(|edge| kept.contains(&edge.group));
    mesh.edge_vertices = vertices;
    mesh.edge_indices = indices;
    mesh.edge_face_normals = normals;
    mesh.edge_groups = groups;
}

fn suppress_edges_on_plane(mesh: &mut MockMesh, origin: Vec3, normal: Vec3) {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut normals = Vec::new();
    let mut groups = Vec::new();
    for (segment, pair) in mesh.edge_indices.chunks_exact(2).enumerate() {
        let ia = pair[0] as usize * 3;
        let ib = pair[1] as usize * 3;
        let a = [
            mesh.edge_vertices[ia],
            mesh.edge_vertices[ia + 1],
            mesh.edge_vertices[ia + 2],
        ];
        let b = [
            mesh.edge_vertices[ib],
            mesh.edge_vertices[ib + 1],
            mesh.edge_vertices[ib + 2],
        ];
        if plane_distance(a, origin, normal).abs() < 1.0e-3
            && plane_distance(b, origin, normal).abs() < 1.0e-3
        {
            continue;
        }
        let base = (vertices.len() / 3) as u32;
        vertices.extend_from_slice(&a);
        vertices.extend_from_slice(&b);
        indices.extend_from_slice(&[base, base + 1]);
        if mesh.edge_face_normals.len() >= (segment + 1) * 6 {
            normals.extend_from_slice(&mesh.edge_face_normals[segment * 6..segment * 6 + 6]);
        }
        groups.push(
            mesh.edge_groups
                .get(segment)
                .copied()
                .unwrap_or(segment as u32),
        );
    }
    let kept: std::collections::HashSet<u32> = groups.iter().copied().collect();
    mesh.edge_refs.retain(|edge| kept.contains(&edge.group));
    mesh.edge_vertices = vertices;
    mesh.edge_indices = indices;
    mesh.edge_face_normals = normals;
    mesh.edge_groups = groups;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn overlapping_mirror_fallback_is_one_continuous_display_body() {
        let source = openrcad::primitives::make_box_operation(
            &openrcad::foundation::Pnt::new(-1.0, 0.0, 0.0),
            4.0,
            2.0,
            1.0,
        )
        .unwrap()
        .value;
        let mirrored = openrcad::primitives::make_box_operation(
            &openrcad::foundation::Pnt::new(-3.0, 0.0, 0.0),
            4.0,
            2.0,
            1.0,
        )
        .unwrap()
        .value;
        let mesh = mirror_join_display_mesh(
            "test_mirror",
            &[source, mirrored],
            Vec3::ZERO,
            Vec3::X,
            None,
            "source",
        );

        assert_eq!(
            mesh.face_refs
                .iter()
                .filter(|face| face.normal[2] > 0.99)
                .count(),
            1,
            "overlapping coplanar top faces should select as one face"
        );
        let internal_outline_segments = mesh
            .edge_indices
            .chunks_exact(2)
            .filter(|edge| {
                let x = |vertex: u32| mesh.edge_vertices[vertex as usize * 3];
                [1.0, -1.0].iter().any(|cut| {
                    (x(edge[0]) - cut).abs() < 1.0e-3 && (x(edge[1]) - cut).abs() < 1.0e-3
                })
            })
            .count();
        assert_eq!(
            internal_outline_segments, 0,
            "overlap boundaries inside the joined result must not be drawn or picked"
        );
        let full_width_edges = mesh
            .edge_refs
            .iter()
            .filter(|edge| {
                let lo = edge.p0[0].min(edge.p1[0]);
                let hi = edge.p0[0].max(edge.p1[0]);
                (lo + 3.0).abs() < 1.0e-3 && (hi - 3.0).abs() < 1.0e-3
            })
            .count();
        assert!(
            full_width_edges >= 1,
            "the mirrored halves' collinear boundary must regroup into one full-width edge"
        );
    }
}
