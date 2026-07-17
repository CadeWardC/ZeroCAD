//! Presentation-only mesh clipping for visual section views.

use super::MockMesh;
use std::collections::{HashMap, HashSet};

#[derive(Debug, Clone)]
pub struct MeshSection {
    pub mesh: MockMesh,
    pub contours: Vec<Vec<[f32; 3]>>,
}

#[derive(Clone, Copy)]
struct Vertex {
    point: [f32; 3],
    normal: [f32; 3],
}

/// Clip a display mesh against an arbitrary world-space plane. This does not
/// mutate B-Rep topology; exact modeling operations continue to use kernel
/// booleans. `normal` is normalized here so datum/face/custom callers share one
/// deterministic contract.
pub fn clip_mesh_by_plane(
    mesh: &MockMesh,
    origin: [f32; 3],
    normal: [f32; 3],
    keep_positive: bool,
) -> Result<MeshSection, String> {
    if !origin.iter().all(|value| value.is_finite())
        || !normal.iter().all(|value| value.is_finite())
    {
        return Err("section plane must contain finite coordinates".to_string());
    }
    let length = (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
    if length <= 1.0e-8 {
        return Err("section plane normal must be non-zero".to_string());
    }
    let normal = [normal[0] / length, normal[1] / length, normal[2] / length];
    let signed = |point: [f32; 3]| {
        let distance = (point[0] - origin[0]) * normal[0]
            + (point[1] - origin[1]) * normal[1]
            + (point[2] - origin[2]) * normal[2];
        if keep_positive {
            distance
        } else {
            -distance
        }
    };
    let interpolate = |a: Vertex, b: Vertex, t: f32| {
        let mut vertex_normal = [
            a.normal[0] + (b.normal[0] - a.normal[0]) * t,
            a.normal[1] + (b.normal[1] - a.normal[1]) * t,
            a.normal[2] + (b.normal[2] - a.normal[2]) * t,
        ];
        let length = (vertex_normal[0] * vertex_normal[0]
            + vertex_normal[1] * vertex_normal[1]
            + vertex_normal[2] * vertex_normal[2])
            .sqrt();
        if length > 1.0e-8 {
            vertex_normal = [
                vertex_normal[0] / length,
                vertex_normal[1] / length,
                vertex_normal[2] / length,
            ];
        }
        Vertex {
            point: [
                a.point[0] + (b.point[0] - a.point[0]) * t,
                a.point[1] + (b.point[1] - a.point[1]) * t,
                a.point[2] + (b.point[2] - a.point[2]) * t,
            ],
            normal: vertex_normal,
        }
    };

    let mut output = MockMesh::empty();
    let mut contour_segments = Vec::new();
    for (triangle_index, triangle) in mesh.indices.chunks_exact(3).enumerate() {
        let vertex = |index: u32| {
            let offset = index as usize * 6;
            Vertex {
                point: [
                    mesh.vertices[offset],
                    mesh.vertices[offset + 1],
                    mesh.vertices[offset + 2],
                ],
                normal: [
                    mesh.vertices[offset + 3],
                    mesh.vertices[offset + 4],
                    mesh.vertices[offset + 5],
                ],
            }
        };
        let input = [
            vertex(triangle[0]),
            vertex(triangle[1]),
            vertex(triangle[2]),
        ];
        let mut polygon = Vec::new();
        let mut intersections = Vec::new();
        for index in 0..input.len() {
            let previous = input[(index + input.len() - 1) % input.len()];
            let current = input[index];
            let previous_distance = signed(previous.point);
            let current_distance = signed(current.point);
            let previous_inside = previous_distance >= -1.0e-6;
            let current_inside = current_distance >= -1.0e-6;
            if previous_inside != current_inside {
                let t = previous_distance / (previous_distance - current_distance);
                let crossing = interpolate(previous, current, t.clamp(0.0, 1.0));
                polygon.push(crossing);
                intersections.push(crossing.point);
            }
            if current_inside {
                polygon.push(current);
            }
        }
        intersections.dedup_by(|a, b| {
            (a[0] - b[0]).abs() <= 1.0e-6
                && (a[1] - b[1]).abs() <= 1.0e-6
                && (a[2] - b[2]).abs() <= 1.0e-6
        });
        if intersections.len() == 2 {
            contour_segments.push((intersections[0], intersections[1]));
        }
        if polygon.len() < 3 {
            continue;
        }
        for fan in 1..polygon.len() - 1 {
            let base = (output.vertices.len() / 6) as u32;
            for vertex in [polygon[0], polygon[fan], polygon[fan + 1]] {
                output.vertices.extend_from_slice(&[
                    vertex.point[0],
                    vertex.point[1],
                    vertex.point[2],
                    vertex.normal[0],
                    vertex.normal[1],
                    vertex.normal[2],
                ]);
            }
            output
                .indices
                .extend_from_slice(&[base, base + 1, base + 2]);
            output
                .face_ids
                .push(mesh.face_ids.get(triangle_index).copied().unwrap_or(0));
        }
    }

    for edge in mesh.edge_indices.chunks_exact(2) {
        let point = |index: u32| {
            let offset = index as usize * 3;
            [
                mesh.edge_vertices[offset],
                mesh.edge_vertices[offset + 1],
                mesh.edge_vertices[offset + 2],
            ]
        };
        let mut a = point(edge[0]);
        let mut b = point(edge[1]);
        let da = signed(a);
        let db = signed(b);
        if da < -1.0e-6 && db < -1.0e-6 {
            continue;
        }
        if (da >= 0.0) != (db >= 0.0) {
            let t = (da / (da - db)).clamp(0.0, 1.0);
            let crossing = [
                a[0] + (b[0] - a[0]) * t,
                a[1] + (b[1] - a[1]) * t,
                a[2] + (b[2] - a[2]) * t,
            ];
            if da < 0.0 {
                a = crossing;
            } else {
                b = crossing;
            }
        }
        let base = (output.edge_vertices.len() / 3) as u32;
        output.edge_vertices.extend_from_slice(&a);
        output.edge_vertices.extend_from_slice(&b);
        output.edge_indices.extend_from_slice(&[base, base + 1]);
        output.edge_groups.push(output.edge_groups.len() as u32);
    }
    output.face_refs = mesh.face_refs.clone();
    Ok(MeshSection {
        mesh: output,
        contours: join_contours(contour_segments),
    })
}

fn join_contours(segments: Vec<([f32; 3], [f32; 3])>) -> Vec<Vec<[f32; 3]>> {
    type Key = (i64, i64, i64);
    let key = |point: [f32; 3]| {
        (
            (point[0] * 1.0e5).round() as i64,
            (point[1] * 1.0e5).round() as i64,
            (point[2] * 1.0e5).round() as i64,
        )
    };
    let mut unique = HashSet::new();
    let segments: Vec<_> = segments
        .into_iter()
        .filter(|(a, b)| {
            let (ka, kb) = (key(*a), key(*b));
            ka != kb && unique.insert(if ka < kb { (ka, kb) } else { (kb, ka) })
        })
        .collect();
    let mut by_endpoint: HashMap<Key, Vec<usize>> = HashMap::new();
    for (index, (a, b)) in segments.iter().enumerate() {
        by_endpoint.entry(key(*a)).or_default().push(index);
        by_endpoint.entry(key(*b)).or_default().push(index);
    }
    let mut used = vec![false; segments.len()];
    let mut loops = Vec::new();
    for start_index in 0..segments.len() {
        if used[start_index] {
            continue;
        }
        used[start_index] = true;
        let (start, mut current) = segments[start_index];
        let mut contour = vec![start, current];
        for _ in 0..segments.len() {
            if key(current) == key(start) {
                break;
            }
            let Some(next_index) = by_endpoint
                .get(&key(current))
                .and_then(|indices| indices.iter().copied().find(|index| !used[*index]))
            else {
                break;
            };
            used[next_index] = true;
            let (a, b) = segments[next_index];
            current = if key(a) == key(current) { b } else { a };
            contour.push(current);
        }
        if contour.len() >= 4 && key(*contour.last().unwrap()) == key(contour[0]) {
            contour.pop();
            loops.push(contour);
        }
    }
    loops
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arbitrary_plane_clips_box_and_closes_contour() {
        let mesh = MockMesh::make_box(10.0, 10.0, 10.0);
        let section = clip_mesh_by_plane(&mesh, [5.0, 5.0, 5.0], [1.0, 1.0, 1.0], true)
            .expect("arbitrary section");
        assert!(!section.mesh.indices.is_empty());
        assert_eq!(section.contours.len(), 1);
        assert!(section.contours[0].len() >= 3);
    }
}
