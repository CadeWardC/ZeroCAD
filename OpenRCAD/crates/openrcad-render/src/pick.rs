//! CPU ray-cast face picking through the per-triangle face-id buffer.
//!
//! The renderer keeps each triangle's source-face index. To resolve a click we
//! shoot a ray from the camera through the cursor and intersect it against a
//! small BVH over the flat-shaded triangle soup. The nearest hit's face id is
//! the picked face. This stays in safe, deterministic, unit-testable Rust: no
//! GPU read-back round trip.

use openrcad_mesh::GpuMesh;

/// Triangle positions (CPU side) paired with each triangle's source face id.
pub struct Picker {
    /// Flattened triangle vertices: 9 floats (3 vertices x xyz) per triangle.
    positions: Vec<f32>,
    /// One source face index per triangle.
    face_ids: Vec<u32>,
    /// Triangle indices ordered by the BVH leaves.
    tri_indices: Vec<usize>,
    /// Binary bounding-volume hierarchy over [`tri_indices`](Self::tri_indices).
    nodes: Vec<Node>,
}

#[derive(Clone, Copy, Debug)]
struct Node {
    min: [f32; 3],
    max: [f32; 3],
    start: usize,
    len: usize,
    left: Option<usize>,
    right: Option<usize>,
}

impl Picker {
    /// Build from the same flat-shaded [`GpuMesh`] the renderer uploaded.
    ///
    /// The mesh is already unwelded (three independent vertices per triangle),
    /// so its `positions`/`face_ids` map one triangle to one face id directly.
    pub fn from_gpu_mesh(mesh: &GpuMesh) -> Self {
        let mut picker = Self {
            positions: mesh.positions.clone(),
            face_ids: mesh.face_ids.clone(),
            tri_indices: (0..mesh.face_ids.len()).collect(),
            nodes: Vec::new(),
        };
        if !picker.tri_indices.is_empty() {
            picker.build_node(0, picker.tri_indices.len());
        }
        picker
    }

    /// Number of triangles.
    pub fn triangle_count(&self) -> usize {
        self.face_ids.len()
    }

    /// Return the source face id of the nearest triangle hit by the ray, if any.
    ///
    /// `dir` need not be normalized. Hits behind the origin (t < 0) are ignored.
    pub fn pick(&self, origin: [f32; 3], dir: [f32; 3]) -> Option<u32> {
        let mut best_t = f32::INFINITY;
        let mut best_face = None;

        let mut stack = Vec::new();
        if !self.nodes.is_empty() {
            stack.push(0usize);
        }

        while let Some(node_id) = stack.pop() {
            let node = self.nodes[node_id];
            let Some(box_t) = ray_box(origin, dir, node.min, node.max) else {
                continue;
            };
            if box_t > best_t {
                continue;
            }

            match (node.left, node.right) {
                (Some(left), Some(right)) => {
                    stack.push(right);
                    stack.push(left);
                }
                _ => {
                    for &tri in &self.tri_indices[node.start..node.start + node.len] {
                        let (a, b, c) = self.triangle(tri);
                        if let Some(t) = ray_triangle(origin, dir, a, b, c) {
                            if t < best_t {
                                best_t = t;
                                best_face = Some(self.face_ids[tri]);
                            }
                        }
                    }
                }
            }
        }

        best_face
    }

    fn build_node(&mut self, start: usize, len: usize) -> usize {
        const LEAF_SIZE: usize = 8;

        let (min, max) = self.bounds_for_range(start, len);
        let node_id = self.nodes.len();
        self.nodes.push(Node {
            min,
            max,
            start,
            len,
            left: None,
            right: None,
        });

        if len > LEAF_SIZE {
            let axis = longest_axis(min, max);
            self.tri_indices[start..start + len].sort_unstable_by(|&a, &b| {
                centroid_axis(&self.positions, a, axis)
                    .partial_cmp(&centroid_axis(&self.positions, b, axis))
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
            let left_len = len / 2;
            let left = self.build_node(start, left_len);
            let right = self.build_node(start + left_len, len - left_len);
            self.nodes[node_id].left = Some(left);
            self.nodes[node_id].right = Some(right);
        }

        node_id
    }

    fn bounds_for_range(&self, start: usize, len: usize) -> ([f32; 3], [f32; 3]) {
        let mut min = [f32::INFINITY; 3];
        let mut max = [f32::NEG_INFINITY; 3];
        for &tri in &self.tri_indices[start..start + len] {
            let (a, b, c) = self.triangle(tri);
            for p in [a, b, c] {
                for k in 0..3 {
                    min[k] = min[k].min(p[k]);
                    max[k] = max[k].max(p[k]);
                }
            }
        }
        (min, max)
    }

    fn triangle(&self, tri: usize) -> ([f32; 3], [f32; 3], [f32; 3]) {
        let base = tri * 9;
        (
            [
                self.positions[base],
                self.positions[base + 1],
                self.positions[base + 2],
            ],
            [
                self.positions[base + 3],
                self.positions[base + 4],
                self.positions[base + 5],
            ],
            [
                self.positions[base + 6],
                self.positions[base + 7],
                self.positions[base + 8],
            ],
        )
    }
}

fn longest_axis(min: [f32; 3], max: [f32; 3]) -> usize {
    let extent = [max[0] - min[0], max[1] - min[1], max[2] - min[2]];
    if extent[0] >= extent[1] && extent[0] >= extent[2] {
        0
    } else if extent[1] >= extent[2] {
        1
    } else {
        2
    }
}

fn centroid_axis(positions: &[f32], tri: usize, axis: usize) -> f32 {
    let base = tri * 9 + axis;
    (positions[base] + positions[base + 3] + positions[base + 6]) / 3.0
}

fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn ray_box(origin: [f32; 3], dir: [f32; 3], min: [f32; 3], max: [f32; 3]) -> Option<f32> {
    let mut t_min = 0.0f32;
    let mut t_max = f32::INFINITY;
    for k in 0..3 {
        if dir[k].abs() < 1e-12 {
            if origin[k] < min[k] || origin[k] > max[k] {
                return None;
            }
        } else {
            let inv = 1.0 / dir[k];
            let mut a = (min[k] - origin[k]) * inv;
            let mut b = (max[k] - origin[k]) * inv;
            if a > b {
                std::mem::swap(&mut a, &mut b);
            }
            t_min = t_min.max(a);
            t_max = t_max.min(b);
            if t_min > t_max {
                return None;
            }
        }
    }
    Some(t_min)
}

/// Moller-Trumbore ray/triangle intersection; returns the ray parameter `t` of
/// the front hit, or `None`. Double-sided because CAD shells can carry mixed
/// winding.
fn ray_triangle(
    origin: [f32; 3],
    dir: [f32; 3],
    a: [f32; 3],
    b: [f32; 3],
    c: [f32; 3],
) -> Option<f32> {
    const EPS: f32 = 1e-7;
    let e1 = sub(b, a);
    let e2 = sub(c, a);
    let p = cross(dir, e2);
    let det = dot(e1, p);
    if det.abs() < EPS {
        return None;
    }
    let inv_det = 1.0 / det;
    let tvec = sub(origin, a);
    let u = dot(tvec, p) * inv_det;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = cross(tvec, e1);
    let v = dot(dir, q) * inv_det;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = dot(e2, q) * inv_det;
    if t > EPS {
        Some(t)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn two_triangles() -> GpuMesh {
        // Triangle face 5 at z = 1, triangle face 9 at z = 2, both around origin.
        GpuMesh {
            positions: vec![
                -1.0, -1.0, 1.0, 1.0, -1.0, 1.0, 0.0, 1.0, 1.0, // face 5
                -1.0, -1.0, 2.0, 1.0, -1.0, 2.0, 0.0, 1.0, 2.0, // face 9
            ],
            normals: vec![0.0; 18],
            indices: (0..6).collect(),
            face_ids: vec![5, 9],
        }
    }

    #[test]
    fn picks_nearest_face_along_ray() {
        let picker = Picker::from_gpu_mesh(&two_triangles());
        let hit = picker.pick([0.0, 0.0, -5.0], [0.0, 0.0, 1.0]);
        assert_eq!(hit, Some(5));
    }

    #[test]
    fn picks_far_face_when_near_is_behind() {
        let picker = Picker::from_gpu_mesh(&two_triangles());
        let hit = picker.pick([0.0, 0.0, 1.5], [0.0, 0.0, 1.0]);
        assert_eq!(hit, Some(9));
    }

    #[test]
    fn ray_missing_geometry_returns_none() {
        let picker = Picker::from_gpu_mesh(&two_triangles());
        let hit = picker.pick([5.0, 5.0, -5.0], [0.0, 0.0, 1.0]);
        assert_eq!(hit, None);
    }

    #[test]
    fn builds_bvh_nodes_for_picker() {
        let picker = Picker::from_gpu_mesh(&two_triangles());
        assert_eq!(picker.triangle_count(), 2);
        assert!(!picker.nodes.is_empty());
    }

    #[test]
    fn ray_box_rejects_miss() {
        assert_eq!(
            ray_box([3.0, 0.0, 0.0], [0.0, 1.0, 0.0], [-1.0; 3], [1.0; 3]),
            None
        );
    }
}
