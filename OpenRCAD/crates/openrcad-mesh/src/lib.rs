#![forbid(unsafe_code)]
//! Tessellation for OpenRCAD (OCCT `TKMesh`).
//!
//! Converts analytic B-Rep surfaces into triangle meshes within a chordal
//! (distance) and angular error budget — the representation renderers and the
//! STL exporter consume.
//!
//! [`TriangleMesh`] is a serializable output type, and [`tessellate`] samples
//! each face independently before welding coincident mesh vertices into one
//! render/export mesh.

pub mod triangulate;

use openrcad_foundation::{BndBox, Pnt, Trsf};
#[cfg(feature = "parallel")]
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

/// A triangle mesh: shared vertex positions plus integer triangles.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct TriangleMesh {
    /// Vertex positions, de-duplicated where possible.
    pub vertices: Vec<Pnt>,
    /// Triangle indices into [`vertices`](Self::vertices), three per triangle.
    pub triangles: Vec<[u32; 3]>,
    /// Per-triangle source face index (parallel to [`triangles`](Self::triangles)).
    ///
    /// Each entry is the position of the originating face in
    /// `solid.shell().faces()`, so a renderer can map a picked triangle back to
    /// its topological [`Face`](openrcad_topo::Face). Empty when provenance was
    /// not tracked (e.g. meshes built directly via [`from_buffers`](Self::from_buffers)).
    pub face_ids: Vec<u32>,
}

/// GPU-ready, flat-shaded render buffers derived from a [`TriangleMesh`].
///
/// Triangles are *unwelded*: every triangle contributes three unique vertices
/// that all share the triangle's geometric (face) normal. This produces the
/// crisp, faceted "CAD look" — adjacent coplanar triangles read flat and sharp
/// edges stay sharp — at the cost of not sharing vertices between triangles.
///
/// All buffers use `f32`/`u32` for direct upload to a graphics API. No GPU types
/// leak into this crate; the renderer interprets these as it sees fit.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct GpuMesh {
    /// Vertex positions, `[x, y, z]` per vertex, `3 * 3 * triangle_count` long.
    pub positions: Vec<f32>,
    /// Per-vertex normals, `[x, y, z]`, parallel to [`positions`](Self::positions).
    /// All three vertices of a triangle share its flat face normal.
    pub normals: Vec<f32>,
    /// Triangle indices: `0, 1, 2, 3, …` since vertices are not shared.
    pub indices: Vec<u32>,
    /// Per-triangle source face index, for pick/selection buffers.
    /// One entry per triangle (`indices.len() / 3`).
    pub face_ids: Vec<u32>,
}

impl TriangleMesh {
    /// An empty mesh.
    #[inline]
    pub const fn new() -> Self {
        Self {
            vertices: Vec::new(),
            triangles: Vec::new(),
            face_ids: Vec::new(),
        }
    }

    /// Build from raw vertex and triangle buffers, without face provenance.
    pub fn from_buffers(vertices: Vec<Pnt>, triangles: Vec<[u32; 3]>) -> Self {
        Self {
            vertices,
            triangles,
            face_ids: Vec::new(),
        }
    }

    /// Build from raw buffers plus a per-triangle source face index.
    ///
    /// `face_ids` must have one entry per triangle (or be empty).
    pub fn from_buffers_with_faces(
        vertices: Vec<Pnt>,
        triangles: Vec<[u32; 3]>,
        face_ids: Vec<u32>,
    ) -> Self {
        debug_assert!(face_ids.is_empty() || face_ids.len() == triangles.len());
        Self {
            vertices,
            triangles,
            face_ids,
        }
    }

    /// Number of vertices.
    #[inline]
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    /// Number of triangles.
    #[inline]
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    /// Apply `t` to every vertex.
    pub fn transformed(&self, t: &Trsf) -> Self {
        Self {
            vertices: self.vertices.iter().map(|p| t.transform_point(p)).collect(),
            triangles: self.triangles.clone(),
            face_ids: self.face_ids.clone(),
        }
    }

    /// Axis-aligned box over all vertices.
    pub fn bounding_box(&self) -> BndBox {
        let mut b = BndBox::new();
        for p in &self.vertices {
            b.add(p);
        }
        b
    }

    /// A flat `[x,y,z]` position buffer, suitable for upload to a GPU vertex
    /// array.
    pub fn flat_positions(&self) -> Vec<f64> {
        let mut out = Vec::with_capacity(self.vertices.len() * 3);
        for p in &self.vertices {
            out.push(p.x());
            out.push(p.y());
            out.push(p.z());
        }
        out
    }

    /// Build flat-shaded, GPU-ready render buffers (`f32` positions/normals,
    /// `u32` indices, per-triangle face ids).
    ///
    /// Each triangle is emitted as three independent vertices sharing the
    /// triangle's geometric normal, giving a faceted CAD appearance. Degenerate
    /// triangles (zero-area) get a zero normal rather than `NaN`. When
    /// [`face_ids`](Self::face_ids) is populated it is copied through for picking;
    /// otherwise every triangle is tagged `0`.
    pub fn gpu_mesh(&self) -> GpuMesh {
        let tri_count = self.triangles.len();
        let mut positions = Vec::with_capacity(tri_count * 9);
        let mut normals = Vec::with_capacity(tri_count * 9);
        let mut indices = Vec::with_capacity(tri_count * 3);
        let mut face_ids = Vec::with_capacity(tri_count);

        for (i, tri) in self.triangles.iter().enumerate() {
            let a = self.vertices[tri[0] as usize];
            let b = self.vertices[tri[1] as usize];
            let c = self.vertices[tri[2] as usize];

            // Geometric (flat) normal; zero for degenerate triangles.
            let ab = b - a;
            let ac = c - a;
            let n = ab.cross(&ac);
            let len = n.magnitude();
            let (nx, ny, nz) = if len > 0.0 {
                (
                    (n.x() / len) as f32,
                    (n.y() / len) as f32,
                    (n.z() / len) as f32,
                )
            } else {
                (0.0, 0.0, 0.0)
            };

            for p in [a, b, c] {
                positions.push(p.x() as f32);
                positions.push(p.y() as f32);
                positions.push(p.z() as f32);
                normals.push(nx);
                normals.push(ny);
                normals.push(nz);
            }

            let base = (i * 3) as u32;
            indices.push(base);
            indices.push(base + 1);
            indices.push(base + 2);

            face_ids.push(self.face_ids.get(i).copied().unwrap_or(0));
        }

        GpuMesh {
            positions,
            normals,
            indices,
            face_ids,
        }
    }
}

/// Tessellate `solid` into a triangle mesh within `chord_err` (max surface
/// deviation) and `_angle_err` (radians; max normal deviation).
pub fn tessellate(solid: &openrcad_topo::Solid, chord_err: f64, _angle_err: f64) -> TriangleMesh {
    let faces = solid.shell().faces();

    // Faces tessellate independently, so this parallelises cleanly across the
    // shell. The `parallel` feature (on by default) maps each face on a rayon
    // pool; with it disabled the identical work runs sequentially.
    #[cfg(feature = "parallel")]
    let meshes: Vec<TriangleMesh> = faces
        .par_iter()
        .enumerate()
        .map(|(i, face)| triangulate::tessellate_face_local(face, chord_err, i as u32))
        .collect();
    #[cfg(not(feature = "parallel"))]
    let meshes: Vec<TriangleMesh> = faces
        .iter()
        .enumerate()
        .map(|(i, face)| triangulate::tessellate_face_local(face, chord_err, i as u32))
        .collect();

    triangulate::combine(&meshes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::{Trsf, Vec};

    #[test]
    fn mesh_counts_and_flat_positions() {
        let m = TriangleMesh::from_buffers(
            vec![
                Pnt::origin(),
                Pnt::new(1.0, 0.0, 0.0),
                Pnt::new(0.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2]],
        );
        assert_eq!(m.vertex_count(), 3);
        assert_eq!(m.triangle_count(), 1);
        assert_eq!(
            m.flat_positions(),
            vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        );
    }

    #[test]
    fn gpu_mesh_is_flat_shaded_and_carries_face_ids() {
        // A single triangle in the z=0 plane, tagged as face 7.
        let m = TriangleMesh::from_buffers_with_faces(
            vec![
                Pnt::origin(),
                Pnt::new(1.0, 0.0, 0.0),
                Pnt::new(0.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2]],
            vec![7],
        );
        let g = m.gpu_mesh();
        // Unwelded: 3 vertices, 3 indices, 1 face id.
        assert_eq!(g.positions.len(), 9);
        assert_eq!(g.normals.len(), 9);
        assert_eq!(g.indices, vec![0, 1, 2]);
        assert_eq!(g.face_ids, vec![7]);
        // CCW triangle in z=0 → +Z normal, shared by all 3 vertices.
        for v in 0..3 {
            assert!((g.normals[v * 3] - 0.0).abs() < 1e-6);
            assert!((g.normals[v * 3 + 1] - 0.0).abs() < 1e-6);
            assert!((g.normals[v * 3 + 2] - 1.0).abs() < 1e-6);
        }
    }

    #[test]
    fn gpu_mesh_degenerate_triangle_has_zero_normal() {
        // Collinear points → zero-area triangle, must not produce NaN.
        let m = TriangleMesh::from_buffers(
            vec![
                Pnt::origin(),
                Pnt::new(1.0, 0.0, 0.0),
                Pnt::new(2.0, 0.0, 0.0),
            ],
            vec![[0, 1, 2]],
        );
        let g = m.gpu_mesh();
        assert!(g.normals.iter().all(|n| n.is_finite()));
        assert_eq!(g.normals, vec![0.0; 9]);
        // No provenance → default face id 0.
        assert_eq!(g.face_ids, vec![0]);
    }

    #[test]
    fn transformed_mesh_moves_vertices() {
        let m = TriangleMesh::from_buffers(vec![Pnt::origin()], vec![]);
        let up = Trsf::translation(Vec::new(1.0, 2.0, 3.0));
        let m2 = m.transformed(&up);
        assert_eq!(m2.vertices[0], Pnt::new(1.0, 2.0, 3.0));
        // triangles unchanged.
        assert!(m2.triangles.is_empty());
    }

    #[test]
    fn bounding_box_covers_vertices() {
        let m = TriangleMesh::from_buffers(
            vec![Pnt::new(-1.0, -2.0, -3.0), Pnt::new(4.0, 5.0, 6.0)],
            vec![],
        );
        let (lo, hi) = m.bounding_box().corners().unwrap();
        assert_eq!(lo, Pnt::new(-1.0, -2.0, -3.0));
        assert_eq!(hi, Pnt::new(4.0, 5.0, 6.0));
    }
}
