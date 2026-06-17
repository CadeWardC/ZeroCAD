//! Binary STL export.
//!
//! STL is the lowest-common-denominator mesh interchange format — every slicer
//! and mesh tool reads it — so it is ZeroCAD's first "get my model out" path.
//! It is intentionally hand-rolled (no extra dependency) straight from the
//! tessellated [`MockMesh`] buffers the viewport already builds.
//!
//! The format is lossy by design: it carries only a triangle soup with
//! per-facet normals, no parametric history, units, or face ids. The editable
//! document remains the `.zcad` JSON; STL is for downstream consumption
//! (3D printing, rendering, mesh inspection).

use std::io::{self, Write};

use crate::MockMesh;

/// Compute a facet normal from a triangle's three corners (right-hand rule).
/// Returns a zero vector for a degenerate (zero-area) triangle, which is a
/// valid STL normal meaning "consumer should derive it".
fn facet_normal(a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> [f32; 3] {
    let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let n = [
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    ];
    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if len > 1e-12 {
        [n[0] / len, n[1] / len, n[2] / len]
    } else {
        [0.0, 0.0, 0.0]
    }
}

/// Pull a triangle's three positions out of a mesh's interleaved
/// `[x,y,z,nx,ny,nz]` vertex buffer, given an index triplet. Returns `None` if
/// any index is out of range (so malformed meshes are skipped, not panicked on).
fn triangle(mesh: &MockMesh, i0: u32, i1: u32, i2: u32) -> Option<[[f32; 3]; 3]> {
    let pos = |idx: u32| -> Option<[f32; 3]> {
        let base = (idx as usize).checked_mul(6)?;
        Some([
            *mesh.vertices.get(base)?,
            *mesh.vertices.get(base + 1)?,
            *mesh.vertices.get(base + 2)?,
        ])
    };
    Some([pos(i0)?, pos(i1)?, pos(i2)?])
}

/// Collect every triangle (as three world-space corners) across all meshes.
fn gather_triangles<'a>(meshes: impl IntoIterator<Item = &'a MockMesh>) -> Vec<[[f32; 3]; 3]> {
    let mut tris = Vec::new();
    for mesh in meshes {
        for tri in mesh.indices.chunks_exact(3) {
            if let Some(t) = triangle(mesh, tri[0], tri[1], tri[2]) {
                tris.push(t);
            }
        }
    }
    tris
}

/// Serialize one or more meshes into a single binary STL blob. The meshes are
/// merged into one triangle soup (STL has no concept of separate bodies).
pub fn meshes_to_binary_stl<'a>(meshes: impl IntoIterator<Item = &'a MockMesh>) -> Vec<u8> {
    let tris = gather_triangles(meshes);
    // 80-byte header + 4-byte count + 50 bytes per triangle.
    let mut out = Vec::with_capacity(84 + tris.len() * 50);
    out.extend_from_slice(&[0u8; 80]);
    out.extend_from_slice(&(tris.len() as u32).to_le_bytes());
    for [a, b, c] in &tris {
        let n = facet_normal(*a, *b, *c);
        for comp in n.iter().chain(a).chain(b).chain(c) {
            out.extend_from_slice(&comp.to_le_bytes());
        }
        // "Attribute byte count" — unused, always zero.
        out.extend_from_slice(&0u16.to_le_bytes());
    }
    out
}

/// Convenience: write the binary STL for `meshes` directly to a writer.
pub fn write_binary_stl<'a, W: Write>(
    meshes: impl IntoIterator<Item = &'a MockMesh>,
    w: &mut W,
) -> io::Result<()> {
    w.write_all(&meshes_to_binary_stl(meshes))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A two-triangle quad mesh (only positions matter for STL).
    fn quad() -> MockMesh {
        let mut m = MockMesh::empty();
        // 4 corners, interleaved with dummy normals (0,0,1).
        m.vertices = vec![
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, // v0
            2.0, 0.0, 0.0, 0.0, 0.0, 1.0, // v1
            2.0, 3.0, 0.0, 0.0, 0.0, 1.0, // v2
            0.0, 3.0, 0.0, 0.0, 0.0, 1.0, // v3
        ];
        m.indices = vec![0, 1, 2, 0, 2, 3];
        m
    }

    #[test]
    fn binary_stl_has_correct_size_and_count() {
        let stl = meshes_to_binary_stl(std::iter::once(&quad()));
        // 2 triangles → 80 + 4 + 2*50.
        assert_eq!(stl.len(), 84 + 2 * 50);
        let count = u32::from_le_bytes([stl[80], stl[81], stl[82], stl[83]]);
        assert_eq!(count, 2);
    }

    #[test]
    fn binary_stl_merges_multiple_meshes() {
        let meshes = [quad(), quad()];
        let stl = meshes_to_binary_stl(meshes.iter());
        let count = u32::from_le_bytes([stl[80], stl[81], stl[82], stl[83]]);
        assert_eq!(count, 4);
        assert_eq!(stl.len(), 84 + 4 * 50);
    }

    #[test]
    fn facet_normal_points_along_plus_z_for_ccw_quad() {
        // First triangle of the quad is counter-clockwise in the z=0 plane,
        // so its facet normal should point along +Z.
        let n = facet_normal([0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [2.0, 3.0, 0.0]);
        assert!(n[2] > 0.99, "expected +Z normal, got {n:?}");
    }

    #[test]
    fn out_of_range_indices_are_skipped_not_panicked() {
        let mut m = MockMesh::empty();
        m.vertices = vec![0.0; 6]; // a single vertex
        m.indices = vec![0, 1, 2]; // references v1/v2 that don't exist
        let stl = meshes_to_binary_stl(std::iter::once(&m));
        let count = u32::from_le_bytes([stl[80], stl[81], stl[82], stl[83]]);
        assert_eq!(count, 0, "degenerate triangle should be dropped");
    }
}
