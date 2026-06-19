//! CPU extraction of topological edge segments for the wireframe overlay.
//!
//! The flat-shaded [`GpuMesh`] is an unwelded triangle soup, but each triangle
//! carries its source B-Rep face id. A mesh edge is a *real* model edge exactly
//! when the two triangles sharing it come from **different** faces (a face
//! boundary), or when only one triangle touches it (an open-shell boundary).
//! Edges interior to a single face — the tessellation's own diagonals — are
//! shared by two same-face triangles and skipped, so a flat box draws its 12
//! crisp edges and nothing across the faces.
//!
//! This is pure, deterministic, GPU-free, and unit-tested.

use std::collections::HashMap;

use openrcad_mesh::GpuMesh;

/// Quantization grid for welding coincident triangle-soup vertices (~2.4e-4 of
/// a unit). Triangles meeting at a shared model edge emit bit-identical vertex
/// positions, so even a coarse grid welds them without merging distinct corners.
const WELD: f32 = 4096.0;

#[inline]
fn quant(v: f32) -> i64 {
    (v * WELD).round() as i64
}

/// Build flat line-list vertices (`xyz` per endpoint, 6 floats per segment) for
/// every topological edge of `mesh`.
pub fn feature_edge_lines(mesh: &GpuMesh) -> Vec<f32> {
    let vcount = mesh.positions.len() / 3;
    if vcount < 3 {
        return Vec::new();
    }

    // Weld vertices to canonical indices by quantized position.
    let mut canon: HashMap<(i64, i64, i64), u32> = HashMap::new();
    let mut canon_pos: Vec<[f32; 3]> = Vec::new();
    let mut vid = vec![0u32; vcount];
    for (i, slot) in vid.iter_mut().enumerate() {
        let b = i * 3;
        let p = [
            mesh.positions[b],
            mesh.positions[b + 1],
            mesh.positions[b + 2],
        ];
        let k = (quant(p[0]), quant(p[1]), quant(p[2]));
        *slot = *canon.entry(k).or_insert_with(|| {
            canon_pos.push(p);
            (canon_pos.len() - 1) as u32
        });
    }

    // For each canonical edge track incidence count, the first adjacent face id,
    // and whether a second *different* face id was seen.
    struct EdgeRec {
        count: u32,
        first_face: u32,
        multi_face: bool,
    }
    let tri_count = mesh.positions.len() / 9;
    let mut edges: HashMap<(u32, u32), EdgeRec> = HashMap::new();
    for t in 0..tri_count {
        let tri = [vid[3 * t], vid[3 * t + 1], vid[3 * t + 2]];
        let face = mesh.face_ids.get(t).copied().unwrap_or(0);
        for e in 0..3 {
            let (a, b) = (tri[e], tri[(e + 1) % 3]);
            if a == b {
                continue; // degenerate triangle edge
            }
            let key = if a < b { (a, b) } else { (b, a) };
            let rec = edges.entry(key).or_insert(EdgeRec {
                count: 0,
                first_face: face,
                multi_face: false,
            });
            rec.count += 1;
            if rec.first_face != face {
                rec.multi_face = true;
            }
        }
    }

    let mut out = Vec::new();
    for (&(a, b), rec) in &edges {
        if rec.count == 1 || rec.multi_face {
            out.extend_from_slice(&canon_pos[a as usize]);
            out.extend_from_slice(&canon_pos[b as usize]);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn single_triangle_has_three_boundary_edges() {
        let mesh = GpuMesh {
            positions: vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            normals: vec![0.0; 9],
            indices: vec![0, 1, 2],
            face_ids: vec![0],
        };
        let lines = feature_edge_lines(&mesh);
        // 3 edges × 2 endpoints × 3 floats.
        assert_eq!(lines.len(), 3 * 2 * 3);
    }

    #[test]
    fn shared_diagonal_of_one_face_is_skipped() {
        // Two triangles forming a unit quad, both face 0, sharing the (1,0)->(0,1)
        // diagonal. The diagonal is interior to face 0 and must NOT be drawn; the
        // 4 outer edges are boundaries and must be.
        let mesh = GpuMesh {
            positions: vec![
                0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, // tri A
                1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0, // tri B
            ],
            normals: vec![0.0; 18],
            indices: (0..6).collect(),
            face_ids: vec![0, 0],
        };
        let lines = feature_edge_lines(&mesh);
        // 4 boundary segments, diagonal excluded.
        assert_eq!(lines.len(), 4 * 2 * 3);
    }

    #[test]
    fn shared_edge_between_different_faces_is_drawn() {
        // Same geometry, but the two triangles belong to different faces — now the
        // shared diagonal is a face boundary and is drawn: 4 boundary + 1 shared.
        let mesh = GpuMesh {
            positions: vec![
                0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0,
                1.0, 0.0,
            ],
            normals: vec![0.0; 18],
            indices: (0..6).collect(),
            face_ids: vec![0, 1],
        };
        let lines = feature_edge_lines(&mesh);
        assert_eq!(lines.len(), 5 * 2 * 3);
    }
}
