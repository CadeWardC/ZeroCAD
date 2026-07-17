//! Validated ASCII/binary STL mesh interchange.
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

use crate::mock_kernel::{MeshEdgeRef, MeshFaceRef};
use crate::MockMesh;

/// Validation facts retained for every imported STL mesh body. Geometry can be
/// displayed even when it is open, but callers can distinguish printable
/// closed meshes from repair candidates without pretending either is a B-Rep.
#[derive(Debug, Clone, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct StlValidationReport {
    pub input_triangles: usize,
    pub accepted_triangles: usize,
    pub degenerate_triangles: usize,
    pub boundary_edges: usize,
    pub non_manifold_edges: usize,
    pub inconsistent_winding_edges: usize,
    pub connected_components: usize,
}

/// A parsed STL body whose triangle buffers and validation facts travel
/// together. This is intentionally not convertible to a kernel solid: callers
/// must opt into a future explicit mesh-to-BRep operation instead of silently
/// crossing representation boundaries.
#[must_use]
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct ValidatedMeshBody {
    pub mesh: MockMesh,
    pub validation: StlValidationReport,
}

impl StlValidationReport {
    #[must_use]
    pub fn is_closed_manifold(&self) -> bool {
        self.accepted_triangles > 0
            && self.boundary_edges == 0
            && self.non_manifold_edges == 0
            && self.inconsistent_winding_edges == 0
    }

    #[must_use]
    pub fn diagnostics(&self) -> Vec<String> {
        let mut diagnostics = Vec::new();
        if self.degenerate_triangles > 0 {
            diagnostics.push(format!(
                "discarded {} degenerate triangle(s)",
                self.degenerate_triangles
            ));
        }
        if self.boundary_edges > 0 {
            diagnostics.push(format!(
                "mesh is open at {} boundary edge(s)",
                self.boundary_edges
            ));
        }
        if self.non_manifold_edges > 0 {
            diagnostics.push(format!(
                "mesh has {} non-manifold edge(s)",
                self.non_manifold_edges
            ));
        }
        if self.inconsistent_winding_edges > 0 {
            diagnostics.push(format!(
                "mesh has {} inconsistently wound shared edge(s)",
                self.inconsistent_winding_edges
            ));
        }
        if self.connected_components > 1 {
            diagnostics.push(format!(
                "mesh contains {} disconnected components",
                self.connected_components
            ));
        }
        diagnostics
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StlImportError {
    Empty,
    Malformed(String),
    NonFiniteVertex,
    NoUsableTriangles,
}

impl std::fmt::Display for StlImportError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Empty => f.write_str("STL data is empty"),
            Self::Malformed(reason) => write!(f, "malformed STL: {reason}"),
            Self::NonFiniteVertex => f.write_str("STL contains a non-finite vertex"),
            Self::NoUsableTriangles => f.write_str("STL contains no usable triangles"),
        }
    }
}

impl std::error::Error for StlImportError {}

/// Parse an ASCII or binary STL into a mesh body plus deterministic topology
/// diagnostics. Degenerate facets are discarded and reported; non-finite or
/// structurally malformed data is rejected.
pub fn read_stl_mesh(data: &[u8]) -> Result<ValidatedMeshBody, StlImportError> {
    if data.is_empty() {
        return Err(StlImportError::Empty);
    }
    let triangles = if looks_like_binary_stl(data) {
        parse_binary_stl(data)?
    } else {
        parse_ascii_stl(data)?
    };
    let (mesh, validation) = build_import_mesh(triangles)?;
    Ok(ValidatedMeshBody { mesh, validation })
}

fn looks_like_binary_stl(data: &[u8]) -> bool {
    if data.len() < 84 {
        return false;
    }
    let count = u32::from_le_bytes(data[80..84].try_into().expect("four-byte STL count"));
    84usize
        .checked_add(count as usize * 50)
        .is_some_and(|expected| expected == data.len())
}

fn parse_binary_stl(data: &[u8]) -> Result<Vec<[[f32; 3]; 3]>, StlImportError> {
    let count = u32::from_le_bytes(data[80..84].try_into().expect("four-byte STL count")) as usize;
    let expected = 84usize
        .checked_add(count.checked_mul(50).ok_or_else(|| {
            StlImportError::Malformed("triangle count overflows input length".into())
        })?)
        .ok_or_else(|| StlImportError::Malformed("input length overflow".into()))?;
    if data.len() != expected {
        return Err(StlImportError::Malformed(format!(
            "binary length is {}, expected {expected}",
            data.len()
        )));
    }
    let mut triangles = Vec::with_capacity(count);
    for facet in data[84..].chunks_exact(50) {
        let mut vertices = [[0.0; 3]; 3];
        for (vertex_index, vertex) in vertices.iter_mut().enumerate() {
            for (axis, coordinate) in vertex.iter_mut().enumerate() {
                let start = 12 + vertex_index * 12 + axis * 4;
                *coordinate = f32::from_le_bytes(
                    facet[start..start + 4]
                        .try_into()
                        .expect("binary STL coordinate"),
                );
            }
        }
        triangles.push(vertices);
    }
    Ok(triangles)
}

fn parse_ascii_stl(data: &[u8]) -> Result<Vec<[[f32; 3]; 3]>, StlImportError> {
    let text = std::str::from_utf8(data)
        .map_err(|_| StlImportError::Malformed("ASCII input is not UTF-8".into()))?;
    let mut vertices = Vec::new();
    for (line_number, line) in text.lines().enumerate() {
        let mut fields = line.split_whitespace();
        if !fields
            .next()
            .is_some_and(|word| word.eq_ignore_ascii_case("vertex"))
        {
            continue;
        }
        let mut vertex = [0.0_f32; 3];
        for coordinate in &mut vertex {
            let token = fields.next().ok_or_else(|| {
                StlImportError::Malformed(format!(
                    "vertex on line {} has fewer than three coordinates",
                    line_number + 1
                ))
            })?;
            *coordinate = token.parse::<f32>().map_err(|_| {
                StlImportError::Malformed(format!(
                    "invalid coordinate '{token}' on line {}",
                    line_number + 1
                ))
            })?;
        }
        vertices.push(vertex);
    }
    if vertices.is_empty() || vertices.len() % 3 != 0 {
        return Err(StlImportError::Malformed(
            "ASCII STL must contain complete three-vertex facets".into(),
        ));
    }
    Ok(vertices
        .chunks_exact(3)
        .map(|vertices| [vertices[0], vertices[1], vertices[2]])
        .collect())
}

type PointKey = (i64, i64, i64);
type EdgeKey = (PointKey, PointKey);

fn point_key(point: [f32; 3]) -> PointKey {
    let quantize = |value: f32| (value as f64 * 1.0e6).round() as i64;
    (quantize(point[0]), quantize(point[1]), quantize(point[2]))
}

fn edge_key(a: [f32; 3], b: [f32; 3]) -> (EdgeKey, bool) {
    let a_key = point_key(a);
    let b_key = point_key(b);
    if a_key <= b_key {
        ((a_key, b_key), true)
    } else {
        ((b_key, a_key), false)
    }
}

#[derive(Default)]
struct EdgeUse {
    a: [f32; 3],
    b: [f32; 3],
    normals: Vec<[f32; 3]>,
    directions: Vec<bool>,
    triangles: Vec<usize>,
}

fn build_import_mesh(
    input: Vec<[[f32; 3]; 3]>,
) -> Result<(MockMesh, StlValidationReport), StlImportError> {
    use std::collections::{HashMap, HashSet, VecDeque};

    let mut report = StlValidationReport {
        input_triangles: input.len(),
        ..StlValidationReport::default()
    };
    let mut mesh = MockMesh::empty();
    let mut edges: HashMap<EdgeKey, EdgeUse> = HashMap::new();
    for triangle in input {
        if triangle.iter().flatten().any(|value| !value.is_finite()) {
            return Err(StlImportError::NonFiniteVertex);
        }
        let normal = facet_normal(triangle[0], triangle[1], triangle[2]);
        if normal == [0.0, 0.0, 0.0]
            || triangle
                .iter()
                .map(|point| point_key(*point))
                .collect::<HashSet<_>>()
                .len()
                < 3
        {
            report.degenerate_triangles += 1;
            continue;
        }
        let triangle_index = report.accepted_triangles;
        let base = (mesh.vertices.len() / 6) as u32;
        for point in triangle {
            mesh.vertices.extend_from_slice(&[
                point[0], point[1], point[2], normal[0], normal[1], normal[2],
            ]);
        }
        mesh.indices.extend_from_slice(&[base, base + 1, base + 2]);
        mesh.face_ids.push(triangle_index as u32);
        let centroid = [
            (triangle[0][0] + triangle[1][0] + triangle[2][0]) / 3.0,
            (triangle[0][1] + triangle[1][1] + triangle[2][1]) / 3.0,
            (triangle[0][2] + triangle[1][2] + triangle[2][2]) / 3.0,
        ];
        mesh.face_refs.push(MeshFaceRef {
            face_id: triangle_index as u32,
            centroid,
            normal,
            topology: None,
        });
        for (a, b) in [
            (triangle[0], triangle[1]),
            (triangle[1], triangle[2]),
            (triangle[2], triangle[0]),
        ] {
            let (key, forward) = edge_key(a, b);
            let edge = edges.entry(key).or_insert_with(|| EdgeUse {
                a,
                b,
                ..EdgeUse::default()
            });
            edge.normals.push(normal);
            edge.directions.push(forward);
            edge.triangles.push(triangle_index);
        }
        report.accepted_triangles += 1;
    }
    if report.accepted_triangles == 0 {
        return Err(StlImportError::NoUsableTriangles);
    }

    let mut ordered_edges: Vec<_> = edges.into_iter().collect();
    ordered_edges.sort_by_key(|(key, _)| *key);
    let mut adjacency = vec![Vec::new(); report.accepted_triangles];
    for (group, (_, edge)) in ordered_edges.into_iter().enumerate() {
        match edge.triangles.as_slice() {
            [_] => report.boundary_edges += 1,
            [a, b] => {
                adjacency[*a].push(*b);
                adjacency[*b].push(*a);
                if edge.directions[0] == edge.directions[1] {
                    report.inconsistent_winding_edges += 1;
                }
            }
            uses => {
                report.non_manifold_edges += 1;
                for &a in uses {
                    for &b in uses {
                        if a != b {
                            adjacency[a].push(b);
                        }
                    }
                }
            }
        }
        let edge_base = (mesh.edge_vertices.len() / 3) as u32;
        mesh.edge_vertices.extend_from_slice(&edge.a);
        mesh.edge_vertices.extend_from_slice(&edge.b);
        mesh.edge_indices
            .extend_from_slice(&[edge_base, edge_base + 1]);
        mesh.edge_groups.push(group as u32);
        let n1 = edge.normals.first().copied().unwrap_or([0.0; 3]);
        let n2 = edge.normals.get(1).copied().unwrap_or(n1);
        mesh.edge_face_normals
            .extend_from_slice(&[n1[0], n1[1], n1[2], n2[0], n2[1], n2[2]]);
        mesh.edge_refs.push(MeshEdgeRef {
            group: group as u32,
            p0: edge.a,
            p1: edge.b,
            n1,
            n2,
            curve: None,
            topology: None,
        });
    }

    let mut visited = vec![false; report.accepted_triangles];
    for seed in 0..visited.len() {
        if visited[seed] {
            continue;
        }
        report.connected_components += 1;
        visited[seed] = true;
        let mut queue = VecDeque::from([seed]);
        while let Some(current) = queue.pop_front() {
            for &next in &adjacency[current] {
                if !visited[next] {
                    visited[next] = true;
                    queue.push_back(next);
                }
            }
        }
    }
    Ok((mesh, report))
}

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

/// Build a 3MF package from named meshes — one 3MF `<object>` per mesh, so
/// multi-body designs stay separate parts in the slicer (unlike STL's single
/// merged soup). Vertices are welded by quantized position, as 3MF indexes a
/// shared vertex table.
pub fn meshes_to_3mf<'a>(meshes: impl IntoIterator<Item = (&'a str, &'a MockMesh)>) -> Vec<u8> {
    use openrcad::foundation::Pnt;
    use openrcad::mesh::TriangleMesh;

    let quant = |v: f32| (v as f64 * 1.0e5).round() as i64;
    let mut owned: Vec<(String, TriangleMesh)> = Vec::new();
    for (name, mesh) in meshes {
        let mut tri = TriangleMesh::new();
        let mut index_of: std::collections::HashMap<(i64, i64, i64), u32> =
            std::collections::HashMap::new();
        let mut remap: Vec<u32> = Vec::with_capacity(mesh.vertices.len() / 6);
        for v in mesh.vertices.chunks(6) {
            let key = (quant(v[0]), quant(v[1]), quant(v[2]));
            let id = *index_of.entry(key).or_insert_with(|| {
                tri.vertices
                    .push(Pnt::new(v[0] as f64, v[1] as f64, v[2] as f64));
                (tri.vertices.len() - 1) as u32
            });
            remap.push(id);
        }
        for t in mesh.indices.chunks(3) {
            let (Some(&a), Some(&b), Some(&c)) = (t.first(), t.get(1), t.get(2)) else {
                continue;
            };
            let map = |i: u32| remap.get(i as usize).copied();
            if let (Some(a), Some(b), Some(c)) = (map(a), map(b), map(c)) {
                // A weld can collapse a sliver triangle to a degenerate one.
                if a != b && b != c && a != c {
                    tri.triangles.push([a, b, c]);
                }
            }
        }
        if !tri.triangles.is_empty() {
            owned.push((name.to_string(), tri));
        }
    }
    let refs: Vec<(String, &TriangleMesh)> = owned.iter().map(|(n, m)| (n.clone(), m)).collect();
    openrcad::exchange::to_3mf_bytes(&refs)
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

    #[test]
    fn ascii_open_mesh_reports_boundary_edges() {
        let ascii = b"solid open\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid open\n";
        let imported = read_stl_mesh(ascii).unwrap();
        let (mesh, report) = (imported.mesh, imported.validation);
        assert_eq!(mesh.indices.len(), 3);
        assert_eq!(report.boundary_edges, 3);
        assert!(!report.is_closed_manifold());
        assert!(report
            .diagnostics()
            .iter()
            .any(|diagnostic| diagnostic.contains("open")));
    }

    #[test]
    fn binary_export_round_trips_through_import() {
        let source = quad();
        let bytes = meshes_to_binary_stl(std::iter::once(&source));
        let imported = read_stl_mesh(&bytes).unwrap();
        let (restored, report) = (imported.mesh, imported.validation);
        assert_eq!(report.input_triangles, 2);
        assert_eq!(report.accepted_triangles, 2);
        assert_eq!(restored.indices.len(), source.indices.len());
        assert_eq!(report.boundary_edges, 4);
    }

    #[test]
    fn degenerate_and_inconsistent_facets_are_diagnosed() {
        let ascii = b"solid bad\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 -1 0\nendloop\nendfacet\nfacet normal 0 0 0\nouter loop\nvertex 2 2 2\nvertex 2 2 2\nvertex 2 2 2\nendloop\nendfacet\nendsolid bad\n";
        let report = read_stl_mesh(ascii).unwrap().validation;
        assert_eq!(report.degenerate_triangles, 1);
        assert_eq!(report.inconsistent_winding_edges, 1);
        assert!(!report.is_closed_manifold());
    }
}
