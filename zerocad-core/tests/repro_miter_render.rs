//! Regression for the "messed up render" at a mitered fillet corner.
//!
//! Two perpendicular top-edge fillets meet at a corner along an elliptical seam.
//! The two cylinder fillet faces are tangent along that seam, and near its
//! stub-vertex corner each face's tessellation used to collapse to a flat fan
//! over the shared seam vertices — producing two coincident, oppositely-wound
//! triangle layers (a non-manifold "double membrane") that z-fought on screen.
//!
//! This exercises the full GUI render path (`MockMesh::from_solid`) and asserts
//! the render mesh is manifold and crack-free across a range of box aspect
//! ratios / radii.

use openrcad::algo::fillet_edges;
use openrcad::foundation::Pnt;
use openrcad::primitives::make_box;
use openrcad::topo::Edge;
use std::collections::HashMap;
use zerocad_core::MockMesh;

type Key = (i64, i64, i64);

/// `(cracks, non_manifold)` edge counts of a render mesh, by welded position.
fn edge_health(mesh: &MockMesh) -> (usize, usize) {
    let q = |i: usize| -> Key {
        let b = i * 6;
        let f = |v: f32| (v as f64 * 1e4).round() as i64;
        (
            f(mesh.vertices[b]),
            f(mesh.vertices[b + 1]),
            f(mesh.vertices[b + 2]),
        )
    };
    let mut edges: HashMap<(Key, Key), u32> = HashMap::new();
    for t in mesh.indices.chunks_exact(3) {
        for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            let (ka, kb) = (q(t[a] as usize), q(t[b] as usize));
            let k = if ka <= kb { (ka, kb) } else { (kb, ka) };
            *edges.entry(k).or_insert(0) += 1;
        }
    }
    (
        edges.values().filter(|&&c| c == 1).count(),
        edges.values().filter(|&&c| c > 2).count(),
    )
}

#[test]
fn mitered_corner_render_mesh_is_manifold() {
    for (w, h, d, r) in [
        (40.0_f64, 30.0, 20.0, 4.0),
        (20.0, 20.0, 20.0, 5.0),
        (50.0, 10.0, 30.0, 2.0),
        (10.0, 10.0, 6.0, 1.26),
        (5.0, 5.0, 5.0, 1.0),
    ] {
        let cube = make_box(&Pnt::origin(), w, h, d);
        let front_top = Edge::between_points(Pnt::new(0.0, 0.0, d), Pnt::new(w, 0.0, d));
        let right_top = Edge::between_points(Pnt::new(w, 0.0, d), Pnt::new(w, h, d));
        let s = fillet_edges(&cube, &[front_top, right_top], r)
            .unwrap_or_else(|e| panic!("miter {w}x{h}x{d} r={r} should fillet: {e:?}"));

        let mesh = MockMesh::from_solid(&s);
        let (cracks, nonmanifold) = edge_health(&mesh);
        assert_eq!(
            nonmanifold, 0,
            "miter {w}x{h}x{d} r={r}: render mesh has a coincident double-membrane at the seam"
        );
        assert_eq!(
            cracks, 0,
            "miter {w}x{h}x{d} r={r}: render mesh has crack edges"
        );
    }
}
