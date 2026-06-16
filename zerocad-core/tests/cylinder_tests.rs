//! Extruded circles must become smooth cylinders (one wrapping wall, not a ring
//! of facets) and booleans involving them must never crash the app.
use std::collections::HashSet;
use zerocad_core::mock_kernel;
use zerocad_core::{CoordinateSystem, MockMesh};

fn circle_pts(cx: f32, cy: f32, r: f32, n: usize) -> Vec<(f32, f32)> {
    (0..n)
        .map(|i| {
            let t = i as f32 / n as f32 * std::f32::consts::TAU;
            (cx + r * t.cos(), cy + r * t.sin())
        })
        .collect()
}

#[test]
fn extruded_circle_is_a_smooth_cylinder() {
    let pts = circle_pts(5.0, 5.0, 3.0, 48);
    let m = MockMesh::make_extruded_sketch(&pts, &[], 6.0, &CoordinateSystem::XY);
    // truck builds the wall from 4 quarter-arc patches: 4 sides + 2 caps = 6,
    // a world away from the ~50 flat faces a 48-gon prism would have.
    let faces: HashSet<u32> = m.face_ids.iter().copied().collect();
    assert!(
        faces.len() <= 6,
        "cylinder should be ~one smooth wall, got {} faces",
        faces.len()
    );
    // Clean wireframe: two rim circles + a handful of silhouette struts, not a
    // strut per facet.
    assert!(
        m.edge_indices.len() / 2 <= 110,
        "too many wireframe edges: {}",
        m.edge_indices.len() / 2
    );
    // No stray/degenerate wireframe vertex (the "fan" artifact).
    assert!(m
        .edge_vertices
        .iter()
        .all(|c| c.is_finite() && c.abs() < 1.0e4));
}

#[test]
fn a_real_polygon_stays_faceted() {
    // A square must NOT be mistaken for a circle.
    let sq = vec![(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)];
    let m = MockMesh::make_extruded_sketch(&sq, &[], 3.0, &CoordinateSystem::XY);
    let faces: HashSet<u32> = m.face_ids.iter().copied().collect();
    assert_eq!(
        faces.len(),
        6,
        "a box has 6 flat faces, got {}",
        faces.len()
    );
}

#[test]
fn booleans_with_a_cylinder_never_panic() {
    let pts = circle_pts(4.0, 4.0, 2.5, 48);
    let cyl = mock_kernel::extruded_region_solid(&pts, &[], 6.0, &CoordinateSystem::XY).unwrap();
    let bx = mock_kernel::box_solid(7.0, 7.0, 4.0);
    // These used to panic inside truck; now they return Some/None, never unwind.
    let _ = mock_kernel::union(&bx, &cyl);
    let _ = mock_kernel::difference(&bx, &cyl);
}
