use super::*;

#[test]
fn chamfer_large_radius_stays_in_bounds() {
    // A 6.48mm chamfer on a 10mm box edge (the screenshot's value). Big, but
    // valid — must bevel the corner, not produce a runaway wedge.
    let g = box_with_edge_mod(6.48, crate::sketch::CornerKind::Chamfer);
    let (bodies, _warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    let mesh = &bodies[0].1;
    let inside = mesh.vertices.chunks(6).all(|v| {
        (-0.5..=10.5).contains(&v[0])
            && (-0.5..=10.5).contains(&v[1])
            && (-0.5..=10.5).contains(&v[2])
    });
    assert!(
        inside,
        "large chamfer flew vertices outside the 10³ block (runaway wedge)"
    );
}

#[test]
fn chamfer_bevels_a_box_edge() {
    let g = box_with_edge_mod(2.0, crate::sketch::CornerKind::Chamfer);
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert_eq!(bodies.len(), 1, "chamfer must stay one body");
    assert!(
        warnings.is_empty(),
        "a clean box-edge chamfer should not warn, got {warnings:?}"
    );
    let mesh = &bodies[0].1;
    // The bevel introduces a face whose outward normal points at ~45° between
    // the two original faces (-Y and -Z): n ≈ (0, -0.707, -0.707).
    let has_bevel = mesh
        .vertices
        .chunks(6)
        .any(|v| v[3].abs() < 0.2 && (v[4] + 0.707).abs() < 0.15 && (v[5] + 0.707).abs() < 0.15);
    assert!(
        has_bevel,
        "chamfer should create a 45° bevel face (normal ~ (0,-0.707,-0.707))"
    );
    // The bottom (y=0) and front (z=0) faces still exist away from the edge…
    let min_y = mesh
        .vertices
        .chunks(6)
        .map(|v| v[1])
        .fold(f32::MAX, f32::min);
    let min_z = mesh
        .vertices
        .chunks(6)
        .map(|v| v[2])
        .fold(f32::MAX, f32::min);
    assert!(
        min_y < 0.01 && min_z < 0.01,
        "the bottom and front faces should survive the chamfer (min_y={min_y}, min_z={min_z})"
    );
    // …but the sharp corner is gone: no vertex sits on BOTH y=0 and z=0 (the
    // beveled edge has been cut back to the tangent lines).
    let sharp_corner = mesh
        .vertices
        .chunks(6)
        .any(|v| v[1].abs() < 0.01 && v[2].abs() < 0.01);
    assert!(
        !sharp_corner,
        "the original y=0,z=0 edge should be beveled away, not still sharp"
    );
}

#[test]
fn fillet_round_is_a_single_brep_face() {
    // The analytic-arc cutter must turn the round into ONE cylindrical B-rep
    // face — not the ~24 flat facets of the faceted fallback. Rounding one
    // edge of a 6-faced box leaves the 6 box faces (two trimmed) plus the one
    // fillet face: a handful, nowhere near 6 + 24. This guards against a
    // change silently regressing the fillet to the faceted path.
    let g = box_with_edge_mod(2.0, crate::sketch::CornerKind::Fillet);
    let (bodies, _) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    let mesh = &bodies[0].1;
    let distinct: std::collections::HashSet<u32> = mesh.face_ids.iter().copied().collect();
    assert!(
        distinct.len() <= 12,
        "fillet should be one cylindrical face (got {} B-rep faces — faceted fallback?)",
        distinct.len()
    );
}

#[test]
fn fillet_rounds_a_box_edge() {
    let g = box_with_edge_mod(2.0, crate::sketch::CornerKind::Fillet);
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert_eq!(bodies.len(), 1, "fillet must stay one body");
    assert!(
        warnings.is_empty(),
        "a clean box-edge fillet should not warn, got {warnings:?}"
    );
    let mesh = &bodies[0].1;
    // A faceted round adds several faces along the edge: the filleted body has
    // more triangles than a plain block.
    let plain = MockMesh::make_box(10.0, 10.0, 10.0);
    assert!(
        mesh.indices.len() / 3 > plain.indices.len() / 3,
        "a filleted edge should add facets (more triangles than the plain box)"
    );
    // Geometry sanity (a self-intersecting cutter used to fly vertices out
    // here): every vertex stays inside the 10³ block, within a small margin
    // for the cutter's overshoot/offset and tessellation noise.
    let inside = mesh.vertices.chunks(6).all(|v| {
        (-0.4..=10.4).contains(&v[0])
            && (-0.4..=10.4).contains(&v[1])
            && (-0.4..=10.4).contains(&v[2])
    });
    assert!(
        inside,
        "filleted body has vertices outside the original block"
    );
    // The sharp y=0,z=0 edge is rounded away: no vertex sits on both faces.
    let sharp = mesh
        .vertices
        .chunks(6)
        .any(|v| v[1].abs() < 0.01 && v[2].abs() < 0.01);
    assert!(!sharp, "the filleted edge should not still be sharp");
    // The round leaves intermediate facet normals between the two faces — at
    // least one vertex normal points partly along BOTH +(-y) and +(-z), i.e.
    // a curved-surface normal, not just the axis-aligned box faces.
    let has_round = mesh
        .vertices
        .chunks(6)
        .any(|v| v[4] < -0.15 && v[5] < -0.15 && v[3].abs() < 0.2);
    assert!(has_round, "expected curved fillet-surface normals");

    // Smooth-face look: the fillet's *lengthwise facet seams* (the lines
    // running along the edge between adjacent round facets) must be suppressed
    // by the crease filter, so the round reads as one face instead of a
    // striped one. Count wireframe edges that run parallel to the edge (+X)
    // and lie strictly inside the rounded corner (off both box faces) — there
    // should be essentially none.
    let nedges = mesh.edge_indices.len() / 2;
    let seam_count = (0..nedges)
        .filter(|&e| {
            let ia = mesh.edge_indices[e * 2] as usize * 3;
            let ib = mesh.edge_indices[e * 2 + 1] as usize * 3;
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
            let along_x = (b[0] - a[0]).abs() > 1.0
                && (b[1] - a[1]).abs() < 0.05
                && (b[2] - a[2]).abs() < 0.05;
            // Strictly inside the round (not on the y=0 or z=0 box faces).
            let interior = |p: &[f32; 3]| p[1] > 0.05 && p[1] < 1.95 && p[2] > 0.05 && p[2] < 1.95;
            along_x && interior(&a) && interior(&b)
        })
        .count();
    assert_eq!(
        seam_count, 0,
        "fillet facet seams should be hidden (got {seam_count} lengthwise seams)"
    );
    // …but the block's genuine sharp edges (90°) must survive: the un-touched
    // top face still has its four long edges, so plenty of wireframe remains.
    assert!(
        mesh.edge_indices.len() / 2 >= 8,
        "real box edges must still be drawn after crease filtering"
    );
}

#[test]
fn fillet_tangent_boundary_edges_are_drawn() {
    let g = box_with_edge_mod(2.0, crate::sketch::CornerKind::Fillet);
    let (bodies, _) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    let mesh = &bodies[0].1;

    let nedges = mesh.edge_indices.len() / 2;
    let tangent_len = |front: bool| -> f32 {
        (0..nedges)
            .filter_map(|e| {
                let ia = mesh.edge_indices[e * 2] as usize * 3;
                let ib = mesh.edge_indices[e * 2 + 1] as usize * 3;
                let g = |i: usize| {
                    [
                        mesh.edge_vertices[i],
                        mesh.edge_vertices[i + 1],
                        mesh.edge_vertices[i + 2],
                    ]
                };
                let (a, b) = (g(ia), g(ib));
                let along_x = (b[1] - a[1]).abs() < 0.05 && (b[2] - a[2]).abs() < 0.05;
                let (on0, off) = if front { (a[2], a[1]) } else { (a[1], a[2]) };
                let on_face = on0.abs() < 0.05 && (0.3..2.5).contains(&off);
                (along_x && on_face).then(|| (b[0] - a[0]).abs())
            })
            .sum()
    };

    let t_true = tangent_len(true);
    let t_false = tangent_len(false);
    // The round is ~10 long; require most of the tangent line to be present.
    assert!(
        t_true > 5.0,
        "the fillet's tangent edge on the front (z=0) face must be drawn (got {})",
        t_true
    );
    assert!(
        t_false > 5.0,
        "the fillet's tangent edge on the bottom (y=0) face must be drawn (got {})",
        t_false
    );
}

#[test]
fn fillet_keeps_adjacent_faces_flat() {
    // A fillet is tangent to its neighbour faces, so the round's first facet
    // sits just a few degrees off them. Plain crease-angle smoothing used to
    // drag those flat faces' edge normals toward the round, making the flat
    // faces render as a slope. The face-aware smoothing must anchor the flat
    // faces so a flat normal survives right up to the tangent line.
    let g = box_with_edge_mod(2.0, crate::sketch::CornerKind::Fillet);
    let (bodies, _) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    let mesh = &bodies[0].1;

    // The two faces adjacent to the rounded y=0,z=0 edge are z=0 (normal
    // (0,0,-1)) and y=0 (normal (0,-1,0)); each meets the round at a tangent
    // line ~`dist`=2 in from the old edge. At that tangent line the flat face
    // must still carry its exact axis-aligned normal — no tilt toward the
    // round. (Interior flat-face vertices were never co-located with the round
    // and so were never affected; the tangent line is where the bleed showed.)
    let on_z0_tangent = mesh.vertices.chunks(6).any(|v| {
        v[2].abs() < 0.02
            && (1.0..3.0).contains(&v[1])
            && v[3].abs() < 1.0e-3
            && v[4].abs() < 1.0e-3
            && v[5] < -0.999
    });
    assert!(
        on_z0_tangent,
        "the z=0 flat face must stay flat (normal (0,0,-1)) at the fillet tangent line"
    );
    let on_y0_tangent = mesh.vertices.chunks(6).any(|v| {
        v[1].abs() < 0.02
            && (1.0..3.0).contains(&v[2])
            && v[3].abs() < 1.0e-3
            && v[5].abs() < 1.0e-3
            && v[4] < -0.999
    });
    assert!(
        on_y0_tangent,
        "the y=0 flat face must stay flat (normal (0,-1,0)) at the fillet tangent line"
    );
}

#[test]
fn smoothing_keeps_a_plain_box_crisp() {
    // The crease-angle normal smoothing runs on every solid mesh; it must be
    // a no-op for a box (all faces flat, edges at 90° > the crease angle), so
    // every vertex normal stays axis-aligned — no accidental rounding.
    let m = MockMesh::make_box(10.0, 10.0, 10.0);
    for v in m.vertices.chunks(6) {
        let n = [v[3].abs(), v[4].abs(), v[5].abs()];
        let max = n.iter().cloned().fold(0.0f32, f32::max);
        let sum: f32 = n.iter().sum();
        assert!(
            (max - 1.0).abs() < 0.02 && (sum - 1.0).abs() < 0.03,
            "box vertex normal must stay axis-aligned after smoothing, got {n:?}"
        );
    }
}
