use zerocad_core::{
    CoordinateSystem, ExtrudeMode, FeatureNode, FeatureType, MockMesh, ParametricGraph,
    SketchCurves, Unit, Vec3,
};

#[test]
fn test_unit_conversions() {
    // mm to base (mm is base)
    assert_eq!(Unit::Millimeter.to_base(100.0), 100.0);
    // inches to base
    assert_eq!(Unit::Inch.to_base(2.0), 50.8);
    // meters to base
    assert_eq!(Unit::Meter.to_base(1.5), 1500.0);

    // base back to units
    assert_eq!(Unit::Inch.from_base(25.4), 1.0);
    assert_eq!(Unit::Meter.from_base(1000.0), 1.0);
}

#[test]
fn test_coordinate_system_projection() {
    let xy_plane = CoordinateSystem::XY;

    // A point at (10, 20, 0) should project to local (10, 20)
    let p1 = Vec3::new(10.0, 20.0, 0.0);
    let (u, v) = xy_plane.project(p1);
    assert_eq!(u, 10.0);
    assert_eq!(v, 20.0);

    // A point at (5, -15, 100) should project to local (5, -15) on XY plane (normal Z is ignored)
    let p2 = Vec3::new(5.0, -15.0, 100.0);
    let (u2, v2) = xy_plane.project(p2);
    assert_eq!(u2, 5.0);
    assert_eq!(v2, -15.0);

    // Unprojecting should yield the 3D point on the plane (Z = 0)
    let p_unprojected = xy_plane.unproject(5.0, -15.0);
    assert_eq!(p_unprojected.x, 5.0);
    assert_eq!(p_unprojected.y, -15.0);
    assert_eq!(p_unprojected.z, 0.0);
}

#[test]
fn test_parametric_graph_box() {
    let mut pg = ParametricGraph::new();

    let box_node = FeatureNode {
        id: "test_box".to_string(),
        name: "Test Block".to_string(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 20.0,
            d: 30.0,
        },
    };

    pg.add_feature(box_node);

    let mesh_res = pg.evaluate();
    assert!(mesh_res.is_ok());

    let mesh = mesh_res.unwrap();
    // Six planar faces, each tessellated to two triangles → 36 indices.
    assert_eq!(mesh.indices.len(), 36);
    // OpenRCAD's flat-shaded `gpu_mesh` is unwelded: every triangle emits three
    // independent vertices carrying its face normal, so 12 triangles × 3 = 36.
    assert_eq!(mesh.vertices.len() / 6, 36);
    // Wireframe: 12 cube edges over 8 distinct corners. The display now derives
    // from the part solid (from_solid), whose wireframe stores per-segment
    // vertices — so weld by position instead of assuming the analytic
    // make_box buffer layout.
    assert_eq!(mesh.edge_indices.len() / 2, 12);
    let mut corners: std::collections::HashSet<(i64, i64, i64)> = std::collections::HashSet::new();
    for v in 0..mesh.edge_vertices.len() / 3 {
        let f = |x: f32| (x as f64 * 1.0e4).round() as i64;
        corners.insert((
            f(mesh.edge_vertices[v * 3]),
            f(mesh.edge_vertices[v * 3 + 1]),
            f(mesh.edge_vertices[v * 3 + 2]),
        ));
    }
    assert_eq!(corners.len(), 8, "a box wireframe has 8 distinct corners");

    // Sanity: every triangle index must be in range and triangles must be
    // non-degenerate (three distinct vertex indices per tri).
    let vert_count = (mesh.vertices.len() / 6) as u32;
    for tri in mesh.indices.chunks_exact(3) {
        assert!(tri.iter().all(|&i| i < vert_count));
        assert!(tri[0] != tri[1] && tri[1] != tri[2] && tri[0] != tri[2]);
    }
}

#[test]
fn test_box_mesh_is_watertight() {
    // A closed B-Rep solid, after tessellation, must produce a manifold mesh:
    // every undirected edge is shared by exactly two triangles.
    use std::collections::HashMap;

    let mut pg = ParametricGraph::new();
    pg.add_feature(FeatureNode {
        id: "wt_box".to_string(),
        name: "Watertight Box".to_string(),
        feature: FeatureType::Box {
            w: 8.0,
            h: 12.0,
            d: 5.0,
        },
    });
    let mesh = pg.evaluate().expect("box should evaluate");

    // Bucket triangle edges by their undirected vertex-coordinate signature
    // (positions, not indices, because each face has its own copies).
    let mut edge_counts: HashMap<((i64, i64, i64), (i64, i64, i64)), u32> = HashMap::new();
    let pos_of = |idx: u32| -> (i64, i64, i64) {
        let base = idx as usize * 6;
        // Quantize to 1e-4 mm to absorb f32 round-trip noise across faces.
        let q = |v: f32| (v as f64 * 10_000.0).round() as i64;
        (
            q(mesh.vertices[base]),
            q(mesh.vertices[base + 1]),
            q(mesh.vertices[base + 2]),
        )
    };
    for tri in mesh.indices.chunks_exact(3) {
        let pts = [pos_of(tri[0]), pos_of(tri[1]), pos_of(tri[2])];
        for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            let mut e = [pts[a], pts[b]];
            e.sort();
            *edge_counts.entry((e[0], e[1])).or_insert(0) += 1;
        }
    }
    for (edge, count) in &edge_counts {
        assert_eq!(
            *count, 2,
            "edge {:?} appears in {} triangles, expected 2",
            edge, count
        );
    }
}

#[test]
fn test_parametric_graph_cylinder() {
    let mut pg = ParametricGraph::new();

    let cyl_node = FeatureNode {
        id: "test_cyl".to_string(),
        name: "Test Cylinder".to_string(),
        feature: FeatureType::Cylinder { r: 5.0, h: 25.0 },
    };

    pg.add_feature(cyl_node);

    let mesh_res = pg.evaluate();
    assert!(mesh_res.is_ok());

    let mesh = mesh_res.unwrap();
    // Verify geometry was constructed
    assert!(mesh.vertices.len() > 0);
    assert!(mesh.indices.len() > 0);
}

#[test]
fn test_parametric_graph_sketch_extrude() {
    let mut pg = ParametricGraph::new();

    let mut curves = SketchCurves::new();
    curves.add_rectangle((0.0, 0.0), (20.0, 20.0));
    let sketch_node = FeatureNode {
        id: "sketch_1".to_string(),
        name: "Sketch 1".to_string(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::XZ,
            curves,
            shapes: vec![],
            corner_mods: vec![],
            on_face: false,
        },
    };
    let extrude_node = FeatureNode {
        id: "extrude_1".to_string(),
        name: "Extrude 1".to_string(),
        feature: FeatureType::Extrude {
            depth: 15.0,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
        },
    };

    pg.add_feature(sketch_node);
    pg.add_feature(extrude_node);
    pg.add_dependency("sketch_1", "extrude_1");

    let mesh_res = pg.evaluate();
    assert!(mesh_res.is_ok());

    let mesh = mesh_res.unwrap();
    assert!(mesh.vertices.len() > 0);
    assert!(mesh.indices.len() > 0);
}

#[test]
fn test_overlapping_rectangles_yield_three_extruded_regions() {
    // The headline Fusion-style behavior: two overlapping rectangles in a
    // sketch produce 3 detected regions, and an Extrude that selects all
    // of them yields 3× the triangle count of a single-region extrude.
    let mut pg = ParametricGraph::new();
    let mut curves = SketchCurves::new();
    curves.add_rectangle((0.0, 0.0), (10.0, 10.0));
    curves.add_rectangle((5.0, 0.0), (15.0, 10.0));
    pg.add_feature(FeatureNode {
        id: "s".to_string(),
        name: "S".to_string(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::XY,
            curves,
            shapes: vec![],
            corner_mods: vec![],
            on_face: false,
        },
    });
    pg.add_feature(FeatureNode {
        id: "e".to_string(),
        name: "E".to_string(),
        feature: FeatureType::Extrude {
            depth: 5.0,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
        },
    });
    pg.add_dependency("s", "e");
    let mesh_multi = pg.evaluate().unwrap();

    // Single-region baseline: just one rectangle, same depth.
    let mut pg2 = ParametricGraph::new();
    let mut single = SketchCurves::new();
    single.add_rectangle((0.0, 0.0), (10.0, 10.0));
    pg2.add_feature(FeatureNode {
        id: "s2".to_string(),
        name: "S2".to_string(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::XY,
            curves: single,
            shapes: vec![],
            corner_mods: vec![],
            on_face: false,
        },
    });
    pg2.add_feature(FeatureNode {
        id: "e2".to_string(),
        name: "E2".to_string(),
        feature: FeatureType::Extrude {
            depth: 5.0,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
        },
    });
    pg2.add_dependency("s2", "e2");
    let mesh_single = pg2.evaluate().unwrap();

    // Three sub-regions ≈ 3× triangles of one region. Allow some slack since
    // each region has the same per-region face count (top/bottom + sides).
    assert!(
        mesh_multi.indices.len() >= mesh_single.indices.len() * 3 - 12,
        "expected ~3× triangle count, got {} vs single {}",
        mesh_multi.indices.len(),
        mesh_single.indices.len()
    );
}

#[test]
fn test_extrude_face_with_hole_is_a_tube() {
    // A square with a square hole, extruded, should produce a tube: more
    // geometry than the same square with no hole (it has inner walls + a
    // capped top/bottom with a hole). This exercises the holed-face path.
    let outer = vec![(-10.0, -10.0), (10.0, -10.0), (10.0, 10.0), (-10.0, 10.0)];
    let hole = vec![(-3.0, -3.0), (3.0, -3.0), (3.0, 3.0), (-3.0, 3.0)];

    let with_hole = MockMesh::make_extruded_sketch(&outer, &[hole], 10.0, &CoordinateSystem::XY);
    let no_hole = MockMesh::make_extruded_sketch(&outer, &[], 10.0, &CoordinateSystem::XY);

    assert!(
        with_hole.vertices.len() > 0,
        "holed extrude produced no mesh"
    );
    assert!(
        with_hole.vertices.len() > no_hole.vertices.len(),
        "holed extrude ({}) should have more geometry than the solid one ({})",
        with_hole.vertices.len(),
        no_hole.vertices.len()
    );
}

/// Every body mesh must expose outward-facing triangle normals on every sketch
/// plane, so the renderer's back-face culling and edge hidden-line removal
/// agree. Regression for extrusions on the left-handed XZ/YZ frames, which
/// truck builds inside-out.
#[test]
fn test_extrusion_normals_outward_on_all_planes() {
    let pts = [(0.0f32, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];
    for cs in [
        CoordinateSystem::XY,
        CoordinateSystem::XZ,
        CoordinateSystem::YZ,
    ] {
        let plane = cs; // kept for the assert message below
        let m = MockMesh::make_extruded_sketch(&pts, &[], 10.0, &cs);
        // Mesh centroid.
        let vcount = m.vertices.len() / 6;
        let (mut cx, mut cy, mut cz) = (0.0f32, 0.0, 0.0);
        for v in 0..vcount {
            cx += m.vertices[v * 6];
            cy += m.vertices[v * 6 + 1];
            cz += m.vertices[v * 6 + 2];
        }
        let inv = 1.0 / vcount as f32;
        let (cx, cy, cz) = (cx * inv, cy * inv, cz * inv);
        for tri in m.indices.chunks_exact(3) {
            let i0 = tri[0] as usize * 6;
            let i1 = tri[1] as usize * 6;
            let i2 = tri[2] as usize * 6;
            let tcx = (m.vertices[i0] + m.vertices[i1] + m.vertices[i2]) / 3.0 - cx;
            let tcy = (m.vertices[i0 + 1] + m.vertices[i1 + 1] + m.vertices[i2 + 1]) / 3.0 - cy;
            let tcz = (m.vertices[i0 + 2] + m.vertices[i1 + 2] + m.vertices[i2 + 2]) / 3.0 - cz;
            let dot =
                m.vertices[i0 + 3] * tcx + m.vertices[i0 + 4] * tcy + m.vertices[i0 + 5] * tcz;
            assert!(dot > 0.0, "inward normal on plane {:?}: dot={}", plane, dot);
        }
    }
}

#[test]
fn test_face_ids_match_triangle_count() {
    // Every triangle in a body mesh must carry exactly one face id.
    let b = MockMesh::make_box(10.0, 20.0, 30.0);
    assert_eq!(b.face_ids.len(), b.indices.len() / 3, "box face_ids");
    // A box has 6 planar faces, so 6 distinct ids.
    let mut ids: Vec<u32> = b.face_ids.clone();
    ids.sort();
    ids.dedup();
    assert_eq!(
        ids.len(),
        6,
        "box should have 6 distinct face ids, got {:?}",
        ids
    );

    let e = MockMesh::make_extruded_sketch(
        &[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)],
        &[],
        5.0,
        &CoordinateSystem::XY,
    );
    assert_eq!(e.face_ids.len(), e.indices.len() / 3, "extrude face_ids");
}

#[test]
fn test_evaluate_bodies_one_mesh_per_solid_node() {
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "box_1".to_string(),
        name: "Box 1".to_string(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
    g.add_feature(FeatureNode {
        id: "box_2".to_string(),
        name: "Box 2".to_string(),
        feature: FeatureType::Box {
            w: 5.0,
            h: 5.0,
            d: 5.0,
        },
    });
    let bodies = g
        .evaluate_bodies(&std::collections::HashSet::new())
        .unwrap();
    assert_eq!(bodies.len(), 2, "expected one mesh per box node");
    let ids: Vec<&str> = bodies.iter().map(|(id, _)| id.as_str()).collect();
    assert!(ids.contains(&"box_1") && ids.contains(&"box_2"));
    for (_, m) in &bodies {
        assert_eq!(m.face_ids.len(), m.indices.len() / 3);
    }
}
