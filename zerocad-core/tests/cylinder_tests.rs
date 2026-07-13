//! Extruded circles must become smooth cylinders (one wrapping wall, not a ring
//! of facets) and booleans involving them must never crash the app.
use std::collections::HashSet;
use zerocad_core::mock_kernel;
use zerocad_core::{
    CoordinateSystem, ExtrudeMode, FeatureNode, FeatureType, MockMesh, ParametricGraph,
    SketchCurves, Vec3,
};

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
    // Clean wireframe: just the two rim circles, with no fake wall struts.
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
fn fifteen_mm_by_two_mm_cylinder_has_compact_display_buffers() {
    let points = circle_pts(0.0, 0.0, 7.5, zerocad_core::CIRCLE_SEGS);
    let mesh = MockMesh::make_extruded_sketch(&points, &[], 2.0, &CoordinateSystem::XY);

    // Regression for the GUI status-bar report: the old generic cylinder grid
    // produced 1,110 triangles and therefore 3,330 triangle-local vertices.
    assert_eq!(mesh.indices.len() / 3, 192);
    assert_eq!(mesh.vertices.len() / 6, 576);
    assert_eq!(mesh.face_ids.len(), 192);
    assert!(mesh.mass_properties().is_some(), "mesh must remain closed");
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

#[test]
fn construction_line_split_circle_extrudes_as_one_clean_cylinder() {
    let mut curves = SketchCurves::new();
    curves.add_circle((0.0, 0.0), 2.0);
    curves.add_line((0.0, -3.0), (0.0, 3.0));
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "sketch_1".into(),
        name: "Split circle".into(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::XY,
            curves,
            shapes: vec![],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
            entity_ids: vec![],
            next_entity_id: 0,
            solver: None,
        },
    });
    graph.add_feature(FeatureNode {
        id: "extrude_2".into(),
        name: "Cylinder".into(),
        feature: FeatureType::Extrude {
            depth: 4.0,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            target: None,
            depth_expr: None,
        },
    });
    graph.add_dependency("sketch_1", "extrude_2");

    let bodies = graph
        .evaluate_bodies(&HashSet::new())
        .expect("split circle eval");
    assert_eq!(bodies.len(), 1);
    let mesh = &bodies[0].1;
    let generator_seams: Vec<_> = mesh
        .edge_refs
        .iter()
        .filter(|edge| {
            matches!(
                edge.curve,
                Some(zerocad_core::mock_kernel::EdgeCurveHint::Line)
            ) && (edge.p0[2] - edge.p1[2]).abs() > 3.9
        })
        .collect();
    assert!(
        generator_seams.is_empty(),
        "split circle still extruded with longitudinal seams: {generator_seams:#?}"
    );
}

#[test]
fn construction_line_split_circle_joins_as_one_clean_cylinder() {
    let mut graph = ParametricGraph::new();
    let mut base = SketchCurves::new();
    base.add_rectangle((0.0, -2.0), (8.0, 2.0));
    graph.add_feature(FeatureNode {
        id: "sketch_1".into(),
        name: "Base sketch".into(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::XY,
            curves: base,
            shapes: vec![],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
            entity_ids: vec![],
            next_entity_id: 0,
            solver: None,
        },
    });
    graph.add_feature(FeatureNode {
        id: "extrude_2".into(),
        name: "Base".into(),
        feature: FeatureType::Extrude {
            depth: 1.0,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            target: None,
            depth_expr: None,
        },
    });
    graph.add_dependency("sketch_1", "extrude_2");

    let mut split_circle = SketchCurves::new();
    split_circle.add_circle((0.0, 0.0), 1.0);
    split_circle.add_line((0.0, -2.0), (0.0, 2.0));
    graph.add_feature(FeatureNode {
        id: "sketch_3".into(),
        name: "Split boss circle".into(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::XY.with_origin(Vec3::new(0.0, 0.0, 1.0)),
            curves: split_circle,
            shapes: vec![],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
            entity_ids: vec![],
            next_entity_id: 0,
            solver: None,
        },
    });
    graph.add_feature(FeatureNode {
        id: "extrude_4".into(),
        name: "Joined cylinder".into(),
        feature: FeatureType::Extrude {
            depth: 2.0,
            region_indices: vec![],
            mode: ExtrudeMode::Join,
            target: Some("extrude_2".into()),
            depth_expr: None,
        },
    });
    graph.add_dependency("sketch_3", "extrude_4");
    graph.add_dependency("extrude_2", "extrude_4");

    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("split circle join eval");
    assert!(warnings.is_empty(), "join warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1);
    let mesh = &bodies[0].1;
    let generator_seams: Vec<_> = mesh
        .edge_refs
        .iter()
        .filter(|edge| {
            let r0 = edge.p0[0].hypot(edge.p0[1]);
            let r1 = edge.p1[0].hypot(edge.p1[1]);
            matches!(
                edge.curve,
                Some(zerocad_core::mock_kernel::EdgeCurveHint::Line)
            ) && (r0 - 1.0).abs() < 2.0e-3
                && (r1 - 1.0).abs() < 2.0e-3
                && (edge.p0[2] - edge.p1[2]).abs() > 1.9
        })
        .collect();
    assert!(
        generator_seams.is_empty(),
        "joined split circle still exposes cylinder seams: {generator_seams:#?}"
    );
}

#[test]
fn capsule_union_hides_tangent_generator_seams() {
    // The screenshot repro: a circle centred on the end of a same-height
    // rectangle. Their union is a capsule; the two vertical tangent generators
    // are construction seams, not display edges. The exposed semicircular cap
    // rims are real sharp boundaries and must remain.
    let mut curves = SketchCurves::new();
    curves.add_rectangle((0.0, 0.0), (10.0, 2.0));
    curves.add_circle((0.0, 1.0), 1.0);
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "sketch_1".into(),
        name: "Capsule sketch".into(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::XY,
            curves,
            shapes: vec![],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
            entity_ids: vec![],
            next_entity_id: 0,
            solver: None,
        },
    });
    graph.add_feature(FeatureNode {
        id: "extrude_2".into(),
        name: "Capsule".into(),
        feature: FeatureType::Extrude {
            depth: 3.0,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            target: None,
            depth_expr: None,
        },
    });
    graph.add_dependency("sketch_1", "extrude_2");
    let bodies = graph
        .evaluate_bodies(&HashSet::new())
        .expect("capsule eval");
    assert_eq!(bodies.len(), 1);
    let mesh = &bodies[0].1;

    let near = |a: f32, b: f32| (a - b).abs() < 2.0e-3;
    let is_unwanted = |p0: [f32; 3], p1: [f32; 3]| {
        let vertical_tangent = near(p0[0], 0.0)
            && near(p1[0], 0.0)
            && (near(p0[1], 0.0) || near(p0[1], 2.0))
            && near(p0[1], p1[1])
            && (p0[2] - p1[2]).abs() > 2.9;
        vertical_tangent
    };
    let seams: Vec<_> = mesh
        .edge_refs
        .iter()
        .filter(|edge| is_unwanted(edge.p0, edge.p1))
        .collect();
    assert!(seams.is_empty(), "capsule still exposes seams: {seams:#?}");
}
