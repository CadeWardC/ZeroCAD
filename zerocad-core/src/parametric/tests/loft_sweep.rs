//! Loft and Sweep as parametric features.

use super::*;
use crate::geometry::Vec3;

fn volume(mesh: &MockMesh) -> f64 {
    mesh.mass_properties().expect("closed mesh").volume
}

fn shifted_xy(z: f32) -> CoordinateSystem {
    CoordinateSystem::XY.with_origin(Vec3::new(0.0, 0.0, z))
}

fn holed_rectangle(
    outer_min: (f32, f32),
    outer_max: (f32, f32),
    hole_min: (f32, f32),
    hole_max: (f32, f32),
) -> (SketchCurves, usize) {
    let mut curves = SketchCurves::new();
    curves.add_rectangle(outer_min, outer_max);
    curves.add_rectangle(hole_min, hole_max);
    let region_index = crate::sketch::detect_regions(&curves)
        .iter()
        .position(|region| !region.holes.is_empty())
        .expect("nested rectangles must expose the material region");
    (curves, region_index)
}

// ---- Loft ----------------------------------------------------------------

#[test]
fn loft_two_squares_is_a_frustum() {
    let mut g = ParametricGraph::new();
    add_sketch_cs(
        &mut g,
        "sketch_1",
        shifted_xy(0.0),
        rect_sketch((0.0, 0.0), (4.0, 4.0)),
    );
    add_sketch_cs(
        &mut g,
        "sketch_2",
        shifted_xy(10.0),
        rect_sketch((1.0, 1.0), (3.0, 3.0)),
    );
    g.add_feature(FeatureNode {
        id: "loft_3".to_string(),
        name: "Loft".to_string(),
        feature: FeatureType::Loft {
            sections: vec![("sketch_1".to_string(), 0), ("sketch_2".to_string(), 0)],
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    g.add_dependency("sketch_1", "loft_3");
    g.add_dependency("sketch_2", "loft_3");
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1);
    // Frustum 4×4 → 2×2 over height 10: h/3·(A0 + A1 + √(A0·A1)).
    let exact = 10.0 / 3.0 * (16.0 + 4.0 + 8.0);
    let v = volume(&bodies[0].1);
    assert!(
        (v - exact).abs() / exact < 0.02,
        "loft volume {v} vs {exact}"
    );
}

#[test]
fn loft_preserves_section_holes() {
    let mut graph = ParametricGraph::new();
    let (bottom, bottom_region) =
        holed_rectangle((-5.0, -5.0), (5.0, 5.0), (-2.0, -2.0), (2.0, 2.0));
    let (top, top_region) = holed_rectangle((-5.0, -5.0), (5.0, 5.0), (-2.0, -2.0), (2.0, 2.0));
    add_sketch_cs(&mut graph, "bottom", shifted_xy(0.0), bottom);
    add_sketch_cs(&mut graph, "top", shifted_xy(6.0), top);
    graph.add_feature(FeatureNode {
        id: "holed_loft".into(),
        name: "Holed loft".into(),
        feature: FeatureType::Loft {
            sections: vec![("bottom".into(), bottom_region), ("top".into(), top_region)],
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    graph.add_dependency("bottom", "holed_loft");
    graph.add_dependency("top", "holed_loft");

    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    let actual = volume(&bodies[0].1);
    let expected = (100.0 - 16.0) * 6.0;
    assert!(
        (actual - expected).abs() / expected < 0.02,
        "holed loft volume {actual} vs {expected}"
    );
}

#[test]
fn loft_needs_two_sections() {
    let mut g = ParametricGraph::new();
    add_sketch_cs(
        &mut g,
        "sketch_1",
        shifted_xy(0.0),
        rect_sketch((0.0, 0.0), (4.0, 4.0)),
    );
    g.add_feature(FeatureNode {
        id: "loft_2".to_string(),
        name: "Loft".to_string(),
        feature: FeatureType::Loft {
            sections: vec![("sketch_1".to_string(), 0)],
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    g.add_dependency("sketch_1", "loft_2");
    let latest = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(1));
    let output = g
        .evaluate_request(
            &std::collections::HashSet::new(),
            EvaluationQuality::Final,
            &EvaluationCancellation::new(1, latest),
        )
        .unwrap();
    assert!(output.bodies.is_empty());
    assert!(
        output.diagnostics.iter().any(|diagnostic| {
            diagnostic.feature_id == "loft_2"
                && diagnostic.code.as_str() == DiagnosticCode::FEATURE_UNRESOLVED
        }),
        "diagnostics: {:?}",
        output.diagnostics
    );
}

// ---- Sweep ---------------------------------------------------------------

/// A path sketch holding one straight open segment.
fn straight_path_sketch(
    g: &mut ParametricGraph,
    id: &str,
    cs: CoordinateSystem,
    a: (f32, f32),
    b: (f32, f32),
) {
    let mut curves = SketchCurves::new();
    curves.add_line(a, b);
    add_sketch_cs(g, id, cs, curves);
}

#[test]
fn sweep_square_along_straight_path_is_a_prism() {
    let mut g = ParametricGraph::new();
    // Profile: 2×2 square centered, on XY (normal +Z).
    add_sketch_cs(
        &mut g,
        "profile",
        CoordinateSystem::XY,
        rect_sketch((-1.0, -1.0), (1.0, 1.0)),
    );
    // Path: a line along world +Z, drawn on the XZ plane (u=X, v=Z).
    straight_path_sketch(
        &mut g,
        "path",
        CoordinateSystem::XZ,
        (0.0, 0.0),
        (0.0, 10.0),
    );
    g.add_feature(FeatureNode {
        id: "sweep_1".to_string(),
        name: "Sweep".to_string(),
        feature: FeatureType::Sweep {
            profile_sketch: "profile".to_string(),
            profile_region: 0,
            path_sketch: "path".to_string(),
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    g.add_dependency("profile", "sweep_1");
    g.add_dependency("path", "sweep_1");
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1);
    let v = volume(&bodies[0].1);
    // 2×2 profile × length 10 = 40.
    assert!((v - 40.0).abs() / 40.0 < 0.02, "sweep volume {v} vs 40");
}

#[test]
fn sweep_preserves_profile_hole() {
    let mut graph = ParametricGraph::new();
    let (profile, profile_region) =
        holed_rectangle((-2.0, -2.0), (2.0, 2.0), (-1.0, -1.0), (1.0, 1.0));
    add_sketch_cs(&mut graph, "profile", CoordinateSystem::XY, profile);
    straight_path_sketch(
        &mut graph,
        "path",
        CoordinateSystem::XZ,
        (0.0, 0.0),
        (0.0, 10.0),
    );
    graph.add_feature(FeatureNode {
        id: "holed_sweep".into(),
        name: "Holed sweep".into(),
        feature: FeatureType::Sweep {
            profile_sketch: "profile".into(),
            profile_region,
            path_sketch: "path".into(),
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    graph.add_dependency("profile", "holed_sweep");
    graph.add_dependency("path", "holed_sweep");

    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    let actual = volume(&bodies[0].1);
    let expected = (16.0 - 4.0) * 10.0;
    assert!(
        (actual - expected).abs() / expected < 0.02,
        "holed sweep volume {actual} vs {expected}"
    );
}

#[test]
fn sweep_along_bent_path_is_watertight() {
    let mut g = ParametricGraph::new();
    add_sketch_cs(
        &mut g,
        "profile",
        CoordinateSystem::XY,
        rect_sketch((-0.5, -0.5), (0.5, 0.5)),
    );
    // L-path on the XZ plane: up 8 then across 6.
    let mut curves = SketchCurves::new();
    curves.add_line((0.0, 0.0), (0.0, 8.0));
    curves.add_line((0.0, 8.0), (6.0, 8.0));
    add_sketch_cs(&mut g, "path", CoordinateSystem::XZ, curves);
    g.add_feature(FeatureNode {
        id: "sweep_1".to_string(),
        name: "Sweep".to_string(),
        feature: FeatureType::Sweep {
            profile_sketch: "profile".to_string(),
            profile_region: 0,
            path_sketch: "path".to_string(),
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    g.add_dependency("profile", "sweep_1");
    g.add_dependency("path", "sweep_1");
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1);
    let mp = bodies[0].1.mass_properties().expect("closed swept mesh");
    // 1×1 profile over a ~14mm path ≈ 14mm³, minus a little at the miter.
    assert!(
        mp.volume > 10.0 && mp.volume < 15.0,
        "swept volume {}",
        mp.volume
    );
}

#[test]
fn sweep_rejects_branching_path() {
    let mut g = ParametricGraph::new();
    add_sketch_cs(
        &mut g,
        "profile",
        CoordinateSystem::XY,
        rect_sketch((-0.5, -0.5), (0.5, 0.5)),
    );
    // A "T" — three segments meeting at a point: not a single open chain.
    let mut curves = SketchCurves::new();
    curves.add_line((0.0, 0.0), (0.0, 5.0));
    curves.add_line((0.0, 5.0), (5.0, 5.0));
    curves.add_line((0.0, 5.0), (-5.0, 5.0));
    add_sketch_cs(&mut g, "path", CoordinateSystem::XZ, curves);
    g.add_feature(FeatureNode {
        id: "sweep_1".to_string(),
        name: "Sweep".to_string(),
        feature: FeatureType::Sweep {
            profile_sketch: "profile".to_string(),
            profile_region: 0,
            path_sketch: "path".to_string(),
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    g.add_dependency("profile", "sweep_1");
    g.add_dependency("path", "sweep_1");
    let latest = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(1));
    let output = g
        .evaluate_request(
            &std::collections::HashSet::new(),
            EvaluationQuality::Final,
            &EvaluationCancellation::new(1, latest),
        )
        .unwrap();
    assert!(output.bodies.is_empty());
    assert!(
        output.diagnostics.iter().any(|diagnostic| {
            diagnostic.feature_id == "sweep_1"
                && diagnostic.code.as_str() == DiagnosticCode::FEATURE_UNRESOLVED
        }),
        "diagnostics: {:?}",
        output.diagnostics
    );
}

fn assert_skinning_contract(graph: &ParametricGraph, feature_id: &str) {
    let hidden = std::collections::HashSet::new();
    let cold = graph.evaluate_bodies_with_warnings(&hidden).unwrap();
    let warm = graph.evaluate_bodies_with_warnings(&hidden).unwrap();
    assert!(cold.1.is_empty() && warm.1.is_empty());
    assert_eq!(cold.0.len(), warm.0.len());
    for ((cold_id, cold_mesh), (warm_id, warm_mesh)) in cold.0.iter().zip(&warm.0) {
        assert_eq!(cold_id, warm_id);
        assert_eq!(cold_mesh.indices, warm_mesh.indices);
        assert_eq!(cold_mesh.face_ids, warm_mesh.face_ids);
    }
    let mesh = warm
        .0
        .iter()
        .find(|(id, _)| id == feature_id)
        .map(|(_, mesh)| mesh)
        .expect("family output body");
    assert!(!mesh.face_refs.is_empty());
    assert!(mesh.face_refs.iter().all(|face| {
        face.topology
            .as_ref()
            .and_then(|topology| topology.face_id.as_deref())
            .is_some_and(|name| super::super::topo_name::TopoName::parse(name).is_durable())
    }));

    let cancelled_generation = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(2));
    assert!(matches!(
        graph.evaluate_request(
            &hidden,
            EvaluationQuality::Interactive,
            &EvaluationCancellation::new(1, cancelled_generation),
        ),
        Err(EvaluationError::Cancelled)
    ));

    let loaded: ParametricGraph =
        serde_json::from_str(&serde_json::to_string(&graph.clone_document()).unwrap()).unwrap();
    let rebuilt = loaded.evaluate_bodies_with_warnings(&hidden).unwrap();
    assert!(rebuilt.1.is_empty(), "load warnings: {:?}", rebuilt.1);
    assert!(rebuilt.0.iter().any(|(id, _)| id == feature_id));
}

#[test]
fn loft_and_sweep_candidate_contract_gates() {
    let mut loft = ParametricGraph::new();
    add_sketch_cs(
        &mut loft,
        "loft_profile_1",
        shifted_xy(0.0),
        rect_sketch((0.0, 0.0), (4.0, 4.0)),
    );
    add_sketch_cs(
        &mut loft,
        "loft_profile_2",
        shifted_xy(5.0),
        rect_sketch((1.0, 1.0), (3.0, 3.0)),
    );
    loft.add_feature(FeatureNode {
        id: "loft_contract".into(),
        name: "Loft contract".into(),
        feature: FeatureType::Loft {
            sections: vec![("loft_profile_1".into(), 0), ("loft_profile_2".into(), 0)],
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    loft.add_dependency("loft_profile_1", "loft_contract");
    loft.add_dependency("loft_profile_2", "loft_contract");
    assert_skinning_contract(&loft, "loft_contract");

    let mut sweep = ParametricGraph::new();
    add_sketch_cs(
        &mut sweep,
        "sweep_profile",
        CoordinateSystem::XY,
        rect_sketch((-0.5, -0.5), (0.5, 0.5)),
    );
    straight_path_sketch(
        &mut sweep,
        "sweep_path",
        CoordinateSystem::XZ,
        (0.0, 0.0),
        (0.0, 4.0),
    );
    sweep.add_feature(FeatureNode {
        id: "sweep_contract".into(),
        name: "Sweep contract".into(),
        feature: FeatureType::Sweep {
            profile_sketch: "sweep_profile".into(),
            profile_region: 0,
            path_sketch: "sweep_path".into(),
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    sweep.add_dependency("sweep_profile", "sweep_contract");
    sweep.add_dependency("sweep_path", "sweep_contract");
    assert_skinning_contract(&sweep, "sweep_contract");
}
