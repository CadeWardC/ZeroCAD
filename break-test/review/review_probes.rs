// Review-only observations copied from the original reproductions on 2026-09-11.
// The inherited characterization assertions are historical, not desired behavior.
// See README.md and docs/break-test-review-and-implementation-plan.md for verdicts.
#[path = "../tests/common/mod.rs"]
mod common;
use common::*;
use std::collections::HashSet;
use zerocad_core::parametric::{
    FaceRef, FeaturePatternComputeMode, FeaturePatternExtentPolicy, FeaturePatternKind,
    TopologyFaceRef,
};
use zerocad_core::{
    AxisBase, CoordinateSystem, ExtrudeMode, FeatureNode, FeatureType, MockMesh, ParametricGraph,
    Vec3,
};
fn observe(g: &ParametricGraph, bodies: &[(String, MockMesh)], warnings: &[String]) {
    println!("WARNINGS {warnings:?}");
    for (id, mesh) in bodies {
        let mut lo = [f32::INFINITY; 3];
        let mut hi = [f32::NEG_INFINITY; 3];
        for v in mesh.vertices.chunks_exact(6) {
            for a in 0..3 {
                lo[a] = lo[a].min(v[a]);
                hi[a] = hi[a].max(v[a]);
            }
        }
        println!(
            "BODY {id} bounds={lo:?}..{hi:?} vertices={} volume={:?}",
            mesh.vertices.len() / 6,
            mesh.mass_properties().map(|m| m.volume)
        );
        println!("INSPECTION {id} {:?}", g.inspect_body(id));
    }
}

fn add_feature(g: &mut ParametricGraph, id: &str, feature: FeatureType, deps: &[&str]) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature,
    });
    for d in deps {
        g.add_dependency(d, id);
    }
}

fn eval_ok(g: &ParametricGraph) -> (Vec<(String, MockMesh)>, Vec<String>) {
    g.evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail")
}

fn capture_face(g: &ParametricGraph, body_id: &str, want_normal: [f32; 3]) -> FaceRef {
    let (bodies, _) = eval_ok(g);
    let mesh = &bodies
        .iter()
        .find(|(id, _)| id == body_id)
        .unwrap_or_else(|| panic!("body {body_id} missing"))
        .1;
    let f = mesh
        .face_refs
        .iter()
        .find(|f| {
            let dot = f.normal[0] * want_normal[0]
                + f.normal[1] * want_normal[1]
                + f.normal[2] * want_normal[2];
            dot > 0.99
        })
        .unwrap_or_else(|| {
            panic!(
                "no face with normal {want_normal:?} among {} faces",
                mesh.face_refs.len()
            )
        });
    FaceRef {
        centroid: f.centroid,
        normal: f.normal,
        topology: f.topology.as_ref().map(|t| TopologyFaceRef {
            body_id: t.body_id.clone(),
            component_id: t.component_id.clone(),
            topology_version: t.topology_version,
            face_id: t.face_id.clone(),
            surface_kind: t.surface_kind.clone(),
            producer_feature_id: t.producer_feature_id.clone(),
            source_entity_id: t.source_entity_id.clone(),
        }),
    }
}

fn eval_pair(g: &ParametricGraph) -> (Vec<(String, zerocad_core::MockMesh)>, Vec<String>) {
    g.evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("must not hard-fail")
}

fn cyl_face_near(g: &ParametricGraph, body: &str, near: [f32; 3]) -> FaceRef {
    let (bodies, _) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("capture eval must not fail");
    let mesh = &bodies.iter().find(|(id, _)| id == body).unwrap().1;
    let is_cyl = |f: &zerocad_core::mock_kernel::MeshFaceRef| {
        f.topology
            .as_ref()
            .and_then(|t| t.surface_kind.as_ref().map(|k| k.as_str()))
            .is_some_and(|k| k.eq_ignore_ascii_case("cylinder"))
            || f.normal[0].abs() + f.normal[1].abs() + f.normal[2].abs() < 0.1
    };
    let f = mesh
        .face_refs
        .iter()
        .filter(|f| is_cyl(f))
        .min_by_key(|f| {
            let d = [
                (f.centroid[0] - near[0]).abs(),
                (f.centroid[1] - near[1]).abs(),
                (f.centroid[2] - near[2]).abs(),
            ];
            ((d[0] + d[1] + d[2]) * 100.0) as u32
        })
        .expect("a cylindrical face must be selectable");
    FaceRef {
        centroid: f.centroid,
        normal: f.normal,
        topology: f.topology.as_ref().map(|t| TopologyFaceRef {
            body_id: t.body_id.clone(),
            component_id: t.component_id.clone(),
            topology_version: t.topology_version,
            face_id: t.face_id.clone(),
            surface_kind: t.surface_kind.clone(),
            producer_feature_id: t.producer_feature_id.clone(),
            source_entity_id: t.source_entity_id.clone(),
        }),
    }
}

#[test]
fn review_characterization_face_move_silently_drops_tangential_component() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    let top = capture_face(&g, "box_1", [0.0, 0.0, 1.0]);
    add_feature(
        &mut g,
        "move_2",
        FeatureType::FaceMove {
            target: "box_1".to_string(),
            face: top,
            translation: [5.0, 0.0, 5.0],
        },
        &["box_1"],
    );
    let (bodies, warnings) = eval_ok(&g);
    observe(&g, &bodies, &warnings);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    assert!(
        warnings.is_empty(),
        "currently the tangential part is dropped SILENTLY: {warnings:?}"
    );
    assert!(
        (v - 20.0 * 20.0 * 15.0).abs() / 3000.0 < 0.02,
        "pinning current behavior: only the normal component applied: {v}"
    );
}

#[test]
fn review_characterization_circular_feature_pattern_rolls_back_on_cylinder() {
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "cyl_1", 20.0, 10.0);
    add_hole(
        &mut g,
        "hole_2",
        "cyl_1",
        [13.0, 10.0, 0.0],
        [0.0, -1.0, 0.0],
        3.0,
        None,
        zerocad_core::HoleKind::Simple,
    );
    add_feature(
        &mut g,
        "pattern_3",
        FeatureType::FeaturePattern {
            target: "cyl_1".to_string(),
            source_feature: "hole_2".to_string(),
            kind: FeaturePatternKind::Circular {
                axis: AxisBase::Y,
                total_angle_deg: 360.0,
                total_angle_expr: None,
                count: 6,
            },
            compute_mode: FeaturePatternComputeMode::Identical,
            extent_policy: FeaturePatternExtentPolicy::ThroughAllLocalTarget,
        },
        &["cyl_1"],
    );
    let (bodies, warnings) = eval_ok(&g);
    observe(&g, &bodies, &warnings);
    assert_meshes_finite(&bodies);
    assert!(
        warnings.iter().any(|w| w.contains("instance 1 failed")),
        "circular ring currently rolls back: {warnings:?}"
    );
    let v = total_volume(&bodies);
    let only_source = std::f64::consts::PI * 400.0 * 10.0 - std::f64::consts::PI * 2.25 * 10.0;
    assert!(
        (v - only_source).abs() / only_source < 0.03,
        "pinning current behavior: only the source hole applied: {v}"
    );
}

#[test]
fn review_characterization_flush_coaxial_plug_join_does_not_fuse() {
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "rod_1", 10.0, 20.0);
    let mut hole = zerocad_core::SketchCurves::new();
    hole.add_circle((0.0, 0.0), 5.0);
    add_sketch_cs(
        &mut g,
        "sk_2",
        CoordinateSystem::new(
            Vec3::new(0.0, -5.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        ),
        hole,
    );
    g.add_feature(FeatureNode {
        id: "drill_3".to_string(),
        name: "drill_3".to_string(),
        feature: FeatureType::Extrude {
            depth: 30.0,
            region_indices: vec![],
            mode: ExtrudeMode::Cut,
            target: Some("rod_1".into()),
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_2", "drill_3");
    let mut plug = zerocad_core::SketchCurves::new();
    plug.add_circle((0.0, 0.0), 5.0);
    add_sketch_cs(
        &mut g,
        "sk_4",
        CoordinateSystem::new(
            Vec3::new(0.0, -5.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        ),
        plug,
    );
    g.add_feature(FeatureNode {
        id: "plug_5".to_string(),
        name: "plug_5".to_string(),
        feature: FeatureType::Extrude {
            depth: 30.0,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            target: None,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_4", "plug_5");
    add_feature(
        &mut g,
        "join_6",
        FeatureType::BodyJoin {
            sources: vec!["rod_1".to_string(), "plug_5".to_string()],
        },
        &["rod_1", "plug_5"],
    );
    let (bodies, warnings) = eval_pair(&g);
    observe(&g, &bodies, &warnings);
    assert_meshes_finite(&bodies);
    assert!(
        warnings.is_empty(),
        "the failed fuse is currently SILENT: {warnings:?}"
    );
    let v = total_volume(&bodies);
    let tube = std::f64::consts::PI * (100.0 - 25.0) * 20.0;
    let plug_vol = std::f64::consts::PI * 25.0 * 30.0;
    assert!(
        (v - (tube + plug_vol)).abs() / (tube + plug_vol) < 0.03,
        "pinning current behavior: tube + plug unfused ({v}); fused union would be {}",
        std::f64::consts::PI * 100.0 * 20.0
    );
}

#[test]
fn review_thread_on_ghost_target_and_far_face_is_graceful() {
    // Ghost target body.
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "rod_1", 6.0, 30.0);
    let face = cyl_face_near(&g, "rod_1", [0.0, 15.0, 0.0]);
    add_feature(
        &mut g,
        "thread_2",
        FeatureType::Thread {
            target: "ghost".into(),
            face: face.clone(),
            internal: false,
            pitch: 1.0,
            depth: 0.15,
            angle_deg: 60.0,
            right_handed: true,
            starts: 1,
            length: None,
            flip: false,
            designation: "M12".into(),
            standard: None,
        },
        &["rod_1"],
    );
    let _ = g.evaluate_bodies_with_warnings(&HashSet::new());

    // Far-away face on a real body: cosmetic fallback must warn and preserve.
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "rod_1", 6.0, 30.0);
    add_feature(
        &mut g,
        "thread_2",
        FeatureType::Thread {
            target: "rod_1".into(),
            face: FaceRef {
                centroid: [500.0, 500.0, 500.0],
                normal: [0.0, 1.0, 0.0],
                topology: None,
            },
            internal: false,
            pitch: 1.0,
            depth: 0.15,
            angle_deg: 60.0,
            right_handed: true,
            starts: 1,
            length: None,
            flip: false,
            designation: "M12".into(),
            standard: None,
        },
        &["rod_1"],
    );
    let (bodies, warnings) = eval_pair(&g);
    observe(&g, &bodies, &warnings);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    let rod = std::f64::consts::PI * 36.0 * 30.0;
    assert!(
        (v - rod).abs() / rod < 0.06,
        "far-face thread preserves rod: {v} vs {rod}"
    );
    // E19 (pinned): the cosmetic fallback for an unmatched face is currently
    // SILENT — no warning is emitted even though nothing was threaded. When
    // fixed, flip this to require a warning.
    assert!(
        warnings.is_empty(),
        "pinning current behavior: silent no-op on unmatched face ({warnings:?})"
    );
}

#[test]
fn review_characterization_body_cut_rejects_tool_based_below_target() {
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "cyl_1", 10.0, 20.0);
    add_sketch_cs(
        &mut g,
        "sk_box",
        CoordinateSystem::new(Vec3::new(-15.0, 0.0, -15.0), Vec3::X, Vec3::Y),
        rect_sketch((0.0, 0.0), (30.0, 30.0)),
    );
    g.add_feature(FeatureNode {
        id: "tool_2".to_string(),
        name: "tool_2".to_string(),
        feature: FeatureType::Extrude {
            depth: 10.0,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            target: None,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_box", "tool_2");
    g.add_feature(FeatureNode {
        id: "cut_3".to_string(),
        name: "cut_3".to_string(),
        feature: FeatureType::BodyCut {
            target: "cyl_1".to_string(),
            tool: "tool_2".to_string(),
            keep_tool: false,
        },
    });
    g.add_dependency("cyl_1", "cut_3");
    g.add_dependency("tool_2", "cut_3");
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail");
    observe(&g, &bodies, &warnings);
    assert_meshes_finite(&bodies);
    assert!(
        warnings.iter().any(|w| w.contains("must overlap")),
        "below-target BodyCut currently claims disjointness: {warnings:?}"
    );
    let v = total_volume(&bodies);
    // Both bodies survive the rejected cut: rod + tool (tessellated).
    let both = std::f64::consts::PI * 100.0 * 20.0 + 9000.0;
    assert!(
        (v - both).abs() / both < 0.02,
        "pinning current behavior: both bodies unchanged ({v}); the cut should remove ~1024 mm³"
    );
}
