
/// Cross-drill a fully threaded 12 mm stud through its middle.
#[test]
fn cross_drill_through_short_threaded_rod() {
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "rod_1", 6.0, 12.0);
    let face = cyl_face_near(&g, "rod_1", [0.0, 6.0, 0.0]);
    add_thread(
        &mut g, "thread_2", "rod_1", face, false, 1.0, 0.15, None, false, 1, true,
    );
    let mut c = zerocad_core::SketchCurves::new();
    c.add_circle((0.0, 6.0), 2.0);
    add_sketch_cs(&mut g, "sk_3", xy_plane_at(-8.0), c);
    g.add_feature(FeatureNode {
        id: "drill_4".to_string(),
        name: "drill_4".to_string(),
        feature: FeatureType::Extrude {
            depth: 16.0,
            region_indices: vec![],
            mode: ExtrudeMode::Cut,
            target: Some("rod_1".into()),
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_3", "drill_4");
    let (bodies, warnings) = eval_pair(&g);
    assert!(warnings.is_empty(), "{warnings:?}");
    let v = total_volume(&bodies);
    let expected = std::f64::consts::PI * 36.0 * 12.0 - std::f64::consts::PI * 4.0 * 12.0;
    assert!(
        (v - expected).abs() / expected < 0.08,
        "cross-drilled short stud: {v} vs {expected}"
    );
}

/// Thread the top half of a 12 mm rod, then cut exactly at the thread's end.
#[test]
fn cut_exactly_at_a_short_thread_end() {
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "rod_1", 6.0, 12.0);
    let face = cyl_face_near(&g, "rod_1", [0.0, 6.0, 0.0]);
    add_thread(
        &mut g,
        "thread_2",
        "rod_1",
        face,
        false,
        1.0,
        0.15,
        Some(6.0),
        false,
        1,
        true,
    );
    let mut c = zerocad_core::SketchCurves::new();
    c.add_rectangle((-10.0, 6.0), (10.0, 20.0));
    add_sketch_cs(&mut g, "sk_3", xy_plane_at(-10.0), c);
    g.add_feature(FeatureNode {
        id: "cut_4".to_string(),
        name: "cut_4".to_string(),
        feature: FeatureType::Extrude {
            depth: 20.0,
            region_indices: vec![],
            mode: ExtrudeMode::Cut,
            target: Some("rod_1".into()),
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_3", "cut_4");
    let (bodies, warnings) = eval_pair(&g);
    assert!(warnings.is_empty(), "{warnings:?}");
    let v = total_volume(&bodies);
    let half_rod = std::f64::consts::PI * 36.0 * 6.0;
    assert!(
        (v - half_rod).abs() / half_rod < 0.06,
        "short threaded end cut off: {v} vs {half_rod}"
    );
}
