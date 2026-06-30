use super::*;

#[test]
fn oversized_circular_bite_runout_fails_safely_and_leaves_body() {
    let unmodified = circular_bite_graph(None)
        .evaluate_bodies(&std::collections::HashSet::new())
        .unwrap();
    let g = circular_bite_cutoff_edge_graph_with_dist(crate::sketch::CornerKind::Fillet, 9.52);
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(
        warnings.iter().any(|w| w.contains("couldn't be rounded")),
        "oversized circular-bite fillet should fail safely in the exact solver, got {warnings:?}"
    );
    assert_eq!(bodies.len(), 1, "failed oversized fillet keeps one body");
    assert_eq!(
        bodies[0].1.indices.len(),
        unmodified[0].1.indices.len(),
        "oversized circular-bite fillet leaves the body unchanged"
    );
}

#[test]
fn curved_circular_rim_selection_reaches_native_solver_and_fails_safely() {
    let mut g = circular_bite_graph(None);
    let x0 = 20.0 - (14.0_f32 * 14.0 - 3.0_f32 * 3.0).sqrt();
    let x1 = 20.0 + (14.0_f32 * 14.0 - 3.0_f32 * 3.0).sqrt();
    g.add_feature(FeatureNode {
        id: "em_rim".to_string(),
        name: "Edge Mod".to_string(),
        feature: FeatureType::EdgeMod {
            target: "e".to_string(),
            edge: EdgeRef {
                p0: [x0, 5.0, 10.0],
                p1: [x1, 5.0, 10.0],
                n1: [0.0, 0.0, 1.0],
                n2: [0.0, -1.0, 0.0],
                curve: Some(EdgeCurveHint::Circle {
                    center: [20.0, 8.0, 10.0],
                    axis: [0.0, 0.0, 1.0],
                    x_dir: [1.0, 0.0, 0.0],
                    radius: 14.0,
                    start: 0.0,
                    end: std::f32::consts::PI,
                    closed: false,
                }),
                topology: None,
            },
            dist: 3.0,
            dist_expr: None,
            scope: EdgeModScope::FullEdge,
            replay: Default::default(),
            kind: crate::sketch::CornerKind::Fillet,
        },
    });
    g.add_dependency("e", "em_rim");

    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert_eq!(bodies.len(), 1, "unsupported rim fillet keeps one body");
    assert!(
        warnings
            .iter()
            .any(|w| w.contains("cut trim requires a cylindrical blend")
                || w.contains("not watertight and healthy")
                || w.contains("non-manifold edges")),
        "curved rim fillet should reach the native solver and fail safely, got {warnings:?}"
    );
    assert!(
        warnings.iter().all(|w| !w.contains("not supported yet")),
        "curved rim fillet must not be rejected by the old app-level gate: {warnings:?}"
    );
}

#[test]
fn edge_mod_validator_rejects_full_cylinder_ghost_for_circular_bite() {
    let reference = circular_bite_graph(None)
        .evaluate_bodies(&std::collections::HashSet::new())
        .unwrap();
    let ghost = crate::mock_kernel::circular_cylinder_tool(
        &circle_points((20.0, 8.0), 14.0),
        &[],
        10.0,
        &CoordinateSystem::XY,
    )
    .expect("test ghost cylinder should build");
    let ghost_mesh = MockMesh::from_solid(&ghost);
    assert!(
        circular_bite_ghost_sample_count(&ghost_mesh) > 0,
        "test setup should include samples inside the removed circular bite volume"
    );

    let err = edge_mod_candidate_stays_inside_reference(&reference[0].1, &ghost)
        .expect_err("full cylinder ghost must be rejected");
    assert!(
        err.contains("outside"),
        "validator should explain the ghost escaped the reference body, got {err}"
    );
}

#[test]
fn edge_mod_oversized_leaves_body_unchanged_and_warns() {
    // A fillet of 30mm on a 10mm box is oversized and should be rejected,
    // leaving the original body intact.
    let g = box_with_edge_mod(30.0, crate::sketch::CornerKind::Fillet);
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert_eq!(bodies.len(), 1, "body must not disappear");
    assert!(!warnings.is_empty(), "an oversized fillet should warn");
    let mesh = &bodies[0].1;
    let plain = MockMesh::make_box(10.0, 10.0, 10.0);
    assert_eq!(
        mesh.indices.len(),
        plain.indices.len(),
        "oversized fillet must leave the body unchanged"
    );
}
