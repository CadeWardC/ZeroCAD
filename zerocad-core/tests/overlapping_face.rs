use zerocad_core::*;

fn add_cut(graph: &mut ParametricGraph, depth: f32) {
    graph.add_feature(FeatureNode {
        id: "extrude_4".into(),
        name: "Cut".into(),
        feature: FeatureType::Extrude {
            depth,
            region_indices: vec![0],
            mode: ExtrudeMode::Cut,
            target: Some("extrude_2".into()),
            depth_expr: None,
            draft_angle_deg: 0.,
            draft_angle_expr: None,
        },
    });
    graph.add_dependency("sketch_3", "extrude_4");
    graph.add_dependency("extrude_2", "extrude_4");
}

#[test]
fn saved_coplanar_cut_preserves_depth_and_existing_keyhole() {
    for depth in [-2., -6.63, -12.] {
        let mut graph = read_document_from_slice(
            include_bytes!("fixtures/overlapping-face-cut.zcad"),
            &Default::default(),
        )
        .unwrap()
        .document
        .into_evaluator_graph();
        add_cut(&mut graph, depth);
        let check = |graph: &ParametricGraph| {
            let (bodies, warnings) = graph
                .evaluate_bodies_with_warnings(&Default::default())
                .unwrap();
            assert!(warnings.is_empty(), "depth {depth}: {warnings:?}");
            assert_eq!(bodies.len(), 1);
            let cut_area = 28.8 * 13.6 - (4. - std::f64::consts::PI) * 25. - (1200. - 1018.0024);
            let expected = 1018.0024 * 9. - cut_area * f64::from((-depth).min(9.));
            let volume = bodies[0].1.mass_properties().unwrap().volume;
            assert!(
                (volume - expected).abs() < 5.,
                "depth {depth}: {volume} != {expected}"
            );
            let solids = graph.evaluated_kernel_bodies(&Default::default()).unwrap();
            let solid = &solids[0].1[0];
            assert!(solid.is_watertight());
            assert!(solid
                .validate_strict_with_policy(&openrcad::foundation::TolerancePolicy::STANDARD)
                .is_ok());
            // The second sketch lies on y=0 and cuts toward positive y.
            // This point is in the wider recess but outside the original stem.
            for y in [1., 4., 8.] {
                let inside = |x, z| {
                    openrcad::algo::boolean::point_in_solid(
                        &openrcad::foundation::Pnt::new(x, y, z),
                        solid,
                    )
                };
                assert_eq!(
                    inside(-27., -12.),
                    y > f64::from(-depth),
                    "pocket depth at y={y}"
                );
                assert!(
                    !inside(-22.4, -19.8),
                    "existing circular hole must stay open"
                );
                assert!(inside(-35., -30.), "outer plate must remain");
            }
        };
        check(&graph);
        check(&graph);
        let bytes = write_document_to_vec(
            &Document::from_graph(graph, Unit::Millimeter),
            &Default::default(),
            &Default::default(),
        )
        .unwrap();
        let reopened = read_document_from_slice(&bytes, &Default::default()).unwrap();
        check(reopened.document.evaluator_graph());
    }
}

#[test]
fn coplanar_profile_cuts_can_cross_outer_edges_and_existing_holes() {
    for holed in [false, true] {
        for depth in [-5., -12.] {
            let mut graph = ParametricGraph::new();
            let mut base = SketchCurves::new();
            base.add_rectangle((0., 0.), (10., 10.));
            if holed {
                base.add_rectangle((4., 4.), (6., 6.));
            }
            let mut cutter = SketchCurves::new();
            cutter.add_rectangle((5., 3.), (12., 7.));
            for (id, curves, z) in [("sketch_1", base, 0.), ("sketch_3", cutter, 10.)] {
                graph.add_feature(FeatureNode {
                    id: id.into(),
                    name: id.into(),
                    feature: FeatureType::Sketch {
                        cs: CoordinateSystem::XY.with_origin(Vec3::new(0., 0., z)),
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
            }
            graph.add_feature(FeatureNode {
                id: "extrude_2".into(),
                name: "Plate".into(),
                feature: FeatureType::Extrude {
                    depth: 10.,
                    region_indices: vec![0],
                    mode: ExtrudeMode::NewBody,
                    target: None,
                    depth_expr: None,
                    draft_angle_deg: 0.,
                    draft_angle_expr: None,
                },
            });
            graph.add_dependency("sketch_1", "extrude_2");
            add_cut(&mut graph, depth);
            let (bodies, warnings) = graph
                .evaluate_bodies_with_warnings(&Default::default())
                .unwrap();
            assert!(
                warnings.is_empty(),
                "holed={holed} depth={depth}: {warnings:?}"
            );
            let expected = (if holed { 960. } else { 1000. })
                - (if holed { 18. } else { 20. }) * f64::from((-depth).min(10.));
            let volume = bodies[0].1.mass_properties().unwrap().volume;
            assert!(
                (volume - expected).abs() < 0.001,
                "holed={holed} depth={depth}: {volume} != {expected}"
            );
        }
    }
}

#[test]
fn saved_overlapping_sketch_extrudes_outer_region_and_reopens() {
    let loaded = read_document_from_slice(
        include_bytes!("fixtures/overlapping-face.zcad"),
        &Default::default(),
    )
    .unwrap();
    let source = loaded.document.evaluator_graph();
    let node = source
        .graph
        .node_weights()
        .find(|n| n.id == "sketch_1")
        .unwrap();
    let FeatureType::Sketch {
        cs,
        curves,
        shapes,
        corner_mods,
        mirrors,
        solver,
        ..
    } = &node.feature
    else {
        panic!("sketch");
    };
    let curves = effective_curves_solved(
        curves,
        shapes,
        corner_mods,
        mirrors,
        solver.as_ref(),
        &source.variable_map(),
    );
    let regions = detect_regions(&curves);
    assert_eq!(regions.len(), 4);
    assert_eq!(
        regions[0].holes.len(),
        1,
        "adjacent cells form a single hole"
    );
    assert!((regions.iter().map(|r| r.area).sum::<f32>() - 1200.).abs() < 0.001);
    for (region, area) in regions
        .iter()
        .zip([1018.0024, 59.279186, 52.255108, 70.463356])
    {
        assert!(
            (region.area - area).abs() < 0.001,
            "persisted region ordering must remain stable"
        );
    }
    for (index, region) in regions.iter().enumerate().take(1) {
        assert!(
            region.degeneracy().is_none(),
            "region {index}: {:?}",
            region.degeneracy()
        );
        let mut graph = loaded.document.evaluator_graph().clone();
        graph.add_feature(FeatureNode {
            id: "extrude_2".into(),
            name: "Extrude".into(),
            feature: FeatureType::Extrude {
                target: None,
                depth: 5.,
                region_indices: vec![index],
                mode: ExtrudeMode::NewBody,
                depth_expr: None,
                draft_angle_deg: 0.,
                draft_angle_expr: None,
            },
        });
        graph.add_dependency("sketch_1", "extrude_2");
        let check = |graph: &ParametricGraph| {
            let (bodies, warnings) = graph
                .evaluate_bodies_with_warnings(&Default::default())
                .unwrap();
            assert!(warnings.is_empty(), "region {index}: {warnings:?}");
            assert_eq!(bodies.len(), 1);
            let volume = bodies[0].1.mass_properties().unwrap().volume;
            let solids = graph.evaluated_kernel_bodies(&Default::default()).unwrap();
            assert_eq!(solids[0].1.len(), 1);
            for (p, inside) in [
                ((-35., -30.), true),
                ((-22.4, -19.8), false),
                ((-22.4, -5.), false),
            ] {
                let world = cs.unproject(p.0 as f32, p.1 as f32).add(cs.n.mul(2.5));
                assert_eq!(
                    openrcad::algo::boolean::point_in_solid(
                        &openrcad::foundation::Pnt::new(
                            world.x as f64,
                            world.y as f64,
                            world.z as f64
                        ),
                        &solids[0].1[0]
                    ),
                    inside,
                    "material at {p:?}"
                );
            }
            // The display mesh approximates arcs; compare to the exact profile
            // area with a 0.04% chordal allowance, independently of point tests.
            assert!(
                (volume - f64::from(region.area) * 5.).abs() < 2.,
                "region {index}: {volume}"
            );
        };
        check(&graph);
        check(&graph);
        let bytes = write_document_to_vec(
            &Document::from_graph(graph, Unit::Millimeter),
            &Default::default(),
            &Default::default(),
        )
        .unwrap();
        let reopened = read_document_from_slice(&bytes, &Default::default()).unwrap();
        check(reopened.document.evaluator_graph());
    }
}
