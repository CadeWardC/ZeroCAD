use super::*;

#[test]
fn newbody_makes_one_body() {
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    add_extrude(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);
    let bodies = g
        .evaluate_bodies(&std::collections::HashSet::new())
        .unwrap();
    assert_eq!(bodies.len(), 1, "new body should yield exactly one body");
    assert!(!bodies[0].1.indices.is_empty());
}

#[test]
fn cut_punches_hole_no_extra_body() {
    let mut g = ParametricGraph::new();
    // Base 10x10x10 block.
    add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    add_extrude(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);
    let plain = g
        .evaluate_bodies(&std::collections::HashSet::new())
        .unwrap();
    let plain_tris = plain[0].1.indices.len() / 3;

    // Cut a 6x6 square clean through it.
    add_sketch(&mut g, "sketch_3", rect_sketch((2.0, 2.0), (8.0, 8.0)));
    add_extrude(&mut g, "extrude_4", "sketch_3", 10.0, ExtrudeMode::Cut);
    let cut = g
        .evaluate_bodies(&std::collections::HashSet::new())
        .unwrap();

    assert_eq!(
        cut.len(),
        1,
        "cut must not add a separate body (got {})",
        cut.len()
    );
    let cut_tris = cut[0].1.indices.len() / 3;
    assert!(
        cut_tris > plain_tris,
        "cut body should have MORE triangles than the plain block (hole walls): plain={plain_tris} cut={cut_tris}"
    );
}

#[test]
fn join_negative_depth_into_body_keeps_it() {
    // A box, then a join whose extrude runs straight back into it (negative
    // depth on the same plane). The tool is swallowed by the box, so the
    // union must keep the box — not delete it (the inside-out-solid bug).
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    add_extrude(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);
    add_sketch(&mut g, "sketch_3", rect_sketch((2.0, 2.0), (8.0, 8.0)));
    add_extrude(&mut g, "extrude_4", "sketch_3", -5.0, ExtrudeMode::Join);
    let bodies = g
        .evaluate_bodies(&std::collections::HashSet::new())
        .unwrap();
    assert_eq!(bodies.len(), 1, "join into a body must stay one body");
    let max_z = bodies[0]
        .1
        .vertices
        .chunks(6)
        .map(|v| v[2])
        .fold(f32::MIN, f32::max);
    assert!(
        max_z >= 9.9,
        "join must keep the original box (top near z=10), got {max_z}"
    );
}

#[test]
fn join_with_no_overlap_is_atomic_and_does_not_create_a_body() {
    // A box, then a join far away that overlaps nothing. Join is strict: the
    // feature stays unresolved and the pre-feature body remains unchanged.
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    add_extrude(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);
    add_sketch(&mut g, "sketch_3", rect_sketch((50.0, 50.0), (60.0, 60.0)));
    add_extrude(&mut g, "extrude_4", "sketch_3", 5.0, ExtrudeMode::Join);
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert_eq!(
        bodies.len(),
        1,
        "a failed Join must not create a stray body"
    );
    assert!(
        warnings.iter().any(|w| w.contains("was not applied")),
        "expected an atomic Join failure warning, got {warnings:?}"
    );
}

#[test]
fn bugcase1_partial_height_split_circle_join_fuses_to_one_solid() {
    let mut curves = SketchCurves::new();
    for (a, b) in [
        ((-13.2, -13.4), (8.2, -13.4)),
        ((8.2, -13.4), (8.2, -6.8999996)),
        ((8.2, -6.8999996), (-13.2, -6.8999996)),
        ((-13.2, -6.8999996), (-13.2, -13.4)),
        ((8.2, -6.8999996), (-10.2, -6.900001)),
        ((-10.2, -6.900001), (-10.2, -6.500001)),
        ((-10.2, -6.900001), (8.2, -6.900001)),
        ((8.2, -6.900001), (8.2, -6.100001)),
        ((8.2, -6.100001), (-10.2, -6.100001)),
        ((-10.2, -6.100001), (-10.2, -6.900001)),
        ((8.2, -6.100001), (-10.3, -6.100001)),
        ((-10.3, -6.100001), (-10.3, -2.9000008)),
        ((-10.3, -2.9000008), (8.2, -2.9000008)),
        ((8.2, -2.9000008), (8.2, -6.100001)),
    ] {
        curves.add_line(a, b);
    }
    curves.add_circle((-10.2, -6.500001), 0.4);
    let regions = crate::sketch::detect_regions(&curves);
    assert_eq!(regions.len(), 5, "the tolerance sliver is normalized first");
    let base_regions: Vec<usize> = regions
        .iter()
        .enumerate()
        .filter_map(|(index, region)| (region.area > 1.0).then_some(index))
        .collect();
    let circle_regions: Vec<usize> = regions
        .iter()
        .enumerate()
        .filter_map(|(index, region)| (region.area < 1.0).then_some(index))
        .collect();
    assert_eq!(base_regions.len(), 3);
    assert_eq!(circle_regions.len(), 2);
    let mut selected = vec![false; regions.len()];
    for &index in &base_regions {
        selected[index] = true;
    }
    let prepared = crate::parametric::extrude::prepare_extrude_regions(&regions, &selected);
    assert_eq!(prepared.len(), 1, "the base faces must become one profile");
    assert!(
        prepared[0].region.analytic.is_some(),
        "the merged profile must retain the exact circular notch"
    );
    let mut selected_circle = vec![false; regions.len()];
    for &index in &circle_regions {
        selected_circle[index] = true;
    }
    let prepared_circle =
        crate::parametric::extrude::prepare_extrude_regions(&regions, &selected_circle);
    assert_eq!(prepared_circle.len(), 1);

    let mut graph = ParametricGraph::new();
    add_sketch(&mut graph, "sketch", curves);
    graph.add_feature(FeatureNode {
        id: "base".into(),
        name: "Base".into(),
        feature: FeatureType::Extrude {
            target: None,
            depth: 2.2,
            region_indices: base_regions,
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    graph.add_dependency("sketch", "base");

    let base_solids = graph
        .debug_kernel_solids(&Default::default())
        .expect("inspect normalized base extrusion");
    assert_eq!(base_solids.len(), 1);
    assert_eq!(
        base_solids[0].1.len(),
        1,
        "adjacent selected faces must be extruded from one combined outline"
    );

    graph.add_feature(FeatureNode {
        id: "plug".into(),
        name: "Plug".into(),
        feature: FeatureType::Extrude {
            target: Some("base".into()),
            // The real UI case uses a different extent from the base. This
            // leaves a partial-height coincident cylindrical wall that must be
            // split and sewn rather than mistaken for line-only tangency.
            depth: 1.43,
            region_indices: circle_regions,
            mode: ExtrudeMode::Join,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    graph.add_dependency("sketch", "plug");
    graph.add_dependency("base", "plug");

    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&Default::default())
        .expect("evaluate split-circle join");
    assert!(
        warnings.is_empty(),
        "the partial-height shared wall must use the sectional Join fallback: {warnings:?}"
    );
    assert_eq!(bodies.len(), 1, "the joined result stays one body");
    let solids = graph
        .debug_kernel_solids(&Default::default())
        .expect("inspect joined kernel body");
    assert_eq!(solids.len(), 1);
    assert_eq!(
        solids[0].1.len(),
        1,
        "the Join must replace the segmented fallback with one kernel solid"
    );
    assert!(solids[0].1[0].is_watertight());
    assert!(solids[0].1[0].health_report().is_healthy());

    let mirror_face = bodies[0]
        .1
        .face_refs
        .iter()
        .find(|face| face.normal[0] > 0.99 && (face.centroid[0] - 8.2).abs() < 1.0e-3)
        .expect("BugCase1 +X end face")
        .clone();
    graph.add_feature(FeatureNode {
        id: "mirror".into(),
        name: "Mirror".into(),
        feature: FeatureType::Pattern {
            source: "base".into(),
            kind: PatternKind::Mirror {
                plane: PlaneBase::YZ,
                face: Some(FaceRef {
                    centroid: mirror_face.centroid,
                    normal: mirror_face.normal,
                    topology: mirror_face.topology.map(|topology| TopologyFaceRef {
                        body_id: topology.body_id,
                        component_id: topology.component_id,
                        topology_version: topology.topology_version,
                        face_id: topology.face_id,
                        surface_kind: topology.surface_kind,
                        producer_feature_id: topology.producer_feature_id,
                        source_entity_id: topology.source_entity_id,
                    }),
                }),
                offset: 0.0,
                offset_expr: None,
                join: true,
            },
        },
    });
    graph.add_dependency("base", "mirror");
    let latest = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(1));
    let mirrored = graph
        .evaluate_request(
            &Default::default(),
            EvaluationQuality::Final,
            &EvaluationCancellation::new(1, latest),
        )
        .expect("evaluate joined mirror");
    assert_eq!(
        mirrored.bodies.len(),
        1,
        "the mirror must join to the source"
    );
    assert!(
        mirrored.diagnostics.iter().all(|diagnostic| {
            !diagnostic
                .message
                .contains("boolean result face could not be attributed")
        }),
        "joined mirror faces must retain input lineage: {:#?}",
        mirrored.diagnostics
    );
}

#[test]
fn line_tangent_cylinder_join_reports_non_manifold_contact() {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "box".into(),
        name: "Box".into(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
    let mut circle = SketchCurves::new();
    // The circle touches the box's x=10 side at one point in the sketch; after
    // extrusion the solids share only one generator line.
    circle.add_circle((13.0, 5.0), 3.0);
    add_sketch(&mut graph, "sketch", circle);
    graph.add_feature(FeatureNode {
        id: "tangent_join".into(),
        name: "Tangent Join".into(),
        feature: FeatureType::Extrude {
            target: Some("box".into()),
            depth: 10.0,
            region_indices: Vec::new(),
            mode: ExtrudeMode::Join,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    graph.add_dependency("sketch", "tangent_join");
    graph.add_dependency("box", "tangent_join");

    let (_bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&Default::default())
        .expect("evaluate line-tangent join");
    assert!(
        warnings
            .iter()
            .any(|warning| warning.contains("non-manifold")),
        "the failed tangent Join must explain how to repair the profile: {warnings:?}"
    );
}

#[test]
fn multi_region_join_rolls_back_when_any_region_cannot_fuse() {
    use crate::geometry::Vec3;

    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "box_1".to_string(),
        name: "box_1".to_string(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
    let mut curves = SketchCurves::new();
    curves.add_circle((5.0, 5.0), 2.0); // fuses to the box
    curves.add_circle((30.0, 30.0), 2.0); // intentionally misses
    add_sketch_cs(
        &mut graph,
        "sketch_2",
        CoordinateSystem::new(Vec3::new(0.0, 0.0, 10.0), Vec3::X, Vec3::Y),
        curves,
    );
    graph.add_feature(FeatureNode {
        id: "extrude_3".to_string(),
        name: "extrude_3".to_string(),
        feature: FeatureType::Extrude {
            target: Some("box_1".to_string()),
            depth: 5.0,
            region_indices: Vec::new(),
            mode: ExtrudeMode::Join,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    graph.add_dependency("sketch_2", "extrude_3");
    graph.add_dependency("box_1", "extrude_3");

    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&Default::default())
        .expect("evaluate atomic multi-region Join");
    assert_eq!(bodies.len(), 1, "failed Join must not create another body");
    let max_z = bodies[0]
        .1
        .vertices
        .chunks_exact(6)
        .map(|vertex| vertex[2])
        .fold(f32::NEG_INFINITY, f32::max);
    assert!(
        (max_z - 10.0).abs() < 0.05,
        "the first region must be rolled back with the failed second region, max z={max_z}"
    );
    assert!(warnings
        .iter()
        .any(|warning| warning.contains("was not applied")));
    let solids = graph
        .debug_kernel_solids(&Default::default())
        .expect("inspect rolled-back Join");
    assert_eq!(
        solids[0].1.len(),
        1,
        "no unfused tool may hide in body.parts"
    );
}

#[test]
fn clean_model_has_no_warnings() {
    // A plain new-body extrude and a normal through-cut should evaluate with
    // zero warnings — successful coplanarity fallbacks must stay silent.
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    add_extrude(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);
    add_sketch(&mut g, "sketch_3", rect_sketch((2.0, 2.0), (8.0, 8.0)));
    add_extrude(&mut g, "extrude_4", "sketch_3", 10.0, ExtrudeMode::Cut);
    let (_bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(
        warnings.is_empty(),
        "a clean cut-through model must not warn, got {warnings:?}"
    );
}

#[test]
fn edge_mod_on_sketched_prism_applies() {
    // The screenshots' box is a sketch→extrude prism (build_extrusion_solid),
    // not a make_box. Both chamfer AND fillet must apply to a top edge of such a
    // prism (pre-fix the fillet failed because the sewn top cap stored an inward
    // normal).
    for kind in [
        crate::sketch::CornerKind::Chamfer,
        crate::sketch::CornerKind::Fillet,
    ] {
        let mut g = ParametricGraph::new();
        add_sketch(&mut g, "s", rect_sketch((0.0, 0.0), (40.0, 30.0)));
        add_extrude(&mut g, "e", "s", 20.0, ExtrudeMode::NewBody);
        // Top-front edge of the prism: from (0,0,20) to (40,0,20); adjacent
        // faces are +Z (top) and -Y (front).
        g.add_feature(FeatureNode {
            id: "em".to_string(),
            name: "Edge Mod".to_string(),
            feature: FeatureType::EdgeMod {
                target: "e".to_string(),
                edge: EdgeRef {
                    p0: [0.0, 0.0, 20.0],
                    p1: [40.0, 0.0, 20.0],
                    n1: [0.0, 0.0, 1.0],
                    n2: [0.0, -1.0, 0.0],
                    curve: None,
                    topology: None,
                },
                dist: 2.11,
                dist_expr: None,
                kind,
            },
        });
        g.add_dependency("e", "em");
        let mut hidden = std::collections::HashSet::new();
        hidden.insert("s".to_string());
        let (bodies, warnings) = g.evaluate_bodies_with_warnings(&hidden).unwrap();
        assert_eq!(bodies.len(), 1, "{kind:?} on a prism must stay one body");
        assert!(
            warnings.is_empty(),
            "{kind:?} on a clean prism edge should not warn, got {warnings:?}"
        );
        // The top-front sharp edge (y=0, z=20) must be gone — the edge-mod applied.
        let m = &bodies[0].1;
        let sharp = m
            .vertices
            .chunks(6)
            .any(|v| v[1].abs() < 0.02 && (v[2] - 20.0).abs() < 0.02);
        assert!(
            !sharp,
            "{kind:?} should have removed the prism's top-front edge"
        );
    }
}

#[test]
fn prism_box_cut_and_join_a_cylinder_through_top() {
    use crate::geometry::Vec3;
    // Full parametric repro of the screenshots: a sketch→extrude PRISM box,
    // then a circle on its top face (a) Cut downward through it and (b) Joined
    // upward as a boss. The cut must actually bore a hole (many more tris than
    // the plain box) and the join must add the boss (reaches z≈23), each one
    // body with no warning.
    let make_base = || {
        let mut g = ParametricGraph::new();
        add_sketch(&mut g, "s_base", rect_sketch((0.0, 0.0), (40.0, 20.0)));
        add_extrude(&mut g, "e_base", "s_base", 15.0, ExtrudeMode::NewBody);
        g
    };
    let plain_tris = make_base()
        .evaluate_bodies(&std::collections::HashSet::new())
        .unwrap()[0]
        .1
        .indices
        .len()
        / 3;
    let top = CoordinateSystem::new(Vec3::new(0.0, 0.0, 15.0), Vec3::X, Vec3::Y);

    // CUT through the top.
    let mut gc = make_base();
    let mut cc = SketchCurves::new();
    cc.add_circle((20.0, 10.0), 4.0);
    add_sketch_cs(&mut gc, "s_cut", top, cc);
    add_extrude(&mut gc, "e_cut", "s_cut", -16.62, ExtrudeMode::Cut);
    gc.add_dependency("e_base", "e_cut");
    let (cb, cw) = gc
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert_eq!(cb.len(), 1, "cut stays one body");
    assert!(
        cw.is_empty(),
        "a clean drill-through should not warn, got {cw:?}"
    );
    assert!(
        cb[0].1.indices.len() / 3 > plain_tris + 6,
        "cut must bore a hole (more tris than the plain box={plain_tris})"
    );

    // JOIN a boss on top.
    let mut gj = make_base();
    let mut jc = SketchCurves::new();
    jc.add_circle((20.0, 10.0), 4.0);
    add_sketch_cs(&mut gj, "s_join", top, jc);
    add_extrude(&mut gj, "e_join", "s_join", 8.0, ExtrudeMode::Join);
    gj.add_dependency("e_base", "e_join");
    let (jb, jw) = gj
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert_eq!(jb.len(), 1, "join stays one body");
    assert!(
        jw.is_empty(),
        "a clean boss join should not warn, got {jw:?}"
    );
    let max_z = jb
        .iter()
        .flat_map(|(_, m)| m.vertices.chunks(6))
        .map(|v| v[2])
        .fold(f32::MIN, f32::max);
    assert!(
        max_z >= 22.9,
        "join must add the boss (top z≈23), got {max_z}"
    );
}

#[test]
fn join_circle_boss_on_top_survives() {
    use crate::geometry::Vec3;
    // A 10×10×10 box, then a Ø6 circular boss sketched on its top face (z=10)
    // and joined upward 5mm — the "boss on a face" case from the screenshots,
    // where the boss base is coplanar with the box top and the boss is now a
    // smooth analytic cylinder. The boss must survive: one body, top near z=15.
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "box_1".to_string(),
        name: "Box".to_string(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
    let top = CoordinateSystem::new(Vec3::new(0.0, 0.0, 10.0), Vec3::X, Vec3::Y);
    let mut circ = SketchCurves::new();
    circ.add_circle((5.0, 5.0), 3.0);
    add_sketch_cs(&mut g, "sketch_2", top, circ);
    add_extrude(&mut g, "extrude_3", "sketch_2", 5.0, ExtrudeMode::Join);
    g.add_dependency("box_1", "extrude_3");

    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert_eq!(
        bodies.len(),
        1,
        "boss-join must stay one body, got {}",
        bodies.len()
    );
    assert!(
        warnings.is_empty(),
        "a coplanar boss join should not warn, got {warnings:?}"
    );
    let max_z = bodies[0]
        .1
        .vertices
        .chunks(6)
        .map(|v| v[2])
        .fold(f32::MIN, f32::max);
    assert!(
        max_z >= 14.9,
        "joined boss must reach z≈15 (top of boss), got {max_z}"
    );
}

#[test]
fn edge_mod_on_boss_union_body_keeps_boss() {
    for kind in [
        crate::sketch::CornerKind::Chamfer,
        crate::sketch::CornerKind::Fillet,
    ] {
        let g = box_with_boss_then_edge_mod(2.0, kind);
        let (bodies, _warnings) = g
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .unwrap();
        assert_eq!(bodies.len(), 1, "{kind:?} on union body must stay one body");
        let m = &bodies[0].1;
        let max_z = m.vertices.chunks(6).map(|v| v[2]).fold(f32::MIN, f32::max);
        assert!(
            max_z >= 14.9,
            "{kind:?} must preserve the boss (top z≈15), got {max_z}"
        );
        // The modified bottom-front corner must not still be sharp.
        let sharp = m
            .vertices
            .chunks(6)
            .any(|v| v[1].abs() < 0.01 && v[2].abs() < 0.01);
        assert!(!sharp, "{kind:?} should have removed the y=0,z=0 corner");
    }
}

#[test]
fn join_overlapping_stays_one_body() {
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    add_extrude(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);
    // Overlapping block joined in (shifted so faces aren't coplanar).
    add_sketch(&mut g, "sketch_3", rect_sketch((5.0, 5.0), (15.0, 15.0)));
    add_extrude(&mut g, "extrude_4", "sketch_3", 5.0, ExtrudeMode::Join);
    let bodies = g
        .evaluate_bodies(&std::collections::HashSet::new())
        .unwrap();
    assert_eq!(
        bodies.len(),
        1,
        "join into overlapping body should stay one body (got {})",
        bodies.len()
    );
}

#[test]
fn end_face_join_is_one_continuous_prism() {
    use crate::geometry::Vec3;

    // Reproduce a full-face Join from the end of a sketched prism. The result
    // should be topologically equivalent to one 24×10×3 prism: no internal cap,
    // no join-plane edge, and one selectable face per continuous outer plane.
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (20.0, 10.0)));
    add_extrude(&mut g, "extrude_2", "sketch_1", 3.0, ExtrudeMode::NewBody);

    let end = CoordinateSystem::new(Vec3::new(20.0, 0.0, 0.0), Vec3::Y, Vec3::Z);
    add_sketch_cs(
        &mut g,
        "sketch_3",
        end,
        rect_sketch((0.0, 0.0), (10.0, 3.0)),
    );
    add_extrude(&mut g, "extrude_4", "sketch_3", 4.0, ExtrudeMode::Join);
    g.add_dependency("extrude_2", "extrude_4");

    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("end-face join evaluates");
    assert_eq!(bodies.len(), 1, "end-face join must remain one body");
    assert!(warnings.is_empty(), "end-face join warned: {warnings:?}");

    let mesh = &bodies[0].1;
    let faces_on_plane = |axis: usize, coordinate: f32| {
        mesh.indices
            .chunks_exact(3)
            .enumerate()
            .filter_map(|(triangle, indices)| {
                indices
                    .iter()
                    .all(|&vertex| {
                        let base = vertex as usize * 6;
                        (mesh.vertices[base + axis] - coordinate).abs() < 0.02
                    })
                    .then(|| mesh.face_ids[triangle])
            })
            .collect::<std::collections::HashSet<u32>>()
    };

    assert_eq!(
        faces_on_plane(2, 3.0).len(),
        1,
        "the joined top must be one continuous selectable face"
    );
    assert_eq!(
        faces_on_plane(1, 0.0).len(),
        1,
        "the joined front must be one continuous selectable face"
    );
    assert!(
        faces_on_plane(0, 20.0).is_empty(),
        "the former end face must not remain as an internal cap"
    );

    let seam_segments = mesh
        .edge_indices
        .chunks_exact(2)
        .filter(|edge| {
            edge.iter().all(|&vertex| {
                let base = vertex as usize * 3;
                (mesh.edge_vertices[base] - 20.0).abs() < 0.02
            })
        })
        .count();
    assert_eq!(
        seam_segments, 0,
        "the fused body must not expose a join-plane edge"
    );
}
