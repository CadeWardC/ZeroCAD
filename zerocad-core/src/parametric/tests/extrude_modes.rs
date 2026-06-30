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
fn join_with_no_overlap_warns_and_makes_separate_body() {
    // A box, then a join far away that overlaps nothing. It still produces a
    // body (Fusion semantics) but the user asked to *join*, so evaluation
    // must surface a warning explaining the stray body.
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    add_extrude(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);
    add_sketch(&mut g, "sketch_3", rect_sketch((50.0, 50.0), (60.0, 60.0)));
    add_extrude(&mut g, "extrude_4", "sketch_3", 5.0, ExtrudeMode::Join);
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert_eq!(bodies.len(), 2, "non-overlapping join still yields a body");
    assert!(
        warnings.iter().any(|w| w.contains("separate body")),
        "expected a 'became a separate body' warning, got {warnings:?}"
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
                scope: EdgeModScope::FullEdge,
                replay: Default::default(),
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
