//! Join through the same feature graph used by the GUI, including save/reopen.
mod common;
use common::*;
use zerocad_core::*;

fn check(g: &ParametricGraph, expected: f64) {
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&Default::default())
        .unwrap();
    assert!(warnings.is_empty(), "{warnings:?}");
    assert_eq!(bodies.len(), 1);
    assert_meshes_finite(&bodies);
    for face in &bodies[0].1.face_refs {
        let topology = face
            .topology
            .as_ref()
            .expect("joined face must be selectable");
        assert!(topology.face_id.as_ref().is_some_and(|id| !id.is_empty()));
        assert_eq!(topology.body_id.as_deref(), Some(bodies[0].0.as_str()));
        let producer = topology
            .producer_feature_id
            .as_deref()
            .expect("face producer");
        assert!(
            g.graph.node_weights().any(|node| node.id == producer),
            "unknown face producer {producer}"
        );
    }
    // Display mass integrates the tessellated circular walls. Allow 1% of
    // added material (not 1% of the much larger supporting block).
    assert!(
        (total_volume(&bodies) - expected).abs() < (expected - 4000.) * 0.01,
        "{} != {expected}",
        total_volume(&bodies)
    );
    let solids = g.evaluated_kernel_bodies(&Default::default()).unwrap();
    assert_eq!(solids.len(), 1);
    assert_eq!(solids[0].1.len(), 1);
    assert!(solids[0].1[0].is_watertight());
    assert_eq!(solids[0].1[0].split_disconnected().len(), 1);
}

fn boss(plane: CoordinateSystem, sign: f32, overhang: bool, round: bool) {
    let mut g = ParametricGraph::new();
    add_sketch_cs(
        &mut g,
        "base_sketch",
        plane,
        rect_sketch((0., 0.), (20., 20.)),
    );
    add_extrude(
        &mut g,
        "base",
        "base_sketch",
        sign * 10.,
        ExtrudeMode::NewBody,
    );
    let center = if overhang { 19. } else { 10. };
    let curves = if round {
        circle_sketch((center, 10.), 4.)
    } else {
        rect_sketch((center - 4., 6.), (center + 4., 14.))
    };
    add_sketch_cs(
        &mut g,
        "boss_sketch",
        plane.with_origin(plane.origin.add(plane.n.mul(sign * 10.))),
        curves,
    );
    add_extrude_full(
        &mut g,
        "boss",
        "boss_sketch",
        sign * 5.,
        ExtrudeMode::Join,
        Some("base".into()),
        0.,
    );
    let area = if round {
        std::f64::consts::PI * 16.
    } else {
        64.
    };
    check(&g, 4000. + area * 5.);
    check(&g.clone_document(), 4000. + area * 5.);
    let bytes = write_document_to_vec(
        &Document::from_graph(g.clone_document(), Unit::Millimeter),
        &Default::default(),
        &Default::default(),
    )
    .unwrap();
    let loaded = read_document_from_slice(&bytes, &Default::default()).unwrap();
    check(loaded.document.evaluator_graph(), 4000. + area * 5.);
    for node in g.graph.node_weights_mut() {
        if node.id == "boss" {
            if let FeatureType::Extrude { depth, .. } = &mut node.feature {
                *depth = sign * 8.;
            }
        }
    }
    g.commit_feature_edit("boss").unwrap();
    check(&g, 4000. + area * 8.);
}

macro_rules! plane_cases {
    ($($name:ident: $plane:expr),* $(,)?) => {$(
        mod $name {
            use super::*;
            #[test] fn rectangular() { boss($plane, 1., false, false); }
            #[test] fn negative() { boss($plane, -1., false, false); }
            #[test] fn overhang() { boss($plane, 1., true, false); }
            #[test] fn negative_overhang() { boss($plane, -1., true, false); }
            #[test] fn circular() { boss($plane, 1., false, true); }
            #[test] fn negative_circular() { boss($plane, -1., false, true); }
            #[test] fn circular_overhang() { boss($plane, 1., true, true); }
            #[test] fn negative_circular_overhang() { boss($plane, -1., true, true); }
        }
    )*};
}
plane_cases! {
    xy: CoordinateSystem::XY,
    xz: CoordinateSystem::XZ,
    yz: CoordinateSystem::YZ,
    translated: CoordinateSystem::XY.with_origin(Vec3::new(120., -75., 30.)),
}
