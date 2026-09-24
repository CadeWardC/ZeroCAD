//! Counterbores through joined bosses must survive subsequent joins and cuts.
mod common;
use common::*;
use zerocad_core::*;

fn counterbored_bosses(plane: CoordinateSystem) {
    let mut graph = ParametricGraph::new();
    add_sketch_cs(
        &mut graph,
        "base_profile",
        plane,
        rect_sketch((0., 0.), (80., 40.)),
    );
    add_extrude(&mut graph, "base", "base_profile", 8., ExtrudeMode::NewBody);
    let mut expected = 25600.;
    for (index, x) in [20., 60.].into_iter().enumerate() {
        let sketch = format!("boss_profile_{index}");
        let boss = format!("boss_{index}");
        add_sketch_cs(
            &mut graph,
            &sketch,
            plane.with_origin(plane.origin.add(plane.n.mul(8.))),
            circle_sketch((x, 20.), 6.),
        );
        add_extrude(&mut graph, &boss, &sketch, 10., ExtrudeMode::Join);
        expected += std::f64::consts::PI * 360.;
        let position = plane.unproject(x, 20.).add(plane.n.mul(18.));
        add_hole(
            &mut graph,
            &format!("hole_{index}"),
            "base",
            [position.x, position.y, position.z],
            [-plane.n.x, -plane.n.y, -plane.n.z],
            4.,
            None,
            HoleKind::Counterbore {
                diameter: 8.,
                depth: 2.,
            },
        );
        expected -= std::f64::consts::PI * (4. * 16. + 16. * 2.);
        let (bodies, warnings) = graph
            .evaluate_bodies_with_warnings(&Default::default())
            .unwrap();
        assert!(warnings.is_empty(), "boss {index}: {warnings:?}");
        assert_meshes_valid(&bodies);
        for face in &bodies[0].1.face_refs {
            let topology = face.topology.as_ref().expect("cut face identity");
            assert_eq!(topology.body_id.as_deref(), Some("base"));
            assert!(topology.face_id.as_ref().is_some_and(|id| !id.is_empty()));
            assert!(topology
                .producer_feature_id
                .as_ref()
                .is_some_and(|id| graph.graph.node_weights().any(|node| &node.id == id)));
        }
        assert_close(
            total_volume(&bodies),
            expected,
            5.,
            0.0001,
            "counterbored boss material",
        );
    }
    let cold = graph.clone_document();
    let (bodies, warnings) = cold
        .evaluate_bodies_with_warnings(&Default::default())
        .unwrap();
    assert!(warnings.is_empty(), "{warnings:?}");
    assert_close(
        total_volume(&bodies),
        expected,
        5.,
        0.0001,
        "cold counterbores",
    );
    let solids = cold.evaluated_kernel_bodies(&Default::default()).unwrap();
    assert_eq!(solids.len(), 1);
    assert_eq!(solids[0].1.len(), 1);
    assert!(solids[0].1[0].is_watertight());
    // The native mesh must have the correct orientation too; display winding
    // repair must not conceal a wrong kernel volume from later operations.
    let native = openrcad::mesh::tessellate_checked(&solids[0].1[0], 0.05, 0.25).unwrap();
    let native_volume = openrcad::mesh::mass_properties(&native).unwrap().volume;
    assert!(
        (native_volume - expected).abs() < (expected - 25600.) * 0.01,
        "native volume {native_volume} != {expected}"
    );
}

#[test]
fn xy_counterbores_after_joins() {
    counterbored_bosses(CoordinateSystem::XY);
}
#[test]
fn xz_counterbores_after_joins() {
    counterbored_bosses(CoordinateSystem::XZ);
}
#[test]
fn yz_counterbores_after_joins() {
    counterbored_bosses(CoordinateSystem::YZ);
}

#[test]
fn through_bore_heals_on_every_plane_and_sweep_direction() {
    for plane in [
        CoordinateSystem::XY,
        CoordinateSystem::XZ,
        CoordinateSystem::YZ,
    ] {
        for sign in [-1., 1.] {
            let mut graph = ParametricGraph::new();
            add_sketch_cs(
                &mut graph,
                "profile",
                plane,
                rect_sketch((0., 0.), (30., 30.)),
            );
            add_extrude(
                &mut graph,
                "base",
                "profile",
                sign * 10.,
                ExtrudeMode::NewBody,
            );
            let start = plane.unproject(15., 15.).add(plane.n.mul(sign * 10.));
            add_hole(
                &mut graph,
                "hole",
                "base",
                [start.x, start.y, start.z],
                [-sign * plane.n.x, -sign * plane.n.y, -sign * plane.n.z],
                8.,
                None,
                HoleKind::Simple,
            );
            let face = capture_face(&graph, "base", [0., 0., 0.]);
            add_feature(
                &mut graph,
                "heal",
                FeatureType::FaceDelete {
                    target: "base".into(),
                    face,
                },
                &["base", "hole"],
            );
            for document in [&graph, &graph.clone_document()] {
                let (bodies, warnings) = document
                    .evaluate_bodies_with_warnings(&Default::default())
                    .unwrap();
                assert!(warnings.is_empty(), "{plane:?}, {sign}: {warnings:?}");
                assert_meshes_valid(&bodies);
                assert_close(total_volume(&bodies), 9000., 0.01, 1e-6, "restored plate");
            }
        }
    }
}
