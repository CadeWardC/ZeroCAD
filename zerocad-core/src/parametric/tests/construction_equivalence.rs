//! Equivalent-history regression tests. These deliberately build the same
//! threaded flanged shaft through different feature sequences; downstream face
//! operations must depend on evaluated geometry + persistent identity, not on
//! whichever Join happened to create the wall.

use super::*;

fn circle(radius: f32) -> SketchCurves {
    let mut curves = SketchCurves::new();
    curves.add_circle((0.0, 0.0), radius);
    curves
}

fn plane_at(z: f32) -> CoordinateSystem {
    CoordinateSystem::new(
        crate::geometry::Vec3::new(0.0, 0.0, z),
        crate::geometry::Vec3::X,
        crate::geometry::Vec3::Y,
    )
}

fn add_targeted_extrude(
    graph: &mut ParametricGraph,
    id: &str,
    sketch: &str,
    target: Option<&str>,
    depth: f32,
    mode: ExtrudeMode,
    region_indices: Vec<usize>,
) {
    graph.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Extrude {
            target: target.map(str::to_string),
            depth,
            region_indices,
            mode,
            depth_expr: None,
        },
    });
    graph.add_dependency(sketch, id);
    if let Some(target) = target {
        graph.add_dependency(target, id);
    }
}

fn add_external_thread(graph: &mut ParametricGraph, id: &str, target: &str) {
    graph.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Thread {
            target: target.to_string(),
            face: FaceRef {
                centroid: [8.0, 0.0, 9.0],
                normal: [1.0, 0.0, 0.0],
                topology: None,
            },
            internal: false,
            pitch: 2.0,
            depth: 0.6,
            angle_deg: 60.0,
            right_handed: true,
            starts: 1,
            length: None,
            flip: false,
            designation: "M16x2".to_string(),
            standard: None,
        },
    });
    graph.add_dependency(target, id);
}

fn flanged_shaft(flange_first: bool) -> ParametricGraph {
    let mut graph = ParametricGraph::new();
    if flange_first {
        add_sketch_cs(&mut graph, "sketch_1", CoordinateSystem::XY, circle(12.0));
        add_targeted_extrude(
            &mut graph,
            "extrude_2",
            "sketch_1",
            None,
            2.0,
            ExtrudeMode::NewBody,
            Vec::new(),
        );
        add_sketch_cs(&mut graph, "sketch_3", plane_at(2.0), circle(8.0));
        add_targeted_extrude(
            &mut graph,
            "extrude_4",
            "sketch_3",
            Some("extrude_2"),
            14.0,
            ExtrudeMode::Join,
            Vec::new(),
        );
        add_external_thread(&mut graph, "thread_5", "extrude_2");
    } else {
        // Reverse the construction: create the shaft first, then Join the flange
        // downward from the shaft's bottom face. The final B-Rep is equivalent.
        add_sketch_cs(&mut graph, "sketch_1", plane_at(2.0), circle(8.0));
        add_targeted_extrude(
            &mut graph,
            "extrude_2",
            "sketch_1",
            None,
            14.0,
            ExtrudeMode::NewBody,
            Vec::new(),
        );
        add_sketch_cs(&mut graph, "sketch_3", plane_at(2.0), circle(12.0));
        add_targeted_extrude(
            &mut graph,
            "extrude_4",
            "sketch_3",
            Some("extrude_2"),
            -2.0,
            ExtrudeMode::Join,
            Vec::new(),
        );
        add_external_thread(&mut graph, "thread_5", "extrude_2");
    }
    graph
}

#[test]
fn equivalent_flanged_shaft_histories_both_accept_the_same_thread() {
    let annulus_first = flanged_shaft(true);
    let solid_then_bore = flanged_shaft(false);
    let (annulus_bodies, annulus_warnings, annulus_statuses) = annulus_first
        .evaluate_bodies_with_status(&Default::default())
        .expect("evaluate annulus-first history");
    let (bored_bodies, bored_warnings, bored_statuses) = solid_then_bore
        .evaluate_bodies_with_status(&Default::default())
        .expect("evaluate solid-then-bore history");

    assert!(
        annulus_warnings.is_empty(),
        "annulus-first construction must fully resolve: {annulus_warnings:?}"
    );
    assert!(
        bored_warnings.is_empty(),
        "solid-then-bore construction must fully resolve: {bored_warnings:?}"
    );
    assert!(annulus_statuses
        .iter()
        .all(|status| !status.is_unresolved()));
    assert!(bored_statuses.iter().all(|status| !status.is_unresolved()));
    assert_eq!(annulus_bodies.len(), 1);
    assert_eq!(bored_bodies.len(), 1);

    let annulus_mesh = &annulus_bodies[0].1;
    let bored_mesh = &bored_bodies[0].1;
    let annulus_volume = annulus_mesh
        .mass_properties()
        .expect("annulus-first mass properties")
        .volume;
    let bored_volume = bored_mesh
        .mass_properties()
        .expect("bored mass properties")
        .volume;
    let relative_delta = (annulus_volume - bored_volume).abs() / bored_volume.max(1.0);
    assert!(
        relative_delta < 0.02,
        "equivalent histories should have equivalent threaded volume: \
         annulus={annulus_volume}, bored={bored_volume}, delta={relative_delta}"
    );

    for (label, mesh) in [
        ("annulus-first", annulus_mesh),
        ("solid-then-bore", bored_mesh),
    ] {
        assert!(
            mesh.face_refs.iter().all(|face| {
                face.topology
                    .as_ref()
                    .and_then(|topology| topology.component_id.as_deref())
                    .is_some()
            }),
            "{label} output must carry component identity on every selectable face"
        );
    }
}
