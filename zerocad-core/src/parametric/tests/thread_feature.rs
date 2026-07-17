//! Thread as a parametric feature: a helical V-groove cut into a cylinder's
//! wall. Verifies the feature evaluates, resolves the cylindrical face, and —
//! when the boolean succeeds — removes material; a stalled boolean must leave
//! the body intact (cosmetic fallback), never crash or delete it.

use super::*;
use crate::parametric::FaceRef;

fn add_cylinder(g: &mut ParametricGraph, id: &str, r: f32, h: f32) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Cylinder { r, h },
    });
}

#[allow(clippy::too_many_arguments)]
fn add_thread(
    g: &mut ParametricGraph,
    id: &str,
    target: &str,
    centroid: [f32; 3],
    normal: [f32; 3],
    internal: bool,
    pitch: f32,
    depth: f32,
) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Thread {
            target: target.to_string(),
            face: FaceRef {
                centroid,
                normal,
                topology: None,
            },
            internal,
            pitch,
            depth,
            angle_deg: 60.0,
            right_handed: true,
            starts: 1,
            length: None,
            flip: false,
            designation: "M-test".to_string(),
            standard: None,
        },
    });
    g.add_dependency(target, id);
}

fn body_mesh<'a>(bodies: &'a [(String, MockMesh)], id: &str) -> &'a MockMesh {
    bodies
        .iter()
        .find(|(bid, _)| bid == id)
        .map(|(_, m)| m)
        .expect("threaded body still present")
}

#[test]
fn external_thread_feature_evaluates_and_keeps_body() {
    // Cylinder primitive: +Y axis, base at origin, radius 4, height 12. The wall
    // centroid sits at (4, 6, 0) with an outward radial normal.
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "cyl_1", 4.0, 5.0);
    add_thread(
        &mut g,
        "thread_2",
        "cyl_1",
        [4.0, 2.5, 0.0],
        [1.0, 0.0, 0.0],
        false,
        1.5,
        0.5,
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();

    // The body must survive regardless of whether the helical boolean succeeded
    // (cosmetic fallback) — never deleted, never a panic.
    let mesh = body_mesh(&bodies, "cyl_1");
    assert!(
        !mesh.vertices.is_empty() && !mesh.indices.is_empty(),
        "threaded body keeps a valid mesh"
    );
    // If material was removed the volume dropped below the plain cylinder; if the
    // boolean stalled it equals it. Either is acceptable, but log which happened.
    if let Some(props) = mesh.mass_properties() {
        let plain = std::f64::consts::PI * 16.0 * 5.0;
        eprintln!(
            "threaded volume={:.2} plain={:.2} (cut removed {:.2})",
            props.volume,
            plain,
            plain - props.volume
        );
        assert!(
            props.volume <= plain + 1.0,
            "thread must not ADD material: {} vs {}",
            props.volume,
            plain
        );
    }
    // The face must resolve — a missing cylinder would warn "no cylindrical face".
    assert!(
        !warnings.iter().any(|w| w.contains("no cylindrical face")),
        "cylindrical face should resolve: {warnings:?}"
    );
}

#[test]
fn thread_targets_clicked_cylinder_across_multipart_body() {
    // A severing operation can intentionally leave several disconnected parts
    // under one feature body. The clicked shaft wall belongs to the second part.
    // Thread must use component identity/selection location, not vector order.
    let flange = crate::mock_kernel::cylinder_solid(12.0, 1.0).expect("short flange");
    let shaft = crate::mock_kernel::cylinder_solid(8.0, 14.0)
        .expect("tall shaft")
        .transformed(&openrcad::foundation::Trsf::translation(
            openrcad::foundation::Vec::new(30.0, 0.0, 0.0),
        ));
    let flange_bounds = crate::mock_kernel::solid_aabb(&flange).expect("flange bounds");
    let plain_shaft_volume = MockMesh::from_solid(&shaft)
        .mass_properties()
        .expect("plain shaft mass properties")
        .volume;
    let mut body = LiveBody {
        id: "joined_1".to_string(),
        parts: vec![flange, shaft],
        pristine: None,
        sketch_source: None,
    };
    let step = ThreadParameters {
        face: FaceRef {
            centroid: [38.0, 7.0, 0.0],
            normal: [1.0, 0.0, 0.0],
            topology: None,
        },
        internal: false,
        pitch: 2.0,
        depth: 0.8,
        angle_deg: 60.0,
        right_handed: true,
        starts: 1,
        length: None,
        flip: false,
    };

    assert!(
        thread_one(&mut body, &step).is_ok(),
        "the picked tall shaft should be threaded"
    );

    assert_eq!(
        crate::mock_kernel::solid_aabb(&body.parts[0]),
        Some(flange_bounds),
        "the short flange must remain untouched"
    );
    let threaded_shaft_volume = MockMesh::from_solid(&body.parts[1])
        .mass_properties()
        .expect("threaded shaft mass properties")
        .volume;
    assert!(
        threaded_shaft_volume < plain_shaft_volume - 1.0,
        "the selected shaft must receive real thread geometry: {threaded_shaft_volume} vs {plain_shaft_volume}"
    );
}

#[test]
fn internal_thread_taps_a_drilled_hole() {
    // 20×20×10 box (Box extrudes the XY rect along +Z... the eval builds it via
    // extruded_region_solid on the XY plane), drilled through with a Ø6 hole
    // from the top center, then tapped with an internal thread on the hole
    // wall. The tap's grooves cut OUTWARD from the wall, so material volume
    // must drop below box-minus-plain-hole.
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "box_1".to_string(),
        name: "box_1".to_string(),
        feature: FeatureType::Box {
            w: 20.0,
            h: 20.0,
            d: 10.0,
        },
    });
    g.add_feature(FeatureNode {
        id: "hole_2".to_string(),
        name: "hole_2".to_string(),
        feature: FeatureType::Hole {
            target: "box_1".to_string(),
            position: [10.0, 10.0, 10.0],
            direction: [0.0, 0.0, -1.0],
            diameter: 6.0,
            diameter_expr: None,
            depth: None,
            kind: Default::default(),
            standard: None,
            manufacturing: None,
        },
    });
    g.add_dependency("box_1", "hole_2");
    add_thread(
        &mut g,
        "thread_3",
        "box_1",
        [13.0, 10.0, 5.0], // on the hole wall (r = 3 from the axis), mid-depth
        [1.0, 0.0, 0.0],
        true,
        1.0,
        0.6,
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();

    let mesh = body_mesh(&bodies, "box_1");
    assert!(
        !mesh.vertices.is_empty() && !mesh.indices.is_empty(),
        "tapped body keeps a valid mesh"
    );
    assert!(
        !warnings.iter().any(|w| w.contains("cosmetic")),
        "internal thread should cut real geometry, not fall back: {warnings:?}"
    );
    if let Some(props) = mesh.mass_properties() {
        let plain = 20.0 * 20.0 * 10.0 - std::f64::consts::PI * 9.0 * 10.0;
        eprintln!(
            "tapped volume={:.2} plain-drilled={:.2} (tap removed {:.2})",
            props.volume,
            plain,
            plain - props.volume
        );
        assert!(
            props.volume < plain - 0.5,
            "the tap must remove material: {} vs {}",
            props.volume,
            plain
        );
        assert!(
            props.volume > plain * 0.9,
            "the tap must not gut the body: {} vs {}",
            props.volume,
            plain
        );
    }
}

#[test]
fn thread_on_missing_body_warns_not_panics() {
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "cyl_1", 4.0, 12.0);
    add_thread(
        &mut g,
        "thread_2",
        "nonexistent",
        [4.0, 6.0, 0.0],
        [1.0, 0.0, 0.0],
        false,
        1.5,
        0.5,
    );
    let (_bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(
        warnings.iter().any(|w| w.contains("no longer exists")),
        "missing target should warn: {warnings:?}"
    );
}
