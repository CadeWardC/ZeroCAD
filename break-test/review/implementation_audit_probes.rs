//! Review-only probes. Expected to fail until the audited defects are fixed.
use openrcad::foundation::Pnt;
use openrcad::geom::{GeomSurface, GregorySurface, Surface};
use openrcad::topo::{Edge, Face, Shell, Solid, Wire};
use std::collections::HashSet;
use zerocad_core::parametric::{FaceRef, TopologyFaceRef};
use zerocad_core::{FeatureNode, FeatureType, ParametricGraph};

#[test]
fn conservative_bounds_must_include_gregory_interior() {
    let p = |x, y, z| Pnt::new(x, y, z);
    let surface = GregorySurface::new(
        p(0., 0., 0.),
        p(0., 1., 0.),
        p(0., 2., 0.),
        p(0., 3., 0.),
        p(1., 0., 0.),
        p(2., 0., 0.),
        p(3., 0., 0.),
        p(3., 1., 0.),
        p(3., 2., 0.),
        p(3., 3., 0.),
        p(1., 3., 0.),
        p(2., 3., 0.),
        p(1., 1., 10.),
        p(1., 1., 10.),
        p(2., 1., 10.),
        p(2., 1., 10.),
        p(1., 2., 10.),
        p(1., 2., 10.),
        p(2., 2., 10.),
        p(2., 2., 10.),
    );
    let middle = surface.point(0.5, 0.5);
    let corners = [p(0., 0., 0.), p(3., 0., 0.), p(3., 3., 0.), p(0., 3., 0.)];
    let wire =
        Wire::from_edges((0..4).map(|i| Edge::between_points(corners[i], corners[(i + 1) % 4])));
    let solid = Solid::new(Shell::from_faces([Face::new(
        Some(GeomSurface::Gregory(surface)),
        wire,
    )]));
    let (lo, hi) = solid.conservative_bounding_box().corners().unwrap();
    assert!(
        middle.z() >= lo.z() && middle.z() <= hi.z(),
        "surface interior={middle:?}; bounds={lo:?}..{hi:?}"
    );
}

#[test]
fn named_planar_cap_must_not_thread_cylindrical_wall() {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "rod".into(),
        name: "rod".into(),
        feature: FeatureType::Cylinder { r: 6., h: 10. },
    });
    let (before, _) = graph
        .evaluate_bodies_with_warnings(&HashSet::new())
        .unwrap();
    let cap = before[0]
        .1
        .face_refs
        .iter()
        .find(|f| f.normal[1] > 0.9)
        .unwrap();
    let face = FaceRef {
        centroid: cap.centroid,
        normal: cap.normal,
        topology: cap.topology.as_ref().map(|t| TopologyFaceRef {
            body_id: t.body_id.clone(),
            component_id: t.component_id.clone(),
            topology_version: t.topology_version,
            face_id: t.face_id.clone(),
            surface_kind: t.surface_kind.clone(),
            producer_feature_id: t.producer_feature_id.clone(),
            source_entity_id: t.source_entity_id.clone(),
        }),
    };
    assert!(
        face.topology.is_some(),
        "probe requires a durable planar selection"
    );
    graph.add_feature(FeatureNode {
        id: "thread".into(),
        name: "thread".into(),
        feature: FeatureType::Thread {
            target: "rod".into(),
            face,
            internal: false,
            pitch: 2.,
            depth: 0.2,
            angle_deg: 60.,
            right_handed: true,
            starts: 1,
            length: None,
            flip: false,
            designation: "audit".into(),
            standard: None,
        },
    });
    graph.add_dependency("rod", "thread");
    let (after, warnings) = graph
        .evaluate_bodies_with_warnings(&HashSet::new())
        .unwrap();
    eprintln!(
        "cap selection warnings={warnings:?}; before vertices={}; after vertices={}",
        before[0].1.vertices.len(),
        after[0].1.vertices.len()
    );
    assert!(
        !warnings.is_empty(),
        "a named planar cap must be rejected, not redirected to a cylindrical wall"
    );
    assert_eq!(
        before[0].1.vertices, after[0].1.vertices,
        "rejected selection must preserve source"
    );
}
