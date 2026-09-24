//! Regression coverage for the break-test implementation audit.
use openrcad::foundation::Pnt;
use openrcad::geom::{GeomSurface, GregorySurface, Surface};
use openrcad::topo::{Edge, Face, Shell, Solid, Wire};
use std::collections::HashSet;
use zerocad_core::parametric::{FaceRef, TopologyFaceRef};
use zerocad_core::{FeatureNode, FeatureType, ParametricGraph};

#[test]
fn missing_offset_trim_produces_unknown_bounds() {
    let wire = Wire::from_edges([
        Edge::between_points(Pnt::new(0., 0., 1.), Pnt::new(1., 0., 1.)),
        Edge::between_points(Pnt::new(1., 0., 1.), Pnt::new(1., 1., 1.)),
        Edge::between_points(Pnt::new(1., 1., 1.), Pnt::new(0., 1., 1.)),
        Edge::between_points(Pnt::new(0., 1., 1.), Pnt::new(0., 0., 1.)),
    ]);
    let base = GeomSurface::Plane(openrcad::geom::Plane::from_point_normal(
        Pnt::origin(),
        openrcad::foundation::Dir::dz(),
    ));
    let face = Face::new(
        Some(GeomSurface::Offset(openrcad::geom::OffsetSurface::new(
            base, 1.,
        ))),
        wire,
    );
    let solid = Solid::new(Shell::from_faces([face]));
    assert!(solid.try_conservative_bounding_box().is_none());
    assert!(zerocad_core::mock_kernel::solid_aabb(&solid).is_none());
}

#[test]
fn huge_patterns_reject_before_allocating_and_preserve_source() {
    use zerocad_core::parametric::{DiagnosticCode, EvaluationCancellation, EvaluationQuality};
    use zerocad_core::{AxisBase, PatternKind};
    for kind in [
        PatternKind::Linear {
            dir: AxisBase::X,
            spacing: 5.0,
            spacing_expr: None,
            count: u32::MAX,
        },
        PatternKind::Circular {
            axis: AxisBase::Z,
            total_angle_deg: 360.0,
            count: u32::MAX,
        },
    ] {
        let mut g = ParametricGraph::new();
        g.add_feature(FeatureNode {
            id: "box".into(),
            name: "box".into(),
            feature: FeatureType::Box {
                w: 2.,
                h: 2.,
                d: 2.,
            },
        });
        g.add_feature(FeatureNode {
            id: "pattern".into(),
            name: "pattern".into(),
            feature: FeatureType::Pattern {
                source: "box".into(),
                kind,
            },
        });
        g.add_dependency("box", "pattern");
        let token = EvaluationCancellation::new(
            0,
            std::sync::Arc::new(std::sync::atomic::AtomicU64::new(0)),
        );
        let result = g
            .evaluate_request(&HashSet::new(), EvaluationQuality::Final, &token)
            .unwrap();
        assert!(result
            .diagnostics
            .iter()
            .any(|d| d.feature_id == "pattern" && d.code == DiagnosticCode::parameter_invalid()));
        let bodies = g.evaluate_bodies(&HashSet::new()).unwrap();
        assert_eq!(bodies.len(), 1);
        assert_eq!(bodies[0].0, "box");
        assert!((bodies[0].1.mass_properties().unwrap().volume - 8.0).abs() < 1e-5);
    }
}

#[test]
fn broad_phase_bounds_round_outward() {
    let origin = Pnt::new(0.1, 0.2, 0.3);
    let solid =
        openrcad::primitives::make_box_operation(&origin, 1.00000003, 2.00000003, 3.00000003)
            .unwrap()
            .value;
    let (lo, hi) = solid
        .try_conservative_bounding_box()
        .unwrap()
        .corners()
        .unwrap();
    let (a, b) = zerocad_core::mock_kernel::solid_aabb(&solid).unwrap();
    for (i, (x, y)) in [(lo.x(), hi.x()), (lo.y(), hi.y()), (lo.z(), hi.z())]
        .into_iter()
        .enumerate()
    {
        assert!(a[i] as f64 <= x && b[i] as f64 >= y);
    }
}

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
    check_named_thread_selection(false, false);
    check_named_thread_selection(true, true);
}

#[test]
fn named_cylinder_wall_must_actually_thread() {
    check_named_thread_selection(true, false);
}

fn check_named_thread_selection(wall: bool, deleted: bool) {
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
        .find(|f| {
            if wall {
                f.topology.as_ref().and_then(|t| t.surface_kind.as_deref()) == Some("cylinder")
            } else {
                f.normal[1] > 0.9
            }
        })
        .unwrap();
    let mut face = FaceRef {
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
    if deleted {
        face.topology.as_mut().unwrap().face_id = Some("deleted-wall".into());
    }
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
    if wall && !deleted {
        assert!(warnings.is_empty(), "valid wall rejected: {warnings:?}");
        assert_ne!(
            before[0].1.vertices, after[0].1.vertices,
            "thread must change the wall"
        );
        return;
    }
    assert!(
        !warnings.is_empty(),
        "a named planar cap must be rejected, not redirected to a cylindrical wall"
    );
    assert_eq!(
        before[0].1.vertices, after[0].1.vertices,
        "rejected selection must preserve source"
    );
}
