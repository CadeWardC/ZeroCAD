//! A sketch of a rectangle plus a circle sitting ON a corner, extruded as the
//! material region (rect minus the circular bite), must produce the SAME
//! geometry as a box with a cylinder cut from that corner: an analytic
//! cylindrical bite wall, not a plain box (and not a faceted prism).

use std::collections::HashSet;

use openrcad::geom::GeomSurface;
use zerocad_core::{
    detect_regions, CoordinateSystem, ExtrudeMode, FeatureNode, FeatureType, ParametricGraph,
    SketchCurves, Vec3,
};

fn xy() -> CoordinateSystem {
    CoordinateSystem::new(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
}

fn corner_bite_graph_on(
    cs: CoordinateSystem,
    center: (f32, f32),
    radius: f32,
    bite_probe: (f32, f32),
) -> ParametricGraph {
    let mut c = SketchCurves::new();
    c.add_rectangle((0.0, 0.0), (30.0, 30.0));
    c.add_circle(center, radius);

    let regions = detect_regions(&c);
    let region = regions
        .iter()
        .position(|r| r.contains((15.0, 15.0)) && !r.contains(bite_probe))
        .unwrap_or_else(|| {
            panic!(
                "expected a material region (rect minus corner bite); got {} regions: {:?}",
                regions.len(),
                regions
                    .iter()
                    .map(|r| (r.area, r.contains((15.0, 15.0)), r.contains(bite_probe)))
                    .collect::<Vec<_>>()
            )
        });

    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "sketch_1".into(),
        name: "S".into(),
        feature: FeatureType::Sketch {
            cs,
            curves: c,
            shapes: vec![],
            corner_mods: vec![],
            on_face: false,
        },
    });
    g.add_feature(FeatureNode {
        id: "extrude_2".into(),
        name: "E".into(),
        feature: FeatureType::Extrude {
            depth: 10.0,
            region_indices: vec![region],
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
        },
    });
    g.add_dependency("sketch_1", "extrude_2");
    g
}

/// World-space probe for a sketch-plane point at extrusion mid-depth.
fn world(cs: &CoordinateSystem, p: (f32, f32), h: f32) -> openrcad::foundation::Pnt {
    let o = cs.origin;
    let u = cs.u;
    let v = cs.v;
    // The extrude feature travels along the STORED plane normal (cs.n) — on the
    // left-handed ground plane that is +Y even though u × v = −Y.
    let n = cs.n;
    openrcad::foundation::Pnt::new(
        (o.x + u.x * p.0 + v.x * p.1 + n.x * h) as f64,
        (o.y + u.y * p.0 + v.y * p.1 + n.y * h) as f64,
        (o.z + u.z * p.0 + v.z * p.1 + n.z * h) as f64,
    )
}

fn assert_corner_bite_solid_probe(
    g: &ParametricGraph,
    cs: &CoordinateSystem,
    bite_probe: (f32, f32),
    label: &str,
) {
    let solids = g.debug_kernel_solids(&HashSet::new()).unwrap();
    let solid = &solids[0].1[0];
    let faces = solid.shell().faces();
    let cyl = faces
        .iter()
        .filter(|f| matches!(f.surface(), Some(GeomSurface::Cylinder(_))))
        .count();

    // Material must be gone inside the bite and present outside it.
    let inside_bite =
        openrcad::algo::boolean::point_in_solid(&world(cs, bite_probe, 5.0), solid);
    let in_material =
        openrcad::algo::boolean::point_in_solid(&world(cs, (15.0, 15.0), 5.0), solid);

    assert!(
        in_material,
        "{label}: material sample (15,15)@5 must be inside the solid"
    );
    assert!(
        !inside_bite,
        "{label}: bite sample (1.5,1.5)@5 must be OUTSIDE the solid — the corner bite was dropped (plain box extruded)"
    );
    assert!(
        cyl >= 1,
        "{label}: the bite wall must be an analytic cylinder face, got {cyl} cylinders across {} faces",
        faces.len()
    );
}

#[test]
fn circle_centered_on_rect_corner_extrudes_with_bite() {
    // Circle center exactly ON the corner vertex (the user's screenshot case).
    let cs = xy();
    let g = corner_bite_graph_on(cs.clone(), (0.0, 0.0), 8.0, (1.5, 1.5));
    assert_corner_bite_solid_probe(&g, &cs, (1.5, 1.5), "center-on-corner");
}

#[test]
fn circle_straddling_rect_corner_extrudes_with_bite() {
    // Circle center just outside the rectangle, overlapping the corner.
    let cs = xy();
    let g = corner_bite_graph_on(cs.clone(), (-2.0, -2.0), 8.0, (1.5, 1.5));
    assert_corner_bite_solid_probe(&g, &cs, (1.5, 1.5), "center-outside-corner");
}

#[test]
fn ground_plane_corner_bite_extrudes_with_bite() {
    // The GUI ground/top plane (CoordinateSystem::XZ) is LEFT-handed — the
    // screenshot case. The bite must survive there too.
    let cs = CoordinateSystem::XZ;
    let g = corner_bite_graph_on(cs.clone(), (0.0, 0.0), 8.0, (1.5, 1.5));
    assert_corner_bite_solid_probe(&g, &cs, (1.5, 1.5), "ground-plane-corner");
}

#[test]
fn fillet_runs_into_corner_bite() {
    // The whole point of canonical geometry: the extruded corner-bite body must
    // accept a fillet on the top edge that terminates INTO the bite wall — the
    // same as box + cylinder-cut + fillet.
    let cs = xy();
    let mut g = corner_bite_graph_on(cs.clone(), (0.0, 0.0), 8.0, (1.5, 1.5));
    // Top rim edge along y=0, trimmed by the bite: runs x=8..30 at z=10.
    let edge = zerocad_core::EdgeRef {
        p0: [8.0, 0.0, 10.0],
        p1: [30.0, 0.0, 10.0],
        n1: [0.0, 0.0, 1.0],
        n2: [0.0, -1.0, 0.0],
        curve: None,
        topology: None,
    };
    let replay = g.edge_mod_replay_intent_for_edge("extrude_2", &edge, &HashSet::new());
    g.add_feature(FeatureNode {
        id: "em_3".into(),
        name: "Fillet".into(),
        feature: FeatureType::EdgeMod {
            target: "extrude_2".into(),
            edge,
            dist: 1.5,
            dist_expr: None,
            scope: zerocad_core::EdgeModScope::FullEdge,
            replay,
            kind: zerocad_core::CornerKind::Fillet,
        },
    });
    g.add_dependency("extrude_2", "em_3");

    let (_bodies, warnings, statuses) = g
        .evaluate_bodies_with_status(&HashSet::new())
        .expect("fillet into corner bite evaluates");
    assert!(
        statuses.iter().all(|s| !s.is_unresolved()),
        "fillet into corner bite must not fail: {statuses:?} (warnings: {warnings:?})"
    );
    // The bite must survive the fillet.
    assert_corner_bite_solid_probe(&g, &cs, (1.5, 1.5), "fillet-into-corner-bite");
}

#[test]
fn circle_straddling_one_edge_still_extrudes_with_bite() {
    // Regression guard: the long-supported single-edge straddle keeps working.
    let cs = xy();
    let g = corner_bite_graph_on(cs.clone(), (15.0, 0.0), 8.0, (15.0, 1.5));
    let solids = g.debug_kernel_solids(&HashSet::new()).unwrap();
    let solid = &solids[0].1[0];
    let cyl = solid
        .shell()
        .faces()
        .iter()
        .filter(|f| matches!(f.surface(), Some(GeomSurface::Cylinder(_))))
        .count();
    assert!(cyl >= 1, "edge-straddle bite keeps its analytic wall");
    assert_corner_bite_solid_probe(&g, &cs, (15.0, 1.5), "edge-straddle");
}
