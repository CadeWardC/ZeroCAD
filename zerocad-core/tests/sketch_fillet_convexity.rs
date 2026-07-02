//! An extruded rounded-rect corner must be CONVEX (material rounded off), never
//! inverted (a scalloped bite). Radius/watertight checks alone cannot tell the
//! two apart - both are healthy 10-face solids with radius-r cylinders - so
//! this probes material presence just inside/outside each fillet arc.
//!
//! Regression for the "inverted corners on commit" bug: the ground/top plane
//! constant (CoordinateSystem::XZ) is a LEFT-handed frame (u=X, v=Z, stored
//! n=+Y but X*Z=-Y), and emit_arc_edges used the stored normal as the arc
//! sense reference - flipping every fillet arc into a scallop on ground-plane
//! sketches while XY-plane tests stayed green.

use std::collections::HashSet;

use zerocad_core::{
    CoordinateSystem, CornerKind, CornerMod, Dimension, ExtrudeMode, FeatureNode, FeatureType,
    ParametricGraph, SketchCurves, Vec3,
};

fn rounded_rect_graph(w: f32, h: f32, radius: f32, depth: f32) -> ParametricGraph {
    rounded_rect_graph_on(
        w,
        h,
        radius,
        depth,
        CoordinateSystem::new(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        ),
    )
}

fn rounded_rect_graph_on(
    w: f32,
    h: f32,
    radius: f32,
    depth: f32,
    cs: CoordinateSystem,
) -> ParametricGraph {
    let mut g = ParametricGraph::new();
    let mut curves = SketchCurves::new();
    curves.add_rectangle((0.0, 0.0), (w, h));
    let corner_mods = [(0.0f32, 0.0f32), (w, 0.0), (w, h), (0.0, h)]
        .iter()
        .map(|&at| CornerMod {
            at,
            radius: Dimension::literal(radius),
            kind: CornerKind::Fillet,
        })
        .collect();
    g.add_feature(FeatureNode {
        id: "sketch_1".into(),
        name: "S".into(),
        feature: FeatureType::Sketch {
            cs,
            curves,
            shapes: vec![],
            corner_mods,
            on_face: false,
        },
    });
    g.add_feature(FeatureNode {
        id: "extrude_2".into(),
        name: "E".into(),
        feature: FeatureType::Extrude {
            depth,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
        },
    });
    g.add_dependency("sketch_1", "extrude_2");
    g
}

#[test]
fn extruded_fillet_corners_are_convex() {
    for &(w, h, r, depth) in &[
        (20.0f32, 12.0, 3.0, 10.0),
        (20.0, 12.0, 5.0, 10.0),
        (14.0, 11.0, 4.0, 8.0),
        (20.0, 12.0, 3.0, -10.0),
    ] {
        let g = rounded_rect_graph(w, h, r, depth);
        let solids = g.debug_kernel_solids(&HashSet::new()).unwrap();
        let solid = &solids[0].1[0];

        let zmid = if depth > 0.0 { depth as f64 / 2.0 } else { depth as f64 / 2.0 };
        // Fillet center of the (0,0) corner is (r, r). Just inside the arc
        // toward the corner: c + 0.98r * (-1/sqrt2, -1/sqrt2). Material must be
        // there for a CONVEX roundover; a scalloped (inverted) corner has none.
        let s = (r as f64) * 0.98 / std::f64::consts::SQRT_2;
        let inside = openrcad::foundation::Pnt::new(r as f64 - s, r as f64 - s, zmid);
        // Just OUTSIDE the arc (past the roundover toward the sharp corner):
        // must be empty for both correct and scalloped bodies.
        let s2 = (r as f64) * 1.05 / std::f64::consts::SQRT_2;
        let outside = openrcad::foundation::Pnt::new(r as f64 - s2, r as f64 - s2, zmid);
        // Rectangle interior far from corners: must always be material.
        let center = openrcad::foundation::Pnt::new(w as f64 / 2.0, h as f64 / 2.0, zmid);

        let inside_ok = openrcad::algo::boolean::point_in_solid(&inside, solid);
        let outside_ok = !openrcad::algo::boolean::point_in_solid(&outside, solid);
        let center_ok = openrcad::algo::boolean::point_in_solid(&center, solid);
        assert!(inside_ok, "w={w} h={h} r={r} depth={depth}: fillet corner must keep material just inside the arc (inverted/scalloped arc otherwise)");
        assert!(outside_ok, "w={w} h={h} r={r} depth={depth}: no material beyond the roundover");
        assert!(center_ok, "w={w} h={h} r={r} depth={depth}: solid center must be material");
    }
}

#[test]
fn ground_plane_fillet_corners_are_convex() {
    // The GUI's ground/top plane: u=X, v=Z, stored n=+Y — a LEFT-handed frame
    // (X × Z = −Y). Sketch 2D (a, b) maps to world (a, 0, b).
    let cs = CoordinateSystem::XZ;
    let (w, h, r, depth) = (14.0f32, 11.0, 4.0, 8.0);
    let g = rounded_rect_graph_on(w, h, r, depth, cs);
    let solids = g.debug_kernel_solids(&std::collections::HashSet::new()).unwrap();
    let solid = &solids[0].1[0];

    // Corner (0,0) in sketch space → world (x, y, z) = (u*a + v*b) with the
    // extrusion along the sweep axis; probe at mid-height of the actual solid.
    let bb = {
        let shell = solid.shell();
        let faces = shell.faces();
        let mut min = [f64::MAX; 3];
        let mut max = [f64::MIN; 3];
        for f in faces.iter() {
            for w in f.wires() {
                for e in w.edges() {
                    for p in [e.start().point(), e.end().point()] {
                        for (k, val) in [p.x(), p.y(), p.z()].into_iter().enumerate() {
                            min[k] = min[k].min(val);
                            max[k] = max[k].max(val);
                        }
                    }
                }
            }
        }
        (min, max)
    };
    let ymid = 0.5 * (bb.0[1] + bb.1[1]);
    let s = (r as f64) * 0.98 / std::f64::consts::SQRT_2;
    // sketch (r - s, r - s) → world (r - s, ymid, r - s)
    let inside = openrcad::foundation::Pnt::new(r as f64 - s, ymid, r as f64 - s);
    let s2 = (r as f64) * 1.05 / std::f64::consts::SQRT_2;
    let outside = openrcad::foundation::Pnt::new(r as f64 - s2, ymid, r as f64 - s2);
    let center = openrcad::foundation::Pnt::new(w as f64 / 2.0, ymid, h as f64 / 2.0);

    let inside_ok = openrcad::algo::boolean::point_in_solid(&inside, solid);
    let outside_ok = !openrcad::algo::boolean::point_in_solid(&outside, solid);
    let center_ok = openrcad::algo::boolean::point_in_solid(&center, solid);
    assert!(inside_ok, "ground plane: fillet corner must keep material just inside the arc");
    assert!(outside_ok, "ground plane: no material beyond the roundover");
    assert!(center_ok, "ground plane: solid center must be material");
}
