//! Repro for two fillet failures on a line-drawn triangle prism with a
//! cylinder boss joined across its slanted (hypotenuse) face:
//!
//! 1. The vertical seam edge where the boss wall meets the slanted planar
//!    face: `candidate expands outside the original part bounds`.
//! 2. The prism's top edge on the slanted face (its adjacent top face has a
//!    circular bite from the boss): `rolling ball: local trimming currently
//!    supports simple outer-loop faces only`.

use std::collections::HashSet;

use zerocad_core::{
    CoordinateSystem, CornerKind, EdgeRef, ExtrudeMode, FeatureNode, FeatureType, ParametricGraph,
    SketchCurves, Vec3,
};

const PRISM_H: f32 = 20.0;
const BOSS_H: f32 = 30.0;
const R: f32 = 6.0;

fn ground() -> CoordinateSystem {
    CoordinateSystem::new(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
}

/// Triangle (0,0)-(40,0)-(0,30) extruded to `PRISM_H`, with a radius-6 circle
/// centered at the hypotenuse midpoint (20,15) joined as a boss to `BOSS_H`.
fn prism_boss_graph() -> ParametricGraph {
    let mut graph = ParametricGraph::new();

    let mut tri = SketchCurves::new();
    tri.add_line((0.0, 0.0), (40.0, 0.0));
    tri.add_line((40.0, 0.0), (0.0, 30.0));
    tri.add_line((0.0, 30.0), (0.0, 0.0));
    graph.add_feature(FeatureNode {
        id: "sketch_1".into(),
        name: "Triangle".into(),
        feature: FeatureType::Sketch {
            entity_ids: vec![],
            next_entity_id: 0,
            solver: None,
            cs: ground(),
            curves: tri,
            shapes: vec![],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
        },
    });
    graph.add_feature(FeatureNode {
        id: "extrude_2".into(),
        name: "Prism".into(),
        feature: FeatureType::Extrude {
            target: None,
            depth: PRISM_H,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
        },
    });
    graph.add_dependency("sketch_1", "extrude_2");

    let mut boss = SketchCurves::new();
    boss.add_circle((20.0, 15.0), R);
    graph.add_feature(FeatureNode {
        id: "sketch_3".into(),
        name: "Boss circle".into(),
        feature: FeatureType::Sketch {
            entity_ids: vec![],
            next_entity_id: 0,
            solver: None,
            cs: ground(),
            curves: boss,
            shapes: vec![],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
        },
    });
    graph.add_feature(FeatureNode {
        id: "extrude_4".into(),
        name: "Boss".into(),
        feature: FeatureType::Extrude {
            target: None,
            depth: BOSS_H,
            region_indices: vec![],
            mode: ExtrudeMode::Join,
            depth_expr: None,
        },
    });
    graph.add_dependency("sketch_3", "extrude_4");
    graph.add_dependency("extrude_2", "extrude_4");
    graph
}

fn eval_warnings(graph: &ParametricGraph) -> Vec<String> {
    let hidden = HashSet::new();
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&hidden)
        .expect("evaluate repro model");
    assert!(!bodies.is_empty(), "repro should produce a body");
    warnings
}

#[test]
fn fused_body_mesh_is_crack_free() {
    let graph = prism_boss_graph();
    let hidden = HashSet::new();
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&hidden)
        .expect("evaluate repro model");
    assert!(warnings.is_empty(), "unexpected warnings: {warnings:?}");
    let (_, mesh) = &bodies[0];

    let q = |i: usize| -> (i64, i64, i64) {
        let b = i * 6;
        let quant = |v: f32| (v as f64 * 10_000.0).round() as i64;
        (
            quant(mesh.vertices[b]),
            quant(mesh.vertices[b + 1]),
            quant(mesh.vertices[b + 2]),
        )
    };
    let mut edges: std::collections::HashMap<((i64, i64, i64), (i64, i64, i64)), u32> =
        std::collections::HashMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            let (ka, kb) = (q(tri[a] as usize), q(tri[b] as usize));
            let key = if ka <= kb { (ka, kb) } else { (kb, ka) };
            *edges.entry(key).or_insert(0) += 1;
        }
    }
    let cracks: Vec<_> = edges.iter().filter(|(_, &c)| c == 1).collect();
    for (k, _) in cracks.iter().take(20) {
        let p = |v: (i64, i64, i64)| {
            format!(
                "[{:.4},{:.4},{:.4}]",
                v.0 as f64 / 1e4,
                v.1 as f64 / 1e4,
                v.2 as f64 / 1e4
            )
        };
        println!("crack: {} - {}", p(k.0), p(k.1));
    }
    assert_eq!(cracks.len(), 0, "fused body render mesh must be crack-free");
}

#[test]
fn debug_dump_edges() {
    let graph = prism_boss_graph();
    let hidden = HashSet::new();
    let solids = graph.debug_kernel_solids(&hidden).expect("kernel solids");
    for (id, parts) in &solids {
        for (pi, part) in parts.iter().enumerate() {
            println!("body {id} part {pi}: watertight={}", part.is_watertight());
            for (fi, face) in part.shell().faces().iter().enumerate() {
                println!("  face {fi}: surface={:?}", face.surface().map(surf_kind));
                for wire in face.wires() {
                    let pts: Vec<String> = wire
                        .edges()
                        .iter()
                        .map(|e| {
                            let a = e.source().point();
                            let b = e.target().point();
                            format!(
                                "[{:.9},{:.9},{:.9}]->[{:.9},{:.9},{:.9}]{}",
                                a.x(),
                                a.y(),
                                a.z(),
                                b.x(),
                                b.y(),
                                b.z(),
                                e.curve().map(|c| curve_kind(&c)).unwrap_or("?")
                            )
                        })
                        .collect();
                    println!("    wire: {}", pts.join(" "));
                }
            }
        }
    }
}

fn surf_kind(s: &openrcad::geom::GeomSurface) -> &'static str {
    use openrcad::geom::GeomSurface::*;
    match s {
        Plane(_) => "Plane",
        Cylinder(_) => "Cylinder",
        Cone(_) => "Cone",
        Sphere(_) => "Sphere",
        Torus(_) => "Torus",
        _ => "Other",
    }
}

fn curve_kind(c: &openrcad::geom::GeomCurve) -> &'static str {
    use openrcad::geom::GeomCurve::*;
    match c {
        Line(_) => " L",
        Circle(_) => " C",
        _ => " O",
    }
}

/// The hypotenuse line is 3x + 4y = 120 with outward normal (0.6, 0.8, 0).
/// The boss circle is centered on it, so the wall∩plane seam edges are at
/// ±R along the line direction (-0.8, 0.6) from (20, 15).
fn seam_xy(sign: f32) -> (f32, f32) {
    (20.0 + sign * -0.8 * R, 15.0 + sign * 0.6 * R)
}

#[test]
fn oversized_boss_plane_seam_fillet_fails_safely() {
    let mut graph = prism_boss_graph();
    let (x, y) = seam_xy(1.0); // (15.2, 18.6)
    graph.add_feature(FeatureNode {
        id: "edgemod_5".into(),
        name: "Seam fillet".into(),
        feature: FeatureType::EdgeMod {
            target: "extrude_2".into(),
            edge: EdgeRef {
                p0: [x, y, 0.0],
                p1: [x, y, PRISM_H],
                n1: [0.6, 0.8, 0.0],
                n2: [-0.8, 0.6, 0.0],
                curve: None,
                topology: None,
            },
            // The valid seam blend is covered in OpenRCAD's active native
            // suite. Keep this app integration test bounded with a radius that
            // must be rejected before the Phase 3 replay path.
            dist: 30.0,
            dist_expr: None,
            kind: CornerKind::Fillet,
        },
    });
    graph.add_dependency("extrude_4", "edgemod_5");

    let warnings = eval_warnings(&graph);
    assert!(
        !warnings.is_empty(),
        "an oversized seam fillet must explain why it was rejected"
    );
    let solids = graph
        .debug_kernel_solids(&HashSet::new())
        .expect("fallback kernel solids");
    assert!(solids.iter().flat_map(|(_, parts)| parts).all(|solid| {
        solid.is_watertight() && solid.health_report().is_healthy() && solid.validate().is_ok()
    }));
    assert!(warnings.iter().all(|warning| !warning.is_empty()));
}

/// A plain line-drawn triangle prism (no boss), mirroring the GUI screenshots:
/// filleting the vertical edge at the sharp ACUTE corner (36.9° at (40,0),
/// between the bottom face and the hypotenuse) must apply without warnings. The
/// bug this guards: the rolling-ball center landed at radius/cos(θ/2) instead of
/// radius/sin(θ/2), so at a non-right corner the blend bulged outside the body
/// and the containment gate rejected it ("candidate vertex … outside the pre-
/// edge body").
fn prism_only_graph() -> ParametricGraph {
    let mut graph = ParametricGraph::new();
    let mut tri = SketchCurves::new();
    tri.add_line((0.0, 0.0), (40.0, 0.0));
    tri.add_line((40.0, 0.0), (0.0, 30.0));
    tri.add_line((0.0, 30.0), (0.0, 0.0));
    graph.add_feature(FeatureNode {
        id: "sketch_1".into(),
        name: "Triangle".into(),
        feature: FeatureType::Sketch {
            entity_ids: vec![],
            next_entity_id: 0,
            solver: None,
            cs: ground(),
            curves: tri,
            shapes: vec![],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
        },
    });
    graph.add_feature(FeatureNode {
        id: "extrude_2".into(),
        name: "Prism".into(),
        feature: FeatureType::Extrude {
            target: None,
            depth: PRISM_H,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
        },
    });
    graph.add_dependency("sketch_1", "extrude_2");
    graph
}

/// A REALLY sharp (~14°) corner through the GUI path: triangle
/// (0,0)-(40,0)-(0,10) has a 14° corner at (40,0); r=3 puts the contact feet
/// ~24.4 from the corner — still on both faces, so the fillet must apply.
#[test]
fn fillet_really_sharp_prism_corner_edge_gui_path() {
    let mut graph = ParametricGraph::new();
    let mut tri = SketchCurves::new();
    tri.add_line((0.0, 0.0), (40.0, 0.0));
    tri.add_line((40.0, 0.0), (0.0, 10.0));
    tri.add_line((0.0, 10.0), (0.0, 0.0));
    graph.add_feature(FeatureNode {
        id: "sketch_1".into(),
        name: "Sharp triangle".into(),
        feature: FeatureType::Sketch {
            entity_ids: vec![],
            next_entity_id: 0,
            solver: None,
            cs: ground(),
            curves: tri,
            shapes: vec![],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
        },
    });
    graph.add_feature(FeatureNode {
        id: "extrude_2".into(),
        name: "Prism".into(),
        feature: FeatureType::Extrude {
            target: None,
            depth: PRISM_H,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
        },
    });
    graph.add_dependency("sketch_1", "extrude_2");

    // Hypotenuse x/4 + y/... : from (40,0) to (0,10): 10x + 40y = 400 →
    // x + 4y = 40, outward normal (1,4)/√17.
    let s = (17.0f32).sqrt();
    graph.add_feature(FeatureNode {
        id: "edgemod_3".into(),
        name: "Sharp corner fillet".into(),
        feature: FeatureType::EdgeMod {
            target: "extrude_2".into(),
            edge: EdgeRef {
                p0: [40.0, 0.0, 0.0],
                p1: [40.0, 0.0, PRISM_H],
                n1: [0.0, -1.0, 0.0],
                n2: [1.0 / s, 4.0 / s, 0.0],
                curve: None,
                topology: None,
            },
            dist: 3.0,
            dist_expr: None,
            kind: CornerKind::Fillet,
        },
    });
    graph.add_dependency("extrude_2", "edgemod_3");

    let warnings = eval_warnings(&graph);
    assert!(
        warnings.is_empty(),
        "sharp corner fillet should apply cleanly, got {warnings:?}"
    );
}

#[test]
fn fillet_acute_prism_corner_edge_gui_path() {
    let mut graph = prism_only_graph();
    graph.add_feature(FeatureNode {
        id: "edgemod_3".into(),
        name: "Sharp corner fillet".into(),
        feature: FeatureType::EdgeMod {
            target: "extrude_2".into(),
            edge: EdgeRef {
                // Vertical edge at the 36.9° corner (40,0): bottom face y=0
                // (outward (0,-1,0)) meets the hypotenuse 3x+4y=120 (outward
                // (0.6,0.8,0)).
                p0: [40.0, 0.0, 0.0],
                p1: [40.0, 0.0, PRISM_H],
                n1: [0.0, -1.0, 0.0],
                n2: [0.6, 0.8, 0.0],
                curve: None,
                topology: None,
            },
            dist: 3.0,
            dist_expr: None,
            kind: CornerKind::Fillet,
        },
    });
    graph.add_dependency("extrude_2", "edgemod_3");

    let warnings = eval_warnings(&graph);
    assert!(
        warnings.is_empty(),
        "acute corner fillet should apply cleanly, got {warnings:?}"
    );
}

#[test]
fn fillet_prism_top_edge_with_boss_bite() {
    let mut graph = prism_boss_graph();
    // Top edge of the hypotenuse face from corner (40,0) toward the boss;
    // the boss pierces the top face, so this edge segment ends at the wall.
    let (bx, by) = seam_xy(-1.0); // (24.8, 11.4)
    graph.add_feature(FeatureNode {
        id: "edgemod_5".into(),
        name: "Top edge fillet".into(),
        feature: FeatureType::EdgeMod {
            target: "extrude_2".into(),
            edge: EdgeRef {
                p0: [40.0, 0.0, PRISM_H],
                p1: [bx, by, PRISM_H],
                n1: [0.0, 0.0, 1.0],
                n2: [0.6, 0.8, 0.0],
                curve: None,
                topology: None,
            },
            dist: 3.0,
            dist_expr: None,
            kind: CornerKind::Fillet,
        },
    });
    graph.add_dependency("extrude_4", "edgemod_5");

    let warnings = eval_warnings(&graph);
    assert!(
        warnings.is_empty(),
        "top edge fillet should apply cleanly, got {warnings:?}"
    );
}
