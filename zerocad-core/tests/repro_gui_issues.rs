//! Repro of the post-fix GUI reports (extruded-sketch bodies, not clean make_box):
//!  1. Filleting an extruded box edge — does it succeed, and how long does ONE
//!     evaluate take (the live preview runs this synchronously on the UI thread)?
//!  2. Cutting a box twice in a row — does the second cut still remove material?
//!  3. Cylinder boss join — are all surface triangles outward-facing (a face that
//!     ends up inward-normal gets back-face culled → "disappears" on screen)?

use std::collections::HashSet;
use std::time::Instant;
use zerocad_core::{
    CoordinateSystem, CornerKind, EdgeModScope, EdgeRef, ExtrudeMode, FeatureNode, FeatureType,
    ParametricGraph, SketchCurves, Vec3,
};

fn add_sketch(g: &mut ParametricGraph, id: &str, cs: CoordinateSystem, curves: SketchCurves) {
    g.add_feature(FeatureNode {
        id: id.into(),
        name: id.into(),
        feature: FeatureType::Sketch {
            cs,
            curves,
            shapes: vec![],
            corner_mods: vec![],
            on_face: true,
        },
    });
}

fn add_extrude(g: &mut ParametricGraph, id: &str, sketch: &str, depth: f32, mode: ExtrudeMode) {
    g.add_feature(FeatureNode {
        id: id.into(),
        name: id.into(),
        feature: FeatureType::Extrude {
            depth,
            region_indices: vec![],
            mode,
            depth_expr: None,
        },
    });
    g.add_dependency(sketch, id);
}

fn rect_sketch(min: (f32, f32), max: (f32, f32)) -> SketchCurves {
    let mut c = SketchCurves::new();
    c.add_rectangle(min, max);
    c
}

fn top_plane(h: f32) -> CoordinateSystem {
    CoordinateSystem::new(
        Vec3::new(0.0, 0.0, h),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
}

/// Count render-mesh boundary edges (used by exactly one triangle) across all
/// bodies — a closed solid must have zero (else it has cracks / disappearing
/// faces). Welds positions to 1e-4.
fn total_mesh_cracks(g: &ParametricGraph) -> usize {
    use std::collections::HashMap;
    let bodies = g.evaluate_bodies(&HashSet::new()).unwrap();
    let mut cracks = 0;
    for (_, m) in &bodies {
        let q = |i: usize| -> (i64, i64, i64) {
            let b = i * 6;
            let g = |v: f32| (v as f64 * 1e4).round() as i64;
            (g(m.vertices[b]), g(m.vertices[b + 1]), g(m.vertices[b + 2]))
        };
        let mut edges: HashMap<((i64, i64, i64), (i64, i64, i64)), u32> = HashMap::new();
        for t in m.indices.chunks_exact(3) {
            for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
                let (ka, kb) = (q(t[a] as usize), q(t[b] as usize));
                let k = if ka <= kb { (ka, kb) } else { (kb, ka) };
                *edges.entry(k).or_insert(0) += 1;
            }
        }
        cracks += edges.values().filter(|&&c| c == 1).count();
    }
    cracks
}

#[test]
fn extruded_box_fillet_succeeds_and_is_fast() {
    let mut g = ParametricGraph::new();
    add_sketch(
        &mut g,
        "sketch_1",
        top_plane(0.0),
        rect_sketch((0.0, 0.0), (40.0, 30.0)),
    );
    add_extrude(&mut g, "extrude_2", "sketch_1", 15.0, ExtrudeMode::NewBody);

    // The front-top edge of the extruded block: y=0, z=15, x in [0,40].
    let edge = EdgeRef {
        p0: [0.0, 0.0, 15.0],
        p1: [40.0, 0.0, 15.0],
        n1: [0.0, 0.0, 1.0],
        n2: [0.0, -1.0, 0.0],
        curve: None,
        topology: None,
    };
    g.add_feature(FeatureNode {
        id: "edgemod_3".into(),
        name: "Fillet1".into(),
        feature: FeatureType::EdgeMod {
            target: "extrude_2".into(),
            edge,
            dist: 4.0,
            dist_expr: None,
            scope: EdgeModScope::FullEdge,
            kind: CornerKind::Fillet,
        },
    });
    g.add_dependency("extrude_2", "edgemod_3");

    let t = Instant::now();
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    let ms = t.elapsed().as_millis();
    println!("extruded-box fillet: {ms} ms, warnings={warnings:?}");
    assert_eq!(bodies.len(), 1, "fillet keeps one body");
    assert!(
        warnings.is_empty(),
        "extruded-box edge fillet should succeed, got {warnings:?}"
    );

    // The round must actually appear: a chamfer/round shaves the sharp z=15,y=0
    // corner, so there must be vertices at intermediate z between the floor and
    // top that are NOT at y=0 (the rolled surface).
    let has_round = bodies[0]
        .1
        .vertices
        .chunks(6)
        .any(|v| v[2] > 11.5 && v[2] < 14.8 && v[1] > 0.05 && v[1] < 4.0);
    assert!(
        has_round,
        "fillet must produce a rolled surface between floor and top"
    );

    let cracks = total_mesh_cracks(&g);
    assert_eq!(
        cracks, 0,
        "filleted extruded box mesh has {cracks} crack edges"
    );

    // The live preview runs this on the UI thread per 0.05mm of drag; if one
    // evaluate is slow the app shows "(Not Responding)". Flag a slow path.
    assert!(
        ms < 1500,
        "single fillet evaluate took {ms} ms — too slow for a live UI-thread preview"
    );
}

#[test]
fn box_can_be_cut_twice() {
    let mut g = ParametricGraph::new();
    add_sketch(
        &mut g,
        "sketch_1",
        top_plane(0.0),
        rect_sketch((0.0, 0.0), (40.0, 30.0)),
    );
    add_extrude(&mut g, "extrude_2", "sketch_1", 15.0, ExtrudeMode::NewBody);

    // First cut: a 6x6 pocket near one corner, drilled down from the top.
    add_sketch(
        &mut g,
        "sketch_3",
        top_plane(15.0),
        rect_sketch((5.0, 5.0), (11.0, 11.0)),
    );
    add_extrude(&mut g, "extrude_4", "sketch_3", -8.0, ExtrudeMode::Cut);
    g.add_dependency("extrude_2", "extrude_4");

    let after_one = g.evaluate_bodies(&HashSet::new()).unwrap();
    assert_eq!(after_one.len(), 1);
    let v_after_one = after_one[0].1.vertices.len();

    // Second cut: a different 6x6 pocket near the opposite corner.
    add_sketch(
        &mut g,
        "sketch_5",
        top_plane(15.0),
        rect_sketch((29.0, 19.0), (35.0, 25.0)),
    );
    add_extrude(&mut g, "extrude_6", "sketch_5", -8.0, ExtrudeMode::Cut);
    g.add_dependency("extrude_4", "extrude_6");

    let (after_two, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_eq!(after_two.len(), 1, "two cuts keep one body");
    assert!(
        warnings.is_empty(),
        "second cut should also remove material cleanly, got {warnings:?}"
    );

    // The second pocket floor sits at z≈7; assert a vertex appeared there in the
    // opposite corner region (proof the 2nd cut actually carved material).
    let second_floor = after_two[0].1.vertices.chunks(6).any(|v| {
        v[0] > 28.0 && v[0] < 36.0 && v[1] > 18.0 && v[1] < 26.0 && v[2] > 6.0 && v[2] < 8.0
    });
    assert!(
        second_floor,
        "the SECOND cut must carve its own pocket floor (z≈7 in the far corner)"
    );
    assert!(
        after_two[0].1.vertices.len() > v_after_one,
        "two pockets must have more geometry than one"
    );
}

#[test]
fn cylinder_boss_join_has_no_inward_faces() {
    // The reported render glitch: faces drop out of the boss-join. A dropped face
    // = a surface triangle whose stored normal points INTO the solid (back-face
    // culled). Verify the shell is a correctly-oriented closed manifold (stored
    // normals agree with outward winding, positive volume, no cracks).
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "box_1".into(),
        name: "Box".into(),
        feature: FeatureType::Box {
            w: 40.0,
            h: 30.0,
            d: 15.0,
        },
    });
    let mut c = SketchCurves::new();
    c.add_circle((20.0, 15.0), 6.0);
    add_sketch(&mut g, "sketch_2", top_plane(15.0), c);
    add_extrude(&mut g, "extrude_3", "sketch_2", 12.0, ExtrudeMode::Join);
    g.add_dependency("box_1", "extrude_3");

    let bodies = g.evaluate_bodies(&HashSet::new()).unwrap();
    assert_eq!(bodies.len(), 1);
    let m = &bodies[0].1;

    // A correctly-oriented closed mesh — the condition under which the renderer's
    // back-face cull shows exactly the front faces (nothing "disappears") — has:
    //  (a) every triangle's stored normal agreeing with its winding normal, and
    //  (b) a positive signed volume (the whole shell winds outward, not inside-out).
    // Both are exactly what `orient_mesh_outward` must guarantee. A vertex-centroid
    // "is it inward" test is unreliable for a non-convex boss (the centroid sits
    // above the box top), so we use this robust pair instead.
    let mut disagree = 0;
    let mut vol6 = 0.0f64;
    for t in m.indices.chunks_exact(3) {
        let p = |i: u32| {
            let b = i as usize * 6;
            [
                m.vertices[b] as f64,
                m.vertices[b + 1] as f64,
                m.vertices[b + 2] as f64,
            ]
        };
        let vn = |i: u32| {
            let b = i as usize * 6;
            [
                m.vertices[b + 3] as f64,
                m.vertices[b + 4] as f64,
                m.vertices[b + 5] as f64,
            ]
        };
        let a = p(t[0]);
        let b = p(t[1]);
        let d = p(t[2]);
        let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let w = [d[0] - a[0], d[1] - a[1], d[2] - a[2]];
        let fn_ = [
            u[1] * w[2] - u[2] * w[1],
            u[2] * w[0] - u[0] * w[2],
            u[0] * w[1] - u[1] * w[0],
        ];
        let navg = [
            (vn(t[0])[0] + vn(t[1])[0] + vn(t[2])[0]) / 3.0,
            (vn(t[0])[1] + vn(t[1])[1] + vn(t[2])[1]) / 3.0,
            (vn(t[0])[2] + vn(t[1])[2] + vn(t[2])[2]) / 3.0,
        ];
        if fn_[0] * navg[0] + fn_[1] * navg[1] + fn_[2] * navg[2] < 0.0 {
            disagree += 1;
        }
        vol6 += a[0] * (b[1] * d[2] - b[2] * d[1]) - a[1] * (b[0] * d[2] - b[2] * d[0])
            + a[2] * (b[0] * d[1] - b[1] * d[0]);
    }
    println!(
        "boss-join: {} triangles, {disagree} normal/winding-disagreements, vol6={vol6:.1}",
        m.indices.len() / 3
    );
    assert_eq!(disagree, 0, "{disagree} triangles' stored normal disagrees with winding → they back-face cull and disappear");
    assert!(
        vol6 > 0.0,
        "the boss-join shell is wound inside-out (vol6={vol6})"
    );
    // Manifold stats: weld by quantized position, count edge usage.
    use std::collections::HashMap;
    let q = |i: u32| -> (i64, i64, i64) {
        let b = i as usize * 6;
        let g = |v: f32| (v as f64 * 1e4).round() as i64;
        (g(m.vertices[b]), g(m.vertices[b + 1]), g(m.vertices[b + 2]))
    };
    let mut edges: HashMap<((i64, i64, i64), (i64, i64, i64)), u32> = HashMap::new();
    for t in m.indices.chunks_exact(3) {
        for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            let (ka, kb) = (q(t[a]), q(t[b]));
            let k = if ka <= kb { (ka, kb) } else { (kb, ka) };
            *edges.entry(k).or_insert(0) += 1;
        }
    }
    let boundary = edges.values().filter(|&&c| c == 1).count();
    let nonmanifold = edges.values().filter(|&&c| c > 2).count();
    println!("boss-join: {boundary} boundary edges, {nonmanifold} non-manifold edges");
    assert_eq!(
        boundary, 0,
        "boss-join mesh has {boundary} crack edges (white lines)"
    );
    assert_eq!(
        nonmanifold, 0,
        "boss-join mesh has {nonmanifold} non-manifold edges"
    );
}

#[test]
fn fillet_preview_drag_is_responsive() {
    use std::time::Instant;
    // Build the GUI-style body: extruded box, then simulate a live fillet drag by
    // re-evaluating the whole graph (with a fresh edge-mod node) at many radii —
    // exactly what the UI thread does each preview frame. None may be slow enough
    // to freeze the UI ("Not Responding").
    let mut base = ParametricGraph::new();
    add_sketch(
        &mut base,
        "sketch_1",
        top_plane(0.0),
        rect_sketch((0.0, 0.0), (40.0, 30.0)),
    );
    add_extrude(
        &mut base,
        "extrude_2",
        "sketch_1",
        15.0,
        ExtrudeMode::NewBody,
    );

    let mut worst = 0u128;
    // Drag from 0.5mm up to an oversize 9mm (radius > half the 15mm height fails).
    for step in 0..40 {
        let r = 0.5 + step as f32 * 0.22;
        let mut g = base.clone();
        let edge = EdgeRef {
            p0: [0.0, 0.0, 15.0],
            p1: [40.0, 0.0, 15.0],
            n1: [0.0, 0.0, 1.0],
            n2: [0.0, -1.0, 0.0],
            curve: None,
            topology: None,
        };
        g.add_feature(FeatureNode {
            id: "edgemod_preview".into(),
            name: "Preview".into(),
            feature: FeatureType::EdgeMod {
                target: "extrude_2".into(),
                edge,
                dist: r,
                dist_expr: None,
                scope: EdgeModScope::FullEdge,
                kind: CornerKind::Fillet,
            },
        });
        g.add_dependency("extrude_2", "edgemod_preview");
        let t = Instant::now();
        let _ = g.evaluate_bodies_draft(&HashSet::new());
        let ms = t.elapsed().as_millis();
        worst = worst.max(ms);
    }
    println!("fillet drag: worst single preview eval = {worst} ms");
    assert!(
        worst < 1500,
        "a preview frame took {worst} ms — would freeze the UI"
    );
}

// Several realistic "second cut" scenarios to find the "cut works once" failure.
fn cut_scenario(
    name: &str,
    second_cut: impl FnOnce(&mut ParametricGraph),
) -> (usize, Vec<String>, f32) {
    let mut g = ParametricGraph::new();
    add_sketch(
        &mut g,
        "sketch_1",
        top_plane(0.0),
        rect_sketch((0.0, 0.0), (40.0, 30.0)),
    );
    add_extrude(&mut g, "extrude_2", "sketch_1", 15.0, ExtrudeMode::NewBody);
    // First cut: a pocket near a corner.
    add_sketch(
        &mut g,
        "sketch_3",
        top_plane(15.0),
        rect_sketch((5.0, 5.0), (11.0, 11.0)),
    );
    add_extrude(&mut g, "extrude_4", "sketch_3", -8.0, ExtrudeMode::Cut);
    g.add_dependency("extrude_2", "extrude_4");
    second_cut(&mut g);
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    let vlen = bodies.first().map(|b| b.1.vertices.len()).unwrap_or(0);
    println!(
        "CUT[{name}]: bodies={}, warnings={warnings:?}",
        bodies.len()
    );
    (bodies.len(), warnings, vlen as f32)
}

#[test]
fn second_cut_scenarios() {
    // A) Second cut: another pocket, far corner (the known-good baseline).
    let (_, w_a, _) = cut_scenario("far-corner", |g| {
        add_sketch(
            g,
            "s5",
            top_plane(15.0),
            rect_sketch((29.0, 19.0), (35.0, 25.0)),
        );
        add_extrude(g, "extrude_6", "s5", -8.0, ExtrudeMode::Cut);
        g.add_dependency("extrude_4", "extrude_6");
    });

    // B) Second cut: a through-hole this time.
    let (_, w_b, _) = cut_scenario("through-hole", |g| {
        add_sketch(
            g,
            "s5",
            top_plane(15.0),
            rect_sketch((25.0, 12.0), (31.0, 18.0)),
        );
        add_extrude(g, "extrude_6", "s5", -20.0, ExtrudeMode::Cut);
        g.add_dependency("extrude_4", "extrude_6");
    });

    // C) Second cut whose profile reaches the BODY EDGE (tool side wall coplanar
    //    with the body's side face — the classic coplanar cut-killer).
    let (_, w_c, _) = cut_scenario("edge-coplanar", |g| {
        add_sketch(
            g,
            "s5",
            top_plane(15.0),
            rect_sketch((30.0, 0.0), (40.0, 10.0)),
        );
        add_extrude(g, "extrude_6", "s5", -8.0, ExtrudeMode::Cut);
        g.add_dependency("extrude_4", "extrude_6");
    });

    // D) Second cut at the SAME footprint as the first but deeper.
    let (_, w_d, _) = cut_scenario("same-spot-deeper", |g| {
        add_sketch(
            g,
            "s5",
            top_plane(15.0),
            rect_sketch((5.0, 5.0), (11.0, 11.0)),
        );
        add_extrude(g, "extrude_6", "s5", -14.0, ExtrudeMode::Cut);
        g.add_dependency("extrude_4", "extrude_6");
    });

    let mut failures = vec![];
    for (n, w) in [
        ("far-corner", &w_a),
        ("through-hole", &w_b),
        ("edge-coplanar", &w_c),
        ("same-spot-deeper", &w_d),
    ] {
        if !w.is_empty() {
            failures.push(format!("{n}: {w:?}"));
        }
    }
    assert!(
        failures.is_empty(),
        "second-cut failures:\n{}",
        failures.join("\n")
    );
}

#[test]
fn cut_with_positive_depth_on_top_face_still_cuts() {
    // The "cut does nothing" bug: a sketch on the top face cut with a POSITIVE
    // depth sweeps the cutter UP, away from the body — it should still carve a
    // pocket DOWN into the body (opposite-direction fallback).
    let mut g = ParametricGraph::new();
    add_sketch(
        &mut g,
        "sketch_1",
        top_plane(0.0),
        rect_sketch((0.0, 0.0), (40.0, 30.0)),
    );
    add_extrude(&mut g, "extrude_2", "sketch_1", 15.0, ExtrudeMode::NewBody);
    // Pocket sketch on the top face (z=15), cut with a POSITIVE +8 depth.
    add_sketch(
        &mut g,
        "sketch_3",
        top_plane(15.0),
        rect_sketch((10.0, 10.0), (20.0, 20.0)),
    );
    add_extrude(&mut g, "extrude_4", "sketch_3", 8.0, ExtrudeMode::Cut);
    g.add_dependency("extrude_2", "extrude_4");

    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_eq!(bodies.len(), 1, "positive-depth cut keeps one body");
    assert!(
        warnings.is_empty(),
        "the cut should carve cleanly, got {warnings:?}"
    );
    // A pocket floor must appear ~8mm down (z≈7). A plain block has vertices only
    // at z=0 and z=15, so any vertex at z≈7 in the pocket footprint proves the cut
    // carved downward into the body (the flat floor only has corner vertices, so
    // we accept the footprint's 10/20 edges too).
    let floor = bodies[0].1.vertices.chunks(6).any(|v| {
        v[0] >= 9.5 && v[0] <= 20.5 && v[1] >= 9.5 && v[1] <= 20.5 && v[2] > 6.0 && v[2] < 8.0
    });
    assert!(
        floor,
        "a positive-depth top-face cut must still carve a pocket into the body"
    );
}
