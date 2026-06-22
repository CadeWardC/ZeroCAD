//! Faithful repro of the GUI's "fillet then fillet again" flow: build an
//! extruded box, fillet the front-top edge, then capture the SURVIVING
//! perpendicular (right-top) edge from the post-fillet-1 mesh exactly as the
//! GUI's `edge_ref_from` does (free ends of the edge group), and apply a second
//! fillet through the real parametric history. The second fillet must take
//! effect (no warning) and the round must actually appear.

use std::collections::{HashMap, HashSet};
use zerocad_core::{
    CornerKind, CoordinateSystem, EdgeRef, ExtrudeMode, FeatureNode, FeatureType, MockMesh,
    ParametricGraph, SketchCurves, Vec3,
};

fn add_sketch(g: &mut ParametricGraph, id: &str, cs: CoordinateSystem, curves: SketchCurves) {
    g.add_feature(FeatureNode {
        id: id.into(),
        name: id.into(),
        feature: FeatureType::Sketch { cs, curves, shapes: vec![], corner_mods: vec![], on_face: true },
    });
}

fn add_extrude(g: &mut ParametricGraph, id: &str, sketch: &str, depth: f32, mode: ExtrudeMode) {
    g.add_feature(FeatureNode {
        id: id.into(),
        name: id.into(),
        feature: FeatureType::Extrude { depth, region_indices: vec![], mode, depth_expr: None },
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

/// Replicate the GUI's `edge_ref_from` faithfully: find the mesh edge segment
/// closest to `probe` that lies on the target corner line, take its WHOLE
/// `edge_groups` group, and return the group's two free ends + the first
/// segment's adjacent face normals — exactly the EdgeRef the GUI would capture.
fn capture_edge(mesh: &MockMesh, pred: impl Fn([f32; 3]) -> bool) -> Option<EdgeRef> {
    let vpos = |seg: usize, which: usize| -> [f32; 3] {
        let vi = mesh.edge_indices[seg * 2 + which] as usize * 3;
        [mesh.edge_vertices[vi], mesh.edge_vertices[vi + 1], mesh.edge_vertices[vi + 2]]
    };
    let seg_count = mesh.edge_indices.len() / 2;
    // The segment the user clicked: pick one that lies on the target corner line.
    let clicked = (0..seg_count).find(|&s| pred(vpos(s, 0)) && pred(vpos(s, 1)))?;
    // Faithful grouping: gather every segment sharing the clicked segment's group.
    let segs: Vec<usize> = if mesh.edge_groups.is_empty() {
        vec![clicked]
    } else {
        let g = mesh.edge_groups[clicked];
        (0..seg_count).filter(|&s| mesh.edge_groups.get(s).copied() == Some(g)).collect()
    };
    println!(
        "group of clicked right-top segment has {} chords; sample ends:",
        segs.len()
    );
    for &s in segs.iter().take(8) {
        let a = vpos(s, 0);
        let b = vpos(s, 1);
        println!("  ({:.2},{:.2},{:.2})->({:.2},{:.2},{:.2})", a[0], a[1], a[2], b[0], b[1], b[2]);
    }
    let &first = segs.first()?;

    let qkey = |p: [f32; 3]| -> (i64, i64, i64) {
        let q = |v: f32| (v as f64 * 10_000.0).round() as i64;
        (q(p[0]), q(p[1]), q(p[2]))
    };
    let mut uses: HashMap<(i64, i64, i64), (u32, [f32; 3])> = HashMap::new();
    for &s in &segs {
        for w in 0..2 {
            let p = vpos(s, w);
            uses.entry(qkey(p)).or_insert((0, p)).0 += 1;
        }
    }
    let mut ends: Vec<[f32; 3]> = uses.values().filter(|(c, _)| *c == 1).map(|(_, p)| *p).collect();
    ends.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let (p0, p1) = if ends.len() >= 2 { (ends[0], ends[1]) } else { (vpos(first, 0), vpos(first, 1)) };

    let fo = first * 6;
    let (n1, n2) = if mesh.edge_face_normals.len() >= fo + 6 {
        (
            [mesh.edge_face_normals[fo], mesh.edge_face_normals[fo + 1], mesh.edge_face_normals[fo + 2]],
            [mesh.edge_face_normals[fo + 3], mesh.edge_face_normals[fo + 4], mesh.edge_face_normals[fo + 5]],
        )
    } else {
        ([0.0, 0.0, 1.0], [1.0, 0.0, 0.0])
    };
    Some(EdgeRef { p0, p1, n1, n2 })
}

#[test]
fn fillet_then_fillet_perpendicular_edge() {
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", top_plane(0.0), rect_sketch((0.0, 0.0), (40.0, 30.0)));
    add_extrude(&mut g, "extrude_2", "sketch_1", 15.0, ExtrudeMode::NewBody);

    // Fillet 1: front-top edge (y=0, z=15, x in [0,40]).
    g.add_feature(FeatureNode {
        id: "edgemod_3".into(),
        name: "Fillet1".into(),
        feature: FeatureType::EdgeMod {
            target: "extrude_2".into(),
            edge: EdgeRef { p0: [0.0, 0.0, 15.0], p1: [40.0, 0.0, 15.0], n1: [0.0, 0.0, 1.0], n2: [0.0, -1.0, 0.0] },
            dist: 4.0,
            dist_expr: None,
            kind: CornerKind::Fillet,
        },
    });
    g.add_dependency("extrude_2", "edgemod_3");

    let (bodies1, w1) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert!(w1.is_empty(), "fillet 1 should succeed, got {w1:?}");
    let mesh1 = &bodies1[0].1;

    // Capture the surviving right-top edge (x≈40, z≈15) exactly as the GUI would.
    let cap = capture_edge(mesh1, |p| (p[0] - 40.0).abs() < 1e-3 && (p[2] - 15.0).abs() < 1e-3);
    let edge2 = cap.expect("the right-top edge must survive fillet 1 and be capturable");
    println!(
        "captured edge2: p0={:?} p1={:?} n1={:?} n2={:?}",
        edge2.p0, edge2.p1, edge2.n1, edge2.n2
    );

    // Fillet 2: the captured perpendicular edge.
    g.add_feature(FeatureNode {
        id: "edgemod_4".into(),
        name: "Fillet2".into(),
        feature: FeatureType::EdgeMod {
            target: "extrude_2".into(),
            edge: edge2,
            dist: 4.0,
            dist_expr: None,
            kind: CornerKind::Fillet,
        },
    });
    g.add_dependency("edgemod_3", "edgemod_4");

    let (bodies2, w2) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    println!("after fillet 2: bodies={}, warnings={w2:?}", bodies2.len());
    assert_eq!(bodies2.len(), 1, "two fillets keep one body");

    // The SECOND round must appear: a rolled surface on the x=40 side, i.e. some
    // vertex with x strictly between 36 and 40 at an intermediate z (11..14.8).
    let mesh2 = &bodies2[0].1;
    let has_second_round = mesh2.vertices.chunks(6).any(|v| {
        v[0] > 36.0 && v[0] < 39.95 && v[2] > 11.0 && v[2] < 14.8 && v[1] > 5.0
    });
    println!("second round present: {has_second_round}");

    assert!(w2.is_empty(), "fillet 2 (perpendicular edge) should succeed, got {w2:?}");
    assert!(has_second_round, "the SECOND fillet must produce a rolled surface on the x=40 edge");

    // Mesh must stay crack-free.
    let mut edges: HashMap<((i64, i64, i64), (i64, i64, i64)), u32> = HashMap::new();
    let q = |i: usize| -> (i64, i64, i64) {
        let b = i * 6;
        let f = |v: f32| (v as f64 * 1e4).round() as i64;
        (f(mesh2.vertices[b]), f(mesh2.vertices[b + 1]), f(mesh2.vertices[b + 2]))
    };
    for t in mesh2.indices.chunks_exact(3) {
        for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            let (ka, kb) = (q(t[a] as usize), q(t[b] as usize));
            let k = if ka <= kb { (ka, kb) } else { (kb, ka) };
            *edges.entry(k).or_insert(0) += 1;
        }
    }
    let cracks = edges.values().filter(|&&c| c == 1).count();
    assert_eq!(cracks, 0, "two-fillet body has {cracks} crack edges");
}
