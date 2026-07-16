//! Threads coexisting with rim blends + partial-length threads: a chamfer or
//! fillet on a rod's rim must SURVIVE threading (the thread fades out through a
//! runout band into a collar below the blend — no silent blend deletion, no
//! "cosmetic" fallback), and a partial `length` must thread only its window.

use openrcad::geom::GeomSurface;
use std::collections::HashSet;
use std::f32::consts::TAU;
use zerocad_core::mock_kernel::{
    chamfer_edge_with_hint, cylinder_face_near, cylinder_solid, threaded_replace_cylinder_wall,
    EdgeCurveHint, KernelSolid, MockMesh, ThreadSpec,
};
use zerocad_core::parametric::FaceRef;
use zerocad_core::{CornerKind, EdgeRef, FeatureNode, FeatureType, ParametricGraph};

/// Cylinder primitive: +Y axis, base at origin.
fn add_cylinder(g: &mut ParametricGraph, id: &str, r: f32, h: f32) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Cylinder { r, h },
    });
}

/// Rim blend on the cylinder's top rim (circle of radius `r` at height `h`).
fn add_rim_blend(
    g: &mut ParametricGraph,
    id: &str,
    target: &str,
    r: f32,
    h: f32,
    kind: CornerKind,
) {
    let rim = EdgeRef {
        p0: [r, h, 0.0],
        p1: [0.0, h, r],
        n1: [0.0, 1.0, 0.0],
        n2: [1.0, 0.0, 0.0],
        curve: Some(EdgeCurveHint::Circle {
            center: [0.0, h, 0.0],
            axis: [0.0, 1.0, 0.0],
            x_dir: [1.0, 0.0, 0.0],
            radius: r,
            start: 0.0,
            end: TAU,
            closed: true,
        }),
        topology: None,
    };
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::EdgeMod {
            target: target.to_string(),
            edge: rim,
            dist: 1.0,
            dist_expr: None,
            kind,
        },
    });
    g.add_dependency(target, id);
}

#[allow(clippy::too_many_arguments)]
fn add_thread(
    g: &mut ParametricGraph,
    id: &str,
    target: &str,
    centroid: [f32; 3],
    internal: bool,
    pitch: f32,
    depth: f32,
    starts: u32,
    length: Option<f32>,
    flip: bool,
) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Thread {
            target: target.to_string(),
            face: FaceRef {
                centroid,
                normal: [1.0, 0.0, 0.0],
                topology: None,
            },
            internal,
            pitch,
            depth,
            angle_deg: 60.0,
            right_handed: true,
            starts,
            length,
            flip,
            designation: "test".to_string(),
        },
    });
    g.add_dependency(target, id);
}

fn eval(g: &ParametricGraph) -> (Vec<(String, MockMesh)>, Vec<String>) {
    g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap()
}

fn volume(bodies: &[(String, MockMesh)], id: &str) -> f64 {
    bodies
        .iter()
        .find(|(bid, _)| bid == id)
        .and_then(|(_, m)| m.mass_properties())
        .map(|p| p.volume)
        .expect("body with measurable volume")
}

/// Blended rod, then a full-length external thread. The blend must survive
/// (a chamfer cone gets the groove cut THROUGH it; a fillet torus gets a
/// runout below it) and the thread must cut real material.
fn blended_rod_keeps_blend_and_threads(kind: CornerKind, label: &str) {
    let (r, h, pitch) = (4.0f32, 10.0f32, 1.5f32);

    // Reference: blended, unthreaded.
    let mut g0 = ParametricGraph::new();
    add_cylinder(&mut g0, "cyl_1", r, h);
    add_rim_blend(&mut g0, "em_2", "cyl_1", r, h, kind);
    let (bodies0, warnings0) = eval(&g0);
    assert!(
        warnings0.iter().all(|w| !w.contains("couldn't be")),
        "{label}: rim blend must commit: {warnings0:?}"
    );
    let vol_blended = volume(&bodies0, "cyl_1");
    let plain = std::f64::consts::PI * (r as f64).powi(2) * h as f64;
    assert!(
        vol_blended < plain - 0.5,
        "{label}: blend must remove material ({vol_blended} vs {plain})"
    );

    // Same model + full-length thread.
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "cyl_1", r, h);
    add_rim_blend(&mut g, "em_2", "cyl_1", r, h, kind);
    add_thread(
        &mut g,
        "thread_3",
        "cyl_1",
        [r, h / 2.0, 0.0],
        false,
        pitch,
        0.9,
        1,
        None,
        false,
    );
    let (bodies, warnings) = eval(&g);
    assert!(
        !warnings.iter().any(|w| w.contains("cosmetic")),
        "{label}: thread on a blended rod must cut real geometry: {warnings:?}"
    );
    let vol = volume(&bodies, "cyl_1");
    eprintln!("{label}: blended={vol_blended:.2} threaded={vol:.2}");
    assert!(
        vol < vol_blended - 0.5,
        "{label}: the thread must remove material from the blended rod ({vol} vs {vol_blended})"
    );
    assert!(
        vol > vol_blended * 0.7,
        "{label}: the thread must not gut the rod ({vol} vs {vol_blended})"
    );
}

#[test]
fn chamfered_rod_thread_cuts_through_the_chamfer() {
    blended_rod_keeps_blend_and_threads(CornerKind::Chamfer, "chamfer");
}

#[test]
fn filleted_rod_thread_runs_out_below_the_fillet() {
    blended_rod_keeps_blend_and_threads(CornerKind::Fillet, "fillet");
}

/// A partial thread's runout + collar must also produce a clean render mesh
/// (manifold, normals following winding) — the guard against the "broken
/// geometry band" artifacts.
#[test]
fn kernel_partial_runout_mesh_is_clean() {
    let (r, h) = (4.0f32, 30.0f32);
    let solid = cylinder_solid(r, h).expect("cylinder primitive");
    let info = cylinder_face_near(&solid, [r, h / 2.0, 0.0]).expect("wall face info");
    let spec = ThreadSpec {
        mean_radius: info.radius,
        pitch: 1.5,
        length: info.axial_max - info.axial_min,
        depth: 0.9,
        half_angle_deg: 30.0,
        right_handed: true,
        internal: false,
        segments_per_turn: 16,
        starts: 1,
    };
    let threaded = threaded_replace_cylinder_wall(&solid, &info, &spec, Some(10.0), false)
        .expect("partial thread");
    assert!(threaded.is_watertight());
    zerocad_core::mock_kernel::try_display_mesh_from_part(&threaded)
        .expect("partial-thread render mesh must pass manifold + winding checks");
}

/// Kernel-level cut-through: threading a chamfered rod must keep the cone,
/// carve the groove THROUGH it (thread roots above the cone base), stay
/// watertight, and produce a render mesh that passes the manifold + winding
/// checks (the "broken geometry" guard).
fn kernel_chamfer_cut_through(chamfer: f32, label: &str) {
    let (r, h) = (4.0f32, 10.0f32);
    let solid = cylinder_solid(r, h).expect("cylinder primitive");
    let hint = EdgeCurveHint::Circle {
        center: [0.0, h, 0.0],
        axis: [0.0, 1.0, 0.0],
        x_dir: [1.0, 0.0, 0.0],
        radius: r,
        start: 0.0,
        end: TAU,
        closed: true,
    };
    let chamfered = chamfer_edge_with_hint(&solid, [r, h, 0.0], [-r, h, 0.0], Some(&hint), chamfer)
        .expect("rim chamfer");
    let info = cylinder_face_near(&chamfered, [r, h / 2.0, 0.0]).expect("wall face info");
    let depth = 0.9f32;
    let spec = ThreadSpec {
        mean_radius: info.radius,
        pitch: 1.5,
        length: info.axial_max - info.axial_min,
        depth,
        half_angle_deg: 30.0,
        right_handed: true,
        internal: false,
        segments_per_turn: 16,
        starts: 1,
    };
    let threaded = threaded_replace_cylinder_wall(&chamfered, &info, &spec, None, false)
        .expect("chamfered wall must thread, not fall back");
    assert!(
        threaded.is_watertight(),
        "{label}: threaded chamfered rod watertight"
    );
    let cones = count_surface(&threaded, |s| matches!(s, GeomSurface::Cone(_)));
    assert!(
        cones >= 1,
        "{label}: the chamfer cone must survive threading, got {cones}"
    );
    let mesh = zerocad_core::mock_kernel::try_display_mesh_from_part(&threaded)
        .expect("render mesh must pass manifold + winding checks");
    // The groove must reach INTO the cone zone (roots above the cone base at
    // y = h - chamfer): Fusion-style cut-through, not a runout below it.
    let zc = h - chamfer;
    let roots_in_cone = mesh
        .vertices
        .chunks(6)
        .filter(|v| {
            let radial = (v[0] * v[0] + v[2] * v[2]).sqrt();
            v[1] > zc + 0.05 && radial < r - 0.45 * depth
        })
        .count();
    assert!(
        roots_in_cone > 0,
        "{label}: thread roots must continue through the chamfer cone"
    );
}

#[test]
fn kernel_deep_chamfer_cut_through_case_a() {
    // Chamfer 1.0 ≥ thread depth 0.9: the groove dies inside the cone.
    kernel_chamfer_cut_through(1.0, "case A");
}

#[test]
fn kernel_shallow_chamfer_cut_through_case_b() {
    // Chamfer 0.5 < thread depth 0.9: the groove punches through to the top
    // cap, which gets a mixed circle+profile rim.
    kernel_chamfer_cut_through(0.5, "case B");
}

fn count_surface(solid: &KernelSolid, pred: impl Fn(&GeomSurface) -> bool) -> usize {
    solid
        .shell()
        .faces()
        .iter()
        .filter(|f| f.surface().map(&pred).unwrap_or(false))
        .count()
}

/// Vertices at thread-root depth (well below the crest radius), excluding the
/// cap planes where disk triangulation reaches all radii.
fn root_vertex_ys(mesh: &MockMesh, r: f32, depth: f32, h: f32) -> Vec<f32> {
    let thresh = r - 0.45 * depth;
    mesh.vertices
        .chunks(6)
        .filter_map(|v| {
            let (x, y, z) = (v[0], v[1], v[2]);
            let radial = (x * x + z * z).sqrt();
            (y > 0.5 && y < h - 0.5 && radial < thresh).then_some(y)
        })
        .collect()
}

/// A partial thread (length 10 on a 30-tall rod, anchored at the top) must
/// thread only the top window and leave the shank plain.
#[test]
fn partial_thread_spans_only_its_window() {
    let (r, h, pitch, depth) = (4.0f32, 30.0f32, 1.5f32, 0.9f32);
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "cyl_1", r, h);
    add_thread(
        &mut g,
        "thread_2",
        "cyl_1",
        [r, h / 2.0, 0.0],
        false,
        pitch,
        depth,
        1,
        Some(10.0),
        false,
    );
    let (bodies, warnings) = eval(&g);
    assert!(
        !warnings.iter().any(|w| w.contains("cosmetic")),
        "partial thread must cut real geometry: {warnings:?}"
    );
    let mesh = &bodies.iter().find(|(id, _)| id == "cyl_1").unwrap().1;
    let roots = root_vertex_ys(mesh, r, depth, h);
    assert!(!roots.is_empty(), "the window must actually be threaded");
    let (lo, hi) = roots
        .iter()
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(a, b), &y| {
            (a.min(y), b.max(y))
        });
    eprintln!("partial thread roots span y = [{lo:.2}, {hi:.2}]");
    assert!(
        lo >= 19.9,
        "thread roots must stay inside the top window (min y {lo})"
    );
    assert!(hi <= 30.01, "thread roots above the rod?! (max y {hi})");
}

/// `flip` anchors the same partial thread at the bottom instead.
#[test]
fn flipped_partial_thread_anchors_at_the_bottom() {
    let (r, h, pitch, depth) = (4.0f32, 30.0f32, 1.5f32, 0.9f32);
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "cyl_1", r, h);
    add_thread(
        &mut g,
        "thread_2",
        "cyl_1",
        [r, h / 2.0, 0.0],
        false,
        pitch,
        depth,
        1,
        Some(10.0),
        true,
    );
    let (bodies, warnings) = eval(&g);
    assert!(
        !warnings.iter().any(|w| w.contains("cosmetic")),
        "flipped partial thread must cut real geometry: {warnings:?}"
    );
    let mesh = &bodies.iter().find(|(id, _)| id == "cyl_1").unwrap().1;
    let roots = root_vertex_ys(mesh, r, depth, h);
    assert!(!roots.is_empty(), "the window must actually be threaded");
    let (lo, hi) = roots
        .iter()
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(a, b), &y| {
            (a.min(y), b.max(y))
        });
    eprintln!("flipped partial thread roots span y = [{lo:.2}, {hi:.2}]");
    assert!(lo >= -0.01, "thread roots below the rod?! (min y {lo})");
    assert!(
        hi <= 10.6,
        "flipped thread roots must stay inside the bottom window (max y {hi})"
    );
}

/// Multi-start partial thread: the per-band machinery must handle starts=2
/// windows + runout without falling back.
#[test]
fn two_start_partial_thread_evaluates() {
    let (r, h) = (4.0f32, 30.0f32);
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "cyl_1", r, h);
    add_thread(
        &mut g,
        "thread_2",
        "cyl_1",
        [r, h / 2.0, 0.0],
        false,
        1.5,
        0.9,
        2,
        Some(12.0),
        false,
    );
    let (bodies, warnings) = eval(&g);
    assert!(
        !warnings.iter().any(|w| w.contains("cosmetic")),
        "two-start partial thread must cut real geometry: {warnings:?}"
    );
    let vol = volume(&bodies, "cyl_1");
    let plain = std::f64::consts::PI * (r as f64).powi(2) * h as f64;
    assert!(vol < plain - 0.5, "must remove material ({vol} vs {plain})");
}

/// Internal thread on a countersunk hole: the mouth chamfer (a concave cone)
/// must survive tapping.
#[test]
fn internal_thread_keeps_the_countersink() {
    fn base_graph() -> ParametricGraph {
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
            },
        });
        g.add_dependency("box_1", "hole_2");
        // Countersink: chamfer the hole's mouth rim (circle r=3 at z=10).
        let rim = EdgeRef {
            p0: [13.0, 10.0, 10.0],
            p1: [10.0, 13.0, 10.0],
            n1: [0.0, 0.0, 1.0],
            n2: [-1.0, 0.0, 0.0],
            curve: Some(EdgeCurveHint::Circle {
                center: [10.0, 10.0, 10.0],
                axis: [0.0, 0.0, 1.0],
                x_dir: [1.0, 0.0, 0.0],
                radius: 3.0,
                start: 0.0,
                end: TAU,
                closed: true,
            }),
            topology: None,
        };
        g.add_feature(FeatureNode {
            id: "em_3".to_string(),
            name: "em_3".to_string(),
            feature: FeatureType::EdgeMod {
                target: "box_1".to_string(),
                edge: rim,
                dist: 1.0,
                dist_expr: None,
                kind: CornerKind::Chamfer,
            },
        });
        g.add_dependency("box_1", "em_3");
        g
    }

    let g0 = base_graph();
    let (bodies0, warnings0) = eval(&g0);
    assert!(
        warnings0.iter().all(|w| !w.contains("couldn't be")),
        "countersink must commit: {warnings0:?}"
    );
    let vol_sunk = volume(&bodies0, "box_1");

    let mut g = base_graph();
    add_thread(
        &mut g,
        "thread_4",
        "box_1",
        [13.0, 10.0, 5.0],
        true,
        1.0,
        0.6,
        1,
        None,
        false,
    );
    let (bodies, warnings) = eval(&g);
    assert!(
        !warnings.iter().any(|w| w.contains("cosmetic")),
        "tap through a countersunk mouth must cut real geometry: {warnings:?}"
    );
    let vol = volume(&bodies, "box_1");
    eprintln!("countersunk={vol_sunk:.2} tapped={vol:.2}");
    assert!(
        vol < vol_sunk - 0.5,
        "the tap must remove material ({vol} vs {vol_sunk})"
    );
    assert!(
        vol > vol_sunk * 0.9,
        "the tap must not gut the body ({vol} vs {vol_sunk})"
    );

    // Fusion-style cut-through (symmetric with the external rod): the groove
    // must continue THROUGH the countersink cone, not run out below it. Axis is
    // Z at (x=10, y=10); bore r=3, internal root at r≈3.6; the chamfer cone
    // spans z=9 (base) → z=10 (mouth). Root-radius mesh vertices must appear
    // above the cone base.
    let mesh = &bodies.iter().find(|(b, _)| b == "box_1").unwrap().1;
    let roots_in_cone = mesh
        .vertices
        .chunks(6)
        .filter(|v| {
            let radial = ((v[0] - 10.0).powi(2) + (v[1] - 10.0).powi(2)).sqrt();
            radial > 3.2 && radial < 3.9 && v[2] > 9.05 && v[2] < 9.95
        })
        .count();
    assert!(
        roots_in_cone > 0,
        "the thread groove must cut THROUGH the countersink cone, got {roots_in_cone} roots in the cone zone"
    );
}

/// Reverse order — partial thread first, then a chamfer on the still-plain
/// collar rim. The body must at minimum SURVIVE (safe degrade); when edge_mod
/// accepts the collar adjacency the chamfer commits too.
#[test]
fn chamfer_after_partial_thread_degrades_safely() {
    let (r, h) = (4.0f32, 30.0f32);
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "cyl_1", r, h);
    // Thread the BOTTOM half so the top rim stays a plain circle on the collar.
    add_thread(
        &mut g,
        "thread_2",
        "cyl_1",
        [r, h / 2.0, 0.0],
        false,
        1.5,
        0.9,
        1,
        Some(10.0),
        true,
    );
    add_rim_blend(&mut g, "em_3", "cyl_1", r, h, CornerKind::Chamfer);
    let (bodies, warnings) = eval(&g);
    eprintln!("thread-then-chamfer warnings: {warnings:?}");
    let mesh = &bodies.iter().find(|(id, _)| id == "cyl_1").unwrap().1;
    assert!(
        !mesh.vertices.is_empty() && !mesh.indices.is_empty(),
        "body must survive thread-then-chamfer"
    );
    assert!(
        !warnings.iter().any(|w| w.contains("cosmetic")),
        "the partial thread itself must still be real geometry: {warnings:?}"
    );
}
