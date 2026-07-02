//! Regression for the reported screenshots: a circular boss/pocket combined with
//! a body must come out as a SMOOTH analytic cylinder — not the striped, sectioned
//! 48-gon prism the old "always facet a circle for booleans" rule produced — and
//! the cut must actually remove material.
//!
//! Smoothness is measured by the number of distinct B-rep faces the result
//! tessellates into (`MockMesh::face_ids`). A smooth cylinder wall is ~3 analytic
//! faces; a 48-gon prism wall is ~48 flat facets, so a faceted result has an order
//! of magnitude more faces. The threshold below sits comfortably between the two.

use std::collections::HashSet;
use zerocad_core::{
    CoordinateSystem, ExtrudeMode, FeatureNode, FeatureType, ParametricGraph, SketchCurves, Vec3,
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

fn circle_sketch(center: (f32, f32), radius: f32) -> SketchCurves {
    let mut c = SketchCurves::new();
    c.add_circle(center, radius);
    c
}

fn top_plane(h: f32) -> CoordinateSystem {
    CoordinateSystem::new(
        Vec3::new(0.0, 0.0, h),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
}

/// Box body via Box feature so the test focuses on the circular tool. Uses a
/// numeric-suffixed id (`box_1`) so it sorts FIRST in creation order — the order
/// join/cut booleans see (an extrude with id `extrude_3` runs after it).
fn box_base(g: &mut ParametricGraph, w: f32, h: f32, d: f32) {
    g.add_feature(FeatureNode {
        id: "box_1".into(),
        name: "Box".into(),
        feature: FeatureType::Box { w, h, d },
    });
}

/// Distinct B-rep faces across all bodies (smoothness proxy).
fn distinct_faces(g: &ParametricGraph) -> usize {
    g.evaluate_bodies(&HashSet::new())
        .unwrap()
        .iter()
        .flat_map(|(_, m)| m.face_ids.iter().copied())
        .collect::<HashSet<u32>>()
        .len()
}

/// A boolean'd body re-tessellates from its kernel solid (no pristine mesh), so a
/// faceted prism would explode the face count. ~50 faces ⇒ faceted; ~10 ⇒ smooth.
const SMOOTH_FACE_CAP: usize = 24;

#[test]
fn cylinder_boss_join_is_smooth_and_merged() {
    let mut g = ParametricGraph::new();
    box_base(&mut g, 40.0, 30.0, 15.0);
    add_sketch(
        &mut g,
        "sketch_2",
        top_plane(15.0),
        circle_sketch((20.0, 15.0), 6.0),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", 12.0, ExtrudeMode::Join);
    g.add_dependency("box_1", "extrude_3");

    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_eq!(bodies.len(), 1, "boss join must stay one merged body");
    assert!(
        warnings.is_empty(),
        "a clean boss join should not warn, got {warnings:?}"
    );

    // Boss reaches z≈27.
    let max_z = bodies[0]
        .1
        .vertices
        .chunks(6)
        .map(|v| v[2])
        .fold(f32::MIN, f32::max);
    assert!(max_z >= 26.9, "boss must rise to z≈27, got {max_z}");

    let faces = distinct_faces(&g);
    println!("boss-join distinct faces = {faces}");
    assert!(
        faces <= SMOOTH_FACE_CAP,
        "joined boss must be a SMOOTH cylinder (≤{SMOOTH_FACE_CAP} faces), got {faces} — \
         a faceted 48-gon prism would be ~55"
    );
}

#[test]
fn cylinder_cut_through_is_smooth_and_bores_a_hole() {
    let mut g = ParametricGraph::new();
    box_base(&mut g, 40.0, 30.0, 15.0);
    let plain_faces = distinct_faces(&g);

    add_sketch(
        &mut g,
        "sketch_2",
        top_plane(15.0),
        circle_sketch((20.0, 15.0), 6.0),
    );
    // Negative depth drills DOWN into the block (top normal is outward +Z).
    add_extrude(&mut g, "extrude_3", "sketch_2", -19.46, ExtrudeMode::Cut);
    g.add_dependency("box_1", "extrude_3");

    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_eq!(bodies.len(), 1, "cut must stay one body");
    assert!(
        warnings.is_empty(),
        "a clean drill-through should not warn, got {warnings:?}"
    );

    let faces = distinct_faces(&g);
    println!("cut-through distinct faces = {faces} (plain box = {plain_faces})");
    assert!(
        faces > plain_faces,
        "a through-drill must add the cylindrical hole wall (more faces than the plain box)"
    );
    assert!(
        faces <= SMOOTH_FACE_CAP,
        "the bored hole must be a SMOOTH cylinder (≤{SMOOTH_FACE_CAP} faces), got {faces}"
    );
}

#[test]
fn cylinder_blind_pocket_is_smooth_and_watertight() {
    let mut g = ParametricGraph::new();
    box_base(&mut g, 40.0, 30.0, 20.0);
    add_sketch(
        &mut g,
        "sketch_2",
        top_plane(20.0),
        circle_sketch((20.0, 15.0), 6.0),
    );
    // A blind pocket: only 8mm into a 20mm-thick block.
    add_extrude(&mut g, "extrude_3", "sketch_2", -8.0, ExtrudeMode::Cut);
    g.add_dependency("box_1", "extrude_3");

    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_eq!(bodies.len(), 1, "blind pocket must stay one body");
    assert!(
        warnings.is_empty(),
        "a clean blind pocket should not warn, got {warnings:?}"
    );

    // The pocket floor sits ~8mm down (z≈12); the uncut block has no surface there.
    let floor = bodies[0]
        .1
        .vertices
        .chunks(6)
        .any(|v| v[2] >= 11.0 && v[2] <= 13.0);
    assert!(floor, "blind pocket must carve a floor near z≈12");

    let faces = distinct_faces(&g);
    println!("blind-pocket distinct faces = {faces}");
    assert!(
        faces <= SMOOTH_FACE_CAP,
        "blind pocket wall must be a SMOOTH cylinder (≤{SMOOTH_FACE_CAP} faces), got {faces}"
    );
}

#[test]
fn cut_hole_rim_renders_as_a_smooth_circle() {
    // A bored hole's rim must render as a SMOOTH circle in the *wireframe* — the
    // analytic curved-edge sampling in `MockMesh::from_solid` — not the coarse
    // per-facet polygon the raw tessellation feature edges used to leave. This is
    // what lets a boolean'd body's display source (from_solid) replace the old
    // dedicated analytic-primitive wireframe path without a visible regression.
    let mut g = ParametricGraph::new();
    box_base(&mut g, 40.0, 30.0, 15.0);
    add_sketch(
        &mut g,
        "sketch_2",
        top_plane(15.0),
        circle_sketch((20.0, 15.0), 6.0),
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", -19.46, ExtrudeMode::Cut);
    g.add_dependency("box_1", "extrude_3");

    let bodies = g.evaluate_bodies(&HashSet::new()).unwrap();
    let m = &bodies[0].1;

    // Distinct wireframe vertices on the TOP rim: z≈15, radius≈6 about (20,15). The
    // box outline also sits at z=15 but at radius ≥14, so it filters out cleanly.
    let mut angs: Vec<f32> = Vec::new();
    let mut seen: HashSet<(i64, i64)> = HashSet::new();
    for v in m.edge_vertices.chunks_exact(3) {
        let (x, y, z) = (v[0], v[1], v[2]);
        if (z - 15.0).abs() > 0.15 {
            continue;
        }
        let (dx, dy) = (x - 20.0, y - 15.0);
        if (dx.hypot(dy) - 6.0).abs() > 0.5 {
            continue;
        }
        if seen.insert(((x * 1e3) as i64, (y * 1e3) as i64)) {
            angs.push(dy.atan2(dx));
        }
    }
    assert!(
        angs.len() >= 24,
        "hole rim must sample ≥24 points (smooth analytic curve), got {} — \
         a coarse tessellation polygon would have far fewer",
        angs.len()
    );
    angs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mut max_gap = 0.0f32;
    for w in angs.windows(2) {
        max_gap = max_gap.max(w[1] - w[0]);
    }
    if let (Some(&first), Some(&last)) = (angs.first(), angs.last()) {
        max_gap = max_gap.max(first + std::f32::consts::TAU - last);
    }
    let max_deg = max_gap.to_degrees();
    println!("top rim points = {}, max angular gap = {max_deg:.1}°", angs.len());
    assert!(
        max_deg <= 12.0,
        "hole rim must render smooth (≤12° between samples), got {max_deg:.1}° — \
         a faceted rim would gap much wider"
    );
}

#[test]
fn cylinder_primitive_is_smooth() {
    // The cylinder PRIMITIVE itself (not an extruded circle) must also be smooth
    // once it participates in a boolean — here cut by a small box pocket. The cut
    // now bores DOWN into the body (the cut auto-directs toward material), so the
    // pocket actually appears; its square walls add faces but the round wall must
    // stay analytic (nowhere near the ~53 faces a 48-gon prism would show).
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "cyl_1".into(),
        name: "Cyl".into(),
        feature: FeatureType::Cylinder { r: 10.0, h: 20.0 },
    });
    // Cut a 4x4 pocket down through the top (axis is +Y, top at y=20).
    let top = CoordinateSystem::new(
        Vec3::new(0.0, 20.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    );
    let mut rect = SketchCurves::new();
    rect.add_rectangle((-2.0, -2.0), (2.0, 2.0));
    add_sketch(&mut g, "sketch_2", top, rect);
    add_extrude(&mut g, "extrude_3", "sketch_2", -6.0, ExtrudeMode::Cut);
    g.add_dependency("cyl_1", "extrude_3");

    let bodies = g.evaluate_bodies(&HashSet::new()).unwrap();
    assert_eq!(bodies.len(), 1, "cut cylinder primitive stays one body");
    let m = &bodies[0].1;

    // The pocket floor sits ~6mm below the top (y≈14); the uncut cylinder has no
    // surface there. Proves the cut actually bored into the body.
    let floor = m.vertices.chunks(6).any(|v| v[1] > 13.0 && v[1] < 15.0);
    assert!(floor, "the cut must bore a pocket floor near y≈14");

    // The bored result must be a valid (watertight) solid — no mesh cracks.
    let mut edges: std::collections::HashMap<((i64, i64, i64), (i64, i64, i64)), u32> =
        std::collections::HashMap::new();
    let q = |i: u32| {
        let b = i as usize * 6;
        let g = |x: f32| (x as f64 * 1e4).round() as i64;
        (g(m.vertices[b]), g(m.vertices[b + 1]), g(m.vertices[b + 2]))
    };
    for t in m.indices.chunks_exact(3) {
        for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            let (ka, kb) = (q(t[a]), q(t[b]));
            *edges
                .entry(if ka <= kb { (ka, kb) } else { (kb, ka) })
                .or_insert(0) += 1;
        }
    }
    let cracks = edges.values().filter(|&&c| c == 1).count();
    assert_eq!(
        cracks, 0,
        "bored cylinder must be watertight, got {cracks} crack edges"
    );

    let faces = distinct_faces(&g);
    println!("cut cylinder-primitive distinct faces = {faces}");
    // A genuinely bored pocket adds its square walls/floor, but the analytic wall
    // must NOT have re-faceted into a 48-gon (~53 faces); 40 sits clearly between.
    assert!(
        faces <= 40,
        "cylinder primitive's round wall must stay analytic through a cut (≤40 faces), got {faces}"
    );
}
