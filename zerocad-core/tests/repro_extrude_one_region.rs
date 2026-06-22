//! Regression: extruding a sketch face must not produce back-face-culled
//! ("disappearing") triangles, even when the face is NON-CONVEX.
//!
//! The GUI report ("extruding a sketch ... the render is doing something wrong",
//! with a box+cylinder full of holes) came from `make_extruded_sketch` orienting
//! its shell with a per-triangle *centroid* test. That test assumes a roughly
//! convex profile; a region split out of an arrangement of OVERLAPPING sketch
//! shapes (a rectangle with a circular bite where a circle crossed it) is
//! non-convex, so triangles on the concave side were misjudged and left
//! inward-facing — they back-face cull and the body renders with holes. The fix
//! switches that path to the robust adjacency + signed-volume orientation
//! (`orient_mesh_outward`) already used for boolean results.
//!
//! Invariant asserted here: zero inward (normal-vs-winding-disagreeing) triangles
//! and zero boundary/crack edges (a watertight outer shell). The lingering
//! NON-manifold edges at arc/straight-line junctions of a non-convex region are a
//! separate, pre-existing kernel-tessellation limitation (printed, not asserted).

use std::collections::HashSet;
use zerocad_core::{
    detect_regions, CoordinateSystem, ExtrudeMode, FeatureNode, FeatureType, ParametricGraph,
    SketchCurves, Vec3,
};

fn top_plane(h: f32) -> CoordinateSystem {
    CoordinateSystem::new(
        Vec3::new(0.0, 0.0, h),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
}

/// (boundary/crack edges, non-manifold edges, inward triangles) for a mesh.
/// Welds positions to 1e-4. "inward" = the triangle's stored normal disagrees
/// with its winding normal, i.e. it back-face culls and vanishes on screen.
fn mesh_stats(m: &zerocad_core::MockMesh) -> (usize, usize, usize) {
    use std::collections::HashMap;
    let q = |i: usize| -> (i64, i64, i64) {
        let b = i * 6;
        let g = |v: f32| (v as f64 * 1e4).round() as i64;
        (g(m.vertices[b]), g(m.vertices[b + 1]), g(m.vertices[b + 2]))
    };
    let mut edges: HashMap<((i64, i64, i64), (i64, i64, i64)), u32> = HashMap::new();
    let mut disagree = 0;
    for t in m.indices.chunks_exact(3) {
        for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            let (ka, kb) = (q(t[a] as usize), q(t[b] as usize));
            let k = if ka <= kb { (ka, kb) } else { (kb, ka) };
            *edges.entry(k).or_insert(0) += 1;
        }
        let p = |i: u32| {
            let b = i as usize * 6;
            [m.vertices[b] as f64, m.vertices[b + 1] as f64, m.vertices[b + 2] as f64]
        };
        let vn = |i: u32| {
            let b = i as usize * 6;
            [m.vertices[b + 3] as f64, m.vertices[b + 4] as f64, m.vertices[b + 5] as f64]
        };
        let a = p(t[0]);
        let b = p(t[1]);
        let d = p(t[2]);
        let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let w = [d[0] - a[0], d[1] - a[1], d[2] - a[2]];
        let fnn = [
            u[1] * w[2] - u[2] * w[1],
            u[2] * w[0] - u[0] * w[2],
            u[0] * w[1] - u[1] * w[0],
        ];
        let na = [
            (vn(t[0])[0] + vn(t[1])[0] + vn(t[2])[0]) / 3.0,
            (vn(t[0])[1] + vn(t[1])[1] + vn(t[2])[1]) / 3.0,
            (vn(t[0])[2] + vn(t[1])[2] + vn(t[2])[2]) / 3.0,
        ];
        if fnn[0] * na[0] + fnn[1] * na[1] + fnn[2] * na[2] < 0.0 {
            disagree += 1;
        }
    }
    let cracks = edges.values().filter(|&&c| c == 1).count();
    let nonmanifold = edges.values().filter(|&&c| c > 2).count();
    (cracks, nonmanifold, disagree)
}

fn one_extrude(curves: SketchCurves, region_indices: Vec<usize>) -> zerocad_core::MockMesh {
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "sketch_1".into(),
        name: "sketch_1".into(),
        feature: FeatureType::Sketch {
            cs: top_plane(0.0),
            curves,
            shapes: vec![],
            corner_mods: vec![],
            on_face: false,
        },
    });
    g.add_feature(FeatureNode {
        id: "extrude_2".into(),
        name: "extrude_2".into(),
        feature: FeatureType::Extrude {
            depth: 15.0,
            region_indices,
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
        },
    });
    g.add_dependency("sketch_1", "extrude_2");
    let bodies = g.evaluate_bodies(&HashSet::new()).unwrap();
    assert_eq!(bodies.len(), 1, "one extrude => one body");
    bodies.into_iter().next().unwrap().1
}

fn separate_rect_and_circle() -> SketchCurves {
    let mut c = SketchCurves::new();
    c.add_rectangle((0.0, 20.0), (40.0, 50.0));
    c.add_circle((20.0, 0.0), 12.0);
    c
}

fn overlapping_rect_and_circle() -> SketchCurves {
    let mut c = SketchCurves::new();
    c.add_rectangle((0.0, 5.0), (40.0, 35.0));
    c.add_circle((20.0, 8.0), 14.0);
    c
}

#[test]
fn extrude_only_selected_region() {
    // Separate rectangle + circle. Selecting just the rectangle (region with the
    // higher centroid) must extrude ONLY it — geometry stays at y >= 20.
    let curves = separate_rect_and_circle();
    let regions = detect_regions(&curves);
    assert_eq!(regions.len(), 2);
    let rect_idx = regions
        .iter()
        .position(|r| {
            let n = r.boundary.len().max(1) as f32;
            r.boundary.iter().map(|p| p.1).sum::<f32>() / n > 20.0
        })
        .expect("a rectangle region");

    let m = one_extrude(curves, vec![rect_idx]);
    let min_y = m.vertices.chunks(6).map(|v| v[1]).fold(f32::INFINITY, f32::min);
    assert!(
        min_y > 19.0,
        "only the rectangle (y>=20) should extrude; geometry reached y={min_y:.1}"
    );
    let (cracks, nm, inward) = mesh_stats(&m);
    println!("rectangle only: cracks={cracks}, nonmanifold={nm}, inward={inward}");
    assert_eq!(inward, 0, "no back-face-culled triangles");
    assert_eq!(cracks, 0, "watertight (no crack/hole edges)");
    assert_eq!(nm, 0, "a convex rectangle prism is fully manifold");
}

#[test]
fn extrude_separate_shapes_together_is_clean() {
    // Both shapes (separate) as one new body — a clean closed manifold.
    let curves = separate_rect_and_circle();
    let m = one_extrude(curves, vec![]);
    let (cracks, nm, inward) = mesh_stats(&m);
    println!("rect+circle together: cracks={cracks}, nonmanifold={nm}, inward={inward}");
    assert_eq!(inward, 0, "no back-face-culled triangles");
    assert_eq!(cracks, 0, "watertight");
    assert_eq!(nm, 0, "two disjoint convex prisms are manifold");
}

#[test]
fn extrude_single_nonconvex_region_has_no_disappearing_faces() {
    // Overlapping shapes split into tiled regions; region 0 is the rectangle with
    // a circular bite (NON-CONVEX). This is the case that used to render with
    // holes. Post-fix: zero inward faces and a watertight outer shell.
    let curves = overlapping_rect_and_circle();
    let regions = detect_regions(&curves);
    assert!(regions.len() >= 2, "overlap splits into multiple regions");

    let m = one_extrude(curves, vec![0]);
    let (cracks, nm, inward) = mesh_stats(&m);
    println!("non-convex region 0: cracks={cracks}, nonmanifold={nm}, inward={inward}");
    assert_eq!(inward, 0, "non-convex region must not leave inward (disappearing) faces");
    assert_eq!(cracks, 0, "outer shell stays watertight (no holes)");
    assert_eq!(
        nm, 0,
        "selected circle-fragment regions must render as a clean manifold mesh"
    );
}

#[test]
fn extrude_whole_overlapping_sketch_has_no_disappearing_faces() {
    // The screenshot case: "Extrude whole Sketch" over overlapping shapes (all
    // tiled regions, NewBody). Used to render full of holes (274 inward faces).
    // Post-fix: zero inward faces — every surface triangle faces outward, so the
    // body reads solid (internal tile walls are interior and occluded).
    let curves = overlapping_rect_and_circle();
    let m = one_extrude(curves, vec![]);
    let (cracks, nm, inward) = mesh_stats(&m);
    println!("whole overlapping sketch: cracks={cracks}, nonmanifold={nm}, inward={inward}");
    assert_eq!(inward, 0, "whole-sketch extrude must not leave inward (disappearing) faces");
}
