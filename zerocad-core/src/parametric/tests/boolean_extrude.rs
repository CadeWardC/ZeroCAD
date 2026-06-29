//! Overlapping sketch shapes combine as a boolean on extrude: selected shapes
//! are kept (and unioned together), overlapping unselected shapes are cut.

use super::*;
use crate::sketch::{Dimension, SketchShape};
use std::collections::HashSet;

fn rect_shape(x0: f32, y0: f32, x1: f32, y1: f32) -> SketchShape {
    SketchShape::Rectangle {
        origin: (x0, y0),
        sx: 1.0,
        sy: 1.0,
        w: Dimension::literal(x1 - x0),
        h: Dimension::literal(y1 - y0),
        from_center: false,
    }
}

fn circle_shape(cx: f32, cy: f32, r: f32) -> SketchShape {
    SketchShape::Circle {
        center: (cx, cy),
        diameter: Dimension::literal(r * 2.0),
    }
}

/// Add a parametric sketch defined by whole `shapes` (so the boolean path can
/// recover the full outlines).
fn add_shape_sketch(g: &mut ParametricGraph, id: &str, shapes: Vec<SketchShape>) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::XY,
            curves: SketchCurves::new(),
            shapes,
            corner_mods: vec![],
            on_face: false,
        },
    });
}

fn add_extrude_sel(
    g: &mut ParametricGraph,
    id: &str,
    sketch_id: &str,
    depth: f32,
    mode: ExtrudeMode,
    region_indices: Vec<usize>,
) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Extrude {
            depth,
            region_indices,
            mode,
            depth_expr: None,
        },
    });
    g.add_dependency(sketch_id, id);
}

/// Indices (into detected regions) of every region whose interior contains `p`.
fn region_indices_containing(
    g: &ParametricGraph,
    shapes: &[SketchShape],
    p: (f32, f32),
) -> Vec<usize> {
    let curves = crate::sketch::build_sketch_curves(shapes, &std::collections::HashMap::new());
    g.cached_regions(&curves)
        .iter()
        .enumerate()
        .filter(|(_, r)| r.contains(p))
        .map(|(i, _)| i)
        .collect()
}

/// Closed-mesh volume via the divergence theorem (signed tetra sum).
fn mesh_volume(m: &MockMesh) -> f32 {
    let p = |i: u32| {
        let b = (i as usize) * 6;
        [
            m.vertices[b] as f64,
            m.vertices[b + 1] as f64,
            m.vertices[b + 2] as f64,
        ]
    };
    let mut vol = 0.0f64;
    for tri in m.indices.chunks(3) {
        let a = p(tri[0]);
        let b = p(tri[1]);
        let c = p(tri[2]);
        vol += a[0] * (b[1] * c[2] - c[1] * b[2]) - a[1] * (b[0] * c[2] - c[0] * b[2])
            + a[2] * (b[0] * c[1] - c[0] * b[1]);
    }
    (vol / 6.0).abs() as f32
}

fn has_cylindrical_face(g: &ParametricGraph) -> bool {
    g.debug_kernel_solids(&HashSet::new())
        .unwrap()
        .iter()
        .any(|(_, parts)| {
            parts
                .iter()
                .any(crate::mock_kernel::solid_has_cylindrical_face)
        })
}

#[test]
fn selected_rect_cuts_unselected_overlapping_circle() {
    // The headline case: a rectangle (kept) with a circle overlapping its right
    // edge (cut). Selecting only the rectangle body region must bore the cylinder
    // out of the box — one body, less than the full box, with a smooth wall.
    let shapes = vec![
        rect_shape(0.0, 0.0, 20.0, 10.0),
        circle_shape(18.0, 5.0, 4.0),
    ];
    let mut g = ParametricGraph::new();
    add_shape_sketch(&mut g, "s", shapes.clone());
    // The rectangle's own material (outside the circle).
    let base = region_indices_containing(&g, &shapes, (5.0, 5.0));
    assert_eq!(base.len(), 1, "expected one rect-only region");
    add_extrude_sel(&mut g, "e", "s", 10.0, ExtrudeMode::NewBody, base);

    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert!(
        warnings.is_empty(),
        "clean box-minus-cylinder, got {warnings:?}"
    );
    assert_eq!(bodies.len(), 1, "one booleaned body");
    let m = &bodies[0].1;
    let (cracks, nm, _inward) = mesh_stats(m);
    assert_eq!(
        (cracks, nm),
        (0, 0),
        "box-minus-cylinder display mesh must be clean"
    );
    let vol = mesh_volume(m);
    // Box 200·10 = 2000; circle Ø8 mostly inside (minus an x>20 sliver ≈9.8mm²),
    // so ≈40.4mm²·10 ≈ 404 removed → ≈ 1596.
    assert!(
        vol > 1500.0 && vol < 1700.0,
        "box-minus-cylinder volume ≈1596, got {vol}"
    );
    assert!(
        has_cylindrical_face(&g),
        "the bored wall should be an analytic cylinder"
    );
}

#[test]
fn two_selected_overlapping_rects_union_into_one_body() {
    let shapes = vec![
        rect_shape(0.0, 0.0, 10.0, 10.0),
        rect_shape(5.0, 0.0, 15.0, 10.0),
    ];
    let mut g = ParametricGraph::new();
    add_shape_sketch(&mut g, "s", shapes.clone());
    let mut sel = region_indices_containing(&g, &shapes, (2.0, 5.0)); // rect A only
    sel.extend(region_indices_containing(&g, &shapes, (12.0, 5.0))); // rect B only
    add_extrude_sel(&mut g, "e", "s", 10.0, ExtrudeMode::NewBody, sel);

    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert!(
        warnings.is_empty(),
        "union should be clean, got {warnings:?}"
    );
    assert_eq!(
        bodies.len(),
        1,
        "two selected overlapping rects fuse to one body"
    );
    let vol = mesh_volume(&bodies[0].1);
    // Union footprint 150 mm² · 10 = 1500 (NOT 2000 = the two areas summed).
    assert!(
        (vol - 1500.0).abs() < 30.0,
        "union volume ≈1500 (no double-counted overlap), got {vol}"
    );
}

#[test]
fn circle_inside_rect_unselected_bores_through() {
    let shapes = vec![
        rect_shape(-10.0, -10.0, 10.0, 10.0),
        circle_shape(0.0, 0.0, 5.0),
    ];
    let mut g = ParametricGraph::new();
    add_shape_sketch(&mut g, "s", shapes.clone());
    // Select the annular rectangle material (a corner point), not the disk.
    let base = region_indices_containing(&g, &shapes, (9.0, 9.0));
    assert_eq!(base.len(), 1, "corner is in the annulus only");
    add_extrude_sel(&mut g, "e", "s", 10.0, ExtrudeMode::NewBody, base);

    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert!(warnings.is_empty(), "clean bore, got {warnings:?}");
    assert_eq!(bodies.len(), 1, "bore leaves one body (no leftover disk)");
    let vol = mesh_volume(&bodies[0].1);
    // 400·10 − (π·25)·10 ≈ 4000 − 785 ≈ 3215.
    assert!(
        vol > 3050.0 && vol < 3350.0,
        "rect-minus-circle volume ≈3215, got {vol}"
    );
    assert!(
        has_cylindrical_face(&g),
        "the bore wall should be cylindrical"
    );
}

#[test]
fn disjoint_shapes_are_not_booleaned() {
    // Far-apart rectangle + circle: no overlap, so both stay as independent
    // material (regression — the boolean path must leave them alone).
    let shapes = vec![
        rect_shape(0.0, 0.0, 10.0, 10.0),
        circle_shape(40.0, 5.0, 4.0),
    ];
    let mut g = ParametricGraph::new();
    add_shape_sketch(&mut g, "s", shapes);
    add_extrude_sel(&mut g, "e", "s", 10.0, ExtrudeMode::NewBody, vec![]); // take all

    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert!(
        warnings.is_empty(),
        "disjoint extrude is clean, got {warnings:?}"
    );
    let vol: f32 = bodies.iter().map(|(_, m)| mesh_volume(&m)).sum();
    // 100·10 + (π·16)·10 ≈ 1000 + 503 = 1503; neither shape is cut by the other.
    assert!(
        vol > 1430.0 && vol < 1560.0,
        "both shapes present (no cut), volume ≈1503, got {vol}"
    );
}

#[test]
fn three_overlapping_rects_all_selected_union() {
    let shapes = vec![
        rect_shape(0.0, 0.0, 4.0, 4.0),
        rect_shape(3.0, 0.0, 7.0, 4.0),
        rect_shape(6.0, 0.0, 10.0, 4.0),
    ];
    let mut g = ParametricGraph::new();
    add_shape_sketch(&mut g, "s", shapes);
    add_extrude_sel(&mut g, "e", "s", 5.0, ExtrudeMode::NewBody, vec![]); // take all → all base

    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert!(
        warnings.is_empty(),
        "transitive union clean, got {warnings:?}"
    );
    assert_eq!(bodies.len(), 1, "three chained rects fuse to one body");
    let vol = mesh_volume(&bodies[0].1);
    // Union footprint: 16·3 − 4 − 4 = 40 mm² · 5 = 200.
    assert!(
        (vol - 200.0).abs() < 8.0,
        "three-rect union volume ≈200, got {vol}"
    );
}

#[test]
fn legacy_curves_sketch_keeps_per_region_behavior() {
    // No `shapes` (a legacy/baked sketch) → the boolean path is skipped and the
    // overlapping rect+circle still evaluate via the per-region path.
    let mut g = ParametricGraph::new();
    let mut curves = SketchCurves::new();
    curves.add_rectangle((0.0, 0.0), (20.0, 10.0));
    curves.add_circle((20.0, 5.0), 5.0);
    add_sketch(&mut g, "s", curves);
    add_extrude(&mut g, "e", "s", 10.0, ExtrudeMode::NewBody);

    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert!(warnings.is_empty(), "legacy path clean, got {warnings:?}");
    assert!(
        !bodies.is_empty(),
        "legacy overlapping sketch still builds a body"
    );
    assert!(bodies.iter().all(|(_, m)| !m.indices.is_empty()));
}
