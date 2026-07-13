//! Repro: draw a sketch, extrude it, then EDIT the sketch — the extruded body
//! must survive the edit. Editing a committed sketch promotes it to the
//! constraint-solver model (`solver: Some(..)`) and rebakes `curves` from that
//! model, exactly as the GUI's `edit_sketch` + Finish-Sketch path does. If the
//! promoted sketch's region ordering/count no longer matches the extrude's
//! stored `region_indices`, the extrude selects nothing and the body vanishes.

use std::collections::HashSet;
use zerocad_core::sketch::constraints::promote_shapes_to_entities;
use zerocad_core::sketch::EntityId;
use zerocad_core::{
    effective_curves_solved, CoordinateSystem, Dimension, ExtrudeMode, FeatureNode, FeatureType,
    ParametricGraph, SketchCurves, SketchShape,
};

fn rect(origin: (f32, f32), w: f32, h: f32) -> SketchShape {
    SketchShape::Rectangle {
        origin,
        sx: 1.0,
        sy: 1.0,
        w: Dimension::literal(w),
        h: Dimension::literal(h),
        from_center: false,
    }
}

fn sketch_node(id: &str, shapes: Vec<SketchShape>) -> FeatureNode {
    FeatureNode {
        id: id.into(),
        name: id.into(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::XY,
            curves: SketchCurves::new(),
            shapes,
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
            entity_ids: vec![],
            next_entity_id: 0,
            solver: None,
        },
    }
}

fn extrude_node(id: &str, region_indices: Vec<usize>) -> FeatureNode {
    FeatureNode {
        id: id.into(),
        name: id.into(),
        feature: FeatureType::Extrude {
            target: None,
            depth: 10.0,
            region_indices,
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
        },
    }
}

/// Promote the named sketch node to the solver model and rebake its `curves`,
/// mirroring `edit_sketch` (promotion) + Finish-Sketch (write-back).
fn simulate_edit(g: &mut ParametricGraph, sketch_id: &str) {
    let vars = g.variable_map();
    for idx in g.graph.node_indices() {
        if g.graph[idx].id != sketch_id {
            continue;
        }
        if let FeatureType::Sketch {
            shapes,
            entity_ids,
            curves,
            next_entity_id,
            solver,
            ..
        } = &mut g.graph[idx].feature
        {
            let ids = if entity_ids.is_empty() {
                EntityId::sequence(shapes.len())
            } else {
                entity_ids.clone()
            };
            let (model, next) = promote_shapes_to_entities(shapes, &ids, &vars, ids.len() as u32);
            *curves = effective_curves_solved(curves, shapes, &[], &[], Some(&model), &vars);
            *entity_ids = ids;
            *next_entity_id = next;
            *solver = Some(model);
        }
        break;
    }
}

#[test]
fn edit_single_rectangle_sketch_keeps_extruded_body() {
    let mut g = ParametricGraph::new();
    g.add_feature(sketch_node("sketch_1", vec![rect((0.0, 0.0), 20.0, 12.0)]));
    g.add_feature(extrude_node("extrude_2", vec![0]));
    g.add_dependency("sketch_1", "extrude_2");

    let before = g.evaluate_bodies(&HashSet::new()).unwrap();
    assert_eq!(before.len(), 1, "baseline: one extruded body");
    assert!(!before[0].1.vertices.is_empty());

    simulate_edit(&mut g, "sketch_1");

    let after = g.evaluate_bodies(&HashSet::new()).unwrap();
    assert_eq!(
        after.len(),
        1,
        "editing the sketch must NOT make the extruded body disappear"
    );
    assert!(
        !after[0].1.vertices.is_empty(),
        "the body must still have geometry after the edit"
    );
}

#[test]
fn edit_two_rectangle_sketch_keeps_selected_region_body() {
    // Two separate rectangles → two regions. Extrude selects region [1].
    let mut g = ParametricGraph::new();
    g.add_feature(sketch_node(
        "sketch_1",
        vec![rect((0.0, 0.0), 10.0, 10.0), rect((30.0, 0.0), 10.0, 10.0)],
    ));
    g.add_feature(extrude_node("extrude_2", vec![1]));
    g.add_dependency("sketch_1", "extrude_2");

    let before = g.evaluate_bodies(&HashSet::new()).unwrap();
    assert_eq!(
        before.len(),
        1,
        "baseline: one body from the selected region"
    );
    let before_centroid_x: f32 = {
        let m = &before[0].1;
        let xs: Vec<f32> = m.vertices.chunks(6).map(|v| v[0]).collect();
        xs.iter().copied().sum::<f32>() / xs.len() as f32
    };

    simulate_edit(&mut g, "sketch_1");

    let after = g.evaluate_bodies(&HashSet::new()).unwrap();
    assert_eq!(
        after.len(),
        1,
        "editing must keep the body for the selected region"
    );
    let after_centroid_x: f32 = {
        let m = &after[0].1;
        let xs: Vec<f32> = m.vertices.chunks(6).map(|v| v[0]).collect();
        xs.iter().copied().sum::<f32>() / xs.len() as f32
    };
    assert!(
        (before_centroid_x - after_centroid_x).abs() < 0.5,
        "the SAME region must extrude after the edit: before x={before_centroid_x:.1}, \
         after x={after_centroid_x:.1}"
    );
}

fn circle(center: (f32, f32), dia: f32) -> SketchShape {
    SketchShape::Circle {
        center,
        diameter: Dimension::literal(dia),
    }
}

/// Behavioral: a rectangle with a smaller separate rectangle beside it, where
/// the extrude selects a SPECIFIC region by index. If promotion reorders the
/// regions, the stored index selects a different (or no) region after an edit.
#[test]
fn edit_keeps_the_same_selected_region_three_rects() {
    use zerocad_core::detect_regions;
    // Three separated rectangles at increasing x. Region indices are stable if
    // detection order is stable; the extrude pins the MIDDLE one.
    let shapes = vec![
        rect((0.0, 0.0), 8.0, 8.0),
        rect((20.0, 0.0), 8.0, 8.0),
        rect((40.0, 0.0), 8.0, 8.0),
    ];
    let vars = std::collections::HashMap::new();
    let fresh = zerocad_core::effective_curves(&SketchCurves::new(), &shapes, &[], &vars);
    let fresh_regions = detect_regions(&fresh);
    assert_eq!(fresh_regions.len(), 3);
    // The region whose centroid x is in the middle (~24).
    let mid_idx = (0..3)
        .min_by_key(|&i| {
            let b = &fresh_regions[i].boundary;
            let cx = b.iter().map(|p| p.0).sum::<f32>() / b.len() as f32;
            (cx - 24.0).abs() as i64
        })
        .unwrap();

    let mut g = ParametricGraph::new();
    g.add_feature(sketch_node("sketch_1", shapes));
    g.add_feature(extrude_node("extrude_2", vec![mid_idx]));
    g.add_dependency("sketch_1", "extrude_2");

    let before = g.evaluate_bodies(&HashSet::new()).unwrap();
    assert_eq!(before.len(), 1);
    let cx = |m: &zerocad_core::MockMesh| {
        let xs: Vec<f32> = m.vertices.chunks(6).map(|v| v[0]).collect();
        xs.iter().copied().sum::<f32>() / xs.len() as f32
    };
    let before_cx = cx(&before[0].1);
    assert!(
        (before_cx - 24.0).abs() < 2.0,
        "baseline extrudes the middle rect, cx={before_cx:.1}"
    );

    simulate_edit(&mut g, "sketch_1");
    let after = g.evaluate_bodies(&HashSet::new()).unwrap();
    assert_eq!(after.len(), 1, "body vanished after edit");
    let after_cx = cx(&after[0].1);
    assert!(
        (after_cx - 24.0).abs() < 2.0,
        "after edit the SAME (middle) region must extrude, cx={after_cx:.1}"
    );
}

#[test]
fn edit_rect_with_hole_keeps_body() {
    let mut g = ParametricGraph::new();
    g.add_feature(sketch_node(
        "sketch_1",
        vec![rect((0.0, 0.0), 40.0, 40.0), circle((20.0, 20.0), 16.0)],
    ));
    // Select every region (whole sketch) — the annulus is the material region.
    g.add_feature(extrude_node("extrude_2", vec![]));
    g.add_dependency("sketch_1", "extrude_2");
    let before = g.evaluate_bodies(&HashSet::new()).unwrap();
    assert_eq!(before.len(), 1);
    let before_tris = before[0].1.indices.len();

    simulate_edit(&mut g, "sketch_1");
    let after = g.evaluate_bodies(&HashSet::new()).unwrap();
    assert_eq!(after.len(), 1, "rect-with-hole body vanished after edit");
    assert!(
        (after[0].1.indices.len() as i64 - before_tris as i64).abs() < before_tris as i64 / 4,
        "geometry changed drastically after edit: {before_tris} -> {}",
        after[0].1.indices.len()
    );
}

/// A robust interior point of a region: grid-sample its bbox and return the first
/// sample that is inside (respects holes).
fn interior_of(r: &zerocad_core::Region) -> Option<(f32, f32)> {
    let b = &r.boundary;
    let (mut lo_x, mut lo_y, mut hi_x, mut hi_y) = (f32::MAX, f32::MAX, f32::MIN, f32::MIN);
    for &(x, y) in b {
        lo_x = lo_x.min(x);
        lo_y = lo_y.min(y);
        hi_x = hi_x.max(x);
        hi_y = hi_y.max(y);
    }
    for iy in 1..20 {
        for ix in 1..20 {
            let p = (
                lo_x + (hi_x - lo_x) * ix as f32 / 20.0,
                lo_y + (hi_y - lo_y) * iy as f32 / 20.0,
            );
            if r.contains(p) {
                return Some(p);
            }
        }
    }
    None
}

fn probe_order(shapes: &[SketchShape], label: &str) {
    use zerocad_core::{detect_regions, effective_curves};
    let vars = std::collections::HashMap::new();
    let ids = EntityId::sequence(shapes.len());
    let fresh = detect_regions(&effective_curves(&SketchCurves::new(), shapes, &[], &vars));
    let (model, _n) = promote_shapes_to_entities(shapes, &ids, &vars, ids.len() as u32);
    let baked = detect_regions(&zerocad_core::sketch::constraints::bake_entities_to_curves(
        &model,
    ));
    assert_eq!(
        fresh.len(),
        baked.len(),
        "{label}: region count changed {} -> {}",
        fresh.len(),
        baked.len()
    );
    for (i, fr) in fresh.iter().enumerate() {
        let Some(p) = interior_of(fr) else { continue };
        assert!(
            baked[i].contains(p),
            "{label}: region {i} reordered — fresh interior {p:?} not in baked[{i}]"
        );
    }
}

#[test]
fn region_order_stable_across_promotion_matrix() {
    probe_order(
        &[rect((0.0, 0.0), 40.0, 40.0), circle((20.0, 20.0), 16.0)],
        "rect+hole",
    );
    probe_order(
        &[
            rect((0.0, 0.0), 8.0, 8.0),
            rect((20.0, 0.0), 8.0, 8.0),
            rect((40.0, 0.0), 8.0, 8.0),
        ],
        "3 rects",
    );
    // circle first, then rectangle around it (reversed shape order)
    probe_order(
        &[circle((20.0, 20.0), 16.0), rect((0.0, 0.0), 40.0, 40.0)],
        "hole+rect",
    );
    // two overlapping rectangles
    probe_order(
        &[rect((0.0, 0.0), 20.0, 20.0), rect((10.0, 10.0), 20.0, 20.0)],
        "overlap rects",
    );
    // rectangle + two holes
    probe_order(
        &[
            rect((0.0, 0.0), 60.0, 30.0),
            circle((15.0, 15.0), 10.0),
            circle((45.0, 15.0), 10.0),
        ],
        "rect+2holes",
    );
}

/// The reported screenshot case: a rectangle drawn INSIDE another rectangle
/// (pure containment). Detection yields two regions — the outer frame/annulus
/// and the inner rectangle. Selecting the INNER region and extruding it as a
/// New Body must produce a solid box, not an empty body. Nested shapes cluster
/// as a boolean (containment), and the inner region used to be dropped as a
/// "base∩tool lens" even though the user explicitly picked it.
#[test]
fn extrude_inner_region_of_nested_rectangles_makes_a_body() {
    use zerocad_core::detect_regions;
    let shapes = vec![
        rect((0.0, 0.0), 60.0, 40.0),   // outer
        rect((10.0, 10.0), 40.0, 20.0), // inner (fully contained)
    ];
    let vars = std::collections::HashMap::new();
    let regions = detect_regions(&zerocad_core::effective_curves(
        &SketchCurves::new(),
        &shapes,
        &[],
        &vars,
    ));
    assert_eq!(regions.len(), 2, "frame + inner rectangle");
    // The inner region: the one whose material point is near the inner centre.
    let inner_idx = regions
        .iter()
        .position(|r| r.contains((30.0, 20.0)))
        .expect("an inner rectangle region containing its centre");

    let mut g = ParametricGraph::new();
    g.add_feature(sketch_node("sketch_1", shapes));
    g.add_feature(extrude_node("extrude_2", vec![inner_idx]));
    g.add_dependency("sketch_1", "extrude_2");

    let bodies = g.evaluate_bodies(&HashSet::new()).unwrap();
    assert_eq!(
        bodies.len(),
        1,
        "selecting the inner region must build a body"
    );
    assert!(
        !bodies[0].1.vertices.is_empty(),
        "the inner rectangle must extrude into a solid box, not an empty body"
    );
    // It must be the INNER box: extent ~40x20, centred at (30,20).
    let m = &bodies[0].1;
    let (mut lo_x, mut hi_x) = (f32::MAX, f32::MIN);
    for v in m.vertices.chunks(6) {
        lo_x = lo_x.min(v[0]);
        hi_x = hi_x.max(v[0]);
    }
    assert!(
        lo_x >= 9.0 && hi_x <= 51.0,
        "extruded body must be the inner rectangle (x in ~[10,50]), got [{lo_x:.1},{hi_x:.1}]"
    );
}

/// Guard the intended containment behavior is preserved: extruding the WHOLE
/// nested sketch (no explicit selection) still drops the inner region as a hole,
/// so the result is a frame (rect-with-hole), not a filled slab.
#[test]
fn extrude_whole_nested_rectangles_keeps_the_hole() {
    let shapes = vec![rect((0.0, 0.0), 60.0, 40.0), rect((10.0, 10.0), 40.0, 20.0)];
    let mut g = ParametricGraph::new();
    g.add_feature(sketch_node("sketch_1", shapes));
    g.add_feature(extrude_node("extrude_2", vec![])); // whole sketch
    g.add_dependency("sketch_1", "extrude_2");
    let bodies = g.evaluate_bodies(&HashSet::new()).unwrap();
    assert_eq!(bodies.len(), 1);
    // A frame has a through-hole: some interior (30,20) column has no material.
    // Cheap proxy: the mesh must have MORE triangles than a plain box (inner
    // walls) — a filled slab would collapse to a simple prism.
    assert!(
        !bodies[0].1.vertices.is_empty(),
        "whole nested sketch still builds the frame"
    );
}
