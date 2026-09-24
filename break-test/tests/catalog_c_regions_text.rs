//! Catalog C — sketch regions, trims, offsets, and text (adversarial plan §6.C).
//!
//! C02's crossing/bowtie grids and C04-style line grids are already pinned in
//! `sketch_expr_torture.rs` (FINDINGS E22/E23); this file covers nested
//! material/void stacks, gap-closure tolerance, offset collapse, projected
//! edges under source edits, and glyph geometry (counters, overlap, baked
//! reopen).

mod common;

use common::*;
use std::collections::HashMap;
use zerocad_core::sketch::{
    detect_regions, Dimension, EntityId, SketchEntity, SketchOffsetOperation, SketchPoint,
    SketchSolverModel,
};
use zerocad_core::{Document, ProjectDocument};
use zerocad_core::{ExtrudeMode, FeatureType, ParametricGraph, Unit};

const HACK: &[u8] = epaint_default_fonts::HACK_REGULAR;

/// C01: nested loops with alternating material and void. Region areas must
/// reflect the nesting, and extruding the middle ring yields a tube with the
/// inner disk and outer surroundings empty.
#[test]
fn c01_nested_loops_alternate_material_and_void() {
    let mut c = zerocad_core::SketchCurves::new();
    c.add_rectangle((-20.0, -20.0), (20.0, 20.0));
    c.add_circle((0.0, 0.0), 15.0);
    c.add_circle((0.0, 0.0), 5.0);
    let regions = detect_regions(&c);
    assert_eq!(regions.len(), 3, "3 nested regions expected: {:?}", {
        let mut v: Vec<f32> = regions.iter().map(|r| r.area).collect();
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        v
    });
    let mut areas: Vec<f64> = regions.iter().map(|r| r.area as f64).collect();
    areas.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let pi = std::f64::consts::PI;
    assert_close(areas[0], pi * 25.0, 1e-3, 0.02, "inner disk area");
    assert_close(
        areas[1],
        pi * 225.0 - pi * 25.0,
        1e-3,
        0.02,
        "middle ring area",
    );
    assert_close(areas[2], 1600.0 - pi * 225.0, 1e-3, 0.02, "outer ring area");

    // Extrude every region: the bounded regions tile the plate, total area
    // is the full rectangle.
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sk_1", c.clone());
    add_extrude(&mut g, "ex_2", "sk_1", 4.0, ExtrudeMode::NewBody);
    assert_close(
        assert_catalog_sane(&g),
        1600.0 * 4.0,
        1e-3,
        0.02,
        "full-tile extrusion volume",
    );

    // Select only the middle ring (index by containment, not position).
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sk_1", c);
    let ring_index = regions
        .iter()
        .position(|r| r.contains((10.0, 0.0)) && !r.contains((0.0, 0.0)))
        .expect("the middle ring region");
    add_feature(
        &mut g,
        "ex_2",
        FeatureType::Extrude {
            depth: 4.0,
            region_indices: vec![ring_index],
            mode: ExtrudeMode::NewBody,
            target: None,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
        &["sk_1"],
    );
    let bodies = eval(&g);
    assert_meshes_valid(&bodies);
    assert_close(
        total_volume(&bodies),
        (pi * 225.0 - pi * 25.0) * 4.0,
        1e-3,
        0.03,
        "ring-only extrusion volume",
    );
    assert_occupancy(
        &bodies[0].1,
        &[[10.0, 0.0, 2.0]],
        &[[0.0, 0.0, 2.0], [17.0, 0.0, 2.0]],
    );
}

/// C02: duplicate lines, reversed duplicates, and zero-length segments must
/// normalize deliberately — duplicated geometry must not invent regions, and
/// zero-length segments must be ignored.
#[test]
fn c02_duplicate_and_degenerate_geometry_normalizes() {
    // Duplicated + reversed-duplicate circles inside a rectangle.
    let mut c = zerocad_core::SketchCurves::new();
    c.add_rectangle((-10.0, -10.0), (10.0, 10.0));
    c.add_circle((0.0, 0.0), 4.0);
    c.add_circle((0.0, 0.0), 4.0); // exact duplicate
    let dup = detect_regions(&c);
    assert_eq!(
        dup.len(),
        2,
        "duplicate circle must not create extra regions"
    );

    // Reversed duplicate lines across the rectangle.
    let mut c = zerocad_core::SketchCurves::new();
    c.add_rectangle((-10.0, -10.0), (10.0, 10.0));
    c.add_line((-10.0, 0.0), (10.0, 0.0));
    c.add_line((10.0, 0.0), (-10.0, 0.0)); // reversed duplicate
    let split = detect_regions(&c);
    assert_eq!(
        split.len(),
        2,
        "one splitting line → two halves regardless of direction"
    );

    // Zero-length segment: ignored, no region, no panic.
    let mut c = zerocad_core::SketchCurves::new();
    c.add_rectangle((-10.0, -10.0), (10.0, 10.0));
    c.add_line((3.0, 3.0), (3.0, 3.0));
    let degenerate = detect_regions(&c);
    assert_eq!(degenerate.len(), 1, "zero-length segment must be ignored");
    for r in &degenerate {
        assert!(r.area.is_finite() && r.area > 0.0);
    }
}

/// C03: endpoint gaps swept through the closure tolerance. A meaningful gap
/// must never be silently closed; sub-tolerance gaps close (or stay open)
/// deterministically.
#[test]
fn c03_endpoint_gap_closure_tolerance() {
    let square_with_gap = |g: f32| {
        let mut c = zerocad_core::SketchCurves::new();
        c.add_line((0.0, 0.0), (10.0, 0.0));
        c.add_line((10.0, 0.0), (10.0, 10.0));
        c.add_line((10.0, 10.0), (0.0, 10.0));
        c.add_line((0.0, 10.0), (0.0, g)); // leaves a gap of size g at the corner
        c
    };
    // A meaningful gap must not close.
    for gap in [0.5_f32, 0.1, 0.01] {
        let regions = detect_regions(&square_with_gap(gap));
        assert!(
            regions.is_empty(),
            "gap {gap} was silently closed into {} region(s)",
            regions.len()
        );
    }
    // Sub-tolerance gaps: whatever the closure contract is, it must be
    // deterministic and, if closed, exactly the full square.
    for gap in [1e-5_f32, 1e-6, 1e-7] {
        let a = detect_regions(&square_with_gap(gap));
        let b = detect_regions(&square_with_gap(gap));
        assert_eq!(
            a.len(),
            b.len(),
            "gap {gap}: region detection not deterministic"
        );
        if !a.is_empty() {
            assert_eq!(a.len(), 1, "gap {gap}: closed into {} regions", a.len());
            assert_close(a[0].area as f64, 100.0, 1e-3, 0.02, "gap {gap} closed area");
        }
    }
}

/// C04: several intersections at one point — four lines through the circle
/// center make exact quarter sectors.
#[test]
fn c04_multiple_intersections_at_one_point() {
    let r = 8.0_f32;
    let mut c = zerocad_core::SketchCurves::new();
    c.add_circle((0.0, 0.0), r);
    c.add_line((-r, 0.0), (r, 0.0));
    c.add_line((0.0, -r), (0.0, r));
    let regions = detect_regions(&c);
    assert_eq!(regions.len(), 4, "four quarter-disk sectors");
    for reg in &regions {
        assert_close(
            reg.area as f64,
            std::f64::consts::PI * 64.0 / 4.0,
            1e-3,
            0.05,
            "sector area",
        );
    }
}

/// C05: offsets through collapse — an inward offset past the radius fails
/// atomically (no partial geometry); a feasible offset returns the offset
/// curve set.
#[test]
fn c05_offset_sweep_through_collapse_is_atomic() {
    let model_with_circle = |radius: f64| {
        let mut m = SketchSolverModel::default();
        m.points.push(SketchPoint {
            id: EntityId(0),
            pos: (0.0, 0.0),
        });
        m.entities.push(SketchEntity::Circle {
            id: EntityId(1),
            center: EntityId(0),
            radius,
            derived_from: None,
        });
        m
    };
    let op = |dist: f32| SketchOffsetOperation {
        id: EntityId(10),
        sources: vec![EntityId(1)],
        distance: Dimension::literal(dist),
        creation_side_seed: (0.0, 0.0), // center seed → positive distance offsets inward
    };
    let vars = HashMap::new();

    // Feasible inward offset.
    let ok = zerocad_core::sketch::evaluate_associative_offset(
        &model_with_circle(10.0),
        &op(4.0),
        &vars,
    );
    assert!(ok.is_ok(), "feasible offset failed: {ok:?}");

    // Collapse: at and past the radius.
    for dist in [10.0_f32, 12.0, 1e9] {
        let out = zerocad_core::sketch::evaluate_associative_offset(
            &model_with_circle(10.0),
            &op(dist),
            &vars,
        );
        assert!(out.is_err(), "offset {dist} must fail, returned {out:?}");
    }

    // Missing source: attributed error, nothing partial.
    let mut bad_op = op(4.0);
    bad_op.sources = vec![EntityId(99)];
    let out =
        zerocad_core::sketch::evaluate_associative_offset(&model_with_circle(10.0), &bad_op, &vars);
    assert!(out.is_err(), "missing source must fail: {out:?}");

    // Invalid distance.
    let out = zerocad_core::sketch::evaluate_associative_offset(
        &model_with_circle(10.0),
        &op(f32::NAN),
        &vars,
    );
    assert!(out.is_err(), "NaN distance must fail: {out:?}");
}

/// C06: trimming and projected edges under source edits — the projected edge
/// refreshes in place when the source moves, and refuses (leaving the model
/// untouched) when the curve kind would change.
#[test]
fn c06_projected_edge_rebuild_follows_source_and_refuses_kind_change() {
    use zerocad_core::mock_kernel::EdgeCurveHint;
    use zerocad_core::{CoordinateSystem, EdgeRef};

    let edge_ref = |len: f32| EdgeRef {
        p0: [0.0, 0.0, 0.0],
        p1: [len, 0.0, 0.0],
        n1: [0.0, 0.0, -1.0],
        n2: [0.0, 1.0, 0.0],
        curve: Some(EdgeCurveHint::Line),
        topology: None,
    };

    let mut model = SketchSolverModel::default();
    let mut next = 100u32;
    zerocad_core::sketch::append_projected_edge(
        &mut model,
        "box_1".to_string(),
        edge_ref(20.0),
        CoordinateSystem::XY,
        &mut next,
    )
    .expect("projection of a straight edge must succeed");
    assert_eq!(model.projected_edges.len(), 1);
    let points_before = model.points.clone();

    // Source edit: the edge grows to 30 — rebuild refreshes the projected
    // endpoints in place.
    zerocad_core::sketch::rebuild_projected_edge(
        &mut model,
        0,
        edge_ref(30.0),
        CoordinateSystem::XY,
    )
    .expect("same-kind rebuild must succeed");
    assert_ne!(
        model.points, points_before,
        "the successful rebuild must have moved the projected endpoints"
    );

    // Kind change: a circular source edge — rebuild must refuse and leave
    // the model untouched.
    let circular = EdgeRef {
        p0: [0.0, 0.0, 0.0],
        p1: [30.0, 0.0, 0.0],
        n1: [0.0, 0.0, -1.0],
        n2: [0.0, 1.0, 0.0],
        curve: Some(EdgeCurveHint::Circle {
            center: [15.0, 0.0, 0.0],
            axis: [0.0, 0.0, 1.0],
            x_dir: [1.0, 0.0, 0.0],
            radius: 15.0,
            start: 0.0,
            end: 180.0,
            closed: false,
        }),
        topology: None,
    };
    let before = model.entities.clone();
    let refused =
        zerocad_core::sketch::rebuild_projected_edge(&mut model, 0, circular, CoordinateSystem::XY);
    assert!(
        refused.is_err(),
        "kind-changing rebuild must refuse: {refused:?}"
    );
    assert_eq!(
        model.entities, before,
        "refused rebuild must not touch the model"
    );
    assert_ne!(
        model.points, points_before,
        "the successful rebuild must have moved geometry"
    );
}

/// C07: text with counters and overlapping glyphs — holes in glyphs remain
/// voids and overlapping ink unions instead of creating internal walls.
#[test]
fn c07_text_counters_and_overlap_fill_correctly() {
    use zerocad_core::text::{bake_text_shape, text_regions, TextParams, TextPlacement};

    // Counters: 'o' has one; '8' has two. Region holes must survive.
    let regions = zerocad_core::text::text_regions(
        HACK,
        0,
        "o8",
        &TextParams::default(),
        TextPlacement::default(),
    )
    .expect("text regions");
    assert!(!regions.is_empty(), "glyph ink must produce regions");
    let holes: usize = regions.iter().map(|r| r.holes.len()).sum();
    assert!(
        holes >= 3,
        "o(1) + 8(2) counters must remain voids; found {holes} holes"
    );
    for r in &regions {
        assert!(
            r.area.is_finite() && r.area > 0.0,
            "degenerate ink region {r:?}"
        );
    }

    // Overlapping glyphs (negative tracking): ink must union — the region
    // count stays small and no sliver regions appear between glyph walls.
    let mut tight = TextParams::default();
    tight.tracking_mm = -6.0; // pull glyphs into each other
    let overlapped = text_regions(HACK, 0, "mm", &tight, TextPlacement::default())
        .expect("overlapped text regions");
    for r in &overlapped {
        assert!(
            r.area > 0.5,
            "overlapping ink must not shatter into slivers: {r:?}"
        );
    }

    // The baked shape is a single sketch item that renders standalone.
    let shape = bake_text_shape(
        HACK,
        0,
        "Rev 2",
        &TextParams::default(),
        TextPlacement {
            origin: (4.0, 7.0),
            rotation_deg: 30.0,
            ..TextPlacement::default()
        },
    )
    .expect("bake");
    let built = shape.build(&HashMap::new());
    assert!(
        !built.segments.is_empty() || !built.circles.is_empty() || !built.splines.is_empty(),
        "baked text must carry curve geometry"
    );
}

/// C08: baked text reopens without the font — baked outlines are authoritative
/// and survive a save/load cycle bit-for-bit in evaluated geometry.
#[test]
fn c08_baked_text_survives_reopen_without_font() {
    use zerocad_core::sketch::SketchShape;
    use zerocad_core::text::{bake_text_shape, TextParams, TextPlacement};

    let shape = bake_text_shape(
        HACK,
        0,
        "ZeroCAD 8",
        &TextParams::default(),
        TextPlacement::default(),
    )
    .expect("bake");
    let SketchShape::Text { curves, .. } = &shape else {
        panic!("expected a text shape");
    };
    let curves_before = curves.clone();

    let mut g = ParametricGraph::new();
    add_feature(
        &mut g,
        "sk_1",
        FeatureType::Sketch {
            cs: zerocad_core::CoordinateSystem::XY,
            curves: zerocad_core::SketchCurves::new(),
            shapes: vec![shape],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
            entity_ids: vec![],
            next_entity_id: 1,
            solver: None,
        },
        &[],
    );
    add_extrude(&mut g, "ex_2", "sk_1", 2.0, ExtrudeMode::NewBody);
    let before = assert_catalog_sane(&g);

    // Round trip. No font catalog participates in load: the baked outlines
    // are the authoritative persisted geometry.
    let doc = Document::from_graph(g, Unit::Millimeter);
    let bytes = zerocad_core::write_project_document_to_vec(
        &ProjectDocument::Part(doc),
        &zerocad_core::SaveOptions::default(),
        &zerocad_core::HydrationBundle::default(),
    )
    .expect("save");
    let loaded = zerocad_core::read_project_document_from_slice(
        &bytes,
        &zerocad_core::LoadOptions::default(),
    )
    .expect("load");
    let ProjectDocument::Part(part) = &loaded.document else {
        panic!("expected a part document");
    };
    let (bodies, warnings) = part
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(
        warnings.is_empty(),
        "reopened text must not warn: {warnings:?}"
    );
    assert_close(
        total_volume(&bodies),
        before,
        1e-9,
        0.0,
        "reopened text geometry identical",
    );

    // The baked curves themselves are unchanged (no re-shaping on load).
    let mut restored_text = None;
    for rec in part.graph.node_weights() {
        if rec.id == "sk_1" {
            if let FeatureType::Sketch { shapes, .. } = &rec.feature {
                restored_text = shapes.iter().find_map(|s| match s {
                    SketchShape::Text { curves, .. } => Some(curves.clone()),
                    _ => None,
                });
            }
        }
    }
    let restored = restored_text.expect("text shape survives the round trip");
    assert_eq!(
        format!(
            "{:?}",
            restored.segments.len() + restored.circles.len() + restored.splines.len()
        ),
        format!(
            "{:?}",
            curves_before.segments.len()
                + curves_before.circles.len()
                + curves_before.splines.len()
        ),
        "baked curve counts must be preserved"
    );
}
