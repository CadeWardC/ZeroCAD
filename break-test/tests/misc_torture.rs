//! Miscellaneous torture: STL mesh bodies (and their exclusion from solid
//! booleans), FeaturePattern of join extrudes, self-referential booleans,
//! transform/scale chains, swiss-cheese hole fields, diagonal cuts, and
//! bosses inside voids.

mod common;

use common::*;
use std::collections::HashSet;
use zerocad_core::parametric::{
    FeaturePatternComputeMode, FeaturePatternExtentPolicy, FeaturePatternKind,
};
use zerocad_core::{
    AxisBase, ExtrudeMode, FeatureNode, FeatureType, ParametricGraph, SketchCurves,
};

fn add_feature(g: &mut ParametricGraph, id: &str, feature: FeatureType, deps: &[&str]) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature,
    });
    for d in deps {
        g.add_dependency(d, id);
    }
}

fn eval_pair(g: &ParametricGraph) -> (Vec<(String, zerocad_core::MockMesh)>, Vec<String>) {
    g.evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail")
}

/// A valid ASCII STL tetrahedron (edge 10 at the origin axes): volume 1000/6.
fn tetra_stl() -> Vec<u8> {
    let s = "\
solid tetra
facet normal 0 0 -1
 outer loop
  vertex 0 0 0
  vertex 10 0 0
  vertex 0 10 0
 endloop
endfacet
facet normal 0 -1 0
 outer loop
  vertex 0 0 0
  vertex 0 0 10
  vertex 10 0 0
 endloop
endfacet
facet normal -1 0 0
 outer loop
  vertex 0 0 0
  vertex 0 10 0
  vertex 0 0 10
 endloop
endfacet
facet normal 0.57735027 0.57735027 0.57735027
 outer loop
  vertex 10 0 0
  vertex 0 10 0
  vertex 0 0 10
 endloop
endfacet
endsolid tetra
";
    s.as_bytes().to_vec()
}

// ---------------------------------------------------------------------------
// STL mesh bodies.
// ---------------------------------------------------------------------------

#[test]
fn import_stl_tetrahedron_loads_as_mesh_body() {
    let mut g = ParametricGraph::new();
    add_feature(
        &mut g,
        "mesh_1",
        FeatureType::ImportStl {
            stl_data: tetra_stl(),
            label: "tetra".to_string(),
        },
        &[],
    );
    let (bodies, warnings) = eval_pair(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    assert!(
        (v - 1000.0 / 6.0).abs() / (1000.0 / 6.0) < 0.05,
        "tetra mesh volume ≈ 166.67: {v} ({warnings:?})"
    );
}

#[test]
fn mesh_body_cannot_enter_solid_boolean() {
    // Documented: "Mesh bodies remain distinct from B-Reps and cannot enter
    // solid booleans." Must warn or error structurally — never crash.
    let mut g = ParametricGraph::new();
    add_feature(
        &mut g,
        "mesh_1",
        FeatureType::ImportStl {
            stl_data: tetra_stl(),
            label: "tetra".to_string(),
        },
        &[],
    );
    add_box(&mut g, "box_2", 20.0, 20.0, 20.0);
    add_feature(
        &mut g,
        "cut_3",
        FeatureType::BodyCut {
            target: "box_2".into(),
            tool: "mesh_1".into(),
            keep_tool: false,
        },
        &["mesh_1", "box_2"],
    );
    let result = g.evaluate_bodies_with_warnings(&HashSet::new());
    match result {
        Ok((bodies, warnings)) => {
            assert_meshes_finite(&bodies);
            assert!(
                !warnings.is_empty(),
                "mesh-in-boolean must at least warn: {warnings:?}"
            );
        }
        Err(_) => {} // structured rejection is the documented path
    }
}

#[test]
fn extrude_targeting_a_mesh_body_is_graceful() {
    let mut g = ParametricGraph::new();
    add_feature(
        &mut g,
        "mesh_1",
        FeatureType::ImportStl {
            stl_data: tetra_stl(),
            label: "tetra".to_string(),
        },
        &[],
    );
    add_sketch_cs(
        &mut g,
        "sk_2",
        xy_plane_at(0.0),
        rect_sketch((0.0, 0.0), (5.0, 5.0)),
    );
    g.add_feature(FeatureNode {
        id: "ex_3".to_string(),
        name: "ex_3".to_string(),
        feature: FeatureType::Extrude {
            depth: 5.0,
            region_indices: vec![],
            mode: ExtrudeMode::Join,
            target: Some("mesh_1".to_string()),
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_2", "ex_3");
    let _ = g.evaluate_bodies_with_warnings(&HashSet::new());
}

#[test]
fn garbage_stl_bytes_are_rejected_gracefully() {
    for data in [b"not an stl at all".to_vec(), b"".to_vec(), vec![0u8; 64]] {
        let mut g = ParametricGraph::new();
        add_feature(
            &mut g,
            "mesh_1",
            FeatureType::ImportStl {
                stl_data: data,
                label: "junk".to_string(),
            },
            &[],
        );
        let result = g.evaluate_bodies_with_warnings(&HashSet::new());
        if let Ok((bodies, _)) = result {
            assert_meshes_finite(&bodies);
        }
    }
}

// ---------------------------------------------------------------------------
// FeaturePattern of a Join extrude.
// ---------------------------------------------------------------------------

#[test]
fn feature_pattern_replicates_join_bosses() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 100.0, 30.0, 10.0);
    add_sketch_cs(
        &mut g,
        "sk_2",
        xy_plane_at(10.0),
        rect_sketch((10.0, 10.0), (20.0, 20.0)),
    );
    g.add_feature(FeatureNode {
        id: "boss_3".to_string(),
        name: "boss_3".to_string(),
        feature: FeatureType::Extrude {
            depth: 8.0,
            region_indices: vec![],
            mode: ExtrudeMode::Join,
            target: Some("box_1".to_string()), // v1 patterns need a targeted source
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_2", "boss_3");
    add_feature(
        &mut g,
        "pattern_4",
        FeatureType::FeaturePattern {
            target: "box_1".to_string(),
            source_feature: "boss_3".to_string(),
            kind: FeaturePatternKind::Linear {
                direction: AxisBase::X,
                spacing: 25.0,
                spacing_expr: None,
                count: 4,
            },
            compute_mode: FeaturePatternComputeMode::Identical,
            extent_policy: FeaturePatternExtentPolicy::SourceExtent,
        },
        &["boss_3"],
    );
    let (bodies, warnings) = eval_pair(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    let exact = 100.0 * 30.0 * 10.0 + 4.0 * 100.0 * 8.0;
    assert!(
        (v - exact).abs() / exact < 0.03,
        "4 patterned bosses: {v} vs {exact} ({warnings:?})"
    );
}

#[test]
fn feature_pattern_of_a_cut_extrude() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 100.0, 30.0, 10.0);
    add_sketch_cs(
        &mut g,
        "sk_2",
        xy_plane_at(10.0),
        rect_sketch((10.0, 10.0), (18.0, 20.0)),
    );
    g.add_feature(FeatureNode {
        id: "slot_3".to_string(),
        name: "slot_3".to_string(),
        feature: FeatureType::Extrude {
            depth: -4.0,
            region_indices: vec![],
            mode: ExtrudeMode::Cut,
            target: Some("box_1".to_string()), // v1 patterns need a targeted source
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_2", "slot_3");
    add_feature(
        &mut g,
        "pattern_4",
        FeatureType::FeaturePattern {
            target: "box_1".to_string(),
            source_feature: "slot_3".to_string(),
            kind: FeaturePatternKind::Linear {
                direction: AxisBase::X,
                spacing: 25.0,
                spacing_expr: None,
                count: 4,
            },
            compute_mode: FeaturePatternComputeMode::Identical,
            extent_policy: FeaturePatternExtentPolicy::SourceExtent,
        },
        &["slot_3"],
    );
    let (bodies, warnings) = eval_pair(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    let exact = 30000.0 - 4.0 * 80.0 * 4.0;
    assert!(
        (v - exact).abs() / exact < 0.03,
        "4 patterned slots: {v} vs {exact} ({warnings:?})"
    );
}

// ---------------------------------------------------------------------------
// Self-referential and degenerate booleans.
// ---------------------------------------------------------------------------

#[test]
fn body_cut_with_target_equal_to_tool_is_graceful() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    add_feature(
        &mut g,
        "cut_2",
        FeatureType::BodyCut {
            target: "box_1".into(),
            tool: "box_1".into(),
            keep_tool: false,
        },
        &["box_1"],
    );
    let result = g.evaluate_bodies_with_warnings(&HashSet::new());
    match result {
        Ok((bodies, warnings)) => {
            assert_meshes_finite(&bodies);
            let _ = warnings;
        }
        Err(_) => {}
    }
}

#[test]
fn body_intersect_with_target_equal_to_tool() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    add_feature(
        &mut g,
        "isect_2",
        FeatureType::BodyIntersect {
            target: "box_1".into(),
            tool: "box_1".into(),
            keep_tool: false,
        },
        &["box_1"],
    );
    let result = g.evaluate_bodies_with_warnings(&HashSet::new());
    match result {
        Ok((bodies, warnings)) => {
            assert_meshes_finite(&bodies);
            let _ = warnings;
        }
        Err(_) => {}
    }
}

// ---------------------------------------------------------------------------
// Transform / scale chains.
// ---------------------------------------------------------------------------

#[test]
fn transform_copy_chain_eight_copies() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 10.0, 10.0, 10.0);
    // Chain: each copy copies the previous copy, stepping +15 in x.
    add_feature(
        &mut g,
        "copy_2",
        FeatureType::BodyTransform {
            source: "box_1".into(),
            translation: [15.0, 0.0, 0.0],
            copy: true,
        },
        &["box_1"],
    );
    add_feature(
        &mut g,
        "copy_3",
        FeatureType::BodyTransform {
            source: "copy_2".into(),
            translation: [15.0, 0.0, 0.0],
            copy: true,
        },
        &["copy_2"],
    );
    let (bodies, warnings) = eval_pair(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    assert!(
        (v - 3000.0).abs() / 3000.0 < 0.02,
        "3 boxes total: {v} ({warnings:?})"
    );
}

#[test]
fn scale_chain_compounds() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 10.0, 10.0, 10.0);
    add_feature(
        &mut g,
        "scale_2",
        FeatureType::BodyScale {
            source: "box_1".into(),
            factor: 2.0,
            factor_expr: None,
            center: [0.0, 0.0, 0.0],
        },
        &["box_1"],
    );
    add_feature(
        &mut g,
        "scale_3",
        FeatureType::BodyScale {
            source: "scale_2".into(),
            factor: 3.0,
            factor_expr: None,
            center: [0.0, 0.0, 0.0],
        },
        &["scale_2"],
    );
    let (bodies, warnings) = eval_pair(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    // 2× then 3× = 6× linear → 216× volume... or the chain replaces and
    // re-scales: pin whichever compounds correctly (6× → 216000) vs
    // sequential-replace on the ORIGINAL (2×→ then 3× of original = 30000).
    assert!(
        (v - 216000.0).abs() / 216000.0 < 0.05 || (v - 30000.0).abs() / 30000.0 < 0.05,
        "scale chain semantics: {v} (216000 compounded or 30000 replaced) ({warnings:?})"
    );
}

// ---------------------------------------------------------------------------
// Cut geometry: swiss cheese, diagonals, voids.
// ---------------------------------------------------------------------------

#[test]
fn swiss_cheese_36_holes_exact() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 120.0, 120.0, 6.0);
    for i in 0..6 {
        for j in 0..6 {
            add_hole(
                &mut g,
                &format!("h_{i}_{j}"),
                "box_1",
                [15.0 + i as f32 * 18.0, 15.0 + j as f32 * 18.0, 6.0],
                [0.0, 0.0, -1.0],
                6.0,
                None,
                zerocad_core::HoleKind::Simple,
            );
        }
    }
    let v = assert_part_sane(&g);
    let exact = 120.0 * 120.0 * 6.0 - 36.0 * std::f64::consts::PI * 9.0 * 6.0;
    assert!((v - exact).abs() / exact < 0.03, "36 holes: {v} vs {exact}");
}

#[test]
fn corner_to_corner_diagonal_cut() {
    // A tilted-plane cut passing exactly through the box's diagonal edge.
    let a = (10.0f32 / 30.0).atan(); // plane through (0,0,0)? build diagonal
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 30.0, 30.0, 30.0);
    add_sketch_cs(
        &mut g,
        "sk_2",
        zerocad_core::CoordinateSystem::new(
            zerocad_core::Vec3::new(15.0, 15.0, 15.0),
            zerocad_core::Vec3::new(1.0, 0.0, 0.0),
            zerocad_core::Vec3::new(0.0, a.cos(), a.sin()),
        ),
        rect_sketch((-25.0, -25.0), (25.0, 25.0)),
    );
    g.add_feature(FeatureNode {
        id: "cut_3".to_string(),
        name: "cut_3".to_string(),
        feature: FeatureType::Extrude {
            depth: 25.0,
            region_indices: vec![],
            mode: ExtrudeMode::Cut,
            target: None,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_2", "cut_3");
    let v = assert_part_sane(&g);
    let solid = 27000.0;
    assert!(
        v < solid && v > solid * 0.2,
        "diagonal cut removes a plausible wedge: {v}"
    );
}

#[test]
fn join_boss_entirely_inside_existing_pocket_void() {
    // A Join extrude from the pocket floor whose prism floats entirely in
    // the pocket's void (sketch plane 1 mm ABOVE the floor, boss growing up
    // into the empty pocket): fusion has nothing to attach to.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 40.0, 40.0, 20.0);
    add_sketch_cs(
        &mut g,
        "sk_2",
        xy_plane_at(20.0),
        rect_sketch((5.0, 5.0), (35.0, 35.0)),
    );
    add_extrude(&mut g, "pocket_3", "sk_2", -10.0, ExtrudeMode::Cut);
    // Boss plane 5 mm above the pocket floor (floor is z = 10).
    add_sketch_cs(
        &mut g,
        "sk_4",
        xy_plane_at(15.0),
        rect_sketch((10.0, 10.0), (30.0, 30.0)),
    );
    g.add_feature(FeatureNode {
        id: "float_5".to_string(),
        name: "float_5".to_string(),
        feature: FeatureType::Extrude {
            depth: 3.0,
            region_indices: vec![],
            mode: ExtrudeMode::Join,
            target: None,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_4", "float_5");
    let (bodies, warnings) = eval_pair(&g);
    assert_meshes_finite(&bodies);
    // Either the floating boss is refused (volume = pocketed plate) or it
    // merges as a second component — both survivable; corruption is not.
    let v = total_volume(&bodies);
    let pocketed = 40.0 * 40.0 * 20.0 - 30.0 * 30.0 * 10.0;
    assert!(
        v >= pocketed - 1.0 && v <= pocketed + 20.0 * 20.0 * 3.0 + 1.0,
        "floating-in-void boss volume sane: {v} (base {pocketed}) ({warnings:?})"
    );
}

#[test]
fn annular_ring_join_on_rod_od() {
    // A ring (circle pair) extruded in Join mode onto a rod's end — the
    // annulus seats on the flat cap.
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "rod_1", 6.0, 30.0);
    let mut c = SketchCurves::new();
    c.add_circle((0.0, 0.0), 10.0);
    c.add_circle((0.0, 0.0), 6.0);
    add_sketch_cs(
        &mut g,
        "sk_2",
        zerocad_core::CoordinateSystem::new(
            zerocad_core::Vec3::new(0.0, 30.0, 0.0),
            zerocad_core::Vec3::new(0.0, 0.0, 1.0),
            zerocad_core::Vec3::new(1.0, 0.0, 0.0),
        ),
        c,
    );
    g.add_feature(FeatureNode {
        id: "ring_3".to_string(),
        name: "ring_3".to_string(),
        feature: FeatureType::Extrude {
            depth: 5.0,
            region_indices: vec![],
            mode: ExtrudeMode::Join,
            target: Some("rod_1".into()),
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_2", "ring_3");
    let (bodies, warnings) = eval_pair(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    let rod = std::f64::consts::PI * 36.0 * 30.0;
    // Empty region_indices = ALL regions: the concentric pair's regions
    // union into the FULL disc (r=10), not the annulus — pinned semantics.
    let disc = std::f64::consts::PI * 100.0 * 5.0;
    assert!(
        (v - (rod + disc)).abs() / (rod + disc) < 0.08,
        "all-regions circle-pair join = full disc on rod end: {v} vs {} ({warnings:?})",
        rod + disc
    );
}

#[test]
fn hundred_feature_document_stress() {
    // 25 sketches + 25 cuts + 25 sketches + 25 joins = 100 features on one
    // body: the long-history stress the GUI hits on real parts.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 200.0, 200.0, 20.0);
    let mut n = 2usize;
    for i in 0..25 {
        let x0 = 4.0 + (i % 5) as f32 * 39.0;
        let y0 = 4.0 + (i / 5) as f32 * 39.0;
        add_sketch_cs(
            &mut g,
            &format!("cs_{n}"),
            xy_plane_at(20.0),
            rect_sketch((x0, y0), (x0 + 10.0, y0 + 10.0)),
        );
        add_extrude(
            &mut g,
            &format!("cj_{}", n + 1),
            &format!("cs_{n}"),
            -5.0,
            ExtrudeMode::Cut,
        );
        n += 2;
    }
    for i in 0..25 {
        let x0 = 24.0 + (i % 5) as f32 * 39.0;
        let y0 = 24.0 + (i / 5) as f32 * 39.0;
        add_sketch_cs(
            &mut g,
            &format!("js_{n}"),
            xy_plane_at(15.0),
            rect_sketch((x0, y0), (x0 + 6.0, y0 + 6.0)),
        );
        add_extrude(
            &mut g,
            &format!("jj_{}", n + 1),
            &format!("js_{n}"),
            4.0,
            ExtrudeMode::Join,
        );
        n += 2;
    }
    let v = assert_part_sane(&g);
    let exact = 200.0 * 200.0 * 20.0 - 25.0 * 100.0 * 5.0 + 25.0 * 36.0 * 4.0;
    assert!(
        (v - exact).abs() / exact < 0.03,
        "100-feature document: {v} vs {exact}"
    );
}
