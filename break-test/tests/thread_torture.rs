//! Thread torture: modeled threads (external + internal), threads interacting
//! with other features — cross-drills through threaded regions, cutting the
//! threaded end off, bosses on threaded ends, threaded rods used as boolean
//! tools, threads on tubes and revolved bodies, and degenerate thread
//! parameters.

mod common;

use common::*;
use std::collections::HashSet;
use zerocad_core::parametric::{FaceRef, TopologyFaceRef};
use zerocad_core::{
    CoordinateSystem, ExtrudeMode, FeatureNode, FeatureType, MockMesh, ParametricGraph, Vec3,
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

fn add_thread(
    g: &mut ParametricGraph,
    id: &str,
    target: &str,
    face: FaceRef,
    internal: bool,
    pitch: f32,
    depth: f32,
    length: Option<f32>,
    flip: bool,
    starts: u32,
    right_handed: bool,
) {
    add_feature(
        g,
        id,
        FeatureType::Thread {
            target: target.to_string(),
            face,
            internal,
            pitch,
            depth,
            angle_deg: 60.0,
            right_handed,
            starts,
            length,
            flip,
            designation: "M12x1".to_string(),
            standard: None,
        },
        &[target],
    );
}

/// Capture the cylindrical wall face nearest `near`. Curved faces are found
/// by `surface_kind == "cylinder"` when named (primitive rods) or by their
/// zero-length normal (cut bores).
fn cyl_face_near(g: &ParametricGraph, body: &str, near: [f32; 3]) -> FaceRef {
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("capture eval must not fail");
    let mesh = &bodies.iter().find(|(id, _)| id == body).unwrap().1;
    let is_cyl = |f: &zerocad_core::mock_kernel::MeshFaceRef| {
        f.topology
            .as_ref()
            .and_then(|t| t.surface_kind.as_ref().map(|k| k.as_str()))
            .is_some_and(|k| k.eq_ignore_ascii_case("cylinder"))
            || f.normal[0].abs() + f.normal[1].abs() + f.normal[2].abs() < 0.1
    };
    let f = mesh
        .face_refs
        .iter()
        .filter(|f| is_cyl(f))
        .min_by_key(|f| {
            let d = [
                (f.centroid[0] - near[0]).abs(),
                (f.centroid[1] - near[1]).abs(),
                (f.centroid[2] - near[2]).abs(),
            ];
            ((d[0] + d[1] + d[2]) * 100.0) as u32
        })
        .unwrap_or_else(|| {
            panic!(
                "a cylindrical face must be selectable; warnings: {warnings:?}; faces: {:?}",
                mesh.face_refs
                    .iter()
                    .map(|face| (face.centroid, face.normal, &face.topology))
                    .collect::<Vec<_>>()
            )
        });
    FaceRef {
        centroid: f.centroid,
        normal: f.normal,
        topology: f.topology.as_ref().map(|t| TopologyFaceRef {
            body_id: t.body_id.clone(),
            component_id: t.component_id.clone(),
            topology_version: t.topology_version,
            face_id: t.face_id.clone(),
            surface_kind: t.surface_kind.clone(),
            producer_feature_id: t.producer_feature_id.clone(),
            source_entity_id: t.source_entity_id.clone(),
        }),
    }
}

fn eval_pair(g: &ParametricGraph) -> (Vec<(String, MockMesh)>, Vec<String>) {
    g.evaluate_bodies_with_warnings(&HashSet::new())
        .expect("must not hard-fail")
}

// ---------------------------------------------------------------------------
// External thread variants.
// ---------------------------------------------------------------------------

#[test]
fn external_thread_full_length_stays_sane() {
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "rod_1", 6.0, 30.0);
    let face = cyl_face_near(&g, "rod_1", [0.0, 15.0, 0.0]);
    add_thread(
        &mut g, "thread_2", "rod_1", face, false, 1.0, 0.15, None, false, 1, true,
    );
    let v = assert_part_sane(&g);
    let rod = std::f64::consts::PI * 36.0 * 30.0;
    assert!(
        (v - rod).abs() / rod < 0.06,
        "threaded rod ≈ rod volume: {v} vs {rod}"
    );
}

#[test]
fn external_thread_partial_length_both_anchors() {
    for flip in [false, true] {
        let mut g = ParametricGraph::new();
        add_cylinder(&mut g, "rod_1", 6.0, 30.0);
        let face = cyl_face_near(&g, "rod_1", [0.0, 15.0, 0.0]);
        add_thread(
            &mut g,
            "thread_2",
            "rod_1",
            face,
            false,
            1.0,
            0.15,
            Some(12.0),
            flip,
            1,
            true,
        );
        let v = assert_part_sane(&g);
        let rod = std::f64::consts::PI * 36.0 * 30.0;
        assert!(
            (v - rod).abs() / rod < 0.06,
            "partial thread (flip={flip}): {v} vs {rod}"
        );
    }
}

#[test]
fn external_thread_left_handed_and_multi_start() {
    for (rh, starts) in [(false, 1u32), (true, 3), (true, 8)] {
        let mut g = ParametricGraph::new();
        add_cylinder(&mut g, "rod_1", 6.0, 30.0);
        let face = cyl_face_near(&g, "rod_1", [0.0, 15.0, 0.0]);
        add_thread(
            &mut g,
            "thread_2",
            "rod_1",
            face,
            false,
            1.5,
            0.2,
            Some(15.0),
            false,
            starts,
            rh,
        );
        let v = assert_part_sane(&g);
        let rod = std::f64::consts::PI * 36.0 * 30.0;
        assert!(
            (v - rod).abs() / rod < 0.08,
            "rh={rh} starts={starts}: {v} vs {rod}"
        );
    }
}

// ---------------------------------------------------------------------------
// Internal threads (tapped holes).
// ---------------------------------------------------------------------------

#[test]
fn internal_thread_in_through_hole() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 30.0, 30.0, 20.0);
    add_hole(
        &mut g,
        "hole_2",
        "box_1",
        [15.0, 15.0, 20.0],
        [0.0, 0.0, -1.0],
        12.0,
        None,
        zerocad_core::HoleKind::Simple,
    );
    let face = cyl_face_near(&g, "box_1", [15.0, 15.0, 10.0]);
    add_thread(
        &mut g, "thread_3", "box_1", face, true, 1.75, 0.2, None, false, 1, true,
    );
    let v = assert_part_sane(&g);
    let exact = 30.0 * 30.0 * 20.0 - std::f64::consts::PI * 36.0 * 20.0;
    assert!(
        (v - exact).abs() / exact < 0.08,
        "tapped plate ≈ plate minus bore: {v} vs {exact}"
    );
}

#[test]
fn internal_thread_partial_depth_tapped() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 30.0, 30.0, 20.0);
    add_hole(
        &mut g,
        "hole_2",
        "box_1",
        [15.0, 15.0, 20.0],
        [0.0, 0.0, -1.0],
        12.0,
        None,
        zerocad_core::HoleKind::Simple,
    );
    let face = cyl_face_near(&g, "box_1", [15.0, 15.0, 10.0]);
    add_thread(
        &mut g,
        "thread_3",
        "box_1",
        face,
        true,
        1.75,
        0.2,
        Some(8.0),
        false,
        1,
        true,
    );
    let v = assert_part_sane(&g);
    let exact = 30.0 * 30.0 * 20.0 - std::f64::consts::PI * 36.0 * 20.0;
    assert!(
        (v - exact).abs() / exact < 0.08,
        "blind tap: {v} vs {exact}"
    );
}

#[test]
fn bolt_shaft_fitted_into_tapped_hole() {
    // Tap a hole, then join a slightly-smaller shaft into it (a bolt in the
    // tapped plate): the assembly must stay manifold and add the annular gap.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 30.0, 30.0, 20.0);
    add_hole(
        &mut g,
        "hole_2",
        "box_1",
        [15.0, 15.0, 20.0],
        [0.0, 0.0, -1.0],
        12.0,
        None,
        zerocad_core::HoleKind::Simple,
    );
    let face = cyl_face_near(&g, "box_1", [15.0, 15.0, 10.0]);
    add_thread(
        &mut g, "thread_3", "box_1", face, true, 1.75, 0.2, None, false, 1, true,
    );
    // Bolt: r=5.5 shaft through the clearance bore PLUS a head flange
    // sitting on the plate's top face (the contact that lets the join fuse —
    // a pure clearance shaft would correctly be refused).
    let mut c = zerocad_core::SketchCurves::new();
    c.add_circle((15.0, 15.0), 5.5);
    add_sketch_cs(&mut g, "sk_4", xy_plane_at(-5.0), c);
    g.add_feature(FeatureNode {
        id: "shaft_5".to_string(),
        name: "shaft_5".to_string(),
        feature: FeatureType::Extrude {
            depth: 25.0,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            target: None,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_4", "shaft_5");
    let mut h = zerocad_core::SketchCurves::new();
    h.add_circle((15.0, 15.0), 9.0);
    add_sketch_cs(&mut g, "sk_6", xy_plane_at(20.0), h);
    g.add_feature(FeatureNode {
        id: "head_7".to_string(),
        name: "head_7".to_string(),
        feature: FeatureType::Extrude {
            depth: 5.0,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            target: None,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_6", "head_7");
    add_feature(
        &mut g,
        "join_8",
        FeatureType::BodyJoin {
            sources: vec![
                "box_1".to_string(),
                "shaft_5".to_string(),
                "head_7".to_string(),
            ],
        },
        &["box_1", "shaft_5", "head_7"],
    );
    let (bodies, warnings) = eval_pair(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    let plate = 30.0 * 30.0 * 20.0 - std::f64::consts::PI * 36.0 * 20.0;
    let shaft = std::f64::consts::PI * 30.25 * 25.0;
    let head = std::f64::consts::PI * 81.0 * 5.0;
    let exact = plate + shaft + head;
    assert!(
        (v - exact).abs() / exact < 0.08,
        "bolt-in-tapped-plate: {v} vs {exact} ({warnings:?})"
    );
}

// ---------------------------------------------------------------------------
// Cutting into and around threads.
// ---------------------------------------------------------------------------

#[test]
fn cross_drill_through_threaded_rod() {
    // The classic cross-hole through a threaded stud.
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "rod_1", 6.0, 30.0);
    let face = cyl_face_near(&g, "rod_1", [0.0, 15.0, 0.0]);
    add_thread(
        &mut g, "thread_2", "rod_1", face, false, 1.0, 0.15, None, false, 1, true,
    );
    // Drill along +Z through the rod middle (plane inside material — see
    // FINDINGS E15 for the BodyCut plane trap; cut-EXTRUDE is safe here).
    let mut c = zerocad_core::SketchCurves::new();
    c.add_circle((0.0, 15.0), 2.0);
    add_sketch_cs(&mut g, "sk_3", xy_plane_at(-8.0), c);
    g.add_feature(FeatureNode {
        id: "drill_4".to_string(),
        name: "drill_4".to_string(),
        feature: FeatureType::Extrude {
            depth: 16.0,
            region_indices: vec![],
            mode: ExtrudeMode::Cut,
            target: Some("rod_1".into()),
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_3", "drill_4");
    let (bodies, warnings) = eval_pair(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    let rod = std::f64::consts::PI * 36.0 * 30.0;
    let hole = std::f64::consts::PI * 4.0 * 12.0;
    assert!(
        (v - (rod - hole)).abs() / (rod - hole) < 0.08,
        "cross-drilled stud: {v} vs {} ({warnings:?})",
        rod - hole
    );
}

#[test]
fn cut_the_threaded_end_off() {
    // Thread only the top half, then cut that half away — the remaining body
    // must be an unthreaded plain cylinder section.
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "rod_1", 6.0, 30.0);
    let face = cyl_face_near(&g, "rod_1", [0.0, 15.0, 0.0]);
    add_thread(
        &mut g,
        "thread_2",
        "rod_1",
        face,
        false,
        1.0,
        0.15,
        Some(15.0),
        false,
        1,
        true,
    );
    let mut c = zerocad_core::SketchCurves::new();
    c.add_rectangle((-10.0, 15.0), (10.0, 40.0));
    add_sketch_cs(&mut g, "sk_3", xy_plane_at(-10.0), c);
    g.add_feature(FeatureNode {
        id: "cut_4".to_string(),
        name: "cut_4".to_string(),
        feature: FeatureType::Extrude {
            depth: 20.0,
            region_indices: vec![],
            mode: ExtrudeMode::Cut,
            target: Some("rod_1".into()),
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_3", "cut_4");
    let v = assert_part_sane(&g);
    let half_rod = std::f64::consts::PI * 36.0 * 15.0;
    assert!(
        (v - half_rod).abs() / half_rod < 0.06,
        "threaded end cut off: {v} vs {half_rod}"
    );
}

#[test]
fn boss_extruded_onto_threaded_rod_end() {
    // Join a collar onto the end of a threaded rod.
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "rod_1", 6.0, 30.0);
    let face = cyl_face_near(&g, "rod_1", [0.0, 15.0, 0.0]);
    add_thread(
        &mut g, "thread_2", "rod_1", face, false, 1.0, 0.15, None, false, 1, true,
    );
    // Collar: cylinder r=9 from the y=30 end cap outward.
    let mut c = zerocad_core::SketchCurves::new();
    c.add_circle((0.0, 0.0), 9.0);
    add_sketch_cs(
        &mut g,
        "sk_3",
        CoordinateSystem::new(
            Vec3::new(0.0, 30.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        ),
        c,
    );
    g.add_feature(FeatureNode {
        id: "collar_4".to_string(),
        name: "collar_4".to_string(),
        feature: FeatureType::Extrude {
            depth: 6.0,
            region_indices: vec![],
            mode: ExtrudeMode::Join,
            target: Some("rod_1".into()),
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_3", "collar_4");
    let (bodies, warnings) = eval_pair(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    let rod = std::f64::consts::PI * 36.0 * 30.0;
    let collar_max = std::f64::consts::PI * 81.0 * 6.0;
    assert!(
        v > rod * 0.95 && v < rod + collar_max * 1.05,
        "collar on threads adds bounded material: {v} (rod {rod}, collar_max {collar_max}) ({warnings:?})"
    );
}

#[test]
fn threaded_rod_used_as_body_cut_tool() {
    // Imprint threads into a plate: BodyCut with a threaded rod as the tool.
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "rod_1", 6.0, 30.0);
    let face = cyl_face_near(&g, "rod_1", [0.0, 15.0, 0.0]);
    add_thread(
        &mut g, "thread_2", "rod_1", face, false, 1.0, 0.15, None, false, 1, true,
    );
    // Plate across the rod's middle (rod along +Y; plate in the XZ plane).
    let mut c = zerocad_core::SketchCurves::new();
    c.add_rectangle((-15.0, -15.0), (15.0, 15.0));
    add_sketch_cs(
        &mut g,
        "sk_3",
        CoordinateSystem::new(
            Vec3::new(0.0, 12.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        ),
        c,
    );
    g.add_feature(FeatureNode {
        id: "plate_4".to_string(),
        name: "plate_4".to_string(),
        feature: FeatureType::Extrude {
            depth: 8.0,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            target: None,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_3", "plate_4");
    add_feature(
        &mut g,
        "cut_5",
        FeatureType::BodyCut {
            target: "plate_4".into(),
            tool: "rod_1".into(),
            keep_tool: false,
        },
        &["thread_2", "plate_4"],
    );
    let (bodies, warnings) = eval_pair(&g);
    assert_meshes_finite(&bodies);
    // E20 (pinned): subtracting a THREADED body currently fails — "the
    // solver could not subtract the overlapping body" — and both bodies are
    // preserved WITH a warning. When fixed, flip to the pierced volume.
    let v = total_volume(&bodies);
    let plate = 900.0 * 8.0;
    let pierced = plate - std::f64::consts::PI * 36.0 * 8.0;
    assert!(
        warnings.iter().any(|w| w.contains("could not subtract")),
        "threaded-tool cut currently fails: {warnings:?}"
    );
    assert!(
        (v - (plate + std::f64::consts::PI * 36.0 * 30.0)).abs()
            / (plate + std::f64::consts::PI * 36.0 * 30.0)
            < 0.05,
        "pinning current behavior: plate + threaded rod both preserved ({v}); pierced would be {pierced}"
    );
}

#[test]
fn thread_after_turning_the_rod_down() {
    // Make a stepped shaft (turn r=10 down to r=6 on half), then thread the
    // untouched r=10 section — the face must reattach after the geometry edit.
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "rod_1", 10.0, 30.0);
    let mut c = zerocad_core::SketchCurves::new();
    c.add_circle((0.0, 0.0), 6.0);
    add_sketch_cs(
        &mut g,
        "sk_2",
        CoordinateSystem::new(
            Vec3::new(0.0, 30.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        ),
        c,
    );
    g.add_feature(FeatureNode {
        id: "turn_3".to_string(),
        name: "turn_3".to_string(),
        feature: FeatureType::Extrude {
            depth: 15.0,
            region_indices: vec![],
            mode: ExtrudeMode::Cut,
            target: Some("rod_1".into()),
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_2", "turn_3");
    // Capture the remaining large section's wall (y ≈ 10 center).
    let face = cyl_face_near(&g, "rod_1", [0.0, 10.0, 0.0]);
    add_thread(
        &mut g,
        "thread_4",
        "rod_1",
        face,
        false,
        1.5,
        0.2,
        Some(10.0),
        false,
        1,
        true,
    );
    let v = assert_part_sane(&g);
    // Turn-down leaves a full r=10 lower half plus an r=10/r=6 annulus top.
    let large = std::f64::consts::PI * 100.0 * 15.0;
    let annulus = std::f64::consts::PI * (100.0 - 36.0) * 15.0;
    assert!(
        (v - (large + annulus)).abs() / (large + annulus) < 0.08,
        "turn+thread: {v}"
    );
}

#[test]
fn thread_on_revolved_body_cylinder() {
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sk_1", rect_sketch((10.0, 0.0), (30.0, 40.0)));
    g.add_feature(FeatureNode {
        id: "rev_2".to_string(),
        name: "rev_2".to_string(),
        feature: FeatureType::Revolve {
            axis: zerocad_core::AxisBase::X,
            angle_deg: 360.0,
            angle_expr: None,
            region_indices: vec![],
            mode: zerocad_core::ExtrudeMode::NewBody,
            target: None,
        },
    });
    g.add_dependency("sk_1", "rev_2");
    let face = cyl_face_near(&g, "rev_2", [20.0, 0.0, 0.0]);
    add_thread(
        &mut g,
        "thread_3",
        "rev_2",
        face,
        false,
        2.0,
        0.25,
        Some(10.0),
        false,
        1,
        true,
    );
    let v = assert_part_sane(&g);
    let hub = std::f64::consts::PI * 1600.0 * 20.0;
    assert!((v - hub).abs() / hub < 0.08, "threaded hub: {v} vs {hub}");
}

#[test]
fn thread_external_on_tube_with_coaxial_bore() {
    // Tube: rod r=10 with coaxial r=6 bore; thread the OUTER wall (captured
    // before the bore exists so the wall face is unambiguous).
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "rod_1", 10.0, 30.0);
    let outer = cyl_face_near(&g, "rod_1", [0.0, 15.0, 0.0]);
    let mut c = zerocad_core::SketchCurves::new();
    c.add_circle((0.0, 0.0), 6.0);
    add_sketch_cs(
        &mut g,
        "sk_2",
        CoordinateSystem::new(
            Vec3::new(0.0, -5.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        ),
        c,
    );
    g.add_feature(FeatureNode {
        id: "bore_3".to_string(),
        name: "bore_3".to_string(),
        feature: FeatureType::Extrude {
            depth: 40.0,
            region_indices: vec![],
            mode: ExtrudeMode::Cut,
            target: Some("rod_1".into()),
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sk_2", "bore_3");
    add_thread(
        &mut g,
        "thread_4",
        "rod_1",
        outer,
        false,
        1.5,
        0.2,
        Some(15.0),
        false,
        1,
        true,
    );
    let v = assert_part_sane(&g);
    let tube = std::f64::consts::PI * (100.0 - 36.0) * 30.0;
    assert!(
        (v - tube).abs() / tube < 0.08,
        "threaded tube: {v} vs {tube}"
    );
    let (_, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert!(warnings.is_empty(), "outer wall must thread: {warnings:?}");
    let solids = g.debug_kernel_solids(&HashSet::new()).unwrap();
    let parts = &solids.iter().find(|(id, _)| id == "rod_1").unwrap().1;
    let mut bore_preserved = false;
    let mut threaded_wall = false;
    for face in parts.iter().flat_map(|p| p.faces()) {
        match face.surface() {
            Some(openrcad::geom::GeomSurface::Cylinder(c)) => {
                bore_preserved |= (c.radius() - 6.0).abs() < 1e-6;
            }
            Some(openrcad::geom::GeomSurface::Ruled(r)) => {
                for curve in [&r.curve1, &r.curve2] {
                    if let openrcad::geom::GeomCurve::Helix(h) = curve {
                        for wire in face.wires() {
                            for i in 0..wire.edges().len() {
                                let pc = wire.pcurve(i).expect("thread trim");
                                for t in [0.0, 1.0] {
                                    assert!(
                                        h.radius_at(pc.point_at_fraction(t).x()) > 9.0,
                                        "thread changed the bore wall"
                                    );
                                }
                            }
                        }
                        threaded_wall = true;
                    }
                }
            }
            _ => {}
        }
    }
    assert!(bore_preserved, "the radius-6 bore must remain cylindrical");
    assert!(
        threaded_wall,
        "the outer wall must contain actual thread surfaces"
    );
}

// ---------------------------------------------------------------------------
// Degenerate thread parameters.
// ---------------------------------------------------------------------------

#[test]
fn thread_degenerate_parameters_do_not_panic() {
    let bad: &[(f32, f32, Option<f32>, u32)] = &[
        (0.0, 0.2, None, 1),
        (-1.0, 0.2, None, 1),
        (1.0, 0.0, None, 1),
        (1.0, -0.2, None, 1),
        (1.0, 5.9, None, 1),  // depth near the full radius
        (1.0, 10.0, None, 1), // depth beyond the radius
        (1.0, 0.2, Some(0.0), 1),
        (1.0, 0.2, Some(-5.0), 1),
        (1.0, 0.2, Some(1e9), 1),
        (1e-6, 0.2, None, 1),
        (1e9, 0.2, None, 1),
        (1.0, 0.2, None, 0),
        (1.0, 0.2, None, 1000),
    ];
    for &(pitch, depth, length, starts) in bad {
        let mut g = ParametricGraph::new();
        add_cylinder(&mut g, "rod_1", 6.0, 30.0);
        let face = cyl_face_near(&g, "rod_1", [0.0, 15.0, 0.0]);
        add_thread(
            &mut g, "thread_2", "rod_1", face, false, pitch, depth, length, false, starts, true,
        );
        let result = g.evaluate_bodies_with_warnings(&HashSet::new());
        if let Ok((bodies, _)) = result {
            assert_meshes_finite(&bodies);
        }
    }
}

// A5 regression (fixed 2026-09-11): non-finite Thread parameters (NaN or
// ±Infinity pitch or depth) used to panic inside the thread band
// construction — `mock_kernel/primitives.rs` `called Option::unwrap() on a
// None value` (a NaN seam angle reaching a `partial_cmp().unwrap()` sort).
// The Thread apply path now validates pitch, depth, angle, derived lead, and
// the optional length before any construction: the feature resolves
// Unresolved with a PARAMETER_INVALID diagnostic and the rod survives
// untouched.
#[test]
fn thread_with_nonfinite_parameters_resolves_to_parameter_diagnostic() {
    for (pitch, depth) in [
        (f32::NAN, 0.2),
        (1.0, f32::NAN),
        (f32::INFINITY, 0.2),
        (1.0, f32::NEG_INFINITY),
    ] {
        let mut g = ParametricGraph::new();
        add_cylinder(&mut g, "rod_1", 6.0, 30.0);
        let face = cyl_face_near(&g, "rod_1", [0.0, 15.0, 0.0]);
        add_thread(
            &mut g, "thread_2", "rod_1", face, false, pitch, depth, None, false, 1, true,
        );
        let (bodies, warnings) = g
            .evaluate_bodies_with_warnings(&HashSet::new())
            .expect("non-finite thread parameters must resolve to a warning, not an error");
        assert_meshes_finite(&bodies);
        assert!(
            warnings
                .iter()
                .any(|m| m.contains("must be finite numbers")),
            "(pitch={pitch}, depth={depth}) must produce the parameter warning, got {warnings:?}"
        );
        let rod = std::f64::consts::PI * 36.0 * 30.0;
        let v = total_volume(&bodies);
        assert!(
            (v - rod).abs() / rod < 0.06,
            "(pitch={pitch}, depth={depth}): the unthreaded rod must survive, got {v} vs {rod}"
        );
    }
}

#[test]
fn thread_on_ghost_target_and_far_face_is_graceful() {
    // Ghost target body.
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "rod_1", 6.0, 30.0);
    let face = cyl_face_near(&g, "rod_1", [0.0, 15.0, 0.0]);
    add_feature(
        &mut g,
        "thread_2",
        FeatureType::Thread {
            target: "ghost".into(),
            face: face.clone(),
            internal: false,
            pitch: 1.0,
            depth: 0.15,
            angle_deg: 60.0,
            right_handed: true,
            starts: 1,
            length: None,
            flip: false,
            designation: "M12".into(),
            standard: None,
        },
        &["rod_1"],
    );
    let _ = g.evaluate_bodies_with_warnings(&HashSet::new());

    // Far-away face on a real body: cosmetic fallback must warn and preserve.
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "rod_1", 6.0, 30.0);
    add_feature(
        &mut g,
        "thread_2",
        FeatureType::Thread {
            target: "rod_1".into(),
            face: FaceRef {
                centroid: [500.0, 500.0, 500.0],
                normal: [0.0, 1.0, 0.0],
                topology: None,
            },
            internal: false,
            pitch: 1.0,
            depth: 0.15,
            angle_deg: 60.0,
            right_handed: true,
            starts: 1,
            length: None,
            flip: false,
            designation: "M12".into(),
            standard: None,
        },
        &["rod_1"],
    );
    let (bodies, warnings) = eval_pair(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    let rod = std::f64::consts::PI * 36.0 * 30.0;
    assert!(
        (v - rod).abs() / rod < 0.06,
        "far-face thread preserves rod: {v} vs {rod}"
    );
    // E19 regression (fixed 2026-09-11): an unmatched face reference used to
    // be a SILENT no-op — `cylinder_face_near` returned the closest cylinder
    // with no distance gate, threads were cut into an arbitrary wall, and the
    // existing tolerance even hid the material change. The legacy fallback
    // now requires the pick point to sit on exactly one cylindrical wall:
    // a far-away reference warns and preserves the source geometry exactly.
    assert!(
        warnings
            .iter()
            .any(|w| w.contains("thread_2") && w.contains("cosmetic")),
        "an unmatched thread selection must warn loudly: {warnings:?}"
    );
    assert!(
        (v - rod).abs() / rod < 0.005,
        "with the warning present, the tolerance must not hide a material change: {v} vs {rod}"
    );
}
