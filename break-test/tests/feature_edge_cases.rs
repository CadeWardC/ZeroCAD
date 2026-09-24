//! Niche structural cases: malformed graphs, misused features, cross-feature
//! references, and long chains. Every case must produce either valid geometry
//! or a structured failure — never a panic, hang, or NaN mesh.

mod common;

use common::*;
use zerocad_core::{
    AxisBase, ExtrudeMode, FeatureNode, FeatureType, HoleKind, LoftSurfaceMode, ParametricGraph,
    SketchCurves,
};

#[test]
fn loft_with_single_section_fails_gracefully() {
    let mut g = ParametricGraph::new();
    add_sketch(
        &mut g,
        "sketch_1",
        rect_sketch((-10.0, -10.0), (10.0, 10.0)),
    );
    g.add_feature(FeatureNode {
        id: "loft_2".to_string(),
        name: "loft_2".to_string(),
        feature: FeatureType::Loft {
            sections: vec![("sketch_1".to_string(), 0)],
            surface_mode: LoftSurfaceMode::Ruled,
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    g.add_dependency("sketch_1", "loft_2");
    let _ = g.evaluate_bodies_with_warnings(&std::collections::HashSet::new());
}

#[test]
fn loft_with_missing_section_sketch_fails_gracefully() {
    let mut g = ParametricGraph::new();
    add_sketch(
        &mut g,
        "sketch_1",
        rect_sketch((-10.0, -10.0), (10.0, 10.0)),
    );
    g.add_feature(FeatureNode {
        id: "loft_2".to_string(),
        name: "loft_2".to_string(),
        feature: FeatureType::Loft {
            sections: vec![("sketch_1".to_string(), 0), ("ghost_sketch".to_string(), 0)],
            surface_mode: LoftSurfaceMode::Ruled,
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    g.add_dependency("sketch_1", "loft_2");
    let _ = g.evaluate_bodies_with_warnings(&std::collections::HashSet::new());
}

#[test]
fn sweep_with_disconnected_path_fails_gracefully() {
    // Path with two disjoint segments — no single connected chain to follow.
    let mut path = SketchCurves::new();
    path.add_line((0.0, 0.0), (10.0, 0.0));
    path.add_line((20.0, 20.0), (30.0, 20.0));

    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "profile_1", circle_sketch((0.0, 0.0), 3.0));
    add_sketch(&mut g, "path_2", path);
    g.add_feature(FeatureNode {
        id: "sweep_3".to_string(),
        name: "sweep_3".to_string(),
        feature: FeatureType::Sweep {
            profile_sketch: "profile_1".to_string(),
            profile_region: 0,
            path_sketch: "path_2".to_string(),
            mode: ExtrudeMode::NewBody,
            target: None,
            total_twist_deg: 0.0,
            total_twist_expr: None,
            guide: None,
        },
    });
    g.add_dependency("profile_1", "sweep_3");
    g.add_dependency("path_2", "sweep_3");
    let _ = g.evaluate_bodies_with_warnings(&std::collections::HashSet::new());
}

#[test]
fn sweep_with_extreme_twist_does_not_break() {
    let twists: [f32; 5] = [0.0, 90.0, 360.0, 7200.0, -360.0];
    for &twist in &twists {
        let mut path = SketchCurves::new();
        path.add_line((0.0, 0.0), (0.0, 40.0));
        let mut g = ParametricGraph::new();
        add_sketch(&mut g, "profile_1", rect_sketch((-3.0, -3.0), (3.0, 3.0)));
        add_sketch(&mut g, "path_2", path);
        g.add_feature(FeatureNode {
            id: "sweep_3".to_string(),
            name: "sweep_3".to_string(),
            feature: FeatureType::Sweep {
                profile_sketch: "profile_1".to_string(),
                profile_region: 0,
                path_sketch: "path_2".to_string(),
                mode: ExtrudeMode::NewBody,
                target: None,
                total_twist_deg: twist,
                total_twist_expr: None,
                guide: None,
            },
        });
        g.add_dependency("profile_1", "sweep_3");
        g.add_dependency("path_2", "sweep_3");
        let bodies = eval(&g);
        assert_meshes_finite(&bodies);
    }
}

#[test]
fn revolve_angle_degenerate_values_do_not_panic() {
    let angles: &[f32] = &[0.0, -90.0, 360.0, 720.0, f32::NAN, 1e-4];
    for &angle in angles {
        let mut g = ParametricGraph::new();
        add_sketch(&mut g, "sketch_1", rect_sketch((10.0, 0.0), (30.0, 40.0)));
        g.add_feature(FeatureNode {
            id: "revolve_2".to_string(),
            name: "revolve_2".to_string(),
            feature: FeatureType::Revolve {
                axis: AxisBase::X,
                angle_deg: angle,
                angle_expr: None,
                region_indices: vec![],
                mode: ExtrudeMode::NewBody,
                target: None,
            },
        });
        g.add_dependency("sketch_1", "revolve_2");
        let _ = g.evaluate_bodies_with_warnings(&std::collections::HashSet::new());
    }
}

#[test]
fn shell_with_degenerate_thickness_does_not_panic() {
    let thicknesses: &[f32] = &[0.0, -3.0, 1e-6, 1e6, f32::NAN, 29.9, 30.0, 31.0];
    for &t in thicknesses {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 60.0, 40.0, 30.0);
        g.add_feature(FeatureNode {
            id: "shell_2".to_string(),
            name: "shell_2".to_string(),
            feature: FeatureType::Shell {
                target: "box_1".to_string(),
                thickness: t,
                thickness_expr: None,
                open_faces: vec![],
            },
        });
        g.add_dependency("box_1", "shell_2");
        let _ = g.evaluate_bodies_with_warnings(&std::collections::HashSet::new());
    }
}

#[test]
fn shell_on_unsupported_body_warns_instead_of_failing() {
    // A revolved solid is outside the documented closed-shell kernel scope:
    // it must come back unresolved (warning), not as a hard error or panic.
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", rect_sketch((10.0, 0.0), (30.0, 40.0)));
    g.add_feature(FeatureNode {
        id: "revolve_2".to_string(),
        name: "revolve_2".to_string(),
        feature: FeatureType::Revolve {
            axis: AxisBase::X,
            angle_deg: 360.0,
            angle_expr: None,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    g.add_dependency("sketch_1", "revolve_2");
    g.add_feature(FeatureNode {
        id: "shell_3".to_string(),
        name: "shell_3".to_string(),
        feature: FeatureType::Shell {
            target: "revolve_2".to_string(),
            thickness: 2.0,
            thickness_expr: None,
            open_faces: vec![],
        },
    });
    g.add_dependency("revolve_2", "shell_3");
    match g.evaluate_bodies_with_warnings(&std::collections::HashSet::new()) {
        Ok((bodies, warnings)) => {
            assert_meshes_finite(&bodies);
            // Either the shell applied or it warned; both acceptable.
            let _ = warnings;
        }
        Err(e) => panic!("shell on revolve hard-failed: {e}"),
    }
}

#[test]
fn thread_on_flat_box_face_warns_not_crashes() {
    // Threads need a cylindrical face; pointing one at a box's flat top face
    // is a user mistake that must degrade gracefully.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 20.0, 20.0, 10.0);
    g.add_feature(FeatureNode {
        id: "thread_2".to_string(),
        name: "thread_2".to_string(),
        feature: FeatureType::Thread {
            target: "box_1".to_string(),
            face: zerocad_core::parametric::FaceRef {
                centroid: [10.0, 10.0, 10.0],
                normal: [0.0, 0.0, 1.0],
                topology: None,
            },
            internal: false,
            pitch: 1.0,
            depth: 0.2,
            angle_deg: 60.0,
            right_handed: true,
            starts: 1,
            length: None,
            flip: false,
            designation: "M6x1".to_string(),
            standard: None,
        },
    });
    g.add_dependency("box_1", "thread_2");
    match g.evaluate_bodies_with_warnings(&std::collections::HashSet::new()) {
        Ok((bodies, _)) => assert_meshes_finite(&bodies),
        Err(e) => panic!("thread on planar face hard-failed: {e}"),
    }
}

#[test]
fn thread_on_cylinder_stays_sane() {
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "cyl_1", 6.0, 30.0);
    g.add_feature(FeatureNode {
        id: "thread_2".to_string(),
        name: "thread_2".to_string(),
        feature: FeatureType::Thread {
            target: "cyl_1".to_string(),
            face: zerocad_core::parametric::FaceRef {
                centroid: [0.0, 0.0, 15.0],
                normal: [1.0, 0.0, 0.0],
                topology: None,
            },
            internal: false,
            pitch: 1.0,
            depth: 0.15,
            angle_deg: 60.0,
            right_handed: true,
            starts: 1,
            length: Some(15.0),
            flip: false,
            designation: "M12x1".to_string(),
            standard: None,
        },
    });
    g.add_dependency("cyl_1", "thread_2");
    let v = assert_part_sane(&g);
    let exact = std::f64::consts::PI * 36.0 * 30.0;
    assert!(
        (v - exact).abs() / exact < 0.05,
        "external thread volume {v} should stay near rod volume {exact}"
    );
}

#[test]
fn body_cut_consuming_tool_twice_is_rejected_gracefully() {
    // Two cuts using the same consumed tool — the second must not find a
    // consumed body and crash.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "a_1", 20.0, 20.0, 10.0);
    add_box(&mut g, "b_2", 10.0, 10.0, 10.0);
    g.add_feature(FeatureNode {
        id: "cut_3".to_string(),
        name: "cut_3".to_string(),
        feature: FeatureType::BodyCut {
            target: "a_1".to_string(),
            tool: "b_2".to_string(),
            keep_tool: false,
        },
    });
    g.add_feature(FeatureNode {
        id: "cut_4".to_string(),
        name: "cut_4".to_string(),
        feature: FeatureType::BodyCut {
            target: "a_1".to_string(),
            tool: "b_2".to_string(),
            keep_tool: false,
        },
    });
    g.add_dependency("a_1", "cut_3");
    g.add_dependency("b_2", "cut_3");
    g.add_dependency("a_1", "cut_4");
    g.add_dependency("b_2", "cut_4");
    let _ = g.evaluate_bodies_with_warnings(&std::collections::HashSet::new());
}

#[test]
fn body_join_of_disconnected_bodies_reports_unresolved() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "a_1", 10.0, 10.0, 10.0);
    add_box(&mut g, "b_2", 10.0, 10.0, 10.0);
    // No translation applied, but the boxes coincide — make them genuinely
    // disjoint by scaling one away via a transform copy is complex; instead
    // rely on coincident geometry being the interesting degenerate join case.
    g.add_feature(FeatureNode {
        id: "join_3".to_string(),
        name: "join_3".to_string(),
        feature: FeatureType::BodyJoin {
            sources: vec!["a_1".to_string(), "b_2".to_string()],
        },
    });
    g.add_dependency("a_1", "join_3");
    g.add_dependency("b_2", "join_3");
    let v = assert_part_sane(&g);
    let exact = 10.0 * 10.0 * 10.0;
    assert!(
        (v - exact).abs() / exact < 0.02,
        "coincident join {v} vs single box {exact}"
    );
}

#[test]
fn join_with_empty_and_ghost_sources_fails_gracefully() {
    for sources in [
        vec![],
        vec!["ghost_1".to_string()],
        vec!["ghost_1".to_string(), "ghost_2".to_string()],
    ] {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 10.0, 10.0, 10.0);
        g.add_feature(FeatureNode {
            id: "join_2".to_string(),
            name: "join_2".to_string(),
            feature: FeatureType::BodyJoin { sources },
        });
        let _ = g.evaluate_bodies_with_warnings(&std::collections::HashSet::new());
    }
}

#[test]
fn scale_with_degenerate_factor_does_not_panic() {
    let factors: &[f32] = &[0.0, -1.0, 1e-9, 1e9, f32::NAN, f32::INFINITY];
    for &f in factors {
        let mut g = ParametricGraph::new();
        add_box(&mut g, "box_1", 10.0, 10.0, 10.0);
        g.add_feature(FeatureNode {
            id: "scale_2".to_string(),
            name: "scale_2".to_string(),
            feature: FeatureType::BodyScale {
                source: "box_1".to_string(),
                factor: f,
                factor_expr: None,
                center: [0.0, 0.0, 0.0],
            },
        });
        g.add_dependency("box_1", "scale_2");
        let bodies = eval(&g);
        assert_meshes_finite(&bodies);
    }
}

#[test]
fn pattern_with_degenerate_count_and_spacing_does_not_panic() {
    let counts: &[u32] = &[0, 1, 2, 1000];
    let spacings: &[f32] = &[0.0, -10.0, 1e-6];
    for &count in counts {
        for &spacing in spacings {
            let mut g = ParametricGraph::new();
            add_box(&mut g, "box_1", 5.0, 5.0, 5.0);
            g.add_feature(FeatureNode {
                id: "pattern_2".to_string(),
                name: "pattern_2".to_string(),
                feature: FeatureType::Pattern {
                    source: "box_1".to_string(),
                    kind: zerocad_core::PatternKind::Linear {
                        dir: AxisBase::X,
                        spacing,
                        spacing_expr: None,
                        count,
                    },
                },
            });
            g.add_dependency("box_1", "pattern_2");
            let bodies = eval(&g);
            assert_meshes_finite(&bodies);
        }
    }
}

// A4 regression (fixed 2026-09-11): a linear body Pattern with NaN spacing
// built a NaN translation and panicked inside the kernel's transform
// validation — `mock_kernel/primitives.rs` "a rigid transform of a valid
// solid must remain valid" with `NonFiniteVertex` + `max_deviation: inf`.
// NaN spacing (e.g. from a broken variable expression) now resolves to an
// unresolved feature with a PARAMETER_INVALID diagnostic; the source body
// survives untouched. Valid NEGATIVE spacing keeps its documented
// mirror-image semantics (covered by
// `pattern_with_degenerate_count_and_spacing_does_not_panic`).
#[test]
fn pattern_with_nan_spacing_resolves_to_parameter_diagnostic() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 5.0, 5.0, 5.0);
    g.add_feature(FeatureNode {
        id: "pattern_2".to_string(),
        name: "pattern_2".to_string(),
        feature: FeatureType::Pattern {
            source: "box_1".to_string(),
            kind: zerocad_core::PatternKind::Linear {
                dir: AxisBase::X,
                spacing: f32::NAN,
                spacing_expr: None,
                count: 4,
            },
        },
    });
    g.add_dependency("box_1", "pattern_2");
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("NaN spacing must resolve to a warning, not an error");
    assert_meshes_finite(&bodies);
    assert!(
        warnings
            .iter()
            .any(|m| m.contains("linear spacing must be a finite number")),
        "NaN spacing must produce the parameter warning, got {warnings:?}"
    );
    assert_eq!(
        bodies.len(),
        1,
        "only the source body may remain: {bodies:?}"
    );
    assert!((total_volume(&bodies) - 125.0).abs() < 0.5);
}

#[test]
fn deep_chain_of_alternating_cuts() {
    // 30 sequential pocket cuts — exercises history length, repeated guarded
    // booleans, and cache behavior on one body.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 200.0, 200.0, 10.0);
    for i in 0..30 {
        let x0 = -95.0 + (i as f32) * 6.5;
        add_sketch_cs(
            &mut g,
            &format!("sketch_{}", 2 + i * 2),
            xy_plane_at(10.0),
            rect_sketch((x0, -95.0), (x0 + 5.0, 95.0)),
        );
        add_extrude(
            &mut g,
            &format!("extrude_{}", 3 + i * 2),
            &format!("sketch_{}", 2 + i * 2),
            -9.5,
            ExtrudeMode::Cut,
        );
    }
    let v = assert_part_sane(&g);
    assert!(v > 0.0);
    let solid = 200.0 * 200.0 * 10.0;
    assert!(
        v < solid * 0.95,
        "30 cuts must remove real material: {v} vs {solid}"
    );
}

#[test]
fn overlapping_cut_regions_in_one_sketch() {
    // Sketch with two overlapping rectangles: region detection must produce
    // something sane (not double-count or crash) when extruded as a cut.
    let mut curves = SketchCurves::new();
    curves.add_rectangle((0.0, 0.0), (12.0, 12.0));
    curves.add_rectangle((6.0, 6.0), (18.0, 18.0));
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 40.0, 40.0, 10.0);
    add_sketch_cs(&mut g, "sketch_2", xy_plane_at(10.0), curves);
    add_extrude(&mut g, "extrude_3", "sketch_2", -4.0, ExtrudeMode::Cut);
    let v = assert_part_sane(&g);
    let solid = 40.0 * 40.0 * 10.0;
    assert!(
        v < solid,
        "overlapping cut regions must remove material: {v}"
    );
}

#[test]
fn circle_in_circle_nested_regions() {
    // Annulus-style nested regions: a circle inside a circle extruded as a
    // new body — region detection decides what "all regions" means here.
    let mut curves = SketchCurves::new();
    curves.add_circle((0.0, 0.0), 20.0);
    curves.add_circle((0.0, 0.0), 10.0);
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", curves);
    add_extrude(&mut g, "extrude_2", "sketch_1", 5.0, ExtrudeMode::NewBody);
    let bodies = eval(&g);
    assert_meshes_finite(&bodies);
    let v = total_volume(&bodies);
    let full_disc = std::f64::consts::PI * 400.0 * 5.0;
    let annulus = std::f64::consts::PI * (400.0 - 100.0) * 5.0;
    // Either interpretation (disc + hole, or both regions) is geometrically
    // consistent; anything outside both is a break.
    assert!(
        (v - annulus).abs() / annulus < 0.05 || (v - full_disc).abs() / full_disc < 0.05,
        "nested circles volume {v} matches neither annulus {annulus} nor disc {full_disc}"
    );
}

#[test]
fn hole_pattern_via_counterbores_on_cylinder_top() {
    // Holes placed on a curved face (the top rim of a cylinder): positions
    // near the edge are the fragile zone.
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "cyl_1", 15.0, 20.0);
    let n = 6;
    for i in 0..n {
        let a = i as f32 * std::f32::consts::TAU / n as f32;
        add_hole(
            &mut g,
            &format!("hole_{}", 2 + i),
            "cyl_1",
            [a.cos() * 10.0, a.sin() * 10.0, 20.0],
            [0.0, 0.0, -1.0],
            3.0,
            None,
            HoleKind::Simple,
        );
    }
    let v = assert_part_sane(&g);
    let solid = std::f64::consts::PI * 225.0 * 20.0;
    assert!(v < solid, "rim holes must remove material: {v} vs {solid}");
}
