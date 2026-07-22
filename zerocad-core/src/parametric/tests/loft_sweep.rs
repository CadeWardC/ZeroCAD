//! Loft and Sweep as parametric features.

use super::*;
use crate::geometry::Vec3;

fn volume(mesh: &MockMesh) -> f64 {
    mesh.mass_properties().expect("closed mesh").volume
}

fn shifted_xy(z: f32) -> CoordinateSystem {
    CoordinateSystem::XY.with_origin(Vec3::new(0.0, 0.0, z))
}

fn loft_cancellation() -> EvaluationCancellation {
    EvaluationCancellation::new(1, std::sync::Arc::new(std::sync::atomic::AtomicU64::new(1)))
}

fn holed_rectangle(
    outer_min: (f32, f32),
    outer_max: (f32, f32),
    hole_min: (f32, f32),
    hole_max: (f32, f32),
) -> (SketchCurves, usize) {
    let mut curves = SketchCurves::new();
    curves.add_rectangle(outer_min, outer_max);
    curves.add_rectangle(hole_min, hole_max);
    let region_index = crate::sketch::detect_regions(&curves)
        .iter()
        .position(|region| !region.holes.is_empty())
        .expect("nested rectangles must expose the material region");
    (curves, region_index)
}

// ---- Loft ----------------------------------------------------------------

#[test]
fn loft_two_squares_is_a_frustum() {
    let mut g = ParametricGraph::new();
    add_sketch_cs(
        &mut g,
        "sketch_1",
        shifted_xy(0.0),
        rect_sketch((0.0, 0.0), (4.0, 4.0)),
    );
    add_sketch_cs(
        &mut g,
        "sketch_2",
        shifted_xy(10.0),
        rect_sketch((1.0, 1.0), (3.0, 3.0)),
    );
    g.add_feature(FeatureNode {
        id: "loft_3".to_string(),
        name: "Loft".to_string(),
        feature: FeatureType::Loft {
            sections: vec![("sketch_1".to_string(), 0), ("sketch_2".to_string(), 0)],
            surface_mode: LoftSurfaceMode::Ruled,
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    g.add_dependency("sketch_1", "loft_3");
    g.add_dependency("sketch_2", "loft_3");
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1);
    // Frustum 4×4 → 2×2 over height 10: h/3·(A0 + A1 + √(A0·A1)).
    let exact = 10.0 / 3.0 * (16.0 + 4.0 + 8.0);
    let v = volume(&bodies[0].1);
    assert!(
        (v - exact).abs() / exact < 0.02,
        "loft volume {v} vs {exact}"
    );
}

#[test]
fn loft_preserves_section_holes() {
    let mut graph = ParametricGraph::new();
    let (bottom, bottom_region) =
        holed_rectangle((-5.0, -5.0), (5.0, 5.0), (-2.0, -2.0), (2.0, 2.0));
    let (top, top_region) = holed_rectangle((-5.0, -5.0), (5.0, 5.0), (-2.0, -2.0), (2.0, 2.0));
    add_sketch_cs(&mut graph, "bottom", shifted_xy(0.0), bottom);
    add_sketch_cs(&mut graph, "top", shifted_xy(6.0), top);
    graph.add_feature(FeatureNode {
        id: "holed_loft".into(),
        name: "Holed loft".into(),
        feature: FeatureType::Loft {
            sections: vec![("bottom".into(), bottom_region), ("top".into(), top_region)],
            surface_mode: LoftSurfaceMode::Ruled,
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    graph.add_dependency("bottom", "holed_loft");
    graph.add_dependency("top", "holed_loft");

    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    let actual = volume(&bodies[0].1);
    let expected = (100.0 - 16.0) * 6.0;
    assert!(
        (actual - expected).abs() / expected < 0.02,
        "holed loft volume {actual} vs {expected}"
    );
}

#[test]
fn loft_needs_two_sections() {
    let mut g = ParametricGraph::new();
    add_sketch_cs(
        &mut g,
        "sketch_1",
        shifted_xy(0.0),
        rect_sketch((0.0, 0.0), (4.0, 4.0)),
    );
    g.add_feature(FeatureNode {
        id: "loft_2".to_string(),
        name: "Loft".to_string(),
        feature: FeatureType::Loft {
            sections: vec![("sketch_1".to_string(), 0)],
            surface_mode: LoftSurfaceMode::Ruled,
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    g.add_dependency("sketch_1", "loft_2");
    let latest = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(1));
    let output = g
        .evaluate_request(
            &std::collections::HashSet::new(),
            EvaluationQuality::Final,
            &EvaluationCancellation::new(1, latest),
        )
        .unwrap();
    assert!(output.bodies.is_empty());
    assert!(
        output.diagnostics.iter().any(|diagnostic| {
            diagnostic.feature_id == "loft_2"
                && diagnostic.code.as_str() == DiagnosticCode::FEATURE_UNRESOLVED
        }),
        "diagnostics: {:?}",
        output.diagnostics
    );
}

fn smooth_rectangle_loft(section_count: usize) -> ParametricGraph {
    let mut graph = ParametricGraph::new();
    let mut sections = Vec::new();
    for section in 0..section_count {
        let fraction = section as f32 / (section_count - 1) as f32;
        let id = format!("smooth_section_{section}");
        let half_width = 2.0 + fraction;
        let half_height = 1.5 + fraction * 0.25;
        add_sketch_cs(
            &mut graph,
            &id,
            CoordinateSystem::XY.with_origin(Vec3::new(fraction * 0.35, 0.0, fraction * 8.0)),
            rect_sketch((-half_width, -half_height), (half_width, half_height)),
        );
        sections.push((id, 0));
    }
    graph.add_feature(FeatureNode {
        id: "smooth_loft".into(),
        name: "Smooth Loft".into(),
        feature: FeatureType::Loft {
            sections: sections.clone(),
            surface_mode: LoftSurfaceMode::Smooth,
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    for (section, _) in sections {
        graph.add_dependency(&section, "smooth_loft");
    }
    graph
}

#[test]
fn smooth_loft_two_through_five_sections_is_resolved_and_cache_equivalent() {
    for section_count in 2..=5 {
        let graph = smooth_rectangle_loft(section_count);
        let hidden = std::collections::HashSet::new();
        let cold = graph
            .evaluate_request(&hidden, EvaluationQuality::Final, &loft_cancellation())
            .unwrap();
        let warm = graph
            .evaluate_request(&hidden, EvaluationQuality::Final, &loft_cancellation())
            .unwrap();
        assert!(
            cold.warnings.is_empty(),
            "cold warnings: {:?}",
            cold.warnings
        );
        assert!(
            cold.diagnostics.is_empty(),
            "cold diagnostics: {:?}",
            cold.diagnostics
        );
        assert_eq!(cold.bodies.len(), 1);
        assert_eq!(cold.bodies[0].1.indices, warm.bodies[0].1.indices);
        assert_eq!(cold.bodies[0].1.face_ids, warm.bodies[0].1.face_ids);
        assert!(cold.bodies[0].1.mass_properties().unwrap().volume > 0.0);
        if section_count == 4 {
            let cancelled_generation = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(2));
            assert!(matches!(
                graph.evaluate_request(
                    &hidden,
                    EvaluationQuality::Interactive,
                    &EvaluationCancellation::new(1, cancelled_generation),
                ),
                Err(EvaluationError::Cancelled)
            ));
        }
    }
}

#[test]
fn smooth_loft_section_family_mismatch_is_typed_and_atomic() {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "existing".into(),
        name: "Existing body".into(),
        feature: FeatureType::Box {
            w: 2.0,
            h: 3.0,
            d: 4.0,
        },
    });
    add_sketch_cs(
        &mut graph,
        "rectangle",
        shifted_xy(0.0),
        rect_sketch((-2.0, -1.0), (2.0, 1.0)),
    );
    let mut circle = SketchCurves::new();
    circle.add_circle((0.0, 0.0), 1.5);
    add_sketch_cs(&mut graph, "circle", shifted_xy(5.0), circle);
    graph.add_feature(FeatureNode {
        id: "invalid_smooth_loft".into(),
        name: "Invalid Smooth Loft".into(),
        feature: FeatureType::Loft {
            sections: vec![("rectangle".into(), 0), ("circle".into(), 0)],
            surface_mode: LoftSurfaceMode::Smooth,
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    graph.add_dependency("rectangle", "invalid_smooth_loft");
    graph.add_dependency("circle", "invalid_smooth_loft");

    let output = graph
        .evaluate_request(
            &std::collections::HashSet::new(),
            EvaluationQuality::Final,
            &loft_cancellation(),
        )
        .unwrap();
    assert_eq!(output.bodies.len(), 1, "the existing body must survive");
    assert_eq!(output.bodies[0].0, "existing");
    let diagnostic = output
        .diagnostics
        .iter()
        .find(|diagnostic| diagnostic.feature_id == "invalid_smooth_loft")
        .expect("typed Smooth Loft rejection");
    assert_eq!(diagnostic.code.as_str(), "loft.section_mismatch");
    assert_eq!(
        diagnostic.parameters.get("failed_stage"),
        Some(&DiagnosticParameterValue::Text("analytic_skinning".into()))
    );
    assert!(
        diagnostic
            .fallback
            .as_deref()
            .is_some_and(|fallback| fallback.contains("Ruled")),
        "the conservative Smooth rejection must name the supported fallback: {diagnostic:?}"
    );
}

#[test]
fn smooth_loft_preserves_periodic_circle_sections_exactly() {
    let mut graph = ParametricGraph::new();
    for (id, z, radius) in [("circle_0", 0.0, 2.0), ("circle_1", 6.0, 3.0)] {
        let mut curves = SketchCurves::new();
        curves.add_circle((0.0, 0.0), radius);
        add_sketch_cs(&mut graph, id, shifted_xy(z), curves);
    }
    graph.add_feature(FeatureNode {
        id: "smooth_circle_loft".into(),
        name: "Smooth circular Loft".into(),
        feature: FeatureType::Loft {
            sections: vec![("circle_0".into(), 0), ("circle_1".into(), 0)],
            surface_mode: LoftSurfaceMode::Smooth,
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    graph.add_dependency("circle_0", "smooth_circle_loft");
    graph.add_dependency("circle_1", "smooth_circle_loft");

    let output = graph
        .evaluate_request(
            &std::collections::HashSet::new(),
            EvaluationQuality::Final,
            &loft_cancellation(),
        )
        .unwrap();
    assert!(output.diagnostics.is_empty(), "{:?}", output.diagnostics);
    assert_eq!(output.bodies.len(), 1);
    let actual = output.bodies[0].1.mass_properties().unwrap().volume;
    let expected = std::f64::consts::PI * 6.0 / 3.0 * (4.0 + 6.0 + 9.0);
    assert!(
        (actual - expected).abs() / expected < 0.02,
        "circular Smooth Loft volume {actual} vs {expected}"
    );
}

#[test]
fn smooth_loft_rejects_sampled_ellipse_instead_of_persisting_approximate_topology() {
    let mut graph = ParametricGraph::new();
    for (id, z) in [("ellipse_0", 0.0), ("ellipse_1", 5.0)] {
        let mut curves = SketchCurves::new();
        curves.add_ellipse((0.0, 0.0), (3.0, 0.0), 1.5);
        graph.add_feature(FeatureNode {
            id: id.into(),
            name: id.into(),
            feature: FeatureType::Sketch {
                cs: shifted_xy(z),
                curves: curves.clone(),
                shapes: vec![crate::sketch::SketchShape::Raw { curves }],
                corner_mods: vec![],
                mirrors: vec![],
                on_face: false,
                entity_ids: vec![],
                next_entity_id: 0,
                solver: None,
            },
        });
    }
    graph.add_feature(FeatureNode {
        id: "ellipse_smooth_loft".into(),
        name: "Unsupported approximate ellipse Loft".into(),
        feature: FeatureType::Loft {
            sections: vec![("ellipse_0".into(), 0), ("ellipse_1".into(), 0)],
            surface_mode: LoftSurfaceMode::Smooth,
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    graph.add_dependency("ellipse_0", "ellipse_smooth_loft");
    graph.add_dependency("ellipse_1", "ellipse_smooth_loft");

    let output = graph
        .evaluate_request(
            &std::collections::HashSet::new(),
            EvaluationQuality::Final,
            &loft_cancellation(),
        )
        .unwrap();
    assert!(output.bodies.is_empty());
    let diagnostic = output
        .diagnostics
        .iter()
        .find(|diagnostic| diagnostic.feature_id == "ellipse_smooth_loft")
        .expect("sampled ellipse must reject with a typed diagnostic");
    assert_eq!(diagnostic.code.as_str(), "loft.unsupported_span");
    assert_eq!(
        diagnostic.parameters.get("curve_kind"),
        Some(&DiagnosticParameterValue::Text("raw_sampled_curve".into()))
    );
}

// ---- Sweep ---------------------------------------------------------------

/// A path sketch holding one straight open segment.
fn straight_path_sketch(
    g: &mut ParametricGraph,
    id: &str,
    cs: CoordinateSystem,
    a: (f32, f32),
    b: (f32, f32),
) {
    let mut curves = SketchCurves::new();
    curves.add_line(a, b);
    add_sketch_cs(g, id, cs, curves);
}

fn add_durable_profile(
    graph: &mut ParametricGraph,
    id: &str,
    cs: CoordinateSystem,
    half_extent: f32,
    entity: crate::sketch::EntityId,
) {
    let curves = rect_sketch((-half_extent, -half_extent), (half_extent, half_extent));
    graph.add_feature(FeatureNode {
        id: id.into(),
        name: id.into(),
        feature: FeatureType::Sketch {
            cs,
            curves: curves.clone(),
            shapes: vec![crate::sketch::SketchShape::Raw { curves }],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
            entity_ids: vec![entity],
            next_entity_id: entity.0 + 1,
            solver: None,
        },
    });
}

fn guided_taper_graph(scale: f32, far_origin: bool) -> ParametricGraph {
    let mut graph = ParametricGraph::new();
    let offset = if far_origin { 1_000.0 * scale } else { 0.0 };
    let origin = Vec3::new(offset, offset, offset);
    let entity = crate::sketch::EntityId(42);
    add_durable_profile(
        &mut graph,
        "guided_profile",
        CoordinateSystem::XY.with_origin(origin),
        scale,
        entity,
    );
    straight_path_sketch(
        &mut graph,
        "guided_spine",
        CoordinateSystem::XZ.with_origin(origin),
        (0.0, 0.0),
        (0.0, 10.0 * scale),
    );
    let anchor_radius = std::f32::consts::SQRT_2 * scale;
    straight_path_sketch(
        &mut graph,
        "guided_rail",
        CoordinateSystem::XZ.with_origin(origin),
        (anchor_radius, 0.0),
        (2.0 * anchor_radius, 10.0 * scale),
    );
    graph.add_feature(FeatureNode {
        id: "guided_sweep".into(),
        name: "Guided Sweep".into(),
        feature: FeatureType::Sweep {
            profile_sketch: "guided_profile".into(),
            profile_region: 0,
            path_sketch: "guided_spine".into(),
            guide: Some(SweepGuide {
                sketch: "guided_rail".into(),
                profile_entity: entity,
                profile_parameter: 0.0,
            }),
            mode: ExtrudeMode::NewBody,
            target: None,
            total_twist_deg: 0.0,
            total_twist_expr: None,
        },
    });
    for dependency in ["guided_profile", "guided_spine", "guided_rail"] {
        graph.add_dependency(dependency, "guided_sweep");
    }
    graph
}

#[test]
fn guided_sweep_tracks_anchor_scale_and_is_cache_equivalent_across_property_sweep() {
    for scale in [1.0e-3, 1.0, 1.0e3] {
        for far_origin in [false, true] {
            let graph = guided_taper_graph(scale, far_origin);
            let hidden = std::collections::HashSet::new();
            let cold = graph
                .evaluate_request(&hidden, EvaluationQuality::Final, &loft_cancellation())
                .unwrap();
            let warm = graph
                .evaluate_request(&hidden, EvaluationQuality::Final, &loft_cancellation())
                .unwrap();
            assert!(cold.diagnostics.is_empty(), "{:?}", cold.diagnostics);
            assert!(cold.warnings.is_empty(), "{:?}", cold.warnings);
            assert_eq!(cold.bodies.len(), 1);
            assert_eq!(cold.bodies[0].1.indices, warm.bodies[0].1.indices);
            assert_eq!(cold.bodies[0].1.face_ids, warm.bodies[0].1.face_ids);
            let actual = volume(&cold.bodies[0].1);
            // Square area grows from 4sÂ² to 16sÂ² linearly in section scale.
            let expected = (280.0 / 3.0) * (scale as f64).powi(3);
            assert!(
                (actual - expected).abs() / expected < 0.03,
                "scale={scale} far={far_origin}: {actual} vs {expected}"
            );
        }
    }
    assert_skinning_contract(&guided_taper_graph(1.0, false), "guided_sweep");
}

#[test]
fn guided_sweep_guide_edit_invalidates_its_warm_geometry_checkpoint() {
    let mut graph = guided_taper_graph(1.0, false);
    let hidden = std::collections::HashSet::new();
    let before = graph
        .evaluate_request(&hidden, EvaluationQuality::Final, &loft_cancellation())
        .unwrap();
    let before_volume = volume(&before.bodies[0].1);

    let guide_index = graph.node_map["guided_rail"];
    let FeatureType::Sketch { curves, .. } = &mut graph.graph[guide_index].feature else {
        unreachable!();
    };
    curves.segments[0].b.0 = 1.5 * std::f32::consts::SQRT_2;
    graph
        .refresh_feature_contract_over_warm_cache_for_test("guided_rail")
        .unwrap();
    let cache_free = graph.clone_document();
    let warm = graph
        .evaluate_request(&hidden, EvaluationQuality::Final, &loft_cancellation())
        .unwrap();
    let cold = cache_free
        .evaluate_request(&hidden, EvaluationQuality::Final, &loft_cancellation())
        .unwrap();
    assert!(warm
        .trace
        .evaluated_features
        .iter()
        .any(|feature| feature.as_str() == "guided_sweep"));
    assert_eq!(warm.bodies[0].1.indices, cold.bodies[0].1.indices);
    assert_eq!(warm.bodies[0].1.face_ids, cold.bodies[0].1.face_ids);
    assert_ne!(volume(&warm.bodies[0].1), before_volume);
}

#[test]
fn guided_sweep_transports_outer_and_hole_loops_through_one_frame() {
    let outer = vec![(-2.0, -2.0), (2.0, -2.0), (2.0, 2.0), (-2.0, 2.0)];
    let holes = vec![vec![(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]];
    let path = [Vec3::ZERO, Vec3::new(0.0, 0.0, 10.0)];
    let anchor_radius = 2.0 * std::f32::consts::SQRT_2;
    let guide = [
        Vec3::new(anchor_radius, 0.0, 0.0),
        Vec3::new(2.0 * anchor_radius, 0.0, 10.0),
    ];
    let solid = crate::mock_kernel::swept_solid_with_guide(
        &CoordinateSystem::XY,
        &outer,
        &holes,
        &path,
        0.0,
        false,
        Some(crate::mock_kernel::SweepGuideGeometry {
            points: &guide,
            closed: false,
            profile_anchor: (-2.0, -2.0),
        }),
    )
    .expect("guided holed profile");
    let mesh = MockMesh::from_solid(&solid);
    let actual = volume(&mesh);
    // Material area starts at 12 and ends at 48 under uniform scale 1 â†’ 2.
    let expected = 10.0 / 3.0 * (12.0 + 24.0 + 48.0);
    assert!((actual - expected).abs() / expected < 0.03);
}

#[test]
fn guided_sweep_adaptively_refines_large_but_continuous_frame_rotation() {
    for (scale, aspect) in [
        (1.0e-3, 1.0),
        (1.0, 1.0e-3),
        (1.0, 1.0),
        (1.0, 1.0e3),
        (1.0e3, 1.0),
    ] {
        for far_origin in [false, true] {
            let offset = if far_origin { 1_000.0 * scale } else { 0.0 };
            let origin = Vec3::new(offset, offset, offset);
            let outer = vec![
                (scale, 0.0),
                (0.0, scale * aspect),
                (-scale, 0.0),
                (0.0, -scale * aspect),
            ];
            let path = [origin, Vec3::new(offset, offset, offset + 10.0 * scale)];
            let rotation = 90.0f32.to_radians();
            let guide = [
                Vec3::new(offset + scale, offset, offset),
                Vec3::new(
                    offset + rotation.cos() * scale,
                    offset + rotation.sin() * scale,
                    offset + 10.0 * scale,
                ),
            ];
            let solid = crate::mock_kernel::swept_solid_with_guide(
                &CoordinateSystem::XY.with_origin(origin),
                &outer,
                &[],
                &path,
                0.0,
                false,
                Some(crate::mock_kernel::SweepGuideGeometry {
                    points: &guide,
                    closed: false,
                    profile_anchor: (scale, 0.0),
                }),
            )
            .unwrap_or_else(|error| {
                panic!(
                    "scale={scale} aspect={aspect} far={far_origin}: guided refinement failed: \
                     {error}"
                )
            });

            assert!(
                solid.shell().faces().len() > outer.len() + 2,
                "scale={scale} aspect={aspect} far={far_origin}: refinement must add \
                 intermediate skin bands"
            );
            let mesh = MockMesh::from_solid(&solid);
            assert!(
                mesh.mass_properties().expect("refined closed skin").volume > 0.0,
                "scale={scale} aspect={aspect} far={far_origin}"
            );
        }
    }
}

#[test]
fn guided_sweep_progress_mismatch_is_actionable_typed_atomic_rejection() {
    let mut graph = guided_taper_graph(1.0, false);
    let guide_index = graph.node_map["guided_rail"];
    let FeatureType::Sketch { curves, .. } = &mut graph.graph[guide_index].feature else {
        unreachable!();
    };
    let anchor_radius = std::f32::consts::SQRT_2;
    let mut freehand = SketchCurves::new();
    freehand.add_line((anchor_radius, 0.0), (3.0 * anchor_radius, 3.0));
    freehand.add_line((3.0 * anchor_radius, 3.0), (2.0 * anchor_radius, 10.0));
    *curves = freehand;
    graph.commit_feature_edit("guided_rail").unwrap();

    let output = graph
        .evaluate_request(
            &std::collections::HashSet::new(),
            EvaluationQuality::Final,
            &loft_cancellation(),
        )
        .unwrap();
    assert!(output.bodies.is_empty(), "a rejected Sweep commits no body");
    let diagnostic = output
        .diagnostics
        .iter()
        .find(|diagnostic| diagnostic.feature_id == "guided_sweep")
        .expect("guided progress mismatch diagnostic");
    assert_eq!(diagnostic.code.as_str(), "sweep.guide_progress_mismatch");
    for parameter in [
        "station",
        "normalized_parameter",
        "axial_offset",
        "tolerance",
    ] {
        assert!(
            diagnostic.parameters.contains_key(parameter),
            "missing {parameter}: {diagnostic:?}"
        );
    }
    assert!(diagnostic.message.contains("progress alongside the path"));
    assert!(diagnostic
        .fallback
        .as_deref()
        .is_some_and(|fallback| fallback.contains("progresses alongside the path")));
}

#[test]
fn guided_sweep_rejects_twist_and_missing_anchor_with_typed_atomic_diagnostics() {
    for (missing_anchor, expected_code) in [
        (false, "sweep.twist_with_guide"),
        (true, DiagnosticCode::REFERENCE_MISSING),
    ] {
        let mut graph = guided_taper_graph(1.0, false);
        graph.add_feature(FeatureNode {
            id: "existing_body".into(),
            name: "Existing body".into(),
            feature: FeatureType::Box {
                w: 2.0,
                h: 3.0,
                d: 4.0,
            },
        });
        let sweep = graph
            .graph
            .node_weights_mut()
            .find(|node| node.id == "guided_sweep")
            .expect("guided Sweep");
        let FeatureType::Sweep {
            guide,
            total_twist_deg,
            ..
        } = &mut sweep.feature
        else {
            unreachable!();
        };
        if missing_anchor {
            guide.as_mut().unwrap().profile_entity = crate::sketch::EntityId(9_999);
        } else {
            *total_twist_deg = 15.0;
        }
        graph.commit_feature_edit("guided_sweep").unwrap();

        let output = graph
            .evaluate_request(
                &std::collections::HashSet::new(),
                EvaluationQuality::Final,
                &loft_cancellation(),
            )
            .unwrap();
        assert_eq!(
            output.bodies.len(),
            1,
            "only the pre-existing body survives"
        );
        assert_eq!(output.bodies[0].0, "existing_body");
        assert!(
            output.diagnostics.iter().any(|diagnostic| {
                diagnostic.feature_id == "guided_sweep" && diagnostic.code.as_str() == expected_code
            }),
            "diagnostics: {:?}",
            output.diagnostics
        );
    }
}

#[test]
fn sweep_square_along_straight_path_is_a_prism() {
    let mut g = ParametricGraph::new();
    // Profile: 2×2 square centered, on XY (normal +Z).
    add_sketch_cs(
        &mut g,
        "profile",
        CoordinateSystem::XY,
        rect_sketch((-1.0, -1.0), (1.0, 1.0)),
    );
    // Path: a line along world +Z, drawn on the XZ plane (u=X, v=Z).
    straight_path_sketch(
        &mut g,
        "path",
        CoordinateSystem::XZ,
        (0.0, 0.0),
        (0.0, 10.0),
    );
    g.add_feature(FeatureNode {
        id: "sweep_1".to_string(),
        name: "Sweep".to_string(),
        feature: FeatureType::Sweep {
            profile_sketch: "profile".to_string(),
            profile_region: 0,
            path_sketch: "path".to_string(),
            mode: ExtrudeMode::NewBody,
            target: None,
            total_twist_deg: 0.0,
            total_twist_expr: None,
            guide: None,
        },
    });
    g.add_dependency("profile", "sweep_1");
    g.add_dependency("path", "sweep_1");
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1);
    let v = volume(&bodies[0].1);
    // 2×2 profile × length 10 = 40.
    assert!((v - 40.0).abs() / 40.0 < 0.02, "sweep volume {v} vs 40");
}

#[test]
fn sweep_v2_distributes_signed_total_twist_without_realigning_profile_indices() {
    let mut graph = ParametricGraph::new();
    add_sketch_cs(
        &mut graph,
        "profile",
        CoordinateSystem::XY,
        rect_sketch((-1.0, -0.5), (1.0, 0.5)),
    );
    straight_path_sketch(
        &mut graph,
        "path",
        CoordinateSystem::XZ,
        (0.0, 0.0),
        (0.0, 10.0),
    );
    graph.add_feature(FeatureNode {
        id: "twisted_sweep".into(),
        name: "Twisted sweep".into(),
        feature: FeatureType::Sweep {
            profile_sketch: "profile".into(),
            profile_region: 0,
            path_sketch: "path".into(),
            mode: ExtrudeMode::NewBody,
            target: None,
            total_twist_deg: 90.0,
            total_twist_expr: None,
            guide: None,
        },
    });
    graph.add_dependency("profile", "twisted_sweep");
    graph.add_dependency("path", "twisted_sweep");
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    let mesh = &bodies[0].1;
    let top: Vec<_> = mesh
        .vertices
        .chunks_exact(6)
        .filter(|vertex| (vertex[2] - 10.0).abs() < 1.0e-3)
        .collect();
    let max_x = top.iter().map(|vertex| vertex[0].abs()).fold(0.0, f32::max);
    let max_y = top.iter().map(|vertex| vertex[1].abs()).fold(0.0, f32::max);
    assert!(
        max_x < 0.55 && max_y > 0.95,
        "twisted top extents x={max_x}, y={max_y}"
    );
}

#[test]
fn closed_circle_sweep_closes_seam_without_end_caps() {
    let mut graph = ParametricGraph::new();
    add_sketch_cs(
        &mut graph,
        "profile",
        CoordinateSystem::XY,
        rect_sketch((-0.4, -0.4), (0.4, 0.4)),
    );
    let mut path = SketchCurves::new();
    path.add_circle((0.0, 0.0), 5.0);
    add_sketch_cs(&mut graph, "closed_path", CoordinateSystem::XZ, path);
    graph.add_feature(FeatureNode {
        id: "closed_sweep".into(),
        name: "Closed sweep".into(),
        feature: FeatureType::Sweep {
            profile_sketch: "profile".into(),
            profile_region: 0,
            path_sketch: "closed_path".into(),
            mode: ExtrudeMode::NewBody,
            target: None,
            total_twist_deg: 0.0,
            total_twist_expr: None,
            guide: None,
        },
    });
    graph.add_dependency("profile", "closed_sweep");
    graph.add_dependency("closed_path", "closed_sweep");
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    let actual = volume(&bodies[0].1);
    let expected = 0.8 * 0.8 * std::f64::consts::TAU * 5.0;
    assert!(
        (actual - expected).abs() / expected < 0.08,
        "closed sweep {actual} vs {expected}"
    );
}

#[test]
fn closed_sweep_rejects_non_integral_seam_twist_atomically() {
    let mut graph = ParametricGraph::new();
    add_sketch_cs(
        &mut graph,
        "profile",
        CoordinateSystem::XY,
        rect_sketch((-0.4, -0.4), (0.4, 0.4)),
    );
    let mut path = SketchCurves::new();
    path.add_circle((0.0, 0.0), 5.0);
    add_sketch_cs(&mut graph, "closed_path", CoordinateSystem::XZ, path);
    graph.add_feature(FeatureNode {
        id: "bad_closed_sweep".into(),
        name: "Bad closed sweep".into(),
        feature: FeatureType::Sweep {
            profile_sketch: "profile".into(),
            profile_region: 0,
            path_sketch: "closed_path".into(),
            mode: ExtrudeMode::NewBody,
            target: None,
            total_twist_deg: 45.0,
            total_twist_expr: None,
            guide: None,
        },
    });
    graph.add_dependency("profile", "bad_closed_sweep");
    graph.add_dependency("closed_path", "bad_closed_sweep");
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(bodies.is_empty());
    assert!(warnings
        .iter()
        .any(|warning| warning.contains("whole number of turns")));
}

#[test]
fn sweep_preserves_profile_hole() {
    let mut graph = ParametricGraph::new();
    let (profile, profile_region) =
        holed_rectangle((-2.0, -2.0), (2.0, 2.0), (-1.0, -1.0), (1.0, 1.0));
    add_sketch_cs(&mut graph, "profile", CoordinateSystem::XY, profile);
    straight_path_sketch(
        &mut graph,
        "path",
        CoordinateSystem::XZ,
        (0.0, 0.0),
        (0.0, 10.0),
    );
    graph.add_feature(FeatureNode {
        id: "holed_sweep".into(),
        name: "Holed sweep".into(),
        feature: FeatureType::Sweep {
            profile_sketch: "profile".into(),
            profile_region,
            path_sketch: "path".into(),
            mode: ExtrudeMode::NewBody,
            target: None,
            total_twist_deg: 0.0,
            total_twist_expr: None,
            guide: None,
        },
    });
    graph.add_dependency("profile", "holed_sweep");
    graph.add_dependency("path", "holed_sweep");

    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    let actual = volume(&bodies[0].1);
    let expected = (16.0 - 4.0) * 10.0;
    assert!(
        (actual - expected).abs() / expected < 0.02,
        "holed sweep volume {actual} vs {expected}"
    );
}

#[test]
fn sweep_along_bent_path_is_watertight() {
    let mut g = ParametricGraph::new();
    add_sketch_cs(
        &mut g,
        "profile",
        CoordinateSystem::XY,
        rect_sketch((-0.5, -0.5), (0.5, 0.5)),
    );
    // L-path on the XZ plane: up 8 then across 6.
    let mut curves = SketchCurves::new();
    curves.add_line((0.0, 0.0), (0.0, 8.0));
    curves.add_line((0.0, 8.0), (6.0, 8.0));
    add_sketch_cs(&mut g, "path", CoordinateSystem::XZ, curves);
    g.add_feature(FeatureNode {
        id: "sweep_1".to_string(),
        name: "Sweep".to_string(),
        feature: FeatureType::Sweep {
            profile_sketch: "profile".to_string(),
            profile_region: 0,
            path_sketch: "path".to_string(),
            mode: ExtrudeMode::NewBody,
            target: None,
            total_twist_deg: 0.0,
            total_twist_expr: None,
            guide: None,
        },
    });
    g.add_dependency("profile", "sweep_1");
    g.add_dependency("path", "sweep_1");
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1);
    let mp = bodies[0].1.mass_properties().expect("closed swept mesh");
    // 1×1 profile over a ~14mm path ≈ 14mm³, minus a little at the miter.
    assert!(
        mp.volume > 10.0 && mp.volume < 15.0,
        "swept volume {}",
        mp.volume
    );
}

#[test]
fn sweep_rejects_branching_path() {
    let mut g = ParametricGraph::new();
    add_sketch_cs(
        &mut g,
        "profile",
        CoordinateSystem::XY,
        rect_sketch((-0.5, -0.5), (0.5, 0.5)),
    );
    // A "T" — three segments meeting at a point: not a single open chain.
    let mut curves = SketchCurves::new();
    curves.add_line((0.0, 0.0), (0.0, 5.0));
    curves.add_line((0.0, 5.0), (5.0, 5.0));
    curves.add_line((0.0, 5.0), (-5.0, 5.0));
    add_sketch_cs(&mut g, "path", CoordinateSystem::XZ, curves);
    g.add_feature(FeatureNode {
        id: "sweep_1".to_string(),
        name: "Sweep".to_string(),
        feature: FeatureType::Sweep {
            profile_sketch: "profile".to_string(),
            profile_region: 0,
            path_sketch: "path".to_string(),
            mode: ExtrudeMode::NewBody,
            target: None,
            total_twist_deg: 0.0,
            total_twist_expr: None,
            guide: None,
        },
    });
    g.add_dependency("profile", "sweep_1");
    g.add_dependency("path", "sweep_1");
    let latest = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(1));
    let output = g
        .evaluate_request(
            &std::collections::HashSet::new(),
            EvaluationQuality::Final,
            &EvaluationCancellation::new(1, latest),
        )
        .unwrap();
    assert!(output.bodies.is_empty());
    assert!(
        output.diagnostics.iter().any(|diagnostic| {
            diagnostic.feature_id == "sweep_1"
                && diagnostic.code.as_str() == DiagnosticCode::FEATURE_UNRESOLVED
        }),
        "diagnostics: {:?}",
        output.diagnostics
    );
}

fn assert_skinning_contract(graph: &ParametricGraph, feature_id: &str) {
    let hidden = std::collections::HashSet::new();
    let cold = graph.evaluate_bodies_with_warnings(&hidden).unwrap();
    let warm = graph.evaluate_bodies_with_warnings(&hidden).unwrap();
    assert!(cold.1.is_empty() && warm.1.is_empty());
    assert_eq!(cold.0.len(), warm.0.len());
    for ((cold_id, cold_mesh), (warm_id, warm_mesh)) in cold.0.iter().zip(&warm.0) {
        assert_eq!(cold_id, warm_id);
        assert_eq!(cold_mesh.indices, warm_mesh.indices);
        assert_eq!(cold_mesh.face_ids, warm_mesh.face_ids);
    }
    let mesh = warm
        .0
        .iter()
        .find(|(id, _)| id == feature_id)
        .map(|(_, mesh)| mesh)
        .expect("family output body");
    assert!(!mesh.face_refs.is_empty());
    assert!(mesh.face_refs.iter().all(|face| {
        face.topology
            .as_ref()
            .and_then(|topology| topology.face_id.as_deref())
            .is_some_and(|name| super::super::topo_name::TopoName::parse(name).is_durable())
    }));

    let cancelled_generation = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(2));
    assert!(matches!(
        graph.evaluate_request(
            &hidden,
            EvaluationQuality::Interactive,
            &EvaluationCancellation::new(1, cancelled_generation),
        ),
        Err(EvaluationError::Cancelled)
    ));

    let loaded: ParametricGraph =
        serde_json::from_str(&serde_json::to_string(&graph.clone_document()).unwrap()).unwrap();
    let rebuilt = loaded.evaluate_bodies_with_warnings(&hidden).unwrap();
    assert!(rebuilt.1.is_empty(), "load warnings: {:?}", rebuilt.1);
    assert!(rebuilt.0.iter().any(|(id, _)| id == feature_id));
}

#[test]
fn loft_and_sweep_candidate_contract_gates() {
    let mut loft = ParametricGraph::new();
    add_sketch_cs(
        &mut loft,
        "loft_profile_1",
        shifted_xy(0.0),
        rect_sketch((0.0, 0.0), (4.0, 4.0)),
    );
    add_sketch_cs(
        &mut loft,
        "loft_profile_2",
        shifted_xy(5.0),
        rect_sketch((1.0, 1.0), (3.0, 3.0)),
    );
    loft.add_feature(FeatureNode {
        id: "loft_contract".into(),
        name: "Loft contract".into(),
        feature: FeatureType::Loft {
            sections: vec![("loft_profile_1".into(), 0), ("loft_profile_2".into(), 0)],
            surface_mode: LoftSurfaceMode::Ruled,
            mode: ExtrudeMode::NewBody,
            target: None,
        },
    });
    loft.add_dependency("loft_profile_1", "loft_contract");
    loft.add_dependency("loft_profile_2", "loft_contract");
    assert_skinning_contract(&loft, "loft_contract");

    let mut sweep = ParametricGraph::new();
    add_sketch_cs(
        &mut sweep,
        "sweep_profile",
        CoordinateSystem::XY,
        rect_sketch((-0.5, -0.5), (0.5, 0.5)),
    );
    straight_path_sketch(
        &mut sweep,
        "sweep_path",
        CoordinateSystem::XZ,
        (0.0, 0.0),
        (0.0, 4.0),
    );
    sweep.add_feature(FeatureNode {
        id: "sweep_contract".into(),
        name: "Sweep contract".into(),
        feature: FeatureType::Sweep {
            profile_sketch: "sweep_profile".into(),
            profile_region: 0,
            path_sketch: "sweep_path".into(),
            mode: ExtrudeMode::NewBody,
            target: None,
            total_twist_deg: 0.0,
            total_twist_expr: None,
            guide: None,
        },
    });
    sweep.add_dependency("sweep_profile", "sweep_contract");
    sweep.add_dependency("sweep_path", "sweep_contract");
    assert_skinning_contract(&sweep, "sweep_contract");
}
