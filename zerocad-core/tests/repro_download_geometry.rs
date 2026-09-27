use openrcad::foundation::Pnt;
use zerocad_core::mock_kernel::KernelSolid;
use zerocad_core::*;

const CHAMFER: &[u8] = include_bytes!("fixtures/chamfer-notwork.zcad");
const CUT: &[u8] = include_bytes!("fixtures/cut-notwork.zcad");

fn document(bytes: &[u8]) -> ParametricGraph {
    read_document_from_slice(bytes, &Default::default())
        .unwrap()
        .document
        .into_evaluator_graph()
        .clone_document()
}

fn reopen(graph: ParametricGraph) -> ParametricGraph {
    let bytes = write_document_to_vec(
        &Document::from_graph(graph, Unit::Millimeter),
        &Default::default(),
        &Default::default(),
    )
    .unwrap();
    document(&bytes)
}

fn checked_solid(graph: &ParametricGraph) -> (KernelSolid, f64) {
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&Default::default())
        .unwrap();
    assert!(warnings.is_empty(), "{warnings:?}");
    assert_eq!(bodies.len(), 1);

    let mut solids = graph.evaluated_kernel_bodies(&Default::default()).unwrap();
    assert_eq!(solids.len(), 1);
    assert_eq!(solids[0].1.len(), 1);
    let solid = solids.remove(0).1.remove(0);
    assert!(solid.is_watertight());
    assert!(solid.health_report().is_healthy());
    solid
        .validate_strict_with_policy(&Default::default())
        .unwrap();
    assert_eq!(solid.split_disconnected().len(), 1);
    let mesh = openrcad::mesh::tessellate_checked(&solid, 0.002, 0.1).unwrap();
    let volume = openrcad::mesh::mass_properties(&mesh).unwrap().volume;
    (solid, volume)
}

fn material(solid: &KernelSolid, radius: f64, z: f64) -> bool {
    openrcad::algo::boolean::point_in_solid(&Pnt::new(radius, 12.5, z), solid)
}

#[test]
fn downloaded_counterbore_enlarges_existing_hole_and_reopens() {
    let graph = document(CUT);
    let mut before = graph.clone_document();
    before.set_feature_suppressed("extrude_22", true);
    let (_, initial_volume) = checked_solid(&before);
    let expected =
        initial_volume - std::f64::consts::PI * (3.925_f64.powi(2) - 2.2_f64.powi(2)) * 3.2;
    for graph in [graph.clone_document(), reopen(graph)] {
        for _ in 0..2 {
            let (solid, volume) = checked_solid(&graph);
            assert!((volume - expected).abs() < 0.15, "{volume} != {expected}");
            for z in [-20.0, -21.5, -22.4] {
                assert!(!material(&solid, 3.5, z), "counterbore at {z}");
                assert!(material(&solid, 4.2, z), "outer wall at {z}");
            }
            for z in [-23.0, -26.0, -29.0] {
                assert!(material(&solid, 3.5, z), "shoulder at {z}");
                assert!(!material(&solid, 1.5, z), "original bore at {z}");
            }
        }
    }
}

#[test]
fn downloaded_concave_chamfer_adds_only_the_inner_shoulder_wedge() {
    for distance in [0.5_f32, 1.0] {
        let mut graph = document(CHAMFER);
        for node in graph.graph.node_weights_mut() {
            if let FeatureType::EdgeBlend { dist, .. } = &mut node.feature {
                *dist = distance;
            }
        }
        graph.commit_feature_edit("edgeblend_19").unwrap();
        let mut before = graph.clone_document();
        before.set_feature_suppressed("edgeblend_19", true);
        let (_, initial_volume) = checked_solid(&before);
        let d = distance as f64;
        let expected = initial_volume + std::f64::consts::PI * (3.925 * d * d - d.powi(3) / 3.0);
        for graph in [graph.clone_document(), reopen(graph)] {
            for _ in 0..2 {
                let (solid, volume) = checked_solid(&graph);
                assert!((volume - expected).abs() < 0.15, "{volume} != {expected}");
                assert!(material(&solid, 3.925 - d * 0.2, -22.65 + d * 0.2));
                assert!(!material(&solid, 3.925 - d * 0.8, -22.65 + d * 0.8));
                assert!(!material(&solid, 1.5, -23.0));
                assert!(material(&solid, 4.2, -23.0));
            }
        }
    }
}

#[test]
fn downloaded_oversized_shoulder_chamfer_preserves_original_material() {
    let graph = document(CHAMFER);
    let mut before = graph.clone_document();
    before.set_feature_suppressed("edgeblend_19", true);
    let (reference, _) = before
        .evaluate_bodies_with_warnings(&Default::default())
        .unwrap();
    let initial_volume = reference[0].1.mass_properties().unwrap().volume;
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&Default::default())
        .unwrap();
    assert!(
        warnings
            .iter()
            .any(|warning| warning.contains("exceeds the available support face")),
        "{warnings:?}"
    );
    assert!((bodies[0].1.mass_properties().unwrap().volume - initial_volume).abs() < 1e-6);
}

#[test]
fn downloaded_cut_crosses_gap_without_damaging_starting_wall() {
    let graph = document(include_bytes!("fixtures/cut-notwork-2.zcad"));
    let mut before = graph.clone_document();
    before.set_feature_suppressed("extrude_23", true);
    let (_, initial) = checked_solid(&before);
    let expected = initial - std::f64::consts::PI * 3.925_f64.powi(2) * 10.0;
    for graph in [graph.clone_document(), reopen(graph)] {
        for _ in 0..2 {
            let (solid, volume) = checked_solid(&graph);
            assert!((volume - expected).abs() < 0.15, "{volume} != {expected}");
            for z in [20.0, 25.0, 29.0] {
                assert!(!material(&solid, 0.0, z));
                assert!(!material(&solid, 3.5, z));
                assert!(material(&solid, 4.2, z));
            }
            assert!(!material(&solid, 3.5, -21.0));
            assert!(material(&solid, 3.5, -26.0));
            assert!(!material(&solid, 1.0, -26.0));
        }
    }
}

fn downloaded_join(indices: Vec<usize>, depth: f32) -> ParametricGraph {
    // The supplied save ends at Sketch_4, before the failed preview was
    // committed. Recreate that preview with an explicit extent and selection.
    let mut graph = document(include_bytes!("fixtures/join-notwork.zcad"));
    graph.add_feature(FeatureNode {
        id: "extrude_29".into(),
        name: "Join".into(),
        feature: FeatureType::Extrude {
            depth,
            region_indices: indices,
            mode: ExtrudeMode::Join,
            target: Some("extrude_11".into()),
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    graph.add_dependency("sketch_26", "extrude_29");
    graph
}

#[test]
fn downloaded_face_join_preserves_sketch_placement_and_reopens() {
    let (_, initial) = checked_solid(&document(include_bytes!("fixtures/join-notwork.zcad")));
    for indices in [vec![0], vec![0, 1]] {
        let whole_rectangle = indices.len() == 2;
        let graph = downloaded_join(indices, 5.0);
        let area = if whole_rectangle {
            30.0 * 58.9
        } else {
            30.0 * 58.9 - 20.0 * 38.9
        };
        for mut graph in [graph.clone_document(), reopen(graph)] {
            for _ in 0..2 {
                let (solid, volume) = checked_solid(&graph);
                let expected = initial + area * 5.0;
                assert!((volume - expected).abs() < 0.15, "{volume} != {expected}");
                let inside =
                    |x, y, z| openrcad::algo::boolean::point_in_solid(&Pnt::new(x, y, z), &solid);
                assert!(
                    inside(9.0, 27.5, 28.5),
                    "saved outline must not shift with mesh centroid"
                );
                assert!(!inside(-21.0, 27.5, 0.0));
                assert!(!inside(11.0, 27.5, 0.0));
                assert_eq!(inside(0.0, 27.5, 0.0), whole_rectangle);
                assert!(!inside(0.0, 12.5, 25.0), "existing drill hole");
                graph.apply_face_reattach();
            }
        }
    }
}

#[test]
fn downloaded_edge_only_join_preserves_original_body() {
    let graph = downloaded_join(vec![1], 5.0);
    let mut before = graph.clone_document();
    before.set_feature_suppressed("extrude_29", true);
    let (reference, _) = before
        .evaluate_bodies_with_warnings(&Default::default())
        .unwrap();
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&Default::default())
        .unwrap();
    assert!(
        warnings
            .iter()
            .any(|w| w.contains("touches the body only along an edge or point")),
        "{warnings:?}"
    );
    assert!(
        (bodies[0].1.mass_properties().unwrap().volume
            - reference[0].1.mass_properties().unwrap().volume)
            .abs()
            < 1e-6
    );
}
#[test]
fn projected_analytic_rims_do_not_create_sliver_cut_regions() {
    for scale in [0.1_f32, 1.0, 100.0] {
        for clockwise in [false, true] {
            let r = 3.925 * scale;
            let mut drawn = SketchCurves::new();
            drawn.add_circle((0.0, 0.0), r);
            let mut boundary = SketchCurves::new();
            boundary.add_rectangle((-10.0 * scale, -10.0 * scale), (10.0 * scale, 10.0 * scale));
            let projected_r = r * (1.0 + 4.0 * f32::EPSILON);
            for i in 0..3 {
                let point = |i: usize| {
                    let angle = i as f32 * std::f32::consts::TAU / 3.0;
                    (projected_r * angle.cos(), projected_r * angle.sin())
                };
                let (a, b) = (point(i), point((i + 1) % 3));
                boundary.arcs.push(zerocad_core::sketch::Arc {
                    center: (0.0, 0.0),
                    radius: projected_r,
                    start: if clockwise { b } else { a },
                    end: if clockwise { a } else { b },
                    clockwise,
                });
            }
            boundary.add_circle((0.0, 0.0), projected_r);
            drawn.extend_face_boundary(&boundary);
            assert_eq!(detect_regions(&drawn).len(), 2);
            assert_eq!(drawn.circles.len(), 1);
            assert!(drawn.arcs.is_empty());
            let mut distinct = SketchCurves::new();
            distinct.add_circle((0.0, 0.0), r + 0.01 * scale);
            drawn.extend_face_boundary(&distinct);
            assert_eq!(drawn.circles.len(), 2, "keep genuine concentric geometry");
            assert_eq!(detect_regions(&drawn).len(), 3);
        }
    }
}
#[test]
fn downloaded_committed_join_combines_adjacent_regions_on_partial_cap() {
    let graph = document(include_bytes!("fixtures/join-notwork-committed.zcad"));
    let mut before = graph.clone_document();
    before.set_feature_suppressed("extrude_27", true);
    let (_, initial) = checked_solid(&before);
    // The selected U-shaped overlap plus the central region form one rectangle
    // bounded by the saved sketch and the supporting face, at the saved depth.
    let expected = initial + (29.45 + 28.894445) * (20.0 + 7.177845) * 5.868814;
    for mut graph in [graph.clone_document(), reopen(graph)] {
        for _ in 0..2 {
            let (solid, volume) = checked_solid(&graph);
            assert!((volume - expected).abs() < 0.15, "{volume} != {expected}");
            let inside =
                |x, y, z| openrcad::algo::boolean::point_in_solid(&Pnt::new(x, y, z), &solid);
            assert!(
                inside(0.0, 28.0, 0.0),
                "bridge joins the selected cap regions"
            );
            assert!(inside(-15.0, 28.0, 0.0));
            assert!(inside(7.0, 30.7, 29.0));
            assert!(!inside(7.5, 28.0, 0.0), "do not expand unselected material");
            assert!(!inside(0.0, 31.0, 0.0));
            assert!(
                !inside(0.0, 24.95, 0.0),
                "no dipped material below the bridge"
            );
            assert!(!inside(0.0, 12.5, 25.0), "preserve opposite-wall drill");
            assert!(!inside(0.0, 12.5, -26.0), "preserve original bore");
            graph.apply_face_reattach();
        }
    }
}
