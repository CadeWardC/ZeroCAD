use super::*;

fn evaluate_output(graph: &ParametricGraph) -> EvaluationOutput {
    let latest = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(1));
    graph
        .evaluate_request(
            &std::collections::HashSet::new(),
            EvaluationQuality::Final,
            &EvaluationCancellation::new(1, latest),
        )
        .unwrap()
}

fn set_draft(graph: &mut ParametricGraph, id: &str, angle: f32, expression: Option<&str>) {
    let node = graph.node_map[id];
    let FeatureType::Extrude {
        draft_angle_deg,
        draft_angle_expr,
        ..
    } = &mut graph.graph[node].feature
    else {
        panic!("{id} is not an extrude")
    };
    *draft_angle_deg = angle;
    *draft_angle_expr = expression.map(str::to_string);
}

fn top_width(mesh: &MockMesh) -> f32 {
    let top = mesh
        .vertices
        .chunks_exact(6)
        .map(|vertex| vertex[2])
        .fold(f32::NEG_INFINITY, f32::max);
    let xs: Vec<_> = mesh
        .vertices
        .chunks_exact(6)
        .filter(|vertex| (vertex[2] - top).abs() <= 1.0e-3)
        .map(|vertex| vertex[0])
        .collect();
    xs.iter().copied().fold(f32::NEG_INFINITY, f32::max)
        - xs.iter().copied().fold(f32::INFINITY, f32::min)
}

#[test]
fn signed_expression_drives_drafted_far_section() {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "vars_1".into(),
        name: "Variables".into(),
        feature: FeatureType::VariableSet {
            variables: vec![Variable {
                name: "draft".into(),
                value: 5.0,
                unit: crate::Unit::Millimeter,
                expression: None,
            }],
        },
    });
    add_sketch(
        &mut graph,
        "sketch_2",
        rect_sketch((0.0, 0.0), (10.0, 10.0)),
    );
    add_extrude(
        &mut graph,
        "extrude_3",
        "sketch_2",
        10.0,
        ExtrudeMode::NewBody,
    );
    set_draft(&mut graph, "extrude_3", 0.0, Some("draft"));

    let expanded = graph
        .evaluate_bodies(&std::collections::HashSet::new())
        .unwrap();
    assert!(top_width(&expanded[0].1) < 9.0);

    let variables = graph.node_map["vars_1"];
    let FeatureType::VariableSet { variables } = &mut graph.graph[variables].feature else {
        unreachable!()
    };
    variables[0].value = -5.0;
    let contracted = graph
        .evaluate_bodies(&std::collections::HashSet::new())
        .unwrap();
    assert!(top_width(&contracted[0].1) > 11.0);
}

#[test]
fn draft_rejection_is_atomic_for_every_extrude_mode() {
    for mode in [ExtrudeMode::NewBody, ExtrudeMode::Join, ExtrudeMode::Cut] {
        let mut graph = ParametricGraph::new();
        graph.add_feature(FeatureNode {
            id: "box_1".into(),
            name: "Box".into(),
            feature: FeatureType::Box {
                w: 10.0,
                h: 10.0,
                d: 10.0,
            },
        });
        let baseline = graph
            .evaluate_bodies(&std::collections::HashSet::new())
            .unwrap();
        add_sketch(&mut graph, "sketch_2", rect_sketch((2.0, 2.0), (4.0, 4.0)));
        add_extrude(&mut graph, "extrude_3", "sketch_2", 10.0, mode);
        set_draft(&mut graph, "extrude_3", 45.0, None);

        for cache_state in ["cold", "warm"] {
            let output = evaluate_output(&graph);
            assert_eq!(
                mesh_digest(&output.bodies),
                mesh_digest(&baseline),
                "mode {mode:?} ({cache_state})"
            );
            let diagnostic = output
                .diagnostics
                .iter()
                .find(|diagnostic| diagnostic.feature_id == "extrude_3")
                .unwrap_or_else(|| {
                    panic!(
                        "mode {mode:?} ({cache_state}) diagnostics: {:?}",
                        output.diagnostics
                    )
                });
            assert_eq!(
                diagnostic.code.as_str(),
                DiagnosticCode::PARAMETER_INVALID,
                "mode {mode:?} ({cache_state})"
            );
            assert_eq!(
                diagnostic.parameters.get("draft_angle_deg"),
                Some(&DiagnosticParameterValue::Decimal("45".into()))
            );
            assert!(diagnostic.parameters.contains_key("reason"));
            assert!(output
                .statuses
                .iter()
                .any(|status| { status.feature_id == "extrude_3" && status.is_unresolved() }));
        }
    }
}

fn zero_draft_graph(feature: FeatureType) -> ParametricGraph {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "box_1".into(),
        name: "Box".into(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
    let top = CoordinateSystem::new(
        crate::geometry::Vec3::new(0.0, 0.0, 10.0),
        crate::geometry::Vec3::X,
        crate::geometry::Vec3::Y,
    );
    add_sketch_cs(
        &mut graph,
        "sketch_2",
        top,
        rect_sketch((2.0, 2.0), (4.0, 4.0)),
    );
    graph.add_feature(FeatureNode {
        id: "extrude_3".into(),
        name: "Extrude".into(),
        feature,
    });
    graph.add_dependency("sketch_2", "extrude_3");
    if matches!(
        &graph.graph[graph.node_map["extrude_3"]].feature,
        FeatureType::Extrude {
            target: Some(_),
            ..
        }
    ) {
        graph.add_dependency("box_1", "extrude_3");
    }
    graph
}

type TopologyNames = Vec<Option<String>>;
type TopologyDigest = Vec<(String, TopologyNames, TopologyNames)>;

fn topology_digest(bodies: &[(String, MockMesh)]) -> TopologyDigest {
    bodies
        .iter()
        .map(|(body_id, mesh)| {
            let mut faces: Vec<_> = mesh
                .face_refs
                .iter()
                .map(|face| {
                    face.topology
                        .as_ref()
                        .and_then(|topology| topology.face_id.clone())
                })
                .collect();
            faces.sort();
            let mut edges: Vec<_> = mesh
                .edge_refs
                .iter()
                .map(|edge| {
                    edge.topology
                        .as_ref()
                        .and_then(|topology| topology.edge_id.clone())
                })
                .collect();
            edges.sort();
            (body_id.clone(), faces, edges)
        })
        .collect()
}

#[test]
fn zero_draft_is_semantically_identical_to_v1_extrude_for_every_mode() {
    for mode in [ExtrudeMode::NewBody, ExtrudeMode::Join, ExtrudeMode::Cut] {
        let feature = FeatureType::Extrude {
            target: (mode != ExtrudeMode::NewBody).then(|| "box_1".to_string()),
            depth: if mode == ExtrudeMode::Cut { -5.0 } else { 5.0 },
            region_indices: Vec::new(),
            mode,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        };
        let v1 = crate::feature_dto::decode_with_decoder(
            "part.extrude",
            1,
            crate::document::FeaturePayloadDecoder::NumericFieldsV1,
            crate::feature_dto::encode_for_schema(&feature, 1),
        )
        .unwrap();
        let v2 = crate::feature_dto::decode_with_decoder(
            "part.extrude",
            2,
            crate::document::FeaturePayloadDecoder::NumericFieldsV2,
            crate::feature_dto::encode_for_schema(&feature, 2),
        )
        .unwrap();
        let old_graph = zero_draft_graph(v1);
        let zero_graph = zero_draft_graph(v2);

        for cache_state in ["cold", "warm"] {
            let old = evaluate_output(&old_graph);
            let zero = evaluate_output(&zero_graph);
            assert_eq!(
                mesh_digest(&old.bodies),
                mesh_digest(&zero.bodies),
                "mode {mode:?} ({cache_state})"
            );
            assert_eq!(
                topology_digest(&old.bodies),
                topology_digest(&zero.bodies),
                "topology mode {mode:?} ({cache_state})"
            );
            assert_eq!(old.warnings, zero.warnings);
            assert_eq!(old.statuses, zero.statuses);
            assert_eq!(old.diagnostics, zero.diagnostics);
        }
    }
}

#[test]
fn curved_draft_remains_unresolved_in_v1() {
    let mut curves = SketchCurves::new();
    curves.add_circle((0.0, 0.0), 5.0);
    let mut graph = ParametricGraph::new();
    add_sketch(&mut graph, "sketch_1", curves);
    add_extrude(
        &mut graph,
        "extrude_2",
        "sketch_1",
        5.0,
        ExtrudeMode::NewBody,
    );
    set_draft(&mut graph, "extrude_2", 3.0, None);
    let output = evaluate_output(&graph);
    assert!(output.bodies.is_empty());
    let diagnostic = output
        .diagnostics
        .iter()
        .find(|diagnostic| diagnostic.feature_id == "extrude_2")
        .expect("curved Draft must emit a typed diagnostic");
    assert_eq!(diagnostic.code.as_str(), DiagnosticCode::FEATURE_UNRESOLVED);
    assert_eq!(
        diagnostic.parameters.get("reason"),
        Some(&DiagnosticParameterValue::Text("unsupported_curves".into()))
    );
}

#[test]
fn drafted_wall_names_are_owned_by_durable_source_curve_ids() {
    use crate::sketch::{Dimension, EntityId, SketchShape};

    let shape = SketchShape::Rectangle {
        origin: (0.0, 0.0),
        sx: 1.0,
        sy: 1.0,
        w: Dimension::literal(10.0),
        h: Dimension::literal(8.0),
        from_center: false,
    };
    let curves = shape.build(&std::collections::HashMap::new());
    let regions = crate::sketch::detect_regions(&curves);
    assert_eq!(regions.len(), 1, "rectangle regions: {regions:?}");
    crate::parametric::drafted_region_solid(
        &regions[0].boundary,
        &regions[0].holes,
        5.0,
        &CoordinateSystem::XY,
        3.0,
    )
    .unwrap_or_else(|error| panic!("rectangle boundary {:?}: {error}", regions[0].boundary));
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "sketch_1".into(),
        name: "Sketch".into(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::XY,
            curves,
            shapes: vec![shape],
            corner_mods: Vec::new(),
            mirrors: Vec::new(),
            on_face: false,
            entity_ids: vec![EntityId(7)],
            next_entity_id: 8,
            solver: None,
        },
    });
    add_extrude(
        &mut graph,
        "extrude_2",
        "sketch_1",
        5.0,
        ExtrudeMode::NewBody,
    );
    set_draft(&mut graph, "extrude_2", 3.0, None);

    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(!bodies.is_empty(), "draft warnings: {warnings:?}");
    let wall_names: std::collections::BTreeSet<_> = bodies[0]
        .1
        .face_refs
        .iter()
        .filter_map(|face| face.topology.as_ref()?.face_id.as_deref())
        .filter(|name| name.contains(":face:wall:fragment:"))
        .map(str::to_string)
        .collect();
    assert_eq!(wall_names.len(), 4, "wall names: {wall_names:?}");
    for edge_index in 0..4 {
        assert!(wall_names
            .iter()
            .any(|name| name.contains(&format!("shape:7:rectangle-edge:{edge_index}"))));
    }
}
