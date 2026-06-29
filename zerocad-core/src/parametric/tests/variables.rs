use super::*;

#[test]
fn sketch_dimension_follows_a_variable() {
    use crate::sketch::{Dimension, SketchShape};
    use crate::units::Unit;
    // A variable "w" and a sketch whose only shape is a square with width &
    // height bound to "w". Extruding it and changing "w" must change the
    // detected region's area (and therefore the solid).
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "vars_1".to_string(),
        name: "Vars".to_string(),
        feature: FeatureType::VariableSet {
            variables: vec![Variable {
                name: "w".to_string(),
                value: 10.0,
                unit: Unit::Millimeter,
            }],
        },
    });
    let wdim = || Dimension {
        value: 10.0,
        expr: Some("w".to_string()),
    };
    g.add_feature(FeatureNode {
        id: "sketch_2".to_string(),
        name: "Sketch".to_string(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::XY,
            curves: SketchCurves::new(),
            shapes: vec![SketchShape::Rectangle {
                origin: (0.0, 0.0),
                sx: 1.0,
                sy: 1.0,
                w: wdim(),
                h: wdim(),
                from_center: false,
            }],
            corner_mods: vec![],
            on_face: false,
        },
    });
    g.add_feature(FeatureNode {
        id: "extrude_3".to_string(),
        name: "Extrude".to_string(),
        feature: FeatureType::Extrude {
            depth: 5.0,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
        },
    });
    g.add_dependency("sketch_2", "extrude_3");

    let footprint = |g: &ParametricGraph| -> f32 {
        // Span of the body in X = the square's width.
        let bodies = g
            .evaluate_bodies(&std::collections::HashSet::new())
            .unwrap();
        let xs: Vec<f32> = bodies[0].1.vertices.chunks(6).map(|v| v[0]).collect();
        let (mn, mx) = xs
            .iter()
            .fold((f32::MAX, f32::MIN), |(a, b), &x| (a.min(x), b.max(x)));
        mx - mn
    };

    assert!((footprint(&g) - 10.0).abs() < 0.05, "width should be w=10");

    for idx in g.graph.node_indices() {
        if let FeatureType::VariableSet { variables } = &mut g.graph[idx].feature {
            variables[0].value = 30.0;
        }
    }
    assert!(
        (footprint(&g) - 30.0).abs() < 0.05,
        "changing the variable must resize the sketch (and the solid)"
    );
}

#[test]
fn topology_edge_ref_reattaches_after_sketch_dimension_edit() {
    use crate::sketch::{Dimension, SketchShape};
    use crate::units::Unit;

    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "vars_1".to_string(),
        name: "Vars".to_string(),
        feature: FeatureType::VariableSet {
            variables: vec![Variable {
                name: "w".to_string(),
                value: 20.0,
                unit: Unit::Millimeter,
            }],
        },
    });
    g.add_feature(FeatureNode {
        id: "sketch_2".to_string(),
        name: "Sketch".to_string(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::XY,
            curves: SketchCurves::new(),
            shapes: vec![SketchShape::Rectangle {
                origin: (0.0, 0.0),
                sx: 1.0,
                sy: 1.0,
                w: Dimension {
                    value: 20.0,
                    expr: Some("w".to_string()),
                },
                h: Dimension::literal(12.0),
                from_center: false,
            }],
            corner_mods: vec![],
            on_face: false,
        },
    });
    add_extrude(&mut g, "extrude_3", "sketch_2", 8.0, ExtrudeMode::NewBody);

    let initial = g
        .evaluate_bodies(&std::collections::HashSet::new())
        .unwrap();
    let mesh = &initial[0].1;
    let captured = mesh
        .edge_refs
        .iter()
        .find(|edge| {
            edge.topology
                .as_ref()
                .and_then(|topology| topology.edge_id.as_deref())
                .is_some_and(|id| id.contains("rectangle-edge:1:role:top"))
        })
        .expect("right-side top sketch edge should have a stable topology id");
    assert!(
        captured.p0[0] > 19.9 && captured.p1[0] > 19.9,
        "captured edge starts on the original width"
    );
    let edge = edge_ref_from_mesh_edge("extrude_3", captured);
    assert!(
        edge.topology
            .as_ref()
            .and_then(|topology| topology.edge_id.as_ref())
            .is_some(),
        "captured edge ref must carry additive topology metadata"
    );

    g.add_feature(FeatureNode {
        id: "edgemod_4".to_string(),
        name: "Fillet".to_string(),
        feature: FeatureType::EdgeMod {
            target: "extrude_3".to_string(),
            edge,
            dist: 1.0,
            dist_expr: None,
            scope: EdgeModScope::FullEdge,
            kind: crate::sketch::CornerKind::Fillet,
        },
    });
    g.add_dependency("extrude_3", "edgemod_4");
    let (_, initial_warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(
        initial_warnings.is_empty(),
        "initial topology-backed edge mod should solve, got {initial_warnings:?}"
    );

    for idx in g.graph.node_indices() {
        if let FeatureType::VariableSet { variables } = &mut g.graph[idx].feature {
            variables[0].value = 30.0;
        }
    }
    let (resized, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(
        warnings.is_empty(),
        "edge mod should reattach after equivalent topology edit, got {warnings:?}"
    );
    let max_x = resized[0]
        .1
        .vertices
        .chunks(6)
        .map(|v| v[0])
        .fold(f32::MIN, f32::max);
    assert!(
        max_x > 29.5,
        "resized body should use the edited width after reattach, max_x={max_x}"
    );
}

#[test]
fn extrude_depth_follows_a_variable() {
    use crate::units::Unit;
    // A variable "h", and an extrude whose depth is the expression "h".
    // Changing the variable must change the resulting solid's height.
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "vars_1".to_string(),
        name: "Vars".to_string(),
        feature: FeatureType::VariableSet {
            variables: vec![Variable {
                name: "h".to_string(),
                value: 5.0,
                unit: Unit::Millimeter,
            }],
        },
    });
    add_sketch(&mut g, "sketch_2", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    g.add_feature(FeatureNode {
        id: "extrude_3".to_string(),
        name: "Extrude".to_string(),
        feature: FeatureType::Extrude {
            depth: 5.0,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            depth_expr: Some("h".to_string()),
        },
    });
    g.add_dependency("sketch_2", "extrude_3");

    let top_z = |g: &ParametricGraph| -> f32 {
        let bodies = g
            .evaluate_bodies(&std::collections::HashSet::new())
            .unwrap();
        bodies[0]
            .1
            .vertices
            .chunks(6)
            .map(|v| v[2])
            .fold(f32::MIN, f32::max)
    };

    assert!(
        (top_z(&g) - 5.0).abs() < 0.01,
        "depth should resolve to h=5"
    );

    // Bump the variable to 20 and rebuild — the extrude must grow with it.
    for idx in g.graph.node_indices() {
        if let FeatureType::VariableSet { variables } = &mut g.graph[idx].feature {
            variables[0].value = 20.0;
        }
    }
    assert!(
        (top_z(&g) - 20.0).abs() < 0.01,
        "changing the variable must change the extrude depth"
    );
}
