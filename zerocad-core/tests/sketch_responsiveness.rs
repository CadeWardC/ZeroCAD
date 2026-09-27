use zerocad_core::{
    read_document_from_slice,
    sketch::{resolution::SolverCache, Constraint, EntityId, SolveOutcome},
    Dimension, FeatureType,
};

const FIXTURES: &[&[u8]] = &[
    include_bytes!("fixtures/lag-fix.zcad"),
    include_bytes!("fixtures/more-lag.zcad"),
];

#[test]
fn saved_small_rectangles_converge_and_reuse_the_same_solution() {
    for bytes in FIXTURES {
        let loaded = read_document_from_slice(bytes, &Default::default()).unwrap();
        let graph = loaded.document.evaluator_graph();
        let variables = graph.variable_map();
        for node in graph.graph.node_weights() {
            let FeatureType::Sketch {
                solver: Some(model),
                ..
            } = &node.feature
            else {
                continue;
            };
            let mut cache = SolverCache::default();
            let first = cache.resolve(model, &variables, &|| false).unwrap();
            assert_eq!(
                first.report.outcome,
                SolveOutcome::Converged,
                "{}: {:?}",
                node.id,
                first.report
            );
            let second = cache.resolve(model, &variables, &|| false).unwrap();
            assert!(std::sync::Arc::ptr_eq(&first, &second));
            assert_eq!(cache.solve_count(), 1);
            // Check actual distance errors independently of the solver status.
            for constraint in &model.constraints {
                if let Constraint::Distance { a, b, d, .. } = constraint {
                    let a = first.model.point(*a).unwrap().pos;
                    let b = first.model.point(*b).unwrap().pos;
                    assert!(
                        ((b.0 - a.0).hypot(b.1 - a.1) - f64::from(d.resolve(&variables))).abs()
                            <= 1.0e-6
                    );
                }
            }
            let mut changed = model.clone();
            // A real 0.01 mm contradiction must not disappear into tolerance.
            let (a, b, d) = model
                .constraints
                .iter()
                .find_map(|c| match c {
                    Constraint::Distance { a, b, d, .. } => Some((*a, *b, d.resolve(&variables))),
                    _ => None,
                })
                .unwrap();
            changed.constraints.push(Constraint::Distance {
                id: EntityId(99999),
                a,
                b,
                d: Dimension::literal(d + 0.01),
            });
            let invalid = cache.resolve(&changed, &variables, &|| false).unwrap();
            assert_ne!(invalid.report.outcome, SolveOutcome::Converged);
            assert_eq!(
                invalid.model, changed,
                "failure must preserve the input geometry"
            );
            assert_eq!(cache.solve_count(), 2);
            let invalid_again = cache.resolve(&changed, &variables, &|| false).unwrap();
            assert!(std::sync::Arc::ptr_eq(&invalid, &invalid_again));
            let mut changed_vars = variables.clone();
            changed_vars.insert("outer_width".into(), 40.0);
            cache.resolve(model, &changed_vars, &|| false).unwrap();
            assert_eq!(cache.solve_count(), 3);
        }
    }
}

#[test]
fn cancellation_does_not_publish_or_cache_a_solution() {
    let loaded = read_document_from_slice(FIXTURES[0], &Default::default()).unwrap();
    let model = loaded
        .document
        .graph
        .node_weights()
        .find_map(|n| match &n.feature {
            FeatureType::Sketch {
                solver: Some(m), ..
            } => Some(m),
            _ => None,
        })
        .unwrap();
    let calls = std::cell::Cell::new(0);
    let mut cache = SolverCache::default();
    assert!(cache
        .resolve(model, &loaded.document.variable_map(), &|| {
            calls.set(calls.get() + 1);
            calls.get() > 2
        })
        .is_none());
    assert!(calls.get() > 2);
    assert!(cache
        .resolve(model, &loaded.document.variable_map(), &|| false)
        .is_some());
    assert_eq!(cache.solve_count(), 2);
}

#[test]
fn corrected_solver_diagnostics_survive_evaluation_and_save_reopen() {
    for (index, bytes) in FIXTURES.iter().enumerate() {
        let loaded = read_document_from_slice(bytes, &Default::default()).unwrap();
        let before = zerocad_core::write_document_to_vec(
            &loaded.document,
            &Default::default(),
            &Default::default(),
        )
        .unwrap();
        let graph = loaded.document.evaluator_graph();
        for pass in 0..2 {
            let before_solves = zerocad_core::sketch::resolution::cached_solve_count();
            let (bodies, warnings) = graph
                .evaluate_bodies_with_warnings(&Default::default())
                .unwrap();
            let solves = zerocad_core::sketch::resolution::cached_solve_count() - before_solves;
            assert!(
                solves <= usize::from(pass == 0),
                "unchanged evaluation re-solved the sketch"
            );
            assert!(
                !warnings.iter().any(|w| w.contains("constraints conflict")),
                "{warnings:?}"
            );
            if index == 1 {
                assert_eq!(
                    bodies.len(),
                    1,
                    "the second fixture's extrusion must still produce a body"
                );
                assert!(warnings.is_empty(), "{warnings:?}");
            } else {
                assert!(
                    warnings.iter().any(|w| w.contains("Join")),
                    "independent invalid Join must remain visible"
                );
            }
        }
        let after = zerocad_core::write_document_to_vec(
            &loaded.document,
            &Default::default(),
            &Default::default(),
        )
        .unwrap();
        assert_eq!(
            before, after,
            "evaluation must not rewrite authoritative dimensions"
        );
        let reopened = read_document_from_slice(&after, &Default::default()).unwrap();
        let (_, warnings) = reopened
            .document
            .evaluate_bodies_with_warnings(&Default::default())
            .unwrap();
        assert!(!warnings.iter().any(|w| w.contains("constraints conflict")));
    }
}

#[test]
fn physical_distance_acceptance_is_local_and_rejects_real_small_conflicts() {
    use zerocad_core::sketch::{solve_model, SketchPoint, SketchSolverModel};
    for length in [0.01_f32, 0.8, 1., 1000.] {
        for offset in [0., 1000., 1_000_000.] {
            let mut model = SketchSolverModel {
                points: vec![
                    SketchPoint {
                        id: EntityId(1),
                        pos: (offset, offset),
                    },
                    SketchPoint {
                        id: EntityId(2),
                        pos: (offset + f64::from(length), offset),
                    },
                ],
                constraints: vec![
                    Constraint::Fixed {
                        id: EntityId(3),
                        p: EntityId(1),
                    },
                    Constraint::Distance {
                        id: EntityId(4),
                        a: EntityId(1),
                        b: EntityId(2),
                        d: Dimension::literal(length),
                    },
                ],
                ..Default::default()
            };
            assert_eq!(
                solve_model(&model, &Default::default()).outcome,
                SolveOutcome::Converged
            );
            model.constraints.push(Constraint::Distance {
                id: EntityId(5),
                a: EntityId(1),
                b: EntityId(2),
                d: Dimension::literal(length + 0.0001),
            });
            assert_ne!(
                solve_model(&model, &Default::default()).outcome,
                SolveOutcome::Converged,
                "must reject contradictory lengths at offset {offset}"
            );
        }
    }
}

#[test]
fn non_finite_geometry_cannot_be_reported_as_converged() {
    use zerocad_core::sketch::{solve_model, SketchPoint, SketchSolverModel};
    for value in [f64::NAN, f64::INFINITY] {
        let model = SketchSolverModel {
            points: vec![SketchPoint {
                id: EntityId(1),
                pos: (value, 0.),
            }],
            ..Default::default()
        };
        assert_ne!(
            solve_model(&model, &Default::default()).outcome,
            SolveOutcome::Converged
        );
    }
}
