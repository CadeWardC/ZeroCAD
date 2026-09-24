//! Catalog B — sketch constraints and solving (adversarial plan §6.B).
//!
//! Drives `SketchSolverModel` + `solve_model` directly (B01–B07). B08 (GUI
//! commit boundaries on Escape/Undo/tool switches) is an interactive-only
//! scenario and lives outside the headless suites.
//!
//! Contracts under test (from the sketch-solver docs):
//! - a consistent system converges; `dof = n_params − rank(J)`;
//! - a stationary-but-unsatisfied system reports `Conflicting` with a
//!   leave-one-out culprit constraint id;
//! - redundancy alone never blocks convergence and never changes dof;
//! - unresolvable references degrade (no equation) rather than mis-solve.

mod common;

use common::*;
use std::collections::HashMap;
use zerocad_core::sketch::{
    solve_model, Constraint, Dimension, EntityId, SketchEntity, SketchPoint, SketchSolverModel,
    SolveOutcome,
};

fn pt(id: u32, x: f64, y: f64) -> SketchPoint {
    SketchPoint {
        id: EntityId(id),
        pos: (x, y),
    }
}

fn line(id: u32, p0: u32, p1: u32) -> SketchEntity {
    SketchEntity::Line {
        id: EntityId(id),
        p0: EntityId(p0),
        p1: EntityId(p1),
        derived_from: None,
    }
}

fn vars() -> HashMap<String, f64> {
    HashMap::new()
}

/// A fully constrained rectangle: 4 shared corner points, H/V pairs, fixed
/// origin anchor, width + height distances. dof must be exactly 0.
fn constrained_rect(w: f64, h: f64) -> SketchSolverModel {
    let mut m = SketchSolverModel::default();
    m.points = vec![
        pt(0, 0.0, 0.0),
        pt(1, 5.0, 0.4),
        pt(2, 4.6, h),
        pt(3, 0.3, h),
    ];
    m.entities = vec![line(4, 0, 1), line(5, 1, 2), line(6, 2, 3), line(7, 3, 0)];
    m.constraints = vec![
        Constraint::Fixed {
            id: EntityId(8),
            p: EntityId(0),
        },
        Constraint::Horizontal {
            id: EntityId(9),
            line: EntityId(4),
        },
        Constraint::Horizontal {
            id: EntityId(10),
            line: EntityId(6),
        },
        Constraint::Vertical {
            id: EntityId(11),
            line: EntityId(5),
        },
        Constraint::Vertical {
            id: EntityId(12),
            line: EntityId(7),
        },
        Constraint::Distance {
            id: EntityId(13),
            a: EntityId(0),
            b: EntityId(1),
            d: Dimension::literal(w as f32),
        },
        Constraint::Distance {
            id: EntityId(14),
            a: EntityId(0),
            b: EntityId(3),
            d: Dimension::literal(h as f32),
        },
    ];
    m
}

fn position_of(report: &zerocad_core::sketch::SolveReport, id: u32) -> (f64, f64) {
    report
        .positions
        .iter()
        .find(|(eid, _)| eid.0 == id)
        .map(|(_, p)| *p)
        .unwrap_or_else(|| panic!("point {id} missing from solve report"))
}

/// B01: adding an incompatible width to a fully constrained rectangle must
/// identify the conflict without destroying the last valid sketch.
#[test]
fn b01_incompatible_width_on_fully_constrained_rect_is_attributed() {
    let mut m = constrained_rect(10.0, 6.0);
    let base = solve_model(&m, &vars());
    assert_eq!(base.outcome, SolveOutcome::Converged, "{base:?}");
    assert_eq!(base.dof, 0, "rectangle must be fully constrained");

    // A second, contradictory width on the same pair.
    m.constraints.push(Constraint::Distance {
        id: EntityId(20),
        a: EntityId(0),
        b: EntityId(1),
        d: Dimension::literal(12.0),
    });
    let conflicted = solve_model(&m, &vars());
    assert_eq!(
        conflicted.outcome,
        SolveOutcome::Conflicting,
        "{conflicted:?}"
    );
    assert!(
        conflicted.conflicting.is_some(),
        "leave-one-out must attribute a culprit constraint"
    );

    // "Without destroying the last valid sketch": applying a conflicting
    // report must not move geometry; the re-solve of the repaired model must
    // reproduce the original solution.
    let mut repaired = m.clone();
    repaired.constraints.retain(|c| c.id().0 != 20);
    let again = solve_model(&repaired, &vars());
    assert_eq!(again.outcome, SolveOutcome::Converged);
    for id in 0u32..4 {
        assert_eq!(position_of(&again, id), position_of(&base, id));
    }
}

/// B01 end-to-end (characterization): a sketch feature whose solver model
/// carries literal-bound conflicting constraints rebuilds from its stored
/// last-valid geometry. Conflicts with literal dimensions are detected in the
/// sketch editor at edit time (the live solve); evaluation only re-solves
/// variable-bound models, because only variable edits can invalidate a
/// converged literal model after the fact. The stored geometry stays exact.
#[test]
fn b01_conflicting_literal_model_uses_last_valid_geometry_deterministically() {
    use zerocad_core::{CoordinateSystem, ExtrudeMode, FeatureType, ParametricGraph};
    // A sketch with a solver model rebuilds from the MODEL's stored positions
    // (the model is the source of truth, `curves` is the legacy fallback), so
    // the fixture carries the solved rectangle plus the appended conflicting
    // constraint — exactly the state the editor persists when the user adds a
    // bad constraint after a good solve.
    let mut solved_model = constrained_rect(10.0, 6.0);
    let ok = solve_model(&solved_model, &vars());
    zerocad_core::sketch::solve::apply_solution(&mut solved_model, &ok);
    let baked = zerocad_core::sketch::constraints::bake_entities_to_curves(&solved_model);
    solved_model.constraints.push(Constraint::Distance {
        id: EntityId(20),
        a: EntityId(0),
        b: EntityId(1),
        d: Dimension::literal(12.0),
    });

    let mut g = ParametricGraph::new();
    add_feature(
        &mut g,
        "sk_1",
        FeatureType::Sketch {
            cs: CoordinateSystem::XY,
            curves: baked,
            shapes: vec![],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
            entity_ids: vec![],
            next_entity_id: 0,
            solver: Some(solved_model),
        },
        &[],
    );
    add_extrude(&mut g, "ex_2", "sk_1", 5.0, ExtrudeMode::NewBody);
    // The last-valid rectangle is used, deterministically, and no partial or
    // collapsed geometry leaks out of the conflicting model.
    assert_close(
        assert_catalog_sane(&g),
        10.0 * 6.0 * 5.0,
        1e-6,
        0.02,
        "last-valid rectangle volume",
    );
}

/// B01 end-to-end (variable-bound): a *variable-bound* conflicting model is
/// re-solved at evaluation time and the conflict is surfaced as a warning —
/// this is the path a variable edit takes after the editor is closed.
#[test]
fn b01_conflicting_variable_bound_model_warns_at_evaluation() {
    use zerocad_core::{CoordinateSystem, ExtrudeMode, FeatureType, ParametricGraph};
    let mut solved_model = constrained_rect(10.0, 6.0);
    let ok = solve_model(&solved_model, &vars());
    zerocad_core::sketch::solve::apply_solution(&mut solved_model, &ok);
    let baked = zerocad_core::sketch::constraints::bake_entities_to_curves(&solved_model);
    // Width bound to w=15; an extra literal width of 12 conflicts with it.
    if let Constraint::Distance { d, .. } = &mut solved_model.constraints[5] {
        d.expr = Some("w".to_string());
    }
    solved_model.constraints.push(Constraint::Distance {
        id: EntityId(20),
        a: EntityId(0),
        b: EntityId(1),
        d: Dimension::literal(12.0),
    });

    let mut g = ParametricGraph::new();
    add_feature(
        &mut g,
        "vars",
        FeatureType::VariableSet {
            variables: vec![zerocad_core::Variable {
                name: "w".to_string(),
                value: 15.0,
                unit: zerocad_core::Unit::Millimeter,
                expression: None,
            }],
        },
        &[],
    );
    add_feature(
        &mut g,
        "sk_1",
        FeatureType::Sketch {
            cs: CoordinateSystem::XY,
            curves: baked,
            shapes: vec![],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
            entity_ids: vec![],
            next_entity_id: 0,
            solver: Some(solved_model),
        },
        &[],
    );
    add_extrude(&mut g, "ex_2", "sk_1", 5.0, ExtrudeMode::NewBody);
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert_close(
        total_volume(&bodies),
        10.0 * 6.0 * 5.0,
        1e-6,
        0.02,
        "conflict keeps last-valid geometry",
    );
    assert!(
        warnings
            .iter()
            .any(|w| w.to_lowercase().contains("conflict") || w.contains("converge")),
        "variable-bound conflict must surface at evaluation: {warnings:?}"
    );
}

/// B02: duplicate constraints — redundancy is distinct from inconsistency,
/// and degrees of freedom stay stable while duplicates are removed one at a
/// time.
#[test]
fn b02_duplicate_constraints_redundant_vs_inconsistent() {
    // An exact duplicate Horizontal is redundant: converges, dof unchanged.
    let mut m = constrained_rect(10.0, 6.0);
    let base = solve_model(&m, &vars());
    m.constraints.push(Constraint::Horizontal {
        id: EntityId(21),
        line: EntityId(4),
    });
    let dup = solve_model(&m, &vars());
    assert_eq!(dup.outcome, SolveOutcome::Converged, "{dup:?}");
    assert_eq!(
        dup.dof, base.dof,
        "a redundant duplicate must not change dof"
    );

    // Removing duplicates one at a time keeps dof stable at 0.
    m.constraints.retain(|c| c.id().0 != 21);
    let after = solve_model(&m, &vars());
    assert_eq!(after.dof, 0);

    // Removing the width dimension frees exactly 1 dof (the rectangle can
    // stretch along x; the height distance is 1 scalar equation).
    m.constraints.retain(|c| c.id().0 != 13);
    let freed = solve_model(&m, &vars());
    assert_eq!(freed.outcome, SolveOutcome::Converged);
    assert_eq!(freed.dof, 1, "removing one width constraint frees 1 dof");
}

/// B03: a nearly collinear triangle dragged through exact collinearity —
/// the solver must stay finite through the degeneracy.
#[test]
fn b03_nearly_collinear_triangle_through_exact_collinearity() {
    for cy in [1e-9_f64, 1e-6, 1e-3, 0.0] {
        let mut m = SketchSolverModel::default();
        m.points = vec![pt(0, 0.0, 0.0), pt(1, 10.0, 0.0), pt(2, 5.0, cy)];
        m.entities = vec![
            line(3, 0, 1),
            line(4, 0, 2),
            line(5, 2, 1),
            line(6, 0, 1), // base reference for PointOnObject
        ];
        m.constraints = vec![
            Constraint::Fixed {
                id: EntityId(7),
                p: EntityId(0),
            },
            Constraint::Distance {
                id: EntityId(8),
                a: EntityId(0),
                b: EntityId(1),
                d: Dimension::literal(10.0),
            },
            Constraint::PointOnObject {
                id: EntityId(9),
                point: EntityId(2),
                object: EntityId(6),
            },
        ];
        let report = solve_model(&m, &vars());
        assert_ne!(
            report.outcome,
            SolveOutcome::Conflicting,
            "cy={cy}: {report:?}"
        );
        for (_, p) in &report.positions {
            assert!(p.0.is_finite() && p.1.is_finite(), "cy={cy} produced {p:?}");
        }
        // Degeneracy handling: a converged solve must land the apex on the
        // line within CAD modeling tolerance (the damped solver may stop by
        // step size with a sub-micron residual); a DidNotConverge exit must
        // still report finite state (never NaN, never a false Conflicting).
        let apex = position_of(&report, 2);
        if report.outcome == SolveOutcome::Converged {
            assert!(
                apex.1.abs() < 1e-3,
                "cy={cy}: apex must land on the line, got {apex:?}"
            );
        } else {
            assert!(apex.0.is_finite() && apex.1.is_finite());
        }
    }
}

/// B04: tiny and large entities in the same sketch — one scale must not
/// hide errors at another; each dimension resolves to its own value.
#[test]
fn b04_mixed_scales_solve_per_entity_accurately() {
    let mut m = SketchSolverModel::default();
    m.points = vec![
        pt(0, 0.0, 0.0),
        pt(1, 0.0, 0.0),
        pt(2, 1.0, 1.0),
        pt(3, 1e4, 1e4),
    ];
    m.entities = vec![
        SketchEntity::Circle {
            id: EntityId(4),
            center: EntityId(0),
            radius: 1e-4,
            derived_from: None,
        },
        SketchEntity::Circle {
            id: EntityId(5),
            center: EntityId(2),
            radius: 1e4,
            derived_from: None,
        },
        line(6, 2, 3),
    ];
    m.constraints = vec![
        Constraint::Fixed {
            id: EntityId(7),
            p: EntityId(0),
        },
        Constraint::Fixed {
            id: EntityId(8),
            p: EntityId(2),
        },
        Constraint::Radius {
            id: EntityId(9),
            circle: EntityId(4),
            r: Dimension::literal(1e-4),
        },
        Constraint::Radius {
            id: EntityId(10),
            circle: EntityId(5),
            r: Dimension::literal(1e4_f32),
        },
        Constraint::Distance {
            id: EntityId(11),
            a: EntityId(2),
            b: EntityId(3),
            d: Dimension::literal(0.0),
        },
    ];
    // A zero distance collapses point 3 onto 2; simpler: give it length 1e4.
    m.constraints.pop();
    m.constraints.push(Constraint::Distance {
        id: EntityId(11),
        a: EntityId(2),
        b: EntityId(3),
        d: Dimension::literal(1e4_f32),
    });
    let report = solve_model(&m, &vars());
    assert_eq!(report.outcome, SolveOutcome::Converged, "{report:?}");
    let radius_of = |id: u32| {
        report
            .radii
            .iter()
            .find(|(eid, _)| eid.0 == id)
            .map(|(_, r)| *r)
            .unwrap_or_else(|| panic!("radius {id} missing"))
    };
    // Solver tolerances are absolute (RESIDUAL_TOL = 1e-9), so a micro radius
    // may deviate by up to that scale; the macro radius by the same absolute
    // amount. Separate tolerances per quantity (plan §3).
    assert!((radius_of(4) - 1e-4).abs() < 1e-8, "micro radius drifted");
    assert!(close(radius_of(5), 1e4, 1e-8, 1e-9), "macro radius drifted");
    assert!(report.residual < 1e-6, "residual {}", report.residual);
}

/// B05: a constrained chain solved from mirrored starting positions reaches
/// the mirrored solution, and re-solving at a solution never flips branches
/// spontaneously. The foot of the constrained segment is pinned so the only
/// remaining freedom is the ±branch.
#[test]
fn b05_mirrored_start_reaches_mirrored_solution_without_flips() {
    let build = |p1: (f64, f64)| -> SketchSolverModel {
        // C is pinned at (2, 0) on the fixed horizontal line 0→3; B is 6 mm
        // from C, perpendicular to the line → B = (2, ±6): two mirrored
        // branches, no continuous freedom.
        let mut m = SketchSolverModel::default();
        m.points = vec![
            pt(0, 0.0, 0.0),
            pt(1, 0.0, 0.0),
            pt(2, 2.0, 0.4),
            pt(3, 8.0, 0.0),
        ];
        m.points[1] = pt(1, p1.0, p1.1);
        m.entities = vec![line(4, 0, 2), line(5, 2, 1), line(6, 0, 3)];
        m.constraints = vec![
            Constraint::Fixed {
                id: EntityId(7),
                p: EntityId(0),
            },
            Constraint::Fixed {
                id: EntityId(8),
                p: EntityId(3),
            },
            Constraint::Horizontal {
                id: EntityId(9),
                line: EntityId(6),
            },
            Constraint::PointOnObject {
                id: EntityId(12),
                point: EntityId(2),
                object: EntityId(6),
            },
            Constraint::DistanceX {
                id: EntityId(13),
                a: EntityId(0),
                b: EntityId(2),
                d: Dimension::literal(2.0),
            },
            Constraint::Distance {
                id: EntityId(10),
                a: EntityId(2),
                b: EntityId(1),
                d: Dimension::literal(6.0),
            },
            Constraint::Perpendicular {
                id: EntityId(11),
                a: EntityId(6),
                b: EntityId(5),
            },
        ];
        m
    };
    // Start "above": point 1 above the line 0→3.
    let above = build((2.0, 5.0));
    let r_above = solve_model(&above, &vars());
    assert_eq!(r_above.outcome, SolveOutcome::Converged, "{r_above:?}");
    let b_above = position_of(&r_above, 1);
    assert!(
        b_above.1 > 0.0,
        "starting above must stay above: {b_above:?}"
    );

    // Start "below" (mirrored): reaches the mirrored solution.
    let below = build((2.0, -5.0));
    let r_below = solve_model(&below, &vars());
    assert_eq!(r_below.outcome, SolveOutcome::Converged, "{r_below:?}");
    let b_below = position_of(&r_below, 1);
    assert!(
        b_below.1 < 0.0,
        "starting below must stay below: {b_below:?}"
    );
    assert!(
        (b_above.0 - b_below.0).abs() < 1e-9 && (b_above.1 + b_below.1).abs() < 1e-9,
        "solutions must be exact mirrors: {b_above:?} vs {b_below:?}"
    );

    // Re-solving from the solution (a warm start) must not flip branches.
    let mut warm = above;
    for p in warm.points.iter_mut() {
        if p.id.0 == 1 {
            p.pos = b_above;
        }
    }
    let r_warm = solve_model(&warm, &vars());
    let b_warm = position_of(&r_warm, 1);
    assert!(
        (b_warm.0 - b_above.0).abs() < 1e-9 && (b_warm.1 - b_above.1).abs() < 1e-9,
        "warm re-solve flipped branch: {b_warm:?} vs {b_above:?}"
    );
}

/// B06: dense equal-length + horizontal + coincident networks converge with
/// useful diagnostics as size increases.
#[test]
fn b06_dense_ladder_network_converges() {
    const N: usize = 20; // 21 points, 20 rungs
    let mut m = SketchSolverModel::default();
    for i in 0..=N {
        // Zig-zag initial guess to give the solver work.
        let y = if i % 2 == 0 { 0.3 } else { -0.2 };
        m.points.push(pt(i as u32, i as f64 * 3.1, y));
    }
    for i in 0..N {
        m.entities
            .push(line((N + 1 + i) as u32, i as u32, (i + 1) as u32));
    }
    let mut c = vec![Constraint::Fixed {
        id: EntityId(100),
        p: EntityId(0),
    }];
    for (i, e) in m.entities.iter().enumerate() {
        let id = EntityId(101 + i as u32);
        c.push(Constraint::Horizontal { id, line: e.id() });
    }
    c.push(Constraint::Distance {
        id: EntityId(200),
        a: EntityId(0),
        b: EntityId(N as u32),
        d: Dimension::literal(100.0),
    });
    for i in 0..N.saturating_sub(1) {
        c.push(Constraint::Equal {
            id: EntityId(201 + i as u32),
            a: m.entities[i].id(),
            b: m.entities[i + 1].id(),
        });
    }
    m.constraints = c;
    let report = solve_model(&m, &vars());
    assert_eq!(
        report.outcome,
        SolveOutcome::Converged,
        "dense ladder must converge: residual {}",
        report.residual
    );
    // All rungs horizontal and equal: spacing 100/20 = 5, all y equal.
    let y0 = position_of(&report, 0).1;
    for i in 0..=N {
        let p = position_of(&report, i as u32);
        assert!((p.1 - y0).abs() < 1e-6, "rung {i} drooped: {p:?}");
        assert!(
            (p.0 - i as f64 * 5.0).abs() < 1e-4,
            "rung {i} spacing drifted: {p:?}"
        );
    }
}

/// B07: the same constraint set in different insertion orders and fresh
/// solves produces the same solution semantics.
#[test]
fn b07_insertion_order_and_repeated_solves_agree() {
    let base = constrained_rect(10.0, 6.0);
    let forward = solve_model(&base, &vars());

    // Reverse the constraint order.
    let mut rev = base.clone();
    rev.constraints.reverse();
    let reversed = solve_model(&rev, &vars());

    // Rotate by 2 (neither sorted nor reversed).
    let mut rot = base.clone();
    rot.constraints.rotate_left(2);
    let rotated = solve_model(&rot, &vars());

    for id in 0u32..4 {
        let f = position_of(&forward, id);
        let r = position_of(&reversed, id);
        let o = position_of(&rotated, id);
        assert!(
            (f.0 - r.0).abs() < 1e-9 && (f.1 - r.1).abs() < 1e-9,
            "reversed order diverged at {id}"
        );
        assert!(
            (f.0 - o.0).abs() < 1e-9 && (f.1 - o.1).abs() < 1e-9,
            "rotated order diverged at {id}"
        );
    }
    assert_eq!(forward.dof, reversed.dof);
    assert_eq!(forward.outcome, reversed.outcome);

    // Repeated solves in the same process are bit-stable.
    let again = solve_model(&base, &vars());
    assert_eq!(forward.residual.to_bits(), again.residual.to_bits());
}
