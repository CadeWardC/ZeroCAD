//! Phase 0 acceptance harness for topological/history **reattachment**.
//!
//! These tests pin the *identity contract* the persistent-naming work must keep:
//! a reference captured on a body must re-resolve to the **same geometric span**
//! after an upstream edit rebuilds that body. They are deliberately cheap — they
//! resolve the captured [`EdgeRef`] directly via [`resolve_edge_ref_by_topology`]
//! rather than running an expensive fillet solve — so the matrix stays fast as it
//! grows. Cases that only pass once boolean name-propagation lands (Phase 3) are
//! marked `#[ignore]` with the reason; each later phase flips the ones it fixes.
//!
//! They also cover the companion Phase 0 deliverable: the per-feature
//! [`FeatureStatus`] channel, which reports *which* feature failed to resolve
//! instead of only a global warning count.

use super::*;
use crate::sketch::{Dimension, SketchShape};
use crate::units::Unit;
use std::collections::HashSet;

fn no_hidden() -> HashSet<String> {
    HashSet::new()
}

/// Variable `w` (default `w0`), a `w × 12` rectangle extruded 8mm as a new body.
/// The right-hand side of the rectangle tracks `w`, so editing `w` moves the
/// captured edge — the perfect probe for "did the reference follow the geometry?"
fn variable_rect_graph(w0: f32) -> ParametricGraph {
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "vars_1".to_string(),
        name: "Vars".to_string(),
        feature: FeatureType::VariableSet {
            variables: vec![Variable {
                name: "w".to_string(),
                value: w0 as f64,
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
                    value: w0,
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
    g
}

fn set_variable(g: &mut ParametricGraph, name: &str, value: f64) {
    for idx in g.graph.node_indices() {
        if let FeatureType::VariableSet { variables } = &mut g.graph[idx].feature {
            for v in variables.iter_mut() {
                if v.name == name {
                    v.value = value;
                }
            }
        }
    }
}

/// Capture the right-side top edge (its stable sketch-sourced topology id) from a
/// freshly evaluated body.
fn capture_right_top_edge(g: &ParametricGraph) -> EdgeRef {
    let bodies = g.evaluate_bodies(&no_hidden()).unwrap();
    let mesh = &bodies[0].1;
    let captured = mesh
        .edge_refs
        .iter()
        .find(|edge| {
            edge.topology
                .as_ref()
                .and_then(|t| t.edge_id.as_deref())
                .is_some_and(|id| id.contains("rectangle-edge:1:role:top"))
        })
        .expect("right-side top sketch edge should carry a stable topology id");
    edge_ref_from_mesh_edge("extrude_3", captured)
}

// ---------------------------------------------------------------------------
// Reattachment identity contract
// ---------------------------------------------------------------------------

#[test]
fn edge_reattaches_to_same_span_after_width_edit() {
    let mut g = variable_rect_graph(20.0);
    let edge = capture_right_top_edge(&g);
    assert!(
        edge.p0[0] > 19.9 && edge.p1[0] > 19.9,
        "captured edge starts on the original width, got {:?}/{:?}",
        edge.p0,
        edge.p1
    );

    // Widen 20 -> 30 and rebuild.
    set_variable(&mut g, "w", 30.0);
    let (live, warnings) = g.build_live(&no_hidden(), false).unwrap();
    assert!(warnings.is_empty(), "clean rebuild, got {warnings:?}");

    let resolved = resolve_edge_ref_by_topology(&live[0], &edge)
        .expect("captured edge must reattach to the widened body by topology id");
    assert!(
        resolved.p0[0] > 29.5 && resolved.p1[0] > 29.5,
        "reattached edge must follow the geometry to the new width, got {:?}/{:?}",
        resolved.p0,
        resolved.p1
    );
}

#[test]
fn edge_reattaches_after_shrink_edit() {
    // The reverse direction: shrinking must also keep the reference on the same side.
    let mut g = variable_rect_graph(20.0);
    let edge = capture_right_top_edge(&g);

    set_variable(&mut g, "w", 14.0);
    let (live, _warnings) = g.build_live(&no_hidden(), false).unwrap();
    let resolved = resolve_edge_ref_by_topology(&live[0], &edge)
        .expect("captured edge must reattach to the narrowed body");
    assert!(
        resolved.p0[0] > 13.5 && resolved.p1[0] > 13.5 && resolved.p0[0] < 14.5,
        "reattached edge should sit at the shrunk width ~14, got {:?}/{:?}",
        resolved.p0,
        resolved.p1
    );
}

#[test]
// Phase-#1 (edge-id unification): the captured edge's sketch id becomes a
// `mesh:group` id after the cut, but its two adjacent faces ("top" and a side)
// survive by name, so the face-owner-pair fallback in `resolve_edge_ref_by_topology`
// reattaches it.
fn edge_survives_added_upstream_cut() {
    let mut g = variable_rect_graph(20.0);
    let edge = capture_right_top_edge(&g);

    // Add a pocket that does NOT touch the captured x≈20 edge (cut spans x 8..12).
    add_sketch(&mut g, "sketch_cut", rect_sketch((8.0, 4.0), (12.0, 8.0)));
    add_extrude(&mut g, "extrude_cut", "sketch_cut", 8.0, ExtrudeMode::Cut);

    let (live, _warnings) = g.build_live(&no_hidden(), false).unwrap();
    let resolved = resolve_edge_ref_by_topology(&live[0], &edge)
        .expect("captured edge must still resolve after an unrelated pocket cut");
    assert!(
        resolved.p0[0] > 19.5 && resolved.p1[0] > 19.5,
        "the untouched right edge must stay put through the cut, got {:?}/{:?}",
        resolved.p0,
        resolved.p1
    );
}

// ---------------------------------------------------------------------------
// Per-feature resolution status (fail-loud, attributable)
// ---------------------------------------------------------------------------

#[test]
fn clean_model_reports_all_features_resolved() {
    let g = variable_rect_graph(20.0);
    let (_bodies, warnings, statuses) = g.evaluate_bodies_with_status(&no_hidden()).unwrap();
    assert!(warnings.is_empty(), "clean model has no warnings, got {warnings:?}");
    assert!(
        statuses.iter().all(|s| s.state == ResolutionState::Resolved),
        "every feature of a clean model must be Resolved, got {statuses:?}"
    );
    assert!(
        statuses.iter().any(|s| s.feature_id == "extrude_3"),
        "the extrude must appear in the status list"
    );
}

#[test]
fn unresolvable_edge_mod_reports_unresolved_status_for_that_feature() {
    // A 30mm fillet on a 10mm box is infeasible and must be rejected. The status
    // channel has to name THAT feature as Unresolved — not just raise a global
    // warning — so the GUI can mark the exact node.
    let g = box_with_edge_mod(30.0, crate::sketch::CornerKind::Fillet);
    let (_bodies, warnings, statuses) = g.evaluate_bodies_with_status(&no_hidden()).unwrap();
    assert!(!warnings.is_empty(), "oversized fillet should warn");

    let em = statuses
        .iter()
        .find(|s| s.feature_id == "edgemod_2")
        .expect("the edge-mod feature must have a status entry");
    assert!(
        em.is_unresolved(),
        "the infeasible fillet must be Unresolved, got {:?}",
        em.state
    );
    assert!(
        em.reason().is_some_and(|r| !r.is_empty()),
        "an unresolved feature must carry a reason"
    );

    // The upstream box itself resolved fine — the failure is attributed, not global.
    let bx = statuses
        .iter()
        .find(|s| s.feature_id == "box_1")
        .expect("the box must have a status entry");
    assert_eq!(bx.state, ResolutionState::Resolved);
}

// ---------------------------------------------------------------------------
// Face naming (Phase 1) — a captured face re-resolves by its stable name
// ---------------------------------------------------------------------------

#[test]
fn face_reattaches_to_same_span_after_width_edit() {
    // Capture the extrude's top cap by its stable face name, widen the sketch, and
    // assert the name re-resolves to the (moved) top cap — face identity by name,
    // not by geometry (the centroid deliberately moves).
    let mut g = variable_rect_graph(20.0);
    let bodies = g.evaluate_bodies(&no_hidden()).unwrap();
    let mesh = &bodies[0].1;
    let captured = mesh
        .face_refs
        .iter()
        .find(|f| {
            f.topology
                .as_ref()
                .and_then(|t| t.face_id.as_deref())
                .is_some_and(|id| id.ends_with(":face:top"))
        })
        .expect("extrude top cap should carry a stable face name");
    let face = FaceRef {
        centroid: captured.centroid,
        normal: captured.normal,
        topology: captured.topology.as_ref().map(|t| TopologyFaceRef {
            body_id: t.body_id.clone().or_else(|| Some("extrude_3".to_string())),
            topology_version: t.topology_version,
            face_id: t.face_id.clone(),
            surface_kind: t.surface_kind.clone(),
        }),
    };
    assert!(
        (9.0..=11.0).contains(&face.centroid[0]),
        "top-cap centroid X ~ w/2 = 10, got {}",
        face.centroid[0]
    );

    set_variable(&mut g, "w", 30.0);
    let (live, _warnings) = g.build_live(&no_hidden(), false).unwrap();
    let resolved = resolve_face_ref_by_topology(&live[0], &face)
        .expect("captured top face must reattach by name after widening");
    assert!(
        (14.0..=16.0).contains(&resolved.centroid[0]),
        "reattached top cap centroid X ~ w/2 = 15, got {}",
        resolved.centroid[0]
    );
    assert!(
        resolved.normal[2].abs() > 0.9,
        "top face normal stays axial (+Z), got {:?}",
        resolved.normal
    );
    assert_eq!(
        resolved.topology.and_then(|t| t.face_id).as_deref(),
        face.topology.and_then(|t| t.face_id).as_deref(),
        "reattached face keeps the same durable name"
    );
}

#[test]
fn face_name_absent_reports_none_rather_than_wrong_face() {
    // A captured face whose name does not exist on the rebuilt body must resolve to
    // None (caller reports it unresolved) — never silently snap to another face.
    let g = variable_rect_graph(20.0);
    let (live, _warnings) = g.build_live(&no_hidden(), false).unwrap();
    let bogus = FaceRef {
        centroid: [10.0, 6.0, 8.0],
        normal: [0.0, 0.0, 1.0],
        topology: Some(TopologyFaceRef {
            body_id: Some("extrude_3".to_string()),
            topology_version: Some(0),
            face_id: Some("sketch:extrude_3:region:0:face:does-not-exist".to_string()),
            surface_kind: None,
        }),
    };
    assert!(
        resolve_face_ref_by_topology(&live[0], &bogus).is_none(),
        "an unknown face name must not silently resolve to a different face"
    );
}

#[test]
fn face_reattaches_through_added_cut() {
    // Phase 3: capture the extrude's top face, then add a through-pocket that holes
    // (but does not remove) the top. The face name must propagate through the cut's
    // boolean so the captured FaceRef still resolves — faces survive booleans.
    let mut g = variable_rect_graph(20.0);
    let bodies = g.evaluate_bodies(&no_hidden()).unwrap();
    let mesh = &bodies[0].1;
    let captured = mesh
        .face_refs
        .iter()
        .find(|f| {
            f.topology
                .as_ref()
                .and_then(|t| t.face_id.as_deref())
                .is_some_and(|id| id.ends_with(":face:top"))
        })
        .expect("extrude top cap should carry a stable face name");
    let face = FaceRef {
        centroid: captured.centroid,
        normal: captured.normal,
        topology: captured.topology.clone().map(|t| TopologyFaceRef {
            body_id: t.body_id.or_else(|| Some("extrude_3".to_string())),
            topology_version: t.topology_version,
            face_id: t.face_id,
            surface_kind: t.surface_kind,
        }),
    };

    // A pocket in the middle (x 8..12, y 4..8) punched through the body: the top
    // face gets a hole but survives on the same plane.
    add_sketch(&mut g, "sketch_cut", rect_sketch((8.0, 4.0), (12.0, 8.0)));
    add_extrude(&mut g, "extrude_cut", "sketch_cut", 8.0, ExtrudeMode::Cut);

    let (live, warnings) = g.build_live(&no_hidden(), false).unwrap();
    assert!(warnings.is_empty(), "cut should apply cleanly, got {warnings:?}");
    let resolved = resolve_face_ref_by_topology(&live[0], &face)
        .expect("captured top face must survive the cut via name propagation");
    assert!(
        resolved.normal[2].abs() > 0.9,
        "reattached top face stays axial (+Z), got {:?}",
        resolved.normal
    );
    assert_eq!(
        resolved.topology.and_then(|t| t.face_id).as_deref(),
        face.topology.and_then(|t| t.face_id).as_deref(),
        "reattached face keeps its durable name through the boolean"
    );
}

#[test]
fn face_survives_a_severing_cut() {
    // A slot cut clean through a bar severs it into two lumps (multi-part body).
    // A captured face must still reattach by name across that split.
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "bar_sketch", rect_sketch((0.0, 0.0), (20.0, 10.0)));
    add_extrude(&mut g, "bar", "bar_sketch", 10.0, ExtrudeMode::NewBody);

    let bodies = g.evaluate_bodies(&no_hidden()).unwrap();
    let bottom = bodies[0]
        .1
        .face_refs
        .iter()
        .find(|f| {
            f.topology
                .as_ref()
                .and_then(|t| t.face_id.as_deref())
                .is_some_and(|id| id.ends_with(":face:bottom"))
        })
        .expect("bar bottom cap should carry a stable face name");
    let face = FaceRef {
        centroid: bottom.centroid,
        normal: bottom.normal,
        topology: bottom.topology.clone().map(|t| TopologyFaceRef {
            body_id: t.body_id.or_else(|| Some("bar".to_string())),
            topology_version: t.topology_version,
            face_id: t.face_id,
            surface_kind: t.surface_kind,
        }),
    };

    // A through-slot (x 9..11, full y and z) severs the bar into [0,9] and [11,20].
    add_sketch(&mut g, "slot_sketch", rect_sketch((9.0, -2.0), (11.0, 12.0)));
    add_extrude(&mut g, "slot", "slot_sketch", 10.0, ExtrudeMode::Cut);

    let (live, _warnings) = g.build_live(&no_hidden(), false).unwrap();
    let body = live.iter().find(|b| b.id == "bar").expect("bar body");
    assert!(
        body.parts.len() >= 2,
        "the slot should sever the bar into ≥2 lumps, got {}",
        body.parts.len()
    );
    let resolved = resolve_face_ref_by_topology(body, &face)
        .expect("the bottom face must reattach by name across the sever");
    assert_eq!(
        resolved.topology.and_then(|t| t.face_id).as_deref(),
        face.topology.and_then(|t| t.face_id).as_deref(),
        "reattached face keeps its name across the multi-part split"
    );
}

#[test]
fn sketch_on_face_plane_follows_the_body() {
    // A base block (top at z=8), a sketch placed on that top face, and a boss
    // extruded from the sketch. When the base grows, the sketch plane — and the
    // boss on it — must follow the top face up, instead of staying frozen at z=8.
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "base_sketch", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    add_extrude(&mut g, "base", "base_sketch", 8.0, ExtrudeMode::NewBody);

    // Capture the base's top face.
    let bodies = g.evaluate_bodies(&no_hidden()).unwrap();
    let top = bodies[0]
        .1
        .face_refs
        .iter()
        .find(|f| {
            f.topology
                .as_ref()
                .and_then(|t| t.face_id.as_deref())
                .is_some_and(|id| id.ends_with(":face:top"))
        })
        .expect("base top cap should carry a stable face name");
    let face = FaceRef {
        centroid: top.centroid,
        normal: top.normal,
        topology: top.topology.clone().map(|t| TopologyFaceRef {
            body_id: t.body_id.or_else(|| Some("base".to_string())),
            topology_version: t.topology_version,
            face_id: t.face_id,
            surface_kind: t.surface_kind,
        }),
    };

    // A sketch on that face + a boss extruded from it.
    add_sketch(&mut g, "on_face_sketch", rect_sketch((2.0, 2.0), (6.0, 6.0)));
    g.sketch_face_refs.insert("on_face_sketch".to_string(), face);
    g.add_dependency("base", "on_face_sketch");
    add_extrude(&mut g, "boss", "on_face_sketch", 3.0, ExtrudeMode::NewBody);

    let boss_base_z = |g: &ParametricGraph| -> f32 {
        let bodies = g.evaluate_bodies(&no_hidden()).unwrap();
        let boss = bodies.iter().find(|(id, _)| id == "boss").expect("boss body");
        boss.1.vertices.chunks(6).map(|v| v[2]).fold(f32::MAX, f32::min)
    };

    assert!(
        (boss_base_z(&g) - 8.0).abs() < 0.2,
        "boss should sit on the top face at z=8, got {}",
        boss_base_z(&g)
    );

    // Grow the base to depth 12 → the top face (and the sketch on it) moves to z=12.
    for idx in g.graph.node_indices() {
        if g.graph[idx].id == "base" {
            if let FeatureType::Extrude { depth, .. } = &mut g.graph[idx].feature {
                *depth = 12.0;
            }
        }
    }
    assert!(
        (boss_base_z(&g) - 12.0).abs() < 0.2,
        "the sketch-on-face (and its boss) must follow the top face to z=12, got {}",
        boss_base_z(&g)
    );
}

#[test]
fn face_reattaches_through_join() {
    // Mirror the known-merging `join_overlapping_stays_one_body` setup: a 10³ block
    // plus an overlapping block joined in. Capture the object's bottom face first;
    // its name must propagate through the union.
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    add_extrude(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);

    let bodies = g.evaluate_bodies(&no_hidden()).unwrap();
    let mesh = &bodies[0].1;
    let captured = mesh
        .face_refs
        .iter()
        .find(|f| {
            f.topology
                .as_ref()
                .and_then(|t| t.face_id.as_deref())
                .is_some_and(|id| id.ends_with(":face:bottom"))
        })
        .expect("extrude bottom cap should carry a stable face name");
    let face = FaceRef {
        centroid: captured.centroid,
        normal: captured.normal,
        topology: captured.topology.clone().map(|t| TopologyFaceRef {
            body_id: t.body_id.or_else(|| Some("extrude_2".to_string())),
            topology_version: t.topology_version,
            face_id: t.face_id,
            surface_kind: t.surface_kind,
        }),
    };

    // Overlapping block joined in (shifted so faces aren't coplanar).
    add_sketch(&mut g, "sketch_3", rect_sketch((5.0, 5.0), (15.0, 15.0)));
    add_extrude(&mut g, "extrude_4", "sketch_3", 5.0, ExtrudeMode::Join);

    let (live, warnings) = g.build_live(&no_hidden(), false).unwrap();
    assert!(
        live.iter().filter(|b| !b.parts.is_empty()).count() == 1,
        "the join must merge into one body, got {warnings:?}"
    );
    let body = live
        .iter()
        .find(|b| b.id == "extrude_2")
        .expect("the merged body keeps the object's id");
    let resolved = resolve_face_ref_by_topology(body, &face)
        .expect("captured bottom face must survive the join via name propagation");
    assert_eq!(
        resolved.topology.and_then(|t| t.face_id).as_deref(),
        face.topology.and_then(|t| t.face_id).as_deref(),
        "reattached face keeps its name through the join"
    );
}
