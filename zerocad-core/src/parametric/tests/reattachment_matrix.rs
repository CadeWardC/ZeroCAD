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
            entity_ids: vec![],
            next_entity_id: 0,
            solver: None,
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
            mirrors: vec![],
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
    add_sketch(&mut g, "sketch_4", rect_sketch((8.0, 4.0), (12.0, 8.0)));
    add_extrude(&mut g, "extrude_5", "sketch_4", 8.0, ExtrudeMode::Cut);

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
// Entity-id identity (Stage 0) — provenance keys by durable id, not Vec index
// ---------------------------------------------------------------------------

/// Sketch with a circle at `shapes[0]` and a variable-width rectangle at
/// `shapes[1]`, ids assigned at "commit" like the GUI does. Deleting the circle
/// later shifts the rectangle's Vec position — the exact edit positional
/// identity mis-keys.
fn circle_then_rect_graph(w0: f32) -> ParametricGraph {
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "sketch_1".to_string(),
        name: "Sketch".to_string(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::XY,
            curves: SketchCurves::new(),
            shapes: vec![
                SketchShape::Circle {
                    center: (32.0, 6.0),
                    diameter: Dimension::literal(6.0),
                },
                SketchShape::Rectangle {
                    origin: (0.0, 0.0),
                    sx: 1.0,
                    sy: 1.0,
                    w: Dimension::literal(w0),
                    h: Dimension::literal(12.0),
                    from_center: false,
                },
            ],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
            entity_ids: vec![crate::sketch::EntityId(0), crate::sketch::EntityId(1)],
            next_entity_id: 2,
            solver: None,
        },
    });
    add_extrude(&mut g, "extrude_2", "sketch_1", 8.0, ExtrudeMode::NewBody);
    g
}

/// Remove `shapes[pos]` (and its id) from every sketch node — the "user deleted
/// a shape" edit. Ids of the surviving shapes are untouched, never renumbered.
fn delete_sketch_shape(g: &mut ParametricGraph, pos: usize) {
    for idx in g.graph.node_indices() {
        if let FeatureType::Sketch {
            shapes, entity_ids, ..
        } = &mut g.graph[idx].feature
        {
            if pos < shapes.len() {
                shapes.remove(pos);
                if pos < entity_ids.len() {
                    entity_ids.remove(pos);
                }
            }
        }
    }
}

#[test]
fn provenance_fragment_owner_survives_shape_deletion() {
    // The mechanism itself: fragment owners come from the durable id. Deleting
    // the circle at position 0 must NOT re-key the rectangle from shape:1 to
    // shape:0 the way positional identity did.
    use crate::sketch::{
        build_region_provenance, effective_curves, EntityId, RegionProvenanceFragment,
    };
    let vars = HashMap::new();
    let shapes = vec![
        SketchShape::Circle {
            center: (32.0, 6.0),
            diameter: Dimension::literal(6.0),
        },
        SketchShape::Rectangle {
            origin: (0.0, 0.0),
            sx: 1.0,
            sy: 1.0,
            w: Dimension::literal(20.0),
            h: Dimension::literal(12.0),
            from_center: false,
        },
    ];
    let ids = vec![EntityId(0), EntityId(1)];
    let rect_owner = |shapes: &[SketchShape], ids: &[EntityId]| {
        let curves = effective_curves(&SketchCurves::new(), shapes, &[], &vars);
        let regions = crate::sketch::detect_regions(&curves);
        let provenance = build_region_provenance(&curves, shapes, ids, &regions);
        provenance[0]
            .fragments
            .iter()
            .find_map(|f| match f {
                RegionProvenanceFragment::RectangleEdge { shape_id, .. } => Some(*shape_id),
                _ => None,
            })
            .expect("rectangle fragments present")
    };

    assert_eq!(rect_owner(&shapes, &ids), Some(1));

    // Delete the circle: the rectangle is now shapes[0], but its id — and
    // therefore every captured reference's owner string — must stay 1.
    let shapes_after: Vec<SketchShape> = shapes[1..].to_vec();
    let ids_after = vec![EntityId(1)];
    assert_eq!(
        rect_owner(&shapes_after, &ids_after),
        Some(1),
        "fragment owner must be keyed by durable id, not Vec position"
    );
}

#[test]
fn edge_reattaches_after_deleting_another_shape() {
    // End-to-end: capture a rectangle edge, delete the unrelated circle shape
    // (shifting the rectangle's Vec position), rebuild — the reference must
    // land back on the same geometric span.
    let g = circle_then_rect_graph(20.0);
    let bodies = g.evaluate_bodies(&no_hidden()).unwrap();
    let captured = bodies[0]
        .1
        .edge_refs
        .iter()
        .find(|edge| {
            edge.topology
                .as_ref()
                .and_then(|t| t.edge_id.as_deref())
                .is_some_and(|id| id.contains("shape:1:rectangle-edge:1:role:top"))
        })
        .expect("rectangle's right-top edge should carry an id-keyed topology id")
        .clone();
    let edge = edge_ref_from_mesh_edge("extrude_2", &captured);

    let mut g = g;
    delete_sketch_shape(&mut g, 0);
    let (live, _warnings) = g.build_live(&no_hidden(), false).unwrap();
    let resolved = resolve_edge_ref_by_topology(&live[0], &edge)
        .expect("captured edge must reattach after an unrelated shape deletion");
    assert!(
        resolved.p0[0] > 19.5 && resolved.p1[0] > 19.5,
        "reattached edge must stay on the rectangle's right side, got {:?}/{:?}",
        resolved.p0,
        resolved.p1
    );
}

// ---------------------------------------------------------------------------
// Primitive face naming (N1) — boxes and cylinders are named at creation
// ---------------------------------------------------------------------------

fn plain_box_graph(w: f32, h: f32, d: f32) -> ParametricGraph {
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "box_1".to_string(),
        name: "Box".to_string(),
        feature: FeatureType::Box { w, h, d },
    });
    g
}

#[test]
fn primitive_box_faces_and_edges_carry_names() {
    let g = plain_box_graph(10.0, 10.0, 10.0);
    let bodies = g.evaluate_bodies(&no_hidden()).unwrap();
    let mesh = &bodies[0].1;
    for role in ["+x", "-x", "+y", "-y", "+z", "-z"] {
        assert!(
            mesh.face_refs.iter().any(|f| {
                f.topology
                    .as_ref()
                    .and_then(|t| t.face_id.as_deref())
                    .is_some_and(|id| id == format!("box_box_1:face:{role}"))
            }),
            "primitive box must name its {role} face"
        );
    }
    // Edge identity derives from the face-owner pair.
    assert!(
        mesh.edge_refs.iter().any(|e| {
            e.topology
                .as_ref()
                .is_some_and(|t| t.adjacent_face_ids.len() == 2)
        }),
        "primitive box edges must carry their adjacent face-owner pair"
    );
}

#[test]
fn primitive_box_edge_reattaches_after_resize() {
    let mut g = plain_box_graph(10.0, 10.0, 10.0);
    let bodies = g.evaluate_bodies(&no_hidden()).unwrap();
    // Capture the vertical edge shared by the +x and +y faces.
    let captured = bodies[0]
        .1
        .edge_refs
        .iter()
        .find(|e| {
            e.topology.as_ref().is_some_and(|t| {
                t.adjacent_face_ids.iter().any(|f| f.ends_with(":face:+x"))
                    && t.adjacent_face_ids.iter().any(|f| f.ends_with(":face:+y"))
            })
        })
        .expect("box should expose the +x/+y edge by face pair")
        .clone();
    let edge = edge_ref_from_mesh_edge("box_1", &captured);

    // Grow the box 10 -> 16 wide; the +x/+y edge moves with it.
    let idx = g.node_map["box_1"];
    if let FeatureType::Box { w, .. } = &mut g.graph[idx].feature {
        *w = 16.0;
    }
    let (live, _warnings) = g.build_live(&no_hidden(), false).unwrap();
    let resolved = resolve_edge_ref_by_topology(&live[0], &edge)
        .expect("the +x/+y edge must reattach by its face-owner pair");
    assert!(
        resolved.p0[0] > 15.5 && resolved.p1[0] > 15.5,
        "reattached edge must follow the widened box, got {:?}/{:?}",
        resolved.p0,
        resolved.p1
    );
}

#[test]
fn primitive_cylinder_faces_carry_names() {
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "cyl_1".to_string(),
        name: "Cylinder".to_string(),
        feature: FeatureType::Cylinder { r: 5.0, h: 12.0 },
    });
    let bodies = g.evaluate_bodies(&no_hidden()).unwrap();
    let mesh = &bodies[0].1;
    for role in ["lateral", "top", "bottom"] {
        assert!(
            mesh.face_refs.iter().any(|f| {
                f.topology
                    .as_ref()
                    .and_then(|t| t.face_id.as_deref())
                    .is_some_and(|id| id == format!("cyl_cyl_1:face:{role}"))
            }),
            "primitive cylinder must name its {role} face; got {:?}",
            mesh.face_refs
                .iter()
                .filter_map(|f| f.topology.as_ref().and_then(|t| t.face_id.clone()))
                .collect::<Vec<_>>()
        );
    }
}

// ---------------------------------------------------------------------------
// Generated-face naming (N2) — cut walls carry durable kernel-history names
// ---------------------------------------------------------------------------

#[test]
fn cut_walls_carry_generated_names_and_survive_upstream_edit() {
    // A pocket cut through the variable-width plate. The pocket's walls are
    // GENERATED faces — before the kernel history they could only be
    // reconstructed as `mesh:{group}`; now each carries a durable
    // `cut:{node}:tool-face:{i}` name derived from the tool face it came from.
    let mut g = variable_rect_graph(20.0);
    add_sketch(&mut g, "sketch_4", rect_sketch((8.0, 4.0), (12.0, 8.0)));
    add_extrude(&mut g, "extrude_5", "sketch_4", 8.0, ExtrudeMode::Cut);

    let bodies = g.evaluate_bodies(&no_hidden()).unwrap();
    let wall = bodies[0]
        .1
        .face_refs
        .iter()
        .find(|f| {
            f.topology
                .as_ref()
                .and_then(|t| t.face_id.as_deref())
                .is_some_and(|id| id.starts_with("cut:extrude_5:tool-face:"))
        })
        .unwrap_or_else(|| {
            panic!(
                "the pocket must expose generated tool-face names on its walls; got {:?}",
                bodies[0]
                    .1
                    .face_refs
                    .iter()
                    .map(|f| f.topology.as_ref().and_then(|t| t.face_id.clone()))
                    .collect::<Vec<_>>()
            )
        })
        .clone();
    let captured = FaceRef {
        centroid: wall.centroid,
        normal: wall.normal,
        topology: wall.topology.as_ref().map(|t| TopologyFaceRef {
            body_id: t.body_id.clone().or_else(|| Some("extrude_3".to_string())),
            component_id: t.component_id.clone(),
            topology_version: t.topology_version,
            face_id: t.face_id.clone(),
            surface_kind: t.surface_kind.clone(),
            producer_feature_id: t.producer_feature_id.clone(),
            source_entity_id: t.source_entity_id.clone(),
        }),
    };

    // Widen the plate 20 → 30: the pocket stays put, the captured wall must
    // re-resolve BY NAME to the same span (the plate edit must not steal it).
    set_variable(&mut g, "w", 30.0);
    let (live, _warnings) = g.build_live(&no_hidden(), false).unwrap();
    let resolved = resolve_face_ref_by_topology(&live[0], &captured)
        .expect("a generated cut wall must reattach by its durable name");
    assert!(
        (resolved.centroid[0] - captured.centroid[0]).abs() < 0.5
            && (resolved.centroid[1] - captured.centroid[1]).abs() < 0.5,
        "the pocket wall must stay in place through the width edit, got {:?} vs {:?}",
        resolved.centroid,
        captured.centroid
    );
}

// ---------------------------------------------------------------------------
// Per-feature resolution status (fail-loud, attributable)
// ---------------------------------------------------------------------------

#[test]
fn clean_model_reports_all_features_resolved() {
    let g = variable_rect_graph(20.0);
    let (_bodies, warnings, statuses) = g.evaluate_bodies_with_status(&no_hidden()).unwrap();
    assert!(
        warnings.is_empty(),
        "clean model has no warnings, got {warnings:?}"
    );
    assert!(
        statuses
            .iter()
            .all(|s| s.state == ResolutionState::Resolved),
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
// Half-space discriminator (N4) — spec'd ahead, implemented on demand
// ---------------------------------------------------------------------------

#[test]
#[ignore = "half-space discriminator (plan stage N4): one face-owner pair yielding \
            two edges needs a side discriminator; implement when a real model \
            hits the collision"]
fn pair_collision_edges_reattach_by_side_discriminator() {
    // A slab cut across a cylinder produces TWO edges with the same adjacent
    // face-owner pair (slab-plane × cylinder-lateral) — one on each side of the
    // axis. Today the multi-candidate face-pair path falls to geometry; the N4
    // design appends `:side:{+,-}` from the edge-group centroid's sign against
    // a reference plane so each side reattaches by identity even after large
    // edits. This row pins the acceptance criterion.
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "cyl_1".to_string(),
        name: "Cylinder".to_string(),
        feature: FeatureType::Cylinder { r: 6.0, h: 20.0 },
    });
    add_sketch(&mut g, "sketch_2", rect_sketch((-8.0, -8.0), (8.0, 8.0)));
    add_extrude(&mut g, "extrude_3", "sketch_2", 4.0, ExtrudeMode::Cut);

    let bodies = g.evaluate_bodies(&no_hidden()).unwrap();
    let side_edges: Vec<_> = bodies[0]
        .1
        .edge_refs
        .iter()
        .filter(|e| {
            e.topology
                .as_ref()
                .is_some_and(|t| t.edge_id.as_deref().is_some_and(|id| id.contains(":side:")))
        })
        .collect();
    assert!(
        side_edges.len() >= 2,
        "pair-collision edges must carry side discriminators, got {side_edges:?}"
    );
}

// ---------------------------------------------------------------------------
// Named boolean targeting + lump-following (N3)
// ---------------------------------------------------------------------------

fn add_extrude_targeted(
    g: &mut ParametricGraph,
    id: &str,
    sketch_id: &str,
    depth: f32,
    mode: ExtrudeMode,
    target: Option<&str>,
) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Extrude {
            depth,
            region_indices: vec![],
            mode,
            depth_expr: None,
            target: target.map(str::to_string),
        },
    });
    g.add_dependency(sketch_id, id);
}

#[test]
fn targeted_cut_ignores_a_bystander_body() {
    // Two plates side by side; a pocket whose tool AABB overlaps BOTH, but the
    // cut is pinned to plate A. Plate B must come through untouched (legacy
    // untargeted cuts would bite both).
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (20.0, 12.0)));
    add_extrude(&mut g, "extrude_2", "sketch_1", 8.0, ExtrudeMode::NewBody);
    add_sketch(&mut g, "sketch_3", rect_sketch((22.0, 0.0), (42.0, 12.0)));
    add_extrude(&mut g, "extrude_4", "sketch_3", 8.0, ExtrudeMode::NewBody);
    // Pocket spanning x 18..26 — its tool AABB overlaps both plates.
    add_sketch(&mut g, "sketch_5", rect_sketch((18.0, 4.0), (26.0, 8.0)));

    let tri_count = |g: &ParametricGraph, id: &str| -> usize {
        let bodies = g.evaluate_bodies(&no_hidden()).unwrap();
        bodies
            .iter()
            .find(|(bid, _)| bid == id)
            .map(|(_, m)| m.indices.len())
            .unwrap_or(0)
    };
    let b_before = tri_count(&g, "extrude_4");

    add_extrude_targeted(
        &mut g,
        "extrude_6",
        "sketch_5",
        8.0,
        ExtrudeMode::Cut,
        Some("extrude_2"),
    );
    let (_, warnings) = g.evaluate_bodies_with_warnings(&no_hidden()).unwrap();
    assert!(
        warnings.is_empty(),
        "targeted cut applies cleanly, got {warnings:?}"
    );
    assert_eq!(
        tri_count(&g, "extrude_4"),
        b_before,
        "the bystander plate must be untouched by a cut targeted elsewhere"
    );
    assert_ne!(
        tri_count(&g, "extrude_2"),
        0,
        "the targeted plate still exists"
    );
}

#[test]
fn cut_with_missing_target_fails_loud_and_touches_nothing() {
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (20.0, 12.0)));
    add_extrude(&mut g, "extrude_2", "sketch_1", 8.0, ExtrudeMode::NewBody);
    add_sketch(&mut g, "sketch_3", rect_sketch((8.0, 4.0), (12.0, 8.0)));
    add_extrude_targeted(
        &mut g,
        "extrude_4",
        "sketch_3",
        8.0,
        ExtrudeMode::Cut,
        Some("gone_99"),
    );

    let (bodies, _warnings, statuses) = g.evaluate_bodies_with_status(&no_hidden()).unwrap();
    let cut = statuses
        .iter()
        .find(|s| s.feature_id == "extrude_4")
        .expect("cut status present");
    assert!(
        cut.is_unresolved(),
        "a cut whose target body is gone must be Unresolved, got {:?}",
        cut.state
    );
    // And the existing plate was NOT cut as a fallback.
    let plate = bodies.iter().find(|(id, _)| id == "extrude_2").unwrap();
    assert_eq!(
        plate.1.face_refs.len(),
        6,
        "the untargeted plate must be untouched (no pocket walls)"
    );
}

#[test]
fn captured_face_follows_its_own_lump_through_a_sever() {
    // Slot a bar into two lumps, capture the RIGHT lump's piece of the bottom
    // face, then widen the slot leftward: both lumps carry ':face:bottom', so
    // resolution must follow the captured centroid to the right lump.
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (20.0, 10.0)));
    add_extrude(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);
    add_sketch(&mut g, "sketch_3", rect_sketch((9.0, -2.0), (11.0, 12.0)));
    add_extrude(&mut g, "extrude_4", "sketch_3", 10.0, ExtrudeMode::Cut);

    let bodies = g.evaluate_bodies(&no_hidden()).unwrap();
    let right_bottom = bodies[0]
        .1
        .face_refs
        .iter()
        .filter(|f| {
            f.topology
                .as_ref()
                .and_then(|t| t.face_id.as_deref())
                .is_some_and(|id| id.ends_with(":face:bottom"))
        })
        .max_by(|a, b| a.centroid[0].partial_cmp(&b.centroid[0]).unwrap())
        .expect("severed bar exposes bottom faces")
        .clone();
    assert!(
        right_bottom.centroid[0] > 10.0,
        "captured the RIGHT lump's bottom, got {:?}",
        right_bottom.centroid
    );
    let face = FaceRef {
        centroid: right_bottom.centroid,
        normal: right_bottom.normal,
        topology: right_bottom.topology.clone().map(|t| TopologyFaceRef {
            body_id: t.body_id.or_else(|| Some("extrude_2".to_string())),
            component_id: t.component_id,
            topology_version: t.topology_version,
            face_id: t.face_id,
            surface_kind: t.surface_kind,
            producer_feature_id: t.producer_feature_id,
            source_entity_id: t.source_entity_id,
        }),
    };

    // Widen the slot leftward (x 5..11): the right lump keeps its position.
    for idx in g.graph.node_indices() {
        if g.graph[idx].id == "sketch_3" {
            if let FeatureType::Sketch { curves, .. } = &mut g.graph[idx].feature {
                *curves = rect_sketch((5.0, -2.0), (11.0, 12.0));
            }
        }
    }
    let (live, _warnings) = g.build_live(&no_hidden(), false).unwrap();
    let resolved = resolve_face_ref_by_topology(&live[0], &face)
        .expect("the bottom face must reattach after the slot edit");
    assert!(
        resolved.centroid[0] > 10.0,
        "the reference must follow the RIGHT lump, got {:?}",
        resolved.centroid
    );
}

// ---------------------------------------------------------------------------
// Constraint solver (S2) — solved sketches extrude; failures degrade loudly
// ---------------------------------------------------------------------------

/// Sketch whose geometry comes from the SOLVER model: a rectangle promoted to
/// points/lines/constraints with `w`-bound width, extruded 8mm.
fn solved_rect_graph(w0: f64) -> ParametricGraph {
    use crate::sketch::constraints::promote_shapes_to_entities;
    let mut vars_map = HashMap::new();
    vars_map.insert("w".to_string(), w0);
    let shapes = vec![SketchShape::Rectangle {
        origin: (0.0, 0.0),
        sx: 1.0,
        sy: 1.0,
        w: Dimension {
            value: w0 as f32,
            expr: Some("w".to_string()),
        },
        h: Dimension::literal(12.0),
        from_center: false,
    }];
    let ids = crate::sketch::EntityId::sequence(shapes.len());
    let (model, next) = promote_shapes_to_entities(&shapes, &ids, &vars_map, 1);

    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "vars_1".to_string(),
        name: "Vars".to_string(),
        feature: FeatureType::VariableSet {
            variables: vec![Variable {
                name: "w".to_string(),
                value: w0,
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
            shapes,
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
            entity_ids: ids,
            next_entity_id: next,
            solver: Some(model),
        },
    });
    add_extrude(&mut g, "extrude_3", "sketch_2", 8.0, ExtrudeMode::NewBody);
    g
}

#[test]
fn solver_backed_sketch_extrudes_and_follows_its_variable() {
    let mut g = solved_rect_graph(20.0);
    let bodies = g.evaluate_bodies(&no_hidden()).unwrap();
    assert!(!bodies.is_empty() && !bodies[0].1.vertices.is_empty());
    let max_x = |mesh: &MockMesh| {
        mesh.vertices
            .chunks(6)
            .map(|v| v[0])
            .fold(f32::MIN, f32::max)
    };
    assert!((max_x(&bodies[0].1) - 20.0).abs() < 0.1, "width 20 baked");

    // Editing the variable re-solves the constraint model and moves the body.
    set_variable(&mut g, "w", 30.0);
    let bodies = g.evaluate_bodies(&no_hidden()).unwrap();
    assert!(
        (max_x(&bodies[0].1) - 30.0).abs() < 0.1,
        "solver-backed extrude must follow the variable, got {}",
        max_x(&bodies[0].1)
    );
}

#[test]
fn conflicting_sketch_degrades_to_last_valid_and_reports_unresolved() {
    let mut g = solved_rect_graph(20.0);
    // Sanity: solves clean first.
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&no_hidden()).unwrap();
    assert!(warnings.is_empty(), "clean baseline, got {warnings:?}");
    let baseline_tris = bodies[0].1.indices.len();

    // Sabotage: a second, contradictory width dimension (also variable-bound so
    // evaluation re-solves rather than trusting stored positions).
    for idx in g.graph.node_indices() {
        if let FeatureType::Sketch {
            solver: Some(model),
            ..
        } = &mut g.graph[idx].feature
        {
            let (a, b) = (model.points[0].id, model.points[1].id);
            model.constraints.push(crate::sketch::Constraint::Distance {
                id: crate::sketch::EntityId(900),
                a,
                b,
                d: Dimension {
                    value: 25.0,
                    expr: Some("w + 5".to_string()),
                },
            });
        }
    }

    let (bodies, _warnings, statuses) = g.evaluate_bodies_with_status(&no_hidden()).unwrap();
    // The body survives on the last-valid bake — never blanked.
    assert_eq!(
        bodies[0].1.indices.len(),
        baseline_tris,
        "a conflicting solve must keep the last valid geometry"
    );
    // And the failure is attributed, not silent: the consuming extrude is
    // Unresolved with a reason.
    let ex = statuses
        .iter()
        .find(|s| s.feature_id == "extrude_3")
        .expect("extrude status present");
    assert!(
        ex.is_unresolved(),
        "the extrude consuming a conflicting sketch must be Unresolved, got {:?}",
        ex.state
    );
    assert!(
        ex.reason().is_some_and(|r| r.contains("conflict")),
        "the reason names the conflict, got {:?}",
        ex.reason()
    );
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
            component_id: t.component_id.clone(),
            topology_version: t.topology_version,
            face_id: t.face_id.clone(),
            surface_kind: t.surface_kind.clone(),
            producer_feature_id: t.producer_feature_id.clone(),
            source_entity_id: t.source_entity_id.clone(),
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
            component_id: None,
            topology_version: Some(0),
            face_id: Some("sketch:extrude_3:region:0:face:does-not-exist".to_string()),
            surface_kind: None,
            producer_feature_id: Some("extrude_3".to_string()),
            source_entity_id: None,
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
            component_id: t.component_id,
            topology_version: t.topology_version,
            face_id: t.face_id,
            surface_kind: t.surface_kind,
            producer_feature_id: t.producer_feature_id,
            source_entity_id: t.source_entity_id,
        }),
    };

    // A pocket in the middle (x 8..12, y 4..8) punched through the body: the top
    // face gets a hole but survives on the same plane.
    add_sketch(&mut g, "sketch_4", rect_sketch((8.0, 4.0), (12.0, 8.0)));
    add_extrude(&mut g, "extrude_5", "sketch_4", 8.0, ExtrudeMode::Cut);

    let (live, warnings) = g.build_live(&no_hidden(), false).unwrap();
    assert!(
        warnings.is_empty(),
        "cut should apply cleanly, got {warnings:?}"
    );
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
            component_id: t.component_id,
            topology_version: t.topology_version,
            face_id: t.face_id,
            surface_kind: t.surface_kind,
            producer_feature_id: t.producer_feature_id,
            source_entity_id: t.source_entity_id,
        }),
    };

    // A through-slot (x 9..11, full y and z) severs the bar into [0,9] and [11,20].
    add_sketch(
        &mut g,
        "slot_sketch",
        rect_sketch((9.0, -2.0), (11.0, 12.0)),
    );
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
            component_id: t.component_id,
            topology_version: t.topology_version,
            face_id: t.face_id,
            surface_kind: t.surface_kind,
            producer_feature_id: t.producer_feature_id,
            source_entity_id: t.source_entity_id,
        }),
    };

    // A sketch on that face + a boss extruded from it.
    add_sketch(
        &mut g,
        "on_face_sketch",
        rect_sketch((2.0, 2.0), (6.0, 6.0)),
    );
    g.sketch_face_refs
        .insert("on_face_sketch".to_string(), face);
    g.add_dependency("base", "on_face_sketch");
    add_extrude(&mut g, "boss", "on_face_sketch", 3.0, ExtrudeMode::NewBody);

    let boss_base_z = |g: &ParametricGraph| -> f32 {
        let bodies = g.evaluate_bodies(&no_hidden()).unwrap();
        let boss = bodies
            .iter()
            .find(|(id, _)| id == "boss")
            .expect("boss body");
        boss.1
            .vertices
            .chunks(6)
            .map(|v| v[2])
            .fold(f32::MAX, f32::min)
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
            component_id: t.component_id,
            topology_version: t.topology_version,
            face_id: t.face_id,
            surface_kind: t.surface_kind,
            producer_feature_id: t.producer_feature_id,
            source_entity_id: t.source_entity_id,
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
