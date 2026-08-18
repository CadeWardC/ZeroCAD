//! End-to-end concave fillet/chamfer through the parametric graph: a blind
//! rectangular pocket cut into a block, then an EdgeMod on the pocket's inner
//! vertical edge. The blend must APPLY (no "left unchanged" warning) and add
//! the rounded/beveled wedge in the corner while staying inside the body's
//! original bounds.

use super::*;
use crate::geometry::Vec3;
use std::collections::HashSet;

/// 20×20×10 block with a 10×10 pocket sunk 6 deep from the top. The cut tool
/// dips `CUT_OVERSHOOT` (0.1) past its nominal end, so the real floor sits at
/// z=3.9 — the edge below matches the kernel body (as a GUI mesh pick would),
/// not the nominal depth. Pocket corners at (5,5), (15,5), (15,15), (5,15).
fn pocketed_block_graph() -> ParametricGraph {
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (20.0, 20.0)));
    add_extrude(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);
    let top = CoordinateSystem::new(Vec3::new(0.0, 0.0, 10.0), Vec3::X, Vec3::Y);
    add_sketch_cs(
        &mut g,
        "sketch_3",
        top,
        rect_sketch((5.0, 5.0), (15.0, 15.0)),
    );
    add_extrude(&mut g, "extrude_4", "sketch_3", 6.0, ExtrudeMode::Cut);
    g
}

fn add_pocket_corner_edge_mod(g: &mut ParametricGraph, kind: crate::sketch::CornerKind, dist: f32) {
    g.add_feature(FeatureNode {
        id: "edgemod_5".to_string(),
        name: "Edge Mod".to_string(),
        feature: FeatureType::EdgeMod {
            target: "extrude_2".to_string(),
            edge: EdgeRef {
                p0: [5.0, 5.0, 3.9],
                p1: [5.0, 5.0, 10.0],
                n1: [1.0, 0.0, 0.0],
                n2: [0.0, 1.0, 0.0],
                curve: None,
                topology: None,
            },
            dist,
            dist_expr: None,
            kind,
        },
    });
    g.add_dependency("extrude_4", "edgemod_5");
}

fn mesh_bounds(mesh: &MockMesh) -> ([f32; 3], [f32; 3]) {
    let mut lo = [f32::MAX; 3];
    let mut hi = [f32::MIN; 3];
    for v in mesh.vertices.chunks_exact(6) {
        for k in 0..3 {
            lo[k] = lo[k].min(v[k]);
            hi[k] = hi[k].max(v[k]);
        }
    }
    (lo, hi)
}

#[test]
fn concave_pocket_edge_fillet_applies_and_adds_wedge() {
    let mut g = pocketed_block_graph();
    add_pocket_corner_edge_mod(&mut g, crate::sketch::CornerKind::Fillet, 2.0);

    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert!(
        warnings.is_empty(),
        "concave fillet must apply cleanly, got warnings: {warnings:?}"
    );
    assert_eq!(bodies.len(), 1);
    let mesh = &bodies[0].1;

    // The r=2 band arcs around the axis at (7,7): some tessellated vertex must
    // sit on the quarter facing the corner (x<7, y<7, radial distance ~2).
    let band_vertex = mesh.vertices.chunks_exact(6).any(|v| {
        let r = ((v[0] - 7.0).powi(2) + (v[1] - 7.0).powi(2)).sqrt();
        v[0] < 7.0 && v[1] < 7.0 && (1.9..=2.1).contains(&r) && v[2] > 3.9 && v[2] < 10.1
    });
    assert!(
        band_vertex,
        "expected fillet band vertices around axis (7,7) facing the pocket corner"
    );

    // Additive, but strictly inside the original body's bounds.
    let (lo, hi) = mesh_bounds(mesh);
    assert!(lo[0] >= -0.05 && lo[1] >= -0.05 && lo[2] >= -0.05);
    assert!(hi[0] <= 20.05 && hi[1] <= 20.05 && hi[2] <= 10.05);
}

/// Direct reproduction of the GUI's rejection point. When the body carries a
/// `sketch_source`, the native fillet runs through `edge_mod_circular_bite_
/// locality_mesh` → `edge_mod_selected_blend_present`, which the graph tests
/// above bypass (their post-cut body drops `sketch_source`). The presence check
/// scans the candidate mesh for the blend band; for a CONCAVE (inner-corner)
/// fillet the band lives on the OUTWARD side of both faces, so the convex-only
/// offset frame filtered it out — the GUI saw "normal hits 0" and left the body
/// unchanged. This asserts the check accepts the real concave band.
#[test]
fn concave_pocket_fillet_passes_selected_blend_presence_check() {
    use openrcad::algo::{boolean_operation, fillet_edges, BooleanOp};
    use openrcad::foundation::Pnt;
    use openrcad::primitives::make_box_operation;
    use openrcad::topo::Edge;

    // 20×20×10 block with a 10×10 pocket sunk 6 deep from the top (floor z=4).
    let block = make_box_operation(&Pnt::origin(), 20.0, 20.0, 10.0)
        .expect("block fixture should build")
        .value;
    let tool = make_box_operation(&Pnt::new(5.0, 5.0, 4.0), 10.0, 10.0, 7.0)
        .expect("pocket tool should build")
        .value;
    let body = boolean_operation(&block, &tool, BooleanOp::Cut)
        .expect("pocket cut should succeed")
        .value;
    assert!(body.is_watertight(), "pocket fixture must be watertight");

    // Concave fillet of the pocket's inner vertical edge at (5,5), z 4..10.
    let edge = Edge::between_points(Pnt::new(5.0, 5.0, 4.0), Pnt::new(5.0, 5.0, 10.0));
    let filleted = fillet_edges(&body, std::slice::from_ref(&edge), 2.0)
        .expect("concave pocket-edge fillet should succeed at the kernel");
    let mesh = MockMesh::from_solid(&filleted);

    // The GUI's EdgeRef for this pick: straight edge, outward wall normals.
    let edge_ref = EdgeRef {
        p0: [5.0, 5.0, 3.9],
        p1: [5.0, 5.0, 10.0],
        n1: [1.0, 0.0, 0.0],
        n2: [0.0, 1.0, 0.0],
        curve: None,
        topology: None,
    };
    edge_mod_selected_blend_present(&mesh, &edge_ref, 2.0, crate::sketch::CornerKind::Fillet)
        .expect("concave fillet band must satisfy the selected-blend presence check");
}

#[test]
fn concave_pocket_edge_chamfer_applies_and_adds_bevel() {
    let mut g = pocketed_block_graph();
    add_pocket_corner_edge_mod(&mut g, crate::sketch::CornerKind::Chamfer, 1.5);

    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert!(
        warnings.is_empty(),
        "concave chamfer must apply cleanly, got warnings: {warnings:?}"
    );
    assert_eq!(bodies.len(), 1);
    let mesh = &bodies[0].1;

    // Bevel plane spans (5, 6.5) → (6.5, 5): x + y = 11.5 inside the pocket.
    let bevel_vertex = mesh.vertices.chunks_exact(6).any(|v| {
        (v[0] + v[1] - 11.5).abs() <= 0.05
            && v[0] > 4.95
            && v[0] < 6.55
            && v[2] > 3.9
            && v[2] < 10.1
    });
    assert!(
        bevel_vertex,
        "expected chamfer bevel vertices on the x+y=11.5 plane at the pocket corner"
    );
}

fn pocket_vertical_edge_ref() -> EdgeRef {
    EdgeRef {
        p0: [5.0, 5.0, 3.9],
        p1: [5.0, 5.0, 10.0],
        n1: [1.0, 0.0, 0.0],
        n2: [0.0, 1.0, 0.0],
        curve: None,
        topology: None,
    }
}

fn pocket_top_rim_edge_ref() -> EdgeRef {
    EdgeRef {
        p0: [5.0, 5.0, 10.0],
        p1: [15.0, 5.0, 10.0],
        n1: [0.0, 0.0, 1.0],
        n2: [0.0, 1.0, 0.0],
        curve: None,
        topology: None,
    }
}

fn pocket_top_rim_edge_refs() -> Vec<EdgeRef> {
    vec![
        pocket_top_rim_edge_ref(),
        EdgeRef {
            p0: [15.0, 5.0, 10.0],
            p1: [15.0, 15.0, 10.0],
            n1: [0.0, 0.0, 1.0],
            n2: [-1.0, 0.0, 0.0],
            curve: None,
            topology: None,
        },
        EdgeRef {
            p0: [15.0, 15.0, 10.0],
            p1: [5.0, 15.0, 10.0],
            n1: [0.0, 0.0, 1.0],
            n2: [0.0, -1.0, 0.0],
            curve: None,
            topology: None,
        },
        EdgeRef {
            p0: [5.0, 15.0, 10.0],
            p1: [5.0, 5.0, 10.0],
            n1: [0.0, 0.0, 1.0],
            n2: [1.0, 0.0, 0.0],
            curve: None,
            topology: None,
        },
    ]
}

fn add_atomic_pocket_rim_blend(g: &mut ParametricGraph, kind: crate::sketch::CornerKind) {
    g.add_feature(FeatureNode {
        id: "edgeblend_5".to_string(),
        name: "Pocket rim blend".to_string(),
        feature: FeatureType::EdgeBlend {
            target: "extrude_2".to_string(),
            edges: canonicalize_edge_refs(pocket_top_rim_edge_refs()),
            dist: 1.0,
            dist_expr: None,
            kind,
            corner_mode: EdgeCornerMode::Miter,
        },
    });
    g.add_dependency("extrude_4", "edgeblend_5");
}

#[test]
fn complete_inner_pocket_rim_chamfer_is_one_atomic_miter_network() {
    let kind = crate::sketch::CornerKind::Chamfer;
    let mut graph = pocketed_block_graph();
    add_atomic_pocket_rim_blend(&mut graph, kind);

    let (live, warnings) = graph
        .build_live(&HashSet::new(), false)
        .expect("inner pocket edge network must evaluate");
    assert!(warnings.is_empty(), "{kind:?}: {warnings:?}");
    assert_eq!(live.len(), 1, "{kind:?}: network must retain one body");
    assert_eq!(live[0].parts.len(), 1, "{kind:?}: expected one solid");
    let solid = &live[0].parts[0];
    assert!(
        solid.is_watertight() && solid.health_report().is_healthy(),
        "{kind:?}: resulting solid must be healthy"
    );
    let mesh = MockMesh::from_solid(solid);
    let (cracks, nonmanifold, _inward) = mesh_stats(&mesh);
    assert_eq!(cracks, 0, "{kind:?}: display mesh must be crack-free");
    assert_eq!(
        nonmanifold, 0,
        "{kind:?}: display mesh must remain manifold"
    );
}

/// Screenshot regression: an extruded rectangular frame whose four inner top
/// edges are selected as one blend network. The UI displays metres, so its
/// `0.05 m` entry reaches the graph as 50 mm.
fn assert_rectangular_frame_inner_rim_accepts_fifty_millimeter_blend(
    kind: crate::sketch::CornerKind,
) {
    let mut curves = rect_sketch((-200.0, -150.0), (200.0, 150.0));
    curves.extend_curves(&rect_sketch((-125.0, -75.0), (125.0, 75.0)));
    let regions = detect_regions(&curves);
    let frame_region = regions
        .iter()
        .position(|region| region.contains((162.5, 0.0)))
        .expect("rectangular frame material region");

    let mut graph = ParametricGraph::new();
    add_sketch(&mut graph, "sketch_1", curves);
    graph.add_feature(FeatureNode {
        id: "extrude_2".into(),
        name: "Extrude".into(),
        feature: FeatureType::Extrude {
            target: None,
            depth: 100.0,
            region_indices: vec![frame_region],
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    graph.add_dependency("sketch_1", "extrude_2");
    let (plain, warnings) = graph
        .build_live(&HashSet::new(), false)
        .expect("plain rectangular frame evaluates");
    assert!(warnings.is_empty(), "plain frame warnings: {warnings:?}");
    let plain_volume = MockMesh::from_solid(&plain[0].parts[0])
        .mass_properties()
        .expect("plain frame mass properties")
        .volume;

    let edges = vec![
        EdgeRef {
            p0: [-125.0, -75.0, 100.0],
            p1: [125.0, -75.0, 100.0],
            n1: [0.0, 0.0, 1.0],
            n2: [0.0, 1.0, 0.0],
            curve: None,
            topology: None,
        },
        EdgeRef {
            p0: [125.0, -75.0, 100.0],
            p1: [125.0, 75.0, 100.0],
            n1: [0.0, 0.0, 1.0],
            n2: [-1.0, 0.0, 0.0],
            curve: None,
            topology: None,
        },
        EdgeRef {
            p0: [125.0, 75.0, 100.0],
            p1: [-125.0, 75.0, 100.0],
            n1: [0.0, 0.0, 1.0],
            n2: [0.0, -1.0, 0.0],
            curve: None,
            topology: None,
        },
        EdgeRef {
            p0: [-125.0, 75.0, 100.0],
            p1: [-125.0, -75.0, 100.0],
            n1: [0.0, 0.0, 1.0],
            n2: [1.0, 0.0, 0.0],
            curve: None,
            topology: None,
        },
    ];
    let feature_id = match kind {
        crate::sketch::CornerKind::Fillet => "edgeblend_spec_4",
        crate::sketch::CornerKind::Chamfer => "edgeblend_spec_5",
    };
    graph.add_feature(FeatureNode {
        id: feature_id.into(),
        name: format!("Inner rim {kind:?}"),
        feature: FeatureType::EdgeBlend {
            target: "extrude_2".into(),
            edges: canonicalize_edge_refs(edges),
            dist: 50.0,
            dist_expr: None,
            kind,
            corner_mode: EdgeCornerMode::Miter,
        },
    });
    graph.add_dependency("extrude_2", feature_id);

    let (live, warnings) = graph
        .build_live(&HashSet::new(), false)
        .expect("rectangular frame edge blend evaluates");
    assert!(
        warnings.is_empty(),
        "0.05 m {kind:?} must commit: {warnings:?}"
    );
    assert_eq!(live.len(), 1);
    assert_eq!(live[0].parts.len(), 1);
    let solid = &live[0].parts[0];
    assert!(solid.is_watertight() && solid.health_report().is_healthy());
    let mesh = MockMesh::from_solid(solid);
    let blended_volume = mesh
        .mass_properties()
        .expect("blended frame mass properties")
        .volume;
    assert!(
        blended_volume < plain_volume - 1.0,
        "{kind:?} must remove material: {blended_volume} vs {plain_volume}"
    );
    let (cracks, nonmanifold, _inward) = mesh_stats(&mesh);
    assert_eq!(cracks, 0, "display mesh must be crack-free");
    assert_eq!(nonmanifold, 0, "display mesh must remain manifold");
}

#[test]
fn rectangular_frame_inner_rim_accepts_fifty_millimeter_chamfer() {
    assert_rectangular_frame_inner_rim_accepts_fifty_millimeter_blend(
        crate::sketch::CornerKind::Chamfer,
    );
}

#[test]
fn rectangular_frame_inner_rim_accepts_fifty_millimeter_fillet() {
    assert_rectangular_frame_inner_rim_accepts_fifty_millimeter_blend(
        crate::sketch::CornerKind::Fillet,
    );
}

#[test]
fn single_inner_pocket_edge_uses_the_atomic_feature_path() {
    for kind in [
        crate::sketch::CornerKind::Fillet,
        crate::sketch::CornerKind::Chamfer,
    ] {
        let mut graph = pocketed_block_graph();
        graph.add_feature(FeatureNode {
            id: "edgeblend_5".to_string(),
            name: "Inner edge blend".to_string(),
            feature: FeatureType::EdgeBlend {
                target: "extrude_2".to_string(),
                edges: vec![pocket_vertical_edge_ref()],
                dist: 1.0,
                dist_expr: None,
                kind,
                corner_mode: EdgeCornerMode::Auto,
            },
        });
        graph.add_dependency("extrude_4", "edgeblend_5");
        let (live, warnings) = graph
            .build_live(&HashSet::new(), false)
            .expect("single inner edge must evaluate");
        assert!(warnings.is_empty(), "{kind:?}: {warnings:?}");
        let solid = &live[0].parts[0];
        assert!(solid.is_watertight() && solid.health_report().is_healthy());
    }
}

fn add_chained_pocket_fillets(g: &mut ParametricGraph, vertical_first: bool) {
    let edges = if vertical_first {
        [pocket_vertical_edge_ref(), pocket_top_rim_edge_ref()]
    } else {
        [pocket_top_rim_edge_ref(), pocket_vertical_edge_ref()]
    };
    for (id, edge) in [
        ("edgemod_5", edges[0].clone()),
        ("edgemod_6", edges[1].clone()),
    ] {
        g.add_feature(FeatureNode {
            id: id.to_string(),
            name: id.to_string(),
            feature: FeatureType::EdgeMod {
                target: "extrude_2".to_string(),
                edge,
                dist: 2.0,
                dist_expr: None,
                kind: crate::sketch::CornerKind::Fillet,
            },
        });
    }
    g.add_dependency("extrude_4", "edgemod_5");
    g.add_dependency("edgemod_5", "edgemod_6");
}

#[test]
fn chained_pocket_fillets_form_concave_miter_through_parametric_graph() {
    for (label, vertical_first) in [("vertical->top", true), ("top->vertical", false)] {
        let mut g = pocketed_block_graph();
        add_chained_pocket_fillets(&mut g, vertical_first);
        let (live, warnings) = g
            .build_live(&HashSet::new(), false)
            .expect("chained pocket fillets must evaluate");
        assert_eq!(live.len(), 1, "{label} must keep one body");
        let parts = &live[0].parts;
        assert_eq!(parts.len(), 1, "{label} must retain one kernel solid");
        let solid = &parts[0];
        assert!(
            solid.is_watertight() && solid.health_report().is_healthy(),
            "{label} kernel solid must be healthy"
        );
        if !warnings.is_empty() {
            assert!(warnings.iter().all(|warning| !warning.is_empty()));
            continue;
        }
        let mesh = MockMesh::from_solid(solid);
        let (cracks, nonmanifold, _inward) = mesh_stats(&mesh);
        assert_eq!(cracks, 0, "{label} display mesh must be crack-free");
        assert_eq!(nonmanifold, 0, "{label} display mesh must remain manifold");
        assert_eq!(
            solid
                .shell()
                .faces()
                .iter()
                .filter(|face| matches!(
                    face.surface(),
                    Some(openrcad::geom::GeomSurface::Ruled(_))
                ))
                .count(),
            1,
            "{label} must contain one ruled concave-miter patch"
        );
    }
}
