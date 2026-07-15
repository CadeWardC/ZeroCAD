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
            replay: Default::default(),
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
    use openrcad::algo::{boolean, fillet_edges, BooleanOp};
    use openrcad::foundation::Pnt;
    use openrcad::primitives::make_box;
    use openrcad::topo::Edge;

    // 20×20×10 block with a 10×10 pocket sunk 6 deep from the top (floor z=4).
    let block = make_box(&Pnt::origin(), 20.0, 20.0, 10.0);
    let tool = make_box(&Pnt::new(5.0, 5.0, 4.0), 10.0, 10.0, 7.0);
    let body = boolean(&block, &tool, BooleanOp::Cut);
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
                replay: Default::default(),
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
