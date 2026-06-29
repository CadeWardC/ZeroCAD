use super::*;
pub(super) fn rect_sketch(min: (f32, f32), max: (f32, f32)) -> SketchCurves {
    let mut c = SketchCurves::new();
    c.add_rectangle(min, max);
    c
}

pub(super) fn add_extrude(
    g: &mut ParametricGraph,
    id: &str,
    sketch_id: &str,
    depth: f32,
    mode: ExtrudeMode,
) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Extrude {
            depth,
            region_indices: vec![],
            mode,
            depth_expr: None,
        },
    });
    g.add_dependency(sketch_id, id);
}

pub(super) fn add_sketch(g: &mut ParametricGraph, id: &str, curves: SketchCurves) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::XY,
            curves,
            shapes: vec![],
            corner_mods: vec![],
            on_face: false,
        },
    });
}

pub(super) fn edge_ref_from_mesh_edge(
    body_id: &str,
    edge: &crate::mock_kernel::MeshEdgeRef,
) -> EdgeRef {
    let topology = edge.topology.as_ref().map(|topology| TopologyEdgeRef {
        body_id: topology
            .body_id
            .clone()
            .or_else(|| Some(body_id.to_string())),
        topology_version: topology.topology_version,
        edge_id: topology.edge_id.clone(),
        adjacent_face_ids: topology.adjacent_face_ids.clone(),
        curve_kind: topology.curve_kind.clone(),
        adjacent_surface_kinds: topology.adjacent_surface_kinds.clone(),
    });
    EdgeRef {
        p0: edge.p0,
        p1: edge.p1,
        n1: edge.n1,
        n2: edge.n2,
        curve: edge.curve.clone(),
        topology,
    }
}

/// Overwrite an edge-mod node's distance in place — mimics a fillet/chamfer
/// radius drag, which changes only that trailing node.
pub(super) fn set_edge_mod_dist(g: &mut ParametricGraph, id: &str, new_dist: f32) {
    let idx = g.node_map[id];
    if let FeatureType::EdgeMod { dist, .. } = &mut g.graph[idx].feature {
        *dist = new_dist;
    }
}

/// A flat (id, vertex bytes, index) summary for exact mesh comparison.
pub(super) fn mesh_digest(bodies: &[(String, MockMesh)]) -> Vec<(String, Vec<u32>, Vec<u32>)> {
    bodies
        .iter()
        .map(|(id, m)| {
            (
                id.clone(),
                m.vertices.iter().map(|f| f.to_bits()).collect(),
                m.indices.clone(),
            )
        })
        .collect()
}

pub(super) fn box_with_edge_mod(dist: f32, kind: crate::sketch::CornerKind) -> ParametricGraph {
    let mut g = ParametricGraph::new();
    // A 10×10×10 block, one corner at the origin.
    g.add_feature(FeatureNode {
        id: "box_1".to_string(),
        name: "Box".to_string(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
    // Bevel/round the bottom-front edge (along +X at y=0, z=0): the two
    // adjacent faces are -Z (front) and -Y (bottom).
    g.add_feature(FeatureNode {
        id: "edgemod_2".to_string(),
        name: "Edge Mod".to_string(),
        feature: FeatureType::EdgeMod {
            target: "box_1".to_string(),
            edge: EdgeRef {
                p0: [0.0, 0.0, 0.0],
                p1: [10.0, 0.0, 0.0],
                n1: [0.0, 0.0, -1.0],
                n2: [0.0, -1.0, 0.0],
                curve: None,
                topology: None,
            },
            dist,
            dist_expr: None,
            scope: EdgeModScope::FullEdge,
            kind,
        },
    });
    g.add_dependency("box_1", "edgemod_2");
    g
}

pub(super) fn add_sketch_cs(
    g: &mut ParametricGraph,
    id: &str,
    cs: CoordinateSystem,
    curves: SketchCurves,
) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Sketch {
            cs,
            curves,
            shapes: vec![],
            corner_mods: vec![],
            on_face: false,
        },
    });
}
pub(super) fn box_with_boss_then_edge_mod(
    dist: f32,
    kind: crate::sketch::CornerKind,
) -> ParametricGraph {
    use crate::geometry::Vec3;
    // 10³ box + a Ø6 boss joined on top (z=10..15), then a fillet/chamfer on a
    // *bottom* box edge (well clear of the boss). Exercises an edge-mod on a
    // boolean-union body whose top is now a smooth analytic cylinder.
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "box_1".to_string(),
        name: "Box".to_string(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
    let top = CoordinateSystem::new(Vec3::new(0.0, 0.0, 10.0), Vec3::X, Vec3::Y);
    let mut circ = SketchCurves::new();
    circ.add_circle((5.0, 5.0), 3.0);
    add_sketch_cs(&mut g, "sketch_2", top, circ);
    add_extrude(&mut g, "extrude_3", "sketch_2", 5.0, ExtrudeMode::Join);
    g.add_dependency("box_1", "extrude_3");
    g.add_feature(FeatureNode {
        id: "edgemod_4".to_string(),
        name: "Edge Mod".to_string(),
        feature: FeatureType::EdgeMod {
            target: "box_1".to_string(),
            edge: EdgeRef {
                p0: [0.0, 0.0, 0.0],
                p1: [10.0, 0.0, 0.0],
                n1: [0.0, 0.0, -1.0],
                n2: [0.0, -1.0, 0.0],
                curve: None,
                topology: None,
            },
            dist,
            dist_expr: None,
            scope: EdgeModScope::FullEdge,
            kind,
        },
    });
    g.add_dependency("extrude_3", "edgemod_4");
    g
}
