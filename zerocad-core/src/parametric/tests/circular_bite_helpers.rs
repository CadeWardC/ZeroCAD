use super::*;
pub(super) fn circular_bite_curves() -> SketchCurves {
    let mut c = SketchCurves::new();
    c.add_rectangle((0.0, 5.0), (40.0, 35.0));
    c.add_circle((20.0, 8.0), 14.0);
    c
}

pub(super) fn circular_bite_region_index(curves: &SketchCurves) -> usize {
    let regions = detect_regions(curves);
    regions
        .iter()
        .position(|r| r.contains((5.0, 30.0)))
        .expect("rectangular material region above the circular bite")
}

pub(super) fn circular_bite_graph_with_depth(
    edge_mod: Option<crate::sketch::CornerKind>,
    depth: f32,
) -> ParametricGraph {
    let curves = circular_bite_curves();
    let region = circular_bite_region_index(&curves);
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "s", curves);
    g.add_feature(FeatureNode {
        id: "e".to_string(),
        name: "e".to_string(),
        feature: FeatureType::Extrude {
            depth,
            region_indices: vec![region],
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
        },
    });
    g.add_dependency("s", "e");

    if let Some(kind) = edge_mod {
        g.add_feature(FeatureNode {
            id: "em".to_string(),
            name: "Edge Mod".to_string(),
            feature: FeatureType::EdgeMod {
                target: "e".to_string(),
                edge: EdgeRef {
                    p0: [0.0, 35.0, depth],
                    p1: [40.0, 35.0, depth],
                    n1: [0.0, 0.0, 1.0],
                    n2: [0.0, 1.0, 0.0],
                    curve: None,
                    topology: None,
                },
                dist: 1.5,
                dist_expr: None,
                scope: EdgeModScope::FullEdge,
                kind,
            },
        });
        g.add_dependency("e", "em");
    }

    g
}

pub(super) fn circular_bite_graph(edge_mod: Option<crate::sketch::CornerKind>) -> ParametricGraph {
    circular_bite_graph_with_depth(edge_mod, 10.0)
}

pub(super) fn circular_bite_cutoff_edge_at_depth(depth: f32) -> EdgeRef {
    let x = 20.0 - (14.0_f32 * 14.0 - 3.0_f32 * 3.0).sqrt();
    EdgeRef {
        p0: [0.0, 5.0, depth],
        p1: [x, 5.0, depth],
        n1: [0.0, 0.0, 1.0],
        n2: [0.0, -1.0, 0.0],
        curve: None,
        topology: None,
    }
}

pub(super) fn circular_bite_cutoff_edge() -> EdgeRef {
    circular_bite_cutoff_edge_at_depth(10.0)
}

pub(super) fn gui_captured_circular_bite_cutoff_edge(depth: f32) -> EdgeRef {
    let g = circular_bite_graph_with_depth(None, depth);
    let bodies = g
        .evaluate_bodies(&std::collections::HashSet::new())
        .unwrap();
    let body_id = bodies[0].0.as_str();
    let mesh = &bodies[0].1;
    let expected = circular_bite_cutoff_edge_at_depth(depth);
    let same_edge = |edge: &crate::mock_kernel::MeshEdgeRef| {
        let d = |a: [f32; 3], b: [f32; 3]| {
            ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
        };
        matches!(
            edge.curve,
            None | Some(crate::mock_kernel::EdgeCurveHint::Line)
        ) && ((d(edge.p0, expected.p0) <= 0.08 && d(edge.p1, expected.p1) <= 0.08)
            || (d(edge.p0, expected.p1) <= 0.08 && d(edge.p1, expected.p0) <= 0.08))
    };
    let edge = mesh
        .edge_refs
        .iter()
        .find(|edge| same_edge(edge))
        .unwrap_or_else(|| {
            panic!(
                "display mesh should expose exact cutoff-edge metadata; refs={:?}",
                mesh.edge_refs
            )
        });
    gui_edge_ref_from_mesh_candidate(body_id, edge)
}

pub(super) fn gui_edge_ref_from_mesh_candidate(
    body_id: &str,
    edge: &crate::mock_kernel::MeshEdgeRef,
) -> EdgeRef {
    let topology = edge.topology.as_ref().map(|topology| {
        let mut topology = TopologyEdgeRef {
            body_id: topology.body_id.clone(),
            topology_version: topology.topology_version,
            edge_id: topology.edge_id.clone(),
            adjacent_face_ids: topology.adjacent_face_ids.clone(),
            curve_kind: topology.curve_kind.clone(),
            adjacent_surface_kinds: topology.adjacent_surface_kinds.clone(),
        };
        if topology.body_id.is_none() {
            topology.body_id = Some(body_id.to_string());
        }
        topology
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

pub(super) fn circular_bite_cutoff_edge_graph_with_dist(
    kind: crate::sketch::CornerKind,
    dist: f32,
) -> ParametricGraph {
    let edge = circular_bite_cutoff_edge();
    circular_bite_cutoff_edge_graph_at_depth_with_edge(kind, dist, 10.0, edge)
}

pub(super) fn circular_bite_cutoff_edge_graph_at_depth_with_edge(
    kind: crate::sketch::CornerKind,
    dist: f32,
    depth: f32,
    edge: EdgeRef,
) -> ParametricGraph {
    let mut g = circular_bite_graph_with_depth(None, depth);
    g.add_feature(FeatureNode {
        id: "em".to_string(),
        name: "Edge Mod".to_string(),
        feature: FeatureType::EdgeMod {
            target: "e".to_string(),
            edge,
            dist,
            dist_expr: None,
            scope: EdgeModScope::FullEdge,
            kind,
        },
    });
    g.add_dependency("e", "em");
    g
}

pub(super) fn circular_bite_cutoff_edge_graph(kind: crate::sketch::CornerKind) -> ParametricGraph {
    circular_bite_cutoff_edge_graph_with_dist(kind, 1.0)
}

pub(super) fn assert_selected_blend_surface(
    mesh: &MockMesh,
    edge: &EdgeRef,
    dist: f32,
    kind: crate::sketch::CornerKind,
    label: &str,
) {
    edge_mod_selected_blend_present(mesh, edge, dist, kind)
        .unwrap_or_else(|reason| panic!("{label}: selected edge lacks local {kind:?}: {reason}"));
}

pub(super) fn same_edge_span(a: &EdgeRef, b: &EdgeRef, tol: f32) -> bool {
    let d = |p: [f32; 3], q: [f32; 3]| {
        ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2)).sqrt()
    };
    (d(a.p0, b.p0) <= tol && d(a.p1, b.p1) <= tol) || (d(a.p0, b.p1) <= tol && d(a.p1, b.p0) <= tol)
}

pub(super) fn mesh_stats(m: &MockMesh) -> (usize, usize, usize) {
    use std::collections::HashMap;
    let q = |i: usize| -> (i64, i64, i64) {
        let b = i * 6;
        let quant = |v: f32| (v as f64 * 1e4).round() as i64;
        (
            quant(m.vertices[b]),
            quant(m.vertices[b + 1]),
            quant(m.vertices[b + 2]),
        )
    };
    let mut edges: HashMap<((i64, i64, i64), (i64, i64, i64)), u32> = HashMap::new();
    let mut inward = 0usize;
    for tri in m.indices.chunks_exact(3) {
        for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            let (ka, kb) = (q(tri[a] as usize), q(tri[b] as usize));
            let key = if ka <= kb { (ka, kb) } else { (kb, ka) };
            *edges.entry(key).or_insert(0) += 1;
        }

        let pos = |i: u32| {
            let b = i as usize * 6;
            [
                m.vertices[b] as f64,
                m.vertices[b + 1] as f64,
                m.vertices[b + 2] as f64,
            ]
        };
        let nrm = |i: u32| {
            let b = i as usize * 6;
            [
                m.vertices[b + 3] as f64,
                m.vertices[b + 4] as f64,
                m.vertices[b + 5] as f64,
            ]
        };
        let a = pos(tri[0]);
        let b = pos(tri[1]);
        let c = pos(tri[2]);
        let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let winding = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ];
        let navg = [
            (nrm(tri[0])[0] + nrm(tri[1])[0] + nrm(tri[2])[0]) / 3.0,
            (nrm(tri[0])[1] + nrm(tri[1])[1] + nrm(tri[2])[1]) / 3.0,
            (nrm(tri[0])[2] + nrm(tri[1])[2] + nrm(tri[2])[2]) / 3.0,
        ];
        if winding[0] * navg[0] + winding[1] * navg[1] + winding[2] * navg[2] < 0.0 {
            inward += 1;
        }
    }

    (
        edges.values().filter(|&&c| c == 1).count(),
        edges.values().filter(|&&c| c > 2).count(),
        inward,
    )
}

pub(super) fn on_internal_circular_bite_wall(x: f32, y: f32) -> bool {
    let r = ((x - 20.0).powi(2) + (y - 8.0).powi(2)).sqrt();
    (r - 14.0).abs() < 0.08 && y > 5.25
}

pub(super) fn circular_bite_internal_wall_struts(m: &MockMesh) -> usize {
    (0..m.edge_indices.len() / 2)
        .filter(|&e| {
            let ia = m.edge_indices[e * 2] as usize * 3;
            let ib = m.edge_indices[e * 2 + 1] as usize * 3;
            let a = [
                m.edge_vertices[ia],
                m.edge_vertices[ia + 1],
                m.edge_vertices[ia + 2],
            ];
            let b = [
                m.edge_vertices[ib],
                m.edge_vertices[ib + 1],
                m.edge_vertices[ib + 2],
            ];
            (a[0] - b[0]).abs() < 0.05
                && (a[1] - b[1]).abs() < 0.05
                && (a[2] - b[2]).abs() > 9.0
                && on_internal_circular_bite_wall(a[0], a[1])
                && on_internal_circular_bite_wall(b[0], b[1])
        })
        .count()
}

pub(super) fn circular_bite_wall_normal_splits(m: &MockMesh) -> usize {
    use std::collections::HashMap;

    let quant = |v: f32| (v as f64 * 10_000.0).round() as i64;
    let mut normals: HashMap<(i64, i64, i64), Vec<[f32; 3]>> = HashMap::new();
    for v in m.vertices.chunks_exact(6) {
        if !on_internal_circular_bite_wall(v[0], v[1]) || v[5].abs() > 0.5 {
            continue;
        }
        normals
            .entry((quant(v[0]), quant(v[1]), quant(v[2])))
            .or_default()
            .push([v[3], v[4], v[5]]);
    }

    normals
        .values()
        .filter(|ns| {
            ns.len() >= 2
                && ns.iter().enumerate().any(|(i, a)| {
                    ns.iter()
                        .skip(i + 1)
                        .any(|b| (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]).clamp(-1.0, 1.0) < 0.999)
                })
        })
        .count()
}

pub(super) fn circular_bite_wall_face_ids(m: &MockMesh) -> std::collections::HashSet<u32> {
    let mut faces = std::collections::HashSet::new();
    for (t, tri) in m.indices.chunks_exact(3).enumerate() {
        let on_wall = tri.iter().all(|&vi| {
            let b = vi as usize * 6;
            on_internal_circular_bite_wall(m.vertices[b], m.vertices[b + 1])
                && m.vertices[b + 5].abs() < 0.5
        });
        if on_wall {
            faces.insert(m.face_ids.get(t).copied().unwrap_or(0));
        }
    }
    faces
}

pub(super) fn mesh_has_wire_edge_between(
    m: &MockMesh,
    p0: [f32; 3],
    p1: [f32; 3],
    tol: f32,
) -> bool {
    let dist = |a: [f32; 3], b: [f32; 3]| {
        ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
    };
    let point = |idx: u32| {
        let b = idx as usize * 3;
        [
            m.edge_vertices[b],
            m.edge_vertices[b + 1],
            m.edge_vertices[b + 2],
        ]
    };
    (0..m.edge_indices.len() / 2).any(|e| {
        let a = point(m.edge_indices[e * 2]);
        let b = point(m.edge_indices[e * 2 + 1]);
        (dist(a, p0) <= tol && dist(b, p1) <= tol) || (dist(a, p1) <= tol && dist(b, p0) <= tol)
    })
}

pub(super) fn inside_removed_circular_bite_volume(p: [f32; 3]) -> bool {
    let r = ((p[0] - 20.0).powi(2) + (p[1] - 8.0).powi(2)).sqrt();
    r < 13.0 && p[1] > 5.1 && (-0.05..=10.05).contains(&p[2])
}

pub(super) fn circular_bite_wall_chord_sample(p: [f32; 3], n: [f32; 3]) -> bool {
    let radial = [p[0] - 20.0, p[1] - 8.0];
    let r = (radial[0] * radial[0] + radial[1] * radial[1]).sqrt();
    if !(12.0..=14.25).contains(&r) || p[1] <= 5.0 || n[2].abs() > 0.55 {
        return false;
    }
    let nl = (n[0] * n[0] + n[1] * n[1]).sqrt();
    if nl <= 1.0e-5 || r <= 1.0e-5 {
        return false;
    }
    let dot = (radial[0] / r) * (n[0] / nl) + (radial[1] / r) * (n[1] / nl);
    dot.abs() > 0.75
}

pub(super) fn circular_bite_ghost_sample_count(m: &MockMesh) -> usize {
    let vertex6 = |vi: u32| {
        let b = vi as usize * 6;
        [
            m.vertices[b],
            m.vertices[b + 1],
            m.vertices[b + 2],
            m.vertices[b + 3],
            m.vertices[b + 4],
            m.vertices[b + 5],
        ]
    };
    let mut count = 0usize;
    for v in m.vertices.chunks_exact(6) {
        let p = [v[0], v[1], v[2]];
        let n = [v[3], v[4], v[5]];
        if inside_removed_circular_bite_volume(p) && !circular_bite_wall_chord_sample(p, n) {
            count += 1;
        }
    }
    for tri in m.indices.chunks_exact(3) {
        let a = vertex6(tri[0]);
        let b = vertex6(tri[1]);
        let c = vertex6(tri[2]);
        let p = [
            (a[0] + b[0] + c[0]) / 3.0,
            (a[1] + b[1] + c[1]) / 3.0,
            (a[2] + b[2] + c[2]) / 3.0,
        ];
        let n = [
            (a[3] + b[3] + c[3]) / 3.0,
            (a[4] + b[4] + c[4]) / 3.0,
            (a[5] + b[5] + c[5]) / 3.0,
        ];
        if inside_removed_circular_bite_volume(p) && !circular_bite_wall_chord_sample(p, n) {
            count += 1;
        }
    }
    count
}

pub(super) fn circle_points(center: (f32, f32), radius: f32) -> Vec<(f32, f32)> {
    (0..crate::CIRCLE_SEGS)
        .map(|i| {
            let a = (i as f32 / crate::CIRCLE_SEGS as f32) * std::f32::consts::TAU;
            (center.0 + radius * a.cos(), center.1 + radius * a.sin())
        })
        .collect()
}
