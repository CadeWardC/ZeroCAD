use super::*;

#[test]
fn rect_with_circular_hole_newbody_renders() {
    // The "plate with a hole vanishes on commit" case: a rectangle with a circle
    // inside, extruded as a New Body. detect_regions yields 2 regions — the
    // annulus (rect ⊖ circle) and the inner disk. take_all, region 0, and
    // region 1 must each produce a visible (non-empty) body.
    let mut g = ParametricGraph::new();
    let mut curves = SketchCurves::new();
    curves.add_rectangle((0.0, 0.0), (40.0, 30.0));
    curves.add_circle((20.0, 15.0), 8.0);
    assert_eq!(
        g.cached_regions(&curves).len(),
        2,
        "rect+circle = annulus + disk"
    );

    add_sketch(&mut g, "sketch_1", curves);
    add_extrude(&mut g, "extrude_2", "sketch_1", 11.62, ExtrudeMode::NewBody);
    let (bodies, _) = g
        .evaluate_bodies(&std::collections::HashSet::new())
        .map(|b| (b, ()))
        .unwrap();
    assert!(!bodies.is_empty(), "rect+hole New Body must not vanish");
    assert!(bodies.iter().all(|(_, m)| !m.indices.is_empty()));

    // Each individual region selection must also render a body.
    for sel in [vec![0usize], vec![1]] {
        let mut g2 = ParametricGraph::new();
        let mut c2 = SketchCurves::new();
        c2.add_rectangle((0.0, 0.0), (40.0, 30.0));
        c2.add_circle((20.0, 15.0), 8.0);
        add_sketch(&mut g2, "s", c2);
        g2.add_feature(FeatureNode {
            id: "e".to_string(),
            name: "e".to_string(),
            feature: FeatureType::Extrude {
                depth: 11.62,
                region_indices: sel.clone(),
                mode: ExtrudeMode::NewBody,
                depth_expr: None,
            },
        });
        g2.add_dependency("s", "e");
        let b2 = g2
            .evaluate_bodies(&std::collections::HashSet::new())
            .unwrap();
        let tris: usize = b2.iter().map(|(_, m)| m.indices.len() / 3).sum();
        assert!(
            tris > 0,
            "region selection {sel:?} must render a body, got {tris} tris"
        );
    }
}

#[test]
fn circular_bite_newbody_keeps_clean_mesh_and_cylindrical_solid() {
    let g = circular_bite_graph(None);
    let bodies = g
        .evaluate_bodies(&std::collections::HashSet::new())
        .unwrap();
    assert_eq!(
        bodies.len(),
        1,
        "selected circular-bite region makes one body"
    );
    let mesh = &bodies[0].1;
    let (cracks, nonmanifold, inward) = mesh_stats(mesh);
    assert_eq!(
        cracks, 0,
        "circular-bite display mesh has {cracks} crack edges"
    );
    assert_eq!(
        nonmanifold, 0,
        "circular-bite display mesh has {nonmanifold} non-manifold edges"
    );
    assert_eq!(
        inward, 0,
        "circular-bite display mesh has {inward} inward triangles"
    );

    let solids = g
        .debug_kernel_solids(&std::collections::HashSet::new())
        .unwrap();
    let has_cylinder = solids.iter().any(|(_, parts)| {
        parts.iter().any(|solid| {
            solid
                .shell()
                .faces()
                .iter()
                .any(|f| matches!(f.surface(), Some(openrcad::geom::GeomSurface::Cylinder(_))))
        })
    });
    assert!(
        has_cylinder,
        "circular-bite body should keep an analytic cylindrical wall in the kernel solid"
    );
}

#[test]
fn circular_bite_newbody_hides_internal_wall_segments() {
    let g = circular_bite_graph(None);
    let bodies = g
        .evaluate_bodies(&std::collections::HashSet::new())
        .unwrap();
    assert_eq!(
        bodies.len(),
        1,
        "selected circular-bite region makes one body"
    );
    let mesh = &bodies[0].1;
    assert_eq!(
        circular_bite_internal_wall_struts(mesh),
        0,
        "circular-bite display should not draw vertical construction seams inside the wall"
    );
    assert_eq!(
        circular_bite_wall_normal_splits(mesh),
        0,
        "circular-bite wall should not shade as separate flat panels"
    );
    let wall_faces = circular_bite_wall_face_ids(mesh);
    assert_eq!(
        wall_faces.len(),
        1,
        "circular-bite wall should select as one face, got face ids {wall_faces:?}"
    );
}

#[test]
fn edge_mod_on_circular_bite_body_clears_pristine_mesh() {
    let kind = crate::sketch::CornerKind::Chamfer;
    let g = circular_bite_graph(Some(kind));
    let (live, warnings) = g
        .build_live(&std::collections::HashSet::new(), false)
        .unwrap();
    assert!(
        warnings.is_empty(),
        "{kind:?} on circular-bite body should not warn, got {warnings:?}"
    );
    assert_eq!(live.len(), 1, "{kind:?} keeps one live body");
    assert!(
        live[0].pristine.is_none(),
        "{kind:?} must clear the pristine sketch display mesh after modifying the B-Rep"
    );

    let bodies = tessellate_bodies(live);
    assert_eq!(bodies.len(), 1, "{kind:?} keeps one rendered body");
    let mesh = &bodies[0].1;
    let (cracks, _nonmanifold, _inward) = mesh_stats(mesh);
    assert_eq!(
        cracks, 0,
        "{kind:?} circular-bite edge-mod mesh has {cracks} cracks"
    );
    let max_z = mesh
        .vertices
        .chunks(6)
        .map(|v| v[2])
        .fold(f32::MIN, f32::max);
    assert!(
        max_z > 9.0,
        "{kind:?} should preserve most of the 10mm body height"
    );
}

#[test]
fn edge_mod_on_circular_bite_cutoff_edge_succeeds() {
    let kind = crate::sketch::CornerKind::Chamfer;
    let edge = circular_bite_cutoff_edge();
    let unmodified = circular_bite_graph(None)
        .evaluate_bodies(&std::collections::HashSet::new())
        .unwrap();
    assert!(
        mesh_has_wire_edge_between(&unmodified[0].1, edge.p0, edge.p1, 0.08),
        "test setup should expose the original sharp edge before {kind:?}"
    );

    let g = circular_bite_cutoff_edge_graph(kind);
    let (live, warnings) = g
        .build_live(&std::collections::HashSet::new(), false)
        .unwrap();
    assert!(
        warnings.is_empty(),
        "{kind:?} on edge ending at circular bite should not warn, got {warnings:?}"
    );
    assert_eq!(live.len(), 1, "{kind:?} keeps one live body");
    assert!(
        live[0].pristine.is_none(),
        "{kind:?} must clear pristine after modifying the B-Rep"
    );

    let bodies = tessellate_bodies(live);
    assert_eq!(bodies.len(), 1, "{kind:?} keeps one rendered body");
    let mesh = &bodies[0].1;
    assert_eq!(
        circular_bite_ghost_sample_count(mesh),
        0,
        "{kind:?} cutoff-edge result should not show a ghost cylinder/disk"
    );
    assert!(
        !mesh_has_wire_edge_between(mesh, edge.p0, edge.p1, 0.08),
        "{kind:?} should remove the original sharp cutoff edge"
    );
    assert_selected_blend_surface(mesh, &edge, 1.0, kind, "cutoff-edge edge-mod");
    let (cracks, _nonmanifold, _inward) = mesh_stats(mesh);
    assert_eq!(cracks, 0, "{kind:?} cutoff-edge result has {cracks} cracks");
}

#[test]
fn fillet_3mm_on_gui_captured_circular_bite_cutoff_edge_succeeds() {
    let kind = crate::sketch::CornerKind::Fillet;
    let edge = gui_captured_circular_bite_cutoff_edge(10.0);
    let mut reversed = edge.clone();
    std::mem::swap(&mut reversed.p0, &mut reversed.p1);

    for (label, edge) in [("captured", edge), ("reversed", reversed)] {
        assert_circular_bite_fillet_3mm_committed(label, edge, kind);
    }
}

fn assert_circular_bite_fillet_3mm_committed(
    label: &str,
    edge: EdgeRef,
    kind: crate::sketch::CornerKind,
) {
    let base_graph = circular_bite_graph(None);
    let unmodified = base_graph
        .evaluate_bodies(&std::collections::HashSet::new())
        .unwrap();
    assert!(
        mesh_has_wire_edge_between(&unmodified[0].1, edge.p0, edge.p1, 0.08),
        "{label}: test setup should expose the original sharp edge before {kind:?}"
    );
    let (base_live, base_warnings) = base_graph
        .build_live(&std::collections::HashSet::new(), false)
        .unwrap();
    assert!(
        base_warnings.is_empty(),
        "{label}: setup should be clean, got {base_warnings:?}"
    );
    let region = base_live[0]
        .sketch_source
        .as_ref()
        .and_then(|source| source.regions.first())
        .cloned()
        .expect("circular-bite setup should retain sketch region provenance");

    let g = circular_bite_cutoff_edge_graph_at_depth_with_edge(kind, 3.0, 10.0, edge.clone());
    let (live, warnings) = g
        .build_live(&std::collections::HashSet::new(), false)
        .unwrap();
    assert!(
        warnings.is_empty(),
        "{label}: {kind:?} on edge ending at circular bite should not warn, got {warnings:?}"
    );
    assert_eq!(live.len(), 1, "{label}: {kind:?} keeps one live body");
    assert!(
        live[0].edge_mod_cut_history_path_used,
        "{label}: {kind:?} must use the guarded cut-history path for the circular-bite sketch case"
    );
    assert!(
        live[0].pristine.is_none(),
        "{label}: {kind:?} must clear pristine after modifying the B-Rep"
    );

    let has_cut_cylinder = live[0].parts.iter().any(|solid| {
        solid.shell().faces().iter().any(|f| {
            matches!(
                f.surface(),
                Some(openrcad::geom::GeomSurface::Cylinder(c))
                    if c.position().direction().dot(&openrcad::foundation::Dir::dz()).abs()
                        > 0.999
                        && (c.radius() - 14.0).abs() < 1.0e-3
            )
        })
    });
    assert!(
        has_cut_cylinder,
        "{label}: {kind:?} should keep the circular bite as an analytic cylinder"
    );
    let has_blend_cylinder = live[0].parts.iter().any(|solid| {
        solid.shell().faces().iter().any(|f| {
            matches!(
                f.surface(),
                Some(openrcad::geom::GeomSurface::Cylinder(c))
                    if (c.radius() - 3.0).abs() < 1.0e-3
            )
        })
    });
    assert!(
        has_blend_cylinder,
        "{label}: {kind:?} should produce a true radius-3 blend cylinder"
    );

    let bodies = tessellate_bodies(live);
    assert_eq!(bodies.len(), 1, "{label}: {kind:?} keeps one rendered body");
    let mesh = &bodies[0].1;
    assert_eq!(
        edge_mod_circular_bite_void_ghost_sample_count(&region, mesh),
        0,
        "{label}: production circular-bite void validator should find no ghost material"
    );
    assert_eq!(
        circular_bite_ghost_sample_count(mesh),
        0,
        "{label}: {kind:?} cutoff-edge result should not show a ghost cylinder/disk"
    );
    assert!(
        !mesh_has_wire_edge_between(mesh, edge.p0, edge.p1, 0.08),
        "{label}: {kind:?} should remove the original sharp cutoff edge"
    );
    assert_selected_blend_surface(
        mesh,
        let edge = gui_captured_circular_bite_cutoff_edge(depth);
        let g = circular_bite_graph_with_depth(None, depth);
        let (live, warnings) = g
            .build_live(&std::collections::HashSet::new(), false)
            .unwrap();
        assert!(warnings.is_empty(), "setup should be clean: {warnings:?}");
        let resolved = resolve_edge_ref_by_topology(&live[0], &edge)
            .expect("captured topology should resolve on the same body");
        assert!(
            same_edge_span(&resolved, &edge, 0.08),
            "depth {depth:.2} topology resolved to {:?}, not captured {:?}",
            resolved,
            edge
        );
    }
}

#[test]
fn replay_fillet_preserves_explicit_cylinder_cut_bite() {
    assert_replay_fillet_preserves_explicit_cylinder_cut_bite(-12.0, "negative depth");
}

#[test]
fn replay_fillet_preserves_explicit_cylinder_cut_bite_positive_depth() {
    assert_replay_fillet_preserves_explicit_cylinder_cut_bite(12.0, "positive depth");
}

fn assert_replay_fillet_preserves_explicit_cylinder_cut_bite(depth: f32, label: &str) {
    let edge = explicit_cylinder_cutoff_edge();
    let mut g = box_with_explicit_cylinder_cut_depth(depth);
    let (base_live, base_warnings) = g
        .build_live(&std::collections::HashSet::new(), false)
        .unwrap();
    assert!(
        base_warnings.is_empty(),
        "{label}: explicit cylinder-cut setup should be clean, got {base_warnings:?}"
    );
    assert!(
        base_live[0].cut_replay.is_some(),
        "{label}: cut body should carry replayable construction history"
    );
    let base_mesh = tessellate_bodies(base_live)[0].1.clone();
    assert!(
        mesh_has_wire_edge_between(&base_mesh, edge.p0, edge.p1, 0.08),
        "{label}: setup should expose the straight cutoff edge before replay fillet"
    );

    add_replay_fillet(&mut g, "cut_3", "fillet_4", edge.clone(), 3.0);
    let (live, warnings) = g
        .build_live(&std::collections::HashSet::new(), false)
        .unwrap();
    assert!(
        warnings.is_empty(),
        "{label}: replay fillet on explicit cylinder cut should not warn, got {warnings:?}"
    );
    assert_eq!(live.len(), 1, "{label}: replay fillet keeps one body");
    assert!(
        live[0].edge_mod_cut_history_path_used,
        "{label}: explicit cylinder-cut fillet must use the guarded cut-history path"
    );
    assert!(
        live_has_cylinder_radius(&live, 14.0),
        "{label}: replayed cut should keep the radius-14 analytic cylinder"
    );
    assert!(
        live_has_cylinder_radius(&live, 3.0),
        "{label}: replay fillet should create a radius-3 analytic blend"
    );

    let mesh = &tessellate_bodies(live)[0].1;
    assert_eq!(
        explicit_circle_bite_ghost_sample_count(mesh),
        0,
        "{label}: replay fillet must not refill the explicit circular bite"
    );
    assert!(
        !mesh_has_wire_edge_between(mesh, edge.p0, edge.p1, 0.08),
        "{label}: replay fillet should remove the selected sharp edge"
    );
    assert_selected_blend_surface(
        mesh,
        &edge,
        3.0,
        crate::sketch::CornerKind::Fillet,
        "explicit cylinder-cut replay fillet",
    );
    let (cracks, _nonmanifold, _inward) = mesh_stats(mesh);
    assert_eq!(
        cracks, 0,
        "{label}: explicit cylinder replay mesh has {cracks} cracks"
    );
}

#[test]
fn replay_fillet_preserves_rectangular_pocket_cut() {
    assert_replay_fillet_preserves_rectangular_pocket_cut(-12.0, "negative depth");
}

#[test]
fn replay_fillet_preserves_rectangular_pocket_cut_positive_depth() {
    assert_replay_fillet_preserves_rectangular_pocket_cut(12.0, "positive depth");
}

fn assert_replay_fillet_preserves_rectangular_pocket_cut(depth: f32, label: &str) {
    let edge = rectangular_pocket_cutoff_edge();
    let mut g = box_with_rectangular_pocket_cut_depth(depth);
    add_replay_fillet(&mut g, "cut_3", "fillet_4", edge.clone(), 2.0);

    let (live, warnings) = g
        .build_live(&std::collections::HashSet::new(), false)
        .unwrap();
    assert!(
        warnings.is_empty(),
        "{label}: replay fillet on rectangular pocket should not warn, got {warnings:?}"
    );
    assert!(
        live[0].edge_mod_cut_history_path_used,
        "{label}: rectangular pocket fillet must use the guarded cut-history path"
    );
    let mesh = &tessellate_bodies(live)[0].1;
    assert_eq!(
        mesh_sample_count_in_box(mesh, [14.0, 2.0, 1.0], [26.0, 10.0, 9.0]),
        0,
        "{label}: replay fillet must not refill the rectangular pocket void"
    );
    assert!(
        !mesh_has_wire_edge_between(mesh, edge.p0, edge.p1, 0.08),
        "{label}: replay fillet should remove the rectangular-pocket selected edge"
    );
    assert_selected_blend_surface(
        mesh,
        &edge,
        2.0,
        crate::sketch::CornerKind::Fillet,
        "rectangular pocket replay fillet",
    );
    let (cracks, _nonmanifold, _inward) = mesh_stats(mesh);
    assert_eq!(
        cracks, 0,
        "{label}: rectangular pocket replay mesh has {cracks} cracks"
    );
}

#[test]
fn replay_fillet_preserves_later_cuts() {
    let edge = explicit_cylinder_cutoff_edge();
    let mut g = box_with_explicit_cylinder_cut();
    add_later_rectangular_pocket_cut(&mut g, "sketch_4", "cut_5");

    let replay =
        g.edge_mod_replay_intent_for_edge("box_1", &edge, &std::collections::HashSet::new());
    assert_eq!(
        replay.replay_cut_nodes,
        vec!["cut_3".to_string(), "cut_5".to_string()],
        "replay intent should capture the full ordered cut chain"
    );
    add_replay_fillet_with_intent(&mut g, "cut_5", "fillet_6", edge.clone(), 3.0, replay);

    let (live, warnings) = g
        .build_live(&std::collections::HashSet::new(), false)
        .unwrap();
    assert!(
        warnings.is_empty(),
        "multi-cut replay fillet should not warn, got {warnings:?}"
    );
    assert!(
        live[0].edge_mod_cut_history_path_used,
        "multi-cut fillet must use the guarded cut-history path"
    );
    let mesh = &tessellate_bodies(live)[0].1;
    assert_eq!(
        explicit_circle_bite_ghost_sample_count(mesh),
        0,
        "multi-cut replay must not refill the circular bite"
    );
    assert_eq!(
        mesh_sample_count_in_box(mesh, [31.0, 19.0, 1.0], [35.0, 25.0, 9.0]),
        0,
        "multi-cut replay must preserve the later rectangular cut"
    );
}

fn explicit_cylinder_cutoff_edge() -> EdgeRef {
    let x = 20.0 - (14.0_f32 * 14.0 - 3.0_f32 * 3.0).sqrt();
    EdgeRef {
        p0: [0.0, 0.0, 10.0],
        p1: [x, 0.0, 10.0],
        n1: [0.0, 0.0, 1.0],
        n2: [0.0, -1.0, 0.0],
        curve: None,
        topology: None,
    }
}

fn rectangular_pocket_cutoff_edge() -> EdgeRef {
    EdgeRef {
        p0: [0.0, 0.0, 10.0],
        p1: [12.0, 0.0, 10.0],
        n1: [0.0, 0.0, 1.0],
        n2: [0.0, -1.0, 0.0],
        curve: None,
        topology: None,
    }
}

fn box_with_explicit_cylinder_cut() -> ParametricGraph {
    box_with_explicit_cylinder_cut_depth(-12.0)
}

fn box_with_explicit_cylinder_cut_depth(depth: f32) -> ParametricGraph {
    let mut g = box_40x30x10();
    let mut circle = SketchCurves::new();
    circle.add_circle((20.0, 3.0), 14.0);
    add_sketch_cs(&mut g, "sketch_2", top_face_10(), circle);
    add_extrude(&mut g, "cut_3", "sketch_2", depth, ExtrudeMode::Cut);
    g.add_dependency("box_1", "cut_3");
    g
}

fn box_with_rectangular_pocket_cut_depth(depth: f32) -> ParametricGraph {
    let mut g = box_40x30x10();
    add_rectangular_pocket_cut_depth(&mut g, "sketch_2", "cut_3", depth);
    g
}

fn box_40x30x10() -> ParametricGraph {
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "box_1".to_string(),
        name: "Box".to_string(),
        feature: FeatureType::Box {
            w: 40.0,
            h: 30.0,
            d: 10.0,
        },
    });
    g
}

fn add_rectangular_pocket_cut_depth(
    g: &mut ParametricGraph,
    sketch_id: &str,
    cut_id: &str,
    depth: f32,
) {
    add_sketch_cs(
        g,
        sketch_id,
        top_face_10(),
        rect_sketch((12.0, -5.0), (28.0, 12.0)),
    );
    add_extrude(g, cut_id, sketch_id, depth, ExtrudeMode::Cut);
    g.add_dependency("box_1", cut_id);
}

fn add_later_rectangular_pocket_cut(g: &mut ParametricGraph, sketch_id: &str, cut_id: &str) {
    add_sketch_cs(
        g,
        sketch_id,
        top_face_10(),
        rect_sketch((30.0, 18.0), (36.0, 26.0)),
    );
    add_extrude(g, cut_id, sketch_id, -12.0, ExtrudeMode::Cut);
    g.add_dependency("box_1", cut_id);
}

fn top_face_10() -> CoordinateSystem {
    CoordinateSystem::new(Vec3::new(0.0, 0.0, 10.0), Vec3::X, Vec3::Y)
}

fn add_replay_fillet(
    g: &mut ParametricGraph,
    dependency: &str,
    id: &str,
    edge: EdgeRef,
    radius: f32,
) {
    let replay =
        g.edge_mod_replay_intent_for_edge("box_1", &edge, &std::collections::HashSet::new());
    add_replay_fillet_with_intent(g, dependency, id, edge, radius, replay);
}

fn add_replay_fillet_with_intent(
    g: &mut ParametricGraph,
    dependency: &str,
    id: &str,
    edge: EdgeRef,
    radius: f32,
    replay: EdgeModReplayIntent,
) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: "Replay Fillet".to_string(),
        feature: FeatureType::EdgeMod {
            target: "box_1".to_string(),
            edge,
            dist: radius,
            dist_expr: None,
            scope: EdgeModScope::FullEdge,
            replay,
            kind: crate::sketch::CornerKind::Fillet,
        },
    });
    g.add_dependency(dependency, id);
}

fn live_has_cylinder_radius(live: &[LiveBody], radius: f64) -> bool {
    live.iter().any(|body| {
        body.parts.iter().any(|solid| {
            solid.shell().faces().iter().any(|face| {
                matches!(
                    face.surface(),
                    Some(openrcad::geom::GeomSurface::Cylinder(c))
                        if (c.radius() - radius).abs() < 1.0e-3
                )
            })
        })
    })
}

fn mesh_sample_count_in_box(mesh: &MockMesh, lo: [f32; 3], hi: [f32; 3]) -> usize {
    let inside = |p: [f32; 3]| (0..3).all(|i| p[i] >= lo[i] && p[i] <= hi[i]);
    let vertex_pos = |vi: u32| {
        let b = vi as usize * 6;
        [mesh.vertices[b], mesh.vertices[b + 1], mesh.vertices[b + 2]]
    };

    let mut count = mesh
        .vertices
        .chunks_exact(6)
        .filter(|v| inside([v[0], v[1], v[2]]))
        .count();
    for tri in mesh.indices.chunks_exact(3) {
        let a = vertex_pos(tri[0]);
        let b = vertex_pos(tri[1]);
        let c = vertex_pos(tri[2]);
        if inside([
            (a[0] + b[0] + c[0]) / 3.0,
            (a[1] + b[1] + c[1]) / 3.0,
            (a[2] + b[2] + c[2]) / 3.0,
        ]) {
            count += 1;
        }
    }
    count
}

fn explicit_circle_bite_ghost_sample_count(mesh: &MockMesh) -> usize {
    let vertex6 = |vi: u32| {
        let b = vi as usize * 6;
        [
            mesh.vertices[b],
            mesh.vertices[b + 1],
            mesh.vertices[b + 2],
            mesh.vertices[b + 3],
            mesh.vertices[b + 4],
            mesh.vertices[b + 5],
        ]
    };
    let mut count = 0usize;
    for v in mesh.vertices.chunks_exact(6) {
        if explicit_circle_bite_non_wall_sample([v[0], v[1], v[2]], [v[3], v[4], v[5]]) {
            count += 1;
        }
    }
    for tri in mesh.indices.chunks_exact(3) {
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
        if explicit_circle_bite_non_wall_sample(p, n) {
            count += 1;
        }
    }
    count
}

fn explicit_circle_bite_non_wall_sample(p: [f32; 3], n: [f32; 3]) -> bool {
    let radial = [p[0] - 20.0, p[1] - 3.0];
    let r = (radial[0] * radial[0] + radial[1] * radial[1]).sqrt();
    if r >= 13.0 || p[1] <= 0.1 || !(-0.05..=10.05).contains(&p[2]) {
        return false;
    }
    if !(12.0..=14.25).contains(&r) || n[2].abs() > 0.55 {
        return true;
    }
    let nl = (n[0] * n[0] + n[1] * n[1]).sqrt();
    if nl <= 1.0e-5 || r <= 1.0e-5 {
        return true;
    }
    let dot = (radial[0] / r) * (n[0] / nl) + (radial[1] / r) * (n[1] / nl);
    dot.abs() <= 0.75
}
