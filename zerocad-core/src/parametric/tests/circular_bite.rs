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
    let unmodified = circular_bite_graph(None)
        .evaluate_bodies(&std::collections::HashSet::new())
        .unwrap();
    assert!(
        mesh_has_wire_edge_between(&unmodified[0].1, edge.p0, edge.p1, 0.08),
        "test setup should expose the original sharp edge before {kind:?}"
    );

    let g = circular_bite_cutoff_edge_graph_at_depth_with_edge(kind, 3.0, 10.0, edge.clone());
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
        "{kind:?} should keep the circular bite as an analytic cylinder"
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
        "{kind:?} should produce a true radius-3 blend cylinder"
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
    assert_selected_blend_surface(mesh, &edge, 3.0, kind, "cutoff-edge edge-mod");
    let (cracks, _nonmanifold, _inward) = mesh_stats(mesh);
    assert_eq!(cracks, 0, "{kind:?} cutoff-edge result has {cracks} cracks");
}

#[test]
fn gui_captured_circular_bite_topology_resolves_to_same_visible_span() {
    for depth in [6.0, 9.2, 10.0] {
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
