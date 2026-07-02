//! Sketch corner fillets must extrude to *analytic* cylindrical walls — the same
//! geometry a 3D edge fillet produces — so "rounded profile, extruded" equals
//! "extrude then fillet the edges".
//!
//! Both single- and multi-corner fillets now extrude to exact analytic cylinders.
//! A single fillet always refit via `loop_to_wire`; multi-corner profiles (rounded
//! rectangles) used to FACET because `loop_to_wire`'s sample-based arc detection
//! only separates runs at *sharp* corners and a rounded rectangle's arc↔straight
//! junctions are tangent-continuous. Phase 2 fixed that by carrying the sketch's
//! exact fillet arcs (center+radius) to `loop_to_wire_with_arcs`, which classifies
//! each boundary point by its known circle and sweeps exact arc edges.

use std::collections::HashSet;

use openrcad::geom::GeomSurface;
use zerocad_core::{
    CoordinateSystem, CornerKind, CornerMod, Dimension, ExtrudeMode, FeatureNode, FeatureType,
    ParametricGraph, SketchCurves, Vec3,
};

/// Extrude a 20×12 rectangle with `radius` fillets at the given corners; return
/// (face count, cylinder-face radii, watertight, healthy).
fn filleted_rect(radius: f64, corners: &[(f32, f32)]) -> (usize, Vec<f64>, bool, bool) {
    let mut g = ParametricGraph::new();
    let mut curves = SketchCurves::new();
    curves.add_rectangle((0.0, 0.0), (20.0, 12.0));
    let corner_mods = corners
        .iter()
        .map(|&at| CornerMod {
            at,
            radius: Dimension::literal(radius as f32),
            kind: CornerKind::Fillet,
        })
        .collect();
    g.add_feature(FeatureNode {
        id: "sketch_1".into(),
        name: "S".into(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::new(
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            ),
            curves,
            shapes: vec![],
            corner_mods,
            on_face: false,
        },
    });
    g.add_feature(FeatureNode {
        id: "extrude_2".into(),
        name: "E".into(),
        feature: FeatureType::Extrude {
            depth: 10.0,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
        },
    });
    g.add_dependency("sketch_1", "extrude_2");

    let solids = g.debug_kernel_solids(&HashSet::new()).unwrap();
    let solid = &solids[0].1[0];
    let cyl: Vec<f64> = solid
        .shell()
        .faces()
        .iter()
        .filter_map(|f| match f.surface() {
            Some(GeomSurface::Cylinder(c)) => Some((c.radius() * 1000.0).round() / 1000.0),
            _ => None,
        })
        .collect();
    (
        solid.shell().faces().len(),
        cyl,
        solid.is_watertight(),
        solid.health_report().is_healthy(),
    )
}

#[test]
fn single_corner_fillet_is_analytic() {
    for &r in &[0.5f64, 1.0, 3.0, 6.0] {
        let (_faces, cyls, wt, healthy) = filleted_rect(r, &[(0.0, 0.0)]);
        assert!(wt && healthy, "r={r}: filleted-rect extrude must be watertight+healthy");
        assert!(
            cyls.iter().any(|c| (c - r).abs() < 1.0e-2),
            "r={r}: single sketch fillet must extrude to an analytic cylinder of that radius, got {cyls:?}"
        );
    }
}

#[test]
fn rounded_rectangle_corners_are_analytic() {
    // Radii chosen to fit a 20×12 rectangle without `apply_corner_mod` clamping
    // (two corners share the 12-tall side, so 2r must stay under ~0.95·12).
    let all4 = [(0.0, 0.0), (20.0, 0.0), (20.0, 12.0), (0.0, 12.0)];
    for &r in &[1.0f64, 3.0, 5.0] {
        let (_faces, cyls, wt, healthy) = filleted_rect(r, &all4);
        assert!(wt && healthy, "r={r}: rounded-rect extrude must be watertight+healthy");
        let n = cyls.iter().filter(|c| (**c - r).abs() < 1.0e-2).count();
        assert_eq!(
            n, 4,
            "r={r}: all four rounded corners must extrude to analytic cylinders of radius {r}, got {cyls:?}"
        );
    }
}

/// The DISPLAY mesh must derive from the analytic part too: 10 B-Rep faces
/// (not one face per facet), so the GUI selects a whole fillet wall at once
/// and shades it smooth. This locks in the shared-edge tessellation — before
/// it, per-face boundary sampling disagreed along the fillet tangent seams,
/// the mesh failed the manifold gate, and eval fell back to a 106-face
/// faceted prism (the reported "I can select the flat faces of the fillet").
#[test]
fn rounded_rectangle_display_mesh_is_the_analytic_part() {
    let mut g = ParametricGraph::new();
    let mut curves = SketchCurves::new();
    curves.add_rectangle((0.0, 0.0), (20.0, 12.0));
    let corner_mods = [(0.0f32, 0.0f32), (20.0, 0.0), (20.0, 12.0), (0.0, 12.0)]
        .iter()
        .map(|&at| CornerMod {
            at,
            radius: Dimension::literal(3.0),
            kind: CornerKind::Fillet,
        })
        .collect();
    g.add_feature(FeatureNode {
        id: "sketch_1".into(),
        name: "S".into(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::new(
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            ),
            curves,
            shapes: vec![],
            corner_mods,
            on_face: false,
        },
    });
    g.add_feature(FeatureNode {
        id: "extrude_2".into(),
        name: "E".into(),
        feature: FeatureType::Extrude {
            depth: 10.0,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
        },
    });
    g.add_dependency("sketch_1", "extrude_2");

    let bodies = g.evaluate_bodies(&HashSet::new()).unwrap();
    let mesh = &bodies[0].1;
    let distinct: std::collections::HashSet<u32> = mesh.face_ids.iter().copied().collect();
    assert_eq!(
        distinct.len(),
        10,
        "display mesh must have the part's 10 faces (2 caps + 4 sides + 4 fillet \
         cylinders), not one face per tessellation facet"
    );

    // The render mesh must be a closed manifold (every undirected edge used by
    // exactly two triangles at quantized positions) — the gate that decides
    // whether the display derives from the part.
    let q = |i: usize| {
        let b = i * 6;
        let f = |v: f32| (v as f64 * 1.0e4).round() as i64;
        (f(mesh.vertices[b]), f(mesh.vertices[b + 1]), f(mesh.vertices[b + 2]))
    };
    let mut edges: std::collections::HashMap<_, u32> = std::collections::HashMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            let (ka, kb) = (q(tri[a] as usize), q(tri[b] as usize));
            let key = if ka <= kb { (ka, kb) } else { (kb, ka) };
            *edges.entry(key).or_insert(0) += 1;
        }
    }
    assert!(
        edges.values().all(|&c| c == 2),
        "rounded-rect display mesh must be a closed manifold"
    );
}
