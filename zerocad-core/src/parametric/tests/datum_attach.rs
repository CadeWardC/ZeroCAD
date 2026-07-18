//! Sketch-on-datum: a sketch attached to a datum plane extrudes from the
//! datum's resolved frame, and editing the datum moves the result.

use super::*;

fn graph_with_datum_sketch(offset: f32) -> ParametricGraph {
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "datum_1".to_string(),
        name: "Offset Plane".to_string(),
        feature: FeatureType::DatumPlane {
            def: DatumPlaneDef::Offset {
                base: PlaneBase::XY,
                distance: offset,
                distance_expr: None,
            },
        },
    });
    add_sketch(&mut g, "sketch_2", rect_sketch((0.0, 0.0), (4.0, 4.0)));
    g.sketch_datum_refs
        .insert("sketch_2".into(), "datum_1".into());
    g.add_dependency("datum_1", "sketch_2");
    add_extrude(&mut g, "extrude_3", "sketch_2", 2.0, ExtrudeMode::NewBody);
    g
}

fn z_range(mesh: &MockMesh) -> (f32, f32) {
    let mut lo = f32::MAX;
    let mut hi = f32::MIN;
    for v in mesh.vertices.chunks(6) {
        lo = lo.min(v[2]);
        hi = hi.max(v[2]);
    }
    (lo, hi)
}

#[test]
fn sketch_on_datum_extrudes_from_datum_plane() {
    let g = graph_with_datum_sketch(5.0);
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1);
    let (lo, hi) = z_range(&bodies[0].1);
    assert!((lo - 5.0).abs() < 1e-3, "z range {lo}..{hi}");
    assert!((hi - 7.0).abs() < 1e-3, "z range {lo}..{hi}");
}

#[test]
fn editing_the_datum_moves_dependent_geometry() {
    let mut g = graph_with_datum_sketch(5.0);
    // First build populates the eval cache; then edit ONLY the datum. If the
    // datum weren't folded into the prefix-cache seed, the second build would
    // reuse the stale checkpoint and keep the body at z=5.
    let _ = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    let idx = g.node_map["datum_1"];
    if let FeatureType::DatumPlane {
        def: DatumPlaneDef::Offset { distance, .. },
    } = &mut g.graph[idx].feature
    {
        *distance = 10.0;
    }
    let (bodies, _) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    let (lo, hi) = z_range(&bodies[0].1);
    assert!((lo - 10.0).abs() < 1e-3, "z range {lo}..{hi}");
    assert!((hi - 12.0).abs() < 1e-3, "z range {lo}..{hi}");
}

#[test]
fn missing_datum_falls_back_to_saved_plane_with_warning() {
    let mut g = graph_with_datum_sketch(5.0);
    // Point the sketch at a datum that doesn't exist: the extrude must fall
    // back to the sketch's saved plane (XY) and warn, not vanish.
    g.sketch_datum_refs
        .insert("sketch_2".into(), "datum_99".into());
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert_eq!(bodies.len(), 1);
    assert!(
        warnings.iter().any(|w| w.contains("datum_99")),
        "warnings: {warnings:?}"
    );
    let (lo, hi) = z_range(&bodies[0].1);
    assert!(lo.abs() < 1e-3 && (hi - 2.0).abs() < 1e-3, "z {lo}..{hi}");
}
