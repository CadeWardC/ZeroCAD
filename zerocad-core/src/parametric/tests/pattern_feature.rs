//! Pattern & Mirror as parametric features: replicate a body node's solids by
//! linear/circular arrays or a mirror plane.

use super::*;
use crate::parametric::{AxisBase, PatternKind, PlaneBase};

fn add_box(g: &mut ParametricGraph, id: &str, w: f32, h: f32, d: f32) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Box { w, h, d },
    });
}

fn add_pattern(g: &mut ParametricGraph, id: &str, source: &str, kind: PatternKind) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Pattern {
            source: source.to_string(),
            kind,
        },
    });
    g.add_dependency(source, id);
}

fn total_volume(bodies: &[(String, MockMesh)]) -> f64 {
    bodies
        .iter()
        .filter_map(|(_, m)| m.mass_properties())
        .map(|mp| mp.volume)
        .sum()
}

#[test]
fn linear_pattern_replicates_box() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 2.0, 3.0, 4.0);
    add_pattern(
        &mut g,
        "pattern_2",
        "box_1",
        PatternKind::Linear {
            dir: AxisBase::X,
            spacing: 10.0,
            spacing_expr: None,
            count: 4,
        },
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    // Source body + one pattern body holding the 3 extra instances.
    assert_eq!(bodies.len(), 2);
    let v = total_volume(&bodies);
    assert!((v - 4.0 * 24.0).abs() < 1e-3, "total volume {v}");
}

#[test]
fn circular_pattern_ring_spacing() {
    // 4 instances through a full 360° about Y: instances at 0/90/180/270 —
    // step must be 360/count so nothing lands back on instance 0.
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 1.0, 1.0, 1.0);
    add_pattern(
        &mut g,
        "pattern_2",
        "box_1",
        PatternKind::Circular {
            axis: AxisBase::Y,
            count: 4,
            total_angle_deg: 360.0,
        },
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    let v = total_volume(&bodies);
    assert!((v - 4.0).abs() < 1e-3, "total volume {v}");
    // The pattern body's mesh must span negative X and Z (ring around origin),
    // not stack on the original.
    let mesh = &bodies.iter().find(|(id, _)| id == "pattern_2").unwrap().1;
    let (mut min_x, mut min_z) = (f32::MAX, f32::MAX);
    for vtx in mesh.vertices.chunks(6) {
        min_x = min_x.min(vtx[0]);
        min_z = min_z.min(vtx[2]);
    }
    assert!(min_x < -0.5, "ring should reach -X, min_x {min_x}");
    assert!(min_z < -0.5, "ring should reach -Z, min_z {min_z}");
}

#[test]
fn mirror_produces_well_oriented_copy() {
    // Box at [0,2]³ mirrored across YZ → copy in x∈[-2,0]. The reflection
    // flips handedness; the re-sew must leave the copy watertight with
    // positive volume and outward normals (mass_properties would go wrong on
    // an inside-out mesh).
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 2.0, 2.0, 2.0);
    add_pattern(
        &mut g,
        "pattern_2",
        "box_1",
        PatternKind::Mirror {
            plane: PlaneBase::YZ,
        },
    );
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert!(warnings.is_empty(), "warnings: {warnings:?}");
    assert_eq!(bodies.len(), 2);
    let mirrored = &bodies.iter().find(|(id, _)| id == "pattern_2").unwrap().1;
    let mp = mirrored.mass_properties().expect("closed mirrored mesh");
    assert!((mp.volume - 8.0).abs() < 1e-3, "mirror volume {}", mp.volume);
    assert!(
        (mp.centroid[0] + 1.0).abs() < 1e-3,
        "mirror centroid x {}",
        mp.centroid[0]
    );
}

#[test]
fn missing_source_warns() {
    let mut g = ParametricGraph::new();
    add_box(&mut g, "box_1", 1.0, 1.0, 1.0);
    g.add_feature(FeatureNode {
        id: "pattern_2".to_string(),
        name: "pattern_2".to_string(),
        feature: FeatureType::Pattern {
            source: "nonexistent".to_string(),
            kind: PatternKind::Linear {
                dir: AxisBase::X,
                spacing: 5.0,
                spacing_expr: None,
                count: 2,
            },
        },
    });
    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert_eq!(bodies.len(), 1); // just the box
    assert!(
        warnings.iter().any(|w| w.contains("pattern_2")),
        "warnings: {warnings:?}"
    );
}
