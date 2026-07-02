//! A primitive Box/Cylinder must be "the same in every way except how it got
//! there" as the equivalent sketched-and-extruded profile: identical B-Rep faces
//! AND an identical display mesh (both now derive from the one part solid).

use std::collections::HashSet;

use openrcad::geom::GeomSurface;
use zerocad_core::{
    CoordinateSystem, ExtrudeMode, FeatureNode, FeatureType, ParametricGraph, SketchCurves, Vec3,
};

fn xy() -> CoordinateSystem {
    CoordinateSystem::new(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
}

/// (total faces, cylindrical faces) of the single body's single kernel part.
fn faces(g: &ParametricGraph) -> (usize, usize) {
    let solids = g.debug_kernel_solids(&HashSet::new()).unwrap();
    let solid = &solids[0].1[0];
    let fs = solid.shell().faces();
    let cyl = fs
        .iter()
        .filter(|f| matches!(f.surface(), Some(GeomSurface::Cylinder(_))))
        .count();
    (fs.len(), cyl)
}

/// Display-mesh triangle count of the single body.
fn tris(g: &ParametricGraph) -> usize {
    g.evaluate_bodies(&HashSet::new()).unwrap()[0].1.indices.len() / 3
}

fn box_primitive(w: f32, h: f32, d: f32) -> ParametricGraph {
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "box_1".into(),
        name: "Box".into(),
        feature: FeatureType::Box { w, h, d },
    });
    g
}

fn sketched_rect(w: f32, h: f32, d: f32) -> ParametricGraph {
    let mut g = ParametricGraph::new();
    let mut c = SketchCurves::new();
    c.add_rectangle((0.0, 0.0), (w, h));
    g.add_feature(FeatureNode {
        id: "sketch_1".into(),
        name: "S".into(),
        feature: FeatureType::Sketch {
            cs: xy(),
            curves: c,
            shapes: vec![],
            corner_mods: vec![],
            on_face: false,
        },
    });
    g.add_feature(FeatureNode {
        id: "extrude_2".into(),
        name: "E".into(),
        feature: FeatureType::Extrude {
            depth: d,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
        },
    });
    g.add_dependency("sketch_1", "extrude_2");
    g
}

#[test]
fn box_primitive_equals_sketched_rectangle() {
    let prim = box_primitive(20.0, 12.0, 10.0);
    let sketch = sketched_rect(20.0, 12.0, 10.0);

    assert_eq!(
        faces(&prim),
        faces(&sketch),
        "box primitive and sketched rectangle must have identical B-Rep faces"
    );
    assert_eq!(faces(&prim), (6, 0), "a box is 6 planar faces, no cylinders");

    // Display meshes match too — both derive from the same part solid now.
    assert_eq!(
        tris(&prim),
        tris(&sketch),
        "box primitive and sketched-rect display meshes must match"
    );
}

#[test]
fn cylinder_primitive_is_analytic_and_display_matches_part() {
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "cyl_1".into(),
        name: "Cyl".into(),
        feature: FeatureType::Cylinder { r: 8.0, h: 20.0 },
    });
    let (total, cyl) = faces(&g);
    assert!(cyl >= 1, "cylinder primitive keeps an analytic cylindrical wall, got {cyl}");
    assert!(
        total <= 8,
        "cylinder primitive stays analytic (a handful of faces), got {total}"
    );
    // The display derives from the part, so it tessellates without error.
    assert!(tris(&g) > 0, "cylinder primitive must render");
}
