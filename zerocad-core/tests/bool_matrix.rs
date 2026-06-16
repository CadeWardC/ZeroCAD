use std::collections::HashSet;
use zerocad_core::{
    CoordinateSystem, ExtrudeMode, FeatureNode, FeatureType, ParametricGraph, SketchCurves, Vec3,
};

fn circ(c: &mut SketchCurves, cx: f32, cy: f32, r: f32) {
    c.add_circle((cx, cy), r);
}
fn rect(min: (f32, f32), max: (f32, f32)) -> SketchCurves {
    let mut c = SketchCurves::new();
    c.add_rectangle(min, max);
    c
}
fn sk(g: &mut ParametricGraph, id: &str, cs: CoordinateSystem, c: SketchCurves) {
    g.add_feature(FeatureNode {
        id: id.into(),
        name: id.into(),
        feature: FeatureType::Sketch {
            cs,
            curves: c,
            shapes: vec![],
            corner_mods: vec![],
            on_face: false,
        },
    });
}
fn ex(g: &mut ParametricGraph, id: &str, s: &str, d: f32, m: ExtrudeMode) {
    g.add_feature(FeatureNode {
        id: id.into(),
        name: id.into(),
        feature: FeatureType::Extrude {
            depth: d,
            region_indices: vec![],
            mode: m,
            depth_expr: None,
        },
    });
    g.add_dependency(s, id);
}
fn topcs(h: f32) -> CoordinateSystem {
    CoordinateSystem::new(
        Vec3::new(0.0, 0.0, h),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
}

fn tris(g: &ParametricGraph) -> Vec<(String, usize)> {
    g.evaluate_bodies(&HashSet::new())
        .unwrap()
        .into_iter()
        .map(|(i, m)| (i, m.indices.len() / 3))
        .collect()
}

#[test]
fn blind_circle_cut_on_top_face() {
    let mut g = ParametricGraph::new();
    sk(
        &mut g,
        "sketch_1",
        CoordinateSystem::XY,
        rect((0.0, 0.0), (10.0, 10.0)),
    );
    ex(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);
    let before = tris(&g);
    println!("box: {:?}", before);
    // circle on the top face (z=10), blind cut 4 deep
    let mut c = SketchCurves::new();
    circ(&mut c, 5.0, 5.0, 2.5);
    sk(&mut g, "sketch_3", topcs(10.0), c);
    ex(&mut g, "extrude_4", "sketch_3", 4.0, ExtrudeMode::Cut);
    let after = tris(&g);
    println!("after blind circle cut: {:?}", after);
    assert_eq!(after.len(), 1, "cut should keep one body");
    assert_ne!(
        after[0].1, before[0].1,
        "blind circular cut should change the body"
    );
}
