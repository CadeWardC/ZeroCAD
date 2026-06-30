use std::collections::{HashMap, HashSet};

use openrcad::foundation::Dir;
use openrcad::geom::GeomSurface;
use openrcad::topo::Solid;
use zerocad_core::{
    CoordinateSystem, CornerKind, EdgeModReplayIntent, EdgeModScope, EdgeRef, ExtrudeMode,
    FeatureNode, FeatureType, MockMesh, ParametricGraph, SketchCurves, Vec3,
};

const BOX_W: f32 = 40.0;
const BOX_H: f32 = 30.0;
const BOX_D: f32 = 15.0;
const FILLET_R: f64 = 3.0;
const CUT_X: f32 = 20.0;
const CUT_Y: f32 = 2.0;
const CUT_R: f64 = 5.0;

fn add_box(g: &mut ParametricGraph) {
    g.add_feature(FeatureNode {
        id: "box_1".into(),
        name: "Box".into(),
        feature: FeatureType::Box {
            w: BOX_W,
            h: BOX_H,
            d: BOX_D,
        },
    });
}

fn add_front_top_fillet(g: &mut ParametricGraph) {
    g.add_feature(FeatureNode {
        id: "edgemod_2".into(),
        name: "Fillet".into(),
        feature: FeatureType::EdgeMod {
            target: "box_1".into(),
            edge: EdgeRef {
                p0: [0.0, 0.0, BOX_D],
                p1: [BOX_W, 0.0, BOX_D],
                n1: [0.0, 0.0, 1.0],
                n2: [0.0, -1.0, 0.0],
                curve: None,
                topology: None,
            },
            dist: FILLET_R as f32,
            dist_expr: None,
            scope: EdgeModScope::FullEdge,
            replay: EdgeModReplayIntent::default(),
            kind: CornerKind::Fillet,
        },
    });
    g.add_dependency("box_1", "edgemod_2");
}

fn add_circle_cut(g: &mut ParametricGraph, depth: f32) {
    let mut curves = SketchCurves::new();
    curves.add_circle((CUT_X, CUT_Y), CUT_R as f32);
    g.add_feature(FeatureNode {
        id: "sketch_3".into(),
        name: "Circle Cut Sketch".into(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::new(
                Vec3::new(0.0, 0.0, BOX_D),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            ),
            curves,
            shapes: vec![],
            corner_mods: vec![],
            on_face: true,
        },
    });
    g.add_feature(FeatureNode {
        id: "extrude_4".into(),
        name: "Circle Cut".into(),
        feature: FeatureType::Extrude {
            depth,
            region_indices: vec![],
            mode: ExtrudeMode::Cut,
            depth_expr: None,
        },
    });
    g.add_dependency("sketch_3", "extrude_4");
    g.add_dependency("edgemod_2", "extrude_4");
}

fn graph(depth: f32) -> ParametricGraph {
    let mut g = ParametricGraph::new();
    add_box(&mut g);
    add_front_top_fillet(&mut g);
    add_circle_cut(&mut g, depth);
    g
}

fn has_cylinder_radius(solid: &Solid, radius: f64) -> bool {
    solid.shell().faces().iter().any(|face| {
        matches!(
            face.surface(),
            Some(GeomSurface::Cylinder(cyl)) if (cyl.radius() - radius).abs() < 1.0e-3
        )
    })
}

fn has_vertical_cut_wall(solid: &Solid) -> bool {
    solid.shell().faces().iter().any(|face| {
        matches!(
            face.surface(),
            Some(GeomSurface::Cylinder(cyl))
                if (cyl.radius() - CUT_R).abs() < 1.0e-3
                    && cyl.position().direction().dot(&Dir::dz()).abs() > 0.999
        )
    })
}

fn cracks(mesh: &MockMesh) -> usize {
    type Key = (i64, i64, i64);
    let q = |i: usize| -> Key {
        let base = i * 6;
        let g = |v: f32| (v as f64 * 1.0e4).round() as i64;
        (
            g(mesh.vertices[base]),
            g(mesh.vertices[base + 1]),
            g(mesh.vertices[base + 2]),
        )
    };
    let mut edges: HashMap<(Key, Key), u32> = HashMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            let (ka, kb) = (q(tri[a] as usize), q(tri[b] as usize));
            let key = if ka <= kb { (ka, kb) } else { (kb, ka) };
            *edges.entry(key).or_insert(0) += 1;
        }
    }
    edges.values().filter(|&&count| count == 1).count()
}

fn non_wall_samples_in_removed_volume(mesh: &MockMesh, solid: &Solid) -> usize {
    let faces = solid.shell().faces();
    let inside_void = |p: [f32; 3]| {
        let r = ((p[0] - CUT_X).powi(2) + (p[1] - CUT_Y).powi(2)).sqrt();
        r < CUT_R as f32 - 0.6 && (-0.05..=BOX_D + 0.05).contains(&p[2])
    };
    let cut_wall = |face_id: u32| {
        matches!(
            faces.get(face_id as usize).and_then(|face| face.surface()),
            Some(GeomSurface::Cylinder(cyl))
                if (cyl.radius() - CUT_R).abs() < 1.0e-3
                    && cyl.position().direction().dot(&Dir::dz()).abs() > 0.999
        )
    };

    let mut count = 0usize;
    for (tri_idx, tri) in mesh.indices.chunks_exact(3).enumerate() {
        if cut_wall(mesh.face_ids.get(tri_idx).copied().unwrap_or(u32::MAX)) {
            continue;
        }
        let mut p = [0.0_f32; 3];
        for &idx in tri {
            let base = idx as usize * 6;
            p[0] += mesh.vertices[base] / 3.0;
            p[1] += mesh.vertices[base + 1] / 3.0;
            p[2] += mesh.vertices[base + 2] / 3.0;
        }
        if inside_void(p) {
            count += 1;
        }
    }
    count
}

fn assert_fillet_then_cut(depth: f32) {
    let g = graph(depth);
    let hidden = HashSet::new();
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&hidden).unwrap();
    assert!(
        warnings.is_empty(),
        "fillet-then-cut depth {depth} must not warn: {warnings:?}"
    );
    assert_eq!(
        bodies.len(),
        1,
        "fillet-then-cut depth {depth} keeps one body"
    );

    let solids = g.debug_kernel_solids(&hidden).unwrap();
    assert_eq!(solids.len(), 1, "depth {depth}: one debug body");
    assert_eq!(solids[0].1.len(), 1, "depth {depth}: one kernel solid part");
    let solid = &solids[0].1[0];
    assert!(
        solid.is_watertight(),
        "depth {depth}: kernel solid is watertight"
    );
    assert!(
        solid.health_report().is_healthy(),
        "depth {depth}: kernel solid is healthy: {:?}",
        solid.health_report().errors
    );
    assert!(
        has_vertical_cut_wall(solid),
        "depth {depth}: circular cut wall remains analytic"
    );
    assert!(
        has_cylinder_radius(solid, FILLET_R),
        "depth {depth}: remaining fillet cylinder survives"
    );

    let mesh = &bodies[0].1;
    assert_eq!(cracks(mesh), 0, "depth {depth}: render mesh has no cracks");
    assert_eq!(
        non_wall_samples_in_removed_volume(mesh, solid),
        0,
        "depth {depth}: no non-wall mesh samples remain inside the cylinder void"
    );
}

#[test]
fn filleted_box_circle_cut_straddling_fillet_succeeds() {
    assert_fillet_then_cut(-20.0);
    assert_fillet_then_cut(20.0);
}
