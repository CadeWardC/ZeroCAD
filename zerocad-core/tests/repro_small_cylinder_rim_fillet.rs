//! Regression for the reported small-cylinder screenshot: a 3mm fillet (or
//! chamfer) on the top rim of a small EXTRUDED-CIRCLE cylinder (r ≈ 4mm,
//! sketched on the ground plane) was rejected by the circular-bite locality
//! gate with
//!   "candidate removed unselected circular-bite side span ..."
//! The GUI's closed-rim EdgeRef carries p0/p1 ONE TESSELLATION CHORD apart
//! (a closed loop has no free ends, so `edge_ref_from` falls back to the first
//! chord's endpoints). That chord sits at the cap loop's bounding-box extreme,
//! which the straight-side classifier mistook for a rect side — fabricating a
//! one-chord "unselected span" next to the seam and vetoing a good blend.
//! Extruded bodies carry a `sketch_source` (which is what arms that gate);
//! primitive `Cylinder` bodies don't, so the circular_rim_blend tests never
//! hit this. The EdgeRef here is derived from the REAL evaluated mesh the way
//! the GUI derives it, so the repro follows the tessellation faithfully.

use std::collections::HashSet;
use zerocad_core::mock_kernel::{EdgeCurveHint, MockMesh};
use zerocad_core::{
    CoordinateSystem, CornerKind, EdgeRef, ExtrudeMode, FeatureNode, FeatureType, ParametricGraph,
    SketchCurves,
};

const R: f32 = 4.0;
const H: f32 = 15.5;

/// Circle sketched on the GROUND plane (XZ — the user's setup), extruded H.
fn extruded_circle_graph() -> ParametricGraph {
    let mut g = ParametricGraph::new();
    let mut curves = SketchCurves::new();
    curves.add_circle((0.0, 0.0), R);
    g.add_feature(FeatureNode {
        id: "sketch_1".into(),
        name: "Sketch".into(),
        feature: FeatureType::Sketch {
            cs: CoordinateSystem::XZ,
            curves,
            shapes: vec![],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
            entity_ids: vec![],
            next_entity_id: 0,
            solver: None,
        },
    });
    g.add_feature(FeatureNode {
        id: "extrude_2".into(),
        name: "Extrude".into(),
        feature: FeatureType::Extrude {
            target: None,
            depth: H,
            region_indices: vec![],
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    g.add_dependency("sketch_1", "extrude_2");
    g
}

/// Derive the far-cap rim EdgeRef from the evaluated mesh the way the GUI's
/// `edge_ref_from` does for a closed loop: first rim chord's endpoints as
/// p0/p1, that chord's stored adjacent-face normals, and a GUI-shaped
/// (~0.5%-off) closed circle hint.
fn rim_edge_from_mesh(mesh: &MockMesh) -> EdgeRef {
    // The cap farthest from the sketch plane along ±Y.
    let mut y_cap = 0.0f32;
    for v in mesh.edge_vertices.chunks_exact(3) {
        if v[1].abs() > y_cap.abs() {
            y_cap = v[1];
        }
    }
    let seg_count = mesh.edge_indices.len() / 2;
    let vpos = |seg: usize, which: usize| -> [f32; 3] {
        let vi = mesh.edge_indices[seg * 2 + which] as usize * 3;
        [
            mesh.edge_vertices[vi],
            mesh.edge_vertices[vi + 1],
            mesh.edge_vertices[vi + 2],
        ]
    };
    let on_rim = |p: [f32; 3]| -> bool {
        (p[1] - y_cap).abs() < 0.01 && (p[0].hypot(p[2]) - R).abs() < 0.05
    };
    let seg = (0..seg_count)
        .find(|&s| on_rim(vpos(s, 0)) && on_rim(vpos(s, 1)))
        .expect("top rim chord exists");
    let (p0, p1) = (vpos(seg, 0), vpos(seg, 1));
    let fo = seg * 6;
    let (n1, n2) = if mesh.edge_face_normals.len() >= fo + 6 {
        (
            [
                mesh.edge_face_normals[fo],
                mesh.edge_face_normals[fo + 1],
                mesh.edge_face_normals[fo + 2],
            ],
            [
                mesh.edge_face_normals[fo + 3],
                mesh.edge_face_normals[fo + 4],
                mesh.edge_face_normals[fo + 5],
            ],
        )
    } else {
        ([0.0, y_cap.signum(), 0.0], [1.0, 0.0, 0.0])
    };
    EdgeRef {
        p0,
        p1,
        n1,
        n2,
        curve: Some(EdgeCurveHint::Circle {
            center: [0.0, y_cap, 0.0],
            axis: [0.0, 1.0, 0.0],
            x_dir: [1.0, 0.0, 0.0],
            radius: R * 1.005,
            start: 0.0,
            end: std::f32::consts::TAU,
            closed: true,
        }),
        topology: None,
    }
}

fn rim_blend_commits(kind: CornerKind, dist: f32) {
    let mut g = extruded_circle_graph();
    let (base, base_warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("plain extruded cylinder evaluates");
    assert!(
        base_warnings.is_empty(),
        "plain body must be clean: {base_warnings:?}"
    );
    let plain_volume = base[0].1.mass_properties().expect("plain volume").volume;
    let rim = rim_edge_from_mesh(&base[0].1);

    g.add_feature(FeatureNode {
        id: "em_3".into(),
        name: "Edge Mod".into(),
        feature: FeatureType::EdgeMod {
            target: "extrude_2".to_string(),
            edge: rim,
            dist,
            dist_expr: None,
            kind,
        },
    });
    g.add_dependency("extrude_2", "em_3");

    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&HashSet::new())
        .expect("blended body evaluates");
    assert!(
        warnings.is_empty(),
        "{kind:?} d={dist} on the small extruded-circle rim must commit, got: {warnings:?}"
    );
    let volume = bodies[0]
        .1
        .mass_properties()
        .expect("blended volume")
        .volume;
    assert!(
        volume < plain_volume - 1.0,
        "{kind:?} d={dist} must remove material: {volume:.2} vs plain {plain_volume:.2}"
    );
}

#[test]
fn small_extruded_circle_rim_fillet_3mm_commits() {
    rim_blend_commits(CornerKind::Fillet, 3.0);
}

#[test]
fn small_extruded_circle_rim_chamfer_3mm_commits() {
    rim_blend_commits(CornerKind::Chamfer, 3.0);
}

#[test]
fn small_extruded_circle_rim_fillet_1mm_commits() {
    rim_blend_commits(CornerKind::Fillet, 1.0);
}
