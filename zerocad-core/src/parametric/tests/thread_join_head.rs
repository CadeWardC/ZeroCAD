//! Joining/cutting a plain solid onto a **threaded** body. A boolean against the
//! analytic helical bands is not viable, so the evaluator routes the boolean
//! through the body's smooth pre-thread base and replays the thread afterward
//! (see `ThreadReplay`). Regression for the reported bug: a head extruded from a
//! threaded shaft's bottom face and set to Join produced no geometry at all —
//! the union silently dropped the head.

use super::*;
use crate::geometry::Vec3;
use crate::parametric::FaceRef;

fn add_cylinder(g: &mut ParametricGraph, id: &str, r: f32, h: f32) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Cylinder { r, h },
    });
}

/// Model a helical thread on the cylindrical wall of `target`. The wall of a
/// `Cylinder { r, h }` (its axis is +Y, base at the origin) has centroid
/// `(r, h/2, 0)` with an outward radial normal.
fn add_thread(g: &mut ParametricGraph, id: &str, target: &str, r: f32, h: f32) {
    g.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Thread {
            target: target.to_string(),
            face: FaceRef {
                centroid: [r, h * 0.5, 0.0],
                normal: [1.0, 0.0, 0.0],
                topology: None,
            },
            internal: false,
            pitch: 1.0,
            depth: 0.4,
            angle_deg: 60.0,
            right_handed: true,
            starts: 1,
            length: None,
            flip: false,
            designation: "M-test".to_string(),
        },
    });
    g.add_dependency(target, id);
}

fn body_mesh<'a>(bodies: &'a [(String, MockMesh)], id: &str) -> &'a MockMesh {
    bodies
        .iter()
        .find(|(bid, _)| bid == id)
        .map(|(_, m)| m)
        .unwrap_or_else(|| panic!("body '{id}' present; have {:?}", ids(bodies)))
}

fn ids(bodies: &[(String, MockMesh)]) -> Vec<String> {
    bodies.iter().map(|(id, _)| id.clone()).collect()
}

/// A plain head fused onto the bottom face of a threaded shaft must actually
/// appear (extend below the shaft) and stay one body — the reported bug left it
/// invisible because the boolean fought the helical bands.
#[test]
fn join_head_onto_threaded_shaft_fuses_and_keeps_threads() {
    let (r, h) = (6.0f32, 10.0f32);
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "cyl_1", r, h);
    add_thread(&mut g, "thread_2", "cyl_1", r, h);

    // Sketch a wider head disc on the shaft's bottom face (the XZ plane at y=0).
    // u=X, v=Z gives the outward normal n = u × v = -Y, so a positive-depth
    // extrude sweeps *downward*, away from the shaft (the boss's near cap dips
    // back into the body). Head radius 9 > shaft 6 so it reads as a real head.
    let bottom = CoordinateSystem::new(Vec3::new(0.0, 0.0, 0.0), Vec3::X, Vec3::Z);
    let mut head = SketchCurves::new();
    head.add_circle((0.0, 0.0), 9.0);
    add_sketch_cs(&mut g, "sketch_3", bottom, head);
    add_extrude(&mut g, "extrude_4", "sketch_3", 4.0, ExtrudeMode::Join);
    g.add_dependency("cyl_1", "extrude_4");

    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();

    // The head must have fused INTO the shaft body, not become a separate lump.
    assert_eq!(bodies.len(), 1, "one fused body; got {:?}", ids(&bodies));
    assert!(
        !warnings.iter().any(|w| w.contains("separate body")),
        "the head must join, not orphan: {warnings:?}"
    );

    let mesh = body_mesh(&bodies, "cyl_1");
    assert!(
        !mesh.vertices.is_empty() && !mesh.indices.is_empty(),
        "fused body keeps a valid mesh"
    );

    // The head extends the body well below the shaft's original bottom (y=0).
    // Before the fix the head vanished and the min stayed ~0.
    let (lo, hi) = mesh_aabb(mesh);
    eprintln!("fused AABB y: [{:.3}, {:.3}]", lo[1], hi[1]);
    assert!(
        lo[1] < -3.0,
        "head should extend ~4mm below y=0 (got min y={:.3})",
        lo[1]
    );
    assert!(
        hi[1] > h - 0.5,
        "shaft top preserved (got max y={:.3})",
        hi[1]
    );

    // The shaft must still carry thread geometry: threaded volume is below the
    // plain (shaft ∪ head) union because the grooves removed wall material,
    // yet far above shaft-only — proving the head is really there.
    let props = mesh.mass_properties().expect("closed fused solid");
    let shaft = std::f64::consts::PI * (r as f64).powi(2) * h as f64;
    let head_vol = std::f64::consts::PI * 9.0f64.powi(2) * 4.0;
    let plain_union = shaft + head_vol;
    eprintln!(
        "fused volume={:.2} plain_union≈{:.2} shaft_only≈{:.2}",
        props.volume, plain_union, shaft
    );
    assert!(
        props.volume > shaft + 0.5 * head_vol,
        "the head's volume must be present: {:.2} vs shaft {:.2}",
        props.volume,
        shaft
    );
    assert!(
        props.volume < plain_union + 1.0,
        "a join only adds material: {:.2} vs {:.2}",
        props.volume,
        plain_union
    );
}

/// Symmetric case: a pocket cut into a threaded shaft must remove material and
/// keep the body whole (the cut routes through the smooth base, then re-threads).
#[test]
fn cut_pocket_into_threaded_shaft_removes_material() {
    let (r, h) = (6.0f32, 10.0f32);
    let mut g = ParametricGraph::new();
    add_cylinder(&mut g, "cyl_1", r, h);
    add_thread(&mut g, "thread_2", "cyl_1", r, h);

    // A small square pocket cut into the top face (y=h), sweeping downward (-Y).
    // u=X, v=Z ⇒ n=-Y at the top; positive depth bores into the shaft.
    let top = CoordinateSystem::new(Vec3::new(0.0, h, 0.0), Vec3::X, Vec3::Z);
    let pocket = rect_sketch((-2.0, -2.0), (2.0, 2.0));
    add_sketch_cs(&mut g, "sketch_3", top, pocket);
    add_extrude(&mut g, "extrude_4", "sketch_3", 3.0, ExtrudeMode::Cut);
    g.add_dependency("cyl_1", "extrude_4");

    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();

    assert_eq!(bodies.len(), 1, "still one body; got {:?}", ids(&bodies));
    let mesh = body_mesh(&bodies, "cyl_1");
    assert!(
        !mesh.vertices.is_empty() && !mesh.indices.is_empty(),
        "pocketed body keeps a valid mesh"
    );
    assert!(
        !warnings.iter().any(|w| w.contains("left intact")),
        "the pocket must subtract, not fail on overlap: {warnings:?}"
    );

    let props = mesh.mass_properties().expect("closed pocketed solid");
    let shaft = std::f64::consts::PI * (r as f64).powi(2) * h as f64;
    eprintln!("pocketed volume={:.2} shaft≈{:.2}", props.volume, shaft);
    assert!(
        props.volume < shaft,
        "the pocket must remove material: {:.2} vs shaft {:.2}",
        props.volume,
        shaft
    );
}

/// Axis-aligned AABB of a mesh, as (min, max).
fn mesh_aabb(mesh: &MockMesh) -> ([f32; 3], [f32; 3]) {
    let mut lo = [f32::INFINITY; 3];
    let mut hi = [f32::NEG_INFINITY; 3];
    for v in mesh.vertices.chunks_exact(3) {
        for k in 0..3 {
            lo[k] = lo[k].min(v[k]);
            hi[k] = hi[k].max(v[k]);
        }
    }
    (lo, hi)
}
