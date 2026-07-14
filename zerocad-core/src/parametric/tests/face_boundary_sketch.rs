//! Sketch-on-face **reference boundary** tests: a sketch placed on a body face
//! carries that face's projected outline in `graph.sketch_face_boundaries`,
//! and the evaluator folds it into region detection. Two contracts:
//!
//! 1. A sketch with NOTHING drawn still has the face outline as a region —
//!    this is what direct face push/pull extrudes.
//! 2. A drawn shape overlapping the face edge splits against the outline
//!    exactly like it would split against another drawn shape, so the lens
//!    (shape ∩ face) is independently selectable/extrudable.

use super::common::*;
use super::*;
use crate::geometry::Vec3;

/// The top face of a 10×10×10 box at the origin, as a sketch plane whose
/// (u, v) run along world X/Y — the face spans (0,0)..(10,10) in uv.
fn top_face_cs() -> CoordinateSystem {
    CoordinateSystem::new(Vec3::new(0.0, 0.0, 10.0), Vec3::X, Vec3::Y)
}

fn box_with_face_sketch(drawn: SketchCurves) -> ParametricGraph {
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "box_1".to_string(),
        name: "Box".to_string(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
    add_sketch_cs(&mut g, "sketch_2", top_face_cs(), drawn);
    g.add_dependency("box_1", "sketch_2");
    // The projected top-face outline, as the GUI captures it at sketch start.
    g.sketch_face_boundaries.insert(
        "sketch_2".to_string(),
        rect_sketch((0.0, 0.0), (10.0, 10.0)),
    );
    g
}

fn mesh_z_range(mesh: &crate::MockMesh) -> (f32, f32) {
    let mut min_z = f32::INFINITY;
    let mut max_z = f32::NEG_INFINITY;
    for v in mesh.vertices.chunks(6) {
        min_z = min_z.min(v[2]);
        max_z = max_z.max(v[2]);
    }
    (min_z, max_z)
}

/// ASSOCIATIVITY: the projected face outline must re-derive from wherever the
/// face is after the parent body changes — an outline-only face sketch's boss
/// follows the widened face instead of staying frozen at the captured 10×10
/// footprint, and the refreshed outline + plane are committed back to the
/// graph by `apply_face_reattach`.
#[test]
fn face_outline_rederives_when_the_body_changes() {
    use std::collections::HashSet;
    let none: HashSet<String> = HashSet::new();

    // Base plate 10×10×8 from a rect sketch (so its width is editable).
    let mut g = ParametricGraph::new();
    add_sketch(&mut g, "base_sketch", rect_sketch((0.0, 0.0), (10.0, 10.0)));
    add_extrude(&mut g, "base", "base_sketch", 8.0, ExtrudeMode::NewBody);

    // Capture the base's top face the way the GUI does: durable ref, a plane
    // at the face centroid, and the projected outline from the built mesh.
    let bodies = g.evaluate_bodies(&none).unwrap();
    let top = bodies[0]
        .1
        .face_refs
        .iter()
        .find(|f| {
            f.topology
                .as_ref()
                .and_then(|t| t.face_id.as_deref())
                .is_some_and(|id| id.ends_with(":face:top"))
        })
        .expect("base top cap should carry a stable face name")
        .clone();
    let face = FaceRef {
        centroid: top.centroid,
        normal: top.normal,
        topology: top.topology.clone().map(|t| TopologyFaceRef {
            body_id: t.body_id.or_else(|| Some("base".to_string())),
            component_id: t.component_id,
            topology_version: t.topology_version,
            face_id: t.face_id,
            surface_kind: t.surface_kind,
        }),
    };
    let cs = CoordinateSystem::new(
        crate::geometry::Vec3::new(top.centroid[0], top.centroid[1], top.centroid[2]),
        crate::geometry::Vec3::X,
        crate::geometry::Vec3::Y,
    );
    let boundary = crate::mock_kernel::mesh_face_boundary_2d(&bodies[0].1, top.face_id, &cs);
    assert!(!boundary.is_empty(), "captured no top-face outline");

    // Outline-only sketch on that face + a boss extruded from it (all regions).
    add_sketch_cs(&mut g, "face_sketch", cs, SketchCurves::new());
    g.sketch_face_refs.insert("face_sketch".to_string(), face);
    g.sketch_face_boundaries
        .insert("face_sketch".to_string(), boundary);
    g.add_dependency("base", "face_sketch");
    add_extrude(&mut g, "boss", "face_sketch", 3.0, ExtrudeMode::NewBody);

    let boss_x_span = |g: &ParametricGraph| -> (f32, f32) {
        let bodies = g.evaluate_bodies(&none).unwrap();
        let boss = bodies
            .iter()
            .find(|(id, _)| id == "boss")
            .expect("boss body");
        boss.1
            .vertices
            .chunks(6)
            .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), v| {
                (lo.min(v[0]), hi.max(v[0]))
            })
    };
    let (x0, x1) = boss_x_span(&g);
    assert!(
        x0.abs() < 0.2 && (x1 - 10.0).abs() < 0.2,
        "boss should cover the 10-wide face, got x {x0}..{x1}"
    );

    // Widen the base plate to 14 — the top face outline grows with it.
    for idx in g.graph.node_indices() {
        if g.graph[idx].id == "base_sketch" {
            if let FeatureType::Sketch { curves, .. } = &mut g.graph[idx].feature {
                *curves = rect_sketch((0.0, 0.0), (14.0, 10.0));
            }
        }
    }
    let (x0, x1) = boss_x_span(&g);
    assert!(
        x0.abs() < 0.2 && (x1 - 14.0).abs() < 0.2,
        "boss must follow the widened face outline, got x {x0}..{x1}"
    );

    // The refresh is committed back: the stored outline snapshot now spans the
    // 14-wide face (±7 around the re-derived centroid plane).
    assert!(g.apply_face_reattach(), "expected a queued face reattach");
    let refreshed = g
        .sketch_face_boundaries
        .get("face_sketch")
        .expect("refreshed outline stored");
    let (u_min, u_max) = refreshed
        .segments
        .iter()
        .flat_map(|s| [s.a.0, s.b.0])
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), u| {
            (lo.min(u), hi.max(u))
        });
    assert!(
        (u_min + 7.0).abs() < 0.2 && (u_max - 7.0).abs() < 0.2,
        "stored outline should now span the 14-wide face, got u {u_min}..{u_max}"
    );
    // And a re-evaluation right after the write-back is stable (no new queue).
    let (x0, x1) = boss_x_span(&g);
    assert!(x0.abs() < 0.2 && (x1 - 14.0).abs() < 0.2);
    assert!(
        !g.apply_face_reattach(),
        "reattach must converge after one write-back"
    );
}

/// Push/pull core path: a sketch with an EMPTY drawn set still evaluates to
/// one region (the face outline), and joining it 5 out grows the box to z=15.
#[test]
fn face_boundary_only_sketch_extrudes_the_whole_face() {
    let mut g = box_with_face_sketch(SketchCurves::new());
    add_extrude(&mut g, "extrude_3", "sketch_2", 5.0, ExtrudeMode::Join);
    g.add_dependency("box_1", "extrude_3");

    let mesh = g.evaluate().expect("evaluate");
    assert!(!mesh.vertices.is_empty(), "join built no geometry");
    let (min_z, max_z) = mesh_z_range(&mesh);
    assert!(
        (min_z - 0.0).abs() < 1.0e-3 && (max_z - 15.0).abs() < 1.0e-3,
        "expected the box to grow to z=15, got z range {min_z}..{max_z}"
    );
}

/// A circle drawn half-off the face splits against the projected outline into
/// three regions (face minus lens, the lens, the overhang), and cutting the
/// lens region bores a pocket bounded by the face edge.
#[test]
fn drawn_circle_splits_against_face_boundary() {
    // Circle centered ON the face's right edge (x=10) → the in-face lens is a
    // half-disc spanning x∈[7,10].
    let mut drawn = SketchCurves::new();
    drawn.add_circle((10.0, 5.0), 3.0);

    // Region split, exactly as the evaluator merges: drawn first, boundary after.
    let mut merged = drawn.clone();
    merged.extend_curves(&rect_sketch((0.0, 0.0), (10.0, 10.0)));
    let regions = crate::sketch::detect_regions(&merged);
    assert_eq!(
        regions.len(),
        3,
        "circle overlapping the face edge should split into 3 regions, got {}",
        regions.len()
    );
    let lens_idx = regions
        .iter()
        .position(|r| r.contains((9.0, 5.0)) && !r.contains((2.0, 5.0)))
        .expect("no lens (circle ∩ face) region found");

    // End-to-end: cut that lens region 5 deep into the box.
    let mut g = box_with_face_sketch(drawn);
    g.add_feature(FeatureNode {
        id: "extrude_3".to_string(),
        name: "Cut".to_string(),
        feature: FeatureType::Extrude {
            target: None,
            depth: -5.0,
            region_indices: vec![lens_idx],
            mode: ExtrudeMode::Cut,
            depth_expr: None,
        },
    });
    g.add_dependency("sketch_2", "extrude_3");
    g.add_dependency("box_1", "extrude_3");

    let mesh = g.evaluate().expect("evaluate");
    assert!(!mesh.vertices.is_empty(), "cut built no geometry");
    let (min_z, max_z) = mesh_z_range(&mesh);
    assert!(
        (min_z - 0.0).abs() < 1.0e-3 && (max_z - 10.0).abs() < 1.0e-3,
        "cut must not change the outer z range, got {min_z}..{max_z}"
    );
    // The pocket floor: some vertex near z=5 inside the lens footprint.
    let has_floor = mesh
        .vertices
        .chunks(6)
        .any(|v| (v[2] - 5.0).abs() < 0.11 && v[0] > 7.0 - 0.11 && (v[1] - 5.0).abs() < 3.11);
    assert!(has_floor, "expected a pocket floor near z=5 under the lens");
}
