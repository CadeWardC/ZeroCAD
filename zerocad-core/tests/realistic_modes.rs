use std::collections::HashSet;
use zerocad_core::{
    CoordinateSystem, ExtrudeMode, FeatureNode, FeatureType, ParametricGraph, SketchCurves, Vec3,
};

fn rect(min: (f32, f32), max: (f32, f32)) -> SketchCurves {
    let mut c = SketchCurves::new();
    c.add_rectangle(min, max);
    c
}

fn add_sketch(g: &mut ParametricGraph, id: &str, cs: CoordinateSystem, curves: SketchCurves) {
    g.add_feature(FeatureNode {
        id: id.into(),
        name: id.into(),
        feature: FeatureType::Sketch {
            cs,
            curves,
            shapes: vec![],
            corner_mods: vec![],
            on_face: false,
        },
    });
}
fn add_extrude(g: &mut ParametricGraph, id: &str, sketch: &str, depth: f32, mode: ExtrudeMode) {
    g.add_feature(FeatureNode {
        id: id.into(),
        name: id.into(),
        feature: FeatureType::Extrude {
            depth,
            region_indices: vec![],
            mode,
            depth_expr: None,
        },
    });
    g.add_dependency(sketch, id);
}

// A plane at z = h, normal +Z (simulating sketching on a body's top face).
fn top_plane(h: f32) -> CoordinateSystem {
    CoordinateSystem::new(
        Vec3::new(0.0, 0.0, h),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
}

// A plane on the front face of the base block (y = 0), normal -Y.
fn front_plane() -> CoordinateSystem {
    CoordinateSystem::new(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
}

// The same sort of face-local frame the GUI derives from a selected front face:
// origin at face centroid, normal -Y, u = -Z, v = +X.
fn gui_front_face_plane() -> CoordinateSystem {
    CoordinateSystem::new(
        Vec3::new(5.0, 0.0, 5.0),
        Vec3::new(0.0, 0.0, -1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
}

fn tris(g: &ParametricGraph) -> Vec<(String, usize)> {
    g.evaluate_bodies(&HashSet::new())
        .unwrap()
        .into_iter()
        .map(|(id, m)| (id, m.indices.len() / 3))
        .collect()
}

// A Cut carves toward the material, whichever way the depth is signed. The
// cutter is built in both sweep directions and `apply_cut` keeps the one whose
// AABB overlaps the body more — so a sketch on the TOP face cut with a POSITIVE
// depth (which sweeps the tool *up*, away from the body) still bites a pocket
// DOWN into the block, matching the Fusion-style "a cut removes material"
// expectation. (Previously a positive top-face cut swept into empty air and did
// nothing — the reported "cut does not work".) A cut still reaches a body that
// lies on the *outward* side, because that direction would then have the overlap.
#[test]
fn cut_on_top_face_carves_into_body_regardless_of_sign() {
    let mut g = ParametricGraph::new();
    add_sketch(
        &mut g,
        "sketch_1",
        CoordinateSystem::XY,
        rect((0.0, 0.0), (10.0, 10.0)),
    );
    add_extrude(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);

    // Sketch on the TOP face (z=10, normal +Z), POSITIVE depth = drawn away from
    // the body — must still carve a pocket into the block below.
    add_sketch(
        &mut g,
        "sketch_3",
        top_plane(10.0),
        rect((2.0, 2.0), (8.0, 8.0)),
    );
    add_extrude(&mut g, "extrude_4", "sketch_3", 5.0, ExtrudeMode::Cut);
    let after = tris(&g);
    println!("after cut(+5 on top): {:?}", after);
    assert_eq!(after.len(), 1, "cut should keep exactly one body");
    assert!(
        has_vertex_z(&g, 4.0, 6.0),
        "a positive-depth cut on the top face must carve a pocket DOWN into the \
         body (floor near z=5), not sweep into empty air and do nothing"
    );
}

#[test]
fn join_on_top_face_positive_depth() {
    let mut g = ParametricGraph::new();
    add_sketch(
        &mut g,
        "sketch_1",
        CoordinateSystem::XY,
        rect((0.0, 0.0), (10.0, 10.0)),
    );
    add_extrude(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);

    add_sketch(
        &mut g,
        "sketch_3",
        top_plane(10.0),
        rect((2.0, 2.0), (8.0, 8.0)),
    );
    add_extrude(&mut g, "extrude_4", "sketch_3", 5.0, ExtrudeMode::Join);
    let after = tris(&g);
    println!("after join(+5 on top): {:?}", after);
    assert_eq!(
        after.len(),
        1,
        "join should produce one merged body, got {:?}",
        after
    );
}

// Helper: does any vertex of body have z within [lo,hi]?
fn has_vertex_z(g: &ParametricGraph, lo: f32, hi: f32) -> bool {
    let bodies = g.evaluate_bodies(&HashSet::new()).unwrap();
    bodies
        .iter()
        .any(|(_, m)| m.vertices.chunks(6).any(|v| v[2] >= lo && v[2] <= hi))
}

// A NEGATIVE depth on the top face (normal +Z) cuts INTO the body: a 5mm-deep
// pocket from z=10 down has its floor near z=5.
#[test]
fn cut_into_top_face_negative_depth_reaches_body() {
    let mut g = ParametricGraph::new();
    add_sketch(
        &mut g,
        "sketch_1",
        CoordinateSystem::XY,
        rect((0.0, 0.0), (10.0, 10.0)),
    );
    add_extrude(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);
    add_sketch(
        &mut g,
        "sketch_3",
        top_plane(10.0),
        rect((2.0, 2.0), (8.0, 8.0)),
    );
    add_extrude(&mut g, "extrude_4", "sketch_3", -5.0, ExtrudeMode::Cut);
    // A real 5mm-deep pocket from the top face (z=10) has its floor near z=5.
    let floor_ok = has_vertex_z(&g, 4.0, 6.0);
    println!("pocket floor near z=5 present: {}", floor_ok);
    assert!(
        floor_ok,
        "cut of depth -5 on the top face should reach ~5mm into the body (floor near z=5)"
    );
}

#[test]
fn join_boss_rises_above_body() {
    let mut g = ParametricGraph::new();
    add_sketch(
        &mut g,
        "sketch_1",
        CoordinateSystem::XY,
        rect((0.0, 0.0), (10.0, 10.0)),
    );
    add_extrude(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);
    add_sketch(
        &mut g,
        "sketch_3",
        top_plane(10.0),
        rect((2.0, 2.0), (8.0, 8.0)),
    );
    add_extrude(&mut g, "extrude_4", "sketch_3", 5.0, ExtrudeMode::Join);
    let bodies = g.evaluate_bodies(&HashSet::new()).unwrap();
    assert_eq!(bodies.len(), 1, "join must yield a single merged body");
    // The boss adds material up to z = 15.
    let top = bodies[0]
        .1
        .vertices
        .chunks(6)
        .map(|v| v[2])
        .fold(f32::MIN, f32::max);
    println!("merged body top z = {top}");
    assert!(
        top > 14.0,
        "joined boss should raise the body to ~z=15, got {top}"
    );
}

// The front face's outward normal is -Y (the body is at y>0), so a POSITIVE
// depth sweeps away from the block and must not carve into it.
// Regression (the screenshot bug): a Join whose extrude runs INTO an existing
// body — a negative depth on a face whose normal points outward — must KEEP the
// body's geometry, not replace it. The tool extrudes down into the block, so the
// union can only ever add (here: nothing, the tool is swallowed), never remove.
// Before the fix, the negative-depth tool was built inside-out, so truck's
// `union` treated it as a complement and deleted the original block.
#[test]
fn join_into_body_negative_depth_keeps_original() {
    let mut g = ParametricGraph::new();
    add_sketch(
        &mut g,
        "sketch_1",
        CoordinateSystem::XY,
        rect((0.0, 0.0), (10.0, 10.0)),
    );
    add_extrude(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);

    // Sketch on the top face (z=10), join DOWNWARD into the block (negative).
    add_sketch(
        &mut g,
        "sketch_3",
        top_plane(10.0),
        rect((2.0, 2.0), (8.0, 8.0)),
    );
    add_extrude(&mut g, "extrude_4", "sketch_3", -5.0, ExtrudeMode::Join);

    let bodies = g.evaluate_bodies(&HashSet::new()).unwrap();
    assert_eq!(
        bodies.len(),
        1,
        "join must yield a single body, got {bodies:?}"
    );

    // The merged body must still enclose the original 10x10x10 block.
    let (mut mn, mut mx) = ([f32::INFINITY; 3], [f32::NEG_INFINITY; 3]);
    for v in bodies[0].1.vertices.chunks(6) {
        for k in 0..3 {
            mn[k] = mn[k].min(v[k]);
            mx[k] = mx[k].max(v[k]);
        }
    }
    println!("merged body AABB: min={mn:?} max={mx:?}");
    assert!(
        mn[0] <= 0.1 && mn[1] <= 0.1 && mn[2] <= 0.1,
        "join into a body deleted the original (min corner moved): {mn:?}"
    );
    assert!(
        mx[0] >= 9.9 && mx[1] >= 9.9 && mx[2] >= 9.9,
        "join into a body deleted the original (max corner shrank): {mx:?}"
    );
    // A join whose tool is swallowed by the body must not grow it meaningfully:
    // the top stays at z≈10 (a sub-0.1mm coplanarity-overshoot sliver is fine,
    // a real boss is not).
    assert!(
        mx[2] <= 10.2,
        "join into a body grew the top to z={} — the tool should be absorbed",
        mx[2]
    );
}

// The front face's outward normal is -Y (the body is at y>0), so a POSITIVE
// depth sweeps away from the block and must not carve into it.
#[test]
fn cut_on_side_face_outward_positive_depth_no_pocket() {
    let mut g = ParametricGraph::new();
    add_sketch(
        &mut g,
        "sketch_1",
        CoordinateSystem::XY,
        rect((0.0, 0.0), (10.0, 10.0)),
    );
    add_extrude(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);

    add_sketch(
        &mut g,
        "sketch_3",
        front_plane(),
        rect((2.0, 2.0), (8.0, 8.0)),
    );
    add_extrude(&mut g, "extrude_4", "sketch_3", 5.0, ExtrudeMode::Cut);
    println!("after cut(+5 outward on front)");
    assert_eq!(
        verts_in_pocket(&g, 5.0, 5.0, 2.5, 2.0, 8.0),
        0,
        "an OUTWARD (+) cut on the front face must not carve into the block \
         (no pocket walls/floor mid-body)"
    );
}

// Into the front face: the body is at y>0 and the outward normal is -Y, so a
// NEGATIVE depth sweeps into the block and carves a blind pocket.
#[test]
fn cut_into_side_face_negative_depth_carves_pocket() {
    let mut g = ParametricGraph::new();
    add_sketch(
        &mut g,
        "sketch_1",
        CoordinateSystem::XY,
        rect((0.0, 0.0), (10.0, 10.0)),
    );
    add_extrude(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);

    add_sketch(
        &mut g,
        "sketch_3",
        front_plane(),
        rect((2.0, 2.0), (8.0, 8.0)),
    );
    add_extrude(&mut g, "extrude_4", "sketch_3", -5.0, ExtrudeMode::Cut);
    let after = g.evaluate_bodies(&HashSet::new()).unwrap();
    assert_eq!(after.len(), 1, "side cut should keep exactly one body");

    // The pocket floor sits ~5mm into the block (near y=5). An uncut block has
    // surfaces only at y=0 and y=10, so any vertex at y≈5 proves the cut carved
    // real material. (Tri count alone is unreliable — it can coincide with the
    // uncut block — and the pocket's corner vertices land on the rect edges, so
    // a centred-column probe can miss them.)
    let floor = after
        .iter()
        .flat_map(|(_, m)| m.vertices.chunks(6))
        .any(|v| v[1] >= 4.0 && v[1] <= 6.0);
    assert!(
        floor,
        "an INTO (-) cut on the front face should carve a pocket reaching ~5mm in (floor near y=5)"
    );
}

// Regression: the bug from the screenshots. A pocket sketched on a body face
// whose profile reaches the EDGE of that face leaves the cut tool's side wall
// coplanar with the body's side face — which made truck's boolean return None,
// so the cut silently removed nothing. The evaluator now retries with a
// slightly expanded tool, so material must still be removed.
#[test]
fn cut_reaching_face_edge_still_removes_material() {
    let mut g = ParametricGraph::new();
    add_sketch(
        &mut g,
        "sketch_1",
        CoordinateSystem::XY,
        rect((0.0, 0.0), (10.0, 10.0)),
    );
    add_extrude(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);
    let plain = tris(&g);
    println!("plain body: {:?}", plain);

    // Pocket on the top face spanning the FULL x extent (0..10): the tool's
    // x=0 and x=10 walls are coplanar with the box's two side faces. Negative
    // depth cuts down INTO the block (the top normal points outward, +Z).
    add_sketch(
        &mut g,
        "sketch_3",
        top_plane(10.0),
        rect((0.0, 2.0), (10.0, 8.0)),
    );
    add_extrude(&mut g, "extrude_4", "sketch_3", -4.0, ExtrudeMode::Cut);
    let after = tris(&g);
    println!("after edge-reaching cut: {:?}", after);

    assert_eq!(after.len(), 1, "cut should keep exactly one body");
    assert_ne!(
        after[0].1, plain[0].1,
        "a pocket reaching the face edge must still remove material"
    );
}

#[test]
fn cut_on_gui_face_coordinate_system() {
    let mut g = ParametricGraph::new();
    add_sketch(
        &mut g,
        "sketch_1",
        CoordinateSystem::XY,
        rect((0.0, 0.0), (10.0, 10.0)),
    );
    add_extrude(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);
    let plain = tris(&g);

    add_sketch(
        &mut g,
        "sketch_3",
        gui_front_face_plane(),
        rect((-3.0, -3.0), (3.0, 3.0)),
    );
    add_extrude(&mut g, "extrude_4", "sketch_3", -5.0, ExtrudeMode::Cut);
    let after = tris(&g);
    println!("after cut in GUI-like front frame: {:?}", after);

    assert_eq!(
        after.len(),
        1,
        "GUI-like side cut should keep exactly one body"
    );
    assert_ne!(
        after[0].1, plain[0].1,
        "GUI-like side cut should change the body geometry"
    );
}

// How many body vertices land inside the cut column `|x-cx|<rad`, `|z-cz|<rad`
// at height `lo..hi`? An uncut solid has no surface there, so a positive count
// means the cut actually carved a pocket into the material (not merely changed
// the triangle count, which an inside-out boolean can do while adding volume).
fn verts_in_pocket(g: &ParametricGraph, cx: f32, cz: f32, rad: f32, lo: f32, hi: f32) -> usize {
    g.evaluate_bodies(&HashSet::new())
        .unwrap()
        .iter()
        .flat_map(|(_, m)| m.vertices.chunks(6))
        .filter(|v| v[1] > lo && v[1] < hi && (v[0] - cx).abs() < rad && (v[2] - cz).abs() < rad)
        .count()
}

// Regression: cutting into a CYLINDER PRIMITIVE used to remove nothing. The
// primitive's boolean solid was a *true* cylinder, whose smooth faces make
// truck's solver panic (caught by the guard -> the op silently no-ops). It is
// now a 48-gon prism, so the cut carves a real pocket.
#[test]
fn cut_into_cylinder_primitive_carves_pocket() {
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "cyl_1".into(),
        name: "cyl".into(),
        feature: FeatureType::Cylinder { r: 10.0, h: 10.0 },
    });
    // 4x4 pocket down through the top (cylinder axis is +Y, top at y=10).
    let top = CoordinateSystem::new(
        Vec3::new(0.0, 10.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    );
    add_sketch(&mut g, "sketch_3", top, rect((-2.0, -2.0), (2.0, 2.0)));
    add_extrude(&mut g, "extrude_4", "sketch_3", 6.0, ExtrudeMode::Cut);

    assert!(
        verts_in_pocket(&g, 0.0, 0.0, 2.5, 3.0, 5.0) > 0,
        "cut into a cylinder primitive must carve a pocket (floor near y=4)"
    );
}

// Regression: a body extruded from an XZ/YZ ORIGIN-PLANE sketch (the left-handed
// frame consts the GUI uses) used to be un-cuttable. truck extrudes left-handed
// frames inside-out, so `difference` ADDED the cut tool instead of subtracting
// it. `build_extrusion_solid` now flips the winding to keep such bodies outward.
#[test]
fn cut_into_xz_origin_plane_body_carves_pocket() {
    let mut g = ParametricGraph::new();
    add_sketch(
        &mut g,
        "sketch_1",
        CoordinateSystem::XZ,
        rect((0.0, 0.0), (10.0, 10.0)),
    );
    add_extrude(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);

    // 4x4 pocket into the top (y=10) of the extruded body, centered at (5,*,5).
    let top = CoordinateSystem::new(
        Vec3::new(5.0, 10.0, 5.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    );
    add_sketch(&mut g, "sketch_3", top, rect((-2.0, -2.0), (2.0, 2.0)));
    add_extrude(&mut g, "extrude_4", "sketch_3", 6.0, ExtrudeMode::Cut);

    assert!(
        verts_in_pocket(&g, 5.0, 5.0, 2.5, 3.0, 5.0) > 0,
        "cut into an XZ-origin-plane body must carve a pocket"
    );
}

// Regression: a cut body's wireframe must (a) carry hidden-line normals so the
// renderer can hide back edges (no x-ray), and (b) never draw edges that shoot
// off the model — the degenerate "fin" edges truck's `difference` can leave.
// Both follow from deriving the wireframe from the tessellation's feature edges
// (`mesh_feature_edges`): a fin has no triangle, so no edge; every kept edge
// borders real faces, so it has two normals and lies within the fill's extent.
#[test]
fn cut_wireframe_stays_within_body_bounds() {
    let mut g = ParametricGraph::new();
    add_sketch(
        &mut g,
        "sketch_1",
        CoordinateSystem::XY,
        rect((0.0, 0.0), (10.0, 10.0)),
    );
    add_extrude(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);
    // Blind pocket on the top face — the configuration that produced the spikes.
    // Negative depth cuts down into the block (top normal is outward, +Z).
    let top = CoordinateSystem::new(
        Vec3::new(0.0, 0.0, 10.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    );
    add_sketch(&mut g, "sketch_3", top, rect((3.0, 3.0), (7.0, 7.0)));
    add_extrude(&mut g, "extrude_4", "sketch_3", -5.0, ExtrudeMode::Cut);

    for (id, m) in g.evaluate_bodies(&HashSet::new()).unwrap() {
        if m.edge_vertices.is_empty() {
            continue;
        }
        // (a) Hidden-line normals present: two per edge (6 floats).
        let num_edges = m.edge_indices.len() / 2;
        assert_eq!(
            m.edge_face_normals.len(),
            num_edges * 6,
            "body {id}: cut wireframe must carry two face normals per edge for HLR"
        );
        // Fill (triangle) AABB = the body's true extent.
        let mut mn = [f32::INFINITY; 3];
        let mut mx = [f32::NEG_INFINITY; 3];
        for v in m.vertices.chunks(6) {
            for k in 0..3 {
                mn[k] = mn[k].min(v[k]);
                mx[k] = mx[k].max(v[k]);
            }
        }
        let ext = (0..3).map(|k| mx[k] - mn[k]).fold(0.0, f32::max);
        let tol = ext * 0.05 + 0.01; // generous: spikes overshoot by ~a body-width
        for (i, p) in m.edge_vertices.chunks(3).enumerate() {
            for k in 0..3 {
                assert!(
                    p[k] >= mn[k] - tol && p[k] <= mx[k] + tol,
                    "body {id}: wireframe vertex {i} component {k} = {} is outside fill bounds [{}, {}] — a stray boolean edge",
                    p[k], mn[k], mx[k]
                );
            }
        }
    }
}
