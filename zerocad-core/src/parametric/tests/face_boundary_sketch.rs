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
    g.sketch_face_boundaries
        .insert("sketch_2".into(), rect_sketch((0.0, 0.0), (10.0, 10.0)));
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

fn attach_direct_top_face(graph: &mut ParametricGraph, body_id: &str, sketch_id: &str) -> usize {
    let bodies = graph
        .evaluate_bodies(&std::collections::HashSet::new())
        .expect("evaluate face owner");
    let (_, mesh) = bodies
        .iter()
        .find(|(id, _)| id == body_id)
        .expect("face owner body");
    let top = mesh
        .face_refs
        .iter()
        .filter(|face| face.normal[2] > 0.99)
        .max_by(|left, right| left.centroid[2].total_cmp(&right.centroid[2]))
        .expect("top planar face")
        .clone();
    let cs = CoordinateSystem::new(
        Vec3::new(top.centroid[0], top.centroid[1], top.centroid[2]),
        Vec3::X,
        Vec3::Y,
    );
    let boundary = crate::mock_kernel::mesh_face_boundary_2d(mesh, top.face_id, &cs);
    let regions = detect_regions(&boundary);
    assert!(!regions.is_empty(), "direct face has no detected material");
    graph.add_feature(FeatureNode {
        id: sketch_id.to_string(),
        name: sketch_id.to_string(),
        feature: FeatureType::Sketch {
            cs,
            curves: SketchCurves::new(),
            shapes: Vec::new(),
            corner_mods: Vec::new(),
            mirrors: Vec::new(),
            on_face: true,
            entity_ids: Vec::new(),
            next_entity_id: 0,
            solver: None,
        },
    });
    graph.add_dependency(body_id, sketch_id);
    graph.sketch_face_refs.insert(
        sketch_id.into(),
        FaceRef {
            centroid: top.centroid,
            normal: top.normal,
            topology: top.topology.map(|topology| TopologyFaceRef {
                body_id: topology.body_id.or_else(|| Some(body_id.to_string())),
                component_id: topology.component_id,
                topology_version: topology.topology_version,
                face_id: topology.face_id,
                surface_kind: topology.surface_kind,
                producer_feature_id: topology.producer_feature_id,
                source_entity_id: topology.source_entity_id,
            }),
        },
    );
    graph
        .sketch_face_boundaries
        .insert(sketch_id.into(), boundary);
    regions
        .iter()
        .enumerate()
        .max_by(|(_, left), (_, right)| left.area.total_cmp(&right.area))
        .map(|(index, _)| index)
        .expect("top material region")
}

fn add_direct_face_extrude(
    graph: &mut ParametricGraph,
    id: &str,
    sketch_id: &str,
    target: Option<&str>,
    region_index: usize,
    depth: f32,
    mode: ExtrudeMode,
) {
    graph.add_feature(FeatureNode {
        id: id.to_string(),
        name: id.to_string(),
        feature: FeatureType::Extrude {
            target: target.map(str::to_string),
            depth,
            region_indices: vec![region_index],
            mode,
            depth_expr: None,
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
        },
    });
    graph.add_dependency(sketch_id, id);
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
            producer_feature_id: t.producer_feature_id,
            source_entity_id: t.source_entity_id,
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
    g.sketch_face_refs.insert("face_sketch".into(), face);
    g.sketch_face_boundaries
        .insert("face_sketch".into(), boundary);
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

#[test]
fn direct_face_join_uses_exact_face_wires_and_removes_the_shared_face() {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "box_1".into(),
        name: "Box".into(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
    let region = attach_direct_top_face(&mut graph, "box_1", "face_sketch");
    add_direct_face_extrude(
        &mut graph,
        "join_3",
        "face_sketch",
        Some("box_1"),
        region,
        6.07,
        ExtrudeMode::Join,
    );

    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("exact face join");
    assert!(warnings.is_empty(), "warnings: {warnings:#?}");
    assert_eq!(bodies.len(), 1);
    assert_eq!(bodies[0].0, "box_1");
    let (min_z, max_z) = mesh_z_range(&bodies[0].1);
    assert!((min_z - 0.0).abs() < 1.0e-4);
    assert!((max_z - 16.07).abs() < 1.0e-4, "max z {max_z}");
    let volume = bodies[0]
        .1
        .mass_properties()
        .expect("joined mass properties")
        .volume;
    assert!((volume - 1_607.0).abs() < 0.05, "volume {volume}");
    assert!(
        bodies[0].1.face_refs.iter().all(|face| {
            !(face.normal[2].abs() > 0.99 && (face.centroid[2] - 10.0).abs() < 1.0e-4)
        }),
        "the coincident input cap must not remain as a modeled face"
    );
    assert!(
        bodies[0].1.face_refs.iter().all(|face| {
            face.topology
                .as_ref()
                .and_then(|topology| topology.face_id.as_deref())
                .is_some()
        }),
        "the exact boolean result must keep durable face identities"
    );
}

#[test]
fn direct_face_cut_keeps_the_requested_far_plane_exact() {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "box_1".into(),
        name: "Box".into(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
    let region = attach_direct_top_face(&mut graph, "box_1", "face_sketch");
    add_direct_face_extrude(
        &mut graph,
        "cut_3",
        "face_sketch",
        Some("box_1"),
        region,
        -4.0,
        ExtrudeMode::Cut,
    );

    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("exact face cut");
    assert!(warnings.is_empty(), "warnings: {warnings:#?}");
    assert_eq!(bodies.len(), 1);
    let (min_z, max_z) = mesh_z_range(&bodies[0].1);
    assert!((min_z - 0.0).abs() < 1.0e-4);
    assert!((max_z - 6.0).abs() < 1.0e-4, "max z {max_z}");
    let volume = bodies[0]
        .1
        .mass_properties()
        .expect("cut mass properties")
        .volume;
    assert!((volume - 600.0).abs() < 0.05, "volume {volume}");
}

#[test]
fn direct_face_new_body_uses_the_exact_selected_face() {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "box_1".into(),
        name: "Box".into(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
    let region = attach_direct_top_face(&mut graph, "box_1", "face_sketch");
    add_direct_face_extrude(
        &mut graph,
        "new_3",
        "face_sketch",
        None,
        region,
        2.5,
        ExtrudeMode::NewBody,
    );

    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("exact face new body");
    assert!(warnings.is_empty(), "warnings: {warnings:#?}");
    assert_eq!(bodies.len(), 2);
    let new_body = bodies
        .iter()
        .find(|(id, _)| id == "new_3")
        .expect("new exact prism body");
    let (min_z, max_z) = mesh_z_range(&new_body.1);
    assert!((min_z - 10.0).abs() < 1.0e-4);
    assert!((max_z - 12.5).abs() < 1.0e-4);
    assert!(
        new_body.1.face_refs.iter().all(|face| face
            .topology
            .as_ref()
            .and_then(|topology| topology.face_id.as_deref())
            .is_some()),
        "new exact prism faces need durable identities"
    );
}

#[test]
fn direct_face_join_preserves_inner_wires() {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "box_1".into(),
        name: "Box".into(),
        feature: FeatureType::Box {
            w: 20.0,
            h: 20.0,
            d: 10.0,
        },
    });
    graph.add_feature(FeatureNode {
        id: "hole_2".into(),
        name: "Hole".into(),
        feature: FeatureType::Hole {
            target: "box_1".into(),
            position: [10.0, 10.0, 10.0],
            direction: [0.0, 0.0, -1.0],
            diameter: 6.0,
            diameter_expr: None,
            depth: None,
            kind: HoleKind::Simple,
            standard: None,
            manufacturing: None,
        },
    });
    graph.add_dependency("box_1", "hole_2");
    let before = graph
        .evaluate_bodies(&std::collections::HashSet::new())
        .expect("body with through hole");
    let before_volume = before[0]
        .1
        .mass_properties()
        .expect("initial holed body")
        .volume;

    let region = attach_direct_top_face(&mut graph, "box_1", "face_sketch");
    add_direct_face_extrude(
        &mut graph,
        "join_4",
        "face_sketch",
        Some("box_1"),
        region,
        2.5,
        ExtrudeMode::Join,
    );
    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("exact annular face join");
    assert!(warnings.is_empty(), "warnings: {warnings:#?}");
    let after_volume = bodies[0]
        .1
        .mass_properties()
        .expect("joined holed body")
        .volume;
    let expected_added = (400.0 - std::f64::consts::PI * 9.0) * 2.5;
    assert!(
        ((after_volume - before_volume) - expected_added).abs() < 0.5,
        "inner wire was not preserved: before={before_volume}, after={after_volume}, expected added={expected_added}"
    );
    let (_, max_z) = mesh_z_range(&bodies[0].1);
    assert!((max_z - 12.5).abs() < 1.0e-4);
}

#[test]
fn direct_face_join_preserves_a_concave_outline() {
    let mut outline = SketchCurves::new();
    let points = [
        (0.0, 0.0),
        (10.0, 0.0),
        (10.0, 4.0),
        (4.0, 4.0),
        (4.0, 10.0),
        (0.0, 10.0),
    ];
    for (&start, &end) in points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .take(points.len())
    {
        outline.add_line(start, end);
    }
    let mut graph = ParametricGraph::new();
    add_sketch(&mut graph, "base_sketch", outline);
    add_extrude(&mut graph, "base", "base_sketch", 5.0, ExtrudeMode::NewBody);
    let region = attach_direct_top_face(&mut graph, "base", "face_sketch");
    add_direct_face_extrude(
        &mut graph,
        "join_3",
        "face_sketch",
        Some("base"),
        region,
        3.0,
        ExtrudeMode::Join,
    );

    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("exact concave face join");
    assert!(warnings.is_empty(), "warnings: {warnings:#?}");
    let volume = bodies[0]
        .1
        .mass_properties()
        .expect("concave joined body")
        .volume;
    assert!((volume - 512.0).abs() < 0.1, "volume {volume}");
    let (_, max_z) = mesh_z_range(&bodies[0].1);
    assert!((max_z - 8.0).abs() < 1.0e-4);
}

#[test]
fn unnamed_ambiguous_face_reference_is_rejected() {
    let solid = crate::mock_kernel::box_solid(10.0, 10.0, 10.0);
    let mesh = MockMesh::from_solid(&solid);
    let top = mesh
        .face_refs
        .iter()
        .find(|face| face.normal[2] > 0.99)
        .expect("top face");
    let live = vec![LiveBody {
        id: "legacy_body".into(),
        parts: vec![solid.clone(), solid],
        pristine: None,
        sketch_source: None,
    }];
    let error = resolve_exact_planar_face(
        &live,
        &FaceRef {
            centroid: top.centroid,
            normal: top.normal,
            topology: Some(TopologyFaceRef {
                body_id: Some("legacy_body".into()),
                component_id: None,
                topology_version: None,
                face_id: None,
                surface_kind: Some("plane".into()),
                producer_feature_id: None,
                source_entity_id: None,
            }),
        },
        Some("legacy_body"),
    )
    .expect_err("two identical unnamed faces must be ambiguous");
    assert!(matches!(error, ExactFaceExtrudeError::Ambiguous));
}

#[test]
fn unique_unnamed_legacy_face_still_uses_exact_brep_geometry() {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "box_1".into(),
        name: "Box".into(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
    let region = attach_direct_top_face(&mut graph, "box_1", "legacy_face_sketch");
    let reference = graph
        .sketch_face_refs
        .get_mut("legacy_face_sketch")
        .expect("captured face");
    let topology = reference.topology.as_mut().expect("body topology");
    topology.face_id = None;
    topology.component_id = None;
    add_direct_face_extrude(
        &mut graph,
        "join_3",
        "legacy_face_sketch",
        Some("box_1"),
        region,
        2.0,
        ExtrudeMode::Join,
    );

    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("legacy exact face join");
    assert!(warnings.is_empty(), "warnings: {warnings:#?}");
    let (_, max_z) = mesh_z_range(&bodies[0].1);
    assert!((max_z - 12.0).abs() < 1.0e-4);
}

#[test]
fn direct_face_resolution_is_stable_at_large_world_coordinates() {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "box_1".into(),
        name: "Box".into(),
        feature: FeatureType::Box {
            w: 10.0,
            h: 10.0,
            d: 10.0,
        },
    });
    graph.add_feature(FeatureNode {
        id: "transform_2".into(),
        name: "Move".into(),
        feature: FeatureType::BodyTransform {
            source: "box_1".into(),
            translation: [1_000_000.0, -1_000_000.0, 0.0],
            copy: false,
        },
    });
    graph.add_dependency("box_1", "transform_2");
    let region = attach_direct_top_face(&mut graph, "transform_2", "face_sketch");
    add_direct_face_extrude(
        &mut graph,
        "join_4",
        "face_sketch",
        Some("transform_2"),
        region,
        1.25,
        ExtrudeMode::Join,
    );

    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .expect("large-coordinate exact face join");
    assert!(warnings.is_empty(), "warnings: {warnings:#?}");
    let (_, max_z) = mesh_z_range(&bodies[0].1);
    assert!((max_z - 11.25).abs() < 1.0e-4);
    let volume = bodies[0]
        .1
        .mass_properties()
        .expect("large-coordinate body")
        .volume;
    assert!((volume - 1_125.0).abs() < 0.1, "volume {volume}");
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
            draft_angle_deg: 0.0,
            draft_angle_expr: None,
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

#[test]
fn named_face_loss_suspends_extrude_instead_of_using_stored_outline() {
    let mut g = box_with_face_sketch(SketchCurves::new());
    g.sketch_face_refs.insert(
        "sketch_2".into(),
        FaceRef {
            centroid: [5.0, 5.0, 10.0],
            normal: [0.0, 0.0, 1.0],
            topology: Some(TopologyFaceRef {
                body_id: Some("box_1".into()),
                component_id: None,
                topology_version: Some(0),
                face_id: Some("box_box_1:face:missing".into()),
                surface_kind: Some("plane".into()),
                producer_feature_id: Some("box_1".into()),
                source_entity_id: None,
            }),
        },
    );
    add_extrude(&mut g, "extrude_3", "sketch_2", 5.0, ExtrudeMode::NewBody);

    let (bodies, warnings) = g
        .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
        .unwrap();
    assert_eq!(bodies.len(), 1, "the unresolved consumer must add no body");
    assert_eq!(bodies[0].0, "box_1");
    assert!(
        warnings
            .iter()
            .any(|warning| warning.contains("named face") && warning.contains("no longer resolves")),
        "missing durable face must be reported, got {warnings:?}"
    );
}
