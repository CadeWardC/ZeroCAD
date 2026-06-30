//! Regression for the cutout fillet "diagonal" artifact.
//!
//! The diagonal was not selectable topology. It came from cylinder-face
//! tessellation: long mixed axial/hoop chords and flat triangle normals on a
//! trimmed cylindrical cut face.

use std::collections::{BTreeMap, HashSet};

use openrcad::foundation::{Pnt, Vec as GeomVec};
use openrcad::geom::{CylindricalSurface, GeomSurface};
use zerocad_core::{
    CoordinateSystem, CornerKind, EdgeModScope, EdgeRef, ExtrudeMode, FeatureNode, FeatureType,
    MockMesh, ParametricGraph, SketchCurves, Vec3,
};

const MESH_CHORD_ERR: f64 = 0.05;

#[test]
fn cutout_fillet_cylinder_mesh_has_no_visual_diagonal() {
    let graph = cutout_fillet_repro_graph();
    let hidden = HashSet::new();

    let (bodies, warnings) = graph
        .evaluate_bodies_with_warnings(&hidden)
        .expect("evaluate repro model");
    assert!(warnings.is_empty(), "unexpected warnings: {warnings:?}");
    assert_eq!(bodies.len(), 1, "repro should produce one visible body");

    let (body_id, mesh) = &bodies[0];
    assert_eq!(
        crack_edge_count(mesh),
        0,
        "render mesh must stay closed; cracks would be drawn as stray edges"
    );

    let solids = graph.debug_kernel_solids(&hidden).expect("kernel solids");
    let (_, parts) = solids
        .iter()
        .find(|(id, _)| id == body_id)
        .unwrap_or_else(|| solids.first().expect("at least one solid body"));
    assert_eq!(parts.len(), 1, "repro body should be one solid part");
    assert!(parts[0].is_watertight(), "B-Rep should remain watertight");
    assert!(
        parts[0].health_report().is_healthy(),
        "B-Rep health check should pass"
    );

    let faces = parts[0].shell().faces();
    let mut checked_cylinders = 0usize;
    for (fid, face) in faces.iter().enumerate() {
        let Some(GeomSurface::Cylinder(cyl)) = face.surface() else {
            continue;
        };
        checked_cylinders += 1;

        let stats = cylinder_face_stats(mesh, fid as u32, *cyl);
        assert!(
            stats.triangles > 0,
            "cylinder face {fid} should have render triangles"
        );
        assert!(
            stats.max_axis_normal_dot <= 1.0e-3,
            "cylinder face {fid} normals are not radial enough: max |axis dot normal| = {}",
            stats.max_axis_normal_dot
        );
        assert!(
            stats.min_consistent_radial_alignment() >= 0.90,
            "cylinder face {fid} normals flip relative to one radial side: min signed radial alignment = {}",
            stats.min_consistent_radial_alignment()
        );
        assert!(
            stats.max_mixed_axial_hoop <= 2.05,
            "cylinder face {fid} still has a long mixed axial/hoop chord: {}",
            stats.max_mixed_axial_hoop
        );
        let max_hoop = cylinder_target_len(cyl.radius()) * 1.35;
        assert!(
            stats.max_hoop_edge <= max_hoop,
            "cylinder face {fid} still has a long pure-hoop chord: {} > {}",
            stats.max_hoop_edge,
            max_hoop
        );
        let max_surface = cylinder_target_len(cyl.radius()) * 1.35;
        assert!(
            stats.max_non_axial_surface_edge <= max_surface,
            "cylinder face {fid} still has an oversized non-axial cylinder-space edge: {} > {} (hoop {}, axial {})",
            stats.max_non_axial_surface_edge,
            max_surface,
            stats.max_non_axial_surface_edge_hoop,
            stats.max_non_axial_surface_edge_axial
        );
        assert!(
            stats.max_edge_sagitta <= MESH_CHORD_ERR * 1.25,
            "cylinder face {fid} has an edge whose cylinder sagitta exceeds tolerance: {}",
            stats.max_edge_sagitta
        );
    }

    assert!(
        checked_cylinders >= 2,
        "repro should include the cut cylinder and the fillet cylinder"
    );
}

fn cutout_fillet_repro_graph() -> ParametricGraph {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "box_1".into(),
        name: "Box".into(),
        feature: FeatureType::Box {
            w: 40.0,
            h: 30.0,
            d: 15.0,
        },
    });

    let mut cut = SketchCurves::new();
    cut.add_circle((20.0, 15.0), 6.0);
    graph.add_feature(FeatureNode {
        id: "sketch_2".into(),
        name: "Cut circle".into(),
        feature: FeatureType::Sketch {
            cs: top_plane(15.0),
            curves: cut,
            shapes: vec![],
            corner_mods: vec![],
            on_face: true,
        },
    });
    graph.add_feature(FeatureNode {
        id: "extrude_3".into(),
        name: "Through cut".into(),
        feature: FeatureType::Extrude {
            depth: -20.0,
            region_indices: vec![],
            mode: ExtrudeMode::Cut,
            depth_expr: None,
        },
    });
    graph.add_dependency("sketch_2", "extrude_3");
    graph.add_dependency("box_1", "extrude_3");

    graph.add_feature(FeatureNode {
        id: "edgemod_4".into(),
        name: "Front top fillet".into(),
        feature: FeatureType::EdgeMod {
            target: "box_1".into(),
            edge: EdgeRef {
                p0: [0.0, 0.0, 15.0],
                p1: [40.0, 0.0, 15.0],
                n1: [0.0, 0.0, 1.0],
                n2: [0.0, -1.0, 0.0],
                curve: None,
                topology: None,
            },
            dist: 3.0,
            dist_expr: None,
            scope: EdgeModScope::FullEdge,
            replay: Default::default(),
            kind: CornerKind::Fillet,
        },
    });
    graph.add_dependency("extrude_3", "edgemod_4");
    graph
}

fn top_plane(z: f32) -> CoordinateSystem {
    CoordinateSystem::new(
        Vec3::new(0.0, 0.0, z),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
}

fn crack_edge_count(mesh: &MockMesh) -> usize {
    let q = |v: f64| (v * 10_000.0).round() as i64;
    let key = |vi: u32| -> (i64, i64, i64) {
        let p = pos(mesh, vi);
        (q(p.x()), q(p.y()), q(p.z()))
    };

    let mut counts: BTreeMap<MeshEdgeKey, u32> = BTreeMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            let ka = key(tri[a]);
            let kb = key(tri[b]);
            let edge = if ka <= kb { (ka, kb) } else { (kb, ka) };
            *counts.entry(edge).or_insert(0) += 1;
        }
    }

    counts.values().filter(|&&count| count == 1).count()
}

struct CylinderStats {
    triangles: usize,
    max_axis_normal_dot: f64,
    max_mixed_axial_hoop: f64,
    max_hoop_edge: f64,
    max_non_axial_surface_edge: f64,
    max_non_axial_surface_edge_hoop: f64,
    max_non_axial_surface_edge_axial: f64,
    max_edge_sagitta: f64,
    radial_samples: usize,
    radial_alignment_sum: f64,
    min_radial_alignment: f64,
    max_radial_alignment: f64,
}

impl Default for CylinderStats {
    fn default() -> Self {
        Self {
            triangles: 0,
            max_axis_normal_dot: 0.0,
            max_mixed_axial_hoop: 0.0,
            max_hoop_edge: 0.0,
            max_non_axial_surface_edge: 0.0,
            max_non_axial_surface_edge_hoop: 0.0,
            max_non_axial_surface_edge_axial: 0.0,
            max_edge_sagitta: 0.0,
            radial_samples: 0,
            radial_alignment_sum: 0.0,
            min_radial_alignment: f64::INFINITY,
            max_radial_alignment: f64::NEG_INFINITY,
        }
    }
}

impl CylinderStats {
    fn min_consistent_radial_alignment(&self) -> f64 {
        if self.radial_samples == 0 {
            return 1.0;
        }
        if self.radial_alignment_sum >= 0.0 {
            self.min_radial_alignment
        } else {
            -self.max_radial_alignment
        }
    }
}

fn cylinder_face_stats(mesh: &MockMesh, fid: u32, cyl: CylindricalSurface) -> CylinderStats {
    let axis = GeomVec::from_dir(cyl.position().direction());
    let edge_faces = mesh_edge_faces(mesh);
    let mut stats = CylinderStats::default();

    for (t, tri) in mesh.indices.chunks_exact(3).enumerate() {
        if mesh.face_ids.get(t).copied() != Some(fid) {
            continue;
        }

        stats.triangles += 1;
        for &vi in tri {
            let p = pos(mesh, vi);
            let n = normal(mesh, vi);
            stats.max_axis_normal_dot = stats.max_axis_normal_dot.max((n.dot(&axis)).abs());
            if let Some(radial) = cylinder_radial(cyl, p) {
                let alignment = n.dot(&radial);
                stats.radial_samples += 1;
                stats.radial_alignment_sum += alignment;
                stats.min_radial_alignment = stats.min_radial_alignment.min(alignment);
                stats.max_radial_alignment = stats.max_radial_alignment.max(alignment);
            }
        }

        for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            let edge_key = mesh_edge_key(mesh, tri[a], tri[b]);
            if edge_faces
                .get(&edge_key)
                .is_some_and(|faces| faces.iter().any(|&edge_fid| edge_fid != fid))
            {
                continue;
            }
            let ua = cylinder_uv(cyl, pos(mesh, tri[a]));
            let ub = cylinder_uv(cyl, pos(mesh, tri[b]));
            let du = shortest_angle_delta(ua.0, ub.0);
            let hoop = du * cyl.radius();
            let axial = (ua.1 - ub.1).abs();
            let surface = hoop.hypot(axial);
            let sagitta = cyl.radius() * (1.0 - (0.5 * du).cos());
            stats.max_mixed_axial_hoop = stats.max_mixed_axial_hoop.max(hoop.min(axial));
            stats.max_hoop_edge = stats.max_hoop_edge.max(hoop);
            if hoop > MESH_CHORD_ERR && surface > stats.max_non_axial_surface_edge {
                stats.max_non_axial_surface_edge = surface;
                stats.max_non_axial_surface_edge_hoop = hoop;
                stats.max_non_axial_surface_edge_axial = axial;
            }
            stats.max_edge_sagitta = stats.max_edge_sagitta.max(sagitta);
        }
    }

    stats
}

type MeshEdgeKey = ((i64, i64, i64), (i64, i64, i64));

fn mesh_edge_faces(mesh: &MockMesh) -> BTreeMap<MeshEdgeKey, HashSet<u32>> {
    let mut edge_faces: BTreeMap<MeshEdgeKey, HashSet<u32>> = BTreeMap::new();
    for (t, tri) in mesh.indices.chunks_exact(3).enumerate() {
        let fid = mesh.face_ids.get(t).copied().unwrap_or(0);
        for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            edge_faces
                .entry(mesh_edge_key(mesh, tri[a], tri[b]))
                .or_default()
                .insert(fid);
        }
    }
    edge_faces
}

fn mesh_edge_key(mesh: &MockMesh, a: u32, b: u32) -> MeshEdgeKey {
    let q = |v: f64| (v * 10_000.0).round() as i64;
    let key = |vi: u32| -> (i64, i64, i64) {
        let p = pos(mesh, vi);
        (q(p.x()), q(p.y()), q(p.z()))
    };
    let ka = key(a);
    let kb = key(b);
    if ka <= kb {
        (ka, kb)
    } else {
        (kb, ka)
    }
}

fn pos(mesh: &MockMesh, vi: u32) -> Pnt {
    let b = vi as usize * 6;
    Pnt::new(
        mesh.vertices[b] as f64,
        mesh.vertices[b + 1] as f64,
        mesh.vertices[b + 2] as f64,
    )
}

fn normal(mesh: &MockMesh, vi: u32) -> GeomVec {
    let b = vi as usize * 6 + 3;
    GeomVec::new(
        mesh.vertices[b] as f64,
        mesh.vertices[b + 1] as f64,
        mesh.vertices[b + 2] as f64,
    )
}

fn cylinder_radial(cyl: CylindricalSurface, p: Pnt) -> Option<GeomVec> {
    let axis = GeomVec::from_dir(cyl.position().direction());
    let diff = p - cyl.position().location();
    let radial = diff.subtracted(&axis.multiplied(diff.dot(&axis)));
    let len = radial.magnitude();
    if len <= 1.0e-12 {
        None
    } else {
        Some(radial / len)
    }
}

fn cylinder_uv(cyl: CylindricalSurface, p: Pnt) -> (f64, f64) {
    let diff = p - cyl.position().location();
    let x = diff.dot(&GeomVec::from_dir(cyl.position().x_direction()));
    let y = diff.dot(&GeomVec::from_dir(cyl.position().y_direction()));
    let v = diff.dot(&GeomVec::from_dir(cyl.position().direction()));
    let mut u = y.atan2(x);
    if u < 0.0 {
        u += std::f64::consts::TAU;
    }
    (u, v)
}

fn cylinder_target_len(radius: f64) -> f64 {
    let theta = if radius > 1.0e-9 {
        2.0 * (2.0 * MESH_CHORD_ERR / radius).sqrt()
    } else {
        std::f64::consts::FRAC_PI_2
    };
    (radius * theta).max(MESH_CHORD_ERR)
}

fn shortest_angle_delta(a: f64, b: f64) -> f64 {
    let mut d = (a - b).abs();
    while d > std::f64::consts::TAU {
        d -= std::f64::consts::TAU;
    }
    if d > std::f64::consts::PI {
        std::f64::consts::TAU - d
    } else {
        d
    }
}
