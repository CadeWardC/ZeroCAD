//! zerocad-core plumbing for the analytic circular-rim fillet/chamfer (WI-6):
//! a `Circle` edge hint must resolve the rim's arc edges on the kernel solid and
//! route through the analytic torus (fillet) / cone (chamfer) chain solver.

use openrcad::geom::GeomSurface;
use std::collections::HashSet;
use zerocad_core::mock_kernel::{
    chamfer_edge_with_hint, cylinder_solid, fillet_edge_with_hint, EdgeCurveHint, KernelSolid,
};
use zerocad_core::{
    detect_regions, CoordinateSystem, EdgeModReplayIntent, EdgeRef, ExtrudeMode, FeatureNode,
    FeatureType, ParametricGraph, SketchCurves, Vec3,
};

/// The cylinder primitive is built about the +Y axis, so its top rim sits at
/// `(0, h, 0)` with axis `+Y`.
fn top_rim_hint(r: f32, h: f32) -> EdgeCurveHint {
    EdgeCurveHint::Circle {
        center: [0.0, h, 0.0],
        axis: [0.0, 1.0, 0.0],
        x_dir: [1.0, 0.0, 0.0],
        radius: r,
        start: 0.0,
        end: std::f32::consts::TAU,
        closed: true,
    }
}

fn count_surface(solid: &KernelSolid, pred: impl Fn(&GeomSurface) -> bool) -> usize {
    solid
        .shell()
        .faces()
        .iter()
        .filter(|f| f.surface().map(&pred).unwrap_or(false))
        .count()
}

#[test]
fn cylinder_rim_fillet_via_circle_hint_makes_a_torus() {
    let solid = cylinder_solid(4.0, 10.0).expect("cylinder primitive");
    let hint = top_rim_hint(4.0, 10.0);
    // p0/p1 are ignored when a Circle hint is present (the chain is resolved from
    // the hint), so any rim points suffice.
    let out = fillet_edge_with_hint(
        &solid,
        [4.0, 10.0, 0.0],
        [-4.0, 10.0, 0.0],
        Some(&hint),
        1.0,
    )
    .expect("cylinder rim fillet via hint should succeed");
    assert!(out.is_watertight(), "filleted rim must be watertight");
    let tori = count_surface(&out, |s| matches!(s, GeomSurface::Torus(_)));
    assert!(tori >= 3, "rim fillet must add a torus band, got {tori}");
}

#[test]
fn cylinder_rim_chamfer_via_circle_hint_makes_a_cone() {
    let solid = cylinder_solid(4.0, 10.0).expect("cylinder primitive");
    let hint = top_rim_hint(4.0, 10.0);
    let out = chamfer_edge_with_hint(
        &solid,
        [4.0, 10.0, 0.0],
        [-4.0, 10.0, 0.0],
        Some(&hint),
        1.0,
    )
    .expect("cylinder rim chamfer via hint should succeed");
    assert!(out.is_watertight(), "chamfered rim must be watertight");
    let cones = count_surface(&out, |s| matches!(s, GeomSurface::Cone(_)));
    assert!(cones >= 3, "rim chamfer must add a cone band, got {cones}");
}

#[test]
fn slightly_off_fitted_radius_still_matches_the_rim() {
    // The GUI circle-fit hint is ~1%-of-radius accurate; a 0.5%-off radius must
    // still resolve to the rim (belt-and-braces tolerance widening).
    let solid = cylinder_solid(4.0, 10.0).expect("cylinder primitive");
    let mut hint = top_rim_hint(4.0, 10.0);
    if let EdgeCurveHint::Circle { radius, .. } = &mut hint {
        *radius = 4.0 * 1.005; // 0.5% high
    }
    let out = fillet_edge_with_hint(
        &solid,
        [4.0, 10.0, 0.0],
        [-4.0, 10.0, 0.0],
        Some(&hint),
        1.0,
    )
    .expect("a slightly-off fitted radius should still match the rim");
    assert!(out.is_watertight());
}

/// End-to-end through the parametric graph AND the edge-mod acceptance gates: a
/// rim EdgeMod on a cylinder body should commit (no warning) and change the
/// display mesh. (Unblocked by the analytic torus `project_point` and the
/// midpoint-anchored `spine_param_span` in the kernel: a rim torus/cone band's
/// display mesh is now crack-free, so the strict acceptance gate passes.)
fn rim_edge_mod_commits(kind: zerocad_core::CornerKind, label: &str) {
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "cyl".to_string(),
        name: "Cylinder".to_string(),
        feature: FeatureType::Cylinder { r: 4.0, h: 10.0 },
    });
    let unmodified = g.evaluate_bodies(&HashSet::new()).unwrap();
    let base_tris = unmodified[0].1.indices.len();

    let rim = EdgeRef {
        p0: [4.0, 10.0, 0.0],
        p1: [0.0, 10.0, 4.0],
        n1: [0.0, 1.0, 0.0],
        n2: [1.0, 0.0, 0.0],
        // A GUI-shaped closed rim hint, deliberately ~0.5% off the fitted radius.
        curve: Some(EdgeCurveHint::Circle {
            center: [0.0, 10.0, 0.0],
            axis: [0.0, 1.0, 0.0],
            x_dir: [1.0, 0.0, 0.0],
            radius: 4.0 * 1.005,
            start: 0.0,
            end: std::f32::consts::TAU,
            closed: true,
        }),
        topology: None,
    };
    g.add_feature(FeatureNode {
        id: "em".to_string(),
        name: "Edge Mod".to_string(),
        feature: FeatureType::EdgeMod {
            target: "cyl".to_string(),
            edge: rim,
            dist: 1.0,
            dist_expr: None,
            replay: EdgeModReplayIntent::default(),
            kind,
        },
    });
    g.add_dependency("cyl", "em");

    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert!(
        warnings.iter().all(|w| !w.contains("couldn't be")),
        "{label}: rim edge-mod should commit, got {warnings:?}"
    );
    assert_eq!(bodies.len(), 1, "{label}: keeps one body");
    assert_ne!(
        bodies[0].1.indices.len(),
        base_tris,
        "{label}: the rim edge-mod must change the display mesh"
    );
}

#[test]
fn rim_fillet_commits_through_the_graph() {
    rim_edge_mod_commits(zerocad_core::CornerKind::Fillet, "fillet");
}

#[test]
fn rim_chamfer_commits_through_the_graph() {
    rim_edge_mod_commits(zerocad_core::CornerKind::Chamfer, "chamfer");
}

/// The rim edge-mod must reach the native analytic solver (no app-level "not
/// supported" gate) and, if any acceptance gate ever rejects it again, degrade
/// *safely* — the body is left unchanged rather than shipping broken geometry.
#[test]
fn rim_edge_mod_reaches_native_solver_and_degrades_safely() {
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "cyl".to_string(),
        name: "Cylinder".to_string(),
        feature: FeatureType::Cylinder { r: 4.0, h: 10.0 },
    });
    let unmodified = g.evaluate_bodies(&HashSet::new()).unwrap();
    let base_tris = unmodified[0].1.indices.len();

    let rim = EdgeRef {
        p0: [4.0, 10.0, 0.0],
        p1: [0.0, 10.0, 4.0],
        n1: [0.0, 1.0, 0.0],
        n2: [1.0, 0.0, 0.0],
        curve: Some(EdgeCurveHint::Circle {
            center: [0.0, 10.0, 0.0],
            axis: [0.0, 1.0, 0.0],
            x_dir: [1.0, 0.0, 0.0],
            radius: 4.0,
            start: 0.0,
            end: std::f32::consts::TAU,
            closed: true,
        }),
        topology: None,
    };
    g.add_feature(FeatureNode {
        id: "em".to_string(),
        name: "Edge Mod".to_string(),
        feature: FeatureType::EdgeMod {
            target: "cyl".to_string(),
            edge: rim,
            dist: 1.0,
            dist_expr: None,
            replay: EdgeModReplayIntent::default(),
            kind: zerocad_core::CornerKind::Fillet,
        },
    });
    g.add_dependency("cyl", "em");

    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert_eq!(bodies.len(), 1, "keeps one body");
    // The native analytic solver was reached (no app-level "not supported" gate),
    // and the only remaining reason is the display-mesh crack check.
    assert!(
        warnings
            .iter()
            .all(|w| !w.contains("not supported") && !w.contains("not supported yet")),
        "rim fillet must reach the native solver, got {warnings:?}"
    );
    if !warnings.is_empty() {
        // Currently blocked at the display gate → body unchanged (safe).
        assert!(
            warnings.iter().any(|w| w.contains("crack")),
            "the only expected block is the display-mesh crack gate, got {warnings:?}"
        );
        assert_eq!(
            bodies[0].1.indices.len(),
            base_tris,
            "a rejected rim fillet must leave the body unchanged"
        );
    }
}

/// A box with a cylindrical bite through its full height (the GUI screenshot
/// case). The bite's top rim is an OPEN co-circular chain; a GUI-shaped OPEN
/// `Circle` hint (`closed: false`, a start..end angular span crossing the
/// circle's 0/TAU seam) must resolve exactly those arcs and route through the
/// analytic open-chain solver with flush end trims.
fn bitten_box() -> KernelSolid {
    use openrcad::foundation::{Ax2, Dir, Pnt};
    let block = openrcad::primitives::make_box(&Pnt::new(0.0, 5.0, 0.0), 40.0, 30.0, 10.0);
    let drill = openrcad::primitives::make_cylinder(
        &Ax2::new(Pnt::new(20.0, 8.0, -1.0), Dir::dz()),
        14.0,
        12.0,
    );
    openrcad::algo::boolean_checked(&block, &drill, openrcad::algo::BooleanOp::Cut)
        .expect("bite cut should be clean")
}

fn bite_rim_hint() -> EdgeCurveHint {
    // The bite spans roughly 6.07..TAU+3.36 in the rim circle's own frame; a
    // slightly generous fitted span (like the GUI's) still excludes everything
    // off the arc.
    EdgeCurveHint::Circle {
        center: [20.0, 8.0, 10.0],
        axis: [0.0, 0.0, 1.0],
        x_dir: [1.0, 0.0, 0.0],
        radius: 14.0,
        start: 6.0,
        end: std::f32::consts::TAU + 3.4,
        closed: false,
    }
}

#[test]
fn bite_arc_fillet_via_open_circle_hint_makes_a_torus() {
    let solid = bitten_box();
    let hint = bite_rim_hint();
    let out = fillet_edge_with_hint(
        &solid,
        [6.33, 5.0, 10.0],
        [33.67, 5.0, 10.0],
        Some(&hint),
        1.5,
    )
    .expect("bite arc fillet via open hint should succeed");
    assert!(out.is_watertight(), "bite arc fillet must be watertight");
    let tori = count_surface(&out, |s| matches!(s, GeomSurface::Torus(_)));
    assert!(tori >= 1, "bite fillet must add a torus band, got {tori}");
}

#[test]
fn bite_arc_chamfer_via_open_circle_hint_makes_a_cone() {
    let solid = bitten_box();
    let hint = bite_rim_hint();
    let out = chamfer_edge_with_hint(
        &solid,
        [6.33, 5.0, 10.0],
        [33.67, 5.0, 10.0],
        Some(&hint),
        1.5,
    )
    .expect("bite arc chamfer via open hint should succeed");
    assert!(out.is_watertight(), "bite arc chamfer must be watertight");
    let cones = count_surface(&out, |s| matches!(s, GeomSurface::Cone(_)));
    assert!(cones >= 1, "bite chamfer must add a cone band, got {cones}");
}

/// Face ids of every display-mesh triangle whose centroid satisfies `on_band`.
fn band_face_ids(
    mesh: &zerocad_core::MockMesh,
    on_band: &dyn Fn(f64, f64, f64) -> bool,
) -> HashSet<u32> {
    let mut ids = HashSet::new();
    for (t, tri) in mesh.indices.chunks_exact(3).enumerate() {
        let mut c = [0.0f64; 3];
        for &v in tri {
            let b = v as usize * 6;
            c[0] += mesh.vertices[b] as f64 / 3.0;
            c[1] += mesh.vertices[b + 1] as f64 / 3.0;
            c[2] += mesh.vertices[b + 2] as f64 / 3.0;
        }
        if on_band(c[0], c[1], c[2]) {
            ids.insert(mesh.face_ids[t]);
        }
    }
    ids
}

/// Wireframe segments whose midpoint satisfies `interior` — used to prove no
/// seam is drawn across a band's interior (its boundary rings are excluded by
/// the predicate).
fn interior_seam_segments(
    mesh: &zerocad_core::MockMesh,
    interior: &dyn Fn(f64, f64, f64) -> bool,
) -> usize {
    mesh.edge_indices
        .chunks_exact(2)
        .filter(|pair| {
            let p = |i: u32| {
                let b = i as usize * 3;
                [
                    mesh.edge_vertices[b] as f64,
                    mesh.edge_vertices[b + 1] as f64,
                    mesh.edge_vertices[b + 2] as f64,
                ]
            };
            let a = p(pair[0]);
            let b = p(pair[1]);
            interior(
                (a[0] + b[0]) * 0.5,
                (a[1] + b[1]) * 0.5,
                (a[2] + b[2]) * 0.5,
            )
        })
        .count()
}

/// GUI round 3: the rim fillet band rendered SEGMENTED — the kernel emits one
/// torus sector per rim fragment (a `make_cylinder` rim is 3 arcs), and each
/// sector drew its own radial seams and selected separately. The display mesh
/// must present the band as ONE face with no interior seams.
#[test]
fn rim_fillet_band_reads_as_one_face_with_no_sector_seams() {
    let solid = cylinder_solid(4.0, 10.0).expect("cylinder primitive");
    let hint = top_rim_hint(4.0, 10.0);
    let out = fillet_edge_with_hint(
        &solid,
        [4.0, 10.0, 0.0],
        [-4.0, 10.0, 0.0],
        Some(&hint),
        1.0,
    )
    .expect("cylinder rim fillet via hint should succeed");
    assert!(
        count_surface(&out, |s| matches!(s, GeomSurface::Torus(_))) >= 2,
        "the premise of this test is a band split into several torus sectors"
    );
    let mesh = zerocad_core::MockMesh::from_solid(&out);
    // Fillet torus: axis +Y through the origin, tube circle radius 3 at y = 9,
    // tube radius 1.
    let on_torus = |x: f64, y: f64, z: f64, tol: f64| {
        let rho = x.hypot(z);
        (((rho - 3.0).hypot(y - 9.0)) - 1.0).abs() < tol
    };
    // Strict interior — clear of the tangent contact rings, where the loose
    // torus-distance test would also catch top-cap/wall triangles.
    let ids = band_face_ids(&mesh, &|x, y, z| {
        let rho = x.hypot(z);
        on_torus(x, y, z, 0.08) && y < 9.9 && rho < 3.9
    });
    assert!(!ids.is_empty(), "band triangles must exist");
    assert_eq!(
        ids.len(),
        1,
        "the fillet band must select as ONE face, got {ids:?}"
    );
    // No wireframe segment across the band interior (the sector seams): the
    // band's real boundary rings sit at y = 10 and y = 9.
    let seams = interior_seam_segments(&mesh, &|x, y, z| {
        on_torus(x, y, z, 0.02) && y > 9.1 && y < 9.9
    });
    assert_eq!(
        seams, 0,
        "no radial sector seam may be drawn across the band"
    );
}

/// The chamfer analogue: the cone band must read as one face with no seams.
#[test]
fn rim_chamfer_band_reads_as_one_face_with_no_sector_seams() {
    let solid = cylinder_solid(4.0, 10.0).expect("cylinder primitive");
    let hint = top_rim_hint(4.0, 10.0);
    let out = chamfer_edge_with_hint(
        &solid,
        [4.0, 10.0, 0.0],
        [-4.0, 10.0, 0.0],
        Some(&hint),
        1.0,
    )
    .expect("cylinder rim chamfer via hint should succeed");
    assert!(
        count_surface(&out, |s| matches!(s, GeomSurface::Cone(_))) >= 2,
        "the premise of this test is a band split into several cone sectors"
    );
    let mesh = zerocad_core::MockMesh::from_solid(&out);
    // 45° chamfer cone from (rho=3, y=10) to (rho=4, y=9): rho = 13 - y.
    let on_cone = |x: f64, y: f64, z: f64, tol: f64| {
        let rho = x.hypot(z);
        (rho - (13.0 - y)).abs() < tol && (8.9..=10.1).contains(&y)
    };
    let ids = band_face_ids(&mesh, &|x, y, z| on_cone(x, y, z, 0.1));
    assert!(!ids.is_empty(), "band triangles must exist");
    assert_eq!(
        ids.len(),
        1,
        "the chamfer band must select as ONE face, got {ids:?}"
    );
    let seams = interior_seam_segments(&mesh, &|x, y, z| {
        on_cone(x, y, z, 0.02) && y > 9.1 && y < 9.9
    });
    assert_eq!(
        seams, 0,
        "no radial sector seam may be drawn across the band"
    );
}

/// The open bite-arc band (GUI round-3 picture 2): the boolean splits the rim
/// arc into fragments, so the band arrives as several co-toroidal sectors with
/// a seam mid-arc. It too must read as one face with no interior seams (the
/// flush END sections at the cap plane y = 5 are real edges and are excluded).
#[test]
fn bite_fillet_band_reads_as_one_face_with_no_mid_arc_seam() {
    let solid = bitten_box();
    let hint = bite_rim_hint();
    let out = fillet_edge_with_hint(
        &solid,
        [6.33, 5.0, 10.0],
        [33.67, 5.0, 10.0],
        Some(&hint),
        1.5,
    )
    .expect("bite arc fillet via open hint should succeed");
    let mesh = zerocad_core::MockMesh::from_solid(&out);
    // Concave fillet torus: axis +Z through (20, 8), tube circle radius 15.5 at
    // z = 8.5, tube radius 1.5.
    let on_torus = |x: f64, y: f64, z: f64, tol: f64| {
        let rho = (x - 20.0).hypot(y - 8.0);
        (((rho - 15.5).hypot(z - 8.5)) - 1.5).abs() < tol
    };
    // Strict interior — clear of the tangent contact rings (top plane z = 10,
    // bite wall rho = 14) and of the flush ends at the y = 5 cap.
    let ids = band_face_ids(&mesh, &|x, y, z| {
        let rho = (x - 20.0).hypot(y - 8.0);
        on_torus(x, y, z, 0.12) && z < 9.85 && rho > 14.15 && y > 5.5
    });
    assert!(!ids.is_empty(), "band triangles must exist");
    assert_eq!(
        ids.len(),
        1,
        "the bite fillet band must select as ONE face, got {ids:?}"
    );
    let seams = interior_seam_segments(&mesh, &|x, y, z| {
        // Interior of the band, clear of the top/wall contact rings AND of the
        // flush end sections against the y = 5 cap.
        on_torus(x, y, z, 0.02) && z > 8.65 && z < 9.85 && y > 5.5
    });
    assert_eq!(
        seams, 0,
        "no seam may be drawn across the bite band interior"
    );
}

/// End-to-end through the parametric graph: a sketch-extruded box with a
/// circular bite, then an EdgeMod on the bite's OPEN top arc (the GUI's exact
/// selection shape: an open `Circle` hint). It must COMMIT — no "couldn't be"
/// warning — and change the display mesh.
fn bite_arc_edge_mod_commits(kind: zerocad_core::CornerKind, label: &str) {
    let mut curves = SketchCurves::new();
    curves.add_rectangle((0.0, 5.0), (40.0, 35.0));
    curves.add_circle((20.0, 8.0), 14.0);
    let regions = detect_regions(&curves);
    let region = regions
        .iter()
        .position(|r| r.contains((5.0, 30.0)) && !r.contains((20.0, 8.0)))
        .expect("material region above the circular bite");

    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "s".into(),
        name: "S".into(),
        feature: FeatureType::Sketch {
            entity_ids: vec![],
            next_entity_id: 0,
            solver: None,
            cs: CoordinateSystem::new(
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            ),
            curves,
            shapes: vec![],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
        },
    });
    g.add_feature(FeatureNode {
        id: "e".into(),
        name: "E".into(),
        feature: FeatureType::Extrude {
            target: None,
            depth: 10.0,
            region_indices: vec![region],
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
        },
    });
    g.add_dependency("s", "e");
    let unmodified = g.evaluate_bodies(&HashSet::new()).unwrap();
    let base_tris = unmodified[0].1.indices.len();

    // The bite arc on the top face (z = 10): ends at x = 20 ∓ √(14² − 3²).
    let half = (14.0f32 * 14.0 - 3.0 * 3.0).sqrt();
    let arc = EdgeRef {
        p0: [20.0 - half, 5.0, 10.0],
        p1: [20.0 + half, 5.0, 10.0],
        n1: [0.0, 0.0, 1.0],
        n2: [0.0, -1.0, 0.0],
        curve: Some(EdgeCurveHint::Circle {
            center: [20.0, 8.0, 10.0],
            axis: [0.0, 0.0, 1.0],
            x_dir: [1.0, 0.0, 0.0],
            radius: 14.0,
            start: 6.0,
            end: std::f32::consts::TAU + 3.4,
            closed: false,
        }),
        topology: None,
    };
    g.add_feature(FeatureNode {
        id: "em".into(),
        name: "Edge Mod".into(),
        feature: FeatureType::EdgeMod {
            target: "e".into(),
            edge: arc,
            dist: 1.5,
            dist_expr: None,
            replay: EdgeModReplayIntent::default(),
            kind,
        },
    });
    g.add_dependency("e", "em");

    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert!(
        warnings.iter().all(|w| !w.contains("couldn't be")),
        "{label}: bite-arc edge-mod should commit, got {warnings:?}"
    );
    assert_eq!(bodies.len(), 1, "{label}: keeps one body");
    assert_ne!(
        bodies[0].1.indices.len(),
        base_tris,
        "{label}: the bite-arc edge-mod must change the display mesh"
    );
}

#[test]
fn bite_arc_fillet_commits_through_the_graph() {
    bite_arc_edge_mod_commits(zerocad_core::CornerKind::Fillet, "fillet");
}

#[test]
fn bite_arc_chamfer_commits_through_the_graph() {
    bite_arc_edge_mod_commits(zerocad_core::CornerKind::Chamfer, "chamfer");
}

/// Shared graph builder: sketch `curves` on the XY plane, extrude `region_pick`
/// to depth 10, then EdgeMod the given edge. Returns (bodies, warnings,
/// base_tris).
fn sketch_extrude_edge_mod(
    curves: SketchCurves,
    region_pick: impl Fn(&zerocad_core::Region) -> bool,
    edge: EdgeRef,
    kind: zerocad_core::CornerKind,
    dist: f32,
) -> (Vec<(String, zerocad_core::MockMesh)>, Vec<String>, usize) {
    let regions = detect_regions(&curves);
    let region = regions
        .iter()
        .position(region_pick)
        .expect("expected sketch region");
    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "s".into(),
        name: "S".into(),
        feature: FeatureType::Sketch {
            entity_ids: vec![],
            next_entity_id: 0,
            solver: None,
            cs: CoordinateSystem::new(
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            ),
            curves,
            shapes: vec![],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
        },
    });
    g.add_feature(FeatureNode {
        id: "e".into(),
        name: "E".into(),
        feature: FeatureType::Extrude {
            target: None,
            depth: 10.0,
            region_indices: vec![region],
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
        },
    });
    g.add_dependency("s", "e");
    let unmodified = g.evaluate_bodies(&HashSet::new()).unwrap();
    let base_tris = unmodified[0].1.indices.len();
    g.add_feature(FeatureNode {
        id: "em".into(),
        name: "Edge Mod".into(),
        feature: FeatureType::EdgeMod {
            target: "e".into(),
            edge,
            dist,
            dist_expr: None,
            replay: EdgeModReplayIntent::default(),
            kind,
        },
    });
    g.add_dependency("e", "em");
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    (bodies, warnings, base_tris)
}

fn assert_committed(
    label: &str,
    bodies: &[(String, zerocad_core::MockMesh)],
    warnings: &[String],
    base_tris: usize,
) {
    assert!(
        warnings.iter().all(|w| !w.contains("couldn't be")),
        "{label}: edge-mod should commit, got {warnings:?}"
    );
    assert_eq!(bodies.len(), 1, "{label}: keeps one body");
    assert_ne!(
        bodies[0].1.indices.len(),
        base_tris,
        "{label}: the edge-mod must change the display mesh"
    );
}

/// GUI regression: the hint's arc may be encoded with `end` BELOW `start` (a
/// negative raw span — the fitted polyline ran the other way). The presence
/// gate must measure the same fragment, not its complement ("covered only 0.00
/// rad of the selected 4.71 rad arc").
#[test]
fn bite_arc_fillet_commits_with_reversed_hint_span() {
    let mut curves = SketchCurves::new();
    curves.add_rectangle((0.0, 5.0), (40.0, 35.0));
    curves.add_circle((20.0, 8.0), 14.0);
    let half = (14.0f32 * 14.0 - 3.0 * 3.0).sqrt();
    let arc = EdgeRef {
        p0: [20.0 + half, 5.0, 10.0],
        p1: [20.0 - half, 5.0, 10.0],
        n1: [0.0, 0.0, 1.0],
        n2: [0.0, -1.0, 0.0],
        // Same arc as `bite_arc_edge_mod_commits`, encoded in the OPPOSITE
        // direction: from just past the seam back down across it.
        curve: Some(EdgeCurveHint::Circle {
            center: [20.0, 8.0, 10.0],
            axis: [0.0, 0.0, 1.0],
            x_dir: [1.0, 0.0, 0.0],
            radius: 14.0,
            start: 3.4,
            end: 3.4 - 3.7,
            closed: false,
        }),
        topology: None,
    };
    let (bodies, warnings, base_tris) = sketch_extrude_edge_mod(
        curves,
        |r| r.contains((5.0, 30.0)) && !r.contains((20.0, 8.0)),
        arc,
        zerocad_core::CornerKind::Fillet,
        1.5,
    );
    assert_committed("reversed-span bite fillet", &bodies, &warnings, base_tris);
}

/// GUI regression: a CORNER bite (the circle removes a rectangle corner, so the
/// arc's two free ends run out onto two DIFFERENT side faces).
#[test]
fn corner_bite_arc_fillet_commits_through_the_graph() {
    let mut curves = SketchCurves::new();
    curves.add_rectangle((0.0, 0.0), (30.0, 30.0));
    curves.add_circle((30.0, 0.0), 10.0);
    // Quarter arc on the top face from (30, 10, 10) around to (20, 0, 10).
    let arc = EdgeRef {
        p0: [30.0, 10.0, 10.0],
        p1: [20.0, 0.0, 10.0],
        n1: [0.0, 0.0, 1.0],
        n2: [1.0, 0.0, 0.0],
        curve: Some(EdgeCurveHint::Circle {
            center: [30.0, 0.0, 10.0],
            axis: [0.0, 0.0, 1.0],
            x_dir: [1.0, 0.0, 0.0],
            radius: 10.0,
            start: std::f32::consts::FRAC_PI_2,
            end: std::f32::consts::PI,
            closed: false,
        }),
        topology: None,
    };
    let (bodies, warnings, base_tris) = sketch_extrude_edge_mod(
        curves,
        |r| r.contains((5.0, 25.0)) && !r.contains((29.0, 1.0)),
        arc,
        zerocad_core::CornerKind::Fillet,
        1.5,
    );
    assert_committed("corner bite fillet", &bodies, &warnings, base_tris);
}

/// GUI regression: a plain sketch-circle cylinder's top rim. The pure-circle
/// boundary must NOT be mistaken for a rect-minus-circle bite (its tangent
/// points against its own bounding box can satisfy the side-hit test), which
/// fabricated a bite "void" covering the whole body ("candidate placed 1749
/// non-wall sample(s) inside the removed circular-bite volume").
#[test]
fn sketch_cylinder_rim_fillet_commits_through_the_graph() {
    let mut curves = SketchCurves::new();
    curves.add_circle((0.0, 0.0), 12.0);
    let arc = EdgeRef {
        p0: [12.0, 0.0, 10.0],
        p1: [-12.0, 0.0, 10.0],
        n1: [0.0, 0.0, 1.0],
        n2: [1.0, 0.0, 0.0],
        curve: Some(EdgeCurveHint::Circle {
            center: [0.0, 0.0, 10.0],
            axis: [0.0, 0.0, 1.0],
            x_dir: [1.0, 0.0, 0.0],
            radius: 12.0,
            start: 0.0,
            end: std::f32::consts::TAU,
            closed: true,
        }),
        topology: None,
    };
    let (bodies, warnings, base_tris) = sketch_extrude_edge_mod(
        curves,
        |r| r.contains((0.0, 0.0)),
        arc,
        zerocad_core::CornerKind::Fillet,
        2.0,
    );
    assert_committed("sketch cylinder rim fillet", &bodies, &warnings, base_tris);
}

/// GUI round 3: fillets on ADJACENT edges must FLOW into each other. On a
/// corner-bite body, fillet the concave arc and the adjacent straight top edge
/// with the same radius, in either order — both EdgeMods must COMMIT through
/// the acceptance gates (the kernel miters the two bands along their band∩band
/// seam; pre-fix the second fillet was rejected with "candidate render mesh has
/// N crack edges" and the body left unchanged).
fn corner_flow_edge_mods_commit(arc_first: bool) {
    let label = if arc_first {
        "arc→straight flow"
    } else {
        "straight→arc flow"
    };
    let mut curves = SketchCurves::new();
    curves.add_rectangle((0.0, 0.0), (30.0, 30.0));
    curves.add_circle((30.0, 0.0), 10.0);
    let regions = detect_regions(&curves);
    let region = regions
        .iter()
        .position(|r| r.contains((5.0, 25.0)) && !r.contains((29.0, 1.0)))
        .expect("corner-bite region");
    let dist = 3.0f32;

    let arc = EdgeRef {
        p0: [30.0, 10.0, 10.0],
        p1: [20.0, 0.0, 10.0],
        n1: [0.0, 0.0, 1.0],
        n2: [1.0, 0.0, 0.0],
        curve: Some(EdgeCurveHint::Circle {
            center: [30.0, 0.0, 10.0],
            axis: [0.0, 0.0, 1.0],
            x_dir: [1.0, 0.0, 0.0],
            radius: 10.0,
            start: std::f32::consts::FRAC_PI_2,
            end: std::f32::consts::PI,
            closed: false,
        }),
        topology: None,
    };
    // The straight top edge along y = 0. When the arc is filleted first it is
    // SHORTENED to x ≤ 30 − (10 + dist) = 17 — the user then clicks the current
    // edge, so capture the endpoints as they exist at selection time.
    let straight_end = if arc_first { 17.0 } else { 20.0 };
    let straight = EdgeRef {
        p0: [0.0, 0.0, 10.0],
        p1: [straight_end, 0.0, 10.0],
        n1: [0.0, 0.0, 1.0],
        n2: [0.0, -1.0, 0.0],
        curve: None,
        topology: None,
    };
    let (first, second) = if arc_first {
        (arc, straight)
    } else {
        (straight, arc)
    };

    let mut g = ParametricGraph::new();
    g.add_feature(FeatureNode {
        id: "s".into(),
        name: "S".into(),
        feature: FeatureType::Sketch {
            entity_ids: vec![],
            next_entity_id: 0,
            solver: None,
            cs: CoordinateSystem::new(
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            ),
            curves,
            shapes: vec![],
            corner_mods: vec![],
            mirrors: vec![],
            on_face: false,
        },
    });
    g.add_feature(FeatureNode {
        id: "e".into(),
        name: "E".into(),
        feature: FeatureType::Extrude {
            target: None,
            depth: 10.0,
            region_indices: vec![region],
            mode: ExtrudeMode::NewBody,
            depth_expr: None,
        },
    });
    g.add_dependency("s", "e");
    let edge_mod = |g: &mut ParametricGraph, id: &str, after: &str, edge: EdgeRef| {
        g.add_feature(FeatureNode {
            id: id.into(),
            name: "Edge Mod".into(),
            feature: FeatureType::EdgeMod {
                target: "e".into(),
                edge,
                dist,
                dist_expr: None,
                replay: EdgeModReplayIntent::default(),
                kind: zerocad_core::CornerKind::Fillet,
            },
        });
        g.add_dependency(after, id);
    };
    edge_mod(&mut g, "em1", "e", first);
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert!(
        warnings.iter().all(|w| !w.contains("couldn't be")),
        "{label}: first edge-mod should commit, got {warnings:?}"
    );
    let after_first_tris = bodies[0].1.indices.len();

    edge_mod(&mut g, "em2", "em1", second);
    let (bodies, warnings) = g.evaluate_bodies_with_warnings(&HashSet::new()).unwrap();
    assert!(
        warnings.iter().all(|w| !w.contains("couldn't be")),
        "{label}: second edge-mod should commit (the bands must miter), got {warnings:?}"
    );
    assert_eq!(bodies.len(), 1, "{label}: keeps one body");
    assert_ne!(
        bodies[0].1.indices.len(),
        after_first_tris,
        "{label}: the second edge-mod must change the display mesh"
    );
}

#[test]
fn adjacent_fillets_flow_arc_then_straight() {
    corner_flow_edge_mods_commit(true);
}

#[test]
fn adjacent_fillets_flow_straight_then_arc() {
    corner_flow_edge_mods_commit(false);
}

#[test]
fn oversized_rim_fillet_via_hint_fails_safely() {
    let solid = cylinder_solid(4.0, 10.0).expect("cylinder primitive");
    let hint = top_rim_hint(4.0, 10.0);
    // Radius exceeds the cap radius → the analytic contact ring is invalid.
    assert!(
        fillet_edge_with_hint(
            &solid,
            [4.0, 10.0, 0.0],
            [-4.0, 10.0, 0.0],
            Some(&hint),
            6.0
        )
        .is_err(),
        "an oversized rim fillet must fail rather than emit broken geometry"
    );
}
