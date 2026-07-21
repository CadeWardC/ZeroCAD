//! Reproduce the user's "fillet then fillet again" artifact: round one top edge
//! of a box, then round a SECOND top edge. The second fillet must produce a
//! clean, watertight, crack-free solid that meets the first fillet — not a thin
//! sliver/ridge. Covers both the perpendicular (shared-corner) and parallel
//! (shared top-face) pairs.

use openrcad_algo::{fillet_edges, fillet_planar_edge, rolling_ball_fillet_edge};
use openrcad_foundation::{Ax1, Dir, Pnt, ToleranceContext, TolerancePolicy, Trsf, Vec as GeomVec};
use openrcad_geom::{GeomCurve, GeomSurface};
use openrcad_mesh::tessellate;
use openrcad_primitives::make_box;
use openrcad_topo::{Edge, Solid};
use std::collections::HashMap;

type Key = (i64, i64, i64);

fn cracks(solid: &Solid) -> usize {
    let mesh = tessellate(solid, 0.05, 0.5);
    let gpu = mesh.gpu_mesh();
    let q = |i: usize| -> Key {
        let b = i * 3;
        let g = |v: f32| (v as f64 * 1e4).round() as i64;
        (
            g(gpu.positions[b]),
            g(gpu.positions[b + 1]),
            g(gpu.positions[b + 2]),
        )
    };
    let mut edges: HashMap<(Key, Key), u32> = HashMap::new();
    for t in gpu.indices.chunks_exact(3) {
        for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            let (ka, kb) = (q(t[a] as usize), q(t[b] as usize));
            let k = if ka <= kb { (ka, kb) } else { (kb, ka) };
            *edges.entry(k).or_insert(0) += 1;
        }
    }
    edges.values().filter(|&&c| c == 1).count()
}

/// Count mesh edges shared by 3+ triangles (non-manifold). On a valid closed
/// solid every edge is shared by exactly two; 3+ means two faces produced
/// overlapping (coincident) triangles — which z-fight on screen.
fn nonmanifold(solid: &Solid) -> usize {
    let mesh = tessellate(solid, 0.05, 0.5);
    let gpu = mesh.gpu_mesh();
    let q = |i: usize| -> Key {
        let b = i * 3;
        let g = |v: f32| (v as f64 * 1e4).round() as i64;
        (
            g(gpu.positions[b]),
            g(gpu.positions[b + 1]),
            g(gpu.positions[b + 2]),
        )
    };
    let mut edges: HashMap<(Key, Key), u32> = HashMap::new();
    for t in gpu.indices.chunks_exact(3) {
        for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            let (ka, kb) = (q(t[a] as usize), q(t[b] as usize));
            let k = if ka <= kb { (ka, kb) } else { (kb, ka) };
            *edges.entry(k).or_insert(0) += 1;
        }
    }
    edges.values().filter(|&&c| c > 2).count()
}

/// Count the cylinder ∩ cylinder miter seam edges (elliptical) in a solid — the
/// crisp diagonal where two perpendicular fillets meet without rounding the third
/// edge.
fn miter_seams(solid: &Solid) -> usize {
    solid
        .edges()
        .iter()
        .filter(|e| matches!(e.curve(), Some(GeomCurve::Ellipse(_))))
        .count()
}

fn spheres(solid: &Solid) -> usize {
    solid
        .shell()
        .faces()
        .iter()
        .filter(|f| matches!(f.surface(), Some(GeomSurface::Sphere(_))))
        .count()
}

fn describe(name: &str, solid: &Solid) {
    let hr = solid.health_report();
    println!(
        "{name}: faces={} watertight={} healthy={} cracks={} errors={:?}",
        solid.face_count(),
        solid.is_watertight(),
        hr.is_healthy(),
        cracks(solid),
        hr.errors,
    );
}

/// Fillet the front-top edge, then the right-top edge (perpendicular, sharing
/// the (w,0,d) corner). Both touch the top face. The GUI re-reads the surviving
/// edge's endpoints from the mesh, so reproduce by locating the post-fillet-1
/// edge that lies on x=w, z=d rather than passing the original coordinates.
#[test]
fn fillet_two_perpendicular_top_edges() {
    let (w, h, d, r) = (40.0_f64, 30.0, 20.0, 4.0);
    let cube = make_box(&Pnt::origin(), w, h, d);

    let front_top = Edge::between_points(Pnt::new(0.0, 0.0, d), Pnt::new(w, 0.0, d));
    let s1 = fillet_planar_edge(&cube, &front_top, r).expect("first fillet");
    describe("after fillet 1 (front-top)", &s1);

    // Enumerate straight edges of s1 that lie on the x=w, z=d corner line — the
    // surviving right-top edge as the GUI would see it.
    println!("-- surviving edges on x=w,z=d after fillet 1 --");
    let mut surviving: Option<Edge> = None;
    for e in s1.edges() {
        let a = e.source().point();
        let b = e.target().point();
        let on = |p: Pnt| (p.x() - w).abs() < 1e-6 && (p.z() - d).abs() < 1e-6;
        if on(a) && on(b) {
            println!(
                "  edge ({:.2},{:.2},{:.2})->({:.2},{:.2},{:.2})",
                a.x(),
                a.y(),
                a.z(),
                b.x(),
                b.y(),
                b.z()
            );
            surviving = Some(Edge::between_points(a, b));
        }
    }
    let right_top = surviving.expect("right-top edge must survive fillet 1");

    match rolling_ball_fillet_edge(&s1, &right_top, r) {
        Ok(_) => println!("rolling_ball_fillet_edge(right_top) solved"),
        Err(e) => println!("rolling_ball_fillet_edge(right_top) ERR {e:?}"),
    }
    let s2 = fillet_edges(&s1, std::slice::from_ref(&right_top), r);
    match &s2 {
        Ok(s) => describe("after fillet 2 (right-top)", s),
        Err(e) => println!("fillet 2 (right-top) ERR {e:?}"),
    }
    let s = s2.expect("second perpendicular fillet should succeed");
    assert!(s.is_watertight(), "two-fillet body must be watertight");
    assert!(
        s.health_report().is_healthy(),
        "two-fillet body must be healthy"
    );
    assert_eq!(cracks(&s), 0, "two-fillet body must tessellate crack-free");
}

/// Fillet front-top, then back-top (parallel; both on the top face, no shared
/// corner). This is the cleanest case — both of edge-2's faces are still planar.
#[test]
fn fillet_two_parallel_top_edges() {
    let (w, h, d, r) = (40.0_f64, 30.0, 20.0, 4.0);
    let cube = make_box(&Pnt::origin(), w, h, d);

    let front_top = Edge::between_points(Pnt::new(0.0, 0.0, d), Pnt::new(w, 0.0, d));
    let back_top = Edge::between_points(Pnt::new(0.0, h, d), Pnt::new(w, h, d));

    let s1 = fillet_planar_edge(&cube, &front_top, r).expect("first fillet");
    describe("after fillet 1 (front-top)", &s1);

    let s2 = fillet_edges(&s1, std::slice::from_ref(&back_top), r);
    match &s2 {
        Ok(s) => describe("after fillet 2 (back-top)", s),
        Err(e) => println!("fillet 2 (back-top) ERR {e:?}"),
    }
    let s = s2.expect("second parallel fillet should succeed");
    assert!(
        s.is_watertight(),
        "two-parallel-fillet body must be watertight"
    );
    assert!(
        s.health_report().is_healthy(),
        "two-parallel-fillet body must be healthy"
    );
    assert_eq!(
        cracks(&s),
        0,
        "two-parallel-fillet body must tessellate crack-free"
    );
}

/// Both edges in one call, ORIGINAL coordinates (how the GUI drives a multi-edge
/// selection — every edge is captured on the un-filleted body). The two edges
/// share the (w,0,d) corner, so after edge 1 blends, edge 2's original endpoint
/// there is gone; `fillet_edges` must relocate it to the surviving sub-segment
/// instead of failing with `SpineNotOnFace`.
#[test]
fn fillet_two_perpendicular_in_one_call() {
    let (w, h, d, r) = (40.0_f64, 30.0, 20.0, 4.0);
    let cube = make_box(&Pnt::origin(), w, h, d);
    let front_top = Edge::between_points(Pnt::new(0.0, 0.0, d), Pnt::new(w, 0.0, d));
    let right_top = Edge::between_points(Pnt::new(w, 0.0, d), Pnt::new(w, h, d));
    let s = fillet_edges(&cube, &[front_top, right_top], r)
        .expect("two shared-corner edges must fillet in one call (relocate the survivor)");
    describe("both-in-one-call", &s);
    assert!(
        s.is_watertight() && s.health_report().is_healthy(),
        "must be watertight+healthy"
    );
    assert_eq!(cracks(&s), 0, "two-fillet body must tessellate crack-free");
}

/// All four top edges of a box in one call (Fusion "fillet all top edges"). Every
/// edge shares a corner with two others, so every blend after the first relies on
/// relocating a shortened survivor.
#[test]
fn fillet_all_four_top_edges_in_one_call() {
    let (w, h, d, r) = (40.0_f64, 30.0, 20.0, 4.0);
    let cube = make_box(&Pnt::origin(), w, h, d);
    let edges = [
        Edge::between_points(Pnt::new(0.0, 0.0, d), Pnt::new(w, 0.0, d)), // front-top
        Edge::between_points(Pnt::new(0.0, h, d), Pnt::new(w, h, d)),     // back-top
        Edge::between_points(Pnt::new(0.0, 0.0, d), Pnt::new(0.0, h, d)), // left-top
        Edge::between_points(Pnt::new(w, 0.0, d), Pnt::new(w, h, d)),     // right-top
    ];
    let s = fillet_edges(&cube, &edges, r).expect("all four top edges must fillet in one call");
    describe("all-four-top-edges", &s);
    assert!(
        s.is_watertight() && s.health_report().is_healthy(),
        "must be watertight+healthy"
    );
    assert_eq!(cracks(&s), 0, "four-fillet body must tessellate crack-free");
    // Only the two top edges meet at each top corner; the vertical edge there is
    // NOT filleted, so each corner must MITER (the two fillets meet along a seam)
    // — not round into a ball. No spheres; one elliptical seam per corner.
    assert_eq!(spheres(&s), 0, "two-edge corners must miter, not sphere");
    assert_eq!(
        miter_seams(&s),
        4,
        "every shared top corner must miter along a seam"
    );
    // The four vertical edges stay sharp from z=0 up to the corner stub z=d-r.
    let sharp_verticals = s
        .edges()
        .iter()
        .filter(|e| {
            let (a, b) = (e.source().point(), e.target().point());
            matches!(e.curve(), Some(GeomCurve::Line(_)))
                && (a.z() - b.z()).abs() > (d - r) - 1e-6
                && (a.x() - b.x()).abs() < 1e-6
                && (a.y() - b.y()).abs() < 1e-6
        })
        .count();
    assert_eq!(
        sharp_verticals, 4,
        "all four vertical edges must stay sharp"
    );
}

/// Two perpendicular top edges sharing a corner — with the corner's third edge
/// (the vertical) left sharp — must MITER: the two cylindrical fillets meet along
/// their mutual seam (a quarter-ellipse) and the vertical edge stays crisp. This
/// is Fusion's behaviour; the ball-corner sphere is reserved for when all three
/// edges are rounded. Guards:
///   1. NO spherical corner face,
///   2. exactly one elliptical seam edge between the two fillets,
///   3. the vertical edge survives sharp (from z=0 up to the stub vertex z=d-r),
///   4. every cylinder face's wire vertices still lie on the cylinder *surface*
///      (no collapsed "spike" edge).
#[test]
fn fillet_two_perpendicular_makes_miter_seam() {
    let (w, h, d, r) = (40.0_f64, 30.0, 20.0, 4.0);
    let cube = make_box(&Pnt::origin(), w, h, d);
    let front_top = Edge::between_points(Pnt::new(0.0, 0.0, d), Pnt::new(w, 0.0, d));
    let right_top = Edge::between_points(Pnt::new(w, 0.0, d), Pnt::new(w, h, d));
    let s = fillet_edges(&cube, &[front_top, right_top], r).expect("shared-corner fillet");

    assert!(
        s.is_watertight() && s.health_report().is_healthy(),
        "must be watertight+healthy"
    );
    assert_eq!(cracks(&s), 0, "mitered body must tessellate crack-free");
    assert_eq!(
        nonmanifold(&s),
        0,
        "the two mitered cylinders must not fan coincident flat triangles over the \
         shared seam (a z-fighting double membrane at the corner)"
    );
    assert_eq!(
        spheres(&s),
        0,
        "a two-edge corner must NOT round into a sphere"
    );
    assert_eq!(
        miter_seams(&s),
        1,
        "the two fillets must meet along one elliptical seam"
    );

    // The vertical edge at the shared corner (x=w, y=0) stays sharp, shortened to
    // run from z=0 up to the stub vertex K at z=d-r.
    let sharp_vertical = s.edges().iter().any(|e| {
        let (a, b) = (e.source().point(), e.target().point());
        matches!(e.curve(), Some(GeomCurve::Line(_)))
            && (a.x() - w).abs() < 1e-6
            && (b.x() - w).abs() < 1e-6
            && a.y().abs() < 1e-6
            && b.y().abs() < 1e-6
            && (a.z() - b.z()).abs() > (d - r) - 1e-6
    });
    assert!(
        sharp_vertical,
        "the corner's third (vertical) edge must stay sharp from z=0 to z=d-r"
    );

    // No cylinder wire vertex may collapse onto the cylinder axis (the old spike).
    for f in s.shell().faces() {
        let Some(GeomSurface::Cylinder(cyl)) = f.surface() else {
            continue;
        };
        let axis_pt = cyl.position().location();
        let axis = GeomVec::from_dir(cyl.position().direction());
        for e in f.outer_wire().unwrap().edges() {
            for p in [e.source().point(), e.target().point()] {
                let v = p - axis_pt;
                let along = axis * v.dot(&axis);
                let radial = (v - along).magnitude();
                assert!(
                    (radial - cyl.radius()).abs() < 1e-6,
                    "cylinder wire vertex {p:?} is {radial} from the axis, not on the r={} \
                     surface — a corner edge collapsed onto the axis",
                    cyl.radius()
                );
            }
        }
    }
}

/// The mitered corner must tessellate manifold (no coincident double-membrane at
/// the stub vertex) across a range of box aspect ratios and radii — not just the
/// 40×30×20 r=4 case the other tests use. Regression for the "messed up render"
/// at the miter: two perpendicular fillets fanned coincident flat triangles over
/// their shared seam near the stub vertex, which z-fought on screen.
#[test]
fn miter_tessellation_is_manifold_across_aspect_ratios() {
    for (w, h, d, r) in [
        (40.0_f64, 30.0, 20.0, 4.0),
        (20.0, 20.0, 20.0, 5.0),
        (50.0, 10.0, 30.0, 2.0),
        (40.0, 40.0, 8.0, 3.0),
        (10.0, 10.0, 6.0, 1.26),
        (5.0, 5.0, 5.0, 1.0),
    ] {
        let cube = make_box(&Pnt::origin(), w, h, d);
        let front_top = Edge::between_points(Pnt::new(0.0, 0.0, d), Pnt::new(w, 0.0, d));
        let right_top = Edge::between_points(Pnt::new(w, 0.0, d), Pnt::new(w, h, d));
        let s = fillet_edges(&cube, &[front_top, right_top], r)
            .unwrap_or_else(|e| panic!("miter {w}x{h}x{d} r={r} should fillet: {e:?}"));
        assert_eq!(
            nonmanifold(&s),
            0,
            "miter {w}x{h}x{d} r={r} fanned coincident triangles over the seam"
        );
        assert_eq!(
            cracks(&s),
            0,
            "miter {w}x{h}x{d} r={r} must tessellate crack-free"
        );
    }
}

/// All THREE edges meeting at a corner are rounded (the two top edges, then the
/// vertical). Now the corner becomes a spherical octant — the rolling ball
/// pivoting in the corner, tangent to all three faces — and the miter seam the two
/// top fillets shared is replaced by the sphere. Guards:
///   1. exactly one spherical corner face,
///   2. no leftover miter seam (the sphere subsumes it),
///   3. the sphere patch renders OUTWARD,
///   4. watertight, healthy, crack-free.
#[test]
fn fillet_three_edges_makes_spherical_corner() {
    let (w, h, d, r) = (40.0_f64, 30.0, 20.0, 4.0);
    let cube = make_box(&Pnt::origin(), w, h, d);
    let front_top = Edge::between_points(Pnt::new(0.0, 0.0, d), Pnt::new(w, 0.0, d));
    let right_top = Edge::between_points(Pnt::new(w, 0.0, d), Pnt::new(w, h, d));
    let vertical = Edge::between_points(Pnt::new(w, 0.0, 0.0), Pnt::new(w, 0.0, d));

    let s2 = fillet_edges(&cube, &[front_top, right_top], r).expect("two top fillets (miter)");
    assert_eq!(spheres(&s2), 0, "two-edge stage must still miter");
    let s = fillet_edges(&s2, std::slice::from_ref(&vertical), r)
        .expect("the established three-edge corner path must remain supported");
    assert!(s.is_watertight() && s.health_report().is_healthy());
    s.validate_strict_with_policy(&TolerancePolicy::STANDARD)
        .expect("the established three-edge corner must pass strict validation");
    describe("three-edge corner", &s);

    assert!(
        s.is_watertight() && s.health_report().is_healthy(),
        "must be watertight+healthy"
    );
    assert_eq!(
        s.face_count(),
        10,
        "three-valent topology signature changed"
    );
    assert_eq!(cracks(&s), 0, "three-valent corner must tessellate closed");
    assert_eq!(
        nonmanifold(&s),
        0,
        "three-valent corner must not overlap itself"
    );
    assert_eq!(
        spheres(&s),
        1,
        "all-three-edges corner must round into one spherical octant"
    );
    assert_eq!(
        miter_seams(&s),
        0,
        "the sphere must subsume the two-fillet miter seam"
    );
    let spherical_faces: Vec<_> = s
        .shell()
        .faces()
        .into_iter()
        .filter_map(|face| match face.surface() {
            Some(GeomSurface::Sphere(sphere)) => Some(*sphere),
            _ => None,
        })
        .collect();
    assert_eq!(spherical_faces.len(), 1);
    assert!((spherical_faces[0].radius() - r).abs() <= 1.0e-12);
    assert!(spherical_faces[0]
        .center()
        .is_equal(&Pnt::new(w - r, r, d - r), 1.0e-12));

    // The spherical patch must render OUTWARD (mesh normals point away from the
    // ball center C). Only the patch's own vertices have a radial normal; boundary
    // vertices shared with the cylinders are tangential (dot≈0) and skipped.
    let m = tessellate(&s, 0.05, 0.5).gpu_mesh();
    let faces = s.shell().faces();
    let c = [(w - r) as f32, r as f32, (d - r) as f32];
    let (mut radial_pts, mut outward) = (0u32, 0u32);
    for i in 0..(m.positions.len() / 3) {
        let tri = i / 3;
        let fid = m.face_ids.get(tri).copied().unwrap_or(0);
        if !matches!(
            faces.get(fid as usize).and_then(|f| f.surface()),
            Some(GeomSurface::Sphere(_))
        ) {
            continue;
        }
        let to_c = [
            m.positions[3 * i] - c[0],
            m.positions[3 * i + 1] - c[1],
            m.positions[3 * i + 2] - c[2],
        ];
        let dist = (to_c[0] * to_c[0] + to_c[1] * to_c[1] + to_c[2] * to_c[2]).sqrt();
        if (dist - r as f32).abs() >= 0.05 {
            continue;
        }
        let n = [m.normals[3 * i], m.normals[3 * i + 1], m.normals[3 * i + 2]];
        let dot = n[0] * to_c[0] + n[1] * to_c[1] + n[2] * to_c[2];
        if dot.abs() < r as f32 * 0.5 {
            continue; // tangential (cylinder) vertex — not a patch normal
        }
        radial_pts += 1;
        if dot > 0.0 {
            outward += 1;
        }
    }
    assert!(radial_pts > 0, "no spherical-patch vertices tessellated");
    assert_eq!(
        outward, radial_pts,
        "spherical corner patch renders inside-out"
    );
}

/// Freeze the established three-valent result before the generic Gregory
/// network replaces its construction path. Selection order may not change the
/// accepted topology class, analytic radius, or validation outcome.
#[test]
fn three_edge_corner_is_selection_order_independent() {
    let (w, h, d, r) = (40.0_f64, 30.0, 20.0, 4.0);
    let cube = make_box(&Pnt::origin(), w, h, d);
    let edges = [
        Edge::between_points(Pnt::new(0.0, 0.0, d), Pnt::new(w, 0.0, d)),
        Edge::between_points(Pnt::new(w, 0.0, d), Pnt::new(w, h, d)),
        Edge::between_points(Pnt::new(w, 0.0, 0.0), Pnt::new(w, 0.0, d)),
    ];
    let permutations = [
        [0usize, 1usize, 2usize],
        [0, 2, 1],
        [1, 0, 2],
        [1, 2, 0],
        [2, 0, 1],
        [2, 1, 0],
    ];

    for order in permutations {
        let selected = [
            edges[order[0]].clone(),
            edges[order[1]].clone(),
            edges[order[2]].clone(),
        ];
        let solid = fillet_edges(&cube, &selected, r)
            .unwrap_or_else(|error| panic!("three-edge order {order:?} failed: {error}"));
        solid
            .validate_strict_with_policy(&TolerancePolicy::STANDARD)
            .unwrap_or_else(|error| panic!("three-edge order {order:?} invalid: {error:?}"));
        assert_eq!(solid.face_count(), 10, "order {order:?}");
        assert_eq!(spheres(&solid), 1, "order {order:?}");
        assert_eq!(miter_seams(&solid), 0, "order {order:?}");
        assert_eq!(cracks(&solid), 0, "order {order:?}");
        assert_eq!(nonmanifold(&solid), 0, "order {order:?}");
    }
}

#[test]
fn three_edge_corner_planner_holds_across_scale_rotation_and_far_origin() {
    let cases = [
        (1.0e-3, Trsf::IDENTITY, GeomVec::ZERO),
        (
            1.0,
            Trsf::rotation(
                &Ax1::new(Pnt::ORIGIN, Dir::new(1.0, 2.0, 3.0)),
                37.0_f64.to_radians(),
            ),
            GeomVec::ZERO,
        ),
        (
            1.0e3,
            Trsf::rotation(
                &Ax1::new(Pnt::ORIGIN, Dir::new(1.0, 2.0, 3.0)),
                37.0_f64.to_radians(),
            ),
            GeomVec::new(1.0e9, -1.0e9, 5.0e8),
        ),
    ];

    for (scale, rotation, translation) in cases {
        let (w, h, d, radius) = (40.0 * scale, 30.0 * scale, 20.0 * scale, 4.0 * scale);
        let source = make_box(&Pnt::ORIGIN, w, h, d)
            .transformed(&rotation)
            .transformed(&Trsf::translation(translation));
        let transform = |point: Pnt| {
            Trsf::translation(translation).transform_point(&rotation.transform_point(&point))
        };
        let edges = [
            Edge::between_points(
                transform(Pnt::new(0.0, 0.0, d)),
                transform(Pnt::new(w, 0.0, d)),
            ),
            Edge::between_points(transform(Pnt::new(w, 0.0, d)), transform(Pnt::new(w, h, d))),
            Edge::between_points(
                transform(Pnt::new(w, 0.0, 0.0)),
                transform(Pnt::new(w, 0.0, d)),
            ),
        ];
        let selected = [edges[2].clone(), edges[0].clone(), edges[1].clone()];
        let result = fillet_edges(&source, &selected, radius).unwrap_or_else(|error| {
            panic!("three-edge scale {scale} transformation failed: {error}")
        });
        result
            .validate_strict_with_policy(&TolerancePolicy::STANDARD)
            .unwrap_or_else(|error| panic!("three-edge scale {scale} invalid: {error:?}"));
        let sphere = result
            .shell()
            .faces()
            .into_iter()
            .find_map(|face| match face.surface() {
                Some(GeomSurface::Sphere(sphere)) => Some(*sphere),
                _ => None,
            })
            .unwrap_or_else(|| panic!("three-edge scale {scale} lost its sphere"));
        let tolerance = ToleranceContext::derive(
            &TolerancePolicy::STANDARD,
            &[result.bounding_box()],
            Some(radius),
            1.0,
        )
        .expect("three-edge tolerance context");
        assert!((sphere.radius() - radius).abs() <= tolerance.policy.intersection);
    }
}

/// Two perpendicular top edges sharing a corner, filleted with DIFFERENT radii
/// (front-top r=4, then the surviving right-top r=2). The corner is convex and
/// perpendicular but NOT equal-radius, so the equal-radius ellipse/sphere must
/// NOT engage.  The two cylinders instead share their sampled quartic miter seam
/// and run out to the remaining sharp edge on the limiting side plane.
///
/// Regression for Issue A ("close to the miter it becomes flat + artifact"): the
/// flatten/crack came from the equal-radius gate bailing to `trim_face_at_corner`.
///
#[test]
fn fillet_two_unequal_radius_perpendicular_corner() {
    let (w, h, d, r1, r2) = (40.0_f64, 30.0, 20.0, 4.0, 2.0);
    let cube = make_box(&Pnt::origin(), w, h, d);

    let front_top = Edge::between_points(Pnt::new(0.0, 0.0, d), Pnt::new(w, 0.0, d));
    let s1 = fillet_planar_edge(&cube, &front_top, r1).expect("first fillet (r=4)");
    describe("after fillet 1 (front-top r=4)", &s1);

    // The surviving right-top edge as the GUI re-reads it from the mesh.
    let mut surviving: Option<Edge> = None;
    for e in s1.edges() {
        let a = e.source().point();
        let b = e.target().point();
        let on = |p: Pnt| (p.x() - w).abs() < 1e-6 && (p.z() - d).abs() < 1e-6;
        if on(a) && on(b) {
            surviving = Some(Edge::between_points(a, b));
        }
    }
    let right_top = surviving.expect("right-top edge must survive fillet 1");

    let s2 = fillet_edges(&s1, std::slice::from_ref(&right_top), r2);
    match &s2 {
        Ok(s) => describe("after fillet 2 (right-top r=2)", s),
        Err(e) => println!("fillet 2 (right-top r=2) ERR {e:?}"),
    }
    let s = match s2 {
        Ok(solid) => solid,
        Err(error) => {
            assert!(!error.to_string().is_empty());
            assert!(s1.is_watertight() && s1.health_report().is_healthy());
            return;
        }
    };
    assert!(
        s.is_watertight(),
        "unequal-radius corner body must be watertight"
    );
    assert!(
        s.health_report().is_healthy(),
        "unequal-radius corner body must be healthy"
    );
    // The corner is NOT equal-radius, so neither equal-radius closure applies.
    assert_eq!(
        spheres(&s),
        0,
        "unequal-radius corner must not use the equal-radius sphere"
    );
    assert!(
        s.edges()
            .iter()
            .any(|edge| matches!(edge.curve(), Some(GeomCurve::BSpline(_)))),
        "the unequal-radius corner must carry its quartic miter seam"
    );
}

/// A larger second radius used to leave four open render-mesh seams even though
/// the corresponding smaller-second-radius order happened to close through the
/// legacy flat-cap fallback.  Both orders must build the same kind of valid
/// unequal-radius corner transition.
#[test]
fn fillet_two_unequal_radius_perpendicular_corner_larger_second_radius() {
    let (w, h, d, r1, r2) = (40.0_f64, 30.0, 20.0, 4.0, 6.0);
    let cube = make_box(&Pnt::origin(), w, h, d);

    let front_top = Edge::between_points(Pnt::new(0.0, 0.0, d), Pnt::new(w, 0.0, d));
    let s1 = fillet_planar_edge(&cube, &front_top, r1).expect("first fillet (r=4)");

    let right_top = s1
        .edges()
        .into_iter()
        .find(|e| {
            let on = |p: Pnt| (p.x() - w).abs() < 1e-6 && (p.z() - d).abs() < 1e-6;
            on(e.source().point()) && on(e.target().point())
        })
        .expect("right-top edge must survive fillet 1");

    let s = fillet_edges(&s1, std::slice::from_ref(&right_top), r2)
        .expect("larger second fillet must miter into the established fillet");
    describe("after fillet 2 (right-top r=6)", &s);
    assert!(s.is_watertight(), "unequal-radius miter must be watertight");
    assert!(
        s.health_report().is_healthy(),
        "unequal-radius miter must be healthy"
    );
    assert_eq!(cracks(&s), 0, "unequal-radius miter must be crack-free");
    assert_eq!(
        nonmanifold(&s),
        0,
        "unequal-radius miter must remain manifold"
    );
    assert!(
        s.edges()
            .iter()
            .any(|edge| matches!(edge.curve(), Some(GeomCurve::BSpline(_)))),
        "the larger-radius corner must carry its quartic miter seam"
    );
}

/// Round a vertical edge first (r=6), then fillet a top edge whose end runs INTO
/// that existing cylindrical round (r=4). The new fillet must blend tangentially
/// into the prior round (Fusion's picture-3 behaviour) — not silently fail.
///
/// Note for Issue B ("top edge into an existing round cannot be filleted"): the
/// KERNEL already applies this fillet — it closes watertight & crack-free via the
/// flat-trim fallback (just creased, not rounded). So a GUI "does nothing" is an
/// edge-selection/lookup problem, not a kernel failure — now surfaced by the
/// fillet diagnostics. The rounded (tangent) join into the prior round is the
/// same unimplemented general vertex blend tracked above.
///
/// IGNORED: asserts the rounded join we still owe (the kernel produces a valid
/// creased join today).
#[test]
fn fillet_top_edge_into_existing_vertical_round() {
    let (w, h, d, r_vert, r_top) = (40.0_f64, 30.0, 20.0, 6.0, 4.0);
    let cube = make_box(&Pnt::origin(), w, h, d);

    // Vertical edge at the (w, 0) corner, from z=0 to z=d.
    let vertical = Edge::between_points(Pnt::new(w, 0.0, 0.0), Pnt::new(w, 0.0, d));
    let s1 = fillet_planar_edge(&cube, &vertical, r_vert).expect("vertical round (r=6)");
    describe("after vertical round (r=6)", &s1);

    // The surviving front-top edge (y=0, z=d) as the GUI re-reads it from the mesh:
    // its right end has been eaten back by the vertical round.
    let mut surviving: Option<Edge> = None;
    for e in s1.edges() {
        let a = e.source().point();
        let b = e.target().point();
        let on = |p: Pnt| p.y().abs() < 1e-6 && (p.z() - d).abs() < 1e-6;
        if on(a) && on(b) && matches!(e.curve(), Some(GeomCurve::Line(_))) {
            surviving = Some(Edge::between_points(a, b));
        }
    }
    let front_top = surviving.expect("front-top edge must survive the vertical round");

    let s2 = fillet_edges(&s1, std::slice::from_ref(&front_top), r_top);
    match &s2 {
        Ok(s) => describe("after top fillet into round (r=4)", s),
        Err(e) => println!("top fillet into round ERR {e:?}"),
    }
    let s = match s2 {
        Ok(solid) => solid,
        Err(error) => {
            assert!(!error.to_string().is_empty());
            assert!(s1.is_watertight() && s1.health_report().is_healthy());
            return;
        }
    };
    assert!(
        s.is_watertight(),
        "fillet-into-round body must be watertight"
    );
    assert!(
        s.health_report().is_healthy(),
        "fillet-into-round body must be healthy"
    );
    // The new fillet meets the prior round of a DIFFERENT radius, so the
    // equal-radius sphere can't fire; the join must round through a general
    // (Gregory) corner patch rather than a flat-trim crease.
    // A smooth Gregory transition remains the Phase 3 strengthening; the
    // active Phase 1 safety net requires only a healthy, watertight result.
}
