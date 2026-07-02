//! Fillet a box edge first, then cut a through-cylinder across that rounded edge.
//!
//! This is the kernel primitive behind the ZeroCAD workflow:
//! box -> selected-edge fillet -> sketch circle cut. The cutter crosses the
//! top plane, the cylindrical fillet face, and the front side face. Both the
//! smooth cylinder and a faceted prism cutter must remove material; otherwise
//! applications exhaust their fallback chain and leave the body unchanged.

use openrcad_algo::{boolean, boolean_checked, fillet_edges, prism, BooleanOp};
use openrcad_foundation::{Ax2, Dir, Pnt, Vec as GeomVec};
use openrcad_geom::{GeomSurface, Plane};
use openrcad_mesh::tessellate;
use openrcad_primitives::{make_box, make_cylinder};
use openrcad_topo::{Edge, Face, Solid, Wire};
use std::collections::HashMap;

const BOX_W: f64 = 40.0;
const BOX_H: f64 = 30.0;
const BOX_D: f64 = 15.0;
const CUT_X: f64 = 20.0;
const CUT_Y: f64 = 2.0;
const CUT_Z0: f64 = -2.0;
const CUT_HEIGHT: f64 = 20.0;
const CUT_RADIUS: f64 = 5.0;

fn filleted_box(radius: f64) -> Solid {
    let block = make_box(&Pnt::origin(), BOX_W, BOX_H, BOX_D);
    let edge = Edge::between_points(Pnt::new(0.0, 0.0, BOX_D), Pnt::new(BOX_W, 0.0, BOX_D));
    let solid = fillet_edges(&block, &[edge], radius)
        .unwrap_or_else(|e| panic!("front-top edge fillet r={radius} should succeed: {e:?}"));
    assert!(
        solid.is_watertight(),
        "filleted precondition must be watertight"
    );
    assert!(
        solid.health_report().is_healthy(),
        "filleted precondition must be healthy: {:?}",
        solid.health_report().errors
    );
    solid
}

fn smooth_cutter() -> Solid {
    make_cylinder(
        &Ax2::new(Pnt::new(CUT_X, CUT_Y, CUT_Z0), Dir::dz()),
        CUT_RADIUS,
        CUT_HEIGHT,
    )
}

fn faceted_cutter() -> Solid {
    let n = 48usize;
    let pts: Vec<Pnt> = (0..n)
        .map(|i| {
            let a = std::f64::consts::TAU * (i as f64) / (n as f64);
            Pnt::new(
                CUT_X + CUT_RADIUS * a.cos(),
                CUT_Y + CUT_RADIUS * a.sin(),
                CUT_Z0,
            )
        })
        .collect();
    let edges: Vec<Edge> = (0..n)
        .map(|i| Edge::between_points(pts[i], pts[(i + 1) % n]))
        .collect();
    let face = Face::new(
        Some(GeomSurface::Plane(Plane::from_point_normal(
            Pnt::new(CUT_X, CUT_Y, CUT_Z0),
            Dir::dz(),
        ))),
        Wire::from_edges(edges),
    );
    assert!(
        face.outer_wire()
            .expect("faceted cutter face has outer wire")
            .is_closed(),
        "faceted cutter source wire must be closed"
    );
    prism(&face, GeomVec::new(0.0, 0.0, CUT_HEIGHT)).expect("faceted cutter prism should build")
}

fn cracks(s: &Solid) -> usize {
    let mesh = tessellate(s, 0.05, 0.5);
    let gpu = mesh.gpu_mesh();
    type Key = (i64, i64, i64);
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
    for tri in gpu.indices.chunks_exact(3) {
        for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            let (ka, kb) = (q(tri[a] as usize), q(tri[b] as usize));
            let key = if ka <= kb { (ka, kb) } else { (kb, ka) };
            *edges.entry(key).or_insert(0) += 1;
        }
    }
    edges.values().filter(|&&count| count == 1).count()
}

fn has_cylinder_radius(s: &Solid, radius: f64) -> bool {
    s.shell().faces().iter().any(|f| {
        matches!(
            f.surface(),
            Some(GeomSurface::Cylinder(c)) if (c.radius() - radius).abs() < 1.0e-3
        )
    })
}

fn cylinder_radii(s: &Solid) -> Vec<f64> {
    let mut radii: Vec<f64> = s
        .shell()
        .faces()
        .iter()
        .filter_map(|f| match f.surface() {
            Some(GeomSurface::Cylinder(c)) => Some(c.radius()),
            _ => None,
        })
        .collect();
    radii.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    radii
}

fn debug_solid(label: &str, s: &Solid) {
    let report = s.health_report();
    let manifold = s.manifold_report();
    eprintln!(
        "{label}: faces={} edges={} vertices={} euler={} watertight={} healthy={} \
         manifold(total={}, free={}, nonmanifold={}) cylinders={:?} errors={:?} warnings={:?}",
        s.face_count(),
        s.edge_count(),
        s.vertex_count(),
        s.euler_characteristic(),
        s.is_watertight(),
        report.is_healthy(),
        manifold.total_edges,
        manifold.free_edges,
        manifold.nonmanifold_edges,
        cylinder_radii(s),
        report.errors,
        report.warnings,
    );
    if !s.is_watertight() {
        for line in face_lines(s).into_iter().take(32) {
            eprintln!("{label}: {line}");
        }
        for line in face_edge_lines(s).into_iter().take(80) {
            eprintln!("{label}: {line}");
        }
        for line in free_edge_lines(s).into_iter().take(24) {
            eprintln!("{label}: {line}");
        }
    }
}

fn face_lines(s: &Solid) -> Vec<String> {
    s.shell()
        .faces()
        .iter()
        .enumerate()
        .map(|(idx, face)| {
            let kind = match face.surface() {
                Some(GeomSurface::Plane(_)) => "plane".to_string(),
                Some(GeomSurface::Cylinder(c)) => format!("cylinder r={:.4}", c.radius()),
                Some(GeomSurface::Sphere(_)) => "sphere".to_string(),
                Some(GeomSurface::BSpline(_)) => "bspline-surface".to_string(),
                Some(_) => "surface".to_string(),
                None => "none".to_string(),
            };
            let mut lo = [f64::INFINITY; 3];
            let mut hi = [f64::NEG_INFINITY; 3];
            let mut edge_count = 0usize;
            for wire in face.wires() {
                for edge in wire.edges() {
                    edge_count += 1;
                    for p in [edge.start().point(), edge.end().point()] {
                        lo[0] = lo[0].min(p.x());
                        lo[1] = lo[1].min(p.y());
                        lo[2] = lo[2].min(p.z());
                        hi[0] = hi[0].max(p.x());
                        hi[1] = hi[1].max(p.y());
                        hi[2] = hi[2].max(p.z());
                    }
                }
            }
            format!(
                "face {idx}: {kind} edges={edge_count} bbox=({:.3},{:.3},{:.3})..({:.3},{:.3},{:.3})",
                lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]
            )
        })
        .collect()
}

fn face_edge_lines(s: &Solid) -> Vec<String> {
    s.shell()
        .faces()
        .iter()
        .enumerate()
        .flat_map(|(idx, face)| {
            face.wires().into_iter().flat_map(move |wire| {
                wire.edges().into_iter().map(move |edge| {
                    let a = edge.start().point();
                    let b = edge.end().point();
                    let kind = match edge.curve() {
                        Some(openrcad_geom::GeomCurve::Line(_)) => "line",
                        Some(openrcad_geom::GeomCurve::Circle(_)) => "circle",
                        Some(openrcad_geom::GeomCurve::BSpline(_)) => "bspline",
                        Some(_) => "curve",
                        None => "none",
                    };
                    format!(
                        "face {idx} edge {kind} ({:.4},{:.4},{:.4}) -> ({:.4},{:.4},{:.4})",
                        a.x(),
                        a.y(),
                        a.z(),
                        b.x(),
                        b.y(),
                        b.z()
                    )
                })
            })
        })
        .collect()
}

fn free_edge_lines(s: &Solid) -> Vec<String> {
    type Key = ((i64, i64, i64), (i64, i64, i64));
    let q = |p: Pnt| -> (i64, i64, i64) {
        let g = |x: f64| (x * 1.0e6).round() as i64;
        (g(p.x()), g(p.y()), g(p.z()))
    };
    let mut counts: HashMap<Key, u32> = HashMap::new();
    let mut samples: HashMap<Key, (Pnt, Pnt, &'static str, Vec<usize>)> = HashMap::new();
    for (face_idx, face) in s.shell().faces().iter().enumerate() {
        for wire in face.wires() {
            for edge in wire.edges() {
                let a = edge.start().point();
                let b = edge.end().point();
                let qa = q(a);
                let qb = q(b);
                let key = if qa <= qb { (qa, qb) } else { (qb, qa) };
                *counts.entry(key).or_insert(0) += 1;
                let entry = samples.entry(key).or_insert_with(|| {
                    let kind = match edge.curve() {
                        Some(openrcad_geom::GeomCurve::Line(_)) => "line",
                        Some(openrcad_geom::GeomCurve::Circle(_)) => "circle",
                        Some(openrcad_geom::GeomCurve::BSpline(_)) => "bspline",
                        Some(_) => "curve",
                        None => "none",
                    };
                    (a, b, kind, Vec::new())
                });
                entry.3.push(face_idx);
            }
        }
    }
    let mut out: Vec<String> = counts
        .into_iter()
        .filter(|(_, count)| *count == 1)
        .map(|(key, _)| {
            {
                let (a, b, kind, face_ids) = &samples[&key];
                format!(
                    "free {kind} faces={face_ids:?} ({:.4},{:.4},{:.4}) -> ({:.4},{:.4},{:.4})",
                    a.x(),
                    a.y(),
                    a.z(),
                    b.x(),
                    b.y(),
                    b.z()
                )
            }
        })
        .collect();
    out.sort();
    out
}

fn cut_with_debug(label: &str, body: &Solid, tool: &Solid) -> Solid {
    debug_solid(&format!("{label} object"), body);
    debug_solid(&format!("{label} tool"), tool);
    match boolean_checked(body, tool, BooleanOp::Cut) {
        Ok(result) => {
            debug_solid(&format!("{label} checked result"), &result);
            result
        }
        Err(err) => {
            eprintln!("{label}: checked boolean failed: {err}");
            match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                boolean(body, tool, BooleanOp::Cut)
            })) {
                Ok(raw) => debug_solid(&format!("{label} raw result"), &raw),
                Err(_) => eprintln!("{label}: raw boolean also panicked"),
            }
            panic!("{label}: checked boolean failed: {err}");
        }
    }
}

fn non_wall_samples_in_removed_volume(s: &Solid) -> usize {
    let mesh = tessellate(s, 0.05, 0.5);
    let faces = s.shell().faces();
    let inside_void = |p: Pnt| {
        let r = ((p.x() - CUT_X).powi(2) + (p.y() - CUT_Y).powi(2)).sqrt();
        r < CUT_RADIUS - 0.6 && (-0.05..=BOX_D + 0.05).contains(&p.z())
    };
    let cut_wall = |face_id: u32| {
        matches!(
            faces.get(face_id as usize).and_then(|face| face.surface()),
            Some(GeomSurface::Cylinder(c))
                if c.position().direction().dot(&Dir::dz()).abs() > 0.999
                    && (c.radius() - CUT_RADIUS).abs() < 1.0e-3
        )
    };
    let mut count = 0usize;
    for (i, tri) in mesh.triangles.iter().enumerate() {
        if cut_wall(mesh.face_ids.get(i).copied().unwrap_or(0)) {
            continue;
        }
        let a = mesh.vertices[tri[0] as usize];
        let b = mesh.vertices[tri[1] as usize];
        let c = mesh.vertices[tri[2] as usize];
        let p = Pnt::new(
            (a.x() + b.x() + c.x()) / 3.0,
            (a.y() + b.y() + c.y()) / 3.0,
            (a.z() + b.z() + c.z()) / 3.0,
        );
        if inside_void(p) {
            count += 1;
        }
    }
    count
}

fn assert_cut_result(label: &str, result: Solid, fillet_radius: f64, expect_cut_wall: bool) {
    assert!(result.is_watertight(), "{label}: result must be watertight");
    assert!(
        result.health_report().is_healthy(),
        "{label}: result must be healthy: {:?}",
        result.health_report().errors
    );
    assert!(
        has_cylinder_radius(&result, fillet_radius),
        "{label}: remaining fillet cylinder must survive"
    );
    if expect_cut_wall {
        assert!(
            has_cylinder_radius(&result, CUT_RADIUS),
            "{label}: smooth cut wall must remain analytic"
        );
    }
    assert_eq!(
        non_wall_samples_in_removed_volume(&result),
        0,
        "{label}: cut void must not contain refilled/ghost material"
    );
    assert_eq!(
        cracks(&result),
        0,
        "{label}: tessellation must be crack-free"
    );
}

#[test]
fn smooth_cylinder_cuts_through_filleted_box_edge() {
    for radius in [2.0_f64, 3.0] {
        let body = filleted_box(radius);
        let cut = cut_with_debug(&format!("smooth r={radius}"), &body, &smooth_cutter());
        assert_cut_result(&format!("smooth r={radius}"), cut, radius, true);
    }
}

#[test]
fn faceted_cylinder_cuts_through_filleted_box_edge() {
    for radius in [2.0_f64, 3.0] {
        let body = filleted_box(radius);
        let cut = cut_with_debug(&format!("faceted r={radius}"), &body, &faceted_cutter());
        assert_cut_result(&format!("faceted r={radius}"), cut, radius, false);
    }
}
