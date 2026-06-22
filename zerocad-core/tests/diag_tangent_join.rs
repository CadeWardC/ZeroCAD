//! TEMP diagnostic (Stage 0 of the tangent-wrap plan): verify that the cylinder
//! blend over edge E and the torus blend over the tangent neighbour edge E' share
//! the SAME cross-arc at the junction p1 — the premise of the shared-arc join.
//! Remove once the construction is grounded.

use std::collections::HashSet;
use openrcad::algo::{rolling_ball_fillet_edge, RollingBallBlend};
use openrcad::foundation::Pnt;
use openrcad::geom::{GeomCurve, GeomSurface};
use openrcad::topo::{Edge, Solid};
use zerocad_core::read_zcad;

fn arc_desc(e: &Edge) -> String {
    let (s, t) = (e.source().point(), e.target().point());
    let kind = match e.curve() {
        Some(GeomCurve::Line(_)) => "Line",
        Some(GeomCurve::Circle(c)) => {
            return format!(
                "Circle r={:.4} center=({:.3},{:.3},{:.3}) {:?}->{:?}",
                c.radius(),
                c.center().x(), c.center().y(), c.center().z(),
                (s.x(), s.y(), s.z()), (t.x(), t.y(), t.z())
            )
        }
        Some(GeomCurve::Ellipse(_)) => "Ellipse",
        _ => "other",
    };
    format!("{kind} {:?}->{:?}", (s.x(), s.y(), s.z()), (t.x(), t.y(), t.z()))
}

fn nearest_arc(blend: &RollingBallBlend, p: Pnt) -> &Edge {
    let ds = blend
        .start_arc
        .source()
        .point()
        .distance(&p)
        .min(blend.start_arc.target().point().distance(&p));
    let de = blend
        .end_arc
        .source()
        .point()
        .distance(&p)
        .min(blend.end_arc.target().point().distance(&p));
    if ds <= de {
        &blend.start_arc
    } else {
        &blend.end_arc
    }
}

#[test]
fn diag_tangent_join_arcs_coincide() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../fillet_problem.zcad");
    let Ok(bytes) = std::fs::read(path) else {
        eprintln!("skipping: {path} not found");
        return;
    };
    let loaded = read_zcad(&bytes).expect("parse .zcad");
    let mut hidden = HashSet::new();
    for node in loaded.graph.graph.node_weights() {
        if matches!(node.feature, zerocad_core::FeatureType::EdgeMod { .. }) {
            hidden.insert(node.id.clone());
        }
    }
    let solids = loaded.graph.debug_kernel_solids(&hidden).expect("pre-fillet body");
    let solid: &Solid = &solids[0].1[0];

    let r = 0.7210718_f64;
    let p1 = Pnt::new(-3.64, 11.908, 3.8);
    let near = |a: Pnt, b: Pnt| a.distance(&b) < 0.05;

    // E: the straight edge between the two planes, with an endpoint at p1.
    let e = solid
        .edges()
        .into_iter()
        .find(|e| {
            matches!(e.curve(), Some(GeomCurve::Line(_)))
                && (near(e.source().point(), p1) || near(e.target().point(), p1))
                && (e.source().point().z() - 3.8).abs() < 0.05
                && (e.target().point().z() - 3.8).abs() < 0.05
        })
        .expect("edge E");
    // E': the circular neighbour arc with an endpoint at p1 (face6 ∩ face7).
    let ep = solid
        .edges()
        .into_iter()
        .find(|e| {
            matches!(e.curve(), Some(GeomCurve::Circle(_)))
                && (near(e.source().point(), p1) || near(e.target().point(), p1))
        })
        .expect("edge E'");

    println!("E  = {}", arc_desc(&e));
    println!("E' = {}", arc_desc(&ep));

    // Tangent directions at p1 (unit), to test whether E and E' are G1-tangent.
    use openrcad::geom::Curve;
    // Finite-difference tangent from sampled curve points (avoids any d1/param
    // ambiguity): direction from the endpoint at p toward a point a little along.
    // Robust: pick the PARAM whose evaluated point is actually nearest p (not the
    // source/target vertex, whose mapping to first/last can be reversed), then step
    // toward the other param end.
    let tangent_at = |edge: &Edge, p: Pnt| -> (f64, f64, f64) {
        let c = edge.curve().unwrap();
        let (u0, u1) = (edge.first(), edge.last());
        let (ua, ub) = if c.point(u0).distance(&p) <= c.point(u1).distance(&p) {
            (u0, u0 + (u1 - u0) * 1e-4)
        } else {
            (u1, u1 + (u0 - u1) * 1e-4)
        };
        let d = c.point(ub) - c.point(ua);
        let m = d.magnitude().max(1e-12);
        (d.x() / m, d.y() / m, d.z() / m)
    };
    // Analytic cross-check for the circle: tangent ⊥ radius, in the circle plane.
    let circle_tangent_at = |edge: &Edge, p: Pnt| -> Option<(f64, f64, f64)> {
        let Some(GeomCurve::Circle(circ)) = edge.curve() else { return None };
        let rad = p - circ.center();
        let n = circ.axis();
        // t = n × rad (perpendicular to radius, in the plane)
        let t = (
            n.y() * rad.z() - n.z() * rad.y(),
            n.z() * rad.x() - n.x() * rad.z(),
            n.x() * rad.y() - n.y() * rad.x(),
        );
        let m = (t.0 * t.0 + t.1 * t.1 + t.2 * t.2).sqrt().max(1e-12);
        Some((t.0 / m, t.1 / m, t.2 / m))
    };
    let te = tangent_at(&e, p1);
    let tep = tangent_at(&ep, p1);
    let dot = (te.0 * tep.0 + te.1 * tep.1 + te.2 * tep.2).abs();
    println!("E  tangent@p1 = ({:.3},{:.3},{:.3})", te.0, te.1, te.2);
    println!("E' tangent@p1 (sampled) = ({:.3},{:.3},{:.3})", tep.0, tep.1, tep.2);
    println!(">>> |E·E'| at p1 (sampled) = {dot:.4}  (1=tangent/G1, 0=perpendicular/sharp)");
    if let Some(ta) = circle_tangent_at(&ep, p1) {
        let dota = (te.0 * ta.0 + te.1 * ta.1 + te.2 * ta.2).abs();
        println!("E' tangent@p1 (analytic n×r) = ({:.3},{:.3},{:.3})", ta.0, ta.1, ta.2);
        println!(">>> |E·E'| at p1 (ANALYTIC) = {dota:.4}");
    }
    // Ground truth: sample E' across [first,last] and print the actual swept points.
    {
        let c = ep.curve().unwrap();
        let (u0, u1) = (ep.first(), ep.last());
        println!("E' params first={u0:.4} last={u1:.4}; sampled points:");
        for i in 0..=6 {
            let u = u0 + (u1 - u0) * (i as f64) / 6.0;
            let p = c.point(u);
            println!("   u={:.4} -> ({:.3},{:.3},{:.3})", u, p.x(), p.y(), p.z());
        }
    }

    // --- Roll-over corner-patch validation ---------------------------------
    // Pivot point P0 = the convex edge l (x=-3.64, z=3.8) offset r into the common
    // far-cap face (y: 11.908 -> 11.908-r). E's ball centre C1 and E''s ball centre
    // C2 should both sit at distance r from P0, 90deg apart -> a torus corner patch.
    {
        let p0pt = Pnt::new(-3.64, 11.908 - r, 3.8);
        let c1 = Pnt::new(-3.64, 11.908 - r, 3.8 - r); // E: tangent to top(z=3.8) & cap
        let c2 = Pnt::new(-3.64 + r, 11.908 - r, 3.8); // E': tangent to wall(x=-3.64) & cap
        let d1 = c1 - p0pt;
        let d2 = c2 - p0pt;
        let n1 = d1.magnitude();
        let n2 = d2.magnitude();
        let ang = (d1.dot(&d2) / (n1 * n2)).acos().to_degrees();
        println!("P0={:?} |C1-P0|={:.4} |C2-P0|={:.4} (r={:.4}) pivot_angle={:.2}deg",
            (p0pt.x(), p0pt.y(), p0pt.z()), n1, n2, r, ang);
    }

    let be = rolling_ball_fillet_edge(solid, &e, r).expect("cylinder blend over E");
    let bep = rolling_ball_fillet_edge(solid, &ep, r);
    let _ = nearest_arc(&be, p1);
    println!("E blend: face={:?}", surf_kind(be.blend_face.surface()));
    println!("   E.start_arc = {}", arc_desc(&be.start_arc));
    println!("   E.end_arc   = {}", arc_desc(&be.end_arc));
    println!("   E.contact_a = {}", arc_desc(&be.contact_a));
    println!("   E.contact_b = {}", arc_desc(&be.contact_b));
    match &bep {
        Ok(b) => {
            println!("E' blend: face={:?}", surf_kind(b.blend_face.surface()));
            println!("   E'.start_arc = {}", arc_desc(&b.start_arc));
            println!("   E'.end_arc   = {}", arc_desc(&b.end_arc));
            println!("   E'.contact_a = {}", arc_desc(&b.contact_a));
            println!("   E'.contact_b = {}", arc_desc(&b.contact_b));
        }
        Err(err) => println!("E' blend FAILED: {err:?}"),
    }
}

fn surf_kind(s: Option<&GeomSurface>) -> &'static str {
    match s {
        Some(GeomSurface::Plane(_)) => "Plane",
        Some(GeomSurface::Cylinder(_)) => "Cylinder",
        Some(GeomSurface::Torus(_)) => "Torus",
        Some(GeomSurface::Sphere(_)) => "Sphere",
        Some(GeomSurface::Gregory(_)) => "Gregory",
        Some(GeomSurface::Ruled(_)) => "Ruled",
        _ => "other",
    }
}
