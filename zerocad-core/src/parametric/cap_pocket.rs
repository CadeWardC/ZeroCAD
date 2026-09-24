//! Exact blind pockets inside the solid core of an analytic threaded shaft.
use super::*;
use openrcad::foundation::{Ax3, Pnt, Vec as GeomVec};
use openrcad::geom::{GeomCurve, GeomSurface};
use openrcad::topo::Face;

pub(super) fn cut(part: &KernelSolid, source: &super::cut::ExactCutSource) -> Option<KernelSolid> {
    let faces = part.faces();
    // Coaxial rails with bounded linear UV trims enclose a solid core.
    // Interrupted, hollow, and freeform shafts use the general kernel path.
    let rail = |curve: &GeomCurve, u0: f64, u1: f64| -> Option<(Ax3, f64)> {
        match curve {
            GeomCurve::Circle(c) => Some((c.position(), c.radius())),
            GeomCurve::Helix(h) => Some((h.position(), h.radius_at(u0).min(h.radius_at(u1)))),
            _ => None,
        }
    };
    let frame = faces.iter().find_map(|f| match f.surface()? {
        GeomSurface::Ruled(r) => rail(&r.curve1, 0.0, 0.0).map(|r| r.0),
        _ => None,
    })?;
    let axis = GeomVec::from_dir(frame.direction());
    let origin = frame.location();
    let aligned = |p: Ax3| {
        let delta = p.location() - origin;
        (delta - axis * delta.dot(&axis)).magnitude() < 1e-6
            && GeomVec::from_dir(p.direction()).dot(&axis) > 1.0 - 1e-7
    };
    let mut radius = f64::INFINITY;
    let mut caps = Vec::new();
    for (i, face) in faces.iter().enumerate() {
        match face.surface()? {
            GeomSurface::Plane(p)
                if face.inner_wires().is_empty()
                    && GeomVec::from_dir(p.normal()).dot(&axis).abs() > 1.0 - 1e-7 =>
            {
                caps.push((i, (p.location() - origin).dot(&axis)));
            }
            GeomSurface::Cylinder(c) if aligned(c.position()) => radius = radius.min(c.radius()),
            GeomSurface::Ruled(r) => {
                let mut u0 = f64::INFINITY;
                let mut u1 = f64::NEG_INFINITY;
                for wire in face.wires() {
                    for i in 0..wire.edges().len() {
                        let pc = wire.pcurve(i)?;
                        if !matches!(pc.curve, openrcad::geom2d::GeomCurve2d::Line(_)) {
                            return None;
                        }
                        for p in [pc.point_at_fraction(0.0), pc.point_at_fraction(1.0)] {
                            if p.y() < -1e-7 || p.y() > 1.0 + 1e-7 {
                                return None;
                            }
                            u0 = u0.min(p.x());
                            u1 = u1.max(p.x());
                        }
                    }
                }
                if !u0.is_finite() || !u1.is_finite() {
                    return None;
                }
                let (a, ra) = rail(&r.curve1, u0, u1)?;
                let (b, rb) = rail(&r.curve2, u0, u1)?;
                if !aligned(a)
                    || !aligned(b)
                    || GeomVec::from_dir(a.x_direction()).dot(&GeomVec::from_dir(b.x_direction()))
                        < 1.0 - 1e-7
                    || GeomVec::from_dir(a.y_direction()).dot(&GeomVec::from_dir(b.y_direction()))
                        < 1.0 - 1e-7
                {
                    return None;
                }
                radius = radius.min(ra).min(rb);
            }
            _ => return None,
        }
    }
    if caps.len() != 2 || radius <= 0.0 || !radius.is_finite() {
        return None;
    }
    let n = GeomVec::new(
        source.cs.n.x as f64,
        source.cs.n.y as f64,
        source.cs.n.z as f64,
    );
    if n.dot(&axis).abs() < 1.0 - 1e-7 {
        return None;
    }
    let start = Pnt::new(
        source.cs.origin.x as f64,
        source.cs.origin.y as f64,
        source.cs.origin.z as f64,
    );
    let z = (start - origin).dot(&axis);
    let end = z + n.dot(&axis) * source.depth as f64;
    let &(cap_index, _) = caps.iter().find(|(_, h)| (h - z).abs() < 1e-6)?;
    let lo = caps[0].1.min(caps[1].1);
    let hi = caps[0].1.max(caps[1].1);
    if end <= lo + 1e-6 || end >= hi - 1e-6 {
        return None;
    }
    let (wire, holes) = crate::mock_kernel::analytic_region_wires(&source.region, &source.cs)?;
    if !holes.is_empty() {
        return None;
    }
    for edge in wire.edges() {
        if !matches!(edge.curve(), Some(GeomCurve::Line(_))) {
            return None;
        }
        for p in [edge.start().point(), edge.end().point()] {
            let v = p - origin;
            if (v - axis * v.dot(&axis)).magnitude() >= radius - 1e-5 {
                return None;
            }
        }
    }
    let tool = crate::mock_kernel::extruded_sketch_region_solid(
        &source.region,
        source.depth,
        &source.cs,
        &source.arc_circles,
    )?;
    let mut result = faces.clone();
    let cap = &faces[cap_index];
    let outer = cap.outer_wire()?;
    let sign = super::cut::wire_signed_area(&outer, &source.cs).signum();
    let hole = super::cut::orient_wire_like(wire, sign, false, &source.cs);
    result[cap_index] = Face::with_wires(
        cap.surface().cloned(),
        Some(outer),
        vec![hole],
        cap.orientation(),
    );
    for face in tool.faces() {
        if super::cut::planar_section_height(&face, &source.cs).is_some_and(|h| h.abs() < 1e-6) {
            continue;
        }
        result.push(face.reversed());
    }
    let shell =
        openrcad::algo::sew_with_policy(&result, &openrcad::foundation::TolerancePolicy::STANDARD)
            .ok()?
            .value;
    Some(KernelSolid::new(shell))
}
