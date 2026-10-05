//! Parameter-space adaptive subdivision intersection solvers with Newton-Raphson refinement.
//! Respects local tolerances and handles curve-curve, curve-surface, and surface-surface cases.

use openrcad_foundation::{Ax3, Dir, Pnt, Vec as GeomVec};
use openrcad_geom::{
    BSplineCurve, Circle, ConicalSurface, Curve, CylindricalSurface, Ellipse, GeomCurve,
    GeomSurface, Plane, Surface, ToroidalSurface, TorusPlaneSection,
};
use openrcad_topo::{containment::point_in_polygon_2d, Face, Orientation};

use core::f64::consts::PI;

use crate::band_topology::{BandTopologyError, GeometryWorkBudget, GeometryWorkStage};

/// Wrap an angle into `[0, 2π)`.
#[inline]
fn norm_angle(a: f64) -> f64 {
    let t = 2.0 * PI;
    let mut x = a % t;
    if x < 0.0 {
        x += t;
    }
    x
}

/// Real roots of `a x² + b x + c = 0` (0, 1, or 2 of them).
fn solve_quadratic(a: f64, b: f64, c: f64) -> Vec<f64> {
    if a.abs() < 1e-14 {
        // Linear: b x + c = 0.
        if b.abs() < 1e-14 {
            return Vec::new();
        }
        return vec![-c / b];
    }
    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        Vec::new()
    } else {
        let s = disc.sqrt();
        let q = -0.5 * (b + b.signum() * s);
        let r1 = q / a;
        if q.abs() < 1e-300 {
            vec![r1]
        } else {
            let r2 = c / q;
            if (r1 - r2).abs() < 1e-12 {
                vec![r1]
            } else {
                vec![r1, r2]
            }
        }
    }
}

/// Deterministic real roots of a polynomial in descending coefficient order.
///
/// Recursive derivative isolation partitions the real line into monotone
/// intervals. Sign-changing intervals contain one root and are bisected;
/// critical points are checked separately so even-multiplicity tangencies are
/// retained. All zero/dedup thresholds are derived from coefficient arithmetic
/// and machine precision rather than a model-unit floor.
fn real_polynomial_roots(coefficients: &[f64]) -> Vec<f64> {
    let coefficient_scale = coefficients
        .iter()
        .map(|coefficient| coefficient.abs())
        .fold(0.0, f64::max);
    if !coefficient_scale.is_finite() || coefficient_scale == 0.0 {
        return Vec::new();
    }
    let first = coefficients
        .iter()
        // Coefficients carry different physical powers of the line parameter;
        // comparing a leading coefficient with the constant term invents a
        // unit scale and incorrectly reduces degree for large geometry.
        .position(|coefficient| *coefficient != 0.0)
        .unwrap_or(coefficients.len());
    if first == coefficients.len() {
        return Vec::new();
    }
    let normalized: Vec<_> = coefficients[first..]
        .iter()
        .map(|coefficient| coefficient / coefficient_scale)
        .collect();
    let degree = normalized.len() - 1;
    if degree == 0 {
        return Vec::new();
    }
    if degree == 1 {
        return vec![-normalized[1] / normalized[0]];
    }

    let derivative: Vec<_> = normalized[..degree]
        .iter()
        .enumerate()
        .map(|(index, coefficient)| coefficient * (degree - index) as f64)
        .collect();
    let mut critical = real_polynomial_roots(&derivative);
    critical.sort_by(f64::total_cmp);
    // Fujiwara's bound accounts for each coefficient's parameter power. A
    // plain Cauchy max would give a length^4 constant term the same weight as
    // a length coefficient, producing an enormous artificial bound and then
    // deduplicating distinct roots at large scales.
    let bound = 2.0
        * normalized[1..]
            .iter()
            .enumerate()
            .map(|(index, coefficient)| {
                (coefficient / normalized[0])
                    .abs()
                    .powf(1.0 / (index + 1) as f64)
            })
            .fold(f64::MIN_POSITIVE, f64::max);
    let evaluate = |value: f64| {
        normalized
            .iter()
            .fold(0.0, |result, coefficient| result * value + coefficient)
    };
    let evaluation_tolerance = |value: f64| {
        let magnitude = normalized.iter().fold(0.0, |result, coefficient| {
            result * value.abs() + coefficient.abs()
        });
        magnitude * f64::EPSILON * 256.0
    };

    let mut roots = Vec::new();
    for &point in &critical {
        if point >= -bound && point <= bound && evaluate(point).abs() <= evaluation_tolerance(point)
        {
            roots.push(point);
        }
    }
    let mut partitions = Vec::with_capacity(critical.len() + 2);
    partitions.push(-bound);
    partitions.extend(
        critical
            .into_iter()
            .filter(|point| *point > -bound && *point < bound),
    );
    partitions.push(bound);
    for interval in partitions.windows(2) {
        let (mut left, mut right) = (interval[0], interval[1]);
        let mut left_value = evaluate(left);
        let right_value = evaluate(right);
        if left_value == 0.0 {
            roots.push(left);
            continue;
        }
        if right_value == 0.0 {
            roots.push(right);
            continue;
        }
        if left_value.is_sign_positive() == right_value.is_sign_positive() {
            continue;
        }
        for _ in 0..96 {
            let middle = left * 0.5 + right * 0.5;
            let middle_value = evaluate(middle);
            if middle_value == 0.0 {
                left = middle;
                right = middle;
                break;
            }
            if left_value.is_sign_positive() == middle_value.is_sign_positive() {
                left = middle;
                left_value = middle_value;
            } else {
                right = middle;
            }
        }
        roots.push(left * 0.5 + right * 0.5);
    }
    roots.sort_by(f64::total_cmp);
    let dedup_tolerance = bound * f64::EPSILON * 4096.0;
    roots.dedup_by(|left, right| (*left - *right).abs() <= dedup_tolerance);
    roots
}

/// Exact line intersection with a regular torus via its implicit quartic.
fn line_torus(origin: &Pnt, direction: &GeomVec, torus: &ToroidalSurface) -> Vec<Pnt> {
    let frame = torus.position();
    let offset = *origin - frame.location();
    let x_axis = GeomVec::from_dir(frame.x_direction());
    let y_axis = GeomVec::from_dir(frame.y_direction());
    let z_axis = GeomVec::from_dir(frame.direction());
    let mut local_origin = GeomVec::new(
        offset.dot(&x_axis),
        offset.dot(&y_axis),
        offset.dot(&z_axis),
    );
    let local_direction = GeomVec::new(
        direction.dot(&x_axis),
        direction.dot(&y_axis),
        direction.dot(&z_axis),
    );
    let direction_squared = local_direction.dot(&local_direction);
    if direction_squared <= f64::MIN_POSITIVE {
        return Vec::new();
    }
    // Form the quartic around the point on the line nearest the torus frame,
    // not around the caller's arbitrary ray origin. Otherwise a unit torus at
    // a 1e9 translation produces coefficients spanning ~36 decimal orders and
    // spurious real roots before any root-isolation tolerance is involved.
    let parameter_shift = -local_origin.dot(&local_direction) / direction_squared;
    local_origin += local_direction * parameter_shift;
    let origin_direction = 2.0 * local_origin.dot(&local_direction);
    let radius_term = local_origin.dot(&local_origin) + torus.major_radius() * torus.major_radius()
        - torus.minor_radius() * torus.minor_radius();
    let radial_origin_squared =
        local_origin.x() * local_origin.x() + local_origin.y() * local_origin.y();
    let radial_origin_direction =
        2.0 * (local_origin.x() * local_direction.x() + local_origin.y() * local_direction.y());
    let radial_direction_squared =
        local_direction.x() * local_direction.x() + local_direction.y() * local_direction.y();
    let four_major_squared = 4.0 * torus.major_radius() * torus.major_radius();
    let coefficients = [
        direction_squared * direction_squared,
        2.0 * direction_squared * origin_direction,
        origin_direction * origin_direction + 2.0 * direction_squared * radius_term
            - four_major_squared * radial_direction_squared,
        2.0 * origin_direction * radius_term - four_major_squared * radial_origin_direction,
        radius_term * radius_term - four_major_squared * radial_origin_squared,
    ];
    real_polynomial_roots(&coefficients)
        .into_iter()
        .map(|parameter| *origin + *direction * (parameter_shift + parameter))
        .collect()
}

/// The analytic surface parameters `(u, v)` of a 3D point `p`.
///
/// Exact for the analytic surfaces (plane/cylinder/cone/sphere); falls back to
/// Newton iteration for B-splines. The angular parameter `u` is returned in
/// `[0, 2π)` for the rotational surfaces.
pub fn uv_of(s: &GeomSurface, p: &Pnt) -> (f64, f64) {
    match s {
        GeomSurface::Plane(pl) => {
            let d = *p - pl.location();
            let u = d.dot(&GeomVec::from_dir(pl.position().x_direction()));
            let v = d.dot(&GeomVec::from_dir(pl.position().y_direction()));
            (u, v)
        }
        GeomSurface::Cylinder(cy) => {
            let (u, v, _) = axial_uv(&cy.position(), p);
            (u, v)
        }
        GeomSurface::Cone(co) => {
            let (u, v, _) = axial_uv(&co.position(), p);
            (u, v)
        }
        GeomSurface::Sphere(sp) => {
            let pos = sp.position();
            let c = pos.location();
            let w = GeomVec::from_dir(pos.direction());
            let x = GeomVec::from_dir(pos.x_direction());
            let y = GeomVec::from_dir(pos.y_direction());
            let d = *p - c;
            let r = sp.radius().max(1e-12);
            let v = (d.dot(&w) / r).clamp(-1.0, 1.0).asin();
            let u = norm_angle(d.dot(&y).atan2(d.dot(&x)));
            (u, v)
        }
        GeomSurface::Torus(t) => {
            // u = angle about the axis; v = angle around the tube measured from the
            // outer equator in the (radial, axis) half-plane through p.
            let pos = t.position();
            let w = GeomVec::from_dir(pos.direction());
            let x = GeomVec::from_dir(pos.x_direction());
            let y = GeomVec::from_dir(pos.y_direction());
            let d = *p - pos.location();
            let axial = d.dot(&w);
            let radial = d - w * axial;
            let u = norm_angle(radial.dot(&y).atan2(radial.dot(&x)));
            let v = norm_angle(axial.atan2(radial.magnitude() - t.major_radius()));
            (u, v)
        }
        GeomSurface::Ruled(r) => match r.helical_uv_hinted(*p, None) {
            // Helical thread bands span many turns; Newton from (0, 0) lands
            // on the wrong turn, so use the analytic projection.
            Some(uv) => uv,
            None => search_nearest_parameter_newton(s, p, (0.0, 0.0)),
        },
        GeomSurface::Offset(offset) => match offset.base.as_ref() {
            GeomSurface::Plane(_) | GeomSurface::Cylinder(_) | GeomSurface::Torus(_) => {
                uv_of(&offset.base, p)
            }
            GeomSurface::Cone(cone) => {
                let (u, axial, _) = axial_uv(&cone.position(), p);
                (u, axial + offset.distance * cone.semi_angle().sin())
            }
            GeomSurface::Sphere(sphere) => {
                let pos = sphere.position();
                let d = *p - sphere.center();
                let magnitude = d.magnitude().max(f64::MIN_POSITIVE);
                let w = GeomVec::from_dir(pos.direction());
                let x = GeomVec::from_dir(pos.x_direction());
                let y = GeomVec::from_dir(pos.y_direction());
                (
                    norm_angle(d.dot(&y).atan2(d.dot(&x))),
                    (d.dot(&w) / magnitude).clamp(-1.0, 1.0).asin(),
                )
            }
            _ => search_nearest_parameter_newton(s, p, (0.0, 0.0)),
        },
        GeomSurface::BSpline(_) | GeomSurface::Gregory(_) => {
            search_nearest_parameter_newton(s, p, (0.0, 0.0))
        }
    }
}

/// Angular/axial parameters of `p` relative to an axis frame: `u` (angle about
/// the axis, in `[0, 2π)`), `v` (signed distance along the axis), and the radial
/// distance from the axis.
fn axial_uv(pos: &Ax3, p: &Pnt) -> (f64, f64, f64) {
    let a = pos.location();
    let w = GeomVec::from_dir(pos.direction());
    let x = GeomVec::from_dir(pos.x_direction());
    let y = GeomVec::from_dir(pos.y_direction());
    let d = *p - a;
    let v = d.dot(&w);
    let radial = d - w * v;
    let u = norm_angle(radial.dot(&y).atan2(radial.dot(&x)));
    (u, v, radial.magnitude())
}

/// Intersect an infinite line (`origin` + t·`dir`) with an analytic surface,
/// returning every intersection point (untrimmed). Returns `None` for B-splines,
/// signalling the caller to fall back to the generic subdivision solver.
pub fn line_surface(origin: &Pnt, dir: &GeomVec, s: &GeomSurface) -> Option<Vec<Pnt>> {
    let pt = |t: f64| *origin + *dir * t;
    match s {
        GeomSurface::Plane(pl) => {
            let n = GeomVec::from_dir(pl.normal());
            let denom = dir.dot(&n);
            if denom.abs() < 1e-12 {
                return Some(Vec::new());
            }
            let t = (pl.location() - *origin).dot(&n) / denom;
            Some(vec![pt(t)])
        }
        GeomSurface::Cylinder(cy) => {
            let pos = cy.position();
            let a = pos.location();
            let w = GeomVec::from_dir(pos.direction());
            let oc = *origin - a;
            let oc_perp = oc - w * oc.dot(&w);
            let d_perp = *dir - w * dir.dot(&w);
            let qa = d_perp.dot(&d_perp);
            let qb = 2.0 * oc_perp.dot(&d_perp);
            let qc = oc_perp.dot(&oc_perp) - cy.radius() * cy.radius();
            Some(solve_quadratic(qa, qb, qc).into_iter().map(pt).collect())
        }
        GeomSurface::Sphere(sp) => {
            let oc = *origin - sp.center();
            let qa = dir.dot(dir);
            let qb = 2.0 * oc.dot(dir);
            let qc = oc.dot(&oc) - sp.radius() * sp.radius();
            Some(solve_quadratic(qa, qb, qc).into_iter().map(pt).collect())
        }
        GeomSurface::Cone(co) => {
            let pos = co.position();
            let a = pos.location();
            let w = GeomVec::from_dir(pos.direction());
            let slope = co.semi_angle().tan();
            let oc = *origin - a;
            let oc_perp = oc - w * oc.dot(&w);
            let d_perp = *dir - w * dir.dot(&w);
            let h0 = oc.dot(&w);
            let hd = dir.dot(&w);
            let a0 = co.ref_radius() + slope * h0;
            let ad = slope * hd;
            let qa = d_perp.dot(&d_perp) - ad * ad;
            let qb = 2.0 * oc_perp.dot(&d_perp) - 2.0 * a0 * ad;
            let qc = oc_perp.dot(&oc_perp) - a0 * a0;
            Some(solve_quadratic(qa, qb, qc).into_iter().map(pt).collect())
        }
        GeomSurface::Torus(torus) => Some(line_torus(origin, dir, torus)),
        GeomSurface::BSpline(_)
        | GeomSurface::Gregory(_)
        | GeomSurface::Offset(_)
        | GeomSurface::Ruled(_) => None,
    }
}

#[inline]
fn clamp_bound(val: f64, default: f64) -> f64 {
    if val.is_infinite() || val.is_nan() {
        default
    } else {
        val
    }
}

fn clamp_ordered(value: f64, min: f64, max: f64) -> f64 {
    match (min.is_finite(), max.is_finite()) {
        (true, true) => {
            let (lo, hi) = if min <= max { (min, max) } else { (max, min) };
            if value.is_nan() {
                lo
            } else {
                value.max(lo).min(hi)
            }
        }
        (true, false) => {
            if value.is_nan() {
                min
            } else {
                value.max(min)
            }
        }
        (false, true) => {
            if value.is_nan() {
                max
            } else {
                value.min(max)
            }
        }
        (false, false) => value,
    }
}

/// Nearest parameter to `p` on `c` within its bounds: coarse sample sweep then a
/// few Newton steps. Used only by [`curves_overlap`].
fn nearest_param_on_curve(c: &GeomCurve, p: &Pnt) -> f64 {
    if let GeomCurve::Line(_) = c {
        let (origin, direction) = c.d1(0.0);
        return (*p - origin).dot(&direction) / direction.dot(&direction);
    }
    let (a, b) = c.bounds();
    let a = clamp_bound(a, -100.0);
    let b = clamp_bound(b, 100.0);
    let steps = 12;
    let mut best_t = a;
    let mut best_d = p.distance(&c.point(a));
    for i in 0..=steps {
        let t = a + (b - a) * (i as f64) / steps as f64;
        let d = p.distance(&c.point(t));
        if d < best_d {
            best_d = d;
            best_t = t;
        }
    }
    let mut t = best_t;
    for _ in 0..6 {
        let (pt, tan) = c.d1(t);
        let diff = pt - *p;
        let dt = diff.dot(&tan) / (tan.dot(&tan) + 1e-15);
        t = clamp_ordered(t - dt, a, b);
    }
    t
}

/// Coincidence/overlap test: does every point of a spread on `c1` lie on `c2`
/// within `tol`? If so the two curves share supporting geometry and have no
/// *isolated* crossing — the caller returns no points and skips the subdivision
/// search, which would otherwise never prune (coincident bounding boxes always
/// overlap) and recurse to its depth cap. This is the dominant cost when a
/// boolean intersection curve coincides with a face's own boundary arc.
fn curves_overlap(c1: &GeomCurve, c2: &GeomCurve, tol: f64) -> bool {
    let (a, b) = c1.bounds();
    let a = clamp_bound(a, -100.0);
    let b = clamp_bound(b, 100.0);
    if (b - a).abs() < tol {
        return false; // degenerate span; leave it to the subdivision solver
    }
    const N: usize = 8;
    for i in 0..N {
        let t = a + (b - a) * ((i as f64) + 0.5) / N as f64;
        let p = c1.point(t);
        let s = nearest_param_on_curve(c2, &p);
        if c2.point(s).distance(&p) > tol {
            return false;
        }
    }
    true
}

/// Closed-form crossing of two infinite 3D lines: the midpoint of their closest
/// approach when that gap is within `tol`, else no point (skew or parallel —
/// coincident lines are caught upstream by [`curves_overlap`]).
fn line_line(a: &openrcad_geom::Line, b: &openrcad_geom::Line, tol: f64) -> Vec<Pnt> {
    let p1 = a.location();
    let p2 = b.location();
    let d1 = GeomVec::from_dir(a.direction());
    let d2 = GeomVec::from_dir(b.direction());
    let w0 = p1 - p2;
    let aa = d1.dot(&d1);
    let bb = d1.dot(&d2);
    let cc = d2.dot(&d2);
    let dd = d1.dot(&w0);
    let ee = d2.dot(&w0);
    let denom = aa * cc - bb * bb;
    if denom.abs() < 1e-14 {
        return Vec::new(); // parallel; coincidence handled by curves_overlap
    }
    let t = (bb * ee - cc * dd) / denom;
    let s = (aa * ee - bb * dd) / denom;
    let pa = p1 + d1 * t;
    let pb = p2 + d2 * s;
    if pa.distance(&pb) <= tol {
        vec![pa.midpoint(&pb)]
    } else {
        Vec::new()
    }
}

/// Closed-form intersection of an infinite line with a circle (0, 1, or 2 pts).
/// A line generically pierces the circle's plane once (1 pt iff it lands on the
/// circle); a line lying in the plane reduces to a 2D line/circle quadratic.
fn line_circle(l: &openrcad_geom::Line, ci: &Circle, tol: f64) -> Vec<Pnt> {
    // Planes and rims built from single-precision sketch points differ by a
    // rounding (1e-7 tilts and offsets), so "the line lies in the circle's
    // plane" is judged at the intersection tolerance over the circle's span;
    // the in-plane solve below uses only in-plane components.
    const IN_PLANE: f64 = 1.0e-5;
    let p = l.location();
    let d = GeomVec::from_dir(l.direction());
    let c = ci.center();
    let n = GeomVec::from_dir(ci.axis());
    let r = ci.radius();
    let on_tol = tol.max(1e-7);
    let dn = d.dot(&n);
    // Where the line passes the circle's centre, and how far it drifts out of
    // the plane across the circle.
    let foot = p + d * (c - p).dot(&d);
    let in_plane =
        dn.abs() * 2.0 * r <= IN_PLANE && (foot - c).dot(&n).abs() <= on_tol.max(IN_PLANE);
    if !in_plane {
        if dn.abs() <= 1e-12 {
            return Vec::new();
        }
        // Pierces the plane at one point; keep it only if it lands on the circle.
        let t = (c - p).dot(&n) / dn;
        let pt = p + d * t;
        if ((pt - c).magnitude() - r).abs() <= on_tol {
            return vec![pt];
        }
        return Vec::new();
    }
    let x = GeomVec::from_dir(ci.position().x_direction());
    let y = GeomVec::from_dir(ci.position().y_direction());
    let po = p - c;
    let (px, py) = (po.dot(&x), po.dot(&y));
    let (dx, dy) = (d.dot(&x), d.dot(&y));
    let qa = dx * dx + dy * dy;
    let qb = 2.0 * (px * dx + py * dy);
    let qc = px * px + py * py - r * r;
    solve_quadratic(qa, qb, qc)
        .into_iter()
        .map(|t| p + d * t)
        .collect()
}

/// Closed-form line/ellipse intersection, as [`line_circle`]. Without it the
/// pair fell to the subdivision search, which clamps an infinite line to
/// parameters [-100, 100]: a tilted cylinder's ruling, parameterized from a
/// distant axis origin, crosses its planar section beyond that window, and the
/// trim then fell back to a bisected containment transition 1e-3 off the true
/// crossing (a dangling spur on a tilted slot cap).
fn line_ellipse(l: &openrcad_geom::Line, ellipse: &Ellipse, tol: f64) -> Vec<Pnt> {
    const IN_PLANE: f64 = 1.0e-5;
    let p = l.location();
    let d = GeomVec::from_dir(l.direction());
    let position = ellipse.position();
    let c = ellipse.center();
    let n = GeomVec::from_dir(position.direction());
    let x = GeomVec::from_dir(position.x_direction());
    let y = GeomVec::from_dir(position.y_direction());
    let (a, b) = (ellipse.major_radius(), ellipse.minor_radius());
    if a <= 0.0 || b <= 0.0 {
        return Vec::new();
    }
    let on_tol = tol.max(1e-7);
    let dn = d.dot(&n);
    let foot = p + d * (c - p).dot(&d);
    let in_plane =
        dn.abs() * 2.0 * a <= IN_PLANE && (foot - c).dot(&n).abs() <= on_tol.max(IN_PLANE);
    if !in_plane {
        if dn.abs() <= 1e-12 {
            return Vec::new();
        }
        // Pierces the plane at one point; keep it only if it lands on the
        // ellipse. The normalized radius is off by about distance / b.
        let t = (c - p).dot(&n) / dn;
        let pt = p + d * t;
        let local = pt - c;
        let (u, v) = (local.dot(&x) / a, local.dot(&y) / b);
        if ((u * u + v * v).sqrt() - 1.0).abs() * b <= on_tol {
            return vec![pt];
        }
        return Vec::new();
    }
    let po = p - c;
    let (px, py) = (po.dot(&x) / a, po.dot(&y) / b);
    let (dx, dy) = (d.dot(&x) / a, d.dot(&y) / b);
    solve_quadratic(
        dx * dx + dy * dy,
        2.0 * (px * dx + py * dy),
        px * px + py * py - 1.0,
    )
    .into_iter()
    .map(|t| p + d * t)
    .collect()
}

/// Closed-form intersection of two coplanar circles via the radical line (0, 1,
/// or 2 pts). Concentric circles return none (full coincidence is handled
/// upstream by [`curves_overlap`]).
fn circle_circle_coplanar(c1: &Circle, c2: &Circle, tol: f64) -> Vec<Pnt> {
    let x = GeomVec::from_dir(c1.position().x_direction());
    let y = GeomVec::from_dir(c1.position().y_direction());
    let o = c1.center();
    let (r1, r2) = (c1.radius(), c2.radius());
    let dvec = c2.center() - o;
    let (cx, cy) = (dvec.dot(&x), dvec.dot(&y));
    let d = (cx * cx + cy * cy).sqrt();
    if d < 1e-12 {
        return Vec::new(); // concentric
    }
    // Distance from c1's center to the radical line, along the center direction.
    let aa = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
    let h2 = r1 * r1 - aa * aa;
    if h2 < -tol.max(1e-9) {
        return Vec::new(); // separate or one inside the other
    }
    let h = h2.max(0.0).sqrt();
    let (ux, uy) = (cx / d, cy / d); // unit center direction (in-plane)
    let (perp_x, perp_y) = (-uy, ux);
    let (base_x, base_y) = (aa * ux, aa * uy);
    let mut out: Vec<Pnt> = Vec::new();
    for sign in [1.0, -1.0] {
        let lx = base_x + sign * h * perp_x;
        let ly = base_y + sign * h * perp_y;
        let pt = o + x * lx + y * ly;
        if !out.iter().any(|q: &Pnt| pt.distance(q) < tol.max(1e-9)) {
            out.push(pt);
        }
    }
    out
}

/// Closed-form intersection of two circles in 3D (0, 1, or 2 pts). Coplanar
/// circles use the radical line; circles in distinct planes intersect along
/// those planes' common line, kept where it meets both circles.
fn circle_circle(c1: &Circle, c2: &Circle, tol: f64) -> Vec<Pnt> {
    let on_tol = tol.max(1e-7);
    let n1 = GeomVec::from_dir(c1.axis());
    let n2 = GeomVec::from_dir(c2.axis());
    let cross = n1.cross(&n2);
    if cross.dot(&cross) < 1e-12 {
        // Parallel planes: coincident only if the centers share the plane.
        if (c2.center() - c1.center()).dot(&n1).abs() > on_tol {
            return Vec::new();
        }
        return circle_circle_coplanar(c1, c2, tol);
    }
    // Distinct planes: their common line is where any intersection must lie.
    let dir = match cross.normalized() {
        Some(d) => d,
        None => return Vec::new(),
    };
    let h1 = n1.dot(&(c1.center() - Pnt::origin()));
    let h2 = n2.dot(&(c2.center() - Pnt::origin()));
    let dd = cross.dot(&cross);
    let p0v = (n2.cross(&cross)) * h1 + (cross.cross(&n1)) * h2;
    let p0 = Pnt::origin() + p0v * (1.0 / dd);
    let line = openrcad_geom::Line::from_point_dir(p0, dir);
    let on1 = line_circle(&line, c1, tol);
    let on2 = line_circle(&line, c2, tol);
    let mut out: Vec<Pnt> = Vec::new();
    for a in &on1 {
        if on2.iter().any(|b| a.distance(b) <= on_tol) {
            out.push(*a);
        }
    }
    out
}

/// Closed-form curve-curve intersection for the analytic cases that otherwise
/// drive the generic subdivision solver into its depth cap (lines have infinite
/// bounds, so the subdivision bisects a clamped ±100 domain down to `tol` for
/// every edge pair — the dominant cost of the boolean split passes). Returns
/// `None` when no closed form applies, signalling the caller to fall back.
fn analytic_curve_curve(c1: &GeomCurve, c2: &GeomCurve, tol: f64) -> Option<Vec<Pnt>> {
    match (c1, c2) {
        (GeomCurve::Line(a), GeomCurve::Line(b)) => Some(line_line(a, b, tol)),
        (GeomCurve::Line(l), GeomCurve::Circle(ci))
        | (GeomCurve::Circle(ci), GeomCurve::Line(l)) => Some(line_circle(l, ci, tol)),
        (GeomCurve::Line(l), GeomCurve::Ellipse(e))
        | (GeomCurve::Ellipse(e), GeomCurve::Line(l)) => Some(line_ellipse(l, e, tol)),
        (GeomCurve::Circle(a), GeomCurve::Circle(b)) => Some(circle_circle(a, b, tol)),
        (GeomCurve::BSpline(a), GeomCurve::BSpline(b)) if a.degree() == 1 && b.degree() == 1 => {
            Some(polyline_polyline_intersections(a.poles(), b.poles(), tol))
        }
        (GeomCurve::BSpline(polyline), other) if polyline.degree() == 1 => {
            polyline_analytic_intersections(polyline.poles(), other, tol)
        }
        (other, GeomCurve::BSpline(polyline)) if polyline.degree() == 1 => {
            polyline_analytic_intersections(polyline.poles(), other, tol)
        }
        _ => None,
    }
}

/// Intersect a degree-one B-spline with an analytic boundary curve segment by
/// segment. Surface/surface intersection polylines can contain hundreds of
/// poles; sending each line/circle boundary through the generic recursive
/// solver makes trimming needlessly quadratic in subdivision depth.
fn polyline_analytic_intersections(poles: &[Pnt], other: &GeomCurve, tol: f64) -> Option<Vec<Pnt>> {
    if !matches!(other, GeomCurve::Line(_) | GeomCurve::Circle(_)) {
        return None;
    }

    let mut intersections = Vec::new();
    for segment in poles.windows(2) {
        let [start, end] = segment else {
            continue;
        };
        let delta = *end - *start;
        let Some(direction) = delta.normalized() else {
            continue;
        };
        let segment_line = openrcad_geom::Line::from_point_dir(*start, direction);
        let candidates = match other {
            GeomCurve::Line(line) => line_line(&segment_line, line, tol),
            GeomCurve::Circle(circle) => line_circle(&segment_line, circle, tol),
            _ => unreachable!("analytic boundary curve checked above"),
        };

        let length = delta.magnitude();
        for point in candidates {
            let along = (point - *start).dot(&GeomVec::from_dir(direction));
            if along >= -tol
                && along <= length + tol
                && !intersections
                    .iter()
                    .any(|existing: &Pnt| existing.distance(&point) <= tol)
            {
                intersections.push(point);
            }
        }
    }
    Some(intersections)
}

/// Direct intersection for two degree-one B-splines. This is the common case
/// when a sampled surface/surface intersection meets an already fitted trim
/// boundary. Recursive spline subdivision scales poorly with hundreds of poles,
/// while the curves are exactly polylines and can be tested segment by segment.
fn polyline_polyline_intersections(a: &[Pnt], b: &[Pnt], tol: f64) -> Vec<Pnt> {
    let mut intersections = Vec::new();
    for first in a.windows(2) {
        let [a0, a1] = first else {
            continue;
        };
        let a_delta = *a1 - *a0;
        let Some(a_direction) = a_delta.normalized() else {
            continue;
        };
        let a_length = a_delta.magnitude();
        let a_line = openrcad_geom::Line::from_point_dir(*a0, a_direction);

        for second in b.windows(2) {
            let [b0, b1] = second else {
                continue;
            };
            if !segments_bbox_overlap(a0, a1, b0, b1, tol) {
                continue;
            }
            let b_delta = *b1 - *b0;
            let Some(b_direction) = b_delta.normalized() else {
                continue;
            };
            let b_length = b_delta.magnitude();
            let b_line = openrcad_geom::Line::from_point_dir(*b0, b_direction);

            for point in line_line(&a_line, &b_line, tol) {
                let a_along = (point - *a0).dot(&GeomVec::from_dir(a_direction));
                let b_along = (point - *b0).dot(&GeomVec::from_dir(b_direction));
                if a_along >= -tol
                    && a_along <= a_length + tol
                    && b_along >= -tol
                    && b_along <= b_length + tol
                    && !intersections
                        .iter()
                        .any(|existing: &Pnt| existing.distance(&point) <= tol)
                {
                    intersections.push(point);
                }
            }
        }
    }
    intersections
}

fn segments_bbox_overlap(a0: &Pnt, a1: &Pnt, b0: &Pnt, b1: &Pnt, tol: f64) -> bool {
    let overlaps = |x0: f64, x1: f64, y0: f64, y1: f64| {
        x0.min(x1) <= y0.max(y1) + tol && y0.min(y1) <= x0.max(x1) + tol
    };
    overlaps(a0.x(), a1.x(), b0.x(), b1.x())
        && overlaps(a0.y(), a1.y(), b0.y(), b1.y())
        && overlaps(a0.z(), a1.z(), b0.z(), b1.z())
}

/// Find intersection points between two 3D curves.
pub fn curve_curve(c1: &GeomCurve, c2: &GeomCurve, tol: f64) -> Vec<Pnt> {
    // Coincident/overlapping curves share their supporting geometry — there is no
    // discrete crossing to find, and the subdivision search would never prune.
    // Detect it up front in both directions (handles full coincidence and the
    // case where one curve is a sub-arc of the other).
    if curves_overlap(c1, c2, tol) || curves_overlap(c2, c1, tol) {
        return Vec::new();
    }

    // Closed-form fast path for analytic curve pairs (lines/circles): exact and
    // O(1), bypassing the subdivision search entirely.
    if let Some(pts) = analytic_curve_curve(c1, c2, tol) {
        let mut results: Vec<Pnt> = Vec::new();
        for p in pts {
            if !results.iter().any(|r: &Pnt| r.distance(&p) < tol) {
                results.push(p);
            }
        }
        return results;
    }
    let (t1_min, t1_max) = c1.bounds();
    let (t2_min, t2_max) = c2.bounds();
    let t1_min = clamp_bound(t1_min, -100.0);
    let t1_max = clamp_bound(t1_max, 100.0);
    let t2_min = clamp_bound(t2_min, -100.0);
    let t2_max = clamp_bound(t2_max, 100.0);

    let mut candidates = Vec::new();
    curve_curve_subdivide(
        c1,
        t1_min,
        t1_max,
        c2,
        t2_min,
        t2_max,
        tol,
        0,
        &mut candidates,
    );

    let mut results = Vec::new();
    for (t, s) in candidates {
        if let Some(p) = curve_curve_refine(c1, t, c2, s, tol) {
            // Keep unique points
            if !results.iter().any(|r: &Pnt| r.distance(&p) < tol) {
                results.push(p);
            }
        }
    }
    results
}

#[allow(clippy::too_many_arguments)] // recursive subdivision threads its bracket state
fn curve_curve_subdivide(
    c1: &GeomCurve,
    t1_min: f64,
    t1_max: f64,
    c2: &GeomCurve,
    t2_min: f64,
    t2_max: f64,
    tol: f64,
    depth: usize,
    candidates: &mut Vec<(f64, f64)>,
) {
    if depth > 20 {
        candidates.push((0.5 * (t1_min + t1_max), 0.5 * (t2_min + t2_max)));
        return;
    }

    // Guaranteed-enclosing bounds over each parameter interval (rigorous
    // pruning — a real crossing can never be dropped, unlike the old 5-sample
    // box which could miss a grazing contact between samples).
    let mut b1 = c1.interval_point(t1_min, t1_max).to_bndbox();
    b1.enlarge(tol);
    let mut b2 = c2.interval_point(t2_min, t2_max).to_bndbox();
    b2.enlarge(tol);

    if b1.is_out_box(&b2) {
        return;
    }

    let w1 = t1_max - t1_min;
    let w2 = t2_max - t2_min;

    if w1 < tol && w2 < tol {
        candidates.push((0.5 * (t1_min + t1_max), 0.5 * (t2_min + t2_max)));
    } else {
        if w1 >= w2 {
            let mid = 0.5 * (t1_min + t1_max);
            curve_curve_subdivide(
                c1,
                t1_min,
                mid,
                c2,
                t2_min,
                t2_max,
                tol,
                depth + 1,
                candidates,
            );
            curve_curve_subdivide(
                c1,
                mid,
                t1_max,
                c2,
                t2_min,
                t2_max,
                tol,
                depth + 1,
                candidates,
            );
        } else {
            let mid = 0.5 * (t2_min + t2_max);
            curve_curve_subdivide(
                c1,
                t1_min,
                t1_max,
                c2,
                t2_min,
                mid,
                tol,
                depth + 1,
                candidates,
            );
            curve_curve_subdivide(
                c1,
                t1_min,
                t1_max,
                c2,
                mid,
                t2_max,
                tol,
                depth + 1,
                candidates,
            );
        }
    }
}

fn curve_curve_refine(
    c1: &GeomCurve,
    t_start: f64,
    c2: &GeomCurve,
    s_start: f64,
    tol: f64,
) -> Option<Pnt> {
    let (t1_min, t1_max) = c1.bounds();
    let (t2_min, t2_max) = c2.bounds();
    let t1_min = clamp_bound(t1_min, -100.0);
    let t1_max = clamp_bound(t1_max, 100.0);
    let t2_min = clamp_bound(t2_min, -100.0);
    let t2_max = clamp_bound(t2_max, 100.0);

    let mut t = t_start;
    let mut s = s_start;

    // Near-tangent crossings (a section curve meeting a blend's tangent
    // edge) converge slowly; the vertex must still land within the kernel's
    // vertex tolerance of both curves.
    for _ in 0..60 {
        let (p1, v1) = c1.d1(t);
        let (p2, v2) = c2.d1(s);

        let b = p2 - p1;
        let a = v1.dot(&v1);
        let b_coeff = -v1.dot(&v2);
        let c = v2.dot(&v2);

        let det = a * c - b_coeff * b_coeff;
        // Relative to the tangent lengths, so a curve parameterized by arc
        // length and one by angle stop at the same near-tangency.
        if det.abs() <= 1e-14 * a * c {
            // Tangent contact
            break;
        }

        let dt = (c * v1.dot(&b) - b_coeff * (-v2.dot(&b))) / det;
        let ds = (a * (-v2.dot(&b)) - b_coeff * v1.dot(&b)) / det;

        t = clamp_ordered(t + dt, t1_min, t1_max);
        s = clamp_ordered(s + ds, t2_min, t2_max);

        if dt.abs() < 1e-9 && ds.abs() < 1e-9 {
            break;
        }
    }

    let p1 = c1.point(t);
    let p2 = c2.point(s);
    if p1.distance(&p2) <= tol {
        Some(p1.midpoint(&p2))
    } else {
        None
    }
}

/// Find intersection points between a 3D curve and a 3D surface.
pub fn curve_surface(c: &GeomCurve, s: &GeomSurface, tol: f64) -> Vec<Pnt> {
    let (t_min, t_max) = c.bounds();
    let (u_min, u_max, v_min, v_max) = s.bounds();
    let t_min = clamp_bound(t_min, -100.0);
    let t_max = clamp_bound(t_max, 100.0);
    let u_min = clamp_bound(u_min, -100.0);
    let u_max = clamp_bound(u_max, 100.0);
    let v_min = clamp_bound(v_min, -100.0);
    let v_max = clamp_bound(v_max, 100.0);

    let mut candidates = Vec::new();
    curve_surface_subdivide(
        c,
        t_min,
        t_max,
        s,
        u_min,
        u_max,
        v_min,
        v_max,
        tol,
        0,
        &mut candidates,
    );

    let mut results = Vec::new();
    for (t, u, v) in candidates {
        if let Some(p) = curve_surface_refine(c, t, s, u, v, tol) {
            if !results.iter().any(|r: &Pnt| r.distance(&p) < tol) {
                results.push(p);
            }
        }
    }
    results
}

#[allow(clippy::too_many_arguments)] // recursive subdivision threads its bracket state
fn curve_surface_subdivide(
    c: &GeomCurve,
    t_min: f64,
    t_max: f64,
    s: &GeomSurface,
    u_min: f64,
    u_max: f64,
    v_min: f64,
    v_max: f64,
    tol: f64,
    depth: usize,
    candidates: &mut Vec<(f64, f64, f64)>,
) {
    if depth > 20 {
        candidates.push((
            0.5 * (t_min + t_max),
            0.5 * (u_min + u_max),
            0.5 * (v_min + v_max),
        ));
        return;
    }

    // Guaranteed-enclosing bounds over the curve segment and the surface patch.
    let mut b_c = c.interval_point(t_min, t_max).to_bndbox();
    b_c.enlarge(tol);
    let mut b_s = s.interval_point(u_min, u_max, v_min, v_max).to_bndbox();
    b_s.enlarge(tol);

    if b_c.is_out_box(&b_s) {
        return;
    }

    let wt = t_max - t_min;
    let wu = u_max - u_min;
    let wv = v_max - v_min;

    if wt < tol && wu < tol && wv < tol {
        candidates.push((
            0.5 * (t_min + t_max),
            0.5 * (u_min + u_max),
            0.5 * (v_min + v_max),
        ));
    } else {
        if wt >= wu && wt >= wv {
            let mid = 0.5 * (t_min + t_max);
            curve_surface_subdivide(
                c,
                t_min,
                mid,
                s,
                u_min,
                u_max,
                v_min,
                v_max,
                tol,
                depth + 1,
                candidates,
            );
            curve_surface_subdivide(
                c,
                mid,
                t_max,
                s,
                u_min,
                u_max,
                v_min,
                v_max,
                tol,
                depth + 1,
                candidates,
            );
        } else if wu >= wv {
            let mid = 0.5 * (u_min + u_max);
            curve_surface_subdivide(
                c,
                t_min,
                t_max,
                s,
                u_min,
                mid,
                v_min,
                v_max,
                tol,
                depth + 1,
                candidates,
            );
            curve_surface_subdivide(
                c,
                t_min,
                t_max,
                s,
                mid,
                u_max,
                v_min,
                v_max,
                tol,
                depth + 1,
                candidates,
            );
        } else {
            let mid = 0.5 * (v_min + v_max);
            curve_surface_subdivide(
                c,
                t_min,
                t_max,
                s,
                u_min,
                u_max,
                v_min,
                mid,
                tol,
                depth + 1,
                candidates,
            );
            curve_surface_subdivide(
                c,
                t_min,
                t_max,
                s,
                u_min,
                u_max,
                mid,
                v_max,
                tol,
                depth + 1,
                candidates,
            );
        }
    }
}

fn solve_3x3(col0: GeomVec, col1: GeomVec, col2: GeomVec, b: GeomVec) -> Option<(f64, f64, f64)> {
    let det = col0.dot(&col1.cross(&col2));
    if det.abs() < 1e-12 {
        return None;
    }
    let det0 = b.dot(&col1.cross(&col2));
    let det1 = col0.dot(&b.cross(&col2));
    let det2 = col0.dot(&col1.cross(&b));
    Some((det0 / det, det1 / det, det2 / det))
}

fn curve_surface_refine(
    c: &GeomCurve,
    t_start: f64,
    s: &GeomSurface,
    u_start: f64,
    v_start: f64,
    tol: f64,
) -> Option<Pnt> {
    let (t_min, t_max) = c.bounds();
    let (u_min, u_max, v_min, v_max) = s.bounds();
    let t_min = clamp_bound(t_min, -100.0);
    let t_max = clamp_bound(t_max, 100.0);
    let u_min = clamp_bound(u_min, -100.0);
    let u_max = clamp_bound(u_max, 100.0);
    let v_min = clamp_bound(v_min, -100.0);
    let v_max = clamp_bound(v_max, 100.0);

    let mut t = t_start;
    let mut u = u_start;
    let mut v = v_start;

    for _ in 0..10 {
        let (pc, vc) = c.d1(t);
        let (ps, su, sv) = eval_d1(s, u, v);

        let rhs = ps - pc;
        if let Some((dt, du, dv)) = solve_3x3(vc, -su, -sv, rhs) {
            t = clamp_ordered(t + dt, t_min, t_max);
            u = clamp_ordered(u + du, u_min, u_max);
            v = clamp_ordered(v + dv, v_min, v_max);

            if dt.abs() < 1e-9 && du.abs() < 1e-9 && dv.abs() < 1e-9 {
                break;
            }
        } else {
            break;
        }
    }

    let pc = c.point(t);
    let ps = s.point(u, v);
    if pc.distance(&ps) <= tol {
        Some(pc.midpoint(&ps))
    } else {
        None
    }
}

/// Exact intersection curves for analytically tractable surface pairs. Returns
/// `None` when no closed-form case applies (caller falls back to subdivision).
pub(crate) fn analytic_surface_surface(
    s1: &GeomSurface,
    s2: &GeomSurface,
    tol: f64,
) -> Option<Vec<GeomCurve>> {
    // Plane ∩ plane is an infinite line (empty when parallel — the coplanar case
    // is handled separately by the boolean engine).
    if let (GeomSurface::Plane(p1), GeomSurface::Plane(p2)) = (s1, s2) {
        let n1 = GeomVec::from_dir(p1.normal());
        let n2 = GeomVec::from_dir(p2.normal());
        let d = n1.cross(&n2);
        let dd = d.dot(&d);
        if dd < 1e-12 {
            return Some(Vec::new()); // parallel planes
        }
        let h1 = n1.dot(&(p1.location() - Pnt::origin()));
        let h2 = n2.dot(&(p2.location() - Pnt::origin()));
        // A point on both planes: p0 = (h1 (n2×d) + h2 (d×n1)) / |d|².
        let p0v = (n2.cross(&d)) * h1 + (d.cross(&n1)) * h2;
        let p0 = Pnt::origin() + p0v * (1.0 / dd);
        let dir = match d.normalized() {
            Some(dir) => dir,
            None => return Some(Vec::new()),
        };
        return Some(vec![GeomCurve::Line(openrcad_geom::Line::from_point_dir(
            p0, dir,
        ))]);
    }

    // Plane ∩ cylinder, with the plane perpendicular to the cylinder axis, is a
    // circle of the cylinder's radius. (Other orientations give ellipses/lines,
    // handled by the generic solver.)
    if let (GeomSurface::Cylinder(c1), GeomSurface::Cylinder(c2)) = (s1, s2) {
        return Some(cylinder_cylinder_curves(c1, c2));
    }

    if let (GeomSurface::Cone(cone), GeomSurface::Cylinder(cylinder))
    | (GeomSurface::Cylinder(cylinder), GeomSurface::Cone(cone)) = (s1, s2)
    {
        if let Some(curves) = coaxial_cone_cylinder_curves(cone, cylinder) {
            return Some(curves);
        }
    }

    match (s1, s2) {
        (GeomSurface::Plane(plane), GeomSurface::Cylinder(cylinder))
        | (GeomSurface::Cylinder(cylinder), GeomSurface::Plane(plane)) => {
            Some(plane_cylinder_curves(plane, cylinder))
        }
        (GeomSurface::Plane(plane), GeomSurface::Cone(cone))
        | (GeomSurface::Cone(cone), GeomSurface::Plane(plane)) => plane_cone_curves(plane, cone),
        (GeomSurface::Plane(plane), GeomSurface::Sphere(sphere))
        | (GeomSurface::Sphere(sphere), GeomSurface::Plane(plane)) => {
            Some(plane_sphere_curves(plane, sphere))
        }
        (GeomSurface::Plane(plane), GeomSurface::Torus(torus))
        | (GeomSurface::Torus(torus), GeomSurface::Plane(plane)) => {
            plane_torus_curves(plane, torus, tol)
        }
        (GeomSurface::Cylinder(cylinder), GeomSurface::Torus(torus))
        | (GeomSurface::Torus(torus), GeomSurface::Cylinder(cylinder)) => {
            coaxial_cylinder_torus_curves(cylinder, torus, tol)
        }
        (GeomSurface::Cone(cone), GeomSurface::Torus(torus))
        | (GeomSurface::Torus(torus), GeomSurface::Cone(cone)) => {
            coaxial_cone_torus_curves(cone, torus, tol)
        }
        (GeomSurface::Torus(first), GeomSurface::Torus(second)) => {
            coaxial_torus_torus_curves(first, second, tol)
        }
        _ => None,
    }
}

fn characteristic_length(values: impl IntoIterator<Item = f64>, tol: f64) -> f64 {
    values
        .into_iter()
        .map(f64::abs)
        .chain(core::iter::once(tol.abs()))
        .fold(f64::MIN_POSITIVE, f64::max)
}

fn axes_are_coaxial(first: &Ax3, second: &Ax3, scale: f64, tol: f64) -> Option<f64> {
    let first_direction = GeomVec::from_dir(first.direction());
    let second_direction = GeomVec::from_dir(second.direction());
    let alignment = first_direction.dot(&second_direction);
    let angular_tolerance = tol.abs() / scale + f64::EPSILON * 64.0;
    if first_direction.cross(&second_direction).magnitude() > angular_tolerance {
        return None;
    }
    let offset = second.location() - first.location();
    let radial = offset - first_direction * offset.dot(&first_direction);
    let linear_tolerance = tol.abs() + scale * f64::EPSILON * 64.0;
    (radial.magnitude() <= linear_tolerance).then_some(alignment.signum())
}

fn circle_on_axis(frame: &Ax3, axial: f64, radius: f64) -> GeomCurve {
    let direction = GeomVec::from_dir(frame.direction());
    let center = frame.location() + direction * axial;
    GeomCurve::Circle(Circle::new(
        Ax3::new_axes(center, frame.direction(), frame.x_direction()),
        radius,
    ))
}

/// Exact perpendicular plane/regular-torus sections.  The two possible rings
/// are kept as independent periodic branches rather than sampled splines.
fn plane_torus_curves(plane: &Plane, torus: &ToroidalSurface, tol: f64) -> Option<Vec<GeomCurve>> {
    let frame = torus.position();
    let scale = characteristic_length([torus.major_radius(), torus.minor_radius()], tol);
    let axis = GeomVec::from_dir(frame.direction());
    let normal = GeomVec::from_dir(plane.normal());
    let angular_tolerance = tol.abs() / scale + f64::EPSILON * 64.0;
    let alignment = axis.dot(&normal).abs();
    if alignment <= angular_tolerance {
        let signed_offset = (plane.location() - frame.location()).dot(&normal);
        let linear_tolerance = tol.abs() + scale * f64::EPSILON * 64.0;
        if signed_offset.abs() > linear_tolerance {
            if signed_offset.abs() >= torus.major_radius() - torus.minor_radius() - linear_tolerance
            {
                // Beyond the inner radius, branches split or vanish. The
                // bounded 4C curve type intentionally covers only two regular
                // closed branches and rejects this ambiguous regime.
                return None;
            }
            return Some(
                [true, false]
                    .into_iter()
                    .map(|positive_branch| {
                        GeomCurve::torus_plane_section(TorusPlaneSection::new(
                            frame,
                            torus.major_radius(),
                            torus.minor_radius(),
                            plane.normal(),
                            signed_offset,
                            positive_branch,
                        ))
                    })
                    .collect(),
            );
        }
        let radial = axis.cross(&normal).normalized()?;
        let radial = GeomVec::from_dir(radial);
        return Some(
            [1.0, -1.0]
                .into_iter()
                .map(|side| {
                    let x_direction = (radial * side).normalized().expect("unit radial");
                    let circle_center = frame.location() + radial * (side * torus.major_radius());
                    GeomCurve::Circle(Circle::new(
                        Ax3::new_axes(circle_center, plane.normal(), x_direction),
                        torus.minor_radius(),
                    ))
                })
                .collect(),
        );
    }
    if (alignment - 1.0).abs() > angular_tolerance {
        return None;
    }
    let denominator = axis.dot(&normal);
    let axial = (plane.location() - frame.location()).dot(&normal) / denominator;
    let squared = torus.minor_radius().powi(2) - axial.powi(2);
    let squared_tolerance = tol.abs() * (torus.minor_radius().abs() + axial.abs() + tol.abs())
        + scale.powi(2) * f64::EPSILON * 64.0;
    if squared < -squared_tolerance {
        return Some(Vec::new());
    }
    let meridian = squared.max(0.0).sqrt();
    let mut radii = vec![torus.major_radius() + meridian];
    let inner = torus.major_radius() - meridian;
    if inner > tol.abs() && (inner - radii[0]).abs() > tol.abs() {
        radii.push(inner);
    }
    Some(
        radii
            .into_iter()
            .map(|radius| circle_on_axis(&frame, axial, radius))
            .collect(),
    )
}

fn coaxial_cylinder_torus_curves(
    cylinder: &CylindricalSurface,
    torus: &ToroidalSurface,
    tol: f64,
) -> Option<Vec<GeomCurve>> {
    let scale = characteristic_length(
        [
            cylinder.radius(),
            torus.major_radius(),
            torus.minor_radius(),
        ],
        tol,
    );
    axes_are_coaxial(&torus.position(), &cylinder.position(), scale, tol)?;
    let radial_delta = cylinder.radius() - torus.major_radius();
    let squared = torus.minor_radius().powi(2) - radial_delta.powi(2);
    let squared_tolerance = tol.abs()
        * (torus.minor_radius().abs() + radial_delta.abs() + tol.abs())
        + scale.powi(2) * f64::EPSILON * 64.0;
    if squared < -squared_tolerance {
        return Some(Vec::new());
    }
    let axial = squared.max(0.0).sqrt();
    let mut curves = vec![circle_on_axis(&torus.position(), axial, cylinder.radius())];
    if axial > tol.abs() {
        curves.push(circle_on_axis(&torus.position(), -axial, cylinder.radius()));
    }
    Some(curves)
}

fn normalized_quadratic_roots(a: f64, b: f64, c: f64) -> Vec<f64> {
    let coefficient_scale = a.abs().max(b.abs()).max(c.abs());
    if coefficient_scale == 0.0 {
        return Vec::new();
    }
    let (a, b, c) = (
        a / coefficient_scale,
        b / coefficient_scale,
        c / coefficient_scale,
    );
    let numeric = f64::EPSILON * 128.0;
    if a.abs() <= numeric {
        return (b.abs() > numeric).then(|| -c / b).into_iter().collect();
    }
    let discriminant = b * b - 4.0 * a * c;
    let discriminant_tolerance = numeric * (b * b + (4.0 * a * c).abs());
    if discriminant < -discriminant_tolerance {
        return Vec::new();
    }
    let root = discriminant.max(0.0).sqrt();
    let q = -0.5 * (b + root.copysign(b));
    if q.abs() <= numeric {
        return vec![-b / (2.0 * a)];
    }
    let first = q / a;
    let second = c / q;
    if (first - second).abs() <= numeric * first.abs().max(second.abs()).max(1.0) {
        vec![first]
    } else {
        vec![first, second]
    }
}

fn coaxial_cone_torus_curves(
    cone: &ConicalSurface,
    torus: &ToroidalSurface,
    tol: f64,
) -> Option<Vec<GeomCurve>> {
    let frame = torus.position();
    let scale = characteristic_length(
        [
            cone.ref_radius(),
            torus.major_radius(),
            torus.minor_radius(),
            (frame.location() - cone.position().location()).magnitude(),
        ],
        tol,
    );
    let direction_sign = axes_are_coaxial(&frame, &cone.position(), scale, tol)?;
    let cone_height_at_torus_origin = (frame.location() - cone.position().location())
        .dot(&GeomVec::from_dir(cone.position().direction()));
    let slope = cone.semi_angle().tan();
    let radial_at_origin = cone.ref_radius() + slope * cone_height_at_torus_origin;
    let radial_slope = slope * direction_sign;

    // Solve in normalized meridian coordinates z'=z/scale, keeping the
    // coefficient comparison dimensionless across the mandatory scale sweep.
    let offset = (radial_at_origin - torus.major_radius()) / scale;
    let minor = torus.minor_radius() / scale;
    let roots = normalized_quadratic_roots(
        radial_slope * radial_slope + 1.0,
        2.0 * offset * radial_slope,
        offset * offset - minor * minor,
    );
    let mut curves = Vec::new();
    for normalized_axial in roots {
        let axial = normalized_axial * scale;
        let radius = radial_at_origin + radial_slope * axial;
        if radius > tol.abs() {
            curves.push(circle_on_axis(&frame, axial, radius));
        }
    }
    Some(curves)
}

fn coaxial_torus_torus_curves(
    first: &ToroidalSurface,
    second: &ToroidalSurface,
    tol: f64,
) -> Option<Vec<GeomCurve>> {
    let first_frame = first.position();
    let scale = characteristic_length(
        [
            first.major_radius(),
            first.minor_radius(),
            second.major_radius(),
            second.minor_radius(),
            (second.position().location() - first_frame.location()).magnitude(),
        ],
        tol,
    );
    axes_are_coaxial(&first_frame, &second.position(), scale, tol)?;
    let axis = GeomVec::from_dir(first_frame.direction());
    let second_axial = (second.position().location() - first_frame.location()).dot(&axis);
    let dx = second.major_radius() - first.major_radius();
    let dz = second_axial;
    let center_distance = dx.hypot(dz);
    let linear_tolerance = tol.abs() + scale * f64::EPSILON * 64.0;
    if center_distance <= linear_tolerance {
        // Coincident meridian circles are the same-domain case; unequal radii
        // do not meet.  Neither case owns a unique transition boundary.
        return Some(Vec::new());
    }
    let r1 = first.minor_radius();
    let r2 = second.minor_radius();
    if center_distance > r1 + r2 + linear_tolerance
        || center_distance < (r1 - r2).abs() - linear_tolerance
    {
        return Some(Vec::new());
    }
    let along = (r1 * r1 - r2 * r2 + center_distance * center_distance) / (2.0 * center_distance);
    let height_squared = r1 * r1 - along * along;
    let squared_tolerance = linear_tolerance * (r1.abs() + along.abs() + linear_tolerance);
    if height_squared < -squared_tolerance {
        return Some(Vec::new());
    }
    let height = height_squared.max(0.0).sqrt();
    let radial_unit = dx / center_distance;
    let axial_unit = dz / center_distance;
    let base_radius = first.major_radius() + along * radial_unit;
    let base_axial = along * axial_unit;
    let perpendicular = [(-axial_unit, radial_unit), (axial_unit, -radial_unit)];
    let mut curves = Vec::new();
    for (index, (radial_direction, axial_direction)) in perpendicular.into_iter().enumerate() {
        if index == 1 && height <= linear_tolerance {
            break;
        }
        let radius = base_radius + height * radial_direction;
        let axial = base_axial + height * axial_direction;
        if radius > linear_tolerance {
            curves.push(circle_on_axis(&first_frame, axial, radius));
        }
    }
    Some(curves)
}

fn plane_sphere_curves(plane: &Plane, sphere: &openrcad_geom::SphericalSurface) -> Vec<GeomCurve> {
    let normal = GeomVec::from_dir(plane.normal());
    let signed_distance = (sphere.center() - plane.location()).dot(&normal);
    if signed_distance.abs() > sphere.radius() {
        return Vec::new();
    }
    let radius_squared =
        (sphere.radius() * sphere.radius() - signed_distance * signed_distance).max(0.0);
    if radius_squared <= f64::EPSILON * sphere.radius().powi(2) {
        return Vec::new();
    }
    let center = sphere.center() - normal * signed_distance;
    let frame = Ax3::new_axes(center, plane.normal(), plane.position().x_direction());
    vec![GeomCurve::Circle(Circle::new(frame, radius_squared.sqrt()))]
}

/// Exact circles where a cone meets a coaxial cylinder. Countersinks rely on
/// this at the bore transition; the generic surface solver otherwise emits
/// fragmented spans around the periodic seam and cannot close the cut wall.
fn coaxial_cone_cylinder_curves(
    cone: &ConicalSurface,
    cylinder: &CylindricalSurface,
) -> Option<Vec<GeomCurve>> {
    let cone_axis = cone.position();
    let cylinder_axis = cylinder.position();
    let cone_direction = GeomVec::from_dir(cone_axis.direction());
    let cylinder_direction = GeomVec::from_dir(cylinder_axis.direction());
    if cone_direction.cross(&cylinder_direction).magnitude() > 1.0e-8 {
        return None;
    }
    let offset = cylinder_axis.location() - cone_axis.location();
    if (offset - cone_direction * offset.dot(&cone_direction)).magnitude() > 1.0e-8 {
        return None;
    }

    let slope = cone.semi_angle().tan();
    if slope.abs() <= 1.0e-12 {
        return Some(Vec::new());
    }
    let heights = [
        (cylinder.radius() - cone.ref_radius()) / slope,
        (-cylinder.radius() - cone.ref_radius()) / slope,
    ];
    let mut curves = Vec::new();
    for height in heights {
        let center = cone_axis.location() + cone_direction * height;
        let frame = Ax3::new_axes(center, cone_axis.direction(), cone_axis.x_direction());
        curves.push(GeomCurve::Circle(Circle::new(frame, cylinder.radius())));
    }
    Some(curves)
}

/// Exact perpendicular plane/cone section. This is the dominant countersink
/// case; returning one analytic circle avoids the fragmented sampled B-spline
/// spans that cannot form a connected partition across the cone wall.
fn plane_cone_curves(plane: &Plane, cone: &ConicalSurface) -> Option<Vec<GeomCurve>> {
    let axis = cone.position();
    let w = GeomVec::from_dir(axis.direction());
    let n = GeomVec::from_dir(plane.normal());
    let denom = w.dot(&n);
    if (denom.abs() - 1.0).abs() > 1.0e-7 {
        return None;
    }

    let height = (plane.location() - axis.location()).dot(&n) / denom;
    let radius = cone.ref_radius() + height * cone.semi_angle().tan();
    if radius.abs() <= 1.0e-12 {
        return Some(Vec::new());
    }
    let center = axis.location() + w * height;
    let frame = Ax3::new_axes(center, axis.direction(), axis.x_direction());
    Some(vec![GeomCurve::Circle(Circle::new(frame, radius.abs()))])
}

fn plane_cylinder_curves(plane: &Plane, cyl: &CylindricalSurface) -> Vec<GeomCurve> {
    let axis = cyl.position();
    let w = GeomVec::from_dir(axis.direction());
    let n = GeomVec::from_dir(plane.normal());
    let align = w.dot(&n).abs();

    if (align - 1.0).abs() <= 1e-7 {
        let denom = w.dot(&n);
        if denom.abs() < 1e-12 {
            return Vec::new();
        }
        let t = (plane.location() - axis.location()).dot(&n) / denom;
        let center = axis.location() + w * t;
        let frame = Ax3::new_axes(center, axis.direction(), axis.x_direction());
        return vec![GeomCurve::Circle(Circle::new(frame, cyl.radius()))];
    }

    if align <= 1e-7 {
        let signed = (axis.location() - plane.location()).dot(&n);
        let r = cyl.radius();
        if signed.abs() > r + 1e-9 {
            return Vec::new();
        }
        let side = match w.cross(&n).normalized() {
            Some(d) => GeomVec::from_dir(d),
            None => return Vec::new(),
        };
        let base = axis.location() + n * (-signed);
        let h2 = (r * r - signed * signed).max(0.0);
        if h2 <= 1e-18 {
            return vec![GeomCurve::Line(openrcad_geom::Line::from_point_dir(
                base,
                axis.direction(),
            ))];
        }
        let h = h2.sqrt();
        return vec![
            GeomCurve::Line(openrcad_geom::Line::from_point_dir(
                base + side * h,
                axis.direction(),
            )),
            GeomCurve::Line(openrcad_geom::Line::from_point_dir(
                base - side * h,
                axis.direction(),
            )),
        ];
    }

    if let Some(lines) = near_parallel_plane_cylinder_lines(plane, cyl) {
        return lines;
    }

    // Oblique plane ∩ cylinder = an exact ellipse. Emitting the analytic curve
    // (instead of a 160-point sampled B-spline) keeps the imprinted seam exactly
    // on both surfaces, so the cut trims watertight instead of leaving the tiny
    // sampling error that broke the result topology (`InvalidEulerCharacteristic`).
    //
    //   center      = where the axis pierces the plane
    //   major dir   = the axis projected into the plane (steepest tilt), |a| = r/|cosθ|
    //   minor dir   = ⊥ to both axis and major (in-plane), |b| = r
    // where cosθ = |w·n| is the tilt of the axis to the plane normal.
    let denom = w.dot(&n);
    if denom.abs() < 1e-12 {
        return Vec::new();
    }
    let center = axis.location() + w * ((plane.location() - axis.location()).dot(&n) / denom);
    let Some(major_dir) = (w - n * denom).normalized() else {
        return Vec::new();
    };
    let major = cyl.radius() / denom.abs();
    let minor = cyl.radius();
    let frame = Ax3::new_axes(center, plane.normal(), major_dir);
    vec![GeomCurve::Ellipse(Ellipse::new(frame, major, minor))]
}

/// A plane within a rounding error of parallel to a cylinder's axis (a wall
/// built from single-precision sketch points) cuts it in an ellipse whose major
/// axis is millions of radii long. Over any real face that ellipse is straight
/// to far below tolerance, but its parameter span there is below the trimming
/// resolution, so the intersection vanishes. Emit instead the ellipse's tangent
/// lines where it crosses the plane's own axial station: they lie exactly in
/// the plane, and leave the cylinder only at second order (curvature
/// `eps^2 / (r sin^3 t)`), which is bounded here. `None` when the plane is
/// not nearly parallel or the lines would stray.
fn near_parallel_plane_cylinder_lines(
    plane: &Plane,
    cyl: &CylindricalSurface,
) -> Option<Vec<GeomCurve>> {
    /// Largest axis tilt (radians) handled as nearly parallel.
    const MAX_TILT: f64 = 1.0e-6;
    /// Half-length over which the lines must hug the cylinder, and how close.
    const REACH: f64 = 100.0;
    const STRAY: f64 = 1.0e-8;
    let axis = cyl.position();
    let w = GeomVec::from_dir(axis.direction());
    let n = GeomVec::from_dir(plane.normal());
    let tilt = w.dot(&n);
    if tilt.abs() > MAX_TILT {
        return None;
    }
    let r = cyl.radius();
    // The cross-section through the plane's location.
    let station = (plane.location() - axis.location()).dot(&w);
    let center = axis.location() + w * station;
    let in_section = n - w * tilt;
    let scale = in_section.magnitude();
    let toward = in_section * (1.0 / scale);
    let side = w.cross(&toward);
    // Points c + r u on the section circle with n.(c + r u - p) = 0.
    let along = (plane.location() - center).dot(&n) / (r * scale);
    if along.abs() > 1.0 + 1.0e-9 {
        return Some(Vec::new());
    }
    let across = (1.0 - along * along).max(0.0).sqrt();
    let curvature = tilt * tilt / (r * across.max(1.0e-3).powi(3));
    if 0.5 * curvature * REACH * REACH > STRAY {
        return None;
    }
    let line = |u: GeomVec| {
        let point = center + u * r;
        // The section curve's tangent: perpendicular to both surface normals.
        let direction = n.cross(&u).normalized()?;
        Some(GeomCurve::Line(openrcad_geom::Line::from_point_dir(
            point, direction,
        )))
    };
    if across <= 1.0e-9 {
        return Some(line(toward * along.signum()).into_iter().collect());
    }
    Some(
        [
            line(toward * along + side * across),
            line(toward * along - side * across),
        ]
        .into_iter()
        .flatten()
        .collect(),
    )
}

/// Parallel cylinders meet along rulings: where their cross-section circles
/// cross in a plane perpendicular to the shared axis direction. Two lines when
/// the circles cross, one where they touch, none when coaxial (coincident or
/// nested walls are not a crossing) or apart.
fn parallel_cylinder_lines(c1: &CylindricalSurface, c2: &CylindricalSurface) -> Vec<GeomCurve> {
    let axis = GeomVec::from_dir(c1.position().direction());
    let o1 = c1.position().location();
    let offset = c2.position().location() - o1;
    let across = offset - axis * offset.dot(&axis);
    let distance = across.magnitude();
    let (r1, r2) = (c1.radius(), c2.radius());
    let scale = r1.max(r2).max(distance);
    let touch = 1.0e-9 * scale;
    if distance <= touch || distance > r1 + r2 + touch || distance < (r1 - r2).abs() - touch {
        return Vec::new();
    }
    let toward = across * (1.0 / distance);
    // Distance from c1's axis, along `toward`, to the chord through both
    // crossings, and the chord's half length.
    let along = (distance * distance + r1 * r1 - r2 * r2) / (2.0 * distance);
    let half = (r1 * r1 - along * along).max(0.0).sqrt();
    let foot = o1 + toward * along;
    let Some(direction) = axis.normalized() else {
        return Vec::new();
    };
    let line = |point: Pnt| GeomCurve::Line(openrcad_geom::Line::from_point_dir(point, direction));
    if half <= touch {
        return vec![line(foot)];
    }
    let side = axis.cross(&toward) * half;
    vec![line(foot + side), line(foot - side)]
}

fn cylinder_cylinder_curves(c1: &CylindricalSurface, c2: &CylindricalSurface) -> Vec<GeomCurve> {
    let w1 = GeomVec::from_dir(c1.position().direction());
    let w2 = GeomVec::from_dir(c2.position().direction());
    if w1.cross(&w2).magnitude() < 1e-9 {
        return parallel_cylinder_lines(c1, c2);
    }

    // The point c1(u, v) lies on c2 when its distance from c2's axis is r2:
    // `qa v^2 + qb(u) v + qc(u) = 0`. `qa = |w1 - (w1.w2) w2|^2` is the squared
    // sine of the axis angle, so it is positive for non-parallel axes.
    let m = w1 - w2 * w1.dot(&w2);
    let qa = m.dot(&m);
    let coefficients = |u: f64| {
        let d = c1.point(u, 0.0) - c2.position().location();
        let n = d - w2 * d.dot(&w2);
        (2.0 * n.dot(&m), n.dot(&n) - c2.radius() * c2.radius())
    };
    let discriminant = |u: f64| {
        let (qb, qc) = coefficients(u);
        qb * qb - 4.0 * qa * qc
    };
    // Both roots, lower first. At a turning point the discriminant is zero and
    // the two coincide, which is what closes a loop exactly.
    let roots = |u: f64| {
        let (qb, qc) = coefficients(u);
        let s = (qb * qb - 4.0 * qa * qc).max(0.0).sqrt();
        ((-qb - s) / (2.0 * qa), (-qb + s) / (2.0 * qa))
    };

    // Where the discriminant changes sign the curve turns back in u. Sampling
    // alone never lands on that point, so a loop traced from samples stops
    // short of it on both branches and leaves a gap that splitting cannot
    // close. Locate each turning point by bisection instead.
    const SAMPLES: usize = 1024;
    // Starting spans per traced branch; adaptive subdivision adds the rest.
    const LOOP_SAMPLES: usize = 32;
    let step = 2.0 * PI / SAMPLES as f64;
    let positive: Vec<bool> = (0..SAMPLES)
        .map(|i| discriminant(i as f64 * step) > 0.0)
        .collect();
    let mut rising = Vec::new();
    let mut falling = Vec::new();
    for i in 0..SAMPLES {
        let (from, to) = (positive[i], positive[(i + 1) % SAMPLES]);
        if from == to {
            continue;
        }
        let (mut lo, mut hi) = (i as f64 * step, (i + 1) as f64 * step);
        for _ in 0..64 {
            let mid = 0.5 * (lo + hi);
            if (discriminant(mid) > 0.0) == from {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let turning = 0.5 * (lo + hi);
        if to {
            rising.push(turning);
        } else {
            falling.push(turning);
        }
    }

    // Samples are (u, upper root?) pairs. Each loop is a C1 cubic Hermite
    // spline through exact points with exact tangents (the cross product of
    // the two surface normals), subdivided until its quarter points lie within
    // OFF_SURFACE of both cylinders. A degree-one polyline needs tens of
    // thousands of points for the same accuracy, and vertices found by
    // intersecting it with other edges must sit well inside the kernel's 1e-5
    // vertex and pcurve tolerances.
    const OFF_SURFACE: f64 = 1.0e-7;
    let radial = |p: Pnt, cylinder: &CylindricalSurface| {
        let axis = GeomVec::from_dir(cylinder.position().direction());
        let d = p - cylinder.position().location();
        (d - axis * d.dot(&axis), cylinder.radius())
    };
    let off_surface = |p: Pnt| {
        let (r1, radius1) = radial(p, c1);
        let (r2, radius2) = radial(p, c2);
        (r1.magnitude() - radius1)
            .abs()
            .max((r2.magnitude() - radius2).abs())
    };
    let at = |u: f64, upper: bool| {
        let (lower, higher) = roots(u);
        let p = c1.point(u, if upper { higher } else { lower });
        let tangent = radial(p, c1).0.cross(&radial(p, c2).0);
        let unit = tangent.normalized().map(GeomVec::from_dir);
        (p, unit.unwrap_or(GeomVec::new(0.0, 0.0, 0.0)))
    };
    let trace = |samples: &[(f64, bool)]| -> (Vec<Pnt>, Vec<GeomVec>) {
        let (p0, t0) = at(samples[0].0, samples[0].1);
        let (mut points, mut tangents) = (vec![p0], vec![t0]);
        for pair in samples.windows(2) {
            let ((u0, _), (u1, upper)) = (pair[0], pair[1]);
            // Across a turning point the branches meet, so the span from it
            // runs along the branch it leads into.
            let start = (
                *points.last().expect("seeded"),
                *tangents.last().expect("seeded"),
            );
            let mut stack = vec![(u0, start, u1, at(u1, upper), 0)];
            while let Some((a, (pa, ta), b, (pb, tb), depth)) = stack.pop() {
                // Orient both tangents along the span before testing it.
                let chord = pb - pa;
                let ta = if ta.dot(&chord) < 0.0 { ta * -1.0 } else { ta };
                let tb = if tb.dot(&chord) < 0.0 { tb * -1.0 } else { tb };
                let length = chord.magnitude();
                let (b1, b2) = (pa + ta * (length / 3.0), pb - tb * (length / 3.0));
                let bezier = |s: f64| {
                    let r = 1.0 - s;
                    let w = [r * r * r, 3.0 * r * r * s, 3.0 * r * s * s, s * s * s];
                    Pnt::new(
                        w[0] * pa.x() + w[1] * b1.x() + w[2] * b2.x() + w[3] * pb.x(),
                        w[0] * pa.y() + w[1] * b1.y() + w[2] * b2.y() + w[3] * pb.y(),
                        w[0] * pa.z() + w[1] * b1.z() + w[2] * b2.z() + w[3] * pb.z(),
                    )
                };
                let error = [0.25, 0.5, 0.75]
                    .into_iter()
                    .map(|s| off_surface(bezier(s)))
                    .fold(0.0, f64::max);
                if depth < 24 && error > OFF_SURFACE {
                    let mid = 0.5 * (a + b);
                    let pm = at(mid, upper);
                    // Right half first on the stack so the left half pops next.
                    stack.push((mid, pm, b, (pb, tb), depth + 1));
                    stack.push((a, (pa, ta), mid, pm, depth + 1));
                } else {
                    // Keep each point's tangent oriented along the loop.
                    *tangents.last_mut().expect("seeded") = ta;
                    points.push(pb);
                    tangents.push(tb);
                }
            }
        }
        // A loop closes on the very same point, not a rounded neighbour.
        let (first, first_tangent) = (points[0], tangents[0]);
        *points.last_mut().expect("non-empty") = first;
        let last = tangents.last_mut().expect("non-empty");
        if last.dot(&first_tangent) < 0.0 {
            *last = *last * -1.0;
        }
        (points, tangents)
    };

    let mut loops: Vec<(Vec<Pnt>, Vec<GeomVec>)> = Vec::new();
    if rising.is_empty() {
        if !positive[0] {
            return Vec::new();
        }
        // c1 passes right through c2: one closed loop per root.
        for upper in [false, true] {
            let samples: Vec<_> = (0..=2 * LOOP_SAMPLES)
                .map(|i| (i as f64 * PI / LOOP_SAMPLES as f64, upper))
                .collect();
            loops.push(trace(&samples));
        }
    } else {
        // Each interval where the discriminant is positive is one loop: out
        // along the lower root, back along the upper, meeting at both turning
        // points. Cosine spacing clusters samples there, where v changes like
        // the square root of the distance in u.
        for &start in &rising {
            let Some(end) = falling
                .iter()
                .map(|&u| if u > start { u } else { u + 2.0 * PI })
                .min_by(f64::total_cmp)
            else {
                continue;
            };
            let u_at = |i: usize| {
                start + (end - start) * 0.5 * (1.0 - (PI * i as f64 / LOOP_SAMPLES as f64).cos())
            };
            let mut samples: Vec<_> = (0..=LOOP_SAMPLES).map(|i| (u_at(i), false)).collect();
            samples.extend((0..LOOP_SAMPLES).rev().map(|i| (u_at(i), true)));
            loops.push(trace(&samples));
        }
    }

    loops
        .into_iter()
        .filter(|(points, _)| points.len() >= 2)
        .map(|(points, tangents)| GeomCurve::BSpline(hermite_bspline(&points, &tangents)))
        .collect()
}

/// A C1 cubic B-spline through `points` with unit `tangents`, parameterized by
/// chord length: each span is the Bezier segment of its Hermite data.
fn hermite_bspline(points: &[Pnt], tangents: &[GeomVec]) -> BSplineCurve {
    let mut poles = vec![points[0]];
    let mut knots = vec![0.0];
    let mut mults = vec![4];
    let mut parameter = 0.0;
    for (span, pair) in points.windows(2).enumerate() {
        let length = pair[0].distance(&pair[1]).max(1.0e-12);
        poles.push(pair[0] + tangents[span] * (length / 3.0));
        poles.push(pair[1] - tangents[span + 1] * (length / 3.0));
        poles.push(pair[1]);
        parameter += length;
        knots.push(parameter);
        mults.push(if span + 2 == points.len() { 4 } else { 3 });
    }
    BSplineCurve::new(3, poles, None, knots, mults)
}

/// Find intersection curves between two surfaces.
pub fn surface_surface(s1: &GeomSurface, s2: &GeomSurface, tol: f64) -> Vec<GeomCurve> {
    let mut budget = GeometryWorkBudget::intersection_default();
    surface_surface_with_budget(s1, s2, tol, &mut budget).unwrap_or_default()
}

/// Budgeted surface/surface intersection for atomic modeling operations.
pub fn surface_surface_with_budget(
    s1: &GeomSurface,
    s2: &GeomSurface,
    tol: f64,
    budget: &mut GeometryWorkBudget,
) -> Result<Vec<GeomCurve>, BandTopologyError> {
    surface_surface_with_budget_and_cancel(
        s1,
        s2,
        tol,
        budget,
        &openrcad_foundation::NeverCancelled,
    )
}

/// Checked intersection with cooperative cancellation and a shared work budget.
pub fn surface_surface_with_budget_and_cancel(
    s1: &GeomSurface,
    s2: &GeomSurface,
    tol: f64,
    budget: &mut GeometryWorkBudget,
    cancel: &dyn openrcad_foundation::CancellationProbe,
) -> Result<Vec<GeomCurve>, BandTopologyError> {
    cancel
        .check_cancelled()
        .map_err(|_| BandTopologyError::Cancelled)?;
    budget.charge(GeometryWorkStage::Intersection, 1)?;
    if let Some(curves) = analytic_surface_surface(s1, s2, tol) {
        return Ok(curves);
    }
    let (u1_min, u1_max, v1_min, v1_max) = s1.bounds();
    let (u2_min, u2_max, v2_min, v2_max) = s2.bounds();
    let u1_min = clamp_bound(u1_min, -100.0);
    let u1_max = clamp_bound(u1_max, 100.0);
    let v1_min = clamp_bound(v1_min, -100.0);
    let v1_max = clamp_bound(v1_max, 100.0);
    let u2_min = clamp_bound(u2_min, -100.0);
    let u2_max = clamp_bound(u2_max, 100.0);
    let v2_min = clamp_bound(v2_min, -100.0);
    let v2_max = clamp_bound(v2_max, 100.0);

    // Adaptive recursive subdivision with rigorous interval pruning. Descend
    // paired parameter rectangles, pruning any pair whose guaranteed-enclosing
    // boxes do not overlap, and refine a point wherever both patches are small.
    // This replaces the previous uniform 16×16×16×16 grid (O(16⁴) regardless of
    // whether the surfaces meet): the pruning makes the work proportional to the
    // actual intersection set, and a real intersection can never be dropped.
    let tau = |a: f64, b: f64| {
        let arithmetic = (a.abs() + b.abs()).max(f64::MIN_POSITIVE) * f64::EPSILON * 64.0;
        ((b - a).abs() / 24.0).max(arithmetic)
    };
    let tauu1 = tau(u1_min, u1_max);
    let tauv1 = tau(v1_min, v1_max);
    let tauu2 = tau(u2_min, u2_max);
    let tauv2 = tau(v2_min, v2_max);

    let mut intersection_points: Vec<Pnt> = Vec::new();
    ssi_subdivide(
        s1,
        u1_min,
        u1_max,
        v1_min,
        v1_max,
        s2,
        u2_min,
        u2_max,
        v2_min,
        v2_max,
        tol,
        0,
        (tauu1, tauv1, tauu2, tauv2),
        &mut intersection_points,
        budget,
        cancel,
    )?;

    let chain_tol = nearest_neighbour_chain_tol(&intersection_points, tol);

    let paths = chain_points(&intersection_points, chain_tol);

    let mut curves = Vec::new();
    for path in paths {
        if path.len() >= 2 {
            curves.push(GeomCurve::BSpline(polyline_to_bspline(&path)));
        }
    }

    Ok(curves)
}

fn eval_d1(s: &GeomSurface, u: f64, v: f64) -> (Pnt, GeomVec, GeomVec) {
    s.d1(u, v)
}

/// Recursive surface-surface subdivision driver. `tau` holds a target width per
/// parameter axis (≈ 1/24 of each surface's domain) — a patch is a leaf once
/// both patches are at or below their targets. Pruning by guaranteed-enclosing
/// interval boxes keeps the visited-pair count proportional to the actual
/// intersection set rather than the full domain product.
#[allow(clippy::too_many_arguments)]
fn ssi_subdivide(
    s1: &GeomSurface,
    u1_min: f64,
    u1_max: f64,
    v1_min: f64,
    v1_max: f64,
    s2: &GeomSurface,
    u2_min: f64,
    u2_max: f64,
    v2_min: f64,
    v2_max: f64,
    tol: f64,
    depth: usize,
    tau: (f64, f64, f64, f64),
    out: &mut Vec<Pnt>,
    budget: &mut GeometryWorkBudget,
    cancel: &dyn openrcad_foundation::CancellationProbe,
) -> Result<(), BandTopologyError> {
    cancel
        .check_cancelled()
        .map_err(|_| BandTopologyError::Cancelled)?;
    budget.charge(GeometryWorkStage::Intersection, 1)?;
    let mut b1 = s1
        .interval_point(u1_min, u1_max, v1_min, v1_max)
        .to_bndbox();
    b1.enlarge(tol);
    let mut b2 = s2
        .interval_point(u2_min, u2_max, v2_min, v2_max)
        .to_bndbox();
    b2.enlarge(tol);
    if b1.is_out_box(&b2) {
        return Ok(());
    }

    let (tauu1, tauv1, tauu2, tauv2) = tau;
    let uw1 = u1_max - u1_min;
    let vw1 = v1_max - v1_min;
    let uw2 = u2_max - u2_min;
    let vw2 = v2_max - v2_min;

    let small1 = uw1 <= tauu1 && vw1 <= tauv1;
    let small2 = uw2 <= tauu2 && vw2 <= tauv2;
    if small1 && small2 || depth > 26 {
        let u1m = 0.5 * (u1_min + u1_max);
        let v1m = 0.5 * (v1_min + v1_max);
        let u2m = 0.5 * (u2_min + u2_max);
        let v2m = 0.5 * (v2_min + v2_max);
        if let Some(p) = refine_surface_surface_point(s1, u1m, v1m, s2, u2m, v2m, tol) {
            if !out.iter().any(|q: &Pnt| q.distance(&p) < tol) {
                out.push(p);
            }
        }
        return Ok(());
    }

    // Bisect the most-oversized parameter dimension (width / target), holding
    // the other patch fixed. Each step halves one dimension, so the depth cap
    // above guarantees termination.
    let oversizes = [uw1 / tauu1, vw1 / tauv1, uw2 / tauu2, vw2 / tauv2];
    let which = (0..4)
        .max_by(|&a, &b| {
            oversizes[a]
                .partial_cmp(&oversizes[b])
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .unwrap_or(0);
    match which {
        0 => {
            let mid = 0.5 * (u1_min + u1_max);
            ssi_subdivide(
                s1,
                u1_min,
                mid,
                v1_min,
                v1_max,
                s2,
                u2_min,
                u2_max,
                v2_min,
                v2_max,
                tol,
                depth + 1,
                tau,
                out,
                budget,
                cancel,
            )?;
            ssi_subdivide(
                s1,
                mid,
                u1_max,
                v1_min,
                v1_max,
                s2,
                u2_min,
                u2_max,
                v2_min,
                v2_max,
                tol,
                depth + 1,
                tau,
                out,
                budget,
                cancel,
            )?;
        }
        1 => {
            let mid = 0.5 * (v1_min + v1_max);
            ssi_subdivide(
                s1,
                u1_min,
                u1_max,
                v1_min,
                mid,
                s2,
                u2_min,
                u2_max,
                v2_min,
                v2_max,
                tol,
                depth + 1,
                tau,
                out,
                budget,
                cancel,
            )?;
            ssi_subdivide(
                s1,
                u1_min,
                u1_max,
                mid,
                v1_max,
                s2,
                u2_min,
                u2_max,
                v2_min,
                v2_max,
                tol,
                depth + 1,
                tau,
                out,
                budget,
                cancel,
            )?;
        }
        2 => {
            let mid = 0.5 * (u2_min + u2_max);
            ssi_subdivide(
                s1,
                u1_min,
                u1_max,
                v1_min,
                v1_max,
                s2,
                u2_min,
                mid,
                v2_min,
                v2_max,
                tol,
                depth + 1,
                tau,
                out,
                budget,
                cancel,
            )?;
            ssi_subdivide(
                s1,
                u1_min,
                u1_max,
                v1_min,
                v1_max,
                s2,
                mid,
                u2_max,
                v2_min,
                v2_max,
                tol,
                depth + 1,
                tau,
                out,
                budget,
                cancel,
            )?;
        }
        _ => {
            let mid = 0.5 * (v2_min + v2_max);
            ssi_subdivide(
                s1,
                u1_min,
                u1_max,
                v1_min,
                v1_max,
                s2,
                u2_min,
                u2_max,
                v2_min,
                mid,
                tol,
                depth + 1,
                tau,
                out,
                budget,
                cancel,
            )?;
            ssi_subdivide(
                s1,
                u1_min,
                u1_max,
                v1_min,
                v1_max,
                s2,
                u2_min,
                u2_max,
                mid,
                v2_max,
                tol,
                depth + 1,
                tau,
                out,
                budget,
                cancel,
            )?;
        }
    }
    Ok(())
}

/// Chaining tolerance for the intersection-point polyline: ~2× the median
/// nearest-neighbour gap, so points on the same arc link up regardless of how
/// the parametrisation's scale maps to world distance.
fn nearest_neighbour_chain_tol(pts: &[Pnt], tol: f64) -> f64 {
    if pts.len() < 2 {
        return tol.abs();
    }
    let mut gaps: Vec<f64> = (0..pts.len())
        .map(|i| {
            (0..pts.len())
                .filter(|&j| j != i)
                .map(|j| pts[i].distance(&pts[j]))
                .fold(f64::INFINITY, f64::min)
        })
        .collect();
    gaps.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    (gaps[gaps.len() / 2] * 2.0).max(tol * 2.0)
}

fn refine_surface_surface_point(
    s1: &GeomSurface,
    u1_start: f64,
    v1_start: f64,
    s2: &GeomSurface,
    u2_start: f64,
    v2_start: f64,
    tol: f64,
) -> Option<Pnt> {
    let (u1_min, u1_max, v1_min, v1_max) = s1.bounds();
    let (u2_min, u2_max, v2_min, v2_max) = s2.bounds();
    let u1_min = clamp_bound(u1_min, -100.0);
    let u1_max = clamp_bound(u1_max, 100.0);
    let v1_min = clamp_bound(v1_min, -100.0);
    let v1_max = clamp_bound(v1_max, 100.0);
    let u2_min = clamp_bound(u2_min, -100.0);
    let u2_max = clamp_bound(u2_max, 100.0);
    let v2_min = clamp_bound(v2_min, -100.0);
    let v2_max = clamp_bound(v2_max, 100.0);

    let mut u1 = u1_start;
    let mut v1 = v1_start;
    let mut u2 = u2_start;
    let mut v2 = v2_start;

    for _ in 0..10 {
        let (p1, s1_u, s1_v) = eval_d1(s1, u1, v1);
        let (p2, s2_u, s2_v) = eval_d1(s2, u2, v2);

        let rhs = p2 - p1;
        if rhs.magnitude() < 1e-12 {
            break;
        }

        let j0 = s1_u;
        let j1 = s1_v;
        let j2 = -s2_u;
        let j3 = -s2_v;

        let m00 = j0.x() * j0.x() + j1.x() * j1.x() + j2.x() * j2.x() + j3.x() * j3.x();
        let m01 = j0.x() * j0.y() + j1.x() * j1.y() + j2.x() * j2.y() + j3.x() * j3.y();
        let m02 = j0.x() * j0.z() + j1.x() * j1.z() + j2.x() * j2.z() + j3.x() * j3.z();

        let m10 = m01;
        let m11 = j0.y() * j0.y() + j1.y() * j1.y() + j2.y() * j2.y() + j3.y() * j3.y();
        let m12 = j0.y() * j0.z() + j1.y() * j1.z() + j2.y() * j2.z() + j3.y() * j3.z();

        let m20 = m02;
        let m21 = m12;
        let m22 = j0.z() * j0.z() + j1.z() * j1.z() + j2.z() * j2.z() + j3.z() * j3.z();

        let col0 = GeomVec::new(m00, m10, m20);
        let col1 = GeomVec::new(m01, m11, m21);
        let col2 = GeomVec::new(m02, m12, m22);

        if let Some((y0, y1, y2)) = solve_3x3(col0, col1, col2, rhs) {
            let y = GeomVec::new(y0, y1, y2);
            let du1 = j0.dot(&y);
            let dv1 = j1.dot(&y);
            let du2 = -j2.dot(&y);
            let dv2 = -j3.dot(&y);

            u1 = clamp_ordered(u1 + du1, u1_min, u1_max);
            v1 = clamp_ordered(v1 + dv1, v1_min, v1_max);
            u2 = clamp_ordered(u2 - du2, u2_min, u2_max);
            v2 = clamp_ordered(v2 - dv2, v2_min, v2_max);

            if du1.abs() < 1e-9 && dv1.abs() < 1e-9 && du2.abs() < 1e-9 && dv2.abs() < 1e-9 {
                break;
            }
        } else {
            break;
        }
    }

    let p1 = s1.point(u1, v1);
    let p2 = s2.point(u2, v2);
    if p1.distance(&p2) <= tol {
        Some(p1.midpoint(&p2))
    } else {
        None
    }
}

fn chain_points(points: &[Pnt], tol: f64) -> Vec<Vec<Pnt>> {
    let mut pts = points.to_vec();
    let mut paths = Vec::new();

    let mut unique_pts = Vec::new();
    for p in pts {
        if !unique_pts.iter().any(|u: &Pnt| u.distance(&p) < 1e-6) {
            unique_pts.push(p);
        }
    }
    pts = unique_pts;

    while !pts.is_empty() {
        let mut path = Vec::new();
        let start = pts.remove(0);
        path.push(start);

        loop {
            let last = path.last().unwrap();
            let mut best_idx = None;
            let mut min_dist = tol;
            for (idx, p) in pts.iter().enumerate() {
                let dist = last.distance(p);
                if dist < min_dist {
                    min_dist = dist;
                    best_idx = Some(idx);
                }
            }
            if let Some(idx) = best_idx {
                path.push(pts.remove(idx));
            } else {
                break;
            }
        }

        loop {
            let first = path.first().unwrap();
            let mut best_idx = None;
            let mut min_dist = tol;
            for (idx, p) in pts.iter().enumerate() {
                let dist = first.distance(p);
                if dist < min_dist {
                    min_dist = dist;
                    best_idx = Some(idx);
                }
            }
            if let Some(idx) = best_idx {
                path.insert(0, pts.remove(idx));
            } else {
                break;
            }
        }

        paths.push(path);
    }

    paths
}

fn polyline_to_bspline(points: &[Pnt]) -> BSplineCurve {
    let n = points.len();
    assert!(n >= 2);
    let degree = 1;
    let mut knots = Vec::new();
    let mut mults = Vec::new();

    knots.push(0.0);
    mults.push(2);

    let mut current_t = 0.0;
    for i in 1..n {
        let dist = points[i].distance(&points[i - 1]);
        current_t += dist.max(1e-5);
        if i < n - 1 {
            knots.push(current_t);
            mults.push(1);
        } else {
            knots.push(current_t);
            mults.push(2);
        }
    }

    BSplineCurve::new(degree, points.to_vec(), None, knots, mults)
}

/// Find the parameters `(u, v)` on a surface nearest a 3D point `p`. Uses the
/// exact analytic projection for analytic surfaces and Newton refinement for
/// B-splines.
pub fn search_nearest_parameter(s: &GeomSurface, p: &Pnt, hint: (f64, f64)) -> (f64, f64) {
    match s {
        GeomSurface::BSpline(_) => search_nearest_parameter_newton(s, p, hint),
        _ => uv_of(s, p),
    }
}

/// Newton-refined nearest parameter (used for B-splines and as a fallback).
fn search_nearest_parameter_newton(s: &GeomSurface, p: &Pnt, hint: (f64, f64)) -> (f64, f64) {
    let (u_min, u_max, v_min, v_max) = s.bounds();
    let u_min = clamp_bound(u_min, -100.0);
    let u_max = clamp_bound(u_max, 100.0);
    let v_min = clamp_bound(v_min, -100.0);
    let v_max = clamp_bound(v_max, 100.0);

    let mut u = clamp_ordered(hint.0, u_min, u_max);
    let mut v = clamp_ordered(hint.1, v_min, v_max);

    for _ in 0..10 {
        let (ps, su, sv) = eval_d1(s, u, v);
        let diff = ps - *p;
        if diff.magnitude() < 1e-12 {
            break;
        }

        let du = diff.dot(&su) / (su.dot(&su) + 1e-15);
        let dv = diff.dot(&sv) / (sv.dot(&sv) + 1e-15);

        u = clamp_ordered(u - du, u_min, u_max);
        v = clamp_ordered(v - dv, v_min, v_max);

        if du.abs() < 1e-9 && dv.abs() < 1e-9 {
            break;
        }
    }
    (u, v)
}

/// One vertex of a sampled boundary loop: its `(u, v)`, and the boundary edge
/// (index into the wire's edges) and curve parameters of the chord from it to
/// the next vertex.
struct LoopSample {
    uv: (f64, f64),
    edge: usize,
    from: f64,
    to: f64,
}

/// Shift a periodic `u` onto the `2π` branch nearest `reference`.
fn unwrap_u(mut u: f64, reference: f64) -> f64 {
    while u - reference > PI {
        u -= 2.0 * PI;
    }
    while reference - u > PI {
        u += 2.0 * PI;
    }
    u
}

/// Sample the boundary loop `edges` of a face carried by `surface` in the
/// surface's `(u, v)` parameter space. Each edge is sampled in its oriented
/// traversal direction; the angular coordinate is unwrapped so the polygon does
/// not jump across the `0 / 2π` seam.
fn loop_uv_samples(surface: &GeomSurface, edges: &[openrcad_topo::Edge]) -> Vec<LoopSample> {
    let periodic = !matches!(surface, GeomSurface::Plane(_) | GeomSurface::BSpline(_));
    let samples = 6;
    let mut poly: Vec<LoopSample> = Vec::new();
    for (index, edge) in edges.iter().enumerate() {
        let reversed = edge.orientation() == Orientation::Reversed;
        let (a, b) = (edge.first(), edge.last());
        let at = |i: usize| {
            let frac = i as f64 / samples as f64;
            if reversed {
                b + (a - b) * frac
            } else {
                a + (b - a) * frac
            }
        };
        for i in 0..samples {
            let t = at(i);
            let p = match edge.curve() {
                Some(c) => c.point(t),
                None => edge.start().point(),
            };
            let (mut pu, pv) = uv_of(surface, &p);
            if periodic {
                if let Some(previous) = poly.last() {
                    pu = unwrap_u(pu, previous.uv.0);
                }
            }
            poly.push(LoopSample {
                uv: (pu, pv),
                edge: index,
                from: t,
                to: at(i + 1),
            });
        }
    }
    poly
}

/// Add the points between `p0` and `p1` (the chord over `t0..t1` of `curve`)
/// needed to decide containment of `query` exactly: a chord is subdivided only
/// while `query` lies within its sag band, where the chord and the curve can
/// disagree about which side the query is on.
#[allow(clippy::too_many_arguments)]
fn refine_chord(
    surface: &GeomSurface,
    curve: &GeomCurve,
    periodic: bool,
    (t0, t1): (f64, f64),
    p0: (f64, f64),
    p1: (f64, f64),
    query: (f64, f64),
    depth: u32,
    out: &mut Vec<(f64, f64)>,
) {
    const MAX_DEPTH: u32 = 30;
    if depth >= MAX_DEPTH {
        return;
    }
    let distance_to_chord = |p: (f64, f64)| {
        let (dx, dy) = (p1.0 - p0.0, p1.1 - p0.1);
        let length2 = dx * dx + dy * dy;
        let s = if length2 > 0.0 {
            (((p.0 - p0.0) * dx + (p.1 - p0.1) * dy) / length2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        (p.0 - p0.0 - s * dx).hypot(p.1 - p0.1 - s * dy)
    };
    let tm = 0.5 * (t0 + t1);
    let (mut mu, mv) = uv_of(surface, &curve.point(tm));
    if periodic {
        mu = unwrap_u(mu, p0.0);
    }
    let mid = (mu, mv);
    // For an arc the midpoint is the farthest point from the chord; twice its
    // offset bounds a smooth curve's sag over a span this short.
    let sag = distance_to_chord(mid);
    if sag <= 1.0e-12 || distance_to_chord(query) > 2.0 * sag {
        return;
    }
    refine_chord(
        surface,
        curve,
        periodic,
        (t0, tm),
        p0,
        mid,
        query,
        depth + 1,
        out,
    );
    out.push(mid);
    refine_chord(
        surface,
        curve,
        periodic,
        (tm, t1),
        mid,
        p1,
        query,
        depth + 1,
        out,
    );
}

/// Bring the test angle `u` onto the same `2π` branch as `poly`'s angular
/// range. A branch inside the polygon's `u` extent wins; aligning to the mean
/// alone fails for a face wider than half a turn (a merged hole wall spans
/// ~335°), where the mean can sit more than π from part of the face.
fn align_u(u: f64, poly: &[(f64, f64)]) -> f64 {
    if poly.is_empty() {
        return u;
    }
    let (low, high) = poly
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(low, high), p| {
            (low.min(p.0), high.max(p.0))
        });
    let mean = poly.iter().map(|p| p.0).sum::<f64>() / poly.len() as f64;
    let mut x = u;
    while x - mean > PI {
        x -= 2.0 * PI;
    }
    while mean - x > PI {
        x += 2.0 * PI;
    }
    if (low..=high).contains(&x) {
        return x;
    }
    [x - 2.0 * PI, x + 2.0 * PI]
        .into_iter()
        .find(|candidate| (low..=high).contains(candidate))
        .unwrap_or(x)
}

/// Whether `(u, v)` lies inside the loop `wire`. The loop is a coarse polygon
/// except near the query, where curved boundary edges are refined until the
/// polygon agrees with the true curve on which side the query lies: six
/// chords on a 120° arc sag 1.5% of its radius, enough to misclassify a point
/// just inside an intersecting hole.
fn is_inside_wire_uv(u: f64, v: f64, surface: &GeomSurface, wire: &openrcad_topo::Wire) -> bool {
    let periodic = !matches!(surface, GeomSurface::Plane(_) | GeomSurface::BSpline(_));
    let edges = wire.edges();
    let samples = loop_uv_samples(surface, &edges);
    let coarse: Vec<(f64, f64)> = samples.iter().map(|sample| sample.uv).collect();
    let aligned_u = if periodic { align_u(u, &coarse) } else { u };
    let query = (aligned_u, v);
    let mut polygon = Vec::with_capacity(coarse.len());
    for (index, sample) in samples.iter().enumerate() {
        polygon.push(sample.uv);
        let Some(curve) = edges[sample.edge].curve() else {
            continue;
        };
        if matches!(curve, GeomCurve::Line(_)) {
            continue;
        }
        let mut next = coarse[(index + 1) % coarse.len()];
        if periodic {
            next.0 = unwrap_u(next.0, sample.uv.0);
        }
        refine_chord(
            surface,
            curve,
            periodic,
            (sample.from, sample.to),
            sample.uv,
            next,
            query,
            0,
            &mut polygon,
        );
    }
    point_in_polygon_2d(query, &polygon)
}

/// Checks if a parametric point `(u, v)` lies inside the face's outer trimming
/// loop, without treating inner holes as support overflow.
///
/// This distinction matters to local operations: crossing a hole can be handled
/// by the existing imprint/trim topology, while leaving the finite outer support
/// requires contact-curve continuation onto a successor face.
pub fn is_inside_outer_trimming_loop(u: f64, v: f64, face: &Face) -> bool {
    let (Some(surface), Some(outer_wire)) = (face.surface(), face.outer_wire()) else {
        return false;
    };
    is_inside_wire_uv(u, v, surface, &outer_wire)
}

/// Checks if a parametric point `(u, v)` lies topologically inside the face's
/// trimming loops (inside the outer loop and outside every hole). Works for both
/// planar and analytic curved faces by testing in `(u, v)` parameter space.
pub fn is_inside_trimming_loops(u: f64, v: f64, face: &Face) -> bool {
    let Some(surface) = face.surface() else {
        return false;
    };

    if !is_inside_outer_trimming_loop(u, v, face) {
        return false;
    }

    for hole in face.inner_wires() {
        if is_inside_wire_uv(u, v, surface, &hole) {
            return false;
        }
    }
    true
}

/// Every forward intersection point of a ray with a face that lies within the
/// face's trimming loops. Uses exact analytic line/surface intersection for
/// analytic surfaces and the generic subdivision solver for B-splines.
pub fn ray_face_all(ray_origin: &Pnt, ray_dir: &GeomVec, face: &Face, tol: f64) -> Vec<Pnt> {
    let surface = match face.surface() {
        Some(s) => s,
        None => return Vec::new(),
    };

    let raw = match line_surface(ray_origin, ray_dir, surface) {
        Some(pts) => pts,
        None => {
            let dir = Dir::new(ray_dir.x(), ray_dir.y(), ray_dir.z());
            let curve = GeomCurve::Line(openrcad_geom::Line::from_point_dir(*ray_origin, dir));
            curve_surface(&curve, surface, tol)
        }
    };

    let mut hits = Vec::new();
    for pt in raw {
        if (pt - *ray_origin).dot(ray_dir) < -tol {
            continue; // behind the ray origin
        }
        let (u, v) = uv_of(surface, &pt);
        if is_inside_trimming_loops(u, v, face) {
            hits.push(pt);
        }
    }
    hits
}

/// The nearest forward intersection of a ray with a face (within its trimming
/// loops), if any.
pub fn ray_face(ray_origin: &Pnt, ray_dir: &GeomVec, face: &Face, tol: f64) -> Option<Pnt> {
    ray_face_all(ray_origin, ray_dir, face, tol)
        .into_iter()
        .min_by(|a, b| {
            let da = (*a - *ray_origin).dot(ray_dir);
            let db = (*b - *ray_origin).dot(ray_dir);
            da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
        })
}

/// Find trimmed intersection curves between two faces.
///
/// Intersects the host surfaces and trims the resulting curves to the boundaries of both faces.
pub fn surface_surface_curves(face1: &Face, face2: &Face, tol: f64) -> Vec<(GeomCurve, f64, f64)> {
    let mut budget = GeometryWorkBudget::intersection_default();
    surface_surface_curves_with_budget(face1, face2, tol, &mut budget).unwrap_or_default()
}

/// Budgeted face/face intersection used by operations that must reject
/// atomically instead of treating an exhausted solver as "no intersection".
pub fn surface_surface_curves_with_budget(
    face1: &Face,
    face2: &Face,
    tol: f64,
    budget: &mut GeometryWorkBudget,
) -> Result<Vec<(GeomCurve, f64, f64)>, BandTopologyError> {
    surface_surface_curves_with_budget_and_cancel(
        face1,
        face2,
        tol,
        budget,
        &openrcad_foundation::NeverCancelled,
    )
}

/// Checked intersection with cooperative cancellation and a shared work budget.
pub fn surface_surface_curves_with_budget_and_cancel(
    face1: &Face,
    face2: &Face,
    tol: f64,
    budget: &mut GeometryWorkBudget,
    cancel: &dyn openrcad_foundation::CancellationProbe,
) -> Result<Vec<(GeomCurve, f64, f64)>, BandTopologyError> {
    cancel
        .check_cancelled()
        .map_err(|_| BandTopologyError::Cancelled)?;
    let s1 = match face1.surface() {
        Some(s) => s,
        None => return Ok(Vec::new()),
    };
    let s2 = match face2.surface() {
        Some(s) => s,
        None => return Ok(Vec::new()),
    };

    let raw_curves = surface_surface_with_budget_and_cancel(s1, s2, tol, budget, cancel)?;
    let mut trimmed_curves = Vec::new();
    let debug = std::env::var_os("OPENRCAD_BOOLEAN_DEBUG").is_some()
        && matches!(
            (s1, s2),
            (GeomSurface::Cylinder(_), GeomSurface::Cylinder(_))
        );

    for curve in raw_curves {
        cancel
            .check_cancelled()
            .map_err(|_| BandTopologyError::Cancelled)?;
        let (mut c_min, mut c_max) = curve.bounds();
        // Plane intersections are infinite lines whose parameter origin is
        // unrelated to either face. A fixed [-100,100] search drops all splits
        // when the part is translated. Project the actual face bounds instead.
        if matches!(curve, GeomCurve::Line(_)) && (!c_min.is_finite() || !c_max.is_finite()) {
            let (origin, direction) = curve.d1(0.0);
            let denominator = direction.dot(&direction);
            let mut low = f64::INFINITY;
            let mut high = f64::NEG_INFINITY;
            for face in [face1, face2] {
                if let Some((lo, hi)) = crate::bvh::compute_face_bounds(face).corners() {
                    for x in [lo.x(), hi.x()] {
                        for y in [lo.y(), hi.y()] {
                            for z in [lo.z(), hi.z()] {
                                let parameter =
                                    (Pnt::new(x, y, z) - origin).dot(&direction) / denominator;
                                low = low.min(parameter);
                                high = high.max(parameter);
                            }
                        }
                    }
                }
            }
            if low.is_finite() && high.is_finite() {
                // Artificial search endpoints must stay outside the parameter
                // deduplication band below; otherwise they replace a real trim
                // crossing and leave a small open edge in the sewn boolean.
                let margin = (tol.max(1.0e-5) * 16.0).max((high - low).abs() * 1.0e-6);
                if !c_min.is_finite() {
                    c_min = (low - margin).min(-100.0);
                }
                if !c_max.is_finite() {
                    c_max = (high + margin).max(100.0);
                }
            }
        }
        let c_min = if c_min.is_infinite() || c_min.is_nan() {
            -100.0
        } else {
            c_min
        };
        let c_max = if c_max.is_infinite() || c_max.is_nan() {
            100.0
        } else {
            c_max
        };

        let mut split_params = vec![c_min, c_max];

        // Find exact analytic crossings of the intersection curve with both faces'
        // boundary edges — the authoritative trim boundaries.
        let mut edge_crossings = Vec::new();
        let add_intersections = |face: &Face, out: &mut Vec<f64>| {
            for wire in face.wires() {
                for edge in wire.edges() {
                    if let Some(edge_curve) = edge.curve() {
                        if curves_overlap(edge_curve, &curve, tol) {
                            for point in [
                                edge_curve.point(edge.first()),
                                edge_curve.point(edge.last()),
                            ] {
                                let t = crate::boolean::project_point_on_curve(
                                    &point, &curve, c_min, c_max,
                                );
                                if t > c_min + tol && t < c_max - tol {
                                    out.push(t);
                                }
                            }
                            continue;
                        }
                        let pts = curve_curve(edge_curve, &curve, tol);
                        for pt in pts {
                            let t =
                                crate::boolean::project_point_on_curve(&pt, &curve, c_min, c_max);
                            if t > c_min + tol && t < c_max - tol {
                                out.push(t);
                            }
                        }
                    }
                }
            }
        };

        add_intersections(face1, &mut edge_crossings);
        add_intersections(face2, &mut edge_crossings);
        split_params.extend(edge_crossings.iter().copied());
        add_containment_transitions(
            &curve,
            c_min,
            c_max,
            face1,
            s1,
            &mut split_params,
            &edge_crossings,
            tol,
        );
        add_containment_transitions(
            &curve,
            c_min,
            c_max,
            face2,
            s2,
            &mut split_params,
            &edge_crossings,
            tol,
        );

        // Sort and deduplicate split parameters
        split_params.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let mut unique_params: Vec<f64> = Vec::new();
        for &t in &split_params {
            if unique_params.iter().all(|&u| (u - t).abs() > 1e-5) {
                unique_params.push(t);
            }
        }
        if debug {
            let a = curve.point(c_min);
            let b = curve.point(c_max);
            eprintln!(
                "ssi cylinder raw ({:.4},{:.4},{:.4}) -> ({:.4},{:.4},{:.4}) params={}",
                a.x(),
                a.y(),
                a.z(),
                b.x(),
                b.y(),
                b.z(),
                unique_params.len()
            );
        }

        // For each segment, check if its midpoint lies inside both faces. Merge
        // adjacent accepted spans before returning them: periodic face tests can
        // contribute redundant transition parameters at a seam, but those must
        // not turn one continuous intersection into hundreds of split edges.
        let mut spans: Vec<(f64, f64)> = Vec::new();
        let mut accepted_span: Option<(f64, f64)> = None;
        for i in 0..unique_params.len() - 1 {
            let t1 = unique_params[i];
            let t2 = unique_params[i + 1];
            if (t2 - t1).abs() < tol {
                continue;
            }
            let t_mid = 0.5 * (t1 + t2);
            let p_mid = curve.point(t_mid);
            let (u1, v1) = uv_of(s1, &p_mid);
            let (u2, v2) = uv_of(s2, &p_mid);

            let in_face1 = is_inside_trimming_loops(u1, v1, face1)
                || point_on_face_boundary(&p_mid, face1, tol);
            let in_face2 = is_inside_trimming_loops(u2, v2, face2)
                || point_on_face_boundary(&p_mid, face2, tol);
            if in_face1 && in_face2 {
                if debug {
                    let a = curve.point(t1);
                    let b = curve.point(t2);
                    eprintln!(
                        "ssi cylinder kept ({:.4},{:.4},{:.4}) -> ({:.4},{:.4},{:.4})",
                        a.x(),
                        a.y(),
                        a.z(),
                        b.x(),
                        b.y(),
                        b.z()
                    );
                }
                match accepted_span.as_mut() {
                    Some((_, end)) if (t1 - *end).abs() <= tol.max(1.0e-8) => *end = t2,
                    Some(_) => {
                        spans.extend(accepted_span.replace((t1, t2)));
                    }
                    None => accepted_span = Some((t1, t2)),
                }
            } else if let Some(span) = accepted_span.take() {
                spans.push(span);
            }
        }
        spans.extend(accepted_span);
        // A periodic section (a circle where a plane crosses a cylinder) is
        // searched over one period starting at an arbitrary origin. When the
        // kept arc runs through that origin it arrives as two spans, each with
        // one end on no boundary; imprinting them leaves a chain through an
        // interior seam vertex that a face partition cannot use. Join them
        // into one span running past the period end.
        if spans.len() >= 2 && curve.is_periodic() {
            let period = curve.period();
            let wraps = (spans[0].0 - c_min).abs() <= tol.max(1.0e-8)
                && (spans[spans.len() - 1].1 - c_max).abs() <= tol.max(1.0e-8)
                && ((c_max - c_min) - period).abs() <= tol.max(1.0e-8);
            if wraps {
                let (_, head_end) = spans.remove(0);
                let tail = spans.last_mut().expect("two spans");
                tail.1 = head_end + period;
            }
        }
        trimmed_curves.extend(
            spans
                .into_iter()
                .map(|(first, last)| (curve.clone(), first, last)),
        );
    }

    Ok(trimmed_curves)
}

/// Inclusive boundary membership used while trimming intersection curves.
/// Point-in-polygon is deliberately strict, but an intersection that coincides
/// with one face's rim still has to be imprinted on the opposite face (the
/// cone-to-bore transition in a countersink is the canonical example).
fn point_on_face_boundary(point: &Pnt, face: &Face, tolerance: f64) -> bool {
    face.wires().iter().any(|wire| {
        wire.edges().iter().any(|edge| {
            let Some(curve) = edge.curve() else {
                return point.distance(&edge.start().point()) <= tolerance;
            };
            let parameter =
                crate::boolean::project_point_on_curve(point, curve, edge.first(), edge.last());
            curve.point(parameter).distance(point) <= tolerance.max(edge.tolerance())
        })
    })
}

#[allow(clippy::too_many_arguments)]
fn add_containment_transitions(
    curve: &GeomCurve,
    first: f64,
    last: f64,
    face: &Face,
    surface: &GeomSurface,
    split_params: &mut Vec<f64>,
    edge_crossings: &[f64],
    tol: f64,
) {
    let inside_at = |t: f64| {
        let p = curve.point(t);
        let (u, v) = uv_of(surface, &p);
        is_inside_trimming_loops(u, v, face)
    };

    let samples = 192usize;
    // A sampled transition lands where `is_inside_trimming_loops` flips, which sits
    // a boundary-margin *inside* the true crossing — so where the curve actually
    // crosses a boundary edge, the exact analytic crossing (`edge_crossings`) is
    // authoritative and the sampled one is a redundant, slightly-short duplicate.
    // Adding it would shrink the trimmed interval (a coplanar chord stopping short
    // of the cylinder rim → the face never splits → a non-manifold boss seam).
    // So skip any sampled transition within one sample step of an exact crossing.
    let snap = ((last - first).abs() / samples as f64).max(2.0 * tol);
    let near_exact = |t: f64| edge_crossings.iter().any(|&c| (c - t).abs() <= snap);

    let mut prev_t = first;
    let mut prev_inside = inside_at(prev_t);
    for i in 1..=samples {
        let t = first + (last - first) * (i as f64) / (samples as f64);
        let inside = inside_at(t);
        if inside != prev_inside {
            let mut lo = prev_t;
            let mut hi = t;
            let lo_inside = prev_inside;
            for _ in 0..32 {
                let mid = 0.5 * (lo + hi);
                if inside_at(mid) == lo_inside {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            let crossing = 0.5 * (lo + hi);
            if crossing > first + tol && crossing < last - tol && !near_exact(crossing) {
                split_params.push(crossing);
            }
        }
        prev_t = t;
        prev_inside = inside;
    }
}

/// Trim a curve segment to the interior of a face.
///
/// Intersects the curve segment with the face's boundary edges and returns
/// all parameter intervals on the curve that lie inside the face.
pub fn trim_curve_to_face(
    curve: &GeomCurve,
    first: f64,
    last: f64,
    face: &Face,
    tol: f64,
) -> Vec<(f64, f64)> {
    let surface = match face.surface() {
        Some(s) => s,
        None => return Vec::new(),
    };

    let first = if first.is_infinite() || first.is_nan() {
        -100.0
    } else {
        first
    };
    let last = if last.is_infinite() || last.is_nan() {
        100.0
    } else {
        last
    };

    let mut split_params = vec![first, last];

    // Exact analytic crossings of the curve with the face's boundary edges. These
    // are the authoritative trim boundaries.
    let mut edge_crossings = Vec::new();
    for wire in face.wires() {
        for edge in wire.edges() {
            if let Some(edge_curve) = edge.curve() {
                let pts = curve_curve(edge_curve, curve, tol);
                for pt in pts {
                    let t = crate::boolean::project_point_on_curve(&pt, curve, first, last);
                    if t > first + tol && t < last - tol {
                        edge_crossings.push(t);
                    }
                }
            }
        }
    }
    split_params.extend(edge_crossings.iter().copied());
    add_containment_transitions(
        curve,
        first,
        last,
        face,
        surface,
        &mut split_params,
        &edge_crossings,
        tol,
    );

    // Sort and deduplicate
    split_params.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let mut unique_params: Vec<f64> = Vec::new();
    for &t in &split_params {
        if unique_params.iter().all(|&u| (u - t).abs() > 1e-5) {
            unique_params.push(t);
        }
    }

    let mut intervals = Vec::new();
    for i in 0..unique_params.len() - 1 {
        let t1 = unique_params[i];
        let t2 = unique_params[i + 1];
        if (t2 - t1).abs() < tol {
            continue;
        }
        let t_mid = 0.5 * (t1 + t2);
        let p_mid = curve.point(t_mid);
        let (u, v) = uv_of(surface, &p_mid);

        if is_inside_trimming_loops(u, v, face) {
            intervals.push((t1, t2));
        }
    }

    intervals
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::{Ax1, Ax3, Dir, Trsf};
    use openrcad_geom::{Line, Plane};

    #[test]
    fn ordered_clamp_accepts_reversed_bounds() {
        assert_eq!(clamp_ordered(2.12, 2.14, 2.09), 2.12);
        assert_eq!(clamp_ordered(2.00, 2.14, 2.09), 2.09);
        assert_eq!(clamp_ordered(2.20, 2.14, 2.09), 2.14);
    }

    #[test]
    fn test_ray_face_intersection() {
        use openrcad_topo::{Edge, Face, Wire};
        // Square XY face [0, 10] x [0, 10] at Z = 0
        let p0 = Pnt::new(0.0, 0.0, 0.0);
        let p1 = Pnt::new(10.0, 0.0, 0.0);
        let p2 = Pnt::new(10.0, 10.0, 0.0);
        let p3 = Pnt::new(0.0, 10.0, 0.0);

        let w = Wire::from_edges([
            Edge::between_points(p0, p1),
            Edge::between_points(p1, p2),
            Edge::between_points(p2, p3),
            Edge::between_points(p3, p0),
        ]);

        let plane = GeomSurface::plane(Plane::from_point_normal(Pnt::origin(), Dir::dz()));
        let face = Face::new(Some(plane), w);

        // Ray passing through center (5, 5) from Z = 10 pointing down (-Z)
        let ray_org = Pnt::new(5.0, 5.0, 10.0);
        let ray_dir = GeomVec::new(0.0, 0.0, -1.0);

        let hit = ray_face(&ray_org, &ray_dir, &face, 1e-5);
        assert!(hit.is_some());
        assert_eq!(hit.unwrap(), Pnt::new(5.0, 5.0, 0.0));

        // Ray passing outside the square
        let ray_org_out = Pnt::new(12.0, 5.0, 10.0);
        let hit_out = ray_face(&ray_org_out, &ray_dir, &face, 1e-5);
        assert!(hit_out.is_none());
    }

    #[test]
    fn test_curve_curve_intersection() {
        let c1 = GeomCurve::Line(Line::new(Ax1::new(
            Pnt::new(0.0, 0.0, 0.0),
            Dir::new(1.0, 1.0, 0.0),
        )));
        let c2 = GeomCurve::Line(Line::new(Ax1::new(
            Pnt::new(2.0, 0.0, 0.0),
            Dir::new(-1.0, 1.0, 0.0),
        )));

        let pts = curve_curve(&c1, &c2, 1e-5);
        assert_eq!(pts.len(), 1);
        let expected = Pnt::new(1.0, 1.0, 0.0);
        assert!(pts[0].distance(&expected) < 1e-4);
    }

    #[test]
    fn line_ellipse_crossings_are_exact_far_along_the_line() {
        use openrcad_foundation::Ax3;
        let ellipse = GeomCurve::Ellipse(Ellipse::new(
            Ax3::new_axes(Pnt::new(2.0, -1.0, 9.0), Dir::dz(), Dir::new(0.8, 0.6, 0.0)),
            3.4,
            2.7,
        ));
        // A point on the ellipse, and a tilted line through it whose
        // parameter there is ~150: past the subdivision search's window.
        let target = ellipse.point(1.234);
        let direction = Dir::new(-0.49, 0.37, 0.79);
        let origin = target - GeomVec::from_dir(direction) * 150.0;
        let line = GeomCurve::Line(Line::new(Ax1::new(origin, direction)));
        let hits = curve_curve(&line, &ellipse, 1e-5);
        assert_eq!(hits.len(), 1, "{hits:?}");
        assert!(
            hits[0].distance(&target) < 1e-9,
            "{:?} vs {target:?}",
            hits[0]
        );

        // In the ellipse's plane: a chord through the centre meets it twice,
        // at the ends of the semi-axis it runs along.
        let chord = GeomCurve::Line(Line::new(Ax1::new(
            Pnt::new(2.0, -1.0, 9.0) - GeomVec::new(0.8, 0.6, 0.0) * 120.0,
            Dir::new(0.8, 0.6, 0.0),
        )));
        let hits = curve_curve(&chord, &ellipse, 1e-5);
        assert_eq!(hits.len(), 2, "{hits:?}");
        for hit in hits {
            assert!((hit.distance(&Pnt::new(2.0, -1.0, 9.0)) - 3.4).abs() < 1e-9);
        }
    }

    #[test]
    fn analytic_line_line_crossing_and_skew() {
        use openrcad_foundation::Ax3;
        use openrcad_geom::Circle;
        // Crossing lines in the z=0 plane meet at (1,1,0).
        let c1 = GeomCurve::Line(Line::new(Ax1::new(Pnt::origin(), Dir::new(1.0, 1.0, 0.0))));
        let c2 = GeomCurve::Line(Line::new(Ax1::new(
            Pnt::new(2.0, 0.0, 0.0),
            Dir::new(-1.0, 1.0, 0.0),
        )));
        let pts = curve_curve(&c1, &c2, 1e-6);
        assert_eq!(pts.len(), 1);
        assert!(pts[0].distance(&Pnt::new(1.0, 1.0, 0.0)) < 1e-9);

        // Skew lines (offset in z) never meet.
        let c3 = GeomCurve::Line(Line::new(Ax1::new(
            Pnt::new(2.0, 0.0, 1.0),
            Dir::new(-1.0, 1.0, 0.0),
        )));
        assert!(curve_curve(&c1, &c3, 1e-6).is_empty());

        // A line piercing a unit circle's plane on the rim hits exactly once.
        let circle = GeomCurve::Circle(Circle::new(Ax3::new(Pnt::origin(), Dir::dz()), 1.0));
        let vline = GeomCurve::Line(Line::new(Ax1::new(Pnt::new(1.0, 0.0, -3.0), Dir::dz())));
        let hit = curve_curve(&circle, &vline, 1e-6);
        assert_eq!(hit.len(), 1);
        assert!(hit[0].distance(&Pnt::new(1.0, 0.0, 0.0)) < 1e-9);
    }

    #[test]
    fn analytic_line_torus_quartic_holds_across_scale_rotation_and_origin() {
        let base = ToroidalSurface::new(Ax3::new(Pnt::origin(), Dir::dz()), 4.0, 1.0);
        for (label, transform) in [
            ("small", Trsf::scale(&Pnt::origin(), 1.0e-3)),
            (
                "rotated",
                Trsf::rotation(
                    &Ax1::new(Pnt::origin(), Dir::new(1.0, 2.0, 3.0)),
                    37.0_f64.to_radians(),
                ),
            ),
            (
                "far",
                Trsf::translation(GeomVec::new(1.0e9, -2.0e9, 3.0e9))
                    .multiply(&Trsf::scale(&Pnt::origin(), 1.0e3)),
            ),
        ] {
            let torus = base.transformed(&transform);
            let direction = GeomVec::from_dir(torus.position().x_direction());
            let origin = torus.position().location()
                - direction * (torus.major_radius() + torus.minor_radius()) * 2.0;
            let points = line_surface(&origin, &direction, &GeomSurface::torus(torus))
                .expect("torus has an analytic line intersection");
            assert_eq!(points.len(), 4, "{label}: {points:?}");
            let arithmetic_tolerance = points
                .iter()
                .flat_map(|point| [point.x().abs(), point.y().abs(), point.z().abs()])
                .fold(torus.major_radius() + torus.minor_radius(), f64::max)
                * f64::EPSILON
                * 512.0;
            for point in points {
                let (u, v) = uv_of(&GeomSurface::torus(torus), &point);
                assert!(
                    point.distance(&torus.point(u, v)) <= arithmetic_tolerance,
                    "quartic root left the torus at {point:?}"
                );
            }
        }

        // A global Cauchy-style dedup tolerance used to merge these four
        // crossings because the ray parameter is near 1e9 even though the
        // torus itself is unit scale. Keep the parameter origin independent of
        // the geometry origin so this numerical regime remains pinned.
        let distant =
            ToroidalSurface::new(Ax3::new(Pnt::new(1.0e9, 0.0, 0.0), Dir::dz()), 4.0, 1.0);
        let points = line_surface(
            &Pnt::origin(),
            &GeomVec::from_dir(Dir::dx()),
            &GeomSurface::torus(distant),
        )
        .expect("distant torus has an analytic line intersection");
        assert_eq!(points.len(), 4, "distant ray crossings: {points:?}");
    }

    #[test]
    fn analytic_circle_circle_coplanar() {
        use openrcad_foundation::Ax3;
        use openrcad_geom::Circle;
        // Two unit circles in z=0, centers 1 apart → two symmetric crossings at
        // x = 0.5, y = ±sqrt(1 − 0.25).
        let a = GeomCurve::Circle(Circle::new(Ax3::new(Pnt::origin(), Dir::dz()), 1.0));
        let b = GeomCurve::Circle(Circle::new(
            Ax3::new(Pnt::new(1.0, 0.0, 0.0), Dir::dz()),
            1.0,
        ));
        let pts = curve_curve(&a, &b, 1e-6);
        assert_eq!(pts.len(), 2);
        let yexp = (1.0f64 - 0.25).sqrt();
        for p in &pts {
            assert!((p.x() - 0.5).abs() < 1e-9);
            assert!((p.y().abs() - yexp).abs() < 1e-9);
            assert!(p.z().abs() < 1e-9);
        }
        // Far apart → no intersection.
        let c = GeomCurve::Circle(Circle::new(
            Ax3::new(Pnt::new(5.0, 0.0, 0.0), Dir::dz()),
            1.0,
        ));
        assert!(curve_curve(&a, &c, 1e-6).is_empty());
    }

    #[test]
    fn test_curve_surface_intersection() {
        let c = GeomCurve::Line(Line::new(Ax1::new(
            Pnt::new(1.0, 1.0, -1.0),
            Dir::new(0.0, 0.0, 1.0),
        )));
        let s = GeomSurface::Plane(Plane::from_point_normal(Pnt::origin(), Dir::dz()));

        let pts = curve_surface(&c, &s, 1e-5);
        assert_eq!(pts.len(), 1);
        let expected = Pnt::new(1.0, 1.0, 0.0);
        assert!(pts[0].distance(&expected) < 1e-4);
    }

    #[test]
    fn test_surface_surface_intersection() {
        // Plane Z = 0
        let s1 = GeomSurface::Plane(Plane::from_point_normal(
            Pnt::origin(),
            openrcad_foundation::Dir::dz(),
        ));
        // Plane Y = 0
        let s2 = GeomSurface::Plane(Plane::from_point_normal(
            Pnt::origin(),
            openrcad_foundation::Dir::dy(),
        ));

        let curves = surface_surface(&s1, &s2, 1e-3);
        assert!(!curves.is_empty());

        // The intersection curve should lie along the X-axis (Y = 0, Z = 0)
        let pt = curves[0].point(0.0);
        assert!(pt.y().abs() < 1e-2);
        assert!(pt.z().abs() < 1e-2);
    }

    #[test]
    fn perpendicular_plane_cone_is_exact_circle() {
        use openrcad_foundation::Ax3;

        let cone = GeomSurface::Cone(ConicalSurface::new(
            Ax3::new(Pnt::origin(), Dir::dz()),
            4.0,
            (-1.0_f64).atan(),
        ));
        let plane =
            GeomSurface::Plane(Plane::from_point_normal(Pnt::new(0.0, 0.0, 2.0), Dir::dz()));
        let curves = surface_surface(&plane, &cone, 1.0e-7);
        assert_eq!(curves.len(), 1);
        let GeomCurve::Circle(circle) = &curves[0] else {
            panic!("perpendicular plane/cone section must stay analytic");
        };
        assert!(circle.center().distance(&Pnt::new(0.0, 0.0, 2.0)) <= 1.0e-12);
        assert!((circle.radius() - 2.0).abs() <= 1.0e-12);
    }

    #[test]
    fn coaxial_cone_cylinder_intersections_are_exact_circles() {
        use openrcad_foundation::Ax3;

        let frame = Ax3::new(Pnt::origin(), Dir::dz());
        let cone = GeomSurface::Cone(ConicalSurface::new(frame, 4.0, (-1.0_f64).atan()));
        let cylinder = GeomSurface::Cylinder(CylindricalSurface::new(frame, 2.0));
        let curves = surface_surface(&cone, &cylinder, 1.0e-7);
        assert_eq!(
            curves.len(),
            2,
            "both sheets of the untrimmed cone intersect"
        );
        let mut heights = curves
            .iter()
            .map(|curve| match curve {
                GeomCurve::Circle(circle) => circle.center().z(),
                _ => panic!("coaxial cone/cylinder intersections must stay analytic"),
            })
            .collect::<Vec<_>>();
        heights.sort_by(f64::total_cmp);
        assert!((heights[0] - 2.0).abs() <= 1.0e-12);
        assert!((heights[1] - 6.0).abs() <= 1.0e-12);
    }

    /// A floor line from a single-precision wall (direction tilted 1e-8 out
    /// of the floor, offset 1.3e-7 from the hole rim's plane) still crosses
    /// the rim twice. Found by the ZeroCAD join/cut census (chains 373, 405):
    /// the tilt sent the line down the "pierces the plane far away" branch,
    /// the crossing was lost, and a slot's floor line overran into the hole.
    #[test]
    fn line_a_rounding_off_a_circles_plane_still_crosses_it() {
        use openrcad_foundation::{Ax3, Dir};
        let circle = Circle::new(
            Ax3::new_axes(Pnt::new(28.89, 15.53, 2.8445082957), Dir::dz(), Dir::dx()),
            5.88,
        );
        let direction = Dir::from_vec(&GeomVec::new(-1.0, 0.0, 1.0e-8)).unwrap();
        let line =
            openrcad_geom::Line::from_point_dir(Pnt::new(0.0, 11.255, 2.8445084295), direction);
        let hits = curve_curve(&GeomCurve::Circle(circle), &GeomCurve::Line(line), 1e-7);
        assert_eq!(hits.len(), 2, "{hits:?}");
        for hit in hits {
            let radial = (hit.x() - 28.89).hypot(hit.y() - 15.53);
            assert!((radial - 5.88).abs() < 1e-6, "off the rim: {hit:?}");
        }
    }

    /// A wall a rounding error off parallel to a hole's axis (found by the
    /// ZeroCAD join/cut census: a slot side 1.04e-7 rad off vertical) must
    /// still meet the hole in two straight lines on both surfaces, not an
    /// ellipse too long to trim.
    #[test]
    fn nearly_parallel_plane_cuts_a_cylinder_in_two_lines() {
        use openrcad_foundation::{Ax3, Dir};
        use openrcad_geom::CylindricalSurface;
        let r = 8.0;
        let cyl = GeomSurface::Cylinder(CylindricalSurface::new(
            Ax3::new(Pnt::new(7.0, 6.0, -2.0), Dir::dz()),
            r,
        ));
        for tilt in [1.04e-7, 6.0e-7] {
            let n = Dir::from_vec(&GeomVec::new(1.0, 0.0, tilt)).unwrap();
            let location = Pnt::new(2.5, -2.0, 4.3);
            let plane = GeomSurface::Plane(Plane::from_point_normal(location, n));
            let curves = surface_surface(&plane, &cyl, 1e-7);
            assert_eq!(curves.len(), 2, "tilt {tilt}: {curves:?}");
            for curve in &curves {
                assert!(matches!(curve, GeomCurve::Line(_)), "{curve:?}");
                // Sample 50 mm either side of the plane's station.
                let (origin, _) = curve.d1(0.0);
                for t in [-50.0, 0.0, 50.0] {
                    let p = curve.point(t - (origin.z() - location.z()));
                    let radial = ((p.x() - 7.0).powi(2) + (p.y() - 6.0).powi(2)).sqrt();
                    assert!(
                        (radial - r).abs() < 1e-9,
                        "tilt {tilt}: off cylinder by {}",
                        radial - r
                    );
                    let on_plane = GeomVec::from_dir(n).dot(&(p - location));
                    assert!(
                        on_plane.abs() < 1e-12,
                        "tilt {tilt}: off plane by {on_plane}"
                    );
                }
            }
        }
    }

    #[test]
    fn oblique_plane_cylinder_is_analytic_ellipse_on_both_surfaces() {
        use openrcad_foundation::{Ax3, Dir};
        use openrcad_geom::CylindricalSurface;
        // Radius-3 cylinder about the Z axis, cut by a 45°-tilted plane through
        // the origin → an exact ellipse (semi-minor 3, semi-major 3/cos45°).
        let r = 3.0;
        let cyl = GeomSurface::Cylinder(CylindricalSurface::new(
            Ax3::new(Pnt::origin(), Dir::dz()),
            r,
        ));
        let n = Dir::from_vec(&GeomVec::new(0.0, 1.0, 1.0)).unwrap();
        let plane = GeomSurface::Plane(Plane::from_point_normal(Pnt::origin(), n));

        let curves = surface_surface(&cyl, &plane, 1e-6);
        assert_eq!(curves.len(), 1, "oblique plane∩cylinder is one curve");
        assert!(
            matches!(curves[0], GeomCurve::Ellipse(_)),
            "must be analytic, got {:?}",
            curves[0]
        );
        // Every sampled point lies on BOTH the cylinder (radius r about Z) and the
        // plane — to far tighter than the old 160-sample B-spline could manage.
        for k in 0..64 {
            let u = 2.0 * PI * k as f64 / 64.0;
            let p = curves[0].point(u);
            let radial = (p.x() * p.x() + p.y() * p.y()).sqrt();
            assert!((radial - r).abs() < 1e-9, "u={u}: off cylinder, r={radial}");
            let on_plane = GeomVec::from_dir(n).dot(&(p - Pnt::origin()));
            assert!(on_plane.abs() < 1e-9, "u={u}: off plane by {on_plane}");
        }
    }

    #[test]
    fn point_in_polygon_robust_basic_and_sliver() {
        // Unit square.
        let square = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
        assert!(point_in_polygon_2d((0.5, 0.5), &square));
        assert!(!point_in_polygon_2d((1.5, 0.5), &square)); // right of it
        assert!(!point_in_polygon_2d((0.5, 1.5), &square)); // above it
        assert!(!point_in_polygon_2d((-0.5, 0.5), &square)); // left of it

        // A long thin sliver triangle whose top edge is *almost* horizontal —
        // the slope-division test divides by `pj.y - pi.y ≈ 1e-9` and is
        // ill-conditioned here; the orient2d formulation is division-free.
        let sliver = [(0.0, 0.0), (1000.0, 1e-9), (500.0, 1.0)];
        assert!(
            point_in_polygon_2d((500.0, 0.3), &sliver),
            "centroid-ish point should be inside the sliver"
        );
        assert!(
            !point_in_polygon_2d((500.0, -0.1), &sliver),
            "point below the near-horizontal base is outside"
        );
        assert!(
            !point_in_polygon_2d((1500.0, 0.3), &sliver),
            "point well to the right is outside"
        );
    }

    #[test]
    fn outer_support_membership_does_not_misclassify_an_inner_hole() {
        use openrcad_topo::{Edge, Face, Wire};

        let square = |min: f64, max: f64| {
            let points = [
                Pnt::new(min, min, 0.0),
                Pnt::new(max, min, 0.0),
                Pnt::new(max, max, 0.0),
                Pnt::new(min, max, 0.0),
            ];
            Wire::from_edges((0..4).map(|index| {
                Edge::between_points(points[index], points[(index + 1) % points.len()])
            }))
        };
        let face = Face::with_wires(
            Some(GeomSurface::plane(Plane::from_point_normal(
                Pnt::origin(),
                Dir::dz(),
            ))),
            Some(square(0.0, 4.0)),
            vec![square(1.0, 3.0)],
            Orientation::Forward,
        );
        let surface = face.surface().expect("planar test face");
        let (u, v) = uv_of(surface, &Pnt::new(2.0, 2.0, 0.0));

        assert!(is_inside_outer_trimming_loop(u, v, &face));
        assert!(!is_inside_trimming_loops(u, v, &face));
    }

    #[test]
    fn test_surface_surface_two_spheres() {
        // Two overlapping spheres: a non-analytic pair, so this exercises the
        // recursive interval-subdivision path (the analytic fast-path only
        // covers plane×plane and plane⊥cylinder).
        use openrcad_foundation::Ax3;
        use openrcad_geom::SphericalSurface;

        let s1 = GeomSurface::Sphere(SphericalSurface::new(
            Ax3::new(Pnt::origin(), Dir::dz()),
            2.0,
        ));
        let s2 = GeomSurface::Sphere(SphericalSurface::new(
            Ax3::new(Pnt::new(3.0, 0.0, 0.0), Dir::dz()),
            2.0,
        ));

        let curves = surface_surface(&s1, &s2, 1e-3);
        assert!(!curves.is_empty(), "expected an intersection curve");

        // The intersection is a circle in the plane x = 1.5, radius
        // sqrt(R² − (d/2)²) = sqrt(4 − 2.25) ≈ 1.323 about (1.5, 0, 0).
        let p = curves[0].point(0.0);
        assert!((p.x() - 1.5).abs() < 0.1, "x = {} (want ~1.5)", p.x());
        let d = p.distance(&Pnt::new(1.5, 0.0, 0.0));
        assert!(
            (d - 1.3229).abs() < 0.1,
            "circle radius = {} (want ~1.32)",
            d
        );
    }
}
