//! Parameter-space adaptive subdivision intersection solvers with Newton-Raphson refinement.
//! Respects local tolerances and handles curve-curve, curve-surface, and surface-surface cases.

use openrcad_foundation::{Ax3, Dir, Pnt, Vec as GeomVec};
use openrcad_geom::{BSplineCurve, Circle, Curve, GeomCurve, GeomSurface, Surface};
use openrcad_topo::{containment::point_in_polygon_2d, Face, Orientation};

use core::f64::consts::PI;

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
        GeomSurface::BSpline(_)
        | GeomSurface::Gregory(_)
        | GeomSurface::Offset(_)
        | GeomSurface::Ruled(_) => search_nearest_parameter_newton(s, p, (0.0, 0.0)),
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
        GeomSurface::Torus(_)
        | GeomSurface::BSpline(_)
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

/// Nearest parameter to `p` on `c` within its bounds: coarse sample sweep then a
/// few Newton steps. Used only by [`curves_overlap`].
fn nearest_param_on_curve(c: &GeomCurve, p: &Pnt) -> f64 {
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
        t = (t - dt).clamp(a, b);
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
    let p = l.location();
    let d = GeomVec::from_dir(l.direction());
    let c = ci.center();
    let n = GeomVec::from_dir(ci.axis());
    let r = ci.radius();
    let on_tol = tol.max(1e-7);
    let dn = d.dot(&n);
    if dn.abs() > 1e-12 {
        // Pierces the plane at one point; keep it only if it lands on the circle.
        let t = (c - p).dot(&n) / dn;
        let pt = p + d * t;
        if ((pt - c).magnitude() - r).abs() <= on_tol {
            return vec![pt];
        }
        return Vec::new();
    }
    // Parallel to the plane: no hit unless the line lies in the plane.
    if (p - c).dot(&n).abs() > on_tol {
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
        (GeomCurve::Circle(a), GeomCurve::Circle(b)) => Some(circle_circle(a, b, tol)),
        _ => None,
    }
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

    for _ in 0..10 {
        let (p1, v1) = c1.d1(t);
        let (p2, v2) = c2.d1(s);

        let b = p2 - p1;
        let a = v1.dot(&v1);
        let b_coeff = -v1.dot(&v2);
        let c = v2.dot(&v2);

        let det = a * c - b_coeff * b_coeff;
        if det.abs() < 1e-12 {
            // Tangent contact
            break;
        }

        let dt = (c * v1.dot(&b) - b_coeff * (-v2.dot(&b))) / det;
        let ds = (a * (-v2.dot(&b)) - b_coeff * v1.dot(&b)) / det;

        t = (t + dt).clamp(t1_min, t1_max);
        s = (s + ds).clamp(t2_min, t2_max);

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
            t = (t + dt).clamp(t_min, t_max);
            u = (u + du).clamp(u_min, u_max);
            v = (v + dv).clamp(v_min, v_max);

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
fn analytic_surface_surface(s1: &GeomSurface, s2: &GeomSurface) -> Option<Vec<GeomCurve>> {
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
    let (plane, cyl) = match (s1, s2) {
        (GeomSurface::Plane(p), GeomSurface::Cylinder(c)) => (p, c),
        (GeomSurface::Cylinder(c), GeomSurface::Plane(p)) => (p, c),
        _ => return None,
    };
    let axis = cyl.position();
    let w = GeomVec::from_dir(axis.direction());
    let n = GeomVec::from_dir(plane.normal());
    let align = w.dot(&n).abs();
    if (align - 1.0).abs() > 1e-7 {
        return None; // not perpendicular
    }
    // Where the axis crosses the plane.
    let denom = w.dot(&n);
    if denom.abs() < 1e-12 {
        return None;
    }
    let t = (plane.location() - axis.location()).dot(&n) / denom;
    let center = axis.location() + w * t;
    let frame = Ax3::new_axes(center, axis.direction(), axis.x_direction());
    Some(vec![GeomCurve::Circle(Circle::new(frame, cyl.radius()))])
}

/// Find intersection curves between two surfaces.
pub fn surface_surface(s1: &GeomSurface, s2: &GeomSurface, tol: f64) -> Vec<GeomCurve> {
    if let Some(curves) = analytic_surface_surface(s1, s2) {
        return curves;
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
    let tau = |a: f64, b: f64| ((b - a) / 24.0).max(1e-6);
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
    );

    let chain_tol = nearest_neighbour_chain_tol(&intersection_points, tol);

    let paths = chain_points(&intersection_points, chain_tol);

    let mut curves = Vec::new();
    for path in paths {
        if path.len() >= 2 {
            curves.push(GeomCurve::BSpline(polyline_to_bspline(&path)));
        }
    }

    curves
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
) {
    let mut b1 = s1
        .interval_point(u1_min, u1_max, v1_min, v1_max)
        .to_bndbox();
    b1.enlarge(tol);
    let mut b2 = s2
        .interval_point(u2_min, u2_max, v2_min, v2_max)
        .to_bndbox();
    b2.enlarge(tol);
    if b1.is_out_box(&b2) {
        return;
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
        return;
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
            );
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
            );
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
            );
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
            );
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
            );
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
            );
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
            );
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
            );
        }
    }
}

/// Chaining tolerance for the intersection-point polyline: ~2× the median
/// nearest-neighbour gap, so points on the same arc link up regardless of how
/// the parametrisation's scale maps to world distance.
fn nearest_neighbour_chain_tol(pts: &[Pnt], tol: f64) -> f64 {
    if pts.len() < 2 {
        return tol.max(1e-6);
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

            u1 = (u1 + du1).clamp(u1_min, u1_max);
            v1 = (v1 + dv1).clamp(v1_min, v1_max);
            u2 = (u2 - du2).clamp(u2_min, u2_max);
            v2 = (v2 - dv2).clamp(v2_min, v2_max);

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

    let mut u = hint.0.clamp(u_min, u_max);
    let mut v = hint.1.clamp(v_min, v_max);

    for _ in 0..10 {
        let (ps, su, sv) = eval_d1(s, u, v);
        let diff = ps - *p;
        if diff.magnitude() < 1e-12 {
            break;
        }

        let du = diff.dot(&su) / (su.dot(&su) + 1e-15);
        let dv = diff.dot(&sv) / (sv.dot(&sv) + 1e-15);

        u = (u - du).clamp(u_min, u_max);
        v = (v - dv).clamp(v_min, v_max);

        if du.abs() < 1e-9 && dv.abs() < 1e-9 {
            break;
        }
    }
    (u, v)
}

/// Sample the boundary `wire` of a face carried by `surface` into a polygon in
/// the surface's `(u, v)` parameter space. Each edge is sampled in its oriented
/// traversal direction; the angular coordinate is unwrapped so the polygon does
/// not jump across the `0 / 2π` seam.
fn loop_uv_polygon(surface: &GeomSurface, wire: &openrcad_topo::Wire) -> Vec<(f64, f64)> {
    let periodic = !matches!(surface, GeomSurface::Plane(_) | GeomSurface::BSpline(_));
    let samples = 6;
    let mut poly: Vec<(f64, f64)> = Vec::new();
    for edge in wire.edges() {
        let reversed = edge.orientation() == Orientation::Reversed;
        let (a, b) = (edge.first(), edge.last());
        for i in 0..samples {
            let frac = i as f64 / samples as f64;
            let t = if reversed {
                b + (a - b) * frac
            } else {
                a + (b - a) * frac
            };
            let p = match edge.curve() {
                Some(c) => c.point(t),
                None => edge.start().point(),
            };
            let (mut pu, pv) = uv_of(surface, &p);
            if periodic {
                if let Some(&(prev_u, _)) = poly.last() {
                    while pu - prev_u > PI {
                        pu -= 2.0 * PI;
                    }
                    while prev_u - pu > PI {
                        pu += 2.0 * PI;
                    }
                }
            }
            poly.push((pu, pv));
        }
    }
    poly
}

/// Bring the test angle `u` onto the same `2π` branch as `poly`'s angular range.
fn align_u(u: f64, poly: &[(f64, f64)]) -> f64 {
    if poly.is_empty() {
        return u;
    }
    let mean = poly.iter().map(|p| p.0).sum::<f64>() / poly.len() as f64;
    let mut x = u;
    while x - mean > PI {
        x -= 2.0 * PI;
    }
    while mean - x > PI {
        x += 2.0 * PI;
    }
    x
}

/// Checks if a parametric point `(u, v)` lies topologically inside the face's
/// trimming loops (inside the outer loop and outside every hole). Works for both
/// planar and analytic curved faces by testing in `(u, v)` parameter space.
pub fn is_inside_trimming_loops(u: f64, v: f64, face: &Face) -> bool {
    let surface = match face.surface() {
        Some(s) => s,
        None => return false,
    };
    let periodic = !matches!(surface, GeomSurface::Plane(_) | GeomSurface::BSpline(_));

    let outer_wire = match face.outer_wire() {
        Some(w) => w,
        None => return false,
    };
    let outer_poly = loop_uv_polygon(surface, &outer_wire);
    let tu = if periodic { align_u(u, &outer_poly) } else { u };
    if !point_in_polygon_2d((tu, v), &outer_poly) {
        return false;
    }

    for hole in face.inner_wires() {
        let hole_poly = loop_uv_polygon(surface, &hole);
        let hu = if periodic { align_u(u, &hole_poly) } else { u };
        let res = point_in_polygon_2d((hu, v), &hole_poly);
        if res {
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
pub fn surface_surface_curves(
    face1: &Face,
    face2: &Face,
    tol: f64,
) -> Vec<(GeomCurve, f64, f64)> {
    let s1 = match face1.surface() {
        Some(s) => s,
        None => return Vec::new(),
    };
    let s2 = match face2.surface() {
        Some(s) => s,
        None => return Vec::new(),
    };

    let raw_curves = surface_surface(s1, s2, tol);
    let mut trimmed_curves = Vec::new();

    for curve in raw_curves {
        let (c_min, c_max) = curve.bounds();
        let c_min = if c_min.is_infinite() || c_min.is_nan() { -100.0 } else { c_min };
        let c_max = if c_max.is_infinite() || c_max.is_nan() { 100.0 } else { c_max };

        let mut split_params = vec![c_min, c_max];

        // Find intersections between the intersection curve and the boundary edges of both faces
        let mut add_intersections = |face: &Face| {
            for wire in face.wires() {
                for edge in wire.edges() {
                    if let Some(edge_curve) = edge.curve() {
                        let pts = curve_curve(edge_curve, &curve, tol);
                        for pt in pts {
                            let t = crate::boolean::project_point_on_curve(&pt, &curve, c_min, c_max);
                            if t > c_min + tol && t < c_max - tol {
                                split_params.push(t);
                            }
                        }
                    }
                }
            }
        };

        add_intersections(face1);
        add_intersections(face2);

        // Sort and deduplicate split parameters
        split_params.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let mut unique_params: Vec<f64> = Vec::new();
        for &t in &split_params {
            if unique_params.iter().all(|&u| (u - t).abs() > 1e-5) {
                unique_params.push(t);
            }
        }

        // For each segment, check if its midpoint lies inside both faces
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

            if is_inside_trimming_loops(u1, v1, face1) && is_inside_trimming_loops(u2, v2, face2) {
                trimmed_curves.push((curve.clone(), t1, t2));
            }
        }
    }

    trimmed_curves
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

    let first = if first.is_infinite() || first.is_nan() { -100.0 } else { first };
    let last = if last.is_infinite() || last.is_nan() { 100.0 } else { last };

    let mut split_params = vec![first, last];

    for wire in face.wires() {
        for edge in wire.edges() {
            if let Some(edge_curve) = edge.curve() {
                let pts = curve_curve(edge_curve, curve, tol);
                for pt in pts {
                    let t = crate::boolean::project_point_on_curve(&pt, curve, first, last);
                    if t > first + tol && t < last - tol {
                        split_params.push(t);
                    }
                }
            }
        }
    }

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
    use openrcad_foundation::{Ax1, Dir};
    use openrcad_geom::{Line, Plane};

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
