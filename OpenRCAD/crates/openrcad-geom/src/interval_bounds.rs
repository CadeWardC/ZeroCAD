//! Guaranteed-enclosing interval bounds for curves and surfaces.
//!
//! [`GeomCurve::interval_point`] / [`GeomSurface::interval_point`] return an
//! [`Interval3`] that *contains* every point of the curve/surface patch over the
//! given parameter range. The intersection solvers consume these to prune their
//! adaptive subdivision rigorously — a bound can never drop a real intersection,
//! because it always encloses the true geometry.
//!
//! ## How each bound is computed
//!
//! - **Line / Plane**: linear in the parameters → exact via interval arithmetic.
//! - **Circle / Ellipse / Cylinder / Cone / Sphere**: the rotational term is
//!   `a·cos u + b·sin u` per world axis, whose exact range is found via
//!   [`cos_sin_range`] (the only extrema are the peak/trough at `atan2(b, a)`).
//!   For cylinder this is exact; cone/sphere couple the radius/latitude with the
//!   axial parameter `v`, which is bounded with interval arithmetic (valid,
//!   slightly conservative).
//! - **Parabola / Hyperbola**: exact monotonic/convex ranges of `u²`, `cosh u`,
//!   `sinh u` combined with interval arithmetic.
//! - **BSpline**: convex hull of the active control poles (valid for NURBS too —
//!   positive weights make the point a convex combination of its poles).

use core::f64::consts::PI;

use openrcad_foundation::{tolerance, Ax3, Interval, Interval3, Pnt};

use crate::{GeomCurve, GeomSurface, Hyperbola, Parabola};

const TAU: f64 = 2.0 * PI;

/// Sum `base + a` as intervals (base is a point value).
#[inline]
fn sum1(base: f64, a: Interval) -> Interval {
    Interval::point(base).add(&a)
}

/// Sum `base + a + b` as intervals (base is a point value).
#[inline]
fn sum2(base: f64, a: Interval, b: Interval) -> Interval {
    Interval::point(base).add(&a).add(&b)
}

/// Is there an integer `k` with `u0 <= theta + TAU·k <= u1`?
fn angle_in_range(theta: f64, u0: f64, u1: f64) -> bool {
    if u1 - u0 >= TAU {
        return true; // the range spans a full period → every angle is in it
    }
    // Shift theta into [u0, u0 + TAU); it is in [u0, u1] iff it is <= u1.
    let t = (theta - u0).rem_euclid(TAU) + u0;
    t <= u1
}

/// Exact range of `a·cos u + b·sin u` over `u ∈ [u0, u1]`.
///
/// Written `A·cos(u − φ)` with `A = hypot(a, b)`, `φ = atan2(b, a)`, the only
/// extrema are the peak `+A` at `u = φ` and the trough `−A` at `u = φ + π`.
fn cos_sin_range(a: f64, b: f64, u0: f64, u1: f64) -> (f64, f64) {
    let f0 = a * u0.cos() + b * u0.sin();
    let f1 = a * u1.cos() + b * u1.sin();
    let mut lo = f0.min(f1);
    let mut hi = f0.max(f1);
    let amp = a.hypot(b);
    if amp == 0.0 {
        return (lo, hi);
    }
    let phi = b.atan2(a);
    if angle_in_range(phi, u0, u1) {
        lo = lo.min(amp);
        hi = hi.max(amp);
    }
    if angle_in_range(phi + PI, u0, u1) {
        lo = lo.min(-amp);
        hi = hi.max(-amp);
    }
    (lo, hi)
}

/// Exact range of `cosh u` over `[u0, u1]`: even, convex, minimum `1` at `0`.
fn cosh_range(u0: f64, u1: f64) -> (f64, f64) {
    let c0 = u0.cosh();
    let c1 = u1.cosh();
    let mut lo = c0.min(c1);
    let hi = c0.max(c1);
    if u0 <= 0.0 && u1 >= 0.0 {
        lo = lo.min(1.0);
    }
    (lo, hi)
}

/// Exact range of `sinh u` over `[u0, u1]`: strictly increasing.
fn sinh_range(u0: f64, u1: f64) -> (f64, f64) {
    (u0.sinh(), u1.sinh())
}

impl GeomCurve {
    /// A guaranteed-enclosing axis-aligned box of the curve over `[t0, t1]`.
    pub fn interval_point(&self, t0: f64, t1: f64) -> Interval3 {
        let (t0, t1) = if t0 <= t1 { (t0, t1) } else { (t1, t0) };
        match self {
            GeomCurve::Line(l) => {
                let o = l.location();
                let d = l.direction();
                let t_iv = Interval::new(t0, t1);
                Interval3::new(
                    sum1(o.x(), t_iv.scale(d.x())),
                    sum1(o.y(), t_iv.scale(d.y())),
                    sum1(o.z(), t_iv.scale(d.z())),
                )
            }
            GeomCurve::Circle(c) => {
                arc_box(c.center(), c.radius(), c.radius(), c.position(), t0, t1)
            }
            GeomCurve::Ellipse(e) => arc_box(
                e.center(),
                e.major_radius(),
                e.minor_radius(),
                e.position(),
                t0,
                t1,
            ),
            GeomCurve::Parabola(p) => parabola_box(p, t0, t1),
            GeomCurve::Hyperbola(h) => hyperbola_box(h, t0, t1),
            GeomCurve::BSpline(b) => b.interval_bbox(t0, t1),
        }
    }
}

/// Box of `center + r_x·cos u·X + r_y·sin u·Y` over `u ∈ [t0, t1]` (circle when
/// `r_x == r_y`, ellipse otherwise). Exact per world axis.
fn arc_box(center: Pnt, r_x: f64, r_y: f64, pos: Ax3, u0: f64, u1: f64) -> Interval3 {
    let x = pos.x_direction();
    let y = pos.y_direction();
    let (cx_lo, cx_hi) = cos_sin_range(r_x * x.x(), r_y * y.x(), u0, u1);
    let (cy_lo, cy_hi) = cos_sin_range(r_x * x.y(), r_y * y.y(), u0, u1);
    let (cz_lo, cz_hi) = cos_sin_range(r_x * x.z(), r_y * y.z(), u0, u1);
    Interval3::new(
        sum1(center.x(), Interval::new(cx_lo, cx_hi)),
        sum1(center.y(), Interval::new(cy_lo, cy_hi)),
        sum1(center.z(), Interval::new(cz_lo, cz_hi)),
    )
}

/// Box of a parabola `apex + (u²/4f)·X + u·Y` (degenerate to a line when `f ≈ 0`).
fn parabola_box(p: &Parabola, u0: f64, u1: f64) -> Interval3 {
    let apex = p.apex();
    let pos = p.position();
    let x = pos.x_direction();
    let y = pos.y_direction();
    if p.focal() <= tolerance::CONFUSION {
        // Degenerate: a line `apex + u·X`.
        let u_iv = Interval::new(u0, u1);
        return Interval3::new(
            sum1(apex.x(), u_iv.scale(x.x())),
            sum1(apex.y(), u_iv.scale(x.y())),
            sum1(apex.z(), u_iv.scale(x.z())),
        );
    }
    let inv4f = 1.0 / (4.0 * p.focal());
    let u_iv = Interval::new(u0, u1);
    let usqr = u_iv.sqr();
    Interval3::new(
        sum2(apex.x(), usqr.scale(x.x() * inv4f), u_iv.scale(y.x())),
        sum2(apex.y(), usqr.scale(x.y() * inv4f), u_iv.scale(y.y())),
        sum2(apex.z(), usqr.scale(x.z() * inv4f), u_iv.scale(y.z())),
    )
}

/// Box of a hyperbola `center + a·cosh u·X + b·sinh u·Y`. Exact.
fn hyperbola_box(h: &Hyperbola, u0: f64, u1: f64) -> Interval3 {
    let center = h.center();
    let pos = h.position();
    let x = pos.x_direction();
    let y = pos.y_direction();
    let a = h.major_radius();
    let b = h.minor_radius();
    let (ch_lo, ch_hi) = cosh_range(u0, u1);
    let (sh_lo, sh_hi) = sinh_range(u0, u1);
    let cosh_iv = Interval::new(ch_lo, ch_hi);
    let sinh_iv = Interval::new(sh_lo, sh_hi);
    Interval3::new(
        sum2(
            center.x(),
            cosh_iv.scale(a * x.x()),
            sinh_iv.scale(b * y.x()),
        ),
        sum2(
            center.y(),
            cosh_iv.scale(a * x.y()),
            sinh_iv.scale(b * y.y()),
        ),
        sum2(
            center.z(),
            cosh_iv.scale(a * x.z()),
            sinh_iv.scale(b * y.z()),
        ),
    )
}

impl GeomSurface {
    /// A guaranteed-enclosing axis-aligned box of the surface over
    /// `[u0, u1] × [v0, v1]`.
    pub fn interval_point(&self, u0: f64, u1: f64, v0: f64, v1: f64) -> Interval3 {
        let (u0, u1) = if u0 <= u1 { (u0, u1) } else { (u1, u0) };
        let (v0, v1) = if v0 <= v1 { (v0, v1) } else { (v1, v0) };
        match self {
            GeomSurface::Plane(p) => plane_box(p.position(), u0, u1, v0, v1),
            GeomSurface::Cylinder(c) => cylinder_box(c.position(), c.radius(), u0, u1, v0, v1),
            GeomSurface::Cone(c) => {
                cone_box(c.position(), c.ref_radius(), c.semi_angle(), u0, u1, v0, v1)
            }
            GeomSurface::Sphere(s) => sphere_box(s.position(), s.radius(), u0, u1, v0, v1),
            GeomSurface::Torus(t) => {
                // The whole torus lies within centre ± (R + r) on every world axis
                // (the farthest surface point is R + r from the centre). Conservative
                // but orientation-independent and guaranteed enclosing.
                let c = t.position().location();
                let reach = t.major_radius() + t.minor_radius();
                Interval3::new(
                    Interval::new(c.x() - reach, c.x() + reach),
                    Interval::new(c.y() - reach, c.y() + reach),
                    Interval::new(c.z() - reach, c.z() + reach),
                )
            }
            GeomSurface::BSpline(b) => b.interval_bbox(u0, u1, v0, v1),
            GeomSurface::Gregory(g) => {
                let mut min_x = f64::INFINITY;
                let mut max_x = f64::NEG_INFINITY;
                let mut min_y = f64::INFINITY;
                let mut max_y = f64::NEG_INFINITY;
                let mut min_z = f64::INFINITY;
                let mut max_z = f64::NEG_INFINITY;

                let pts = [
                    g.p00, g.p01, g.p02, g.p03, g.p10, g.p20, g.p30, g.p31, g.p32, g.p33, g.p13,
                    g.p23, g.p11_u, g.p11_v, g.p21_u, g.p21_v, g.p12_u, g.p12_v, g.p22_u, g.p22_v,
                ];
                for p in &pts {
                    min_x = min_x.min(p.x());
                    max_x = max_x.max(p.x());
                    min_y = min_y.min(p.y());
                    max_y = max_y.max(p.y());
                    min_z = min_z.min(p.z());
                    max_z = max_z.max(p.z());
                }
                Interval3::new(
                    Interval::new(min_x, max_x),
                    Interval::new(min_y, max_y),
                    Interval::new(min_z, max_z),
                )
            }
            GeomSurface::Offset(o) => {
                let base_box = o.base.interval_point(u0, u1, v0, v1);
                let r = o.distance.abs();
                Interval3::new(
                    Interval::new(base_box.x.lo - r, base_box.x.hi + r),
                    Interval::new(base_box.y.lo - r, base_box.y.hi + r),
                    Interval::new(base_box.z.lo - r, base_box.z.hi + r),
                )
            }
            GeomSurface::Ruled(r) => {
                let c1_box = r.curve1.interval_point(u0, u1);
                let c2_box = r.curve2.interval_point(u0, u1);
                let v_iv = Interval::new(v0, v1);
                let one_minus_v = Interval::point(1.0).sub(&v_iv);

                Interval3::new(
                    one_minus_v.mul(&c1_box.x).add(&v_iv.mul(&c2_box.x)),
                    one_minus_v.mul(&c1_box.y).add(&v_iv.mul(&c2_box.y)),
                    one_minus_v.mul(&c1_box.z).add(&v_iv.mul(&c2_box.z)),
                )
            }
        }
    }
}

/// Box of a plane `location + u·X + v·Y`. Exact.
fn plane_box(pos: Ax3, u0: f64, u1: f64, v0: f64, v1: f64) -> Interval3 {
    let loc = pos.location();
    let x = pos.x_direction();
    let y = pos.y_direction();
    let u_iv = Interval::new(u0, u1);
    let v_iv = Interval::new(v0, v1);
    Interval3::new(
        sum2(loc.x(), u_iv.scale(x.x()), v_iv.scale(y.x())),
        sum2(loc.y(), u_iv.scale(x.y()), v_iv.scale(y.y())),
        sum2(loc.z(), u_iv.scale(x.z()), v_iv.scale(y.z())),
    )
}

/// Box of a cylinder `location + R·(cos u·X + sin u·Y) + v·Z`. Exact in `u` and
/// `v` independently; the two terms are summed per axis.
fn cylinder_box(pos: Ax3, r: f64, u0: f64, u1: f64, v0: f64, v1: f64) -> Interval3 {
    let loc = pos.location();
    let x = pos.x_direction();
    let y = pos.y_direction();
    let z = pos.direction();
    let v_iv = Interval::new(v0, v1);
    let (cx_lo, cx_hi) = cos_sin_range(r * x.x(), r * y.x(), u0, u1);
    let (cy_lo, cy_hi) = cos_sin_range(r * x.y(), r * y.y(), u0, u1);
    let (cz_lo, cz_hi) = cos_sin_range(r * x.z(), r * y.z(), u0, u1);
    Interval3::new(
        sum2(loc.x(), Interval::new(cx_lo, cx_hi), v_iv.scale(z.x())),
        sum2(loc.y(), Interval::new(cy_lo, cy_hi), v_iv.scale(z.y())),
        sum2(loc.z(), Interval::new(cz_lo, cz_hi), v_iv.scale(z.z())),
    )
}

/// Box of a cone `location + r(v)·(cos u·X + sin u·Y) + v·Z` with
/// `r(v) = ref + v·tan α`. Exact in the `(u)` arc and the `r(v)`·arc product;
/// the `v·Z` term couples with `r(v)`, so it is added with interval arithmetic
/// (valid, slightly conservative).
fn cone_box(
    pos: Ax3,
    ref_r: f64,
    semi_angle: f64,
    u0: f64,
    u1: f64,
    v0: f64,
    v1: f64,
) -> Interval3 {
    let loc = pos.location();
    let x = pos.x_direction();
    let y = pos.y_direction();
    let z = pos.direction();
    let slope = semi_angle.tan();
    let v_iv = Interval::new(v0, v1);
    // r(v) is linear in v → its range is the interval of the two endpoints.
    let r_iv = Interval::new(ref_r + v0 * slope, ref_r + v1 * slope);
    Interval3::new(
        cone_axis(loc.x(), r_iv, x.x(), y.x(), u0, u1, v_iv.scale(z.x())),
        cone_axis(loc.y(), r_iv, x.y(), y.y(), u0, u1, v_iv.scale(z.y())),
        cone_axis(loc.z(), r_iv, x.z(), y.z(), u0, u1, v_iv.scale(z.z())),
    )
}

/// One axis of the cone: `loc + r_iv · range(cos u·Xk + sin u·Yk) + vterm`.
#[inline]
fn cone_axis(
    loc: f64,
    r_iv: Interval,
    xk: f64,
    yk: f64,
    u0: f64,
    u1: f64,
    vterm: Interval,
) -> Interval {
    let (c_lo, c_hi) = cos_sin_range(xk, yk, u0, u1);
    let arc = r_iv.mul(&Interval::new(c_lo, c_hi));
    sum2(loc, arc, vterm)
}

/// Box of a sphere `center + R·cos v·(cos u·X + sin u·Y) + R·sin v·Z`. The
/// latitude `v` couples `cos v` with the longitude arc and `sin v` with the
/// axis term; both are bounded with interval arithmetic (valid).
fn sphere_box(pos: Ax3, r: f64, u0: f64, u1: f64, v0: f64, v1: f64) -> Interval3 {
    let center = pos.location();
    let x = pos.x_direction();
    let y = pos.y_direction();
    let z = pos.direction();
    // range of cos v and sin v over the latitude interval (exact).
    let (cv_lo, cv_hi) = cos_sin_range(1.0, 0.0, v0, v1);
    let (sv_lo, sv_hi) = cos_sin_range(0.0, 1.0, v0, v1);
    let cv = Interval::new(cv_lo, cv_hi).scale(r);
    let sv = Interval::new(sv_lo, sv_hi).scale(r);
    Interval3::new(
        sphere_axis(center.x(), cv, sv, x.x(), y.x(), z.x(), u0, u1),
        sphere_axis(center.y(), cv, sv, x.y(), y.y(), z.y(), u0, u1),
        sphere_axis(center.z(), cv, sv, x.z(), y.z(), z.z(), u0, u1),
    )
}

/// One axis of the sphere: `center + cv·range(cos u·Xk + sin u·Yk) + sv·Zk`.
#[inline]
#[allow(clippy::too_many_arguments)] // one scalar per basis component of the axis formula
fn sphere_axis(
    center: f64,
    cv: Interval,
    sv: Interval,
    xk: f64,
    yk: f64,
    zk: f64,
    u0: f64,
    u1: f64,
) -> Interval {
    let (c_lo, c_hi) = cos_sin_range(xk, yk, u0, u1);
    let arc = cv.mul(&Interval::new(c_lo, c_hi));
    sum2(center, arc, sv.scale(zk))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::curve::Curve;
    use crate::surface::Surface;
    use crate::{
        Circle, ConicalSurface, CylindricalSurface, Ellipse, Line, Plane, SphericalSurface,
    };
    use openrcad_foundation::{Dir, Pnt};

    /// Assert every sampled point of `c` over `[t0, t1]` lies in its interval box.
    fn encloses_curve(c: &GeomCurve, t0: f64, t1: f64) {
        let b = c.interval_point(t0, t1);
        for i in 0..64 {
            let t = t0 + (t1 - t0) * (i as f64) / 64.0;
            let p = c.point(t);
            assert!(
                b.x.contains(p.x()),
                "x {} in [{}, {}]",
                p.x(),
                b.x.lo,
                b.x.hi
            );
            assert!(
                b.y.contains(p.y()),
                "y {} in [{}, {}]",
                p.y(),
                b.y.lo,
                b.y.hi
            );
            assert!(
                b.z.contains(p.z()),
                "z {} in [{}, {}]",
                p.z(),
                b.z.lo,
                b.z.hi
            );
        }
    }

    fn encloses_surface(s: &GeomSurface, u0: f64, u1: f64, v0: f64, v1: f64) {
        let b = s.interval_point(u0, u1, v0, v1);
        for i in 0..32 {
            let u = u0 + (u1 - u0) * (i as f64) / 32.0;
            for j in 0..32 {
                let v = v0 + (v1 - v0) * (j as f64) / 32.0;
                let p = s.point(u, v);
                assert!(
                    b.x.contains(p.x()),
                    "x {} in [{}, {}]",
                    p.x(),
                    b.x.lo,
                    b.x.hi
                );
                assert!(
                    b.y.contains(p.y()),
                    "y {} in [{}, {}]",
                    p.y(),
                    b.y.lo,
                    b.y.hi
                );
                assert!(
                    b.z.contains(p.z()),
                    "z {} in [{}, {}]",
                    p.z(),
                    b.z.lo,
                    b.z.hi
                );
            }
        }
    }

    #[test]
    fn circle_and_line_enclose() {
        encloses_curve(
            &GeomCurve::Circle(Circle::new(
                Ax3::new(Pnt::new(1.0, 2.0, 3.0), Dir::dz()),
                2.0,
            )),
            0.0,
            1.3,
        );
        encloses_curve(
            &GeomCurve::Circle(Circle::new(Ax3::new(Pnt::origin(), Dir::dz()), 2.0)),
            0.0,
            TAU,
        );
        encloses_curve(
            &GeomCurve::Line(Line::from_point_dir(Pnt::origin(), Dir::new(1.0, 2.0, 3.0))),
            -2.0,
            5.0,
        );
    }

    #[test]
    fn ellipse_encloses() {
        encloses_curve(
            &GeomCurve::Ellipse(Ellipse::new(Ax3::new(Pnt::origin(), Dir::dz()), 4.0, 2.0)),
            0.5,
            2.8,
        );
    }

    #[test]
    fn parabola_hyperbola_enclose() {
        encloses_curve(
            &GeomCurve::Parabola(Parabola::new(Ax3::new(Pnt::origin(), Dir::dz()), 2.0)),
            -3.0,
            3.0,
        );
        encloses_curve(
            &GeomCurve::Hyperbola(Hyperbola::new(Ax3::new(Pnt::origin(), Dir::dz()), 3.0, 4.0)),
            -1.5,
            2.0,
        );
    }

    #[test]
    fn analytic_surfaces_enclose() {
        encloses_surface(
            &GeomSurface::Plane(Plane::from_point_normal(Pnt::origin(), Dir::dz())),
            -2.0,
            3.0,
            -1.0,
            4.0,
        );
        encloses_surface(
            &GeomSurface::Cylinder(CylindricalSurface::new(
                Ax3::new(Pnt::origin(), Dir::dz()),
                2.0,
            )),
            0.0,
            1.7,
            -3.0,
            3.0,
        );
        // A 45° cone narrowing upward.
        let alpha = (-1.0_f64).atan();
        encloses_surface(
            &GeomSurface::Cone(ConicalSurface::new(
                Ax3::new(Pnt::origin(), Dir::dz()),
                4.0,
                alpha,
            )),
            0.0,
            2.0,
            0.0,
            3.5,
        );
        encloses_surface(
            &GeomSurface::Sphere(SphericalSurface::new(
                Ax3::new(Pnt::origin(), Dir::dz()),
                3.0,
            )),
            0.0,
            2.3,
            -1.0,
            1.0,
        );
    }

    #[test]
    fn full_sphere_box_is_r_cubed() {
        // Full longitude + latitude: the sphere box is [-R, R]^3 about the center.
        let s = GeomSurface::Sphere(SphericalSurface::new(
            Ax3::new(Pnt::origin(), Dir::dz()),
            3.0,
        ));
        let b = s.interval_point(0.0, TAU, -PI / 2.0, PI / 2.0);
        for iv in [b.x, b.y, b.z] {
            assert!(iv.lo <= -3.0 + 1e-9, "lo {}", iv.lo);
            assert!(iv.hi >= 3.0 - 1e-9, "hi {}", iv.hi);
            // Should not be wildly wider than the exact [-3, 3].
            assert!(iv.lo > -3.5 && iv.hi < 3.5, "iv [{}, {}]", iv.lo, iv.hi);
        }
    }
}
