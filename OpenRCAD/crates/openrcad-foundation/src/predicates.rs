//! Adaptive robust geometric predicates using float error bounds and DoubleDouble fallback.
//! Implements 2D and 3D orientation tests based on Shewchuk's robust predicates.

use crate::double_double::DoubleDouble;
use crate::pnt::{Pnt, Pnt2d};

const EPS: f64 = 1.1102230246251565e-16; // 2^-53 (machine epsilon for f64)

/// Robust 2D orientation test.
///
/// Returns:
/// - A positive value if `a`, `b`, and `c` form a counterclockwise turn.
/// - A negative value if they form a clockwise turn.
/// - Zero if they are exactly collinear.
pub fn orient2d(a: Pnt2d, b: Pnt2d, c: Pnt2d) -> f64 {
    let acx = a.x() - c.x();
    let bcy = b.y() - c.y();
    let acy = a.y() - c.y();
    let bcx = b.x() - c.x();

    let det = acx * bcy - acy * bcx;

    // Static error bound calculation (Shewchuk's orient2d bound)
    let err_bound = (3.0 + 16.0 * EPS) * EPS * ((acx * bcy).abs() + (acy * bcx).abs());

    if det.abs() > err_bound {
        det
    } else {
        // Fallback to exact DoubleDouble calculation
        let dd_acx = DoubleDouble::from_f64(a.x()) - DoubleDouble::from_f64(c.x());
        let dd_bcy = DoubleDouble::from_f64(b.y()) - DoubleDouble::from_f64(c.y());
        let dd_acy = DoubleDouble::from_f64(a.y()) - DoubleDouble::from_f64(c.y());
        let dd_bcx = DoubleDouble::from_f64(b.x()) - DoubleDouble::from_f64(c.x());

        let dd_det = (dd_acx * dd_bcy) - (dd_acy * dd_bcx);
        dd_det.to_f64()
    }
}

/// Robust 3D orientation test.
///
/// Returns:
/// - A positive value if `d` lies below the plane defined by `a`, `b`, and `c`
///   (when looking at the plane from the "positive" side defined by the right-hand rule
///   applied to `a -> b -> c`).
/// - A negative value if `d` lies above the plane.
/// - Zero if all four points are coplanar.
pub fn orient3d(a: Pnt, b: Pnt, c: Pnt, d: Pnt) -> f64 {
    let adx = a.x() - d.x();
    let ady = a.y() - d.y();
    let adz = a.z() - d.z();

    let bdx = pb_x_minus_pd_x(b, d);
    let bdy = pb_y_minus_pd_y(b, d);
    let bdz = pb_z_minus_pd_z(b, d);

    let cdx = pc_x_minus_pd_x(c, d);
    let cdy = pc_y_minus_pd_y(c, d);
    let cdz = pc_z_minus_pd_z(c, d);

    let t1 = bdy * cdz - bdz * cdy;
    let t2 = bdz * cdx - bdx * cdz;
    let t3 = bdx * cdy - bdy * cdx;

    let det = adx * t1 + ady * t2 + adz * t3;

    // Static error bound calculation (Shewchuk's orient3d bound)
    let max_bound = ((bdy * cdz).abs() + (bdz * cdy).abs()) * adx.abs()
        + ((bdz * cdx).abs() + (bdx * cdz).abs()) * ady.abs()
        + ((bdx * cdy).abs() + (bdy * cdx).abs()) * adz.abs();

    let err_bound = (7.0 + 56.0 * EPS) * EPS * max_bound;

    if det.abs() > err_bound {
        det
    } else {
        // Fallback to exact DoubleDouble calculation
        let dd_adx = DoubleDouble::from_f64(a.x()) - DoubleDouble::from_f64(d.x());
        let dd_ady = DoubleDouble::from_f64(a.y()) - DoubleDouble::from_f64(d.y());
        let dd_adz = DoubleDouble::from_f64(a.z()) - DoubleDouble::from_f64(d.z());

        let dd_bdx = DoubleDouble::from_f64(b.x()) - DoubleDouble::from_f64(d.x());
        let dd_bdy = DoubleDouble::from_f64(b.y()) - DoubleDouble::from_f64(d.y());
        let dd_bdz = DoubleDouble::from_f64(b.z()) - DoubleDouble::from_f64(d.z());

        let dd_cdx = DoubleDouble::from_f64(c.x()) - DoubleDouble::from_f64(d.x());
        let dd_cdy = DoubleDouble::from_f64(c.y()) - DoubleDouble::from_f64(d.y());
        let dd_cdz = DoubleDouble::from_f64(c.z()) - DoubleDouble::from_f64(d.z());

        let dd_t1 = (dd_bdy * dd_cdz) - (dd_bdz * dd_cdy);
        let dd_t2 = (dd_bdz * dd_cdx) - (dd_bdx * dd_cdz);
        let dd_t3 = (dd_bdx * dd_cdy) - (dd_bdy * dd_cdx);

        let dd_det = (dd_adx * dd_t1) + (dd_ady * dd_t2) + (dd_adz * dd_t3);
        dd_det.to_f64()
    }
}

#[inline(always)]
fn pb_x_minus_pd_x(b: Pnt, d: Pnt) -> f64 {
    b.x() - d.x()
}
#[inline(always)]
fn pb_y_minus_pd_y(b: Pnt, d: Pnt) -> f64 {
    b.y() - d.y()
}
#[inline(always)]
fn pb_z_minus_pd_z(b: Pnt, d: Pnt) -> f64 {
    b.z() - d.z()
}
#[inline(always)]
fn pc_x_minus_pd_x(c: Pnt, d: Pnt) -> f64 {
    c.x() - d.x()
}
#[inline(always)]
fn pc_y_minus_pd_y(c: Pnt, d: Pnt) -> f64 {
    c.y() - d.y()
}
#[inline(always)]
fn pc_z_minus_pd_z(c: Pnt, d: Pnt) -> f64 {
    c.z() - d.z()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_orient2d() {
        let a = Pnt2d::new(0.0, 0.0);
        let b = Pnt2d::new(2.0, 0.0);
        let c = Pnt2d::new(1.0, 1.0);
        let d = Pnt2d::new(1.0, -1.0);
        let e = Pnt2d::new(1.0, 0.0);

        // a -> b -> c is CCW (positive)
        assert!(orient2d(a, b, c) > 0.0);
        // a -> b -> d is CW (negative)
        assert!(orient2d(a, b, d) < 0.0);
        // a -> b -> e is collinear (zero)
        assert_eq!(orient2d(a, b, e), 0.0);

        // Test robustness with almost collinear points
        let pa = Pnt2d::new(0.5, 0.5);
        let pb = Pnt2d::new(12.0, 12.0);
        let pc = Pnt2d::new(24.0, 24.0);
        assert_eq!(orient2d(pa, pb, pc), 0.0);
    }

    #[test]
    fn test_orient3d() {
        let a = Pnt::new(0.0, 0.0, 0.0);
        let b = Pnt::new(1.0, 0.0, 0.0);
        let c = Pnt::new(0.0, 1.0, 0.0);
        let d = Pnt::new(0.0, 0.0, -1.0); // below plane (Z = 0)
        let e = Pnt::new(0.0, 0.0, 1.0); // above plane
        let f = Pnt::new(0.5, 0.5, 0.0); // on plane

        assert!(orient3d(a, b, c, d) > 0.0);
        assert!(orient3d(a, b, c, e) < 0.0);
        assert_eq!(orient3d(a, b, c, f), 0.0);
    }
}
