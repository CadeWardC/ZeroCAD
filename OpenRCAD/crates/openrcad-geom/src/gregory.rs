use crate::Surface;
use openrcad_foundation::{Pnt, Trsf, Vec};
use serde::{Deserialize, Serialize};

/// A 4-sided Gregory patch with G1 boundary derivatives resolved.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GregorySurface {
    /// 12 boundary control points and 8 internal control points (2 per corner)
    pub p00: Pnt,
    pub p01: Pnt,
    pub p02: Pnt,
    pub p03: Pnt,
    pub p10: Pnt,
    pub p20: Pnt,
    pub p30: Pnt,
    pub p31: Pnt,
    pub p32: Pnt,
    pub p33: Pnt,
    pub p13: Pnt,
    pub p23: Pnt,

    // Internal control points
    pub p11_u: Pnt,
    pub p11_v: Pnt,
    pub p21_u: Pnt,
    pub p21_v: Pnt,
    pub p12_u: Pnt,
    pub p12_v: Pnt,
    pub p22_u: Pnt,
    pub p22_v: Pnt,
}

impl GregorySurface {
    /// Create a new Gregory surface.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        p00: Pnt,
        p01: Pnt,
        p02: Pnt,
        p03: Pnt,
        p10: Pnt,
        p20: Pnt,
        p30: Pnt,
        p31: Pnt,
        p32: Pnt,
        p33: Pnt,
        p13: Pnt,
        p23: Pnt,
        p11_u: Pnt,
        p11_v: Pnt,
        p21_u: Pnt,
        p21_v: Pnt,
        p12_u: Pnt,
        p12_v: Pnt,
        p22_u: Pnt,
        p22_v: Pnt,
    ) -> Self {
        Self {
            p00,
            p01,
            p02,
            p03,
            p10,
            p20,
            p30,
            p31,
            p32,
            p33,
            p13,
            p23,
            p11_u,
            p11_v,
            p21_u,
            p21_v,
            p12_u,
            p12_v,
            p22_u,
            p22_v,
        }
    }
}

impl Surface for GregorySurface {
    fn point(&self, u: f64, v: f64) -> Pnt {
        let u = u.clamp(0.0, 1.0);
        let v = v.clamp(0.0, 1.0);

        // Compute the four blended internal control points
        let p11 = blend_point(u, v, self.p11_u, self.p11_v);
        let p21 = blend_point(1.0 - u, v, self.p21_u, self.p21_v);
        let p12 = blend_point(u, 1.0 - v, self.p12_u, self.p12_v);
        let p22 = blend_point(1.0 - u, 1.0 - v, self.p22_u, self.p22_v);

        // Control point grid (4x4)
        let poles = [
            [self.p00, self.p01, self.p02, self.p03],
            [self.p10, p11, p12, self.p13],
            [self.p20, p21, p22, self.p23],
            [self.p30, self.p31, self.p32, self.p33],
        ];

        // Evaluate bicubic Bezier surface point at (u, v)
        evaluate_bezier(&poles, u, v)
    }

    fn bounds(&self) -> (f64, f64, f64, f64) {
        (0.0, 1.0, 0.0, 1.0)
    }

    fn transformed(&self, t: &Trsf) -> Self {
        Self {
            p00: t.transform_point(&self.p00),
            p01: t.transform_point(&self.p01),
            p02: t.transform_point(&self.p02),
            p03: t.transform_point(&self.p03),
            p10: t.transform_point(&self.p10),
            p20: t.transform_point(&self.p20),
            p30: t.transform_point(&self.p30),
            p31: t.transform_point(&self.p31),
            p32: t.transform_point(&self.p32),
            p33: t.transform_point(&self.p33),
            p13: t.transform_point(&self.p13),
            p23: t.transform_point(&self.p23),
            p11_u: t.transform_point(&self.p11_u),
            p11_v: t.transform_point(&self.p11_v),
            p21_u: t.transform_point(&self.p21_u),
            p21_v: t.transform_point(&self.p21_v),
            p12_u: t.transform_point(&self.p12_u),
            p12_v: t.transform_point(&self.p12_v),
            p22_u: t.transform_point(&self.p22_u),
            p22_v: t.transform_point(&self.p22_v),
        }
    }
}

impl GregorySurface {
    /// Numerically evaluate first derivatives.
    pub fn d1(&self, u: f64, v: f64) -> (Pnt, Vec, Vec) {
        let pt = self.point(u, v);
        let h = 1e-5;

        // Clamp parameters so we don't evaluate outside the valid bounds [0, 1]
        let u_plus = f64::min(u + h, 1.0);
        let u_minus = f64::max(u - h, 0.0);
        let du = (self.point(u_plus, v) - self.point(u_minus, v)) / (u_plus - u_minus);

        let v_plus = f64::min(v + h, 1.0);
        let v_minus = f64::max(v - h, 0.0);
        let dv = (self.point(u, v_plus) - self.point(u, v_minus)) / (v_plus - v_minus);

        (pt, du, dv)
    }
}

fn blend_point(s: f64, t: f64, ps: Pnt, pt: Pnt) -> Pnt {
    let sum = s + t;
    if sum < 1e-12 {
        Pnt::from_xyz((ps.coord() + pt.coord()) * 0.5)
    } else {
        Pnt::from_xyz((pt.coord() * s + ps.coord() * t) / sum)
    }
}

fn bernstein(i: usize, t: f64) -> f64 {
    let mt = 1.0 - t;
    match i {
        0 => mt * mt * mt,
        1 => 3.0 * t * mt * mt,
        2 => 3.0 * t * t * mt,
        3 => t * t * t,
        _ => 0.0,
    }
}

fn evaluate_bezier(poles: &[[Pnt; 4]; 4], u: f64, v: f64) -> Pnt {
    let mut sum = openrcad_foundation::Xyz::new(0.0, 0.0, 0.0);
    for i in 0..4 {
        let bu = bernstein(i, u);
        for j in 0..4 {
            let bv = bernstein(j, v);
            sum += poles[i][j].coord() * (bu * bv);
        }
    }
    Pnt::from_xyz(sum)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gregory_surface_evaluation() {
        let p00 = Pnt::new(0.0, 0.0, 0.0);
        let p01 = Pnt::new(0.0, 1.0, 0.0);
        let p02 = Pnt::new(0.0, 2.0, 0.0);
        let p03 = Pnt::new(0.0, 3.0, 0.0);

        let p10 = Pnt::new(1.0, 0.0, 0.0);
        let p20 = Pnt::new(2.0, 0.0, 0.0);
        let p30 = Pnt::new(3.0, 0.0, 0.0);

        let p31 = Pnt::new(3.0, 1.0, 0.0);
        let p32 = Pnt::new(3.0, 2.0, 0.0);
        let p33 = Pnt::new(3.0, 3.0, 0.0);

        let p13 = Pnt::new(1.0, 3.0, 0.0);
        let p23 = Pnt::new(2.0, 3.0, 0.0);

        let p11_u = Pnt::new(1.0, 1.0, 0.0);
        let p11_v = Pnt::new(1.0, 1.0, 0.0);
        let p21_u = Pnt::new(2.0, 1.0, 0.0);
        let p21_v = Pnt::new(2.0, 1.0, 0.0);
        let p12_u = Pnt::new(1.0, 2.0, 0.0);
        let p12_v = Pnt::new(1.0, 2.0, 0.0);
        let p22_u = Pnt::new(2.0, 2.0, 0.0);
        let p22_v = Pnt::new(2.0, 2.0, 0.0);

        let surf = GregorySurface::new(
            p00, p01, p02, p03, p10, p20, p30, p31, p32, p33, p13, p23, p11_u, p11_v, p21_u, p21_v,
            p12_u, p12_v, p22_u, p22_v,
        );

        // Check corners
        assert_eq!(surf.point(0.0, 0.0), p00);
        assert_eq!(surf.point(1.0, 0.0), p30);
        assert_eq!(surf.point(0.0, 1.0), p03);
        assert_eq!(surf.point(1.0, 1.0), p33);

        // Check derivative evaluation
        let (pt, du, dv) = surf.d1(0.5, 0.5);
        assert!((pt.x() - 1.5).abs() < 1e-9);
        assert!((pt.y() - 1.5).abs() < 1e-9);
        assert!(du.x() > 0.0);
        assert!(dv.y() > 0.0);
    }
}
