use crate::{GeomSurface, Surface};
use openrcad_foundation::{Pnt, Trsf, Vec};
use serde::{Deserialize, Serialize};

/// An offset surface shifted along the normal vector of a base surface.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct OffsetSurface {
    pub base: Box<GeomSurface>,
    pub distance: f64,
}

impl OffsetSurface {
    /// Create a new offset surface.
    pub fn new(base: GeomSurface, distance: f64) -> Self {
        Self {
            base: Box::new(base),
            distance,
        }
    }
}

impl Surface for OffsetSurface {
    fn point(&self, u: f64, v: f64) -> Pnt {
        let (pt, du, dv) = self.base.d1(u, v);
        let normal = du.cross(&dv);
        let normal_dir = normal.normalized().unwrap_or_else(|| {
            // Degenerate normal: perturb parameters slightly
            let h = 1e-5;
            let (_, du2, dv2) = self.base.d1(u + h, v + h);
            du2.cross(&dv2)
                .normalized()
                .unwrap_or(openrcad_foundation::Dir::dz())
        });

        pt + Vec::from_dir(normal_dir) * self.distance
    }

    fn bounds(&self) -> (f64, f64, f64, f64) {
        self.base.bounds()
    }

    fn transformed(&self, t: &Trsf) -> Self {
        Self {
            base: Box::new(self.base.transformed(t)),
            distance: self.distance,
        }
    }
}

impl OffsetSurface {
    /// Analytical/numerical derivatives.
    pub fn d1(&self, u: f64, v: f64) -> (Pnt, Vec, Vec) {
        let pt = self.point(u, v);
        let h = 1e-5;

        // Use central differences for offset surface derivatives
        let (u_min, u_max, v_min, v_max) = self.bounds();

        let u_plus = if u_max.is_infinite() {
            u + h
        } else {
            f64::min(u + h, u_max)
        };
        let u_minus = if u_min.is_infinite() {
            u - h
        } else {
            f64::max(u - h, u_min)
        };
        let du = (self.point(u_plus, v) - self.point(u_minus, v)) / (u_plus - u_minus);

        let v_plus = if v_max.is_infinite() {
            v + h
        } else {
            f64::min(v + h, v_max)
        };
        let v_minus = if v_min.is_infinite() {
            v - h
        } else {
            f64::max(v - h, v_min)
        };
        let dv = (self.point(u, v_plus) - self.point(u, v_minus)) / (v_plus - v_minus);

        (pt, du, dv)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Plane;
    use openrcad_foundation::Dir;

    #[test]
    fn test_offset_surface_evaluation() {
        let base_plane = GeomSurface::plane(Plane::from_point_normal(Pnt::origin(), Dir::dz()));
        let offset_surf = OffsetSurface::new(base_plane, 5.0);

        let p = offset_surf.point(0.0, 0.0);
        assert!(p.x().abs() < 1e-9);
        assert!(p.y().abs() < 1e-9);
        assert!((p.z() - 5.0).abs() < 1e-9);

        let p2 = offset_surf.point(3.0, 4.0);
        assert!((p2.x() - 3.0).abs() < 1e-9);
        assert!((p2.y() - 4.0).abs() < 1e-9);
        assert!((p2.z() - 5.0).abs() < 1e-9);

        let (pt, du, dv) = offset_surf.d1(3.0, 4.0);
        assert!((pt.z() - 5.0).abs() < 1e-9);
        assert!(du.x() > 0.9);
        assert!(dv.y() > 0.9);
    }
}
