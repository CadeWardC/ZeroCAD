//! Transforms — [`Trsf`] (rigid + uniform scale + mirror) and [`GTrsf`].
//!
//! [`Trsf`] mirrors OCCT's `gp_Trsf`. It stores a `scale`, a 3×3 linear
//! [`Mat`](crate::mat::Mat)rix, a translation `loc`, and a [`TrsfForm`] tag, and
//! obeys the unified apply rule:
//!
//! ```text
//! transform_point(P) = scale * (matrix · P) + loc
//! transform_vec(v)   = scale * (matrix · v)          // vectors ignore translation
//! transform_dir(d)   = normalize(matrix · d)          // directions ignore scale
//! ```
//!
//! Every elementary transform is built so that rule holds, and composition is
//! `scale = s1·s2`, `matrix = M1·M2`, `loc = s1·(M1·l2) + l1` — exactly OCCT's
//! convention, so future algorithmic code behaves identically.

use serde::{Deserialize, Serialize};

use crate::axis::Ax1;
use crate::dir::Dir;
use crate::frame::Ax2;
use crate::mat::Mat;
use crate::pnt::Pnt;
use crate::vec::Vec;
use crate::xyz::Xyz;

/// The kind of transform (OCCT `gp_TrsfForm`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum TrsfForm {
    /// The identity.
    #[default]
    Identity,
    /// A pure translation.
    Translation,
    /// A rotation about an axis.
    Rotation,
    /// A uniform scale about a point.
    Scale,
    /// A mirror through a point (point reflection).
    PntMirror,
    /// A mirror about an axis (line, a half-turn).
    Ax1Mirror,
    /// A mirror about a plane.
    Ax2Mirror,
    /// A composition of two or more elementary transforms.
    Compound,
    /// Any other affine transform.
    Other,
}

/// A geometric transform: rigid + uniform scale + mirror (OCCT `gp_Trsf`).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Trsf {
    scale: f64,
    matrix: Mat,
    loc: Vec,
    form: TrsfForm,
}

impl Default for Trsf {
    #[inline]
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Trsf {
    /// The identity transform.
    pub const IDENTITY: Self = Self {
        scale: 1.0,
        matrix: Mat::IDENTITY,
        loc: Vec::ZERO,
        form: TrsfForm::Identity,
    };

    /// The identity transform.
    #[inline]
    pub const fn new() -> Self {
        Self::IDENTITY
    }

    /// Reset to the identity.
    pub fn set_identity(&mut self) {
        *self = Self::IDENTITY;
    }

    /// True if this is exactly the identity.
    #[inline]
    pub fn is_identity(&self) -> bool {
        self.form == TrsfForm::Identity
    }

    /// The transform's form tag.
    #[inline]
    pub const fn form(&self) -> TrsfForm {
        self.form
    }

    /// The uniform scale factor.
    #[inline]
    pub const fn scale_factor(&self) -> f64 {
        self.scale
    }

    /// The linear part.
    #[inline]
    pub const fn matrix(&self) -> Mat {
        self.matrix
    }

    /// The translation part (OCCT `gp_Trsf::TranslationPart`).
    #[inline]
    pub const fn translation_part(&self) -> Vec {
        self.loc
    }

    // --- elementary setters ------------------------------------------------

    /// Pure translation by `v`.
    pub fn set_translation(&mut self, v: &Vec) {
        self.scale = 1.0;
        self.matrix = Mat::IDENTITY;
        self.loc = *v;
        self.form = TrsfForm::Translation;
    }

    /// Translation that maps `from` to `to`.
    pub fn set_translation_points(&mut self, from: &Pnt, to: &Pnt) {
        self.set_translation(&Vec::from_xyz(to.coord().subtracted(&from.coord())));
    }

    /// Set a linear transform `P ↦ scale·(M·(P − C)) + C` and derive the
    /// translation from the fixed point `center`. All rigid/scale/mirror
    /// transforms share this shape.
    fn set_linear(&mut self, center: &Pnt, matrix: Mat, scale: f64, form: TrsfForm) {
        let mc = matrix.multiply_xyz(&center.coord());
        let loc = center.coord().subtracted(&mc.multiplied(scale));
        self.scale = scale;
        self.matrix = matrix;
        self.loc = Vec::from_xyz(loc);
        self.form = form;
    }

    /// Rotation by `ang` radians about `axis` (Rodrigues' formula).
    pub fn set_rotation(&mut self, axis: &Ax1, ang: f64) {
        let d = axis.direction();
        let (x, y, z) = (d.x(), d.y(), d.z());
        let c = ang.cos();
        let s = ang.sin();
        let t = 1.0 - c;
        let matrix = Mat::from_rows([
            [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
        ]);
        self.set_linear(&axis.location(), matrix, 1.0, TrsfForm::Rotation);
    }

    /// Uniform scale by `factor` about `center`.
    pub fn set_scale(&mut self, center: &Pnt, factor: f64) {
        self.set_linear(center, Mat::IDENTITY, factor, TrsfForm::Scale);
    }

    /// Mirror through a point (point reflection): `P ↦ 2C − P`.
    pub fn set_mirror_point(&mut self, center: &Pnt) {
        let neg_i = Mat::from_rows([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]);
        self.set_linear(center, neg_i, 1.0, TrsfForm::PntMirror);
    }

    /// Mirror about an axis (line) — a half-turn about the line.
    pub fn set_mirror_axis(&mut self, axis: &Ax1) {
        let d = axis.direction();
        let m = outer(&d); // d ⊗ d
                           // R = 2(d⊗d) − I
        let matrix = Mat::from_rows([
            [2.0 * m.at(0, 0) - 1.0, 2.0 * m.at(0, 1), 2.0 * m.at(0, 2)],
            [2.0 * m.at(1, 0), 2.0 * m.at(1, 1) - 1.0, 2.0 * m.at(1, 2)],
            [2.0 * m.at(2, 0), 2.0 * m.at(2, 1), 2.0 * m.at(2, 2) - 1.0],
        ]);
        self.set_linear(&axis.location(), matrix, 1.0, TrsfForm::Ax1Mirror);
    }

    /// Mirror about the plane defined by `frame` (its main direction is the
    /// plane normal): `P ↦ P − 2·((P−C)·n)·n`.
    pub fn set_mirror_plane(&mut self, frame: &Ax2) {
        let n = frame.direction();
        let m = outer(&n); // n ⊗ n
                           // R = I − 2(n⊗n)
        let matrix = Mat::from_rows([
            [1.0 - 2.0 * m.at(0, 0), -2.0 * m.at(0, 1), -2.0 * m.at(0, 2)],
            [-2.0 * m.at(1, 0), 1.0 - 2.0 * m.at(1, 1), -2.0 * m.at(1, 2)],
            [-2.0 * m.at(2, 0), -2.0 * m.at(2, 1), 1.0 - 2.0 * m.at(2, 2)],
        ]);
        self.set_linear(&frame.location(), matrix, 1.0, TrsfForm::Ax2Mirror);
    }

    // --- apply -------------------------------------------------------------

    /// Apply to a raw coordinate triple (as a point).
    #[inline]
    pub fn transform_xyz(&self, v: &Xyz) -> Xyz {
        let mv = self.matrix.multiply_xyz(v);
        mv.multiplied(self.scale).added(&self.loc.coord())
    }

    /// Apply to a point.
    #[inline]
    pub fn transform_point(&self, p: &Pnt) -> Pnt {
        Pnt::from_xyz(self.transform_xyz(&p.coord()))
    }

    /// Apply to a free vector (translation is ignored, scale is applied).
    #[inline]
    pub fn transform_vec(&self, v: &Vec) -> Vec {
        Vec::from_xyz(self.matrix.multiply_xyz(&v.coord()).multiplied(self.scale))
    }

    /// Apply to a unit direction (translation and scale are ignored).
    #[inline]
    pub fn transform_dir(&self, d: &Dir) -> Dir {
        let r = self.matrix.multiply_xyz(&d.coord());
        Dir::new(r.x(), r.y(), r.z())
    }

    // --- composition -------------------------------------------------------

    /// Compose: `self` applied after `other` — i.e. `(self * other)(P)`.
    ///
    /// With `self = (s1, M1, l1)` and `other = (s2, M2, l2)`, the product is
    /// `(s1·s2, M1·M2, s1·(M1·l2) + l1)`.
    pub fn multiply(&self, other: &Self) -> Self {
        let matrix = self.matrix.multiplied(&other.matrix);
        let m1_l2 = self.matrix.multiply_xyz(&other.loc.coord());
        let loc = m1_l2.multiplied(self.scale).added(&self.loc.coord());
        Trsf {
            scale: self.scale * other.scale,
            matrix,
            loc: Vec::from_xyz(loc),
            form: TrsfForm::Compound,
        }
    }

    /// The inverse transform, or `None` if non-invertible (singular matrix or
    /// zero scale).
    pub fn inverted(&self) -> Option<Self> {
        if self.scale.abs() < crate::tolerance::CONFUSION {
            return None;
        }
        let inv_matrix = self.matrix.inverted()?;
        let inv_scale = 1.0 / self.scale;
        // If forward is P' = s·M·P + l, inverse is P = (1/s)·M⁻¹·P' − (1/s)·M⁻¹·l.
        let loc = inv_matrix
            .multiply_xyz(&self.loc.coord())
            .multiplied(-inv_scale);
        Some(Trsf {
            scale: inv_scale,
            matrix: inv_matrix,
            loc: Vec::from_xyz(loc),
            form: TrsfForm::Compound,
        })
    }

    // --- convenience builders (by value) -----------------------------------

    /// A translation by `v`.
    #[inline]
    pub fn translation(v: Vec) -> Self {
        let mut t = Self::new();
        t.set_translation(&v);
        t
    }

    /// A rotation by `ang` about `axis`.
    #[inline]
    pub fn rotation(axis: &Ax1, ang: f64) -> Self {
        let mut t = Self::new();
        t.set_rotation(axis, ang);
        t
    }

    /// A uniform scale by `factor` about `center`.
    #[inline]
    pub fn scale(center: &Pnt, factor: f64) -> Self {
        let mut t = Self::new();
        t.set_scale(center, factor);
        t
    }

    /// A point mirror through `center`.
    #[inline]
    pub fn mirror_point(center: &Pnt) -> Self {
        let mut t = Self::new();
        t.set_mirror_point(center);
        t
    }

    /// An axis (line) mirror.
    #[inline]
    pub fn mirror_axis(axis: &Ax1) -> Self {
        let mut t = Self::new();
        t.set_mirror_axis(axis);
        t
    }

    /// A plane mirror.
    #[inline]
    pub fn mirror_plane(frame: &Ax2) -> Self {
        let mut t = Self::new();
        t.set_mirror_plane(frame);
        t
    }
}

/// The outer product `d ⊗ d` as a 3×3 matrix.
fn outer(d: &Dir) -> Mat {
    let (x, y, z) = (d.x(), d.y(), d.z());
    Mat::from_rows([
        [x * x, x * y, x * z],
        [y * x, y * y, y * z],
        [z * x, z * y, z * z],
    ])
}

/// A general affine transform — 3×3 linear part plus translation, with no
/// scale/form constraint (OCCT `gp_GTrsf`). Used where a [`Trsf`] is too
/// restrictive (e.g. non-uniform scaling or shear).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct GTrsf {
    matrix: Mat,
    loc: Vec,
}

impl GTrsf {
    /// The identity.
    pub const IDENTITY: Self = Self {
        matrix: Mat::IDENTITY,
        loc: Vec::ZERO,
    };

    /// Promote a [`Trsf`] to a general transform.
    pub fn from_trsf(t: &Trsf) -> Self {
        Self {
            matrix: Mat::from_rows([
                [
                    t.scale_factor() * t.matrix().at(0, 0),
                    t.scale_factor() * t.matrix().at(0, 1),
                    t.scale_factor() * t.matrix().at(0, 2),
                ],
                [
                    t.scale_factor() * t.matrix().at(1, 0),
                    t.scale_factor() * t.matrix().at(1, 1),
                    t.scale_factor() * t.matrix().at(1, 2),
                ],
                [
                    t.scale_factor() * t.matrix().at(2, 0),
                    t.scale_factor() * t.matrix().at(2, 1),
                    t.scale_factor() * t.matrix().at(2, 2),
                ],
            ]),
            loc: t.translation_part(),
        }
    }

    /// Apply to a point.
    #[inline]
    pub fn transform_point(&self, p: &Pnt) -> Pnt {
        Pnt::from_xyz(
            self.matrix
                .multiply_xyz(&p.coord())
                .added(&self.loc.coord()),
        )
    }
}

impl Default for GTrsf {
    #[inline]
    fn default() -> Self {
        Self::IDENTITY
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dir::Dir;

    fn approx(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    #[test]
    fn translation_moves_points_not_vectors_or_dirs() {
        let t = Trsf::translation(Vec::new(1.0, 2.0, 3.0));
        let p = Pnt::origin();
        assert_eq!(t.transform_point(&p), Pnt::new(1.0, 2.0, 3.0));
        // A free vector is unchanged by a pure translation.
        let v = Vec::new(5.0, 0.0, 0.0);
        assert_eq!(t.transform_vec(&v), v);
    }

    #[test]
    fn rotation_about_z_maps_x_to_y() {
        let t = Trsf::rotation(
            &Ax1::new(Pnt::origin(), Dir::dz()),
            core::f64::consts::FRAC_PI_2,
        );
        let q = t.transform_point(&Pnt::new(1.0, 0.0, 0.0));
        assert!(approx(q.x(), 0.0) && approx(q.y(), 1.0) && approx(q.z(), 0.0));
    }

    #[test]
    fn rotation_about_offset_axis() {
        // Rotate 90° about an axis through (1,0,0) along Z: (2,0,0) -> (1,1,0).
        let axis = Ax1::new(Pnt::new(1.0, 0.0, 0.0), Dir::dz());
        let t = Trsf::rotation(&axis, core::f64::consts::FRAC_PI_2);
        let q = t.transform_point(&Pnt::new(2.0, 0.0, 0.0));
        assert!(approx(q.x(), 1.0) && approx(q.y(), 1.0));
    }

    #[test]
    fn uniform_scale_about_center() {
        let t = Trsf::scale(&Pnt::new(1.0, 0.0, 0.0), 2.0);
        // (2,0,0) -> 1 + 2*((2-1)) = 3
        let q = t.transform_point(&Pnt::new(2.0, 0.0, 0.0));
        assert!(approx(q.x(), 3.0));
        // center is fixed
        assert_eq!(
            t.transform_point(&Pnt::new(1.0, 0.0, 0.0)),
            Pnt::new(1.0, 0.0, 0.0)
        );
    }

    #[test]
    fn point_mirror_negates_about_center() {
        let t = Trsf::mirror_point(&Pnt::origin());
        assert_eq!(
            t.transform_point(&Pnt::new(1.0, 2.0, 3.0)),
            Pnt::new(-1.0, -2.0, -3.0)
        );
    }

    #[test]
    fn plane_mirror_flips_normal_axis() {
        let plane = Ax2::new(Pnt::origin(), Dir::dz());
        let t = Trsf::mirror_plane(&plane);
        // (1,2,3) -> (1,2,-3) across z=0
        assert_eq!(
            t.transform_point(&Pnt::new(1.0, 2.0, 3.0)),
            Pnt::new(1.0, 2.0, -3.0)
        );
    }

    #[test]
    fn composition_then_inverse_round_trips() {
        let r = Trsf::rotation(&Ax1::new(Pnt::origin(), Dir::dz()), 0.7);
        let s = Trsf::translation(Vec::new(3.0, -1.0, 2.0));
        let composed = r.multiply(&s);
        let inv = composed.inverted().expect("invertible");
        let p = Pnt::new(1.5, -2.5, 0.5);
        let rt = inv.transform_point(&composed.transform_point(&p));
        assert!(approx(rt.x(), p.x()));
        assert!(approx(rt.y(), p.y()));
        assert!(approx(rt.z(), p.z()));
    }
}
