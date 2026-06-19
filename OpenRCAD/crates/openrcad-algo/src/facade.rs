//! A fluent, chainable modeling facade over the free algorithm functions.
//!
//! [`SolidExt`] hangs the boolean, blend, and query operations directly off a
//! [`Solid`] so models read as a pipeline:
//!
//! ```
//! use openrcad_primitives::make_box;
//! use openrcad_foundation::{Dir, Pnt};
//! use openrcad_algo::SolidExt;
//!
//! let block = make_box(&Pnt::origin(), 20.0, 20.0, 10.0);
//! // Pick the cap facing +Z, then hollow the block leaving that face open.
//! let top = block.faces_facing(&Dir::dz());
//! assert_eq!(top.len(), 1);
//! let cup = block.hollow(2.0, &top)?;
//! assert!(cup.face_count() > block.face_count());
//! # Ok::<(), openrcad_algo::BlendError>(())
//! ```

use openrcad_foundation::Dir;
use openrcad_geom::GeomSurface;
use openrcad_topo::{Face, Solid};

use crate::blend::BlendError;
use crate::{boolean, BooleanOp};

/// Chainable modeling operations on a [`Solid`] — an ergonomic facade over
/// [`boolean`](crate::boolean), [`fillet`](crate::fillet),
/// [`chamfer`](crate::chamfer), and [`shell_solid`](crate::shell_solid), plus a
/// couple of face selectors.
///
/// ```
/// use openrcad_primitives::make_box;
/// use openrcad_foundation::Pnt;
/// use openrcad_algo::SolidExt;
///
/// let rounded = make_box(&Pnt::origin(), 10.0, 10.0, 10.0).fillet(1.0)?;
/// assert_eq!(rounded.face_count(), 26); // 6 planes + 12 cylinders + 8 spheres
/// # Ok::<(), openrcad_algo::BlendError>(())
/// ```
pub trait SolidExt {
    /// Fuse `self` with `tool` (boolean union, [`BooleanOp::Fuse`]).
    fn union(&self, tool: &Solid) -> Solid;
    /// Subtract `tool` from `self` (boolean difference, [`BooleanOp::Cut`]).
    fn subtract(&self, tool: &Solid) -> Solid;
    /// Keep only the volume common to `self` and `tool` ([`BooleanOp::Common`]).
    fn intersect(&self, tool: &Solid) -> Solid;
    /// Roll a constant-`radius` fillet along every edge (see [`fillet`](crate::fillet)).
    fn fillet(&self, radius: f64) -> Result<Solid, BlendError>;
    /// Chamfer every edge by `distance` (see [`chamfer`](crate::chamfer)).
    fn chamfer(&self, distance: f64) -> Result<Solid, BlendError>;
    /// Hollow to wall `thickness`, leaving `open_faces` open (see
    /// [`shell_solid`](crate::shell_solid)). Named `hollow` to avoid colliding
    /// with the inherent [`Solid::shell`](openrcad_topo::Solid::shell) accessor.
    fn hollow(&self, thickness: f64, open_faces: &[Face]) -> Result<Solid, BlendError>;
    /// Every face backed by a planar surface.
    fn planar_faces(&self) -> Vec<Face>;
    /// Planar faces whose **outward** normal points along `dir` (within `1e-6`).
    /// Handy for picking the cap to open before [`shell`](SolidExt::shell), e.g.
    /// `solid.faces_facing(&Dir::dz())` for the top face.
    fn faces_facing(&self, dir: &Dir) -> Vec<Face>;
}

impl SolidExt for Solid {
    fn union(&self, tool: &Solid) -> Solid {
        boolean::boolean(self, tool, BooleanOp::Fuse)
    }
    fn subtract(&self, tool: &Solid) -> Solid {
        boolean::boolean(self, tool, BooleanOp::Cut)
    }
    fn intersect(&self, tool: &Solid) -> Solid {
        boolean::boolean(self, tool, BooleanOp::Common)
    }
    fn fillet(&self, radius: f64) -> Result<Solid, BlendError> {
        crate::fillet(self, radius)
    }
    fn chamfer(&self, distance: f64) -> Result<Solid, BlendError> {
        crate::chamfer(self, distance)
    }
    fn hollow(&self, thickness: f64, open_faces: &[Face]) -> Result<Solid, BlendError> {
        crate::shell_solid(self, thickness, open_faces)
    }
    fn planar_faces(&self) -> Vec<Face> {
        self.shell()
            .faces()
            .into_iter()
            .filter(|f| matches!(f.surface(), Some(GeomSurface::Plane(_))))
            .collect()
    }
    fn faces_facing(&self, dir: &Dir) -> Vec<Face> {
        self.shell()
            .faces()
            .into_iter()
            .filter(|f| match f.surface() {
                Some(GeomSurface::Plane(p)) => {
                    // Plane normal points outward when the face is Forward; a
                    // Reversed face flips it. `n · dir ≈ 1` ⇒ same direction.
                    let d = p.normal().dot(dir);
                    let outward = if f.orientation().is_forward() { d } else { -d };
                    outward > 1.0 - 1e-6
                }
                _ => false,
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::{Dir, Pnt};
    use openrcad_primitives::make_box;

    #[test]
    fn selectors_find_axis_faces() {
        let b = make_box(&Pnt::origin(), 2.0, 3.0, 4.0);
        // A box has six planar faces, exactly one facing +Z.
        assert_eq!(b.planar_faces().len(), 6);
        let top = b.faces_facing(&Dir::dz());
        assert_eq!(top.len(), 1);
        // ...and one facing each of the other axes.
        assert_eq!(b.faces_facing(&Dir::dx()).len(), 1);
        assert_eq!(b.faces_facing(&Dir::new(0.0, -1.0, 0.0)).len(), 1);
    }

    #[test]
    fn fluent_blend_matches_free_function() {
        let b = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let via_trait = b.fillet(1.0).unwrap();
        let via_fn = crate::fillet(&b, 1.0).unwrap();
        assert_eq!(via_trait.face_count(), via_fn.face_count());
    }
}
