//! A topological face — a bounded region of a [`GeomSurface`] (OCCT `TopoDS_Face`).
//!
//! A face carries its supporting surface, an outer [`Wire`] (the boundary loop)
//! and any number of inner wires (holes). Like edges, the surface is owned by
//! value (an [`GeomSurface`] sum type) so the face is `Clone` + `Serialize`.

use openrcad_foundation::Trsf;
use openrcad_geom::GeomSurface;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

use crate::arena::{BRep, FaceData, FaceId};
use crate::orientation::Orientation;
use crate::wire::Wire;

/// A face: a trimmed patch of a surface.
#[derive(Clone, Debug)]
pub struct Face {
    pub(crate) brep: Arc<BRep>,
    pub(crate) id: FaceId,
    pub(crate) orientation: Orientation,
}

impl Face {
    /// Get the Face ID in the arena.
    #[inline]
    pub fn id(&self) -> FaceId {
        self.id
    }

    /// Get the underlying BRep storage.
    #[inline]
    pub fn brep(&self) -> &Arc<BRep> {
        &self.brep
    }

    /// Construct a Face handle from a shared BRep storage and Face ID.
    #[inline]
    pub fn from_id(brep: Arc<BRep>, id: FaceId, orientation: Orientation) -> Self {
        Self {
            brep,
            id,
            orientation,
        }
    }

    /// A face on `surface` bounded by `outer_wire`, no holes, `Forward`.
    pub fn new(surface: Option<GeomSurface>, outer_wire: Wire) -> Self {
        let mut brep = BRep::new();
        let map_wire = brep.merge(&outer_wire.brep);
        let new_outer = map_wire.loops[&outer_wire.id];
        let data = FaceData {
            surface,
            outer_wire: Some(new_outer),
            inner_wires: Vec::new(),
            orientation: Orientation::Forward,
        };
        let id = brep.faces.insert(data);
        Self {
            brep: Arc::new(brep),
            id,
            orientation: Orientation::Forward,
        }
    }

    /// A face with explicit wires and orientation.
    pub fn with_wires(
        surface: Option<GeomSurface>,
        outer_wire: Option<Wire>,
        inner_wires: Vec<Wire>,
        orientation: Orientation,
    ) -> Self {
        let mut brep = BRep::new();
        let new_outer = outer_wire.map(|w| {
            let map = brep.merge(&w.brep);
            map.loops[&w.id]
        });
        let mut new_inners = Vec::new();
        for w in inner_wires {
            let map = brep.merge(&w.brep);
            new_inners.push(map.loops[&w.id]);
        }
        let data = FaceData {
            surface,
            outer_wire: new_outer,
            inner_wires: new_inners,
            orientation: Orientation::Forward,
        };
        let id = brep.faces.insert(data);
        Self {
            brep: Arc::new(brep),
            id,
            orientation,
        }
    }

    /// The supporting surface, if any.
    #[inline]
    pub fn surface(&self) -> Option<&GeomSurface> {
        self.brep.faces[self.id].surface.as_ref()
    }

    /// The outer boundary loop, if any.
    #[inline]
    pub fn outer_wire(&self) -> Option<Wire> {
        self.brep.faces[self.id].outer_wire.map(|loop_id| Wire {
            brep: self.brep.clone(),
            id: loop_id,
        })
    }

    /// The inner (hole) loops.
    #[inline]
    pub fn inner_wires(&self) -> Vec<Wire> {
        self.brep.faces[self.id]
            .inner_wires
            .iter()
            .map(|&loop_id| Wire {
                brep: self.brep.clone(),
                id: loop_id,
            })
            .collect()
    }

    /// The orientation of this face.
    #[inline]
    pub fn orientation(&self) -> Orientation {
        self.orientation
    }

    /// Every wire of this face (outer first, then inner), by value.
    pub fn wires(&self) -> Vec<Wire> {
        let mut out = Vec::with_capacity(1 + self.brep.faces[self.id].inner_wires.len());
        if let Some(w) = self.outer_wire() {
            out.push(w);
        }
        for w in self.inner_wires() {
            out.push(w);
        }
        out
    }

    /// Apply `t` to the surface and every wire.
    pub fn transformed(&self, t: &Trsf) -> Self {
        let transformed_brep = self.brep.transformed(t);
        Self {
            brep: Arc::new(transformed_brep),
            id: self.id,
            orientation: self.orientation,
        }
    }

    /// Invert/reverse the orientation of the face.
    #[inline]
    pub fn reversed(&self) -> Self {
        Self {
            brep: self.brep.clone(),
            id: self.id,
            orientation: self.orientation.reversed(),
        }
    }
}

impl PartialEq for Face {
    fn eq(&self, other: &Self) -> bool {
        self.surface() == other.surface()
            && self.outer_wire() == other.outer_wire()
            && self.inner_wires() == other.inner_wires()
            && self.orientation() == other.orientation()
    }
}

impl Serialize for Face {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        (&*self.brep, self.id, self.orientation).serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Face {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let (brep, id, orientation) = <(BRep, FaceId, Orientation)>::deserialize(deserializer)?;
        Ok(Self {
            brep: Arc::new(brep),
            id,
            orientation,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::edge::Edge;
    use openrcad_foundation::{Dir, Pnt, Trsf, Vec};
    use openrcad_geom::{GeomSurface, Plane, Surface};

    fn unit_square_xy() -> Wire {
        Wire::from_edges([
            Edge::between_points(Pnt::origin(), Pnt::new(1.0, 0.0, 0.0)),
            Edge::between_points(Pnt::new(1.0, 0.0, 0.0), Pnt::new(1.0, 1.0, 0.0)),
            Edge::between_points(Pnt::new(1.0, 1.0, 0.0), Pnt::new(0.0, 1.0, 0.0)),
            Edge::between_points(Pnt::new(0.0, 1.0, 0.0), Pnt::origin()),
        ])
    }

    #[test]
    fn face_exposes_its_wires() {
        let face = Face::new(None, unit_square_xy());
        assert_eq!(face.wires().len(), 1);
        assert_eq!(face.outer_wire().unwrap().len(), 4);
        assert!(face.inner_wires().is_empty());
    }

    #[test]
    fn face_with_hole_has_outer_and_inner() {
        let outer = unit_square_xy();
        let inner = Wire::from_edges([
            Edge::between_points(Pnt::new(0.25, 0.25, 0.0), Pnt::new(0.75, 0.25, 0.0)),
            Edge::between_points(Pnt::new(0.75, 0.25, 0.0), Pnt::new(0.75, 0.75, 0.0)),
            Edge::between_points(Pnt::new(0.75, 0.75, 0.0), Pnt::new(0.25, 0.75, 0.0)),
            Edge::between_points(Pnt::new(0.25, 0.75, 0.0), Pnt::new(0.25, 0.25, 0.0)),
        ]);
        let face = Face::with_wires(None, Some(outer), vec![inner], Orientation::Forward);
        assert_eq!(face.wires().len(), 2);
        assert_eq!(face.inner_wires().len(), 1);
    }

    #[test]
    fn transformed_face_moves_surface_and_wire() {
        let surf = GeomSurface::plane(Plane::from_point_normal(Pnt::origin(), Dir::dz()));
        let face = Face::new(Some(surf), unit_square_xy());
        let up = Trsf::translation(Vec::new(0.0, 0.0, 5.0));
        let moved = face.transformed(&up);
        // The lifted face's surface passes through z = 5.
        let s = moved.surface().unwrap();
        assert!((s.point(0.0, 0.0).z() - 5.0).abs() < 1e-9);
        // And its first edge's start vertex is lifted too.
        let first_start = moved.outer_wire().unwrap().edges()[0].start().point();
        assert!((first_start.z() - 5.0).abs() < 1e-9);
    }
}
