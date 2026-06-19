//! A topological edge — a bounded piece of a 3D curve (OCCT `TopoDS_Edge`).
//!
//! An edge is the segment of a [`GeomCurve`] between two parameter values
//! (`first`..`last`), bounded by a [`start`] and [`end`] [`Vertex`]. The curve is
//! stored by value (an owned [`GeomCurve`] sum type), so an edge is `Clone` +
//! `Serialize` with no lifetime or `Box<dyn>` indirection — the same property the
//! README's "owned enum geometry" decision calls for.
//!
//! [`start`]: Edge::start
//! [`end`]: Edge::end

use openrcad_foundation::{Pnt, Trsf, Vec};
use openrcad_geom::{GeomCurve, Line};
use serde::{Deserialize, Serialize};
use std::sync::Arc;

use crate::arena::{BRep, EdgeData, EdgeId};
use crate::orientation::Orientation;
use crate::vertex::Vertex;

/// A bounded edge of a [`crate::wire::Wire`].
#[derive(Clone, Debug)]
pub struct Edge {
    pub(crate) brep: Arc<BRep>,
    pub(crate) id: EdgeId,
    pub(crate) orientation: Orientation,
}

impl Edge {
    /// Get the Edge ID in the arena.
    #[inline]
    pub fn id(&self) -> EdgeId {
        self.id
    }

    /// Get the underlying BRep storage.
    #[inline]
    pub fn brep(&self) -> &Arc<BRep> {
        &self.brep
    }

    /// An edge along `curve` from `first` to `last`, bounded by `start` and `end`,
    /// with the default `Forward` orientation.
    pub fn new(
        curve: Option<GeomCurve>,
        first: f64,
        last: f64,
        start: Vertex,
        end: Vertex,
    ) -> Self {
        Self::new_with_tolerance(
            curve,
            first,
            last,
            start,
            end,
            openrcad_foundation::tolerance::CONFUSION,
        )
    }

    /// An edge along `curve` from `first` to `last` with a specific `tolerance`.
    pub fn new_with_tolerance(
        curve: Option<GeomCurve>,
        first: f64,
        last: f64,
        start: Vertex,
        end: Vertex,
        tolerance: f64,
    ) -> Self {
        let mut brep = BRep::new();
        let map_start = brep.merge(&start.brep);
        let (new_start, new_end) = if Arc::ptr_eq(&start.brep, &end.brep) {
            (map_start.vertices[&start.id], map_start.vertices[&end.id])
        } else {
            let map_end = brep.merge(&end.brep);
            (map_start.vertices[&start.id], map_end.vertices[&end.id])
        };

        let data = EdgeData {
            curve,
            first,
            last,
            start: new_start,
            end: new_end,
            tolerance,
        };
        let id = brep.edges.insert(data);
        Self {
            brep: Arc::new(brep),
            id,
            orientation: Orientation::Forward,
        }
    }

    /// A straight segment from `p0` to `p1` — a [`Line`] of length `|p1 - p0|`.
    ///
    /// This is the workhorse builder for polygonal shapes (boxes, prisms).
    pub fn between_points(p0: Pnt, p1: Pnt) -> Self {
        let disp: Vec = p1 - p0;
        let length = disp.magnitude();
        let start = Vertex::new(p0);
        let end = Vertex::new(p1);
        if length <= openrcad_foundation::tolerance::CONFUSION {
            // Degenerate edge (zero length): keep the endpoints, carry no curve.
            return Self::new(None, 0.0, 0.0, start, end);
        }
        let dir = disp.normalized().expect("non-zero displacement");
        let line = GeomCurve::line(Line::from_point_dir(p0, dir));
        Self::new(Some(line), 0.0, length, start, end)
    }

    /// The supporting curve, if any.
    #[inline]
    pub fn curve(&self) -> Option<&GeomCurve> {
        self.brep.edges[self.id].curve.as_ref()
    }

    /// The start parameter.
    #[inline]
    pub fn first(&self) -> f64 {
        self.brep.edges[self.id].first
    }

    /// The end parameter.
    #[inline]
    pub fn last(&self) -> f64 {
        self.brep.edges[self.id].last
    }

    /// The local uncertainty tolerance.
    #[inline]
    pub fn tolerance(&self) -> f64 {
        self.brep.edges[self.id].tolerance
    }

    /// The start vertex.
    #[inline]
    pub fn start(&self) -> Vertex {
        Vertex {
            brep: self.brep.clone(),
            id: self.brep.edges[self.id].start,
        }
    }

    /// The end vertex.
    #[inline]
    pub fn end(&self) -> Vertex {
        Vertex {
            brep: self.brep.clone(),
            id: self.brep.edges[self.id].end,
        }
    }

    /// The orientation (sense) of this edge.
    #[inline]
    pub fn orientation(&self) -> Orientation {
        self.orientation
    }

    /// The start vertex taking [`orientation`](Self::orientation) into account:
    /// `Forward` edges start at [`start`](Self::start), `Reversed` at [`end`](Self::end).
    #[inline]
    pub fn source(&self) -> Vertex {
        match self.orientation {
            Orientation::Forward | Orientation::Internal | Orientation::External => self.start(),
            Orientation::Reversed => self.end(),
        }
    }

    /// The end vertex taking [`orientation`](Self::orientation) into account.
    #[inline]
    pub fn target(&self) -> Vertex {
        match self.orientation {
            Orientation::Forward | Orientation::Internal | Orientation::External => self.end(),
            Orientation::Reversed => self.start(),
        }
    }

    /// The edge's chord length (`|end - start|`).
    #[inline]
    pub fn length(&self) -> f64 {
        self.start().point().distance(&self.end().point())
    }

    /// Apply `t` to the curve and both vertices, returning a new edge.
    pub fn transformed(&self, t: &Trsf) -> Self {
        let transformed_brep = self.brep.transformed(t);
        Self {
            brep: Arc::new(transformed_brep),
            id: self.id,
            orientation: self.orientation,
        }
    }

    /// Invert/reverse the orientation of the edge.
    #[inline]
    pub fn reversed(&self) -> Self {
        Self {
            brep: self.brep.clone(),
            id: self.id,
            orientation: self.orientation.reversed(),
        }
    }
}

impl PartialEq for Edge {
    fn eq(&self, other: &Self) -> bool {
        self.curve() == other.curve()
            && self.first() == other.first()
            && self.last() == other.last()
            && self.start() == other.start()
            && self.end() == other.end()
            && self.orientation() == other.orientation()
    }
}

impl Serialize for Edge {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        (&*self.brep, self.id, self.orientation).serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Edge {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let (brep, id, orientation) = <(BRep, EdgeId, Orientation)>::deserialize(deserializer)?;
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
    use openrcad_foundation::{Ax1, Dir};
    use openrcad_geom::Curve;

    #[test]
    fn segment_carries_a_line_and_endpoints() {
        let e = Edge::between_points(Pnt::origin(), Pnt::new(0.0, 0.0, 5.0));
        assert_eq!(e.start().point(), Pnt::origin());
        assert_eq!(e.end().point(), Pnt::new(0.0, 0.0, 5.0));
        assert!((e.length() - 5.0).abs() < 1e-12);
        assert!((e.last() - 5.0).abs() < 1e-12);
        // The line evaluates to the endpoints at its parameter bounds.
        let c = e.curve().expect("segment has a line");
        assert_eq!(c.point(e.first()), e.start().point());
        assert_eq!(c.point(e.last()), e.end().point());
    }

    #[test]
    fn transformed_moves_endpoints_and_curve() {
        let e = Edge::between_points(Pnt::origin(), Pnt::new(2.0, 0.0, 0.0));
        let up = Trsf::translation(Vec::new(0.0, 0.0, 10.0));
        let e2 = e.transformed(&up);
        assert_eq!(e2.start().point(), Pnt::new(0.0, 0.0, 10.0));
        assert_eq!(e2.end().point(), Pnt::new(2.0, 0.0, 10.0));
        assert!((e2.length() - 2.0).abs() < 1e-12);
    }

    #[test]
    fn rotated_segment_keeps_length() {
        // A 3-4-5 segment (length 5).
        let e = Edge::between_points(Pnt::origin(), Pnt::new(3.0, 4.0, 0.0));
        assert!((e.length() - 5.0).abs() < 1e-12);
        let r = Trsf::rotation(
            &Ax1::new(Pnt::origin(), Dir::dz()),
            core::f64::consts::FRAC_PI_2,
        );
        let e2 = e.transformed(&r);
        assert!((e2.length() - 5.0).abs() < 1e-9);
    }
}
