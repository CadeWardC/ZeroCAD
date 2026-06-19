#![forbid(unsafe_code)]
//! Interop between `openrcad_topo` and the [`truck`](https://crates.io/crates/truck-topology) CAD kernel.
//!
//! Provides bidirectional conversion contexts that map between `truck_topology`
//! entities and `openrcad_topo` entities while preserving topological
//! vertex/edge/face sharing (a vertex shared by two edges in the source maps to a
//! single shared vertex in the output).
//!
//! This adapter is intentionally a separate crate rather than an
//! `openrcad-topo` feature: the core topology crate stays dependency-minimal, and
//! only projects that actually bridge to `truck` pull `truck-topology` into their
//! graph.

use std::collections::HashMap;

use openrcad_foundation::Pnt;
use openrcad_geom::{GeomCurve, GeomSurface};
use openrcad_topo::{
    Edge, EdgeId, Face, FaceId, Orientation, Shell, Solid, Vertex, VertexId, Wire,
};

/// Context for converting from `truck_topology` to `openrcad_topo`.
///
/// Keeps track of shared vertex, edge, and face mappings to ensure
/// topological connectivity is preserved in the output BRep.
pub struct TruckToOpenRcadContext<P, C, S> {
    pub vertices: HashMap<truck_topology::VertexID<P>, Vertex>,
    pub edges: HashMap<truck_topology::EdgeID<C>, Edge>,
    pub faces: HashMap<truck_topology::FaceID<S>, Face>,
}

impl<P, C, S> Default for TruckToOpenRcadContext<P, C, S> {
    fn default() -> Self {
        Self::new()
    }
}

impl<P, C, S> TruckToOpenRcadContext<P, C, S> {
    /// Create a new conversion context.
    pub fn new() -> Self {
        Self {
            vertices: HashMap::new(),
            edges: HashMap::new(),
            faces: HashMap::new(),
        }
    }

    /// Convert a truck Vertex.
    pub fn convert_vertex<F>(&mut self, v: &truck_topology::Vertex<P>, mut map_point: F) -> Vertex
    where
        F: FnMut(&P) -> Pnt,
        P: Clone,
    {
        let id = v.id();
        self.vertices
            .entry(id)
            .or_insert_with(|| Vertex::new(map_point(&v.point())))
            .clone()
    }

    /// Convert a truck Edge.
    pub fn convert_edge<FP, FC>(
        &mut self,
        e: &truck_topology::Edge<P, C>,
        map_point: FP,
        mut map_curve: FC,
    ) -> Edge
    where
        FP: FnMut(&P) -> Pnt + Clone,
        FC: FnMut(&C) -> Option<GeomCurve>,
        P: Clone,
        C: Clone,
    {
        let id = e.id();
        if let Some(edge) = self.edges.get(&id) {
            let mut edge = edge.clone();
            if !e.orientation() {
                edge = edge.reversed();
            }
            return edge;
        }

        let start = self.convert_vertex(e.absolute_front(), map_point.clone());
        let end = self.convert_vertex(e.absolute_back(), map_point.clone());
        let curve = map_curve(&e.curve());
        let first = 0.0;
        let last = 1.0;
        let new_edge = Edge::new(curve, first, last, start, end);
        self.edges.insert(id, new_edge.clone());

        let mut edge = new_edge;
        if !e.orientation() {
            edge = edge.reversed();
        }
        edge
    }

    /// Convert a truck Wire.
    pub fn convert_wire<FP, FC>(
        &mut self,
        w: &truck_topology::Wire<P, C>,
        map_point: FP,
        map_curve: FC,
    ) -> Wire
    where
        FP: FnMut(&P) -> Pnt + Clone,
        FC: FnMut(&C) -> Option<GeomCurve> + Clone,
        P: Clone,
        C: Clone,
    {
        let edges: Vec<Edge> = w
            .iter()
            .map(|e| self.convert_edge(e, map_point.clone(), map_curve.clone()))
            .collect();
        Wire::new(edges)
    }

    /// Convert a truck Face.
    pub fn convert_face<FP, FC, FS>(
        &mut self,
        f: &truck_topology::Face<P, C, S>,
        map_point: FP,
        map_curve: FC,
        mut map_surface: FS,
    ) -> Face
    where
        FP: FnMut(&P) -> Pnt + Clone,
        FC: FnMut(&C) -> Option<GeomCurve> + Clone,
        FS: FnMut(&S) -> Option<GeomSurface>,
        P: Clone,
        C: Clone,
        S: Clone,
    {
        let id = f.id();
        let face_orientation = if f.orientation() {
            Orientation::Forward
        } else {
            Orientation::Reversed
        };
        if let Some(face) = self.faces.get(&id) {
            // Re-handle the cached face with the requested orientation.
            return Face::from_id(face.brep().clone(), face.id(), face_orientation);
        }

        let surface = map_surface(&f.surface());
        let boundaries = f.boundaries();
        let outer_wire = boundaries
            .first()
            .map(|w| self.convert_wire(w, map_point.clone(), map_curve.clone()));
        let inner_wires = if boundaries.len() > 1 {
            boundaries[1..]
                .iter()
                .map(|w| self.convert_wire(w, map_point.clone(), map_curve.clone()))
                .collect()
        } else {
            Vec::new()
        };

        let new_face = Face::with_wires(surface, outer_wire, inner_wires, face_orientation);
        self.faces.insert(id, new_face.clone());
        new_face
    }

    /// Convert a truck Shell.
    pub fn convert_shell<FP, FC, FS>(
        &mut self,
        s: &truck_topology::Shell<P, C, S>,
        map_point: FP,
        map_curve: FC,
        map_surface: FS,
    ) -> Shell
    where
        FP: FnMut(&P) -> Pnt + Clone,
        FC: FnMut(&C) -> Option<GeomCurve> + Clone,
        FS: FnMut(&S) -> Option<GeomSurface> + Clone,
        P: Clone,
        C: Clone,
        S: Clone,
    {
        let faces: Vec<Face> = s
            .iter()
            .map(|f| {
                self.convert_face(f, map_point.clone(), map_curve.clone(), map_surface.clone())
            })
            .collect();
        Shell::new(faces)
    }

    /// Convert a truck Solid.
    pub fn convert_solid<FP, FC, FS>(
        &mut self,
        s: &truck_topology::Solid<P, C, S>,
        map_point: FP,
        map_curve: FC,
        map_surface: FS,
    ) -> Solid
    where
        FP: FnMut(&P) -> Pnt + Clone,
        FC: FnMut(&C) -> Option<GeomCurve> + Clone,
        FS: FnMut(&S) -> Option<GeomSurface> + Clone,
        P: Clone,
        C: Clone,
        S: Clone,
    {
        let outer_shell = s
            .boundaries()
            .first()
            .map(|shell| {
                self.convert_shell(
                    shell,
                    map_point.clone(),
                    map_curve.clone(),
                    map_surface.clone(),
                )
            })
            .unwrap_or_default();
        Solid::new(outer_shell)
    }
}

/// Context for converting from `openrcad_topo` to `truck_topology`.
///
/// Keeps track of shared vertex, edge, and face mappings to ensure
/// topological connectivity is preserved.
pub struct OpenRcadToTruckContext<P, C, S> {
    pub vertices: HashMap<VertexId, truck_topology::Vertex<P>>,
    pub edges: HashMap<EdgeId, truck_topology::Edge<P, C>>,
    pub faces: HashMap<FaceId, truck_topology::Face<P, C, S>>,
}

impl<P, C, S> Default for OpenRcadToTruckContext<P, C, S> {
    fn default() -> Self {
        Self::new()
    }
}

impl<P, C, S> OpenRcadToTruckContext<P, C, S> {
    /// Create a new conversion context.
    pub fn new() -> Self {
        Self {
            vertices: HashMap::new(),
            edges: HashMap::new(),
            faces: HashMap::new(),
        }
    }

    /// Convert an openrcad Vertex.
    pub fn convert_vertex<F>(&mut self, v: &Vertex, mut map_point: F) -> truck_topology::Vertex<P>
    where
        F: FnMut(&Pnt) -> P,
    {
        self.vertices
            .entry(v.id())
            .or_insert_with(|| truck_topology::Vertex::new(map_point(&v.point())))
            .clone()
    }

    /// Convert an openrcad Edge.
    pub fn convert_edge<FP, FC>(
        &mut self,
        e: &Edge,
        map_point: FP,
        mut map_curve: FC,
    ) -> truck_topology::Edge<P, C>
    where
        FP: FnMut(&Pnt) -> P + Clone,
        FC: FnMut(Option<&GeomCurve>) -> C,
    {
        let mut truck_edge = if let Some(edge) = self.edges.get(&e.id()) {
            edge.clone()
        } else {
            let start = self.convert_vertex(&e.start(), map_point.clone());
            let end = self.convert_vertex(&e.end(), map_point.clone());
            let curve = map_curve(e.curve());
            let edge = truck_topology::Edge::new(&start, &end, curve);
            self.edges.insert(e.id(), edge.clone());
            edge
        };
        if !e.orientation().is_forward() {
            truck_edge = truck_edge.inverse();
        }
        truck_edge
    }

    /// Convert an openrcad Wire.
    pub fn convert_wire<FP, FC>(
        &mut self,
        w: &Wire,
        map_point: FP,
        map_curve: FC,
    ) -> truck_topology::Wire<P, C>
    where
        FP: FnMut(&Pnt) -> P + Clone,
        FC: FnMut(Option<&GeomCurve>) -> C + Clone,
    {
        let edges: Vec<truck_topology::Edge<P, C>> = w
            .edges()
            .iter()
            .map(|e| self.convert_edge(e, map_point.clone(), map_curve.clone()))
            .collect();
        truck_topology::Wire::from(edges)
    }

    /// Convert an openrcad Face.
    pub fn convert_face<FP, FC, FS>(
        &mut self,
        f: &Face,
        map_point: FP,
        map_curve: FC,
        mut map_surface: FS,
    ) -> truck_topology::Face<P, C, S>
    where
        FP: FnMut(&Pnt) -> P + Clone,
        FC: FnMut(Option<&GeomCurve>) -> C + Clone,
        FS: FnMut(Option<&GeomSurface>) -> S,
    {
        let mut truck_face = if let Some(face) = self.faces.get(&f.id()) {
            face.clone()
        } else {
            let surface = map_surface(f.surface());
            let mut boundaries = Vec::new();
            if let Some(outer) = f.outer_wire() {
                boundaries.push(self.convert_wire(&outer, map_point.clone(), map_curve.clone()));
            }
            for inner in f.inner_wires() {
                boundaries.push(self.convert_wire(&inner, map_point.clone(), map_curve.clone()));
            }
            truck_topology::Face::new(boundaries, surface)
        };
        if !f.orientation().is_forward() {
            truck_face = truck_face.inverse();
        }
        truck_face
    }

    /// Convert an openrcad Shell.
    pub fn convert_shell<FP, FC, FS>(
        &mut self,
        s: &Shell,
        map_point: FP,
        map_curve: FC,
        map_surface: FS,
    ) -> truck_topology::Shell<P, C, S>
    where
        FP: FnMut(&Pnt) -> P + Clone,
        FC: FnMut(Option<&GeomCurve>) -> C + Clone,
        FS: FnMut(Option<&GeomSurface>) -> S + Clone,
    {
        let faces: Vec<truck_topology::Face<P, C, S>> = s
            .faces()
            .iter()
            .map(|f| {
                self.convert_face(f, map_point.clone(), map_curve.clone(), map_surface.clone())
            })
            .collect();
        truck_topology::Shell::from(faces)
    }

    /// Convert an openrcad Solid.
    pub fn convert_solid<FP, FC, FS>(
        &mut self,
        s: &Solid,
        map_point: FP,
        map_curve: FC,
        map_surface: FS,
    ) -> truck_topology::Solid<P, C, S>
    where
        FP: FnMut(&Pnt) -> P + Clone,
        FC: FnMut(Option<&GeomCurve>) -> C + Clone,
        FS: FnMut(Option<&GeomSurface>) -> S + Clone,
    {
        let outer_shell = self.convert_shell(&s.shell(), map_point, map_curve, map_surface);
        truck_topology::Solid::new(vec![outer_shell])
    }
}
