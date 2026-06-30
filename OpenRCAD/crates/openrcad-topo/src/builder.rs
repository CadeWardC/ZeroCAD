//! A mutable staging builder for topological BRep updates.
//!
//! Enables local Euler operators (e.g. splitting edges and faces)
//! on a mutable BRep state, which can then be sealed into an immutable Arc<BRep>.

use crate::arena::{BRep, EdgeData, EdgeId, FaceData, FaceId, LoopData, OrientedEdge, VertexId};
use crate::containment::point_in_polygon_2d;
use crate::orientation::Orientation;
use core::f64::consts::PI;
use openrcad_geom::{Curve, GeomSurface, Surface};
use std::sync::Arc;

fn norm_angle(a: f64) -> f64 {
    let t = 2.0 * PI;
    let mut x = a % t;
    if x < 0.0 {
        x += t;
    }
    x
}

fn analytic_surface_uv(surface: &GeomSurface, pt: openrcad_foundation::Pnt) -> Option<(f64, f64)> {
    match surface {
        GeomSurface::Plane(plane) => {
            let diff = pt - plane.location();
            let u = diff.dot(&openrcad_foundation::Vec::from_dir(
                plane.position().x_direction(),
            ));
            let v = diff.dot(&openrcad_foundation::Vec::from_dir(
                plane.position().y_direction(),
            ));
            Some((u, v))
        }
        GeomSurface::Cylinder(cyl) => {
            let pos = cyl.position();
            let axis_pt = pos.location();
            let axis_dir = openrcad_foundation::Vec::from_dir(pos.direction());
            let x_dir = openrcad_foundation::Vec::from_dir(pos.x_direction());
            let y_dir = openrcad_foundation::Vec::from_dir(pos.y_direction());
            let diff = pt - axis_pt;
            let v = diff.dot(&axis_dir);
            let radial = diff - axis_dir * v;
            let u = norm_angle(radial.dot(&y_dir).atan2(radial.dot(&x_dir)));
            Some((u, v))
        }
        _ => None,
    }
}

fn surface_debug_name(surface: &GeomSurface) -> &'static str {
    match surface {
        GeomSurface::Plane(_) => "plane",
        GeomSurface::Cylinder(_) => "cylinder",
        GeomSurface::Sphere(_) => "sphere",
        GeomSurface::Torus(_) => "torus",
        GeomSurface::BSpline(_) => "bspline",
        _ => "surface",
    }
}

fn edge_debug_line(brep: &BRep, oe: OrientedEdge) -> String {
    let Some(edge) = brep.edges.get(oe.id) else {
        return format!("{:?} {:?} missing", oe.id, oe.orientation);
    };
    let a = brep
        .vertices
        .get(edge.start)
        .map(|v| v.point)
        .unwrap_or_else(openrcad_foundation::Pnt::origin);
    let b = brep
        .vertices
        .get(edge.end)
        .map(|v| v.point)
        .unwrap_or_else(openrcad_foundation::Pnt::origin);
    let kind = match edge.curve.as_ref() {
        Some(openrcad_geom::GeomCurve::Line(_)) => "line",
        Some(openrcad_geom::GeomCurve::Circle(_)) => "circle",
        Some(openrcad_geom::GeomCurve::BSpline(_)) => "bspline",
        Some(_) => "curve",
        None => "none",
    };
    format!(
        "{:?} {:?} {kind} ({:.4},{:.4},{:.4})->({:.4},{:.4},{:.4})",
        oe.id,
        oe.orientation,
        a.x(),
        a.y(),
        a.z(),
        b.x(),
        b.y(),
        b.z()
    )
}

/// Locate the `(u, v)` parameters of the point on `surface` nearest to `pt`.
///
/// Used to project 3D points into a general (non-planar) surface's parameter
/// space for 2D containment tests. A coarse parameter sweep seeds a short
/// bounded Gauss-Newton refinement using the surface's first derivatives.
fn search_nearest_parameter(surface: &GeomSurface, pt: openrcad_foundation::Pnt) -> (f64, f64) {
    let (mut u0, mut u1, mut v0, mut v1) = surface.bounds();
    // Clamp unbounded directions to a finite working window (cf. `to_bspline`).
    if !u0.is_finite() {
        u0 = -100.0;
    }
    if !u1.is_finite() {
        u1 = 100.0;
    }
    if !v0.is_finite() {
        v0 = -100.0;
    }
    if !v1.is_finite() {
        v1 = 100.0;
    }
    let (u0, u1) = ordered_bounds(u0, u1);
    let (v0, v1) = ordered_bounds(v0, v1);

    // Coarse sweep for a good initial guess.
    let n = 16;
    let mut best = (u0, v0);
    let mut best_d2 = f64::INFINITY;
    for i in 0..=n {
        let u = u0 + (u1 - u0) * (i as f64) / (n as f64);
        for j in 0..=n {
            let v = v0 + (v1 - v0) * (j as f64) / (n as f64);
            let d2 = surface.point(u, v).distance_squared(&pt);
            if d2 < best_d2 {
                best_d2 = d2;
                best = (u, v);
            }
        }
    }

    // Gauss-Newton refinement: minimize ½‖S(u,v) − pt‖² using d1.
    let (mut u, mut v) = best;
    for _ in 0..16 {
        let (s, du, dv) = surface.d1(u, v);
        let r = s - pt;
        let guu = du.dot(&du);
        let gvv = dv.dot(&dv);
        let guv = du.dot(&dv);
        let bu = r.dot(&du);
        let bv = r.dot(&dv);
        let det = guu * gvv - guv * guv;
        if det.abs() <= 1e-14 {
            break;
        }
        let step_u = (bu * gvv - bv * guv) / det;
        let step_v = (guu * bv - guv * bu) / det;
        u = clamp_ordered(u - step_u, u0, u1);
        v = clamp_ordered(v - step_v, v0, v1);
        if step_u.abs() + step_v.abs() <= 1e-12 {
            break;
        }
    }
    (u, v)
}

fn ordered_bounds(a: f64, b: f64) -> (f64, f64) {
    if a <= b {
        (a, b)
    } else {
        (b, a)
    }
}

fn clamp_ordered(value: f64, min: f64, max: f64) -> f64 {
    let (lo, hi) = ordered_bounds(min, max);
    if value.is_nan() {
        lo
    } else {
        value.max(lo).min(hi)
    }
}

/// A mutable staging B-Rep builder.
#[derive(Clone, Debug, Default)]
pub struct BRepBuilder {
    brep: BRep,
}

impl BRepBuilder {
    /// Create an empty BRep staging builder.
    #[inline]
    pub fn new() -> Self {
        Self { brep: BRep::new() }
    }

    /// Create a staging builder from an existing BRep.
    #[inline]
    pub fn from_brep(brep: BRep) -> Self {
        Self { brep }
    }

    /// Seal the staging builder and return an immutable BRep.
    #[inline]
    pub fn build(self) -> Arc<BRep> {
        Arc::new(self.brep)
    }

    /// Access the underlying BRep read-only.
    #[inline]
    pub fn brep(&self) -> &BRep {
        &self.brep
    }

    /// Access the underlying BRep mutably.
    #[inline]
    pub fn brep_mut(&mut self) -> &mut BRep {
        &mut self.brep
    }

    /// Split an edge into two edges at a parameter `t` using an existing or new vertex `new_v`.
    ///
    /// Replaces the old edge `edge_id` in all referencing loops with the two new edges
    /// in the correct sequence depending on loop/edge traversal orientation.
    pub fn split_edge(&mut self, edge_id: EdgeId, new_v: VertexId, t: f64) -> (EdgeId, EdgeId) {
        let orig = self
            .brep
            .edges
            .get(edge_id)
            .expect("split_edge: edge not found")
            .clone();

        // 1. Insert the two new sub-edges.
        let e1_data = EdgeData {
            curve: orig.curve.clone(),
            first: orig.first,
            last: t,
            start: orig.start,
            end: new_v,
            tolerance: orig.tolerance,
        };

        let e2_data = EdgeData {
            curve: orig.curve.clone(),
            first: t,
            last: orig.last,
            start: new_v,
            end: orig.end,
            tolerance: orig.tolerance,
        };

        let e1_id = self.brep.edges.insert(e1_data);
        let e2_id = self.brep.edges.insert(e2_data);

        // 2. Replace the original edge in every loop with its two sub-edges, in
        //    the order that keeps that *specific* loop connected.
        //
        //    A single edge may be shared by two loops traversed in opposite
        //    directions (e.g. the seam between two cylinder wall faces). Because
        //    `EdgeData` stores only one orientation, we cannot rely on it to pick
        //    the sub-edge order — instead we look at the preceding edge in each
        //    loop and emit `[start-side, end-side]` or its reverse so the chain
        //    `prev -> e1 -> e2 -> next` (or `prev -> e2 -> e1 -> next`) stays
        //    contiguous. This is what makes splitting shared seams robust.
        let loop_ids: Vec<_> = self
            .brep
            .loops
            .iter()
            .filter(|(_, l)| l.edges.iter().any(|oe| oe.id == edge_id))
            .map(|(id, _)| id)
            .collect();
        for lid in loop_ids {
            let old = self.brep.loops[lid].edges.clone();
            let n = old.len();
            let mut new_edges = Vec::with_capacity(n + 1);
            for &oe in &old {
                if oe.id != edge_id {
                    new_edges.push(oe);
                    continue;
                }
                if oe.orientation == Orientation::Reversed {
                    new_edges.push(OrientedEdge {
                        id: e2_id,
                        orientation: Orientation::Reversed,
                    });
                    new_edges.push(OrientedEdge {
                        id: e1_id,
                        orientation: Orientation::Reversed,
                    });
                } else {
                    new_edges.push(OrientedEdge {
                        id: e1_id,
                        orientation: Orientation::Forward,
                    });
                    new_edges.push(OrientedEdge {
                        id: e2_id,
                        orientation: Orientation::Forward,
                    });
                }
            }
            self.brep.loops[lid].edges = new_edges;
        }

        // 3. Remove the original edge.
        self.brep.edges.remove(edge_id);

        (e1_id, e2_id)
    }

    /// Split a face into two faces along a path of splitting edges.
    ///
    /// The splitting edges must form a simple path connecting two vertices on the outer loop of the face.
    /// Distributes any inner loops (holes) of the original face to the correct new face using 2D parameter-space containment.
    pub fn split_face(&mut self, face_id: FaceId, splitting_edges: &[EdgeId]) -> (FaceId, FaceId) {
        let face_data = self
            .brep
            .faces
            .get(face_id)
            .expect("split_face: face not found")
            .clone();
        let outer_loop_id = face_data
            .outer_wire
            .expect("split_face: face has no outer wire");
        let outer_loop = self
            .brep
            .loops
            .get(outer_loop_id)
            .expect("split_face: loop not found")
            .clone();

        let get_edge_endpoints = |brep: &BRep, oe: OrientedEdge| {
            let e = &brep.edges[oe.id];
            match oe.orientation {
                Orientation::Reversed => (e.end, e.start),
                _ => (e.start, e.end),
            }
        };

        // 1. Trace the vertices of the outer loop in order.
        let outer_edges = &outer_loop.edges;
        let mut outer_vertices = Vec::with_capacity(outer_edges.len());
        for &oe in outer_edges {
            let (start, _) = get_edge_endpoints(&self.brep, oe);
            outer_vertices.push(start);
        }

        // 2. Find endpoints of the splitting path (V_A and V_B).
        let first_split_edge = splitting_edges[0];
        let last_split_edge = splitting_edges[splitting_edges.len() - 1];
        let (v_a, _) = get_edge_endpoints(
            &self.brep,
            OrientedEdge {
                id: first_split_edge,
                orientation: Orientation::Forward,
            },
        );
        let (_, v_b) = get_edge_endpoints(
            &self.brep,
            OrientedEdge {
                id: last_split_edge,
                orientation: Orientation::Forward,
            },
        );

        let idx_a = outer_vertices
            .iter()
            .position(|&v| v == v_a)
            .expect("split_face: V_A not found on outer loop");
        let idx_b = outer_vertices
            .iter()
            .position(|&v| v == v_b)
            .expect("split_face: V_B not found on outer loop");

        let mut loop1_edges = Vec::new();
        let mut loop2_edges = Vec::new();

        let n = outer_edges.len();

        // Path 1: From idx_a to idx_b along the outer loop.
        let mut curr = idx_a;
        while curr != idx_b {
            loop1_edges.push(outer_edges[curr]);
            curr = (curr + 1) % n;
        }

        // Path 2: From idx_b to idx_a along the outer loop.
        let mut curr = idx_b;
        while curr != idx_a {
            loop2_edges.push(outer_edges[curr]);
            curr = (curr + 1) % n;
        }

        // 3. Connect split loops using the splitting path.
        // Loop 1 needs to go from v_b back to v_a: add reversed splitting edges.
        for &e_id in splitting_edges.iter().rev() {
            loop1_edges.push(OrientedEdge {
                id: e_id,
                orientation: Orientation::Reversed,
            });
        }

        // Loop 2 needs to go from v_a to v_b: add forward splitting edges.
        for &e_id in splitting_edges {
            loop2_edges.push(OrientedEdge {
                id: e_id,
                orientation: Orientation::Forward,
            });
        }

        // 4. Create new LoopIds in the BRep.
        let loop1_id = self.brep.loops.insert(LoopData { edges: loop1_edges });
        let loop2_id = self.brep.loops.insert(LoopData { edges: loop2_edges });

        // 5. Distribute holes (inner loops) using 2D point-in-polygon checks on planar surfaces.
        let mut face1_inners = Vec::new();
        let mut face2_inners = Vec::new();

        if let Some(GeomSurface::Plane(plane)) = &face_data.surface {
            // Reconstruct Loop 1 outer boundary in 2D parametric coordinates.
            let mut loop1_poly = Vec::new();
            for &oe in &self.brep.loops[loop1_id].edges {
                let (start_v, _) = get_edge_endpoints(&self.brep, oe);
                let p = self.brep.vertices[start_v].point;
                let diff = p - plane.location();
                let u = diff.dot(&openrcad_foundation::Vec::from_dir(
                    plane.position().x_direction(),
                ));
                let v = diff.dot(&openrcad_foundation::Vec::from_dir(
                    plane.position().y_direction(),
                ));
                loop1_poly.push((u, v));
            }

            for &inner_loop_id in &face_data.inner_wires {
                let inner_loop = &self.brep.loops[inner_loop_id];
                if let Some(&first_edge) = inner_loop.edges.first() {
                    let (start_v, _) = get_edge_endpoints(&self.brep, first_edge);
                    let p = self.brep.vertices[start_v].point;
                    let diff = p - plane.location();
                    let u = diff.dot(&openrcad_foundation::Vec::from_dir(
                        plane.position().x_direction(),
                    ));
                    let v = diff.dot(&openrcad_foundation::Vec::from_dir(
                        plane.position().y_direction(),
                    ));

                    if point_in_polygon_2d((u, v), &loop1_poly) {
                        face1_inners.push(inner_loop_id);
                    } else {
                        face2_inners.push(inner_loop_id);
                    }
                }
            }
        } else {
            // Fallback: allocate all inner wires to Face 1.
            face1_inners.extend(&face_data.inner_wires);
        }

        // 6. Create the two new faces.
        let face1_data = FaceData {
            surface: face_data.surface.clone(),
            outer_wire: Some(loop1_id),
            inner_wires: face1_inners,
            orientation: face_data.orientation,
        };
        let face2_data = FaceData {
            surface: face_data.surface.clone(),
            outer_wire: Some(loop2_id),
            inner_wires: face2_inners,
            orientation: face_data.orientation,
        };

        let face1_id = self.brep.faces.insert(face1_data);
        let face2_id = self.brep.faces.insert(face2_data);

        // 7. Remove original face and outer loop.
        self.brep.loops.remove(outer_loop_id);
        self.brep.faces.remove(face_id);

        // 8. Update all shells referencing the original face.
        for (_, shell_data) in &mut self.brep.shells {
            let mut new_faces = Vec::with_capacity(shell_data.faces.len() + 1);
            for &f in &shell_data.faces {
                if f == face_id {
                    new_faces.push(face1_id);
                    new_faces.push(face2_id);
                } else {
                    new_faces.push(f);
                }
            }
            shell_data.faces = new_faces;
        }

        (face1_id, face2_id)
    }

    /// Partition a face into N faces along a network of splitting edges.
    ///
    /// The splitting edges can form arbitrary networks of edges (e.g. sharing internal vertices),
    /// partitioning the face into multiple regions.
    /// Distributes any inner loops (holes) of the original face to the correct new face.
    pub fn partition_face(&mut self, face_id: FaceId, splitting_edges: &[EdgeId]) -> Vec<FaceId> {
        let debug =
            std::env::var_os("OPENRCAD_BOOLEAN_DEBUG").is_some() && !splitting_edges.is_empty();
        let face_data = self
            .brep
            .faces
            .get(face_id)
            .expect("partition_face: face not found")
            .clone();
        let outer_loop_id = face_data
            .outer_wire
            .expect("partition_face: face has no outer wire");
        let outer_loop = self
            .brep
            .loops
            .get(outer_loop_id)
            .expect("partition_face: loop not found")
            .clone();
        let surface = face_data
            .surface
            .as_ref()
            .expect("partition_face: face has no surface");
        if debug {
            eprintln!(
                "partition start face={face_id:?} surface={} outer_edges={} split_edges={}",
                surface_debug_name(surface),
                outer_loop.edges.len(),
                splitting_edges.len()
            );
            for &e_id in splitting_edges {
                eprintln!(
                    "  split {}",
                    edge_debug_line(
                        &self.brep,
                        OrientedEdge {
                            id: e_id,
                            orientation: Orientation::Forward,
                        }
                    )
                );
            }
        }

        let get_edge_endpoints = |brep: &BRep, oe: OrientedEdge| {
            let e = &brep.edges[oe.id];
            match oe.orientation {
                Orientation::Reversed => (e.end, e.start),
                _ => (e.start, e.end),
            }
        };

        // 1. Gather all edges (outer boundary + splitting edges) and build both Forward and Reversed half-edges
        // so that the graph is symmetric and every edge is traversed in both directions (avoiding dead ends / bijections breaking).
        let mut edges_pool = Vec::new();
        edges_pool.extend(outer_loop.edges.iter().map(|oe| oe.id));
        edges_pool.extend(splitting_edges.iter().copied());

        let mut half_edges = Vec::new();
        for e_id in edges_pool {
            half_edges.push(OrientedEdge {
                id: e_id,
                orientation: Orientation::Forward,
            });
            half_edges.push(OrientedEdge {
                id: e_id,
                orientation: Orientation::Reversed,
            });
        }

        // 2. Build adjacency mapping of outgoing half-edges from each vertex.
        let mut adjacency: std::collections::HashMap<VertexId, Vec<OrientedEdge>> =
            std::collections::HashMap::new();
        for &oe in &half_edges {
            let (start, _) = get_edge_endpoints(&self.brep, oe);
            adjacency.entry(start).or_default().push(oe);
        }

        let periodic = matches!(surface, openrcad_geom::GeomSurface::Cylinder(_));
        let u_anchor = outer_loop
            .edges
            .first()
            .and_then(|&oe| {
                let (v_start, _) = get_edge_endpoints(&self.brep, oe);
                let pt = self.brep.vertices.get(v_start)?.point;
                Some(
                    analytic_surface_uv(surface, pt)
                        .unwrap_or_else(|| search_nearest_parameter(surface, pt))
                        .0,
                )
            })
            .unwrap_or(0.0);

        // Helper to project 3D point to 2D parametric UV coordinates. Analytic
        // cylinders are periodic in `u`, so align every projected point to the
        // same angular branch as this face's outer loop before doing planar graph
        // tracing.
        let project_point_on_surface =
            |pt: openrcad_foundation::Pnt, s: &openrcad_geom::GeomSurface| -> (f64, f64) {
                let (mut u, v) =
                    analytic_surface_uv(s, pt).unwrap_or_else(|| search_nearest_parameter(s, pt));
                if periodic {
                    while u - u_anchor > PI {
                        u -= 2.0 * PI;
                    }
                    while u_anchor - u > PI {
                        u += 2.0 * PI;
                    }
                }
                (u, v)
            };

        // Helper to get polar angle of outgoing tangent direction of oriented edge at its start vertex
        let get_tangent_angle =
            |brep: &BRep, oe: OrientedEdge, surf: &openrcad_geom::GeomSurface| -> f64 {
                let e = &brep.edges[oe.id];
                let curve = e.curve.as_ref().expect("partition_face: edge has no curve");
                let (first, last) = (e.first, e.last);
                let (t_start, _) = match oe.orientation {
                    Orientation::Reversed => (last, first),
                    _ => (first, last),
                };
                let dt = 1e-4 * (last - first);
                let t_step = if oe.orientation == Orientation::Reversed {
                    t_start - dt
                } else {
                    t_start + dt
                };
                let p0 = curve.point(t_start);
                let p1 = curve.point(t_step);
                let uv0 = project_point_on_surface(p0, surf);
                let uv1 = project_point_on_surface(p1, surf);
                (uv1.1 - uv0.1).atan2(uv1.0 - uv0.0)
            };

        // Sort outgoing half-edges counter-clockwise by polar angle of outgoing tangent direction
        for list in adjacency.values_mut() {
            let self_brep = &self.brep;
            list.sort_by(|&a, &b| {
                let angle_a = get_tangent_angle(self_brep, a, surface);
                let angle_b = get_tangent_angle(self_brep, b, surface);
                angle_a
                    .partial_cmp(&angle_b)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
        }

        // 3. Trace closed loops (faces) using the rotation system
        let mut new_loop_edges_list = Vec::new();
        let mut visited_edges = std::collections::HashSet::new();

        for &start_he in &half_edges {
            if visited_edges.contains(&start_he) {
                continue;
            }

            let mut loop_edges = Vec::new();
            let mut curr_he = start_he;

            loop {
                visited_edges.insert(curr_he);
                loop_edges.push(curr_he);

                let (_, end_v) = get_edge_endpoints(&self.brep, curr_he);
                let list = match adjacency.get(&end_v) {
                    Some(l) => l,
                    None => break, // dead end
                };
                let k = list.len();
                if k == 0 {
                    break;
                }

                // Find the opposite of the current half-edge (starts at end_v)
                let opp = OrientedEdge {
                    id: curr_he.id,
                    orientation: curr_he.orientation.reversed(),
                };

                // Find the index of opp in the sorted outgoing list at end_v.
                // Fallback: if not found, pick the first.
                let j = list.iter().position(|&oe| oe == opp).unwrap_or_default();

                // The successor of curr_he is the next outgoing edge clockwise from opp
                // which is index (j - 1) mod k in the counter-clockwise sorted list
                let next_idx = if j == 0 { k - 1 } else { j - 1 };
                let next_he = list[next_idx];

                if next_he == start_he {
                    break;
                }
                curr_he = next_he;
            }

            if !loop_edges.is_empty() {
                if debug {
                    let mut ids = std::collections::HashSet::new();
                    let repeated = loop_edges.iter().filter(|oe| !ids.insert(oe.id)).count();
                    let closed = loop_edges.last().is_some_and(|&last| {
                        let (_, end) = get_edge_endpoints(&self.brep, last);
                        let (start, _) = get_edge_endpoints(&self.brep, start_he);
                        end == start
                    });
                    eprintln!(
                        "  traced loop edges={} repeated_edges={} closed={closed}",
                        loop_edges.len(),
                        repeated
                    );
                    for &oe in &loop_edges {
                        eprintln!("    {}", edge_debug_line(&self.brep, oe));
                    }
                }
                new_loop_edges_list.push(loop_edges);
            }
        }

        let mut orig_poly = Vec::new();
        for &oe in &outer_loop.edges {
            let (v_start, _) = get_edge_endpoints(&self.brep, oe);
            let p = self.brep.vertices[v_start].point;
            orig_poly.push(project_point_on_surface(p, surface));
        }
        let mut orig_area = 0.0;
        if orig_poly.len() >= 3 {
            for i in 0..orig_poly.len() {
                let (x1, y1) = orig_poly[i];
                let (x2, y2) = orig_poly[(i + 1) % orig_poly.len()];
                orig_area += x1 * y2 - x2 * y1;
            }
        }
        let orig_sign = orig_area.signum();

        let mut inner_loop_edges_list = Vec::new();
        for loop_edges in new_loop_edges_list {
            let mut has_outer_edges = false;
            let mut loop_is_reversed = false;
            for &oe in &loop_edges {
                if let Some(orig_oe) = outer_loop.edges.iter().find(|o| o.id == oe.id) {
                    has_outer_edges = true;
                    if oe.orientation == orig_oe.orientation.reversed() {
                        loop_is_reversed = true;
                        break;
                    }
                }
            }
            let is_exterior = if has_outer_edges {
                loop_is_reversed == (orig_sign > 0.0)
            } else {
                false
            };
            if !is_exterior {
                // Calculate area to check for degeneracy
                let mut poly = Vec::new();
                for &oe in &loop_edges {
                    let (v_start, _) = get_edge_endpoints(&self.brep, oe);
                    let p = self.brep.vertices[v_start].point;
                    poly.push(project_point_on_surface(p, surface));
                }
                let mut area = 0.0;
                if poly.len() >= 3 {
                    for i in 0..poly.len() {
                        let (x1, y1) = poly[i];
                        let (x2, y2) = poly[(i + 1) % poly.len()];
                        area += x1 * y2 - x2 * y1;
                    }
                }
                area *= 0.5;
                if debug {
                    eprintln!(
                        "  candidate loop edges={} area={area:.8} exterior={is_exterior}",
                        loop_edges.len()
                    );
                }
                if area.abs() >= 1e-5 {
                    inner_loop_edges_list.push(loop_edges);
                }
            } else if debug {
                eprintln!("  candidate loop edges={} exterior=true", loop_edges.len());
            }
        }

        let mut loop_polys = Vec::new();
        let mut new_loop_ids = Vec::new();

        for new_edges in &inner_loop_edges_list {
            let l_id = self.brep.loops.insert(LoopData {
                edges: new_edges.clone(),
            });
            new_loop_ids.push(l_id);

            // Reconstruct outer boundary polygon in UV space
            let mut poly = Vec::new();
            for &oe in new_edges {
                let (v_start, _) = get_edge_endpoints(&self.brep, oe);
                let p = self.brep.vertices[v_start].point;
                poly.push(project_point_on_surface(p, surface));
            }
            loop_polys.push(poly);
        }

        // Initialize classification arrays
        let n_loops = new_loop_ids.len();
        let mut loop_areas = Vec::new();
        for poly in &loop_polys {
            let mut area = 0.0;
            if poly.len() >= 3 {
                for i in 0..poly.len() {
                    let (x1, y1) = poly[i];
                    let (x2, y2) = poly[(i + 1) % poly.len()];
                    area += x1 * y2 - x2 * y1;
                }
            }
            loop_areas.push(0.5 * area);
        }

        // Find containers for each loop
        let mut containers = vec![Vec::new(); n_loops];
        for i in 0..n_loops {
            let set_i: std::collections::HashSet<_> =
                inner_loop_edges_list[i].iter().map(|oe| oe.id).collect();
            for j in 0..n_loops {
                if i != j {
                    let set_j: std::collections::HashSet<_> =
                        inner_loop_edges_list[j].iter().map(|oe| oe.id).collect();
                    if set_i == set_j {
                        continue;
                    }
                    // Check if loop i is inside loop j
                    let oe = inner_loop_edges_list[i][0];
                    let e = &self.brep.edges[oe.id];
                    let mid_t = 0.5 * (e.first + e.last);
                    let mid_p = e.curve.as_ref().unwrap().point(mid_t);
                    let (_, tangent) = e.curve.as_ref().unwrap().d1(mid_t);
                    let tangent = if oe.orientation == Orientation::Reversed {
                        -tangent
                    } else {
                        tangent
                    };

                    let normal = match surface {
                        openrcad_geom::GeomSurface::Plane(plane) => plane.normal(),
                        _ => {
                            if let openrcad_geom::GeomSurface::Cylinder(cyl) = surface {
                                let axis_pt = cyl.position().axis().location();
                                let axis_dir = openrcad_foundation::Vec::from_dir(
                                    cyl.position().axis().direction(),
                                );
                                let diff = mid_p - axis_pt;
                                let proj = axis_pt + axis_dir * diff.dot(&axis_dir);
                                (mid_p - proj)
                                    .normalized()
                                    .unwrap_or(openrcad_foundation::Dir::dz())
                            } else {
                                openrcad_foundation::Dir::dz()
                            }
                        }
                    };
                    let left_dir = tangent
                        .cross(&openrcad_foundation::Vec::from_dir(normal))
                        .normalized()
                        .unwrap();
                    let left_vec = openrcad_foundation::Vec::from_dir(left_dir);
                    let probe_p = mid_p + left_vec * 1e-3;
                    let uv = project_point_on_surface(probe_p, surface);

                    if point_in_polygon_2d(uv, &loop_polys[j]) {
                        containers[i].push(j);
                    }
                }
            }
        }

        let mut is_face = vec![true; n_loops];
        let mut hole_to_face = std::collections::HashMap::new();

        for i in 0..n_loops {
            if !containers[i].is_empty() {
                // Find the direct container (parent), which is the container with the largest number of containers
                let &parent_idx = containers[i]
                    .iter()
                    .max_by_key(|&&c_idx| containers[c_idx].len())
                    .unwrap();

                // If the loop's sign is opposite to the parent's sign, it's a hole of the parent
                let parent_area = loop_areas[parent_idx];
                let current_area = loop_areas[i];
                if parent_area.signum() != current_area.signum() {
                    is_face[i] = false;
                    hole_to_face.insert(i, parent_idx);
                }
            }
        }

        // Initialize inner wires arrays for each loop
        let mut distributed_inners = vec![Vec::new(); n_loops];

        // Distribute original face's inner wires to the new faces
        for &inner_loop_id in &face_data.inner_wires {
            let inner_loop = &self.brep.loops[inner_loop_id];
            if let Some(&first_edge) = inner_loop.edges.first() {
                let (start_v, _) = get_edge_endpoints(&self.brep, first_edge);
                let p = self.brep.vertices[start_v].point;
                let uv = project_point_on_surface(p, surface);

                // Find which face loop contains this hole
                for idx in 0..n_loops {
                    if is_face[idx] && point_in_polygon_2d(uv, &loop_polys[idx]) {
                        distributed_inners[idx].push(inner_loop_id);
                        break;
                    }
                }
            }
        }

        // Distribute the new hole loops to their containing faces
        for i in 0..n_loops {
            if !is_face[i] {
                if let Some(&face_idx) = hole_to_face.get(&i) {
                    distributed_inners[face_idx].push(new_loop_ids[i]);
                }
            }
        }

        // 5. Create new faces (only for loops classified as faces)
        let mut new_face_ids = Vec::new();
        for idx in 0..n_loops {
            if is_face[idx] {
                let loop_id = new_loop_ids[idx];
                let face_data = FaceData {
                    surface: face_data.surface.clone(),
                    outer_wire: Some(loop_id),
                    inner_wires: distributed_inners[idx].clone(),
                    orientation: face_data.orientation,
                };
                let f_id = self.brep.faces.insert(face_data);
                if debug {
                    eprintln!(
                        "  emitted face={f_id:?} loop_edges={} area={:.8} inners={}",
                        inner_loop_edges_list[idx].len(),
                        loop_areas[idx],
                        distributed_inners[idx].len()
                    );
                }
                new_face_ids.push(f_id);
            }
        }

        // 6. Remove original face and outer loop
        self.brep.loops.remove(outer_loop_id);
        self.brep.faces.remove(face_id);

        // 7. Update shells
        for (_, shell_data) in &mut self.brep.shells {
            let mut updated_faces = Vec::with_capacity(shell_data.faces.len() + new_face_ids.len());
            for &fid in &shell_data.faces {
                if fid == face_id {
                    updated_faces.extend(new_face_ids.iter().copied());
                } else {
                    updated_faces.push(fid);
                }
            }
            shell_data.faces = updated_faces;
        }

        new_face_ids
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::edge::Edge;
    use crate::face::Face;
    use crate::wire::Wire;
    use openrcad_foundation::Pnt;
    use openrcad_geom::{GeomSurface, Plane};

    #[test]
    fn ordered_clamp_accepts_reversed_surface_bounds() {
        assert_eq!(clamp_ordered(2.12, 2.14, 2.09), 2.12);
        assert_eq!(clamp_ordered(2.00, 2.14, 2.09), 2.09);
        assert_eq!(clamp_ordered(2.20, 2.14, 2.09), 2.14);
    }

    #[test]
    fn test_split_edge() {
        let e = Edge::between_points(Pnt::new(0.0, 0.0, 0.0), Pnt::new(10.0, 0.0, 0.0));
        let w = Wire::from_edges([e.clone()]);
        let face = Face::new(None, w);

        let mut builder = BRepBuilder::from_brep((*face.brep).clone());
        let edge_id = builder.brep.edges.keys().next().unwrap();
        let loop_id = builder.brep.loops.keys().next().unwrap();

        // Create a new vertex at (5, 0, 0)
        let new_v = builder.brep.vertices.insert(crate::arena::VertexData {
            point: Pnt::new(5.0, 0.0, 0.0),
            tolerance: openrcad_foundation::tolerance::CONFUSION,
        });

        // Split edge at parameter 5.0 (midpoint)
        let (e1, e2) = builder.split_edge(edge_id, new_v, 5.0);

        // Verify that original edge is gone and the loop contains the two sub-edges.
        assert!(!builder.brep.edges.contains_key(edge_id));
        assert!(builder.brep.edges.contains_key(e1));
        assert!(builder.brep.edges.contains_key(e2));

        let l_edges = &builder.brep.loops[loop_id].edges;
        assert_eq!(l_edges.len(), 2);
        assert_eq!(l_edges[0].id, e1);
        assert_eq!(l_edges[1].id, e2);

        // Verify geometry bounds of sub-edges
        assert_eq!(builder.brep.edges[e1].first, 0.0);
        assert_eq!(builder.brep.edges[e1].last, 5.0);
        assert_eq!(builder.brep.edges[e2].first, 5.0);
        assert_eq!(builder.brep.edges[e2].last, 10.0);
    }

    #[test]
    fn split_edge_keeps_both_loops_contiguous_across_a_shared_seam() {
        // A single edge `e` (a -> b) shared by two triangular loops, used in
        // OPPOSITE senses: Forward in loop1 (a,b,c), Reversed in loop2 (b,a,d).
        // This is the cylinder-seam case. Splitting `e` must keep BOTH loops
        // tracing cleanly, which only works if orientation is a per-use property
        // of each co-edge rather than a single value stored on the shared edge.
        use crate::arena::{LoopData, OrientedEdge, VertexData};
        use openrcad_foundation::tolerance::CONFUSION;

        let mut brep = BRep::new();
        let mk_v = |brep: &mut BRep, x: f64| {
            brep.vertices.insert(VertexData {
                point: Pnt::new(x, 0.0, 0.0),
                tolerance: CONFUSION,
            })
        };
        let a = mk_v(&mut brep, 0.0);
        let b = mk_v(&mut brep, 10.0);
        let c = mk_v(&mut brep, 5.0);
        let d = mk_v(&mut brep, -5.0);

        let mk_e = |brep: &mut BRep, s, t| {
            brep.edges.insert(EdgeData {
                curve: None,
                first: 0.0,
                last: 1.0,
                start: s,
                end: t,
                tolerance: CONFUSION,
            })
        };
        let e = mk_e(&mut brep, a, b); // the shared seam
        let bc = mk_e(&mut brep, b, c);
        let ca = mk_e(&mut brep, c, a);
        let ad = mk_e(&mut brep, a, d);
        let db = mk_e(&mut brep, d, b);

        let fwd = |id| OrientedEdge {
            id,
            orientation: Orientation::Forward,
        };
        let rev = |id| OrientedEdge {
            id,
            orientation: Orientation::Reversed,
        };
        let loop1 = brep.loops.insert(LoopData {
            edges: vec![fwd(e), fwd(bc), fwd(ca)],
        });
        let loop2 = brep.loops.insert(LoopData {
            edges: vec![rev(e), fwd(ad), fwd(db)],
        });

        let mut builder = BRepBuilder::from_brep(brep);
        let m = builder.brep.vertices.insert(VertexData {
            point: Pnt::new(5.0, 0.0, 0.0),
            tolerance: CONFUSION,
        });
        builder.split_edge(e, m, 0.5);

        // Directed endpoints of a co-edge, honoring its per-use orientation.
        let dir_ends = |brep: &BRep, oe: OrientedEdge| {
            let ed = &brep.edges[oe.id];
            match oe.orientation {
                Orientation::Reversed => (ed.end, ed.start),
                _ => (ed.start, ed.end),
            }
        };
        let assert_contiguous = |brep: &BRep, lid| {
            let edges = &brep.loops[lid].edges;
            assert_eq!(edges.len(), 4, "seam split should grow the loop to 4 edges");
            for i in 0..edges.len() {
                let (_, end) = dir_ends(brep, edges[i]);
                let (next_start, _) = dir_ends(brep, edges[(i + 1) % edges.len()]);
                assert_eq!(
                    end, next_start,
                    "loop edge {i} does not connect to the next"
                );
            }
        };
        assert_contiguous(&builder.brep, loop1);
        assert_contiguous(&builder.brep, loop2);
    }

    #[test]
    fn test_split_face() {
        // Create a planar square face [0, 10] x [0, 10] in XY plane.
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

        let plane = GeomSurface::plane(Plane::from_point_normal(
            Pnt::origin(),
            openrcad_foundation::Dir::dz(),
        ));
        let face = Face::new(Some(plane), w);

        let mut builder = BRepBuilder::from_brep((*face.brep).clone());
        let face_id = builder.brep.faces.keys().next().unwrap();

        // Let's create a splitting edge from (5, 0, 0) to (5, 10, 0) dividing the square in half.
        // We must first split the bottom and top boundary edges so that we have vertices at the split endpoints.
        let bottom_edge_id = builder
            .brep
            .edges
            .iter()
            .find(|(_, data)| {
                let p = builder.brep.vertices[data.start].point;
                (p.x() - p0.x()).abs() < 1e-5 && (p.y() - p0.y()).abs() < 1e-5
            })
            .map(|(k, _)| k)
            .unwrap();

        let top_edge_id = builder
            .brep
            .edges
            .iter()
            .find(|(_, data)| {
                let p = builder.brep.vertices[data.start].point;
                (p.x() - p2.x()).abs() < 1e-5 && (p.y() - p2.y()).abs() < 1e-5
            })
            .map(|(k, _)| k)
            .unwrap();

        let v_bottom_split = builder.brep.vertices.insert(crate::arena::VertexData {
            point: Pnt::new(5.0, 0.0, 0.0),
            tolerance: openrcad_foundation::tolerance::CONFUSION,
        });
        let v_top_split = builder.brep.vertices.insert(crate::arena::VertexData {
            point: Pnt::new(5.0, 10.0, 0.0),
            tolerance: openrcad_foundation::tolerance::CONFUSION,
        });

        // Split the bottom and top edges.
        let (_, _) = builder.split_edge(bottom_edge_id, v_bottom_split, 5.0);
        let (_, _) = builder.split_edge(top_edge_id, v_top_split, 5.0);

        // Now insert the splitting edge from v_bottom_split to v_top_split.
        let split_edge = Edge::between_points(Pnt::new(5.0, 0.0, 0.0), Pnt::new(5.0, 10.0, 0.0));
        let map_split = builder.brep.merge(&split_edge.brep);
        let new_split_edge_id = map_split.edges[&split_edge.id];

        // Now split the face!
        let (f1, f2) = builder.split_face(face_id, &[new_split_edge_id]);

        // Verify that original face is removed, and we have two new faces.
        assert!(!builder.brep.faces.contains_key(face_id));
        assert!(builder.brep.faces.contains_key(f1));
        assert!(builder.brep.faces.contains_key(f2));

        // Verify loop counts of the new faces.
        let w1_id = builder.brep.faces[f1].outer_wire.unwrap();
        let w2_id = builder.brep.faces[f2].outer_wire.unwrap();
        assert_eq!(builder.brep.loops[w1_id].edges.len(), 4);
        assert_eq!(builder.brep.loops[w2_id].edges.len(), 4);
    }

    #[test]
    fn test_partition_face_3_anchor() {
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

        let plane = GeomSurface::plane(Plane::from_point_normal(
            Pnt::origin(),
            openrcad_foundation::Dir::dz(),
        ));
        let face = Face::new(Some(plane), w);

        let mut builder = BRepBuilder::from_brep((*face.brep).clone());
        let face_id = builder.brep.faces.keys().next().unwrap();

        // Find the top edge (from (10,10) to (0,10)) and split it at (5, 10)
        let top_edge_id = builder
            .brep
            .edges
            .iter()
            .find(|(_, data)| {
                let p = builder.brep.vertices[data.start].point;
                (p.x() - p2.x()).abs() < 1e-5 && (p.y() - p2.y()).abs() < 1e-5
            })
            .map(|(k, _)| k)
            .unwrap();

        let v_top_split = builder.brep.vertices.insert(crate::arena::VertexData {
            point: Pnt::new(5.0, 10.0, 0.0),
            tolerance: openrcad_foundation::tolerance::CONFUSION,
        });

        builder.split_edge(top_edge_id, v_top_split, 5.0);

        // Get the corner vertex ids
        let v0 = builder
            .brep
            .vertices
            .iter()
            .find(|(_, d)| d.point.distance(&p0) < 1e-5)
            .map(|(k, _)| k)
            .unwrap();
        let v1 = builder
            .brep
            .vertices
            .iter()
            .find(|(_, d)| d.point.distance(&p1) < 1e-5)
            .map(|(k, _)| k)
            .unwrap();

        // Create the center vertex
        let v_center = builder.brep.vertices.insert(crate::arena::VertexData {
            point: Pnt::new(5.0, 5.0, 0.0),
            tolerance: openrcad_foundation::tolerance::CONFUSION,
        });

        // Add 3 splitting edges connecting center to v0, v1, and v_top_split
        let e_c0 = Edge::between_points(Pnt::new(5.0, 5.0, 0.0), p0);
        let e_c1 = Edge::between_points(Pnt::new(5.0, 5.0, 0.0), p1);
        let e_ct = Edge::between_points(Pnt::new(5.0, 5.0, 0.0), Pnt::new(5.0, 10.0, 0.0));

        let map_c0 = builder.brep.merge(&e_c0.brep);
        let map_c1 = builder.brep.merge(&e_c1.brep);
        let map_ct = builder.brep.merge(&e_ct.brep);

        let e_c0_id = map_c0.edges[&e_c0.id];
        let e_c1_id = map_c1.edges[&e_c1.id];
        let e_ct_id = map_ct.edges[&e_ct.id];

        // We must update the newly merged edge vertices to be the exact same shared vertices
        builder.brep.edges.get_mut(e_c0_id).unwrap().start = v_center;
        builder.brep.edges.get_mut(e_c0_id).unwrap().end = v0;

        builder.brep.edges.get_mut(e_c1_id).unwrap().start = v_center;
        builder.brep.edges.get_mut(e_c1_id).unwrap().end = v1;

        builder.brep.edges.get_mut(e_ct_id).unwrap().start = v_center;
        builder.brep.edges.get_mut(e_ct_id).unwrap().end = v_top_split;

        let splitting_edges = vec![e_c0_id, e_c1_id, e_ct_id];

        // Partition!
        let new_faces = builder.partition_face(face_id, &splitting_edges);

        // Verify we got exactly 3 faces
        assert_eq!(new_faces.len(), 3);
        for &fid in &new_faces {
            assert!(builder.brep.faces.contains_key(fid));
        }
    }

    #[test]
    fn test_partition_face_4_anchor() {
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

        let plane = GeomSurface::plane(Plane::from_point_normal(
            Pnt::origin(),
            openrcad_foundation::Dir::dz(),
        ));
        let face = Face::new(Some(plane), w);

        let mut builder = BRepBuilder::from_brep((*face.brep).clone());
        let face_id = builder.brep.faces.keys().next().unwrap();

        // Find and split all 4 boundary edges at their midpoints
        let bottom_edge_id = builder
            .brep
            .edges
            .iter()
            .find(|(_, data)| {
                let p = builder.brep.vertices[data.start].point;
                (p.x() - p0.x()).abs() < 1e-5 && (p.y() - p0.y()).abs() < 1e-5
            })
            .map(|(k, _)| k)
            .unwrap();

        let right_edge_id = builder
            .brep
            .edges
            .iter()
            .find(|(_, data)| {
                let p = builder.brep.vertices[data.start].point;
                (p.x() - p1.x()).abs() < 1e-5 && (p.y() - p1.y()).abs() < 1e-5
            })
            .map(|(k, _)| k)
            .unwrap();

        let top_edge_id = builder
            .brep
            .edges
            .iter()
            .find(|(_, data)| {
                let p = builder.brep.vertices[data.start].point;
                (p.x() - p2.x()).abs() < 1e-5 && (p.y() - p2.y()).abs() < 1e-5
            })
            .map(|(k, _)| k)
            .unwrap();

        let left_edge_id = builder
            .brep
            .edges
            .iter()
            .find(|(_, data)| {
                let p = builder.brep.vertices[data.start].point;
                (p.x() - p3.x()).abs() < 1e-5 && (p.y() - p3.y()).abs() < 1e-5
            })
            .map(|(k, _)| k)
            .unwrap();

        let v_bottom_split = builder.brep.vertices.insert(crate::arena::VertexData {
            point: Pnt::new(5.0, 0.0, 0.0),
            tolerance: openrcad_foundation::tolerance::CONFUSION,
        });
        let v_right_split = builder.brep.vertices.insert(crate::arena::VertexData {
            point: Pnt::new(10.0, 5.0, 0.0),
            tolerance: openrcad_foundation::tolerance::CONFUSION,
        });
        let v_top_split = builder.brep.vertices.insert(crate::arena::VertexData {
            point: Pnt::new(5.0, 10.0, 0.0),
            tolerance: openrcad_foundation::tolerance::CONFUSION,
        });
        let v_left_split = builder.brep.vertices.insert(crate::arena::VertexData {
            point: Pnt::new(0.0, 5.0, 0.0),
            tolerance: openrcad_foundation::tolerance::CONFUSION,
        });

        builder.split_edge(bottom_edge_id, v_bottom_split, 5.0);
        builder.split_edge(right_edge_id, v_right_split, 5.0);
        builder.split_edge(top_edge_id, v_top_split, 5.0);
        builder.split_edge(left_edge_id, v_left_split, 5.0);

        // Create the center vertex
        let _v_center = builder.brep.vertices.insert(crate::arena::VertexData {
            point: Pnt::new(5.0, 5.0, 0.0),
            tolerance: openrcad_foundation::tolerance::CONFUSION,
        });

        // Add 4 splitting edges from center to midpoints
        let e_b = Edge::between_points(Pnt::new(5.0, 5.0, 0.0), Pnt::new(5.0, 0.0, 0.0));
        let e_r = Edge::between_points(Pnt::new(5.0, 5.0, 0.0), Pnt::new(10.0, 5.0, 0.0));
        let e_t = Edge::between_points(Pnt::new(5.0, 5.0, 0.0), Pnt::new(5.0, 10.0, 0.0));
        let e_l = Edge::between_points(Pnt::new(5.0, 5.0, 0.0), Pnt::new(0.0, 5.0, 0.0));

        let map_b = builder.brep.merge(&e_b.brep);
        let map_r = builder.brep.merge(&e_r.brep);
        let map_t = builder.brep.merge(&e_t.brep);
        let map_l = builder.brep.merge(&e_l.brep);

        let splitting_edges = vec![
            map_b.edges[&e_b.id],
            map_r.edges[&e_r.id],
            map_t.edges[&e_t.id],
            map_l.edges[&e_l.id],
        ];

        // Partition!
        let new_faces = builder.partition_face(face_id, &splitting_edges);

        // Verify we got exactly 4 faces
        assert_eq!(new_faces.len(), 4);
        for &fid in &new_faces {
            assert!(builder.brep.faces.contains_key(fid));
        }
    }
}
