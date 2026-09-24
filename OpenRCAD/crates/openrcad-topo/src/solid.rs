//! A topological solid — a closed volume bounded by a [`Shell`] (OCCT `TopoDS_Solid`).
//!
//! A solid is the outer boundary shell plus (optionally) internal voids. The
//! counting helpers here ([`vertex_count`](Solid::vertex_count),
//! [`edge_count`](Solid::edge_count)) **deduplicate** the entities shared between
//! adjacent faces — so a box, built from six independent four-edge faces, reports
//! 8 distinct vertices and 12 distinct edges rather than 48.

use openrcad_foundation::{BndBox, Pnt, Trsf};
use openrcad_geom::{Curve, GeomCurve, GeomSurface, Surface};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::sync::Arc;

use crate::arena::{BRep, SolidData, SolidId};
use crate::edge::Edge;
use crate::face::Face;
use crate::shell::Shell;
use crate::vertex::Vertex;

/// A solid volume.
#[derive(Clone, Debug)]
pub struct Solid {
    pub(crate) brep: Arc<BRep>,
    pub(crate) id: SolidId,
}

impl Solid {
    /// Construct a Solid handle from a shared BRep storage and Solid ID.
    #[inline]
    pub fn from_id(brep: std::sync::Arc<BRep>, id: SolidId) -> Self {
        Self { brep, id }
    }

    /// Get the Solid ID in the arena.
    #[inline]
    pub fn id(&self) -> SolidId {
        self.id
    }

    /// Get the underlying BRep storage.
    #[inline]
    pub fn brep(&self) -> &Arc<BRep> {
        &self.brep
    }

    /// A solid bounded by `outer_shell`.
    #[inline]
    pub fn new(outer_shell: Shell) -> Self {
        Self::from_shells([outer_shell]).expect("a one-shell solid is non-empty")
    }

    /// Build one material region from an outer boundary shell and any enclosed
    /// void shells. Shell order is significant: the first shell is the outer
    /// boundary and subsequent shells bound voids.
    ///
    /// Returns `None` for an empty iterator because a solid without a boundary
    /// is not a representable volume.
    pub fn from_shells<I: IntoIterator<Item = Shell>>(shells: I) -> Option<Self> {
        let mut brep = BRep::new();
        let mut merged = HashMap::<usize, (Arc<BRep>, crate::arena::MergeMap)>::new();
        let mut new_shells = Vec::new();
        for shell in shells {
            let ptr = Arc::as_ptr(&shell.brep) as usize;
            let (_, map) = merged
                .entry(ptr)
                .or_insert_with(|| (shell.brep.clone(), brep.merge(&shell.brep)));
            new_shells.push(map.shells[&shell.id]);
        }
        if new_shells.is_empty() {
            return None;
        }
        let id = brep.solids.insert(SolidData { shells: new_shells });
        Some(Self {
            brep: Arc::new(brep),
            id,
        })
    }

    /// The outer boundary shell.
    #[inline]
    pub fn shell(&self) -> Shell {
        let solid_data = &self.brep.solids[self.id];
        let shell_id = solid_data.shells[0];
        Shell {
            brep: self.brep.clone(),
            id: shell_id,
        }
    }

    /// All boundary shells, with the outer shell first followed by void shells.
    pub fn shells(&self) -> Vec<Shell> {
        self.brep.solids[self.id]
            .shells
            .iter()
            .map(|&id| Shell {
                brep: self.brep.clone(),
                id,
            })
            .collect()
    }

    /// All boundary faces across the outer and void shells.
    pub fn faces(&self) -> Vec<Face> {
        self.shells()
            .into_iter()
            .flat_map(|shell| shell.faces())
            .collect()
    }

    /// All distinct [`Vertex`] locations (coincident endpoints merged).
    pub fn vertices(&self) -> Vec<Vertex> {
        let mut out: Vec<Vertex> = Vec::new();
        let mut seen = HashSet::new();
        for face in self.faces() {
            for wire in face.wires() {
                for edge in wire.edges() {
                    for v in [edge.start(), edge.end()] {
                        if seen.insert(vertex_key(&v)) {
                            out.push(v);
                        }
                    }
                }
            }
        }
        out
    }

    /// Number of distinct vertices.
    #[inline]
    pub fn vertex_count(&self) -> usize {
        self.vertices().len()
    }

    /// All distinct [`Edge`]s (shared between two faces counted once). Two edges
    /// match when they share the same unordered endpoint pair.
    pub fn edges(&self) -> Vec<Edge> {
        let mut out: Vec<Edge> = Vec::new();
        let mut seen = HashSet::new();
        for face in self.faces() {
            for wire in face.wires() {
                for edge in wire.edges() {
                    if seen.insert(edge_key(&edge)) {
                        out.push(edge);
                    }
                }
            }
        }
        out
    }

    /// Number of distinct edges.
    #[inline]
    pub fn edge_count(&self) -> usize {
        self.edges().len()
    }

    /// Number of faces.
    #[inline]
    pub fn face_count(&self) -> usize {
        self.shells().iter().map(Shell::len).sum()
    }

    /// The axis-aligned bounding box of all vertices.
    ///
    /// Exact for polygonal solids, but it can MISS curved-surface extrema
    /// whose supporting vertices sit elsewhere — e.g. a cylinder whose seam
    /// vertices never visit one side of the axis. Use
    /// [`Solid::conservative_bounding_box`] wherever a too-small box could
    /// prove a false disjoint or under-size a tool.
    pub fn bounding_box(&self) -> BndBox {
        let mut b = BndBox::new();
        for v in self.vertices() {
            b.add(&v.point());
        }
        b
    }

    /// A guaranteed-enclosing axis-aligned box of the boundary geometry.
    ///
    /// Starts from the vertex box and adds, per face:
    /// - the exact parameter-range box of every boundary edge curve
    ///   ([`GeomCurve::interval_point`]), so rim arcs contribute their true
    ///   extremes instead of only their endpoints, and
    /// - for analytic supporting surfaces, the surface's own conservative
    ///   extent: the full-rotation band between the face's axial edge
    ///   extremes for cylinders and cones (the axial coordinate has no
    ///   interior extremum there, so the boundary range bounds the face),
    ///   the whole sphere/torus boxes, and the control-pole hull for spline
    ///   surfaces (positive weights make a NURBS point a convex combination
    ///   of its poles).
    ///
    /// The result can be slightly LARGER than the true solid. Callers may
    /// use it to reject disjointness or size tools; they must not use it to
    /// prove that interior geometry is contained. Offset, ruled, and Gregory
    /// faces use interval enclosures of their UV domains. Unknown domains
    /// yield an infinite box; finite-tool callers use the fallible variant.
    pub fn conservative_bounding_box(&self) -> BndBox {
        self.try_conservative_bounding_box().unwrap_or_else(|| {
            let mut bounds = BndBox::new();
            bounds.add(&Pnt::new(
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
            ));
            bounds.add(&Pnt::new(f64::INFINITY, f64::INFINITY, f64::INFINITY));
            bounds
        })
    }

    /// Finite enclosure, or `None` when a surface's trimmed domain cannot be
    /// bounded. Unknown bounds must never be used to prove disjointness.
    pub fn try_conservative_bounding_box(&self) -> Option<BndBox> {
        use std::f64::consts::TAU;

        let mut bounds = self.bounding_box();
        let valid = std::cell::Cell::new(true);
        let add_interval = |bounds: &mut BndBox, interval: openrcad_foundation::Interval3| {
            let (x, y, z) = (interval.x, interval.y, interval.z);
            if x.lo.is_finite()
                && x.hi.is_finite()
                && y.lo.is_finite()
                && y.hi.is_finite()
                && z.lo.is_finite()
                && z.hi.is_finite()
            {
                bounds.add(&Pnt::new(x.lo, y.lo, z.lo));
                bounds.add(&Pnt::new(x.hi, y.hi, z.hi));
            } else {
                valid.set(false);
            }
        };

        for face in self.faces() {
            // Exact boxes of this face's boundary edge spans.
            let mut face_bounds = BndBox::new();
            for wire in face.wires() {
                for edge in wire.edges() {
                    if let Some(curve) = edge.curve() {
                        add_interval(
                            &mut face_bounds,
                            curve.interval_point(edge.first(), edge.last()),
                        );
                    }
                }
            }
            let (flo, fhi) = face_bounds.corners()?;
            // Union into the running box (an empty face box has no corners and
            // was skipped above, so this face contributed at least its edges).
            bounds.add(&flo);
            bounds.add(&fhi);

            let Some(surface) = face.surface() else {
                continue;
            };
            match surface {
                GeomSurface::Cylinder(_) | GeomSurface::Cone(_) => {
                    let position = match surface {
                        GeomSurface::Cylinder(c) => c.position(),
                        GeomSurface::Cone(c) => c.position(),
                        _ => unreachable!("matched above"),
                    };
                    let location = position.location();
                    let axis = openrcad_foundation::Vec::from_dir(position.direction());
                    let e1 = openrcad_foundation::Vec::from_dir(position.x_direction());
                    let e2 = openrcad_foundation::Vec::from_dir(position.y_direction());
                    // Both surface parameters are linear in space: the axial
                    // coordinate has no interior extremum, and the angular
                    // coordinate's extrema over the trimmed region lie on the
                    // boundary. So the (already conservative) edge box bounds
                    // BOTH parameters: project its corners onto the axis for
                    // the axial range and into the (e1, e2) frame for the
                    // angular range. A partial cylinder (e.g. a fillet's
                    // quarter arc) then bounds tightly instead of as a full
                    // band — containment-style callers reject on over-loose
                    // boxes, so this matters beyond tightness.
                    let mut axial_lo = f64::INFINITY;
                    let mut axial_hi = f64::NEG_INFINITY;
                    let mut u1_range = [f64::INFINITY, f64::NEG_INFINITY];
                    let mut u2_range = [f64::INFINITY, f64::NEG_INFINITY];
                    for x in [flo.x(), fhi.x()] {
                        for y in [flo.y(), fhi.y()] {
                            for z in [flo.z(), fhi.z()] {
                                let rel = Pnt::new(x, y, z) - location;
                                let t = rel.dot(&axis);
                                axial_lo = axial_lo.min(t);
                                axial_hi = axial_hi.max(t);
                                let a = rel.dot(&e1);
                                let b = rel.dot(&e2);
                                u1_range[0] = u1_range[0].min(a);
                                u1_range[1] = u1_range[1].max(a);
                                u2_range[0] = u2_range[0].min(b);
                                u2_range[1] = u2_range[1].max(b);
                            }
                        }
                    }
                    if axial_lo.is_finite() && axial_hi >= axial_lo {
                        let angular =
                            rectangle_angular_range(u1_range, u2_range).unwrap_or((0.0, TAU));
                        add_interval(
                            &mut bounds,
                            surface.interval_point(angular.0, angular.1, axial_lo, axial_hi),
                        );
                    }
                }
                GeomSurface::Sphere(sphere) => {
                    let center = sphere.center();
                    let radius = sphere.radius();
                    if radius.is_finite() {
                        bounds.add(&Pnt::new(
                            center.x() - radius,
                            center.y() - radius,
                            center.z() - radius,
                        ));
                        bounds.add(&Pnt::new(
                            center.x() + radius,
                            center.y() + radius,
                            center.z() + radius,
                        ));
                    }
                }
                GeomSurface::Torus(torus) => {
                    let center = torus.position().location();
                    let reach = torus.major_radius() + torus.minor_radius();
                    if reach.is_finite() {
                        bounds.add(&Pnt::new(
                            center.x() - reach,
                            center.y() - reach,
                            center.z() - reach,
                        ));
                        bounds.add(&Pnt::new(
                            center.x() + reach,
                            center.y() + reach,
                            center.z() + reach,
                        ));
                    }
                }
                GeomSurface::BSpline(spline) => {
                    if let (Some((u0, u1)), Some((v0, v1))) = (
                        spline_param_range(spline.u_knots()),
                        spline_param_range(spline.v_knots()),
                    ) {
                        add_interval(&mut bounds, spline.interval_bbox(u0, u1, v0, v1));
                    }
                }
                GeomSurface::Plane(_) => {}
                GeomSurface::Gregory(_) => {
                    add_interval(&mut bounds, surface.interval_point(0.0, 1.0, 0.0, 1.0));
                }
                GeomSurface::Offset(_) | GeomSurface::Ruled(_) => {
                    // Linear pcurves give an exact enclosing UV rectangle.
                    // Curved/missing trims with an unbounded support remain
                    // unknown rather than silently falling back to edge boxes.
                    let mut uv = [
                        f64::INFINITY,
                        f64::NEG_INFINITY,
                        f64::INFINITY,
                        f64::NEG_INFINITY,
                    ];
                    let mut complete = true;
                    for wire in face.wires() {
                        for i in 0..wire.edges().len() {
                            if let Some(pc) = wire.pcurve(i).filter(|pc| {
                                matches!(pc.curve, openrcad_geom2d::GeomCurve2d::Line(_))
                            }) {
                                for p in [pc.point_at_fraction(0.0), pc.point_at_fraction(1.0)] {
                                    uv[0] = uv[0].min(p.x());
                                    uv[1] = uv[1].max(p.x());
                                    uv[2] = uv[2].min(p.y());
                                    uv[3] = uv[3].max(p.y());
                                }
                            } else {
                                complete = false;
                            }
                        }
                    }
                    let (u0, u1, v0, v1) = if complete && uv.iter().all(|x| x.is_finite()) {
                        (uv[0], uv[1], uv[2], uv[3])
                    } else {
                        surface.bounds()
                    };
                    if ![u0, u1, v0, v1].iter().all(|x| x.is_finite()) {
                        return None;
                    }
                    let b = surface.interval_point(u0, u1, v0, v1);
                    if ![b.x.lo, b.x.hi, b.y.lo, b.y.hi, b.z.lo, b.z.hi]
                        .iter()
                        .all(|x| x.is_finite())
                    {
                        return None;
                    }
                    add_interval(&mut bounds, b);
                }
            }
        }
        valid.get().then_some(bounds)
    }

    /// Split a solid whose boundary is several disconnected pieces into one
    /// [`Solid`] per connected component.
    ///
    /// A boolean cut that *severs* a body (e.g. slicing a bar in two) yields a
    /// single shell holding two disjoint, individually-watertight boxes — a
    /// valid B-Rep, but really two bodies. This groups faces into connected
    /// components (two faces are connected when they share a boundary edge,
    /// matched by quantized endpoints plus a curve midpoint like
    /// [`edges`](Solid::edges) and
    /// [`manifold_report`](Solid::manifold_report)) and rebuilds one solid per
    /// component.
    ///
    /// Returns a single-element vector (a clone of `self`) when the boundary is
    /// already one connected piece — so callers can treat the result uniformly.
    pub fn split_disconnected(&self) -> Vec<Solid> {
        // Multiple shells intentionally represent one material region with
        // enclosed voids. They must not be split into separate bodies merely
        // because their boundary graphs are disconnected.
        if self.shells().len() > 1 {
            return vec![self.clone()];
        }
        let faces = self.faces();
        let n = faces.len();
        if n <= 1 {
            return vec![self.clone()];
        }

        // Union-find over face indices, joined wherever two faces share an edge.
        fn find(parent: &mut [usize], mut x: usize) -> usize {
            while parent[x] != x {
                parent[x] = parent[parent[x]];
                x = parent[x];
            }
            x
        }

        let mut parent: Vec<usize> = (0..n).collect();
        let mut edge_owner: HashMap<_, usize> = HashMap::new();
        for (fi, face) in faces.iter().enumerate() {
            for wire in face.wires() {
                for edge in wire.edges() {
                    match edge_owner.get(&edge_key(&edge)) {
                        Some(&other) => {
                            let ra = find(&mut parent, fi);
                            let rb = find(&mut parent, other);
                            if ra != rb {
                                parent[ra] = rb;
                            }
                        }
                        None => {
                            edge_owner.insert(edge_key(&edge), fi);
                        }
                    }
                }
            }
        }

        // Group faces by component root, in first-seen order so the output is
        // deterministic (a stated kernel invariant).
        let mut group_of: HashMap<usize, usize> = HashMap::new();
        let mut groups: Vec<Vec<Face>> = Vec::new();
        for (fi, face) in faces.iter().enumerate() {
            let root = find(&mut parent, fi);
            let gi = *group_of.entry(root).or_insert_with(|| {
                groups.push(Vec::new());
                groups.len() - 1
            });
            groups[gi].push(face.clone());
        }

        if groups.len() <= 1 {
            return vec![self.clone()];
        }
        groups
            .into_iter()
            .map(|component| Solid::new(Shell::from_faces(component)))
            .collect()
    }

    /// Apply `t` to the whole boundary shell.
    pub fn transformed(&self, t: &Trsf) -> Self {
        let transformed_brep = self.brep.transformed(t);
        Self {
            brep: Arc::new(transformed_brep),
            id: self.id,
        }
    }

    /// Transform without rebuilding arena topology, retaining exact lineage
    /// when canonical geometric deduplication keeps the same representatives.
    pub fn transformed_with_history(&self, t: &Trsf) -> (Self, crate::TopologyHistory) {
        let result = self.transformed(t);
        let same_vertices = self
            .vertices()
            .iter()
            .map(|v| v.id())
            .eq(result.vertices().iter().map(|v| v.id()));
        let same_edges = self
            .edges()
            .iter()
            .map(|e| e.id())
            .eq(result.edges().iter().map(|e| e.id()));
        let history = if same_vertices && same_edges {
            crate::history::arena_preserving_history(&result)
        } else {
            // Very small scales or far coordinates can change quantized
            // representatives. Do not invent positional identity in that case.
            crate::TopologyHistory::conservative_unary(self, &result)
        };
        (result, history)
    }
}

fn vertex_key(v: &Vertex) -> (i64, i64, i64) {
    point_key(&v.point())
}

/// The parameter span of a knot vector (the surface's full supported range),
/// or `None` for a degenerate/empty knot sequence.
fn spline_param_range(knots: &[f64]) -> Option<(f64, f64)> {
    let first = *knots.first()?;
    let last = *knots.last()?;
    (first.is_finite() && last.is_finite() && last >= first).then_some((first, last))
}

/// The minimal angular interval covering an axis-aligned rectangle in the
/// `(e1, e2)` frame, or `None` when the rectangle strictly contains the axis
/// (any angle is then possible). The angle extrema of a convex region not
/// containing the origin lie at its vertices, so the corners' angles give
/// the exact hull; the covering arc is the complement of the largest gap
/// between consecutive corner angles. A rectangle that merely TOUCHES the
/// origin (corner contact from coordinate noise on an axis-touching trim,
/// e.g. a fillet cutter spanning one quadrant) still yields its quadrant
/// arc: only interior containment forces the full circle.
fn rectangle_angular_range(e1_range: [f64; 2], e2_range: [f64; 2]) -> Option<(f64, f64)> {
    use std::f64::consts::TAU;
    // Interior containment only: `tol` absorbs floating-point noise on trims
    // that legitimately touch an axis direction.
    let tol = 1.0e-9;
    if e1_range[0] < -tol && e1_range[1] > tol && e2_range[0] < -tol && e2_range[1] > tol {
        return None;
    }
    // Snap the contact noise so a corner sitting on an axis contributes the
    // axis direction itself, not a spurious third-quadrant angle.
    let clean = |interval: [f64; 2]| {
        [
            if interval[0].abs() <= tol {
                0.0
            } else {
                interval[0]
            },
            if interval[1].abs() <= tol {
                0.0
            } else {
                interval[1]
            },
        ]
    };
    let [a0, a1] = clean(e1_range);
    let [b0, b1] = clean(e2_range);
    let mut angles = [
        (b0).atan2(a0),
        (b0).atan2(a1),
        (b1).atan2(a0),
        (b1).atan2(a1),
    ];
    angles.sort_by(f64::total_cmp);
    let gaps = [
        angles[1] - angles[0],
        angles[2] - angles[1],
        angles[3] - angles[2],
        TAU - (angles[3] - angles[0]),
    ];
    let (mut largest, mut largest_index) = (f64::NEG_INFINITY, 0);
    for (index, gap) in gaps.iter().enumerate() {
        if *gap > largest {
            largest = *gap;
            largest_index = index;
        }
    }
    // The covering arc starts just after the largest gap. Index 3 is the
    // wraparound gap (from angles[3] to angles[0] + TAU). Both ends carry a
    // tiny angular slack so sub-nanometre corner noise cannot shave the true
    // trim.
    let slack = 1.0e-6;
    Some(match largest_index {
        3 => (angles[0] - slack, angles[3] + slack),
        k => (angles[k + 1] - slack, angles[k] + TAU + slack),
    })
}

fn edge_key(edge: &Edge) -> [(i64, i64, i64); 3] {
    let mut endpoints = [vertex_key(&edge.start()), vertex_key(&edge.end())];
    endpoints.sort_unstable();
    let start = edge.start().point();
    let end = edge.end().point();
    let midpoint = match edge.curve() {
        // Sewing may merge a line's endpoints while retaining a slightly
        // offset source line. Topological identity follows the merged vertices
        // there. Curved edges need an interior sample to distinguish two arcs
        // that legitimately share the same endpoints.
        None | Some(GeomCurve::Line(_)) => start + (end - start) * 0.5,
        Some(curve) => curve.point(0.5 * (edge.first() + edge.last())),
    };
    [endpoints[0], endpoints[1], point_key(&midpoint)]
}

fn point_key(p: &openrcad_foundation::Pnt) -> (i64, i64, i64) {
    let tol = openrcad_foundation::tolerance::CONFUSION;
    (
        (p.x() / tol).round() as i64,
        (p.y() / tol).round() as i64,
        (p.z() / tol).round() as i64,
    )
}

impl PartialEq for Solid {
    fn eq(&self, other: &Self) -> bool {
        self.shells() == other.shells()
    }
}

impl Serialize for Solid {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        (&*self.brep, self.id).serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Solid {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let (brep, id) = <(BRep, SolidId)>::deserialize(deserializer)?;
        Ok(Self {
            brep: Arc::new(brep),
            id,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::edge::Edge;
    use crate::face::Face;
    use crate::wire::Wire;
    use openrcad_foundation::Pnt;

    /// A degenerate "solid": two coincident triangles back-to-back sharing 3
    /// vertices and 3 edges. Good enough to exercise the dedup logic.
    fn flat_triangle_solid() -> Solid {
        let p = [
            Pnt::origin(),
            Pnt::new(1.0, 0.0, 0.0),
            Pnt::new(0.0, 1.0, 0.0),
        ];
        let wire = Wire::from_edges([
            Edge::between_points(p[0], p[1]),
            Edge::between_points(p[1], p[2]),
            Edge::between_points(p[2], p[0]),
        ]);
        let face = Face::new(None, wire);
        Solid::new(Shell::from_faces([face]))
    }

    #[test]
    fn triangle_dedups_to_3_vertices_and_3_edges() {
        let s = flat_triangle_solid();
        assert_eq!(s.vertex_count(), 3);
        assert_eq!(s.edge_count(), 3);
        assert_eq!(s.face_count(), 1);
    }

    #[test]
    fn bounding_box_covers_vertices() {
        let s = flat_triangle_solid();
        let (lo, hi) = s.bounding_box().corners().unwrap();
        assert_eq!(lo, Pnt::origin());
        assert_eq!(hi, Pnt::new(1.0, 1.0, 0.0));
    }

    /// The cylindrical wall of a radius-10 rod along +Y spanning y ∈ [0, 20],
    /// split at `seam` (the angle whose two vertices are the ONLY vertices on
    /// the wall: every other angle is pure surface between the rim arcs).
    fn cylinder_wall_solid(seam: f64) -> Solid {
        use crate::edge::Edge;
        use crate::wire::Wire;
        use openrcad_foundation::{Ax3, Dir};
        use openrcad_geom::{Circle, CylindricalSurface, Line};
        use std::f64::consts::{PI, TAU};

        let bottom = GeomCurve::Circle(Circle::new(Ax3::new(Pnt::origin(), Dir::dy()), 10.0));
        let top = GeomCurve::Circle(Circle::new(
            Ax3::new(Pnt::new(0.0, 20.0, 0.0), Dir::dy()),
            10.0,
        ));
        let pb0 = bottom.point(seam);
        let pb1 = bottom.point(seam + PI);
        let pt0 = top.point(seam);
        let pt1 = top.point(seam + PI);
        let arc = |curve: &GeomCurve, from: f64, to: f64, a: Pnt, b: Pnt| {
            Edge::new(
                Some(curve.clone()),
                from,
                to,
                Vertex::new(a),
                Vertex::new(b),
            )
        };
        let line = |a: Pnt, b: Pnt| {
            let d = (b - a).normalized().unwrap();
            Edge::new(
                Some(GeomCurve::Line(Line::from_point_dir(a, d))),
                0.0,
                a.distance(&b),
                Vertex::new(a),
                Vertex::new(b),
            )
        };
        let wire = Wire::from_edges([
            arc(&bottom, seam, seam + PI, pb0, pb1),
            line(pb1, pt1),
            arc(&top, seam + PI, seam + TAU, pt1, pt0),
            line(pt0, pb0),
        ]);
        let face = Face::new(
            Some(GeomSurface::Cylinder(CylindricalSurface::new(
                Ax3::new(Pnt::origin(), Dir::dy()),
                10.0,
            ))),
            wire,
        );
        Solid::new(Shell::from_faces([face]))
    }

    /// E15 regression: a radius-10 cylinder must report x/z ∈ [-10, 10] from
    /// its conservative bound regardless of where the seam (and therefore the
    /// only wall vertices) sits. The vertex-only box misses the curved
    /// extrema — at seam 0 it claims z ∈ [0, 0], which made a tool sketched
    /// below the rod look disjoint from it.
    #[test]
    fn conservative_bounds_cover_cylindrical_extrema_beyond_vertices() {
        for seam in [0.0, 0.25 * std::f64::consts::PI, 1.1] {
            let solid = cylinder_wall_solid(seam);
            let (lo, hi) = solid.conservative_bounding_box().corners().unwrap();
            for (got, want) in [
                (lo.x(), -10.0),
                (hi.x(), 10.0),
                (lo.z(), -10.0),
                (hi.z(), 10.0),
                (lo.y(), 0.0),
                (hi.y(), 20.0),
            ] {
                assert!(
                    (got - want).abs() < 1e-6,
                    "seam {seam}: expected {want}, got {got} (box {lo:?}..{hi:?})"
                );
            }
        }
    }

    fn square_face(x0: f64) -> Face {
        let wire = Wire::from_edges([
            Edge::between_points(Pnt::new(x0, 0.0, 0.0), Pnt::new(x0 + 1.0, 0.0, 0.0)),
            Edge::between_points(Pnt::new(x0 + 1.0, 0.0, 0.0), Pnt::new(x0 + 1.0, 1.0, 0.0)),
            Edge::between_points(Pnt::new(x0 + 1.0, 1.0, 0.0), Pnt::new(x0, 1.0, 0.0)),
            Edge::between_points(Pnt::new(x0, 1.0, 0.0), Pnt::new(x0, 0.0, 0.0)),
        ]);
        Face::new(None, wire)
    }

    #[test]
    fn connected_boundary_stays_one_body() {
        // A single face is trivially one component.
        let s = Solid::new(Shell::from_faces([square_face(0.0)]));
        assert_eq!(s.split_disconnected().len(), 1);
    }

    #[test]
    fn disjoint_faces_split_into_separate_bodies() {
        // Two squares 10 units apart share no edges → two independent components.
        let s = Solid::new(Shell::from_faces([square_face(0.0), square_face(10.0)]));
        let bodies = s.split_disconnected();
        assert_eq!(bodies.len(), 2);
        assert!(bodies.iter().all(|b| b.face_count() == 1));
    }

    #[test]
    fn edge_sharing_faces_stay_one_body() {
        // Two coincident squares share all four edges → still one component.
        let s = Solid::new(Shell::from_faces([square_face(0.0), square_face(0.0)]));
        assert_eq!(s.split_disconnected().len(), 1);
    }
}
