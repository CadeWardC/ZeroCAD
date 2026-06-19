//! Structural validity checking for assembled topology.
//!
//! [`Solid::validate`] walks the solid's shells and verifies the invariants that
//! every well-formed B-Rep must satisfy regardless of geometry:
//!
//! - **Reference integrity** — every shell/face/loop/edge/vertex handle reached
//!   from the solid actually resolves in the arena (a `sew` or boolean step that
//!   removes a merged entity must repair every reference to it).
//! - **Non-empty loops** — a boundary loop has at least one edge.
//! - **Loop contiguity & closure** — walking a loop in per-use traversal order,
//!   each edge's traversal-end coincides (within tolerance) with the next edge's
//!   traversal-start, and the last closes back onto the first. This is the
//!   invariant the loop-oriented [`OrientedEdge`](crate::arena::OrientedEdge)
//!   refactor must preserve, so it is the one a regression is most likely to break.
//!
//! Contiguity is measured by *point distance* (not `VertexId` equality): wires are
//! built from independently-constructed edges whose coincident endpoints are
//! distinct arena vertices sharing a location, exactly as [`Wire::is_closed`](crate::Wire::is_closed)
//! treats them.
//!
//! [`Solid::assert_valid`] is the panicking wrapper for `debug_assert!`-style use
//! in tests and debug builds.

use openrcad_foundation::{tolerance, Pnt};

use crate::arena::{BRep, EdgeId, FaceId, LoopId, OrientedEdge, ShellId, VertexId};
use crate::orientation::Orientation;
use crate::solid::Solid;

/// A structured diagnostic summary for a [`Solid`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct HealthReport {
    /// Hard failures that make the solid unsafe to feed to later modeling ops.
    pub errors: Vec<HealthError>,
    /// Suspicious but not always invalid conditions.
    pub warnings: Vec<HealthWarning>,
}

impl HealthReport {
    /// True when no hard health errors were found.
    #[inline]
    pub fn is_healthy(&self) -> bool {
        self.errors.is_empty()
    }
}

/// A hard health error found while auditing a solid.
#[derive(Clone, Debug, PartialEq)]
pub enum HealthError {
    /// The existing structural validator failed.
    Validation(ValidationError),
    /// The solid references no boundary shells.
    EmptySolid,
    /// A face has no outer boundary.
    FaceWithoutOuterWire(FaceId),
    /// A face has no usable boundary loops.
    EmptyFace(FaceId),
    /// A vertex coordinate is `NaN` or infinite.
    NonFiniteVertex(VertexId),
    /// An edge's chord length is at or below tolerance.
    DegenerateEdge { edge: EdgeId, length: f64 },
}

/// A softer health warning. These are useful for import diagnostics and test
/// triage, but do not necessarily mean the B-Rep is structurally broken.
#[derive(Clone, Debug, PartialEq)]
pub enum HealthWarning {
    /// A closed genus-0 solid normally has Euler characteristic 2.
    SuspiciousEulerCharacteristic { value: i64 },
    /// One or more boundary edges are shared by three or more faces.
    NonManifoldEdges { count: usize },
}

/// A violated topological invariant reported by [`Solid::validate`].
#[derive(Clone, Debug, PartialEq)]
pub enum ValidationError {
    /// A shell handle referenced by the solid is missing from the arena.
    DanglingShell(ShellId),
    /// A face handle referenced by a shell is missing from the arena.
    DanglingFace(FaceId),
    /// A loop handle referenced by a face is missing from the arena.
    DanglingLoop(LoopId),
    /// An edge referenced by a loop is missing from the arena.
    DanglingEdge { loop_id: LoopId, edge: EdgeId },
    /// A vertex referenced by an edge is missing from the arena.
    DanglingVertex { edge: EdgeId },
    /// A loop carries no edges.
    EmptyLoop(LoopId),
    /// Consecutive edges in a loop do not meet (or the loop does not close): the
    /// gap between one edge's traversal-end and the next's traversal-start
    /// exceeds the endpoints' combined tolerance.
    LoopNotContiguous { loop_id: LoopId, gap: f64 },
}

impl core::fmt::Display for ValidationError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ValidationError::DanglingShell(id) => write!(f, "dangling shell reference {id:?}"),
            ValidationError::DanglingFace(id) => write!(f, "dangling face reference {id:?}"),
            ValidationError::DanglingLoop(id) => write!(f, "dangling loop reference {id:?}"),
            ValidationError::DanglingEdge { loop_id, edge } => {
                write!(f, "loop {loop_id:?} references missing edge {edge:?}")
            }
            ValidationError::DanglingVertex { edge } => {
                write!(f, "edge {edge:?} references a missing vertex")
            }
            ValidationError::EmptyLoop(id) => write!(f, "loop {id:?} has no edges"),
            ValidationError::LoopNotContiguous { loop_id, gap } => {
                write!(f, "loop {loop_id:?} is not contiguous (gap {gap:e})")
            }
        }
    }
}

impl std::error::Error for ValidationError {}

/// The traversal start/end points of an oriented edge, plus the tolerance of the
/// vertex at each, or `None` if any referenced entity is missing from the arena.
fn oriented_endpoints(
    brep: &BRep,
    oe: &OrientedEdge,
    loop_id: LoopId,
) -> Result<(Pnt, Pnt, f64, f64), ValidationError> {
    let e = brep.edges.get(oe.id).ok_or(ValidationError::DanglingEdge {
        loop_id,
        edge: oe.id,
    })?;
    let sv = brep
        .vertices
        .get(e.start)
        .ok_or(ValidationError::DanglingVertex { edge: oe.id })?;
    let ev = brep
        .vertices
        .get(e.end)
        .ok_or(ValidationError::DanglingVertex { edge: oe.id })?;
    // start/end are stored in the edge's natural sense; only a Reversed use flips
    // the traversal direction (Internal/External embedded edges keep natural sense).
    Ok(if oe.orientation == Orientation::Reversed {
        (ev.point, sv.point, ev.tolerance, sv.tolerance)
    } else {
        (sv.point, ev.point, sv.tolerance, ev.tolerance)
    })
}

fn validate_loop(brep: &BRep, loop_id: LoopId) -> Result<(), ValidationError> {
    let loop_data = brep
        .loops
        .get(loop_id)
        .ok_or(ValidationError::DanglingLoop(loop_id))?;
    let n = loop_data.edges.len();
    if n == 0 {
        return Err(ValidationError::EmptyLoop(loop_id));
    }

    // Traversal start/end point and endpoint tolerance for each edge in order.
    let mut ends: Vec<(Pnt, Pnt, f64, f64)> = Vec::with_capacity(n);
    for oe in &loop_data.edges {
        ends.push(oriented_endpoints(brep, oe, loop_id)?);
    }

    // Each edge's traversal-end must meet the next edge's traversal-start; the
    // i == n-1 step also enforces closure (last end → first start).
    for i in 0..n {
        let (_, cur_end, _, cur_end_tol) = ends[i];
        let (next_start, _, next_start_tol, _) = ends[(i + 1) % n];
        let gap = cur_end.distance(&next_start);
        // Honour the meeting vertices' own uncertainty radii, with a small floor
        // so exact (zero-gap) primitive geometry is never rejected.
        let tol = (cur_end_tol + next_start_tol).max(tolerance::CONFUSION * 16.0);
        if gap > tol {
            return Err(ValidationError::LoopNotContiguous { loop_id, gap });
        }
    }
    Ok(())
}

impl Solid {
    /// Verify the solid's structural topological invariants (see the
    /// [module docs](crate::validate)). Returns the first violation found.
    pub fn validate(&self) -> Result<(), ValidationError> {
        let brep = self.brep.as_ref();
        // The solid's own id is guaranteed present (we hold a handle to it).
        for &shell_id in &brep.solids[self.id].shells {
            let shell = brep
                .shells
                .get(shell_id)
                .ok_or(ValidationError::DanglingShell(shell_id))?;
            for &face_id in &shell.faces {
                let face = brep
                    .faces
                    .get(face_id)
                    .ok_or(ValidationError::DanglingFace(face_id))?;
                if let Some(outer) = face.outer_wire {
                    validate_loop(brep, outer)?;
                }
                for &inner in &face.inner_wires {
                    validate_loop(brep, inner)?;
                }
            }
        }
        Ok(())
    }

    /// Panic if [`validate`](Solid::validate) reports a violation. Intended for
    /// `debug_assert!`-style invariant checks in tests and debug builds.
    #[track_caller]
    pub fn assert_valid(&self) {
        if let Err(e) = self.validate() {
            panic!("topology invariant violated: {e}");
        }
    }

    /// Audit structural and basic geometric health without panicking.
    ///
    /// This is intentionally cheap and conservative: it checks reference/loop
    /// validity via [`validate`](Solid::validate), then scans the reached B-Rep
    /// for non-finite coordinates, degenerate edges, faces without boundaries,
    /// and suspicious Euler characteristic. It does not claim full CAD-kernel
    /// validity (self-intersection and exact watertightness still need deeper
    /// algorithmic checks), but it gives booleans, importers, and renderers a
    /// shared diagnostic hook.
    pub fn health_report(&self) -> HealthReport {
        let mut report = HealthReport::default();
        if let Err(err) = self.validate() {
            report.errors.push(HealthError::Validation(err));
        }

        let brep = self.brep.as_ref();
        let Some(solid_data) = brep.solids.get(self.id) else {
            report.errors.push(HealthError::EmptySolid);
            return report;
        };
        if solid_data.shells.is_empty() {
            report.errors.push(HealthError::EmptySolid);
        }

        for (vertex_id, vertex) in &brep.vertices {
            let p = vertex.point;
            if !p.x().is_finite() || !p.y().is_finite() || !p.z().is_finite() {
                report.errors.push(HealthError::NonFiniteVertex(vertex_id));
            }
        }

        for (edge_id, edge) in &brep.edges {
            let Some(start) = brep.vertices.get(edge.start) else {
                continue;
            };
            let Some(end) = brep.vertices.get(edge.end) else {
                continue;
            };
            let length = start.point.distance(&end.point);
            let tol = edge.tolerance.max(tolerance::CONFUSION);
            if length <= tol {
                report.errors.push(HealthError::DegenerateEdge {
                    edge: edge_id,
                    length,
                });
            }
        }

        for &shell_id in &solid_data.shells {
            let Some(shell) = brep.shells.get(shell_id) else {
                continue;
            };
            for &face_id in &shell.faces {
                let Some(face) = brep.faces.get(face_id) else {
                    continue;
                };
                if face.outer_wire.is_none() {
                    report
                        .errors
                        .push(HealthError::FaceWithoutOuterWire(face_id));
                }
                if face.outer_wire.is_none() && face.inner_wires.is_empty() {
                    report.errors.push(HealthError::EmptyFace(face_id));
                }
            }
        }

        let euler = self.euler_characteristic();
        if self.face_count() > 1 && euler != 2 {
            report
                .warnings
                .push(HealthWarning::SuspiciousEulerCharacteristic { value: euler });
        }

        let manifold = self.manifold_report();
        if manifold.nonmanifold_edges > 0 {
            report.warnings.push(HealthWarning::NonManifoldEdges {
                count: manifold.nonmanifold_edges,
            });
        }

        report
    }

    /// The Euler characteristic `V − E + F` over the solid's deduplicated
    /// boundary entities. For a closed, genus-0 solid this is `2`
    /// (Euler–Poincaré); each handle through a hole subtracts `2`.
    pub fn euler_characteristic(&self) -> i64 {
        self.vertex_count() as i64 - self.edge_count() as i64 + self.face_count() as i64
    }

    /// Tally how the solid's boundary edges are shared between faces.
    ///
    /// Faces are built from independently-constructed edges, so two faces that
    /// meet along an edge hold *distinct* arena edges at the same location. This
    /// matches undirected segments by their endpoint positions (quantized to a
    /// fine grid) and counts how many faces use each: a closed two-manifold
    /// solid shares every edge by exactly two faces.
    pub fn manifold_report(&self) -> ManifoldReport {
        // Quantize a point to a fine integer grid so coincident endpoints from
        // independently-built edges hash together.
        const GRID: f64 = 1.0e6;
        let q = |p: &Pnt| -> QuantPoint {
            (
                (p.x() * GRID).round() as i64,
                (p.y() * GRID).round() as i64,
                (p.z() * GRID).round() as i64,
            )
        };
        let mut counts: std::collections::HashMap<EdgeKey, u32> = std::collections::HashMap::new();
        for face in self.shell().faces() {
            for wire in face.wires() {
                for edge in wire.edges() {
                    let a = q(&edge.start().point());
                    let b = q(&edge.end().point());
                    let key = if a <= b { (a, b) } else { (b, a) };
                    *counts.entry(key).or_insert(0) += 1;
                }
            }
        }
        let mut report = ManifoldReport {
            total_edges: counts.len(),
            free_edges: 0,
            nonmanifold_edges: 0,
        };
        for &c in counts.values() {
            match c {
                1 => report.free_edges += 1,
                2 => {}
                _ => report.nonmanifold_edges += 1,
            }
        }
        report
    }

    /// True when the boundary is closed and two-manifold: every edge is shared
    /// by exactly two faces (no free/open edges, no non-manifold edges). This is
    /// the structural half of "watertight" — what booleans, sewing, and STL
    /// export must produce.
    pub fn is_watertight(&self) -> bool {
        let m = self.manifold_report();
        m.free_edges == 0 && m.nonmanifold_edges == 0 && m.total_edges > 0
    }

    /// True when no edge is shared by three or more faces (open shells allowed).
    pub fn is_manifold(&self) -> bool {
        self.manifold_report().nonmanifold_edges == 0
    }
}

/// A point quantized to the manifold-check integer grid.
type QuantPoint = (i64, i64, i64);
/// An undirected boundary edge keyed by its two quantized endpoints (sorted).
type EdgeKey = (QuantPoint, QuantPoint);

/// How a solid's boundary edges are shared between faces (see
/// [`Solid::manifold_report`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ManifoldReport {
    /// Distinct undirected edge locations on the boundary.
    pub total_edges: usize,
    /// Edges used by exactly one face — an open/free boundary (not watertight).
    pub free_edges: usize,
    /// Edges used by three or more faces — non-manifold.
    pub nonmanifold_edges: usize,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::edge::Edge;
    use crate::face::Face;
    use crate::shell::Shell;
    use crate::wire::Wire;

    fn square_face(z: f64) -> Face {
        let w = Wire::from_edges([
            Edge::between_points(Pnt::new(0.0, 0.0, z), Pnt::new(1.0, 0.0, z)),
            Edge::between_points(Pnt::new(1.0, 0.0, z), Pnt::new(1.0, 1.0, z)),
            Edge::between_points(Pnt::new(1.0, 1.0, z), Pnt::new(0.0, 1.0, z)),
            Edge::between_points(Pnt::new(0.0, 1.0, z), Pnt::new(0.0, 0.0, z)),
        ]);
        Face::new(None, w)
    }

    #[test]
    fn valid_square_solid_passes() {
        let s = Solid::new(Shell::from_faces([square_face(0.0)]));
        assert!(s.validate().is_ok());
        s.assert_valid();
        // One open square: V−E+F = 4−4+1 = 1.
        assert_eq!(s.euler_characteristic(), 1);
        assert!(s.health_report().is_healthy());
    }

    #[test]
    fn open_square_is_not_watertight() {
        // A lone face: all four edges are free, none shared — open boundary.
        let s = Solid::new(Shell::from_faces([square_face(0.0)]));
        let m = s.manifold_report();
        assert_eq!(m.free_edges, 4);
        assert_eq!(m.nonmanifold_edges, 0);
        assert!(!s.is_watertight());
        assert!(s.is_manifold()); // open, but still two-manifold
    }

    #[test]
    fn closed_two_faces_share_all_edges() {
        // Two coincident squares (opposite orientation) form a degenerate closed
        // shell: every one of the 4 edges is shared by exactly two faces.
        let s = Solid::new(Shell::from_faces([square_face(0.0), square_face(0.0)]));
        let m = s.manifold_report();
        assert_eq!(m.total_edges, 4);
        assert_eq!(m.free_edges, 0);
        assert_eq!(m.nonmanifold_edges, 0);
        assert!(s.is_watertight());
    }

    #[test]
    fn non_contiguous_loop_is_detected() {
        // A "loop" whose third edge does not connect — a broken boundary.
        let w = Wire::from_edges([
            Edge::between_points(Pnt::new(0.0, 0.0, 0.0), Pnt::new(1.0, 0.0, 0.0)),
            Edge::between_points(Pnt::new(1.0, 0.0, 0.0), Pnt::new(1.0, 1.0, 0.0)),
            // Jumps to a disconnected location instead of closing the loop.
            Edge::between_points(Pnt::new(5.0, 5.0, 0.0), Pnt::new(0.0, 0.0, 0.0)),
        ]);
        let s = Solid::new(Shell::from_faces([Face::new(None, w)]));
        match s.validate() {
            Err(ValidationError::LoopNotContiguous { gap, .. }) => assert!(gap > 1.0),
            other => panic!("expected LoopNotContiguous, got {other:?}"),
        }
    }

    #[test]
    #[should_panic(expected = "topology invariant violated")]
    fn assert_valid_panics_on_broken_loop() {
        let w = Wire::from_edges([
            Edge::between_points(Pnt::new(0.0, 0.0, 0.0), Pnt::new(1.0, 0.0, 0.0)),
            Edge::between_points(Pnt::new(2.0, 2.0, 0.0), Pnt::new(0.0, 0.0, 0.0)),
        ]);
        Solid::new(Shell::from_faces([Face::new(None, w)])).assert_valid();
    }

    #[test]
    fn health_report_flags_degenerate_edge() {
        let w = Wire::from_edges([Edge::between_points(Pnt::origin(), Pnt::origin())]);
        let s = Solid::new(Shell::from_faces([Face::new(None, w)]));
        let report = s.health_report();
        assert!(report
            .errors
            .iter()
            .any(|e| matches!(e, HealthError::DegenerateEdge { .. })));
    }

    #[test]
    fn health_report_includes_validation_errors() {
        let w = Wire::from_edges([
            Edge::between_points(Pnt::origin(), Pnt::new(1.0, 0.0, 0.0)),
            Edge::between_points(Pnt::new(2.0, 0.0, 0.0), Pnt::origin()),
        ]);
        let s = Solid::new(Shell::from_faces([Face::new(None, w)]));
        let report = s.health_report();
        assert!(report
            .errors
            .iter()
            .any(|e| matches!(e, HealthError::Validation(_))));
    }
}
