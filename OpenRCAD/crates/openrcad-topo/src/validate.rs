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

use std::collections::{HashMap, HashSet};

use openrcad_foundation::{Pnt, Pnt2d, TolerancePolicy};
use openrcad_geom::{Curve, GeomSurface, Surface};

use crate::arena::{BRep, EdgeId, FaceId, LoopId, OrientedEdge, PcurveId, ShellId, VertexId};
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
    /// A closed orientable shell must have an even Euler characteristic no
    /// larger than two (summed across shells for a multi-shell solid).
    InvalidEulerCharacteristic { value: i64 },
}

/// A softer health warning. These are useful for import diagnostics and test
/// triage, but do not necessarily mean the B-Rep is structurally broken.
#[derive(Clone, Debug, PartialEq)]
pub enum HealthWarning {
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
    /// A pcurve referenced by a coedge is missing from the arena.
    DanglingPcurve {
        loop_id: LoopId,
        edge: EdgeId,
        pcurve: PcurveId,
    },
    /// A vertex referenced by an edge is missing from the arena.
    DanglingVertex { edge: EdgeId },
    /// A loop carries no edges.
    EmptyLoop(LoopId),
    /// A coedge has a pcurve, but its face has no supporting surface.
    PcurveWithoutSurface {
        face: FaceId,
        loop_id: LoopId,
        edge: EdgeId,
    },
    /// A surface-backed coedge has no face-specific pcurve.
    MissingPcurve {
        face: FaceId,
        loop_id: LoopId,
        edge: EdgeId,
    },
    /// A pcurve has an invalid range or invalid periodicity metadata.
    InvalidPcurve {
        loop_id: LoopId,
        edge: EdgeId,
        pcurve: PcurveId,
    },
    /// A pcurve lifted through its surface diverges from its 3D edge curve.
    PcurveMismatch {
        face: FaceId,
        loop_id: LoopId,
        edge: EdgeId,
        max_deviation: f64,
        tolerance: f64,
    },
    /// Consecutive coedge pcurves do not meet in face parameter space.
    UvLoopNotContiguous { loop_id: LoopId, gap: f64 },
    /// Two non-adjacent boundary segments cross in face parameter space.
    UvLoopSelfIntersection { loop_id: LoopId },
    /// A boundary edge belongs to only one face of a strict solid shell.
    FreeEdge { loop_id: LoopId, edge: EdgeId },
    /// A boundary edge is used by more than two faces.
    NonManifoldEdge { edge: EdgeId, uses: usize },
    /// The two faces sharing an edge traverse it in the same direction.
    SameDirectionSharedCoedge {
        first_loop: LoopId,
        second_loop: LoopId,
        edge: EdgeId,
    },
    /// One shell contains multiple disconnected face components.
    DisconnectedShell { shell: ShellId, components: usize },
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
            ValidationError::DanglingPcurve {
                loop_id,
                edge,
                pcurve,
            } => write!(
                f,
                "loop {loop_id:?} edge {edge:?} references missing pcurve {pcurve:?}"
            ),
            ValidationError::DanglingVertex { edge } => {
                write!(f, "edge {edge:?} references a missing vertex")
            }
            ValidationError::EmptyLoop(id) => write!(f, "loop {id:?} has no edges"),
            ValidationError::PcurveWithoutSurface {
                face,
                loop_id,
                edge,
            } => write!(
                f,
                "face {face:?} loop {loop_id:?} edge {edge:?} has a pcurve but no surface"
            ),
            ValidationError::MissingPcurve {
                face,
                loop_id,
                edge,
            } => write!(
                f,
                "face {face:?} loop {loop_id:?} edge {edge:?} has no pcurve"
            ),
            ValidationError::InvalidPcurve {
                loop_id,
                edge,
                pcurve,
            } => write!(
                f,
                "loop {loop_id:?} edge {edge:?} has invalid pcurve {pcurve:?}"
            ),
            ValidationError::PcurveMismatch {
                face,
                loop_id,
                edge,
                max_deviation,
                tolerance,
            } => write!(
                f,
                "face {face:?} loop {loop_id:?} edge {edge:?} pcurve deviates by {max_deviation:e} (tolerance {tolerance:e})"
            ),
            ValidationError::UvLoopNotContiguous { loop_id, gap } => write!(
                f,
                "loop {loop_id:?} has a UV boundary gap of {gap:e}"
            ),
            ValidationError::UvLoopSelfIntersection { loop_id } => {
                write!(f, "loop {loop_id:?} self-intersects in parameter space")
            }
            ValidationError::FreeEdge { loop_id, edge } => write!(
                f,
                "strict shell has free boundary edge {edge:?} in loop {loop_id:?}"
            ),
            ValidationError::NonManifoldEdge { edge, uses } => write!(
                f,
                "strict shell edge {edge:?} has {uses} face uses"
            ),
            ValidationError::SameDirectionSharedCoedge {
                first_loop,
                second_loop,
                edge,
            } => write!(
                f,
                "shared edge {edge:?} has the same traversal direction in loops {first_loop:?} and {second_loop:?}"
            ),
            ValidationError::DisconnectedShell { shell, components } => write!(
                f,
                "shell {shell:?} has {components} disconnected face components"
            ),
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

fn validate_loop(
    brep: &BRep,
    face_id: FaceId,
    surface: Option<&GeomSurface>,
    loop_id: LoopId,
    policy: &TolerancePolicy,
) -> Result<(), ValidationError> {
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
        validate_pcurve(brep, face_id, surface, loop_id, oe, policy)?;
    }

    // Each edge's traversal-end must meet the next edge's traversal-start; the
    // i == n-1 step also enforces closure (last end → first start).
    for i in 0..n {
        let (_, cur_end, _, cur_end_tol) = ends[i];
        let (next_start, _, next_start_tol, _) = ends[(i + 1) % n];
        let gap = cur_end.distance(&next_start);
        // Honour the meeting vertices' own uncertainty radii, with a small floor
        // so exact (zero-gap) primitive geometry is never rejected.
        let tol = (cur_end_tol + next_start_tol).max(policy.linear * 16.0);
        if gap > tol {
            return Err(ValidationError::LoopNotContiguous { loop_id, gap });
        }
    }
    Ok(())
}

fn align_periodic(value: f64, target: f64, period: Option<f64>) -> f64 {
    period.map_or(value, |period| {
        value + ((target - value) / period).round() * period
    })
}

fn validate_uv_loop(
    brep: &BRep,
    face_id: FaceId,
    surface: &GeomSurface,
    loop_id: LoopId,
    policy: &TolerancePolicy,
) -> Result<(), ValidationError> {
    let wire = brep
        .loops
        .get(loop_id)
        .ok_or(ValidationError::DanglingLoop(loop_id))?;
    let mut vertices = Vec::<Pnt2d>::with_capacity(wire.edges.len() + 2);
    let mut first_periodicity = None;

    for coedge in &wire.edges {
        let pcurve_id = coedge.pcurve.ok_or(ValidationError::MissingPcurve {
            face: face_id,
            loop_id,
            edge: coedge.id,
        })?;
        let pcurve = brep
            .pcurves
            .get(pcurve_id)
            .ok_or(ValidationError::DanglingPcurve {
                loop_id,
                edge: coedge.id,
                pcurve: pcurve_id,
            })?;
        let (mut start, mut end) = if coedge.orientation == Orientation::Reversed {
            (pcurve.point_at_fraction(1.0), pcurve.point_at_fraction(0.0))
        } else {
            (pcurve.point_at_fraction(0.0), pcurve.point_at_fraction(1.0))
        };
        first_periodicity.get_or_insert(pcurve.periodicity);
        if let Some(&previous) = vertices.last() {
            let aligned_x = align_periodic(start.x(), previous.x(), pcurve.periodicity.u_period);
            let aligned_y = align_periodic(start.y(), previous.y(), pcurve.periodicity.v_period);
            let offset_x = aligned_x - start.x();
            let offset_y = aligned_y - start.y();
            start = Pnt2d::new(aligned_x, aligned_y);
            end = Pnt2d::new(end.x() + offset_x, end.y() + offset_y);
            let gap = start.distance(&previous);
            if gap > policy.pcurve_consistency {
                let lifted_gap = surface
                    .point(previous.x(), previous.y())
                    .distance(&surface.point(start.x(), start.y()));
                if lifted_gap > policy.linear * 16.0 {
                    return Err(ValidationError::UvLoopNotContiguous { loop_id, gap });
                }
                // A surface singularity (for example a cone apex or sphere
                // pole) can collapse a finite UV connector to one 3D point.
                // Keep that connector explicit for the self-intersection pass.
                vertices.push(start);
            }
        } else {
            vertices.push(start);
        }
        vertices.push(end);
    }

    let Some(&last) = vertices.last() else {
        return Err(ValidationError::EmptyLoop(loop_id));
    };
    let first = vertices[0];
    let first_periodicity = first_periodicity.expect("non-empty loop checked above");
    let closed = Pnt2d::new(
        align_periodic(first.x(), last.x(), first_periodicity.u_period),
        align_periodic(first.y(), last.y(), first_periodicity.v_period),
    );
    let closure_gap = last.distance(&closed);
    if closure_gap > policy.pcurve_consistency {
        let lifted_gap = surface
            .point(last.x(), last.y())
            .distance(&surface.point(closed.x(), closed.y()));
        if lifted_gap > policy.linear * 16.0 {
            return Err(ValidationError::UvLoopNotContiguous {
                loop_id,
                gap: closure_gap,
            });
        }
        // A singular surface point can require an explicit finite connector in
        // UV even though both ends lift to the same 3D pole/apex.
        vertices.push(closed);
    } else if let Some(last) = vertices.last_mut() {
        // Snap numerical round-trip noise onto the first point's unwrapped
        // periodic branch. Appending another nearly-zero closure segment would
        // make the first and last real segments look non-adjacent to the
        // self-intersection pass.
        *last = closed;
    }

    let segment_count = vertices.len() - 1;
    for first_index in 0..segment_count {
        for second_index in (first_index + 1)..segment_count {
            let adjacent = second_index == first_index + 1
                || (first_index == 0 && second_index + 1 == segment_count);
            if adjacent {
                continue;
            }
            if segments_cross(
                vertices[first_index],
                vertices[first_index + 1],
                vertices[second_index],
                vertices[second_index + 1],
                policy.pcurve_consistency,
            ) {
                return Err(ValidationError::UvLoopSelfIntersection { loop_id });
            }
        }
    }
    Ok(())
}

fn segments_cross(a: Pnt2d, b: Pnt2d, c: Pnt2d, d: Pnt2d, tolerance: f64) -> bool {
    let orient = |p: Pnt2d, q: Pnt2d, r: Pnt2d| {
        (q.x() - p.x()) * (r.y() - p.y()) - (q.y() - p.y()) * (r.x() - p.x())
    };
    let ab_c = orient(a, b, c);
    let ab_d = orient(a, b, d);
    let cd_a = orient(c, d, a);
    let cd_b = orient(c, d, b);
    ab_c * ab_d < -tolerance * tolerance && cd_a * cd_b < -tolerance * tolerance
}

#[derive(Clone, Copy)]
struct StrictEdgeUse {
    face_index: usize,
    loop_id: LoopId,
    edge: EdgeId,
    traversal_start: QuantPoint,
    traversal_end: QuantPoint,
}

fn strict_edge_key(
    brep: &BRep,
    coedge: &OrientedEdge,
    face_orientation: Orientation,
    loop_id: LoopId,
    policy: &TolerancePolicy,
) -> Result<(EdgeKey, QuantPoint, QuantPoint), ValidationError> {
    let edge = brep
        .edges
        .get(coedge.id)
        .ok_or(ValidationError::DanglingEdge {
            loop_id,
            edge: coedge.id,
        })?;
    let start = brep
        .vertices
        .get(edge.start)
        .ok_or(ValidationError::DanglingVertex { edge: coedge.id })?
        .point;
    let end = brep
        .vertices
        .get(edge.end)
        .ok_or(ValidationError::DanglingVertex { edge: coedge.id })?
        .point;
    let midpoint = edge.curve.as_ref().map_or_else(
        || {
            Pnt::new(
                0.5 * (start.x() + end.x()),
                0.5 * (start.y() + end.y()),
                0.5 * (start.z() + end.z()),
            )
        },
        |curve| curve.point(0.5 * (edge.first + edge.last)),
    );
    let grid = 1.0 / policy.approximation;
    let quantize = |point: Pnt| {
        (
            (point.x() * grid).round() as i64,
            (point.y() * grid).round() as i64,
            (point.z() * grid).round() as i64,
        )
    };
    let natural_start = quantize(start);
    let natural_end = quantize(end);
    let middle = quantize(midpoint);
    let key = if natural_start <= natural_end {
        (natural_start, natural_end, middle)
    } else {
        (natural_end, natural_start, middle)
    };
    let reversed =
        (coedge.orientation == Orientation::Reversed) ^ (face_orientation == Orientation::Reversed);
    let (traversal_start, traversal_end) = if reversed {
        (natural_end, natural_start)
    } else {
        (natural_start, natural_end)
    };
    Ok((key, traversal_start, traversal_end))
}

fn component_root(parents: &mut [usize], index: usize) -> usize {
    if parents[index] != index {
        parents[index] = component_root(parents, parents[index]);
    }
    parents[index]
}

fn union_components(parents: &mut [usize], first: usize, second: usize) {
    let first = component_root(parents, first);
    let second = component_root(parents, second);
    if first != second {
        parents[second] = first;
    }
}

fn validate_pcurve(
    brep: &BRep,
    face_id: FaceId,
    surface: Option<&GeomSurface>,
    loop_id: LoopId,
    coedge: &OrientedEdge,
    policy: &TolerancePolicy,
) -> Result<(), ValidationError> {
    let Some(pcurve_id) = coedge.pcurve else {
        return Ok(());
    };
    let pcurve = brep
        .pcurves
        .get(pcurve_id)
        .ok_or(ValidationError::DanglingPcurve {
            loop_id,
            edge: coedge.id,
            pcurve: pcurve_id,
        })?;
    if !pcurve.is_valid() {
        return Err(ValidationError::InvalidPcurve {
            loop_id,
            edge: coedge.id,
            pcurve: pcurve_id,
        });
    }
    let surface = surface.ok_or(ValidationError::PcurveWithoutSurface {
        face: face_id,
        loop_id,
        edge: coedge.id,
    })?;
    let edge = brep
        .edges
        .get(coedge.id)
        .ok_or(ValidationError::DanglingEdge {
            loop_id,
            edge: coedge.id,
        })?;
    let Some(curve) = edge.curve.as_ref() else {
        // Degenerate seam/pole edges may legitimately have only a pcurve.
        return Ok(());
    };

    let mut max_deviation = 0.0_f64;
    for sample in 0..=8 {
        let fraction = f64::from(sample) / 8.0;
        let uv = pcurve.point_at_fraction(fraction);
        let lifted = surface.point(uv.x(), uv.y());
        let edge_parameter = edge.first + (edge.last - edge.first) * fraction;
        let deviation = lifted.distance(&curve.point(edge_parameter));
        if !deviation.is_finite() {
            max_deviation = f64::INFINITY;
            break;
        }
        max_deviation = max_deviation.max(deviation);
    }
    let tolerance = policy.pcurve_consistency.max(edge.tolerance);
    if max_deviation > tolerance {
        return Err(ValidationError::PcurveMismatch {
            face: face_id,
            loop_id,
            edge: coedge.id,
            max_deviation,
            tolerance,
        });
    }
    Ok(())
}

impl Solid {
    /// Verify the solid's structural topological invariants (see the
    /// [module docs](crate::validate)). Returns the first violation found.
    pub fn validate(&self) -> Result<(), ValidationError> {
        self.validate_with_policy(&TolerancePolicy::STANDARD)
    }

    /// Verify structural invariants using the supplied document tolerance
    /// policy. Operation entry points validate the policy before calling this
    /// method.
    pub fn validate_with_policy(&self, policy: &TolerancePolicy) -> Result<(), ValidationError> {
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
                    validate_loop(brep, face_id, face.surface.as_ref(), outer, policy)?;
                }
                for &inner in &face.inner_wires {
                    validate_loop(brep, face_id, face.surface.as_ref(), inner, policy)?;
                }
            }
        }
        Ok(())
    }

    /// Strict Phase 1 validation. In addition to structural validity, every
    /// coedge on a surface-backed face must carry a pcurve.
    pub fn validate_strict_with_policy(
        &self,
        policy: &TolerancePolicy,
    ) -> Result<(), ValidationError> {
        self.validate_with_policy(policy)?;
        let brep = self.brep.as_ref();
        for &shell_id in &brep.solids[self.id].shells {
            let shell = &brep.shells[shell_id];
            let mut edge_uses = HashMap::<EdgeKey, Vec<StrictEdgeUse>>::new();
            for (face_index, &face_id) in shell.faces.iter().enumerate() {
                let face = &brep.faces[face_id];
                if face.surface.is_none() {
                    continue;
                }
                let loops = face
                    .outer_wire
                    .into_iter()
                    .chain(face.inner_wires.iter().copied());
                for loop_id in loops {
                    for coedge in &brep.loops[loop_id].edges {
                        if coedge.pcurve.is_none() {
                            return Err(ValidationError::MissingPcurve {
                                face: face_id,
                                loop_id,
                                edge: coedge.id,
                            });
                        }
                        let (key, traversal_start, traversal_end) =
                            strict_edge_key(brep, coedge, face.orientation, loop_id, policy)?;
                        edge_uses.entry(key).or_default().push(StrictEdgeUse {
                            face_index,
                            loop_id,
                            edge: coedge.id,
                            traversal_start,
                            traversal_end,
                        });
                    }
                    validate_uv_loop(
                        brep,
                        face_id,
                        face.surface.as_ref().expect("surface checked above"),
                        loop_id,
                        policy,
                    )?;
                }
            }

            let mut parents = (0..shell.faces.len()).collect::<Vec<_>>();
            for uses in edge_uses.values() {
                match uses.as_slice() {
                    [only] => {
                        return Err(ValidationError::FreeEdge {
                            loop_id: only.loop_id,
                            edge: only.edge,
                        })
                    }
                    [first, second] => {
                        if first.traversal_start != second.traversal_end
                            || first.traversal_end != second.traversal_start
                        {
                            return Err(ValidationError::SameDirectionSharedCoedge {
                                first_loop: first.loop_id,
                                second_loop: second.loop_id,
                                edge: first.edge,
                            });
                        }
                        union_components(&mut parents, first.face_index, second.face_index);
                    }
                    many => {
                        return Err(ValidationError::NonManifoldEdge {
                            edge: many[0].edge,
                            uses: many.len(),
                        })
                    }
                }
            }
            let components = (0..parents.len())
                .map(|index| component_root(&mut parents, index))
                .collect::<HashSet<_>>()
                .len();
            if components > 1 {
                return Err(ValidationError::DisconnectedShell {
                    shell: shell_id,
                    components,
                });
            }
        }
        Ok(())
    }

    /// True when every surface-backed coedge carries a pcurve.
    pub fn has_complete_pcurves(&self) -> bool {
        let brep = self.brep.as_ref();
        brep.solids.get(self.id).is_some_and(|solid| {
            solid.shells.iter().all(|shell_id| {
                brep.shells.get(*shell_id).is_some_and(|shell| {
                    shell.faces.iter().all(|face_id| {
                        brep.faces.get(*face_id).is_some_and(|face| {
                            face.surface.is_none()
                                || face
                                    .outer_wire
                                    .into_iter()
                                    .chain(face.inner_wires.iter().copied())
                                    .all(|loop_id| {
                                        brep.loops.get(loop_id).is_some_and(|wire| {
                                            wire.edges.iter().all(|coedge| coedge.pcurve.is_some())
                                        })
                                    })
                        })
                    })
                })
            })
        })
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
    /// and an impossible Euler characteristic. It does not claim full CAD-kernel
    /// validity (self-intersection and exact watertightness still need deeper
    /// algorithmic checks), but it gives booleans, importers, and renderers a
    /// shared diagnostic hook.
    pub fn health_report(&self) -> HealthReport {
        self.health_report_with_policy(&TolerancePolicy::STANDARD)
    }

    /// Audit structural and basic geometric health under a document tolerance
    /// policy.
    pub fn health_report_with_policy(&self, policy: &TolerancePolicy) -> HealthReport {
        let mut report = HealthReport::default();
        if let Err(err) = self.validate_with_policy(policy) {
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

        for vertex in self.vertices() {
            let vertex_id = vertex.id();
            let p = vertex.point();
            if !p.x().is_finite() || !p.y().is_finite() || !p.z().is_finite() {
                report.errors.push(HealthError::NonFiniteVertex(vertex_id));
            }
        }

        for reached_edge in self.edges() {
            let edge_id = reached_edge.id();
            let edge = &brep.edges[edge_id];
            let Some(start) = brep.vertices.get(edge.start) else {
                continue;
            };
            let Some(end) = brep.vertices.get(edge.end) else {
                continue;
            };
            let chord = start.point.distance(&end.point);
            // A closed analytic edge (most often a periodic seam or a full
            // circle) legitimately has coincident vertices. Its interior must
            // still span space; chord length alone would reject it as collapsed.
            let length = edge.curve.as_ref().map_or(chord, |curve| {
                let mid = curve.point(0.5 * (edge.first + edge.last));
                chord
                    .max(start.point.distance(&mid))
                    .max(end.point.distance(&mid))
            });
            let tol = edge.tolerance.max(policy.linear);
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
        let maximum_euler = 2 * solid_data.shells.len() as i64;
        if self.face_count() > 1 && (euler.rem_euclid(2) != 0 || euler > maximum_euler) {
            report
                .errors
                .push(HealthError::InvalidEulerCharacteristic { value: euler });
        }

        let manifold = self.manifold_report_with_policy(policy);
        if manifold.nonmanifold_edges > 0 {
            report.warnings.push(HealthWarning::NonManifoldEdges {
                count: manifold.nonmanifold_edges,
            });
        }

        report
    }

    /// The Euler characteristic over the solid's deduplicated boundary cell
    /// complex. A face with `n` inner wires contributes `1 − n`, rather than
    /// being incorrectly counted as a disk. For a closed genus-0 solid the
    /// result is `2`; each handle through a hole subtracts `2`.
    pub fn euler_characteristic(&self) -> i64 {
        let face_cells = self
            .faces()
            .into_iter()
            .map(|face| 1_i64 - face.inner_wires().len() as i64)
            .sum::<i64>();
        self.vertex_count() as i64 - self.edge_count() as i64 + face_cells
    }

    /// Tally how the solid's boundary edges are shared between faces.
    ///
    /// Faces are built from independently-constructed edges, so two faces that
    /// meet along an edge hold *distinct* arena edges at the same location. This
    /// matches undirected segments by their endpoint positions (quantized to a
    /// fine grid) and counts how many faces use each: a closed two-manifold
    /// solid shares every edge by exactly two faces.
    pub fn manifold_report(&self) -> ManifoldReport {
        self.manifold_report_with_policy(&TolerancePolicy::STANDARD)
    }

    /// Tally boundary-edge sharing using the supplied policy's approximation
    /// tolerance as the positional matching grid.
    pub fn manifold_report_with_policy(&self, policy: &TolerancePolicy) -> ManifoldReport {
        // Quantize a point to a fine integer grid so coincident endpoints from
        // independently-built edges hash together.
        let grid = 1.0 / policy.approximation;
        let q = |p: &Pnt| -> QuantPoint {
            (
                (p.x() * grid).round() as i64,
                (p.y() * grid).round() as i64,
                (p.z() * grid).round() as i64,
            )
        };
        let mut counts: std::collections::HashMap<EdgeKey, u32> = std::collections::HashMap::new();
        for face in self.faces() {
            for wire in face.wires() {
                for edge in wire.edges() {
                    let start = edge.start().point();
                    let end = edge.end().point();
                    let a = q(&start);
                    let b = q(&end);
                    // A third key sample at the edge's MIDPOINT distinguishes the
                    // two arcs of one circle that share both endpoints (e.g. two
                    // semicircles): endpoint-only keying merges them into one
                    // "edge" used 4× and falsely reports non-manifold. Sample the
                    // curve at its mid-parameter when present, else the chord
                    // midpoint (a straight edge — its own midpoint is unambiguous).
                    let mid = match edge.curve() {
                        Some(c) => c.point(0.5 * (edge.first() + edge.last())),
                        None => Pnt::new(
                            0.5 * (start.x() + end.x()),
                            0.5 * (start.y() + end.y()),
                            0.5 * (start.z() + end.z()),
                        ),
                    };
                    let m = q(&mid);
                    let key = if a <= b { (a, b, m) } else { (b, a, m) };
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
        self.is_watertight_with_policy(&TolerancePolicy::STANDARD)
    }

    /// True when the boundary is closed and two-manifold under `policy`.
    pub fn is_watertight_with_policy(&self, policy: &TolerancePolicy) -> bool {
        let m = self.manifold_report_with_policy(policy);
        m.free_edges == 0 && m.nonmanifold_edges == 0 && m.total_edges > 0
    }

    /// True when no edge is shared by three or more faces (open shells allowed).
    pub fn is_manifold(&self) -> bool {
        self.is_manifold_with_policy(&TolerancePolicy::STANDARD)
    }

    /// True when no edge is shared by three or more faces under `policy`.
    pub fn is_manifold_with_policy(&self, policy: &TolerancePolicy) -> bool {
        self.manifold_report_with_policy(policy).nonmanifold_edges == 0
    }
}

/// A point quantized to the manifold-check integer grid.
type QuantPoint = (i64, i64, i64);
/// An undirected boundary edge keyed by its two quantized endpoints (sorted)
/// plus a quantized midpoint sample — the midpoint separates co-endpoint arcs
/// of one circle that would otherwise collide on endpoints alone.
type EdgeKey = (QuantPoint, QuantPoint, QuantPoint);

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
    use crate::pcurve::PcurveData;
    use crate::shell::Shell;
    use crate::wire::Wire;
    use crate::BRepBuilder;
    use openrcad_foundation::{Dir2d, Pnt2d, Trsf, Vec as GeomVec};
    use openrcad_geom::{GeomSurface, Plane};
    use openrcad_geom2d::{GeomCurve2d, Line2d};

    fn square_with_pcurves(offset_y: f64) -> Solid {
        let face = Face::new(
            Some(GeomSurface::plane(Plane::from_point_normal(
                Pnt::origin(),
                openrcad_foundation::Dir::dz(),
            ))),
            Wire::from_edges([
                Edge::between_points(Pnt::new(0.0, 0.0, 0.0), Pnt::new(1.0, 0.0, 0.0)),
                Edge::between_points(Pnt::new(1.0, 0.0, 0.0), Pnt::new(1.0, 1.0, 0.0)),
                Edge::between_points(Pnt::new(1.0, 1.0, 0.0), Pnt::new(0.0, 1.0, 0.0)),
                Edge::between_points(Pnt::new(0.0, 1.0, 0.0), Pnt::new(0.0, 0.0, 0.0)),
            ]),
        );
        let mut builder = BRepBuilder::from_brep((*face.brep).clone());
        let face_id = builder.brep().faces.keys().next().unwrap();
        let loop_id = builder.brep().faces[face_id].outer_wire.unwrap();
        let edge_ids: Vec<_> = builder.brep().loops[loop_id]
            .edges
            .iter()
            .map(|coedge| coedge.id)
            .collect();
        for (index, edge_id) in edge_ids.into_iter().enumerate() {
            let (start, end) = {
                let edge = &builder.brep().edges[edge_id];
                (
                    builder.brep().vertices[edge.start].point,
                    builder.brep().vertices[edge.end].point,
                )
            };
            let dx = end.x() - start.x();
            let dy = end.y() - start.y();
            let length = dx.hypot(dy);
            let pcurve = PcurveData::new(
                GeomCurve2d::line(Line2d::from_point_dir(
                    Pnt2d::new(start.x(), start.y() + offset_y),
                    Dir2d::new(dx / length, dy / length),
                )),
                0.0,
                length,
            );
            builder.attach_pcurve(loop_id, index, pcurve).unwrap();
        }
        let shell_id = builder.brep_mut().shells.insert(crate::arena::ShellData {
            faces: vec![face_id],
        });
        let solid_id = builder.brep_mut().solids.insert(crate::arena::SolidData {
            shells: vec![shell_id],
        });
        Solid::from_id(builder.build(), solid_id)
    }

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
    fn consistent_face_pcurves_pass_validation() {
        let solid = square_with_pcurves(0.0);
        assert!(solid.validate().is_ok());
    }

    #[test]
    fn pcurve_surface_edge_mismatch_is_rejected() {
        let solid = square_with_pcurves(0.01);
        assert!(matches!(
            solid.validate(),
            Err(ValidationError::PcurveMismatch {
                max_deviation,
                ..
            }) if max_deviation > 0.009
        ));
    }

    #[test]
    fn strict_validation_rejects_free_edges() {
        let solid = square_with_pcurves(0.0);
        assert!(matches!(
            solid.validate_strict_with_policy(&TolerancePolicy::STANDARD),
            Err(ValidationError::FreeEdge { .. })
        ));
    }

    #[test]
    fn strict_validation_rejects_same_direction_shared_coedges() {
        let face = square_with_pcurves(0.0).shell().faces()[0].clone();
        let duplicate = face.transformed(&Trsf::IDENTITY);
        let solid = Solid::new(Shell::from_faces([face, duplicate]));
        assert!(matches!(
            solid.validate_strict_with_policy(&TolerancePolicy::STANDARD),
            Err(ValidationError::SameDirectionSharedCoedge { .. })
        ));
    }

    #[test]
    fn strict_validation_rejects_non_manifold_edge_uses() {
        let face = square_with_pcurves(0.0).shell().faces()[0].clone();
        let duplicate = face.transformed(&Trsf::IDENTITY);
        let third = face.transformed(&Trsf::IDENTITY);
        let solid = Solid::new(Shell::from_faces([face, duplicate, third]));
        assert!(matches!(
            solid.validate_strict_with_policy(&TolerancePolicy::STANDARD),
            Err(ValidationError::NonManifoldEdge { uses: 3, .. })
        ));
    }

    #[test]
    fn strict_validation_rejects_disconnected_closed_components() {
        let face = square_with_pcurves(0.0).shell().faces()[0].clone();
        let opposite = face.transformed(&Trsf::IDENTITY).reversed();
        let translation = Trsf::translation(GeomVec::new(10.0, 0.0, 0.0));
        let shifted = face.transformed(&translation);
        let shifted_opposite = shifted.transformed(&Trsf::IDENTITY).reversed();
        let solid = Solid::new(Shell::from_faces([
            face,
            opposite,
            shifted,
            shifted_opposite,
        ]));
        assert!(matches!(
            solid.validate_strict_with_policy(&TolerancePolicy::STANDARD),
            Err(ValidationError::DisconnectedShell { components: 2, .. })
        ));
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
    fn two_semicircle_arcs_of_one_circle_are_distinct_edges() {
        // A disc bounded by TWO semicircle arcs (0→π, π→2π) of one circle, paired
        // with its coincident opposite twin. The two arcs share BOTH endpoints,
        // so endpoint-only keying merged them into one edge used 4× → false
        // non-manifold. The midpoint sample separates them: 2 edges, each used
        // twice → watertight. This is what let revolve drop the full-turn-thirds
        // workaround (a full circle needs only two half-arcs, not three thirds).
        use crate::vertex::Vertex;
        use openrcad_foundation::{Ax3, Dir};
        use openrcad_geom::{Circle, GeomCurve};
        let circle = Circle::new(Ax3::new(Pnt::origin(), Dir::dz()), 2.0);
        let pi = std::f64::consts::PI;
        let arc = |t0: f64, t1: f64| {
            Edge::new(
                Some(GeomCurve::circle(circle)),
                t0,
                t1,
                Vertex::new(circle.point(t0)),
                Vertex::new(circle.point(t1)),
            )
        };
        let disc = || Face::new(None, Wire::from_edges([arc(0.0, pi), arc(pi, 2.0 * pi)]));
        let s = Solid::new(Shell::from_faces([disc(), disc()]));
        let m = s.manifold_report();
        assert_eq!(
            m.total_edges, 2,
            "two semicircles must be two distinct edges; got {m:?}"
        );
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
