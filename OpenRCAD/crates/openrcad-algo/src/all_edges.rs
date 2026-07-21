//! Strict whole-solid Fillet and Chamfer convenience operations.
//!
//! The legacy primitive builders remain the fastest path for boxes and
//! cylinders. Arbitrary solids use the selected-edge engines in one immutable
//! candidate. If any edge blocks the operation, every blocker discovered in the
//! canonical pass is returned and the caller receives no partial solid.

use core::cmp::Ordering;
use core::fmt;

use openrcad_foundation::{Pnt, TolerancePolicy, TolerancePolicyError};
use openrcad_geom::GeomCurve;
use openrcad_topo::{Edge, Solid};

use crate::{BlendError, ChamferError, RollingBallError};

/// The strict whole-solid operation being planned.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AllEdgeBlendKind {
    Fillet,
    Chamfer,
}

impl fmt::Display for AllEdgeBlendKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Fillet => f.write_str("fillet"),
            Self::Chamfer => f.write_str("chamfer"),
        }
    }
}

/// Why one canonical source edge blocked a strict whole-solid operation.
#[derive(Clone, Debug, PartialEq)]
pub enum AllEdgeBlendFailure {
    /// A primitive whole-solid builder found a global geometric limit.
    WholeSolid(BlendError),
    /// The selected-edge rolling-ball builder rejected this edge.
    Fillet(RollingBallError),
    /// The selected-edge chamfer builder rejected this edge.
    Chamfer(ChamferError),
    /// The complete edge set failed as an interaction even though every edge
    /// could be applied independently in the diagnostic pass.
    Interaction { reason: String },
}

impl fmt::Display for AllEdgeBlendFailure {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WholeSolid(error) => write!(f, "{error}"),
            Self::Fillet(error) => write!(f, "{error}"),
            Self::Chamfer(error) => write!(f, "{error}"),
            Self::Interaction { reason } => write!(f, "edge-set interaction failed: {reason}"),
        }
    }
}

/// One blocked edge, ordered by the operation's deterministic semantic sort.
#[derive(Clone, Debug, PartialEq)]
pub struct AllEdgeBlocker {
    /// Stable position in the canonical edge list for this source solid.
    pub ordinal: usize,
    /// The exact immutable source edge that failed.
    pub edge: Edge,
    /// Typed failure reported by the relevant builder.
    pub failure: AllEdgeBlendFailure,
}

/// Atomic failure from a strict whole-solid Fillet or Chamfer.
#[derive(Clone, Debug, PartialEq)]
pub enum AllEdgeBlendError {
    InvalidTolerancePolicy(TolerancePolicyError),
    InvalidValue {
        kind: AllEdgeBlendKind,
        value: f64,
    },
    NoEligibleEdges,
    BlockingEdges {
        kind: AllEdgeBlendKind,
        value: f64,
        blockers: Vec<AllEdgeBlocker>,
    },
}

impl AllEdgeBlendError {
    /// The complete canonical blocker list, when geometry reached edge planning.
    pub fn blockers(&self) -> &[AllEdgeBlocker] {
        match self {
            Self::BlockingEdges { blockers, .. } => blockers,
            _ => &[],
        }
    }
}

impl fmt::Display for AllEdgeBlendError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidTolerancePolicy(error) => {
                write!(f, "all-edge blend: invalid tolerance policy: {error}")
            }
            Self::InvalidValue { kind, value } => {
                write!(
                    f,
                    "all-edge {kind}: value must be finite and non-negative, got {value}"
                )
            }
            Self::NoEligibleEdges => f.write_str("all-edge blend: the solid has no edges"),
            Self::BlockingEdges {
                kind,
                value,
                blockers,
            } => write!(
                f,
                "all-edge {kind} at {value} was rejected by {} edge(s)",
                blockers.len()
            ),
        }
    }
}

impl std::error::Error for AllEdgeBlendError {}

/// Failure from the structured strict all-edge operation adapter.
#[derive(Clone, Debug, PartialEq)]
pub enum AllEdgeBlendOperationError {
    Blend(AllEdgeBlendError),
    Candidate(crate::ModelingOperationError),
}

impl fmt::Display for AllEdgeBlendOperationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Blend(error) => write!(f, "{error}"),
            Self::Candidate(error) => write!(f, "{error}"),
        }
    }
}

impl std::error::Error for AllEdgeBlendOperationError {}

/// Fillet every topological edge as one atomic operation.
pub fn fillet_all_edges_strict(solid: &Solid, radius: f64) -> Result<Solid, AllEdgeBlendError> {
    fillet_all_edges_strict_with_policy(solid, radius, &TolerancePolicy::STANDARD)
}

/// Fillet every topological edge with one validated tolerance policy.
pub fn fillet_all_edges_strict_with_policy(
    solid: &Solid,
    radius: f64,
    policy: &TolerancePolicy,
) -> Result<Solid, AllEdgeBlendError> {
    apply_all_edges_strict(solid, AllEdgeBlendKind::Fillet, radius, policy)
}

/// Structured strict all-edge Fillet with validation, recovery, and history.
pub fn fillet_all_edges_strict_operation_with_policy(
    solid: &Solid,
    radius: f64,
    policy: &TolerancePolicy,
) -> Result<openrcad_topo::OperationResult<Solid>, AllEdgeBlendOperationError> {
    let value = fillet_all_edges_strict_with_policy(solid, radius, policy)
        .map_err(AllEdgeBlendOperationError::Blend)?;
    crate::finish_unary_operation(solid, value, policy)
        .map_err(AllEdgeBlendOperationError::Candidate)
}

/// Chamfer every topological edge as one atomic operation.
pub fn chamfer_all_edges_strict(solid: &Solid, distance: f64) -> Result<Solid, AllEdgeBlendError> {
    chamfer_all_edges_strict_with_policy(solid, distance, &TolerancePolicy::STANDARD)
}

/// Chamfer every topological edge with one validated tolerance policy.
pub fn chamfer_all_edges_strict_with_policy(
    solid: &Solid,
    distance: f64,
    policy: &TolerancePolicy,
) -> Result<Solid, AllEdgeBlendError> {
    apply_all_edges_strict(solid, AllEdgeBlendKind::Chamfer, distance, policy)
}

/// Structured strict all-edge Chamfer with validation, recovery, and history.
pub fn chamfer_all_edges_strict_operation_with_policy(
    solid: &Solid,
    distance: f64,
    policy: &TolerancePolicy,
) -> Result<openrcad_topo::OperationResult<Solid>, AllEdgeBlendOperationError> {
    let value = chamfer_all_edges_strict_with_policy(solid, distance, policy)
        .map_err(AllEdgeBlendOperationError::Blend)?;
    crate::finish_unary_operation(solid, value, policy)
        .map_err(AllEdgeBlendOperationError::Candidate)
}

fn apply_all_edges_strict(
    solid: &Solid,
    kind: AllEdgeBlendKind,
    value: f64,
    policy: &TolerancePolicy,
) -> Result<Solid, AllEdgeBlendError> {
    policy
        .validate()
        .map_err(AllEdgeBlendError::InvalidTolerancePolicy)?;
    if !value.is_finite() || value < 0.0 {
        return Err(AllEdgeBlendError::InvalidValue { kind, value });
    }
    if value <= policy.linear {
        return Ok(solid.clone());
    }

    let edges = canonical_edges(solid);
    if edges.is_empty() {
        return Err(AllEdgeBlendError::NoEligibleEdges);
    }

    // Preserve the exact primitive implementations. Besides being faster, they
    // solve all corners simultaneously and therefore remain the authoritative
    // all-edge result for boxes and cylinders.
    let primitive = match kind {
        AllEdgeBlendKind::Fillet => crate::fillet::fillet_with_policy(solid, value, policy),
        AllEdgeBlendKind::Chamfer => crate::chamfer::chamfer_with_policy(solid, value, policy),
    };
    match primitive {
        Ok(result) => return Ok(result),
        Err(BlendError::UnsupportedShape) => {}
        Err(error) => {
            return Err(block_every_edge(
                kind,
                value,
                &edges,
                AllEdgeBlendFailure::WholeSolid(error),
            ));
        }
    }

    // Try the completed multi-edge/corner machinery first. This keeps verified
    // simultaneous N-valent behavior available to the convenience API.
    let grouped = match kind {
        AllEdgeBlendKind::Fillet => {
            crate::rolling_ball::fillet_edges_with_policy(solid, &edges, value, policy)
                .map_err(AllEdgeBlendFailure::Fillet)
        }
        AllEdgeBlendKind::Chamfer => {
            crate::chamfer::chamfer_edges_with_policy(solid, &edges, value, policy)
                .map_err(AllEdgeBlendFailure::Chamfer)
        }
    };
    let grouped_failure = match grouped {
        Ok(result) => return Ok(result),
        Err(error) => error,
    };

    // Diagnostic replay is local and immutable. Successful candidates advance
    // only this scratch solid; failed edges are recorded and skipped so callers
    // receive the complete blocker set instead of merely the first error. The
    // scratch result is returned only when every edge succeeds.
    let mut current = solid.clone();
    let mut blockers = Vec::new();
    for (ordinal, edge) in edges.iter().enumerate() {
        let result = match kind {
            AllEdgeBlendKind::Fillet => crate::rolling_ball::fillet_edges_with_policy(
                &current,
                std::slice::from_ref(edge),
                value,
                policy,
            )
            .map_err(AllEdgeBlendFailure::Fillet),
            AllEdgeBlendKind::Chamfer => crate::chamfer::chamfer_edges_with_policy(
                &current,
                std::slice::from_ref(edge),
                value,
                policy,
            )
            .map_err(AllEdgeBlendFailure::Chamfer),
        };
        match result {
            Ok(candidate) => current = candidate,
            Err(failure) => blockers.push(AllEdgeBlocker {
                ordinal,
                edge: edge.clone(),
                failure,
            }),
        }
    }

    if blockers.is_empty() {
        // The grouped solver can reject a set whose canonical sequential form is
        // nevertheless strict. Returning the fully-applied scratch candidate is
        // still atomic and never skips an edge.
        return Ok(current);
    }
    if blockers.len() == edges.len()
        && blockers.iter().all(|blocker| {
            matches!(
                blocker.failure,
                AllEdgeBlendFailure::Fillet(RollingBallError::SpineNotOnFace)
                    | AllEdgeBlendFailure::Chamfer(ChamferError::SpineNotOnFace)
            )
        })
    {
        // Preserve the more informative grouped reason when every diagnostic
        // relocation failed before reaching geometry.
        let reason = grouped_failure.to_string();
        for blocker in &mut blockers {
            blocker.failure = AllEdgeBlendFailure::Interaction {
                reason: reason.clone(),
            };
        }
    }
    Err(AllEdgeBlendError::BlockingEdges {
        kind,
        value,
        blockers,
    })
}

fn block_every_edge(
    kind: AllEdgeBlendKind,
    value: f64,
    edges: &[Edge],
    failure: AllEdgeBlendFailure,
) -> AllEdgeBlendError {
    AllEdgeBlendError::BlockingEdges {
        kind,
        value,
        blockers: edges
            .iter()
            .enumerate()
            .map(|(ordinal, edge)| AllEdgeBlocker {
                ordinal,
                edge: edge.clone(),
                failure: failure.clone(),
            })
            .collect(),
    }
}

fn canonical_edges(solid: &Solid) -> Vec<Edge> {
    let mut edges = solid.edges();
    edges.sort_by(compare_edges);
    edges
}

fn compare_edges(left: &Edge, right: &Edge) -> Ordering {
    let (left_a, left_b) = canonical_endpoints(left);
    let (right_a, right_b) = canonical_endpoints(right);
    compare_points(&left_a, &right_a)
        .then_with(|| compare_points(&left_b, &right_b))
        .then_with(|| curve_rank(left).cmp(&curve_rank(right)))
        .then_with(|| left.first().total_cmp(&right.first()))
        .then_with(|| left.last().total_cmp(&right.last()))
}

fn canonical_endpoints(edge: &Edge) -> (Pnt, Pnt) {
    let source = edge.source().point();
    let target = edge.target().point();
    if compare_points(&source, &target).is_gt() {
        (target, source)
    } else {
        (source, target)
    }
}

fn compare_points(left: &Pnt, right: &Pnt) -> Ordering {
    left.x()
        .total_cmp(&right.x())
        .then_with(|| left.y().total_cmp(&right.y()))
        .then_with(|| left.z().total_cmp(&right.z()))
}

fn curve_rank(edge: &Edge) -> u8 {
    match edge.curve() {
        None => 0,
        Some(GeomCurve::Line(_)) => 1,
        Some(GeomCurve::Circle(_)) => 2,
        Some(GeomCurve::Ellipse(_)) => 3,
        Some(GeomCurve::Parabola(_)) => 4,
        Some(GeomCurve::Hyperbola(_)) => 5,
        Some(GeomCurve::BSpline(_)) => 6,
        Some(GeomCurve::Helix(_)) => 7,
        Some(GeomCurve::TorusPlaneSection(_)) => 8,
        Some(GeomCurve::Reparametrized(_)) => 9,
        Some(GeomCurve::TorusSurfaceCurve(_)) => 10,
    }
}
