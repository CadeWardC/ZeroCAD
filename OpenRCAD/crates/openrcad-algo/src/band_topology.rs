//! Shared construction contracts for analytic transition bands.
//!
//! Shell, fillet, and recut operations use this module to build an immutable
//! boundary candidate before any topology is committed.  In particular, a
//! generated coedge always carries its exact 3D curve and both construction-
//! time pcurves; callers cannot accidentally commit half of that record.

use openrcad_foundation::ToleranceContext;
use openrcad_geom::{Curve, GeomCurve, GeomSurface, Surface};
use openrcad_topo::{Edge, InputTopologyRef, PcurveData, Vertex};

use crate::native_pcurve::analytic_line_pcurve;

/// Deterministic stages charged against an operation's geometry work budget.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GeometryWorkStage {
    Intersection,
    RootIsolation,
    Imprinting,
    Recut,
    Classification,
}

/// A deterministic work counter.  This is deliberately unrelated to wall
/// time, so the same recipe reaches the same accept/reject decision on every
/// machine and under a debugger.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GeometryWorkBudget {
    limit: u64,
    consumed: u64,
}

impl GeometryWorkBudget {
    pub const DEFAULT_INTERSECTION_LIMIT: u64 = 500_000;

    pub const fn new(limit: u64) -> Self {
        Self { limit, consumed: 0 }
    }

    pub const fn intersection_default() -> Self {
        Self::new(Self::DEFAULT_INTERSECTION_LIMIT)
    }

    pub const fn limit(&self) -> u64 {
        self.limit
    }

    pub const fn consumed(&self) -> u64 {
        self.consumed
    }

    pub const fn remaining(&self) -> u64 {
        self.limit.saturating_sub(self.consumed)
    }

    pub fn charge(
        &mut self,
        stage: GeometryWorkStage,
        units: u64,
    ) -> Result<(), BandTopologyError> {
        let next = self.consumed.saturating_add(units);
        if next > self.limit {
            return Err(BandTopologyError::OperationBudgetExhausted {
                stage,
                limit: self.limit,
                consumed: self.consumed,
            });
        }
        self.consumed = next;
        Ok(())
    }
}

/// Analytic support family used in stable failure details and owner keys.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum BandSupportKind {
    Plane,
    Cylinder,
    Cone,
    Sphere,
    Torus,
    Other,
}

impl BandSupportKind {
    pub fn of(surface: &GeomSurface) -> Self {
        match surface {
            GeomSurface::Plane(_) => Self::Plane,
            GeomSurface::Cylinder(_) => Self::Cylinder,
            GeomSurface::Cone(_) => Self::Cone,
            GeomSurface::Sphere(_) => Self::Sphere,
            GeomSurface::Torus(_) => Self::Torus,
            _ => Self::Other,
        }
    }
}

/// Stable provenance for one side of a generated boundary.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct BandSource {
    pub operand: usize,
    pub kind: openrcad_topo::TopologyKind,
    pub index: usize,
}

impl BandSource {
    fn ordering_key(self) -> (usize, u8, usize) {
        let kind = match self.kind {
            openrcad_topo::TopologyKind::Vertex => 0,
            openrcad_topo::TopologyKind::Edge => 1,
            openrcad_topo::TopologyKind::Wire => 2,
            openrcad_topo::TopologyKind::Face => 3,
            openrcad_topo::TopologyKind::Shell => 4,
            openrcad_topo::TopologyKind::Solid => 5,
        };
        (self.operand, kind, self.index)
    }
}

impl PartialOrd for BandSource {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for BandSource {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        self.ordering_key().cmp(&other.ordering_key())
    }
}

impl From<InputTopologyRef> for BandSource {
    fn from(value: InputTopologyRef) -> Self {
        Self {
            operand: value.operand,
            kind: value.entity.kind,
            index: value.entity.index,
        }
    }
}

/// Traversal-independent owner for a shared transition boundary.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct BandOwnerKey {
    pub first: BandSource,
    pub second: BandSource,
    pub branch: u32,
}

impl BandOwnerKey {
    pub fn canonical(left: BandSource, right: BandSource, branch: u32) -> Self {
        let (first, second) = if left <= right {
            (left, right)
        } else {
            (right, left)
        };
        Self {
            first,
            second,
            branch,
        }
    }
}

/// Immutable two-sided coedge candidate owned by the operation orchestrator.
#[derive(Clone, Debug)]
pub struct BandBoundary {
    pub edge: Edge,
    pub left_pcurve: PcurveData,
    pub right_pcurve: PcurveData,
    pub owner: BandOwnerKey,
    pub left_support: BandSupportKind,
    pub right_support: BandSupportKind,
}

/// Typed, fail-closed errors shared by Shell, Fillet, and recut operations.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BandTopologyError {
    OperationBudgetExhausted {
        stage: GeometryWorkStage,
        limit: u64,
        consumed: u64,
    },
    UnsupportedTransition {
        left: BandSupportKind,
        right: BandSupportKind,
    },
    InconsistentPcurve {
        side: &'static str,
        sample: usize,
    },
    DegenerateBoundary,
    AmbiguousOwnership,
}

impl BandTopologyError {
    /// Stable diagnostic code used by higher-level typed adapters.
    pub const fn diagnostic_code(&self) -> &'static str {
        match self {
            Self::OperationBudgetExhausted { .. } => "operation.budget_exhausted",
            Self::UnsupportedTransition { .. } => "shell.unsupported_band_transition",
            Self::InconsistentPcurve { .. } => "topology.inconsistent_pcurve",
            Self::DegenerateBoundary => "topology.degenerate_boundary",
            Self::AmbiguousOwnership => "topology.ambiguous_ownership",
        }
    }
}

impl core::fmt::Display for BandTopologyError {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::OperationBudgetExhausted {
                stage,
                limit,
                consumed,
            } => write!(
                formatter,
                "geometry work budget exhausted during {stage:?} ({consumed}/{limit})"
            ),
            Self::UnsupportedTransition { left, right } => {
                write!(
                    formatter,
                    "unsupported analytic band transition {left:?}-{right:?}"
                )
            }
            Self::InconsistentPcurve { side, sample } => {
                write!(
                    formatter,
                    "{side} pcurve disagrees with 3D boundary at sample {sample}"
                )
            }
            Self::DegenerateBoundary => {
                formatter.write_str("generated band boundary is degenerate")
            }
            Self::AmbiguousOwnership => formatter.write_str("band boundary ownership is ambiguous"),
        }
    }
}

impl std::error::Error for BandTopologyError {}

/// Construct and validate an exact two-sided boundary without mutating a B-Rep.
#[allow(clippy::too_many_arguments)]
pub fn build_two_sided_boundary(
    curve: GeomCurve,
    first: f64,
    last: f64,
    start: openrcad_foundation::Pnt,
    end: openrcad_foundation::Pnt,
    left_surface: &GeomSurface,
    right_surface: &GeomSurface,
    left_source: BandSource,
    right_source: BandSource,
    branch: u32,
    context: &ToleranceContext,
    budget: &mut GeometryWorkBudget,
) -> Result<BandBoundary, BandTopologyError> {
    budget.charge(GeometryWorkStage::Imprinting, 1)?;
    if !first.is_finite() || !last.is_finite() || first == last {
        return Err(BandTopologyError::DegenerateBoundary);
    }
    if ![start.x(), start.y(), start.z(), end.x(), end.y(), end.z()]
        .into_iter()
        .all(f64::is_finite)
    {
        return Err(BandTopologyError::DegenerateBoundary);
    }
    let endpoint_tolerance = context.policy.classification + context.arithmetic_floor * 8.0;
    if curve.point(first).distance(&start) > endpoint_tolerance
        || curve.point(last).distance(&end) > endpoint_tolerance
    {
        return Err(BandTopologyError::DegenerateBoundary);
    }
    let edge = Edge::new_with_tolerance(
        Some(curve),
        first,
        last,
        Vertex::new(start),
        Vertex::new(end),
        context.policy.linear,
    );
    let left_pcurve =
        openrcad_topo::pcurve_build::exact_pcurve_for_edge(left_surface, &edge, &context.policy)
            .unwrap_or_else(|| analytic_line_pcurve(left_surface, &edge));
    let right_pcurve =
        openrcad_topo::pcurve_build::exact_pcurve_for_edge(right_surface, &edge, &context.policy)
            .unwrap_or_else(|| analytic_line_pcurve(right_surface, &edge));
    validate_pcurve(&edge, left_surface, &left_pcurve, "left", context, budget)?;
    validate_pcurve(
        &edge,
        right_surface,
        &right_pcurve,
        "right",
        context,
        budget,
    )?;
    Ok(BandBoundary {
        edge,
        left_pcurve,
        right_pcurve,
        owner: BandOwnerKey::canonical(left_source, right_source, branch),
        left_support: BandSupportKind::of(left_surface),
        right_support: BandSupportKind::of(right_surface),
    })
}

fn validate_pcurve(
    edge: &Edge,
    surface: &GeomSurface,
    pcurve: &PcurveData,
    side: &'static str,
    context: &ToleranceContext,
    budget: &mut GeometryWorkBudget,
) -> Result<(), BandTopologyError> {
    let Some(curve) = edge.curve() else {
        return Err(BandTopologyError::DegenerateBoundary);
    };
    let threshold = context.policy.classification + context.arithmetic_floor * 8.0;
    for (sample, fraction) in [0.0, 0.25, 0.5, 0.75, 1.0].into_iter().enumerate() {
        budget.charge(GeometryWorkStage::Imprinting, 1)?;
        let parameter = edge.first() + (edge.last() - edge.first()) * fraction;
        let point_3d = curve.point(parameter);
        let uv = pcurve.wrapped_point_at_fraction(fraction);
        let point_surface = surface.point(uv.x(), uv.y());
        if point_3d.distance(&point_surface) > threshold {
            return Err(BandTopologyError::InconsistentPcurve { side, sample });
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::{Ax3, Dir, Pnt, TolerancePolicy};
    use openrcad_geom::{Circle, Plane, ToroidalSurface};
    use openrcad_topo::{TopologyKind, TopologyRef};

    fn source(operand: usize, index: usize) -> BandSource {
        InputTopologyRef::new(operand, TopologyRef::new(TopologyKind::Face, index)).into()
    }

    #[test]
    fn owner_is_independent_of_traversal_order() {
        assert_eq!(
            BandOwnerKey::canonical(source(0, 9), source(1, 2), 3),
            BandOwnerKey::canonical(source(1, 2), source(0, 9), 3)
        );
    }

    #[test]
    fn budget_exhaustion_is_deterministic_and_typed() {
        let mut budget = GeometryWorkBudget::new(2);
        budget.charge(GeometryWorkStage::Intersection, 2).unwrap();
        let error = budget
            .charge(GeometryWorkStage::Intersection, 1)
            .unwrap_err();
        assert_eq!(error.diagnostic_code(), "operation.budget_exhausted");
        assert_eq!(budget.consumed(), 2);
    }

    #[test]
    fn exact_plane_torus_circle_carries_two_valid_pcurves() {
        let scale = 3.0;
        let frame = Ax3::new(Pnt::origin(), Dir::dz());
        let plane = GeomSurface::plane(Plane::from_point_normal(Pnt::origin(), Dir::dz()));
        let torus = GeomSurface::torus(ToroidalSurface::new(frame, 4.0 * scale, scale));
        let circle = GeomCurve::Circle(Circle::new(frame, 5.0 * scale));
        let context =
            ToleranceContext::derive(&TolerancePolicy::STANDARD, &[], Some(scale), scale).unwrap();
        let mut budget = GeometryWorkBudget::new(32);
        let start = circle.point(0.0);
        let end = circle.point(core::f64::consts::TAU);
        let boundary = build_two_sided_boundary(
            circle,
            0.0,
            core::f64::consts::TAU,
            start,
            end,
            &plane,
            &torus,
            source(0, 0),
            source(0, 1),
            0,
            &context,
            &mut budget,
        )
        .unwrap();
        assert_eq!(boundary.left_support, BandSupportKind::Plane);
        assert_eq!(boundary.right_support, BandSupportKind::Torus);
        assert!(boundary.left_pcurve.is_valid());
        assert!(boundary.right_pcurve.is_valid());
    }
}
