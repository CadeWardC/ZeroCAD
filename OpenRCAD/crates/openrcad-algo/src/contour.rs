//! Lightweight contour input for selected-edge blends.
//!
//! This is intentionally a thin facade over the existing selected-edge fillet
//! and chamfer solvers. It gives applications one typed request shape without
//! replacing the hand-tuned rolling-ball/chamfer internals.

use core::fmt;

use openrcad_topo::{Edge, Solid};

use crate::{
    chamfer_edges, fillet_circular_edge_chain, fillet_edges, ChamferError, RollingBallError,
};

/// Selected-edge operation kind.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BlendKind {
    /// Constant-radius rolling-ball fillet.
    Fillet,
    /// Constant-distance chamfer.
    Chamfer,
}

/// Radius/distance law for a contour.
#[derive(Clone, Debug, PartialEq)]
pub enum BlendLaw {
    /// One value along the whole contour.
    Constant(f64),
    /// Placeholder for the public API; intentionally not implemented yet.
    Variable,
}

impl BlendLaw {
    fn constant(&self) -> Option<f64> {
        match self {
            Self::Constant(value) => Some(*value),
            Self::Variable => None,
        }
    }
}

/// Coarse analytic hint for a contour's spine.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BlendCurveHint {
    /// Straight line or no special grouping.
    Line,
    /// Co-circular fragments should be solved as one logical contour.
    Circle,
}

/// A logical selected-edge contour.
#[derive(Clone, Debug)]
pub struct BlendContour {
    /// Ordered edge fragments in this contour.
    pub edges: Vec<Edge>,
    /// Fillet or chamfer.
    pub kind: BlendKind,
    /// Radius/distance law.
    pub law: BlendLaw,
    /// Optional analytic curve hint.
    pub curve_hint: Option<BlendCurveHint>,
}

impl BlendContour {
    /// Construct a constant-radius/distance contour from already-ordered edges.
    pub fn constant(
        edges: impl Into<Vec<Edge>>,
        kind: BlendKind,
        value: f64,
        curve_hint: Option<BlendCurveHint>,
    ) -> Self {
        Self {
            edges: edges.into(),
            kind,
            law: BlendLaw::Constant(value),
            curve_hint,
        }
    }
}

/// Error returned by the contour facade.
#[derive(Clone, Debug, PartialEq)]
pub enum BlendContourError {
    /// The contour has no edge fragments.
    EmptyContour,
    /// Variable laws are part of the API shape but not implemented yet.
    VariableLawUnsupported,
    /// Fillet failed.
    Fillet(RollingBallError),
    /// Chamfer failed.
    Chamfer(ChamferError),
}

impl fmt::Display for BlendContourError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyContour => f.write_str("blend contour: no edges were provided"),
            Self::VariableLawUnsupported => {
                f.write_str("blend contour: variable radius/distance laws are not implemented")
            }
            Self::Fillet(err) => write!(f, "{err}"),
            Self::Chamfer(err) => write!(f, "{err}"),
        }
    }
}

impl std::error::Error for BlendContourError {}

/// Apply one logical blend contour to a solid.
pub fn apply_blend_contour(
    solid: &Solid,
    contour: &BlendContour,
) -> Result<Solid, BlendContourError> {
    if contour.edges.is_empty() {
        return Err(BlendContourError::EmptyContour);
    }
    let value = contour
        .law
        .constant()
        .ok_or(BlendContourError::VariableLawUnsupported)?;

    match contour.kind {
        BlendKind::Fillet => {
            if matches!(contour.curve_hint, Some(BlendCurveHint::Circle)) && contour.edges.len() > 1
            {
                let spine = circular_spine_from_chain(&contour.edges)
                    .unwrap_or_else(|| contour.edges[0].clone());
                fillet_circular_edge_chain(solid, &contour.edges, &spine, value)
                    .map_err(BlendContourError::Fillet)
            } else {
                fillet_edges(solid, &contour.edges, value).map_err(BlendContourError::Fillet)
            }
        }
        BlendKind::Chamfer => {
            chamfer_edges(solid, &contour.edges, value).map_err(BlendContourError::Chamfer)
        }
    }
}

fn circular_spine_from_chain(edges: &[Edge]) -> Option<Edge> {
    let first = edges.first()?;
    let last = edges.last()?;
    let curve = first.curve().cloned()?;
    Some(Edge::new(
        Some(curve),
        first.first(),
        last.last(),
        first.source().clone(),
        last.target().clone(),
    ))
}
