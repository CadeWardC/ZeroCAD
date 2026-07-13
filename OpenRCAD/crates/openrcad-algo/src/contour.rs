//! Lightweight contour input for selected-edge blends.
//!
//! This is intentionally a thin facade over the existing selected-edge fillet
//! and chamfer solvers. It gives applications one typed request shape without
//! replacing the hand-tuned rolling-ball/chamfer internals.

use core::fmt;

use openrcad_foundation::Vec as GeomVec;
use openrcad_geom::GeomCurve;
use openrcad_topo::{Edge, Solid};

use crate::{
    chamfer_circular_edge_chain, chamfer_edges, chamfer_tangent_edge_chain,
    fillet_circular_edge_chain, fillet_edges, fillet_tangent_edge_chain, ChamferError,
    RollingBallError,
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

    let mixed = is_mixed_tangent_chain(&contour.edges);
    if mixed {
        return match contour.kind {
            BlendKind::Fillet => fillet_tangent_edge_chain(solid, &contour.edges, value)
                .map_err(BlendContourError::Fillet),
            BlendKind::Chamfer => {
                let circular: Vec<Edge> = contour
                    .edges
                    .iter()
                    .filter(|edge| matches!(edge.curve(), Some(GeomCurve::Circle(_))))
                    .cloned()
                    .collect();
                let spine =
                    circular_spine_from_chain(&circular).unwrap_or_else(|| circular[0].clone());
                chamfer_tangent_edge_chain(solid, &contour.edges, &spine, value)
                    .map_err(BlendContourError::Chamfer)
            }
        };
    }

    match contour.kind {
        BlendKind::Fillet => {
            // A Circle hint routes to the co-circular chain solver for ANY edge
            // count — a single full-circle rim (a plain cylinder top, a bored
            // hole) is as much a circular contour as a multi-arc chain. If the
            // chain solver declines (returns `Err`), fall back to the generic
            // per-edge fillet, preserving the previous behaviour.
            if matches!(contour.curve_hint, Some(BlendCurveHint::Circle)) {
                let spine = circular_spine_from_chain(&contour.edges)
                    .unwrap_or_else(|| contour.edges[0].clone());
                match fillet_circular_edge_chain(solid, &contour.edges, &spine, value) {
                    Ok(result) => Ok(result),
                    Err(_) => fillet_edges(solid, &contour.edges, value)
                        .map_err(BlendContourError::Fillet),
                }
            } else {
                fillet_edges(solid, &contour.edges, value).map_err(BlendContourError::Fillet)
            }
        }
        BlendKind::Chamfer => {
            // A Circle hint routes a closed rim to the cone-frustum chain solver;
            // on decline, fall back to the generic per-edge chamfer.
            if matches!(contour.curve_hint, Some(BlendCurveHint::Circle)) {
                let spine = circular_spine_from_chain(&contour.edges)
                    .unwrap_or_else(|| contour.edges[0].clone());
                match chamfer_circular_edge_chain(solid, &contour.edges, &spine, value) {
                    Ok(result) => Ok(result),
                    Err(_) => chamfer_edges(solid, &contour.edges, value)
                        .map_err(BlendContourError::Chamfer),
                }
            } else {
                chamfer_edges(solid, &contour.edges, value).map_err(BlendContourError::Chamfer)
            }
        }
    }
}

fn is_mixed_tangent_chain(edges: &[Edge]) -> bool {
    let circles: Vec<&Edge> = edges
        .iter()
        .filter(|edge| matches!(edge.curve(), Some(GeomCurve::Circle(_))))
        .collect();
    let lines: Vec<&Edge> = edges
        .iter()
        .filter(|edge| !matches!(edge.curve(), Some(GeomCurve::Circle(_))))
        .collect();
    if circles.is_empty() || lines.is_empty() {
        return false;
    }
    lines.iter().all(|line| {
        let (a, b) = (line.source().point(), line.target().point());
        let Some(line_dir) = (b - a).normalized() else {
            return false;
        };
        circles.iter().any(|arc| {
            let Some(GeomCurve::Circle(circle)) = arc.curve() else {
                return false;
            };
            [arc.source().point(), arc.target().point()]
                .into_iter()
                .any(|point| {
                    if point.distance(&a) > 1.0e-5 && point.distance(&b) > 1.0e-5 {
                        return false;
                    }
                    let tangent =
                        GeomVec::from_dir(circle.axis()).cross(&(point - circle.center()));
                    tangent
                        .normalized()
                        .is_some_and(|tangent| tangent.dot(&line_dir).abs() >= 0.985)
                })
        })
    })
}

/// One contiguous spine edge covering the chain's co-circular fragments.
///
/// The fragments arrive in arbitrary order (face-wire order, sorted-by-param
/// order — whatever the caller had) and their param intervals live on a
/// periodic circle, so a chain that crosses the 0/TAU seam cannot be spanned by
/// naively taking `first.first()..last.last()`: that picks the COMPLEMENTARY
/// arc and puts the spine's free endpoints at mid-chain junctions. Instead,
/// order the fragments by angular adjacency and unwrap across the seam, so the
/// spine's param interval traverses exactly the selected arc and its vertices
/// are the chain's true free ends. Returns `None` (→ per-edge fallback) when
/// the fragments are not one contiguous co-circular chain.
fn circular_spine_from_chain(edges: &[Edge]) -> Option<Edge> {
    use core::f64::consts::TAU;
    use openrcad_geom::{Curve, GeomCurve};
    use openrcad_topo::Vertex;

    let first = edges.first()?;
    let curve = first.curve().cloned()?;
    let GeomCurve::Circle(reference) = &curve else {
        // Non-circular "circle" hints keep the old naive span.
        let last = edges.last()?;
        return Some(Edge::new(
            Some(curve.clone()),
            first.first(),
            last.last(),
            first.source().clone(),
            last.target().clone(),
        ));
    };

    // Canonical RIGHT-handed copy of the rim circle: the spine's param frame
    // must agree with `spine_parameter_for_point`'s `axis × x` convention, and
    // a boolean-produced rim circle can carry a left-handed stored frame.
    let center = reference.center();
    let reference = openrcad_geom::Circle::new(
        openrcad_foundation::Ax3::new_axes(
            center,
            reference.axis(),
            reference.position().x_direction(),
        ),
        reference.radius(),
    );
    let curve = GeomCurve::circle(reference);

    // Angle of `p` in that frame, normalized to [0, TAU).
    let x = openrcad_foundation::Vec::from_dir(reference.position().x_direction());
    let y = openrcad_foundation::Vec::from_dir(reference.axis()).cross(&x);
    let angle_of = |p: openrcad_foundation::Pnt| -> f64 {
        let v = p - center;
        let mut a = v.dot(&y).atan2(v.dot(&x));
        if a < 0.0 {
            a += TAU;
        }
        a
    };

    // Each fragment as an ascending angular interval [start, start + span) in
    // the reference frame, anchored on its own midpoint (a circle's param IS
    // its angle, so the raw param span is the angular extent).
    let mut spans: Vec<(f64, f64)> = Vec::with_capacity(edges.len());
    for e in edges {
        let c = e.curve()?;
        let span = (e.last() - e.first()).abs();
        if span >= TAU - 1.0e-6 {
            // A single full-circle edge is its own spine.
            let t0 = angle_of(c.point(e.first()));
            return Some(Edge::new(
                Some(curve.clone()),
                t0,
                t0 + TAU,
                Vertex::new(reference.point(t0)),
                Vertex::new(reference.point(t0 + TAU)),
            ));
        }
        if span <= 1.0e-9 {
            continue;
        }
        let mid = angle_of(c.point(0.5 * (e.first() + e.last())));
        let mut start = mid - 0.5 * span;
        if start < 0.0 {
            start += TAU;
        }
        spans.push((start, span));
    }
    if spans.is_empty() {
        return None;
    }

    let ang_tol = 1.0e-4;
    let close = |a: f64, b: f64| -> bool {
        let d = (a - b).rem_euclid(TAU);
        d <= ang_tol || TAU - d <= ang_tol
    };

    // A fragment whose start no other fragment ends at is the chain's free
    // start; none ⇒ the chain wraps the full circle.
    let free_start = spans.iter().position(|&(s, _)| {
        !spans
            .iter()
            .any(|&(os, ospan)| close((os + ospan).rem_euclid(TAU), s))
    });
    let start_idx = match free_start {
        Some(i) => i,
        None => {
            let total: f64 = spans.iter().map(|&(_, sp)| sp).sum();
            if total < TAU - 1.0e-3 {
                return None; // disjoint arcs that individually chain to nothing
            }
            let t0 = spans[0].0;
            return Some(Edge::new(
                Some(curve.clone()),
                t0,
                t0 + TAU,
                Vertex::new(reference.point(t0)),
                Vertex::new(reference.point(t0 + TAU)),
            ));
        }
    };

    // Walk the chain start → end, accumulating the unwrapped total span.
    let t0 = spans[start_idx].0;
    let mut total = spans[start_idx].1;
    let mut used = vec![false; spans.len()];
    used[start_idx] = true;
    for _ in 1..spans.len() {
        let cur_end = (t0 + total).rem_euclid(TAU);
        let Some(next) = spans
            .iter()
            .enumerate()
            .position(|(i, &(s, _))| !used[i] && close(s, cur_end))
        else {
            return None; // gap: not one contiguous chain
        };
        used[next] = true;
        total += spans[next].1;
    }
    if total > TAU + ang_tol {
        return None; // overlapping fragments
    }

    Some(Edge::new(
        Some(curve.clone()),
        t0,
        t0 + total,
        Vertex::new(reference.point(t0)),
        Vertex::new(reference.point(t0 + total)),
    ))
}
