use super::*;

/// Fillet every topological edge of one connected solid as a single validated
/// kernel operation. The returned value does not escape the canonical outcome
/// boundary unless representation validation and topology-history coverage are
/// both complete.
pub(crate) fn fillet_all_edges_strict(
    solid: &KernelSolid,
    radius: f32,
) -> Result<KernelOutcome, String> {
    consume_operation(
        "all-edge fillet",
        openrcad::algo::fillet_all_edges_strict_operation_with_policy(
            solid,
            radius as f64,
            &TolerancePolicy::STANDARD,
        )
        .map_err(format_all_edge_operation_error),
    )
}

/// Chamfer every topological edge of one connected solid atomically.
pub(crate) fn chamfer_all_edges_strict(
    solid: &KernelSolid,
    distance: f32,
) -> Result<KernelOutcome, String> {
    consume_operation(
        "all-edge chamfer",
        openrcad::algo::chamfer_all_edges_strict_operation_with_policy(
            solid,
            distance as f64,
            &TolerancePolicy::STANDARD,
        )
        .map_err(format_all_edge_operation_error),
    )
}

fn format_all_edge_operation_error(error: openrcad::algo::AllEdgeBlendOperationError) -> String {
    use openrcad::algo::{AllEdgeBlendError, AllEdgeBlendOperationError};

    match error {
        AllEdgeBlendOperationError::Blend(AllEdgeBlendError::BlockingEdges {
            kind,
            value,
            blockers,
        }) => {
            let details = blockers
                .iter()
                .map(|blocker| format!("#{} ({})", blocker.ordinal, blocker.failure))
                .collect::<Vec<_>>()
                .join(", ");
            format!(
                "all-edge {kind} at {value} was rejected by {} edge(s): {details}",
                blockers.len()
            )
        }
        other => other.to_string(),
    }
}

/// Round the edge running from `p0` to `p1` of `solid` by `radius`, using the
/// native rolling-ball blend (no booleans). The edge is located in the solid's
/// topology by matching its endpoints, so `p0`/`p1` are the world-space edge
/// endpoints captured in an [`crate::parametric::EdgeRef`]. Returns `Err` with a
/// human-readable reason when the edge isn't found, isn't a blendable corner, or
/// the blend fails — the caller surfaces it to the user instead of a generic
/// "couldn't be rounded".
pub fn fillet_edge(
    solid: &KernelSolid,
    p0: [f32; 3],
    p1: [f32; 3],
    radius: f32,
) -> Result<KernelSolid, String> {
    fillet_edge_with_hint(solid, p0, p1, None, radius)
}

/// True when the straight edge `p0..p1` of `solid` is a CONCAVE (reflex
/// material) edge — an inner pocket corner. Blending such an edge ADDS a
/// wedge of material, so the caller must swap the subtractive containment
/// gate for the additive one. False for convex edges and whenever the probe
/// is inconclusive (unlocatable edge, curved faces).
pub fn edge_wedge_is_concave(solid: &KernelSolid, p0: [f32; 3], p1: [f32; 3]) -> bool {
    let e = Edge::between_points(
        snap_point_to_topology(solid, p0),
        snap_point_to_topology(solid, p1),
    );
    openrcad::algo::edge_material_wedge_is_concave(solid, &e) == Some(true)
}

/// Snap a mesh-derived f32 endpoint to the exact kernel vertex it names.
///
/// `EdgeRef` endpoints are f32 casts of tessellated vertices, so they miss the
/// f64 B-Rep topology by up to ~|coord|·f32-eps (≈4e-7 at coordinate 15.2) —
/// orders of magnitude beyond the kernel's CONFUSION-scale edge matching. Only
/// exactly-representable coordinates (integer-sized boxes) ever matched without
/// this, which is why fillets located axis-aligned edges but failed on angled
/// sketch geometry. A point with no nearby vertex is returned unchanged
/// (circular-rim selections name arc midpoints, not vertices).
fn snap_point_to_topology(solid: &KernelSolid, p: [f32; 3]) -> Pnt {
    let q = Pnt::new(p[0] as f64, p[1] as f64, p[2] as f64);
    let mut best: Option<(f64, Pnt)> = None;
    for e in solid.edges() {
        for v in [e.start().point(), e.end().point()] {
            let d = v.distance(&q);
            if best.is_none_or(|(bd, _)| d < bd) {
                best = Some((d, v));
            }
        }
    }
    // f32 quantization noise stays below ~1e-4 for coordinates up to ~1e3;
    // real model vertices are far more than 1e-3 apart.
    match best {
        Some((d, v)) if d <= 1.0e-3 => v,
        _ => q,
    }
}

pub fn fillet_edge_with_hint(
    solid: &KernelSolid,
    p0: [f32; 3],
    p1: [f32; 3],
    curve: Option<&EdgeCurveHint>,
    radius: f32,
) -> Result<KernelSolid, String> {
    if let Some(hint @ EdgeCurveHint::Circle { .. }) = curve {
        let mut chain = circle_edge_requests(solid, hint).ok_or_else(|| {
            "curved circular-rim fillet could not be matched to the body topology".to_string()
        })?;
        append_tangent_line_edges(solid, &mut chain);
        if chain.len() == 1 {
            let contour = BlendContour::constant(
                chain,
                BlendKind::Fillet,
                radius as f64,
                Some(BlendCurveHint::Circle),
            );
            return consume_blend_contour(solid, &contour);
        }
        let contour = BlendContour::constant(
            chain,
            BlendKind::Fillet,
            radius as f64,
            Some(BlendCurveHint::Circle),
        );
        return consume_blend_contour(solid, &contour);
    }

    let e = Edge::between_points(
        snap_point_to_topology(solid, p0),
        snap_point_to_topology(solid, p1),
    );
    let contour = BlendContour::constant(vec![e], BlendKind::Fillet, radius as f64, None);
    consume_blend_contour(solid, &contour)
}

fn consume_blend_contour(
    solid: &KernelSolid,
    contour: &BlendContour,
) -> Result<KernelSolid, String> {
    consume_operation(
        "edge blend",
        openrcad::algo::blend_contour_operation_with_policy(
            solid,
            contour,
            &TolerancePolicy::STANDARD,
        ),
    )
    .map(|outcome| outcome.solid)
}

fn append_tangent_line_edges(solid: &KernelSolid, chain: &mut Vec<Edge>) {
    let circular = chain.clone();
    for arc in &circular {
        let Some(GeomCurve::Circle(circle)) = arc.curve() else {
            continue;
        };
        for point in [arc.source().point(), arc.target().point()] {
            let radial = point - circle.center();
            let tangent = GeomVec::from_dir(circle.axis()).cross(&radial);
            let Some(tangent) = tangent.normalized() else {
                continue;
            };
            for candidate in solid.edges() {
                if !matches!(candidate.curve(), Some(GeomCurve::Line(_)) | None) {
                    continue;
                }
                let (a, b) = (candidate.source().point(), candidate.target().point());
                if a.distance(&point) > 1.0e-5 && b.distance(&point) > 1.0e-5 {
                    continue;
                }
                let Some(direction) = (b - a).normalized() else {
                    continue;
                };
                if direction.dot(&tangent).abs() < 0.9995 {
                    continue;
                }
                if !chain.iter().any(|edge| {
                    (edge.source().point().distance(&a) < 1.0e-5
                        && edge.target().point().distance(&b) < 1.0e-5)
                        || (edge.source().point().distance(&b) < 1.0e-5
                            && edge.target().point().distance(&a) < 1.0e-5)
                }) {
                    chain.push(candidate);
                }
            }
        }
    }
}

pub(crate) fn circle_edge_requests(solid: &KernelSolid, hint: &EdgeCurveHint) -> Option<Vec<Edge>> {
    let EdgeCurveHint::Circle {
        center,
        axis,
        x_dir,
        radius,
        start,
        end,
        closed,
    } = *hint
    else {
        return None;
    };
    if radius <= 1.0e-5 {
        return None;
    }

    let mut matching = Vec::new();
    for edge in solid.edges() {
        let Some(GeomCurve::Circle(circle)) = edge.curve() else {
            continue;
        };
        if !circle_matches_hint(circle, center, axis, radius) {
            continue;
        }
        if closed
            || circle_edge_midpoint_in_span(&edge, center, axis, x_dir, start as f64, end as f64)
        {
            matching.push(edge);
        }
    }
    matching.sort_by(|a, b| {
        a.first()
            .partial_cmp(&b.first())
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    (!matching.is_empty()).then_some(matching)
}

pub(crate) fn circle_matches_hint(
    circle: &Circle,
    center: [f32; 3],
    axis: [f32; 3],
    radius: f32,
) -> bool {
    let c = circle.center();
    let dc = ((c.x() - center[0] as f64).powi(2)
        + (c.y() - center[1] as f64).powi(2)
        + (c.z() - center[2] as f64).powi(2))
    .sqrt();
    let r = radius as f64;
    // 1% of radius: the GUI's circle-fit hint residual bound (picking.rs) is ~1%,
    // so a tighter 0.2% gate rejected legitimate matches on committed bodies. The
    // axis gate below still keeps distinct coaxial circles apart.
    if dc > (0.01 * r).max(1.0e-3) || (circle.radius() - r).abs() > (0.01 * r).max(1.0e-3) {
        return false;
    }
    let a = GeomVec::from_dir(circle.axis());
    let b = GeomVec::new(axis[0] as f64, axis[1] as f64, axis[2] as f64);
    let bl = b.magnitude();
    bl > 1.0e-9 && (a.dot(&b) / bl).abs() > 0.999
}

pub(crate) fn circle_edge_midpoint_in_span(
    edge: &Edge,
    center: [f32; 3],
    axis: [f32; 3],
    x_dir: [f32; 3],
    start: f64,
    end: f64,
) -> bool {
    let Some(GeomCurve::Circle(circle)) = edge.curve() else {
        return false;
    };
    let mid = circle.point((edge.first() + edge.last()) * 0.5);
    let Some(angle) = hint_circle_angle(mid, center, axis, x_dir) else {
        return false;
    };
    angle_in_span(angle, start, end, 0.05)
}

pub(crate) fn normalized_dir(v: [f32; 3]) -> Option<Dir> {
    let l = ((v[0] as f64).powi(2) + (v[1] as f64).powi(2) + (v[2] as f64).powi(2)).sqrt();
    (l > 1.0e-9).then(|| Dir::new(v[0] as f64 / l, v[1] as f64 / l, v[2] as f64 / l))
}

pub(crate) fn hint_circle_angle(
    point: Pnt,
    center: [f32; 3],
    axis: [f32; 3],
    x_dir: [f32; 3],
) -> Option<f64> {
    let c = Pnt::new(center[0] as f64, center[1] as f64, center[2] as f64);
    let axis = GeomVec::from_dir(normalized_dir(axis)?);
    let mut x = GeomVec::from_dir(normalized_dir(x_dir)?);
    x = x - axis * x.dot(&axis);
    let x = GeomVec::from_dir(x.normalized()?);
    let y = axis.cross(&x);
    let v = point - c;
    Some(v.dot(&y).atan2(v.dot(&x)))
}

pub(crate) fn angle_in_span(angle: f64, start: f64, end: f64, tol: f64) -> bool {
    let span = end - start;
    if span.abs() >= std::f64::consts::TAU - tol {
        return true;
    }
    if span >= 0.0 {
        let mut rel = angle - start;
        while rel < -tol {
            rel += std::f64::consts::TAU;
        }
        while rel > std::f64::consts::TAU + tol {
            rel -= std::f64::consts::TAU;
        }
        rel <= span + tol
    } else {
        let mut rel = start - angle;
        while rel < -tol {
            rel += std::f64::consts::TAU;
        }
        while rel > std::f64::consts::TAU + tol {
            rel -= std::f64::consts::TAU;
        }
        rel <= -span + tol
    }
}

/// Bevel the edge running from `p0` to `p1` of `solid` by `distance`, using the
/// native selected-edge chamfer path (no boolean cutter fallback).
pub fn chamfer_edge(
    solid: &KernelSolid,
    p0: [f32; 3],
    p1: [f32; 3],
    distance: f32,
) -> Result<KernelSolid, String> {
    chamfer_edge_with_hint(solid, p0, p1, None, distance)
}

/// Bevel the edge from `p0` to `p1`, routing a circular-rim selection (a `Circle`
/// hint) through the analytic cone-frustum chain solver — the chamfer analogue of
/// [`fillet_edge_with_hint`].
pub fn chamfer_edge_with_hint(
    solid: &KernelSolid,
    p0: [f32; 3],
    p1: [f32; 3],
    curve: Option<&EdgeCurveHint>,
    distance: f32,
) -> Result<KernelSolid, String> {
    if let Some(hint @ EdgeCurveHint::Circle { .. }) = curve {
        let mut chain = circle_edge_requests(solid, hint).ok_or_else(|| {
            "curved circular-rim chamfer could not be matched to the body topology".to_string()
        })?;
        append_tangent_line_edges(solid, &mut chain);
        let contour = BlendContour::constant(
            chain,
            BlendKind::Chamfer,
            distance as f64,
            Some(BlendCurveHint::Circle),
        );
        return consume_blend_contour(solid, &contour);
    }

    let e = Edge::between_points(
        snap_point_to_topology(solid, p0),
        snap_point_to_topology(solid, p1),
    );
    let contour = BlendContour::constant(vec![e], BlendKind::Chamfer, distance as f64, None);
    consume_blend_contour(solid, &contour)
}
