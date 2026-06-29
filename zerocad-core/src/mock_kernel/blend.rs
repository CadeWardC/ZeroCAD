use super::*;

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

pub fn fillet_edge_with_hint(
    solid: &KernelSolid,
    p0: [f32; 3],
    p1: [f32; 3],
    curve: Option<&EdgeCurveHint>,
    radius: f32,
) -> Result<KernelSolid, String> {
    if let Some(hint @ EdgeCurveHint::Circle { .. }) = curve {
        let chain = circle_edge_requests(solid, hint).ok_or_else(|| {
            "curved circular-rim fillet could not be matched to the body topology".to_string()
        })?;
        if chain.len() == 1 {
            let contour = BlendContour::constant(
                chain,
                BlendKind::Fillet,
                radius as f64,
                Some(BlendCurveHint::Circle),
            );
            return apply_blend_contour(solid, &contour).map_err(|err| err.to_string());
        }
        let contour = BlendContour::constant(
            chain,
            BlendKind::Fillet,
            radius as f64,
            Some(BlendCurveHint::Circle),
        );
        return apply_blend_contour(solid, &contour).map_err(|err| err.to_string());
    }

    let a = Pnt::new(p0[0] as f64, p0[1] as f64, p0[2] as f64);
    let b = Pnt::new(p1[0] as f64, p1[1] as f64, p1[2] as f64);
    let e = Edge::between_points(a, b);
    let contour = BlendContour::constant(vec![e], BlendKind::Fillet, radius as f64, None);
    apply_blend_contour(solid, &contour).map_err(|err| err.to_string())
}

#[allow(dead_code)]
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
    if dc > (0.002 * r).max(1.0e-3) || (circle.radius() - r).abs() > (0.002 * r).max(1.0e-3) {
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
    let a = Pnt::new(p0[0] as f64, p0[1] as f64, p0[2] as f64);
    let b = Pnt::new(p1[0] as f64, p1[1] as f64, p1[2] as f64);
    let e = Edge::between_points(a, b);
    let contour = BlendContour::constant(vec![e], BlendKind::Chamfer, distance as f64, None);
    apply_blend_contour(solid, &contour).map_err(|err| err.to_string())
}
