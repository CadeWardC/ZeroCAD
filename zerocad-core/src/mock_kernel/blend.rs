use super::*;

/// Apply a persisted edge network through one canonical OpenRCAD operation.
/// No intermediate solid escapes this boundary.
pub(crate) fn blend_edge_network(
    solid: &KernelSolid,
    edges: &[crate::parametric::EdgeRef],
    value: f32,
    kind: crate::sketch::CornerKind,
    corner_mode: crate::parametric::EdgeCornerMode,
) -> Result<KernelOutcome, String> {
    if edges.is_empty() {
        return Err("edge blend selection is empty".into());
    }

    let mut kernel_edges = Vec::new();
    for edge in edges {
        if let Some(hint @ EdgeCurveHint::Circle { .. }) = edge.curve.as_ref() {
            let chain = circle_edge_requests(solid, hint).ok_or_else(|| {
                "selected circular edge could not be matched to the body topology".to_string()
            })?;
            kernel_edges.extend(chain);
        } else {
            kernel_edges.push(Edge::between_points(
                snap_point_to_topology(solid, edge.p0),
                snap_point_to_topology(solid, edge.p1),
            ));
        }
    }

    let request = openrcad::algo::BlendNetworkRequest {
        edges: kernel_edges,
        kind: match kind {
            crate::sketch::CornerKind::Fillet => openrcad::algo::BlendKind::Fillet,
            crate::sketch::CornerKind::Chamfer => openrcad::algo::BlendKind::Chamfer,
        },
        value: value as f64,
        curve_hint: edges
            .iter()
            .all(|edge| matches!(edge.curve, Some(EdgeCurveHint::Circle { .. })))
            .then_some(openrcad::algo::BlendCurveHint::Circle),
        corner_mode: match corner_mode {
            crate::parametric::EdgeCornerMode::Auto => openrcad::algo::BlendCornerMode::Auto,
            crate::parametric::EdgeCornerMode::Miter => openrcad::algo::BlendCornerMode::Miter,
            crate::parametric::EdgeCornerMode::RollingBall => {
                openrcad::algo::BlendCornerMode::RollingBall
            }
            crate::parametric::EdgeCornerMode::Setback => openrcad::algo::BlendCornerMode::Setback,
        },
    };
    let native = consume_operation(
        "edge blend network",
        openrcad::algo::blend_edge_network_operation_with_policy(
            solid,
            &request,
            &TolerancePolicy::STANDARD,
        ),
    );
    match native {
        Ok(outcome) => Ok(outcome),
        Err(native_error)
            if kind == crate::sketch::CornerKind::Fillet
                && matches!(
                    corner_mode,
                    crate::parametric::EdgeCornerMode::Auto
                        | crate::parametric::EdgeCornerMode::Miter
                )
                && edges.len() > 1
                && edges.iter().all(|edge| edge.curve.is_none())
                && edges
                    .iter()
                    .all(|edge| !edge_wedge_is_concave(solid, edge.p0, edge.p1)) =>
        {
            fillet_straight_edge_network_with_cutters(solid, edges, value).map_err(|fallback| {
                format!("native failed: {native_error}; simultaneous cutter failed: {fallback}")
            })
        }
        Err(native_error)
            if kind == crate::sketch::CornerKind::Chamfer
                && matches!(
                    corner_mode,
                    crate::parametric::EdgeCornerMode::Auto
                        | crate::parametric::EdgeCornerMode::Miter
                )
                && edges.iter().all(|edge| edge.curve.is_none()) =>
        {
            chamfer_straight_edge_network_with_cutters(solid, edges, value).map_err(|fallback| {
                format!("native failed: {native_error}; simultaneous cutter failed: {fallback}")
            })
        }
        Err(error) => Err(error),
    }
}

/// Boolean recut for a degree-two network of straight, orthogonal material
/// edges. Each exact circular crescent is generated from the original support
/// planes and extended through both selected endpoints. Applying all cutters to
/// a scratch solid lets adjacent cylindrical surfaces intersect and trim to one
/// shared miter seam; no intermediate body is exposed to the feature graph.
fn fillet_straight_edge_network_with_cutters(
    solid: &KernelSolid,
    edges: &[crate::parametric::EdgeRef],
    radius: f32,
) -> Result<KernelOutcome, String> {
    let policy = &TolerancePolicy::STANDARD;
    let mut combined_cutter: Option<KernelSolid> = None;
    for edge in edges {
        let p0 = Pnt::new(edge.p0[0] as f64, edge.p0[1] as f64, edge.p0[2] as f64);
        let p1 = Pnt::new(edge.p1[0] as f64, edge.p1[1] as f64, edge.p1[2] as f64);
        let tangent_dir = (p1 - p0)
            .normalized()
            .ok_or_else(|| "selected fillet edge is degenerate".to_string())?;
        let tangent = GeomVec::from_dir(tangent_dir);
        let n1 = GeomVec::new(edge.n1[0] as f64, edge.n1[1] as f64, edge.n1[2] as f64)
            .normalized()
            .ok_or_else(|| "first adjacent face normal is degenerate".to_string())?;
        let n2 = GeomVec::new(edge.n2[0] as f64, edge.n2[1] as f64, edge.n2[2] as f64)
            .normalized()
            .ok_or_else(|| "second adjacent face normal is degenerate".to_string())?;
        let n1 = GeomVec::from_dir(n1);
        let n2 = GeomVec::from_dir(n2);
        if n1.dot(&n2).abs() > 1.0e-6 {
            return Err("circular-crescent fallback requires orthogonal support planes".into());
        }

        // The two contact directions lie in their respective supporting faces
        // and point into the material wedge removed by a convex fillet.
        let contact_1 = (n2 * -1.0 + n1 * n1.dot(&n2))
            .normalized()
            .ok_or_else(|| "adjacent faces do not define a fillet wedge".to_string())?;
        let contact_2 = (n1 * -1.0 + n2 * n1.dot(&n2))
            .normalized()
            .ok_or_else(|| "adjacent faces do not define a fillet wedge".to_string())?;
        let contact_1 = GeomVec::from_dir(contact_1);
        let contact_2 = GeomVec::from_dir(contact_2);
        let r = radius as f64;
        let grow = (r * 0.05).max(0.2);
        let start = p0 - tangent * grow;
        let sweep = tangent * (p0.distance(&p1) + 2.0 * grow);
        let center = start + (contact_1 + contact_2) * r;
        // Build the cutter as an oversized rectangular prism minus the exact
        // tangent cylinder. Unlike a crescent profile, none of the tool's
        // planar faces are coincident with the body's supporting faces; only
        // the desired analytic cylinder meets them tangentially. This avoids a
        // boolean free edge at the two contact rails.
        let margin = grow;
        let a = start - contact_1 * margin - contact_2 * margin;
        let b = start + contact_1 * (r + margin) - contact_2 * margin;
        let c = start + contact_1 * (r + margin) + contact_2 * (r + margin);
        let d = start - contact_1 * margin + contact_2 * (r + margin);
        let profile = Face::new(
            Some(GeomSurface::plane(Plane::from_point_normal(
                start,
                tangent_dir,
            ))),
            Wire::from_edges([
                Edge::between_points(a, b),
                Edge::between_points(b, c),
                Edge::between_points(c, d),
                Edge::between_points(d, a),
            ]),
        );
        let envelope = consume_operation(
            "fillet network cutter envelope",
            openrcad::algo::prism_operation_with_policy(&profile, sweep, policy),
        )?
        .solid;
        let cylinder = consume_operation(
            "fillet network cutter cylinder",
            openrcad::primitives::make_cylinder_operation_with_policy(
                &Ax2::new(center - tangent * margin, tangent_dir),
                r,
                sweep.magnitude() + 2.0 * margin,
                policy,
            ),
        )?
        .solid;
        let cutter = difference_diagnostic(&envelope, &cylinder)
            .map_err(|error| format!("could not hollow the fillet cutter: {error}"))?;
        combined_cutter = Some(match combined_cutter.take() {
            None => cutter,
            Some(existing) => union_diagnostic(&existing, &cutter)
                .map_err(|error| format!("could not join fillet network cutters: {error}"))?,
        });
    }
    let cutter = combined_cutter.ok_or_else(|| "fillet network has no cutters".to_string())?;
    let current = difference_diagnostic(solid, &cutter)
        .map_err(|error| format!("could not apply the fillet network cutter: {error}"))?;
    consume_local_unary_operation("fillet edge network", solid, current, policy)
}

fn chamfer_straight_edge_network_with_cutters(
    solid: &KernelSolid,
    edges: &[crate::parametric::EdgeRef],
    distance: f32,
) -> Result<KernelOutcome, String> {
    let policy = &TolerancePolicy::STANDARD;
    let mut combined_cutter: Option<KernelSolid> = None;
    for edge in edges {
        let p0 = Pnt::new(edge.p0[0] as f64, edge.p0[1] as f64, edge.p0[2] as f64);
        let p1 = Pnt::new(edge.p1[0] as f64, edge.p1[1] as f64, edge.p1[2] as f64);
        let tangent = (p1 - p0)
            .normalized()
            .ok_or_else(|| "selected chamfer edge is degenerate".to_string())?;
        let n1 = GeomVec::new(edge.n1[0] as f64, edge.n1[1] as f64, edge.n1[2] as f64)
            .normalized()
            .ok_or_else(|| "first adjacent face normal is degenerate".to_string())?;
        let n2 = GeomVec::new(edge.n2[0] as f64, edge.n2[1] as f64, edge.n2[2] as f64)
            .normalized()
            .ok_or_else(|| "second adjacent face normal is degenerate".to_string())?;
        let n1 = GeomVec::from_dir(n1);
        let n2 = GeomVec::from_dir(n2);
        let along_face_1 = (n2 * -1.0 + n1 * n1.dot(&n2))
            .normalized()
            .ok_or_else(|| "adjacent faces do not define a chamfer wedge".to_string())?;
        let along_face_2 = (n1 * -1.0 + n2 * n1.dot(&n2))
            .normalized()
            .ok_or_else(|| "adjacent faces do not define a chamfer wedge".to_string())?;
        let offset_1 = GeomVec::from_dir(along_face_1) * distance as f64;
        let offset_2 = GeomVec::from_dir(along_face_2) * distance as f64;
        let tangent = GeomVec::from_dir(tangent);
        let grow = (distance as f64 * 0.05).max(0.2);
        let start = p0 - tangent * grow;
        let end = p1 + tangent * grow;
        let margin = grow;
        let rings = vec![
            vec![
                start
                    - GeomVec::from_dir(along_face_1) * margin
                    - GeomVec::from_dir(along_face_2) * margin,
                start + offset_1 + GeomVec::from_dir(along_face_1) * margin
                    - GeomVec::from_dir(along_face_2) * margin,
                start - GeomVec::from_dir(along_face_1) * margin
                    + offset_2
                    + GeomVec::from_dir(along_face_2) * margin,
            ],
            vec![
                end - GeomVec::from_dir(along_face_1) * margin
                    - GeomVec::from_dir(along_face_2) * margin,
                end + offset_1 + GeomVec::from_dir(along_face_1) * margin
                    - GeomVec::from_dir(along_face_2) * margin,
                end - GeomVec::from_dir(along_face_1) * margin
                    + offset_2
                    + GeomVec::from_dir(along_face_2) * margin,
            ],
        ];
        let cutter = consume_operation(
            "chamfer network cutter",
            openrcad::algo::skin_polygon_rings_operation_with_policy(&rings, policy),
        )?
        .solid;
        combined_cutter = Some(match combined_cutter.take() {
            None => cutter,
            Some(existing) => union_diagnostic(&existing, &cutter)
                .map_err(|error| format!("could not join chamfer network cutters: {error}"))?,
        });
    }
    let cutter = combined_cutter.ok_or_else(|| "chamfer network has no cutters".to_string())?;
    let current = difference_diagnostic(solid, &cutter)?;
    consume_local_unary_operation("chamfer edge network", solid, current, policy)
}

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
