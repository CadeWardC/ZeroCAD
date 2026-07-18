//! Read-only inspection operations over evaluated kernel bodies.

use super::*;
use crate::mock_kernel::{common_bodies_with_history, CommonBodiesError};

#[derive(Debug, Clone, PartialEq)]
pub struct BodyInspection {
    pub body_id: String,
    pub part_count: usize,
    /// Enclosed volume. Open mesh bodies have no well-defined enclosed volume
    /// and report `None` rather than a misleading numeric zero.
    pub volume_mm3: Option<f64>,
    pub surface_area_mm2: f64,
    pub centroid: [f64; 3],
    pub bounds_min: [f64; 3],
    pub bounds_max: [f64; 3],
    /// Display/material density selected by the caller. Geometry remains in
    /// millimetres, so `mass_g = volume_mm3 / 1000 * density_g_cm3` when the
    /// body has a defined enclosed volume.
    pub density_g_cm3: f64,
    /// Density-derived mass. This is unavailable for open mesh bodies.
    pub mass_g: Option<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct InterferencePair {
    pub first_body: String,
    pub second_body: String,
    pub volume_mm3: f64,
    pub centroid: [f64; 3],
}

#[derive(Debug, Clone, PartialEq)]
pub struct FaceInspection {
    pub body_id: String,
    pub face_id: u32,
    pub area_mm2: f64,
}

/// Exact kernel-curve measurements for a selected edge. The tangent is sampled
/// at the edge parameter midpoint and normalized; length is integrated from the
/// stored 3D curve, never from viewport chords.
#[derive(Debug, Clone, PartialEq)]
pub struct EdgeInspection {
    pub body_id: String,
    pub length_mm: f64,
    pub midpoint: [f64; 3],
    pub tangent: [f64; 3],
    pub curve_kind: String,
}

#[derive(Debug, Clone, PartialEq)]
pub struct EdgePairInspection {
    pub minimum_distance_mm: f64,
    pub tangent_angle_deg: f64,
}

impl ParametricGraph {
    /// Measure one evaluated body from strict, fine tessellations of its B-Rep
    /// parts. This path never reads the viewport mesh and therefore remains
    /// stable when display quality changes.
    pub fn inspect_body(&self, body_id: &str) -> Result<BodyInspection, String> {
        self.inspect_body_with_density(body_id, 1.0)
    }

    pub fn inspect_body_with_density(
        &self,
        body_id: &str,
        density_g_cm3: f64,
    ) -> Result<BodyInspection, String> {
        if !density_g_cm3.is_finite() || density_g_cm3 < 0.0 {
            return Err("material density must be finite and non-negative".to_string());
        }
        let hidden = std::collections::HashSet::new();
        let (live, _) = self.build_live(&hidden, false)?;
        let body = live
            .iter()
            .find(|body| body.id == body_id)
            .ok_or_else(|| format!("body '{body_id}' does not exist"))?;
        inspect_live_body(body, density_g_cm3)
    }

    /// Measure one picked display face from a fresh strict kernel
    /// tessellation. Face ids use the same analytic-surface grouping and
    /// multipart rebasing as the viewport mesh.
    pub fn inspect_face(&self, body_id: &str, face_id: u32) -> Result<FaceInspection, String> {
        let hidden = std::collections::HashSet::new();
        let (live, _) = self.build_live(&hidden, false)?;
        let body = live
            .iter()
            .find(|body| body.id == body_id)
            .ok_or_else(|| format!("body '{body_id}' does not exist"))?;
        if body.parts.is_empty() {
            let mesh = body
                .pristine
                .as_deref()
                .ok_or_else(|| format!("mesh body '{body_id}' has no display geometry"))?;
            let mut area = 0.0;
            let mut found = false;
            for (triangle, &triangle_face) in mesh.indices.chunks_exact(3).zip(&mesh.face_ids) {
                if triangle_face != face_id {
                    continue;
                }
                let Some([a, b, c]) = mesh_triangle(mesh, triangle) else {
                    continue;
                };
                found = true;
                area += triangle_area(a, b, c);
            }
            return found
                .then_some(FaceInspection {
                    body_id: body_id.to_string(),
                    face_id,
                    area_mm2: area,
                })
                .ok_or_else(|| format!("face {face_id} does not exist on mesh body '{body_id}'"));
        }
        let mut face_offset = 0_u32;
        let mut area = 0.0;
        let mut found = false;
        for part in &body.parts {
            let mesh = openrcad::mesh::tessellate_checked(part, 0.005, 0.05)
                .map_err(|error| format!("strict face tessellation failed: {error}"))?;
            let surface_groups = crate::mock_kernel::cylinder_surface_groups(part);
            let mut canonical = std::collections::HashMap::new();
            for (index, group) in surface_groups.iter().copied().enumerate() {
                canonical.entry(group).or_insert(index as u32);
            }
            let display_face = |local: u32| {
                surface_groups
                    .get(local as usize)
                    .and_then(|group| canonical.get(group))
                    .copied()
                    .unwrap_or(local)
            };
            let local_max = mesh.face_ids.iter().copied().map(display_face).max();
            for (triangle, local_face) in mesh.triangles.iter().zip(&mesh.face_ids) {
                if display_face(*local_face) + face_offset != face_id {
                    continue;
                }
                found = true;
                let [a, b, c] = triangle.map(|index| mesh.vertices[index as usize]);
                let ab = b - a;
                let ac = c - a;
                area += 0.5 * ab.cross(&ac).magnitude();
            }
            if let Some(local_max) = local_max {
                face_offset += local_max + 1;
            }
        }
        if !found {
            return Err(format!("face {face_id} does not exist on body '{body_id}'"));
        }
        Ok(FaceInspection {
            body_id: body_id.to_string(),
            face_id,
            area_mm2: area,
        })
    }

    /// Measure a captured edge from the evaluated B-Rep curve. Closed circular
    /// display groups retain an exact analytic hint because one logical rim may
    /// consist of several physical coedges; every other B-Rep edge is resolved
    /// against the selected endpoints and integrated directly.
    pub fn inspect_edge(
        &self,
        body_id: &str,
        reference: &EdgeRef,
    ) -> Result<EdgeInspection, String> {
        let hidden = std::collections::HashSet::new();
        let (live, _) = self.build_live(&hidden, false)?;
        let body = live
            .iter()
            .find(|body| body.id == body_id)
            .ok_or_else(|| format!("body '{body_id}' does not exist"))?;
        inspect_live_edge(body, reference)
    }

    /// Approximate minimum separation and tangent angle for two selected
    /// kernel curves. Edge resolution accepts the nearest endpoint match within
    /// five percent of the captured edge scale; closest parameters come from a
    /// deterministic 32x32 grid followed by bounded refinement. The result is
    /// independent of viewport tessellation, but is not an exact curve-extrema
    /// solver.
    pub fn inspect_edge_pair(
        &self,
        first_body: &str,
        first: &EdgeRef,
        second_body: &str,
        second: &EdgeRef,
    ) -> Result<EdgePairInspection, String> {
        let hidden = std::collections::HashSet::new();
        let (live, _) = self.build_live(&hidden, false)?;
        let first_live = live
            .iter()
            .find(|body| body.id == first_body)
            .ok_or_else(|| format!("body '{first_body}' does not exist"))?;
        let second_live = live
            .iter()
            .find(|body| body.id == second_body)
            .ok_or_else(|| format!("body '{second_body}' does not exist"))?;
        let first_curve = resolved_inspection_curve(first_live, first)?;
        let second_curve = resolved_inspection_curve(second_live, second)?;
        let (first_parameter, second_parameter, distance) =
            closest_curve_parameters(&first_curve, &second_curve);
        let first_tangent = first_curve.tangent(first_parameter);
        let second_tangent = second_curve.tangent(second_parameter);
        let cosine = dot3(first_tangent, second_tangent).clamp(-1.0, 1.0);
        Ok(EdgePairInspection {
            minimum_distance_mm: distance,
            tangent_angle_deg: cosine.acos().to_degrees(),
        })
    }

    /// Exact Common-based interference check. Contact-only pairs are omitted;
    /// every returned row has strictly more than `minimum_volume_mm3` common
    /// volume. Inputs are sorted/deduplicated for deterministic pair order.
    pub fn inspect_interference(
        &self,
        body_ids: &[String],
        minimum_volume_mm3: f64,
    ) -> Result<Vec<InterferencePair>, String> {
        if !minimum_volume_mm3.is_finite() || minimum_volume_mm3 < 0.0 {
            return Err("minimum interference volume must be finite and non-negative".to_string());
        }
        let hidden = std::collections::HashSet::new();
        let (live, _) = self.build_live(&hidden, false)?;
        let mut ids = body_ids.to_vec();
        ids.sort();
        ids.dedup();
        if ids.len() < 2 {
            return Err("interference checking needs at least two bodies".to_string());
        }
        let mut reports = Vec::new();
        for first_index in 0..ids.len() {
            for second_index in first_index + 1..ids.len() {
                let first = live
                    .iter()
                    .find(|body| body.id == ids[first_index])
                    .ok_or_else(|| format!("body '{}' does not exist", ids[first_index]))?;
                let second = live
                    .iter()
                    .find(|body| body.id == ids[second_index])
                    .ok_or_else(|| format!("body '{}' does not exist", ids[second_index]))?;
                if first.parts.is_empty() || second.parts.is_empty() {
                    return Err(format!(
                        "interference checking requires B-Rep solids; '{}' or '{}' is a mesh body",
                        first.id, second.id
                    ));
                }
                let mut volume = 0.0;
                let mut centroid_sum = [0.0_f64; 3];
                for first_part in &first.parts {
                    for second_part in &second.parts {
                        let common = match common_bodies_with_history(first_part, second_part, None)
                        {
                            Ok(common) => common,
                            Err(CommonBodiesError::Empty) => continue,
                            Err(CommonBodiesError::Failed(error)) => return Err(error),
                        };
                        for solid in common.bodies {
                            let properties = solid_mass_properties(&solid)?;
                            if properties.volume <= minimum_volume_mm3 {
                                continue;
                            }
                            volume += properties.volume;
                            for (sum, coordinate) in
                                centroid_sum.iter_mut().zip(properties.centroid)
                            {
                                *sum += coordinate * properties.volume;
                            }
                        }
                    }
                }
                if volume > minimum_volume_mm3 {
                    reports.push(InterferencePair {
                        first_body: first.id.clone(),
                        second_body: second.id.clone(),
                        volume_mm3: volume,
                        centroid: centroid_sum.map(|component| component / volume),
                    });
                }
            }
        }
        Ok(reports)
    }
}

#[derive(Clone)]
enum InspectionCurve {
    Kernel(openrcad::topo::Edge),
    Circle {
        center: [f64; 3],
        axis: [f64; 3],
        x_dir: [f64; 3],
        radius: f64,
        first: f64,
        last: f64,
    },
    Segment {
        first: [f64; 3],
        last: [f64; 3],
    },
}

impl InspectionCurve {
    fn bounds(&self) -> (f64, f64) {
        match self {
            Self::Kernel(edge) => (edge.first(), edge.last()),
            Self::Circle { first, last, .. } => (*first, *last),
            Self::Segment { .. } => (0.0, 1.0),
        }
    }

    fn point(&self, parameter: f64) -> [f64; 3] {
        use openrcad::geom::Curve;
        match self {
            Self::Kernel(edge) => edge.curve().map_or_else(
                || {
                    let (first, last) = (edge.start().point(), edge.end().point());
                    let span = (edge.last() - edge.first()).abs().max(f64::EPSILON);
                    let fraction = ((parameter - edge.first()) / span).clamp(0.0, 1.0);
                    [
                        first.x() + (last.x() - first.x()) * fraction,
                        first.y() + (last.y() - first.y()) * fraction,
                        first.z() + (last.z() - first.z()) * fraction,
                    ]
                },
                |curve| {
                    let point = curve.point(parameter);
                    [point.x(), point.y(), point.z()]
                },
            ),
            Self::Circle {
                center,
                axis,
                x_dir,
                radius,
                ..
            } => {
                let y_dir = cross3(*axis, *x_dir);
                let (sin, cos) = parameter.sin_cos();
                [
                    center[0] + radius * (x_dir[0] * cos + y_dir[0] * sin),
                    center[1] + radius * (x_dir[1] * cos + y_dir[1] * sin),
                    center[2] + radius * (x_dir[2] * cos + y_dir[2] * sin),
                ]
            }
            Self::Segment { first, last } => {
                let t = parameter.clamp(0.0, 1.0);
                [
                    first[0] + (last[0] - first[0]) * t,
                    first[1] + (last[1] - first[1]) * t,
                    first[2] + (last[2] - first[2]) * t,
                ]
            }
        }
    }

    fn derivative(&self, parameter: f64) -> [f64; 3] {
        use openrcad::geom::Curve;
        match self {
            Self::Kernel(edge) => edge.curve().map_or_else(
                || {
                    let first = edge.start().point();
                    let last = edge.end().point();
                    [
                        last.x() - first.x(),
                        last.y() - first.y(),
                        last.z() - first.z(),
                    ]
                },
                |curve| {
                    let derivative = curve.d1(parameter).1;
                    [derivative.x(), derivative.y(), derivative.z()]
                },
            ),
            Self::Circle {
                axis,
                x_dir,
                radius,
                ..
            } => {
                let y_dir = cross3(*axis, *x_dir);
                let (sin, cos) = parameter.sin_cos();
                [
                    radius * (-x_dir[0] * sin + y_dir[0] * cos),
                    radius * (-x_dir[1] * sin + y_dir[1] * cos),
                    radius * (-x_dir[2] * sin + y_dir[2] * cos),
                ]
            }
            Self::Segment { first, last } => {
                [last[0] - first[0], last[1] - first[1], last[2] - first[2]]
            }
        }
    }

    fn tangent(&self, parameter: f64) -> [f64; 3] {
        normalize3(self.derivative(parameter))
    }

    fn length(&self) -> f64 {
        let (first, last) = self.bounds();
        if let Self::Circle { radius, .. } = self {
            return radius * (last - first).abs();
        }
        let speed = |parameter: f64| magnitude3(self.derivative(parameter));
        let middle = (first + last) * 0.5;
        let whole = simpson(first, last, speed(first), speed(middle), speed(last));
        adaptive_simpson(
            &speed,
            first,
            last,
            speed(first),
            speed(middle),
            speed(last),
            whole,
            12,
        )
    }
}

fn inspect_live_edge(body: &LiveBody, reference: &EdgeRef) -> Result<EdgeInspection, String> {
    let curve = resolved_inspection_curve(body, reference)?;
    let (first, last) = curve.bounds();
    let parameter = (first + last) * 0.5;
    let curve_kind = match &curve {
        InspectionCurve::Kernel(edge) => edge.curve().map_or("degenerate", |curve| match curve {
            openrcad::geom::GeomCurve::Line(_) => "line",
            openrcad::geom::GeomCurve::Circle(_) => "circle",
            openrcad::geom::GeomCurve::Ellipse(_) => "ellipse",
            openrcad::geom::GeomCurve::Parabola(_) => "parabola",
            openrcad::geom::GeomCurve::Hyperbola(_) => "hyperbola",
            openrcad::geom::GeomCurve::BSpline(_) => "b-spline",
            openrcad::geom::GeomCurve::Helix(_) => "helix",
        }),
        InspectionCurve::Circle { .. } => "circle",
        InspectionCurve::Segment { .. } => "line",
    };
    Ok(EdgeInspection {
        body_id: body.id.clone(),
        length_mm: curve.length(),
        midpoint: curve.point(parameter),
        tangent: curve.tangent(parameter),
        curve_kind: curve_kind.to_string(),
    })
}

fn resolved_inspection_curve(
    body: &LiveBody,
    reference: &EdgeRef,
) -> Result<InspectionCurve, String> {
    if let Some(crate::mock_kernel::EdgeCurveHint::Circle {
        center,
        axis,
        x_dir,
        radius,
        start,
        end,
        closed,
    }) = &reference.curve
    {
        return Ok(InspectionCurve::Circle {
            center: center.map(f64::from),
            axis: normalize3(axis.map(f64::from)),
            x_dir: normalize3(x_dir.map(f64::from)),
            radius: f64::from(*radius),
            first: f64::from(*start),
            last: if *closed {
                f64::from(*start) + std::f64::consts::TAU
            } else {
                f64::from(*end)
            },
        });
    }
    let captured_first = reference.p0.map(f64::from);
    let captured_last = reference.p1.map(f64::from);
    let endpoint_score = |edge: &openrcad::topo::Edge| {
        let first = edge.start().point();
        let last = edge.end().point();
        let first = [first.x(), first.y(), first.z()];
        let last = [last.x(), last.y(), last.z()];
        (distance3d(first, captured_first) + distance3d(last, captured_last))
            .min(distance3d(first, captured_last) + distance3d(last, captured_first))
    };
    let exact = body
        .parts
        .iter()
        .flat_map(KernelSolid::edges)
        .min_by(|left, right| endpoint_score(left).total_cmp(&endpoint_score(right)));
    if let Some(edge) = exact {
        let model_scale = distance3d(captured_first, captured_last).max(1.0);
        if endpoint_score(&edge) <= model_scale * 0.05 + 1.0e-3 {
            return Ok(InspectionCurve::Kernel(edge));
        }
    }
    if body.parts.is_empty()
        || matches!(
            reference.curve,
            Some(crate::mock_kernel::EdgeCurveHint::Line)
        )
    {
        return Ok(InspectionCurve::Segment {
            first: captured_first,
            last: captured_last,
        });
    }
    Err("selected edge could not be resolved to an evaluated kernel curve".to_string())
}

fn closest_curve_parameters(first: &InspectionCurve, second: &InspectionCurve) -> (f64, f64, f64) {
    let (first_min, first_max) = first.bounds();
    let (second_min, second_max) = second.bounds();
    let samples = 32;
    let mut best = (first_min, second_min, f64::INFINITY);
    for first_index in 0..=samples {
        let first_parameter =
            first_min + (first_max - first_min) * first_index as f64 / samples as f64;
        let first_point = first.point(first_parameter);
        for second_index in 0..=samples {
            let second_parameter =
                second_min + (second_max - second_min) * second_index as f64 / samples as f64;
            let distance = distance3d(first_point, second.point(second_parameter));
            if distance < best.2 {
                best = (first_parameter, second_parameter, distance);
            }
        }
    }
    let mut first_span = (first_max - first_min).abs() / samples as f64;
    let mut second_span = (second_max - second_min).abs() / samples as f64;
    for _ in 0..18 {
        for (first_delta, second_delta) in [
            (-first_span, 0.0),
            (first_span, 0.0),
            (0.0, -second_span),
            (0.0, second_span),
            (-first_span, -second_span),
            (-first_span, second_span),
            (first_span, -second_span),
            (first_span, second_span),
        ] {
            let first_parameter = (best.0 + first_delta).clamp(first_min, first_max);
            let second_parameter = (best.1 + second_delta).clamp(second_min, second_max);
            let distance = distance3d(first.point(first_parameter), second.point(second_parameter));
            if distance < best.2 {
                best = (first_parameter, second_parameter, distance);
            }
        }
        first_span *= 0.5;
        second_span *= 0.5;
    }
    best
}

#[allow(clippy::too_many_arguments)]
fn adaptive_simpson(
    function: &impl Fn(f64) -> f64,
    first: f64,
    last: f64,
    first_value: f64,
    middle_value: f64,
    last_value: f64,
    whole: f64,
    depth: u8,
) -> f64 {
    let middle = (first + last) * 0.5;
    let left_middle = (first + middle) * 0.5;
    let right_middle = (middle + last) * 0.5;
    let left_middle_value = function(left_middle);
    let right_middle_value = function(right_middle);
    let left = simpson(first, middle, first_value, left_middle_value, middle_value);
    let right = simpson(middle, last, middle_value, right_middle_value, last_value);
    if depth == 0 || (left + right - whole).abs() <= 1.0e-9 * (1.0 + whole.abs()) {
        return left + right + (left + right - whole) / 15.0;
    }
    adaptive_simpson(
        function,
        first,
        middle,
        first_value,
        left_middle_value,
        middle_value,
        left,
        depth - 1,
    ) + adaptive_simpson(
        function,
        middle,
        last,
        middle_value,
        right_middle_value,
        last_value,
        right,
        depth - 1,
    )
}

fn simpson(first: f64, last: f64, first_value: f64, middle_value: f64, last_value: f64) -> f64 {
    (last - first).abs() * (first_value + 4.0 * middle_value + last_value) / 6.0
}

fn dot3(first: [f64; 3], second: [f64; 3]) -> f64 {
    first[0] * second[0] + first[1] * second[1] + first[2] * second[2]
}

fn cross3(first: [f64; 3], second: [f64; 3]) -> [f64; 3] {
    [
        first[1] * second[2] - first[2] * second[1],
        first[2] * second[0] - first[0] * second[2],
        first[0] * second[1] - first[1] * second[0],
    ]
}

fn magnitude3(vector: [f64; 3]) -> f64 {
    dot3(vector, vector).sqrt()
}

fn normalize3(vector: [f64; 3]) -> [f64; 3] {
    let magnitude = magnitude3(vector);
    if magnitude <= f64::EPSILON {
        [0.0; 3]
    } else {
        vector.map(|coordinate| coordinate / magnitude)
    }
}

fn distance3d(first: [f64; 3], second: [f64; 3]) -> f64 {
    magnitude3([
        first[0] - second[0],
        first[1] - second[1],
        first[2] - second[2],
    ])
}

fn inspect_live_body(body: &LiveBody, density_g_cm3: f64) -> Result<BodyInspection, String> {
    if body.parts.is_empty() {
        let mesh = body
            .pristine
            .as_deref()
            .ok_or_else(|| format!("mesh body '{}' has no geometry", body.id))?;
        return inspect_mesh_body(&body.id, mesh, density_g_cm3);
    }
    let mut volume = 0.0;
    let mut surface_area = 0.0;
    let mut centroid_sum = [0.0_f64; 3];
    let mut bounds_min = [f64::INFINITY; 3];
    let mut bounds_max = [f64::NEG_INFINITY; 3];
    for part in &body.parts {
        let properties = solid_mass_properties(part)?;
        volume += properties.volume;
        surface_area += properties.surface_area;
        for (sum, coordinate) in centroid_sum.iter_mut().zip(properties.centroid) {
            *sum += coordinate * properties.volume;
        }
        if let Some((min, max)) = crate::mock_kernel::solid_aabb(part) {
            for axis in 0..3 {
                bounds_min[axis] = bounds_min[axis].min(min[axis] as f64);
                bounds_max[axis] = bounds_max[axis].max(max[axis] as f64);
            }
        }
    }
    if volume <= 0.0 {
        return Err(format!(
            "body '{}' has no positive measurable volume",
            body.id
        ));
    }
    Ok(BodyInspection {
        body_id: body.id.clone(),
        part_count: body.parts.len(),
        volume_mm3: Some(volume),
        surface_area_mm2: surface_area,
        centroid: centroid_sum.map(|component| component / volume),
        bounds_min,
        bounds_max,
        density_g_cm3,
        mass_g: Some(volume / 1_000.0 * density_g_cm3),
    })
}

fn inspect_mesh_body(
    body_id: &str,
    mesh: &MockMesh,
    density_g_cm3: f64,
) -> Result<BodyInspection, String> {
    let mut surface_area = 0.0;
    let mut area_centroid = [0.0_f64; 3];
    let mut bounds_min = [f64::INFINITY; 3];
    let mut bounds_max = [f64::NEG_INFINITY; 3];
    for vertex in mesh.vertices.chunks_exact(6) {
        for axis in 0..3 {
            let coordinate = vertex[axis] as f64;
            bounds_min[axis] = bounds_min[axis].min(coordinate);
            bounds_max[axis] = bounds_max[axis].max(coordinate);
        }
    }
    for triangle in mesh.indices.chunks_exact(3) {
        let Some([a, b, c]) = mesh_triangle(mesh, triangle) else {
            continue;
        };
        let area = triangle_area(a, b, c);
        surface_area += area;
        for axis in 0..3 {
            area_centroid[axis] += (a[axis] + b[axis] + c[axis]) / 3.0 * area;
        }
    }
    if surface_area <= 0.0 || !bounds_min[0].is_finite() {
        return Err(format!("mesh body '{body_id}' has no measurable triangles"));
    }
    let (volume, centroid) = match mesh.mass_properties() {
        Some(properties) => (Some(properties.volume), properties.centroid),
        None => (
            None,
            area_centroid.map(|component| component / surface_area),
        ),
    };
    Ok(BodyInspection {
        body_id: body_id.to_string(),
        part_count: 1,
        volume_mm3: volume,
        surface_area_mm2: surface_area,
        centroid,
        bounds_min,
        bounds_max,
        density_g_cm3,
        mass_g: volume.map(|volume| volume / 1_000.0 * density_g_cm3),
    })
}

fn mesh_triangle(mesh: &MockMesh, indices: &[u32]) -> Option<[[f64; 3]; 3]> {
    let point = |index: u32| {
        let start = index as usize * 6;
        Some([
            *mesh.vertices.get(start)? as f64,
            *mesh.vertices.get(start + 1)? as f64,
            *mesh.vertices.get(start + 2)? as f64,
        ])
    };
    Some([
        point(*indices.first()?)?,
        point(*indices.get(1)?)?,
        point(*indices.get(2)?)?,
    ])
}

fn triangle_area(a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let cross = [
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    ];
    0.5 * (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt()
}

fn solid_mass_properties(solid: &KernelSolid) -> Result<openrcad::mesh::MassProperties, String> {
    let mesh = openrcad::mesh::tessellate_checked(solid, 0.005, 0.05)
        .map_err(|error| format!("strict inspection tessellation failed: {error}"))?;
    openrcad::mesh::mass_properties(&mesh)
        .ok_or_else(|| "strict inspection mesh is open or degenerate".to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn body_measurement_is_independent_of_viewport_mesh_quality() {
        let mut graph = ParametricGraph::new();
        graph.add_feature(FeatureNode {
            id: "box_1".to_string(),
            name: "Box".to_string(),
            feature: FeatureType::Box {
                w: 10.0,
                h: 20.0,
                d: 30.0,
            },
        });
        let measured = graph.inspect_body("box_1").unwrap();
        assert!((measured.volume_mm3.expect("closed volume") - 6_000.0).abs() < 1.0e-6);
        assert!((measured.surface_area_mm2 - 2_200.0).abs() < 1.0e-6);
        assert_eq!(measured.centroid, [5.0, 10.0, 15.0]);
        let steel = graph.inspect_body_with_density("box_1", 7.85).unwrap();
        assert!((steel.mass_g.expect("closed mass") - 47.1).abs() < 1.0e-9);
    }

    #[test]
    fn common_based_interference_omits_disjoint_and_reports_overlap() {
        let mut graph = ParametricGraph::new();
        graph.add_feature(FeatureNode {
            id: "box_1".to_string(),
            name: "Box A".to_string(),
            feature: FeatureType::Box {
                w: 10.0,
                h: 10.0,
                d: 10.0,
            },
        });
        graph.add_feature(FeatureNode {
            id: "box_2".to_string(),
            name: "Box B".to_string(),
            feature: FeatureType::Box {
                w: 10.0,
                h: 10.0,
                d: 10.0,
            },
        });
        let report = graph
            .inspect_interference(&["box_1".to_string(), "box_2".to_string()], 1.0e-6)
            .unwrap();
        assert_eq!(report.len(), 1);
        assert!((report[0].volume_mm3 - 1_000.0).abs() < 1.0e-6);
    }

    #[test]
    fn picked_face_area_is_measured_from_strict_kernel_geometry() {
        let mut graph = ParametricGraph::new();
        graph.add_feature(FeatureNode {
            id: "box_1".to_string(),
            name: "Box".to_string(),
            feature: FeatureType::Box {
                w: 10.0,
                h: 20.0,
                d: 30.0,
            },
        });
        let mut areas = (0..6)
            .map(|face| graph.inspect_face("box_1", face).unwrap().area_mm2)
            .collect::<Vec<_>>();
        areas.sort_by(f64::total_cmp);
        assert_eq!(areas, [200.0, 200.0, 300.0, 300.0, 600.0, 600.0]);
    }

    #[test]
    fn exact_edge_measurement_uses_kernel_curves_and_analytic_hints() {
        let mut graph = ParametricGraph::new();
        graph.add_feature(FeatureNode {
            id: "box_1".to_string(),
            name: "Box".to_string(),
            feature: FeatureType::Box {
                w: 10.0,
                h: 20.0,
                d: 30.0,
            },
        });
        let (bodies, warnings) = graph
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .unwrap();
        assert!(warnings.is_empty());
        let first = &bodies[0].1.edge_refs[0];
        let line = EdgeRef {
            p0: first.p0,
            p1: first.p1,
            n1: first.n1,
            n2: first.n2,
            curve: first.curve.clone(),
            topology: first.topology.as_ref().map(|topology| TopologyEdgeRef {
                body_id: topology.body_id.clone(),
                topology_version: topology.topology_version,
                edge_id: topology.edge_id.clone(),
                adjacent_face_ids: topology.adjacent_face_ids.clone(),
                curve_kind: topology.curve_kind.clone(),
                adjacent_surface_kinds: topology.adjacent_surface_kinds.clone(),
                producer_feature_id: topology.producer_feature_id.clone(),
                source_entity_id: topology.source_entity_id.clone(),
            }),
        };
        let measured = graph.inspect_edge("box_1", &line).unwrap();
        assert_eq!(measured.curve_kind, "line");
        assert!([10.0, 20.0, 30.0]
            .into_iter()
            .any(|length| (measured.length_mm - length).abs() < 1.0e-8));

        let circle = EdgeRef {
            p0: [5.0, 0.0, 0.0],
            p1: [5.0, 0.0, 0.0],
            n1: [0.0; 3],
            n2: [0.0; 3],
            curve: Some(crate::mock_kernel::EdgeCurveHint::Circle {
                center: [0.0; 3],
                axis: [0.0, 0.0, 1.0],
                x_dir: [1.0, 0.0, 0.0],
                radius: 5.0,
                start: 0.0,
                end: std::f32::consts::TAU,
                closed: true,
            }),
            topology: None,
        };
        let measured_circle = graph.inspect_edge("box_1", &circle).unwrap();
        assert_eq!(measured_circle.curve_kind, "circle");
        assert!((measured_circle.length_mm - 10.0 * std::f64::consts::PI).abs() < 1.0e-8);
    }
}
