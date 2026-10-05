//! Exact unions of parallel sketch prisms, including overhanging face profiles.
use super::profile_cut::{from_analytic, in_frame};
use super::*;
use openrcad::topo::Face;

pub(super) fn joined_source(body: &LiveBody, tools: &[JoinTool]) -> Option<SketchExtrudeSource> {
    let source = body.sketch_source.as_ref()?;
    let mut prisms = source.regions.clone();
    prisms.extend(source.joined_prisms.iter().cloned());
    if prisms.is_empty() {
        return None;
    }
    for tool in tools {
        let profile = tool.profile.as_ref()?;
        prisms.push(SketchExtrudeRegionSource {
            boundary: profile.region.boundary.clone(),
            holes: profile.region.holes.clone(),
            analytic: profile.region.analytic.clone(),
            depth: profile.depth,
            cs: profile.cs,
            rect_circle: None,
        });
    }
    Some(SketchExtrudeSource {
        regions: Vec::new(),
        joined_prisms: prisms,
    })
}

// Resolve axis-aligned line supports at the sketch input's endpoint precision.
// Arrangement vertex welding alone cannot do this: the end of an overhang can
// lie beyond the original segment while still sharing its supporting line.
fn align_line_supports(profiles: &mut [Region], tolerance: f64) -> Option<()> {
    use openrcad::foundation::{Dir2d, Pnt2d};
    use openrcad::geom2d::{GeomCurve2d, Line2d};
    let mut xs: Vec<f64> = Vec::new();
    let mut ys: Vec<f64> = Vec::new();
    for region in profiles {
        let Some(analytic) = region.analytic.as_mut() else {
            continue;
        };
        for loop_ in std::iter::once(&mut analytic.outer).chain(&mut analytic.holes) {
            if !loop_
                .spans
                .iter()
                .all(|s| matches!(s.curve, GeomCurve2d::Line(_)))
            {
                continue;
            }
            let mut points: Vec<_> = loop_.spans.iter().map(|s| s.start()).collect();
            for span in &loop_.spans {
                let a = span.start();
                let b = span.end();
                if (a.x() - b.x()).abs() < 1e-10
                    && !xs.iter().any(|x| (x - a.x()).abs() <= tolerance)
                {
                    xs.push(a.x());
                }
                if (a.y() - b.y()).abs() < 1e-10
                    && !ys.iter().any(|y| (y - a.y()).abs() <= tolerance)
                {
                    ys.push(a.y());
                }
            }
            for (i, point) in points.iter_mut().enumerate() {
                let adjacent = [
                    &loop_.spans[i],
                    &loop_.spans[(i + loop_.spans.len() - 1) % loop_.spans.len()],
                ];
                let vertical = adjacent
                    .iter()
                    .any(|s| (s.start().x() - s.end().x()).abs() < 1e-10);
                let horizontal = adjacent
                    .iter()
                    .any(|s| (s.start().y() - s.end().y()).abs() < 1e-10);
                let x = xs
                    .iter()
                    .copied()
                    .find(|x| vertical && (x - point.x()).abs() <= tolerance)
                    .unwrap_or(point.x());
                let y = ys
                    .iter()
                    .copied()
                    .find(|y| horizontal && (y - point.y()).abs() <= tolerance)
                    .unwrap_or(point.y());
                *point = Pnt2d::new(x, y);
            }
            for (i, span) in loop_.spans.iter_mut().enumerate() {
                let a = points[i];
                let b = points[(i + 1) % points.len()];
                let dx = b.x() - a.x();
                let dy = b.y() - a.y();
                let direction = Dir2d::try_new(dx, dy)?;
                span.curve = GeomCurve2d::line(Line2d::from_point_dir(a, direction));
                span.first = 0.;
                span.last = dx.hypot(dy);
            }
            loop_.signed_area = points
                .iter()
                .zip(points.iter().cycle().skip(1))
                .take(points.len())
                .map(|(a, b)| a.x() * b.y() - b.x() * a.y())
                .sum::<f64>()
                * 0.5;
        }
        analytic.area =
            analytic.outer.area() - analytic.holes.iter().map(|h| h.area()).sum::<f64>();
        *region = from_analytic(analytic.clone());
    }
    Some(())
}

/// Arrange all footprints together, union the occupied cells in each axial
/// interval, and sew only the exposed boundary. Shared interfaces never enter
/// the shell, even when a tool crosses a concave supporting cap. All profiles
/// participate in one transaction; edge/point-only contacts still fail the
/// strict one-solid gate.
/// The result has one solid per lump: a body that earlier cuts severed keeps
/// its untouched lumps, and a Join may reunite lumps but never add one.
pub(super) fn try_profile_join(body: &LiveBody, tools: &[JoinTool]) -> Option<Vec<KernelSolid>> {
    const TOL: f32 = 1.0e-4;
    let retained = joined_source(body, tools)?;
    let source = retained.joined_prisms.first()?;
    if body.parts.is_empty() || tools.is_empty() || !source.depth.is_finite() {
        return None;
    }
    let length = source.depth.abs();
    if length <= TOL {
        return None;
    }
    let sign = source.depth.signum();
    let mut profiles = Vec::new();
    let mut ranges = Vec::new();
    let mut levels = Vec::new();
    for exact in &retained.joined_prisms {
        if !exact.depth.is_finite() || exact.cs.n.cross(source.cs.n).length() > 1.0e-6 {
            return None;
        }
        let start = exact.cs.origin.sub(source.cs.origin).dot(source.cs.n) * sign;
        let end = start + exact.depth * exact.cs.n.dot(source.cs.n) * sign;
        let range = (start.min(end), start.max(end));
        if range.1 - range.0 <= TOL {
            return None;
        }
        profiles.push(in_frame(&source_region(exact)?, &exact.cs, &source.cs)?);
        ranges.push(range);
        levels.extend([range.0, range.1]);
    }
    let mut spans = Vec::new();
    // Arrangement inputs originate in f32 sketch frames. Use the same
    // endpoint resolution as selected-region boundary cancellation so tiny
    // frame-rounding slivers do not become unbuildable helper prisms.
    let scale = profiles
        .iter()
        .flat_map(|r| r.boundary.iter().chain(r.holes.iter().flatten()))
        .flat_map(|&(x, y)| [f64::from(x.abs()), f64::from(y.abs())])
        .fold(0.0_f64, f64::max);
    let mut tolerance = (scale * f64::from(f32::EPSILON) * 8.0).max(1.0e-8);
    align_line_supports(
        &mut profiles,
        tolerance.min(openrcad::foundation::TolerancePolicy::STANDARD.sewing),
    )?;
    for region in &profiles {
        let analytic = region.analytic.as_ref()?;
        for loop_ in std::iter::once(&analytic.outer).chain(&analytic.holes) {
            tolerance = tolerance.max(loop_.junction_tolerance);
            spans.extend(loop_.spans.iter().cloned());
        }
    }
    let arranged = openrcad::sketch::arrange_curve_spans(
        &spans,
        openrcad::sketch::ArrangementOptions {
            tolerance,
            ..Default::default()
        },
    )
    .ok()?;
    let cells: Vec<_> = arranged.regions.into_iter().map(from_analytic).collect();
    let probes: Vec<_> = cells.iter().map(cell_probe_point).collect();
    let footprints: Vec<Vec<_>> = profiles
        .iter()
        .map(|region| probes.iter().map(|&p| region.contains(p)).collect())
        .collect();
    levels.sort_by(f32::total_cmp);
    levels.dedup_by(|a, b| (*a - *b).abs() < TOL);
    let masks: Vec<Vec<bool>> = levels
        .windows(2)
        .map(|pair| {
            let mid = (pair[0] + pair[1]) * 0.5;
            (0..cells.len())
                .map(|i| {
                    ranges
                        .iter()
                        .enumerate()
                        .any(|(p, &(lo, hi))| mid > lo && mid < hi && footprints[p][i])
                })
                .collect()
        })
        .collect();
    let build = |mask: &[bool], lo: f32, hi: f32| -> Option<Vec<KernelSolid>> {
        let cs = source
            .cs
            .with_origin(source.cs.origin.add(source.cs.n.mul(sign * lo)));
        prepare_extrude_regions(&cells, mask)
            .into_iter()
            .map(|p| {
                if p.region.analytic.is_none()
                    && p.region.holes.is_empty()
                    && p.source_indices.iter().all(|&i| {
                        cells[i].analytic.as_ref().is_some_and(|a| {
                            std::iter::once(&a.outer)
                                .chain(&a.holes)
                                .flat_map(|l| &l.spans)
                                .all(|s| s.kind() == openrcad::geom2d::CurveKind2d::Line)
                        })
                    })
                {
                    // These are known line spans, including the persisted
                    // ellipse polyline. Never refit them into circular arcs.
                    return crate::mock_kernel::build_extrusion_solid(
                        &p.region.boundary,
                        &[],
                        f64::from(sign * (hi - lo)),
                        &cs,
                        false,
                    );
                }
                crate::mock_kernel::build_analytic_section_solid(
                    p.region.analytic.as_ref()?,
                    f64::from(sign * (hi - lo)),
                    &cs,
                )
            })
            .collect()
    };
    let at_height = |face: &Face, height: f32| {
        planar_section_height(face, &source.cs).is_some_and(|h| (h - sign * height).abs() < TOL)
    };
    let mut faces = Vec::new();
    for (layer, pair) in levels.windows(2).enumerate() {
        for solid in build(&masks[layer], pair[0], pair[1])? {
            faces.extend(
                solid
                    .shell()
                    .faces()
                    .into_iter()
                    .filter(|f| !at_height(f, pair[0]) && !at_height(f, pair[1])),
            );
        }
    }
    let empty = vec![false; cells.len()];
    for (level, &height) in levels.iter().enumerate() {
        let below = if level == 0 {
            &empty
        } else {
            &masks[level - 1]
        };
        let above = masks.get(level).unwrap_or(&empty);
        for (material, void, upper) in [(below, above, true), (above, below, false)] {
            let exposed: Vec<_> = material.iter().zip(void).map(|(&a, &b)| a && !b).collect();
            // A helper prism provides only the correctly oriented exposed cap.
            let (lo, hi) = if upper {
                (height - 1., height)
            } else {
                (height, height + 1.)
            };
            for solid in build(&exposed, lo, hi)? {
                faces.extend(
                    solid
                        .shell()
                        .faces()
                        .into_iter()
                        .filter(|f| at_height(f, height)),
                );
            }
        }
    }
    // Each lump is sewn and strictly validated on its own; one shell holding
    // several lumps would fail the single-solid gate.
    let policy = openrcad::foundation::TolerancePolicy::STANDARD;
    let lumps: Vec<KernelSolid> = openrcad::algo::sew_bodies_with_policy(&faces, &policy)
        .ok()?
        .into_iter()
        .map(|body| KernelSolid::new(body.value))
        .collect();
    if lumps.is_empty() || lumps.len() > body.parts.len() {
        return None;
    }
    let mut bounds: Option<([f32; 3], [f32; 3])> = None;
    for lump in &lumps {
        let (lo, hi) = crate::mock_kernel::solid_aabb(lump)?;
        bounds = Some(match bounds {
            None => (lo, hi),
            Some((a, b)) => (
                std::array::from_fn(|i| a[i].min(lo[i])),
                std::array::from_fn(|i| b[i].max(hi[i])),
            ),
        });
    }
    let bounds = bounds?;
    for input in body.parts.iter().map(Some).chain(
        tools
            .iter()
            .map(|tool| tool.exact.as_ref().or(tool.smooth.as_ref())),
    ) {
        let input_bounds = crate::mock_kernel::solid_aabb(input?)?;
        if !crate::mock_kernel::aabb_contains(&bounds, &input_bounds, TOL) {
            return None;
        }
    }
    Some(lumps)
}

/// Join an outward prism to a planar extremity without rebuilding unrelated
/// walls (for example a bracket already drilled across a different axis).
/// Only the common cap interface changes: retain body-only and tool-only
/// patches, discard their overlap, and sew the unchanged walls to those caps.
pub(super) fn try_cap_join(body: &LiveBody, tool: &JoinTool) -> Option<KernelSolid> {
    use openrcad::foundation::Pnt;
    let [part] = body.parts.as_slice() else {
        return None;
    };
    let profile = tool.profile.as_ref()?;
    let exact = tool.exact.as_ref()?;
    let cs = profile.cs;
    let sign = profile.depth.signum();
    if profile.depth.abs() < 1e-6 {
        return None;
    }
    let (lo, hi) = crate::mock_kernel::solid_aabb(part)?;
    let scale = lo
        .iter()
        .chain(&hi)
        .map(|v| v.abs())
        .fold(1.0_f32, f32::max);
    let policy = openrcad::foundation::TolerancePolicy::STANDARD;
    let tolerance = (scale * f32::EPSILON * 8.0).min(policy.sewing as f32);
    // The complete enclosing box must be behind the joining plane. This is a
    // conservative proof of no positive-volume overlap, even for curved walls.
    for x in [lo[0], hi[0]] {
        for y in [lo[1], hi[1]] {
            for z in [lo[2], hi[2]] {
                if Vec3::new(x, y, z).sub(cs.origin).dot(cs.n) * sign > tolerance {
                    return None;
                }
            }
        }
    }
    let at_cap = |f: &Face| planar_section_height(f, &cs).is_some_and(|h| h.abs() <= tolerance);
    let caps: Vec<_> = part.faces().into_iter().filter(&at_cap).collect();
    if caps.is_empty() {
        return None;
    }
    let mut profiles = Vec::new();
    for cap in &caps {
        let boundary = crate::mock_kernel::kernel_face_boundary_2d(cap, &cs)?;
        for region in crate::sketch::detect_regions(&boundary) {
            let (u, v) = region_material_point(&region);
            let point = cs.unproject(u, v);
            let (u, v) = openrcad::algo::intersect::uv_of(
                cap.surface()?,
                &Pnt::new(point.x as f64, point.y as f64, point.z as f64),
            );
            if openrcad::algo::intersect::is_inside_trimming_loops(u, v, cap) {
                profiles.push(region);
            }
        }
    }
    let cap_count = profiles.len();
    if cap_count == 0 {
        return None;
    }
    profiles.push(profile.region.clone());
    let tol = f64::from(tolerance).min(policy.sewing);
    align_line_supports(&mut profiles, tol)?;
    let mut spans = Vec::new();
    for region in &profiles {
        let analytic = region.analytic.as_ref()?;
        for wire in std::iter::once(&analytic.outer).chain(&analytic.holes) {
            spans.extend(wire.spans.iter().cloned());
        }
    }
    let cells: Vec<_> = openrcad::sketch::arrange_curve_spans(
        &spans,
        openrcad::sketch::ArrangementOptions {
            tolerance: tol,
            ..Default::default()
        },
    )
    .ok()?
    .regions
    .into_iter()
    .map(from_analytic)
    .collect();
    let mut body_only = Vec::new();
    let mut tool_only = Vec::new();
    let mut shared_area = 0.0_f64;
    for cell in &cells {
        let p = cell_probe_point(cell);
        let in_body = profiles[..cap_count].iter().any(|r| r.contains(p));
        let in_tool = profiles[cap_count].contains(p);
        body_only.push(in_body && !in_tool);
        tool_only.push(in_tool && !in_body);
        if in_body && in_tool {
            shared_area += f64::from(cell.area);
        }
    }
    if shared_area <= tol * tol {
        return None;
    }
    let mut faces: Vec<_> = part
        .faces()
        .into_iter()
        .chain(exact.faces())
        .filter(|f| !at_cap(f))
        .collect();
    for (mask, upper) in [(&body_only, true), (&tool_only, false)] {
        let frame = if upper {
            cs.with_origin(cs.origin.sub(cs.n.mul(sign)))
        } else {
            cs
        };
        for prepared in prepare_extrude_regions(&cells, mask) {
            let helper = crate::mock_kernel::build_analytic_section_solid(
                prepared.region.analytic.as_ref()?,
                f64::from(sign),
                &frame,
            )?;
            faces.extend(helper.faces().into_iter().filter(&at_cap));
        }
    }
    let shell = openrcad::algo::sew_with_policy(&faces, &policy).ok()?.value;
    let solid = KernelSolid::new(shell);
    (solid.is_watertight()
        && solid.health_report().is_healthy()
        && solid.validate_strict_with_policy(&policy).is_ok()
        && solid.split_disconnected().len() == 1)
        .then_some(solid)
}
