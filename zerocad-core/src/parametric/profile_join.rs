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
pub(super) fn try_profile_join(body: &LiveBody, tools: &[JoinTool]) -> Option<KernelSolid> {
    const TOL: f32 = 1.0e-4;
    let retained = joined_source(body, tools)?;
    let source = retained.joined_prisms.first()?;
    if body.parts.len() != 1 || tools.is_empty() || !source.depth.is_finite() {
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
    let probes: Vec<_> = cells.iter().map(region_material_point).collect();
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
    let policy = openrcad::foundation::TolerancePolicy::STANDARD;
    let shell = openrcad::algo::sew_with_policy(&faces, &policy).ok()?.value;
    let solid = KernelSolid::new(shell);
    let bounds = crate::mock_kernel::solid_aabb(&solid)?;
    for input in std::iter::once(Some(&body.parts[0])).chain(
        tools
            .iter()
            .map(|tool| tool.exact.as_ref().or(tool.smooth.as_ref())),
    ) {
        let input_bounds = crate::mock_kernel::solid_aabb(input?)?;
        if !crate::mock_kernel::aabb_contains(&bounds, &input_bounds, TOL) {
            return None;
        }
    }
    (solid.is_watertight()
        && solid.health_report().is_healthy()
        && solid.validate_strict_with_policy(&policy).is_ok()
        && solid.split_disconnected().len() == 1)
        .then_some(solid)
}
