//! Exact sectional difference for parallel line/arc sketch prisms.
use super::*;
use openrcad::foundation::{Ax22d, Dir2d, Pnt2d};
use openrcad::geom2d::{Circle2d, GeomCurve2d, Line2d};
use openrcad::topo::Face;

pub(super) fn from_analytic(mut analytic: crate::sketch::AnalyticSketchRegion) -> Region {
    // Boundary cancellation requires material on the left: outer CCW, holes
    // CW. Arrangement nesting accepts either hole winding, but using a child's
    // CCW loop unchanged would retain the shared wall when that child is kept.
    for (index, loop_) in std::iter::once(&mut analytic.outer)
        .chain(&mut analytic.holes)
        .enumerate()
    {
        if (index == 0 && loop_.signed_area < 0.) || (index != 0 && loop_.signed_area > 0.) {
            loop_.spans = loop_.spans.drain(..).rev().map(|s| s.reversed()).collect();
            loop_.signed_area = -loop_.signed_area;
        }
    }
    Region {
        boundary: crate::sketch::sample_analytic_loop(&analytic.outer),
        holes: analytic
            .holes
            .iter()
            .map(crate::sketch::sample_analytic_loop)
            .collect(),
        area: analytic.area as f32,
        analytic: Some(analytic),
    }
}

/// Change only the support frame; preserve exact circle radii and parameters.
pub(super) fn in_frame(
    region: &Region,
    from: &CoordinateSystem,
    to: &CoordinateSystem,
) -> Option<Region> {
    let mut analytic = region.analytic.clone()?;
    let dot = |a: Vec3, b: Vec3| {
        f64::from(a.x) * f64::from(b.x)
            + f64::from(a.y) * f64::from(b.y)
            + f64::from(a.z) * f64::from(b.z)
    };
    let (xx, xy, yx, yy) = (
        dot(from.u, to.u),
        dot(from.v, to.u),
        dot(from.u, to.v),
        dot(from.v, to.v),
    );
    // Subtract in kernel precision too: rounding the translated origin to f32
    // first can separate otherwise coincident profile edges by several microns.
    let offset = |axis: Vec3| {
        (f64::from(from.origin.x) - f64::from(to.origin.x)) * f64::from(axis.x)
            + (f64::from(from.origin.y) - f64::from(to.origin.y)) * f64::from(axis.y)
            + (f64::from(from.origin.z) - f64::from(to.origin.z)) * f64::from(axis.z)
    };
    let (ox, oy) = (offset(to.u), offset(to.v));
    let point = |p: Pnt2d| Pnt2d::new(ox + xx * p.x() + xy * p.y(), oy + yx * p.x() + yy * p.y());
    let direction = |d: Dir2d| Dir2d::try_new(xx * d.x() + xy * d.y(), yx * d.x() + yy * d.y());
    let reflected = xx * yy - xy * yx < 0.0;
    for loop_ in std::iter::once(&mut analytic.outer).chain(&mut analytic.holes) {
        for span in &mut loop_.spans {
            span.curve = match &span.curve {
                GeomCurve2d::Line(line) => GeomCurve2d::line(Line2d::from_point_dir(
                    point(line.location()),
                    direction(line.direction())?,
                )),
                GeomCurve2d::Circle(circle) => {
                    if reflected {
                        span.first = -span.first;
                        span.last = -span.last;
                    }
                    GeomCurve2d::circle(Circle2d::new(
                        Ax22d::new(point(circle.center()), direction(circle.x_axis())?),
                        circle.radius(),
                    ))
                }
                _ => return None,
            };
        }
        if reflected {
            loop_.signed_area = -loop_.signed_area;
        }
    }
    Some(from_analytic(analytic))
}

/// Rebuild every constant-depth section once, then sew only exposed boundaries.
/// No lateral/axial expansion, triangle boolean, or document mutation is used.
/// Unsupported source curves or non-prismatic bodies remain on the normal path.
pub(super) fn try_profile_difference(
    body: &LiveBody,
    subtractor: &LiveBody,
) -> Option<Vec<KernelSolid>> {
    let source = subtractor.sketch_source.as_ref()?;
    let tools: Option<Vec<_>> = source
        .regions
        .iter()
        .chain(&source.joined_prisms)
        .map(|prism| {
            let mut tool = CutTool::single_direction(None, None, None, None);
            tool.exact_source = Some(super::cut::ExactCutSource {
                region: source_region(prism)?,
                cs: prism.cs,
                depth: prism.depth,
                arc_circles: Vec::new(),
            });
            Some(tool)
        })
        .collect();
    try_profile_cut(body, &tools?).map(|(parts, _)| parts)
}

pub(super) fn try_profile_cut(
    body: &LiveBody,
    tools: &[CutTool],
) -> Option<(Vec<KernelSolid>, SketchExtrudeSource)> {
    let retained = body.sketch_source.as_ref()?;
    let sources: Vec<_> = retained
        .regions
        .iter()
        .chain(&retained.joined_prisms)
        .collect();
    let source = *sources.first()?;
    if body.parts.len() != 1 || tools.is_empty() || !source.depth.is_finite() {
        return None;
    }
    let length = source.depth.abs();
    let sign = source.depth.signum();
    const TOL: f32 = 1.0e-4;
    if length <= TOL {
        return None;
    }
    let mut profiles = Vec::new();
    let mut ranges = Vec::new();
    let mut levels = Vec::new();
    for exact in &sources {
        if !exact.depth.is_finite() || exact.cs.n.cross(source.cs.n).length() > 1.0e-6 {
            return None;
        }
        let start = exact.cs.origin.sub(source.cs.origin).dot(source.cs.n) * sign;
        let end = start + exact.depth * exact.cs.n.dot(source.cs.n) * sign;
        let range = (start.min(end), start.max(end));
        profiles.push(in_frame(&source_region(exact)?, &exact.cs, &source.cs)?);
        ranges.push(range);
        levels.extend([range.0, range.1]);
    }
    let base_count = profiles.len();
    let low = levels.iter().copied().fold(f32::INFINITY, f32::min);
    let high = levels.iter().copied().fold(f32::NEG_INFINITY, f32::max);
    for tool in tools {
        let mut candidates = Vec::new();
        for exact in [tool.exact_source.as_ref(), tool.exact_rev_source.as_ref()]
            .into_iter()
            .flatten()
        {
            if !exact.depth.is_finite() || exact.cs.n.cross(source.cs.n).length() > 1.0e-6 {
                continue;
            }
            let start = exact.cs.origin.sub(source.cs.origin).dot(source.cs.n) * sign;
            let end = start + exact.depth * exact.cs.n.dot(source.cs.n) * sign;
            let range = (start.min(end).max(low), start.max(end).min(high));
            if range.1 - range.0 > TOL {
                candidates.push((exact, range));
            }
        }
        // Preserve the established forward-first cut direction rule.
        let (exact, range) = candidates.first().copied()?;
        profiles.push(in_frame(&exact.region, &exact.cs, &source.cs)?);
        ranges.push(range);
        levels.extend([range.0, range.1]);
    }
    let mut spans = Vec::new();
    let mut tolerance: f64 = 1.0e-8;
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
    let footprint_masks: Vec<Vec<_>> = profiles
        .iter()
        .map(|r| probes.iter().map(|&p| r.contains(p)).collect())
        .collect();
    if !(0..cells.len()).any(|i| {
        footprint_masks[..base_count].iter().any(|m| m[i])
            && footprint_masks[base_count..].iter().any(|m| m[i])
    }) {
        return None;
    }
    levels.sort_by(f32::total_cmp);
    levels.dedup_by(|a, b| (*a - *b).abs() < TOL);
    let masks: Vec<Vec<bool>> = levels
        .windows(2)
        .map(|pair| {
            let mid = (pair[0] + pair[1]) * 0.5;
            (0..cells.len())
                .map(|i| {
                    let occupied =
                        |t: usize| mid > ranges[t].0 && mid < ranges[t].1 && footprint_masks[t][i];
                    (0..base_count).any(occupied) && !(base_count..ranges.len()).any(occupied)
                })
                .collect()
        })
        .collect();
    let build = |mask: &[bool], lo: f32, hi: f32| -> Option<Vec<KernelSolid>> {
        let cs = source
            .cs
            .with_origin(source.cs.origin.add(source.cs.n.mul(sign * lo)));
        let prepared = prepare_extrude_regions(&cells, mask);
        prepared
            .into_iter()
            .map(|p| {
                let solid = match p.region.analytic.as_ref() {
                    Some(analytic) => crate::mock_kernel::build_analytic_section_solid(
                        analytic,
                        f64::from(sign * (hi - lo)),
                        &cs,
                    ),
                    None => crate::mock_kernel::extruded_sketch_region_solid(
                        &p.region,
                        sign * (hi - lo),
                        &cs,
                        &[],
                    ),
                };
                if solid.is_none() {
                    recut_debug(format!(
                        "profile cut: failed section {lo}..{hi}, area {}",
                        p.region.area
                    ));
                }
                solid
            })
            .collect()
    };
    let at_height = |face: &Face, h: f32| {
        planar_section_height(face, &source.cs).is_some_and(|v| (v - sign * h).abs() < TOL)
    };
    let mut faces = Vec::new();
    let mut remaining_prisms = Vec::new();
    for (layer, pair) in levels.windows(2).enumerate() {
        let cs = source
            .cs
            .with_origin(source.cs.origin.add(source.cs.n.mul(sign * pair[0])));
        for prepared in prepare_extrude_regions(&cells, &masks[layer]) {
            remaining_prisms.push(SketchExtrudeRegionSource {
                boundary: prepared.region.boundary,
                holes: prepared.region.holes,
                analytic: prepared.region.analytic,
                depth: sign * (pair[1] - pair[0]),
                cs,
                rect_circle: None,
            });
        }
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
            // A one-unit helper prism supplies the cap with the correct outward
            // sense for either sweep direction; only that cap enters the shell.
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
    if faces.is_empty() {
        return Some((
            Vec::new(),
            SketchExtrudeSource {
                regions: Vec::new(),
                joined_prisms: Vec::new(),
            },
        ));
    }
    let policy = openrcad::foundation::TolerancePolicy::STANDARD;
    let shell = openrcad::algo::sew_with_policy(&faces, &policy)
        .map_err(|e| recut_debug(format!("profile cut: sew failed {e:?}")))
        .ok()?
        .value;
    let solid = KernelSolid::new(shell);
    if !solid.is_watertight()
        || !solid.health_report().is_healthy()
        || solid.validate_strict_with_policy(&policy).is_err()
    {
        recut_debug(format!(
            "profile cut: invalid sew {:?}, {:?}",
            solid.health_report(),
            solid.validate_strict_with_policy(&policy)
        ));
        return None;
    }
    Some((
        solid.split_disconnected(),
        SketchExtrudeSource {
            regions: Vec::new(),
            joined_prisms: remaining_prisms,
        },
    ))
}
