use super::*;

/// How far a tool overshoots the sketch plane to break coplanarity, in mm.
/// Comfortably above the boolean solver's tolerance so the dip is unambiguous,
/// yet small enough to be invisible at part scale.
pub(crate) const CUT_OVERSHOOT: f32 = 0.1;

/// How far a cut tool's side walls are pushed past a coplanar body face, in mm.
/// The in-plane analogue of `CUT_OVERSHOOT` (which handles the end caps).
pub(crate) const CUT_WALL_GROW: f32 = 0.1;

/// A join's tool, tried in order. `smooth` is the true analytic cylinder for a
/// circular boss (the kernel fuses it watertight, so a Ø-boss reads round, not
/// faceted); `exact` is the faceted prism with perfect dimensions; `dipped` is
/// a faceted fallback whose near cap dips into the target to dodge coplanar
/// faces. `smooth` is `None` for non-circular profiles, which fall straight to
/// the prism.
pub(crate) struct JoinTool {
    pub(crate) smooth: Option<KernelSolid>,
    pub(crate) exact: Option<KernelSolid>,
    pub(crate) dipped: Option<KernelSolid>,
}

/// Grow (`outward`) or shrink a closed 2D loop about its centroid so its
/// outermost vertex moves by `CUT_WALL_GROW`mm. Used to nudge a cut tool's side
/// walls just clear of a body face they'd otherwise be coplanar with — the
/// in-plane counterpart to `directional_cut`'s end-cap overshoot. Holes are
/// shrunk (`outward = false`) so their walls move the same way relative to the
/// removed volume. Every vertex moves at most `CUT_WALL_GROW`mm (displacement
/// is `dist_to_centroid · CUT_WALL_GROW / max_dist`), so the loop stays simple
/// for the convex and mildly-concave profiles sketches produce.
pub(crate) fn grow_loop(points: &[(f32, f32)], outward: bool) -> Vec<(f32, f32)> {
    let n = points.len();
    if n == 0 {
        return Vec::new();
    }
    let (mut cx, mut cy) = (0.0f32, 0.0f32);
    for &(x, y) in points {
        cx += x;
        cy += y;
    }
    cx /= n as f32;
    cy /= n as f32;
    let r = points
        .iter()
        .map(|&(x, y)| ((x - cx).powi(2) + (y - cy).powi(2)).sqrt())
        .fold(0.0f32, f32::max);
    if r < 1.0e-4 {
        return points.to_vec();
    }
    let f = if outward {
        1.0 + CUT_WALL_GROW / r
    } else {
        (1.0 - CUT_WALL_GROW / r).max(0.0)
    };
    points
        .iter()
        .map(|&(x, y)| (cx + (x - cx) * f, cy + (y - cy) * f))
        .collect()
}

/// Sketch plane nudged one `CUT_OVERSHOOT` back along the sweep direction, so a
/// join tool's near cap sits just behind the face the sketch is on instead of
/// flush on it (breaking the coplanarity the solver chokes on). "Behind the
/// sweep" is where the body sits for the common case — growing a boss off a
/// face — so the dip is swallowed by that body and leaves no artifact. For the
/// rarer into-body join (sweep runs into the material) the dip instead pokes a
/// sub-0.1mm sliver out of the face; the orientation fix + `apply_join`'s
/// keep-the-body guard still preserve the original geometry, which is what
/// matters.
pub(crate) fn overshoot_cs(cs: &CoordinateSystem, depth: f32) -> CoordinateSystem {
    let back = cs.n.mul(-depth.signum() * CUT_OVERSHOOT);
    CoordinateSystem::new(cs.origin.add(back), cs.u, cs.v)
}

/// Depth extended by `ends` overshoot lengths along the sweep direction. Paired
/// with `overshoot_cs` (which moves the start back by one overshoot): `ends = 1`
/// keeps the far cap where it was (near-only dip, for join).
pub(crate) fn overshoot_depth(depth: f32, ends: f32) -> f32 {
    depth + depth.signum() * ends * CUT_OVERSHOOT
}

/// Apply a Join extrude: union each tool into the first existing body it
/// overlaps. For each candidate body it tries the exact tool first (perfect
/// geometry), then the dipped tool (breaks coplanar faces). A tool that joins
/// nothing — exact or dipped — becomes a standalone new body under the
/// extrude's id, matching Fusion's "join with nothing creates a body"; that
/// outcome is surfaced as a warning since the user asked to *join*, not to
/// create a separate lump.
pub(crate) fn apply_join(
    live: &mut Vec<LiveBody>,
    extrude_id: &str,
    tools: Vec<JoinTool>,
    warnings: &mut Vec<String>,
) {
    let mut orphans: Vec<KernelSolid> = Vec::new();
    for tool in tools {
        // Bounding box from whichever variant exists, for the overlap pre-test.
        let tbb = tool
            .smooth
            .as_ref()
            .or(tool.exact.as_ref())
            .or(tool.dipped.as_ref())
            .and_then(crate::mock_kernel::solid_aabb);

        let mut merged = false;
        if let Some(tbb) = tbb {
            'bodies: for body in live.iter_mut() {
                for part in body.parts.iter_mut() {
                    let overlaps = crate::mock_kernel::solid_aabb(part).map_or(true, |pbb| {
                        crate::mock_kernel::aabbs_overlap(&pbb, &tbb, 0.05)
                    });
                    if !overlaps {
                        continue;
                    }
                    // Smooth analytic cylinder first (round boss), then the faceted
                    // prism variants as robustness fallbacks.
                    let unioned = tool
                        .smooth
                        .as_ref()
                        .and_then(|t| crate::mock_kernel::union(part, t))
                        .or_else(|| {
                            tool.exact
                                .as_ref()
                                .and_then(|t| crate::mock_kernel::union(part, t))
                        })
                        .or_else(|| {
                            tool.dipped
                                .as_ref()
                                .and_then(|t| crate::mock_kernel::union(part, t))
                        });
                    if let Some(u) = unioned {
                        // A join must never destroy existing material: `a ∪ b`
                        // always contains `a`. truck can still hand back a
                        // degenerate solid (e.g. an inverted tool that subtracts
                        // the body) whose bounds no longer enclose the original —
                        // reject those and leave the body untouched so the join
                        // can only ever add, never remove.
                        let keeps_body = match (
                            crate::mock_kernel::solid_aabb(part),
                            crate::mock_kernel::solid_aabb(&u),
                        ) {
                            (Some(pbb), Some(ubb)) => {
                                crate::mock_kernel::aabb_contains(&ubb, &pbb, 0.05)
                            }
                            _ => true,
                        };
                        if keeps_body {
                            *part = u;
                            body.pristine = None;
                            body.sketch_source = None;
                            merged = true;
                            break 'bodies;
                        }
                    }
                    if let Some(fallback) = tool
                        .smooth
                        .as_ref()
                        .or(tool.exact.as_ref())
                        .or(tool.dipped.as_ref())
                        .cloned()
                    {
                        body.parts.push(fallback);
                        body.pristine = None;
                        body.sketch_source = None;
                        merged = true;
                        break 'bodies;
                    }
                }
            }
        }
        if !merged {
            // Joined nothing — keep the (preferably smooth) un-dipped volume as its
            // own body.
            if let Some(s) = tool.smooth.or(tool.exact).or(tool.dipped) {
                warnings.push(format!(
                    "Join '{extrude_id}': the extruded volume didn't overlap an \
                     existing body, so it became a separate body."
                ));
                orphans.push(s);
            }
        }
    }
    if !orphans.is_empty() {
        live.push(LiveBody {
            id: extrude_id.to_string(),
            parts: orphans,
            pristine: None,
            sketch_source: None,
            cut_tools: Vec::new(),
        });
    }
}
