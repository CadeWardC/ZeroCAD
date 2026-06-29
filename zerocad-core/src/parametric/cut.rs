use super::*;

/// A cut's tool, tried in order, mirroring `JoinTool`: `smooth` is the analytic
/// cylinder for a circular pocket/drill (a clean round hole, not a 48-gon one);
/// `exact` is the faceted prism with the drawn dimensions; `expanded` is a
/// faceted fallback whose walls poke ~`CUT_WALL_GROW`mm past the body's faces to
/// dodge the coplanar-face case. `smooth` is `None` for non-circular profiles.
pub(crate) struct CutTool {
    pub(crate) smooth: Option<KernelSolid>,
    pub(crate) exact: Option<KernelSolid>,
    pub(crate) expanded: Option<KernelSolid>,
    // The same three tools swept the OPPOSITE direction. A cut is meant to remove
    // material; when the drawn direction sweeps into empty air (e.g. a positive
    // "pocket depth" on a top-face sketch, which `directional_cut` sends *up* away
    // from the body) it bites nothing and the op silently does nothing — the
    // reported "cut works once then never again". `apply_cut` falls back to these
    // when the drawn direction removes nothing from a body it should have cut.
    pub(crate) smooth_rev: Option<KernelSolid>,
    pub(crate) exact_rev: Option<KernelSolid>,
    pub(crate) expanded_rev: Option<KernelSolid>,
    pub(crate) circle: Option<Circle>,
}

/// A cut tool that sweeps in the **drawn direction** (the sign of `depth`):
/// a negative depth cuts *into* the body the sketch sits on, a positive depth
/// sweeps *outward* from the sketch face. The tool starts one `CUT_OVERSHOOT`
/// behind the sketch plane and runs `|depth| + 2·CUT_OVERSHOOT` along the sweep
/// direction, so both end caps clear the body's faces — the near cap clears the
/// face the sketch sits on, and the far cap clears the back face when the cut
/// punches clean through. Returns `(start_plane, signed_sweep_depth)`.
pub(crate) fn directional_cut(cs: &CoordinateSystem, depth: f32) -> (CoordinateSystem, f32) {
    let sign = if depth < 0.0 { -1.0 } else { 1.0 };
    let start = cs.origin.add(cs.n.mul(-sign * CUT_OVERSHOOT));
    (
        CoordinateSystem::new(start, cs.u, cs.v),
        depth + sign * 2.0 * CUT_OVERSHOOT,
    )
}

/// Subtract one direction's tool variants from `part`, trying smooth → exact →
/// expanded (and their axis-aligned fallbacks). `None` if the tool's AABB misses
/// the part or the solver couldn't subtract it. All use the body-splitting
/// difference, so a cut that severs the part yields separate parts.
pub(crate) fn cut_part_one_dir(
    part: &KernelSolid,
    pbb: Option<&([f32; 3], [f32; 3])>,
    smooth: &Option<KernelSolid>,
    exact: &Option<KernelSolid>,
    expanded: &Option<KernelSolid>,
    tbb: Option<&([f32; 3], [f32; 3])>,
) -> Option<Vec<KernelSolid>> {
    let tbb = tbb?;
    let overlaps = pbb.is_none_or(|p| crate::mock_kernel::aabbs_overlap(p, tbb, 0.05));
    if !overlaps {
        return None;
    }
    smooth
        .as_ref()
        .and_then(|t| crate::mock_kernel::difference_bodies(part, t))
        .or_else(|| {
            exact
                .as_ref()
                .and_then(|t| crate::mock_kernel::difference_bodies(part, t))
        })
        .or_else(|| {
            expanded
                .as_ref()
                .and_then(|t| crate::mock_kernel::difference_bodies(part, t))
        })
        .or_else(|| {
            exact
                .as_ref()
                .and_then(|t| crate::mock_kernel::axis_aligned_through_cut(part, t))
                .map(|d| vec![d])
        })
        .or_else(|| {
            expanded
                .as_ref()
                .and_then(|t| crate::mock_kernel::axis_aligned_through_cut(part, t))
                .map(|d| vec![d])
        })
        .or_else(|| {
            exact
                .as_ref()
                .and_then(|t| crate::mock_kernel::axis_aligned_cut_parts(part, t))
        })
        .or_else(|| {
            expanded
                .as_ref()
                .and_then(|t| crate::mock_kernel::axis_aligned_cut_parts(part, t))
        })
}

/// Apply a Cut extrude: subtract each tool from every body part whose AABB it
/// overlaps. For each part it tries the **drawn** direction first (smooth → exact
/// → expanded), then falls back to the **opposite** sweep when the drawn one
/// removes nothing — so a cut drawn away from the body (a positive pocket depth
/// on a top face) still bites instead of silently doing nothing. A solver failure
/// on a body the tool genuinely overlaps leaves the part intact (safer than
/// dropping a valid body) and warns; a part fully consumed by the cut is removed.
pub(crate) fn apply_cut(
    live: &mut [LiveBody],
    extrude_id: &str,
    tools: Vec<CutTool>,
    warnings: &mut Vec<String>,
) {
    for tool in &tools {
        // Pre-test bbox per direction (expanded ⊇ exact ⊇ smooth).
        let fwd_bb = tool
            .expanded
            .as_ref()
            .or(tool.exact.as_ref())
            .or(tool.smooth.as_ref())
            .and_then(crate::mock_kernel::solid_aabb);
        let rev_bb = tool
            .expanded_rev
            .as_ref()
            .or(tool.exact_rev.as_ref())
            .or(tool.smooth_rev.as_ref())
            .and_then(crate::mock_kernel::solid_aabb);
        if fwd_bb.is_none() && rev_bb.is_none() {
            continue;
        }
        // Did the solver fail to subtract this tool from a body it actually
        // overlapped (either direction)? If so the body keeps material the user
        // meant to remove.
        let mut failed_on_overlap = false;
        for body in live.iter_mut() {
            let mut changed = false;
            let mut next: Vec<KernelSolid> = Vec::with_capacity(body.parts.len());
            for part in body.parts.drain(..) {
                let pbb = crate::mock_kernel::solid_aabb(&part);
                let overlaps_dir = |tbb: Option<&([f32; 3], [f32; 3])>| {
                    tbb.is_some_and(|t| {
                        pbb.as_ref()
                            .is_none_or(|p| crate::mock_kernel::aabbs_overlap(p, t, 0.05))
                    })
                };
                // How much of this part the tool's AABB encloses, per direction.
                // The drawn direction's `CUT_OVERSHOOT` dips ~0.1mm past the sketch
                // plane, so a cut aimed *away* from the body still nicks a sliver and
                // would count as "done"; ordering by overlap volume instead sends the
                // cut the way the body actually lies (a deep pocket beats a sliver).
                let overlap_vol = |tbb: Option<&([f32; 3], [f32; 3])>| -> f32 {
                    match (pbb.as_ref(), tbb) {
                        (Some(p), Some(t)) => (0..3)
                            .map(|i| (p.1[i].min(t.1[i]) - p.0[i].max(t.0[i])).max(0.0))
                            .product(),
                        _ => 0.0,
                    }
                };
                let fwd = (&tool.smooth, &tool.exact, &tool.expanded, fwd_bb.as_ref());
                let rev = (
                    &tool.smooth_rev,
                    &tool.exact_rev,
                    &tool.expanded_rev,
                    rev_bb.as_ref(),
                );
                let (first, second) = if overlap_vol(rev_bb.as_ref()) > overlap_vol(fwd_bb.as_ref())
                {
                    (rev, fwd)
                } else {
                    (fwd, rev)
                };
                let cut_parts =
                    cut_part_one_dir(&part, pbb.as_ref(), first.0, first.1, first.2, first.3)
                        .or_else(|| {
                            cut_part_one_dir(
                                &part,
                                pbb.as_ref(),
                                second.0,
                                second.1,
                                second.2,
                                second.3,
                            )
                        });
                match cut_parts {
                    Some(parts) => {
                        changed = true;
                        next.extend(parts);
                    }
                    None => {
                        // Only a genuine solver failure (the tool overlapped this
                        // part in some direction) warrants the warning — a tool that
                        // simply misses the body is normal once both directions are
                        // tried.
                        if overlaps_dir(fwd_bb.as_ref()) || overlaps_dir(rev_bb.as_ref()) {
                            failed_on_overlap = true;
                        }
                        next.push(part);
                    }
                }
            }
            body.parts = next;
            if changed {
                let next_source = body.sketch_source.as_ref().and_then(|source| {
                    tool.circle
                        .and_then(|circle| sketch_source_after_circle_cut(source, circle))
                });
                body.pristine = None;
                body.sketch_source = next_source;
                body.cut_tools.extend(cut_tool_recutter_solids(tool));
            }
        }
        if failed_on_overlap {
            warnings.push(format!(
                "Cut '{extrude_id}': the solver couldn't subtract the tool from a \
                 body it overlaps, so that material was left intact. Try nudging \
                 the sketch off the coplanar face."
            ));
        }
    }
}

pub(crate) fn cut_tool_recutter_solids(tool: &CutTool) -> Vec<KernelSolid> {
    [tool.expanded.as_ref(), tool.expanded_rev.as_ref()]
        .into_iter()
        .flatten()
        .cloned()
        .collect()
}
