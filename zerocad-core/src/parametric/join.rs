use super::*;

/// Combine finished bodies into one persistent body. Every source is validated
/// before anything is consumed, so a dangling history reference leaves the
/// still-valid inputs visible instead of partially applying the operation.
/// Connected parts are fused pairwise. If the complete input set cannot become
/// one connected solid, the operation fails atomically and keeps every source.
pub(crate) fn apply_body_join(
    node_id: &str,
    sources: &[String],
    live: &mut Vec<LiveBody>,
    warnings: &mut Vec<String>,
) {
    let mut unique_sources = Vec::new();
    for source in sources {
        if !unique_sources.contains(source) {
            unique_sources.push(source.clone());
        }
    }
    if unique_sources.len() < 2 {
        warnings.push(format!(
            "Body join '{node_id}': select at least two different bodies."
        ));
        return;
    }

    let mut source_indices = Vec::with_capacity(unique_sources.len());
    for source in &unique_sources {
        let Some(index) = live.iter().position(|body| body.id == *source) else {
            warnings.push(format!(
                "Body join '{node_id}': source body '{source}' no longer exists."
            ));
            return;
        };
        if live[index].parts.is_empty() {
            warnings.push(format!(
                "Body join '{node_id}': source body '{source}' has no solid geometry."
            ));
            return;
        }
        source_indices.push(index);
    }

    let mut incoming = Vec::new();
    for &index in &source_indices {
        incoming.extend(live[index].parts.iter().cloned());
    }

    let mut parts: Vec<KernelSolid> = Vec::new();
    for part in incoming {
        let mut candidate = Some(part);
        let mut index = 0;
        while index < parts.len() {
            let current = candidate.as_ref().expect("join candidate missing");
            if let Some(unioned) = try_body_union(&parts[index], current) {
                parts[index] = unioned;
                candidate = None;
                // The new union can bridge another retained part. Pull it out
                // and scan the complete accumulator again until stable.
                let mut merged = parts.remove(index);
                let mut other = 0;
                while other < parts.len() {
                    if let Some(unioned) = try_body_union(&merged, &parts[other]) {
                        merged = unioned;
                        parts.remove(other);
                        other = 0;
                    } else {
                        other += 1;
                    }
                }
                parts.push(merged);
                break;
            }
            index += 1;
        }
        if let Some(candidate) = candidate {
            parts.push(candidate);
        }
    }

    if parts.len() > 1 {
        warnings.push(format!(
            "Body join '{node_id}': the selected bodies must touch or overlap; \
             both original bodies were left unchanged."
        ));
        return;
    }

    // The complete input set fused to one connected solid. Consume the source
    // bodies only now, so a failed/disconnected Join is atomic.
    source_indices.sort_unstable();
    source_indices.dedup();
    for index in source_indices.into_iter().rev() {
        live.remove(index);
    }
    live.push(LiveBody {
        id: node_id.to_string(),
        parts,
        pristine: None,
        sketch_source: None,
        cut_tools: Vec::new(),
        cut_replay: None,
        edge_mod_cut_history_path_used: false,
        thread_replay: None,
    });
}

/// Attempt a material-preserving union. The AABB gate avoids asking the kernel
/// to fuse clearly disjoint solids, and the containment check rejects the same
/// degenerate boolean result guarded against by sketch-based Join.
fn try_body_union(a: &KernelSolid, b: &KernelSolid) -> Option<KernelSolid> {
    let abb = crate::mock_kernel::solid_aabb(a)?;
    let bbb = crate::mock_kernel::solid_aabb(b)?;
    if !crate::mock_kernel::aabbs_overlap(&abb, &bbb, 0.05) {
        return None;
    }
    let unioned = crate::mock_kernel::union(a, b)?;
    let ubb = crate::mock_kernel::solid_aabb(&unioned)?;
    (crate::mock_kernel::aabb_contains(&ubb, &abb, 0.05)
        && crate::mock_kernel::aabb_contains(&ubb, &bbb, 0.05))
    .then_some(unioned)
}

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
    // Origin shift only — `CoordinateSystem::new` recomputes n = u × v, which
    // flips the sweep on the left-handed ground plane (see with_origin docs).
    cs.with_origin(cs.origin.add(back))
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
///
/// `draft` is set for live drag previews: joining onto a threaded body then
/// skips replaying the (expensive) helical thread so the preview stays fast —
/// the shaft reads smooth mid-drag and the threads return on the committed
/// (non-draft) rebuild.
pub(crate) fn apply_join(
    live: &mut Vec<LiveBody>,
    extrude_id: &str,
    tools: Vec<JoinTool>,
    boolean_target: Option<&str>,
    draft: bool,
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
                // Named targeting: only the pinned body may receive the boss.
                if boolean_target.is_some_and(|t| t != body.id) {
                    continue;
                }
                // Threaded body: a boolean against the helical bands is not viable
                // (neither robust nor fast). Union the boss into the smooth
                // pre-thread base instead, then replay the thread steps so the
                // shaft stays threaded and the boss stays smooth. A miss falls
                // through so the boss can still become its own body, never vanish.
                if body.thread_replay.is_some() {
                    if join_into_threaded_base(body, &tool, !draft) {
                        merged = true;
                        break 'bodies;
                    }
                    continue;
                }
                // Snapshot the input body's named mesh before mutating any part, so a
                // captured face can survive the join's boolean (single-part case).
                let input_mesh = if body.parts.len() == 1 {
                    body.pristine.clone()
                } else {
                    None
                };
                // Exact-history plumbing: input face names per shell position.
                // Join intentionally allows coplanar result faces to merge into
                // one continuous face; the history assigns that merged face its
                // surviving object-side identity.
                let input_names: Option<Vec<Option<String>>> = match (&input_mesh, &body.parts[..])
                {
                    (Some(mesh), [part]) => {
                        Some(crate::mock_kernel::input_shell_face_names(mesh, part))
                    }
                    _ => None,
                };
                let body_id = body.id.clone();
                for part in body.parts.iter_mut() {
                    let overlaps = crate::mock_kernel::solid_aabb(part).map_or(true, |pbb| {
                        crate::mock_kernel::aabbs_overlap(&pbb, &tbb, 0.05)
                    });
                    if !overlaps {
                        continue;
                    }
                    // Smooth analytic cylinder first (round boss), then the faceted
                    // prism variants as robustness fallbacks. With a named input,
                    // run through the history-emitting kernel entry so the union
                    // gets exact face provenance + owner-aware merging.
                    let try_union = |t: &KernelSolid| -> Option<(
                        KernelSolid,
                        Option<crate::mock_kernel::BooleanFaceHistory>,
                    )> {
                        if input_names.is_some() {
                            // No owner classes for Fuse: different historical
                            // owners must not leave a seam between coplanar faces.
                            crate::mock_kernel::union_with_history(part, t, None)
                                .map(|(u, h)| (u, Some(h)))
                        } else {
                            crate::mock_kernel::union(part, t).map(|u| (u, None))
                        }
                    };
                    let unioned = tool
                        .smooth
                        .as_ref()
                        .and_then(&try_union)
                        .or_else(|| tool.exact.as_ref().and_then(&try_union))
                        .or_else(|| tool.dipped.as_ref().and_then(&try_union));
                    if let Some((u, history)) = unioned {
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
                            // Propagate face names from the object body to the union
                            // result. With an exact kernel history the boss's own new
                            // faces ALSO get durable generated names
                            // (`join:{node}:tool-face:{i}`); the matcher path leaves
                            // them unnamed.
                            let named = match (&history, &input_names, &input_mesh) {
                                (Some(history), Some(names), Some(m)) => {
                                    Some(crate::mock_kernel::propagate_face_names_via_history(
                                        m,
                                        names,
                                        &u,
                                        history,
                                        &body_id,
                                        &format!("join:{extrude_id}"),
                                    ))
                                }
                                _ => input_mesh.as_ref().map(|m| {
                                    crate::mock_kernel::propagate_face_names(m, &u, &body_id)
                                }),
                            };
                            *part = u;
                            body.pristine = named.map(std::sync::Arc::new);
                            body.sketch_source = None;
                            body.cut_replay = None;
                            body.edge_mod_cut_history_path_used = false;
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
                        body.cut_replay = None;
                        body.edge_mod_cut_history_path_used = false;
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
            cut_replay: None,
            edge_mod_cut_history_path_used: false,
            thread_replay: None,
        });
    }
}

/// Union `tool` into a threaded body via its smooth pre-thread base
/// (`body.thread_replay`), then replay the thread steps. Keeps the smooth base
/// intact (so still-later booleans keep working) while the displayed geometry
/// is base + boss with the shaft re-threaded. Returns `true` if the boolean
/// merged the boss; `false` if it didn't overlap the base or the union failed,
/// so the caller can treat the boss as a separate body rather than losing it.
///
/// `rethread` re-applies the helical thread steps after the union. It is `false`
/// for draft previews, where the shaft is left smooth for speed (the threads
/// come back on the committed rebuild).
fn join_into_threaded_base(body: &mut LiveBody, tool: &JoinTool, rethread: bool) -> bool {
    let Some(replay) = body.thread_replay.as_mut() else {
        return false;
    };
    let Some(tbb) = tool
        .smooth
        .as_ref()
        .or(tool.exact.as_ref())
        .or(tool.dipped.as_ref())
        .and_then(crate::mock_kernel::solid_aabb)
    else {
        return false;
    };

    // Union into the first smooth base part the boss overlaps. The base is a
    // plain cylinder + rim blends, so this boolean is the robust one the
    // helical geometry never was. `keeps_body` guards against a degenerate
    // union that would remove material (a join must only ever add).
    let mut joined = false;
    for base in replay.base_parts.iter_mut() {
        let overlaps = crate::mock_kernel::solid_aabb(base).map_or(true, |pbb| {
            crate::mock_kernel::aabbs_overlap(&pbb, &tbb, 0.05)
        });
        if !overlaps {
            continue;
        }
        let try_union = |t: &KernelSolid| crate::mock_kernel::union(base, t);
        let unioned = tool
            .smooth
            .as_ref()
            .and_then(&try_union)
            .or_else(|| tool.exact.as_ref().and_then(&try_union))
            .or_else(|| tool.dipped.as_ref().and_then(&try_union));
        if let Some(u) = unioned {
            let keeps_body = match (
                crate::mock_kernel::solid_aabb(base),
                crate::mock_kernel::solid_aabb(&u),
            ) {
                (Some(pbb), Some(ubb)) => crate::mock_kernel::aabb_contains(&ubb, &pbb, 0.05),
                _ => true,
            };
            if keeps_body {
                *base = u;
                joined = true;
                break;
            }
        }
    }
    if !joined {
        return false;
    }

    // Rebuild the displayed geometry from the (now boss-joined) smooth base and
    // replay every thread step on top. `body.parts` gets its own copy so the
    // base held in `thread_replay` stays smooth for future booleans. A step
    // that can no longer re-cut leaves that region smooth — still better than
    // dropping the joined volume.
    let new_base = replay.base_parts.clone();
    let steps = replay.steps.clone();
    body.parts = new_base;
    body.sketch_source = None;
    body.cut_replay = None;
    body.edge_mod_cut_history_path_used = false;
    if rethread {
        for step in &steps {
            let _ = thread_one(body, step);
        }
    }
    refresh_thread_pristine(body);
    true
}
