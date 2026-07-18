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
    apply_new(
        live,
        LiveBody {
            id: node_id.into(),
            parts,
            pristine: None,
            sketch_source: None,
        },
    );
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
    let connected = unioned.split_disconnected().len() <= 1;
    (crate::mock_kernel::aabb_contains(&ubb, &abb, 0.05)
        && crate::mock_kernel::aabb_contains(&ubb, &bbb, 0.05)
        && unioned.is_watertight()
        && unioned.health_report().is_healthy()
        && connected)
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

/// Apply a Join extrude as an atomic feature transaction. Every tool region must
/// produce a connected, valid union. A failed Join never degrades into a
/// separate body or an unfused component hidden inside the target body.
///
/// `draft` is reserved for preview-quality kernel settings; it never changes
/// construction history or invokes a feature-specific replay path.
pub(crate) fn apply_join(
    live: &mut Vec<LiveBody>,
    extrude_id: &str,
    tools: Vec<JoinTool>,
    boolean_target: Option<&str>,
    _draft: bool,
    warnings: &mut Vec<String>,
) {
    // Join is a feature-level transaction. Every region must fuse successfully;
    // otherwise none of them are committed. This prevents a visually plausible
    // but topologically false body containing overlapping, unfused solids.
    let original = live.clone();
    for tool in &tools {
        let mut merged = false;
        for body in live.iter_mut() {
            if boolean_target.is_some_and(|target| target != body.id) {
                continue;
            }

            let mut candidate = body.clone();
            let success = join_tool_into_body(&mut candidate, tool, extrude_id);
            if success {
                *body = candidate;
                merged = true;
                break;
            }
        }

        if !merged {
            *live = original;
            warnings.push(format!(
                "Join '{extrude_id}' could not produce one valid fused solid; \
                 the feature was not applied and its input bodies were left unchanged."
            ));
            return;
        }
    }
}

/// Transactionally union one Join tool into a non-threaded body. All tool
/// variants are tried against a clone; no body state changes until a validated,
/// connected union exists. If the tool bridges multiple body components, the
/// newly fused result is repeatedly unioned with every component it now touches.
fn join_tool_into_body(body: &mut LiveBody, tool: &JoinTool, extrude_id: &str) -> bool {
    let input_mesh = (body.parts.len() == 1)
        .then(|| body.pristine.clone())
        .flatten();
    let input_names = match (&input_mesh, &body.parts[..]) {
        (Some(mesh), [part]) => Some(crate::mock_kernel::input_shell_face_names(mesh, part)),
        _ => None,
    };

    for variant in [
        tool.smooth.as_ref(),
        tool.exact.as_ref(),
        tool.dipped.as_ref(),
    ]
    .into_iter()
    .flatten()
    {
        let Some((parts, history)) =
            union_variant_into_parts(&body.parts, variant, input_names.is_some())
        else {
            continue;
        };

        let named = match (&history, &input_names, &input_mesh, parts.as_slice()) {
            (Some(history), Some(names), Some(mesh), [part]) => {
                Some(crate::mock_kernel::propagate_face_names_via_history(
                    mesh,
                    names,
                    part,
                    history,
                    &body.id,
                    &format!("join:{extrude_id}"),
                ))
            }
            (_, _, Some(mesh), [part]) => Some(crate::mock_kernel::propagate_face_names(
                mesh, part, &body.id,
            )),
            _ => None,
        };

        body.parts = parts;
        body.pristine = named.map(std::sync::Arc::new);
        body.sketch_source = None;
        return true;
    }
    false
}

/// Fuse `tool` into a cloned part set and normalize the connected component it
/// enters. The returned history is valid only for the single-part first union.
fn union_variant_into_parts(
    source_parts: &[KernelSolid],
    tool: &KernelSolid,
    capture_history: bool,
) -> Option<(
    Vec<KernelSolid>,
    Option<crate::mock_kernel::BooleanFaceHistory>,
)> {
    let tool_bb = crate::mock_kernel::solid_aabb(tool)?;
    for start in 0..source_parts.len() {
        let part_bb = crate::mock_kernel::solid_aabb(&source_parts[start])?;
        if !crate::mock_kernel::aabbs_overlap(&part_bb, &tool_bb, 0.05) {
            continue;
        }

        let (mut merged, history) = if capture_history && source_parts.len() == 1 {
            let (unioned, history) = match crate::mock_kernel::union_with_history_diagnostic(
                &source_parts[start],
                tool,
                None,
            ) {
                Ok(result) => result,
                Err(error) => {
                    log::debug!("join boolean candidate rejected: {error}");
                    if std::env::var_os("OPENRCAD_BOOLEAN_DEBUG").is_some() {
                        eprintln!("join boolean candidate rejected: {error}");
                    }
                    continue;
                }
            };
            if !valid_union_result(&source_parts[start], tool, &unioned) {
                continue;
            }
            (unioned, Some(history))
        } else {
            let Some(unioned) = try_body_union(&source_parts[start], tool) else {
                continue;
            };
            (unioned, None)
        };

        let mut remaining: Vec<KernelSolid> = source_parts
            .iter()
            .enumerate()
            .filter(|(index, _)| *index != start)
            .map(|(_, part)| part.clone())
            .collect();
        let mut index = 0;
        while index < remaining.len() {
            if let Some(unioned) = try_body_union(&merged, &remaining[index]) {
                merged = unioned;
                remaining.remove(index);
                index = 0;
            } else {
                index += 1;
            }
        }
        remaining.push(merged);
        remaining.sort_by_key(crate::mock_kernel::part_key);
        return Some((remaining, history));
    }
    None
}

fn valid_union_result(a: &KernelSolid, b: &KernelSolid, result: &KernelSolid) -> bool {
    let (Some(abb), Some(bbb), Some(rbb)) = (
        crate::mock_kernel::solid_aabb(a),
        crate::mock_kernel::solid_aabb(b),
        crate::mock_kernel::solid_aabb(result),
    ) else {
        return false;
    };
    crate::mock_kernel::aabb_contains(&rbb, &abb, 0.05)
        && crate::mock_kernel::aabb_contains(&rbb, &bbb, 0.05)
        && result.is_watertight()
        && result.health_report().is_healthy()
        && result.split_disconnected().len() <= 1
}
