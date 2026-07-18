use super::*;

/// A cut's tool, tried in order, mirroring `JoinTool`: `smooth` is the analytic
/// cylinder for a circular pocket/drill (a clean round hole, not a 48-gon one);
/// `exact` is the faceted prism with the drawn dimensions; `expanded` is a
/// faceted fallback whose walls poke ~`CUT_WALL_GROW`mm past the body's faces to
/// dodge the coplanar-face case. `smooth` is `None` for non-circular profiles.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
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

impl CutTool {
    pub(crate) fn single_direction(
        smooth: Option<KernelSolid>,
        exact: Option<KernelSolid>,
        expanded: Option<KernelSolid>,
        circle: Option<Circle>,
    ) -> Self {
        Self {
            smooth,
            exact,
            expanded,
            smooth_rev: None,
            exact_rev: None,
            expanded_rev: None,
            circle,
        }
    }
}

/// Combine-style body cut: subtract the complete `tool` body from `target`.
/// The operation is atomic and requires a positive-volume overlap. A miss or
/// unchanged/failed boolean leaves both inputs untouched. On success the result
/// is the body produced by `node_id`; the tool is consumed unless `keep_tool` is
/// true. Face provenance still records which target/tool topology contributed.
pub(crate) fn apply_body_cut(
    node_id: &str,
    target: &str,
    tool: &str,
    keep_tool: bool,
    live: &mut Vec<LiveBody>,
    warnings: &mut Vec<String>,
) {
    if target == tool {
        warnings.push(format!(
            "Body cut '{node_id}': target and cutting body must be different."
        ));
        return;
    }
    let Some(target_index) = live.iter().position(|body| body.id == target) else {
        warnings.push(format!(
            "Body cut '{node_id}': target body '{target}' no longer exists."
        ));
        return;
    };
    let Some(tool_index) = live.iter().position(|body| body.id == tool) else {
        warnings.push(format!(
            "Body cut '{node_id}': cutting body '{tool}' no longer exists."
        ));
        return;
    };
    let target_body = live[target_index].clone();
    let tool_body = live[tool_index].clone();
    if target_body.parts.is_empty() || tool_body.parts.is_empty() {
        warnings.push(format!(
            "Body cut '{node_id}': both selected bodies need solid geometry."
        ));
        return;
    }

    let positive_overlap = target_body.parts.iter().any(|target_part| {
        tool_body
            .parts
            .iter()
            .any(|tool_part| solids_overlap_with_volume(target_part, tool_part))
    });
    if !positive_overlap {
        warnings.push(format!(
            "Body cut '{node_id}': the target and cutting body must overlap; \
             both original bodies were left unchanged."
        ));
        return;
    }

    // The exact-history path is unambiguous when one target part is cut by one
    // tool part. Preserve the target's existing face names and give generated
    // tool faces durable names while the boolean still has its lineage table.
    // Retaining this mesh also avoids tessellating the unchanged result again
    // on every checkpoint hit.
    let input_names: Option<Vec<Option<String>>> =
        match (&target_body.pristine, &target_body.parts[..]) {
            (Some(mesh), [part]) => Some(crate::mock_kernel::input_shell_face_names(mesh, part)),
            _ => None,
        };
    let owner_classes = input_names
        .as_ref()
        .map(|names| crate::mock_kernel::owner_classes_from_names(names));
    let exact_history_eligible = target_body.parts.len() == 1 && tool_body.parts.len() == 1;
    let mut result_parts = target_body.parts.clone();
    let mut cut_trace: Option<Vec<crate::mock_kernel::BooleanFaceHistory>> = None;
    let mut changed = false;
    for tool_part in &tool_body.parts {
        let mut next = Vec::new();
        for target_part in result_parts {
            if !solids_overlap_with_volume(&target_part, tool_part) {
                next.push(target_part);
                continue;
            }
            match crate::mock_kernel::difference_bodies_with_history(
                &target_part,
                tool_part,
                exact_history_eligible
                    .then_some(owner_classes.as_deref())
                    .flatten(),
            ) {
                Some(outcome) if cut_parts_changed(&target_part, &outcome.bodies) => {
                    changed = true;
                    if exact_history_eligible {
                        cut_trace = Some(outcome.face_history.clone());
                    }
                    next.extend(outcome.bodies);
                }
                _ => next.push(target_part),
            }
        }
        result_parts = next;
    }
    if !changed {
        warnings.push(format!(
            "Body cut '{node_id}': the solver could not subtract the overlapping body; \
             both original bodies were left unchanged."
        ));
        return;
    }

    let pristine = match (&cut_trace, &input_names, target_body.pristine.as_deref()) {
        (Some(histories), Some(names), Some(input_mesh)) => {
            crate::mock_kernel::propagate_face_names_via_body_histories(
                input_mesh,
                names,
                &result_parts,
                histories,
                node_id,
                &format!("cut:{node_id}"),
            )
            .map(std::sync::Arc::new)
        }
        _ => propagate_cut_face_names(
            &target_body.parts,
            target_body.pristine.as_deref(),
            &result_parts,
            node_id,
        )
        .map(std::sync::Arc::new),
    };

    let mut remove = vec![target_index];
    if !keep_tool {
        remove.push(tool_index);
    }
    remove.sort_unstable();
    remove.dedup();
    for index in remove.into_iter().rev() {
        live.remove(index);
    }
    if !result_parts.is_empty() {
        apply_new(
            live,
            LiveBody {
                id: node_id.into(),
                parts: result_parts,
                pristine,
                sketch_source: None,
            },
        );
    }
}

fn solids_overlap_with_volume(a: &KernelSolid, b: &KernelSolid) -> bool {
    let (Some((alo, ahi)), Some((blo, bhi))) = (
        crate::mock_kernel::solid_aabb(a),
        crate::mock_kernel::solid_aabb(b),
    ) else {
        return false;
    };
    (0..3).all(|axis| ahi[axis].min(bhi[axis]) - alo[axis].max(blo[axis]) > 1.0e-5)
}

pub(crate) fn recut_debug_enabled() -> bool {
    std::env::var_os("ZEROCAD_RECUT_DEBUG").is_some()
}

pub(crate) fn recut_debug(message: impl AsRef<str>) {
    if recut_debug_enabled() {
        eprintln!("[zerocad-recut] {}", message.as_ref());
    }
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
/// A successful one-direction cut: the severed parts, plus — when the general
/// boolean produced them — the kernel's exact face history and the pre-split
/// combined result its face indices refer to. Axis-aligned fallback paths
/// rebuild geometry without a boolean, so they carry no trace.
pub(crate) struct CutOutcome {
    pub(crate) parts: Vec<KernelSolid>,
    pub(crate) trace: Option<Vec<crate::mock_kernel::BooleanFaceHistory>>,
}

pub(crate) fn cut_part_one_dir(
    part: &KernelSolid,
    pbb: Option<&([f32; 3], [f32; 3])>,
    smooth: &Option<KernelSolid>,
    exact: &Option<KernelSolid>,
    expanded: &Option<KernelSolid>,
    tbb: Option<&([f32; 3], [f32; 3])>,
    obj_classes: Option<&[Option<u64>]>,
) -> Option<CutOutcome> {
    let tbb = tbb?;
    let overlaps = pbb.is_none_or(|p| crate::mock_kernel::aabbs_overlap(p, tbb, 0.05));
    if !overlaps {
        return None;
    }
    let changed_difference = |label: &str, tool: &KernelSolid| {
        let outcome = crate::mock_kernel::difference_bodies_with_history(part, tool, obj_classes)?;
        if cut_parts_changed(part, &outcome.bodies) {
            if label == "expanded" {
                let Some(exact_tool) = exact.as_ref() else {
                    recut_debug("expanded cut candidate rejected: no exact reference tool");
                    return None;
                };
                match super::recovery_certificate::certify_expanded_cut(
                    part,
                    exact_tool,
                    tool,
                    &outcome.bodies,
                ) {
                    Ok(certificate) => recut_debug(format!(
                        "expanded cut recovery certified: {}",
                        certificate.summary()
                    )),
                    Err(error) => {
                        recut_debug(format!("expanded cut candidate rejected: {error}"));
                        return None;
                    }
                }
            }
            recut_debug(format!("cut variant '{label}' changed part"));
            Some(CutOutcome {
                parts: outcome.bodies,
                trace: Some(outcome.face_history),
            })
        } else {
            recut_debug(format!("cut variant '{label}' made no geometry change"));
            None
        }
    };
    if let Some(outcome) = smooth
        .as_ref()
        .and_then(|tool| changed_difference("smooth", tool))
    {
        return Some(outcome);
    }
    if let Some(outcome) = exact
        .as_ref()
        .and_then(|tool| changed_difference("exact", tool))
    {
        return Some(outcome);
    }
    if let Some(outcome) = expanded
        .as_ref()
        .and_then(|tool| changed_difference("expanded", tool))
    {
        return Some(outcome);
    }
    recut_debug("all cut variants failed or missed");
    None
}

fn cut_parts_changed(original: &KernelSolid, parts: &[KernelSolid]) -> bool {
    let before = solid_change_signature(original);
    let mut after_mesh = MockMesh::empty();
    for part in parts {
        after_mesh.append(MockMesh::from_solid(part));
    }
    let after = mesh_change_signature(&after_mesh);
    before != after
}

fn solid_change_signature(solid: &KernelSolid) -> (usize, usize, [i64; 6], [i64; 3]) {
    mesh_change_signature(&MockMesh::from_solid(solid))
}

fn mesh_change_signature(mesh: &MockMesh) -> (usize, usize, [i64; 6], [i64; 3]) {
    let q = |v: f32| (v as f64 * 10_000.0).round() as i64;
    let mut lo = [i64::MAX; 3];
    let mut hi = [i64::MIN; 3];
    let mut sum = [0i64; 3];
    for v in mesh.vertices.chunks_exact(6) {
        for axis in 0..3 {
            let value = q(v[axis]);
            lo[axis] = lo[axis].min(value);
            hi[axis] = hi[axis].max(value);
            sum[axis] = sum[axis].wrapping_add(value);
        }
    }
    if mesh.vertices.is_empty() {
        lo = [0; 3];
        hi = [0; 3];
    }
    (
        mesh.vertices.len(),
        mesh.indices.len(),
        [lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]],
        sum,
    )
}

pub(crate) fn cut_tool_bboxes(
    tool: &CutTool,
) -> (Option<([f32; 3], [f32; 3])>, Option<([f32; 3], [f32; 3])>) {
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
    (fwd_bb, rev_bb)
}

/// Apply a Cut extrude: subtract each tool from every body part whose AABB it
/// overlaps. For each part it tries the **drawn** direction first (smooth → exact
/// → expanded), then falls back to the **opposite** sweep when the drawn one
/// removes nothing — so a cut drawn away from the body (a positive pocket depth
/// on a top face) still bites instead of silently doing nothing. A solver failure
/// on a body the tool genuinely overlaps leaves the part intact (safer than
/// dropping a valid body) and warns; a part fully consumed by the cut is removed.
/// `draft` is reserved for preview-quality kernel settings; it never changes
/// construction history or invokes a feature-specific replay path.
pub(crate) fn apply_cut(
    live: &mut [LiveBody],
    extrude_id: &str,
    tools: Vec<CutTool>,
    boolean_target: Option<&str>,
    _draft: bool,
    warnings: &mut Vec<String>,
) {
    for tool in &tools {
        // Pre-test bbox per direction (expanded ⊇ exact ⊇ smooth).
        let (fwd_bb, rev_bb) = cut_tool_bboxes(tool);
        if fwd_bb.is_none() && rev_bb.is_none() {
            continue;
        }
        // Did the solver fail to subtract this tool from a body it actually
        // overlapped (either direction)? If so the body keeps material the user
        // meant to remove.
        let mut failed_on_overlap = false;
        for body in live.iter_mut() {
            // Named targeting: only the pinned body is cut (legacy `None`
            // keeps the historical every-overlapping-body behavior).
            if boolean_target.is_some_and(|t| t != body.id) {
                continue;
            }
            let before_parts = body.parts.clone();
            let before_pristine = body.pristine.clone();
            // Exact-history plumbing for the single-part common case: the input
            // body's face names per shell-face position, and the derived owner
            // classes the kernel's owner-aware merge respects.
            let input_names: Option<Vec<Option<String>>> =
                match (&before_pristine, &before_parts[..]) {
                    (Some(mesh), [part]) => {
                        Some(crate::mock_kernel::input_shell_face_names(mesh, part))
                    }
                    _ => None,
                };
            let owner_classes: Option<Vec<Option<u64>>> = input_names
                .as_ref()
                .map(|names| crate::mock_kernel::owner_classes_from_names(names));
            let mut cut_trace: Option<Vec<crate::mock_kernel::BooleanFaceHistory>> = None;
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
                let cut_parts = cut_part_one_dir(
                    &part,
                    pbb.as_ref(),
                    first.0,
                    first.1,
                    first.2,
                    first.3,
                    owner_classes.as_deref(),
                )
                .or_else(|| {
                    cut_part_one_dir(
                        &part,
                        pbb.as_ref(),
                        second.0,
                        second.1,
                        second.2,
                        second.3,
                        owner_classes.as_deref(),
                    )
                });
                match cut_parts {
                    Some(outcome) => {
                        changed = true;
                        cut_trace = outcome.trace;
                        next.extend(outcome.parts);
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
                // Propagate the input body's face names through the cut so a captured
                // face survives the boolean (Phase 3). When the kernel emitted an
                // exact history (general-boolean path, single named input part),
                // names ride the history — MODIFIED faces inherit their owner and
                // the cut's GENERATED walls get durable `cut:{node}:tool-face:{i}`
                // names. Otherwise the geometric matcher covers it; any mismatch
                // falls back to `None` (re-tessellation), never a wrong name.
                body.pristine = match (&cut_trace, &input_names) {
                    (Some(histories), Some(names))
                        if before_parts.len() == 1 && before_pristine.is_some() =>
                    {
                        crate::mock_kernel::propagate_face_names_via_body_histories(
                            before_pristine.as_ref().unwrap(),
                            names,
                            &body.parts,
                            histories,
                            &body.id,
                            &format!("cut:{extrude_id}"),
                        )
                        .map(std::sync::Arc::new)
                    }
                    _ => propagate_cut_face_names(
                        &before_parts,
                        before_pristine.as_deref(),
                        &body.parts,
                        &body.id,
                    )
                    .map(std::sync::Arc::new),
                };
                body.sketch_source = next_source;
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

/// Build a name-propagated pristine mesh for a cut result, or `None` to fall back
/// to plain re-tessellation. Gated to one-part-in / one-part-out with a named input
/// mesh — the common case — so a captured face reattaches through the cut while
/// severing/multi-part cuts degrade safely to today's behavior.
fn propagate_cut_face_names(
    before_parts: &[KernelSolid],
    before_pristine: Option<&MockMesh>,
    after_parts: &[KernelSolid],
    body_id: &str,
) -> Option<MockMesh> {
    // Propagate from a single named input body; the cut may still SEVER it into
    // several lumps, and each lump inherits the names of the faces it continues
    // (mesh-to-mesh matching is per-part, so multi-part output is fine).
    if before_parts.len() != 1 || after_parts.is_empty() {
        return None;
    }
    let input_mesh = before_pristine?;
    let mut mesh = MockMesh::empty();
    for part in after_parts {
        mesh.append(crate::mock_kernel::propagate_face_names(
            input_mesh, part, body_id,
        ));
    }
    Some(mesh)
}
