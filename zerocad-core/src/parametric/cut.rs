use super::*;

/// A cut's tool, tried in order, mirroring `JoinTool`: `smooth` is the analytic
/// cylinder for a circular pocket/drill (a clean round hole, not a 48-gon one);
/// `exact` is the faceted prism with the drawn dimensions; `expanded` is a
/// faceted fallback whose walls poke ~`CUT_WALL_GROW`mm past the body's faces to
/// dodge the coplanar-face case. `smooth` is `None` for non-circular profiles.
#[derive(Debug, Clone)]
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

    pub(crate) fn has_any_solid(&self) -> bool {
        [
            self.smooth.as_ref(),
            self.exact.as_ref(),
            self.expanded.as_ref(),
            self.smooth_rev.as_ref(),
            self.exact_rev.as_ref(),
            self.expanded_rev.as_ref(),
        ]
        .into_iter()
        .any(|solid| solid.is_some())
    }
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
    let changed_difference = |label: &str, tool: &KernelSolid| {
        let parts = crate::mock_kernel::difference_bodies(part, tool)?;
        if cut_parts_changed(part, &parts) {
            recut_debug(format!("cut variant '{label}' changed part"));
            Some(parts)
        } else {
            recut_debug(format!("cut variant '{label}' made no geometry change"));
            None
        }
    };
    if let Some(parts) = smooth
        .as_ref()
        .and_then(|tool| changed_difference("smooth", tool))
    {
        return Some(parts);
    }
    if let Some(parts) = exact
        .as_ref()
        .and_then(|tool| changed_difference("exact", tool))
    {
        return Some(parts);
    }
    if let Some(parts) = expanded
        .as_ref()
        .and_then(|tool| changed_difference("expanded", tool))
    {
        return Some(parts);
    }
    if let Some(part) = exact
        .as_ref()
        .and_then(|tool| crate::mock_kernel::axis_aligned_through_cut(part, tool))
    {
        recut_debug("cut variant 'exact axis-aligned through' changed part");
        return Some(vec![part]);
    }
    if let Some(part) = expanded
        .as_ref()
        .and_then(|tool| crate::mock_kernel::axis_aligned_through_cut(part, tool))
    {
        recut_debug("cut variant 'expanded axis-aligned through' changed part");
        return Some(vec![part]);
    }
    if let Some(parts) = exact
        .as_ref()
        .and_then(|tool| crate::mock_kernel::axis_aligned_cut_parts(part, tool))
    {
        recut_debug("cut variant 'exact axis-aligned parts' changed part");
        return Some(parts);
    }
    if let Some(parts) = expanded
        .as_ref()
        .and_then(|tool| crate::mock_kernel::axis_aligned_cut_parts(part, tool))
    {
        recut_debug("cut variant 'expanded axis-aligned parts' changed part");
        return Some(parts);
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

pub(crate) fn cut_part_with_tool(part: &KernelSolid, tool: &CutTool) -> Option<Vec<KernelSolid>> {
    let (fwd_bb, rev_bb) = cut_tool_bboxes(tool);
    if fwd_bb.is_none() && rev_bb.is_none() {
        return None;
    }
    let pbb = crate::mock_kernel::solid_aabb(part);
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
    let (first, second) = if overlap_vol(rev_bb.as_ref()) > overlap_vol(fwd_bb.as_ref()) {
        (rev, fwd)
    } else {
        (fwd, rev)
    };
    cut_part_one_dir(part, pbb.as_ref(), first.0, first.1, first.2, first.3)
        .or_else(|| cut_part_one_dir(part, pbb.as_ref(), second.0, second.1, second.2, second.3))
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
        let (fwd_bb, rev_bb) = cut_tool_bboxes(tool);
        if fwd_bb.is_none() && rev_bb.is_none() {
            continue;
        }
        // Did the solver fail to subtract this tool from a body it actually
        // overlapped (either direction)? If so the body keeps material the user
        // meant to remove.
        let mut failed_on_overlap = false;
        for body in live.iter_mut() {
            let before_parts = body.parts.clone();
            let before_pristine = body.pristine.clone();
            let before_sketch_source = body.sketch_source.clone();
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
                let replay_tool = next_source
                    .as_ref()
                    .and_then(cut_replay_tool_from_source)
                    .unwrap_or_else(|| tool.clone());
                // Propagate the input body's face names through the cut so a captured
                // face survives the boolean (Phase 3). Safety-gated to the single-part
                // common case; any shape/name mismatch falls back to `None`
                // (re-tessellation), never a wrong name.
                body.pristine = propagate_cut_face_names(
                    &before_parts,
                    before_pristine.as_ref(),
                    &body.parts,
                    &body.id,
                );
                body.sketch_source = next_source;
                body.cut_tools.extend(cut_tool_recutter_tools(&replay_tool));
                body.edge_mod_cut_history_path_used = false;
                let mut replay = body.cut_replay.clone().unwrap_or_else(|| CutReplayHistory {
                    base_body_id: body.id.clone(),
                    base_parts: before_parts,
                    base_pristine: before_pristine,
                    base_sketch_source: before_sketch_source,
                    steps: Vec::new(),
                });
                replay.steps.push(CutReplayStep {
                    node_id: extrude_id.to_string(),
                    tool: replay_tool,
                });
                body.cut_replay = Some(replay);
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

pub(crate) fn cut_tool_recutter_tools(tool: &CutTool) -> Vec<CutTool> {
    tool.has_any_solid()
        .then_some(tool.clone())
        .into_iter()
        .collect()
}

fn cut_replay_tool_from_source(source: &SketchExtrudeSource) -> Option<CutTool> {
    source.regions.iter().find_map(|region| {
        let canonical = region.rect_circle.as_ref()?;
        let expanded = crate::mock_kernel::rect_minus_circle_region_base_and_grown_cutter(
            &region.boundary,
            &region.holes,
            region.depth,
            &region.cs,
            CUT_WALL_GROW,
        )
        .map(|(_, cutter)| cutter);
        Some(CutTool::single_direction(
            Some(canonical.cutter.clone()),
            None,
            expanded,
            None,
        ))
    })
}

pub(crate) fn replay_nodes_are_ordered_subset(
    steps: &[CutReplayStep],
    requested: &[String],
) -> bool {
    let mut cursor = 0usize;
    for wanted in requested {
        let Some(offset) = steps[cursor..]
            .iter()
            .position(|step| step.node_id == *wanted)
        else {
            return false;
        };
        cursor += offset + 1;
    }
    true
}

pub(crate) fn replay_cut_history(
    mut parts: Vec<KernelSolid>,
    steps: &[CutReplayStep],
) -> Result<Vec<KernelSolid>, String> {
    for (step_index, step) in steps.iter().enumerate() {
        recut_debug(format!(
            "replay step {step_index} '{}' across {} part(s)",
            step.node_id,
            parts.len()
        ));
        let (fwd_bb, rev_bb) = cut_tool_bboxes(&step.tool);
        let mut next = Vec::with_capacity(parts.len());
        for (part_index, part) in parts.into_iter().enumerate() {
            let pbb = crate::mock_kernel::solid_aabb(&part);
            let overlaps = |tbb: Option<&([f32; 3], [f32; 3])>| {
                tbb.is_some_and(|t| {
                    pbb.as_ref()
                        .is_none_or(|p| crate::mock_kernel::aabbs_overlap(p, t, 0.05))
                })
            };
            match cut_part_with_tool(&part, &step.tool) {
                Some(cut_parts) => {
                    recut_debug(format!(
                        "replay step '{}' cut part {part_index} into {} part(s)",
                        step.node_id,
                        cut_parts.len()
                    ));
                    next.extend(cut_parts);
                }
                None if overlaps(fwd_bb.as_ref()) || overlaps(rev_bb.as_ref()) => {
                    let reason = format!("cut '{}' could not be replayed", step.node_id);
                    recut_debug(&reason);
                    return Err(reason);
                }
                None => {
                    recut_debug(format!(
                        "replay step '{}' missed part {part_index}",
                        step.node_id
                    ));
                    next.push(part);
                }
            }
        }
        parts = next;
    }
    if parts.is_empty() {
        recut_debug("replayed cuts consumed the entire body");
        Err("replayed cuts consumed the entire body".to_string())
    } else {
        Ok(parts)
    }
}

pub(crate) fn recut_replayed_parts_with_tools(
    mut parts: Vec<KernelSolid>,
    tools: &[CutTool],
) -> Result<Vec<KernelSolid>, String> {
    for (tool_index, tool) in tools.iter().enumerate() {
        recut_debug(format!(
            "recut cached tool {tool_index} across {} part(s)",
            parts.len()
        ));
        let (fwd_bb, rev_bb) = cut_tool_bboxes(tool);
        let mut next = Vec::with_capacity(parts.len());
        for (part_index, part) in parts.into_iter().enumerate() {
            let pbb = crate::mock_kernel::solid_aabb(&part);
            let overlaps = |tbb: Option<&([f32; 3], [f32; 3])>| {
                tbb.is_some_and(|t| {
                    pbb.as_ref()
                        .is_none_or(|p| crate::mock_kernel::aabbs_overlap(p, t, 0.05))
                })
            };
            if !overlaps(fwd_bb.as_ref()) && !overlaps(rev_bb.as_ref()) {
                recut_debug(format!(
                    "recut cached tool {tool_index} missed part {part_index}"
                ));
                next.push(part);
                continue;
            }
            match cut_part_with_tool(&part, tool) {
                Some(cut_parts) => {
                    recut_debug(format!(
                        "recut cached tool {tool_index} cut part {part_index} into {} part(s)",
                        cut_parts.len()
                    ));
                    next.extend(cut_parts);
                }
                None => {
                    recut_debug(format!(
                        "recut cached tool {tool_index} overlapped part {part_index} but failed"
                    ));
                    next.push(part);
                }
            }
        }
        parts = next;
    }
    if parts.is_empty() {
        recut_debug("grown recut consumed the entire replayed body");
        Err("grown recut consumed the entire replayed body".to_string())
    } else {
        Ok(parts)
    }
}

pub(crate) fn recut_candidate_with_tools(
    candidate: KernelSolid,
    tools: &[CutTool],
) -> Result<Option<KernelSolid>, String> {
    if tools.is_empty() {
        return Ok(None);
    }
    let mut recut = recut_replayed_parts_with_tools(vec![candidate], tools)?;
    if recut.len() == 1 {
        Ok(recut.pop())
    } else {
        Err(format!(
            "recut produced {} separate parts where one part was expected",
            recut.len()
        ))
    }
}

pub(crate) fn replay_cut_void_ghost_sample_count(
    history: &CutReplayHistory,
    candidate_mesh: &MockMesh,
) -> usize {
    let voids: Vec<ReplayCutVoid> = history
        .steps
        .iter()
        .flat_map(|step| replay_cut_void_solids(&step.tool))
        .map(|solid| {
            let mesh = MockMesh::from_solid(solid);
            let aabb = mesh_position_aabb(&mesh);
            ReplayCutVoid { mesh, aabb }
        })
        .filter(|void| !void.mesh.indices.is_empty())
        .collect();
    if voids.is_empty() {
        return 0;
    }

    let vertex6 = |vi: u32| {
        let b = vi as usize * 6;
        [
            candidate_mesh.vertices[b],
            candidate_mesh.vertices[b + 1],
            candidate_mesh.vertices[b + 2],
        ]
    };
    let mut count = 0usize;
    for v in candidate_mesh.vertices.chunks_exact(6) {
        let p = [v[0], v[1], v[2]];
        if replay_cut_void_contains_non_wall_sample(&voids, p) {
            count += 1;
        }
    }
    for tri in candidate_mesh.indices.chunks_exact(3) {
        let a = vertex6(tri[0]);
        let b = vertex6(tri[1]);
        let c = vertex6(tri[2]);
        let p = [
            (a[0] + b[0] + c[0]) / 3.0,
            (a[1] + b[1] + c[1]) / 3.0,
            (a[2] + b[2] + c[2]) / 3.0,
        ];
        if replay_cut_void_contains_non_wall_sample(&voids, p) {
            count += 1;
        }
    }
    count
}

pub(crate) fn replay_cut_void_solids(tool: &CutTool) -> impl Iterator<Item = &KernelSolid> {
    [
        tool.smooth.as_ref().or(tool.exact.as_ref()),
        tool.smooth_rev.as_ref().or(tool.exact_rev.as_ref()),
    ]
    .into_iter()
    .flatten()
}

struct ReplayCutVoid {
    mesh: MockMesh,
    aabb: Option<([f32; 3], [f32; 3])>,
}

fn replay_cut_void_contains_non_wall_sample(voids: &[ReplayCutVoid], p: [f32; 3]) -> bool {
    const WALL_TOL: f32 = 0.18;
    voids.iter().any(|void| {
        if let Some((lo, hi)) = void.aabb {
            if !point_in_aabb(p, lo, hi, WALL_TOL) {
                return false;
            }
        }
        if mesh_surface_distance_sq(&void.mesh, p) <= WALL_TOL * WALL_TOL {
            return false;
        }
        point_inside_triangle_mesh(&void.mesh, p, 0.02)
    })
}

fn mesh_surface_distance_sq(mesh: &MockMesh, p: [f32; 3]) -> f32 {
    let mut best = f32::INFINITY;
    for tri in mesh.indices.chunks_exact(3) {
        let a = mesh_vertex_pos6(mesh, tri[0]);
        let b = mesh_vertex_pos6(mesh, tri[1]);
        let c = mesh_vertex_pos6(mesh, tri[2]);
        best = best.min(point_triangle_distance_sq(p, a, b, c));
    }
    best
}
