use super::*;

/// Edge-cutter facet cap for the boolean fallback. Native
/// fillet/chamfer edits call this path only after native failure. The cutter
/// tessellates adaptively (~3.6°/segment) up to this cap, so a right-angle edge
/// rounds with ~24 facets — smooth enough that, with the facet-boundary lines
/// suppressed (see `mesh_feature_edges`), the fillet reads as one curved face —
/// while keeping truck's boolean cutter face count bounded.
#[allow(dead_code)]
pub(crate) const EDGE_FILLET_SEGS: usize = 24;

/// Robust fallback edge-cutter grow amount. Must clear `BOOL_TOL`
/// (0.05mm) by a healthy margin so the cutter's tangent edges read as cleanly
/// *outside* the body faces rather than tangent — the configuration truck's
/// boolean solver rejects. Costs up to this much chamfer/fillet size in the
/// fallback path, the price of a boolean that resolves at all.
pub(crate) const EDGE_MOD_GROW: f32 = 0.2;

/// Robust fallback cutter overshoot past each selected edge endpoint. The exact
/// fallback tries no overshoot first; this second pass clears endpoint caps and
/// curved-wall runout tangencies.
#[allow(dead_code)]
pub(crate) const EDGE_MOD_END_OVERSHOOT: f32 = 1.0;

/// A fillet/chamfer is subtractive, but B-Rep kernels can occasionally return a
/// topologically valid-looking result that renders new material. Permit only a
/// tiny numerical skin outside the pre-edge-mod body.
pub(crate) const EDGE_MOD_CONTAINMENT_TOL: f32 = EDGE_MOD_GROW + 0.05;

fn edge_mod_timing_enabled() -> bool {
    std::env::var_os("ZEROCAD_TIMING").is_some() || std::env::var_os("ZEROCAD_PERF").is_some()
}

fn edge_mod_timing(label: impl AsRef<str>, started: std::time::Instant) {
    if edge_mod_timing_enabled() {
        eprintln!(
            "[zerocad-timing] {}: {:.1?}",
            label.as_ref(),
            started.elapsed()
        );
    }
}

/// Apply a 3D fillet or chamfer to the target body.
///
/// **Fillet** uses OpenRCAD's native rolling-ball blend
/// ([`crate::mock_kernel::fillet_edge`]): the captured edge is located in each
/// part's B-Rep by its endpoints and replaced by a true cylindrical fillet face
/// with no draft/commit split. Radius feasibility is delegated to the exact
/// kernel solve and candidate validation so valid runouts are not blocked by a
/// conservative app-level clearance estimate.
///
/// **Chamfer** uses OpenRCAD's native selected-edge bevel
/// ([`crate::mock_kernel::chamfer_edge`]).
///
/// Degenerate selections and invalid distances are rejected up front. Other
/// difficult cases enter the exact solver and are accepted only if validation
/// proves the edit is local and subtractive; if a candidate refills a cut void or
/// adds visible material, the body is left unchanged with a warning.
///
/// `draft` is retained for API compatibility but no longer changes the result:
/// edge modifiers resolve in a single pass, so the live preview and
/// the committed model are identical.
pub(crate) fn apply_edge_mod(
    mod_id: &str,
    target: &str,
    edge: &EdgeRef,
    _scope: &EdgeModScope,
    replay: &EdgeModReplayIntent,
    dist: f32,
    kind: crate::sketch::CornerKind,
    _draft: bool,
    live: &mut [LiveBody],
    warnings: &mut Vec<String>,
) {
    let Some(body) = live.iter_mut().find(|b| b.id == target) else {
        warnings.push(format!(
            "Fillet/Chamfer '{mod_id}': its target body no longer exists, so it \
             had no effect."
        ));
        return;
    };

    let label = match kind {
        crate::sketch::CornerKind::Fillet => "Fillet",
        crate::sketch::CornerKind::Chamfer => "Chamfer",
    };
    let resolved_edge = resolve_edge_ref_by_topology(body, edge).unwrap_or_else(|| edge.clone());
    if let Err(reason) = edge_mod_preflight(body, &resolved_edge, dist) {
        warnings.push(format!(
            "{label} '{mod_id}': {reason}, so the body was left unchanged."
        ));
        return;
    }
    let selection = EdgeModSelection::new(&resolved_edge);

    match kind {
        crate::sketch::CornerKind::Fillet => {
            apply_fillet(mod_id, &selection, replay, dist, body, warnings)
        }
        crate::sketch::CornerKind::Chamfer => {
            apply_chamfer(mod_id, &selection, dist, body, warnings)
        }
    }
}

pub(crate) fn resolve_edge_ref_by_topology(body: &LiveBody, edge: &EdgeRef) -> Option<EdgeRef> {
    let requested = edge.topology.as_ref()?;
    if requested
        .body_id
        .as_deref()
        .is_some_and(|body_id| body_id != body.id)
    {
        return None;
    }

    // 1. Exact edge-id match (a stable design id survives an equivalent edit).
    //    A boolean can split one design edge into several fragments that all
    //    carry the same id (a bite cuts the middle out of a rectangle's top
    //    edge), so an id match alone is ambiguous — disambiguate by geometry,
    //    never by enumeration order.
    if let Some(requested_edge_id) = requested.edge_id.as_deref() {
        let pick_by_id = |mesh: &MockMesh| -> Option<EdgeRef> {
            let matches: Vec<&crate::mock_kernel::MeshEdgeRef> = mesh
                .edge_refs
                .iter()
                .filter(|candidate| topology_edge_id(candidate) == Some(requested_edge_id))
                .collect();
            let span_dist = |c: &crate::mock_kernel::MeshEdgeRef| -> f32 {
                let fwd = distance3(c.p0, edge.p0) + distance3(c.p1, edge.p1);
                let rev = distance3(c.p0, edge.p1) + distance3(c.p1, edge.p0);
                fwd.min(rev)
            };
            let best = match matches.len() {
                0 => None,
                1 => matches
                    .into_iter()
                    .find(|c| mesh_candidate_matches_captured_edge(c, edge)),
                _ => {
                    let geometric: Vec<&crate::mock_kernel::MeshEdgeRef> = matches
                        .iter()
                        .copied()
                        .filter(|c| mesh_candidate_matches_captured_edge(c, edge))
                        .collect();
                    // Prefer candidates that also match geometrically; if an
                    // edit moved every fragment (the stable-id escape hatch
                    // exists for exactly that), fall back to the nearest.
                    let pool = if geometric.is_empty() { matches } else { geometric };
                    pool.into_iter().min_by(|a, b| {
                        span_dist(a)
                            .partial_cmp(&span_dist(b))
                            .unwrap_or(std::cmp::Ordering::Equal)
                    })
                }
            };
            best.map(|candidate| edge_ref_from_mesh_candidate(body, candidate, requested))
        };

        if let Some(resolved) = body.pristine.as_ref().and_then(|mesh| pick_by_id(mesh)) {
            return Some(resolved);
        }
        if let Some(resolved) = pick_by_id(&edge_mod_reference_mesh(body)) {
            return Some(resolved);
        }
    }

    // 2. Face-owner-pair fallback: an edge is identified by the pair of faces it
    //    separates, and those faces survive a boolean (Phase 3/4a) even when the
    //    edge's own id is re-derived (a sketch id becomes a `mesh:group` id after a
    //    cut). Match on the same face-owner pair, disambiguated by geometry.
    resolve_edge_by_face_pair(body, edge, requested)
}

fn resolve_edge_by_face_pair(
    body: &LiveBody,
    edge: &EdgeRef,
    requested: &TopologyEdgeRef,
) -> Option<EdgeRef> {
    if requested.adjacent_face_ids.len() != 2 {
        return None;
    }
    let mut want = requested.adjacent_face_ids.clone();
    want.sort();
    let pick = |mesh: &MockMesh| -> Option<EdgeRef> {
        mesh.edge_refs
            .iter()
            .filter(|candidate| {
                let mut got = candidate
                    .topology
                    .as_ref()
                    .map(|t| t.adjacent_face_ids.clone())
                    .unwrap_or_default();
                got.sort();
                got.len() == 2 && got == want
            })
            .find(|candidate| mesh_candidate_matches_captured_edge(candidate, edge))
            .map(|candidate| edge_ref_from_mesh_candidate(body, candidate, requested))
    };
    body.pristine
        .as_ref()
        .and_then(pick)
        .or_else(|| pick(&edge_mod_reference_mesh(body)))
}

pub(crate) fn topology_edge_id(edge: &crate::mock_kernel::MeshEdgeRef) -> Option<&str> {
    edge.topology
        .as_ref()
        .and_then(|topology| topology.edge_id.as_deref())
}

pub(crate) fn edge_ref_from_mesh_candidate(
    body: &LiveBody,
    candidate: &crate::mock_kernel::MeshEdgeRef,
    requested: &TopologyEdgeRef,
) -> EdgeRef {
    let mut topology = candidate.topology.as_ref().map(|topology| TopologyEdgeRef {
        body_id: topology.body_id.clone().or_else(|| Some(body.id.clone())),
        topology_version: topology.topology_version,
        edge_id: topology.edge_id.clone(),
        adjacent_face_ids: topology.adjacent_face_ids.clone(),
        curve_kind: topology.curve_kind.clone(),
        adjacent_surface_kinds: topology.adjacent_surface_kinds.clone(),
    });
    if topology.is_none() {
        topology = Some(requested.clone());
    }
    EdgeRef {
        p0: candidate.p0,
        p1: candidate.p1,
        n1: candidate.n1,
        n2: candidate.n2,
        curve: candidate.curve.clone(),
        topology,
    }
}

/// Resolve a captured [`FaceRef`] against a rebuilt body — the face analogue of
/// [`resolve_edge_ref_by_topology`]. A face carrying a durable name is resolved by
/// that name (pristine mesh first, then the tessellated reference mesh). If the
/// name is gone we return `None` so the caller reports the feature **unresolved**
/// rather than silently retargeting the wrong face (the "suspend, don't
/// substitute" rule). A face with no name falls back to nearest-by-geometry.
// Consumed by the Phase 1 reattachment tests today; wired into sketch-on-face and
// cut/join targeting in Phase 4.
#[allow(dead_code)]
pub(crate) fn resolve_face_ref_by_topology(body: &LiveBody, face: &FaceRef) -> Option<FaceRef> {
    if let Some(requested) = face.topology.as_ref() {
        if let Some(requested_face_id) = requested.face_id.as_deref() {
            if requested
                .body_id
                .as_deref()
                .is_some_and(|body_id| body_id != body.id)
            {
                return None;
            }
            if let Some(resolved) = body.pristine.as_ref().and_then(|mesh| {
                mesh.face_refs
                    .iter()
                    .find(|c| topology_face_id(c) == Some(requested_face_id))
                    .map(|c| face_ref_from_mesh_face(body, c, requested))
            }) {
                return Some(resolved);
            }
            let mesh = edge_mod_reference_mesh(body);
            return mesh
                .face_refs
                .iter()
                .find(|c| topology_face_id(c) == Some(requested_face_id))
                .map(|c| face_ref_from_mesh_face(body, c, requested));
        }
    }
    // Unnamed capture (legacy, or a not-yet-named boolean-result face): fall back
    // to the nearest face pointing the same way.
    resolve_face_ref_by_geometry(body, face)
}

#[allow(dead_code)]
pub(crate) fn topology_face_id(face: &crate::mock_kernel::MeshFaceRef) -> Option<&str> {
    face.topology
        .as_ref()
        .and_then(|topology| topology.face_id.as_deref())
}

#[allow(dead_code)]
fn face_ref_from_mesh_face(
    body: &LiveBody,
    candidate: &crate::mock_kernel::MeshFaceRef,
    requested: &TopologyFaceRef,
) -> FaceRef {
    let topology = candidate
        .topology
        .as_ref()
        .map(|t| TopologyFaceRef {
            body_id: t.body_id.clone().or_else(|| Some(body.id.clone())),
            topology_version: t.topology_version,
            face_id: t.face_id.clone(),
            surface_kind: t.surface_kind.clone(),
        })
        .or_else(|| Some(requested.clone()));
    FaceRef {
        centroid: candidate.centroid,
        normal: candidate.normal,
        topology,
    }
}

#[allow(dead_code)]
fn resolve_face_ref_by_geometry(body: &LiveBody, face: &FaceRef) -> Option<FaceRef> {
    let pick = |mesh: &MockMesh| -> Option<FaceRef> {
        mesh.face_refs
            .iter()
            .filter(|c| dot3(c.normal, face.normal) >= 0.7)
            .min_by(|a, b| {
                distance3(a.centroid, face.centroid)
                    .partial_cmp(&distance3(b.centroid, face.centroid))
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|c| FaceRef {
                centroid: c.centroid,
                normal: c.normal,
                topology: c.topology.as_ref().map(|t| TopologyFaceRef {
                    body_id: t.body_id.clone().or_else(|| Some(body.id.clone())),
                    topology_version: t.topology_version,
                    face_id: t.face_id.clone(),
                    surface_kind: t.surface_kind.clone(),
                }),
            })
    };
    body.pristine
        .as_ref()
        .and_then(pick)
        .or_else(|| pick(&edge_mod_reference_mesh(body)))
}

pub(crate) fn mesh_candidate_matches_captured_edge(
    candidate: &crate::mock_kernel::MeshEdgeRef,
    edge: &EdgeRef,
) -> bool {
    if !edge_curves_are_compatible(candidate.curve.as_ref(), edge.curve.as_ref()) {
        return false;
    }
    if captured_edge_uses_stable_design_topology(candidate, edge) {
        return true;
    }

    let requested = sub3(edge.p1, edge.p0);
    let req_len = length_sq3(requested).sqrt();
    if req_len <= 1.0e-4 {
        return false;
    }
    let candidate_run = sub3(candidate.p1, candidate.p0);
    let cand_len = length_sq3(candidate_run).sqrt();
    if cand_len <= 1.0e-4 {
        return false;
    }

    let tol = (req_len * 0.05).clamp(0.08, 0.35);
    let endpoint_match = (distance3(candidate.p0, edge.p0) <= tol
        && distance3(candidate.p1, edge.p1) <= tol)
        || (distance3(candidate.p0, edge.p1) <= tol && distance3(candidate.p1, edge.p0) <= tol);
    if endpoint_match {
        return true;
    }

    let axis = mul3(requested, 1.0 / req_len);
    if point_line_distance3(candidate.p0, edge.p0, axis) > tol
        || point_line_distance3(candidate.p1, edge.p0, axis) > tol
    {
        return false;
    }

    let t0 = dot3(sub3(candidate.p0, edge.p0), axis);
    let t1 = dot3(sub3(candidate.p1, edge.p0), axis);
    let overlap = t0.max(t1).min(req_len) - t0.min(t1).max(0.0);
    overlap >= (req_len * 0.35).min(0.75).max(tol)
}

pub(crate) fn captured_edge_uses_stable_design_topology(
    candidate: &crate::mock_kernel::MeshEdgeRef,
    edge: &EdgeRef,
) -> bool {
    let candidate_id = candidate
        .topology
        .as_ref()
        .and_then(|topology| topology.edge_id.as_deref());
    let requested_id = edge
        .topology
        .as_ref()
        .and_then(|topology| topology.edge_id.as_deref());
    // An `:occ:N` suffix is an *enumeration* disambiguator, not a durable design
    // name: when a boolean splits one design edge into fragments that share a
    // base id, which fragment gets which occurrence number depends on mesh
    // build order (and changes when the tessellation changes). So an occ id
    // must ALSO match geometrically — only unsuffixed sketch ids are trusted on
    // identity alone.
    matches!(
        (candidate_id, requested_id),
        (Some(candidate), Some(requested))
            if candidate == requested
                && requested.starts_with("sketch:")
                && !requested.contains(":occ:")
    )
}

pub(crate) fn edge_curves_are_compatible(
    candidate: Option<&EdgeCurveHint>,
    requested: Option<&EdgeCurveHint>,
) -> bool {
    matches!(
        (candidate, requested),
        (None, None)
            | (None, Some(EdgeCurveHint::Line))
            | (Some(EdgeCurveHint::Line), None)
            | (Some(EdgeCurveHint::Line), Some(EdgeCurveHint::Line))
            | (
                Some(EdgeCurveHint::Circle { .. }),
                Some(EdgeCurveHint::Circle { .. })
            )
    )
}

#[derive(Debug, Clone)]
pub(crate) struct EdgeModSelection {
    pub(crate) original_edge: EdgeRef,
    pub(crate) active_edge: EdgeRef,
}

impl EdgeModSelection {
    fn new(edge: &EdgeRef) -> Self {
        Self {
            original_edge: edge.clone(),
            active_edge: edge.clone(),
        }
    }
}

#[derive(Clone, Copy)]
pub(crate) struct CircularBiteLocality<'a> {
    pub(crate) region: &'a SketchExtrudeRegionSource,
    pub(crate) selection: &'a EdgeModSelection,
    pub(crate) dist: f32,
    pub(crate) kind: crate::sketch::CornerKind,
}

pub(crate) struct EdgeModResult {
    pub(crate) parts: Vec<KernelSolid>,
    pub(crate) pristine: Option<MockMesh>,
    pub(crate) cut_replay: Option<CutReplayHistory>,
}

impl EdgeModResult {
    fn single(part: KernelSolid) -> Self {
        Self {
            parts: vec![part],
            pristine: None,
            cut_replay: None,
        }
    }
}

enum ReplayAttempt {
    Applied(EdgeModResult),
    Failed(String),
    NotApplicable,
}

fn edge_mod_try_construction_replay(
    body: &LiveBody,
    selection: &EdgeModSelection,
    replay: &EdgeModReplayIntent,
    dist: f32,
) -> ReplayAttempt {
    let total_started = std::time::Instant::now();
    if matches!(replay.mode, EdgeModReplayMode::NativeOnly) || edge_mod_native_only(selection) {
        return ReplayAttempt::NotApplicable;
    }
    let has_replay_intent = replay.pre_cut_target.is_some()
        || !replay.replay_cut_nodes.is_empty()
        || replay.selected_span.is_some();
    if replay.pre_cut_target.is_none()
        && replay.replay_cut_nodes.is_empty()
        && replay.selected_span.is_none()
    {
        return ReplayAttempt::NotApplicable;
    }
    let Some(history) = body.cut_replay.as_ref() else {
        return ReplayAttempt::NotApplicable;
    };
    if history.base_parts.is_empty() || history.steps.is_empty() {
        return ReplayAttempt::NotApplicable;
    }
    if let Err(reason) = edge_mod_circular_bite_replay_runout_guard(body, history, selection, dist)
    {
        return ReplayAttempt::Failed(reason);
    }
    let replay_required = has_replay_intent;
    if let Some(target) = replay.pre_cut_target.as_deref() {
        if target != history.base_body_id && target != body.id {
            return ReplayAttempt::Failed(format!(
                "saved replay target '{target}' no longer matches body '{}'",
                body.id
            ));
        }
    }
    if !replay.replay_cut_nodes.is_empty()
        && !replay_nodes_are_ordered_subset(&history.steps, &replay.replay_cut_nodes)
    {
        return ReplayAttempt::Failed(
            "saved replay cut chain no longer matches the target body's cut history".to_string(),
        );
    }

    let mut failures = Vec::new();
    let mut filleted = false;
    let mut saw_split = false;
    for i in 0..history.base_parts.len() {
        let mut split_options = split_pre_cut_part_options(
            history,
            &history.base_parts[i],
            &selection.active_edge,
            dist,
        );
        if split_options.is_empty() {
            split_options.push(PreCutSplit {
                parts: vec![history.base_parts[i].clone()],
                edge: selection.active_edge.clone(),
                split_found: false,
            });
        } else {
            saw_split = true;
        }

        for split in split_options {
            let split_parts = split.parts;
            let Some(target_part) = split_parts.first() else {
                continue;
            };
            let reference = MockMesh::from_solid(target_part);
            let construction_selection = EdgeModSelection::new(&split.edge);
            match edge_mod_try_native_fillet(
                &reference,
                target_part,
                target_part,
                &construction_selection,
                dist,
                "construction replay native",
                &[],
                None,
            ) {
                Ok(part) => {
                    let mut replacement = split_parts;
                    replacement[0] = part;
                    let replacement = if split.split_found {
                        fuse_overlapping_solids(replacement)
                    } else {
                        replacement
                    };
                    filleted = true;
                    let mut candidate_base = history.base_parts.clone();
                    candidate_base.splice(i..=i, replacement);
                    match finish_construction_replay(body, selection, dist, history, candidate_base)
                    {
                        Ok(result) => {
                            edge_mod_timing("edge construction replay total", total_started);
                            return ReplayAttempt::Applied(result);
                        }
                        Err(reason) => failures.push(reason),
                    }
                }
                Err(reason) => failures.push(reason),
            }
        }
    }
    if !filleted {
        if !saw_split && !replay_required {
            return ReplayAttempt::NotApplicable;
        }
        return ReplayAttempt::Failed(if failures.is_empty() {
            "pre-cut selected edge could not be filleted after imprint".to_string()
        } else {
            format!(
                "pre-cut selected edge could not be filleted after imprint: {}",
                failures.join("; ")
            )
        });
    }
    ReplayAttempt::Failed(if failures.is_empty() {
        "construction replay produced no valid replayed body".to_string()
    } else {
        format!(
            "construction replay produced no valid replayed body: {}",
            failures.join("; ")
        )
    })
}

fn finish_construction_replay(
    body: &LiveBody,
    selection: &EdgeModSelection,
    dist: f32,
    history: &CutReplayHistory,
    base_parts: Vec<KernelSolid>,
) -> Result<EdgeModResult, String> {
    let replay_started = std::time::Instant::now();
    let replayed = match replay_cut_history(base_parts.clone(), &history.steps) {
        Ok(parts) => parts,
        Err(reason) => return Err(reason),
    };
    edge_mod_timing("construction replay cuts", replay_started);
    let validate_started = std::time::Instant::now();
    let (replayed, candidate_mesh) =
        match validate_replayed_edge_mod_body(body, selection, dist, history, &replayed) {
            Ok(mesh) => (replayed, mesh),
            Err(first_reason) if !body.cut_tools.is_empty() => {
                let recut = match recut_replayed_parts_with_tools(replayed, &body.cut_tools) {
                    Ok(parts) => parts,
                    Err(reason) => {
                        return Err(format!("{first_reason}; {reason}"));
                    }
                };
                match validate_replayed_edge_mod_body(body, selection, dist, history, &recut) {
                    Ok(mesh) => (recut, mesh),
                    Err(reason) => {
                        return Err(format!(
                            "{first_reason}; grown recut validation failed: {reason}"
                        ));
                    }
                }
            }
            Err(reason) => return Err(reason),
        };
    edge_mod_timing("construction replay validation", validate_started);

    let mut next_history = history.clone();
    next_history.base_parts = base_parts;
    next_history.base_pristine = None;
    Ok(EdgeModResult {
        parts: replayed,
        pristine: (!candidate_mesh.indices.is_empty()
            && history.base_pristine.is_some()
            && history.steps.is_empty())
        .then_some(candidate_mesh),
        cut_replay: Some(next_history),
    })
}

struct PreCutSplit {
    parts: Vec<KernelSolid>,
    edge: EdgeRef,
    split_found: bool,
}

fn split_pre_cut_part_options(
    history: &CutReplayHistory,
    part: &KernelSolid,
    edge: &EdgeRef,
    dist: f32,
) -> Vec<PreCutSplit> {
    let runout = 0.05_f32.min((dist * 0.05).max(0.0));
    let mut out = Vec::new();
    if let Some(source) = history.base_sketch_source.as_ref() {
        for region in &source.regions {
            if let Some(split) = imprinted_rect_base_for_edge(region, edge, runout) {
                out.push(split);
            }
            if let Some(split) = split_rect_base_parts_for_edge(region, edge, runout) {
                out.push(split);
            }
        }
    }
    out.extend(split_axis_aligned_box_for_edge(part, edge, runout));
    out
}

fn split_axis_aligned_box_for_edge(
    part: &KernelSolid,
    edge: &EdgeRef,
    runout: f32,
) -> Vec<PreCutSplit> {
    let mesh = MockMesh::from_solid(part);
    let Some((lo, hi)) = mesh_position_aabb(&mesh) else {
        return Vec::new();
    };
    if !mesh_is_aabb_box(&mesh, lo, hi, 0.08) {
        return Vec::new();
    }
    let dx = hi[0] - lo[0];
    let dy = hi[1] - lo[1];
    let dz = hi[2] - lo[2];
    let mut candidates = Vec::new();
    if dz.abs() > 1.0e-4 {
        candidates.push(SketchExtrudeRegionSource {
            boundary: vec![
                (lo[0], lo[1]),
                (hi[0], lo[1]),
                (hi[0], hi[1]),
                (lo[0], hi[1]),
            ],
            holes: Vec::new(),
            depth: dz,
            cs: CoordinateSystem::new(
                Vec3::new(lo[0], lo[1], lo[2]),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            ),
            rect_circle: None,
        });
    }
    if dx.abs() > 1.0e-4 {
        candidates.push(SketchExtrudeRegionSource {
            boundary: vec![
                (lo[1], lo[2]),
                (hi[1], lo[2]),
                (hi[1], hi[2]),
                (lo[1], hi[2]),
            ],
            holes: Vec::new(),
            depth: dx,
            cs: CoordinateSystem::new(
                Vec3::new(lo[0], lo[1], lo[2]),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            ),
            rect_circle: None,
        });
    }
    if dy.abs() > 1.0e-4 {
        candidates.push(SketchExtrudeRegionSource {
            boundary: vec![
                (lo[2], lo[0]),
                (hi[2], lo[0]),
                (hi[2], hi[0]),
                (lo[2], hi[0]),
            ],
            holes: Vec::new(),
            depth: dy,
            cs: CoordinateSystem::new(
                Vec3::new(lo[0], lo[1], lo[2]),
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 0.0),
            ),
            rect_circle: None,
        });
    }

    let mut out = Vec::new();
    for region in &candidates {
        if let Some(split) = imprinted_rect_base_for_edge(region, edge, runout) {
            out.push(split);
        }
        if let Some(split) = split_rect_base_parts_for_edge(region, edge, runout) {
            out.push(split);
        }
    }
    out
}

fn validate_replayed_edge_mod_body(
    body: &LiveBody,
    selection: &EdgeModSelection,
    dist: f32,
    history: &CutReplayHistory,
    parts: &[KernelSolid],
) -> Result<MockMesh, String> {
    let started = std::time::Instant::now();
    let reference_mesh = edge_mod_reference_mesh(body);
    let mut candidate_mesh = MockMesh::empty();
    for part in parts {
        candidate_mesh.append(MockMesh::from_solid(part));
    }
    edge_mod_timing("replayed body tessellation", started);
    if candidate_mesh.indices.is_empty() {
        return Err("replayed body tessellated to an empty mesh".to_string());
    }
    let cracks_started = std::time::Instant::now();
    edge_mod_render_mesh_has_no_cracks(&candidate_mesh)?;
    edge_mod_timing("replayed body crack check", cracks_started);

    let has_circular_bite_source = body.sketch_source.as_ref().is_some_and(|source| {
        source
            .regions
            .iter()
            .any(|region| region.rect_circle.is_some())
    });
    if !has_circular_bite_source {
        let inward = edge_mod_render_mesh_inward_triangles(&candidate_mesh);
        if inward > 0 {
            return Err(format!("replayed body has {inward} inward triangles"));
        }
    }

    for (part_index, part) in parts.iter().enumerate() {
        if let Some(original_part) = body.parts.get(part_index) {
            if !crate::mock_kernel::preserves_cylindrical_faces(original_part, part) {
                return Err("replayed candidate lost an analytic cylindrical face".to_string());
            }
        }
    }

    let bounds_started = std::time::Instant::now();
    edge_mod_mesh_stays_inside_reference_bounds(
        &reference_mesh,
        &candidate_mesh,
        EDGE_MOD_CONTAINMENT_TOL,
    )?;
    edge_mod_timing("replayed body bounds check", bounds_started);
    if !has_circular_bite_source {
        let ghost_started = std::time::Instant::now();
        let ghost_samples = replay_cut_void_ghost_sample_count(history, &candidate_mesh);
        edge_mod_timing("replayed cut void ghost check", ghost_started);
        if ghost_samples > 0 {
            return Err(format!(
                "replayed cuts left {ghost_samples} non-wall sample(s) inside removed cut volume"
            ));
        }
    }
    let locality_started = std::time::Instant::now();
    if let Some(source) = body.sketch_source.as_ref() {
        for region in &source.regions {
            if region.rect_circle.is_some() {
                edge_mod_circular_bite_locality_mesh(
                    CircularBiteLocality {
                        region,
                        selection,
                        dist,
                        kind: crate::sketch::CornerKind::Fillet,
                    },
                    &candidate_mesh,
                )?;
            }
        }
    }
    edge_mod_timing("replayed circular-bite locality check", locality_started);
    let blend_started = std::time::Instant::now();
    edge_mod_selected_blend_present(
        &candidate_mesh,
        &selection.active_edge,
        dist,
        crate::sketch::CornerKind::Fillet,
    )?;
    edge_mod_timing("replayed selected blend check", blend_started);
    let seams_started = std::time::Instant::now();
    let seams = edge_mod_selected_blend_lengthwise_wire_seams(
        &candidate_mesh,
        &selection.active_edge,
        dist,
    );
    edge_mod_timing("replayed selected seam check", seams_started);
    if seams > 0 {
        return Err(format!(
            "replayed fillet exposed {seams} lengthwise seam edge(s) on the selected surface"
        ));
    }
    Ok(candidate_mesh)
}

pub(crate) fn edge_mod_preflight(
    _body: &LiveBody,
    edge: &EdgeRef,
    dist: f32,
) -> Result<(), String> {
    if !dist.is_finite() || dist <= 0.0 {
        return Err("distance must be positive".to_string());
    }

    if edge_ref_local_clearance(edge) <= 1.0e-4 {
        return Err("selected edge is too short".to_string());
    }
    Ok(())
}

pub(crate) fn edge_ref_length(edge: &EdgeRef) -> f32 {
    let dx = edge.p1[0] - edge.p0[0];
    let dy = edge.p1[1] - edge.p0[1];
    let dz = edge.p1[2] - edge.p0[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

pub(crate) fn edge_ref_local_clearance(edge: &EdgeRef) -> f32 {
    match edge.curve {
        Some(EdgeCurveHint::Circle { radius, .. }) => radius.abs(),
        _ => edge_ref_length(edge),
    }
}

pub(crate) fn edge_mod_native_only(selection: &EdgeModSelection) -> bool {
    matches!(
        selection.active_edge.curve,
        Some(EdgeCurveHint::Circle { .. })
    )
}

/// Whether a plain native fillet/chamfer on the *current* (post-cut) body is
/// equivalent to reconstructing the edit through the saved cut history — so the
/// far cheaper native-on-final solve can be tried first.
///
/// The construction-/cut-history replay path exists so an edge that a later cut
/// *truncated* (a "cutoff" edge whose span is only part of a pre-cut base side)
/// gets filleted on the pre-cut geometry and then re-cut, which differs from
/// filleting the final body. `split_pre_cut_part_options` surfaces exactly that: a
/// split whose own edge matches the selected edge but yields more than one part
/// (the selected middle piece plus the untouched base remainder) means the
/// selected span is a strict subset of a base side, i.e. a later cut truncated it.
/// (It also over-generates unrelated candidate splits, so the split edge must
/// match the selection to count as evidence.) When no such truncation is found,
/// filleting before vs. after the cuts yields identical geometry and native-first
/// is safe.
///
/// Returns `true` in that case (and trivially when there is no cut history, or the
/// edge is a native-only curved rim). The acceptance gates remain the correctness
/// backstop; this gate only decides *ordering*.
fn edge_mod_native_first_safe(body: &LiveBody, selection: &EdgeModSelection, dist: f32) -> bool {
    if edge_mod_native_only(selection) {
        // A curved rim/arc edge never routes through cut-history replay.
        return true;
    }
    let Some(history) = body.cut_replay.as_ref() else {
        // No replayable cut history: the replay paths are NotApplicable and the
        // native solve already runs unconditionally, so ordering cannot matter.
        return true;
    };
    if history.base_parts.is_empty() || history.steps.is_empty() {
        return true;
    }
    let edge = &selection.active_edge;
    // The split may extend the span by a small runout at each end, so match the
    // endpoints loosely (order-independent).
    let matches_selection = |candidate: &EdgeRef| {
        let close = |p: [f32; 3], q: [f32; 3]| {
            (p[0] - q[0]).abs() < 0.2 && (p[1] - q[1]).abs() < 0.2 && (p[2] - q[2]).abs() < 0.2
        };
        (close(candidate.p0, edge.p0) && close(candidate.p1, edge.p1))
            || (close(candidate.p0, edge.p1) && close(candidate.p1, edge.p0))
    };
    for base_part in &history.base_parts {
        for split in split_pre_cut_part_options(history, base_part, edge, dist) {
            if split.parts.len() > 1 && matches_selection(&split.edge) {
                // The selected span is a strict subset of a pre-cut base side — a
                // later cut truncated the edge here, so replay is authoritative.
                return false;
            }
        }
    }
    true
}

/// The result of a per-part native edge modification over a body's parts.
struct NativeEdgeModOutcome {
    parts: Vec<KernelSolid>,
    /// Present only when every applied part contributed a pristine display mesh.
    pristine: Option<MockMesh>,
    /// At least one part was successfully modified.
    applied: bool,
    /// Combined failure reason of the last part that could not be modified.
    last_err: Option<String>,
}

/// Native rolling-ball fillet of the captured edge on every part in `parts`.
/// Parts that cannot be filleted are returned unchanged; the caller decides
/// whether a partial result is acceptable or a replay path should run instead.
fn edge_mod_native_fillet_all_parts(
    mod_id: &str,
    selection: &EdgeModSelection,
    dist: f32,
    parts: Vec<KernelSolid>,
    reference_mesh: &MockMesh,
    sketch_source: &Option<SketchExtrudeSource>,
    recut_tools: &[CutTool],
) -> NativeEdgeModOutcome {
    let mut applied = false;
    let mut last_err: Option<String> = None;
    let mut next: Vec<KernelSolid> = Vec::with_capacity(parts.len());
    let mut next_pristine = MockMesh::empty();
    let mut can_use_pristine = true;
    let native_only = edge_mod_native_only(selection);
    for (part_index, part) in parts.into_iter().enumerate() {
        let mut part_failures = Vec::new();
        // No pre-size gate: the kernel's rolling-ball blend rejects a radius too
        // large for the local geometry (a non-watertight result → `Err`), which is
        // the correct, geometry-aware bound. The old global-AABB heuristic was both
        // wrong (it measured the part's *thinnest* axis, not the filleted edge's
        // adjacent-face extents, so it blocked radii the kernel handles) and
        // asymmetric — chamfer never had it, which is why a radius would chamfer
        // but refuse to fillet.
        let sketch_region = sketch_source
            .as_ref()
            .and_then(|source| source.regions.get(part_index));
        let circular_bite_locality = sketch_region.map(|region| CircularBiteLocality {
            region,
            selection,
            dist,
            kind: crate::sketch::CornerKind::Fillet,
        });
        let mut accepted: Option<EdgeModResult> = None;
        match edge_mod_try_native_fillet(
            reference_mesh,
            &part,
            &part,
            selection,
            dist,
            "native",
            recut_tools,
            circular_bite_locality,
        ) {
            Ok(f) => accepted = Some(EdgeModResult::single(f)),
            Err(reason) => part_failures.push(reason),
        }

        let alternate_parts = if accepted.is_none() && !native_only {
            sketch_source
                .as_ref()
                .map(|source| sketch_source_alternate_parts(source, part_index))
                .unwrap_or_default()
        } else {
            Vec::new()
        };

        if accepted.is_none() && !native_only {
            for (label, alternate_part) in &alternate_parts {
                match edge_mod_try_native_fillet(
                    reference_mesh,
                    &part,
                    alternate_part,
                    selection,
                    dist,
                    &format!("{label} native"),
                    recut_tools,
                    circular_bite_locality,
                ) {
                    Ok(f) => {
                        // The real part's native fillet failed; this records which
                        // canonical fallback (e.g. "box-cylinder sketch") actually
                        // carried the edit, so a bite-vs-box regression is traceable.
                        log::debug!(
                            "Fillet '{mod_id}' part {part_index}: native fillet failed, \
                             accepted '{label}' alternate part"
                        );
                        accepted = Some(EdgeModResult::single(f));
                        break;
                    }
                    Err(reason) => {
                        part_failures.push(reason);
                    }
                }
            }
        }

        if let Some(result) = accepted {
            applied = true;
            if let Some(mesh) = result.pristine {
                next_pristine.append(mesh);
            } else {
                can_use_pristine = false;
            }
            next.extend(result.parts);
        } else {
            can_use_pristine = false;
            if !part_failures.is_empty() {
                last_err = Some(part_failures.join("; "));
            }
            next.push(part);
        }
    }
    let pristine = (can_use_pristine && !next_pristine.indices.is_empty()).then_some(next_pristine);
    NativeEdgeModOutcome {
        parts: next,
        pristine,
        applied,
        last_err,
    }
}

/// Native selected-edge chamfer of the captured edge on every part in `parts`.
/// Mirrors [`edge_mod_native_fillet_all_parts`].
fn edge_mod_native_chamfer_all_parts(
    selection: &EdgeModSelection,
    dist: f32,
    parts: Vec<KernelSolid>,
    reference_mesh: &MockMesh,
    sketch_source: &Option<SketchExtrudeSource>,
    recut_tools: &[CutTool],
) -> NativeEdgeModOutcome {
    let edge = &selection.active_edge;
    let mut applied = false;
    let mut last_err: Option<String> = None;
    let mut next: Vec<KernelSolid> = Vec::with_capacity(parts.len());
    let mut next_pristine = MockMesh::empty();
    let mut can_use_pristine = true;
    let native_only = edge_mod_native_only(selection);
    for (part_index, part) in parts.into_iter().enumerate() {
        let sketch_region = sketch_source
            .as_ref()
            .and_then(|source| source.regions.get(part_index));
        let circular_bite_locality = sketch_region.map(|region| CircularBiteLocality {
            region,
            selection,
            dist,
            kind: crate::sketch::CornerKind::Chamfer,
        });
        let mut accepted: Option<EdgeModResult> = None;
        let mut part_failures = Vec::new();
        match crate::mock_kernel::chamfer_edge(&part, edge.p0, edge.p1, dist) {
            Ok(chamfered) => match edge_mod_accept_candidate_or_recut(
                reference_mesh,
                &part,
                chamfered,
                recut_tools,
                circular_bite_locality,
            ) {
                Ok(chamfered) => {
                    match edge_mod_reject_unhealthy_native_curve_result(selection, &chamfered) {
                        Ok(()) => accepted = Some(EdgeModResult::single(chamfered)),
                        Err(reason) => {
                            part_failures.push(format!("native result rejected: {reason}"))
                        }
                    }
                }
                Err(reason) => part_failures.push(format!("native result rejected: {reason}")),
            },
            Err(reason) => part_failures.push(format!("native failed: {reason}")),
        }

        let alternate_parts = if accepted.is_none() && !native_only {
            sketch_source
                .as_ref()
                .map(|source| sketch_source_alternate_parts(source, part_index))
                .unwrap_or_default()
        } else {
            Vec::new()
        };

        if accepted.is_none() && !native_only {
            for (label, alternate_part) in &alternate_parts {
                match crate::mock_kernel::chamfer_edge(alternate_part, edge.p0, edge.p1, dist) {
                    Ok(chamfered) => {
                        match edge_mod_accept_candidate_or_recut(
                            reference_mesh,
                            &part,
                            chamfered,
                            recut_tools,
                            circular_bite_locality,
                        ) {
                            Ok(chamfered) => {
                                accepted = Some(EdgeModResult::single(chamfered));
                                break;
                            }
                            Err(reason) => part_failures
                                .push(format!("{label} native result rejected: {reason}")),
                        }
                    }
                    Err(reason) => {
                        part_failures.push(format!("{label} native failed: {reason}"));
                    }
                }
            }
        }

        if let Some(result) = accepted {
            applied = true;
            if let Some(mesh) = result.pristine {
                next_pristine.append(mesh);
            } else {
                can_use_pristine = false;
            }
            next.extend(result.parts);
        } else {
            can_use_pristine = false;
            if !part_failures.is_empty() {
                last_err = Some(part_failures.join("; "));
            }
            next.push(part);
        }
    }
    let pristine = (can_use_pristine && !next_pristine.indices.is_empty()).then_some(next_pristine);
    NativeEdgeModOutcome {
        parts: next,
        pristine,
        applied,
        last_err,
    }
}

/// Native rolling-ball fillet of the captured edge on every part of `body`.
pub(crate) fn apply_fillet(
    mod_id: &str,
    selection: &EdgeModSelection,
    replay: &EdgeModReplayIntent,
    dist: f32,
    body: &mut LiveBody,
    warnings: &mut Vec<String>,
) {
    // Native-first: a plain per-part native fillet on the current body is far
    // cheaper than construction-/cut-history replay. When the selected edge stays
    // clear of every replayable cut void (so filleting before vs. after those cuts
    // agree), try it first and take it only if it cleanly handles every part.
    if edge_mod_native_first_safe(body, selection, dist) {
        let reference_mesh = edge_mod_reference_mesh(body);
        let sketch_source = body.sketch_source.clone();
        let recut_tools = body.cut_tools.clone();
        let outcome = edge_mod_native_fillet_all_parts(
            mod_id,
            selection,
            dist,
            body.parts.clone(),
            &reference_mesh,
            &sketch_source,
            &recut_tools,
        );
        if outcome.applied && outcome.last_err.is_none() {
            body.parts = outcome.parts;
            body.pristine = outcome.pristine;
            body.sketch_source = None;
            body.cut_replay = None;
            body.edge_mod_cut_history_path_used = false;
            return;
        }
    }

    let prefer_cut_history = edge_mod_has_replayable_cut_history(body, selection, replay);
    let mut cut_history_path_used = false;
    let mut replay_failure: Option<String> = None;

    if prefer_cut_history {
        cut_history_path_used = true;
        match edge_mod_try_construction_replay(body, selection, replay, dist) {
            ReplayAttempt::Applied(result) => {
                body.parts = result.parts;
                body.pristine = result.pristine;
                body.sketch_source = None;
                body.cut_replay = result.cut_replay;
                body.edge_mod_cut_history_path_used = true;
                return;
            }
            ReplayAttempt::Failed(reason) => replay_failure = Some(reason),
            ReplayAttempt::NotApplicable => {}
        }

        match edge_mod_try_native_cut_history_replay(body, selection, replay, dist) {
            ReplayAttempt::Applied(result) => {
                body.parts = result.parts;
                body.pristine = result.pristine;
                body.sketch_source = None;
                body.cut_replay = result.cut_replay;
                body.edge_mod_cut_history_path_used = true;
                return;
            }
            ReplayAttempt::Failed(reason) => {
                replay_failure = Some(match replay_failure {
                    Some(prefix_reason) => {
                        format!("{prefix_reason}; native cut-history replay failed ({reason})")
                    }
                    None => reason,
                });
            }
            ReplayAttempt::NotApplicable => {}
        }
    } else {
        match edge_mod_try_construction_replay(body, selection, replay, dist) {
            ReplayAttempt::Applied(result) => {
                body.parts = result.parts;
                body.pristine = result.pristine;
                body.sketch_source = None;
                body.cut_replay = result.cut_replay;
                body.edge_mod_cut_history_path_used = true;
                return;
            }
            ReplayAttempt::Failed(reason) => {
                cut_history_path_used = true;
                replay_failure = Some(match replay_failure {
                    Some(prefix_reason) => {
                        format!("{prefix_reason}; construction replay failed ({reason})")
                    }
                    None => reason,
                });
            }
            ReplayAttempt::NotApplicable => {}
        }
    }

    // Final fallback: native per-part solve on the untouched body (accepts a
    // partial result). Reached when native-first was skipped (a cutoff edge) or
    // declined, and every replay path was NotApplicable or Failed.
    let reference_mesh = edge_mod_reference_mesh(body);
    let sketch_source = body.sketch_source.clone();
    let recut_tools = body.cut_tools.clone();
    let outcome = edge_mod_native_fillet_all_parts(
        mod_id,
        selection,
        dist,
        std::mem::take(&mut body.parts),
        &reference_mesh,
        &sketch_source,
        &recut_tools,
    );
    body.parts = outcome.parts;
    if outcome.applied {
        body.pristine = outcome.pristine;
        body.sketch_source = None;
        body.cut_replay = None;
        body.edge_mod_cut_history_path_used = cut_history_path_used;
    } else {
        // Surface the kernel's actual reason (radius too large, edge not found on
        // an adjacent face, non-blendable wedge, …) instead of a generic guess.
        let native_reason = outcome
            .last_err
            .unwrap_or_else(|| "the edge is no longer on the body".to_string());
        let reason = replay_failure
            .map(|replay| {
                format!(
                    "construction replay failed ({replay}); native solve failed ({native_reason})"
                )
            })
            .unwrap_or(native_reason);
        warnings.push(format!(
            "Fillet '{mod_id}': the edge couldn't be rounded ({reason}), so the \
             body was left unchanged."
        ));
    }
}

fn edge_mod_try_native_cut_history_replay(
    body: &LiveBody,
    selection: &EdgeModSelection,
    replay: &EdgeModReplayIntent,
    dist: f32,
) -> ReplayAttempt {
    let total_started = std::time::Instant::now();
    if matches!(replay.mode, EdgeModReplayMode::NativeOnly) || edge_mod_native_only(selection) {
        return ReplayAttempt::NotApplicable;
    }
    let Some(history) = body.cut_replay.as_ref() else {
        return ReplayAttempt::NotApplicable;
    };
    if history.base_parts.is_empty() || history.steps.is_empty() {
        return ReplayAttempt::NotApplicable;
    }
    if let Err(reason) = edge_mod_circular_bite_replay_runout_guard(body, history, selection, dist)
    {
        return ReplayAttempt::Failed(reason);
    }
    if let Some(target) = replay.pre_cut_target.as_deref() {
        if target != history.base_body_id && target != body.id {
            return ReplayAttempt::Failed(format!(
                "saved replay target '{target}' no longer matches body '{}'",
                body.id
            ));
        }
    }
    if !replay.replay_cut_nodes.is_empty()
        && !replay_nodes_are_ordered_subset(&history.steps, &replay.replay_cut_nodes)
    {
        return ReplayAttempt::Failed(
            "saved replay cut chain no longer matches the target body's cut history".to_string(),
        );
    }

    let mut failures = Vec::new();
    // Incremental prefix replay: apply one cut step at a time to a running clean
    // prefix instead of re-replaying `steps[..prefix_len]` from scratch on every
    // iteration (O(N²) → O(N) booleans). Each fillet attempt mutates a *clone*, so
    // `clean_prefix_parts` stays fillet-free for the next longer prefix.
    let mut clean_prefix_parts = history.base_parts.clone();
    for prefix_len in 1..=history.steps.len() {
        recut_debug(format!(
            "trying cut-history prefix {prefix_len}/{} for selected edge",
            history.steps.len()
        ));
        let replay_started = std::time::Instant::now();
        clean_prefix_parts = match replay_cut_history(
            clean_prefix_parts,
            &history.steps[prefix_len - 1..prefix_len],
        ) {
            Ok(parts) => parts,
            Err(reason) => {
                // Once one step can't replay, no longer prefix can either (each is a
                // superset of this same chain), so stop rather than retry from scratch.
                failures.push(format!("prefix {prefix_len} replay failed: {reason}"));
                break;
            }
        };
        edge_mod_timing(
            format!("native cut-history prefix {prefix_len} replay"),
            replay_started,
        );
        let mut prefix_parts = clean_prefix_parts.clone();
        let mut reference_mesh = MockMesh::empty();
        for part in &prefix_parts {
            reference_mesh.append(MockMesh::from_solid(part));
        }
        if reference_mesh.indices.is_empty() {
            failures.push(format!("prefix {prefix_len} tessellated to an empty mesh"));
            continue;
        }

        let mut applied = false;
        let mut prefix_failures = Vec::new();
        let native_started = std::time::Instant::now();
        for part in &mut prefix_parts {
            let original = part.clone();
            match edge_mod_try_native_fillet(
                &reference_mesh,
                &original,
                &original,
                selection,
                dist,
                "cut-history native",
                &[],
                None,
            ) {
                Ok(filleted) => {
                    *part = filleted;
                    applied = true;
                }
                Err(reason) => prefix_failures.push(reason),
            }
        }
        edge_mod_timing(
            format!("native cut-history prefix {prefix_len} fillet"),
            native_started,
        );
        if !applied {
            failures.push(if prefix_failures.is_empty() {
                format!("prefix {prefix_len} native solve did not find the selected edge")
            } else {
                format!("prefix {prefix_len}: {}", prefix_failures.join("; "))
            });
            continue;
        }

        let modified_prefix_parts = prefix_parts;
        let suffix_started = std::time::Instant::now();
        let replayed =
            match replay_cut_history(modified_prefix_parts.clone(), &history.steps[prefix_len..]) {
                Ok(parts) => parts,
                Err(reason) => {
                    failures.push(format!(
                        "prefix {prefix_len} suffix replay failed: {reason}"
                    ));
                    continue;
                }
            };
        edge_mod_timing(
            format!("native cut-history prefix {prefix_len} suffix replay"),
            suffix_started,
        );
        let validate_started = std::time::Instant::now();
        match validate_replayed_edge_mod_body(body, selection, dist, history, &replayed) {
            Ok(_) => {
                edge_mod_timing(
                    format!("native cut-history prefix {prefix_len} validation"),
                    validate_started,
                );
                edge_mod_timing("native cut-history replay total", total_started);
                let cut_replay = if prefix_len < history.steps.len() {
                    Some(CutReplayHistory {
                        base_body_id: history.base_body_id.clone(),
                        base_parts: modified_prefix_parts,
                        base_pristine: None,
                        base_sketch_source: None,
                        steps: history.steps[prefix_len..].to_vec(),
                    })
                } else {
                    None
                };
                return ReplayAttempt::Applied(EdgeModResult {
                    parts: replayed,
                    pristine: None,
                    cut_replay,
                });
            }
            Err(reason) => {
                edge_mod_timing(
                    format!("native cut-history prefix {prefix_len} validation"),
                    validate_started,
                );
                recut_debug(format!(
                    "prefix {prefix_len} validation failed after replay: {reason}"
                ));
                failures.push(format!("prefix {prefix_len} validation failed: {reason}"));
            }
        }
    }
    ReplayAttempt::Failed(if failures.is_empty() {
        "native solve on replayed cut-history prefixes did not find the selected edge".to_string()
    } else {
        failures.join("; ")
    })
}

fn edge_mod_has_replayable_cut_history(
    body: &LiveBody,
    selection: &EdgeModSelection,
    replay: &EdgeModReplayIntent,
) -> bool {
    if matches!(replay.mode, EdgeModReplayMode::NativeOnly) || edge_mod_native_only(selection) {
        return false;
    }
    let has_replay_intent = replay.pre_cut_target.is_some()
        || !replay.replay_cut_nodes.is_empty()
        || replay.selected_span.is_some();
    has_replay_intent
        && body
            .cut_replay
            .as_ref()
            .is_some_and(|history| history.steps.iter().any(|step| step.tool.has_any_solid()))
}

pub(crate) fn edge_mod_try_native_fillet(
    reference_mesh: &MockMesh,
    original_part: &KernelSolid,
    fillet_part: &KernelSolid,
    selection: &EdgeModSelection,
    dist: f32,
    label: &str,
    recut_tools: &[CutTool],
    circular_bite_locality: Option<CircularBiteLocality<'_>>,
) -> Result<KernelSolid, String> {
    let edge = &selection.active_edge;
    let mut failures = Vec::new();
    for (suffix, p0, p1) in [("", edge.p0, edge.p1), (" reversed", edge.p1, edge.p0)] {
        let started = std::time::Instant::now();
        match crate::mock_kernel::fillet_edge_with_hint(
            fillet_part,
            p0,
            p1,
            edge.curve.as_ref(),
            dist,
        ) {
            Ok(f) => match edge_mod_accept_candidate_or_recut(
                reference_mesh,
                original_part,
                f,
                recut_tools,
                circular_bite_locality,
            ) {
                Ok(f) => match edge_mod_reject_unhealthy_native_curve_result(selection, &f) {
                    Ok(()) => return Ok(f),
                    Err(reason) => {
                        failures.push(format!("{label}{suffix} result rejected: {reason}"))
                    }
                },
                Err(reason) => failures.push(format!("{label}{suffix} result rejected: {reason}")),
            },
            Err(reason) => failures.push(format!("{label}{suffix} failed: {reason}")),
        }
        edge_mod_timing(format!("native fillet {label}{suffix}"), started);
    }
    Err(failures.join("; "))
}

/// Native selected-edge chamfer of the captured edge on every part of `body`.
pub(crate) fn apply_chamfer(
    mod_id: &str,
    selection: &EdgeModSelection,
    dist: f32,
    body: &mut LiveBody,
    warnings: &mut Vec<String>,
) {
    // Chamfer has no construction-/cut-history replay path (it is native-only), so
    // this is already the equivalent of the fillet native-first fast path.
    let reference_mesh = edge_mod_reference_mesh(body);
    let sketch_source = body.sketch_source.clone();
    let recut_tools = body.cut_tools.clone();
    let outcome = edge_mod_native_chamfer_all_parts(
        selection,
        dist,
        std::mem::take(&mut body.parts),
        &reference_mesh,
        &sketch_source,
        &recut_tools,
    );
    body.parts = outcome.parts;
    if outcome.applied {
        body.pristine = outcome.pristine;
        body.sketch_source = None;
        body.cut_replay = None;
        body.edge_mod_cut_history_path_used = false;
    } else {
        let reason = outcome
            .last_err
            .unwrap_or_else(|| "the edge is no longer on the body".to_string());
        warnings.push(format!(
            "Chamfer '{mod_id}': the edge couldn't be beveled ({reason}), so the \
             body was left unchanged."
        ));
    }
}

pub(crate) fn split_rect_base_for_edge(
    region: &SketchExtrudeRegionSource,
    edge: &EdgeRef,
) -> Option<KernelSolid> {
    let ((min_x, min_y), (max_x, max_y)) = loop_bounds_2d(&region.boundary)?;
    let p0 = region
        .cs
        .project(Vec3::new(edge.p0[0], edge.p0[1], edge.p0[2]));
    let p1 = region
        .cs
        .project(Vec3::new(edge.p1[0], edge.p1[1], edge.p1[2]));
    let side_eps = 0.12;
    let push_unique = |out: &mut Vec<(f32, f32)>, p: (f32, f32)| {
        if out
            .last()
            .is_none_or(|q| (q.0 - p.0).hypot(q.1 - p.1) > 1.0e-4)
        {
            out.push(p);
        }
    };
    let mut profile = Vec::new();
    if (p0.1 - min_y).abs() <= side_eps && (p1.1 - min_y).abs() <= side_eps {
        let mut split = [p0, p1];
        split.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        for p in [
            (min_x, min_y),
            split[0],
            split[1],
            (max_x, min_y),
            (max_x, max_y),
            (min_x, max_y),
        ] {
            push_unique(&mut profile, p);
        }
    } else if (p0.0 - max_x).abs() <= side_eps && (p1.0 - max_x).abs() <= side_eps {
        let mut split = [p0, p1];
        split.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        for p in [
            (min_x, min_y),
            (max_x, min_y),
            split[0],
            split[1],
            (max_x, max_y),
            (min_x, max_y),
        ] {
            push_unique(&mut profile, p);
        }
    } else if (p0.1 - max_y).abs() <= side_eps && (p1.1 - max_y).abs() <= side_eps {
        let mut split = [p0, p1];
        split.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
        for p in [
            (min_x, min_y),
            (max_x, min_y),
            (max_x, max_y),
            split[0],
            split[1],
            (min_x, max_y),
        ] {
            push_unique(&mut profile, p);
        }
    } else if (p0.0 - min_x).abs() <= side_eps && (p1.0 - min_x).abs() <= side_eps {
        let mut split = [p0, p1];
        split.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        for p in [
            (min_x, min_y),
            (max_x, min_y),
            (max_x, max_y),
            (min_x, max_y),
            split[0],
            split[1],
        ] {
            push_unique(&mut profile, p);
        }
    } else {
        return None;
    }
    if profile.len() >= 2 {
        let first = profile[0];
        if profile
            .last()
            .is_some_and(|last| (last.0 - first.0).hypot(last.1 - first.1) <= 1.0e-4)
        {
            profile.pop();
        }
    }
    crate::mock_kernel::extruded_region_solid(&profile, &[], region.depth, &region.cs)
}

fn split_rect_base_parts_for_edge(
    region: &SketchExtrudeRegionSource,
    edge: &EdgeRef,
    runout: f32,
) -> Option<PreCutSplit> {
    let ((min_x, min_y), (max_x, max_y)) = loop_bounds_2d(&region.boundary)?;
    let p0 = region
        .cs
        .project(Vec3::new(edge.p0[0], edge.p0[1], edge.p0[2]));
    let p1 = region
        .cs
        .project(Vec3::new(edge.p1[0], edge.p1[1], edge.p1[2]));
    let edge_depth = {
        let depth_at = |world: [f32; 3], local: (f32, f32)| {
            let on_plane = region.cs.unproject(local.0, local.1);
            Vec3::new(world[0], world[1], world[2])
                .sub(on_plane)
                .dot(region.cs.n)
        };
        (depth_at(edge.p0, p0) + depth_at(edge.p1, p1)) * 0.5
    };
    let side_eps = 0.12;
    let min_span = 1.0e-3;
    let push_piece = |parts: &mut Vec<KernelSolid>, x0: f32, y0: f32, x1: f32, y1: f32| {
        if x1 <= x0 + min_span || y1 <= y0 + min_span {
            return;
        }
        let boundary = vec![(x0, y0), (x1, y0), (x1, y1), (x0, y1)];
        if let Some(part) =
            crate::mock_kernel::extruded_region_solid(&boundary, &[], region.depth, &region.cs)
        {
            parts.push(part);
        }
    };
    let make_edge = |along0: f32, along1: f32, fixed: f32, along_x: bool| {
        let local_p0 = if along_x {
            (along0, fixed)
        } else {
            (fixed, along0)
        };
        let local_p1 = if along_x {
            (along1, fixed)
        } else {
            (fixed, along1)
        };
        let world_at = |local: (f32, f32)| {
            let p = region
                .cs
                .unproject(local.0, local.1)
                .add(region.cs.n.mul(edge_depth));
            [p.x, p.y, p.z]
        };
        let mut construction = edge.clone();
        construction.p0 = world_at(local_p0);
        construction.p1 = world_at(local_p1);
        construction.curve = None;
        construction.topology = None;
        construction
    };
    let interval_parts = |lo: f32,
                          hi: f32,
                          min_a: f32,
                          max_a: f32,
                          min_b: f32,
                          max_b: f32,
                          fixed: f32,
                          along_x: bool| {
        let selected_lo = lo.min(hi);
        let selected_hi = lo.max(hi);
        let lo = (selected_lo - runout).max(min_a).min(max_a);
        let hi = (selected_hi + runout).max(min_a).min(max_a);
        if hi <= lo + min_span {
            return None;
        }
        let mut parts = Vec::new();
        if along_x {
            push_piece(&mut parts, lo, min_b, hi, max_b);
            push_piece(&mut parts, min_a, min_b, lo, max_b);
            push_piece(&mut parts, hi, min_b, max_a, max_b);
        } else {
            push_piece(&mut parts, min_b, lo, max_b, hi);
            push_piece(&mut parts, min_b, min_a, max_b, lo);
            push_piece(&mut parts, min_b, hi, max_b, max_a);
        }
        if parts.is_empty() {
            return None;
        }
        let edge_lo = if p0.0 <= p1.0 || p0.1 <= p1.1 { lo } else { hi };
        let edge_hi = if p0.0 <= p1.0 || p0.1 <= p1.1 { hi } else { lo };
        Some(PreCutSplit {
            parts,
            edge: make_edge(edge_lo, edge_hi, fixed, along_x),
            split_found: true,
        })
    };

    if (p0.1 - min_y).abs() <= side_eps && (p1.1 - min_y).abs() <= side_eps {
        let lo = p0.0.min(p1.0);
        let hi = p0.0.max(p1.0);
        interval_parts(lo, hi, min_x, max_x, min_y, max_y, min_y, true)
    } else if (p0.1 - max_y).abs() <= side_eps && (p1.1 - max_y).abs() <= side_eps {
        let lo = p0.0.min(p1.0);
        let hi = p0.0.max(p1.0);
        interval_parts(lo, hi, min_x, max_x, min_y, max_y, max_y, true)
    } else if (p0.0 - min_x).abs() <= side_eps && (p1.0 - min_x).abs() <= side_eps {
        let lo = p0.1.min(p1.1);
        let hi = p0.1.max(p1.1);
        interval_parts(lo, hi, min_y, max_y, min_x, max_x, min_x, false)
    } else if (p0.0 - max_x).abs() <= side_eps && (p1.0 - max_x).abs() <= side_eps {
        let lo = p0.1.min(p1.1);
        let hi = p0.1.max(p1.1);
        interval_parts(lo, hi, min_y, max_y, min_x, max_x, max_x, false)
    } else {
        None
    }
}

fn imprinted_rect_base_for_edge(
    region: &SketchExtrudeRegionSource,
    edge: &EdgeRef,
    runout: f32,
) -> Option<PreCutSplit> {
    let split = split_rect_base_parts_for_edge(region, edge, runout)?;
    let solid = split_rect_base_for_edge(region, &split.edge)?;
    Some(PreCutSplit {
        parts: vec![solid],
        edge: split.edge,
        split_found: false,
    })
}

pub(crate) fn loop_bounds_2d(points: &[(f32, f32)]) -> Option<((f32, f32), (f32, f32))> {
    let mut min_x = f32::INFINITY;
    let mut min_y = f32::INFINITY;
    let mut max_x = f32::NEG_INFINITY;
    let mut max_y = f32::NEG_INFINITY;
    let mut any = false;
    for &(x, y) in points {
        any = true;
        min_x = min_x.min(x);
        min_y = min_y.min(y);
        max_x = max_x.max(x);
        max_y = max_y.max(y);
    }
    any.then_some(((min_x, min_y), (max_x, max_y)))
}

#[allow(dead_code)]
pub(crate) fn edge_mod_fallback_cut(
    part: &KernelSolid,
    edge: &EdgeRef,
    dist: f32,
    kind: crate::sketch::CornerKind,
    reference_mesh: &MockMesh,
    alternate_parts: &[(&'static str, KernelSolid)],
) -> Result<KernelSolid, String> {
    let mut failures = Vec::new();
    match edge_mod_fallback_cut_against_part(part, part, edge, dist, kind, reference_mesh, "") {
        Ok(result) => return Ok(result),
        Err(reason) => failures.push(reason),
    }
    for (label, alternate_part) in alternate_parts {
        let prefix = format!("{label} ");
        match edge_mod_fallback_cut_against_part(
            alternate_part,
            part,
            edge,
            dist,
            kind,
            reference_mesh,
            &prefix,
        ) {
            Ok(result) => return Ok(result),
            Err(reason) => failures.push(reason),
        }
    }

    Err(if failures.is_empty() {
        "no fallback candidate was produced".to_string()
    } else {
        failures.join("; ")
    })
}

#[allow(dead_code)]
pub(crate) fn edge_mod_fallback_cut_against_part(
    cut_part: &KernelSolid,
    original_part: &KernelSolid,
    edge: &EdgeRef,
    dist: f32,
    kind: crate::sketch::CornerKind,
    reference_mesh: &MockMesh,
    label_prefix: &str,
) -> Result<KernelSolid, String> {
    edge_mod_fallback_cut_candidates_against_part(
        cut_part,
        original_part,
        edge,
        dist,
        kind,
        reference_mesh,
        label_prefix,
        |_label, candidate, _failures| Some(candidate),
    )
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn edge_mod_fallback_cut_candidates_against_part<T>(
    cut_part: &KernelSolid,
    original_part: &KernelSolid,
    edge: &EdgeRef,
    dist: f32,
    kind: crate::sketch::CornerKind,
    reference_mesh: &MockMesh,
    label_prefix: &str,
    mut try_candidate: impl FnMut(&str, KernelSolid, &mut Vec<String>) -> Option<T>,
) -> Result<T, String> {
    let fillet = matches!(kind, crate::sketch::CornerKind::Fillet);
    let robust_overshoot = EDGE_MOD_END_OVERSHOOT;
    let mut failures = Vec::new();
    for (orientation, cutter_edge) in [
        ("", edge.clone()),
        (" reversed", reversed_edge_ref_for_cutter(edge)),
    ] {
        for (label, grow, end_overshoot) in [
            ("exact cutter", 0.0, 0.0),
            ("grown cutter", EDGE_MOD_GROW, 0.0),
            ("overshot cutter", 0.0, robust_overshoot),
            ("robust cutter", EDGE_MOD_GROW, robust_overshoot),
        ] {
            let label = format!("{label_prefix}{label}{orientation}");
            let Some(cutter) = crate::mock_kernel::edge_corner_cutter(
                cutter_edge.p0,
                cutter_edge.p1,
                cutter_edge.n1,
                cutter_edge.n2,
                dist,
                fillet,
                EDGE_FILLET_SEGS,
                grow,
                end_overshoot,
            ) else {
                failures.push(format!("{label} could not be built"));
                continue;
            };
            let Some(result) = crate::mock_kernel::difference(cut_part, &cutter) else {
                failures.push(format!("{label} boolean failed"));
                continue;
            };
            match edge_mod_accept_candidate(reference_mesh, original_part, result) {
                Ok(result) => {
                    if let Some(accepted) = try_candidate(&label, result, &mut failures) {
                        return Ok(accepted);
                    }
                }
                Err(reason) => failures.push(format!("{label} rejected: {reason}")),
            }
        }

        if fillet {
            for (label, grow, end_overshoot) in [
                ("piecewise exact cutter", 0.0, 0.0),
                ("piecewise grown cutter", EDGE_MOD_GROW, 0.0),
                ("piecewise robust cutter", EDGE_MOD_GROW, robust_overshoot),
            ] {
                let label = format!("{label_prefix}{label}{orientation}");
                let Some(pieces) = crate::mock_kernel::edge_corner_cutter_pieces(
                    cutter_edge.p0,
                    cutter_edge.p1,
                    cutter_edge.n1,
                    cutter_edge.n2,
                    dist,
                    true,
                    EDGE_FILLET_SEGS,
                    grow,
                    end_overshoot,
                ) else {
                    failures.push(format!("{label} could not be built"));
                    continue;
                };

                let mut result = cut_part.clone();
                let mut failed = None;
                for cutter in pieces {
                    match crate::mock_kernel::difference(&result, &cutter) {
                        Some(next) => result = next,
                        None => {
                            failed = Some(format!("{label} boolean failed"));
                            break;
                        }
                    }
                }
                if let Some(reason) = failed {
                    failures.push(reason);
                    continue;
                }

                match edge_mod_accept_candidate(reference_mesh, original_part, result) {
                    Ok(result) => {
                        if let Some(accepted) = try_candidate(&label, result, &mut failures) {
                            return Ok(accepted);
                        }
                    }
                    Err(reason) => failures.push(format!("{label} rejected: {reason}")),
                }
            }

            for trim in [0.05, EDGE_MOD_GROW, 0.5] {
                let label =
                    format!("{label_prefix}trimmed piecewise cutter {trim:.2}{orientation}");
                let Some(trimmed) = trimmed_edge_ref(&cutter_edge, trim) else {
                    failures.push(format!("{label} could not be built"));
                    continue;
                };
                let Some(pieces) = crate::mock_kernel::edge_corner_cutter_pieces(
                    trimmed.p0,
                    trimmed.p1,
                    trimmed.n1,
                    trimmed.n2,
                    dist,
                    true,
                    EDGE_FILLET_SEGS,
                    EDGE_MOD_GROW,
                    0.0,
                ) else {
                    failures.push(format!("{label} could not be built"));
                    continue;
                };

                let mut result = cut_part.clone();
                let mut failed = None;
                for cutter in pieces {
                    match crate::mock_kernel::difference(&result, &cutter) {
                        Some(next) => result = next,
                        None => {
                            failed = Some(format!("{label} boolean failed"));
                            break;
                        }
                    }
                }
                if let Some(reason) = failed {
                    failures.push(reason);
                    continue;
                }

                match edge_mod_accept_candidate(reference_mesh, original_part, result) {
                    Ok(result) => {
                        if let Some(accepted) = try_candidate(&label, result, &mut failures) {
                            return Ok(accepted);
                        }
                    }
                    Err(reason) => failures.push(format!("{label} rejected: {reason}")),
                }
            }
        }
    }

    Err(if failures.is_empty() {
        "no fallback candidate was produced".to_string()
    } else {
        failures.join("; ")
    })
}

pub(crate) fn reversed_edge_ref_for_cutter(edge: &EdgeRef) -> EdgeRef {
    let mut reversed = edge.clone();
    std::mem::swap(&mut reversed.p0, &mut reversed.p1);
    reversed
}

pub(crate) fn sketch_source_alternate_parts(
    source: &SketchExtrudeSource,
    part_index: usize,
) -> Vec<(&'static str, KernelSolid)> {
    let mut out = Vec::new();
    let Some(region) = source.regions.get(part_index) else {
        return out;
    };

    if let Some(canonical) = &region.rect_circle {
        if let Some(part) = canonical.body.clone() {
            out.push(("box-cylinder sketch", part));
        }
        return out;
    }

    if let Some(part) = crate::mock_kernel::rect_minus_circle_region_solid(
        &region.boundary,
        &region.holes,
        region.depth,
        &region.cs,
    ) {
        out.push(("box-cylinder sketch", part));
        return out;
    }

    if let Some(part) = crate::mock_kernel::extruded_region_faceted_solid(
        &region.boundary,
        &region.holes,
        region.depth,
        &region.cs,
    ) {
        out.push(("faceted sketch", part));
    }

    out
}

#[allow(dead_code)]
pub(crate) fn trimmed_edge_ref(edge: &EdgeRef, trim: f32) -> Option<EdgeRef> {
    trimmed_edge_ref_asymmetric(edge, trim, trim)
}

pub(crate) fn trimmed_edge_ref_asymmetric(
    edge: &EdgeRef,
    start_trim: f32,
    end_trim: f32,
) -> Option<EdgeRef> {
    let d = [
        edge.p1[0] - edge.p0[0],
        edge.p1[1] - edge.p0[1],
        edge.p1[2] - edge.p0[2],
    ];
    let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    if len <= start_trim + end_trim + 1.0e-4 {
        return None;
    }
    let t = [d[0] / len, d[1] / len, d[2] / len];
    Some(EdgeRef {
        p0: [
            edge.p0[0] + t[0] * start_trim,
            edge.p0[1] + t[1] * start_trim,
            edge.p0[2] + t[2] * start_trim,
        ],
        p1: [
            edge.p1[0] - t[0] * end_trim,
            edge.p1[1] - t[1] * end_trim,
            edge.p1[2] - t[2] * end_trim,
        ],
        n1: edge.n1,
        n2: edge.n2,
        curve: None,
        topology: None,
    })
}

pub(crate) fn edge_mod_reference_mesh(body: &LiveBody) -> MockMesh {
    let mut mesh = MockMesh::empty();
    for part in &body.parts {
        mesh.append(MockMesh::from_solid(part));
    }
    if !mesh.indices.is_empty() {
        mesh
    } else {
        body.pristine.clone().unwrap_or_else(MockMesh::empty)
    }
}

pub(crate) fn edge_mod_accept_candidate_or_recut(
    reference_mesh: &MockMesh,
    original_part: &KernelSolid,
    candidate: KernelSolid,
    recut_tools: &[CutTool],
    circular_bite_locality: Option<CircularBiteLocality<'_>>,
) -> Result<KernelSolid, String> {
    let first_reason = match edge_mod_accept_candidate_for_edge(
        reference_mesh,
        original_part,
        candidate.clone(),
        circular_bite_locality,
    ) {
        Ok(candidate) => return Ok(candidate),
        Err(reason) => reason,
    };

    let mut failures = Vec::new();
    match recut_candidate_with_tools(candidate, recut_tools) {
        Ok(Some(recut)) => {
            match edge_mod_accept_candidate_for_edge(
                reference_mesh,
                original_part,
                recut,
                circular_bite_locality,
            ) {
                Ok(recut) => return Ok(recut),
                Err(reason) => failures.push(format!("recut result rejected: {reason}")),
            }
        }
        Ok(None) => {}
        Err(reason) => failures.push(format!("recut boolean failed: {reason}")),
    }

    if failures.is_empty() {
        Err(first_reason)
    } else {
        Err(format!(
            "{first_reason}; analytic recut failed: {}",
            failures.join("; ")
        ))
    }
}

pub(crate) fn edge_mod_accept_candidate_for_edge(
    reference_mesh: &MockMesh,
    original_part: &KernelSolid,
    candidate: KernelSolid,
    circular_bite_locality: Option<CircularBiteLocality<'_>>,
) -> Result<KernelSolid, String> {
    let (candidate, candidate_mesh) =
        edge_mod_accept_candidate_with_mesh(reference_mesh, original_part, candidate)?;
    if let Some(locality) = circular_bite_locality {
        if let Some(candidate_mesh) = candidate_mesh.as_ref() {
            edge_mod_circular_bite_locality_mesh(locality, candidate_mesh)?;
        } else {
            edge_mod_circular_bite_locality(locality, &candidate)?;
        }
    }
    Ok(candidate)
}

pub(crate) fn edge_mod_circular_bite_locality(
    locality: CircularBiteLocality<'_>,
    candidate: &KernelSolid,
) -> Result<(), String> {
    let region = locality.region;
    if region.rect_circle.is_none() {
        return Ok(());
    }
    let candidate_mesh = MockMesh::from_solid(candidate);
    if candidate_mesh.indices.is_empty() {
        return Err("candidate tessellated to an empty mesh".to_string());
    }

    edge_mod_circular_bite_locality_mesh(locality, &candidate_mesh)
}

fn edge_mod_circular_bite_replay_runout_guard(
    body: &LiveBody,
    history: &CutReplayHistory,
    selection: &EdgeModSelection,
    dist: f32,
) -> Result<(), String> {
    let check_source = |source: &SketchExtrudeSource| -> Result<(), String> {
        for region in &source.regions {
            let Some(limit) = circular_bite_selected_side_runout_limit(region, selection) else {
                continue;
            };
            if dist > limit + 0.05 {
                return Err(format!(
                    "selected circular-bite side has only {limit:.3} mm of straight runout before the curved cut wall"
                ));
            }
        }
        Ok(())
    };

    if let Some(source) = body.sketch_source.as_ref() {
        check_source(source)?;
    }
    if let Some(source) = history.base_sketch_source.as_ref() {
        check_source(source)?;
    }
    Ok(())
}

fn circular_bite_selected_side_runout_limit(
    region: &SketchExtrudeRegionSource,
    selection: &EdgeModSelection,
) -> Option<f32> {
    let bite = circular_bite_void_from_region(region)?;
    let edge = &selection.active_edge;
    if !matches!(edge.curve.as_ref(), None | Some(EdgeCurveHint::Line)) {
        return None;
    }

    let p0_world = Vec3::new(edge.p0[0], edge.p0[1], edge.p0[2]);
    let p1_world = Vec3::new(edge.p1[0], edge.p1[1], edge.p1[2]);
    let p0 = region.cs.project(p0_world);
    let p1 = region.cs.project(p1_world);
    let offset0 = p0_world
        .sub(region.cs.unproject(p0.0, p0.1))
        .dot(region.cs.n);
    let offset1 = p1_world
        .sub(region.cs.unproject(p1.0, p1.1))
        .dot(region.cs.n);
    let offset = (offset0 + offset1) * 0.5;
    let cap_tol = 0.2;
    if offset.abs() > cap_tol && (offset - region.depth).abs() > cap_tol {
        return None;
    }

    let side_eps = 0.12;
    let (along_min, along_max, fixed, center_fixed, center_along, fixed0, fixed1, along0, along1) =
        match bite.side {
            0 => (
                bite.rect_min.1,
                bite.rect_max.1,
                bite.rect_min.0,
                bite.circle_center.0,
                bite.circle_center.1,
                p0.0,
                p1.0,
                p0.1,
                p1.1,
            ),
            1 => (
                bite.rect_min.1,
                bite.rect_max.1,
                bite.rect_max.0,
                bite.circle_center.0,
                bite.circle_center.1,
                p0.0,
                p1.0,
                p0.1,
                p1.1,
            ),
            2 => (
                bite.rect_min.0,
                bite.rect_max.0,
                bite.rect_min.1,
                bite.circle_center.1,
                bite.circle_center.0,
                p0.1,
                p1.1,
                p0.0,
                p1.0,
            ),
            _ => (
                bite.rect_min.0,
                bite.rect_max.0,
                bite.rect_max.1,
                bite.circle_center.1,
                bite.circle_center.0,
                p0.1,
                p1.1,
                p0.0,
                p1.0,
            ),
        };
    if (fixed0 - fixed).abs() > side_eps || (fixed1 - fixed).abs() > side_eps {
        return None;
    }

    let fixed_delta = fixed - center_fixed;
    let hit_sq = bite.circle_radius * bite.circle_radius - fixed_delta * fixed_delta;
    if hit_sq <= 0.0 {
        return None;
    }
    let hit_span = hit_sq.sqrt();
    let hit_lo = (center_along - hit_span).clamp(along_min, along_max);
    let hit_hi = (center_along + hit_span).clamp(along_min, along_max);
    if hit_hi <= hit_lo + 0.15 {
        return None;
    }

    let sel_lo = along0.min(along1);
    let sel_hi = along0.max(along1);
    let selected_len = sel_hi - sel_lo;
    if selected_len <= 0.15 {
        return None;
    }

    let match_segment = |seg_lo: f32, seg_hi: f32, circle_end: f32| {
        if seg_hi <= seg_lo + 0.15 {
            return None;
        }
        let selected_inside_segment = sel_lo >= seg_lo - side_eps && sel_hi <= seg_hi + side_eps;
        let selected_touches_circle =
            (sel_lo - circle_end).abs() <= side_eps || (sel_hi - circle_end).abs() <= side_eps;
        (selected_inside_segment && selected_touches_circle).then_some(selected_len)
    };

    match_segment(along_min, hit_lo, hit_lo).or_else(|| match_segment(hit_hi, along_max, hit_hi))
}

pub(crate) fn edge_mod_circular_bite_locality_mesh(
    locality: CircularBiteLocality<'_>,
    candidate_mesh: &MockMesh,
) -> Result<(), String> {
    let region = locality.region;
    let selection = locality.selection;
    let ghost_samples = edge_mod_circular_bite_void_ghost_sample_count(region, candidate_mesh);
    if ghost_samples > 0 {
        return Err(format!(
            "candidate placed {ghost_samples} non-wall sample(s) inside the removed circular-bite volume"
        ));
    }

    for (p0, p1) in circular_bite_unselected_side_segments(region, selection) {
        if !mesh_wire_path_covers(candidate_mesh, p0, p1, 0.08) {
            return Err(format!(
                "candidate removed unselected circular-bite side span [{:.3}, {:.3}, {:.3}] -> [{:.3}, {:.3}, {:.3}]",
                p0[0], p0[1], p0[2], p1[0], p1[1], p1[2]
            ));
        }
    }

    edge_mod_selected_blend_present(
        candidate_mesh,
        &selection.active_edge,
        locality.dist,
        locality.kind,
    )?;
    if matches!(locality.kind, crate::sketch::CornerKind::Fillet) {
        let seams = edge_mod_selected_blend_lengthwise_wire_seams(
            candidate_mesh,
            &selection.active_edge,
            locality.dist,
        );
        if seams > 0 {
            return Err(format!(
                "candidate exposed {seams} lengthwise seam edge(s) on the selected fillet surface"
            ));
        }
    }

    Ok(())
}

#[derive(Clone, Copy)]
pub(crate) struct CircularBiteVoid {
    pub(crate) rect_min: (f32, f32),
    pub(crate) rect_max: (f32, f32),
    pub(crate) circle_center: (f32, f32),
    pub(crate) circle_radius: f32,
    pub(crate) side: usize,
}

pub(crate) fn edge_mod_circular_bite_void_ghost_sample_count(
    region: &SketchExtrudeRegionSource,
    candidate_mesh: &MockMesh,
) -> usize {
    let Some(bite) = circular_bite_void_from_region(region) else {
        return 0;
    };

    let vertex6 = |vi: u32| {
        let b = vi as usize * 6;
        [
            candidate_mesh.vertices[b],
            candidate_mesh.vertices[b + 1],
            candidate_mesh.vertices[b + 2],
            candidate_mesh.vertices[b + 3],
            candidate_mesh.vertices[b + 4],
            candidate_mesh.vertices[b + 5],
        ]
    };
    let mut count = 0usize;
    for v in candidate_mesh.vertices.chunks_exact(6) {
        let sample = [v[0], v[1], v[2], v[3], v[4], v[5]];
        if circular_bite_void_contains_non_wall_sample(region, bite, sample) {
            count += 1;
        }
    }
    for tri in candidate_mesh.indices.chunks_exact(3) {
        let a = vertex6(tri[0]);
        let b = vertex6(tri[1]);
        let c = vertex6(tri[2]);
        let sample = [
            (a[0] + b[0] + c[0]) / 3.0,
            (a[1] + b[1] + c[1]) / 3.0,
            (a[2] + b[2] + c[2]) / 3.0,
            (a[3] + b[3] + c[3]) / 3.0,
            (a[4] + b[4] + c[4]) / 3.0,
            (a[5] + b[5] + c[5]) / 3.0,
        ];
        if circular_bite_void_contains_non_wall_sample(region, bite, sample) {
            count += 1;
        }
    }
    count
}

pub(crate) fn circular_bite_void_from_region(
    region: &SketchExtrudeRegionSource,
) -> Option<CircularBiteVoid> {
    region.rect_circle.as_ref()?;
    let (rect_min, rect_max, circle_center, circle_radius) =
        crate::mock_kernel::rect_minus_circle_region_primitives(&region.boundary, &region.holes)?;
    let side = circular_bite_side_for_region(
        &region.boundary,
        rect_min,
        rect_max,
        circle_center,
        circle_radius,
    )?;
    Some(CircularBiteVoid {
        rect_min,
        rect_max,
        circle_center,
        circle_radius,
        side,
    })
}

fn circular_bite_side_for_region(
    boundary: &[(f32, f32)],
    rect_min: (f32, f32),
    rect_max: (f32, f32),
    circle_center: (f32, f32),
    circle_radius: f32,
) -> Option<usize> {
    let circle_tol = (0.02 * circle_radius).max(0.12);
    let near_circle = |p: (f32, f32)| {
        ((p.0 - circle_center.0).hypot(p.1 - circle_center.1) - circle_radius).abs() <= circle_tol
    };
    let side_hits = |side: usize| {
        boundary
            .iter()
            .filter(|&&p| {
                let on_side = match side {
                    0 => (p.0 - rect_min.0).abs() <= 0.08,
                    1 => (p.0 - rect_max.0).abs() <= 0.08,
                    2 => (p.1 - rect_min.1).abs() <= 0.08,
                    _ => (p.1 - rect_max.1).abs() <= 0.08,
                };
                on_side && near_circle(p)
            })
            .count()
    };
    (0..4)
        .map(|side| (side, side_hits(side)))
        .max_by_key(|&(_, hits)| hits)
        .and_then(|(side, hits)| (hits >= 2).then_some(side))
}

fn circular_bite_void_contains_non_wall_sample(
    region: &SketchExtrudeRegionSource,
    bite: CircularBiteVoid,
    sample: [f32; 6],
) -> bool {
    let world = Vec3::new(sample[0], sample[1], sample[2]);
    let local = region.cs.project(world);
    let on_plane = region.cs.unproject(local.0, local.1);
    let depth = world.sub(on_plane).dot(region.cs.n);
    let depth_min = 0.0_f32.min(region.depth) - 0.05;
    let depth_max = 0.0_f32.max(region.depth) + 0.05;
    if depth < depth_min || depth > depth_max {
        return false;
    }

    let side_margin = 0.10_f32.max(bite.circle_radius * 0.01);
    if !point_inside_rect(local, bite.rect_min, bite.rect_max, side_margin)
        || !point_on_circular_bite_material_side(local, bite, side_margin)
    {
        return false;
    }

    let radial = (local.0 - bite.circle_center.0).hypot(local.1 - bite.circle_center.1);
    let empty_margin = (bite.circle_radius * 0.07).clamp(0.35, 1.0);
    if radial >= bite.circle_radius - empty_margin {
        return false;
    }

    let normal = Vec3::new(sample[3], sample[4], sample[5]);
    !circular_bite_wall_chord_sample(region, bite, local, normal, radial)
}

fn point_inside_rect(
    p: (f32, f32),
    rect_min: (f32, f32),
    rect_max: (f32, f32),
    margin: f32,
) -> bool {
    p.0 >= rect_min.0 - margin
        && p.0 <= rect_max.0 + margin
        && p.1 >= rect_min.1 - margin
        && p.1 <= rect_max.1 + margin
}

fn point_on_circular_bite_material_side(
    p: (f32, f32),
    bite: CircularBiteVoid,
    margin: f32,
) -> bool {
    match bite.side {
        0 => p.0 > bite.rect_min.0 + margin,
        1 => p.0 < bite.rect_max.0 - margin,
        2 => p.1 > bite.rect_min.1 + margin,
        _ => p.1 < bite.rect_max.1 - margin,
    }
}

fn circular_bite_wall_chord_sample(
    region: &SketchExtrudeRegionSource,
    bite: CircularBiteVoid,
    local: (f32, f32),
    normal: Vec3,
    radial: f32,
) -> bool {
    let wall_band = (bite.circle_radius * 0.15).clamp(0.75, 2.0);
    if radial < bite.circle_radius - wall_band || radial > bite.circle_radius + 0.25 {
        return false;
    }

    let normal_in_plane = (normal.dot(region.cs.u), normal.dot(region.cs.v));
    let normal_depth = normal.dot(region.cs.n).abs();
    if normal_depth > 0.55 {
        return false;
    }
    let nl = (normal_in_plane.0 * normal_in_plane.0 + normal_in_plane.1 * normal_in_plane.1).sqrt();
    if nl <= 1.0e-5 || radial <= 1.0e-5 {
        return false;
    }
    let radial_vec = (
        local.0 - bite.circle_center.0,
        local.1 - bite.circle_center.1,
    );
    let dot = (radial_vec.0 / radial) * (normal_in_plane.0 / nl)
        + (radial_vec.1 / radial) * (normal_in_plane.1 / nl);
    dot.abs() > 0.75
}

pub(crate) fn edge_mod_selected_blend_present(
    candidate_mesh: &MockMesh,
    edge: &EdgeRef,
    dist: f32,
    kind: crate::sketch::CornerKind,
) -> Result<(), String> {
    if !matches!(edge.curve.as_ref(), None | Some(EdgeCurveHint::Line)) {
        return Err(
            "candidate selected-edge locality validation is only implemented for straight edges"
                .to_string(),
        );
    }

    let run = sub3(edge.p1, edge.p0);
    let len = length_sq3(run).sqrt();
    if len <= 1.0e-4 {
        return Err(
            "candidate selected-edge locality validation got a zero-length edge".to_string(),
        );
    }

    let t = mul3(run, 1.0 / len);
    let n1 = normalize3(edge.n1);
    let n2 = normalize3(edge.n2);
    if length_sq3(cross3(n1, n2)) <= 1.0e-6 {
        return Err(
            "candidate selected-edge locality validation got parallel face normals".to_string(),
        );
    }

    let inward_on_face1 = mul3(n2, -1.0);
    let inward_on_face2 = mul3(n1, -1.0);
    let f1_raw = sub3(inward_on_face1, mul3(n1, dot3(inward_on_face1, n1)));
    let f2_raw = sub3(inward_on_face2, mul3(n2, dot3(inward_on_face2, n2)));
    if length_sq3(f1_raw) <= 0.25 || length_sq3(f2_raw) <= 0.25 {
        return Err(
            "candidate selected-edge locality validation could not build an edge-local frame"
                .to_string(),
        );
    }
    let f1 = normalize3(f1_raw);
    let f2 = normalize3(f2_raw);

    let span_slack = (dist * 0.08).clamp(0.08, 0.5);
    let min_offset = (dist * 0.04).max(0.025);
    let max_offset = dist + EDGE_MOD_GROW + 0.30;
    let mut samples = 0usize;
    let mut span_hits = 0usize;
    let mut offset_hits = 0usize;
    let mut normal_hits = 0usize;
    let mut min_s = f32::INFINITY;
    let mut max_s = f32::NEG_INFINITY;
    let mut normal_bins = std::collections::HashSet::new();

    for tri in candidate_mesh.indices.chunks_exact(3) {
        let a = mesh_vertex_pos6(candidate_mesh, tri[0]);
        let b = mesh_vertex_pos6(candidate_mesh, tri[1]);
        let c = mesh_vertex_pos6(candidate_mesh, tri[2]);
        let p = mul3(add3(add3(a, b), c), 1.0 / 3.0);
        let rel = sub3(p, edge.p0);
        let s = dot3(rel, t);
        if s < -span_slack || s > len + span_slack {
            continue;
        }
        span_hits += 1;

        let u = dot3(rel, f1);
        let v = dot3(rel, f2);
        if u < min_offset || v < min_offset || u > max_offset || v > max_offset {
            continue;
        }
        offset_hits += 1;

        let na = mesh_vertex_normal6(candidate_mesh, tri[0]);
        let nb = mesh_vertex_normal6(candidate_mesh, tri[1]);
        let nc = mesh_vertex_normal6(candidate_mesh, tri[2]);
        let normal_raw = mul3(add3(add3(na, nb), nc), 1.0 / 3.0);
        let face_normal_raw = cross3(sub3(b, a), sub3(c, a));
        let expected = normalize3(add3(n1, n2));
        let face_normal = if dot3(face_normal_raw, expected) < 0.0 {
            mul3(face_normal_raw, -1.0)
        } else {
            face_normal_raw
        };
        let Some(bin) = [normal_raw, face_normal]
            .into_iter()
            .find_map(|normal| edge_mod_blend_normal_bin(normal, n1, n2))
        else {
            continue;
        };
        normal_hits += 1;

        samples += 1;
        min_s = min_s.min(s);
        max_s = max_s.max(s);
        normal_bins.insert(bin);
    }

    let required_samples = match kind {
        crate::sketch::CornerKind::Fillet => 3,
        crate::sketch::CornerKind::Chamfer => 1,
    };
    if samples < required_samples {
        return Err(format!(
            "candidate did not create a {kind:?} surface on the selected circular-bite edge \
             (span hits {span_hits}, offset hits {offset_hits}, normal hits {normal_hits})"
        ));
    }

    let required_span = (len * 0.25).clamp(0.20, len * 0.75);
    if max_s - min_s < required_span {
        return Err(format!(
            "candidate {kind:?} surface covered only {:.2}mm of the selected {:.2}mm edge",
            max_s - min_s,
            len
        ));
    }

    if matches!(kind, crate::sketch::CornerKind::Fillet) && normal_bins.len() < 2 {
        return Err(
            "candidate fillet surface on the selected circular-bite edge did not have rounded normals"
                .to_string(),
        );
    }

    Ok(())
}

pub(crate) fn edge_mod_selected_blend_lengthwise_wire_seams(
    mesh: &MockMesh,
    edge: &EdgeRef,
    dist: f32,
) -> usize {
    if !matches!(edge.curve.as_ref(), None | Some(EdgeCurveHint::Line)) {
        return 0;
    }
    let run = sub3(edge.p1, edge.p0);
    let len = length_sq3(run).sqrt();
    if len <= 1.0e-4 {
        return 0;
    }
    let t = mul3(run, 1.0 / len);
    let inward1 = normalize3(mul3(edge.n1, -1.0));
    let inward2 = normalize3(mul3(edge.n2, -1.0));
    let min_offset = (dist * 0.03).max(0.06);
    let max_offset = dist + EDGE_MOD_GROW + 0.35;

    (0..mesh.edge_indices.len() / 2)
        .filter(|&e| {
            let ia = mesh.edge_indices[e * 2] as usize * 3;
            let ib = mesh.edge_indices[e * 2 + 1] as usize * 3;
            let a = [
                mesh.edge_vertices[ia],
                mesh.edge_vertices[ia + 1],
                mesh.edge_vertices[ia + 2],
            ];
            let b = [
                mesh.edge_vertices[ib],
                mesh.edge_vertices[ib + 1],
                mesh.edge_vertices[ib + 2],
            ];
            let d = sub3(b, a);
            let seg_len = length_sq3(d).sqrt();
            if seg_len < 0.45 {
                return false;
            }
            let along = (dot3(d, t) / seg_len).abs();
            if along < 0.96 {
                return false;
            }
            let inside = |p: [f32; 3]| {
                let rel = sub3(p, edge.p0);
                let s = dot3(rel, t);
                let o1 = dot3(rel, inward1);
                let o2 = dot3(rel, inward2);
                s > 0.08
                    && s < len - 0.08
                    && o1 > min_offset
                    && o1 < max_offset
                    && o2 > min_offset
                    && o2 < max_offset
            };
            inside(a) && inside(b)
        })
        .count()
}

pub(crate) fn edge_mod_blend_normal_bin(
    normal: [f32; 3],
    n1: [f32; 3],
    n2: [f32; 3],
) -> Option<i32> {
    if length_sq3(normal) <= 1.0e-8 {
        return None;
    }
    let normal = normalize3(normal);
    let d1 = dot3(normal, n1).clamp(-1.0, 1.0);
    let d2 = dot3(normal, n2).clamp(-1.0, 1.0);
    if d1 <= 0.08 || d2 <= 0.08 || d1.abs() > 0.985 || d2.abs() > 0.985 {
        return None;
    }
    let angle = d2.atan2(d1);
    Some((angle * 16.0 / std::f32::consts::FRAC_PI_2).round() as i32)
}

pub(crate) fn circular_bite_unselected_side_segments(
    region: &SketchExtrudeRegionSource,
    selection: &EdgeModSelection,
) -> Vec<([f32; 3], [f32; 3])> {
    let Some(((min_x, min_y), (max_x, max_y))) = loop_bounds_2d(&region.boundary) else {
        return Vec::new();
    };
    let original = &selection.original_edge;
    let selected = &selection.active_edge;
    let p0_world = Vec3::new(original.p0[0], original.p0[1], original.p0[2]);
    let p1_world = Vec3::new(original.p1[0], original.p1[1], original.p1[2]);
    let p0 = region.cs.project(p0_world);
    let p1 = region.cs.project(p1_world);
    let sel0_world = Vec3::new(selected.p0[0], selected.p0[1], selected.p0[2]);
    let sel1_world = Vec3::new(selected.p1[0], selected.p1[1], selected.p1[2]);
    let sel0 = region.cs.project(sel0_world);
    let sel1 = region.cs.project(sel1_world);
    let offset0 = p0_world
        .sub(region.cs.unproject(p0.0, p0.1))
        .dot(region.cs.n);
    let offset1 = p1_world
        .sub(region.cs.unproject(p1.0, p1.1))
        .dot(region.cs.n);
    let offset = (offset0 + offset1) * 0.5;
    let cap_tol = 0.2;
    if offset.abs() > cap_tol && (offset - region.depth).abs() > cap_tol {
        return Vec::new();
    }
    let side_eps = 0.12;

    let side = if (p0.1 - min_y).abs() <= side_eps && (p1.1 - min_y).abs() <= side_eps {
        0usize
    } else if (p0.0 - max_x).abs() <= side_eps && (p1.0 - max_x).abs() <= side_eps {
        1
    } else if (p0.1 - max_y).abs() <= side_eps && (p1.1 - max_y).abs() <= side_eps {
        2
    } else if (p0.0 - min_x).abs() <= side_eps && (p1.0 - min_x).abs() <= side_eps {
        3
    } else {
        return Vec::new();
    };

    let on_side = |p: (f32, f32)| match side {
        0 => (p.1 - min_y).abs() <= side_eps,
        1 => (p.0 - max_x).abs() <= side_eps,
        2 => (p.1 - max_y).abs() <= side_eps,
        _ => (p.0 - min_x).abs() <= side_eps,
    };
    let same_original_segment = |a: (f32, f32), b: (f32, f32)| {
        (dist2(a, p0) <= side_eps && dist2(b, p1) <= side_eps)
            || (dist2(a, p1) <= side_eps && dist2(b, p0) <= side_eps)
    };
    let selected_matches_boundary = (0..region.boundary.len()).any(|i| {
        let a = region.boundary[i];
        let b = region.boundary[(i + 1) % region.boundary.len()];
        on_side(a) && on_side(b) && same_original_segment(a, b)
    });
    if !selected_matches_boundary {
        return Vec::new();
    }
    let same_segment = |a: (f32, f32), b: (f32, f32)| {
        (dist2(a, sel0) <= side_eps && dist2(b, sel1) <= side_eps)
            || (dist2(a, sel1) <= side_eps && dist2(b, sel0) <= side_eps)
    };
    let to_world = |p: (f32, f32)| {
        let q = region.cs.unproject(p.0, p.1).add(region.cs.n.mul(offset));
        [q.x, q.y, q.z]
    };
    let coord = |p: (f32, f32)| {
        if side == 0 || side == 2 {
            p.0
        } else {
            p.1
        }
    };
    let point_at = |a: (f32, f32), v: f32| {
        if side == 0 || side == 2 {
            (v, a.1)
        } else {
            (a.0, v)
        }
    };

    let mut out = Vec::new();
    for i in 0..region.boundary.len() {
        let a = region.boundary[i];
        let b = region.boundary[(i + 1) % region.boundary.len()];
        if !on_side(a) || !on_side(b) || dist2(a, b) <= 0.15 || same_segment(a, b) {
            continue;
        }
        let lo = coord(a).min(coord(b));
        let hi = coord(a).max(coord(b));
        let sel_lo = coord(sel0).min(coord(sel1));
        let sel_hi = coord(sel0).max(coord(sel1));
        let overlap_lo = lo.max(sel_lo);
        let overlap_hi = hi.min(sel_hi);
        let mut push_span = |s0: f32, s1: f32| {
            if (s1 - s0).abs() <= 0.15 {
                return;
            }
            let q0 = point_at(a, s0);
            let q1 = point_at(a, s1);
            if coord(a) <= coord(b) {
                out.push((to_world(q0), to_world(q1)));
            } else {
                out.push((to_world(q1), to_world(q0)));
            }
        };
        if overlap_hi <= overlap_lo + 1.0e-3 {
            push_span(lo, hi);
        } else {
            push_span(lo, overlap_lo);
            push_span(overlap_hi, hi);
        }
    }
    out
}

pub(crate) fn dist2(a: (f32, f32), b: (f32, f32)) -> f32 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

pub(crate) fn edge_mod_accept_candidate(
    reference_mesh: &MockMesh,
    original_part: &KernelSolid,
    candidate: KernelSolid,
) -> Result<KernelSolid, String> {
    edge_mod_accept_candidate_with_mesh(reference_mesh, original_part, candidate)
        .map(|(candidate, _)| candidate)
}

pub(crate) fn edge_mod_accept_candidate_with_mesh(
    reference_mesh: &MockMesh,
    original_part: &KernelSolid,
    candidate: KernelSolid,
) -> Result<(KernelSolid, Option<MockMesh>), String> {
    if !edge_mod_keeps_body(original_part, &candidate) {
        return Err("candidate expands outside the original part bounds".to_string());
    }
    if !crate::mock_kernel::preserves_cylindrical_faces(original_part, &candidate) {
        return Err(
            "candidate lost an analytic cylindrical face from the original body".to_string(),
        );
    }
    if let Some((lo, hi)) = mesh_position_aabb(reference_mesh) {
        if mesh_is_aabb_box(reference_mesh, lo, hi, EDGE_MOD_CONTAINMENT_TOL) {
            return Ok((candidate, None));
        }
    }
    let candidate_mesh = MockMesh::from_solid(&candidate);
    if candidate_mesh.indices.is_empty() {
        return Err("candidate tessellated to an empty mesh".to_string());
    }
    let preserved_cylinder_faces =
        crate::mock_kernel::preserved_cylindrical_face_ids(original_part, &candidate);
    edge_mod_render_mesh_has_no_cracks(&candidate_mesh)?;
    edge_mod_mesh_stays_inside_reference(
        reference_mesh,
        &candidate_mesh,
        EDGE_MOD_CONTAINMENT_TOL,
        Some(&preserved_cylinder_faces),
    )?;
    Ok((candidate, Some(candidate_mesh)))
}

pub(crate) fn edge_mod_render_mesh_has_no_cracks(mesh: &MockMesh) -> Result<(), String> {
    let q = |i: usize| -> (i64, i64, i64) {
        let b = i * 6;
        let quant = |v: f32| (v as f64 * 10_000.0).round() as i64;
        (
            quant(mesh.vertices[b]),
            quant(mesh.vertices[b + 1]),
            quant(mesh.vertices[b + 2]),
        )
    };
    let mut edges: std::collections::HashMap<((i64, i64, i64), (i64, i64, i64)), u32> =
        std::collections::HashMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            let (ka, kb) = (q(tri[a] as usize), q(tri[b] as usize));
            let key = if ka <= kb { (ka, kb) } else { (kb, ka) };
            *edges.entry(key).or_insert(0) += 1;
        }
    }

    let cracks = edges.values().filter(|&&count| count == 1).count();
    if cracks == 0 {
        Ok(())
    } else {
        Err(format!("candidate render mesh has {cracks} crack edges"))
    }
}

pub(crate) fn edge_mod_reject_unhealthy_native_curve_result(
    selection: &EdgeModSelection,
    candidate: &KernelSolid,
) -> Result<(), String> {
    if !edge_mod_native_only(selection) {
        return Ok(());
    }
    let mesh = MockMesh::from_solid(candidate);
    if mesh.indices.is_empty() {
        return Err("candidate tessellated to an empty mesh".to_string());
    }
    let nonmanifold = edge_mod_render_mesh_nonmanifold_edges(&mesh);
    if nonmanifold == 0 {
        Ok(())
    } else {
        Err(format!(
            "candidate render mesh is not watertight and healthy \
             (0 crack edges, {nonmanifold} non-manifold edges)"
        ))
    }
}

pub(crate) fn edge_mod_render_mesh_nonmanifold_edges(mesh: &MockMesh) -> usize {
    let q = |i: usize| -> (i64, i64, i64) {
        let b = i * 6;
        let quant = |v: f32| (v as f64 * 10_000.0).round() as i64;
        (
            quant(mesh.vertices[b]),
            quant(mesh.vertices[b + 1]),
            quant(mesh.vertices[b + 2]),
        )
    };
    let mut edges: std::collections::HashMap<((i64, i64, i64), (i64, i64, i64)), u32> =
        std::collections::HashMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            let (ka, kb) = (q(tri[a] as usize), q(tri[b] as usize));
            let key = if ka <= kb { (ka, kb) } else { (kb, ka) };
            *edges.entry(key).or_insert(0) += 1;
        }
    }
    edges.values().filter(|&&count| count > 2).count()
}

pub(crate) fn edge_mod_render_mesh_inward_triangles(mesh: &MockMesh) -> usize {
    let mut inward = 0;
    let pos = |i: u32| {
        let b = i as usize * 6;
        [
            mesh.vertices[b] as f64,
            mesh.vertices[b + 1] as f64,
            mesh.vertices[b + 2] as f64,
        ]
    };
    let nrm = |i: u32| {
        let b = i as usize * 6;
        [
            mesh.vertices[b + 3] as f64,
            mesh.vertices[b + 4] as f64,
            mesh.vertices[b + 5] as f64,
        ]
    };
    for tri in mesh.indices.chunks_exact(3) {
        let a = pos(tri[0]);
        let b = pos(tri[1]);
        let c = pos(tri[2]);
        let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let winding = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ];
        let navg = [
            (nrm(tri[0])[0] + nrm(tri[1])[0] + nrm(tri[2])[0]) / 3.0,
            (nrm(tri[0])[1] + nrm(tri[1])[1] + nrm(tri[2])[1]) / 3.0,
            (nrm(tri[0])[2] + nrm(tri[1])[2] + nrm(tri[2])[2]) / 3.0,
        ];
        if winding[0] * navg[0] + winding[1] * navg[1] + winding[2] * navg[2] < 0.0 {
            inward += 1;
        }
    }
    inward
}

#[cfg(test)]
pub(crate) fn edge_mod_candidate_stays_inside_reference(
    reference_mesh: &MockMesh,
    candidate: &KernelSolid,
) -> Result<(), String> {
    let candidate_mesh = MockMesh::from_solid(candidate);
    if candidate_mesh.indices.is_empty() {
        return Err("candidate tessellated to an empty mesh".to_string());
    }
    edge_mod_mesh_stays_inside_reference(
        reference_mesh,
        &candidate_mesh,
        EDGE_MOD_CONTAINMENT_TOL,
        None,
    )
}

pub(crate) fn edge_mod_mesh_stays_inside_reference(
    reference_mesh: &MockMesh,
    candidate_mesh: &MockMesh,
    tol: f32,
    preserved_cylinder_faces: Option<&std::collections::HashSet<u32>>,
) -> Result<(), String> {
    if reference_mesh.indices.is_empty() {
        return Err("reference body could not be tessellated".to_string());
    }
    if candidate_mesh.indices.is_empty() {
        return Err("candidate body could not be tessellated".to_string());
    }

    let Some((lo, hi)) = mesh_position_aabb(reference_mesh) else {
        return Err("reference body has no render vertices".to_string());
    };
    let aabb_only = mesh_is_aabb_box(reference_mesh, lo, hi, tol);

    for (i, v) in candidate_mesh.vertices.chunks_exact(6).enumerate() {
        let p = [v[0], v[1], v[2]];
        if !point_in_aabb(p, lo, hi, tol) {
            return Err(format!(
                "candidate vertex {i} at [{:.3}, {:.3}, {:.3}] is outside the pre-edge body bounds",
                p[0], p[1], p[2]
            ));
        }
        if !aabb_only && !point_inside_triangle_mesh(reference_mesh, p, tol) {
            return Err(format!(
                "candidate vertex {i} at [{:.3}, {:.3}, {:.3}] is outside the pre-edge body",
                p[0], p[1], p[2]
            ));
        }
    }

    for (i, tri) in candidate_mesh.indices.chunks_exact(3).enumerate() {
        let a = mesh_vertex_pos6(candidate_mesh, tri[0]);
        let b = mesh_vertex_pos6(candidate_mesh, tri[1]);
        let c = mesh_vertex_pos6(candidate_mesh, tri[2]);
        let p = [
            (a[0] + b[0] + c[0]) / 3.0,
            (a[1] + b[1] + c[1]) / 3.0,
            (a[2] + b[2] + c[2]) / 3.0,
        ];
        if !point_in_aabb(p, lo, hi, tol) {
            return Err(format!(
                "candidate triangle {i} centroid at [{:.3}, {:.3}, {:.3}] is outside the pre-edge body bounds",
                p[0], p[1], p[2]
            ));
        }
        if !aabb_only && !point_inside_triangle_mesh(reference_mesh, p, tol) {
            let face_id = candidate_mesh.face_ids.get(i).copied().unwrap_or(0);
            if preserved_cylinder_faces.is_some_and(|faces| faces.contains(&face_id)) {
                continue;
            }
            return Err(format!(
                "candidate triangle {i} centroid at [{:.3}, {:.3}, {:.3}] is outside the pre-edge body",
                p[0], p[1], p[2]
            ));
        }
    }

    Ok(())
}

pub(crate) fn edge_mod_mesh_stays_inside_reference_bounds(
    reference_mesh: &MockMesh,
    candidate_mesh: &MockMesh,
    tol: f32,
) -> Result<(), String> {
    if reference_mesh.indices.is_empty() {
        return Err("reference body could not be tessellated".to_string());
    }
    if candidate_mesh.indices.is_empty() {
        return Err("candidate body could not be tessellated".to_string());
    }

    let Some((lo, hi)) = mesh_position_aabb(reference_mesh) else {
        return Err("reference body has no render vertices".to_string());
    };

    for (i, v) in candidate_mesh.vertices.chunks_exact(6).enumerate() {
        let p = [v[0], v[1], v[2]];
        if !point_in_aabb(p, lo, hi, tol) {
            return Err(format!(
                "candidate vertex {i} at [{:.3}, {:.3}, {:.3}] is outside the pre-edge body bounds",
                p[0], p[1], p[2]
            ));
        }
    }

    for (i, tri) in candidate_mesh.indices.chunks_exact(3).enumerate() {
        let a = mesh_vertex_pos6(candidate_mesh, tri[0]);
        let b = mesh_vertex_pos6(candidate_mesh, tri[1]);
        let c = mesh_vertex_pos6(candidate_mesh, tri[2]);
        let p = [
            (a[0] + b[0] + c[0]) / 3.0,
            (a[1] + b[1] + c[1]) / 3.0,
            (a[2] + b[2] + c[2]) / 3.0,
        ];
        if !point_in_aabb(p, lo, hi, tol) {
            return Err(format!(
                "candidate triangle {i} centroid at [{:.3}, {:.3}, {:.3}] is outside the pre-edge body bounds",
                p[0], p[1], p[2]
            ));
        }
    }

    Ok(())
}

pub(crate) fn mesh_is_aabb_box(mesh: &MockMesh, lo: [f32; 3], hi: [f32; 3], tol: f32) -> bool {
    let key = |p: [f32; 3]| -> (i64, i64, i64) {
        let q = |v: f32| (v as f64 * 10_000.0).round() as i64;
        (q(p[0]), q(p[1]), q(p[2]))
    };
    let mut unique = std::collections::HashSet::new();
    for v in mesh.vertices.chunks_exact(6) {
        let p = [v[0], v[1], v[2]];
        if !(0..3).all(|k| (p[k] - lo[k]).abs() <= tol || (p[k] - hi[k]).abs() <= tol) {
            return false;
        }
        unique.insert(key(p));
    }
    !unique.is_empty() && unique.len() <= 8
}

pub(crate) fn mesh_position_aabb(mesh: &MockMesh) -> Option<([f32; 3], [f32; 3])> {
    let mut lo = [f32::INFINITY; 3];
    let mut hi = [f32::NEG_INFINITY; 3];
    let mut any = false;
    for v in mesh.vertices.chunks_exact(6) {
        any = true;
        for k in 0..3 {
            lo[k] = lo[k].min(v[k]);
            hi[k] = hi[k].max(v[k]);
        }
    }
    any.then_some((lo, hi))
}

pub(crate) fn point_in_aabb(p: [f32; 3], lo: [f32; 3], hi: [f32; 3], tol: f32) -> bool {
    (0..3).all(|k| p[k] >= lo[k] - tol && p[k] <= hi[k] + tol)
}

pub(crate) fn point_inside_triangle_mesh(mesh: &MockMesh, p: [f32; 3], tol: f32) -> bool {
    let tol2 = tol * tol;
    for tri in mesh.indices.chunks_exact(3) {
        let a = mesh_vertex_pos6(mesh, tri[0]);
        let b = mesh_vertex_pos6(mesh, tri[1]);
        let c = mesh_vertex_pos6(mesh, tri[2]);
        if point_triangle_distance_sq(p, a, b, c) <= tol2 {
            return true;
        }
    }

    let dir = normalize3([2.0, 3.0, 5.0]);
    let mut hits = Vec::new();
    for tri in mesh.indices.chunks_exact(3) {
        let a = mesh_vertex_pos6(mesh, tri[0]);
        let b = mesh_vertex_pos6(mesh, tri[1]);
        let c = mesh_vertex_pos6(mesh, tri[2]);
        if let Some(t) = ray_triangle_intersection(p, dir, a, b, c) {
            if t > tol.max(1.0e-5) {
                hits.push(t);
            }
        }
    }
    hits.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    hits.dedup_by(|a, b| (*a - *b).abs() <= 1.0e-4);
    hits.len() % 2 == 1
}

pub(crate) fn mesh_vertex_pos6(mesh: &MockMesh, vi: u32) -> [f32; 3] {
    let b = vi as usize * 6;
    [mesh.vertices[b], mesh.vertices[b + 1], mesh.vertices[b + 2]]
}

pub(crate) fn mesh_vertex_normal6(mesh: &MockMesh, vi: u32) -> [f32; 3] {
    let b = vi as usize * 6;
    [
        mesh.vertices[b + 3],
        mesh.vertices[b + 4],
        mesh.vertices[b + 5],
    ]
}

pub(crate) fn mesh_edge_vertex_pos3(mesh: &MockMesh, vi: u32) -> [f32; 3] {
    let b = vi as usize * 3;
    [
        mesh.edge_vertices[b],
        mesh.edge_vertices[b + 1],
        mesh.edge_vertices[b + 2],
    ]
}

pub(crate) fn mesh_wire_path_covers(mesh: &MockMesh, p0: [f32; 3], p1: [f32; 3], tol: f32) -> bool {
    let axis = sub3(p1, p0);
    let len_sq = length_sq3(axis);
    if len_sq <= 1.0e-12 {
        return false;
    }
    let margin = (tol / len_sq.sqrt()).max(1.0e-4);
    let endpoint_interval = |p: [f32; 3]| -> Option<f32> {
        let t = dot3(sub3(p, p0), axis) / len_sq;
        if !(-margin..=1.0 + margin).contains(&t) {
            return None;
        }
        let nearest = add3(p0, mul3(axis, t.clamp(0.0, 1.0)));
        (length_sq3(sub3(p, nearest)).sqrt() <= tol).then_some(t.clamp(0.0, 1.0))
    };

    let mut intervals = Vec::new();
    for edge in mesh.edge_indices.chunks_exact(2) {
        let a = mesh_edge_vertex_pos3(mesh, edge[0]);
        let b = mesh_edge_vertex_pos3(mesh, edge[1]);
        let (Some(ta), Some(tb)) = (endpoint_interval(a), endpoint_interval(b)) else {
            continue;
        };
        if (ta - tb).abs() <= margin {
            continue;
        }
        intervals.push((ta.min(tb), ta.max(tb)));
    }
    if intervals.is_empty() {
        return false;
    }
    intervals.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    let mut covered = 0.0f32;
    for (start, end) in intervals {
        if start > covered + margin {
            break;
        }
        covered = covered.max(end);
        if covered >= 1.0 - margin {
            return true;
        }
    }
    false
}

pub(crate) fn ray_triangle_intersection(
    origin: [f32; 3],
    dir: [f32; 3],
    a: [f32; 3],
    b: [f32; 3],
    c: [f32; 3],
) -> Option<f32> {
    const EPS: f32 = 1.0e-7;
    let e1 = sub3(b, a);
    let e2 = sub3(c, a);
    let h = cross3(dir, e2);
    let det = dot3(e1, h);
    if det.abs() < EPS {
        return None;
    }
    let inv_det = 1.0 / det;
    let s = sub3(origin, a);
    let u = dot3(s, h) * inv_det;
    if !(-EPS..=1.0 + EPS).contains(&u) {
        return None;
    }
    let q = cross3(s, e1);
    let v = dot3(dir, q) * inv_det;
    if v < -EPS || u + v > 1.0 + EPS {
        return None;
    }
    let t = dot3(e2, q) * inv_det;
    (t > EPS).then_some(t)
}

pub(crate) fn point_triangle_distance_sq(
    p: [f32; 3],
    a: [f32; 3],
    b: [f32; 3],
    c: [f32; 3],
) -> f32 {
    let ab = sub3(b, a);
    let ac = sub3(c, a);
    let ap = sub3(p, a);
    let d1 = dot3(ab, ap);
    let d2 = dot3(ac, ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return length_sq3(ap);
    }

    let bp = sub3(p, b);
    let d3 = dot3(ab, bp);
    let d4 = dot3(ac, bp);
    if d3 >= 0.0 && d4 <= d3 {
        return length_sq3(bp);
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return length_sq3(sub3(p, add3(a, mul3(ab, v))));
    }

    let cp = sub3(p, c);
    let d5 = dot3(ab, cp);
    let d6 = dot3(ac, cp);
    if d6 >= 0.0 && d5 <= d6 {
        return length_sq3(cp);
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return length_sq3(sub3(p, add3(a, mul3(ac, w))));
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return length_sq3(sub3(p, add3(b, mul3(sub3(c, b), w))));
    }

    let n = cross3(ab, ac);
    let n_len_sq = length_sq3(n);
    if n_len_sq <= 1.0e-12 {
        return length_sq3(ap).min(length_sq3(bp)).min(length_sq3(cp));
    }
    let dist = dot3(ap, n).abs() / n_len_sq.sqrt();
    dist * dist
}

pub(crate) fn add3(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub(crate) fn sub3(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub(crate) fn distance3(a: [f32; 3], b: [f32; 3]) -> f32 {
    length_sq3(sub3(a, b)).sqrt()
}

pub(crate) fn mul3(a: [f32; 3], s: f32) -> [f32; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

pub(crate) fn dot3(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub(crate) fn cross3(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

pub(crate) fn length_sq3(a: [f32; 3]) -> f32 {
    dot3(a, a)
}

pub(crate) fn point_line_distance3(p: [f32; 3], origin: [f32; 3], unit_dir: [f32; 3]) -> f32 {
    let v = sub3(p, origin);
    let nearest = add3(origin, mul3(unit_dir, dot3(v, unit_dir)));
    distance3(p, nearest)
}

pub(crate) fn normalize3(a: [f32; 3]) -> [f32; 3] {
    let len = length_sq3(a).sqrt();
    if len <= 1.0e-12 {
        [1.0, 0.0, 0.0]
    } else {
        [a[0] / len, a[1] / len, a[2] / len]
    }
}

/// Guard against a degenerate edge-mod boolean. A fillet/chamfer is a *pure
/// subtraction*, so a correct result must (a) still retain the bulk of the part
/// — it only shaves a corner — and (b) never extend **beyond** the original part
/// (`a − b ⊆ a`). A tangent/inverted boolean that self-intersects or adds
/// material instead flares the result's bounds outside the part; rejecting that
/// forces the caller to fall through to the robust cutter (or keep the body
/// intact). Missing bounds → accept (vertexless can't be judged).
pub(crate) fn edge_mod_keeps_body(part: &KernelSolid, result: &KernelSolid) -> bool {
    match (
        crate::mock_kernel::solid_aabb(part),
        crate::mock_kernel::solid_aabb(result),
    ) {
        (Some(p), Some(r)) => {
            let vol = |b: &([f32; 3], [f32; 3])| {
                ((b.1[0] - b.0[0]) * (b.1[1] - b.0[1]) * (b.1[2] - b.0[2])).abs()
            };
            let pv = vol(&p);
            // Must keep the bulk of the part (corner removal is small).
            let keeps_bulk = pv <= 1.0e-6 || vol(&r) >= pv * 0.5;
            // Must not extend past the part — a subtraction can only remove. The
            // slack covers the cutter's own end-overshoot/grow and tessellation
            // noise; real garbage flares out far more than this.
            const SLACK: f32 = 0.3;
            let within = (0..3).all(|k| r.0[k] >= p.0[k] - SLACK && r.1[k] <= p.1[k] + SLACK);
            keeps_bulk && within
        }
        (None, None) => true,
        _ => false,
    }
}
