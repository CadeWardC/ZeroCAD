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
        crate::sketch::CornerKind::Fillet => apply_fillet(mod_id, &selection, dist, body, warnings),
        crate::sketch::CornerKind::Chamfer => {
            apply_chamfer(mod_id, &selection, dist, body, warnings)
        }
    }
}

pub(crate) fn resolve_edge_ref_by_topology(body: &LiveBody, edge: &EdgeRef) -> Option<EdgeRef> {
    let requested = edge.topology.as_ref()?;
    let requested_edge_id = requested.edge_id.as_deref()?;
    if requested
        .body_id
        .as_deref()
        .is_some_and(|body_id| body_id != body.id)
    {
        return None;
    }

    if let Some(resolved) = body.pristine.as_ref().and_then(|mesh| {
        mesh.edge_refs
            .iter()
            .find(|candidate| topology_edge_id(candidate) == Some(requested_edge_id))
            .filter(|candidate| mesh_candidate_matches_captured_edge(candidate, edge))
            .map(|candidate| edge_ref_from_mesh_candidate(body, candidate, requested))
    }) {
        return Some(resolved);
    }

    let mesh = edge_mod_reference_mesh(body);
    mesh.edge_refs
        .iter()
        .find(|candidate| topology_edge_id(candidate) == Some(requested_edge_id))
        .filter(|candidate| mesh_candidate_matches_captured_edge(candidate, edge))
        .map(|candidate| edge_ref_from_mesh_candidate(body, candidate, requested))
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
    matches!(
        (candidate_id, requested_id),
        (Some(candidate), Some(requested))
            if candidate == requested && requested.starts_with("sketch:")
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
}

impl EdgeModResult {
    fn single(part: KernelSolid) -> Self {
        Self {
            parts: vec![part],
            pristine: None,
        }
    }
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

/// Native rolling-ball fillet of the captured edge on every part of `body`.
pub(crate) fn apply_fillet(
    mod_id: &str,
    selection: &EdgeModSelection,
    dist: f32,
    body: &mut LiveBody,
    warnings: &mut Vec<String>,
) {
    let mut applied = false;
    let mut last_err: Option<String> = None;
    let mut next: Vec<KernelSolid> = Vec::with_capacity(body.parts.len());
    let mut next_pristine = MockMesh::empty();
    let mut can_use_pristine = true;
    let reference_mesh = edge_mod_reference_mesh(body);
    let sketch_source = body.sketch_source.clone();
    let recut_tools = body.cut_tools.clone();
    for (part_index, part) in body.parts.drain(..).enumerate() {
        let mut part_failures = Vec::new();
        // No pre-size gate: the kernel's rolling-ball blend rejects a radius too
        // large for the local geometry (a non-watertight result → `Err`), which
        // is the correct, geometry-aware bound. The old global-AABB heuristic was
        // both wrong (it measured the part's *thinnest* axis, not the filleted
        // edge's adjacent-face extents, so it blocked radii the kernel handles)
        // and asymmetric — chamfer never had it, which is why a radius would
        // chamfer but refuse to fillet.
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
        let native_only = edge_mod_native_only(selection);
        let has_rect_circle_recipe = !native_only
            && sketch_region
                .and_then(|region| region.rect_circle.as_ref())
                .is_some();
        let native_reason = if accepted.is_none() {
            match edge_mod_try_native_fillet(
                &reference_mesh,
                &part,
                &part,
                selection,
                dist,
                "native",
                &recut_tools,
                circular_bite_locality,
            ) {
                Ok(f) => {
                    accepted = Some(EdgeModResult::single(f));
                    None
                }
                Err(reason) => Some(reason),
            }
        } else {
            None
        };
        if accepted.is_none() {
            if let Some(reason) = native_reason.clone() {
                part_failures.push(reason);
            }
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
                    &reference_mesh,
                    &part,
                    alternate_part,
                    selection,
                    dist,
                    &format!("{label} native"),
                    &recut_tools,
                    circular_bite_locality,
                ) {
                    Ok(f) => {
                        accepted = Some(EdgeModResult::single(f));
                        break;
                    }
                    Err(reason) => {
                        part_failures.push(reason);
                    }
                }
            }
        }

        if accepted.is_none() && has_rect_circle_recipe {
            if let Some(region) = sketch_region {
                match edge_mod_rect_circle_precut_fallback(
                    region,
                    &part,
                    selection,
                    dist,
                    crate::sketch::CornerKind::Fillet,
                    &reference_mesh,
                ) {
                    Ok(fallback) => accepted = Some(fallback),
                    Err(reason) => part_failures.push(reason),
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
    body.parts = next;
    if applied {
        body.pristine =
            (can_use_pristine && !next_pristine.indices.is_empty()).then_some(next_pristine);
        body.sketch_source = None;
    } else {
        // Surface the kernel's actual reason (radius too large, edge not found on
        // an adjacent face, non-blendable wedge, …) instead of a generic guess.
        let reason = last_err.unwrap_or_else(|| "the edge is no longer on the body".to_string());
        warnings.push(format!(
            "Fillet '{mod_id}': the edge couldn't be rounded ({reason}), so the \
             body was left unchanged."
        ));
    }
}

pub(crate) fn edge_mod_try_native_fillet(
    reference_mesh: &MockMesh,
    original_part: &KernelSolid,
    fillet_part: &KernelSolid,
    selection: &EdgeModSelection,
    dist: f32,
    label: &str,
    recut_tools: &[KernelSolid],
    circular_bite_locality: Option<CircularBiteLocality<'_>>,
) -> Result<KernelSolid, String> {
    let edge = &selection.active_edge;
    let mut failures = Vec::new();
    for (suffix, p0, p1) in [("", edge.p0, edge.p1), (" reversed", edge.p1, edge.p0)] {
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
    let edge = &selection.active_edge;
    let mut applied = false;
    let mut last_err: Option<String> = None;
    let mut next: Vec<KernelSolid> = Vec::with_capacity(body.parts.len());
    let mut next_pristine = MockMesh::empty();
    let mut can_use_pristine = true;
    let reference_mesh = edge_mod_reference_mesh(body);
    let sketch_source = body.sketch_source.clone();
    let recut_tools = body.cut_tools.clone();
    for (part_index, part) in body.parts.drain(..).enumerate() {
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
        let native_only = edge_mod_native_only(selection);
        let prefer_rect_circle_recipe = !native_only
            && sketch_region
                .and_then(|region| region.rect_circle.as_ref())
                .is_some();
        if prefer_rect_circle_recipe {
            if let Some(region) = sketch_region {
                match edge_mod_rect_circle_precut_fallback(
                    region,
                    &part,
                    selection,
                    dist,
                    crate::sketch::CornerKind::Chamfer,
                    &reference_mesh,
                ) {
                    Ok(fallback) => accepted = Some(fallback),
                    Err(reason) => part_failures.push(reason),
                }
            }
        }
        let native_reason = if accepted.is_none() {
            match crate::mock_kernel::chamfer_edge(&part, edge.p0, edge.p1, dist) {
                Ok(chamfered) => match edge_mod_accept_candidate_or_recut(
                    &reference_mesh,
                    &part,
                    chamfered,
                    &recut_tools,
                    circular_bite_locality,
                ) {
                    Ok(chamfered) => {
                        match edge_mod_reject_unhealthy_native_curve_result(selection, &chamfered) {
                            Ok(()) => {
                                accepted = Some(EdgeModResult::single(chamfered));
                                None
                            }
                            Err(reason) => Some(format!("native result rejected: {reason}")),
                        }
                    }
                    Err(reason) => Some(format!("native result rejected: {reason}")),
                },
                Err(reason) => Some(format!("native failed: {reason}")),
            }
        } else {
            None
        };
        if accepted.is_none() {
            if let Some(reason) = native_reason.clone() {
                part_failures.push(reason);
            }
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
                let candidate =
                    crate::mock_kernel::chamfer_edge(alternate_part, edge.p0, edge.p1, dist);
                match candidate {
                    Ok(chamfered) => {
                        match edge_mod_accept_candidate_or_recut(
                            &reference_mesh,
                            &part,
                            chamfered,
                            &recut_tools,
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

        if accepted.is_none() && !native_only && !prefer_rect_circle_recipe {
            if let Some(region) = sketch_region {
                match edge_mod_rect_circle_precut_fallback(
                    region,
                    &part,
                    selection,
                    dist,
                    crate::sketch::CornerKind::Chamfer,
                    &reference_mesh,
                ) {
                    Ok(fallback) => accepted = Some(fallback),
                    Err(reason) => part_failures.push(reason),
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
    body.parts = next;
    if applied {
        body.pristine =
            (can_use_pristine && !next_pristine.indices.is_empty()).then_some(next_pristine);
        body.sketch_source = None;
    } else {
        let reason = last_err.unwrap_or_else(|| "the edge is no longer on the body".to_string());
        warnings.push(format!(
            "Chamfer '{mod_id}': the edge couldn't be beveled ({reason}), so the \
             body was left unchanged."
        ));
    }
}

pub(crate) fn edge_mod_rect_circle_precut_fallback(
    region: &SketchExtrudeRegionSource,
    original_part: &KernelSolid,
    selection: &EdgeModSelection,
    dist: f32,
    kind: crate::sketch::CornerKind,
    reference_mesh: &MockMesh,
) -> Result<EdgeModResult, String> {
    let edge = &selection.active_edge;
    let (base, circle_cutter) = if let Some(canonical) = &region.rect_circle {
        (canonical.base.clone(), canonical.cutter.clone())
    } else if let Some((base, circle_cutter)) =
        crate::mock_kernel::rect_minus_circle_region_base_and_cutter(
            &region.boundary,
            &region.holes,
            region.depth,
            &region.cs,
        )
    {
        (base, circle_cutter)
    } else {
        return Err("pre-cut circular-bite fallback did not match this sketch region".to_string());
    };
    let mut circle_cutters = vec![("exact circle", circle_cutter)];
    if let Some((_base, grown_cutter)) =
        crate::mock_kernel::rect_minus_circle_region_base_and_grown_cutter(
            &region.boundary,
            &region.holes,
            region.depth,
            &region.cs,
            CUT_WALL_GROW,
        )
    {
        circle_cutters.push(("grown circle", grown_cutter));
    }
    if let Some((_base, faceted_cutter)) =
        crate::mock_kernel::rect_minus_circle_region_base_and_faceted_cutter(
            &region.boundary,
            &region.holes,
            region.depth,
            &region.cs,
            0.0,
        )
    {
        circle_cutters.push(("faceted circle", faceted_cutter));
    }
    if let Some((_base, faceted_grown_cutter)) =
        crate::mock_kernel::rect_minus_circle_region_base_and_faceted_cutter(
            &region.boundary,
            &region.holes,
            region.depth,
            &region.cs,
            CUT_WALL_GROW,
        )
    {
        circle_cutters.push(("faceted grown circle", faceted_grown_cutter));
    }

    let split_base = split_rect_base_for_edge(region, edge).unwrap_or_else(|| base.clone());
    let fillet = matches!(kind, crate::sketch::CornerKind::Fillet);
    let mut failures = Vec::new();

    // Prefer the Fusion-style construction recipe for sketch circular bites:
    // round/bevel the reconstructed base box edge first, then re-cut the analytic
    // cylinder. Native edge mods on the already-cut body are kept as fallbacks
    // because they can solve nearby cases, but they should not beat the canonical
    // box-minus-cylinder route for an eligible rectangle+circle sketch.
    let circular_bite_locality = Some(CircularBiteLocality {
        region,
        selection,
        dist,
        kind,
    });
    let try_circle_recuts =
        |stage: &str, edge_cut_base: &KernelSolid, failures: &mut Vec<String>| {
            for (cutter_label, cutter) in &circle_cutters {
                match crate::mock_kernel::difference(edge_cut_base, cutter) {
                    Some(result) => {
                        let accepted = if cutter_label.contains("faceted") {
                            edge_mod_accept_candidate_allow_cylinder_rebuild(
                                reference_mesh,
                                original_part,
                                result,
                            )
                            .and_then(|candidate| {
                                if let Some(locality) = circular_bite_locality {
                                    edge_mod_circular_bite_locality(locality, &candidate)?;
                                }
                                Ok(candidate)
                            })
                        } else {
                            edge_mod_accept_candidate_for_edge(
                                reference_mesh,
                                original_part,
                                result,
                                circular_bite_locality,
                            )
                        };
                        match accepted {
                            Ok(result) => return Some(EdgeModResult::single(result)),
                            Err(reason) => failures
                                .push(format!("{stage} {cutter_label} result rejected: {reason}")),
                        }
                    }
                    None => failures.push(format!("{stage} {cutter_label} boolean failed")),
                }
            }
            None
        };

    let split_base_reference = MockMesh::from_solid(&split_base);
    if fillet {
        // The faceted cutter is the reliable construction-aware route for a
        // rectangle-minus-circle bite: round the base edge, then re-cut the circle.
        // Try it before the rolling-ball solver so live previews do not stall here.
        let base_reference = MockMesh::from_solid(&base);
        for (label, cut_part, reference) in [
            (
                "pre-cut split-base cutter",
                &split_base,
                &split_base_reference,
            ),
            ("pre-cut base cutter", &base, &base_reference),
        ] {
            match edge_mod_fallback_cut_against_part(
                cut_part, cut_part, edge, dist, kind, reference, label,
            ) {
                Ok(edge_cut_base) => {
                    if let Some(result) = try_circle_recuts(label, &edge_cut_base, &mut failures) {
                        return Ok(result);
                    }
                }
                Err(reason) => failures.push(reason),
            }
        }
        match edge_mod_fallback_cut_against_part(
            original_part,
            original_part,
            edge,
            dist,
            kind,
            reference_mesh,
            "post-cut cutter ",
        ) {
            Ok(result) => {
                match edge_mod_accept_candidate_for_edge(
                    reference_mesh,
                    original_part,
                    result,
                    circular_bite_locality,
                ) {
                    Ok(result) => return Ok(EdgeModResult::single(result)),
                    Err(reason) => {
                        failures.push(format!("post-cut cutter result rejected: {reason}"))
                    }
                }
            }
            Err(reason) => failures.push(reason),
        }
        return Err(if failures.is_empty() {
            "pre-cut circular-bite fallback produced no candidate".to_string()
        } else {
            format!(
                "pre-cut circular-bite fallback failed: {}",
                failures.join("; ")
            )
        });
    }

    let native = crate::mock_kernel::chamfer_edge(&split_base, edge.p0, edge.p1, dist);
    match native {
        Ok(edge_cut_base) => {
            if let Some(result) = try_circle_recuts("pre-cut native", &edge_cut_base, &mut failures)
            {
                return Ok(result);
            }
        }
        Err(reason) => failures.push(format!("pre-cut native failed: {reason}")),
    }

    match edge_mod_fallback_cut_against_part(
        &split_base,
        &split_base,
        edge,
        dist,
        kind,
        &split_base_reference,
        "pre-cut cutter ",
    ) {
        Ok(edge_cut_base) => {
            if let Some(result) = try_circle_recuts("pre-cut cutter", &edge_cut_base, &mut failures)
            {
                return Ok(result);
            }
        }
        Err(reason) => failures.push(reason),
    }

    let native_on_cut = crate::mock_kernel::chamfer_edge(original_part, edge.p0, edge.p1, dist);
    match native_on_cut {
        Ok(edge_modded) => {
            if let Some(result) = try_circle_recuts("post-cut native", &edge_modded, &mut failures)
            {
                return Ok(result);
            }
        }
        Err(reason) => failures.push(format!("post-cut native failed: {reason}")),
    }

    match edge_mod_fallback_cut_against_part(
        original_part,
        original_part,
        edge,
        dist,
        kind,
        reference_mesh,
        "post-cut cutter ",
    ) {
        Ok(result) => {
            match edge_mod_accept_candidate_for_edge(
                reference_mesh,
                original_part,
                result,
                circular_bite_locality,
            ) {
                Ok(result) => return Ok(EdgeModResult::single(result)),
                Err(reason) => failures.push(format!("post-cut cutter result rejected: {reason}")),
            }
        }
        Err(reason) => failures.push(reason),
    }

    Err(if failures.is_empty() {
        "pre-cut circular-bite fallback produced no candidate".to_string()
    } else {
        format!(
            "pre-cut circular-bite fallback failed: {}",
            failures.join("; ")
        )
    })
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
    let fillet = matches!(kind, crate::sketch::CornerKind::Fillet);
    let robust_overshoot = EDGE_MOD_END_OVERSHOOT;
    let mut failures = Vec::new();
    for (label, grow, end_overshoot) in [
        ("exact cutter", 0.0, 0.0),
        ("grown cutter", EDGE_MOD_GROW, 0.0),
        ("overshot cutter", 0.0, robust_overshoot),
        ("robust cutter", EDGE_MOD_GROW, robust_overshoot),
    ] {
        let Some(cutter) = crate::mock_kernel::edge_corner_cutter(
            edge.p0,
            edge.p1,
            edge.n1,
            edge.n2,
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
            Ok(result) => return Ok(result),
            Err(reason) => failures.push(format!("{label} rejected: {reason}")),
        }
    }

    if fillet {
        for (label, grow, end_overshoot) in [
            ("piecewise exact cutter", 0.0, 0.0),
            ("piecewise grown cutter", EDGE_MOD_GROW, 0.0),
            ("piecewise robust cutter", EDGE_MOD_GROW, robust_overshoot),
        ] {
            let Some(pieces) = crate::mock_kernel::edge_corner_cutter_pieces(
                edge.p0,
                edge.p1,
                edge.n1,
                edge.n2,
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
                Ok(result) => return Ok(result),
                Err(reason) => failures.push(format!("{label} rejected: {reason}")),
            }
        }

        for trim in [0.05, EDGE_MOD_GROW, 0.5] {
            let Some(trimmed) = trimmed_edge_ref(edge, trim) else {
                failures.push(format!(
                    "trimmed piecewise cutter {trim:.2} could not be built"
                ));
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
                failures.push(format!(
                    "trimmed piecewise cutter {trim:.2} could not be built"
                ));
                continue;
            };

            let mut result = cut_part.clone();
            let mut failed = None;
            for cutter in pieces {
                match crate::mock_kernel::difference(&result, &cutter) {
                    Some(next) => result = next,
                    None => {
                        failed = Some(format!("trimmed piecewise cutter {trim:.2} boolean failed"));
                        break;
                    }
                }
            }
            if let Some(reason) = failed {
                failures.push(reason);
                continue;
            }

            match edge_mod_accept_candidate(reference_mesh, original_part, result) {
                Ok(result) => return Ok(result),
                Err(reason) => failures.push(format!(
                    "trimmed piecewise cutter {trim:.2} rejected: {reason}"
                )),
            }
        }
    }

    Err(if failures.is_empty() {
        "no fallback candidate was produced".to_string()
    } else {
        format!("{label_prefix}{}", failures.join("; "))
    })
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
    recut_tools: &[KernelSolid],
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
    for tool in recut_tools {
        match crate::mock_kernel::difference(&candidate, tool) {
            Some(recut) => {
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
            None => failures.push("recut boolean failed".to_string()),
        }
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

pub(crate) fn edge_mod_circular_bite_locality_mesh(
    locality: CircularBiteLocality<'_>,
    candidate_mesh: &MockMesh,
) -> Result<(), String> {
    let region = locality.region;
    let selection = locality.selection;
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

pub(crate) fn edge_mod_accept_candidate_allow_cylinder_rebuild(
    reference_mesh: &MockMesh,
    original_part: &KernelSolid,
    candidate: KernelSolid,
) -> Result<KernelSolid, String> {
    if !edge_mod_keeps_body(original_part, &candidate) {
        return Err("candidate expands outside the original part bounds".to_string());
    }
    if let Some((lo, hi)) = mesh_position_aabb(reference_mesh) {
        if mesh_is_aabb_box(reference_mesh, lo, hi, EDGE_MOD_CONTAINMENT_TOL) {
            return Ok(candidate);
        }
    }
    let candidate_mesh = MockMesh::from_solid(&candidate);
    if candidate_mesh.indices.is_empty() {
        return Err("candidate tessellated to an empty mesh".to_string());
    }
    edge_mod_render_mesh_has_no_cracks(&candidate_mesh)?;
    edge_mod_mesh_stays_inside_reference(
        reference_mesh,
        &candidate_mesh,
        EDGE_MOD_CONTAINMENT_TOL,
        None,
    )?;
    Ok(candidate)
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
