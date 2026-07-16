use super::*;

/// Numerical margin used by blend containment and locality validation.
pub(crate) const EDGE_MOD_GROW: f32 = 0.2;

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
/// Edge modifiers resolve in a single pass, so the background preview and the
/// committed model use the same operation.
pub(crate) fn apply_edge_mod(
    mod_id: &str,
    target: &str,
    edge: &EdgeRef,
    dist: f32,
    kind: crate::sketch::CornerKind,
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
    let selector = crate::document::SemanticSelector::from_edge(edge);
    let requested = edge.topology.as_ref()?;
    if !selector.applies_to_body(&body.id) {
        return None;
    }

    // 1. Exact edge-id match (a stable design id survives an equivalent edit).
    //    A boolean can split one design edge into several fragments that all
    //    carry the same id (a bite cuts the middle out of a rectangle's top
    //    edge), so an id match alone is ambiguous — disambiguate by geometry,
    //    never by enumeration order.
    if let Some(requested_edge_id) = selector.topology.entity_id.as_deref() {
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
                    let pool = if geometric.is_empty() {
                        matches
                    } else {
                        geometric
                    };
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
        let matches: Vec<&crate::mock_kernel::MeshEdgeRef> = mesh
            .edge_refs
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
            .collect();
        // Identity-first, mirroring the exact-id path above: a UNIQUE face-owner
        // pair IS the edge (an edit may have moved it arbitrarily far — a box
        // resize relocates the +x/+y edge by the growth amount, and geometry
        // gates must not veto identity). Only a pair collision (a bite splits an
        // edge into fragments sharing both owners) needs geometry to pick the
        // fragment, falling to the nearest span when everything moved.
        let span_dist = |c: &crate::mock_kernel::MeshEdgeRef| -> f32 {
            let fwd = distance3(c.p0, edge.p0) + distance3(c.p1, edge.p1);
            let rev = distance3(c.p0, edge.p1) + distance3(c.p1, edge.p0);
            fwd.min(rev)
        };
        let best = match matches.len() {
            0 => None,
            1 => Some(matches[0]),
            _ => {
                let geometric: Vec<&crate::mock_kernel::MeshEdgeRef> = matches
                    .iter()
                    .copied()
                    .filter(|c| mesh_candidate_matches_captured_edge(c, edge))
                    .collect();
                let pool = if geometric.is_empty() {
                    matches
                } else {
                    geometric
                };
                pool.into_iter().min_by(|a, b| {
                    span_dist(a)
                        .partial_cmp(&span_dist(b))
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
            }
        };
        best.map(|candidate| edge_ref_from_mesh_candidate(body, candidate, requested))
    };
    body.pristine
        .as_deref()
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
        producer_feature_id: topology.producer_feature_id.clone(),
        source_entity_id: topology.source_entity_id.clone(),
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
pub(crate) fn resolve_face_ref_by_topology(body: &LiveBody, face: &FaceRef) -> Option<FaceRef> {
    resolve_face_on_body(body, face).map(|resolved| resolved.face)
}

/// One authoritative face-resolution result shared by face-driven features.
/// Besides the reattached face it identifies the connected component that owns
/// it, so downstream operations never scan a body's unrelated parts again.
pub(crate) struct ResolvedBodyFace {
    pub(crate) face: FaceRef,
    pub(crate) component_index: usize,
}

pub(crate) fn resolve_face_on_body(body: &LiveBody, face: &FaceRef) -> Option<ResolvedBodyFace> {
    let selector = crate::document::SemanticSelector::from_face(face);
    if let Some(requested) = face.topology.as_ref() {
        if let Some(requested_face_id) = selector.topology.entity_id.as_deref() {
            if !selector.applies_to_body(&body.id) {
                return None;
            }
            // A severing cut can leave SEVERAL faces with the same name — one
            // per lump (both halves of a slotted bar keep `:face:bottom`).
            // Identity narrows to the name; the LUMP is position-based (the
            // same principle as `part_key`): pick the candidate nearest the
            // captured centroid so the reference follows its own lump instead
            // of whichever match enumerates first.
            let pick = |mesh: &MockMesh| -> Option<ResolvedBodyFace> {
                let named: Vec<&crate::mock_kernel::MeshFaceRef> = mesh
                    .face_refs
                    .iter()
                    .filter(|c| topology_face_id(c) == Some(requested_face_id))
                    .collect();
                let exact_component: Vec<&crate::mock_kernel::MeshFaceRef> = requested
                    .component_id
                    .as_deref()
                    .map(|component_id| {
                        named
                            .iter()
                            .copied()
                            .filter(|candidate| {
                                candidate
                                    .topology
                                    .as_ref()
                                    .and_then(|topology| topology.component_id.as_deref())
                                    == Some(component_id)
                            })
                            .collect()
                    })
                    .unwrap_or_default();
                let candidates = if exact_component.is_empty() {
                    named
                } else {
                    exact_component
                };
                candidates
                    .into_iter()
                    .min_by(|a, b| {
                        let d = |c: &crate::mock_kernel::MeshFaceRef| {
                            distance3(c.centroid, face.centroid)
                        };
                        d(a).partial_cmp(&d(b)).unwrap_or(std::cmp::Ordering::Equal)
                    })
                    .and_then(|c| resolved_face_from_mesh_face(body, c, requested))
            };
            let pristine = body.pristine.as_deref().map(|mesh| {
                let mut mesh = mesh.clone();
                crate::mock_kernel::stamp_body_face_components(&mut mesh, &body.id, &body.parts);
                mesh
            });
            if let Some(resolved) = pristine.as_ref().and_then(pick) {
                return Some(resolved);
            }
            return pick(&edge_mod_reference_mesh(body));
        }
    }
    // Unnamed capture (legacy, or a not-yet-named boolean-result face): fall back
    // to the nearest face pointing the same way.
    resolve_face_ref_by_geometry(body, face)
}

pub(crate) fn topology_face_id(face: &crate::mock_kernel::MeshFaceRef) -> Option<&str> {
    face.topology
        .as_ref()
        .and_then(|topology| topology.face_id.as_deref())
}

fn resolved_face_from_mesh_face(
    body: &LiveBody,
    candidate: &crate::mock_kernel::MeshFaceRef,
    requested: &TopologyFaceRef,
) -> Option<ResolvedBodyFace> {
    let topology = candidate
        .topology
        .as_ref()
        .map(|t| TopologyFaceRef {
            body_id: t.body_id.clone().or_else(|| Some(body.id.clone())),
            component_id: t.component_id.clone(),
            topology_version: t.topology_version,
            face_id: t.face_id.clone(),
            surface_kind: t.surface_kind.clone(),
            producer_feature_id: t.producer_feature_id.clone(),
            source_entity_id: t.source_entity_id.clone(),
        })
        .or_else(|| Some(requested.clone()));
    let component_index = component_index_for_face(body, candidate)?;
    Some(ResolvedBodyFace {
        face: FaceRef {
            centroid: candidate.centroid,
            normal: candidate.normal,
            topology,
        },
        component_index,
    })
}

fn resolve_face_ref_by_geometry(body: &LiveBody, face: &FaceRef) -> Option<ResolvedBodyFace> {
    let pick = |mesh: &MockMesh| -> Option<ResolvedBodyFace> {
        mesh.face_refs
            .iter()
            .filter(|c| dot3(c.normal, face.normal) >= 0.7)
            .min_by(|a, b| {
                distance3(a.centroid, face.centroid)
                    .partial_cmp(&distance3(b.centroid, face.centroid))
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .and_then(|c| {
                let requested = c
                    .topology
                    .as_ref()
                    .map(|t| TopologyFaceRef {
                        body_id: t.body_id.clone().or_else(|| Some(body.id.clone())),
                        component_id: t.component_id.clone(),
                        topology_version: t.topology_version,
                        face_id: t.face_id.clone(),
                        surface_kind: t.surface_kind.clone(),
                        producer_feature_id: t.producer_feature_id.clone(),
                        source_entity_id: t.source_entity_id.clone(),
                    })
                    .unwrap_or_default();
                resolved_face_from_mesh_face(body, c, &requested)
            })
    };
    let pristine = body.pristine.as_deref().map(|mesh| {
        let mut mesh = mesh.clone();
        crate::mock_kernel::stamp_body_face_components(&mut mesh, &body.id, &body.parts);
        mesh
    });
    pristine
        .as_ref()
        .and_then(pick)
        .or_else(|| pick(&edge_mod_reference_mesh(body)))
}

fn component_index_for_face(
    body: &LiveBody,
    face: &crate::mock_kernel::MeshFaceRef,
) -> Option<usize> {
    if let Some(component_id) = face
        .topology
        .as_ref()
        .and_then(|topology| topology.component_id.as_deref())
    {
        if let Some(index) = body
            .parts
            .iter()
            .position(|part| crate::mock_kernel::component_id(part) == component_id)
        {
            return Some(index);
        }
    }
    body.parts
        .iter()
        .enumerate()
        .filter_map(|(index, part)| {
            crate::mock_kernel::solid_aabb(part).map(|(min, max)| {
                let distance = (0..3)
                    .map(|axis| {
                        if face.centroid[axis] < min[axis] {
                            (min[axis] - face.centroid[axis]).powi(2)
                        } else if face.centroid[axis] > max[axis] {
                            (face.centroid[axis] - max[axis]).powi(2)
                        } else {
                            0.0
                        }
                    })
                    .sum::<f32>();
                (index, distance)
            })
        })
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(index, _)| index)
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
/// Parts that cannot be filleted are returned unchanged; the caller rejects the
/// feature transaction unless every required part succeeds.
fn edge_mod_native_fillet_all_parts(
    mod_id: &str,
    selection: &EdgeModSelection,
    dist: f32,
    parts: Vec<KernelSolid>,
    reference_mesh: &MockMesh,
    sketch_source: &Option<SketchExtrudeSource>,
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
        let additive = edge_mod_concave_allowance(&part, edge, dist);
        match crate::mock_kernel::chamfer_edge_with_hint(
            &part,
            edge.p0,
            edge.p1,
            edge.curve.as_ref(),
            dist,
        ) {
            Ok(chamfered) => match edge_mod_accept_candidate_for_edge(
                reference_mesh,
                &part,
                chamfered,
                circular_bite_locality,
                additive,
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
                        match edge_mod_accept_candidate_for_edge(
                            reference_mesh,
                            &part,
                            chamfered,
                            circular_bite_locality,
                            additive,
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
    dist: f32,
    body: &mut LiveBody,
    warnings: &mut Vec<String>,
) {
    let reference_mesh = edge_mod_reference_mesh(body);
    let sketch_source = body.sketch_source.clone();
    let original_parts = body.parts.clone();
    let outcome = edge_mod_native_fillet_all_parts(
        mod_id,
        selection,
        dist,
        std::mem::take(&mut body.parts),
        &reference_mesh,
        &sketch_source,
    );
    if outcome.applied && outcome.last_err.is_none() {
        body.parts = outcome.parts;
        body.pristine = outcome.pristine.map(std::sync::Arc::new);
        body.sketch_source = None;
    } else {
        body.parts = original_parts;
        let reason = outcome
            .last_err
            .unwrap_or_else(|| "the edge is no longer on the body".to_string());
        warnings.push(format!(
            "Fillet '{mod_id}': the native operation failed ({reason}), so the body was left unchanged."
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
    circular_bite_locality: Option<CircularBiteLocality<'_>>,
) -> Result<KernelSolid, String> {
    let edge = &selection.active_edge;
    let additive = edge_mod_concave_allowance(original_part, edge, dist);
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
            Ok(f) => match edge_mod_accept_candidate_for_edge(
                reference_mesh,
                original_part,
                f,
                circular_bite_locality,
                additive,
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
    let reference_mesh = edge_mod_reference_mesh(body);
    let sketch_source = body.sketch_source.clone();
    let original_parts = body.parts.clone();
    let outcome = edge_mod_native_chamfer_all_parts(
        selection,
        dist,
        std::mem::take(&mut body.parts),
        &reference_mesh,
        &sketch_source,
    );
    if outcome.applied && outcome.last_err.is_none() {
        body.parts = outcome.parts;
        body.pristine = outcome.pristine.map(std::sync::Arc::new);
        body.sketch_source = None;
    } else {
        body.parts = original_parts;
        let reason = outcome
            .last_err
            .unwrap_or_else(|| "the edge is no longer on the body".to_string());
        warnings.push(format!(
            "Chamfer '{mod_id}': the edge couldn't be beveled ({reason}), so the \
             body was left unchanged."
        ));
    }
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

pub(crate) fn edge_mod_reference_mesh(body: &LiveBody) -> MockMesh {
    let mut mesh = MockMesh::empty();
    for part in &body.parts {
        let Ok(mut part_mesh) = MockMesh::try_from_solid(part) else {
            return body
                .pristine
                .as_ref()
                .map(|mesh| (**mesh).clone())
                .unwrap_or_else(MockMesh::empty);
        };
        crate::mock_kernel::stamp_face_component(&mut part_mesh, &body.id, part);
        mesh.append(part_mesh);
    }
    if !mesh.indices.is_empty() {
        mesh
    } else {
        body.pristine
            .as_ref()
            .map(|mesh| (**mesh).clone())
            .unwrap_or_else(MockMesh::empty)
    }
}

pub(crate) fn edge_mod_accept_candidate_for_edge(
    reference_mesh: &MockMesh,
    original_part: &KernelSolid,
    candidate: KernelSolid,
    circular_bite_locality: Option<CircularBiteLocality<'_>>,
    additive: Option<ConcaveBlendAllowance>,
) -> Result<KernelSolid, String> {
    let (candidate, candidate_mesh) = edge_mod_accept_candidate_with_mesh_gated(
        reference_mesh,
        original_part,
        candidate,
        additive,
    )?;
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
    let candidate_mesh = MockMesh::try_from_solid(candidate)
        .map_err(|reason| format!("candidate display tessellation failed: {reason}"))?;
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
    if matches!(edge.curve.as_ref(), Some(EdgeCurveHint::Circle { .. })) {
        return edge_mod_selected_blend_present_circular(candidate_mesh, edge, dist, kind);
    }
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
    // |f_raw|² = sin²(angle between the normals): only genuinely (anti)parallel
    // normals make the projected in-face direction unusable — the parallel
    // check above already rejects those. (An earlier 0.25 gate here refused any
    // wedge sharper than 30° or blunter than 150°, failing sharp-corner
    // fillets with "could not build an edge-local frame".)
    if length_sq3(f1_raw) <= 1.0e-6 || length_sq3(f2_raw) <= 1.0e-6 {
        return Err(
            "candidate selected-edge locality validation could not build an edge-local frame"
                .to_string(),
        );
    }
    let f1 = normalize3(f1_raw);
    let f2 = normalize3(f2_raw);

    let span_slack = (dist * 0.08).clamp(0.08, 0.5);
    let min_offset = (dist * 0.04).max(0.025);
    // The blend band's tangency feet sit dist/tan(θ/2) along each face (θ =
    // the material wedge angle, cos θ = −n1·n2) — `dist` alone is the 90°
    // special case, and a sharp wedge's band lies entirely beyond it.
    let cos_theta = (-dot3(n1, n2)).clamp(-1.0, 1.0);
    let sin_theta = (1.0 - cos_theta * cos_theta).max(0.0).sqrt();
    let tan_half = (sin_theta / (1.0 + cos_theta).max(1.0e-4)).max(1.0e-3);
    let reach = (dist / tan_half).max(dist);
    let max_offset = reach + EDGE_MOD_GROW + 0.30;
    let required_samples = match kind {
        crate::sketch::CornerKind::Fillet => 3,
        crate::sketch::CornerKind::Chamfer => 1,
    };
    let required_span = (len * 0.25).clamp(0.20, len * 0.75);

    // Result of scanning the candidate mesh with one offset-frame orientation.
    struct BlendScan {
        samples: usize,
        span_hits: usize,
        offset_hits: usize,
        normal_hits: usize,
        min_s: f32,
        max_s: f32,
        normal_bins: std::collections::HashSet<i32>,
    }

    // A convex blend's band lies on the INWARD side of both faces (`f1`,`f2`
    // point into material); a concave (inner-corner, additive) blend's band
    // lies on the OUTWARD side, so its triangles sit at negative `u`,`v` in the
    // convex frame and get culled by the offset gate. Scan both orientations and
    // keep whichever finds the band — a convex candidate has no material on the
    // outward side and a concave one has none on the inward side, so the two
    // frames never cross-accept.
    let scan = |sign: f32| -> BlendScan {
        let f1 = mul3(f1, sign);
        let f2 = mul3(f2, sign);
        let mut out = BlendScan {
            samples: 0,
            span_hits: 0,
            offset_hits: 0,
            normal_hits: 0,
            min_s: f32::INFINITY,
            max_s: f32::NEG_INFINITY,
            normal_bins: std::collections::HashSet::new(),
        };
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
            out.span_hits += 1;

            let u = dot3(rel, f1);
            let v = dot3(rel, f2);
            if u < min_offset || v < min_offset || u > max_offset || v > max_offset {
                continue;
            }
            out.offset_hits += 1;

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
            out.normal_hits += 1;

            out.samples += 1;
            out.min_s = out.min_s.min(s);
            out.max_s = out.max_s.max(s);
            out.normal_bins.insert(bin);
        }
        out
    };

    // Prefer the orientation that actually satisfies the acceptance thresholds.
    let convex = scan(1.0);
    let convex_ok = convex.samples >= required_samples
        && convex.max_s - convex.min_s >= required_span
        && (!matches!(kind, crate::sketch::CornerKind::Fillet) || convex.normal_bins.len() >= 2);
    let scan = if convex_ok { convex } else { scan(-1.0) };

    if scan.samples < required_samples {
        return Err(format!(
            "candidate did not create a {kind:?} surface on the selected edge \
             (span hits {}, offset hits {}, normal hits {})",
            scan.span_hits, scan.offset_hits, scan.normal_hits
        ));
    }

    if scan.max_s - scan.min_s < required_span {
        return Err(format!(
            "candidate {kind:?} surface covered only {:.2}mm of the selected {:.2}mm edge",
            scan.max_s - scan.min_s,
            len
        ));
    }

    if matches!(kind, crate::sketch::CornerKind::Fillet) && scan.normal_bins.len() < 2 {
        return Err(
            "candidate fillet surface on the selected edge did not have rounded normals"
                .to_string(),
        );
    }

    Ok(())
}

/// Circular analogue of the straight-edge presence check: the blend band around
/// a circular rim (a bite arc, a rim fragment) lives within `dist` of the rim
/// circle, offset into the material off BOTH support faces, inside the selected
/// angular span, with normals sweeping between the cap and the wall.
fn edge_mod_selected_blend_present_circular(
    candidate_mesh: &MockMesh,
    edge: &EdgeRef,
    dist: f32,
    kind: crate::sketch::CornerKind,
) -> Result<(), String> {
    let Some(EdgeCurveHint::Circle {
        center,
        axis,
        x_dir,
        radius,
        start,
        end,
        closed,
    }) = edge.curve
    else {
        return Err("circular presence check called without a circle hint".to_string());
    };
    let tau = std::f32::consts::TAU;
    let axis_n = normalize3(axis);
    if length_sq3(axis_n) <= 0.25 || radius <= 1.0e-4 {
        return Err("circular presence check got a degenerate rim frame".to_string());
    }
    // Orthonormal in-plane frame for angles, matching the hint's convention.
    let x_raw = sub3(x_dir, mul3(axis_n, dot3(x_dir, axis_n)));
    if length_sq3(x_raw) <= 1.0e-6 {
        return Err("circular presence check got a degenerate x direction".to_string());
    }
    let x_hat = normalize3(x_raw);
    let y_hat = cross3(axis_n, x_hat);
    // The hint's arc may run in either direction (`end` below `start`), exactly
    // like the kernel's `angle_in_span`: a negative raw span means the arc goes
    // clockwise from `start`. Normalize to "angular distance from `start` along
    // the arc's own direction" so both encodings measure the same fragment.
    let raw_span = end - start;
    let forward = raw_span >= 0.0;
    let span = if closed {
        tau
    } else {
        let mut s = raw_span.abs().rem_euclid(tau);
        if s <= 1.0e-3 {
            s = tau;
        }
        s
    };
    let rel_angle = |theta: f32| -> f32 {
        if forward {
            (theta - start).rem_euclid(tau)
        } else {
            (start - theta).rem_euclid(tau)
        }
    };
    let ang_slack = ((dist * 0.08).clamp(0.08, 0.5) / radius).max(0.02);

    let min_offset = (dist * 0.04).max(0.025);
    let max_offset = dist + EDGE_MOD_GROW + 0.30;
    let mut samples = 0usize;
    let mut min_ang = f32::INFINITY;
    let mut max_ang = f32::NEG_INFINITY;
    let mut normal_bins = std::collections::HashSet::new();

    for tri in candidate_mesh.indices.chunks_exact(3) {
        let a = mesh_vertex_pos6(candidate_mesh, tri[0]);
        let b = mesh_vertex_pos6(candidate_mesh, tri[1]);
        let c = mesh_vertex_pos6(candidate_mesh, tri[2]);
        let p = mul3(add3(add3(a, b), c), 1.0 / 3.0);
        let rel = sub3(p, center);
        let z = dot3(rel, axis_n);
        let radial = sub3(rel, mul3(axis_n, z));
        let rho = length_sq3(radial).sqrt();
        // Offset off BOTH supports (like the straight check's u/v): past the
        // wall radially AND below/above the cap axially — the supports
        // themselves sit at zero on one of the two and are excluded.
        let u = (rho - radius).abs();
        let v = z.abs();
        if u < min_offset || v < min_offset || u > max_offset || v > max_offset {
            continue;
        }

        // Inside the selected arc's angular span (with wrap tolerance).
        let theta = dot3(rel, y_hat).atan2(dot3(rel, x_hat));
        let rel_ang = rel_angle(theta);
        if !closed && rel_ang > span + ang_slack && rel_ang < tau - ang_slack {
            continue;
        }

        // Blend-surface normal: sweeps between the cap normal (|n·axis| = 1)
        // and the wall normal (|n·axis| = 0). Bin the axis component so a
        // fillet must show a genuinely rounded sweep.
        let na = mesh_vertex_normal6(candidate_mesh, tri[0]);
        let nb = mesh_vertex_normal6(candidate_mesh, tri[1]);
        let nc = mesh_vertex_normal6(candidate_mesh, tri[2]);
        let normal = normalize3(mul3(add3(add3(na, nb), nc), 1.0 / 3.0));
        if length_sq3(normal) <= 0.25 {
            continue;
        }
        let axial = dot3(normal, axis_n).abs().clamp(0.0, 1.0);

        samples += 1;
        let clamped = rel_ang.min(span);
        min_ang = min_ang.min(clamped);
        max_ang = max_ang.max(clamped);
        normal_bins.insert((axial * 3.999) as i32);
    }

    let required_samples = match kind {
        crate::sketch::CornerKind::Fillet => 3,
        crate::sketch::CornerKind::Chamfer => 1,
    };
    if samples < required_samples {
        return Err(format!(
            "candidate did not create a {kind:?} band on the selected circular edge \
             ({samples} band samples)"
        ));
    }
    let required_span = (span * 0.25).clamp(0.1, span * 0.75);
    if max_ang - min_ang < required_span {
        return Err(format!(
            "candidate {kind:?} band covered only {:.2} rad of the selected {:.2} rad arc",
            max_ang - min_ang,
            span
        ));
    }
    if matches!(kind, crate::sketch::CornerKind::Fillet) && normal_bins.len() < 2 {
        return Err(
            "candidate fillet band on the selected circular edge did not have rounded normals"
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
    // Same wedge-angle reach as edge_mod_selected_blend_present: the band ends
    // dist/tan(θ/2) along each face, not `dist` (the 90° special case).
    let n1n = normalize3(edge.n1);
    let n2n = normalize3(edge.n2);
    let cos_theta = (-dot3(n1n, n2n)).clamp(-1.0, 1.0);
    let sin_theta = (1.0 - cos_theta * cos_theta).max(0.0).sqrt();
    let tan_half = (sin_theta / (1.0 + cos_theta).max(1.0e-4)).max(1.0e-3);
    let max_offset = (dist / tan_half).max(dist) + EDGE_MOD_GROW + 0.35;

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
    // A CLOSED rim selection (the full circle — a small cylinder's top edge, a
    // bored hole's rim) has no unselected remainder by definition, and its seam
    // endpoint always sits on the loop's bounding-box extreme, which the
    // straight-side classifier below would otherwise mistake for a rect side —
    // fabricating a one-tessellation-chord "unselected span" next to the seam
    // that rejects a perfectly good blend. Bail before classifying.
    let closed_circle = |e: &EdgeRef| {
        matches!(
            e.curve.as_ref(),
            Some(EdgeCurveHint::Circle { closed: true, .. })
        )
    };
    if closed_circle(original) || closed_circle(selected) {
        return Vec::new();
    }
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
    // Same degenerate case when no curve hint survived: a closed-loop selection
    // projects both endpoints onto (nearly) the same point — never a straight
    // rect side.
    if dist2(p0, p1) <= side_eps {
        return Vec::new();
    }

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

/// Permission for a fillet/chamfer candidate to ADD material: granted only
/// when the selected straight edge's material wedge is reflex (a concave
/// inner corner), where the blend fills the corner void instead of carving
/// the corner off. The added geometry must stay within `reach` of the
/// `p0..p1` segment — anything further afield is a fabricated result, not
/// the corner wedge.
#[derive(Clone, Copy)]
pub(crate) struct ConcaveBlendAllowance {
    pub p0: [f32; 3],
    pub p1: [f32; 3],
    pub reach: f32,
}

/// Build the additive allowance for a selection, or `None` when the edge is
/// not a straight concave edge (the ordinary subtractive gates then apply).
pub(crate) fn edge_mod_concave_allowance(
    part: &KernelSolid,
    edge: &EdgeRef,
    dist: f32,
) -> Option<ConcaveBlendAllowance> {
    if !matches!(edge.curve.as_ref(), None | Some(EdgeCurveHint::Line)) {
        return None;
    }
    crate::mock_kernel::edge_wedge_is_concave(part, edge.p0, edge.p1).then_some(
        ConcaveBlendAllowance {
            p0: edge.p0,
            p1: edge.p1,
            // The wedge's farthest point from the edge is the contact line at
            // dist/tan(θ_void/2); ×4 covers void wedges down to ~28°.
            reach: (dist * 4.0).max(1.0),
        },
    )
}

fn point_segment_distance_f32(p: [f32; 3], a: [f32; 3], b: [f32; 3]) -> f32 {
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    let len2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
    let t = if len2 <= f32::EPSILON {
        0.0
    } else {
        ((ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / len2).clamp(0.0, 1.0)
    };
    let d = [
        p[0] - (a[0] + ab[0] * t),
        p[1] - (a[1] + ab[1] * t),
        p[2] - (a[2] + ab[2] * t),
    ];
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

/// Additive counterpart of `edge_mod_mesh_stays_inside_reference`: the
/// candidate may leave the reference *solid* (a concave blend fills former
/// void) but must stay within the reference bounds, and every point that
/// escaped the reference solid must be local to the blended edge.
fn edge_mod_additive_mesh_is_local(
    reference_mesh: &MockMesh,
    candidate_mesh: &MockMesh,
    allow: &ConcaveBlendAllowance,
    tol: f32,
) -> Result<(), String> {
    for (i, v) in candidate_mesh.vertices.chunks_exact(6).enumerate() {
        let p = [v[0], v[1], v[2]];
        if point_inside_triangle_mesh(reference_mesh, p, tol) {
            continue;
        }
        let d = point_segment_distance_f32(p, allow.p0, allow.p1);
        if d > allow.reach + tol {
            return Err(format!(
                "candidate vertex {i} at [{:.3}, {:.3}, {:.3}] adds material {d:.3} from the \
                 concave edge (allowed {:.3})",
                p[0], p[1], p[2], allow.reach
            ));
        }
    }
    Ok(())
}

pub(crate) fn edge_mod_accept_candidate_with_mesh_gated(
    reference_mesh: &MockMesh,
    original_part: &KernelSolid,
    candidate: KernelSolid,
    additive: Option<ConcaveBlendAllowance>,
) -> Result<(KernelSolid, Option<MockMesh>), String> {
    if !edge_mod_keeps_body(original_part, &candidate)? {
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
    let candidate_mesh = MockMesh::try_from_solid(&candidate)
        .map_err(|reason| format!("candidate display tessellation failed: {reason}"))?;
    if candidate_mesh.indices.is_empty() {
        return Err("candidate tessellated to an empty mesh".to_string());
    }
    let mut preserved_cylinder_faces =
        crate::mock_kernel::preserved_cylindrical_face_ids(original_part, &candidate);
    // Analytic blend bands (torus fillet / cone chamfer around a circular rim)
    // are subtractive by construction and bounds-guarded in the kernel, but
    // their chorded triangles can sag inside the reference mesh's own chorded
    // wall by more than the containment tolerance on coarse hoops — exempt them
    // exactly like preserved cylindrical walls.
    for (i, face) in candidate.shell().faces().iter().enumerate() {
        if matches!(
            face.surface(),
            Some(openrcad::geom::GeomSurface::Torus(_))
                | Some(openrcad::geom::GeomSurface::Cone(_))
        ) {
            preserved_cylinder_faces.insert(i as u32);
        }
    }
    edge_mod_render_mesh_adds_no_cracks(
        &candidate_mesh,
        mock_mesh_crack_edge_count(reference_mesh),
    )?;
    if let Some(allow) = additive.as_ref() {
        // Concave blend: material is ADDED in the corner void, so strict
        // containment in the reference solid cannot hold. Gate on the
        // reference bounds plus locality of everything that escaped.
        edge_mod_mesh_stays_inside_reference_bounds(
            reference_mesh,
            &candidate_mesh,
            EDGE_MOD_CONTAINMENT_TOL,
        )?;
        edge_mod_additive_mesh_is_local(
            reference_mesh,
            &candidate_mesh,
            allow,
            EDGE_MOD_CONTAINMENT_TOL,
        )?;
    } else {
        edge_mod_mesh_stays_inside_reference(
            reference_mesh,
            &candidate_mesh,
            EDGE_MOD_CONTAINMENT_TOL,
            Some(&preserved_cylinder_faces),
        )?;
    }
    Ok((candidate, Some(candidate_mesh)))
}

/// Reject a candidate whose render mesh has more crack (single-triangle) edges
/// than `allowed`. Callers pass the REFERENCE mesh's own crack count: a body
/// whose pre-fillet tessellation already carries a boolean sliver must stay
/// filletable — the gate exists to catch cracks the blend itself introduces.
pub(crate) fn edge_mod_render_mesh_adds_no_cracks(
    mesh: &MockMesh,
    allowed: usize,
) -> Result<(), String> {
    let cracks = mock_mesh_crack_edge_count(mesh);
    if cracks <= allowed {
        Ok(())
    } else {
        Err(format!(
            "candidate render mesh has {cracks} crack edges (reference has {allowed})"
        ))
    }
}

pub(crate) fn mock_mesh_crack_edge_count(mesh: &MockMesh) -> usize {
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
    edges.values().filter(|&&count| count == 1).count()
}

pub(crate) fn edge_mod_reject_unhealthy_native_curve_result(
    selection: &EdgeModSelection,
    candidate: &KernelSolid,
) -> Result<(), String> {
    if !edge_mod_native_only(selection) {
        return Ok(());
    }
    let mesh = MockMesh::try_from_solid(candidate)
        .map_err(|reason| format!("candidate display tessellation failed: {reason}"))?;
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
    let candidate_mesh = MockMesh::try_from_solid(candidate)
        .map_err(|reason| format!("candidate display tessellation failed: {reason}"))?;
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

/// Concavity of a straight edge from the body's display mesh alone — the GUI
/// holds the `MockMesh`, not the `KernelSolid`, so this mirrors the kernel's
/// `planar_edge_material_wedge_is_concave` on tessellated geometry. Probes just
/// off the edge midpoint along ±normalize(n1 − n2); when BOTH samples land
/// inside the solid the wedge between the two faces is filled — a reflex
/// (inner-corner) edge whose blend ADDS material. Returns `None` when the two
/// samples disagree (an ordinary edge) or the frame is degenerate.
pub fn edge_wedge_is_concave_mesh(
    mesh: &MockMesh,
    p0: [f32; 3],
    p1: [f32; 3],
    n1: [f32; 3],
    n2: [f32; 3],
) -> Option<bool> {
    let mid = [
        (p0[0] + p1[0]) * 0.5,
        (p0[1] + p1[1]) * 0.5,
        (p0[2] + p1[2]) * 0.5,
    ];
    let d = sub3(n1, n2);
    if length_sq3(d) <= 1.0e-9 {
        return None;
    }
    let dir = normalize3(d);
    let edge_len = length_sq3(sub3(p1, p0)).sqrt();
    let eps = (0.05 * edge_len).clamp(1.0e-3, 0.5);
    // The mesh is a chorded approximation, so the inside test needs a surface
    // tolerance comparable to the tessellation gap.
    let tol = 0.02;
    let plus = point_inside_triangle_mesh(mesh, add3(mid, mul3(dir, eps)), tol);
    let minus = point_inside_triangle_mesh(mesh, sub3(mid, mul3(dir, eps)), tol);
    match (plus, minus) {
        (true, true) => Some(true),
        (false, false) => Some(false),
        _ => None,
    }
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
pub(crate) fn edge_mod_keeps_body(
    part: &KernelSolid,
    result: &KernelSolid,
) -> Result<bool, String> {
    match (
        crate::mock_kernel::solid_aabb(part),
        crate::mock_kernel::solid_aabb(result),
    ) {
        (Some(p), Some(r)) => {
            // Must not extend past the part — a subtraction can only remove. The
            // slack covers the cutter's own end-overshoot/grow and tessellation
            // noise; real garbage flares out far more than this.
            const SLACK: f32 = 0.3;
            let within = (0..3).all(|k| r.0[k] >= p.0[k] - SLACK && r.1[k] <= p.1[k] + SLACK);
            if !within {
                return Ok(false);
            }
            // Must keep the bulk of the part. TRUE enclosed volume, not AABB
            // volume: a large-radius fillet on a SHARP sliver corner legitimately
            // shortens the part's AABB by half while removing little material —
            // the old AABB-volume proxy rejected exactly those fillets. A coarse
            // tessellation is plenty accurate for a 50% ratio test.
            let pv = solid_volume_estimate(part)
                .map_err(|reason| format!("source volume validation failed: {reason}"))?;
            let result_volume = solid_volume_estimate(result)
                .map_err(|reason| format!("candidate volume validation failed: {reason}"))?;
            Ok(pv <= 1.0e-6 || result_volume >= pv * 0.5)
        }
        (None, None) => Ok(true),
        _ => Ok(false),
    }
}

/// Enclosed volume of `solid` from a coarse tessellation via the divergence
/// theorem (⅙·Σ p0·(p1×p2) over triangles, absolute value — winding-agnostic).
/// Accuracy is bounded by the chord error, which is far tighter than the 50%
/// bulk gate this feeds.
fn solid_volume_estimate(solid: &KernelSolid) -> Result<f64, String> {
    let mesh = openrcad::mesh::tessellate_checked_for_display_with_policy_and_cancel(
        solid,
        0.5,
        std::f64::consts::PI,
        &openrcad::foundation::TolerancePolicy::STANDARD,
        &openrcad::foundation::NeverCancelled,
    )
    .map_err(|error| error.to_string())?;
    let mut vol6 = 0.0f64;
    for tri in &mesh.triangles {
        let a = mesh.vertices[tri[0] as usize];
        let b = mesh.vertices[tri[1] as usize];
        let c = mesh.vertices[tri[2] as usize];
        vol6 += a.x() * (b.y() * c.z() - b.z() * c.y())
            + a.y() * (b.z() * c.x() - b.x() * c.z())
            + a.z() * (b.x() * c.y() - b.y() * c.x());
    }
    Ok((vol6 / 6.0).abs())
}
