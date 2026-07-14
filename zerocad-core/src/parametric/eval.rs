use super::*;

impl ParametricGraph {
    pub fn new() -> Self {
        let mut pg = Self {
            graph: DiGraph::new(),
            sketch_face_refs: HashMap::new(),
            sketch_datum_refs: HashMap::new(),
            sketch_face_boundaries: HashMap::new(),
            node_map: HashMap::new(),
            region_cache: RefCell::new(HashMap::new()),
            pending_face_reattach: RefCell::new(FaceReattach::default()),
            eval_cache: RefCell::new(std::sync::Arc::new(EvalCache::default())),
        };
        pg.bootstrap_origin();
        pg
    }

    /// Clone only authoritative document state, dropping all derived caches.
    /// Undo/autosave paths must use this instead of `Clone`: an evaluation cache
    /// can contain many checkpoint bodies and meshes that serde would skip only
    /// *after* paying to deep-clone them.
    pub fn clone_document(&self) -> Self {
        let mut graph = Self {
            graph: self.graph.clone(),
            sketch_face_refs: self.sketch_face_refs.clone(),
            sketch_datum_refs: self.sketch_datum_refs.clone(),
            sketch_face_boundaries: self.sketch_face_boundaries.clone(),
            node_map: HashMap::new(),
            region_cache: RefCell::new(HashMap::new()),
            pending_face_reattach: RefCell::new(FaceReattach::default()),
            eval_cache: RefCell::new(std::sync::Arc::new(EvalCache::default())),
        };
        graph.rebuild_node_map();
        graph
    }

    /// Copy the current derived evaluator state for transfer between graph
    /// clones or optional hydrated document caches.
    pub fn evaluation_cache_snapshot(&self) -> EvaluationCacheSnapshot {
        EvaluationCacheSnapshot {
            cache: self.eval_cache.borrow().clone(),
        }
    }

    /// Seed this graph with a cache created from an equivalent recipe. Prefix
    /// keys are validated lazily by the next evaluation, so stale data can
    /// never be applied to a changed model.
    pub fn install_evaluation_cache(&self, snapshot: EvaluationCacheSnapshot) {
        *self.eval_cache.borrow_mut() = snapshot.cache;
    }

    /// Add base coordinate system planes
    fn bootstrap_origin(&mut self) {
        let origin = FeatureNode {
            id: "origin".to_string(),
            name: "Base Origin".to_string(),
            feature: FeatureType::Origin,
        };
        let idx = self.graph.add_node(origin);
        self.node_map.insert("origin".to_string(), idx);
    }

    /// Add a feature node to the tree
    pub fn add_feature(&mut self, node: FeatureNode) -> NodeIndex {
        let id = node.id.clone();
        let idx = self.graph.add_node(node);
        self.node_map.insert(id, idx);
        idx
    }

    /// Establish a directional dependency (e.g. Extrude depends on Sketch)
    pub fn add_dependency(&mut self, parent_id: &str, child_id: &str) {
        // A single New Body feature may emit several independently selectable
        // runtime bodies. Downstream features target the exact output id, but
        // the dependency graph still points to the one feature that owns it.
        let parent_feature_id = body_output_owner_id(parent_id);
        if let (Some(parent_idx), Some(child_idx)) = (
            self.resolve_node(parent_feature_id),
            self.resolve_node(child_id),
        ) {
            self.graph.add_edge(parent_idx, child_idx, ());
        }
    }

    /// Rebuild the id → `NodeIndex` lookup from the live graph. The map is
    /// `#[serde(skip)]`, so a deserialized graph (undo/redo snapshot, `.zcad`
    /// load) starts with it empty, and petgraph's `remove_node` swap-moves the
    /// last node's index — both leave stale entries that would silently break
    /// `add_dependency` for later features (an extrude that can't find its
    /// sketch builds no body).
    pub fn rebuild_node_map(&mut self) {
        self.node_map.clear();
        for idx in self.graph.node_indices() {
            self.node_map.insert(self.graph[idx].id.clone(), idx);
        }
    }

    /// Resolve a feature id to its current index, self-healing a missing or
    /// stale `node_map` entry (see [`Self::rebuild_node_map`]).
    fn resolve_node(&mut self, id: &str) -> Option<NodeIndex> {
        if let Some(&idx) = self.node_map.get(id) {
            if self.graph.node_weight(idx).map(|n| n.id.as_str()) == Some(id) {
                return Some(idx);
            }
        }
        self.rebuild_node_map();
        self.node_map.get(id).copied()
    }

    /// Remove a feature node by id, keeping `node_map` consistent (petgraph
    /// swap-removes, which changes another node's index). Returns whether the
    /// node existed.
    pub fn remove_feature(&mut self, id: &str) -> bool {
        let Some(idx) = self.resolve_node(id) else {
            return false;
        };
        self.graph.remove_node(idx);
        // Drop the side-map entries keyed by this id — feature ids can be
        // reused across sessions (the GUI counter restarts on load), and a
        // stale face ref/boundary would silently attach to the newcomer.
        self.sketch_face_refs.remove(id);
        self.sketch_datum_refs.remove(id);
        self.sketch_face_boundaries.remove(id);
        self.rebuild_node_map();
        true
    }

    /// Commit the face-reattachment updates queued by the last evaluation of
    /// this graph instance (see [`ParametricGraph::pending_face_reattach`]):
    /// refreshed sketch-on-face outlines replace their
    /// `sketch_face_boundaries` snapshots, and the affected extrudes' stored
    /// `region_indices` are rewritten to the remapped values — atomically, so
    /// the invariant "indices are in the space of drawn ⊕ stored-boundary
    /// regions" holds at all times. Returns whether anything changed. A
    /// self-healing normalization (like node-map resync), not a user edit —
    /// callers should NOT push an undo step for it.
    pub fn apply_face_reattach(&mut self) -> bool {
        let pending = std::mem::take(&mut *self.pending_face_reattach.borrow_mut());
        self.apply_face_reattach_updates(pending)
    }

    /// Apply reattachment updates produced by an evaluation of a graph clone.
    /// The GUI's background evaluator uses this only for the still-current
    /// generation, so updates can never cross from a stale model revision.
    pub fn apply_face_reattach_updates(&mut self, pending: FaceReattach) -> bool {
        if pending.boundaries.is_empty() && pending.region_indices.is_empty() {
            return false;
        }
        for (sketch_id, boundary) in pending.boundaries {
            self.sketch_face_boundaries.insert(sketch_id, boundary);
        }
        for (sketch_id, plane) in pending.planes {
            if let Some(idx) = self.resolve_node(&sketch_id) {
                if let FeatureType::Sketch { cs, .. } = &mut self.graph[idx].feature {
                    *cs = plane;
                }
            }
        }
        for (node_id, indices) in pending.region_indices {
            if let Some(idx) = self.resolve_node(&node_id) {
                if let FeatureType::Extrude { region_indices, .. } = &mut self.graph[idx].feature {
                    *region_indices = indices;
                }
            }
        }
        true
    }

    /// Ids of `id`'s direct parents that are sketches and feed **only** `id`.
    /// When the GUI deletes a body it uses this to reveal the sketch that was
    /// auto-hidden when the body consumed it.
    pub fn sole_sketch_parents(&self, id: &str) -> Vec<String> {
        let Some(idx) = self.graph.node_indices().find(|i| self.graph[*i].id == id) else {
            return Vec::new();
        };
        self.graph
            .neighbors_directed(idx, petgraph::Direction::Incoming)
            .filter(|p| matches!(self.graph[*p].feature, FeatureType::Sketch { .. }))
            .filter(|p| {
                self.graph
                    .neighbors_directed(*p, petgraph::Direction::Outgoing)
                    .all(|c| c == idx)
            })
            .map(|p| self.graph[p].id.clone())
            .collect()
    }

    /// Clear all nodes except base Origin
    pub fn clear(&mut self) {
        self.graph.clear();
        self.node_map.clear();
        self.bootstrap_origin();
    }

    /// Every named variable in the document, mapped to its value in the **base
    /// unit (mm)** — the form expression-driven dimensions resolve against. All
    /// variable sets contribute (visibility is a rendering concern, not a
    /// definition one); a duplicate name keeps the last one seen.
    pub fn variable_map(&self) -> HashMap<String, f64> {
        let mut map = HashMap::new();
        for idx in self.graph.node_indices() {
            if let FeatureType::VariableSet { variables } = &self.graph[idx].feature {
                for v in variables {
                    if !v.name.trim().is_empty() {
                        map.insert(v.name.clone(), v.value_in_base());
                    }
                }
            }
        }
        map
    }

    /// Perform a topological sort of the history graph and evaluate the 3D model.
    pub fn evaluate(&self) -> Result<MockMesh, String> {
        self.evaluate_with_hidden(&std::collections::HashSet::new())
    }

    /// Evaluate the model, skipping any body whose node id is in `hidden`.
    /// Returns one combined mesh (faces stay distinct via rebased face ids).
    pub fn evaluate_with_hidden(
        &self,
        hidden: &std::collections::HashSet<String>,
    ) -> Result<MockMesh, String> {
        let bodies = self.evaluate_bodies(hidden)?;
        let mut final_mesh = MockMesh::empty();
        for (_id, mesh) in bodies {
            final_mesh.append(mesh);
        }
        Ok(final_mesh)
    }

    /// Evaluate the model into one mesh **per solid body**, tagged with that
    /// body's node id. Each mesh keeps its own face ids so the viewport can
    /// select individual faces, edges and points of a body. Sketches are not
    /// meshed; hiding only affects solid bodies.
    ///
    /// Bodies are processed in **creation order** (the monotonic suffix of each
    /// node id) rather than topological order, because join/cut extrudes act on
    /// whatever bodies already exist at their point in history — an ordering a
    /// pure topo-sort doesn't capture (a box and a cut extrude have no
    /// dependency edge between them). Sketch → extrude order is still honoured
    /// because a sketch's id is always allocated before the extrude that uses it.
    pub fn evaluate_bodies(
        &self,
        hidden: &std::collections::HashSet<String>,
    ) -> Result<Vec<(String, MockMesh)>, String> {
        self.evaluate_bodies_with_warnings(hidden)
            .map(|(bodies, _warnings)| bodies)
    }

    /// Like [`evaluate_bodies`], but also returns any **non-fatal** warnings
    /// raised while assembling the model — e.g. a Cut or Join whose boolean the
    /// solver could not resolve, so the feature was left unapplied. These are
    /// results the user did *not* ask for, so the GUI surfaces them rather than
    /// quietly come out wrong. Successful coplanarity fallbacks are **not**
    /// warned about: they produce the geometry the user drew, so they're noise.
    ///
    /// Bodies are processed in **creation order** (the monotonic suffix of each
    /// node id) rather than topological order, because join/cut extrudes act on
    /// whatever bodies already exist at their point in history — an ordering a
    /// pure topo-sort doesn't capture (a box and a cut extrude have no
    /// dependency edge between them). Sketch → extrude order is still honoured
    /// because a sketch's id is always allocated before the extrude that uses it.
    pub fn evaluate_bodies_with_warnings(
        &self,
        hidden: &std::collections::HashSet<String>,
    ) -> Result<(Vec<(String, MockMesh)>, Vec<String>), String> {
        self.evaluate_bodies_inner(hidden, false)
    }

    /// Like [`evaluate_bodies_with_warnings`], but also returns a per-feature
    /// [`FeatureStatus`] list (creation order). Each feature that raised a warning
    /// while being applied is reported [`ResolutionState::Unresolved`] with that
    /// message; the rest are [`ResolutionState::Resolved`]. This is the structured
    /// form of the warning list, so a caller (the GUI history tree) can mark *which*
    /// feature failed to reattach instead of only showing a global count.
    pub fn evaluate_bodies_with_status(
        &self,
        hidden: &std::collections::HashSet<String>,
    ) -> Result<(Vec<(String, MockMesh)>, Vec<String>, Vec<FeatureStatus>), String> {
        let (live, warnings) = self.build_live(hidden, false)?;
        // `build_live` just refreshed the checkpoint cache; its final checkpoint
        // holds the cumulative per-feature statuses.
        let statuses = self
            .eval_cache
            .borrow()
            .checkpoints
            .iter()
            .rev()
            .flatten()
            .next()
            .map(|cp| cp.statuses.clone())
            .unwrap_or_default();
        Ok((tessellate_bodies(live), warnings, statuses))
    }

    /// Cancellable, timed evaluator used by interactive schedulers. Existing
    /// `evaluate_bodies*` methods remain the compatibility/final-quality API.
    pub fn evaluate_request(
        &self,
        hidden: &std::collections::HashSet<String>,
        quality: EvaluationQuality,
        cancellation: &EvaluationCancellation,
    ) -> Result<EvaluationOutput, EvaluationError> {
        let total_started = std::time::Instant::now();
        let draft = quality == EvaluationQuality::Interactive;
        let run_inner = || {
            if cancellation.is_cancelled() {
                return Err(EvaluationError::Cancelled);
            }
            let build_started = std::time::Instant::now();
            let (live, warnings) = self
                .build_live_with_cancel(hidden, draft, Some(cancellation))
                .map_err(|message| {
                    if cancellation.is_cancelled() {
                        EvaluationError::Cancelled
                    } else {
                        EvaluationError::Failed(message)
                    }
                })?;
            let build = build_started.elapsed();
            let statuses = self
                .eval_cache
                .borrow()
                .checkpoints
                .iter()
                .rev()
                .flatten()
                .next()
                .map(|cp| cp.statuses.clone())
                .unwrap_or_default();
            let diagnostics = statuses
                .iter()
                .filter_map(|status| {
                    status.reason().map(|message| EvaluationDiagnostic {
                        feature_id: status.feature_id.clone(),
                        operation: "feature evaluation".to_string(),
                        failure_class: "unresolved_feature".to_string(),
                        fallback: Some("kept last valid body".to_string()),
                        severity: DiagnosticSeverity::Warning,
                        message: message.to_string(),
                    })
                })
                .collect();
            let feature_timings = self
                .eval_cache
                .borrow()
                .checkpoints
                .iter()
                .enumerate()
                .filter_map(|(i, checkpoint)| {
                    checkpoint.as_ref().map(|checkpoint| FeatureTiming {
                        feature_id: self
                            .body_nodes_in_creation_order()
                            .get(i)
                            .map(|idx| self.graph[*idx].id.clone())
                            .unwrap_or_default(),
                        duration: checkpoint.feature_duration,
                    })
                })
                .collect();
            let tess_started = std::time::Instant::now();
            let bodies = tessellate_bodies_with_cancel(live, Some(cancellation))?;
            let tessellation = tess_started.elapsed();
            let face_reattach = std::mem::take(&mut *self.pending_face_reattach.borrow_mut());
            Ok(EvaluationOutput {
                bodies,
                warnings,
                statuses,
                diagnostics,
                face_reattach,
                timings: EvaluationTimings {
                    total: total_started.elapsed(),
                    build,
                    tessellation,
                },
                feature_timings,
                cache_snapshot: self.evaluation_cache_snapshot(),
            })
        };
        let run = || crate::mock_kernel::with_kernel_cancellation(cancellation.clone(), run_inner);
        if draft {
            crate::mock_kernel::with_preview_tess(run)
        } else {
            run()
        }
    }

    /// **Draft** evaluation for live previews (a fillet drag, an extrude
    /// preview): identical to [`evaluate_bodies_with_warnings`] except every 3D
    /// fillet uses the fast **faceted** cutter instead of the analytic-arc one.
    /// The arc cutter's boolean is ~50× slower (truck's curve–surface
    /// intersection), so re-solving it on every drag frame freezes the UI. The
    /// committed model still rebuilds with the arc cutter via the non-draft path,
    /// which runs only once per edit — so the user drags a fast faceted preview
    /// and lands on the smooth single-face result.
    pub fn evaluate_bodies_draft(
        &self,
        hidden: &std::collections::HashSet<String>,
    ) -> Result<Vec<(String, MockMesh)>, String> {
        self.evaluate_bodies_inner(hidden, true)
            .map(|(bodies, _warnings)| bodies)
    }

    /// [`evaluate_bodies_draft`] but also returning warnings — the draft variant
    /// the GUI uses for an instant on-screen rebuild before refining to the slow
    /// arc-fillet result in the background.
    pub fn evaluate_bodies_with_warnings_draft(
        &self,
        hidden: &std::collections::HashSet<String>,
    ) -> Result<(Vec<(String, MockMesh)>, Vec<String>), String> {
        self.evaluate_bodies_inner(hidden, true)
    }

    /// Whether the current draft result needs a background refinement pass before
    /// it should be treated as final. Native rolling-ball fillets make draft and
    /// committed geometry identical, so this is currently always false.
    pub fn has_arc_fillet(&self, hidden: &std::collections::HashSet<String>) -> bool {
        let _ = hidden;
        false
    }

    fn evaluate_bodies_inner(
        &self,
        hidden: &std::collections::HashSet<String>,
        draft: bool,
    ) -> Result<(Vec<(String, MockMesh)>, Vec<String>), String> {
        let run = || -> Result<(Vec<(String, MockMesh)>, Vec<String>), String> {
            let (live, warnings) = self.build_live(hidden, draft)?;
            Ok((tessellate_bodies(live), warnings))
        };
        // Draft previews mesh newly-built bodies at the coarse preview budget (a
        // fillet/chamfer/boolean preview lands ~2× faster). Draft evaluation only
        // ever runs on a throwaway graph clone on a background worker, so the
        // thread-local budget can't leak into the committed (fine) result, and
        // any reused prefix checkpoint keeps whatever budget it was built at.
        if draft {
            crate::mock_kernel::with_preview_tess(run)
        } else {
            run()
        }
    }

    /// Diagnostic/test only: the raw B-Rep kernel solids per body, before
    /// tessellation, keyed by body node id. Mirrors [`evaluate_bodies_inner`]
    /// but skips meshing so callers can inspect surface/topology directly.
    #[doc(hidden)]
    pub fn debug_kernel_solids(
        &self,
        hidden: &std::collections::HashSet<String>,
    ) -> Result<Vec<(String, Vec<crate::mock_kernel::KernelSolid>)>, String> {
        let (live, _) = self.build_live(hidden, false)?;
        Ok(live.into_iter().map(|b| (b.id, b.parts)).collect())
    }

    pub(crate) fn build_live(
        &self,
        hidden: &std::collections::HashSet<String>,
        draft: bool,
    ) -> Result<(Vec<LiveBody>, Vec<String>), String> {
        self.build_live_with_cancel(hidden, draft, None)
    }

    fn build_live_with_cancel(
        &self,
        hidden: &std::collections::HashSet<String>,
        draft: bool,
        cancellation: Option<&EvaluationCancellation>,
    ) -> Result<(Vec<LiveBody>, Vec<String>), String> {
        if cancellation.is_some_and(EvaluationCancellation::is_cancelled) {
            return Err("model evaluation was superseded".to_string());
        }
        // Surface circular dependencies (toposort result is otherwise unused,
        // but a cycle should still fail the whole evaluation).
        toposort(&self.graph, None)
            .map_err(|_| "Circular dependency detected in history tree!".to_string())?;

        // Resolved once per build so every expression-driven dimension (extrude
        // depth and sketch dimensions alike) sees the current variable values.
        let vars = self.variable_map();
        let sketch_cache = self.sketch_region_cache(&vars);
        // Datums resolve in a pure pre-pass (their inputs never include live
        // bodies). Their warnings stay OUT of the checkpointed `warnings` —
        // they are re-derived fresh each build and appended at return, so a
        // reused prefix can't double-report them.
        let mut datum_warnings = Vec::new();
        let datums = self.resolve_datums(&vars, &mut datum_warnings);

        if cancellation.is_some_and(EvaluationCancellation::is_cancelled) {
            return Err("model evaluation was superseded".to_string());
        }

        // Body-eval nodes in creation order, with a cumulative content hash after
        // each one (see [`eval_prefix_keys`]). An edit that touches only a trailing
        // node — dragging a fillet/chamfer radius, say — leaves every earlier key
        // identical, so the matching prefix (and its expensive booleans) is
        // restored from the previous evaluation instead of recomputed.
        let nodes: Vec<NodeIndex> = self.body_nodes_in_creation_order();
        let keys = self.eval_prefix_keys(&nodes, hidden, &vars);

        let (mut live, mut warnings, mut statuses, reuse, mut checkpoints) = {
            let cache = self.eval_cache.borrow();
            let cps = &cache.checkpoints;
            let matched = (0..keys.len().min(cps.len()))
                .rev()
                .find(|&i| cps[i].as_ref().is_some_and(|cp| cp.key == keys[i]));
            if let Some(last) = matched {
                let cp = cps[last].as_ref().expect("matched checkpoint missing");
                let mut retained = vec![None; keys.len()];
                for i in 0..=last {
                    if let Some(old) = cps.get(i).and_then(Option::as_ref) {
                        if old.key == keys[i] {
                            retained[i] = Some(old.clone());
                        }
                    }
                }
                (
                    cp.live.clone(),
                    cp.warnings.clone(),
                    cp.statuses.clone(),
                    last + 1,
                    retained,
                )
            } else {
                (
                    Vec::new(),
                    Vec::new(),
                    Vec::new(),
                    0usize,
                    vec![None; keys.len()],
                )
            }
        };

        for (i, &idx) in nodes.iter().enumerate() {
            if cancellation.is_some_and(EvaluationCancellation::is_cancelled) {
                return Err("model evaluation was superseded".to_string());
            }
            // Reused prefix: its checkpoints (and so its `live`/`warnings`) were
            // restored above; skip recomputing it.
            if i < reuse {
                continue;
            }
            let feature_started = std::time::Instant::now();
            let node = &self.graph[idx];
            let warn_before = warnings.len();
            let live_before_feature = live.clone();
            if !hidden.contains(&node.id) {
                match &node.feature {
                    FeatureType::Box { w, h, d } => {
                        let source = SketchExtrudeSource {
                            regions: vec![SketchExtrudeRegionSource {
                                boundary: vec![(0.0, 0.0), (*w, 0.0), (*w, *h), (0.0, *h)],
                                holes: Vec::new(),
                                depth: *d,
                                cs: CoordinateSystem::XY,
                                rect_circle: None,
                            }],
                        };
                        let solid = crate::mock_kernel::extruded_region_solid(
                            &source.regions[0].boundary,
                            &source.regions[0].holes,
                            source.regions[0].depth,
                            &source.regions[0].cs,
                        )
                        .unwrap_or_else(|| crate::mock_kernel::box_solid(*w, *h, *d));
                        // Derive the display from the part (single source of truth) so a
                        // primitive box matches a sketched-extruded rectangle exactly;
                        // the analytic make_box mesh is only the cracked-mesh fallback.
                        let mut pristine = crate::mock_kernel::try_display_mesh_from_part(&solid)
                            .unwrap_or_else(|| MockMesh::make_box(*w, *h, *d));
                        stamp_box_face_refs(&mut pristine, &node.id);
                        crate::mock_kernel::populate_edge_adjacent_face_names(&mut pristine);
                        live.push(LiveBody {
                            id: node.id.clone(),
                            parts: vec![solid],
                            pristine: Some(pristine.into()),
                            sketch_source: Some(source),
                            cut_tools: Vec::new(),
                            cut_replay: None,
                            edge_mod_cut_history_path_used: false,
                            thread_replay: None,
                        });
                    }
                    FeatureType::Cylinder { r, h } => {
                        if let Some(solid) = crate::mock_kernel::cylinder_solid(*r, *h) {
                            // Display derives from the part (single source of truth); the
                            // analytic make_cylinder mesh is the cracked-mesh fallback.
                            let mut pristine =
                                crate::mock_kernel::try_display_mesh_from_part(&solid)
                                    .unwrap_or_else(|| MockMesh::make_cylinder(*r, *h, 32));
                            stamp_cylinder_face_refs(&mut pristine, &node.id);
                            crate::mock_kernel::populate_edge_adjacent_face_names(&mut pristine);
                            live.push(LiveBody {
                                id: node.id.clone(),
                                parts: vec![solid],
                                pristine: Some(pristine.into()),
                                sketch_source: None,
                                cut_tools: Vec::new(),
                                cut_replay: None,
                                edge_mod_cut_history_path_used: false,
                                thread_replay: None,
                            });
                        }
                    }
                    FeatureType::Import { step_data, label } => {
                        match openrcad::exchange::read_step_str(step_data) {
                            Ok(solid) => {
                                let mut pristine = MockMesh::from_solid(&solid);
                                if pristine.indices.is_empty() {
                                    warnings.push(format!(
                                        "Import '{}' ({}): STEP body tessellated empty.",
                                        node.id, label
                                    ));
                                }
                                stamp_import_face_refs(&mut pristine, &node.id);
                                crate::mock_kernel::populate_edge_adjacent_face_names(
                                    &mut pristine,
                                );
                                live.push(LiveBody {
                                    id: node.id.clone(),
                                    parts: vec![solid],
                                    pristine: Some(pristine.into()),
                                    sketch_source: None,
                                    cut_tools: Vec::new(),
                                    cut_replay: None,
                                    edge_mod_cut_history_path_used: false,
                                    thread_replay: None,
                                });
                            }
                            Err(e) => warnings.push(format!(
                                "Import '{}' ({}): failed to parse STEP data: {}.",
                                node.id, label, e
                            )),
                        }
                    }
                    FeatureType::Extrude {
                        depth,
                        region_indices,
                        mode,
                        depth_expr,
                        target,
                    } => {
                        // An expression that still resolves drives the depth; a
                        // missing/broken variable falls back to the stored value and
                        // surfaces a warning (otherwise the model silently builds
                        // with a stale depth — e.g. after a referenced variable is
                        // deleted).
                        let eff_depth = match depth_expr.as_ref() {
                            Some(e) => match crate::expr::eval(e, &vars) {
                                Ok(v) => v as f32,
                                Err(_) => {
                                    warnings.push(format!(
                                    "Extrude '{}': depth expression \"{}\" no longer evaluates; \
                                     using last value {:.3}.",
                                    node.id, e, depth
                                ));
                                    *depth
                                }
                            },
                            None => *depth,
                        };
                        self.apply_extrude(
                            idx,
                            &node.id,
                            eff_depth,
                            region_indices,
                            *mode,
                            target.as_deref(),
                            &sketch_cache,
                            &datums,
                            draft,
                            &mut live,
                            &mut warnings,
                        );
                    }
                    FeatureType::Revolve {
                        axis,
                        angle_deg,
                        angle_expr,
                        region_indices,
                        mode,
                        target,
                    } => {
                        let eff_angle = match angle_expr.as_ref() {
                            Some(e) => match crate::expr::eval(e, &vars) {
                                Ok(v) => v as f32,
                                Err(_) => {
                                    warnings.push(format!(
                                        "Revolve '{}': angle expression \"{}\" no longer \
                                         evaluates; using last value {:.3}.",
                                        node.id, e, angle_deg
                                    ));
                                    *angle_deg
                                }
                            },
                            None => *angle_deg,
                        };
                        self.apply_revolve(
                            idx,
                            &node.id,
                            axis,
                            eff_angle,
                            region_indices,
                            *mode,
                            target.as_deref(),
                            &sketch_cache,
                            &datums,
                            &mut live,
                            &mut warnings,
                        );
                    }
                    FeatureType::Pattern { source, kind } => {
                        apply_pattern(
                            &node.id,
                            source,
                            kind,
                            &vars,
                            &datums,
                            &mut live,
                            &mut warnings,
                        );
                    }
                    FeatureType::BodyTransform {
                        source,
                        translation,
                        copy,
                    } => {
                        apply_body_transform(
                            &node.id,
                            source,
                            *translation,
                            *copy,
                            &mut live,
                            &mut warnings,
                        );
                    }
                    FeatureType::BodyJoin { sources } => {
                        apply_body_join(&node.id, sources, &mut live, &mut warnings);
                    }
                    FeatureType::BodyCut {
                        target,
                        tool,
                        keep_tool,
                    } => {
                        apply_body_cut(
                            &node.id,
                            target,
                            tool,
                            *keep_tool,
                            &mut live,
                            &mut warnings,
                        );
                    }
                    FeatureType::Thread {
                        target,
                        face,
                        internal,
                        pitch,
                        depth,
                        angle_deg,
                        right_handed,
                        starts,
                        length,
                        flip,
                        ..
                    } => {
                        apply_thread(
                            &node.id,
                            target,
                            face,
                            *internal,
                            *pitch,
                            *depth,
                            *angle_deg,
                            *right_handed,
                            *starts,
                            *length,
                            *flip,
                            &mut live,
                            &mut warnings,
                        );
                    }
                    FeatureType::Loft {
                        sections,
                        mode,
                        target,
                    } => {
                        self.apply_loft(
                            &node.id,
                            sections,
                            *mode,
                            target.as_deref(),
                            &sketch_cache,
                            &datums,
                            &mut live,
                            &mut warnings,
                        );
                    }
                    FeatureType::Sweep {
                        profile_sketch,
                        profile_region,
                        path_sketch,
                        mode,
                        target,
                    } => {
                        self.apply_sweep(
                            &node.id,
                            profile_sketch,
                            *profile_region,
                            path_sketch,
                            *mode,
                            target.as_deref(),
                            &sketch_cache,
                            &datums,
                            &mut live,
                            &mut warnings,
                        );
                    }
                    FeatureType::Shell {
                        target,
                        thickness,
                        thickness_expr,
                        open_faces,
                    } => {
                        let eff_thickness = match thickness_expr.as_ref() {
                            Some(e) => match crate::expr::eval(e, &vars) {
                                Ok(v) => v as f32,
                                Err(_) => {
                                    warnings.push(format!(
                                        "Shell '{}': thickness expression \"{}\" no longer \
                                         evaluates; using last value {:.3}.",
                                        node.id, e, thickness
                                    ));
                                    *thickness
                                }
                            },
                            None => *thickness,
                        };
                        apply_shell(
                            &node.id,
                            target,
                            eff_thickness,
                            open_faces,
                            &mut live,
                            &mut warnings,
                        );
                    }
                    FeatureType::Hole {
                        target,
                        position,
                        direction,
                        diameter,
                        diameter_expr,
                        depth,
                        kind,
                    } => {
                        let eff_diameter = match diameter_expr.as_ref() {
                            Some(e) => match crate::expr::eval(e, &vars) {
                                Ok(v) => v as f32,
                                Err(_) => {
                                    warnings.push(format!(
                                        "Hole '{}': diameter expression \"{}\" no longer \
                                         evaluates; using last value {:.3}.",
                                        node.id, e, diameter
                                    ));
                                    *diameter
                                }
                            },
                            None => *diameter,
                        };
                        apply_hole(
                            &node.id,
                            target,
                            *position,
                            *direction,
                            eff_diameter,
                            *depth,
                            kind,
                            &mut live,
                            &mut warnings,
                        );
                    }
                    FeatureType::EdgeMod {
                        target,
                        edge,
                        dist,
                        dist_expr,
                        replay,
                        kind,
                    } => {
                        let eff_dist = match dist_expr.as_ref() {
                            Some(e) => match crate::expr::eval(e, &vars) {
                                Ok(v) => v as f32,
                                Err(_) => {
                                    warnings.push(format!(
                                        "Edge modifier '{}': distance expression \"{}\" no longer \
                                     evaluates; using last value {:.3}.",
                                        node.id, e, dist
                                    ));
                                    *dist
                                }
                            },
                            None => *dist,
                        };
                        apply_edge_mod(
                            &node.id,
                            target,
                            edge,
                            replay,
                            eff_dist,
                            *kind,
                            &mut live,
                            &mut warnings,
                        );
                    }
                    _ => {}
                }
                if let Err(reason) = validate_live_body_state(&live) {
                    live = live_before_feature;
                    warnings.push(format!(
                        "Feature '{}' produced invalid solid topology ({reason}); \
                         the feature was not applied.",
                        node.id
                    ));
                }
                // Per-feature resolution status: this node is Unresolved iff it
                // raised a warning while being applied — each warning names its own
                // feature and this node's dispatch is the only thing that ran since
                // `warn_before`, so the new warnings are exactly this feature's. This
                // is what lets the GUI flag the precise feature that failed rather
                // than a global count, and encodes "report unresolved, don't silently
                // mis-apply."
                let state = if warnings.len() > warn_before {
                    ResolutionState::Unresolved(warnings[warn_before..].join(" "))
                } else {
                    ResolutionState::Resolved
                };
                statuses.push(FeatureStatus {
                    feature_id: node.id.clone(),
                    feature_name: node.name.clone(),
                    state,
                });
            }
            // Snapshot the assembled bodies after this node so a later evaluation
            // that shares this prefix can resume from here.
            checkpoints[i] = Some(EvalCheckpoint {
                key: keys[i],
                live: live.clone(),
                warnings: warnings.clone(),
                statuses: statuses.clone(),
                feature_duration: feature_started.elapsed(),
            });
        }

        *self.eval_cache.borrow_mut() = std::sync::Arc::new(EvalCache { checkpoints });

        if !datum_warnings.is_empty() {
            datum_warnings.extend(warnings);
            warnings = datum_warnings;
        }
        Ok((live, warnings))
    }

    pub fn edge_mod_replay_intent_for_edge(
        &self,
        target: &str,
        edge: &EdgeRef,
        hidden: &std::collections::HashSet<String>,
    ) -> EdgeModReplayIntent {
        let mut intent = EdgeModReplayIntent::auto_for(target.to_string(), edge.clone());
        if let Ok((live, _)) = self.build_live(hidden, false) {
            if let Some(history) = live
                .iter()
                .find(|body| body.id == target)
                .and_then(|body| body.cut_replay.as_ref())
            {
                intent.pre_cut_target = Some(history.base_body_id.clone());
                intent.replay_cut_nodes = history
                    .steps
                    .iter()
                    .map(|step| step.node_id.clone())
                    .collect();
            }
        }
        intent
    }

    /// Cumulative content hash of the geometry inputs for each node in `nodes`,
    /// in order — `keys[i]` covers nodes `0..=i`. Folds `vars` (the seed, so any
    /// variable change invalidates everything), then per node its id, hidden
    /// state, feature, and its inputs' features (e.g. an extrude's parent sketch).
    /// Two evaluations agree on a prefix exactly when the geometry of that prefix
    /// is identical, which is what makes reusing a cached checkpoint sound.
    /// Hashing only — no geometry is built here.
    ///
    /// NOTE: evaluation quality is deliberately not folded into these graph-input
    /// keys. Edge modifiers use the same geometry in both qualities. Cut and Join
    /// do use `draft` to defer expensive thread replay, but interactive evaluation
    /// runs on a throwaway graph clone, so those draft checkpoints never populate
    /// the authoritative graph's cache. If previews ever share their checkpoint
    /// cache with committed evaluation, quality must be folded into the seed.
    pub(crate) fn eval_prefix_keys(
        &self,
        nodes: &[NodeIndex],
        hidden: &std::collections::HashSet<String>,
        vars: &HashMap<String, f64>,
    ) -> Vec<u64> {
        let mut h = std::collections::hash_map::DefaultHasher::new();
        let mut kv: Vec<(&String, &f64)> = vars.iter().collect();
        kv.sort_by(|a, b| a.0.cmp(b.0));
        for (k, v) in kv {
            h.write(k.as_bytes());
            h.write_u8(0xff);
            h.write_u64(v.to_bits());
        }
        // Datums fold into the SEED (like variables): they resolve in a
        // pre-pass and any consumer anywhere downstream reads them, so one
        // coarse rule — any datum edit invalidates every checkpoint — is what
        // keeps prefix reuse sound without per-consumer dependency tracking.
        // The sketch→datum attachment map folds too (re-attaching a sketch to
        // a different datum changes geometry without touching any feature).
        let mut datum_nodes: Vec<NodeIndex> = self
            .graph
            .node_indices()
            .filter(|&i| {
                matches!(
                    self.graph[i].feature,
                    FeatureType::DatumPlane { .. }
                        | FeatureType::DatumAxis { .. }
                        | FeatureType::DatumPoint { .. }
                )
            })
            .collect();
        datum_nodes.sort_by_key(|&i| creation_key(&self.graph[i].id));
        for idx in datum_nodes {
            let node = &self.graph[idx];
            h.write(node.id.as_bytes());
            fold_feature(&mut h, &node.feature);
        }
        let mut datum_refs: Vec<(&String, &String)> = self.sketch_datum_refs.iter().collect();
        datum_refs.sort();
        for (sketch_id, datum_id) in datum_refs {
            h.write(sketch_id.as_bytes());
            h.write_u8(0xfe);
            h.write(datum_id.as_bytes());
        }

        let mut keys = Vec::with_capacity(nodes.len());
        for &idx in nodes {
            let node = &self.graph[idx];
            h.write(node.id.as_bytes());
            h.write_u8(hidden.contains(&node.id) as u8);
            fold_feature(&mut h, &node.feature);
            // An input node's geometry feeds this one (an extrude reads its parent
            // sketch's plane + curves), so a change there must invalidate from here.
            for p in self
                .graph
                .neighbors_directed(idx, petgraph::Direction::Incoming)
            {
                let pn = &self.graph[p];
                h.write(pn.id.as_bytes());
                fold_feature(&mut h, &pn.feature);
            }
            keys.push(h.finish());
        }
        keys
    }

    /// Detect (or fetch from the region cache) the planar regions of every
    /// sketch in the graph, keyed by node index. Sketches are cached even when
    /// hidden — hiding a sketch must not break dependent extrudes.
    fn sketch_region_cache(&self, vars: &HashMap<String, f64>) -> HashMap<NodeIndex, SketchEval> {
        let mut cache = HashMap::new();
        for idx in self.graph.node_indices() {
            if let FeatureType::Sketch {
                cs,
                curves,
                shapes,
                corner_mods,
                mirrors,
                entity_ids,
                solver,
                ..
            } = &self.graph[idx].feature
            {
                // A parametric sketch is rebuilt from its shapes against the
                // current variables (or from its solver model when present);
                // the region-cache key (a hash of the resolved curves) then
                // changes whenever a variable does.
                let effective = crate::sketch::effective_curves_solved(
                    curves,
                    shapes,
                    corner_mods,
                    mirrors,
                    solver.as_ref(),
                    vars,
                );
                // Sketch-on-face: region detection sees the projected face
                // boundary (reference curves) so drawn shapes split against the
                // face outline and the outline itself is a region. Appended
                // AFTER the drawn curves — the same order every GUI region site
                // uses, so the region indices stored on extrudes stay
                // consistent. `effective` itself stays drawn-only (the shape
                // recognizers downstream depend on that).
                let face_boundary = self
                    .sketch_face_boundaries
                    .get(&self.graph[idx].id)
                    .cloned();
                // Fail-loud: a variable-driven constraint model that no longer
                // solves keeps its last-valid geometry, and the failure reason
                // rides along so the consuming extrude can report it.
                let solve_failure = solver
                    .as_ref()
                    .filter(|m| {
                        !m.is_empty() && crate::sketch::solve::has_variable_bound_constraint(m)
                    })
                    .and_then(|m| {
                        let report = crate::sketch::solve_model(m, vars);
                        match report.outcome {
                            crate::sketch::SolveOutcome::Converged => None,
                            crate::sketch::SolveOutcome::DidNotConverge => Some(
                                "its constraints did not converge; keeping the last valid geometry"
                                    .to_string(),
                            ),
                            crate::sketch::SolveOutcome::Conflicting => {
                                Some(match report.conflicting {
                                    Some(id) => format!(
                                    "its constraints conflict (constraint {}); keeping the last \
                                     valid geometry",
                                    id.0
                                ),
                                    None => "its constraints conflict; keeping the last valid \
                                         geometry"
                                        .to_string(),
                                })
                            }
                        }
                    });
                let regions = match &face_boundary {
                    Some(boundary) => {
                        let mut merged = effective.clone();
                        merged.extend_curves(boundary);
                        self.cached_regions(&merged)
                    }
                    None => self.cached_regions(&effective),
                };
                let provenance = build_region_provenance(&effective, shapes, entity_ids, &regions);
                // Whole-shape outlines drive the overlapping-shapes-as-boolean
                // path. Sketch fillets/chamfers (`corner_mods`) reshape the
                // displayed geometry, which the raw shape outlines wouldn't
                // reflect, so those sketches fall back to the per-region path.
                let shape_loops = if corner_mods.is_empty() {
                    crate::sketch::shape_loops(shapes, vars)
                } else {
                    Vec::new()
                };
                cache.insert(
                    idx,
                    SketchEval {
                        cs: *cs,
                        regions,
                        provenance,
                        curves: effective,
                        face_boundary,
                        shape_loops,
                        solve_failure,
                    },
                );
            }
        }
        cache
    }

    /// [`detect_regions`] memoized on a content hash of the curves. A miss runs
    /// the O(n²) arrangement once and stores it; identical curves (every frame
    /// of an extrude-drag preview, say) hit the cache. See [`region_cache`].
    pub(crate) fn cached_regions(&self, curves: &SketchCurves) -> Vec<Region> {
        let key = hash_curves(curves);
        if let Some(regions) = self.region_cache.borrow().get(&key) {
            return regions.clone();
        }
        let regions = detect_regions(curves);
        let mut cache = self.region_cache.borrow_mut();
        // Bound growth across a long editing session (each distinct sketch state
        // is a new key). The cache is a pure accelerator, so dropping it is safe.
        if cache.len() >= REGION_CACHE_CAP {
            cache.clear();
        }
        cache.insert(key, regions.clone());
        regions
    }

    /// Solid-producing nodes (Box / Cylinder / Extrude) in creation order — the
    /// order booleans must see (see [`evaluate_bodies_with_warnings`]).
    pub(crate) fn body_nodes_in_creation_order(&self) -> Vec<NodeIndex> {
        let mut nodes: Vec<NodeIndex> = self
            .graph
            .node_indices()
            .filter(|&i| {
                matches!(
                    self.graph[i].feature,
                    FeatureType::Box { .. }
                        | FeatureType::Cylinder { .. }
                        | FeatureType::Extrude { .. }
                        | FeatureType::EdgeMod { .. }
                        | FeatureType::Import { .. }
                        | FeatureType::Revolve { .. }
                        | FeatureType::Pattern { .. }
                        | FeatureType::BodyTransform { .. }
                        | FeatureType::BodyJoin { .. }
                        | FeatureType::BodyCut { .. }
                        | FeatureType::Hole { .. }
                        | FeatureType::Shell { .. }
                        | FeatureType::Loft { .. }
                        | FeatureType::Sweep { .. }
                        | FeatureType::Thread { .. }
                )
            })
            .collect();
        nodes.sort_by_key(|&i| creation_key(&self.graph[i].id));
        nodes
    }

    /// Evaluate one Extrude node against the bodies assembled so far. Resolves
    /// the parent sketch, builds the per-region tool solids for the chosen mode,
    /// then dispatches to the new-body / join / cut assembler. Pushes any
    /// non-fatal anomalies onto `warnings`.
    #[allow(clippy::too_many_arguments)]
    fn apply_extrude(
        &self,
        idx: NodeIndex,
        node_id: &str,
        depth: f32,
        region_indices: &[usize],
        mode: ExtrudeMode,
        boolean_target: Option<&str>,
        sketch_cache: &HashMap<NodeIndex, SketchEval>,
        datums: &HashMap<String, DatumValue>,
        draft: bool,
        live: &mut Vec<LiveBody>,
        warnings: &mut Vec<String>,
    ) {
        // Resolve the parent sketch's plane + regions.
        let parent_idx = self
            .graph
            .neighbors_directed(idx, petgraph::Direction::Incoming)
            .find(|p| sketch_cache.contains_key(p));
        let Some(parent_idx) = parent_idx else {
            return;
        };
        let sketch = &sketch_cache[&parent_idx];
        // Sketch-on-face: re-derive the plane from wherever its face is now (the
        // parent body has already been assembled into `live`), so the sketch — and
        // everything extruded from it — follows the body. Regions are 2D and
        // cs-independent, so only the placement `cs` changes, not the shapes.
        let sketch_id = &self.graph[parent_idx].id;
        // Fail-loud: the parent sketch's constraint model no longer solves.
        // Its geometry is the last-valid bake (never blank), and this extrude —
        // the body node consuming it — carries the attributed warning so the
        // feature tree marks the failure instead of silently building stale.
        if let Some(reason) = &sketch.solve_failure {
            warnings.push(format!(
                "Extrude '{node_id}': its sketch '{sketch_id}' {reason}."
            ));
        }
        // Plane priority: a datum attachment re-derives from the datum's current
        // resolution (editing the datum moves everything sketched on it); a face
        // attachment re-derives from wherever the face is now; otherwise the
        // sketch's saved plane. A datum that no longer resolves fails loud and
        // falls back to the saved plane snapshot.
        let datum_cs =
            self.sketch_datum_refs
                .get(sketch_id)
                .and_then(|datum_id| match datums.get(datum_id) {
                    Some(DatumValue::Plane(cs)) => Some(*cs),
                    _ => {
                        warnings.push(format!(
                            "Extrude '{node_id}': sketch '{sketch_id}' is attached to datum plane \
                         '{datum_id}', which did not resolve; using the sketch's saved plane."
                        ));
                        None
                    }
                });
        let cs_owned = datum_cs
            .or_else(|| {
                self.sketch_face_refs
                    .get(sketch_id)
                    .and_then(|face_ref| rederive_sketch_cs(face_ref, live))
            })
            .unwrap_or(sketch.cs);
        let cs = &cs_owned;

        // Associative face outline: a sketch-on-face carries the projected face
        // boundary as reference geometry. Re-project it from wherever the face
        // is NOW (the body may have changed upstream); when it differs from the
        // stored snapshot, re-split the regions against the fresh outline and
        // remap this extrude's stored region indices onto them — an old region
        // keeps its identity by material-point containment (same index
        // preferred). The refreshed outline + indices are queued for a
        // persistent write-back (`apply_face_reattach`) so the GUI shows the
        // moved outline and the stored indices stay in the space they were
        // detected in. On any failure the stored snapshot is used, as before.
        let mut refreshed: Option<(Vec<Region>, Vec<usize>)> = None;
        if let (Some(stored_boundary), Some(face_ref)) = (
            sketch.face_boundary.as_ref(),
            self.sketch_face_refs.get(sketch_id),
        ) {
            if let Some(fresh_boundary) = rederive_face_boundary(face_ref, live, cs) {
                if hash_curves(&fresh_boundary) != hash_curves(stored_boundary) {
                    let mut merged = sketch.curves.clone();
                    merged.extend_curves(&fresh_boundary);
                    let fresh_regions = self.cached_regions(&merged);
                    let mut remapped: Vec<usize> = Vec::new();
                    let mut lost = 0usize;
                    for &i in region_indices {
                        let Some(old) = sketch.regions.get(i) else {
                            lost += 1;
                            continue;
                        };
                        let p = region_material_point(old);
                        let target = if fresh_regions.get(i).is_some_and(|r| r.contains(p)) {
                            Some(i)
                        } else {
                            fresh_regions.iter().position(|r| r.contains(p))
                        };
                        match target {
                            Some(j) => {
                                if !remapped.contains(&j) {
                                    remapped.push(j);
                                }
                            }
                            None => lost += 1,
                        }
                    }
                    if lost > 0 {
                        warnings.push(format!(
                            "Extrude '{node_id}': {lost} selected region(s) of sketch \
                             '{sketch_id}' did not survive the face outline change."
                        ));
                    }
                    // A selection that lost EVERY region would degenerate to
                    // "all regions" (empty selector) — keep the stored snapshot
                    // instead and fail loud above.
                    let selection_survives = region_indices.is_empty() || !remapped.is_empty();
                    if !fresh_regions.is_empty() && selection_survives {
                        let mut pending = self.pending_face_reattach.borrow_mut();
                        pending.boundaries.insert(sketch_id.clone(), fresh_boundary);
                        pending.planes.insert(sketch_id.clone(), *cs);
                        if remapped != region_indices {
                            pending
                                .region_indices
                                .insert(node_id.to_string(), remapped.clone());
                        }
                        refreshed = Some((fresh_regions, remapped));
                    }
                }
            }
        }
        let (regions_owned, indices_owned): (Vec<Region>, Vec<usize>) = match refreshed {
            Some((r, i)) => (r, i),
            None => (sketch.regions.clone(), region_indices.to_vec()),
        };
        let regions = &regions_owned;
        let region_indices: &[usize] = &indices_owned;
        if regions.is_empty() {
            return;
        }

        // Overlapping-shapes-as-boolean. The drawn shapes are grouped into overlap
        // clusters; within a cluster the user's selection marks BASE shapes (kept)
        // and the rest are TOOL shapes (cut). The boolean is resolved in 2D using
        // the planar regions `detect_regions` already produced: a region is kept
        // when it lies in some base shape and in NO tool shape (so the overlap
        // "lens" of base∩tool is dropped — the cut — while base∩base is kept — the
        // union). Those kept regions then flow through the unchanged per-region
        // extrude, which already turns a rect-with-circular-bite region into a
        // clean box-minus-cylinder. Empty `shape_loops` (legacy sketch / sketch
        // corner-mods) leaves every region on the normal selection path.
        // Per region: is it part of a multi-shape boolean cluster, and (if so)
        // should it be kept? `region_is_boolean` regions ignore `region_indices`
        // (the shape selection decides); other regions use the normal rule. This
        // is the SAME classification the live extrude ghost uses, so a preview
        // keeps exactly the regions this commit will (see `boolean_region_plan`).
        let loops = &sketch.shape_loops;
        let plan = crate::parametric::extrude::boolean_region_plan(loops, regions, region_indices);
        let region_is_boolean = plan.is_boolean;
        let process_region = plan.process;
        // Did any boolean-cluster region get built in NewBody mode? Its adjacent
        // kept pieces are fused at the end so a unioned cluster reads as one solid.
        let mut newbody_has_boolean = false;

        // Build solid tool(s) per selected region (empty selector = all regions).
        // New body also accumulates an analytic mesh so pristine bodies keep
        // their nice hidden-line wireframes.
        //
        // The sketch's analytic fillet arcs (center+radius) are handed to the wire
        // builder so a rounded profile sweeps to EXACT cylindrical walls, instead of
        // `loop_to_wire`'s sample refit that facets a multi-arc rounded rectangle.
        let arc_circles: Vec<((f32, f32), f32)> = sketch
            .curves
            .arcs
            .iter()
            .map(|a| (a.center, a.radius))
            .collect();
        let region_solid = |r: &Region, cs: &CoordinateSystem, d: f32| {
            crate::mock_kernel::extruded_region_solid_with_arcs(
                &r.boundary,
                &r.holes,
                d,
                cs,
                &arc_circles,
            )
        };
        // The smooth native-cylinder tool for a circular, hole-free region (None
        // otherwise). Tried before the faceted prism so a round boss/pocket reads
        // smooth — the kernel fuses/bores analytic cylinders watertight.
        let cyl_tool = |r: &Region, cs: &CoordinateSystem, d: f32| {
            crate::mock_kernel::circular_cylinder_tool(&r.boundary, &r.holes, d, cs)
        };

        let mut newbody_tools: Vec<KernelSolid> = Vec::new();
        let mut newbody_cut_tools: Vec<CutTool> = Vec::new();
        let mut cut_tools: Vec<CutTool> = Vec::new();
        let mut join_tools: Vec<JoinTool> = Vec::new();
        let mut sketch_source = SketchExtrudeSource {
            regions: Vec::new(),
        };
        let mut newbody_mesh = MockMesh::empty();
        let mut newbody_part_meshes: Vec<([i64; 6], MockMesh)> = Vec::new();
        let mut newbody_body_count = 0usize;
        let mut newbody_cut_replay: Option<CutReplayHistory> = None;

        // An open construction/projected line can partition a drawn circle into
        // two or more selected regions. Extruding those pieces independently
        // leaves touching half-cylinders (and their diameter/generator seams)
        // when the kernel cannot fuse an exactly coincident interface. When every
        // atomic region inside a circle is selected for New Body or Join,
        // reconstruct the original circle as one analytic cylinder and skip its
        // fragments below.
        let mut collapsed_circle_regions: std::collections::HashSet<usize> =
            std::collections::HashSet::new();
        if matches!(
            mode,
            ExtrudeMode::NewBody | ExtrudeMode::Join | ExtrudeMode::Cut
        ) {
            for (circle, inside) in crate::parametric::extrude::complete_selected_circles(
                &sketch.curves.circles,
                regions,
                &process_region,
            ) {
                let boundary: Vec<(f32, f32)> = (0..crate::CIRCLE_SEGS)
                    .map(|i| {
                        let angle = i as f32 / crate::CIRCLE_SEGS as f32 * std::f32::consts::TAU;
                        (
                            circle.center.0 + circle.radius * angle.cos(),
                            circle.center.1 + circle.radius * angle.sin(),
                        )
                    })
                    .collect();
                let smooth = crate::mock_kernel::circular_cylinder_tool(&boundary, &[], depth, cs);
                if let Some(solid) = smooth {
                    match mode {
                        ExtrudeMode::NewBody => {
                            newbody_tools.push(solid);
                            newbody_body_count += 1;
                        }
                        ExtrudeMode::Join => {
                            let dipped = crate::mock_kernel::circular_cylinder_tool(
                                &boundary,
                                &[],
                                overshoot_depth(depth, 1.0),
                                &overshoot_cs(cs, depth),
                            );
                            join_tools.push(JoinTool {
                                smooth: Some(solid),
                                exact: None,
                                dipped,
                            });
                        }
                        ExtrudeMode::Cut => {
                            let (cut_cs, cut_depth) = directional_cut(cs, depth);
                            let smooth = crate::mock_kernel::circular_cylinder_tool(
                                &boundary,
                                &[],
                                cut_depth,
                                &cut_cs,
                            );
                            let exact = crate::mock_kernel::extruded_region_solid(
                                &boundary,
                                &[],
                                cut_depth,
                                &cut_cs,
                            );
                            let grown = grow_loop(&boundary, true);
                            let expanded = crate::mock_kernel::extruded_region_solid(
                                &grown,
                                &[],
                                cut_depth,
                                &cut_cs,
                            );
                            let (rev_cs, rev_depth) = directional_cut(cs, -depth);
                            let smooth_rev = crate::mock_kernel::circular_cylinder_tool(
                                &boundary,
                                &[],
                                rev_depth,
                                &rev_cs,
                            );
                            let exact_rev = crate::mock_kernel::extruded_region_solid(
                                &boundary,
                                &[],
                                rev_depth,
                                &rev_cs,
                            );
                            let expanded_rev = crate::mock_kernel::extruded_region_solid(
                                &grown,
                                &[],
                                rev_depth,
                                &rev_cs,
                            );
                            cut_tools.push(CutTool {
                                smooth,
                                exact,
                                expanded,
                                smooth_rev,
                                exact_rev,
                                expanded_rev,
                                circle: Some(circle),
                            });
                        }
                    }
                    collapsed_circle_regions.extend(inside);
                }
            }
        }

        for (i, region) in regions.iter().enumerate() {
            if collapsed_circle_regions.contains(&i) {
                continue;
            }
            if !process_region[i] {
                continue;
            }
            if region_is_boolean[i] && matches!(mode, ExtrudeMode::NewBody) {
                newbody_has_boolean = true;
            }
            // Provenance fragments are identical for every region of a sketch
            // (see `build_region_provenance`), so when a refreshed face outline
            // yields MORE regions than the snapshot had, the extras borrow the
            // first entry rather than losing recognizer support.
            let provenance = sketch
                .provenance
                .get(i)
                .or_else(|| sketch.provenance.first());
            match mode {
                ExtrudeMode::NewBody => {
                    // A filleted profile (analytic corner arcs) is NOT a rectangle
                    // with a circular bite; the rect-minus-circle recognisers mis-fit
                    // its corner arcs into one big circle (≈ the half-diagonal), so
                    // skip them when the sketch handed us fillet arcs and let the
                    // exact-arc `region_solid` build the rounded body.
                    let rect_circle_exact = if arc_circles.is_empty() {
                        provenance
                            .and_then(|provenance| {
                                rect_circle_region_base_and_cutter_from_provenance(
                                    provenance, region, depth, cs, 0.0,
                                )
                            })
                            .or_else(|| {
                                rect_circle_region_base_and_cutter_from_sketch(
                                    &sketch.curves,
                                    region,
                                    depth,
                                    cs,
                                    0.0,
                                )
                            })
                            .or_else(|| {
                                crate::mock_kernel::rect_minus_circle_region_base_and_cutter(
                                    &region.boundary,
                                    &region.holes,
                                    depth,
                                    cs,
                                )
                            })
                    } else {
                        None
                    };
                    let canonical_rect_circle = rect_circle_exact.as_ref().map(|(base, cutter)| {
                        RectCircleCanonicalSource {
                            base: base.clone(),
                            cutter: cutter.clone(),
                            body: crate::mock_kernel::difference(base, cutter),
                        }
                    });
                    // Prefer the smooth analytic cylinder for a circular profile so
                    // a new-body cylinder stays round if it is later joined/cut
                    // (re-tessellated from this solid); falls back to the prism.
                    let body_tool = canonical_rect_circle
                        .as_ref()
                        .and_then(|canonical| canonical.body.clone())
                        .or_else(|| cyl_tool(region, cs, depth))
                        .or_else(|| region_solid(region, cs, depth));
                    // Keep the part so the display mesh derives directly from it
                    // (single source of truth) instead of an independently rebuilt twin.
                    let region_part = body_tool.clone();
                    if let Some(s) = body_tool {
                        newbody_tools.push(s);
                        newbody_body_count += 1;
                    }
                    let grown_replay_cutter = provenance
                        .and_then(|provenance| {
                            rect_circle_region_base_and_cutter_from_provenance(
                                provenance,
                                region,
                                depth,
                                cs,
                                CUT_WALL_GROW,
                            )
                        })
                        .or_else(|| {
                            rect_circle_region_base_and_cutter_from_sketch(
                                &sketch.curves,
                                region,
                                depth,
                                cs,
                                CUT_WALL_GROW,
                            )
                        });
                    let expanded_replay_cutter = grown_replay_cutter
                        .clone()
                        .map(|(_, cutter)| cutter)
                        .or_else(|| {
                            crate::mock_kernel::rect_minus_circle_region_base_and_grown_cutter(
                                &region.boundary,
                                &region.holes,
                                depth,
                                cs,
                                CUT_WALL_GROW,
                            )
                            .map(|(_, cutter)| cutter)
                        });
                    let region_source = SketchExtrudeRegionSource {
                        boundary: region.boundary.clone(),
                        holes: region.holes.clone(),
                        depth,
                        cs: *cs,
                        rect_circle: canonical_rect_circle,
                    };
                    if let Some(canonical) = region_source.rect_circle.as_ref() {
                        let replay_tool = CutTool::single_direction(
                            Some(canonical.cutter.clone()),
                            None,
                            expanded_replay_cutter.clone(),
                            None,
                        );
                        newbody_cut_tools.extend(cut_tool_recutter_tools(&replay_tool));
                        newbody_cut_replay = Some(CutReplayHistory {
                            base_body_id: node_id.to_string(),
                            base_parts: vec![canonical.base.clone()],
                            base_pristine: None,
                            base_sketch_source: Some(SketchExtrudeSource {
                                regions: vec![region_source.clone()],
                            }),
                            steps: vec![CutReplayStep {
                                node_id: node_id.to_string(),
                                tool: replay_tool,
                            }],
                        });
                    }
                    sketch_source.regions.push(region_source);
                    let mut region_mesh = match region_part.as_ref() {
                        Some(part) => crate::mock_kernel::display_mesh_from_part(
                            part,
                            &region.boundary,
                            &region.holes,
                            depth,
                            cs,
                        ),
                        None => crate::mock_kernel::extruded_region_display_mesh(
                            &region.boundary,
                            &region.holes,
                            depth,
                            cs,
                        ),
                    };
                    stamp_sketch_extrude_edge_refs(
                        &mut region_mesh,
                        node_id,
                        i,
                        provenance,
                        cs,
                        depth,
                    );
                    stamp_sketch_extrude_face_refs(&mut region_mesh, node_id, i, cs, depth);
                    crate::mock_kernel::populate_edge_adjacent_face_names(&mut region_mesh);
                    if let Some(part) = region_part.as_ref() {
                        newbody_part_meshes
                            .push((crate::mock_kernel::part_key(part), region_mesh.clone()));
                    }
                    newbody_mesh.append(region_mesh);
                }
                ExtrudeMode::Cut => {
                    // Cut in the drawn direction (the sign of `depth`): a negative
                    // depth cuts *into* the body the sketch sits on, a positive
                    // depth sweeps *outward* from the sketch face, removing
                    // material from whatever body lies in that path. Overshoot
                    // keeps both end caps off the body's faces (which the solver
                    // can't resolve), so a cut that punches clean through a body
                    // still exits cleanly.
                    //
                    // exact = the drawn pocket (precise dimensions, used whenever
                    // the solver accepts it). expanded = walls nudged ~0.1mm
                    // outward so a pocket reaching the edge of a face doesn't
                    // leave the tool's side wall coplanar with the body's side
                    // face — the other half of the coplanarity problem
                    // `directional_cut` solves only for the end caps.
                    let (cut_cs, cut_depth) = directional_cut(cs, depth);
                    let smooth = cyl_tool(region, &cut_cs, cut_depth);
                    let exact = region_solid(region, &cut_cs, cut_depth);
                    let grown_boundary = grow_loop(&region.boundary, true);
                    let grown_holes: Vec<Vec<(f32, f32)>> =
                        region.holes.iter().map(|h| grow_loop(h, false)).collect();
                    let expanded = crate::mock_kernel::extruded_region_solid(
                        &grown_boundary,
                        &grown_holes,
                        cut_depth,
                        &cut_cs,
                    );
                    // The same tool swept the other way, for the fall-back when the
                    // drawn direction misses the body (see `CutTool`).
                    let (rev_cs, rev_depth) = directional_cut(cs, -depth);
                    let smooth_rev = cyl_tool(region, &rev_cs, rev_depth);
                    let exact_rev = region_solid(region, &rev_cs, rev_depth);
                    let expanded_rev = crate::mock_kernel::extruded_region_solid(
                        &grown_boundary,
                        &grown_holes,
                        rev_depth,
                        &rev_cs,
                    );
                    if smooth.is_some() || exact.is_some() || expanded.is_some() {
                        let circle = if sketch.curves.segments.is_empty()
                            && sketch.curves.circles.len() == 1
                            && region.holes.is_empty()
                        {
                            sketch.curves.circles.first().copied()
                        } else {
                            None
                        };
                        cut_tools.push(CutTool {
                            smooth,
                            exact,
                            expanded,
                            smooth_rev,
                            exact_rev,
                            expanded_rev,
                            circle,
                        });
                    }
                }
                ExtrudeMode::Join => {
                    // smooth = analytic cylinder for a round boss (the kernel bores
                    // its coplanar cap as a true circle, so the boss reads round).
                    // exact = perfect prism geometry when it resolves; dipped = near
                    // cap nudged INTO existing material to break the (almost always
                    // present) coplanarity with the face the sketch sits on. The
                    // dip is absorbed by the body it joins, leaving no artifact.
                    let smooth = cyl_tool(region, cs, depth);
                    let exact = region_solid(region, cs, depth);
                    let dipped = region_solid(
                        region,
                        &overshoot_cs(cs, depth),
                        overshoot_depth(depth, 1.0),
                    );
                    if smooth.is_some() || exact.is_some() || dipped.is_some() {
                        join_tools.push(JoinTool {
                            smooth,
                            exact,
                            dipped,
                        });
                    }
                }
            }
        }

        match mode {
            ExtrudeMode::NewBody => {
                let before_fuse = newbody_tools.len();
                if newbody_has_boolean || before_fuse > 1 {
                    // Disjoint lumps remain separate; adjacent/touching sketch
                    // regions are offered to the union builder so their shared
                    // boundary becomes internal topology.
                    newbody_tools = fuse_overlapping_solids(newbody_tools);
                }
                // A kernel union can package multiple shells into one `Solid`.
                // New Body semantics are one independently selectable body per
                // connected component, so normalize every result before assigning
                // stable output ids. The position key makes Body_1/Body_2 ordering
                // deterministic across rebuilds.
                newbody_tools = newbody_tools
                    .into_iter()
                    .flat_map(|solid| {
                        let components = solid.split_disconnected();
                        if crate::mock_kernel::components_form_connected_material(&components) {
                            vec![solid]
                        } else {
                            components
                        }
                    })
                    .collect();
                newbody_tools.sort_by_key(crate::mock_kernel::part_key);
                let merged_regions = newbody_tools.len() < before_fuse;
                if newbody_tools.len() == 1 {
                    live.push(LiveBody {
                        id: node_id.to_string(),
                        parts: newbody_tools,
                        // Per-region meshes retain the shared sketch boundary.
                        // Tessellate the fused B-Rep after a successful union so
                        // a continuous coplanar face has no internal display edge.
                        pristine: (!merged_regions && !newbody_mesh.indices.is_empty())
                            .then(|| std::sync::Arc::new(newbody_mesh)),
                        sketch_source: (!sketch_source.regions.is_empty()).then_some(sketch_source),
                        cut_tools: newbody_cut_tools,
                        cut_replay: (newbody_body_count == 1)
                            .then_some(())
                            .and(newbody_cut_replay),
                        edge_mod_cut_history_path_used: false,
                        thread_replay: None,
                    });
                } else {
                    // The feature owns several bodies. Keep the first output id
                    // backward-compatible (`extrude_N`) and suffix later bodies.
                    // Each receives its own mesh so viewport picking, selection,
                    // targeting, and export all see distinct bodies.
                    for (output_index, part) in newbody_tools.into_iter().enumerate() {
                        let output_id = body_output_id(node_id, output_index);
                        let part_key = crate::mock_kernel::part_key(&part);
                        let part_source_regions: Vec<SketchExtrudeRegionSource> = sketch_source
                            .regions
                            .iter()
                            .filter(|source| {
                                let source_solid = source
                                    .rect_circle
                                    .as_ref()
                                    .and_then(|canonical| canonical.body.clone())
                                    .or_else(|| {
                                        crate::mock_kernel::extruded_region_solid(
                                            &source.boundary,
                                            &source.holes,
                                            source.depth,
                                            &source.cs,
                                        )
                                    });
                                source_solid.as_ref().is_some_and(|source_part| {
                                    crate::mock_kernel::part_key(source_part) == part_key
                                })
                            })
                            .cloned()
                            .collect();

                        // Preserve the per-region pristine mesh whenever this
                        // output is an unfused sketch region. It carries the
                        // durable shape/edge provenance used for reattachment;
                        // rebuilding it generically here would reduce a real
                        // Body_2 to anonymous tessellation edges.
                        let mut mesh = newbody_part_meshes
                            .iter()
                            .position(|(key, _)| *key == part_key)
                            .map(|index| newbody_part_meshes.remove(index).1)
                            .unwrap_or_else(|| {
                                let mut mesh = MockMesh::from_solid(&part);
                                let (mesh_cs, mesh_depth) = part_source_regions
                                    .first()
                                    .map(|source| (source.cs, source.depth))
                                    .unwrap_or((*cs, depth));
                                stamp_sketch_extrude_face_refs(
                                    &mut mesh,
                                    &output_id,
                                    output_index,
                                    &mesh_cs,
                                    mesh_depth,
                                );
                                crate::mock_kernel::populate_edge_adjacent_face_names(&mut mesh);
                                mesh
                            });
                        for edge in &mut mesh.edge_refs {
                            if let Some(topology) = edge.topology.as_mut() {
                                topology.body_id = Some(output_id.clone());
                            }
                        }
                        crate::mock_kernel::stamp_face_component(&mut mesh, &output_id, &part);

                        live.push(LiveBody {
                            id: output_id,
                            parts: vec![part],
                            pristine: (!mesh.indices.is_empty()).then(|| std::sync::Arc::new(mesh)),
                            sketch_source: (!part_source_regions.is_empty()).then_some(
                                SketchExtrudeSource {
                                    regions: part_source_regions,
                                },
                            ),
                            // Canonical circular-bite replay metadata is assembled
                            // feature-wide above. It cannot safely be shared across
                            // independently targeted bodies; native body operations
                            // remain available and clear/rebuild this state normally.
                            cut_tools: Vec::new(),
                            cut_replay: None,
                            edge_mod_cut_history_path_used: false,
                            thread_replay: None,
                        });
                    }
                }
            }
            ExtrudeMode::Join | ExtrudeMode::Cut => {
                // Named targeting: when the feature pins a target body, the
                // boolean applies to that body ONLY. A missing target is a
                // fail-loud no-op — never "hit whatever else overlaps".
                if let Some(target_id) = boolean_target {
                    if !live.iter().any(|b| b.id == target_id) {
                        warnings.push(format!(
                            "{} '{node_id}': its target body '{target_id}' no longer \
                             exists, so it had no effect.",
                            if mode == ExtrudeMode::Cut {
                                "Cut"
                            } else {
                                "Join"
                            },
                        ));
                        return;
                    }
                }
                if mode == ExtrudeMode::Join {
                    apply_join(live, node_id, join_tools, boolean_target, draft, warnings);
                } else {
                    apply_cut(live, node_id, cut_tools, boolean_target, draft, warnings);
                }
            }
        }
    }
}

/// Apply a persistent rigid translation to a live body. A copy leaves the
/// source untouched; a move consumes it and gives the transform node the new
/// body identity so subsequent features can target the moved result.
fn apply_body_transform(
    node_id: &str,
    source: &str,
    translation: [f32; 3],
    copy: bool,
    live: &mut Vec<LiveBody>,
    warnings: &mut Vec<String>,
) {
    let Some(source_index) = live.iter().position(|body| body.id == source) else {
        warnings.push(format!(
            "Body transform '{node_id}': source body '{source}' no longer exists."
        ));
        return;
    };
    let source_body = live[source_index].clone();
    if source_body.parts.is_empty() {
        warnings.push(format!(
            "Body transform '{node_id}': source body '{source}' has no solid geometry."
        ));
        return;
    }

    use openrcad::foundation::{Trsf, Vec as GeomVec};
    let transform = Trsf::translation(GeomVec::new(
        translation[0] as f64,
        translation[1] as f64,
        translation[2] as f64,
    ));
    let parts: Vec<KernelSolid> = source_body
        .parts
        .iter()
        .map(|part| crate::mock_kernel::transformed_solid(part, &transform, false))
        .collect();
    let mut mesh = MockMesh::empty();
    for part in &parts {
        let mut part_mesh = MockMesh::from_solid(part);
        stamp_pattern_face_refs(&mut part_mesh, node_id, 0);
        crate::mock_kernel::populate_edge_adjacent_face_names(&mut part_mesh);
        mesh.append(part_mesh);
    }
    if !copy {
        live.remove(source_index);
    }
    live.push(LiveBody {
        id: node_id.to_string(),
        parts,
        pristine: (!mesh.indices.is_empty()).then(|| std::sync::Arc::new(mesh)),
        sketch_source: None,
        cut_tools: Vec::new(),
        cut_replay: None,
        edge_mod_cut_history_path_used: false,
        thread_replay: None,
    });
}

impl ParametricGraph {
    /// Evaluate one Revolve node: resolve the parent sketch and the axis, build
    /// a solid of revolution per selected region, then assemble by mode. The
    /// revolve analogue of [`apply_extrude`], deliberately leaner — no
    /// rect-circle canonical forms or cut-replay history (those are extrude
    /// tricks for prismatic pockets); the kernel solid IS the analytic result.
    #[allow(clippy::too_many_arguments)]
    fn apply_revolve(
        &self,
        idx: NodeIndex,
        node_id: &str,
        axis: &AxisBase,
        angle_deg: f32,
        region_indices: &[usize],
        mode: ExtrudeMode,
        boolean_target: Option<&str>,
        sketch_cache: &HashMap<NodeIndex, SketchEval>,
        datums: &HashMap<String, DatumValue>,
        live: &mut Vec<LiveBody>,
        warnings: &mut Vec<String>,
    ) {
        let parent_idx = self
            .graph
            .neighbors_directed(idx, petgraph::Direction::Incoming)
            .find(|p| sketch_cache.contains_key(p));
        let Some(parent_idx) = parent_idx else {
            warnings.push(format!("Revolve '{node_id}': no parent sketch."));
            return;
        };
        let sketch = &sketch_cache[&parent_idx];
        let sketch_id = &self.graph[parent_idx].id;
        if let Some(reason) = &sketch.solve_failure {
            warnings.push(format!(
                "Revolve '{node_id}': its sketch '{sketch_id}' {reason}."
            ));
        }
        // Same plane priority as extrude: datum attachment, face attachment,
        // saved plane.
        let datum_cs =
            self.sketch_datum_refs
                .get(sketch_id)
                .and_then(|datum_id| match datums.get(datum_id) {
                    Some(DatumValue::Plane(cs)) => Some(*cs),
                    _ => None,
                });
        let cs_owned = datum_cs
            .or_else(|| {
                self.sketch_face_refs
                    .get(sketch_id)
                    .and_then(|face_ref| rederive_sketch_cs(face_ref, live))
            })
            .unwrap_or(sketch.cs);
        let cs = &cs_owned;
        let regions = &sketch.regions;
        if regions.is_empty() {
            return;
        }

        // Resolve the axis to world space.
        let axis_world: Option<(Vec3, Vec3)> = match axis {
            AxisBase::X => Some((Vec3::ZERO, Vec3::X)),
            AxisBase::Y => Some((Vec3::ZERO, Vec3::Y)),
            AxisBase::Z => Some((Vec3::ZERO, Vec3::Z)),
            AxisBase::Datum(id) => match datums.get(id) {
                Some(DatumValue::Axis { origin, dir }) => Some((*origin, *dir)),
                _ => None,
            },
            AxisBase::TwoPoints { a, b } => {
                let av = Vec3::new(a[0], a[1], a[2]);
                let bv = Vec3::new(b[0], b[1], b[2]);
                let d = bv.sub(av).normalize();
                (d != Vec3::ZERO).then_some((av, d))
            }
        };
        let Some((axis_origin, axis_dir)) = axis_world else {
            warnings.push(format!(
                "Revolve '{node_id}': its axis could not be resolved."
            ));
            return;
        };
        if !(angle_deg > 0.0 && angle_deg <= 360.0 + 1e-3) {
            warnings.push(format!(
                "Revolve '{node_id}': angle {angle_deg}° is outside (0, 360]."
            ));
            return;
        }
        let angle_rad = (angle_deg as f64).to_radians().min(std::f64::consts::TAU);

        let arc_circles: Vec<((f32, f32), f32)> = sketch
            .curves
            .arcs
            .iter()
            .map(|a| (a.center, a.radius))
            .collect();
        let take_all = region_indices.is_empty();

        let mut newbody_parts: Vec<KernelSolid> = Vec::new();
        let mut newbody_mesh = MockMesh::empty();
        let mut cut_tools: Vec<CutTool> = Vec::new();
        let mut join_tools: Vec<JoinTool> = Vec::new();
        for (i, region) in regions.iter().enumerate() {
            if !(take_all || region_indices.contains(&i)) {
                continue;
            }
            let solid = crate::mock_kernel::revolved_region_solid(
                &region.boundary,
                &region.holes,
                cs,
                axis_origin,
                axis_dir,
                angle_rad,
                &arc_circles,
            );
            let Some(solid) = solid else {
                warnings.push(format!(
                    "Revolve '{node_id}': region {i} could not be revolved (the profile \
                     must lie on one side of the axis, and the axis in the sketch plane)."
                ));
                continue;
            };
            match mode {
                ExtrudeMode::NewBody => {
                    let mut mesh = MockMesh::from_solid(&solid);
                    stamp_revolve_face_refs(&mut mesh, node_id, i);
                    crate::mock_kernel::populate_edge_adjacent_face_names(&mut mesh);
                    newbody_mesh.append(mesh);
                    newbody_parts.push(solid);
                }
                ExtrudeMode::Cut => {
                    cut_tools.push(CutTool::single_direction(None, Some(solid), None, None));
                }
                ExtrudeMode::Join => {
                    join_tools.push(JoinTool {
                        smooth: None,
                        exact: Some(solid),
                        dipped: None,
                    });
                }
            }
        }

        match mode {
            ExtrudeMode::NewBody => {
                if !newbody_parts.is_empty() {
                    live.push(LiveBody {
                        id: node_id.to_string(),
                        parts: newbody_parts,
                        pristine: (!newbody_mesh.indices.is_empty())
                            .then(|| std::sync::Arc::new(newbody_mesh)),
                        sketch_source: None,
                        cut_tools: Vec::new(),
                        cut_replay: None,
                        edge_mod_cut_history_path_used: false,
                        thread_replay: None,
                    });
                }
            }
            ExtrudeMode::Join | ExtrudeMode::Cut => {
                if let Some(target_id) = boolean_target {
                    if !live.iter().any(|b| b.id == target_id) {
                        warnings.push(format!(
                            "{} '{node_id}': its target body '{target_id}' no longer \
                             exists, so it had no effect.",
                            if mode == ExtrudeMode::Cut {
                                "Revolve cut"
                            } else {
                                "Revolve join"
                            },
                        ));
                        return;
                    }
                }
                if mode == ExtrudeMode::Join {
                    apply_join(live, node_id, join_tools, boolean_target, false, warnings);
                } else {
                    apply_cut(live, node_id, cut_tools, boolean_target, false, warnings);
                }
            }
        }
    }
}

impl ParametricGraph {
    /// The effective placement plane of a sketch: a datum attachment first
    /// (re-derived from the datum's current resolution), then a face attachment
    /// (re-derived from the body), else the sketch's saved plane. Shared by
    /// extrude/revolve/loft/sweep.
    fn effective_sketch_cs(
        &self,
        sketch_id: &str,
        saved: CoordinateSystem,
        datums: &HashMap<String, DatumValue>,
        live: &[LiveBody],
    ) -> CoordinateSystem {
        let datum_cs =
            self.sketch_datum_refs
                .get(sketch_id)
                .and_then(|datum_id| match datums.get(datum_id) {
                    Some(DatumValue::Plane(cs)) => Some(*cs),
                    _ => None,
                });
        datum_cs
            .or_else(|| {
                self.sketch_face_refs
                    .get(sketch_id)
                    .and_then(|face_ref| rederive_sketch_cs(face_ref, live))
            })
            .unwrap_or(saved)
    }

    /// Look up a cached sketch evaluation by its node id.
    fn sketch_eval_by_id<'a>(
        &self,
        cache: &'a HashMap<NodeIndex, SketchEval>,
        id: &str,
    ) -> Option<&'a SketchEval> {
        self.node_map.get(id).and_then(|idx| cache.get(idx))
    }

    /// Evaluate one Loft node: skin a solid through the ordered section
    /// profiles. NewBody pushes the lofted solid; Join/Cut apply it as a
    /// boolean tool against the target body.
    #[allow(clippy::too_many_arguments)]
    fn apply_loft(
        &self,
        node_id: &str,
        sections: &[(String, usize)],
        mode: ExtrudeMode,
        boolean_target: Option<&str>,
        sketch_cache: &HashMap<NodeIndex, SketchEval>,
        datums: &HashMap<String, DatumValue>,
        live: &mut Vec<LiveBody>,
        warnings: &mut Vec<String>,
    ) {
        if sections.len() < 2 {
            warnings.push(format!(
                "Loft '{node_id}': needs at least two section profiles."
            ));
            return;
        }
        let mut resolved: Vec<(CoordinateSystem, Vec<(f32, f32)>)> = Vec::new();
        for (sketch_id, region_index) in sections {
            let Some(sketch) = self.sketch_eval_by_id(sketch_cache, sketch_id) else {
                warnings.push(format!(
                    "Loft '{node_id}': section sketch '{sketch_id}' not found."
                ));
                return;
            };
            let Some(region) = sketch.regions.get(*region_index) else {
                warnings.push(format!(
                    "Loft '{node_id}': sketch '{sketch_id}' has no region {region_index}."
                ));
                return;
            };
            let cs = self.effective_sketch_cs(sketch_id, sketch.cs, datums, live);
            resolved.push((cs, region.boundary.clone()));
        }
        let Some(solid) = crate::mock_kernel::lofted_solid(&resolved) else {
            warnings.push(format!(
                "Loft '{node_id}': the sections could not be skinned into a solid \
                 (check they're ordered and similarly shaped)."
            ));
            return;
        };
        self.assemble_generated_body(node_id, solid, mode, boolean_target, live, warnings, |m| {
            stamp_generated_face_refs(m, node_id, "loft")
        });
    }

    /// Evaluate one Sweep node: transport the profile along the path via RMF.
    #[allow(clippy::too_many_arguments)]
    fn apply_sweep(
        &self,
        node_id: &str,
        profile_sketch: &str,
        profile_region: usize,
        path_sketch: &str,
        mode: ExtrudeMode,
        boolean_target: Option<&str>,
        sketch_cache: &HashMap<NodeIndex, SketchEval>,
        datums: &HashMap<String, DatumValue>,
        live: &mut Vec<LiveBody>,
        warnings: &mut Vec<String>,
    ) {
        let Some(profile) = self.sketch_eval_by_id(sketch_cache, profile_sketch) else {
            warnings.push(format!(
                "Sweep '{node_id}': profile sketch '{profile_sketch}' not found."
            ));
            return;
        };
        let Some(region) = profile.regions.get(profile_region) else {
            warnings.push(format!(
                "Sweep '{node_id}': profile sketch has no region {profile_region}."
            ));
            return;
        };
        let profile_cs = self.effective_sketch_cs(profile_sketch, profile.cs, datums, live);
        let Some(path) = self.sketch_eval_by_id(sketch_cache, path_sketch) else {
            warnings.push(format!(
                "Sweep '{node_id}': path sketch '{path_sketch}' not found."
            ));
            return;
        };
        let path_cs = self.effective_sketch_cs(path_sketch, path.cs, datums, live);
        let Some(path_2d) = path.curves.path_polyline(1e-3) else {
            warnings.push(format!(
                "Sweep '{node_id}': the path sketch must be a single open chain of \
                 lines/arcs (no branches, loops, or gaps)."
            ));
            return;
        };
        let path_3d: Vec<Vec3> = path_2d
            .iter()
            .map(|&(u, v)| path_cs.unproject(u, v))
            .collect();
        let Some(solid) = crate::mock_kernel::swept_solid(&profile_cs, &region.boundary, &path_3d)
        else {
            warnings.push(format!(
                "Sweep '{node_id}': the profile could not be swept along the path \
                 (it may self-intersect on a tight bend)."
            ));
            return;
        };
        self.assemble_generated_body(node_id, solid, mode, boolean_target, live, warnings, |m| {
            stamp_generated_face_refs(m, node_id, "sweep")
        });
    }

    /// Shared tail for loft/sweep: put a freshly generated solid into `live`
    /// as a NewBody, or apply it as a Join/Cut boolean tool against the target.
    #[allow(clippy::too_many_arguments)]
    fn assemble_generated_body(
        &self,
        node_id: &str,
        solid: KernelSolid,
        mode: ExtrudeMode,
        boolean_target: Option<&str>,
        live: &mut Vec<LiveBody>,
        warnings: &mut Vec<String>,
        stamp: impl Fn(&mut MockMesh),
    ) {
        match mode {
            ExtrudeMode::NewBody => {
                let mut mesh = MockMesh::from_solid(&solid);
                if mesh.indices.is_empty() {
                    warnings.push(format!("Feature '{node_id}': result tessellated empty."));
                    return;
                }
                stamp(&mut mesh);
                crate::mock_kernel::populate_edge_adjacent_face_names(&mut mesh);
                live.push(LiveBody {
                    id: node_id.to_string(),
                    parts: vec![solid],
                    pristine: Some(mesh.into()),
                    sketch_source: None,
                    cut_tools: Vec::new(),
                    cut_replay: None,
                    edge_mod_cut_history_path_used: false,
                    thread_replay: None,
                });
            }
            ExtrudeMode::Join | ExtrudeMode::Cut => {
                if let Some(target_id) = boolean_target {
                    if !live.iter().any(|b| b.id == target_id) {
                        warnings.push(format!(
                            "Feature '{node_id}': its target body '{target_id}' no longer exists."
                        ));
                        return;
                    }
                }
                if mode == ExtrudeMode::Join {
                    apply_join(
                        live,
                        node_id,
                        vec![JoinTool {
                            smooth: None,
                            exact: Some(solid),
                            dipped: None,
                        }],
                        boolean_target,
                        false,
                        warnings,
                    );
                } else {
                    apply_cut(
                        live,
                        node_id,
                        vec![CutTool::single_direction(None, Some(solid), None, None)],
                        boolean_target,
                        false,
                        warnings,
                    );
                }
            }
        }
    }
}

/// Stamp a generated body's faces `{kind}:{node}:face:{k}` in quantized-
/// centroid order (stable across runs).
fn stamp_generated_face_refs(mesh: &mut MockMesh, body_id: &str, kind: &str) {
    let quant = |v: f32| (v as f64 * 1.0e3).round() as i64;
    let mut order: Vec<usize> = (0..mesh.face_refs.len())
        .filter(|&i| mesh.face_refs[i].topology.is_none())
        .collect();
    order.sort_by_key(|&i| {
        let c = mesh.face_refs[i].centroid;
        (quant(c[0]), quant(c[1]), quant(c[2]))
    });
    for (k, &i) in order.iter().enumerate() {
        mesh.face_refs[i].topology = Some(crate::mock_kernel::MeshTopologyFaceRef {
            body_id: Some(body_id.to_string()),
            component_id: None,
            topology_version: Some(0),
            face_id: Some(format!("{kind}:{body_id}:face:{k}")),
            surface_kind: None,
        });
    }
}

/// Evaluate one Shell node: hollow the target body in place. Open faces are
/// resolved geometrically (captured centroid+normal → nearest matching kernel
/// face); the shelled solid replaces the body's parts and its display mesh is
/// re-derived from the result.
// `&mut Vec<LiveBody>` (not a slice) matches every sibling `apply_*` helper's
// signature — a uniform body-list handle across the evaluator.
#[allow(clippy::ptr_arg)]
fn apply_shell(
    node_id: &str,
    target: &str,
    thickness: f32,
    open_faces: &[FaceRef],
    live: &mut Vec<LiveBody>,
    warnings: &mut Vec<String>,
) {
    let Some(body_idx) = live.iter().position(|b| b.id == target) else {
        warnings.push(format!(
            "Shell '{node_id}': its target body '{target}' no longer exists."
        ));
        return;
    };
    if thickness <= 0.0 {
        warnings.push(format!("Shell '{node_id}': thickness must be positive."));
        return;
    }
    if open_faces.is_empty() {
        warnings.push(format!(
            "Shell '{node_id}': select at least one face to remove."
        ));
        return;
    }
    let body = &live[body_idx];
    let mut new_parts: Vec<KernelSolid> = Vec::new();
    for part in &body.parts {
        let kernel_open: Vec<_> = open_faces
            .iter()
            .flat_map(|fref| {
                crate::mock_kernel::kernel_faces_matching(part, fref.centroid, fref.normal)
            })
            .collect();
        if kernel_open.is_empty() {
            // This part doesn't own any of the removed faces (multi-part body)
            // — shell doesn't apply to it; keep it unchanged.
            new_parts.push(part.clone());
            continue;
        }
        match openrcad::algo::shell_solid(part, thickness as f64, &kernel_open) {
            Ok(shelled) => new_parts.push(shelled),
            Err(e) => {
                warnings.push(format!(
                    "Shell '{node_id}': the kernel couldn't hollow this body ({e:?}). \
                     Supported: boxes, cylinders, and straight-edged planar solids."
                ));
                return;
            }
        }
    }
    let body = &mut live[body_idx];
    body.parts = new_parts;
    // The analytic mesh no longer matches; re-derive display from the parts.
    body.pristine = None;
    body.cut_replay = None;
}

/// Evaluate one Hole node: compose the drill from analytic cylinder/cone
/// cutters (with the standard `CUT_OVERSHOOT` so end caps never sit coplanar
/// with body faces) and apply them through the guarded cut pipeline against
/// the target body only.
// `&mut Vec<LiveBody>` matches every sibling `apply_*` helper (see `apply_shell`).
#[allow(clippy::too_many_arguments, clippy::ptr_arg)]
fn apply_hole(
    node_id: &str,
    target: &str,
    position: [f32; 3],
    direction: [f32; 3],
    diameter: f32,
    depth: Option<f32>,
    kind: &HoleKind,
    live: &mut Vec<LiveBody>,
    warnings: &mut Vec<String>,
) {
    let Some(body) = live.iter().find(|b| b.id == target) else {
        warnings.push(format!(
            "Hole '{node_id}': its target body '{target}' no longer exists."
        ));
        return;
    };
    let pos = Vec3::new(position[0], position[1], position[2]);
    let dir = Vec3::new(direction[0], direction[1], direction[2]).normalize();
    if dir == Vec3::ZERO || diameter <= 0.0 {
        warnings.push(format!(
            "Hole '{node_id}': needs a non-zero direction and a positive diameter."
        ));
        return;
    }
    // Through-all length: comfortably past the target's bounding diagonal.
    let diag = body
        .parts
        .iter()
        .filter_map(|p| p.bounding_box().corners())
        .map(|(lo, hi)| {
            let dx = hi.x() - lo.x();
            let dy = hi.y() - lo.y();
            let dz = hi.z() - lo.z();
            (dx * dx + dy * dy + dz * dz).sqrt() as f32
        })
        .fold(0.0f32, f32::max);
    let overshoot = CUT_OVERSHOOT;
    let start = pos.sub(dir.mul(overshoot));
    let bore_len = match depth {
        Some(d) if d > 0.0 => d + 2.0 * overshoot,
        Some(_) => {
            warnings.push(format!("Hole '{node_id}': depth must be positive."));
            return;
        }
        None => diag.max(1.0) + 2.0 * overshoot,
    };

    // The HEAD cutter (counterbore/countersink) is applied FIRST: it starts at
    // the surface, so it cuts virgin material cleanly, and the bore then
    // drills through its flat/conical bottom. The other order asks the solver
    // to subtract a wide cylinder whose flat bottom meets an existing narrow
    // coaxial bore wall in a full circle — the periodic-face band-partition the
    // boolean engine can't yet do (pinned by the ignored kernel test
    // `blind_counterbore_over_through_bore` in boolean_robustness_matrix.rs).
    // Head-before-bore is also the natural machining order, so this is a
    // correct model, not merely a dodge.
    let mut cut_tools: Vec<CutTool> = Vec::new();
    match kind {
        HoleKind::Simple => {}
        HoleKind::Counterbore {
            diameter: cb_d,
            depth: cb_depth,
        } => {
            if *cb_d > diameter && *cb_depth > 0.0 {
                if let Some(tool) = crate::mock_kernel::cylinder_tool_at(
                    start,
                    dir,
                    *cb_d as f64 / 2.0,
                    (*cb_depth + overshoot) as f64,
                ) {
                    cut_tools.push(CutTool::single_direction(Some(tool), None, None, None));
                }
            } else {
                warnings.push(format!(
                    "Hole '{node_id}': counterbore needs diameter > bore and depth > 0."
                ));
            }
        }
        HoleKind::Countersink {
            diameter: cs_d,
            angle_deg,
        } => {
            if *cs_d > diameter && *angle_deg > 0.0 && *angle_deg < 180.0 {
                let half = (*angle_deg as f64 / 2.0).to_radians();
                let cs_depth = ((*cs_d - diameter) as f64 / 2.0) / half.tan();
                // Slope the overshoot extension so the cone stays the same cone.
                let k = ((*cs_d - diameter) as f64 / 2.0) / cs_depth;
                let r1 = *cs_d as f64 / 2.0 + overshoot as f64 * k;
                if let Some(tool) = crate::mock_kernel::cone_tool_at(
                    start,
                    dir,
                    r1,
                    diameter as f64 / 2.0,
                    cs_depth + overshoot as f64,
                ) {
                    cut_tools.push(CutTool::single_direction(None, Some(tool), None, None));
                }
            } else {
                warnings.push(format!(
                    "Hole '{node_id}': countersink needs diameter > bore and angle in (0, 180)."
                ));
            }
        }
    }
    let bore =
        crate::mock_kernel::cylinder_tool_at(start, dir, diameter as f64 / 2.0, bore_len as f64);
    match bore {
        Some(tool) => cut_tools.push(CutTool::single_direction(Some(tool), None, None, None)),
        None => {
            warnings.push(format!("Hole '{node_id}': bore cutter failed to build."));
            return;
        }
    }
    apply_cut(live, node_id, cut_tools, Some(target), false, warnings);
}

/// Evaluate one Thread node: model a helical thread on a cylindrical face of the
/// target body. A helical-tool boolean against a smooth cylinder is neither
/// robust nor fast in this kernel, so threads are modeled **directly** with
/// analytic helix-railed faces (see [`crate::mock_kernel::thread_wall_faces`]):
/// an EXTERNAL thread on a plain cylinder body replaces the whole part with a
/// threaded cylinder; an INTERNAL thread (tapped hole) — or an external thread
/// on a boss — replaces just the body's cylindrical wall faces and re-sews
/// (see [`crate::mock_kernel::threaded_replace_cylinder_wall`]). If neither
/// path closes watertight the body is left intact with a cosmetic note — the
/// honest fallback the user opted into.
#[allow(clippy::too_many_arguments)]
fn apply_thread(
    node_id: &str,
    target: &str,
    face: &FaceRef,
    internal: bool,
    pitch: f32,
    depth: f32,
    angle_deg: f32,
    right_handed: bool,
    starts: u32,
    length: Option<f32>,
    flip: bool,
    live: &mut Vec<LiveBody>,
    warnings: &mut Vec<String>,
) {
    if pitch <= 1e-3 || depth <= 1e-3 || angle_deg <= 0.0 || angle_deg >= 180.0 {
        warnings.push(format!(
            "Thread '{node_id}': needs positive pitch/depth and an angle in (0, 180)."
        ));
        return;
    }
    let Some(bi) = live.iter().position(|b| b.id == target) else {
        warnings.push(format!(
            "Thread '{node_id}': its target body '{target}' no longer exists."
        ));
        return;
    };

    let step = ThreadReplayStep {
        face: face.clone(),
        internal,
        pitch,
        depth,
        angle_deg,
        right_handed,
        starts,
        length,
        flip,
    };

    // Snapshot the smooth pre-thread solids the FIRST time this body is threaded,
    // *before* `thread_one` mutates `parts`. A later Join/Cut runs its boolean
    // against these instead of the helical bands, then replays the thread steps.
    let base_snapshot = live[bi]
        .thread_replay
        .is_none()
        .then(|| live[bi].parts.clone());

    match thread_one(&mut live[bi], &step) {
        Ok(()) => {
            let tr = live[bi].thread_replay.get_or_insert_with(|| ThreadReplay {
                base_parts: base_snapshot.unwrap_or_default(),
                steps: Vec::new(),
            });
            tr.steps.push(step);
        }
        Err(ThreadFailure::NoCylinderFace) => warnings.push(format!(
            "Thread '{node_id}': no cylindrical face found near the selection — thread left cosmetic."
        )),
        Err(ThreadFailure::WallReplaceFailed) => warnings.push(format!(
            "Thread '{node_id}': modeled as cosmetic — the thread wall could not be cut into this \
             body's cylindrical face (interrupted wall or too-short thread length)."
        )),
    }
}

/// Why a single thread application could not cut real geometry (see
/// [`thread_one`]). Both outcomes leave the body's `parts` untouched, so the
/// caller keeps the un-threaded solid rather than losing it.
pub(crate) enum ThreadFailure {
    /// No cylindrical face resolved near the selection on any part.
    NoCylinderFace,
    /// The face resolved but the analytic wall replacement didn't close
    /// watertight (interrupted wall, crossing feature, or too-short window).
    WallReplaceFailed,
}

/// Apply ONE thread step to `body` in place: resolve the cylindrical wall it
/// targets, build the [`crate::mock_kernel::ThreadSpec`], replace the wall with
/// analytic helix-railed faces, and refresh the body's pristine mesh. Only
/// mutates `body` on success, so a failure leaves the input geometry intact.
///
/// Shared by the graph-driven [`apply_thread`] and by the Join/Cut replay path,
/// which re-runs each stored step against a freshly-booleaned smooth base so a
/// threaded body can still absorb later booleans (see [`ThreadReplay`]).
pub(crate) fn thread_one(
    body: &mut LiveBody,
    step: &ThreadReplayStep,
) -> Result<(), ThreadFailure> {
    // Resolve the selected cylindrical face and which component it belongs to.
    // A LiveBody may intentionally contain several parts after a severing cut. Do
    // not stop at the first part that happens to contain a cylinder: the thread
    // preview captured a real point on the picked wall, so rank the best
    // cylinder from EVERY part by its distance from that point. The axial term
    // also disambiguates coaxial walls with the same radius but different spans.
    let selection_error = |info: &crate::mock_kernel::CylinderFaceInfo| {
        let p = step.face.centroid;
        let rel = [
            p[0] - info.origin[0],
            p[1] - info.origin[1],
            p[2] - info.origin[2],
        ];
        let axial = rel[0] * info.dir[0] + rel[1] * info.dir[1] + rel[2] * info.dir[2];
        let radial_vec = [
            rel[0] - info.dir[0] * axial,
            rel[1] - info.dir[1] * axial,
            rel[2] - info.dir[2] * axial,
        ];
        let radial = (radial_vec[0] * radial_vec[0]
            + radial_vec[1] * radial_vec[1]
            + radial_vec[2] * radial_vec[2])
            .sqrt();
        let radial_error = (radial - info.radius).abs();
        let axial_error = if axial < info.axial_min {
            info.axial_min - axial
        } else if axial > info.axial_max {
            axial - info.axial_max
        } else {
            0.0
        };
        radial_error.hypot(axial_error)
    };
    let component = resolve_face_on_body(body, &step.face).map(|resolved| resolved.component_index);
    let resolved = body
        .parts
        .iter()
        .enumerate()
        .filter(|(index, _)| component.is_none_or(|component| *index == component))
        .filter_map(|(pi, part)| {
            crate::mock_kernel::cylinder_face_near(part, step.face.centroid).map(|info| (pi, info))
        })
        .min_by(|(_, a), (_, b)| {
            selection_error(a)
                .partial_cmp(&selection_error(b))
                .unwrap_or(std::cmp::Ordering::Equal)
        });
    let Some((pi, info)) = resolved else {
        return Err(ThreadFailure::NoCylinderFace);
    };

    let face_len = info.axial_max - info.axial_min;
    let lead = step.pitch * step.starts.max(1) as f32;
    let spec = crate::mock_kernel::ThreadSpec {
        mean_radius: info.radius,
        pitch: lead,
        length: face_len,
        depth: step.depth,
        half_angle_deg: step.angle_deg * 0.5,
        right_handed: step.right_handed,
        internal: step.internal,
        segments_per_turn: 16,
        starts: step.starts.max(1),
    };

    // Replace the picked cylindrical wall faces (hole wall for internal, rod or
    // boss wall for external) with the analytic thread wall and re-sew the
    // body. Ends against a flat cap cut straight through the profile; ends
    // against anything else (a chamfer cone, fillet torus, or a partial-length
    // stop) fade out through a runout band into an untouched cylinder collar,
    // so rim blends survive.
    let Some(threaded) = crate::mock_kernel::threaded_replace_cylinder_wall(
        &body.parts[pi],
        &info,
        &spec,
        step.length.map(|l| l as f64),
        step.flip,
    ) else {
        return Err(ThreadFailure::WallReplaceFailed);
    };
    body.parts[pi] = threaded;

    // Tessellating the dense helical bands is the expensive part of a thread
    // (hundreds of ms for a long/large one), so do it ONCE here and store it as
    // the body's pristine mesh — identical to what `tessellate_bodies` would
    // build from the parts, but carried by the eval checkpoints instead of
    // being rebuilt on every evaluation (previews re-run the model constantly).
    refresh_thread_pristine(body);
    Ok(())
}

/// Rebuild `body.pristine` from its `parts` (the analytic mesh a threaded body
/// carries so it need not re-tessellate the dense bands each evaluation).
pub(crate) fn refresh_thread_pristine(body: &mut LiveBody) {
    let mut mesh = MockMesh::empty();
    for part in &body.parts {
        mesh.append(MockMesh::from_solid(part));
    }
    body.pristine = (!mesh.indices.is_empty()).then(|| std::sync::Arc::new(mesh));
}

/// Evaluate one Pattern node: replicate the source BODY's solids by the
/// pattern's transforms. All instances land in ONE new live body (id = the
/// pattern node), so the whole array selects/hides/deletes as a unit; the
/// source body is left untouched.
fn apply_pattern(
    node_id: &str,
    source: &str,
    kind: &PatternKind,
    vars: &HashMap<String, f64>,
    datums: &HashMap<String, DatumValue>,
    live: &mut Vec<LiveBody>,
    warnings: &mut Vec<String>,
) {
    let Some(src) = live.iter().find(|b| b.id == source) else {
        warnings.push(format!(
            "Pattern '{node_id}': its source body '{source}' no longer exists."
        ));
        return;
    };
    let parts = src.parts.clone();
    if parts.is_empty() {
        warnings.push(format!(
            "Pattern '{node_id}': source body '{source}' has no solid geometry."
        ));
        return;
    }

    use openrcad::foundation::{Ax1, Ax2, Dir, Pnt, Trsf, Vec as GeomVec};
    let to_pnt = |v: Vec3| Pnt::new(v.x as f64, v.y as f64, v.z as f64);
    let to_dir = |v: Vec3| Dir::new(v.x as f64, v.y as f64, v.z as f64);

    // Instance transforms, EXCLUDING the identity instance 0 (that's the
    // source body itself). `(transform, is_reflection)`.
    let mut mirror_join_plane: Option<(Vec3, Vec3)> = None;
    let transforms: Vec<(Trsf, bool)> = match kind {
        PatternKind::Linear {
            dir,
            spacing,
            spacing_expr,
            count,
        } => {
            let Some((_, d)) = super::datum::resolve_axis_base_world(dir, datums) else {
                warnings.push(format!(
                    "Pattern '{node_id}': its direction could not be resolved."
                ));
                return;
            };
            let step = match spacing_expr.as_ref() {
                Some(e) => match crate::expr::eval(e, vars) {
                    Ok(v) => v as f32,
                    Err(_) => {
                        warnings.push(format!(
                            "Pattern '{node_id}': spacing expression \"{e}\" no longer \
                             evaluates; using last value {spacing:.3}."
                        ));
                        *spacing
                    }
                },
                None => *spacing,
            };
            if *count < 2 || step.abs() < 1e-6 {
                warnings.push(format!(
                    "Pattern '{node_id}': needs count ≥ 2 and a non-zero spacing."
                ));
                return;
            }
            (1..*count)
                .map(|k| {
                    let offset = GeomVec::new(
                        d.x as f64 * step as f64 * k as f64,
                        d.y as f64 * step as f64 * k as f64,
                        d.z as f64 * step as f64 * k as f64,
                    );
                    (Trsf::translation(offset), false)
                })
                .collect()
        }
        PatternKind::Circular {
            axis,
            count,
            total_angle_deg,
        } => {
            let Some((origin, d)) = super::datum::resolve_axis_base_world(axis, datums) else {
                warnings.push(format!(
                    "Pattern '{node_id}': its axis could not be resolved."
                ));
                return;
            };
            if *count < 2 {
                warnings.push(format!("Pattern '{node_id}': needs count ≥ 2."));
                return;
            }
            let total = (*total_angle_deg as f64).to_radians();
            // Full ring: step = total/count (an instance AT 360° would coincide
            // with instance 0). Partial arc: step = total/(count-1) so the last
            // instance lands exactly at the requested angle.
            let full = (*total_angle_deg - 360.0).abs() < 1e-3;
            let step = if full {
                total / *count as f64
            } else {
                total / (*count as f64 - 1.0)
            };
            let ax = Ax1::new(to_pnt(origin), to_dir(d));
            (1..*count)
                .map(|k| (Trsf::rotation(&ax, step * k as f64), false))
                .collect()
        }
        PatternKind::Mirror {
            plane,
            face,
            offset,
            offset_expr,
            ..
        } => {
            let resolved = match face {
                Some(face_ref) => rederive_sketch_cs(face_ref, live),
                None => super::datum::resolve_plane_base_world(plane, datums),
            };
            let Some(cs) = resolved else {
                warnings.push(format!(
                    "Pattern '{node_id}': its mirror plane could not be resolved."
                ));
                return;
            };
            let frame = Ax2::new(to_pnt(cs.origin), to_dir(cs.n));
            let mirror = Trsf::mirror_plane(&frame);
            let effective_offset = match offset_expr.as_ref() {
                Some(expr) => match crate::expr::eval(expr, vars) {
                    Ok(value) => value as f32,
                    Err(_) => {
                        warnings.push(format!(
                            "Mirror '{node_id}': offset expression \"{expr}\" no longer \
                             evaluates; using last value {offset:.3}."
                        ));
                        *offset
                    }
                },
                None => *offset,
            };
            let translation = Trsf::translation(GeomVec::new(
                cs.n.x as f64 * effective_offset as f64,
                cs.n.y as f64 * effective_offset as f64,
                cs.n.z as f64 * effective_offset as f64,
            ));
            if effective_offset.abs() < 1.0e-4 {
                mirror_join_plane = Some((cs.origin, cs.n));
            }
            vec![(translation.multiply(&mirror), true)]
        }
    };
    if transforms.is_empty() {
        return;
    }

    let mut new_parts: Vec<KernelSolid> = Vec::new();
    let mut mesh = MockMesh::empty();
    for (k, (t, is_reflection)) in transforms.iter().enumerate() {
        for part in &parts {
            let s = crate::mock_kernel::transformed_solid(part, t, *is_reflection);
            let mut m = MockMesh::from_solid(&s);
            if m.indices.is_empty() {
                warnings.push(format!(
                    "Pattern '{node_id}': instance {} tessellated empty.",
                    k + 1
                ));
                continue;
            }
            stamp_pattern_face_refs(&mut m, node_id, k + 1);
            crate::mock_kernel::populate_edge_adjacent_face_names(&mut m);
            mesh.append(m);
            new_parts.push(s);
        }
    }
    let join_mirror = matches!(kind, PatternKind::Mirror { join: true, .. });
    if join_mirror && !new_parts.is_empty() {
        log::debug!(
            "[mirror_join:{node_id}] source={source} source_parts={} mirrored_parts={} plane_origin=({:.4},{:.4},{:.4}) plane_normal=({:.4},{:.4},{:.4})",
            parts.len(),
            new_parts.len(),
            mirror_join_plane.map_or(Vec3::ZERO, |plane| plane.0).x,
            mirror_join_plane.map_or(Vec3::ZERO, |plane| plane.0).y,
            mirror_join_plane.map_or(Vec3::ZERO, |plane| plane.0).z,
            mirror_join_plane.map_or(Vec3::ZERO, |plane| plane.1).x,
            mirror_join_plane.map_or(Vec3::ZERO, |plane| plane.1).y,
            mirror_join_plane.map_or(Vec3::ZERO, |plane| plane.1).z,
        );
        if let Some(joined_parts) = try_join_mirrored_parts(node_id, &parts, &new_parts) {
            let pristine = mirror_join_plane.map(|(origin, normal)| {
                std::sync::Arc::new(mirror_join_display_mesh(
                    node_id,
                    &joined_parts,
                    origin,
                    normal,
                ))
            });
            if let Some(source_index) = live.iter().position(|body| body.id == source) {
                live.remove(source_index);
            }
            live.push(LiveBody {
                // Mirror+Join modifies the selected source body; the Pattern node
                // is an operation, not a second body identity.
                id: source.to_string(),
                parts: joined_parts,
                pristine,
                sketch_source: None,
                cut_tools: Vec::new(),
                cut_replay: None,
                edge_mod_cut_history_path_used: false,
                thread_replay: None,
            });
            log::info!(
                "[mirror_join:{node_id}] completed as body={source} kernel_parts={} display_cleanup={}",
                live.last().map_or(0, |body| body.parts.len()),
                mirror_join_plane.is_some()
            );
            return;
        }
        log::warn!(
            "[mirror_join:{node_id}] requested Join could not connect source={source}; keeping mirrored result as a separate body"
        );
        warnings.push(format!(
            "Mirror '{node_id}': Join was requested, but its copy could not be connected to source body '{source}'; it remains separate."
        ));
    }
    if !new_parts.is_empty() {
        live.push(LiveBody {
            id: node_id.to_string(),
            parts: new_parts,
            pristine: (!mesh.indices.is_empty()).then(|| std::sync::Arc::new(mesh)),
            sketch_source: None,
            cut_tools: Vec::new(),
            cut_replay: None,
            edge_mod_cut_history_path_used: false,
            thread_replay: None,
        });
    }
}

/// Strict all-or-nothing Mirror Join. Every mirrored part must produce a real
/// union with the source (or a source+earlier-mirror union); otherwise the
/// caller keeps both bodies separate.
fn try_join_mirrored_parts(
    node_id: &str,
    source_parts: &[KernelSolid],
    mirrored_parts: &[KernelSolid],
) -> Option<Vec<KernelSolid>> {
    // A joined source can itself contain several kernel parts. A reflected part
    // may meet another reflected part before that chain reaches the source, so a
    // greedy part-by-part connectivity check can incorrectly leave a visually
    // joined mirror as a second body. Validate the complete contact graph once.
    let connected_fallback = mirrored_parts_reach_source(source_parts, mirrored_parts);
    log::debug!(
        "[mirror_join:{node_id}] complete contact graph reaches_source={connected_fallback}"
    );
    let mut joined = source_parts.to_vec();
    for (mirrored_index, mirrored) in mirrored_parts.iter().enumerate() {
        let mirrored_bounds = crate::mock_kernel::solid_aabb(mirrored)?;
        let mut merged = false;
        for (source_index, source) in joined.iter_mut().enumerate() {
            let source_bounds = crate::mock_kernel::solid_aabb(source)?;
            if !crate::mock_kernel::aabbs_overlap(&source_bounds, &mirrored_bounds, 0.05) {
                continue;
            }
            let union = match crate::mock_kernel::union_diagnostic(source, mirrored) {
                Ok(union) => union,
                Err(error) => {
                    log::debug!(
                        "[mirror_join:{node_id}] exact union failed mirrored_part={mirrored_index} source_part={source_index}: {error}"
                    );
                    continue;
                }
            };
            let union_bounds = crate::mock_kernel::solid_aabb(&union)?;
            if crate::mock_kernel::aabb_contains(&union_bounds, &source_bounds, 0.05)
                && crate::mock_kernel::aabb_contains(&union_bounds, &mirrored_bounds, 0.05)
            {
                *source = union;
                merged = true;
                log::debug!(
                    "[mirror_join:{node_id}] exact union succeeded mirrored_part={mirrored_index} source_part={source_index}"
                );
                break;
            }
            log::debug!(
                "[mirror_join:{node_id}] rejected exact union mirrored_part={mirrored_index} source_part={source_index}: result bounds did not contain both inputs"
            );
        }
        // Reflected shells can hit a guarded-boolean limitation even when their
        // material demonstrably overlaps or they share a real face. Match the
        // existing Join feature's safe degradation: keep both kernel parts in
        // one selectable body, but only after a geometric connection check.
        if !merged && connected_fallback {
            joined.push(mirrored.clone());
            merged = true;
            log::debug!(
                "[mirror_join:{node_id}] using guarded multi-part fallback for mirrored_part={mirrored_index}"
            );
        }
        if !merged {
            log::debug!(
                "[mirror_join:{node_id}] no valid exact union or connected fallback for mirrored_part={mirrored_index}"
            );
            return None;
        }
    }
    Some(joined)
}

fn mirrored_parts_reach_source(
    source_parts: &[KernelSolid],
    mirrored_parts: &[KernelSolid],
) -> bool {
    if source_parts.is_empty() || mirrored_parts.is_empty() {
        return false;
    }
    let all: Vec<&KernelSolid> = source_parts.iter().chain(mirrored_parts).collect();
    let bounds: Option<Vec<_>> = all
        .iter()
        .map(|solid| crate::mock_kernel::solid_aabb(solid))
        .collect();
    let Some(bounds) = bounds else {
        return false;
    };
    let mut reached = vec![false; all.len()];
    let mut queue = std::collections::VecDeque::new();
    for i in 0..source_parts.len() {
        reached[i] = true;
        queue.push_back(i);
    }
    while let Some(i) = queue.pop_front() {
        for j in 0..all.len() {
            if reached[j]
                || !crate::mock_kernel::aabbs_overlap(&bounds[i], &bounds[j], 0.05)
                || !solids_are_connected(all[i], all[j], &bounds[i], &bounds[j])
            {
                continue;
            }
            reached[j] = true;
            queue.push_back(j);
        }
    }
    reached[source_parts.len()..].iter().all(|reached| *reached)
}

fn solids_are_connected(
    a: &KernelSolid,
    b: &KernelSolid,
    abb: &([f32; 3], [f32; 3]),
    bbb: &([f32; 3], [f32; 3]),
) -> bool {
    let lo = [
        abb.0[0].max(bbb.0[0]),
        abb.0[1].max(bbb.0[1]),
        abb.0[2].max(bbb.0[2]),
    ];
    let hi = [
        abb.1[0].min(bbb.1[0]),
        abb.1[1].min(bbb.1[1]),
        abb.1[2].min(bbb.1[2]),
    ];
    let extent = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
    if extent.iter().any(|value| *value < -0.01) {
        return false;
    }
    let positive_axes = extent.iter().filter(|value| **value > 0.01).count();
    // A shared face has area on two axes and near-zero thickness on the third.
    if positive_axes == 2 {
        return true;
    }
    if positive_axes != 3 {
        return false;
    }
    // A single centre probe can land in a slot/hole even when broad mirrored
    // plates overlap elsewhere. Sample the overlap volume deterministically.
    const FRACTIONS: [f32; 5] = [0.1, 0.3, 0.5, 0.7, 0.9];
    let grid_overlap = FRACTIONS.iter().any(|&fx| {
        FRACTIONS.iter().any(|&fy| {
            FRACTIONS.iter().any(|&fz| {
                let probe = openrcad::foundation::Pnt::new(
                    (lo[0] + extent[0] * fx) as f64,
                    (lo[1] + extent[1] * fy) as f64,
                    (lo[2] + extent[2] * fz) as f64,
                );
                openrcad::prelude::boolean::point_in_solid(&probe, a)
                    && openrcad::prelude::boolean::point_in_solid(&probe, b)
            })
        })
    });
    grid_overlap || solid_surface_enters_other(a, b) || solid_surface_enters_other(b, a)
}

/// Probe just inside tessellated boundary faces. This catches thin, slotted, or
/// highly concave overlaps whose material misses a coarse AABB grid (the Razor
/// joined mirror is one such case). Moving opposite the outward normal avoids
/// asking the point classifier about a numerically ambiguous boundary point.
fn solid_surface_enters_other(surface: &KernelSolid, other: &KernelSolid) -> bool {
    let mesh = MockMesh::from_solid(surface);
    let triangle_count = mesh.indices.len() / 3;
    let stride = (triangle_count / 512).max(1);
    mesh.indices
        .chunks_exact(3)
        .enumerate()
        .step_by(stride)
        .any(|(_, triangle)| {
            let mut centroid = [0.0f32; 3];
            let mut normal = [0.0f32; 3];
            for &vertex in triangle {
                let base = vertex as usize * 6;
                for axis in 0..3 {
                    centroid[axis] += mesh.vertices[base + axis] / 3.0;
                    normal[axis] += mesh.vertices[base + 3 + axis] / 3.0;
                }
            }
            let length =
                (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
            if length < 1.0e-6 {
                return false;
            }
            let probe = openrcad::foundation::Pnt::new(
                (centroid[0] - normal[0] / length * 1.0e-3) as f64,
                (centroid[1] - normal[1] / length * 1.0e-3) as f64,
                (centroid[2] - normal[2] / length * 1.0e-3) as f64,
            );
            openrcad::prelude::boolean::point_in_solid(&probe, other)
        })
}

/// Display mesh for Mirror+Join. Guarded boolean fallbacks can retain coincident
/// caps and distinct face ids at the mirror plane even though they are one logical
/// body. Remove those internal caps and merge continuous coplanar faces before
/// suppressing the construction edge at the interface.
fn mirror_join_display_mesh(
    node_id: &str,
    parts: &[KernelSolid],
    origin: Vec3,
    normal: Vec3,
) -> MockMesh {
    let mut combined = MockMesh::empty();
    let mut removed_plane_triangles = 0usize;
    let mut removed_plane_edges = 0usize;
    let mut removed_overlap_edges = 0usize;
    let raw_meshes: Vec<MockMesh> = parts.iter().map(MockMesh::from_solid).collect();
    for (part_index, mut mesh) in raw_meshes.iter().cloned().enumerate() {
        let triangles_before = mesh.indices.len() / 3;
        suppress_faces_on_plane(&mut mesh, origin, normal);
        removed_plane_triangles += triangles_before - mesh.indices.len() / 3;
        let edges_before = mesh.edge_indices.len() / 2;
        suppress_edges_on_plane(&mut mesh, origin, normal);
        removed_plane_edges += edges_before - mesh.edge_indices.len() / 2;
        let edges_before = mesh.edge_indices.len() / 2;
        suppress_edges_inside_other_parts(&mut mesh, part_index, parts, &raw_meshes);
        removed_overlap_edges += edges_before - mesh.edge_indices.len() / 2;
        combined.append(mesh);
    }
    let faces_before = combined.face_refs.len();
    merge_coplanar_faces_across_plane(&mut combined, origin, normal);
    let edge_segments_before_regroup = combined.edge_indices.len() / 2;
    let edge_groups_before_regroup = combined
        .edge_groups
        .iter()
        .copied()
        .collect::<std::collections::HashSet<_>>()
        .len();
    regroup_joined_mirror_edges(&mut combined);
    log::debug!(
        "[mirror_join:{node_id}] display cleanup parts={} removed_plane_triangles={} removed_plane_edges={} removed_overlap_edges={} merged_faces={} edge_segments={}->{} edge_groups={}->{} final_triangles={} final_faces={}",
        parts.len(),
        removed_plane_triangles,
        removed_plane_edges,
        removed_overlap_edges,
        faces_before.saturating_sub(combined.face_refs.len()),
        edge_segments_before_regroup,
        combined.edge_indices.len() / 2,
        edge_groups_before_regroup,
        combined.edge_refs.len(),
        combined.indices.len() / 3,
        combined.face_refs.len(),
    );
    combined
}

/// Rebuild edge ownership after all joined-mirror parts have been combined.
/// `MockMesh::append` deliberately keeps each input's edge groups separate; that
/// is correct for separate bodies but leaves a continuous mirrored boundary as
/// two selectable half-edges. Here tangent-connected and collinear-overlapping
/// segments are unioned globally. Straight runs are collapsed to one segment;
/// curved runs retain their chords so circle/arc fitting remains analytic.
fn regroup_joined_mirror_edges(mesh: &mut MockMesh) {
    let segment_count = mesh.edge_indices.len() / 2;
    if segment_count == 0 {
        mesh.edge_groups.clear();
        mesh.edge_refs.clear();
        return;
    }

    let initial =
        crate::mock_kernel::group_edge_segments(&mesh.edge_vertices, &mesh.edge_indices, None);
    let mut parent: Vec<usize> = (0..segment_count).collect();
    fn find(parent: &mut [usize], mut value: usize) -> usize {
        while parent[value] != value {
            parent[value] = parent[parent[value]];
            value = parent[value];
        }
        value
    }
    fn union(parent: &mut [usize], a: usize, b: usize) {
        let (a, b) = (find(parent, a), find(parent, b));
        if a != b {
            parent[a.max(b)] = a.min(b);
        }
    }
    for a in 0..segment_count {
        for b in (a + 1)..segment_count {
            if initial[a] == initial[b] || collinear_segments_overlap(mesh, a, b) {
                union(&mut parent, a, b);
            }
        }
    }

    let mut dense = std::collections::HashMap::new();
    let mut groups = vec![0u32; segment_count];
    let mut next = 0u32;
    for (segment, group) in groups.iter_mut().enumerate() {
        let root = find(&mut parent, segment);
        *group = *dense.entry(root).or_insert_with(|| {
            let value = next;
            next += 1;
            value
        });
    }

    let mut members: std::collections::BTreeMap<u32, Vec<usize>> =
        std::collections::BTreeMap::new();
    for (segment, group) in groups.iter().copied().enumerate() {
        members.entry(group).or_default().push(segment);
    }
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut normals = Vec::new();
    let mut rebuilt_groups = Vec::new();
    for (group, segments) in members {
        if let Some((start, end)) = collapsed_collinear_run(mesh, &segments) {
            let base = (vertices.len() / 3) as u32;
            vertices.extend_from_slice(&start);
            vertices.extend_from_slice(&end);
            indices.extend_from_slice(&[base, base + 1]);
            let first = segments[0];
            if mesh.edge_face_normals.len() >= (first + 1) * 6 {
                normals.extend_from_slice(&mesh.edge_face_normals[first * 6..first * 6 + 6]);
            }
            rebuilt_groups.push(group);
        } else {
            for segment in segments {
                let base = (vertices.len() / 3) as u32;
                for &vertex in &mesh.edge_indices[segment * 2..segment * 2 + 2] {
                    let offset = vertex as usize * 3;
                    vertices.extend_from_slice(&mesh.edge_vertices[offset..offset + 3]);
                }
                indices.extend_from_slice(&[base, base + 1]);
                if mesh.edge_face_normals.len() >= (segment + 1) * 6 {
                    normals
                        .extend_from_slice(&mesh.edge_face_normals[segment * 6..segment * 6 + 6]);
                }
                rebuilt_groups.push(group);
            }
        }
    }
    mesh.edge_vertices = vertices;
    mesh.edge_indices = indices;
    mesh.edge_face_normals = normals;
    mesh.edge_groups = rebuilt_groups;
    mesh.edge_refs = crate::mock_kernel::mesh_edge_refs_from_groups(
        &mesh.vertices,
        &mesh.indices,
        &mesh.edge_vertices,
        &mesh.edge_indices,
        &mesh.edge_face_normals,
        &mesh.edge_groups,
    );
    crate::mock_kernel::populate_edge_adjacent_face_names(mesh);
}

fn edge_segment_points(mesh: &MockMesh, segment: usize) -> ([f32; 3], [f32; 3]) {
    let read = |vertex: u32| {
        let offset = vertex as usize * 3;
        [
            mesh.edge_vertices[offset],
            mesh.edge_vertices[offset + 1],
            mesh.edge_vertices[offset + 2],
        ]
    };
    (
        read(mesh.edge_indices[segment * 2]),
        read(mesh.edge_indices[segment * 2 + 1]),
    )
}

fn collinear_segments_overlap(mesh: &MockMesh, a: usize, b: usize) -> bool {
    let ((a0, a1), (b0, b1)) = (edge_segment_points(mesh, a), edge_segment_points(mesh, b));
    collinear_edge_points_overlap(a0, a1, b0, b1)
}

fn collinear_edge_points_overlap(a0: [f32; 3], a1: [f32; 3], b0: [f32; 3], b1: [f32; 3]) -> bool {
    let direction = [a1[0] - a0[0], a1[1] - a0[1], a1[2] - a0[2]];
    let length =
        (direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2])
            .sqrt();
    if length < 1.0e-6 {
        return false;
    }
    let unit = [
        direction[0] / length,
        direction[1] / length,
        direction[2] / length,
    ];
    let projection = |point: [f32; 3]| {
        (point[0] - a0[0]) * unit[0] + (point[1] - a0[1]) * unit[1] + (point[2] - a0[2]) * unit[2]
    };
    let distance_to_line = |point: [f32; 3]| {
        let t = projection(point);
        let nearest = [
            a0[0] + unit[0] * t,
            a0[1] + unit[1] * t,
            a0[2] + unit[2] * t,
        ];
        crate::mock_kernel::dist3(point, nearest)
    };
    if distance_to_line(b0) > 1.0e-3 || distance_to_line(b1) > 1.0e-3 {
        return false;
    }
    let (b0, b1) = (projection(b0), projection(b1));
    let (b_lo, b_hi) = (b0.min(b1), b0.max(b1));
    b_hi >= -1.0e-3 && b_lo <= length + 1.0e-3
}

fn collapsed_collinear_run(mesh: &MockMesh, segments: &[usize]) -> Option<([f32; 3], [f32; 3])> {
    let (origin, first_end) = edge_segment_points(mesh, *segments.first()?);
    let direction = [
        first_end[0] - origin[0],
        first_end[1] - origin[1],
        first_end[2] - origin[2],
    ];
    let length =
        (direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2])
            .sqrt();
    if length < 1.0e-6 {
        return None;
    }
    let unit = [
        direction[0] / length,
        direction[1] / length,
        direction[2] / length,
    ];
    let mut lo = 0.0f32;
    let mut hi = length;
    for &segment in segments {
        let (a, b) = edge_segment_points(mesh, segment);
        for point in [a, b] {
            let t = (point[0] - origin[0]) * unit[0]
                + (point[1] - origin[1]) * unit[1]
                + (point[2] - origin[2]) * unit[2];
            let nearest = [
                origin[0] + unit[0] * t,
                origin[1] + unit[1] * t,
                origin[2] + unit[2] * t,
            ];
            if crate::mock_kernel::dist3(point, nearest) > 1.0e-3 {
                return None;
            }
            lo = lo.min(t);
            hi = hi.max(t);
        }
    }
    Some((
        [
            origin[0] + unit[0] * lo,
            origin[1] + unit[1] * lo,
            origin[2] + unit[2] * lo,
        ],
        [
            origin[0] + unit[0] * hi,
            origin[1] + unit[1] * hi,
            origin[2] + unit[2] * hi,
        ],
    ))
}

fn plane_distance(p: [f32; 3], origin: Vec3, normal: Vec3) -> f32 {
    (p[0] - origin.x) * normal.x + (p[1] - origin.y) * normal.y + (p[2] - origin.z) * normal.z
}

fn suppress_faces_on_plane(mesh: &mut MockMesh, origin: Vec3, normal: Vec3) {
    let mut indices = Vec::with_capacity(mesh.indices.len());
    let mut face_ids = Vec::with_capacity(mesh.face_ids.len());
    for (triangle, tri) in mesh.indices.chunks_exact(3).enumerate() {
        let on_plane = tri.iter().all(|&vertex| {
            let base = vertex as usize * 6;
            plane_distance(
                [
                    mesh.vertices[base],
                    mesh.vertices[base + 1],
                    mesh.vertices[base + 2],
                ],
                origin,
                normal,
            )
            .abs()
                < 1.0e-3
        });
        if !on_plane {
            indices.extend_from_slice(tri);
            face_ids.push(mesh.face_ids.get(triangle).copied().unwrap_or(0));
        }
    }
    mesh.indices = indices;
    mesh.face_ids = face_ids;
    let topology: std::collections::HashMap<_, _> = mesh
        .face_refs
        .iter()
        .map(|face| (face.face_id, face.topology.clone()))
        .collect();
    mesh.face_refs =
        crate::mock_kernel::mesh_face_refs(&mesh.vertices, &mesh.indices, &mesh.face_ids);
    for face in &mut mesh.face_refs {
        face.topology = topology.get(&face.face_id).cloned().flatten();
    }
}

fn merge_coplanar_faces_across_plane(mesh: &mut MockMesh, origin: Vec3, normal: Vec3) {
    use std::collections::{HashMap, HashSet};

    let mut plane_points: HashMap<u32, HashSet<(i64, i64, i64)>> = HashMap::new();
    for (triangle, tri) in mesh.indices.chunks_exact(3).enumerate() {
        let fid = mesh.face_ids.get(triangle).copied().unwrap_or(0);
        for &vertex in tri {
            let base = vertex as usize * 6;
            let p = [
                mesh.vertices[base],
                mesh.vertices[base + 1],
                mesh.vertices[base + 2],
            ];
            if plane_distance(p, origin, normal).abs() < 1.0e-3 {
                let q = |v: f32| (v as f64 * 10_000.0).round() as i64;
                plane_points
                    .entry(fid)
                    .or_default()
                    .insert((q(p[0]), q(p[1]), q(p[2])));
            }
        }
    }

    let mut face_bounds: HashMap<u32, ([f32; 3], [f32; 3])> = HashMap::new();
    for (triangle, tri) in mesh.indices.chunks_exact(3).enumerate() {
        let fid = mesh.face_ids.get(triangle).copied().unwrap_or(0);
        let bounds = face_bounds
            .entry(fid)
            .or_insert(([f32::INFINITY; 3], [f32::NEG_INFINITY; 3]));
        for &vertex in tri {
            let base = vertex as usize * 6;
            for axis in 0..3 {
                bounds.0[axis] = bounds.0[axis].min(mesh.vertices[base + axis]);
                bounds.1[axis] = bounds.1[axis].max(mesh.vertices[base + axis]);
            }
        }
    }

    let mut remap: HashMap<u32, u32> = mesh
        .face_refs
        .iter()
        .map(|face| (face.face_id, face.face_id))
        .collect();
    for (i, a) in mesh.face_refs.iter().enumerate() {
        for b in mesh.face_refs.iter().skip(i + 1) {
            let dot =
                a.normal[0] * b.normal[0] + a.normal[1] * b.normal[1] + a.normal[2] * b.normal[2];
            let plane_delta = (a.normal[0] * (a.centroid[0] - b.centroid[0])
                + a.normal[1] * (a.centroid[1] - b.centroid[1])
                + a.normal[2] * (a.centroid[2] - b.centroid[2]))
                .abs();
            let shared = plane_points.get(&a.face_id).is_some_and(|points| {
                plane_points
                    .get(&b.face_id)
                    .is_some_and(|other| points.intersection(other).take(2).count() >= 2)
            });
            let overlapping_footprints = face_bounds
                .get(&a.face_id)
                .zip(face_bounds.get(&b.face_id))
                .is_some_and(|(a, b)| {
                    // Coplanar faces have one near-zero local dimension. Their
                    // remaining footprint must overlap or touch on both axes;
                    // this joins overlapping mirror faces without grouping
                    // unrelated coplanar faces elsewhere in the body.
                    (0..3)
                        .filter(|&axis| {
                            a.1[axis].min(b.1[axis]) >= a.0[axis].max(b.0[axis]) - 1.0e-3
                        })
                        .count()
                        >= 2
                });
            if dot > 0.999 && plane_delta < 1.0e-3 && (shared || overlapping_footprints) {
                let canonical = remap[&a.face_id].min(remap[&b.face_id]);
                let old_a = remap[&a.face_id];
                let old_b = remap[&b.face_id];
                for value in remap.values_mut() {
                    if *value == old_a || *value == old_b {
                        *value = canonical;
                    }
                }
            }
        }
    }
    for fid in &mut mesh.face_ids {
        *fid = remap.get(fid).copied().unwrap_or(*fid);
    }
    let topology: HashMap<_, _> = mesh
        .face_refs
        .iter()
        .filter_map(|face| {
            face.topology
                .clone()
                .map(|topology| (remap[&face.face_id], topology))
        })
        .collect();
    mesh.face_refs =
        crate::mock_kernel::mesh_face_refs(&mesh.vertices, &mesh.indices, &mesh.face_ids);
    for face in &mut mesh.face_refs {
        face.topology = topology.get(&face.face_id).cloned();
    }
}

fn suppress_edges_inside_other_parts(
    mesh: &mut MockMesh,
    owner: usize,
    parts: &[KernelSolid],
    raw_meshes: &[MockMesh],
) {
    let mut hidden_segments = std::collections::HashSet::new();
    for (segment, pair) in mesh.edge_indices.chunks_exact(2).enumerate() {
        let read = |vertex: u32| {
            let base = vertex as usize * 3;
            [
                mesh.edge_vertices[base],
                mesh.edge_vertices[base + 1],
                mesh.edge_vertices[base + 2],
            ]
        };
        let a = read(pair[0]);
        let b = read(pair[1]);
        let midpoint = [
            (a[0] + b[0]) * 0.5,
            (a[1] + b[1]) * 0.5,
            (a[2] + b[2]) * 0.5,
        ];
        let mut probes = vec![midpoint];
        if let Some(normals) = mesh.edge_face_normals.get(segment * 6..segment * 6 + 6) {
            for normal in [
                [normals[0], normals[1], normals[2]],
                [normals[3], normals[4], normals[5]],
            ] {
                probes.push([
                    midpoint[0] - normal[0] * 1.0e-3,
                    midpoint[1] - normal[1] * 1.0e-3,
                    midpoint[2] - normal[2] * 1.0e-3,
                ]);
            }
        }
        let inside_other = parts.iter().enumerate().any(|(index, part)| {
            index != owner
                && probes.iter().any(|probe| {
                    openrcad::prelude::boolean::point_in_solid(
                        &openrcad::foundation::Pnt::new(
                            probe[0] as f64,
                            probe[1] as f64,
                            probe[2] as f64,
                        ),
                        part,
                    )
                })
        });
        // A coincident exterior boundary is present in both input meshes. Keep
        // both spans for now: the global regrouping below unions/collapses them
        // into one full edge. An internal cross-boundary has no collinear mate
        // in the other mesh and is correctly removed here.
        let has_coincident_exterior = raw_meshes.iter().enumerate().any(|(index, other)| {
            index != owner
                && (0..other.edge_indices.len() / 2).any(|other_segment| {
                    let (c, d) = edge_segment_points(other, other_segment);
                    collinear_edge_points_overlap(a, b, c, d)
                })
        });
        if inside_other && !has_coincident_exterior {
            hidden_segments.insert(segment);
        }
    }
    if hidden_segments.is_empty() {
        return;
    }
    retain_edge_segments(mesh, |segment, _| !hidden_segments.contains(&segment));
}

fn retain_edge_segments(mesh: &mut MockMesh, keep: impl Fn(usize, u32) -> bool) {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut normals = Vec::new();
    let mut groups = Vec::new();
    for (segment, pair) in mesh.edge_indices.chunks_exact(2).enumerate() {
        let group = mesh
            .edge_groups
            .get(segment)
            .copied()
            .unwrap_or(segment as u32);
        if !keep(segment, group) {
            continue;
        }
        let base = (vertices.len() / 3) as u32;
        for &vertex in pair {
            let offset = vertex as usize * 3;
            vertices.extend_from_slice(&mesh.edge_vertices[offset..offset + 3]);
        }
        indices.extend_from_slice(&[base, base + 1]);
        if mesh.edge_face_normals.len() >= (segment + 1) * 6 {
            normals.extend_from_slice(&mesh.edge_face_normals[segment * 6..segment * 6 + 6]);
        }
        groups.push(group);
    }
    let kept: std::collections::HashSet<_> = groups.iter().copied().collect();
    mesh.edge_refs.retain(|edge| kept.contains(&edge.group));
    mesh.edge_vertices = vertices;
    mesh.edge_indices = indices;
    mesh.edge_face_normals = normals;
    mesh.edge_groups = groups;
}

fn suppress_edges_on_plane(mesh: &mut MockMesh, origin: Vec3, normal: Vec3) {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut normals = Vec::new();
    let mut groups = Vec::new();
    for (segment, pair) in mesh.edge_indices.chunks_exact(2).enumerate() {
        let ia = pair[0] as usize * 3;
        let ib = pair[1] as usize * 3;
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
        if plane_distance(a, origin, normal).abs() < 1.0e-3
            && plane_distance(b, origin, normal).abs() < 1.0e-3
        {
            continue;
        }
        let base = (vertices.len() / 3) as u32;
        vertices.extend_from_slice(&a);
        vertices.extend_from_slice(&b);
        indices.extend_from_slice(&[base, base + 1]);
        if mesh.edge_face_normals.len() >= (segment + 1) * 6 {
            normals.extend_from_slice(&mesh.edge_face_normals[segment * 6..segment * 6 + 6]);
        }
        groups.push(
            mesh.edge_groups
                .get(segment)
                .copied()
                .unwrap_or(segment as u32),
        );
    }
    let kept: std::collections::HashSet<u32> = groups.iter().copied().collect();
    mesh.edge_refs.retain(|edge| kept.contains(&edge.group));
    mesh.edge_vertices = vertices;
    mesh.edge_indices = indices;
    mesh.edge_face_normals = normals;
    mesh.edge_groups = groups;
}

/// Tessellate each assembled body: reuse the analytic mesh when the body was
/// never touched by a boolean, else extract a mesh from its kernel solid parts.
/// Bodies that tessellate to nothing are dropped.
/// Re-derive a sketch-on-face coordinate system from the current geometry: find
/// the body the captured face belongs to (already assembled into `live`), resolve
/// the face there, and build a plane from its centroid + normal. `None` if the
/// body or face is gone (the caller then keeps the frozen placement).
fn rederive_sketch_cs(face_ref: &FaceRef, live: &[LiveBody]) -> Option<CoordinateSystem> {
    let body_id = face_ref.topology.as_ref()?.body_id.as_deref()?;
    let body = live.iter().find(|b| b.id == body_id)?;
    let resolved = resolve_face_ref_by_topology(body, face_ref)?;
    Some(cs_from_face(resolved.centroid, resolved.normal))
}

/// Re-project a face-attached sketch's boundary outline from wherever its face
/// is NOW, into `cs` (the sketch's already re-derived placement plane, so the
/// outline and the drawn curves land in the same 2D space). Resolves the face
/// the same way plane re-derivation does (topology name first, geometry
/// fallback), then reads the numeric mesh face id off the nearest matching
/// `MeshFaceRef` and extracts its boundary loops with the SAME shared code the
/// GUI used at capture time — an unchanged face reproduces the stored snapshot
/// bit-for-bit, which is the caller's cheap "did anything move?" test.
fn rederive_face_boundary(
    face_ref: &FaceRef,
    live: &[LiveBody],
    cs: &CoordinateSystem,
) -> Option<SketchCurves> {
    let body_id = face_ref.topology.as_ref()?.body_id.as_deref()?;
    let body = live.iter().find(|b| b.id == body_id)?;
    let resolved = resolve_face_ref_by_topology(body, face_ref)?;
    let pick = |mesh: &MockMesh| -> Option<SketchCurves> {
        let f = mesh
            .face_refs
            .iter()
            .filter(|c| dot3(c.normal, resolved.normal) >= 0.99)
            .min_by(|a, b| {
                distance3(a.centroid, resolved.centroid)
                    .partial_cmp(&distance3(b.centroid, resolved.centroid))
                    .unwrap_or(std::cmp::Ordering::Equal)
            })?;
        if distance3(f.centroid, resolved.centroid) > 1.0e-3 {
            return None;
        }
        let boundary = crate::mock_kernel::mesh_face_boundary_2d(mesh, f.face_id, cs);
        (!boundary.is_empty()).then_some(boundary)
    };
    body.pristine
        .as_deref()
        .and_then(pick)
        .or_else(|| pick(&edge_mod_reference_mesh(body)))
}

/// A sketch-plane coordinate system for a face at `centroid` with outward
/// `normal`. In-plane axes are chosen deterministically (Y×n, or X×n for a
/// horizontal face), mirroring the GUI's `face_cs` so a re-derived plane matches
/// the one first picked.
fn cs_from_face(centroid: [f32; 3], normal: [f32; 3]) -> CoordinateSystem {
    let cross = |a: [f32; 3], b: [f32; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let norm = |v: [f32; 3]| {
        let l = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
        if l > 1.0e-9 {
            [v[0] / l, v[1] / l, v[2] / l]
        } else {
            v
        }
    };
    let n = norm(normal);
    let mut u = cross([0.0, 1.0, 0.0], n);
    if (u[0] * u[0] + u[1] * u[1] + u[2] * u[2]).sqrt() < 1.0e-4 {
        u = cross([1.0, 0.0, 0.0], n);
    }
    let u = norm(u);
    let v = norm(cross(n, u));
    CoordinateSystem::new(
        Vec3::new(centroid[0], centroid[1], centroid[2]),
        Vec3::new(u[0], u[1], u[2]),
        Vec3::new(v[0], v[1], v[2]),
    )
}

pub(crate) fn tessellate_bodies(live: Vec<LiveBody>) -> Vec<(String, MockMesh)> {
    tessellate_bodies_with_cancel(live, None).expect("uncancellable tessellation cannot cancel")
}

/// Global evaluator invariant: every runtime part is exactly one connected,
/// watertight, healthy B-Rep component. Features are evaluated against a cloned
/// pre-state and rolled back if this check fails, so invalid kernel output can
/// never poison later history or be serialized into a hydrated checkpoint.
fn validate_live_body_state(live: &[LiveBody]) -> Result<(), String> {
    let validate_parts = |owner: &str, parts: &[KernelSolid]| -> Result<(), String> {
        for (index, part) in parts.iter().enumerate() {
            if !part.is_watertight() {
                return Err(format!("{owner} part {index} is not watertight"));
            }
            if !part.health_report().is_healthy() {
                return Err(format!("{owner} part {index} is topologically unhealthy"));
            }
            let components = part.split_disconnected();
            if components.len() > 1
                && !crate::mock_kernel::components_form_connected_material(&components)
            {
                return Err(format!(
                    "{owner} part {index} contains multiple disconnected components"
                ));
            }
        }
        Ok(())
    };

    for body in live {
        validate_parts(&format!("body '{}'", body.id), &body.parts)?;
        if let Some(replay) = &body.thread_replay {
            validate_parts(
                &format!("body '{}' thread replay", body.id),
                &replay.base_parts,
            )?;
        }
        if let Some(replay) = &body.cut_replay {
            validate_parts(
                &format!("body '{}' cut replay", body.id),
                &replay.base_parts,
            )?;
        }
    }
    Ok(())
}

fn tessellate_bodies_with_cancel(
    live: Vec<LiveBody>,
    cancellation: Option<&EvaluationCancellation>,
) -> Result<Vec<(String, MockMesh)>, EvaluationError> {
    let mut bodies: Vec<(String, MockMesh)> = Vec::new();
    for body in live {
        if cancellation.is_some_and(EvaluationCancellation::is_cancelled) {
            return Err(EvaluationError::Cancelled);
        }
        let mesh = match body.pristine {
            Some(m) => {
                let mut mesh = (*m).clone();
                crate::mock_kernel::stamp_body_face_components(&mut mesh, &body.id, &body.parts);
                mesh
            }
            None => {
                let mut m = MockMesh::empty();
                for part in &body.parts {
                    let mut part_mesh = match cancellation {
                        Some(cancel) => MockMesh::from_solid_with_cancel(part, cancel)
                            .map_err(|_| EvaluationError::Cancelled)?,
                        None => MockMesh::from_solid(part),
                    };
                    crate::mock_kernel::stamp_face_component(&mut part_mesh, &body.id, part);
                    m.append(part_mesh);
                }
                m
            }
        };
        if !mesh.indices.is_empty() {
            bodies.push((body.id, mesh));
        }
    }
    Ok(bodies)
}

/// Upper bound on distinct sketch states retained in [`ParametricGraph::region_cache`].
/// Each edit to a sketch produces a new key; this caps memory across a long
/// session. The cache is a pure accelerator, so clearing it on overflow only
/// costs a one-time recompute.
pub(crate) const REGION_CACHE_CAP: usize = 256;

/// A 64-bit content hash of a sketch's curves, used as the region-cache key.
/// f32 isn't `Hash`, so we hash the raw bit patterns; two `SketchCurves` that
/// are bit-identical (the common case across preview frames) hash equal, which
/// is exactly when [`detect_regions`] would return the same regions.
pub(crate) fn hash_curves(c: &SketchCurves) -> u64 {
    let mut h = std::collections::hash_map::DefaultHasher::new();
    h.write_usize(c.segments.len());
    for s in &c.segments {
        for v in [s.a.0, s.a.1, s.b.0, s.b.1] {
            h.write_u32(v.to_bits());
        }
    }
    h.write_usize(c.circles.len());
    for circle in &c.circles {
        for v in [circle.center.0, circle.center.1, circle.radius] {
            h.write_u32(v.to_bits());
        }
    }
    h.write_usize(c.arcs.len());
    for arc in &c.arcs {
        for v in [
            arc.center.0,
            arc.center.1,
            arc.radius,
            arc.start.0,
            arc.start.1,
            arc.end.0,
            arc.end.1,
        ] {
            h.write_u32(v.to_bits());
        }
    }
    h.finish()
}

/// Fold a feature into the running [`eval_prefix_keys`] hash via its serde form.
/// JSON of a given value is deterministic across frames (identical floats format
/// identically), so two bit-identical features hash equal — exactly when they
/// build identical geometry. A trailing separator keeps adjacent fields from
/// running together. Serialization can't realistically fail here; if it ever did,
/// skipping the bytes only risks a missed invalidation, never a crash.
pub(crate) fn fold_feature(h: &mut impl Hasher, f: &FeatureType) {
    if let Ok(bytes) = serde_json::to_vec(f) {
        h.write(&bytes);
    }
    h.write_u8(0xfe);
}

/// Creation order for a node id: the trailing numeric suffix (`extrude_12` → 12)
/// from the shared monotonic counter. Ids without a suffix (e.g. `origin`) sort
/// first. This is stable across deletions, unlike petgraph's `NodeIndex`.
pub(crate) fn creation_key(id: &str) -> u64 {
    id.rsplit('_')
        .next()
        .and_then(|s| s.parse::<u64>().ok())
        .unwrap_or(0)
}

#[cfg(test)]
mod mirror_join_display_tests {
    use super::*;

    #[test]
    fn overlapping_mirror_fallback_is_one_continuous_display_body() {
        let source = openrcad::primitives::make_box(
            &openrcad::foundation::Pnt::new(-1.0, 0.0, 0.0),
            4.0,
            2.0,
            1.0,
        );
        let mirrored = openrcad::primitives::make_box(
            &openrcad::foundation::Pnt::new(-3.0, 0.0, 0.0),
            4.0,
            2.0,
            1.0,
        );
        let mesh =
            mirror_join_display_mesh("test_mirror", &[source, mirrored], Vec3::ZERO, Vec3::X);

        assert_eq!(
            mesh.face_refs
                .iter()
                .filter(|face| face.normal[2] > 0.99)
                .count(),
            1,
            "overlapping coplanar top faces should select as one face"
        );
        let internal_outline_segments = mesh
            .edge_indices
            .chunks_exact(2)
            .filter(|edge| {
                let x = |vertex: u32| mesh.edge_vertices[vertex as usize * 3];
                [1.0, -1.0].iter().any(|cut| {
                    (x(edge[0]) - cut).abs() < 1.0e-3 && (x(edge[1]) - cut).abs() < 1.0e-3
                })
            })
            .count();
        assert_eq!(
            internal_outline_segments, 0,
            "overlap boundaries inside the joined result must not be drawn or picked"
        );
        let full_width_edges = mesh
            .edge_refs
            .iter()
            .filter(|edge| {
                let lo = edge.p0[0].min(edge.p1[0]);
                let hi = edge.p0[0].max(edge.p1[0]);
                (lo + 3.0).abs() < 1.0e-3 && (hi - 3.0).abs() < 1.0e-3
            })
            .count();
        assert!(
            full_width_edges >= 1,
            "the mirrored halves' collinear boundary must regroup into one full-width edge"
        );
    }
}
