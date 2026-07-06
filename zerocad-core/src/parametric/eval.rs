use super::*;

impl ParametricGraph {
    pub fn new() -> Self {
        let mut pg = Self {
            graph: DiGraph::new(),
            sketch_face_refs: HashMap::new(),
            sketch_datum_refs: HashMap::new(),
            node_map: HashMap::new(),
            region_cache: RefCell::new(HashMap::new()),
            eval_cache: RefCell::new(EvalCache::default()),
        };
        pg.bootstrap_origin();
        pg
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
        if let (Some(parent_idx), Some(child_idx)) =
            (self.resolve_node(parent_id), self.resolve_node(child_id))
        {
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
        self.rebuild_node_map();
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
    /// raised while assembling the model — e.g. a Cut whose boolean the solver
    /// could not resolve (so material was left intact) or a Join that overlapped
    /// nothing (so it became a separate body). These are results the user did
    /// *not* ask for, so the GUI surfaces them rather than letting the model
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
            .last()
            .map(|cp| cp.statuses.clone())
            .unwrap_or_default();
        Ok((tessellate_bodies(live), warnings, statuses))
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
        let (live, warnings) = self.build_live(hidden, draft)?;
        Ok((tessellate_bodies(live), warnings))
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
            let mut m = 0;
            while m < keys.len() && m < cps.len() && keys[m] == cps[m].key {
                m += 1;
            }
            if m > 0 {
                let cp = &cps[m - 1];
                (
                    cp.live.clone(),
                    cp.warnings.clone(),
                    cp.statuses.clone(),
                    m,
                    cps[..m].to_vec(),
                )
            } else {
                (Vec::new(), Vec::new(), Vec::new(), 0usize, Vec::new())
            }
        };

        for (i, &idx) in nodes.iter().enumerate() {
            // Reused prefix: its checkpoints (and so its `live`/`warnings`) were
            // restored above; skip recomputing it.
            if i < reuse {
                continue;
            }
            let node = &self.graph[idx];
            let warn_before = warnings.len();
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
                            pristine: Some(pristine),
                            sketch_source: Some(source),
                            cut_tools: Vec::new(),
                            cut_replay: None,
                            edge_mod_cut_history_path_used: false,
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
                                pristine: Some(pristine),
                                sketch_source: None,
                                cut_tools: Vec::new(),
                                cut_replay: None,
                                edge_mod_cut_history_path_used: false,
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
                                    pristine: Some(pristine),
                                    sketch_source: None,
                                    cut_tools: Vec::new(),
                                    cut_replay: None,
                                    edge_mod_cut_history_path_used: false,
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
                        scope,
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
                            scope,
                            replay,
                            eff_dist,
                            *kind,
                            draft,
                            &mut live,
                            &mut warnings,
                        );
                    }
                    _ => {}
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
            checkpoints.push(EvalCheckpoint {
                key: keys[i],
                live: live.clone(),
                warnings: warnings.clone(),
                statuses: statuses.clone(),
            });
        }

        *self.eval_cache.borrow_mut() = EvalCache { checkpoints };

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
    /// NOTE: the `draft` flag is deliberately NOT folded in — it is currently a
    /// no-op (`apply_edge_mod` ignores it), so draft previews and committed
    /// rebuilds produce identical geometry and should share cached checkpoints. If
    /// `draft` is ever made to change geometry again (e.g. a faceted draft fillet),
    /// it MUST be folded into the seed here, or a draft preview would serve a
    /// committed body's cached result (and vice versa).
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
                    solver.as_ref(),
                    vars,
                );
                // Fail-loud: a variable-driven constraint model that no longer
                // solves keeps its last-valid geometry, and the failure reason
                // rides along so the consuming extrude can report it.
                let solve_failure = solver
                    .as_ref()
                    .filter(|m| !m.is_empty() && crate::sketch::solve::has_variable_bound_constraint(m))
                    .and_then(|m| {
                        let report = crate::sketch::solve_model(m, vars);
                        match report.outcome {
                            crate::sketch::SolveOutcome::Converged => None,
                            crate::sketch::SolveOutcome::DidNotConverge => Some(
                                "its constraints did not converge; keeping the last valid geometry"
                                    .to_string(),
                            ),
                            crate::sketch::SolveOutcome::Conflicting => Some(match report
                                .conflicting
                            {
                                Some(id) => format!(
                                    "its constraints conflict (constraint {}); keeping the last \
                                     valid geometry",
                                    id.0
                                ),
                                None => "its constraints conflict; keeping the last valid \
                                         geometry"
                                    .to_string(),
                            }),
                        }
                    });
                let regions = self.cached_regions(&effective);
                let provenance =
                    build_region_provenance(&effective, shapes, entity_ids, &regions);
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
                        | FeatureType::Hole { .. }
                        | FeatureType::Shell { .. }
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
        let datum_cs = self.sketch_datum_refs.get(sketch_id).and_then(|datum_id| {
            match datums.get(datum_id) {
                Some(DatumValue::Plane(cs)) => Some(*cs),
                _ => {
                    warnings.push(format!(
                        "Extrude '{node_id}': sketch '{sketch_id}' is attached to datum plane \
                         '{datum_id}', which did not resolve; using the sketch's saved plane."
                    ));
                    None
                }
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
        let regions = &sketch.regions;
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
        let take_all = region_indices.is_empty();
        let loops = &sketch.shape_loops;
        let clusters = if loops.is_empty() {
            Vec::new()
        } else {
            crate::sketch::overlap_clusters(loops)
        };
        let mut shape_cluster = vec![usize::MAX; loops.len()];
        for (ci, c) in clusters.iter().enumerate() {
            for &s in c {
                shape_cluster[s] = ci;
            }
        }
        let cluster_is_multi: Vec<bool> = clusters.iter().map(|c| c.len() >= 2).collect();
        let selected_mask = selected_shape_mask(regions, region_indices, loops);
        // Per region: is it part of a multi-shape boolean cluster, and (if so)
        // should it be kept? `region_is_boolean` regions ignore `region_indices`
        // (the shape selection decides); other regions use the normal rule.
        let mut region_is_boolean = vec![false; regions.len()];
        let mut process_region = vec![false; regions.len()];
        for (i, r) in regions.iter().enumerate() {
            let interior = region_material_point(r);
            let containing = region_containing_shapes(interior, loops);
            let in_multi = containing
                .iter()
                .any(|&s| cluster_is_multi[shape_cluster[s]]);
            if in_multi {
                let in_base = containing.iter().any(|&s| selected_mask[s]);
                let in_tool = containing.iter().any(|&s| !selected_mask[s]);
                region_is_boolean[i] = true;
                process_region[i] = in_base && !in_tool;
            } else {
                process_region[i] = take_all || region_indices.contains(&i);
            }
        }
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
        let mut newbody_body_count = 0usize;
        let mut newbody_cut_replay: Option<CutReplayHistory> = None;

        for (i, region) in regions.iter().enumerate() {
            if !process_region[i] {
                continue;
            }
            if region_is_boolean[i] && matches!(mode, ExtrudeMode::NewBody) {
                newbody_has_boolean = true;
            }
            let provenance = sketch.provenance.get(i);
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
                if newbody_has_boolean {
                    // Fuse the cluster's adjacent kept regions so a union reads as
                    // one solid for later edge-mods. The clean per-region analytic
                    // mesh accumulated in `newbody_mesh` still drives display
                    // (kept as `pristine`), so this only affects the kernel parts.
                    newbody_tools = fuse_overlapping_solids(newbody_tools);
                }
                if !newbody_tools.is_empty() {
                    live.push(LiveBody {
                        id: node_id.to_string(),
                        parts: newbody_tools,
                        pristine: (!newbody_mesh.indices.is_empty()).then_some(newbody_mesh),
                        sketch_source: (!sketch_source.regions.is_empty()).then_some(sketch_source),
                        cut_tools: newbody_cut_tools,
                        cut_replay: (newbody_body_count == 1)
                            .then_some(())
                            .and(newbody_cut_replay),
                        edge_mod_cut_history_path_used: false,
                    });
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
                            if mode == ExtrudeMode::Cut { "Cut" } else { "Join" },
                        ));
                        return;
                    }
                }
                if mode == ExtrudeMode::Join {
                    apply_join(live, node_id, join_tools, boolean_target, warnings);
                } else {
                    apply_cut(live, node_id, cut_tools, boolean_target, warnings);
                }
            }
        }
    }
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
        let datum_cs = self.sketch_datum_refs.get(sketch_id).and_then(|datum_id| {
            match datums.get(datum_id) {
                Some(DatumValue::Plane(cs)) => Some(*cs),
                _ => None,
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
                        pristine: (!newbody_mesh.indices.is_empty()).then_some(newbody_mesh),
                        sketch_source: None,
                        cut_tools: Vec::new(),
                        cut_replay: None,
                        edge_mod_cut_history_path_used: false,
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
                    apply_join(live, node_id, join_tools, boolean_target, warnings);
                } else {
                    apply_cut(live, node_id, cut_tools, boolean_target, warnings);
                }
            }
        }
    }
}

/// Evaluate one Shell node: hollow the target body in place. Open faces are
/// resolved geometrically (captured centroid+normal → nearest matching kernel
/// face); the shelled solid replaces the body's parts and its display mesh is
/// re-derived from the result.
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
#[allow(clippy::too_many_arguments)]
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
    // to subtract a wide cylinder around an existing COAXIAL bore wall (no
    // wall-wall intersection curve), which it rejects.
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
    let bore = crate::mock_kernel::cylinder_tool_at(
        start,
        dir,
        diameter as f64 / 2.0,
        bore_len as f64,
    );
    match bore {
        Some(tool) => cut_tools.push(CutTool::single_direction(Some(tool), None, None, None)),
        None => {
            warnings.push(format!("Hole '{node_id}': bore cutter failed to build."));
            return;
        }
    }
    apply_cut(live, node_id, cut_tools, Some(target), warnings);
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
        PatternKind::Mirror { plane } => {
            let Some(cs) = super::datum::resolve_plane_base_world(plane, datums) else {
                warnings.push(format!(
                    "Pattern '{node_id}': its mirror plane could not be resolved."
                ));
                return;
            };
            let frame = Ax2::new(to_pnt(cs.origin), to_dir(cs.n));
            vec![(Trsf::mirror_plane(&frame), true)]
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
    if !new_parts.is_empty() {
        live.push(LiveBody {
            id: node_id.to_string(),
            parts: new_parts,
            pristine: (!mesh.indices.is_empty()).then_some(mesh),
            sketch_source: None,
            cut_tools: Vec::new(),
            cut_replay: None,
            edge_mod_cut_history_path_used: false,
        });
    }
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
    let mut bodies: Vec<(String, MockMesh)> = Vec::new();
    for body in live {
        let mesh = match body.pristine {
            Some(m) => m,
            None => {
                let mut m = MockMesh::empty();
                for part in &body.parts {
                    m.append(MockMesh::from_solid(part));
                }
                m
            }
        };
        if !mesh.indices.is_empty() {
            bodies.push((body.id, mesh));
        }
    }
    bodies
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
