use crate::geometry::CoordinateSystem;
use crate::mock_kernel::{KernelSolid, MockMesh};
use crate::sketch::{detect_regions, Region, SketchCurves};
use crate::units::Unit;
use petgraph::algo::toposort;
use petgraph::graph::{DiGraph, NodeIndex};
use std::cell::RefCell;
use std::collections::HashMap;
use std::hash::Hasher;

/// How an extrude combines with the bodies already in the model.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ExtrudeMode {
    /// Create a standalone new body (the historical default).
    NewBody,
    /// Fuse (union) the extruded volume into any body it overlaps.
    Join,
    /// Subtract (difference) the extruded volume from any body in its path.
    Cut,
}

impl Default for ExtrudeMode {
    fn default() -> Self {
        ExtrudeMode::NewBody
    }
}

/// A single named, dimensioned value inside a [`FeatureType::VariableSet`].
/// `value` is expressed in `unit` (the same units offered in Settings), so the
/// UI can display it directly and convert to the base unit when needed.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Variable {
    pub name: String,
    pub value: f64,
    pub unit: Unit,
}

impl Variable {
    /// A fresh variable with a placeholder name, zero value, and the given unit.
    pub fn new(name: impl Into<String>, unit: Unit) -> Self {
        Self {
            name: name.into(),
            value: 0.0,
            unit,
        }
    }

    /// The variable's value converted to the base unit (millimeters).
    pub fn value_in_base(&self) -> f64 {
        self.unit.to_base(self.value)
    }
}

/// A solid edge captured geometrically for a 3D fillet/chamfer. The endpoints
/// and the two adjacent face normals are recorded in **world space** at
/// selection time (read straight from the body's wireframe — see
/// `MockMesh::edge_vertices` / `edge_face_normals`), which is all
/// [`crate::mock_kernel::edge_corner_cutter`] needs to orient its cutter.
///
/// Because the edge is frozen in world space, an `EdgeMod` does not follow an
/// upstream dimension change; if the body it targets is later resized by a
/// variable, the modifier still cuts at the captured location (the guarded
/// boolean degrades gracefully if that no longer meets material).
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct EdgeRef {
    pub p0: [f32; 3],
    pub p1: [f32; 3],
    pub n1: [f32; 3],
    pub n2: [f32; 3],
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub enum FeatureType {
    Origin,
    Box {
        w: f32,
        h: f32,
        d: f32,
    },
    Cylinder {
        r: f32,
        h: f32,
    },
    /// 2D sketch on a plane, holding the raw drawn curves. Region detection
    /// (the "faces" Fusion 360 auto-creates from intersecting shapes) is
    /// performed by the evaluator on demand.
    Sketch {
        /// The full plane the sketch lives on (origin + axes + normal), so a
        /// sketch can sit on an origin plane OR on an arbitrary body face.
        cs: crate::geometry::CoordinateSystem,
        /// Baked geometry. For parametric sketches this is a snapshot (the live
        /// geometry is rebuilt from `shapes`); for documents saved before
        /// `shapes` existed it is the authoritative geometry.
        curves: SketchCurves,
        /// Parametric construction: the shapes the sketch was drawn from, whose
        /// dimensions may reference variables. When non-empty this is the source
        /// of truth — the effective curves are rebuilt from it against the
        /// current variables (so a dimension follows its variable). Empty for
        /// legacy documents, which fall back to `curves`.
        #[serde(default)]
        shapes: Vec<crate::sketch::SketchShape>,
        /// Fillet/chamfer modifiers applied to corners after the shapes are
        /// built (see [`crate::sketch::effective_curves`]).
        #[serde(default)]
        corner_mods: Vec<crate::sketch::CornerMod>,
        /// True when the sketch was placed on an existing body face rather than
        /// an origin plane. Lets the extrude tool default to Join/Cut (combine
        /// with that body) instead of New Body. Defaults to `false` for
        /// documents saved before this field existed.
        #[serde(default)]
        on_face: bool,
    },
    /// Extrude one or more detected regions of the parent sketch by `depth`.
    /// `region_indices` selects which regions to extrude — empty means "all".
    /// `mode` decides whether the result is a new body, joined to existing
    /// bodies, or cut out of them.
    Extrude {
        depth: f32,
        region_indices: Vec<usize>,
        #[serde(default)]
        mode: ExtrudeMode,
        /// Optional expression (over the document's variables) that drives the
        /// depth. When set, it is re-evaluated against the current variables on
        /// every build, so editing a variable updates the extrude. `depth` then
        /// holds the last resolved value (a fallback when a variable is missing).
        #[serde(default)]
        depth_expr: Option<String>,
    },
    /// Round (fillet) or bevel (chamfer) one edge of an existing solid body by
    /// `dist`. Applied as a guarded boolean subtraction of an edge-aligned cutter
    /// during evaluation (see [`apply_edge_mod`]). `target` is the node id of the
    /// body it modifies; the modifier is processed after that body in creation
    /// order, like a cut extrude.
    EdgeMod {
        /// Node id of the body being modified.
        target: String,
        /// The edge to round/bevel, captured in world space.
        edge: EdgeRef,
        /// Fillet radius / chamfer setback, in base units (mm).
        dist: f32,
        /// Optional expression (over the document's variables) driving `dist`,
        /// re-evaluated every build. `dist` holds the last resolved value.
        #[serde(default)]
        dist_expr: Option<String>,
        /// Whether to round (Fillet) or bevel (Chamfer) the edge.
        kind: crate::sketch::CornerKind,
    },
    /// A named collection of parametric variables. Carries no geometry — it's a
    /// container the user fills with dimensioned values for later reference.
    VariableSet {
        variables: Vec<Variable>,
    },
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct FeatureNode {
    pub id: String,
    pub name: String,
    pub feature: FeatureType,
}

#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct ParametricGraph {
    pub graph: DiGraph<FeatureNode, ()>,
    #[serde(skip)]
    node_map: HashMap<String, NodeIndex>,
    /// Memoized planar-arrangement results, keyed by a content hash of a
    /// sketch's curves (see [`hash_curves`]). [`detect_regions`] is a pure,
    /// O(n²) function of the curves, so the same sketch yields the same regions
    /// every evaluation. Caching across calls keeps region detection off the
    /// hot path of extrude-drag previews, which re-evaluate the whole model on
    /// every frame while the sketches themselves never change. Skipped by serde
    /// and carried by `Clone` (so the per-frame graph clone in the preview path
    /// starts warm); it is a transparent accelerator, never persisted state.
    #[serde(skip)]
    region_cache: RefCell<HashMap<u64, Vec<Region>>>,
    /// Per-node geometry checkpoints from the previous evaluation, used to skip
    /// re-solving the unchanged prefix of the feature tree. Each entry holds the
    /// assembled bodies *after* one node, keyed by a cumulative content hash of
    /// every input that node's geometry depends on (see [`evaluate_bodies_inner`]).
    /// When an edit changes only a trailing node — e.g. dragging a fillet/chamfer
    /// radius — the prefix hashes still match, so the expensive upstream booleans
    /// are restored from here instead of recomputed every frame. Skipped by serde
    /// and a transparent accelerator (dropping it only costs a one-time rebuild).
    #[serde(skip)]
    eval_cache: RefCell<EvalCache>,
}

/// Checkpoints of [`evaluate_bodies_inner`], one per processed body node, in
/// creation order. A pure accelerator — see [`ParametricGraph::eval_cache`].
#[derive(Debug, Clone, Default)]
struct EvalCache {
    checkpoints: Vec<EvalCheckpoint>,
}

/// The assembled bodies and accumulated warnings immediately after one node was
/// applied, tagged with the cumulative hash of all geometry inputs up to it.
#[derive(Debug, Clone)]
struct EvalCheckpoint {
    key: u64,
    live: Vec<LiveBody>,
    warnings: Vec<String>,
}

impl ParametricGraph {
    pub fn new() -> Self {
        let mut pg = Self {
            graph: DiGraph::new(),
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
        if let (Some(&parent_idx), Some(&child_idx)) =
            (self.node_map.get(parent_id), self.node_map.get(child_id))
        {
            self.graph.add_edge(parent_idx, child_idx, ());
        }
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

    /// Whether the (non-hidden) model contains a 3D fillet whose committed,
    /// arc-cutter geometry differs from its fast faceted draft. The GUI uses this
    /// to decide if a background refine pass is worth spawning: with no such
    /// fillet the draft result *is* the final result, so it skips the extra
    /// (and otherwise redundant) async evaluation. Chamfers are excluded — their
    /// bevel is a single planar face either way, so draft and final agree.
    pub fn has_arc_fillet(&self, hidden: &std::collections::HashSet<String>) -> bool {
        self.graph.node_weights().any(|n| {
            !hidden.contains(&n.id)
                && matches!(
                    &n.feature,
                    FeatureType::EdgeMod {
                        kind: crate::sketch::CornerKind::Fillet,
                        ..
                    }
                )
        })
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

    fn build_live(
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

        // Body-eval nodes in creation order, with a cumulative content hash after
        // each one (see [`eval_prefix_keys`]). An edit that touches only a trailing
        // node — dragging a fillet/chamfer radius, say — leaves every earlier key
        // identical, so the matching prefix (and its expensive booleans) is
        // restored from the previous evaluation instead of recomputed.
        let nodes: Vec<NodeIndex> = self.body_nodes_in_creation_order();
        let keys = self.eval_prefix_keys(&nodes, hidden, &vars);

        let (mut live, mut warnings, reuse, mut checkpoints) = {
            let cache = self.eval_cache.borrow();
            let cps = &cache.checkpoints;
            let mut m = 0;
            while m < keys.len() && m < cps.len() && keys[m] == cps[m].key {
                m += 1;
            }
            if m > 0 {
                let cp = &cps[m - 1];
                (cp.live.clone(), cp.warnings.clone(), m, cps[..m].to_vec())
            } else {
                (Vec::new(), Vec::new(), 0usize, Vec::new())
            }
        };

        for (i, &idx) in nodes.iter().enumerate() {
            // Reused prefix: its checkpoints (and so its `live`/`warnings`) were
            // restored above; skip recomputing it.
            if i < reuse {
                continue;
            }
            let node = &self.graph[idx];
            if !hidden.contains(&node.id) {
            match &node.feature {
                FeatureType::Box { w, h, d } => {
                    live.push(LiveBody {
                        id: node.id.clone(),
                        parts: vec![crate::mock_kernel::box_solid(*w, *h, *d)],
                        pristine: Some(MockMesh::make_box(*w, *h, *d)),
                    });
                }
                FeatureType::Cylinder { r, h } => {
                    if let Some(solid) = crate::mock_kernel::cylinder_solid(*r, *h) {
                        live.push(LiveBody {
                            id: node.id.clone(),
                            parts: vec![solid],
                            pristine: Some(MockMesh::make_cylinder(*r, *h, 32)),
                        });
                    }
                }
                FeatureType::Extrude {
                    depth,
                    region_indices,
                    mode,
                    depth_expr,
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
                        &sketch_cache,
                        &mut live,
                        &mut warnings,
                    );
                }
                FeatureType::EdgeMod {
                    target,
                    edge,
                    dist,
                    dist_expr,
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
                        eff_dist,
                        *kind,
                        draft,
                        &mut live,
                        &mut warnings,
                    );
                }
                _ => {}
            }
            }
            // Snapshot the assembled bodies after this node so a later evaluation
            // that shares this prefix can resume from here.
            checkpoints.push(EvalCheckpoint {
                key: keys[i],
                live: live.clone(),
                warnings: warnings.clone(),
            });
        }

        *self.eval_cache.borrow_mut() = EvalCache { checkpoints };

        Ok((live, warnings))
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
    fn eval_prefix_keys(
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
    fn sketch_region_cache(
        &self,
        vars: &HashMap<String, f64>,
    ) -> HashMap<NodeIndex, (CoordinateSystem, Vec<Region>)> {
        let mut cache = HashMap::new();
        for idx in self.graph.node_indices() {
            if let FeatureType::Sketch {
                cs,
                curves,
                shapes,
                corner_mods,
                ..
            } = &self.graph[idx].feature
            {
                // A parametric sketch is rebuilt from its shapes against the
                // current variables; the region-cache key (a hash of the
                // resolved curves) then changes whenever a variable does.
                let effective = crate::sketch::effective_curves(curves, shapes, corner_mods, vars);
                cache.insert(idx, (*cs, self.cached_regions(&effective)));
            }
        }
        cache
    }

    /// [`detect_regions`] memoized on a content hash of the curves. A miss runs
    /// the O(n²) arrangement once and stores it; identical curves (every frame
    /// of an extrude-drag preview, say) hit the cache. See [`region_cache`].
    fn cached_regions(&self, curves: &SketchCurves) -> Vec<Region> {
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
    fn body_nodes_in_creation_order(&self) -> Vec<NodeIndex> {
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
        sketch_cache: &HashMap<NodeIndex, (CoordinateSystem, Vec<Region>)>,
        live: &mut Vec<LiveBody>,
        warnings: &mut Vec<String>,
    ) {
        // Resolve the parent sketch's plane + regions.
        let parent = self
            .graph
            .neighbors_directed(idx, petgraph::Direction::Incoming)
            .find_map(|p| sketch_cache.get(&p));
        let Some((cs, regions)) = parent else {
            return;
        };
        if regions.is_empty() {
            return;
        }

        // Build solid tool(s) per selected region (empty selector = all regions).
        // New body also accumulates an analytic mesh so pristine bodies keep
        // their nice hidden-line wireframes.
        let take_all = region_indices.is_empty();
        let region_solid = |r: &Region, cs: &CoordinateSystem, d: f32| {
            crate::mock_kernel::extruded_region_solid(&r.boundary, &r.holes, d, cs)
        };
        // The smooth native-cylinder tool for a circular, hole-free region (None
        // otherwise). Tried before the faceted prism so a round boss/pocket reads
        // smooth — the kernel fuses/bores analytic cylinders watertight.
        let cyl_tool = |r: &Region, cs: &CoordinateSystem, d: f32| {
            crate::mock_kernel::circular_cylinder_tool(&r.boundary, &r.holes, d, cs)
        };

        let mut newbody_tools: Vec<KernelSolid> = Vec::new();
        let mut cut_tools: Vec<CutTool> = Vec::new();
        let mut join_tools: Vec<JoinTool> = Vec::new();
        let mut newbody_mesh = MockMesh::empty();

        for (i, region) in regions.iter().enumerate() {
            if !take_all && !region_indices.contains(&i) {
                continue;
            }
            match mode {
                ExtrudeMode::NewBody => {
                    // Prefer the smooth analytic cylinder for a circular profile so
                    // a new-body cylinder stays round if it is later joined/cut
                    // (re-tessellated from this solid); falls back to the prism.
                    if let Some(s) =
                        cyl_tool(region, cs, depth).or_else(|| region_solid(region, cs, depth))
                    {
                        newbody_tools.push(s);
                    }
                    newbody_mesh.append(MockMesh::make_extruded_sketch(
                        &region.boundary,
                        &region.holes,
                        depth,
                        cs,
                    ));
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
                        cut_tools.push(CutTool {
                            smooth,
                            exact,
                            expanded,
                            smooth_rev,
                            exact_rev,
                            expanded_rev,
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
                        join_tools.push(JoinTool { smooth, exact, dipped });
                    }
                }
            }
        }

        match mode {
            ExtrudeMode::NewBody => {
                if !newbody_tools.is_empty() {
                    live.push(LiveBody {
                        id: node_id.to_string(),
                        parts: newbody_tools,
                        pristine: (!newbody_mesh.indices.is_empty()).then_some(newbody_mesh),
                    });
                }
            }
            ExtrudeMode::Join => apply_join(live, node_id, join_tools, warnings),
            ExtrudeMode::Cut => apply_cut(live, node_id, cut_tools, warnings),
        }
    }
}

/// Tessellate each assembled body: reuse the analytic mesh when the body was
/// never touched by a boolean, else extract a mesh from its kernel solid parts.
/// Bodies that tessellate to nothing are dropped.
fn tessellate_bodies(live: Vec<LiveBody>) -> Vec<(String, MockMesh)> {
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

/// A body being assembled during evaluation. `parts` are the kernel solids that
/// make it up (more than one only when disjoint lumps share a node); `pristine`
/// holds the analytic mesh while the body is untouched by any boolean, so plain
/// bodies keep their nice hidden-line wireframes. A boolean clears it, forcing a
/// fresh tessellation from `parts`.
#[derive(Debug, Clone)]
struct LiveBody {
    id: String,
    parts: Vec<KernelSolid>,
    pristine: Option<MockMesh>,
}

/// Upper bound on distinct sketch states retained in [`ParametricGraph::region_cache`].
/// Each edit to a sketch produces a new key; this caps memory across a long
/// session. The cache is a pure accelerator, so clearing it on overflow only
/// costs a one-time recompute.
const REGION_CACHE_CAP: usize = 256;

/// A 64-bit content hash of a sketch's curves, used as the region-cache key.
/// f32 isn't `Hash`, so we hash the raw bit patterns; two `SketchCurves` that
/// are bit-identical (the common case across preview frames) hash equal, which
/// is exactly when [`detect_regions`] would return the same regions.
fn hash_curves(c: &SketchCurves) -> u64 {
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
    h.finish()
}

/// Fold a feature into the running [`eval_prefix_keys`] hash via its serde form.
/// JSON of a given value is deterministic across frames (identical floats format
/// identically), so two bit-identical features hash equal — exactly when they
/// build identical geometry. A trailing separator keeps adjacent fields from
/// running together. Serialization can't realistically fail here; if it ever did,
/// skipping the bytes only risks a missed invalidation, never a crash.
fn fold_feature(h: &mut impl Hasher, f: &FeatureType) {
    if let Ok(bytes) = serde_json::to_vec(f) {
        h.write(&bytes);
    }
    h.write_u8(0xfe);
}

/// How far a tool overshoots the sketch plane to break coplanarity, in mm.
/// Comfortably above the boolean solver's tolerance so the dip is unambiguous,
/// yet small enough to be invisible at part scale.
const CUT_OVERSHOOT: f32 = 0.1;

/// How far a cut tool's side walls are pushed past a coplanar body face, in mm.
/// The in-plane analogue of `CUT_OVERSHOOT` (which handles the end caps).
const CUT_WALL_GROW: f32 = 0.1;

/// A join's tool, tried in order. `smooth` is the true analytic cylinder for a
/// circular boss (the kernel fuses it watertight, so a Ø-boss reads round, not
/// faceted); `exact` is the faceted prism with perfect dimensions; `dipped` is
/// a faceted fallback whose near cap dips into the target to dodge coplanar
/// faces. `smooth` is `None` for non-circular profiles, which fall straight to
/// the prism.
struct JoinTool {
    smooth: Option<KernelSolid>,
    exact: Option<KernelSolid>,
    dipped: Option<KernelSolid>,
}

/// A cut's tool, tried in order, mirroring `JoinTool`: `smooth` is the analytic
/// cylinder for a circular pocket/drill (a clean round hole, not a 48-gon one);
/// `exact` is the faceted prism with the drawn dimensions; `expanded` is a
/// faceted fallback whose walls poke ~`CUT_WALL_GROW`mm past the body's faces to
/// dodge the coplanar-face case. `smooth` is `None` for non-circular profiles.
struct CutTool {
    smooth: Option<KernelSolid>,
    exact: Option<KernelSolid>,
    expanded: Option<KernelSolid>,
    // The same three tools swept the OPPOSITE direction. A cut is meant to remove
    // material; when the drawn direction sweeps into empty air (e.g. a positive
    // "pocket depth" on a top-face sketch, which `directional_cut` sends *up* away
    // from the body) it bites nothing and the op silently does nothing — the
    // reported "cut works once then never again". `apply_cut` falls back to these
    // when the drawn direction removes nothing from a body it should have cut.
    smooth_rev: Option<KernelSolid>,
    exact_rev: Option<KernelSolid>,
    expanded_rev: Option<KernelSolid>,
}

/// Grow (`outward`) or shrink a closed 2D loop about its centroid so its
/// outermost vertex moves by `CUT_WALL_GROW`mm. Used to nudge a cut tool's side
/// walls just clear of a body face they'd otherwise be coplanar with — the
/// in-plane counterpart to `directional_cut`'s end-cap overshoot. Holes are
/// shrunk (`outward = false`) so their walls move the same way relative to the
/// removed volume. Every vertex moves at most `CUT_WALL_GROW`mm (displacement
/// is `dist_to_centroid · CUT_WALL_GROW / max_dist`), so the loop stays simple
/// for the convex and mildly-concave profiles sketches produce.
fn grow_loop(points: &[(f32, f32)], outward: bool) -> Vec<(f32, f32)> {
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

/// Creation order for a node id: the trailing numeric suffix (`extrude_12` → 12)
/// from the shared monotonic counter. Ids without a suffix (e.g. `origin`) sort
/// first. This is stable across deletions, unlike petgraph's `NodeIndex`.
fn creation_key(id: &str) -> u64 {
    id.rsplit('_')
        .next()
        .and_then(|s| s.parse::<u64>().ok())
        .unwrap_or(0)
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
fn overshoot_cs(cs: &CoordinateSystem, depth: f32) -> CoordinateSystem {
    let back = cs.n.mul(-depth.signum() * CUT_OVERSHOOT);
    CoordinateSystem::new(cs.origin.add(back), cs.u, cs.v)
}

/// Depth extended by `ends` overshoot lengths along the sweep direction. Paired
/// with `overshoot_cs` (which moves the start back by one overshoot): `ends = 1`
/// keeps the far cap where it was (near-only dip, for join).
fn overshoot_depth(depth: f32, ends: f32) -> f32 {
    depth + depth.signum() * ends * CUT_OVERSHOOT
}

/// A cut tool that sweeps in the **drawn direction** (the sign of `depth`):
/// a negative depth cuts *into* the body the sketch sits on, a positive depth
/// sweeps *outward* from the sketch face. The tool starts one `CUT_OVERSHOOT`
/// behind the sketch plane and runs `|depth| + 2·CUT_OVERSHOOT` along the sweep
/// direction, so both end caps clear the body's faces — the near cap clears the
/// face the sketch sits on, and the far cap clears the back face when the cut
/// punches clean through. Returns `(start_plane, signed_sweep_depth)`.
fn directional_cut(cs: &CoordinateSystem, depth: f32) -> (CoordinateSystem, f32) {
    let sign = if depth < 0.0 { -1.0 } else { 1.0 };
    let start = cs.origin.add(cs.n.mul(-sign * CUT_OVERSHOOT));
    (
        CoordinateSystem::new(start, cs.u, cs.v),
        depth + sign * 2.0 * CUT_OVERSHOOT,
    )
}

/// Apply a Join extrude: union each tool into the first existing body it
/// overlaps. For each candidate body it tries the exact tool first (perfect
/// geometry), then the dipped tool (breaks coplanar faces). A tool that joins
/// nothing — exact or dipped — becomes a standalone new body under the
/// extrude's id, matching Fusion's "join with nothing creates a body"; that
/// outcome is surfaced as a warning since the user asked to *join*, not to
/// create a separate lump.
fn apply_join(
    live: &mut Vec<LiveBody>,
    extrude_id: &str,
    tools: Vec<JoinTool>,
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
                for part in body.parts.iter_mut() {
                    let overlaps = crate::mock_kernel::solid_aabb(part).map_or(true, |pbb| {
                        crate::mock_kernel::aabbs_overlap(&pbb, &tbb, 0.05)
                    });
                    if !overlaps {
                        continue;
                    }
                    // Smooth analytic cylinder first (round boss), then the faceted
                    // prism variants as robustness fallbacks.
                    let unioned = tool
                        .smooth
                        .as_ref()
                        .and_then(|t| crate::mock_kernel::union(part, t))
                        .or_else(|| {
                            tool.exact
                                .as_ref()
                                .and_then(|t| crate::mock_kernel::union(part, t))
                        })
                        .or_else(|| {
                            tool.dipped
                                .as_ref()
                                .and_then(|t| crate::mock_kernel::union(part, t))
                        });
                    if let Some(u) = unioned {
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
                            *part = u;
                            body.pristine = None;
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
        });
    }
}

/// Subtract one direction's tool variants from `part`, trying smooth → exact →
/// expanded (and their axis-aligned fallbacks). `None` if the tool's AABB misses
/// the part or the solver couldn't subtract it. All use the body-splitting
/// difference, so a cut that severs the part yields separate parts.
fn cut_part_one_dir(
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
fn apply_cut(
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
                body.pristine = None;
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

/// Upper bound on facets approximating a 3D fillet's rounded surface. The cutter
/// tessellates adaptively (~3.6°/segment) up to this cap, so a right-angle edge
/// rounds with ~24 facets — smooth enough that, with the facet-boundary lines
/// suppressed (see `mesh_feature_edges`), the fillet reads as one curved face —
/// while keeping truck's boolean cutter face count bounded.
const EDGE_FILLET_SEGS: usize = 24;

/// How far the fallback edge cutter inflates outward (mm). Must clear `BOOL_TOL`
/// (0.05mm) by a healthy margin so the cutter's tangent edges read as cleanly
/// *outside* the body faces rather than tangent — the configuration truck's
/// boolean solver rejects. Costs up to this much chamfer/fillet size in the
/// fallback path, the price of a boolean that resolves at all.
const EDGE_MOD_GROW: f32 = 0.2;

/// Apply a 3D fillet or chamfer to the target body.
///
/// **Fillet** uses OpenRCAD's native rolling-ball blend
/// ([`crate::mock_kernel::fillet_edge`]): the captured edge is located in each
/// part's B-Rep by its endpoints and replaced by a true cylindrical fillet face
/// — no booleans, no draft/commit split. An oversized radius (≥ half the part's
/// smallest dimension, the same bar OpenRCAD's all-edge `fillet` uses) is
/// rejected so the body is left intact rather than self-intersecting.
///
/// **Chamfer** still subtracts a faceted edge cutter
/// ([`crate::mock_kernel::edge_corner_cutter`]) via a guarded boolean, since the
/// kernel has no native single-edge chamfer yet. As with [`apply_cut`], a
/// boolean the kernel can't resolve leaves the part intact and warns rather than
/// dropping a valid body.
///
/// `draft` is retained for API compatibility but no longer changes the result:
/// the native fillet is exact in a single pass, so the live preview and the
/// committed model are identical.
fn apply_edge_mod(
    mod_id: &str,
    target: &str,
    edge: &EdgeRef,
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

    match kind {
        crate::sketch::CornerKind::Fillet => apply_fillet(mod_id, edge, dist, body, warnings),
        crate::sketch::CornerKind::Chamfer => apply_chamfer(mod_id, edge, dist, body, warnings),
    }
}

/// Native rolling-ball fillet of the captured edge on every part of `body`.
fn apply_fillet(
    mod_id: &str,
    edge: &EdgeRef,
    dist: f32,
    body: &mut LiveBody,
    warnings: &mut Vec<String>,
) {
    let mut applied = false;
    let mut last_err: Option<String> = None;
    let mut next: Vec<KernelSolid> = Vec::with_capacity(body.parts.len());
    for part in body.parts.drain(..) {
        // No pre-size gate: the kernel's rolling-ball blend rejects a radius too
        // large for the local geometry (a non-watertight result → `Err`), which
        // is the correct, geometry-aware bound. The old global-AABB heuristic was
        // both wrong (it measured the part's *thinnest* axis, not the filleted
        // edge's adjacent-face extents, so it blocked radii the kernel handles)
        // and asymmetric — chamfer never had it, which is why a radius would
        // chamfer but refuse to fillet.
        match crate::mock_kernel::fillet_edge(&part, edge.p0, edge.p1, dist) {
            Ok(f) => {
                applied = true;
                next.push(f);
            }
            Err(reason) => {
                last_err = Some(reason);
                next.push(part);
            }
        }
    }
    body.parts = next;
    if applied {
        body.pristine = None;
    } else {
        // Surface the kernel's actual reason (radius too large, edge not found on
        // an adjacent face, non-blendable wedge, …) instead of a generic guess.
        let reason = last_err.unwrap_or_else(|| {
            "the edge is no longer on the body".to_string()
        });
        warnings.push(format!(
            "Fillet '{mod_id}': the edge couldn't be rounded ({reason}), so the \
             body was left unchanged."
        ));
    }
}

/// Faceted cutter-subtraction chamfer (no native single-edge chamfer yet).
fn apply_chamfer(
    mod_id: &str,
    edge: &EdgeRef,
    dist: f32,
    body: &mut LiveBody,
    warnings: &mut Vec<String>,
) {
    // exact = corner on the body edge (best geometry when the boolean takes it);
    // robust = legs lifted just outside the body so they aren't coplanar with its
    // faces. Both overshoot the edge ends.
    let mut cutters: Vec<KernelSolid> = Vec::with_capacity(2);
    cutters.extend(crate::mock_kernel::edge_corner_cutter(
        edge.p0, edge.p1, edge.n1, edge.n2, dist, false, EDGE_FILLET_SEGS, 0.0, CUT_OVERSHOOT,
    ));
    cutters.extend(crate::mock_kernel::edge_corner_cutter(
        edge.p0, edge.p1, edge.n1, edge.n2, dist, false, EDGE_FILLET_SEGS, EDGE_MOD_GROW, CUT_OVERSHOOT,
    ));
    if cutters.is_empty() {
        warnings.push(format!(
            "Chamfer '{mod_id}': the edge couldn't be turned into a cutter \
             (degenerate edge or size), so the body was left unchanged."
        ));
        return;
    }

    let mut applied = false;
    let mut next: Vec<KernelSolid> = Vec::with_capacity(body.parts.len());
    for part in body.parts.drain(..) {
        let cut = cutters.iter().find_map(|tool| {
            crate::mock_kernel::difference(&part, tool).filter(|d| edge_mod_keeps_body(&part, d))
        });
        match cut {
            Some(d) => {
                applied = true;
                next.push(d);
            }
            None => next.push(part),
        }
    }
    body.parts = next;
    if applied {
        body.pristine = None;
    } else {
        warnings.push(format!(
            "Chamfer '{mod_id}': the solver couldn't apply it to the body (the \
             edge may not be a clean convex corner), so the body was left unchanged."
        ));
    }
}

/// Guard against a degenerate edge-mod boolean. A fillet/chamfer is a *pure
/// subtraction*, so a correct result must (a) still retain the bulk of the part
/// — it only shaves a corner — and (b) never extend **beyond** the original part
/// (`a − b ⊆ a`). A tangent/inverted boolean that self-intersects or adds
/// material instead flares the result's bounds outside the part; rejecting that
/// forces the caller to fall through to the robust cutter (or keep the body
/// intact). Missing bounds → accept (vertexless can't be judged).
fn edge_mod_keeps_body(part: &KernelSolid, result: &KernelSolid) -> bool {
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
            let within = (0..3).all(|k| {
                r.0[k] >= p.0[k] - SLACK && r.1[k] <= p.1[k] + SLACK
            });
            keeps_bulk && within
        }
        (None, None) => true,
        _ => false,
    }
}

#[cfg(test)]
mod extrude_mode_tests {
    use super::*;
    use crate::geometry::CoordinateSystem;
    use crate::sketch::SketchCurves;

    fn rect_sketch(min: (f32, f32), max: (f32, f32)) -> SketchCurves {
        let mut c = SketchCurves::new();
        c.add_rectangle(min, max);
        c
    }

    fn add_extrude(
        g: &mut ParametricGraph,
        id: &str,
        sketch_id: &str,
        depth: f32,
        mode: ExtrudeMode,
    ) {
        g.add_feature(FeatureNode {
            id: id.to_string(),
            name: id.to_string(),
            feature: FeatureType::Extrude {
                depth,
                region_indices: vec![],
                mode,
                depth_expr: None,
            },
        });
        g.add_dependency(sketch_id, id);
    }

    fn add_sketch(g: &mut ParametricGraph, id: &str, curves: SketchCurves) {
        g.add_feature(FeatureNode {
            id: id.to_string(),
            name: id.to_string(),
            feature: FeatureType::Sketch {
                cs: CoordinateSystem::XY,
                curves,
                shapes: vec![],
                corner_mods: vec![],
                on_face: false,
            },
        });
    }

    #[test]
    fn sketch_dimension_follows_a_variable() {
        use crate::sketch::{Dimension, SketchShape};
        use crate::units::Unit;
        // A variable "w" and a sketch whose only shape is a square with width &
        // height bound to "w". Extruding it and changing "w" must change the
        // detected region's area (and therefore the solid).
        let mut g = ParametricGraph::new();
        g.add_feature(FeatureNode {
            id: "vars_1".to_string(),
            name: "Vars".to_string(),
            feature: FeatureType::VariableSet {
                variables: vec![Variable {
                    name: "w".to_string(),
                    value: 10.0,
                    unit: Unit::Millimeter,
                }],
            },
        });
        let wdim = || Dimension {
            value: 10.0,
            expr: Some("w".to_string()),
        };
        g.add_feature(FeatureNode {
            id: "sketch_2".to_string(),
            name: "Sketch".to_string(),
            feature: FeatureType::Sketch {
                cs: CoordinateSystem::XY,
                curves: SketchCurves::new(),
                shapes: vec![SketchShape::Rectangle {
                    origin: (0.0, 0.0),
                    sx: 1.0,
                    sy: 1.0,
                    w: wdim(),
                    h: wdim(),
                    from_center: false,
                }],
                corner_mods: vec![],
                on_face: false,
            },
        });
        g.add_feature(FeatureNode {
            id: "extrude_3".to_string(),
            name: "Extrude".to_string(),
            feature: FeatureType::Extrude {
                depth: 5.0,
                region_indices: vec![],
                mode: ExtrudeMode::NewBody,
                depth_expr: None,
            },
        });
        g.add_dependency("sketch_2", "extrude_3");

        let footprint = |g: &ParametricGraph| -> f32 {
            // Span of the body in X = the square's width.
            let bodies = g.evaluate_bodies(&std::collections::HashSet::new()).unwrap();
            let xs: Vec<f32> = bodies[0].1.vertices.chunks(6).map(|v| v[0]).collect();
            let (mn, mx) = xs.iter().fold((f32::MAX, f32::MIN), |(a, b), &x| (a.min(x), b.max(x)));
            mx - mn
        };

        assert!((footprint(&g) - 10.0).abs() < 0.05, "width should be w=10");

        for idx in g.graph.node_indices() {
            if let FeatureType::VariableSet { variables } = &mut g.graph[idx].feature {
                variables[0].value = 30.0;
            }
        }
        assert!(
            (footprint(&g) - 30.0).abs() < 0.05,
            "changing the variable must resize the sketch (and the solid)"
        );
    }

    #[test]
    fn extrude_depth_follows_a_variable() {
        use crate::units::Unit;
        // A variable "h", and an extrude whose depth is the expression "h".
        // Changing the variable must change the resulting solid's height.
        let mut g = ParametricGraph::new();
        g.add_feature(FeatureNode {
            id: "vars_1".to_string(),
            name: "Vars".to_string(),
            feature: FeatureType::VariableSet {
                variables: vec![Variable {
                    name: "h".to_string(),
                    value: 5.0,
                    unit: Unit::Millimeter,
                }],
            },
        });
        add_sketch(&mut g, "sketch_2", rect_sketch((0.0, 0.0), (10.0, 10.0)));
        g.add_feature(FeatureNode {
            id: "extrude_3".to_string(),
            name: "Extrude".to_string(),
            feature: FeatureType::Extrude {
                depth: 5.0,
                region_indices: vec![],
                mode: ExtrudeMode::NewBody,
                depth_expr: Some("h".to_string()),
            },
        });
        g.add_dependency("sketch_2", "extrude_3");

        let top_z = |g: &ParametricGraph| -> f32 {
            let bodies = g.evaluate_bodies(&std::collections::HashSet::new()).unwrap();
            bodies[0]
                .1
                .vertices
                .chunks(6)
                .map(|v| v[2])
                .fold(f32::MIN, f32::max)
        };

        assert!((top_z(&g) - 5.0).abs() < 0.01, "depth should resolve to h=5");

        // Bump the variable to 20 and rebuild — the extrude must grow with it.
        for idx in g.graph.node_indices() {
            if let FeatureType::VariableSet { variables } = &mut g.graph[idx].feature {
                variables[0].value = 20.0;
            }
        }
        assert!(
            (top_z(&g) - 20.0).abs() < 0.01,
            "changing the variable must change the extrude depth"
        );
    }

    #[test]
    fn newbody_makes_one_body() {
        let mut g = ParametricGraph::new();
        add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));
        add_extrude(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);
        let bodies = g
            .evaluate_bodies(&std::collections::HashSet::new())
            .unwrap();
        assert_eq!(bodies.len(), 1, "new body should yield exactly one body");
        assert!(!bodies[0].1.indices.is_empty());
    }

    #[test]
    fn cut_punches_hole_no_extra_body() {
        let mut g = ParametricGraph::new();
        // Base 10x10x10 block.
        add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));
        add_extrude(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);
        let plain = g
            .evaluate_bodies(&std::collections::HashSet::new())
            .unwrap();
        let plain_tris = plain[0].1.indices.len() / 3;

        // Cut a 6x6 square clean through it.
        add_sketch(&mut g, "sketch_3", rect_sketch((2.0, 2.0), (8.0, 8.0)));
        add_extrude(&mut g, "extrude_4", "sketch_3", 10.0, ExtrudeMode::Cut);
        let cut = g
            .evaluate_bodies(&std::collections::HashSet::new())
            .unwrap();

        assert_eq!(
            cut.len(),
            1,
            "cut must not add a separate body (got {})",
            cut.len()
        );
        let cut_tris = cut[0].1.indices.len() / 3;
        assert!(
            cut_tris > plain_tris,
            "cut body should have MORE triangles than the plain block (hole walls): plain={plain_tris} cut={cut_tris}"
        );
    }

    #[test]
    fn join_negative_depth_into_body_keeps_it() {
        // A box, then a join whose extrude runs straight back into it (negative
        // depth on the same plane). The tool is swallowed by the box, so the
        // union must keep the box — not delete it (the inside-out-solid bug).
        let mut g = ParametricGraph::new();
        add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));
        add_extrude(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);
        add_sketch(&mut g, "sketch_3", rect_sketch((2.0, 2.0), (8.0, 8.0)));
        add_extrude(&mut g, "extrude_4", "sketch_3", -5.0, ExtrudeMode::Join);
        let bodies = g
            .evaluate_bodies(&std::collections::HashSet::new())
            .unwrap();
        assert_eq!(bodies.len(), 1, "join into a body must stay one body");
        let max_z = bodies[0]
            .1
            .vertices
            .chunks(6)
            .map(|v| v[2])
            .fold(f32::MIN, f32::max);
        assert!(
            max_z >= 9.9,
            "join must keep the original box (top near z=10), got {max_z}"
        );
    }

    #[test]
    fn join_with_no_overlap_warns_and_makes_separate_body() {
        // A box, then a join far away that overlaps nothing. It still produces a
        // body (Fusion semantics) but the user asked to *join*, so evaluation
        // must surface a warning explaining the stray body.
        let mut g = ParametricGraph::new();
        add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));
        add_extrude(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);
        add_sketch(&mut g, "sketch_3", rect_sketch((50.0, 50.0), (60.0, 60.0)));
        add_extrude(&mut g, "extrude_4", "sketch_3", 5.0, ExtrudeMode::Join);
        let (bodies, warnings) = g
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .unwrap();
        assert_eq!(bodies.len(), 2, "non-overlapping join still yields a body");
        assert!(
            warnings.iter().any(|w| w.contains("separate body")),
            "expected a 'became a separate body' warning, got {warnings:?}"
        );
    }

    #[test]
    fn clean_model_has_no_warnings() {
        // A plain new-body extrude and a normal through-cut should evaluate with
        // zero warnings — successful coplanarity fallbacks must stay silent.
        let mut g = ParametricGraph::new();
        add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));
        add_extrude(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);
        add_sketch(&mut g, "sketch_3", rect_sketch((2.0, 2.0), (8.0, 8.0)));
        add_extrude(&mut g, "extrude_4", "sketch_3", 10.0, ExtrudeMode::Cut);
        let (_bodies, warnings) = g
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .unwrap();
        assert!(
            warnings.is_empty(),
            "a clean cut-through model must not warn, got {warnings:?}"
        );
    }

    #[test]
    fn region_cache_returns_consistent_regions() {
        // The cached path (second call) must return the same regions as the
        // first, uncached call — the cache is a transparent accelerator.
        let g = ParametricGraph::new();
        let curves = rect_sketch((0.0, 0.0), (10.0, 10.0));
        let first = g.cached_regions(&curves);
        let second = g.cached_regions(&curves);
        assert_eq!(first, second, "cache must not change the result");
        assert_eq!(first.len(), 1, "a rectangle is exactly one region");
    }

    /// Overwrite an edge-mod node's distance in place — mimics a fillet/chamfer
    /// radius drag, which changes only that trailing node.
    fn set_edge_mod_dist(g: &mut ParametricGraph, id: &str, new_dist: f32) {
        let idx = g.node_map[id];
        if let FeatureType::EdgeMod { dist, .. } = &mut g.graph[idx].feature {
            *dist = new_dist;
        }
    }

    /// A flat (id, vertex bytes, index) summary for exact mesh comparison.
    fn mesh_digest(bodies: &[(String, MockMesh)]) -> Vec<(String, Vec<u32>, Vec<u32>)> {
        bodies
            .iter()
            .map(|(id, m)| {
                (
                    id.clone(),
                    m.vertices.iter().map(|f| f.to_bits()).collect(),
                    m.indices.clone(),
                )
            })
            .collect()
    }

    #[test]
    fn eval_cache_matches_cold_eval_after_radius_drag() {
        // The prefix cache is a pure accelerator: re-evaluating after changing only
        // a trailing edge-mod (a radius drag) must yield byte-identical geometry to
        // a freshly built graph at the same final state — never a stale prefix.
        let empty = std::collections::HashSet::new();

        let mut warm = box_with_edge_mod(2.0, crate::sketch::CornerKind::Fillet);
        // Warm the cache at radius 2.0, then "drag" to 3.0 and re-evaluate (the box
        // prefix is reused from the checkpoint; only the fillet re-runs).
        let _ = warm.evaluate_bodies_with_warnings(&empty).unwrap();
        set_edge_mod_dist(&mut warm, "edgemod_2", 3.0);
        let (warm_bodies, warm_warn) = warm.evaluate_bodies_with_warnings(&empty).unwrap();

        // Cold: an identical graph at radius 3.0 with an empty cache.
        let cold = box_with_edge_mod(3.0, crate::sketch::CornerKind::Fillet);
        let (cold_bodies, cold_warn) = cold.evaluate_bodies_with_warnings(&empty).unwrap();

        assert_eq!(
            mesh_digest(&warm_bodies),
            mesh_digest(&cold_bodies),
            "cached re-eval after a radius drag must match a cold rebuild exactly"
        );
        assert_eq!(warm_warn, cold_warn, "warnings must match the cold rebuild too");
    }

    #[test]
    fn eval_cache_is_invalidated_when_an_upstream_node_changes() {
        // Changing an *upstream* dimension (the box size) must not serve a stale
        // cached body — the prefix key changes, forcing a rebuild that matches cold.
        let empty = std::collections::HashSet::new();
        let mut g = box_with_edge_mod(2.0, crate::sketch::CornerKind::Fillet);
        let _ = g.evaluate_bodies_with_warnings(&empty).unwrap();

        // Grow the box: this is the very first node, so nothing downstream can be
        // reused.
        let box_idx = g.node_map["box_1"];
        if let FeatureType::Box { w, h, d } = &mut g.graph[box_idx].feature {
            *w = 20.0;
            *h = 20.0;
            *d = 20.0;
        }
        let (warm_bodies, _) = g.evaluate_bodies_with_warnings(&empty).unwrap();

        let mut cold = box_with_edge_mod(2.0, crate::sketch::CornerKind::Fillet);
        let cold_idx = cold.node_map["box_1"];
        if let FeatureType::Box { w, h, d } = &mut cold.graph[cold_idx].feature {
            *w = 20.0;
            *h = 20.0;
            *d = 20.0;
        }
        let (cold_bodies, _) = cold.evaluate_bodies_with_warnings(&empty).unwrap();

        assert_eq!(
            mesh_digest(&warm_bodies),
            mesh_digest(&cold_bodies),
            "an upstream change must invalidate the cache, not serve stale geometry"
        );
    }

    fn box_with_edge_mod(dist: f32, kind: crate::sketch::CornerKind) -> ParametricGraph {
        let mut g = ParametricGraph::new();
        // A 10×10×10 block, one corner at the origin.
        g.add_feature(FeatureNode {
            id: "box_1".to_string(),
            name: "Box".to_string(),
            feature: FeatureType::Box {
                w: 10.0,
                h: 10.0,
                d: 10.0,
            },
        });
        // Bevel/round the bottom-front edge (along +X at y=0, z=0): the two
        // adjacent faces are -Z (front) and -Y (bottom).
        g.add_feature(FeatureNode {
            id: "edgemod_2".to_string(),
            name: "Edge Mod".to_string(),
            feature: FeatureType::EdgeMod {
                target: "box_1".to_string(),
                edge: EdgeRef {
                    p0: [0.0, 0.0, 0.0],
                    p1: [10.0, 0.0, 0.0],
                    n1: [0.0, 0.0, -1.0],
                    n2: [0.0, -1.0, 0.0],
                },
                dist,
                dist_expr: None,
                kind,
            },
        });
        g.add_dependency("box_1", "edgemod_2");
        g
    }

    fn add_sketch_cs(g: &mut ParametricGraph, id: &str, cs: CoordinateSystem, curves: SketchCurves) {
        g.add_feature(FeatureNode {
            id: id.to_string(),
            name: id.to_string(),
            feature: FeatureType::Sketch {
                cs,
                curves,
                shapes: vec![],
                corner_mods: vec![],
                on_face: false,
            },
        });
    }

    #[test]
    fn rect_with_circular_hole_newbody_renders() {
        // The "plate with a hole vanishes on commit" case: a rectangle with a circle
        // inside, extruded as a New Body. detect_regions yields 2 regions — the
        // annulus (rect ⊖ circle) and the inner disk. take_all, region 0, and
        // region 1 must each produce a visible (non-empty) body.
        let mut g = ParametricGraph::new();
        let mut curves = SketchCurves::new();
        curves.add_rectangle((0.0, 0.0), (40.0, 30.0));
        curves.add_circle((20.0, 15.0), 8.0);
        assert_eq!(g.cached_regions(&curves).len(), 2, "rect+circle = annulus + disk");

        add_sketch(&mut g, "sketch_1", curves);
        add_extrude(&mut g, "extrude_2", "sketch_1", 11.62, ExtrudeMode::NewBody);
        let (bodies, _) = g
            .evaluate_bodies(&std::collections::HashSet::new())
            .map(|b| (b, ()))
            .unwrap();
        assert!(!bodies.is_empty(), "rect+hole New Body must not vanish");
        assert!(bodies.iter().all(|(_, m)| !m.indices.is_empty()));

        // Each individual region selection must also render a body.
        for sel in [vec![0usize], vec![1]] {
            let mut g2 = ParametricGraph::new();
            let mut c2 = SketchCurves::new();
            c2.add_rectangle((0.0, 0.0), (40.0, 30.0));
            c2.add_circle((20.0, 15.0), 8.0);
            add_sketch(&mut g2, "s", c2);
            g2.add_feature(FeatureNode {
                id: "e".to_string(),
                name: "e".to_string(),
                feature: FeatureType::Extrude {
                    depth: 11.62,
                    region_indices: sel.clone(),
                    mode: ExtrudeMode::NewBody,
                    depth_expr: None,
                },
            });
            g2.add_dependency("s", "e");
            let b2 = g2.evaluate_bodies(&std::collections::HashSet::new()).unwrap();
            let tris: usize = b2.iter().map(|(_, m)| m.indices.len() / 3).sum();
            assert!(tris > 0, "region selection {sel:?} must render a body, got {tris} tris");
        }
    }

    #[test]
    fn edge_mod_on_sketched_prism_applies() {
        // The screenshots' box is a sketch→extrude prism (build_extrusion_solid),
        // not a make_box. Both chamfer AND fillet must apply to a top edge of such a
        // prism (pre-fix the fillet failed because the sewn top cap stored an inward
        // normal).
        for kind in [crate::sketch::CornerKind::Chamfer, crate::sketch::CornerKind::Fillet] {
            let mut g = ParametricGraph::new();
            add_sketch(&mut g, "s", rect_sketch((0.0, 0.0), (40.0, 30.0)));
            add_extrude(&mut g, "e", "s", 20.0, ExtrudeMode::NewBody);
            // Top-front edge of the prism: from (0,0,20) to (40,0,20); adjacent
            // faces are +Z (top) and -Y (front).
            g.add_feature(FeatureNode {
                id: "em".to_string(),
                name: "Edge Mod".to_string(),
                feature: FeatureType::EdgeMod {
                    target: "e".to_string(),
                    edge: EdgeRef {
                        p0: [0.0, 0.0, 20.0],
                        p1: [40.0, 0.0, 20.0],
                        n1: [0.0, 0.0, 1.0],
                        n2: [0.0, -1.0, 0.0],
                    },
                    dist: 2.11,
                    dist_expr: None,
                    kind,
                },
            });
            g.add_dependency("e", "em");
            let mut hidden = std::collections::HashSet::new();
            hidden.insert("s".to_string());
            let (bodies, warnings) = g.evaluate_bodies_with_warnings(&hidden).unwrap();
            assert_eq!(bodies.len(), 1, "{kind:?} on a prism must stay one body");
            assert!(warnings.is_empty(), "{kind:?} on a clean prism edge should not warn, got {warnings:?}");
            // The top-front sharp edge (y=0, z=20) must be gone — the edge-mod applied.
            let m = &bodies[0].1;
            let sharp = m.vertices.chunks(6).any(|v| v[1].abs() < 0.02 && (v[2] - 20.0).abs() < 0.02);
            assert!(!sharp, "{kind:?} should have removed the prism's top-front edge");
        }
    }

    #[test]
    fn prism_box_cut_and_join_a_cylinder_through_top() {
        use crate::geometry::Vec3;
        // Full parametric repro of the screenshots: a sketch→extrude PRISM box,
        // then a circle on its top face (a) Cut downward through it and (b) Joined
        // upward as a boss. The cut must actually bore a hole (many more tris than
        // the plain box) and the join must add the boss (reaches z≈23), each one
        // body with no warning.
        let make_base = || {
            let mut g = ParametricGraph::new();
            add_sketch(&mut g, "s_base", rect_sketch((0.0, 0.0), (40.0, 20.0)));
            add_extrude(&mut g, "e_base", "s_base", 15.0, ExtrudeMode::NewBody);
            g
        };
        let plain_tris = make_base()
            .evaluate_bodies(&std::collections::HashSet::new())
            .unwrap()[0]
            .1
            .indices
            .len()
            / 3;
        let top = CoordinateSystem::new(Vec3::new(0.0, 0.0, 15.0), Vec3::X, Vec3::Y);

        // CUT through the top.
        let mut gc = make_base();
        let mut cc = SketchCurves::new();
        cc.add_circle((20.0, 10.0), 4.0);
        add_sketch_cs(&mut gc, "s_cut", top, cc);
        add_extrude(&mut gc, "e_cut", "s_cut", -16.62, ExtrudeMode::Cut);
        gc.add_dependency("e_base", "e_cut");
        let (cb, cw) = gc.evaluate_bodies_with_warnings(&std::collections::HashSet::new()).unwrap();
        assert_eq!(cb.len(), 1, "cut stays one body");
        assert!(cw.is_empty(), "a clean drill-through should not warn, got {cw:?}");
        assert!(
            cb[0].1.indices.len() / 3 > plain_tris + 6,
            "cut must bore a hole (more tris than the plain box={plain_tris})"
        );

        // JOIN a boss on top.
        let mut gj = make_base();
        let mut jc = SketchCurves::new();
        jc.add_circle((20.0, 10.0), 4.0);
        add_sketch_cs(&mut gj, "s_join", top, jc);
        add_extrude(&mut gj, "e_join", "s_join", 8.0, ExtrudeMode::Join);
        gj.add_dependency("e_base", "e_join");
        let (jb, jw) = gj.evaluate_bodies_with_warnings(&std::collections::HashSet::new()).unwrap();
        assert_eq!(jb.len(), 1, "join stays one body");
        assert!(jw.is_empty(), "a clean boss join should not warn, got {jw:?}");
        let max_z = jb.iter().flat_map(|(_, m)| m.vertices.chunks(6)).map(|v| v[2]).fold(f32::MIN, f32::max);
        assert!(max_z >= 22.9, "join must add the boss (top z≈23), got {max_z}");
    }

    #[test]
    fn join_circle_boss_on_top_survives() {
        use crate::geometry::Vec3;
        // A 10×10×10 box, then a Ø6 circular boss sketched on its top face (z=10)
        // and joined upward 5mm — the "boss on a face" case from the screenshots,
        // where the boss base is coplanar with the box top and the boss is now a
        // smooth analytic cylinder. The boss must survive: one body, top near z=15.
        let mut g = ParametricGraph::new();
        g.add_feature(FeatureNode {
            id: "box_1".to_string(),
            name: "Box".to_string(),
            feature: FeatureType::Box { w: 10.0, h: 10.0, d: 10.0 },
        });
        let top = CoordinateSystem::new(Vec3::new(0.0, 0.0, 10.0), Vec3::X, Vec3::Y);
        let mut circ = SketchCurves::new();
        circ.add_circle((5.0, 5.0), 3.0);
        add_sketch_cs(&mut g, "sketch_2", top, circ);
        add_extrude(&mut g, "extrude_3", "sketch_2", 5.0, ExtrudeMode::Join);
        g.add_dependency("box_1", "extrude_3");

        let (bodies, warnings) = g
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .unwrap();
        assert_eq!(bodies.len(), 1, "boss-join must stay one body, got {}", bodies.len());
        assert!(warnings.is_empty(), "a coplanar boss join should not warn, got {warnings:?}");
        let max_z = bodies[0].1.vertices.chunks(6).map(|v| v[2]).fold(f32::MIN, f32::max);
        assert!(max_z >= 14.9, "joined boss must reach z≈15 (top of boss), got {max_z}");
    }

    fn box_with_boss_then_edge_mod(dist: f32, kind: crate::sketch::CornerKind) -> ParametricGraph {
        use crate::geometry::Vec3;
        // 10³ box + a Ø6 boss joined on top (z=10..15), then a fillet/chamfer on a
        // *bottom* box edge (well clear of the boss). Exercises an edge-mod on a
        // boolean-union body whose top is now a smooth analytic cylinder.
        let mut g = ParametricGraph::new();
        g.add_feature(FeatureNode {
            id: "box_1".to_string(),
            name: "Box".to_string(),
            feature: FeatureType::Box { w: 10.0, h: 10.0, d: 10.0 },
        });
        let top = CoordinateSystem::new(Vec3::new(0.0, 0.0, 10.0), Vec3::X, Vec3::Y);
        let mut circ = SketchCurves::new();
        circ.add_circle((5.0, 5.0), 3.0);
        add_sketch_cs(&mut g, "sketch_2", top, circ);
        add_extrude(&mut g, "extrude_3", "sketch_2", 5.0, ExtrudeMode::Join);
        g.add_dependency("box_1", "extrude_3");
        g.add_feature(FeatureNode {
            id: "edgemod_4".to_string(),
            name: "Edge Mod".to_string(),
            feature: FeatureType::EdgeMod {
                target: "box_1".to_string(),
                edge: EdgeRef {
                    p0: [0.0, 0.0, 0.0],
                    p1: [10.0, 0.0, 0.0],
                    n1: [0.0, 0.0, -1.0],
                    n2: [0.0, -1.0, 0.0],
                },
                dist,
                dist_expr: None,
                kind,
            },
        });
        g.add_dependency("extrude_3", "edgemod_4");
        g
    }

    #[test]
    fn edge_mod_on_boss_union_body_keeps_boss() {
        for kind in [crate::sketch::CornerKind::Chamfer, crate::sketch::CornerKind::Fillet] {
            let g = box_with_boss_then_edge_mod(2.0, kind);
            let (bodies, _warnings) = g
                .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
                .unwrap();
            assert_eq!(bodies.len(), 1, "{kind:?} on union body must stay one body");
            let m = &bodies[0].1;
            let max_z = m.vertices.chunks(6).map(|v| v[2]).fold(f32::MIN, f32::max);
            assert!(max_z >= 14.9, "{kind:?} must preserve the boss (top z≈15), got {max_z}");
            // The modified bottom-front corner must not still be sharp.
            let sharp = m.vertices.chunks(6).any(|v| v[1].abs() < 0.01 && v[2].abs() < 0.01);
            assert!(!sharp, "{kind:?} should have removed the y=0,z=0 corner");
        }
    }

    #[test]
    fn chamfer_large_radius_stays_in_bounds() {
        // A 6.48mm chamfer on a 10mm box edge (the screenshot's value). Big, but
        // valid — must bevel the corner, not produce a runaway wedge.
        let g = box_with_edge_mod(6.48, crate::sketch::CornerKind::Chamfer);
        let (bodies, _warnings) = g
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .unwrap();
        let mesh = &bodies[0].1;
        let inside = mesh.vertices.chunks(6).all(|v| {
            (-0.5..=10.5).contains(&v[0])
                && (-0.5..=10.5).contains(&v[1])
                && (-0.5..=10.5).contains(&v[2])
        });
        assert!(inside, "large chamfer flew vertices outside the 10³ block (runaway wedge)");
    }

    #[test]
    fn chamfer_bevels_a_box_edge() {
        let g = box_with_edge_mod(2.0, crate::sketch::CornerKind::Chamfer);
        let (bodies, warnings) = g
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .unwrap();
        assert_eq!(bodies.len(), 1, "chamfer must stay one body");
        assert!(
            warnings.is_empty(),
            "a clean box-edge chamfer should not warn, got {warnings:?}"
        );
        let mesh = &bodies[0].1;
        // The bevel introduces a face whose outward normal points at ~45° between
        // the two original faces (-Y and -Z): n ≈ (0, -0.707, -0.707).
        let has_bevel = mesh.vertices.chunks(6).any(|v| {
            v[3].abs() < 0.2 && (v[4] + 0.707).abs() < 0.15 && (v[5] + 0.707).abs() < 0.15
        });
        assert!(
            has_bevel,
            "chamfer should create a 45° bevel face (normal ~ (0,-0.707,-0.707))"
        );
        // The bottom (y=0) and front (z=0) faces still exist away from the edge…
        let min_y = mesh.vertices.chunks(6).map(|v| v[1]).fold(f32::MAX, f32::min);
        let min_z = mesh.vertices.chunks(6).map(|v| v[2]).fold(f32::MAX, f32::min);
        assert!(
            min_y < 0.01 && min_z < 0.01,
            "the bottom and front faces should survive the chamfer (min_y={min_y}, min_z={min_z})"
        );
        // …but the sharp corner is gone: no vertex sits on BOTH y=0 and z=0 (the
        // beveled edge has been cut back to the tangent lines).
        let sharp_corner = mesh
            .vertices
            .chunks(6)
            .any(|v| v[1].abs() < 0.01 && v[2].abs() < 0.01);
        assert!(
            !sharp_corner,
            "the original y=0,z=0 edge should be beveled away, not still sharp"
        );
    }

    #[test]
    fn fillet_round_is_a_single_brep_face() {
        // The analytic-arc cutter must turn the round into ONE cylindrical B-rep
        // face — not the ~24 flat facets of the faceted fallback. Rounding one
        // edge of a 6-faced box leaves the 6 box faces (two trimmed) plus the one
        // fillet face: a handful, nowhere near 6 + 24. This guards against a
        // change silently regressing the fillet to the faceted path.
        let g = box_with_edge_mod(2.0, crate::sketch::CornerKind::Fillet);
        let (bodies, _) = g
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .unwrap();
        let mesh = &bodies[0].1;
        let distinct: std::collections::HashSet<u32> = mesh.face_ids.iter().copied().collect();
        assert!(
            distinct.len() <= 12,
            "fillet should be one cylindrical face (got {} B-rep faces — faceted fallback?)",
            distinct.len()
        );
    }

    #[test]
    fn fillet_rounds_a_box_edge() {
        let g = box_with_edge_mod(2.0, crate::sketch::CornerKind::Fillet);
        let (bodies, warnings) = g
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .unwrap();
        assert_eq!(bodies.len(), 1, "fillet must stay one body");
        assert!(
            warnings.is_empty(),
            "a clean box-edge fillet should not warn, got {warnings:?}"
        );
        let mesh = &bodies[0].1;
        // A faceted round adds several faces along the edge: the filleted body has
        // more triangles than a plain block.
        let plain = MockMesh::make_box(10.0, 10.0, 10.0);
        assert!(
            mesh.indices.len() / 3 > plain.indices.len() / 3,
            "a filleted edge should add facets (more triangles than the plain box)"
        );
        // Geometry sanity (a self-intersecting cutter used to fly vertices out
        // here): every vertex stays inside the 10³ block, within a small margin
        // for the cutter's overshoot/offset and tessellation noise.
        let inside = mesh.vertices.chunks(6).all(|v| {
            (-0.4..=10.4).contains(&v[0])
                && (-0.4..=10.4).contains(&v[1])
                && (-0.4..=10.4).contains(&v[2])
        });
        assert!(inside, "filleted body has vertices outside the original block");
        // The sharp y=0,z=0 edge is rounded away: no vertex sits on both faces.
        let sharp = mesh
            .vertices
            .chunks(6)
            .any(|v| v[1].abs() < 0.01 && v[2].abs() < 0.01);
        assert!(!sharp, "the filleted edge should not still be sharp");
        // The round leaves intermediate facet normals between the two faces — at
        // least one vertex normal points partly along BOTH +(-y) and +(-z), i.e.
        // a curved-surface normal, not just the axis-aligned box faces.
        let has_round = mesh.vertices.chunks(6).any(|v| {
            v[4] < -0.15 && v[5] < -0.15 && v[3].abs() < 0.2
        });
        assert!(has_round, "expected curved fillet-surface normals");

        // Smooth-face look: the fillet's *lengthwise facet seams* (the lines
        // running along the edge between adjacent round facets) must be suppressed
        // by the crease filter, so the round reads as one face instead of a
        // striped one. Count wireframe edges that run parallel to the edge (+X)
        // and lie strictly inside the rounded corner (off both box faces) — there
        // should be essentially none.
        let nedges = mesh.edge_indices.len() / 2;
        let seam_count = (0..nedges)
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
                let along_x = (b[0] - a[0]).abs() > 1.0
                    && (b[1] - a[1]).abs() < 0.05
                    && (b[2] - a[2]).abs() < 0.05;
                // Strictly inside the round (not on the y=0 or z=0 box faces).
                let interior = |p: &[f32; 3]| {
                    p[1] > 0.05 && p[1] < 1.95 && p[2] > 0.05 && p[2] < 1.95
                };
                along_x && interior(&a) && interior(&b)
            })
            .count();
        assert_eq!(
            seam_count, 0,
            "fillet facet seams should be hidden (got {seam_count} lengthwise seams)"
        );
        // …but the block's genuine sharp edges (90°) must survive: the un-touched
        // top face still has its four long edges, so plenty of wireframe remains.
        assert!(
            mesh.edge_indices.len() / 2 >= 8,
            "real box edges must still be drawn after crease filtering"
        );
    }

    #[test]
    fn fillet_tangent_boundary_edges_are_drawn() {
        let g = box_with_edge_mod(2.0, crate::sketch::CornerKind::Fillet);
        let (bodies, _) = g
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .unwrap();
        let mesh = &bodies[0].1;

        let nedges = mesh.edge_indices.len() / 2;
        let tangent_len = |front: bool| -> f32 {
            (0..nedges)
                .filter_map(|e| {
                    let ia = mesh.edge_indices[e * 2] as usize * 3;
                    let ib = mesh.edge_indices[e * 2 + 1] as usize * 3;
                    let g = |i: usize| {
                        [
                            mesh.edge_vertices[i],
                            mesh.edge_vertices[i + 1],
                            mesh.edge_vertices[i + 2],
                        ]
                    };
                    let (a, b) = (g(ia), g(ib));
                    let along_x = (b[1] - a[1]).abs() < 0.05 && (b[2] - a[2]).abs() < 0.05;
                    let (on0, off) = if front { (a[2], a[1]) } else { (a[1], a[2]) };
                    let on_face = on0.abs() < 0.05 && (0.3..2.5).contains(&off);
                    (along_x && on_face).then(|| (b[0] - a[0]).abs())
                })
                .sum()
        };

        let t_true = tangent_len(true);
        let t_false = tangent_len(false);
        // The round is ~10 long; require most of the tangent line to be present.
        assert!(
            t_true > 5.0,
            "the fillet's tangent edge on the front (z=0) face must be drawn (got {})",
            t_true
        );
        assert!(
            t_false > 5.0,
            "the fillet's tangent edge on the bottom (y=0) face must be drawn (got {})",
            t_false
        );
    }

    #[test]
    fn fillet_keeps_adjacent_faces_flat() {
        // A fillet is tangent to its neighbour faces, so the round's first facet
        // sits just a few degrees off them. Plain crease-angle smoothing used to
        // drag those flat faces' edge normals toward the round, making the flat
        // faces render as a slope. The face-aware smoothing must anchor the flat
        // faces so a flat normal survives right up to the tangent line.
        let g = box_with_edge_mod(2.0, crate::sketch::CornerKind::Fillet);
        let (bodies, _) = g
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .unwrap();
        let mesh = &bodies[0].1;

        // The two faces adjacent to the rounded y=0,z=0 edge are z=0 (normal
        // (0,0,-1)) and y=0 (normal (0,-1,0)); each meets the round at a tangent
        // line ~`dist`=2 in from the old edge. At that tangent line the flat face
        // must still carry its exact axis-aligned normal — no tilt toward the
        // round. (Interior flat-face vertices were never co-located with the round
        // and so were never affected; the tangent line is where the bleed showed.)
        let on_z0_tangent = mesh.vertices.chunks(6).any(|v| {
            v[2].abs() < 0.02
                && (1.0..3.0).contains(&v[1])
                && v[3].abs() < 1.0e-3
                && v[4].abs() < 1.0e-3
                && v[5] < -0.999
        });
        assert!(
            on_z0_tangent,
            "the z=0 flat face must stay flat (normal (0,0,-1)) at the fillet tangent line"
        );
        let on_y0_tangent = mesh.vertices.chunks(6).any(|v| {
            v[1].abs() < 0.02
                && (1.0..3.0).contains(&v[2])
                && v[3].abs() < 1.0e-3
                && v[5].abs() < 1.0e-3
                && v[4] < -0.999
        });
        assert!(
            on_y0_tangent,
            "the y=0 flat face must stay flat (normal (0,-1,0)) at the fillet tangent line"
        );
    }

    #[test]
    fn smoothing_keeps_a_plain_box_crisp() {
        // The crease-angle normal smoothing runs on every solid mesh; it must be
        // a no-op for a box (all faces flat, edges at 90° > the crease angle), so
        // every vertex normal stays axis-aligned — no accidental rounding.
        let m = MockMesh::make_box(10.0, 10.0, 10.0);
        for v in m.vertices.chunks(6) {
            let n = [v[3].abs(), v[4].abs(), v[5].abs()];
            let max = n.iter().cloned().fold(0.0f32, f32::max);
            let sum: f32 = n.iter().sum();
            assert!(
                (max - 1.0).abs() < 0.02 && (sum - 1.0).abs() < 0.03,
                "box vertex normal must stay axis-aligned after smoothing, got {n:?}"
            );
        }
    }

    #[test]
    fn join_overlapping_stays_one_body() {
        let mut g = ParametricGraph::new();
        add_sketch(&mut g, "sketch_1", rect_sketch((0.0, 0.0), (10.0, 10.0)));
        add_extrude(&mut g, "extrude_2", "sketch_1", 10.0, ExtrudeMode::NewBody);
        // Overlapping block joined in (shifted so faces aren't coplanar).
        add_sketch(&mut g, "sketch_3", rect_sketch((5.0, 5.0), (15.0, 15.0)));
        add_extrude(&mut g, "extrude_4", "sketch_3", 5.0, ExtrudeMode::Join);
        let bodies = g
            .evaluate_bodies(&std::collections::HashSet::new())
            .unwrap();
        assert_eq!(
            bodies.len(),
            1,
            "join into overlapping body should stay one body (got {})",
            bodies.len()
        );
    }

    #[test]
    fn edge_mod_oversized_leaves_body_unchanged_and_warns() {
        // A fillet of 30mm on a 10mm box is oversized and should be rejected,
        // leaving the original body intact.
        let g = box_with_edge_mod(30.0, crate::sketch::CornerKind::Fillet);
        let (bodies, warnings) = g
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .unwrap();
        assert_eq!(bodies.len(), 1, "body must not disappear");
        assert!(
            !warnings.is_empty(),
            "an oversized fillet should warn"
        );
        let mesh = &bodies[0].1;
        let plain = MockMesh::make_box(10.0, 10.0, 10.0);
        assert_eq!(
            mesh.indices.len(), plain.indices.len(),
            "oversized fillet must leave the body unchanged"
        );
    }
}
