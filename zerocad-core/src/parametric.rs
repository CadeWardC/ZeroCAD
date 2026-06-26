use crate::geometry::{CoordinateSystem, Vec3};
use crate::mock_kernel::{EdgeCurveHint, KernelSolid, MeshTopologyEdgeRef, MockMesh};
use crate::sketch::{
    build_region_provenance, detect_regions, Circle, Region, RegionProvenance,
    RegionProvenanceFragment, SketchCurves,
};
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

/// Optional stable topology identity for a selected edge.
///
/// These fields are additive document metadata. Edge modifiers try this stable
/// identity first, then fall back to the captured world-space geometry for
/// legacy documents or genuinely changed topology.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct TopologyEdgeRef {
    #[serde(default)]
    pub body_id: Option<String>,
    #[serde(default)]
    pub topology_version: Option<u64>,
    #[serde(default)]
    pub edge_id: Option<String>,
    #[serde(default)]
    pub adjacent_face_ids: Vec<String>,
    #[serde(default)]
    pub curve_kind: Option<String>,
    #[serde(default)]
    pub adjacent_surface_kinds: Vec<String>,
}

/// A solid edge captured geometrically for a 3D fillet/chamfer. The endpoints
/// and the two adjacent face normals are recorded in **world space** at
/// selection time (read straight from the body's wireframe — see
/// `MockMesh::edge_vertices` / `edge_face_normals`), which is all
/// [`crate::mock_kernel::edge_corner_cutter`] needs to orient its cutter.
///
/// The topology field lets an `EdgeMod` follow equivalent upstream dimension
/// edits. If the stable identity no longer resolves, the captured world-space
/// edge is still used as a legacy geometric fallback.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct EdgeRef {
    pub p0: [f32; 3],
    pub p1: [f32; 3],
    pub n1: [f32; 3],
    pub n2: [f32; 3],
    #[serde(default)]
    pub curve: Option<EdgeCurveHint>,
    #[serde(default)]
    pub topology: Option<TopologyEdgeRef>,
}

/// Legacy serialized edge-mod span. The current fillet/chamfer tool no longer
/// exposes or evaluates separate full/partial modes; this remains only so older
/// `.zcad` documents with a `scope` field can still load.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum EdgeModScope {
    #[default]
    FullEdge,
    Partial {
        start_t: f32,
        end_t: f32,
    },
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
        /// Legacy field kept so older documents deserialize. New fillet/chamfer
        /// edits always use the selected edge as captured.
        #[serde(default)]
        scope: EdgeModScope,
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

#[derive(Debug, Clone)]
struct SketchEval {
    cs: CoordinateSystem,
    curves: SketchCurves,
    regions: Vec<Region>,
    provenance: Vec<RegionProvenance>,
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
                            sketch_source: None,
                            cut_tools: Vec::new(),
                        });
                    }
                    FeatureType::Cylinder { r, h } => {
                        if let Some(solid) = crate::mock_kernel::cylinder_solid(*r, *h) {
                            live.push(LiveBody {
                                id: node.id.clone(),
                                parts: vec![solid],
                                pristine: Some(MockMesh::make_cylinder(*r, *h, 32)),
                                sketch_source: None,
                                cut_tools: Vec::new(),
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
                        scope,
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
    fn sketch_region_cache(&self, vars: &HashMap<String, f64>) -> HashMap<NodeIndex, SketchEval> {
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
                let regions = self.cached_regions(&effective);
                let provenance = build_region_provenance(&effective, shapes, &regions);
                cache.insert(
                    idx,
                    SketchEval {
                        cs: *cs,
                        regions,
                        provenance,
                        curves: effective,
                    },
                );
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
        sketch_cache: &HashMap<NodeIndex, SketchEval>,
        live: &mut Vec<LiveBody>,
        warnings: &mut Vec<String>,
    ) {
        // Resolve the parent sketch's plane + regions.
        let parent = self
            .graph
            .neighbors_directed(idx, petgraph::Direction::Incoming)
            .find_map(|p| sketch_cache.get(&p));
        let Some(sketch) = parent else {
            return;
        };
        let cs = &sketch.cs;
        let regions = &sketch.regions;
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
        let mut newbody_cut_tools: Vec<KernelSolid> = Vec::new();
        let mut cut_tools: Vec<CutTool> = Vec::new();
        let mut join_tools: Vec<JoinTool> = Vec::new();
        let mut sketch_source = SketchExtrudeSource {
            regions: Vec::new(),
        };
        let mut newbody_mesh = MockMesh::empty();

        for (i, region) in regions.iter().enumerate() {
            if !take_all && !region_indices.contains(&i) {
                continue;
            }
            let provenance = sketch.provenance.get(i);
            match mode {
                ExtrudeMode::NewBody => {
                    let source_rect_circle_exact = provenance
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
                        });
                    let rect_circle_exact = source_rect_circle_exact.clone().or_else(|| {
                        crate::mock_kernel::rect_minus_circle_region_base_and_cutter(
                            &region.boundary,
                            &region.holes,
                            depth,
                            cs,
                        )
                    });
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
                    if let Some(s) = body_tool {
                        newbody_tools.push(s);
                    }
                    if let Some((_base, cutter)) = provenance
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
                        })
                    {
                        newbody_cut_tools.push(cutter);
                    } else if let Some((_base, cutter)) =
                        crate::mock_kernel::rect_minus_circle_region_base_and_grown_cutter(
                            &region.boundary,
                            &region.holes,
                            depth,
                            cs,
                            CUT_WALL_GROW,
                        )
                    {
                        newbody_cut_tools.push(cutter);
                    } else if let Some((_, cutter)) = rect_circle_exact.as_ref() {
                        newbody_cut_tools.push(cutter.clone());
                    }
                    sketch_source.regions.push(SketchExtrudeRegionSource {
                        boundary: region.boundary.clone(),
                        holes: region.holes.clone(),
                        depth,
                        cs: *cs,
                        rect_circle: canonical_rect_circle,
                    });
                    let mut region_mesh = crate::mock_kernel::extruded_region_display_mesh(
                        &region.boundary,
                        &region.holes,
                        depth,
                        cs,
                    );
                    stamp_sketch_extrude_edge_refs(
                        &mut region_mesh,
                        node_id,
                        i,
                        provenance,
                        cs,
                        depth,
                    );
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
                if !newbody_tools.is_empty() {
                    live.push(LiveBody {
                        id: node_id.to_string(),
                        parts: newbody_tools,
                        pristine: (!newbody_mesh.indices.is_empty()).then_some(newbody_mesh),
                        sketch_source: (!sketch_source.regions.is_empty()).then_some(sketch_source),
                        cut_tools: newbody_cut_tools,
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

fn stamp_sketch_extrude_edge_refs(
    mesh: &mut MockMesh,
    body_id: &str,
    region_index: usize,
    provenance: Option<&RegionProvenance>,
    cs: &CoordinateSystem,
    depth: f32,
) {
    let Some(provenance) = provenance else {
        return;
    };
    if provenance.fragments.is_empty() {
        return;
    }

    let mut occurrences: HashMap<String, usize> = HashMap::new();
    for edge_ref in &mut mesh.edge_refs {
        let role = sketch_extrude_edge_role(edge_ref.p0, edge_ref.p1, cs, depth);
        let fragment_id = sketch_extrude_edge_fragment_id(edge_ref, provenance, cs)
            .unwrap_or_else(|| "unknown".to_string());
        let base_id =
            format!("sketch:{body_id}:region:{region_index}:fragment:{fragment_id}:role:{role}");
        let occurrence = occurrences.entry(base_id.clone()).or_insert(0);
        let edge_id = if *occurrence == 0 {
            base_id
        } else {
            format!("{base_id}:occ:{occurrence}")
        };
        *occurrence += 1;

        let curve_kind = match edge_ref.curve {
            Some(EdgeCurveHint::Circle { .. }) => Some("circle".to_string()),
            Some(EdgeCurveHint::Line) => Some("line".to_string()),
            None => None,
        };
        edge_ref.topology = Some(MeshTopologyEdgeRef {
            body_id: Some(body_id.to_string()),
            topology_version: Some(0),
            edge_id: Some(edge_id),
            curve_kind,
            adjacent_surface_kinds: Vec::new(),
            adjacent_face_ids: Vec::new(),
        });
    }
}

fn sketch_extrude_edge_role(
    p0: [f32; 3],
    p1: [f32; 3],
    cs: &CoordinateSystem,
    depth: f32,
) -> &'static str {
    let offset = |p: [f32; 3]| {
        Vec3::new(p[0], p[1], p[2])
            .sub(cs.origin)
            .dot(cs.n.normalize())
    };
    let a = offset(p0);
    let b = offset(p1);
    let tol = (depth.abs() * 1.0e-3).max(0.02);
    if a.abs() <= tol && b.abs() <= tol {
        "bottom"
    } else if (a - depth).abs() <= tol && (b - depth).abs() <= tol {
        "top"
    } else {
        "side"
    }
}

fn sketch_extrude_edge_fragment_id(
    edge_ref: &crate::mock_kernel::MeshEdgeRef,
    provenance: &RegionProvenance,
    cs: &CoordinateSystem,
) -> Option<String> {
    match edge_ref.curve {
        Some(EdgeCurveHint::Circle { center, radius, .. }) => {
            let center_2d = cs.project(Vec3::new(center[0], center[1], center[2]));
            provenance
                .fragments
                .iter()
                .enumerate()
                .find_map(|(i, fragment)| match fragment {
                    RegionProvenanceFragment::CircleArc {
                        shape_id,
                        center,
                        radius: source_radius,
                    } if (center.0 - center_2d.0).hypot(center.1 - center_2d.1) <= 0.05
                        && (*source_radius - radius).abs() <= 0.05 =>
                    {
                        Some(provenance_fragment_stable_id(i, fragment, *shape_id))
                    }
                    _ => None,
                })
        }
        _ => sketch_extrude_linear_fragment_id(edge_ref, provenance, cs),
    }
}

fn sketch_extrude_linear_fragment_id(
    edge_ref: &crate::mock_kernel::MeshEdgeRef,
    provenance: &RegionProvenance,
    cs: &CoordinateSystem,
) -> Option<String> {
    let mid = [
        (edge_ref.p0[0] + edge_ref.p1[0]) * 0.5,
        (edge_ref.p0[1] + edge_ref.p1[1]) * 0.5,
        (edge_ref.p0[2] + edge_ref.p1[2]) * 0.5,
    ];
    let mid_2d = cs.project(Vec3::new(mid[0], mid[1], mid[2]));

    let mut best_rect: Option<(f32, String)> = None;
    let mut best_circle: Option<(f32, String)> = None;
    let mut raw: Option<String> = None;
    for (i, fragment) in provenance.fragments.iter().enumerate() {
        match fragment {
            RegionProvenanceFragment::RectangleEdge {
                shape_id,
                edge_index,
                rect_min,
                rect_max,
            } => {
                let dist = distance_to_rect_edge(mid_2d, *edge_index, *rect_min, *rect_max);
                if best_rect.as_ref().map_or(true, |(best, _)| dist < *best) {
                    best_rect = Some((dist, provenance_fragment_stable_id(i, fragment, *shape_id)));
                }
            }
            RegionProvenanceFragment::CircleArc {
                shape_id,
                center,
                radius,
            } => {
                let dist = ((mid_2d.0 - center.0).hypot(mid_2d.1 - center.1) - radius).abs();
                if best_circle.as_ref().map_or(true, |(best, _)| dist < *best) {
                    best_circle =
                        Some((dist, provenance_fragment_stable_id(i, fragment, *shape_id)));
                }
            }
            RegionProvenanceFragment::RawPolyline { shape_id }
            | RegionProvenanceFragment::SketchFilletArc { shape_id }
            | RegionProvenanceFragment::SketchChamferEdge { shape_id }
            | RegionProvenanceFragment::Slot { shape_id }
            | RegionProvenanceFragment::RoundedRectangle { shape_id } => {
                raw.get_or_insert_with(|| provenance_fragment_stable_id(i, fragment, *shape_id));
            }
        }
    }

    if let Some((dist, id)) = best_circle {
        if dist <= 0.08 {
            return Some(id);
        }
    }
    if let Some((dist, id)) = best_rect {
        if dist <= 0.08 {
            return Some(id);
        }
    }
    raw
}

fn distance_to_rect_edge(
    p: (f32, f32),
    edge_index: usize,
    rect_min: (f32, f32),
    rect_max: (f32, f32),
) -> f32 {
    match edge_index {
        0 => {
            let x = p.0.clamp(rect_min.0, rect_max.0);
            (p.0 - x).hypot(p.1 - rect_min.1)
        }
        1 => {
            let y = p.1.clamp(rect_min.1, rect_max.1);
            (p.0 - rect_max.0).hypot(p.1 - y)
        }
        2 => {
            let x = p.0.clamp(rect_min.0, rect_max.0);
            (p.0 - x).hypot(p.1 - rect_max.1)
        }
        _ => {
            let y = p.1.clamp(rect_min.1, rect_max.1);
            (p.0 - rect_min.0).hypot(p.1 - y)
        }
    }
}

fn provenance_fragment_stable_id(
    fallback_index: usize,
    fragment: &RegionProvenanceFragment,
    shape_id: Option<usize>,
) -> String {
    let owner = shape_id
        .map(|id| format!("shape:{id}"))
        .unwrap_or_else(|| format!("fragment:{fallback_index}"));
    match fragment {
        RegionProvenanceFragment::RectangleEdge { edge_index, .. } => {
            format!("{owner}:rectangle-edge:{edge_index}")
        }
        RegionProvenanceFragment::CircleArc { .. } => format!("{owner}:circle"),
        RegionProvenanceFragment::SketchFilletArc { .. } => format!("{owner}:sketch-fillet"),
        RegionProvenanceFragment::SketchChamferEdge { .. } => format!("{owner}:sketch-chamfer"),
        RegionProvenanceFragment::Slot { .. } => format!("{owner}:slot"),
        RegionProvenanceFragment::RoundedRectangle { .. } => {
            format!("{owner}:rounded-rectangle")
        }
        RegionProvenanceFragment::RawPolyline { .. } => format!("{owner}:raw-polyline"),
    }
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
    sketch_source: Option<SketchExtrudeSource>,
    cut_tools: Vec<KernelSolid>,
}

#[derive(Debug, Clone)]
struct SketchExtrudeSource {
    regions: Vec<SketchExtrudeRegionSource>,
}

#[derive(Debug, Clone)]
struct SketchExtrudeRegionSource {
    boundary: Vec<(f32, f32)>,
    holes: Vec<Vec<(f32, f32)>>,
    depth: f32,
    cs: CoordinateSystem,
    rect_circle: Option<RectCircleCanonicalSource>,
}

#[derive(Debug, Clone)]
struct RectCircleCanonicalSource {
    base: KernelSolid,
    cutter: KernelSolid,
    body: Option<KernelSolid>,
}

fn rect_circle_region_base_and_cutter_from_sketch(
    curves: &SketchCurves,
    region: &Region,
    depth: f32,
    cs: &CoordinateSystem,
    radius_grow: f32,
) -> Option<(KernelSolid, KernelSolid)> {
    if region.holes.len() > 0 {
        return None;
    }
    let (rect_min, rect_max) = rectangle_bounds_from_source_curves(curves)?;
    let circle = curves.circles.first().copied()?;
    if curves.circles.len() != 1 {
        return None;
    }
    if !circle_intersects_rect_boundary(rect_min, rect_max, circle.center, circle.radius) {
        return None;
    }
    if !region_is_rect_minus_circle_material(
        region,
        rect_min,
        rect_max,
        circle.center,
        circle.radius,
    ) {
        return None;
    }
    crate::mock_kernel::rect_circle_base_and_cutter_from_primitives(
        rect_min,
        rect_max,
        circle.center,
        circle.radius,
        depth,
        cs,
        radius_grow,
    )
}

fn rect_circle_region_base_and_cutter_from_provenance(
    provenance: &RegionProvenance,
    region: &Region,
    depth: f32,
    cs: &CoordinateSystem,
    radius_grow: f32,
) -> Option<(KernelSolid, KernelSolid)> {
    if !region.holes.is_empty() {
        return None;
    }
    let mut rect: Option<((f32, f32), (f32, f32))> = None;
    let mut circle: Option<((f32, f32), f32)> = None;
    for fragment in &provenance.fragments {
        match fragment {
            RegionProvenanceFragment::RectangleEdge {
                rect_min, rect_max, ..
            } => {
                let candidate = (*rect_min, *rect_max);
                if rect.is_none() {
                    rect = Some(candidate);
                } else if rect != Some(candidate) {
                    return None;
                }
            }
            RegionProvenanceFragment::CircleArc { center, radius, .. } => {
                let candidate = (*center, *radius);
                if circle.is_none() {
                    circle = Some(candidate);
                } else if circle != Some(candidate) {
                    return None;
                }
            }
            RegionProvenanceFragment::RawPolyline { .. } => return None,
            _ => {}
        }
    }
    let (rect_min, rect_max) = rect?;
    let (circle_center, circle_radius) = circle?;
    if !circle_intersects_rect_boundary(rect_min, rect_max, circle_center, circle_radius) {
        return None;
    }
    if !region_is_rect_minus_circle_material(
        region,
        rect_min,
        rect_max,
        circle_center,
        circle_radius,
    ) {
        return None;
    }
    crate::mock_kernel::rect_circle_base_and_cutter_from_primitives(
        rect_min,
        rect_max,
        circle_center,
        circle_radius,
        depth,
        cs,
        radius_grow,
    )
}

fn sketch_source_after_circle_cut(
    source: &SketchExtrudeSource,
    circle: Circle,
) -> Option<SketchExtrudeSource> {
    let mut next = source.clone();
    let mut any = false;
    for region in &mut next.regions {
        if !region.holes.is_empty() || region.rect_circle.is_some() {
            continue;
        }
        let (rect_min, rect_max) = loop_bounds_2d(&region.boundary)?;
        if !circle_intersects_rect_boundary(rect_min, rect_max, circle.center, circle.radius) {
            continue;
        }
        let mut provenance_curves = SketchCurves::new();
        provenance_curves.add_rectangle(rect_min, rect_max);
        provenance_curves.add_circle(circle.center, circle.radius);
        if let Some(material_region) = detect_regions(&provenance_curves).into_iter().find(|r| {
            region_is_rect_minus_circle_material(
                r,
                rect_min,
                rect_max,
                circle.center,
                circle.radius,
            )
        }) {
            region.boundary = material_region.boundary;
            region.holes = material_region.holes;
        }
        let Some((base, cutter)) = crate::mock_kernel::rect_circle_base_and_cutter_from_primitives(
            rect_min,
            rect_max,
            circle.center,
            circle.radius,
            region.depth,
            &region.cs,
            0.0,
        ) else {
            continue;
        };
        region.rect_circle = Some(RectCircleCanonicalSource {
            body: crate::mock_kernel::difference(&base, &cutter),
            base,
            cutter,
        });
        any = true;
    }
    any.then_some(next)
}

fn rectangle_bounds_from_source_curves(curves: &SketchCurves) -> Option<((f32, f32), (f32, f32))> {
    if curves.segments.len() != 4 {
        return None;
    }
    let mut pts: Vec<(f32, f32)> = Vec::new();
    for seg in &curves.segments {
        for p in [seg.a, seg.b] {
            if !pts.iter().any(|q| (q.0 - p.0).hypot(q.1 - p.1) <= 1.0e-4) {
                pts.push(p);
            }
        }
    }
    if pts.len() != 4 {
        return None;
    }

    let (mut min_x, mut min_y) = (f32::INFINITY, f32::INFINITY);
    let (mut max_x, mut max_y) = (f32::NEG_INFINITY, f32::NEG_INFINITY);
    for (x, y) in pts {
        min_x = min_x.min(x);
        min_y = min_y.min(y);
        max_x = max_x.max(x);
        max_y = max_y.max(y);
    }
    if max_x - min_x <= 1.0e-3 || max_y - min_y <= 1.0e-3 {
        return None;
    }

    let has_corner = |p: (f32, f32)| {
        curves
            .segments
            .iter()
            .flat_map(|s| [s.a, s.b])
            .any(|q| (q.0 - p.0).abs() <= 1.0e-4 && (q.1 - p.1).abs() <= 1.0e-4)
    };
    for corner in [
        (min_x, min_y),
        (max_x, min_y),
        (max_x, max_y),
        (min_x, max_y),
    ] {
        if !has_corner(corner) {
            return None;
        }
    }

    let mut sides = [false; 4];
    for seg in &curves.segments {
        let horizontal = (seg.a.1 - seg.b.1).abs() <= 1.0e-4;
        let vertical = (seg.a.0 - seg.b.0).abs() <= 1.0e-4;
        if horizontal {
            if (seg.a.1 - min_y).abs() <= 1.0e-4 {
                sides[0] = true;
            } else if (seg.a.1 - max_y).abs() <= 1.0e-4 {
                sides[2] = true;
            } else {
                return None;
            }
        } else if vertical {
            if (seg.a.0 - max_x).abs() <= 1.0e-4 {
                sides[1] = true;
            } else if (seg.a.0 - min_x).abs() <= 1.0e-4 {
                sides[3] = true;
            } else {
                return None;
            }
        } else {
            return None;
        }
    }
    sides
        .iter()
        .all(|s| *s)
        .then_some(((min_x, min_y), (max_x, max_y)))
}

fn circle_intersects_rect_boundary(
    rect_min: (f32, f32),
    rect_max: (f32, f32),
    center: (f32, f32),
    radius: f32,
) -> bool {
    let ((min_x, min_y), (max_x, max_y)) = ordered_rect(rect_min, rect_max);
    let mut hits = 0usize;
    let eps = 1.0e-4;
    for y in [min_y, max_y] {
        let dy = y - center.1;
        if dy.abs() <= radius + eps {
            let dx2 = radius * radius - dy * dy;
            if dx2 >= -eps {
                let dx = dx2.max(0.0).sqrt();
                for x in [center.0 - dx, center.0 + dx] {
                    if x >= min_x - eps && x <= max_x + eps {
                        hits += 1;
                    }
                }
            }
        }
    }
    for x in [min_x, max_x] {
        let dx = x - center.0;
        if dx.abs() <= radius + eps {
            let dy2 = radius * radius - dx * dx;
            if dy2 >= -eps {
                let dy = dy2.max(0.0).sqrt();
                for y in [center.1 - dy, center.1 + dy] {
                    if y >= min_y - eps && y <= max_y + eps {
                        hits += 1;
                    }
                }
            }
        }
    }
    hits >= 2
}

fn region_is_rect_minus_circle_material(
    region: &Region,
    rect_min: (f32, f32),
    rect_max: (f32, f32),
    center: (f32, f32),
    radius: f32,
) -> bool {
    if region.contains(center) {
        return false;
    }
    let ((min_x, min_y), (max_x, max_y)) = ordered_rect(rect_min, rect_max);
    let rect_area = (max_x - min_x) * (max_y - min_y);
    if region.area <= 1.0e-3 || region.area >= rect_area - 1.0e-3 {
        return false;
    }

    let mut has_material_sample = false;
    let mut has_removed_circle_sample = false;
    for ix in 1..5 {
        for iy in 1..5 {
            let x = min_x + (max_x - min_x) * (ix as f32 / 5.0);
            let y = min_y + (max_y - min_y) * (iy as f32 / 5.0);
            let inside_circle = (x - center.0).hypot(y - center.1) < radius - 0.05;
            let inside_region = region.contains((x, y));
            if inside_region && !inside_circle {
                has_material_sample = true;
            }
            if inside_region && inside_circle {
                has_removed_circle_sample = true;
            }
        }
    }
    has_material_sample && !has_removed_circle_sample
}

fn ordered_rect(a: (f32, f32), b: (f32, f32)) -> ((f32, f32), (f32, f32)) {
    ((a.0.min(b.0), a.1.min(b.1)), (a.0.max(b.0), a.1.max(b.1)))
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
    circle: Option<Circle>,
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
                            body.sketch_source = None;
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
                let next_source = body.sketch_source.as_ref().and_then(|source| {
                    tool.circle
                        .and_then(|circle| sketch_source_after_circle_cut(source, circle))
                });
                body.pristine = None;
                body.sketch_source = next_source;
                body.cut_tools.extend(cut_tool_recutter_solids(tool));
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

fn cut_tool_recutter_solids(tool: &CutTool) -> Vec<KernelSolid> {
    [tool.expanded.as_ref(), tool.expanded_rev.as_ref()]
        .into_iter()
        .flatten()
        .cloned()
        .collect()
}

/// Edge-cutter facet cap for the boolean fallback. Native
/// fillet/chamfer edits call this path only after native failure. The cutter
/// tessellates adaptively (~3.6°/segment) up to this cap, so a right-angle edge
/// rounds with ~24 facets — smooth enough that, with the facet-boundary lines
/// suppressed (see `mesh_feature_edges`), the fillet reads as one curved face —
/// while keeping truck's boolean cutter face count bounded.
#[allow(dead_code)]
const EDGE_FILLET_SEGS: usize = 24;

/// Robust fallback edge-cutter grow amount. Must clear `BOOL_TOL`
/// (0.05mm) by a healthy margin so the cutter's tangent edges read as cleanly
/// *outside* the body faces rather than tangent — the configuration truck's
/// boolean solver rejects. Costs up to this much chamfer/fillet size in the
/// fallback path, the price of a boolean that resolves at all.
const EDGE_MOD_GROW: f32 = 0.2;

/// Robust fallback cutter overshoot past each selected edge endpoint. The exact
/// fallback tries no overshoot first; this second pass clears endpoint caps and
/// curved-wall runout tangencies.
#[allow(dead_code)]
const EDGE_MOD_END_OVERSHOOT: f32 = 1.0;

/// A fillet/chamfer is subtractive, but B-Rep kernels can occasionally return a
/// topologically valid-looking result that renders new material. Permit only a
/// tiny numerical skin outside the pre-edge-mod body.
const EDGE_MOD_CONTAINMENT_TOL: f32 = EDGE_MOD_GROW + 0.05;

/// Apply a 3D fillet or chamfer to the target body.
///
/// **Fillet** uses OpenRCAD's native rolling-ball blend
/// ([`crate::mock_kernel::fillet_edge`]): the captured edge is located in each
/// part's B-Rep by its endpoints and replaced by a true cylindrical fillet face
/// — no booleans, no draft/commit split. An oversized radius (≥ half the part's
/// smallest dimension, the same bar OpenRCAD's all-edge `fillet` uses) is
/// rejected so the body is left intact rather than self-intersecting.
///
/// **Chamfer** uses OpenRCAD's native selected-edge bevel
/// ([`crate::mock_kernel::chamfer_edge`]).
///
/// Unsupported curved rims, oversized distances, and large runouts into curved
/// cut walls are rejected before entering the kernel. Native candidates are also
/// validated as subtractive edits; if a candidate refills a cut void or adds
/// visible material, the body is left unchanged with a warning.
///
/// `draft` is retained for API compatibility but no longer changes the result:
/// edge modifiers resolve in a single pass, so the live preview and
/// the committed model are identical.
fn apply_edge_mod(
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

fn resolve_edge_ref_by_topology(body: &LiveBody, edge: &EdgeRef) -> Option<EdgeRef> {
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
            .map(|candidate| edge_ref_from_mesh_candidate(body, candidate, requested))
    }) {
        return Some(resolved);
    }

    let mesh = edge_mod_reference_mesh(body);
    mesh.edge_refs
        .iter()
        .find(|candidate| topology_edge_id(candidate) == Some(requested_edge_id))
        .map(|candidate| edge_ref_from_mesh_candidate(body, candidate, requested))
}

fn topology_edge_id(edge: &crate::mock_kernel::MeshEdgeRef) -> Option<&str> {
    edge.topology
        .as_ref()
        .and_then(|topology| topology.edge_id.as_deref())
}

fn edge_ref_from_mesh_candidate(
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

#[derive(Debug, Clone)]
struct EdgeModSelection {
    original_edge: EdgeRef,
    active_edge: EdgeRef,
}

impl EdgeModSelection {
    fn new(edge: &EdgeRef) -> Self {
        Self {
            original_edge: edge.clone(),
            active_edge: edge.clone(),
        }
    }
}

struct EdgeModResult {
    parts: Vec<KernelSolid>,
    pristine: Option<MockMesh>,
}

impl EdgeModResult {
    fn single(part: KernelSolid) -> Self {
        Self {
            parts: vec![part],
            pristine: None,
        }
    }
}

fn edge_mod_preflight(_body: &LiveBody, edge: &EdgeRef, dist: f32) -> Result<(), String> {
    if !dist.is_finite() || dist <= 0.0 {
        return Err("distance must be positive".to_string());
    }

    let run = edge_ref_local_clearance(edge);
    if run <= 1.0e-4 {
        return Err("selected edge is too short".to_string());
    }
    let max_dist = run * 0.5;
    if dist > max_dist + 1.0e-4 {
        return Err(format!(
            "requested distance {dist:.2}mm is larger than this edge's local clearance \
             (about {max_dist:.2}mm)"
        ));
    }
    Ok(())
}

fn edge_ref_length(edge: &EdgeRef) -> f32 {
    let dx = edge.p1[0] - edge.p0[0];
    let dy = edge.p1[1] - edge.p0[1];
    let dz = edge.p1[2] - edge.p0[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn edge_ref_local_clearance(edge: &EdgeRef) -> f32 {
    match edge.curve {
        Some(EdgeCurveHint::Circle { radius, .. }) => radius.abs(),
        _ => edge_ref_length(edge),
    }
}

fn edge_mod_native_only(selection: &EdgeModSelection) -> bool {
    matches!(
        selection.active_edge.curve,
        Some(EdgeCurveHint::Circle { .. })
    )
}

/// Native rolling-ball fillet of the captured edge on every part of `body`.
fn apply_fillet(
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
        let circular_bite_locality = sketch_region.map(|region| (region, selection, dist));
        let mut accepted: Option<EdgeModResult> = None;
        let native_only = edge_mod_native_only(selection);
        let native_reason = match edge_mod_try_native_fillet(
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

        if accepted.is_none() && !native_only {
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

fn edge_mod_try_native_fillet(
    reference_mesh: &MockMesh,
    original_part: &KernelSolid,
    fillet_part: &KernelSolid,
    selection: &EdgeModSelection,
    dist: f32,
    label: &str,
    recut_tools: &[KernelSolid],
    circular_bite_locality: Option<(&SketchExtrudeRegionSource, &EdgeModSelection, f32)>,
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
fn apply_chamfer(
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
        let circular_bite_locality = sketch_region.map(|region| (region, selection, dist));
        let mut accepted: Option<EdgeModResult> = None;
        let mut part_failures = Vec::new();
        let native_only = edge_mod_native_only(selection);
        let native_reason = match crate::mock_kernel::chamfer_edge(&part, edge.p0, edge.p1, dist) {
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

        if accepted.is_none() && !native_only {
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

fn edge_mod_rect_circle_precut_fallback(
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

    let split_base = split_rect_base_for_edge(region, edge).unwrap_or_else(|| base.clone());
    let fillet = matches!(kind, crate::sketch::CornerKind::Fillet);
    let mut failures = Vec::new();

    // Native edge mods on the already-cut body can sometimes build the right
    // local blend but leave a sliver/cap inside the circular void. Re-cutting the
    // result with the original analytic cylinder removes any such added material
    // while preserving the requested radius/distance.
    let circular_bite_locality = Some((region, selection, dist));

    let native_on_cut = if fillet {
        crate::mock_kernel::fillet_edge(original_part, edge.p0, edge.p1, dist)
    } else {
        crate::mock_kernel::chamfer_edge(original_part, edge.p0, edge.p1, dist)
    };
    match native_on_cut {
        Ok(edge_modded) => match crate::mock_kernel::difference(&edge_modded, &circle_cutter) {
            Some(result) => {
                match edge_mod_accept_candidate_for_edge(
                    reference_mesh,
                    original_part,
                    result,
                    circular_bite_locality,
                ) {
                    Ok(result) => return Ok(EdgeModResult::single(result)),
                    Err(reason) => {
                        failures.push(format!("post-cut native result rejected: {reason}"))
                    }
                }
            }
            None => failures.push("post-cut native circle boolean failed".to_string()),
        },
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

    let native = if fillet {
        crate::mock_kernel::fillet_edge(&split_base, edge.p0, edge.p1, dist)
    } else {
        crate::mock_kernel::chamfer_edge(&split_base, edge.p0, edge.p1, dist)
    };
    match native {
        Ok(edge_cut_base) => match crate::mock_kernel::difference(&edge_cut_base, &circle_cutter) {
            Some(result) => {
                match edge_mod_accept_candidate_for_edge(
                    reference_mesh,
                    original_part,
                    result,
                    circular_bite_locality,
                ) {
                    Ok(result) => return Ok(EdgeModResult::single(result)),
                    Err(reason) => {
                        failures.push(format!("pre-cut native result rejected: {reason}"))
                    }
                }
            }
            None => failures.push("pre-cut native circle boolean failed".to_string()),
        },
        Err(reason) => failures.push(format!("pre-cut native failed: {reason}")),
    }

    let split_base_reference = MockMesh::from_solid(&split_base);
    match edge_mod_fallback_cut_against_part(
        &split_base,
        &split_base,
        edge,
        dist,
        kind,
        &split_base_reference,
        "pre-cut cutter ",
    ) {
        Ok(edge_cut_base) => match crate::mock_kernel::difference(&edge_cut_base, &circle_cutter) {
            Some(result) => {
                match edge_mod_accept_candidate_for_edge(
                    reference_mesh,
                    original_part,
                    result,
                    circular_bite_locality,
                ) {
                    Ok(result) => return Ok(EdgeModResult::single(result)),
                    Err(reason) => {
                        failures.push(format!("pre-cut cutter result rejected: {reason}"))
                    }
                }
            }
            None => failures.push("pre-cut cutter circle boolean failed".to_string()),
        },
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

fn split_rect_base_for_edge(
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

fn loop_bounds_2d(points: &[(f32, f32)]) -> Option<((f32, f32), (f32, f32))> {
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
fn edge_mod_fallback_cut(
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
fn edge_mod_fallback_cut_against_part(
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

fn sketch_source_alternate_parts(
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
fn trimmed_edge_ref(edge: &EdgeRef, trim: f32) -> Option<EdgeRef> {
    trimmed_edge_ref_asymmetric(edge, trim, trim)
}

fn trimmed_edge_ref_asymmetric(edge: &EdgeRef, start_trim: f32, end_trim: f32) -> Option<EdgeRef> {
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

fn edge_mod_reference_mesh(body: &LiveBody) -> MockMesh {
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

fn edge_mod_accept_candidate_or_recut(
    reference_mesh: &MockMesh,
    original_part: &KernelSolid,
    candidate: KernelSolid,
    recut_tools: &[KernelSolid],
    circular_bite_locality: Option<(&SketchExtrudeRegionSource, &EdgeModSelection, f32)>,
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

fn edge_mod_accept_candidate_for_edge(
    reference_mesh: &MockMesh,
    original_part: &KernelSolid,
    candidate: KernelSolid,
    circular_bite_locality: Option<(&SketchExtrudeRegionSource, &EdgeModSelection, f32)>,
) -> Result<KernelSolid, String> {
    let candidate = edge_mod_accept_candidate(reference_mesh, original_part, candidate)?;
    if let Some((region, selection, _dist)) = circular_bite_locality {
        edge_mod_circular_bite_locality(region, &candidate, selection)?;
    }
    Ok(candidate)
}

fn edge_mod_circular_bite_locality(
    region: &SketchExtrudeRegionSource,
    candidate: &KernelSolid,
    selection: &EdgeModSelection,
) -> Result<(), String> {
    if region.rect_circle.is_none() {
        return Ok(());
    }
    let candidate_mesh = MockMesh::from_solid(candidate);
    if candidate_mesh.indices.is_empty() {
        return Err("candidate tessellated to an empty mesh".to_string());
    }

    edge_mod_circular_bite_locality_mesh(region, &candidate_mesh, selection)
}

fn edge_mod_circular_bite_locality_mesh(
    region: &SketchExtrudeRegionSource,
    candidate_mesh: &MockMesh,
    selection: &EdgeModSelection,
) -> Result<(), String> {
    for (p0, p1) in circular_bite_unselected_side_segments(region, selection) {
        if !mesh_wire_path_covers(&candidate_mesh, p0, p1, 0.08) {
            return Err(format!(
                "candidate removed unselected circular-bite side span [{:.3}, {:.3}, {:.3}] -> [{:.3}, {:.3}, {:.3}]",
                p0[0], p0[1], p0[2], p1[0], p1[1], p1[2]
            ));
        }
    }

    Ok(())
}

fn circular_bite_unselected_side_segments(
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

fn dist2(a: (f32, f32), b: (f32, f32)) -> f32 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

fn edge_mod_accept_candidate(
    reference_mesh: &MockMesh,
    original_part: &KernelSolid,
    candidate: KernelSolid,
) -> Result<KernelSolid, String> {
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
            return Ok(candidate);
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
    Ok(candidate)
}

fn edge_mod_render_mesh_has_no_cracks(mesh: &MockMesh) -> Result<(), String> {
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

fn edge_mod_reject_unhealthy_native_curve_result(
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

fn edge_mod_render_mesh_nonmanifold_edges(mesh: &MockMesh) -> usize {
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
fn edge_mod_candidate_stays_inside_reference(
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

fn edge_mod_mesh_stays_inside_reference(
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

fn mesh_is_aabb_box(mesh: &MockMesh, lo: [f32; 3], hi: [f32; 3], tol: f32) -> bool {
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

fn mesh_position_aabb(mesh: &MockMesh) -> Option<([f32; 3], [f32; 3])> {
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

fn point_in_aabb(p: [f32; 3], lo: [f32; 3], hi: [f32; 3], tol: f32) -> bool {
    (0..3).all(|k| p[k] >= lo[k] - tol && p[k] <= hi[k] + tol)
}

fn point_inside_triangle_mesh(mesh: &MockMesh, p: [f32; 3], tol: f32) -> bool {
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

fn mesh_vertex_pos6(mesh: &MockMesh, vi: u32) -> [f32; 3] {
    let b = vi as usize * 6;
    [mesh.vertices[b], mesh.vertices[b + 1], mesh.vertices[b + 2]]
}

fn mesh_edge_vertex_pos3(mesh: &MockMesh, vi: u32) -> [f32; 3] {
    let b = vi as usize * 3;
    [
        mesh.edge_vertices[b],
        mesh.edge_vertices[b + 1],
        mesh.edge_vertices[b + 2],
    ]
}

fn mesh_wire_path_covers(mesh: &MockMesh, p0: [f32; 3], p1: [f32; 3], tol: f32) -> bool {
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

fn ray_triangle_intersection(
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

fn point_triangle_distance_sq(p: [f32; 3], a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> f32 {
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

fn add3(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn sub3(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn mul3(a: [f32; 3], s: f32) -> [f32; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

fn dot3(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross3(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn length_sq3(a: [f32; 3]) -> f32 {
    dot3(a, a)
}

fn normalize3(a: [f32; 3]) -> [f32; 3] {
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
            let within = (0..3).all(|k| r.0[k] >= p.0[k] - SLACK && r.1[k] <= p.1[k] + SLACK);
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

    fn edge_ref_from_mesh_edge(body_id: &str, edge: &crate::mock_kernel::MeshEdgeRef) -> EdgeRef {
        let topology = edge.topology.as_ref().map(|topology| TopologyEdgeRef {
            body_id: topology
                .body_id
                .clone()
                .or_else(|| Some(body_id.to_string())),
            topology_version: topology.topology_version,
            edge_id: topology.edge_id.clone(),
            adjacent_face_ids: topology.adjacent_face_ids.clone(),
            curve_kind: topology.curve_kind.clone(),
            adjacent_surface_kinds: topology.adjacent_surface_kinds.clone(),
        });
        EdgeRef {
            p0: edge.p0,
            p1: edge.p1,
            n1: edge.n1,
            n2: edge.n2,
            curve: edge.curve.clone(),
            topology,
        }
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
            let bodies = g
                .evaluate_bodies(&std::collections::HashSet::new())
                .unwrap();
            let xs: Vec<f32> = bodies[0].1.vertices.chunks(6).map(|v| v[0]).collect();
            let (mn, mx) = xs
                .iter()
                .fold((f32::MAX, f32::MIN), |(a, b), &x| (a.min(x), b.max(x)));
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
    fn topology_edge_ref_reattaches_after_sketch_dimension_edit() {
        use crate::sketch::{Dimension, SketchShape};
        use crate::units::Unit;

        let mut g = ParametricGraph::new();
        g.add_feature(FeatureNode {
            id: "vars_1".to_string(),
            name: "Vars".to_string(),
            feature: FeatureType::VariableSet {
                variables: vec![Variable {
                    name: "w".to_string(),
                    value: 20.0,
                    unit: Unit::Millimeter,
                }],
            },
        });
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
                    w: Dimension {
                        value: 20.0,
                        expr: Some("w".to_string()),
                    },
                    h: Dimension::literal(12.0),
                    from_center: false,
                }],
                corner_mods: vec![],
                on_face: false,
            },
        });
        add_extrude(&mut g, "extrude_3", "sketch_2", 8.0, ExtrudeMode::NewBody);

        let initial = g
            .evaluate_bodies(&std::collections::HashSet::new())
            .unwrap();
        let mesh = &initial[0].1;
        let captured = mesh
            .edge_refs
            .iter()
            .find(|edge| {
                edge.topology
                    .as_ref()
                    .and_then(|topology| topology.edge_id.as_deref())
                    .is_some_and(|id| id.contains("rectangle-edge:1:role:top"))
            })
            .expect("right-side top sketch edge should have a stable topology id");
        assert!(
            captured.p0[0] > 19.9 && captured.p1[0] > 19.9,
            "captured edge starts on the original width"
        );
        let edge = edge_ref_from_mesh_edge("extrude_3", captured);
        assert!(
            edge.topology
                .as_ref()
                .and_then(|topology| topology.edge_id.as_ref())
                .is_some(),
            "captured edge ref must carry additive topology metadata"
        );

        g.add_feature(FeatureNode {
            id: "edgemod_4".to_string(),
            name: "Fillet".to_string(),
            feature: FeatureType::EdgeMod {
                target: "extrude_3".to_string(),
                edge,
                dist: 1.0,
                dist_expr: None,
                scope: EdgeModScope::FullEdge,
                kind: crate::sketch::CornerKind::Fillet,
            },
        });
        g.add_dependency("extrude_3", "edgemod_4");
        let (_, initial_warnings) = g
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .unwrap();
        assert!(
            initial_warnings.is_empty(),
            "initial topology-backed edge mod should solve, got {initial_warnings:?}"
        );

        for idx in g.graph.node_indices() {
            if let FeatureType::VariableSet { variables } = &mut g.graph[idx].feature {
                variables[0].value = 30.0;
            }
        }
        let (resized, warnings) = g
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .unwrap();
        assert!(
            warnings.is_empty(),
            "edge mod should reattach after equivalent topology edit, got {warnings:?}"
        );
        let max_x = resized[0]
            .1
            .vertices
            .chunks(6)
            .map(|v| v[0])
            .fold(f32::MIN, f32::max);
        assert!(
            max_x > 29.5,
            "resized body should use the edited width after reattach, max_x={max_x}"
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
            let bodies = g
                .evaluate_bodies(&std::collections::HashSet::new())
                .unwrap();
            bodies[0]
                .1
                .vertices
                .chunks(6)
                .map(|v| v[2])
                .fold(f32::MIN, f32::max)
        };

        assert!(
            (top_z(&g) - 5.0).abs() < 0.01,
            "depth should resolve to h=5"
        );

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
        assert_eq!(
            warm_warn, cold_warn,
            "warnings must match the cold rebuild too"
        );
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
                    curve: None,
                    topology: None,
                },
                dist,
                dist_expr: None,
                scope: EdgeModScope::FullEdge,
                kind,
            },
        });
        g.add_dependency("box_1", "edgemod_2");
        g
    }

    fn add_sketch_cs(
        g: &mut ParametricGraph,
        id: &str,
        cs: CoordinateSystem,
        curves: SketchCurves,
    ) {
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

    fn circular_bite_curves() -> SketchCurves {
        let mut c = SketchCurves::new();
        c.add_rectangle((0.0, 5.0), (40.0, 35.0));
        c.add_circle((20.0, 8.0), 14.0);
        c
    }

    fn circular_bite_region_index(curves: &SketchCurves) -> usize {
        let regions = detect_regions(curves);
        regions
            .iter()
            .position(|r| r.contains((5.0, 30.0)))
            .expect("rectangular material region above the circular bite")
    }

    fn circular_bite_graph_with_depth(
        edge_mod: Option<crate::sketch::CornerKind>,
        depth: f32,
    ) -> ParametricGraph {
        let curves = circular_bite_curves();
        let region = circular_bite_region_index(&curves);
        let mut g = ParametricGraph::new();
        add_sketch(&mut g, "s", curves);
        g.add_feature(FeatureNode {
            id: "e".to_string(),
            name: "e".to_string(),
            feature: FeatureType::Extrude {
                depth,
                region_indices: vec![region],
                mode: ExtrudeMode::NewBody,
                depth_expr: None,
            },
        });
        g.add_dependency("s", "e");

        if let Some(kind) = edge_mod {
            g.add_feature(FeatureNode {
                id: "em".to_string(),
                name: "Edge Mod".to_string(),
                feature: FeatureType::EdgeMod {
                    target: "e".to_string(),
                    edge: EdgeRef {
                        p0: [0.0, 35.0, depth],
                        p1: [40.0, 35.0, depth],
                        n1: [0.0, 0.0, 1.0],
                        n2: [0.0, 1.0, 0.0],
                        curve: None,
                        topology: None,
                    },
                    dist: 1.5,
                    dist_expr: None,
                    scope: EdgeModScope::FullEdge,
                    kind,
                },
            });
            g.add_dependency("e", "em");
        }

        g
    }

    fn circular_bite_graph(edge_mod: Option<crate::sketch::CornerKind>) -> ParametricGraph {
        circular_bite_graph_with_depth(edge_mod, 10.0)
    }

    fn circular_bite_cutoff_edge_at_depth(depth: f32) -> EdgeRef {
        let x = 20.0 - (14.0_f32 * 14.0 - 3.0_f32 * 3.0).sqrt();
        EdgeRef {
            p0: [0.0, 5.0, depth],
            p1: [x, 5.0, depth],
            n1: [0.0, 0.0, 1.0],
            n2: [0.0, -1.0, 0.0],
            curve: None,
            topology: None,
        }
    }

    fn circular_bite_cutoff_edge() -> EdgeRef {
        circular_bite_cutoff_edge_at_depth(10.0)
    }

    fn circular_bite_opposite_cutoff_edge_at_depth(depth: f32) -> EdgeRef {
        let x = 20.0 + (14.0_f32 * 14.0 - 3.0_f32 * 3.0).sqrt();
        EdgeRef {
            p0: [x, 5.0, depth],
            p1: [40.0, 5.0, depth],
            n1: [0.0, 0.0, 1.0],
            n2: [0.0, -1.0, 0.0],
            curve: None,
            topology: None,
        }
    }

    fn gui_captured_circular_bite_cutoff_edge(depth: f32) -> EdgeRef {
        let g = circular_bite_graph_with_depth(None, depth);
        let bodies = g
            .evaluate_bodies(&std::collections::HashSet::new())
            .unwrap();
        let mesh = &bodies[0].1;
        let expected = circular_bite_cutoff_edge_at_depth(depth);
        let same_edge = |edge: &crate::mock_kernel::MeshEdgeRef| {
            let d = |a: [f32; 3], b: [f32; 3]| {
                ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
            };
            matches!(
                edge.curve,
                None | Some(crate::mock_kernel::EdgeCurveHint::Line)
            ) && ((d(edge.p0, expected.p0) <= 0.08 && d(edge.p1, expected.p1) <= 0.08)
                || (d(edge.p0, expected.p1) <= 0.08 && d(edge.p1, expected.p0) <= 0.08))
        };
        let edge = mesh
            .edge_refs
            .iter()
            .find(|edge| same_edge(edge))
            .unwrap_or_else(|| {
                panic!(
                    "display mesh should expose exact cutoff-edge metadata; refs={:?}",
                    mesh.edge_refs
                )
            });
        EdgeRef {
            p0: edge.p0,
            p1: edge.p1,
            n1: edge.n1,
            n2: edge.n2,
            curve: edge.curve.clone(),
            topology: None,
        }
    }

    fn circular_bite_cutoff_edge_graph_with_dist(
        kind: crate::sketch::CornerKind,
        dist: f32,
    ) -> ParametricGraph {
        let edge = circular_bite_cutoff_edge();
        let mut g = circular_bite_graph(None);
        g.add_feature(FeatureNode {
            id: "em".to_string(),
            name: "Edge Mod".to_string(),
            feature: FeatureType::EdgeMod {
                target: "e".to_string(),
                edge,
                dist,
                dist_expr: None,
                scope: EdgeModScope::FullEdge,
                kind,
            },
        });
        g.add_dependency("e", "em");
        g
    }

    fn circular_bite_cutoff_edge_graph(kind: crate::sketch::CornerKind) -> ParametricGraph {
        circular_bite_cutoff_edge_graph_with_dist(kind, 1.0)
    }

    fn box_cut_circular_bite_graph_with_dist(
        kind: crate::sketch::CornerKind,
        dist: f32,
    ) -> ParametricGraph {
        box_cut_circular_bite_graph_at_depth_with_dist(kind, dist, 10.0)
    }

    fn box_cut_circular_bite_graph_at_depth_with_dist(
        kind: crate::sketch::CornerKind,
        dist: f32,
        depth: f32,
    ) -> ParametricGraph {
        let edge = circular_bite_cutoff_edge_at_depth(depth);
        let mut g = ParametricGraph::new();
        add_sketch(&mut g, "s_base", rect_sketch((0.0, 5.0), (40.0, 35.0)));
        add_extrude(&mut g, "e_base", "s_base", depth, ExtrudeMode::NewBody);

        let top = CoordinateSystem::new(Vec3::new(0.0, 0.0, depth), Vec3::X, Vec3::Y);
        let mut circle = SketchCurves::new();
        circle.add_circle((20.0, 8.0), 14.0);
        add_sketch_cs(&mut g, "s_cut", top, circle);
        add_extrude(
            &mut g,
            "e_cut",
            "s_cut",
            -(depth.abs() + 1.5),
            ExtrudeMode::Cut,
        );
        g.add_dependency("e_base", "e_cut");

        g.add_feature(FeatureNode {
            id: "em".to_string(),
            name: "Edge Mod".to_string(),
            feature: FeatureType::EdgeMod {
                target: "e_base".to_string(),
                edge,
                dist,
                dist_expr: None,
                scope: EdgeModScope::FullEdge,
                kind,
            },
        });
        g.add_dependency("e_cut", "em");
        g
    }

    fn mesh_stats(m: &MockMesh) -> (usize, usize, usize) {
        use std::collections::HashMap;
        let q = |i: usize| -> (i64, i64, i64) {
            let b = i * 6;
            let quant = |v: f32| (v as f64 * 1e4).round() as i64;
            (
                quant(m.vertices[b]),
                quant(m.vertices[b + 1]),
                quant(m.vertices[b + 2]),
            )
        };
        let mut edges: HashMap<((i64, i64, i64), (i64, i64, i64)), u32> = HashMap::new();
        let mut inward = 0usize;
        for tri in m.indices.chunks_exact(3) {
            for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
                let (ka, kb) = (q(tri[a] as usize), q(tri[b] as usize));
                let key = if ka <= kb { (ka, kb) } else { (kb, ka) };
                *edges.entry(key).or_insert(0) += 1;
            }

            let pos = |i: u32| {
                let b = i as usize * 6;
                [
                    m.vertices[b] as f64,
                    m.vertices[b + 1] as f64,
                    m.vertices[b + 2] as f64,
                ]
            };
            let nrm = |i: u32| {
                let b = i as usize * 6;
                [
                    m.vertices[b + 3] as f64,
                    m.vertices[b + 4] as f64,
                    m.vertices[b + 5] as f64,
                ]
            };
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

        (
            edges.values().filter(|&&c| c == 1).count(),
            edges.values().filter(|&&c| c > 2).count(),
            inward,
        )
    }

    fn on_internal_circular_bite_wall(x: f32, y: f32) -> bool {
        let r = ((x - 20.0).powi(2) + (y - 8.0).powi(2)).sqrt();
        (r - 14.0).abs() < 0.08 && y > 5.25
    }

    fn circular_bite_internal_wall_struts(m: &MockMesh) -> usize {
        (0..m.edge_indices.len() / 2)
            .filter(|&e| {
                let ia = m.edge_indices[e * 2] as usize * 3;
                let ib = m.edge_indices[e * 2 + 1] as usize * 3;
                let a = [
                    m.edge_vertices[ia],
                    m.edge_vertices[ia + 1],
                    m.edge_vertices[ia + 2],
                ];
                let b = [
                    m.edge_vertices[ib],
                    m.edge_vertices[ib + 1],
                    m.edge_vertices[ib + 2],
                ];
                (a[0] - b[0]).abs() < 0.05
                    && (a[1] - b[1]).abs() < 0.05
                    && (a[2] - b[2]).abs() > 9.0
                    && on_internal_circular_bite_wall(a[0], a[1])
                    && on_internal_circular_bite_wall(b[0], b[1])
            })
            .count()
    }

    fn circular_bite_wall_normal_splits(m: &MockMesh) -> usize {
        use std::collections::HashMap;

        let quant = |v: f32| (v as f64 * 10_000.0).round() as i64;
        let mut normals: HashMap<(i64, i64, i64), Vec<[f32; 3]>> = HashMap::new();
        for v in m.vertices.chunks_exact(6) {
            if !on_internal_circular_bite_wall(v[0], v[1]) || v[5].abs() > 0.5 {
                continue;
            }
            normals
                .entry((quant(v[0]), quant(v[1]), quant(v[2])))
                .or_default()
                .push([v[3], v[4], v[5]]);
        }

        normals
            .values()
            .filter(|ns| {
                ns.len() >= 2
                    && ns.iter().enumerate().any(|(i, a)| {
                        ns.iter().skip(i + 1).any(|b| {
                            (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]).clamp(-1.0, 1.0) < 0.999
                        })
                    })
            })
            .count()
    }

    fn circular_bite_wall_face_ids(m: &MockMesh) -> std::collections::HashSet<u32> {
        let mut faces = std::collections::HashSet::new();
        for (t, tri) in m.indices.chunks_exact(3).enumerate() {
            let on_wall = tri.iter().all(|&vi| {
                let b = vi as usize * 6;
                on_internal_circular_bite_wall(m.vertices[b], m.vertices[b + 1])
                    && m.vertices[b + 5].abs() < 0.5
            });
            if on_wall {
                faces.insert(m.face_ids.get(t).copied().unwrap_or(0));
            }
        }
        faces
    }

    fn mesh_has_wire_edge_between(m: &MockMesh, p0: [f32; 3], p1: [f32; 3], tol: f32) -> bool {
        let dist = |a: [f32; 3], b: [f32; 3]| {
            ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
        };
        let point = |idx: u32| {
            let b = idx as usize * 3;
            [
                m.edge_vertices[b],
                m.edge_vertices[b + 1],
                m.edge_vertices[b + 2],
            ]
        };
        (0..m.edge_indices.len() / 2).any(|e| {
            let a = point(m.edge_indices[e * 2]);
            let b = point(m.edge_indices[e * 2 + 1]);
            (dist(a, p0) <= tol && dist(b, p1) <= tol) || (dist(a, p1) <= tol && dist(b, p0) <= tol)
        })
    }

    fn inside_removed_circular_bite_volume(p: [f32; 3]) -> bool {
        let r = ((p[0] - 20.0).powi(2) + (p[1] - 8.0).powi(2)).sqrt();
        r < 13.0 && p[1] > 5.1 && (-0.05..=10.05).contains(&p[2])
    }

    fn circular_bite_wall_chord_sample(p: [f32; 3], n: [f32; 3]) -> bool {
        let radial = [p[0] - 20.0, p[1] - 8.0];
        let r = (radial[0] * radial[0] + radial[1] * radial[1]).sqrt();
        if !(12.0..=14.25).contains(&r) || p[1] <= 5.0 || n[2].abs() > 0.55 {
            return false;
        }
        let nl = (n[0] * n[0] + n[1] * n[1]).sqrt();
        if nl <= 1.0e-5 || r <= 1.0e-5 {
            return false;
        }
        let dot = (radial[0] / r) * (n[0] / nl) + (radial[1] / r) * (n[1] / nl);
        dot.abs() > 0.75
    }

    fn circular_bite_ghost_sample_count(m: &MockMesh) -> usize {
        let vertex6 = |vi: u32| {
            let b = vi as usize * 6;
            [
                m.vertices[b],
                m.vertices[b + 1],
                m.vertices[b + 2],
                m.vertices[b + 3],
                m.vertices[b + 4],
                m.vertices[b + 5],
            ]
        };
        let mut count = 0usize;
        for v in m.vertices.chunks_exact(6) {
            let p = [v[0], v[1], v[2]];
            let n = [v[3], v[4], v[5]];
            if inside_removed_circular_bite_volume(p) && !circular_bite_wall_chord_sample(p, n) {
                count += 1;
            }
        }
        for tri in m.indices.chunks_exact(3) {
            let a = vertex6(tri[0]);
            let b = vertex6(tri[1]);
            let c = vertex6(tri[2]);
            let p = [
                (a[0] + b[0] + c[0]) / 3.0,
                (a[1] + b[1] + c[1]) / 3.0,
                (a[2] + b[2] + c[2]) / 3.0,
            ];
            let n = [
                (a[3] + b[3] + c[3]) / 3.0,
                (a[4] + b[4] + c[4]) / 3.0,
                (a[5] + b[5] + c[5]) / 3.0,
            ];
            if inside_removed_circular_bite_volume(p) && !circular_bite_wall_chord_sample(p, n) {
                count += 1;
            }
        }
        count
    }

    fn circle_points(center: (f32, f32), radius: f32) -> Vec<(f32, f32)> {
        (0..crate::CIRCLE_SEGS)
            .map(|i| {
                let a = (i as f32 / crate::CIRCLE_SEGS as f32) * std::f32::consts::TAU;
                (center.0 + radius * a.cos(), center.1 + radius * a.sin())
            })
            .collect()
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
        assert_eq!(
            g.cached_regions(&curves).len(),
            2,
            "rect+circle = annulus + disk"
        );

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
            let b2 = g2
                .evaluate_bodies(&std::collections::HashSet::new())
                .unwrap();
            let tris: usize = b2.iter().map(|(_, m)| m.indices.len() / 3).sum();
            assert!(
                tris > 0,
                "region selection {sel:?} must render a body, got {tris} tris"
            );
        }
    }

    #[test]
    fn circular_bite_newbody_keeps_clean_mesh_and_cylindrical_solid() {
        let g = circular_bite_graph(None);
        let bodies = g
            .evaluate_bodies(&std::collections::HashSet::new())
            .unwrap();
        assert_eq!(
            bodies.len(),
            1,
            "selected circular-bite region makes one body"
        );
        let mesh = &bodies[0].1;
        let (cracks, nonmanifold, inward) = mesh_stats(mesh);
        assert_eq!(
            cracks, 0,
            "circular-bite display mesh has {cracks} crack edges"
        );
        assert_eq!(
            nonmanifold, 0,
            "circular-bite display mesh has {nonmanifold} non-manifold edges"
        );
        assert_eq!(
            inward, 0,
            "circular-bite display mesh has {inward} inward triangles"
        );

        let solids = g
            .debug_kernel_solids(&std::collections::HashSet::new())
            .unwrap();
        let has_cylinder =
            solids.iter().any(|(_, parts)| {
                parts.iter().any(|solid| {
                    solid.shell().faces().iter().any(|f| {
                        matches!(f.surface(), Some(openrcad::geom::GeomSurface::Cylinder(_)))
                    })
                })
            });
        assert!(
            has_cylinder,
            "circular-bite body should keep an analytic cylindrical wall in the kernel solid"
        );
    }

    #[test]
    fn circular_bite_newbody_hides_internal_wall_segments() {
        let g = circular_bite_graph(None);
        let bodies = g
            .evaluate_bodies(&std::collections::HashSet::new())
            .unwrap();
        assert_eq!(
            bodies.len(),
            1,
            "selected circular-bite region makes one body"
        );
        let mesh = &bodies[0].1;
        assert_eq!(
            circular_bite_internal_wall_struts(mesh),
            0,
            "circular-bite display should not draw vertical construction seams inside the wall"
        );
        assert_eq!(
            circular_bite_wall_normal_splits(mesh),
            0,
            "circular-bite wall should not shade as separate flat panels"
        );
        let wall_faces = circular_bite_wall_face_ids(mesh);
        assert_eq!(
            wall_faces.len(),
            1,
            "circular-bite wall should select as one face, got face ids {wall_faces:?}"
        );
    }

    #[test]
    fn edge_mod_on_circular_bite_body_clears_pristine_mesh() {
        for kind in [
            crate::sketch::CornerKind::Chamfer,
            crate::sketch::CornerKind::Fillet,
        ] {
            let g = circular_bite_graph(Some(kind));
            let (live, warnings) = g
                .build_live(&std::collections::HashSet::new(), false)
                .unwrap();
            assert!(
                warnings.is_empty(),
                "{kind:?} on circular-bite body should not warn, got {warnings:?}"
            );
            assert_eq!(live.len(), 1, "{kind:?} keeps one live body");
            assert!(
                live[0].pristine.is_none(),
                "{kind:?} must clear the pristine sketch display mesh after modifying the B-Rep"
            );

            let bodies = tessellate_bodies(live);
            assert_eq!(bodies.len(), 1, "{kind:?} keeps one rendered body");
            let mesh = &bodies[0].1;
            let (cracks, _nonmanifold, _inward) = mesh_stats(mesh);
            assert_eq!(
                cracks, 0,
                "{kind:?} circular-bite edge-mod mesh has {cracks} cracks"
            );
            let max_z = mesh
                .vertices
                .chunks(6)
                .map(|v| v[2])
                .fold(f32::MIN, f32::max);
            assert!(
                max_z > 9.0,
                "{kind:?} should preserve most of the 10mm body height"
            );
        }
    }

    #[test]
    fn edge_mod_on_circular_bite_cutoff_edge_succeeds() {
        for kind in [
            crate::sketch::CornerKind::Chamfer,
            crate::sketch::CornerKind::Fillet,
        ] {
            let edge = circular_bite_cutoff_edge();
            let unmodified = circular_bite_graph(None)
                .evaluate_bodies(&std::collections::HashSet::new())
                .unwrap();
            assert!(
                mesh_has_wire_edge_between(&unmodified[0].1, edge.p0, edge.p1, 0.08),
                "test setup should expose the original sharp edge before {kind:?}"
            );

            let g = circular_bite_cutoff_edge_graph(kind);
            let (live, warnings) = g
                .build_live(&std::collections::HashSet::new(), false)
                .unwrap();
            assert!(
                warnings.is_empty(),
                "{kind:?} on edge ending at circular bite should not warn, got {warnings:?}"
            );
            assert_eq!(live.len(), 1, "{kind:?} keeps one live body");
            assert!(
                live[0].pristine.is_none(),
                "{kind:?} must clear pristine after modifying the B-Rep"
            );

            let bodies = tessellate_bodies(live);
            assert_eq!(bodies.len(), 1, "{kind:?} keeps one rendered body");
            let mesh = &bodies[0].1;
            assert_eq!(
                circular_bite_ghost_sample_count(mesh),
                0,
                "{kind:?} cutoff-edge result should not show a ghost cylinder/disk"
            );
            assert!(
                !mesh_has_wire_edge_between(mesh, edge.p0, edge.p1, 0.08),
                "{kind:?} should remove the original sharp cutoff edge"
            );
            let (cracks, _nonmanifold, _inward) = mesh_stats(mesh);
            assert_eq!(cracks, 0, "{kind:?} cutoff-edge result has {cracks} cracks");
        }
    }

    #[test]
    fn screenshot_radius_edge_mod_on_circular_bite_cutoff_edge_succeeds() {
        let edge = circular_bite_cutoff_edge();
        for kind in [
            crate::sketch::CornerKind::Fillet,
            crate::sketch::CornerKind::Chamfer,
        ] {
            let g = circular_bite_cutoff_edge_graph_with_dist(kind, 3.0);
            let (live, warnings) = g
                .build_live(&std::collections::HashSet::new(), false)
                .unwrap();
            assert!(
                warnings.is_empty(),
                "3.0mm {kind:?} runout should succeed without warning, got {warnings:?}"
            );
            assert_eq!(live.len(), 1, "3.0mm {kind:?} keeps one live body");
            assert!(
                live[0].pristine.is_none(),
                "3.0mm {kind:?} must clear pristine after modifying the B-Rep"
            );

            let bodies = tessellate_bodies(live);
            assert_eq!(bodies.len(), 1, "3.0mm {kind:?} keeps one rendered body");
            let mesh = &bodies[0].1;
            assert_eq!(
                circular_bite_ghost_sample_count(mesh),
                0,
                "3.0mm {kind:?} should not show a ghost cylinder/disk"
            );
            assert!(
                !mesh_has_wire_edge_between(mesh, edge.p0, edge.p1, 0.08),
                "3.0mm {kind:?} should remove the original sharp cutoff edge"
            );
            let (cracks, _nonmanifold, _inward) = mesh_stats(mesh);
            assert_eq!(cracks, 0, "3.0mm {kind:?} result has {cracks} cracks");
        }
    }

    #[test]
    fn gui_captured_circular_bite_cutoff_edge_fillet_matches_cylinder_cut() {
        let depth = 7.23;
        let edge = gui_captured_circular_bite_cutoff_edge(depth);
        let untouched = circular_bite_opposite_cutoff_edge_at_depth(depth);
        for kind in [
            crate::sketch::CornerKind::Fillet,
            crate::sketch::CornerKind::Chamfer,
        ] {
            let mut g = circular_bite_graph_with_depth(None, depth);
            g.add_feature(FeatureNode {
                id: format!("em_gui_{kind:?}"),
                name: "GUI Edge Mod".to_string(),
                feature: FeatureType::EdgeMod {
                    target: "e".to_string(),
                    edge: edge.clone(),
                    dist: 3.0,
                    dist_expr: None,
                    scope: EdgeModScope::FullEdge,
                    kind,
                },
            });
            g.add_dependency("e", &format!("em_gui_{kind:?}"));

            let (live, warnings) = g
                .build_live(&std::collections::HashSet::new(), false)
                .unwrap();
            assert!(
                warnings.is_empty(),
                "GUI-captured 3mm {kind:?} should not warn, got {warnings:?}"
            );
            assert_eq!(live.len(), 1, "GUI-captured {kind:?} keeps one body");
            assert!(
                live[0].pristine.is_none(),
                "GUI-captured {kind:?} must clear pristine after modifying the B-Rep"
            );

            let bodies = tessellate_bodies(live);
            assert_eq!(bodies.len(), 1, "GUI-captured {kind:?} renders one body");
            let mesh = &bodies[0].1;
            assert_eq!(
                circular_bite_ghost_sample_count(mesh),
                0,
                "GUI-captured {kind:?} should not show a ghost cylinder/disk"
            );
            assert!(
                !mesh_has_wire_edge_between(mesh, edge.p0, edge.p1, 0.08),
                "GUI-captured {kind:?} should remove the original sharp edge"
            );
            assert!(
                mesh_wire_path_covers(mesh, untouched.p0, untouched.p1, 0.08),
                "GUI-captured {kind:?} must not modify the opposite unselected front span"
            );
            assert_eq!(
                circular_bite_wall_normal_splits(mesh),
                0,
                "GUI-captured {kind:?} must keep the circular wall smooth"
            );
            let (cracks, _nonmanifold, _inward) = mesh_stats(mesh);
            assert_eq!(
                cracks, 0,
                "GUI-captured {kind:?} result has {cracks} cracks"
            );
        }
    }

    #[test]
    fn reported_depth6_1_56mm_circular_bite_fillet_succeeds() {
        let dist = 1.56;
        for depth in [6.0, 9.2] {
            let edge = gui_captured_circular_bite_cutoff_edge(depth);
            let untouched = circular_bite_opposite_cutoff_edge_at_depth(depth);

            let mut sketch = circular_bite_graph_with_depth(None, depth);
            sketch.add_feature(FeatureNode {
                id: format!("em_reported_{depth:.1}"),
                name: "Reported Fillet".to_string(),
                feature: FeatureType::EdgeMod {
                    target: "e".to_string(),
                    edge: edge.clone(),
                    dist,
                    dist_expr: None,
                    scope: EdgeModScope::FullEdge,
                    kind: crate::sketch::CornerKind::Fillet,
                },
            });
            sketch.add_dependency("e", &format!("em_reported_{depth:.1}"));

            let explicit = box_cut_circular_bite_graph_at_depth_with_dist(
                crate::sketch::CornerKind::Fillet,
                dist,
                depth,
            );

            for (label, graph) in [("sketch", sketch), ("box-minus-cylinder", explicit)] {
                let (bodies, warnings) = graph
                    .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
                    .unwrap();
                assert!(
                    warnings.is_empty(),
                    "{label} depth {depth:.2} 1.56mm fillet should not warn, got {warnings:?}"
                );
                assert_eq!(
                    bodies.len(),
                    1,
                    "{label} depth {depth:.2} fillet keeps one body"
                );
                let mesh = &bodies[0].1;
                assert_eq!(
                    circular_bite_ghost_sample_count(mesh),
                    0,
                    "{label} depth {depth:.2} fillet should not show ghost cylinder material"
                );
                assert!(
                    !mesh_has_wire_edge_between(mesh, edge.p0, edge.p1, 0.08),
                    "{label} depth {depth:.2} fillet should remove the original selected sharp edge"
                );
                assert!(
                    mesh_wire_path_covers(mesh, untouched.p0, untouched.p1, 0.08),
                    "{label} depth {depth:.2} fillet must preserve the unselected same-side span"
                );
                assert_eq!(
                    circular_bite_wall_normal_splits(mesh),
                    0,
                    "{label} depth {depth:.2} fillet must keep the circular wall smooth"
                );
                let (cracks, _nonmanifold, _inward) = mesh_stats(mesh);
                assert_eq!(
                    cracks, 0,
                    "{label} depth {depth:.2} fillet result has {cracks} cracks"
                );

                let solids = graph
                    .debug_kernel_solids(&std::collections::HashSet::new())
                    .unwrap();
                assert!(
                    solids.iter().any(|(_, parts)| parts
                        .iter()
                        .any(crate::mock_kernel::solid_has_cylindrical_face)),
                    "{label} depth {depth:.2} fillet should preserve analytic cylindrical topology"
                );
            }
        }
    }

    #[test]
    fn circular_bite_runout_does_not_accept_full_side_fallback() {
        for kind in [
            crate::sketch::CornerKind::Fillet,
            crate::sketch::CornerKind::Chamfer,
        ] {
            let g = circular_bite_cutoff_edge_graph_with_dist(kind, 3.0);
            let (bodies, warnings) = g
                .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
                .unwrap();
            assert!(
                warnings.is_empty(),
                "sketch circular-bite {kind:?} should not warn, got {warnings:?}"
            );
            let untouched = circular_bite_opposite_cutoff_edge_at_depth(10.0);
            assert!(
                mesh_wire_path_covers(&bodies[0].1, untouched.p0, untouched.p1, 0.08),
                "sketch circular-bite {kind:?} must preserve the unselected same-side span"
            );
            assert_eq!(
                circular_bite_ghost_sample_count(&bodies[0].1),
                0,
                "sketch circular-bite {kind:?} should not show ghost cylinder material"
            );

            let g = box_cut_circular_bite_graph_with_dist(kind, 3.0);
            let (bodies, warnings) = g
                .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
                .unwrap();
            assert!(
                warnings.is_empty(),
                "box-minus-cylinder {kind:?} should not warn, got {warnings:?}"
            );
            assert!(
                mesh_wire_path_covers(&bodies[0].1, untouched.p0, untouched.p1, 0.08),
                "box-minus-cylinder {kind:?} must preserve the unselected same-side span"
            );
        }
    }

    #[test]
    fn oversized_circular_bite_runout_fails_fast_and_leaves_body() {
        let unmodified = circular_bite_graph(None)
            .evaluate_bodies(&std::collections::HashSet::new())
            .unwrap();
        let g = circular_bite_cutoff_edge_graph_with_dist(crate::sketch::CornerKind::Fillet, 9.52);
        let (bodies, warnings) = g
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .unwrap();
        assert!(
            warnings.iter().any(|w| w.contains("local clearance")),
            "oversized circular-bite fillet should fail preflight, got {warnings:?}"
        );
        assert_eq!(bodies.len(), 1, "failed oversized fillet keeps one body");
        assert_eq!(
            bodies[0].1.indices.len(),
            unmodified[0].1.indices.len(),
            "oversized circular-bite fillet leaves the body unchanged"
        );
    }

    #[test]
    fn curved_circular_rim_selection_reaches_native_solver_and_fails_safely() {
        let mut g = circular_bite_graph(None);
        let x0 = 20.0 - (14.0_f32 * 14.0 - 3.0_f32 * 3.0).sqrt();
        let x1 = 20.0 + (14.0_f32 * 14.0 - 3.0_f32 * 3.0).sqrt();
        g.add_feature(FeatureNode {
            id: "em_rim".to_string(),
            name: "Edge Mod".to_string(),
            feature: FeatureType::EdgeMod {
                target: "e".to_string(),
                edge: EdgeRef {
                    p0: [x0, 5.0, 10.0],
                    p1: [x1, 5.0, 10.0],
                    n1: [0.0, 0.0, 1.0],
                    n2: [0.0, -1.0, 0.0],
                    curve: Some(EdgeCurveHint::Circle {
                        center: [20.0, 8.0, 10.0],
                        axis: [0.0, 0.0, 1.0],
                        x_dir: [1.0, 0.0, 0.0],
                        radius: 14.0,
                        start: 0.0,
                        end: std::f32::consts::PI,
                        closed: false,
                    }),
                    topology: None,
                },
                dist: 3.0,
                dist_expr: None,
                scope: EdgeModScope::FullEdge,
                kind: crate::sketch::CornerKind::Fillet,
            },
        });
        g.add_dependency("e", "em_rim");

        let (bodies, warnings) = g
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .unwrap();
        assert_eq!(bodies.len(), 1, "unsupported rim fillet keeps one body");
        assert!(
            warnings
                .iter()
                .any(|w| w.contains("cut trim requires a cylindrical blend")
                    || w.contains("not watertight and healthy")
                    || w.contains("non-manifold edges")),
            "curved rim fillet should reach the native solver and fail safely, got {warnings:?}"
        );
        assert!(
            warnings.iter().all(|w| !w.contains("not supported yet")),
            "curved rim fillet must not be rejected by the old app-level gate: {warnings:?}"
        );
    }

    #[test]
    fn direct_box_cylinder_cut_3mm_edge_mod_parity() {
        for kind in [
            crate::sketch::CornerKind::Fillet,
            crate::sketch::CornerKind::Chamfer,
        ] {
            let g = box_cut_circular_bite_graph_with_dist(kind, 3.0);
            let (bodies, warnings) = g
                .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
                .unwrap();
            assert_eq!(
                bodies.len(),
                1,
                "direct box-minus-cylinder {kind:?} keeps one body"
            );
            assert!(
                warnings.is_empty(),
                "direct box-minus-cylinder {kind:?} runout should not warn, got {warnings:?}"
            );
            let untouched = circular_bite_opposite_cutoff_edge_at_depth(10.0);
            assert!(
                mesh_wire_path_covers(&bodies[0].1, untouched.p0, untouched.p1, 0.08),
                "direct box-minus-cylinder {kind:?} must preserve the unselected front span"
            );
        }
    }

    #[test]
    fn edge_mod_validator_rejects_full_cylinder_ghost_for_circular_bite() {
        let reference = circular_bite_graph(None)
            .evaluate_bodies(&std::collections::HashSet::new())
            .unwrap();
        let ghost = crate::mock_kernel::circular_cylinder_tool(
            &circle_points((20.0, 8.0), 14.0),
            &[],
            10.0,
            &CoordinateSystem::XY,
        )
        .expect("test ghost cylinder should build");
        let ghost_mesh = MockMesh::from_solid(&ghost);
        assert!(
            circular_bite_ghost_sample_count(&ghost_mesh) > 0,
            "test setup should include samples inside the removed circular bite volume"
        );

        let err = edge_mod_candidate_stays_inside_reference(&reference[0].1, &ghost)
            .expect_err("full cylinder ghost must be rejected");
        assert!(
            err.contains("outside"),
            "validator should explain the ghost escaped the reference body, got {err}"
        );
    }

    #[test]
    fn edge_mod_on_sketched_prism_applies() {
        // The screenshots' box is a sketch→extrude prism (build_extrusion_solid),
        // not a make_box. Both chamfer AND fillet must apply to a top edge of such a
        // prism (pre-fix the fillet failed because the sewn top cap stored an inward
        // normal).
        for kind in [
            crate::sketch::CornerKind::Chamfer,
            crate::sketch::CornerKind::Fillet,
        ] {
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
                        curve: None,
                        topology: None,
                    },
                    dist: 2.11,
                    dist_expr: None,
                    scope: EdgeModScope::FullEdge,
                    kind,
                },
            });
            g.add_dependency("e", "em");
            let mut hidden = std::collections::HashSet::new();
            hidden.insert("s".to_string());
            let (bodies, warnings) = g.evaluate_bodies_with_warnings(&hidden).unwrap();
            assert_eq!(bodies.len(), 1, "{kind:?} on a prism must stay one body");
            assert!(
                warnings.is_empty(),
                "{kind:?} on a clean prism edge should not warn, got {warnings:?}"
            );
            // The top-front sharp edge (y=0, z=20) must be gone — the edge-mod applied.
            let m = &bodies[0].1;
            let sharp = m
                .vertices
                .chunks(6)
                .any(|v| v[1].abs() < 0.02 && (v[2] - 20.0).abs() < 0.02);
            assert!(
                !sharp,
                "{kind:?} should have removed the prism's top-front edge"
            );
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
        let (cb, cw) = gc
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .unwrap();
        assert_eq!(cb.len(), 1, "cut stays one body");
        assert!(
            cw.is_empty(),
            "a clean drill-through should not warn, got {cw:?}"
        );
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
        let (jb, jw) = gj
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .unwrap();
        assert_eq!(jb.len(), 1, "join stays one body");
        assert!(
            jw.is_empty(),
            "a clean boss join should not warn, got {jw:?}"
        );
        let max_z = jb
            .iter()
            .flat_map(|(_, m)| m.vertices.chunks(6))
            .map(|v| v[2])
            .fold(f32::MIN, f32::max);
        assert!(
            max_z >= 22.9,
            "join must add the boss (top z≈23), got {max_z}"
        );
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
            feature: FeatureType::Box {
                w: 10.0,
                h: 10.0,
                d: 10.0,
            },
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
        assert_eq!(
            bodies.len(),
            1,
            "boss-join must stay one body, got {}",
            bodies.len()
        );
        assert!(
            warnings.is_empty(),
            "a coplanar boss join should not warn, got {warnings:?}"
        );
        let max_z = bodies[0]
            .1
            .vertices
            .chunks(6)
            .map(|v| v[2])
            .fold(f32::MIN, f32::max);
        assert!(
            max_z >= 14.9,
            "joined boss must reach z≈15 (top of boss), got {max_z}"
        );
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
            feature: FeatureType::Box {
                w: 10.0,
                h: 10.0,
                d: 10.0,
            },
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
                    curve: None,
                    topology: None,
                },
                dist,
                dist_expr: None,
                scope: EdgeModScope::FullEdge,
                kind,
            },
        });
        g.add_dependency("extrude_3", "edgemod_4");
        g
    }

    #[test]
    fn edge_mod_on_boss_union_body_keeps_boss() {
        for kind in [
            crate::sketch::CornerKind::Chamfer,
            crate::sketch::CornerKind::Fillet,
        ] {
            let g = box_with_boss_then_edge_mod(2.0, kind);
            let (bodies, _warnings) = g
                .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
                .unwrap();
            assert_eq!(bodies.len(), 1, "{kind:?} on union body must stay one body");
            let m = &bodies[0].1;
            let max_z = m.vertices.chunks(6).map(|v| v[2]).fold(f32::MIN, f32::max);
            assert!(
                max_z >= 14.9,
                "{kind:?} must preserve the boss (top z≈15), got {max_z}"
            );
            // The modified bottom-front corner must not still be sharp.
            let sharp = m
                .vertices
                .chunks(6)
                .any(|v| v[1].abs() < 0.01 && v[2].abs() < 0.01);
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
        assert!(
            inside,
            "large chamfer flew vertices outside the 10³ block (runaway wedge)"
        );
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
        let min_y = mesh
            .vertices
            .chunks(6)
            .map(|v| v[1])
            .fold(f32::MAX, f32::min);
        let min_z = mesh
            .vertices
            .chunks(6)
            .map(|v| v[2])
            .fold(f32::MAX, f32::min);
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
        assert!(
            inside,
            "filleted body has vertices outside the original block"
        );
        // The sharp y=0,z=0 edge is rounded away: no vertex sits on both faces.
        let sharp = mesh
            .vertices
            .chunks(6)
            .any(|v| v[1].abs() < 0.01 && v[2].abs() < 0.01);
        assert!(!sharp, "the filleted edge should not still be sharp");
        // The round leaves intermediate facet normals between the two faces — at
        // least one vertex normal points partly along BOTH +(-y) and +(-z), i.e.
        // a curved-surface normal, not just the axis-aligned box faces.
        let has_round = mesh
            .vertices
            .chunks(6)
            .any(|v| v[4] < -0.15 && v[5] < -0.15 && v[3].abs() < 0.2);
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
                let interior =
                    |p: &[f32; 3]| p[1] > 0.05 && p[1] < 1.95 && p[2] > 0.05 && p[2] < 1.95;
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
        assert!(!warnings.is_empty(), "an oversized fillet should warn");
        let mesh = &bodies[0].1;
        let plain = MockMesh::make_box(10.0, 10.0, 10.0);
        assert_eq!(
            mesh.indices.len(),
            plain.indices.len(),
            "oversized fillet must leave the body unchanged"
        );
    }
}
