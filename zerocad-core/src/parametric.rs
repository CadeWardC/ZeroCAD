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
}

impl ParametricGraph {
    pub fn new() -> Self {
        let mut pg = Self {
            graph: DiGraph::new(),
            node_map: HashMap::new(),
            region_cache: RefCell::new(HashMap::new()),
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
        // Surface circular dependencies (toposort result is otherwise unused,
        // but a cycle should still fail the whole evaluation).
        toposort(&self.graph, None)
            .map_err(|_| "Circular dependency detected in history tree!".to_string())?;

        // Resolved once per build so every expression-driven dimension (extrude
        // depth and sketch dimensions alike) sees the current variable values.
        let vars = self.variable_map();
        let sketch_cache = self.sketch_region_cache(&vars);
        let mut live: Vec<LiveBody> = Vec::new();
        let mut warnings: Vec<String> = Vec::new();

        for idx in self.body_nodes_in_creation_order() {
            let node = &self.graph[idx];
            if hidden.contains(&node.id) {
                continue;
            }
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
                        &mut live,
                        &mut warnings,
                    );
                }
                _ => {}
            }
        }

        Ok((tessellate_bodies(live), warnings))
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
                    if let Some(s) = region_solid(region, cs, depth) {
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
                    if exact.is_some() || expanded.is_some() {
                        cut_tools.push(CutTool { exact, expanded });
                    }
                }
                ExtrudeMode::Join => {
                    // exact = perfect geometry when it resolves; dipped = near cap
                    // nudged INTO existing material to break the (almost always
                    // present) coplanarity with the face the sketch sits on. The
                    // dip is absorbed by the body it joins, leaving no artifact.
                    let exact = region_solid(region, cs, depth);
                    let dipped = region_solid(
                        region,
                        &overshoot_cs(cs, depth),
                        overshoot_depth(depth, 1.0),
                    );
                    if exact.is_some() || dipped.is_some() {
                        join_tools.push(JoinTool { exact, dipped });
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

/// How far a tool overshoots the sketch plane to break coplanarity, in mm.
/// Comfortably above the boolean solver's tolerance so the dip is unambiguous,
/// yet small enough to be invisible at part scale.
const CUT_OVERSHOOT: f32 = 0.1;

/// How far a cut tool's side walls are pushed past a coplanar body face, in mm.
/// The in-plane analogue of `CUT_OVERSHOOT` (which handles the end caps).
const CUT_WALL_GROW: f32 = 0.1;

/// A join's tool in two forms: the exact extrusion (perfect geometry when the
/// solver accepts it) and a fallback whose near cap dips into the target to
/// dodge coplanar faces.
struct JoinTool {
    exact: Option<KernelSolid>,
    dipped: Option<KernelSolid>,
}

/// A cut's tool in two forms, mirroring `JoinTool`: the exact extrusion (used
/// when the solver accepts it, so the pocket keeps the dimensions the user
/// drew) and an `expanded` fallback whose walls poke ~`CUT_WALL_GROW`mm past
/// the body's faces to dodge the coplanar-face case that makes truck's boolean
/// return `None`.
struct CutTool {
    exact: Option<KernelSolid>,
    expanded: Option<KernelSolid>,
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
            .exact
            .as_ref()
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
                    let unioned = tool
                        .exact
                        .as_ref()
                        .and_then(|t| crate::mock_kernel::union(part, t))
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
                }
            }
        }
        if !merged {
            // Joined nothing — keep the exact (un-dipped) volume as its own body.
            if let Some(s) = tool.exact.or(tool.dipped) {
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

/// Apply a Cut extrude: subtract each tool from every body part whose AABB it
/// overlaps. For each part it tries the exact tool first (the drawn pocket),
/// then the expanded tool (walls clear of coplanar body faces). A solver
/// failure on *both* leaves the original part intact (safer than dropping a
/// valid body) and is surfaced as a warning, since the body then comes out
/// without the pocket the user drew; a part fully consumed by the cut is removed.
fn apply_cut(
    live: &mut [LiveBody],
    extrude_id: &str,
    tools: Vec<CutTool>,
    warnings: &mut Vec<String>,
) {
    for tool in &tools {
        // Pre-test bbox from whichever variant exists (expanded ⊇ exact).
        let Some(tbb) = tool
            .expanded
            .as_ref()
            .or(tool.exact.as_ref())
            .and_then(crate::mock_kernel::solid_aabb)
        else {
            continue;
        };
        // Did the solver fail to subtract this tool from a body it actually
        // overlapped? If so the body keeps material the user meant to remove.
        let mut failed_on_overlap = false;
        for body in live.iter_mut() {
            let mut changed = false;
            let mut next: Vec<KernelSolid> = Vec::with_capacity(body.parts.len());
            for part in body.parts.drain(..) {
                let overlaps = crate::mock_kernel::solid_aabb(&part).map_or(true, |pbb| {
                    crate::mock_kernel::aabbs_overlap(&pbb, &tbb, 0.05)
                });
                if !overlaps {
                    next.push(part);
                    continue;
                }
                // Exact first (precise pocket), then the expanded fallback that
                // breaks wall-coplanarity. Both None → keep the part intact.
                let cut = tool
                    .exact
                    .as_ref()
                    .and_then(|t| crate::mock_kernel::difference(&part, t))
                    .or_else(|| {
                        tool.expanded
                            .as_ref()
                            .and_then(|t| crate::mock_kernel::difference(&part, t))
                    });
                match cut {
                    Some(d) => {
                        changed = true;
                        next.push(d);
                    }
                    // Solver failure or fully consumed. Keep the original part so
                    // a failed boolean doesn't delete material.
                    None => {
                        failed_on_overlap = true;
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

/// Apply a 3D fillet/chamfer: subtract the edge cutter from the target body's
/// parts. Mirrors [`apply_cut`]'s guarded, body-preserving strategy — a cutter
/// the solver can't subtract leaves the part intact and raises a warning rather
/// than dropping a valid body. The corner-push and end-overshoot offsets baked
/// into the cutter dodge the coplanar-face failures (see
/// [`crate::mock_kernel::edge_corner_cutter`]); the cutter is built once per
/// part-attempt so the precise corner is tried before the pushed-out fallback.
fn apply_edge_mod(
    mod_id: &str,
    target: &str,
    edge: &EdgeRef,
    dist: f32,
    kind: crate::sketch::CornerKind,
    live: &mut [LiveBody],
    warnings: &mut Vec<String>,
) {
    let fillet = matches!(kind, crate::sketch::CornerKind::Fillet);

    // exact: corner kept on the body edge (perfect geometry if the solver takes
    // it). robust: corner pushed just outside the body so the leg faces aren't
    // coplanar with the body faces. Both overshoot the edge ends.
    let exact = crate::mock_kernel::edge_corner_cutter(
        edge.p0, edge.p1, edge.n1, edge.n2, dist, fillet, EDGE_FILLET_SEGS, 0.0, CUT_OVERSHOOT,
    );
    let robust = crate::mock_kernel::edge_corner_cutter(
        edge.p0,
        edge.p1,
        edge.n1,
        edge.n2,
        dist,
        fillet,
        EDGE_FILLET_SEGS,
        EDGE_MOD_GROW,
        CUT_OVERSHOOT,
    );
    if exact.is_none() && robust.is_none() {
        warnings.push(format!(
            "Fillet/Chamfer '{mod_id}': the edge couldn't be turned into a cutter \
             (degenerate edge or size), so the body was left unchanged."
        ));
        return;
    }

    let Some(body) = live.iter_mut().find(|b| b.id == target) else {
        warnings.push(format!(
            "Fillet/Chamfer '{mod_id}': its target body no longer exists, so it \
             had no effect."
        ));
        return;
    };

    let mut applied = false;
    let mut next: Vec<KernelSolid> = Vec::with_capacity(body.parts.len());
    for part in body.parts.drain(..) {
        let cut = exact
            .as_ref()
            .and_then(|t| crate::mock_kernel::difference(&part, t))
            .or_else(|| {
                robust
                    .as_ref()
                    .and_then(|t| crate::mock_kernel::difference(&part, t))
            });
        match cut {
            // A fillet/chamfer must never delete the body: `a − cutter` is a
            // subset of `a`, but truck can hand back a degenerate solid whose
            // bounds collapse. Reject anything that lost most of the part's box.
            Some(d) if edge_mod_keeps_body(&part, &d) => {
                applied = true;
                next.push(d);
            }
            _ => next.push(part),
        }
    }
    body.parts = next;
    if applied {
        body.pristine = None;
    } else {
        warnings.push(format!(
            "Fillet/Chamfer '{mod_id}': the solver couldn't apply it to the body \
             (the edge may not be a clean convex corner), so the body was left \
             unchanged."
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
        // The bevel introduces a face whose outward normal points at ~45° between
        // the two original faces (-Y and -Z): n ≈ (0, -0.707, -0.707).
        let mesh = &bodies[0].1;
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
