//! Flat B-Rep storage utilizing generational index arenas (`slotmap`).
//!
//! Stores all elements (Vertices, Edges, Loops, Faces, Shells, Solids)
//! in flat pools. Adjacency is stored via versioned keys (e.g. `VertexId`, `EdgeId`)
//! which avoids circular pointers, references, or locks, and enables cache-friendly
//! traversal and lock-free parallel execution (via Arc).

use serde::{Deserialize, Serialize};
use slotmap::{new_key_type, SlotMap};

use crate::orientation::Orientation;
use crate::pcurve::PcurveData;
use openrcad_foundation::Pnt;
use openrcad_geom::{Curve, GeomCurve, GeomSurface, Surface};

new_key_type! {
    /// Generational index for a Vertex.
    pub struct VertexId;
    /// Generational index for an Edge.
    pub struct EdgeId;
    /// Generational index for a face-specific parametric curve.
    pub struct PcurveId;
    /// Generational index for a Loop (Wire).
    pub struct LoopId;
    /// Generational index for a Face.
    pub struct FaceId;
    /// Generational index for a Shell.
    pub struct ShellId;
    /// Generational index for a Solid.
    pub struct SolidId;
}

/// The geometric coordinate data of a Vertex in the B-Rep.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct VertexData {
    /// The coordinates of the vertex.
    pub point: Pnt,
    /// The local uncertainty tolerance of this vertex.
    pub tolerance: f64,
}

/// The bounding curve and parameter data of an Edge in the B-Rep.
///
/// An edge is stored once, in its natural sense: it always runs from `start` to
/// `end` along the curve's increasing parameter (`first <= last`). It carries **no
/// orientation of its own** — traversal direction is a property of each *use* of
/// the edge within a loop, recorded per-use in [`OrientedEdge`]. This makes the
/// co-edge the single source of truth for orientation, so an edge shared by two
/// loops traversed in opposite directions stays unambiguous.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct EdgeData {
    /// The supporting 3D curve (optional if degenerate).
    pub curve: Option<GeomCurve>,
    /// The parameter of the start vertex on the curve.
    pub first: f64,
    /// The parameter of the end vertex on the curve.
    pub last: f64,
    /// The start vertex index.
    pub start: VertexId,
    /// The end vertex index.
    pub end: VertexId,
    /// The local uncertainty tolerance of this edge.
    pub tolerance: f64,
}
/// An edge reference with its loop-specific traversal orientation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct OrientedEdge {
    /// The edge ID.
    pub id: EdgeId,
    /// The traversal orientation relative to the edge's natural direction.
    pub orientation: Orientation,
    /// Face-specific 2D representation of this edge-use.
    #[serde(default)]
    pub pcurve: Option<PcurveId>,
}

impl OrientedEdge {
    /// Construct an edge-use without a parametric curve.
    #[inline]
    pub const fn new(id: EdgeId, orientation: Orientation) -> Self {
        Self {
            id,
            orientation,
            pcurve: None,
        }
    }

    /// Attach a face-specific parametric curve to this edge-use.
    #[inline]
    pub const fn with_pcurve(mut self, pcurve: PcurveId) -> Self {
        self.pcurve = Some(pcurve);
        self
    }
}

/// Explicit name for an oriented, face-specific use of a 3D edge.
pub type Coedge = OrientedEdge;

/// The ordered list of edges forming a Loop (Wire) in the B-Rep.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LoopData {
    /// The boundary edges in sequence.
    pub edges: Vec<OrientedEdge>,
}

/// The parametric surface and boundary loops of a Face in the B-Rep.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FaceData {
    /// The carrying surface.
    pub surface: Option<GeomSurface>,
    /// The outer boundary loop.
    pub outer_wire: Option<LoopId>,
    /// The inner boundary loops (holes).
    pub inner_wires: Vec<LoopId>,
    /// The orientation of the face normal.
    pub orientation: Orientation,
}

/// The faces belonging to a Shell in the B-Rep.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ShellData {
    /// The face indices.
    pub faces: Vec<FaceId>,
}

/// The boundary shells of a Solid in the B-Rep.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SolidData {
    /// The boundary shell indices.
    pub shells: Vec<ShellId>,
}

/// Central topological arena containing all data of a CAD shape.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct BRep {
    /// All vertices.
    pub vertices: SlotMap<VertexId, VertexData>,
    /// All edges.
    pub edges: SlotMap<EdgeId, EdgeData>,
    /// Face-specific parametric curves referenced by coedges.
    #[serde(default)]
    pub pcurves: SlotMap<PcurveId, PcurveData>,
    /// All loops.
    pub loops: SlotMap<LoopId, LoopData>,
    /// All faces.
    pub faces: SlotMap<FaceId, FaceData>,
    /// All shells.
    pub shells: SlotMap<ShellId, ShellData>,
    /// All solids.
    pub solids: SlotMap<SolidId, SolidData>,
}

/// Do two edges with equal curves and shared endpoints also cover the same
/// span? Compares a curve-midpoint sample so two sub-arcs of one circle (which
/// share a curve AND both endpoints but sweep different ranges) are recognised
/// as distinct. Curve-less degenerate edges carry no span, so equal endpoints
/// already imply the same edge.
fn edge_midpoints_match(a: &EdgeData, b: &EdgeData) -> bool {
    match (&a.curve, &b.curve) {
        (Some(ca), Some(cb)) => {
            let ma = ca.point(0.5 * (a.first + a.last));
            let mb = cb.point(0.5 * (b.first + b.last));
            ma.distance(&mb) <= openrcad_foundation::tolerance::CONFUSION * 10.0
        }
        _ => true,
    }
}

impl BRep {
    /// Create an empty B-Rep arena.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }

    /// Merges another B-Rep's contents into `self` while:
    /// 1. Re-mapping keys correctly.
    /// 2. Deduplicating vertices within Precision confusion tolerance (coincident).
    /// 3. Deduplicating edges sharing the same start/end vertices and curves.
    pub fn merge(&mut self, other: &Self) -> MergeMap {
        self.merge_many(&[other]).pop().unwrap()
    }

    /// Bulk [`BRep::merge`]: merge many source arenas in one pass, returning one
    /// [`MergeMap`] per source, in order. The vertex and edge deduplication
    /// indexes (a spatial hash grid and an endpoint-pair bucket map) are built
    /// once over `self` and maintained incrementally across all sources, so
    /// merging N small arenas costs O(total entities) instead of the O(n²) the
    /// per-element linear scans gave — sewing a skinned solid's thousands of
    /// quad faces spent whole seconds there. Dedup semantics match serial
    /// `merge` calls exactly.
    pub fn merge_many(&mut self, others: &[&Self]) -> Vec<MergeMap> {
        let tol = openrcad_foundation::tolerance::CONFUSION;
        // Cells no smaller than the match tolerance: two points within `tol`
        // always land in the same or an adjacent cell, so the 27-neighbour
        // probe sees every candidate the old full scan would have.
        let cell = tol.max(1e-12);
        let key_of = |p: &Pnt| -> (i64, i64, i64) {
            (
                (p.x() / cell).floor() as i64,
                (p.y() / cell).floor() as i64,
                (p.z() / cell).floor() as i64,
            )
        };
        let mut vgrid: std::collections::HashMap<(i64, i64, i64), Vec<VertexId>> =
            std::collections::HashMap::new();
        for (v_id, v_data) in &self.vertices {
            vgrid.entry(key_of(&v_data.point)).or_default().push(v_id);
        }
        // Edges can only merge when they share BOTH (deduplicated) endpoint
        // vertices, in either direction — bucket by the unordered pair.
        let pair_of = |a: VertexId, b: VertexId| if a <= b { (a, b) } else { (b, a) };
        let mut ebuckets: std::collections::HashMap<(VertexId, VertexId), Vec<EdgeId>> =
            std::collections::HashMap::new();
        for (e_id, e_data) in &self.edges {
            ebuckets
                .entry(pair_of(e_data.start, e_data.end))
                .or_default()
                .push(e_id);
        }

        let mut maps = Vec::with_capacity(others.len());
        for other in others {
            let mut map = MergeMap::new();

            // 1. Merge and deduplicate vertices
            for (v_id, v_data) in &other.vertices {
                let (kx, ky, kz) = key_of(&v_data.point);
                let mut matched = None;
                'probe: for dx in -1..=1_i64 {
                    for dy in -1..=1_i64 {
                        for dz in -1..=1_i64 {
                            let Some(bucket) = vgrid.get(&(kx + dx, ky + dy, kz + dz)) else {
                                continue;
                            };
                            for &cand in bucket {
                                if self.vertices[cand].point.is_equal(&v_data.point, tol) {
                                    matched = Some(cand);
                                    break 'probe;
                                }
                            }
                        }
                    }
                }
                let new_id = if let Some(m_id) = matched {
                    m_id
                } else {
                    let id = self.vertices.insert(*v_data);
                    vgrid.entry((kx, ky, kz)).or_default().push(id);
                    id
                };
                map.vertices.insert(v_id, new_id);
            }

            // 2. Merge and deduplicate edges
            for (e_id, e_data) in &other.edges {
                let new_start = map.vertices[&e_data.start];
                let new_end = map.vertices[&e_data.end];
                let bkey = pair_of(new_start, new_end);

                let mut matched = None;
                if let Some(bucket) = ebuckets.get(&bkey) {
                    for &cand in bucket {
                        let self_e_data = &self.edges[cand];
                        let endpoints_match = (self_e_data.start == new_start
                            && self_e_data.end == new_end)
                            || (self_e_data.start == new_end && self_e_data.end == new_start);
                        // Curve equality + shared endpoints is not enough to call two
                        // edges the same: the TWO arcs of one circle (e.g. two
                        // semicircles, or the thirds of a cylinder rim) share a curve
                        // AND both endpoints but cover different spans. Merging them
                        // loses one arc's parameter range and collapses distinct
                        // boundary edges into one — a false non-manifold downstream.
                        // A midpoint sample distinguishes co-endpoint sub-arcs; for
                        // lines (and identical arcs) the midpoints coincide, so this is
                        // a no-op there.
                        if endpoints_match
                            && self_e_data.curve == e_data.curve
                            && edge_midpoints_match(self_e_data, e_data)
                        {
                            matched = Some(cand);
                            break;
                        }
                    }
                }

                let new_id = if let Some(m_id) = matched {
                    m_id
                } else {
                    let id = self.edges.insert(EdgeData {
                        curve: e_data.curve.clone(),
                        first: e_data.first,
                        last: e_data.last,
                        start: new_start,
                        end: new_end,
                        tolerance: e_data.tolerance,
                    });
                    ebuckets.entry(bkey).or_default().push(id);
                    id
                };
                map.edges.insert(e_id, new_id);
            }

            // Pcurves are owned by coedges, so they remain face-specific even
            // when their corresponding 3D edges are deduplicated.
            for (pcurve_id, pcurve) in &other.pcurves {
                let new_id = self.pcurves.insert(pcurve.clone());
                map.pcurves.insert(pcurve_id, new_id);
            }

            // 3. Merge loops
            for (l_id, l_data) in &other.loops {
                let new_edges: Vec<OrientedEdge> = l_data
                    .edges
                    .iter()
                    .map(|&oe| OrientedEdge {
                        id: map.edges[&oe.id],
                        orientation: oe.orientation,
                        pcurve: oe.pcurve.map(|id| map.pcurves[&id]),
                    })
                    .collect();
                let new_id = self.loops.insert(LoopData { edges: new_edges });
                map.loops.insert(l_id, new_id);
            }

            // 4. Merge faces
            for (f_id, f_data) in &other.faces {
                let new_outer = f_data.outer_wire.map(|w| map.loops[&w]);
                let new_inner: Vec<LoopId> =
                    f_data.inner_wires.iter().map(|&w| map.loops[&w]).collect();
                let new_id = self.faces.insert(FaceData {
                    surface: f_data.surface.clone(),
                    outer_wire: new_outer,
                    inner_wires: new_inner,
                    orientation: f_data.orientation,
                });
                map.faces.insert(f_id, new_id);
            }

            // 5. Merge shells
            for (s_id, s_data) in &other.shells {
                let new_faces: Vec<FaceId> = s_data.faces.iter().map(|&f| map.faces[&f]).collect();
                let new_id = self.shells.insert(ShellData { faces: new_faces });
                map.shells.insert(s_id, new_id);
            }

            // 6. Merge solids
            for (so_id, so_data) in &other.solids {
                let new_shells: Vec<ShellId> =
                    so_data.shells.iter().map(|&s| map.shells[&s]).collect();
                let new_id = self.solids.insert(SolidData { shells: new_shells });
                map.solids.insert(so_id, new_id);
            }

            maps.push(map);
        }
        maps
    }

    /// Discard every face not in `keep`, plus every loop, edge, and vertex no
    /// longer reachable from a kept face. Any shell that references a dropped
    /// face — and any solid that references a dropped shell — is removed too, so
    /// the arena never keeps a dangling reference. Callers that prune faces are
    /// expected to (re)build the shell they want afterwards.
    ///
    /// This is the garbage collector for [`BRep::merge`]: `merge` copies *all*
    /// entities of a source arena, so assembling a face from edges that live in
    /// another face's arena (as prism/sweep do) drags that whole face — and its
    /// shell — in as an orphan. Sewing then leaves the arena holding faces the
    /// shell never references — and worse, orphan faces can share a loop with a
    /// real face, so a later `partition_face` that removes that loop leaves the
    /// orphan dangling and any full-arena traversal (BVH build, validation,
    /// `merge`) panics. Pruning to the reachable set keeps the arena in
    /// lock-step with the shell.
    pub fn retain_faces(&mut self, keep: &[FaceId]) {
        use std::collections::HashSet;
        let keep_faces: HashSet<FaceId> = keep.iter().copied().collect();

        let mut keep_loops: HashSet<LoopId> = HashSet::new();
        for &f in &keep_faces {
            if let Some(fd) = self.faces.get(f) {
                if let Some(o) = fd.outer_wire {
                    keep_loops.insert(o);
                }
                keep_loops.extend(fd.inner_wires.iter().copied());
            }
        }

        let mut keep_edges: HashSet<EdgeId> = HashSet::new();
        let mut keep_pcurves: HashSet<PcurveId> = HashSet::new();
        for &l in &keep_loops {
            if let Some(ld) = self.loops.get(l) {
                keep_edges.extend(ld.edges.iter().map(|oe| oe.id));
                keep_pcurves.extend(ld.edges.iter().filter_map(|oe| oe.pcurve));
            }
        }

        let mut keep_verts: HashSet<VertexId> = HashSet::new();
        for &e in &keep_edges {
            if let Some(ed) = self.edges.get(e) {
                keep_verts.insert(ed.start);
                keep_verts.insert(ed.end);
            }
        }

        self.faces.retain(|id, _| keep_faces.contains(&id));
        self.loops.retain(|id, _| keep_loops.contains(&id));
        self.pcurves.retain(|id, _| keep_pcurves.contains(&id));
        self.edges.retain(|id, _| keep_edges.contains(&id));
        self.vertices.retain(|id, _| keep_verts.contains(&id));

        // Drop shells that referenced any dropped face, and solids that
        // referenced a dropped shell, so no dangling reference survives.
        self.shells
            .retain(|_, s| s.faces.iter().all(|f| keep_faces.contains(f)));
        let keep_shells: HashSet<ShellId> = self.shells.keys().collect();
        self.solids
            .retain(|_, so| so.shells.iter().all(|s| keep_shells.contains(s)));
    }

    /// Returns a transformed copy of this B-Rep, transforming all vertices, curves, and surfaces.
    pub fn transformed(&self, t: &openrcad_foundation::Trsf) -> Self {
        let mut cloned = self.clone();
        // Local uncertainty is expressed in model units. Rigid transforms keep
        // it unchanged, while a uniform scale must carry it into the scaled
        // coordinate system just like every other length-valued quantity.
        let tolerance_scale = t.scale_factor().abs();
        let pcurve_scales = exact_pcurve_coordinate_scales(self, t);
        for (_, v_data) in &mut cloned.vertices {
            v_data.point = t.transform_point(&v_data.point);
            v_data.tolerance *= tolerance_scale;
        }
        for (_, e_data) in &mut cloned.edges {
            if let Some(ref mut c) = e_data.curve {
                // Lines and parabolas use distance-valued parameters. Their
                // supporting geometry is scaled below, so their trimmed range
                // must be scaled as well to keep both vertices on the curve.
                if matches!(
                    c,
                    openrcad_geom::GeomCurve::Line(_) | openrcad_geom::GeomCurve::Parabola(_)
                ) {
                    e_data.first *= tolerance_scale;
                    e_data.last *= tolerance_scale;
                }
                *c = c.transformed(t);
            }
            e_data.tolerance *= tolerance_scale;
        }
        for (_, f_data) in &mut cloned.faces {
            if let Some(ref mut s) = f_data.surface {
                *s = s.transformed(t);
            }
        }
        for (pcurve_id, (u_scale, v_scale)) in pcurve_scales {
            let Some(mapped) = self.pcurves[pcurve_id].scaled_surface_coordinates(u_scale, v_scale)
            else {
                continue;
            };
            cloned.pcurves[pcurve_id] = mapped;
        }
        cloned
    }
}

/// Exact UV coordinate scaling induced by a positive uniform 3D scale.
fn surface_coordinate_scale(surface: &GeomSurface, scale: f64) -> Option<(f64, f64)> {
    if scale == 1.0 {
        return Some((1.0, 1.0));
    }
    match surface {
        GeomSurface::Plane(_) => Some((scale, scale)),
        GeomSurface::Cylinder(_) | GeomSurface::Cone(_) => Some((1.0, scale)),
        GeomSurface::Sphere(_)
        | GeomSurface::Torus(_)
        | GeomSurface::BSpline(_)
        | GeomSurface::Gregory(_) => Some((1.0, 1.0)),
        GeomSurface::Offset(offset) => surface_coordinate_scale(&offset.base, scale),
        GeomSurface::Ruled(ruled) => {
            let first = curve_parameter_scale(&ruled.curve1, scale);
            let second = curve_parameter_scale(&ruled.curve2, scale);
            (first == second).then_some((first, 1.0))
        }
    }
}

fn curve_parameter_scale(curve: &GeomCurve, scale: f64) -> f64 {
    match curve {
        // These parameters are distances in model units.
        GeomCurve::Line(_) | GeomCurve::Parabola(_) => scale,
        // The remaining curve parameters are angular, dimensionless, or the
        // unchanged knot coordinate of transformed control points.
        GeomCurve::Circle(_)
        | GeomCurve::Ellipse(_)
        | GeomCurve::Hyperbola(_)
        | GeomCurve::BSpline(_)
        | GeomCurve::Helix(_)
        | GeomCurve::TorusPlaneSection(_)
        | GeomCurve::Reparametrized(_)
        | GeomCurve::TorusSurfaceCurve(_) => 1.0,
    }
}

fn exact_pcurve_coordinate_scales(
    brep: &BRep,
    transform: &openrcad_foundation::Trsf,
) -> std::collections::HashMap<PcurveId, (f64, f64)> {
    let scale = transform.scale_factor();
    if !scale.is_finite() || scale <= 0.0 || transform.reverses_orientation() {
        return std::collections::HashMap::new();
    }

    let mut mappings = std::collections::HashMap::new();
    for face in brep.faces.values() {
        let Some(surface) = face.surface.as_ref() else {
            continue;
        };
        let Some(coordinate_scale) = surface_coordinate_scale(surface, scale) else {
            continue;
        };
        for loop_id in face.outer_wire.iter().chain(&face.inner_wires) {
            for coedge in &brep.loops[*loop_id].edges {
                let Some(pcurve_id) = coedge.pcurve else {
                    continue;
                };
                mappings
                    .entry(pcurve_id)
                    .and_modify(|existing| {
                        if *existing != coordinate_scale {
                            *existing = (f64::NAN, f64::NAN);
                        }
                    })
                    .or_insert(coordinate_scale);
            }
        }
    }
    mappings.retain(|_, (u_scale, v_scale)| u_scale.is_finite() && v_scale.is_finite());
    mappings
}

/// Helper mapping old keys to new keys after merging BReps.
pub struct MergeMap {
    /// Vertex mapping
    pub vertices: std::collections::HashMap<VertexId, VertexId>,
    /// Edge mapping
    pub edges: std::collections::HashMap<EdgeId, EdgeId>,
    /// Pcurve mapping
    pub pcurves: std::collections::HashMap<PcurveId, PcurveId>,
    /// Loop mapping
    pub loops: std::collections::HashMap<LoopId, LoopId>,
    /// Face mapping
    pub faces: std::collections::HashMap<FaceId, FaceId>,
    /// Shell mapping
    pub shells: std::collections::HashMap<ShellId, ShellId>,
    /// Solid mapping
    pub solids: std::collections::HashMap<SolidId, SolidId>,
}

impl MergeMap {
    fn new() -> Self {
        Self {
            vertices: std::collections::HashMap::new(),
            edges: std::collections::HashMap::new(),
            pcurves: std::collections::HashMap::new(),
            loops: std::collections::HashMap::new(),
            faces: std::collections::HashMap::new(),
            shells: std::collections::HashMap::new(),
            solids: std::collections::HashMap::new(),
        }
    }
}
