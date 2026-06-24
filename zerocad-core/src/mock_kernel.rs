//! Geometry kernel — backed by the `openrcad` pure-Rust B-Rep CAD kernel.
//!
//! The `MockMesh` name and field layout are preserved so existing parametric
//! and rendering code keeps working unchanged. Internally each constructor
//! builds a real `openrcad::topo::Solid`, tessellates it via `openrcad::mesh`,
//! and flattens the result into the same interleaved position+normal vertex
//! buffer the egui painter expects.
//!
//! Wireframe edges are still produced analytically (matching the previous
//! procedural output) — extracting them from the B-Rep topology is deferred
//! to the GPU-viewport phase.

use std::collections::{HashMap, HashSet};

use openrcad::algo::{boolean_checked, chamfer_edges, fillet_edges, prism, BooleanOp};
use openrcad::foundation::{Ax2, Ax3, Dir, Pnt, Vec as GeomVec};
use openrcad::geom::{Circle, Curve, CylindricalSurface, GeomCurve, GeomSurface, Plane};
use openrcad::mesh::tessellate;
use openrcad::primitives::{make_box, make_cylinder};
use openrcad::topo::{Edge, Face, Orientation, Solid, Vertex, Wire};

use crate::geometry::Vec3;

/// The kernel's solid type (an `openrcad` B-Rep solid). Re-exported so the
/// parametric evaluator can hold solids between features and combine them with
/// boolean operations (join/cut) before tessellating to a `MockMesh`.
pub type KernelSolid = Solid;

/// Tessellation chordal tolerance (in model units / mm). 0.05mm produces a
/// smooth cylinder without explosive triangle counts. Will become a user-facing
/// setting in a later phase.
const TESS_TOL: f64 = 0.05;

/// Tessellation angular tolerance (radians) handed to `openrcad::mesh::tessellate`.
const TESS_ANGLE: f64 = 0.5;

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct MockMesh {
    /// Interleaved [x, y, z, nx, ny, nz] per vertex.
    pub vertices: Vec<f32>,
    pub indices: Vec<u32>,
    /// Flat [x, y, z] per vertex, paired in order for line segments.
    pub edge_vertices: Vec<f32>,
    /// Index pairs into `edge_vertices` (each consecutive pair = one line).
    pub edge_indices: Vec<u32>,
    /// The two adjacent face normals per edge, as [n1x,n1y,n1z, n2x,n2y,n2z].
    /// Length is `(edge_indices.len() / 2) * 6`. Used for hidden-line removal:
    /// an edge is hidden only when *both* its faces point away from the camera.
    /// May be empty for legacy meshes — renderers must treat empty as "no info".
    #[serde(default)]
    pub edge_face_normals: Vec<f32>,
    /// One B-rep face id per triangle (length == `indices.len() / 3`). Triangles
    /// sharing an id belong to the same planar/cylindrical face, which lets the
    /// viewport select a whole face at once. Ids are unique within a mesh and are
    /// rebased on `append`/merge so combined meshes keep distinct faces.
    #[serde(default)]
    pub face_ids: Vec<u32>,
    /// One group id per edge **segment** (length == `edge_indices.len() / 2`).
    /// Segments sharing an id form a single topological edge — every chord of one
    /// fillet arc, or the thirds of a cylinder's rim — so the viewport can select
    /// and highlight a whole curved edge at once instead of a lone chord. Empty on
    /// legacy/cached meshes; consumers then fall back to per-segment selection.
    #[serde(default)]
    pub edge_groups: Vec<u32>,
}

impl MockMesh {
    pub fn empty() -> Self {
        Self {
            vertices: Vec::new(),
            indices: Vec::new(),
            edge_vertices: Vec::new(),
            edge_indices: Vec::new(),
            edge_face_normals: Vec::new(),
            face_ids: Vec::new(),
            edge_groups: Vec::new(),
        }
    }

    /// Largest face id currently in this mesh, or `None` when there are no faces.
    fn max_face_id(&self) -> Option<u32> {
        self.face_ids.iter().copied().max()
    }

    /// Append another mesh into this one, rebasing its indices. Used to build a
    /// combined mesh (e.g. an extrude preview spanning several faces).
    pub fn append(&mut self, other: MockMesh) {
        let v_offset = (self.vertices.len() / 6) as u32;
        let e_offset = (self.edge_vertices.len() / 3) as u32;
        // Shift incoming face ids past ours so the two meshes' faces stay distinct.
        let f_offset = self.max_face_id().map_or(0, |m| m + 1);
        // Likewise shift edge group ids so the two meshes' topological edges
        // (e.g. each body's fillet arcs) stay independently selectable.
        let g_offset = self.edge_groups.iter().copied().max().map_or(0, |m| m + 1);

        self.vertices.reserve(other.vertices.len());
        self.indices.reserve(other.indices.len());
        self.edge_vertices.reserve(other.edge_vertices.len());
        self.edge_indices.reserve(other.edge_indices.len());
        self.edge_face_normals
            .reserve(other.edge_face_normals.len());
        self.face_ids.reserve(other.face_ids.len());
        self.edge_groups.reserve(other.edge_groups.len());

        self.vertices.extend(other.vertices);
        for idx in other.indices {
            self.indices.push(idx + v_offset);
        }
        self.edge_vertices.extend(other.edge_vertices);
        for idx in other.edge_indices {
            self.edge_indices.push(idx + e_offset);
        }
        self.edge_face_normals.extend(other.edge_face_normals);
        for fid in other.face_ids {
            self.face_ids.push(fid + f_offset);
        }
        for g in other.edge_groups {
            self.edge_groups.push(g + g_offset);
        }
    }

    /// Axis-aligned box with one corner at the origin, opposite corner at (w, h, d).
    pub fn make_box(w: f32, h: f32, d: f32) -> Self {
        let solid = box_solid(w, h, d);

        let (vertices, indices, face_ids) = solid_to_flat_mesh(&solid, false, false);

        let (edge_vertices, edge_indices, edge_face_normals) = build_box_wireframe(w, h, d);
        let edge_groups = group_edge_segments(&edge_vertices, &edge_indices, None);

        Self {
            vertices,
            indices,
            edge_vertices,
            edge_indices,
            edge_face_normals,
            face_ids,
            edge_groups,
        }
    }

    /// Cylinder along the +Y axis, base centered at origin, radius r, height h.
    /// `segments` is currently advisory — truck tessellates to TESS_TOL.
    pub fn make_cylinder(r: f32, h: f32, segments: u32) -> Self {
        let solid = match build_cylinder_solid(r as f64, h as f64) {
            Some(s) => s,
            None => return Self::empty(),
        };

        let (vertices, indices, face_ids) = solid_to_flat_mesh(&solid, false, false);
        let (edge_vertices, edge_indices, edge_face_normals) =
            build_cylinder_wireframe(r, h, segments.max(4));
        let edge_groups = group_edge_segments(&edge_vertices, &edge_indices, None);

        Self {
            vertices,
            indices,
            edge_vertices,
            edge_indices,
            edge_face_normals,
            face_ids,
            edge_groups,
        }
    }

    /// Extrude a 2D face (outer `points` plus optional `holes`, given in the
    /// 2D `cs` plane coordinates) by `depth` along the plane normal. Holes
    /// produce a through-pocket (e.g. a square hole drawn inside a circle
    /// extrudes to a tube).
    pub fn make_extruded_sketch(
        points: &[(f32, f32)],
        holes: &[Vec<(f32, f32)>],
        depth: f32,
        cs: &crate::geometry::CoordinateSystem,
    ) -> Self {
        if points.len() < 3 || depth.abs() < f32::EPSILON {
            return Self::empty();
        }

        // A hole-free circular profile is displayed as a true cylinder: a smooth
        // curved wall plus a clean wireframe (two rim circles + a few silhouette
        // struts), instead of the faceted prism the boolean path uses. Anything
        // else (polygons, holed profiles) extrudes as a prism.
        let circle = if holes.is_empty() {
            circle_profile(points)
        } else {
            None
        };

        let solid = match circle {
            Some((cu, cv, r)) => oriented_cylinder_solid(cs, cu, cv, r, depth),
            None => build_extrusion_solid(points, holes, depth as f64, cs, false)
                .or_else(|| build_extrusion_solid(points, &[], depth as f64, cs, false)),
        };
        let solid = match solid {
            Some(s) => s,
            None => return Self::empty(),
        };

        // Orient the shell robustly (triangle adjacency + signed volume) rather
        // than the cheap per-triangle centroid repair. A region split out of an
        // arrangement of overlapping sketch shapes is frequently NON-CONVEX (e.g.
        // a rectangle with a circular bite where a circle crossed it); the
        // centroid test then misjudges triangles on the concave side and leaves
        // them inward-facing, so they back-face cull and the body renders with
        // holes. Convex profiles (a plain rectangle, the cylinder cap) orient
        // identically either way, so this is strictly safer.
        let (vertices, indices, face_ids) = solid_to_flat_mesh(&solid, true, false);

        let (edge_vertices, edge_indices, edge_face_normals) = match circle {
            Some((cu, cv, r)) => build_oriented_cylinder_wireframe(cs, cu, cv, r, depth),
            None => build_extrusion_wireframe(points, holes, depth, cs),
        };
        let edge_groups = group_edge_segments(&edge_vertices, &edge_indices, None);

        Self {
            vertices,
            indices,
            edge_vertices,
            edge_indices,
            edge_face_normals,
            face_ids,
            edge_groups,
        }
    }

    /// Tessellate an arbitrary kernel solid (typically a boolean result) into a
    /// renderable mesh. Unlike the analytic constructors above, the wireframe is
    /// extracted from the solid's B-Rep edges, and hidden-line normals are left
    /// empty (the renderer then shows every edge).
    pub fn from_solid(solid: &KernelSolid) -> Self {
        let (vertices, indices, face_ids) = solid_to_flat_mesh(solid, true, false);
        // Derive the wireframe from the tessellation's *feature* edges (borders
        // between two distinct faces), not the raw B-Rep edge list. This gives
        // every edge its two adjacent face normals — so the renderer's
        // hidden-line removal works for boolean results just like it does for
        // primitives, instead of x-raying every edge — and it silently drops the
        // degenerate zero-area "fin" edges a boolean can leave in the B-Rep
        // (they produce no triangle, so no feature edge, so no stray spike).
        // Faces on the same analytic cylinder (a wall the kernel splits into arc-
        // faces) share a group id, so their construction seams aren't drawn as edges.
        let surface_group = cylinder_surface_groups(solid);
        let (mut edge_vertices, mut edge_indices, mut edge_face_normals, mut edge_pairs) =
            mesh_feature_edges(&vertices, &indices, &face_ids, &surface_group);
        add_missing_straight_brep_edges(
            solid,
            &vertices,
            &indices,
            &face_ids,
            &surface_group,
            &mut edge_vertices,
            &mut edge_indices,
            &mut edge_face_normals,
            &mut edge_pairs,
        );
        // Chain the per-triangle chord segments back into whole topological edges:
        // two connected segments belong to the same edge when they border the same
        // pair of *surfaces* (`edge_pairs`, canonicalized through `surface_group` so
        // a cylinder split into arc-faces still reads as one). This is what lets the
        // viewport select a fillet arc — or a full circular rim — as one curve.
        let edge_groups = group_edge_segments(&edge_vertices, &edge_indices, Some(&edge_pairs));
        Self {
            vertices,
            indices,
            edge_vertices,
            edge_indices,
            edge_face_normals,
            face_ids,
            edge_groups,
        }
    }
}

// ---------------------------------------------------------------------------
// Public solid builders + boolean operations (used by the parametric evaluator
// to compose join/cut features). Each returns an `openrcad` solid so several
// features can be combined before a single tessellation pass.
// ---------------------------------------------------------------------------

/// Axis-aligned box solid, one corner at the origin, opposite at (w, h, d).
pub fn box_solid(w: f32, h: f32, d: f32) -> KernelSolid {
    make_box(&Pnt::origin(), w as f64, h as f64, d as f64)
}

/// Boolean-ready solid for a cylinder primitive: a **true smooth cylinder**
/// along +Y, base centered at origin.
///
/// OpenRCAD's boolean engine handles smooth cylindrical faces natively — cuts,
/// blind pockets, and (cylinder-as-object) booleans all come back watertight
/// (see the kernel's `repro_cylinder` tests). So the body keeps its smooth
/// analytic wall through a join/cut instead of re-tessellating into the striped
/// 48-gon prism the old "always facet it" rule produced (a workaround for a
/// since-retired truck panic). A 48-gon prism is kept only as a defensive
/// fallback should the native build ever fail.
pub fn cylinder_solid(r: f32, h: f32) -> Option<KernelSolid> {
    build_cylinder_solid(r as f64, h as f64).or_else(|| {
        use crate::geometry::{CoordinateSystem, Vec3};
        let segs = crate::CIRCLE_SEGS;
        let pts: Vec<(f32, f32)> = (0..segs)
            .map(|i| {
                let a = (i as f32 / segs as f32) * std::f32::consts::TAU;
                (r * a.cos(), r * a.sin())
            })
            .collect();
        // Right-handed frame whose normal is +Y (u = Z, v = X ⇒ u × v = +Y),
        // giving the base-at-origin, +Y-axis cylinder the primitive expects.
        let frame = CoordinateSystem::new(Vec3::ZERO, Vec3::Z, Vec3::X);
        build_extrusion_solid(&pts, &[], h as f64, &frame, true)
    })
}

/// Solid for one extruded sketch region. Straight boundary runs sweep to planar
/// laterals; co-circular runs (a drawn circle, a sketch-fillet arc) are rebuilt
/// into true circular-arc edges by [`loop_to_wire`] so they sweep to *smooth*
/// cylindrical walls instead of a fan of facets. Holed profiles try the holed
/// plane first and fall back to the outer boundary alone if the kernel can't
/// attach it.
///
/// OpenRCAD's boolean engine resolves native cylinder cuts/joins/bosses
/// watertight (see the kernel's `repro_cylinder` tests), so feeding it smooth
/// arc walls yields clean round pockets/bosses — not the striped facet result
/// the old "always a prism" rule produced. The parametric assembler still tries
/// the fully-analytic [`circular_cylinder_tool`] first for whole-circle
/// profiles; this is the general fallback for everything else.
pub fn extruded_region_solid(
    points: &[(f32, f32)],
    holes: &[Vec<(f32, f32)>],
    depth: f32,
    cs: &crate::geometry::CoordinateSystem,
) -> Option<KernelSolid> {
    if points.len() < 3 || depth.abs() < f32::EPSILON {
        return None;
    }
    build_extrusion_solid(points, holes, depth as f64, cs, true)
        .or_else(|| build_extrusion_solid(points, &[], depth as f64, cs, true))
}

/// Build the **cutter solid** for a 3D edge fillet/chamfer: subtract it from a
/// body and the sharp edge from `p0` to `p1` (with adjacent outward face normals
/// `n1`, `n2`) becomes a rounded (`fillet`) or beveled corner of size `dist`.
///
/// The cross-section perpendicular to the edge is the corner sliver to remove —
/// a right triangle for a chamfer, or that triangle minus a circular segment
/// (faceted into `segments` chords) for a fillet — swept the length of the edge.
/// It is built by [`extruded_region_solid`] on an edge-aligned frame, so it
/// reuses the same tested, outward-facing prism path the extrude cut uses.
///
/// Two robustness offsets dodge truck's coplanar/tangent-face boolean failures,
/// the same hazards the extrude cut fights (see `directional_cut` / `grow_loop`):
/// * `grow` inflates the whole cross-section outward about its centroid, so the
///   tangent points lift *off* the body faces and the cutter slices through them
///   transversally instead of lying tangent — the configuration truck's solver
///   chokes on. It costs ~`grow`mm of size, so it's used as the fallback cutter.
/// * `end_overshoot` extends the prism past both ends of the edge, so its end
///   caps clear the body's perpendicular faces.
///
/// Returns `None` for a degenerate edge (zero length, `dist <= 0`, or
/// near-parallel face normals that don't define a corner).
#[allow(clippy::too_many_arguments)]
pub fn edge_corner_cutter(
    p0: [f32; 3],
    p1: [f32; 3],
    n1: [f32; 3],
    n2: [f32; 3],
    dist: f32,
    fillet: bool,
    segments: usize,
    grow: f32,
    end_overshoot: f32,
) -> Option<KernelSolid> {
    use crate::geometry::{CoordinateSystem, Vec3};

    if dist <= 1.0e-4 {
        return None;
    }
    let p0 = Vec3::new(p0[0], p0[1], p0[2]);
    let p1 = Vec3::new(p1[0], p1[1], p1[2]);
    let n1 = Vec3::new(n1[0], n1[1], n1[2]).normalize();
    let n2 = Vec3::new(n2[0], n2[1], n2[2]).normalize();

    let edge = p1.sub(p0);
    let len = edge.length();
    if len < 1.0e-4 {
        return None;
    }
    let t = edge.mul(1.0 / len);

    // The two face normals must define a real corner (not the same face).
    if n1.cross(n2).length() < 1.0e-3 {
        return None;
    }

    // Into-body directions along each face: the component of the *other* face's
    // inward normal that lies in this face. For a 90° box corner these reduce to
    // `-n2` and `-n1`.
    let f1 = n2.mul(-1.0).sub(n1.mul(n2.mul(-1.0).dot(n1))).normalize();
    let f2 = n1.mul(-1.0).sub(n2.mul(n1.mul(-1.0).dot(n2))).normalize();
    if f1.length() < 0.5 || f2.length() < 0.5 {
        return None;
    }

    // Edge-aligned frame: u = n1 (already ⊥ t, since the edge lies in face 1),
    // v = t × u, so u × v = t = the sweep normal. Start one overshoot behind p0.
    let u_axis = n1.sub(t.mul(n1.dot(t))).normalize();
    let v_axis = t.cross(u_axis).normalize();
    let origin = p0.sub(t.mul(end_overshoot));
    let cs = CoordinateSystem::new(origin, u_axis, v_axis);

    // 2D cross-section coordinates, taken relative to the corner point p0 (the
    // along-edge offset of `origin` is ⊥ u/v, so it doesn't affect these).
    let proj = |pt: Vec3| -> (f32, f32) {
        let d = pt.sub(p0);
        (d.dot(u_axis), d.dot(v_axis))
    };

    let t1 = p0.add(f1.mul(dist)); // tangent point on face 1
    let t2 = p0.add(f2.mul(dist)); // tangent point on face 2
    let t1_2d = proj(t1);
    let t2_2d = proj(t2);
    let corner_2d = (0.0f32, 0.0f32); // the edge itself, projected

    let mut loop_pts: Vec<(f32, f32)> = Vec::new();
    loop_pts.push(corner_2d);
    loop_pts.push(t1_2d);
    if fillet {
        // Faceted quarter-ish arc from T1 to T2, bulging toward the corner. The
        // centre sits one `dist` off each face (exact for a right-angle corner).
        let center = p0.add(f1.mul(dist)).add(f2.mul(dist));
        let c_2d = proj(center);
        let a0 = (t1_2d.1 - c_2d.1).atan2(t1_2d.0 - c_2d.0);
        let a1 = (t2_2d.1 - c_2d.1).atan2(t2_2d.0 - c_2d.0);
        // Sweep the short way (|Δ| ≤ π) so the arc hugs the corner.
        let mut delta = a1 - a0;
        while delta > std::f32::consts::PI {
            delta -= std::f32::consts::TAU;
        }
        while delta < -std::f32::consts::PI {
            delta += std::f32::consts::TAU;
        }
        let r = ((t1_2d.0 - c_2d.0).powi(2) + (t1_2d.1 - c_2d.1).powi(2)).sqrt();
        // Tessellate to ~3.6°/segment so the round reads smooth, capped by
        // `segments` to keep the boolean cutter's face count (and so truck's
        // solver cost/fragility) bounded.
        let steps = ((delta.abs() / 0.063).ceil() as usize).clamp(6, segments.max(6));
        // Interior arc points only (endpoints are T1/T2, already placed).
        for k in 1..steps {
            let a = a0 + delta * (k as f32 / steps as f32);
            loop_pts.push((c_2d.0 + r * a.cos(), c_2d.1 + r * a.sin()));
        }
    }
    loop_pts.push(t2_2d);

    // Wind CCW as seen from +n (= +t) *first*, so the extrusion builder yields an
    // outward-facing solid the boolean accepts — and so the outward edge-offset
    // below pushes the right way.
    let area: f32 = (0..loop_pts.len())
        .map(|i| {
            let (x0, y0) = loop_pts[i];
            let (x1, y1) = loop_pts[(i + 1) % loop_pts.len()];
            x0 * y1 - x1 * y0
        })
        .sum::<f32>()
        * 0.5;
    if area.abs() < 1.0e-6 {
        return None;
    }
    if area < 0.0 {
        loop_pts.reverse();
    }

    // Fallback robustness: inflate the section outward so the tangent points lift
    // off the body faces (no tangency) and the cutter slices through them
    // transversally — the configuration truck's boolean accepts. This is a proper
    // per-edge polygon offset (each edge slid out by `grow`, new vertices at the
    // intersections of consecutive offset edges), NOT a radial scale about the
    // centroid: the fillet's cross-section is *concave* (the arc bulges toward
    // the corner), and a radial scale collapses/self-intersects the arc vertices
    // that sit near the centroid — which is what made filleted bodies come out
    // garbled while chamfers (a convex triangle) were fine.
    if grow > 1.0e-6 {
        loop_pts = offset_polygon_outward(&loop_pts, grow);
    }

    extruded_region_solid(&loop_pts, &[], len + 2.0 * end_overshoot, &cs)
}

/// Offset a simple **CCW** polygon outward by `grow`, the robust way: slide every
/// edge out along its outward normal, then place each new vertex at the
/// intersection of the two consecutive offset edges. Unlike a radial scale about
/// the centroid this stays valid for concave polygons (e.g. the fillet cutter),
/// so it never folds the arc back over its legs.
fn offset_polygon_outward(pts: &[(f32, f32)], grow: f32) -> Vec<(f32, f32)> {
    let n = pts.len();
    if n < 3 {
        return pts.to_vec();
    }
    // Per edge i (pts[i] → pts[i+1]): a point slid out along the outward normal,
    // plus the (unit) edge direction. For a CCW loop the outward (right-hand)
    // normal of direction (dx, dy) is (dy, -dx).
    let mut off_pt = Vec::with_capacity(n);
    let mut dir = Vec::with_capacity(n);
    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        let (mut dx, mut dy) = (b.0 - a.0, b.1 - a.1);
        let l = (dx * dx + dy * dy).sqrt();
        if l < 1.0e-9 {
            dx = 1.0;
            dy = 0.0;
        } else {
            dx /= l;
            dy /= l;
        }
        off_pt.push((a.0 + dy * grow, a.1 - dx * grow));
        dir.push((dx, dy));
    }
    // Each output vertex i is the intersection of offset edge (i-1) and edge i.
    let intersect = |p1: (f32, f32), d1: (f32, f32), p2: (f32, f32), d2: (f32, f32)| {
        let denom = d1.0 * d2.1 - d1.1 * d2.0;
        if denom.abs() < 1.0e-9 {
            return None; // parallel (a straight run) — caller falls back
        }
        let t = ((p2.0 - p1.0) * d2.1 - (p2.1 - p1.1) * d2.0) / denom;
        Some((p1.0 + t * d1.0, p1.1 + t * d1.1))
    };
    // Sanity bound: when two consecutive edges are *nearly* (but not exactly)
    // parallel, `denom` is tiny-but-finite, so `t` blows up and the miter vertex
    // shoots astronomically far away — a spike that corrupts the cutter and
    // leaves stray vertices (and lines) in the boolean result. Only such
    // degenerate (or non-finite) intersections are rejected; every legitimate
    // miter, even on a sharp corner, stays far inside this bound and is untouched.
    const SPIKE_LIMIT: f32 = 1.0e4; // mm
    (0..n)
        .map(|i| {
            let prev = (i + n - 1) % n;
            match intersect(off_pt[prev], dir[prev], off_pt[i], dir[i]) {
                Some(p)
                    if p.0.is_finite()
                        && p.1.is_finite()
                        && p.0.abs() < SPIKE_LIMIT
                        && p.1.abs() < SPIKE_LIMIT =>
                {
                    p
                }
                _ => off_pt[i],
            }
        })
        .collect()
}

/// Number of segments used to draw a smooth circular wireframe outline.
const CYL_WIRE_SEGS: usize = crate::CIRCLE_SEGS;

/// Recognise a boundary that is (within tolerance) a circle, returning its
/// centre `(u, v)` and radius in sketch-plane coordinates. Requires enough
/// points that it can't be mistaken for a coarse regular polygon the user
/// actually wants faceted, and every point within 2% of the mean radius.
fn circle_profile(points: &[(f32, f32)]) -> Option<(f32, f32, f32)> {
    let n = points.len();
    if n < 12 {
        return None;
    }
    let (mut cx, mut cy) = (0.0f32, 0.0f32);
    for p in points {
        cx += p.0;
        cy += p.1;
    }
    cx /= n as f32;
    cy /= n as f32;
    let dists: Vec<f32> = points
        .iter()
        .map(|p| ((p.0 - cx).powi(2) + (p.1 - cy).powi(2)).sqrt())
        .collect();
    let r = dists.iter().sum::<f32>() / n as f32;
    if r < 1.0e-3 {
        return None;
    }
    dists
        .iter()
        .all(|d| (d - r).abs() <= 0.02 * r)
        .then_some((cx, cy, r))
}

/// A real cylinder: a circular face of radius `r` centred at `(cu, cv)` on the
/// sketch plane, swept `depth` along the plane normal. Uses the native cylinder
/// primitive so the side is a smooth analytic cylindrical surface (not a prism).
fn oriented_cylinder_solid(
    cs: &crate::geometry::CoordinateSystem,
    cu: f32,
    cv: f32,
    r: f32,
    depth: f32,
) -> Option<KernelSolid> {
    if r <= 0.0 || depth.abs() < f32::EPSILON {
        return None;
    }
    let center = cs.unproject(cu, cv);
    // `make_cylinder` builds the wall along +axis from the base; for a negative
    // sweep, base the cylinder at the far (lower) rim and use the positive height.
    let base = if depth >= 0.0 {
        center
    } else {
        center.add(cs.n.mul(depth))
    };
    let axis = Ax2::new(
        Pnt::new(base.x as f64, base.y as f64, base.z as f64),
        Dir::new(cs.n.x as f64, cs.n.y as f64, cs.n.z as f64),
    );
    Some(make_cylinder(&axis, r as f64, depth.abs() as f64))
}

/// The **smooth native-cylinder** boolean tool for a circular, hole-free region:
/// the same swept volume [`extruded_region_solid`] builds as a 48-gon prism, but
/// with a true analytic cylindrical wall. Returns `None` for non-circular or
/// holed profiles (those stay prisms).
///
/// OpenRCAD's boolean engine resolves smooth cylinder cuts, blind pockets and
/// coplanar boss-unions natively and watertight (see the kernel's
/// `repro_cylinder` tests), so feeding the smooth tool — tried *before* the
/// faceted prism fallback in [`crate::parametric`]'s join/cut assembler — yields
/// a clean round hole / boss instead of the striped, sectioned facet result the
/// old "always a prism" rule produced.
pub fn circular_cylinder_tool(
    boundary: &[(f32, f32)],
    holes: &[Vec<(f32, f32)>],
    depth: f32,
    cs: &crate::geometry::CoordinateSystem,
) -> Option<KernelSolid> {
    if !holes.is_empty() || depth.abs() < f32::EPSILON {
        return None;
    }
    let (cu, cv, r) = circle_profile(boundary)?;
    oriented_cylinder_solid(cs, cu, cv, r, depth)
}

/// Run `f` with the panic hook silenced, restoring it afterward. `boolean_checked`
/// already catches the kernel's panics, but the *default* hook still prints the
/// panic (and any diagnostic dump) to stderr — which would spam the console on
/// every degraded boolean (e.g. a drag frame). Silencing it keeps recoverable
/// boolean failures quiet; the caller still just sees `None`.
fn quiet_panic<R>(f: impl FnOnce() -> R) -> R {
    let prev = std::panic::take_hook();
    std::panic::set_hook(Box::new(|_| {}));
    let r = f();
    std::panic::set_hook(prev);
    r
}

/// Boolean union (`a ∪ b`). Returns `None` if the kernel can't resolve the
/// configuration, panics, or produces a non-watertight result — callers decide
/// how to degrade. `boolean_checked` catches panics and rejects leaky output.
pub fn union(a: &KernelSolid, b: &KernelSolid) -> Option<KernelSolid> {
    quiet_panic(|| boolean_checked(a, b, BooleanOp::Fuse).ok())
}

/// Boolean difference (`a − b`): subtract `b`'s volume from `a`. Returns `None`
/// on kernel failure or non-watertight output.
pub fn difference(a: &KernelSolid, b: &KernelSolid) -> Option<KernelSolid> {
    quiet_panic(|| boolean_checked(a, b, BooleanOp::Cut).ok())
}

/// Boolean difference (`a − b`) that returns **one solid per connected
/// component** instead of a single shell.
///
/// A cut that *severs* `a` — e.g. a slot sliced clean through a bar, leaving two
/// separate lumps — comes back from the kernel as one watertight shell holding
/// both disjoint pieces (a valid B-Rep, but really two bodies). `difference`
/// hands that back as a single `KernelSolid`; the parametric evaluator instead
/// wants each lump as its own selectable body part. This runs the same guarded
/// cut, then splits the result into its connected components via
/// [`Solid::split_disconnected`].
///
/// Returns `None` on kernel failure or non-watertight output (the caller keeps
/// the part intact rather than dropping material). On success the vector always
/// has at least one element — an un-severed cut yields a single body.
pub fn difference_bodies(a: &KernelSolid, b: &KernelSolid) -> Option<Vec<KernelSolid>> {
    let result = difference(a, b)?;
    let parts = result.split_disconnected();
    Some(if parts.is_empty() {
        vec![result]
    } else {
        parts
    })
}

/// Fallback for the common "rectangular pocket clean through an axis-aligned
/// block" case when the general boolean engine cannot resolve the coplanar
/// split. The result is rebuilt as one extruded face with a rectangular hole.
pub fn axis_aligned_through_cut(part: &KernelSolid, tool: &KernelSolid) -> Option<KernelSolid> {
    let (plo, phi) = solid_aabb(part)?;
    let (tlo, thi) = solid_aabb(tool)?;
    const EPS: f32 = 0.25;

    for axis in 0..3 {
        if tlo[axis] > plo[axis] + EPS || thi[axis] < phi[axis] - EPS {
            continue;
        }

        let axes: Vec<usize> = (0..3).filter(|&k| k != axis).collect();
        let a = axes[0];
        let b = axes[1];
        let ha0 = tlo[a].max(plo[a]);
        let ha1 = thi[a].min(phi[a]);
        let hb0 = tlo[b].max(plo[b]);
        let hb1 = thi[b].min(phi[b]);

        if ha0 <= plo[a] + EPS
            || ha1 >= phi[a] - EPS
            || hb0 <= plo[b] + EPS
            || hb1 >= phi[b] - EPS
            || ha1 <= ha0 + EPS
            || hb1 <= hb0 + EPS
        {
            continue;
        }

        let outer = vec![
            (0.0, 0.0),
            (phi[a] - plo[a], 0.0),
            (phi[a] - plo[a], phi[b] - plo[b]),
            (0.0, phi[b] - plo[b]),
        ];
        let hole = vec![
            (ha0 - plo[a], hb0 - plo[b]),
            (ha1 - plo[a], hb0 - plo[b]),
            (ha1 - plo[a], hb1 - plo[b]),
            (ha0 - plo[a], hb1 - plo[b]),
        ];

        let origin = Vec3::new(plo[0], plo[1], plo[2]);
        let cs = match axis {
            0 => crate::geometry::CoordinateSystem::new(origin, Vec3::Y, Vec3::Z),
            1 => crate::geometry::CoordinateSystem::new(origin, Vec3::X, Vec3::Z),
            _ => crate::geometry::CoordinateSystem::new(origin, Vec3::X, Vec3::Y),
        };

        return build_extrusion_solid(&outer, &[hole], (phi[axis] - plo[axis]) as f64, &cs, false);
    }

    None
}

/// Fallback for an axis-aligned cut when the exact boolean fails. It approximates
/// the removed volume by the tool AABB and decomposes `part - tool` into up to
/// six non-overlapping boxes, all kept as parts of the same ZeroCAD body.
pub fn axis_aligned_cut_parts(part: &KernelSolid, tool: &KernelSolid) -> Option<Vec<KernelSolid>> {
    let (plo, phi) = solid_aabb(part)?;
    let (tlo, thi) = solid_aabb(tool)?;
    const EPS: f32 = 0.01;

    let rlo = [tlo[0].max(plo[0]), tlo[1].max(plo[1]), tlo[2].max(plo[2])];
    let rhi = [thi[0].min(phi[0]), thi[1].min(phi[1]), thi[2].min(phi[2])];
    if (0..3).any(|k| rhi[k] <= rlo[k] + EPS) {
        return None;
    }

    let mut pieces = Vec::new();
    let mut push_box = |lo: [f32; 3], hi: [f32; 3]| {
        let d = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
        if d.iter().all(|&v| v > EPS) {
            pieces.push(make_box(
                &Pnt::new(lo[0] as f64, lo[1] as f64, lo[2] as f64),
                d[0] as f64,
                d[1] as f64,
                d[2] as f64,
            ));
        }
    };

    push_box(plo, [rlo[0], phi[1], phi[2]]);
    push_box([rhi[0], plo[1], plo[2]], phi);

    let xlo = rlo[0];
    let xhi = rhi[0];
    push_box([xlo, plo[1], plo[2]], [xhi, rlo[1], phi[2]]);
    push_box([xlo, rhi[1], plo[2]], [xhi, phi[1], phi[2]]);

    let ylo = rlo[1];
    let yhi = rhi[1];
    push_box([xlo, ylo, plo[2]], [xhi, yhi, rlo[2]]);
    push_box([xlo, ylo, rhi[2]], [xhi, yhi, phi[2]]);

    (!pieces.is_empty()).then_some(pieces)
}

/// Round the edge running from `p0` to `p1` of `solid` by `radius`, using the
/// native rolling-ball blend (no booleans). The edge is located in the solid's
/// topology by matching its endpoints, so `p0`/`p1` are the world-space edge
/// endpoints captured in an [`crate::parametric::EdgeRef`]. Returns `Err` with a
/// human-readable reason when the edge isn't found, isn't a blendable corner, or
/// the blend fails — the caller surfaces it to the user instead of a generic
/// "couldn't be rounded".
pub fn fillet_edge(
    solid: &KernelSolid,
    p0: [f32; 3],
    p1: [f32; 3],
    radius: f32,
) -> Result<KernelSolid, String> {
    let a = Pnt::new(p0[0] as f64, p0[1] as f64, p0[2] as f64);
    let b = Pnt::new(p1[0] as f64, p1[1] as f64, p1[2] as f64);
    let e = Edge::between_points(a, b);
    fillet_edges(solid, std::slice::from_ref(&e), radius as f64).map_err(|err| err.to_string())
}

/// Bevel the edge running from `p0` to `p1` of `solid` by `distance`, using the
/// native selected-edge chamfer path (no boolean cutter fallback).
pub fn chamfer_edge(
    solid: &KernelSolid,
    p0: [f32; 3],
    p1: [f32; 3],
    distance: f32,
) -> Result<KernelSolid, String> {
    let a = Pnt::new(p0[0] as f64, p0[1] as f64, p0[2] as f64);
    let b = Pnt::new(p1[0] as f64, p1[1] as f64, p1[2] as f64);
    let e = Edge::between_points(a, b);
    chamfer_edges(solid, std::slice::from_ref(&e), distance as f64).map_err(|err| err.to_string())
}

/// Axis-aligned bounding box of a solid from its B-Rep vertices, as
/// `(min, max)`. Exact for polygonal solids; a conservative-enough estimate for
/// curved ones (used only for cheap overlap pre-tests). `None` if vertexless.
pub fn solid_aabb(solid: &KernelSolid) -> Option<([f32; 3], [f32; 3])> {
    let (lo, hi) = solid.bounding_box().corners()?;
    Some((
        [lo.x() as f32, lo.y() as f32, lo.z() as f32],
        [hi.x() as f32, hi.y() as f32, hi.z() as f32],
    ))
}

/// Whether two AABBs overlap (or touch within `eps`). Used to skip boolean
/// attempts between solids that can't possibly interact.
pub fn aabbs_overlap(a: &([f32; 3], [f32; 3]), b: &([f32; 3], [f32; 3]), eps: f32) -> bool {
    (0..3).all(|k| a.0[k] - eps <= b.1[k] && b.0[k] - eps <= a.1[k])
}

/// Whether `outer` fully encloses `inner` (allowing `eps` slack on every side).
/// A boolean union `a ∪ b` must contain `a`, so its AABB must contain `a`'s —
/// used as a cheap sanity check to reject a degenerate union that would
/// otherwise silently delete the body it merged into.
pub fn aabb_contains(outer: &([f32; 3], [f32; 3]), inner: &([f32; 3], [f32; 3]), eps: f32) -> bool {
    (0..3).all(|k| outer.0[k] - eps <= inner.0[k] && inner.1[k] <= outer.1[k] + eps)
}

// ---------------------------------------------------------------------------
// openrcad Solid builders
// ---------------------------------------------------------------------------

fn build_cylinder_solid(r: f64, h: f64) -> Option<KernelSolid> {
    if r <= 0.0 || h <= 0.0 {
        return None;
    }
    // Base centered at the origin, swept along +Y — the axis the primitive
    // display path (`MockMesh::make_cylinder`) and its wireframe expect.
    Some(make_cylinder(&Ax2::new(Pnt::origin(), Dir::dy()), r, h))
}

// ---------------------------------------------------------------------------
// Arc reconstruction for extruded sketch profiles
//
// Sketch region boundaries arrive as dense polylines: every circle/arc the user
// drew (and every sketch-fillet) was flattened to line segments by region
// detection, which keeps only points (no curve provenance). Extruding those
// straight edges produces a fan of planar laterals — a "segmented" cylinder that
// shades as facets and litters the wireframe with vertical struts.
//
// `loop_to_wire` rebuilds the true geometry before the sweep: it finds maximal
// runs of consecutive boundary vertices that lie on a common circle and emits a
// single circular-arc `Edge` per run (split into <=135° pieces, matching the
// kernel's cylinder convention). `prism` then makes those arc edges into smooth
// cylindrical walls. Polygons have no co-circular run, so their corners stay
// sharp line edges — a rectangle still extrudes to a clean box.
// ---------------------------------------------------------------------------

/// Maximum central angle (radians) for a single reconstructed arc edge. Longer
/// arcs are split into equal pieces — mirroring `make_cylinder`'s 3×120° wall so
/// each cylindrical lateral face stays a comfortable sub-half-circle the
/// tessellator handles cleanly.
const MAX_ARC_PIECE: f64 = 0.75 * std::f64::consts::PI; // 135°

/// Per-vertex turn beyond which a boundary vertex is a true corner, not a point
/// on a smooth arc. A `CIRCLE_SEGS`-faceted circle turns ~7.5°/vertex and a
/// sketch fillet arc <=3.6°; an octagon turns 45° and a hexagon 60°. ~23°
/// cleanly separates real arcs from polygons the user wants left faceted.
const ARC_MAX_TURN: f64 = 0.40;

/// Minimum count of consecutive co-circular interior vertices for a run to count
/// as an arc, so a stray pair of points can't masquerade as one.
const ARC_MIN_RUN: usize = 3;

/// Circumcircle `(cx, cy, r)` of three 2D points, or `None` when (near) collinear.
fn circumcircle_2d(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> Option<(f64, f64, f64)> {
    let d = 2.0 * (a.0 * (b.1 - c.1) + b.0 * (c.1 - a.1) + c.0 * (a.1 - b.1));
    if d.abs() < 1e-12 {
        return None;
    }
    let a2 = a.0 * a.0 + a.1 * a.1;
    let b2 = b.0 * b.0 + b.1 * b.1;
    let c2 = c.0 * c.0 + c.1 * c.1;
    let ux = (a2 * (b.1 - c.1) + b2 * (c.1 - a.1) + c2 * (a.1 - b.1)) / d;
    let uy = (a2 * (c.0 - b.0) + b2 * (a.0 - c.0) + c2 * (b.0 - a.0)) / d;
    let r = ((ux - a.0).powi(2) + (uy - a.1).powi(2)).sqrt();
    Some((ux, uy, r))
}

/// Exterior turn angle (radians) at `cur` between the incoming and outgoing
/// segments. ~0 on a straight run, large at a polygon corner.
fn turn_angle_2d(prev: (f64, f64), cur: (f64, f64), next: (f64, f64)) -> f64 {
    let v1 = (cur.0 - prev.0, cur.1 - prev.1);
    let v2 = (next.0 - cur.0, next.1 - cur.1);
    let l1 = v1.0.hypot(v1.1);
    let l2 = v2.0.hypot(v2.1);
    if l1 < 1e-12 || l2 < 1e-12 {
        return 0.0;
    }
    (((v1.0 * v2.0 + v1.1 * v2.1) / (l1 * l2)).clamp(-1.0, 1.0)).acos()
}

/// Whether two per-vertex circumcircles describe the same underlying circle
/// (so the vertices between them lie on one arc).
fn same_circle(a: (f64, f64, f64), b: (f64, f64, f64)) -> bool {
    let dc = (a.0 - b.0).hypot(a.1 - b.1);
    let r = a.2.max(b.2);
    dc <= 0.05 * r + 1e-2 && (a.2 - b.2).abs() <= 0.07 * r + 1e-2
}

/// Build a wire from a closed 2D boundary loop (`cs`-plane coordinates),
/// reconstructing circular arcs from co-circular runs so an extruded sketch arc
/// becomes one smooth cylindrical wall. Returns `None` for fewer than 3 distinct
/// points.
fn loop_to_wire(loop_pts: &[(f32, f32)], cs: &crate::geometry::CoordinateSystem) -> Option<Wire> {
    // Dedup coincident consecutive (and wrap-around) points in 2D.
    let mut pts: Vec<(f64, f64)> = Vec::with_capacity(loop_pts.len());
    for &(u, v) in loop_pts {
        let p = (u as f64, v as f64);
        if pts
            .last()
            .is_none_or(|q: &(f64, f64)| (q.0 - p.0).hypot(q.1 - p.1) > 1e-7)
        {
            pts.push(p);
        }
    }
    if pts.len() >= 2 {
        let (f, l) = (pts[0], *pts.last().unwrap());
        if (f.0 - l.0).hypot(f.1 - l.1) <= 1e-7 {
            pts.pop();
        }
    }
    let n = pts.len();
    if n < 3 {
        return None;
    }

    let to_pnt = |p: (f64, f64)| -> Pnt {
        let q = cs.unproject(p.0 as f32, p.1 as f32);
        Pnt::new(q.x as f64, q.y as f64, q.z as f64)
    };
    let axis_n = Dir::new(cs.n.x as f64, cs.n.y as f64, cs.n.z as f64);
    // Map a 2D plane offset (du, dv) to its 3D direction vector.
    let dir3d = |d: (f64, f64)| -> GeomVec {
        GeomVec::new(
            cs.u.x as f64 * d.0 + cs.v.x as f64 * d.1,
            cs.u.y as f64 * d.0 + cs.v.y as f64 * d.1,
            cs.u.z as f64 * d.0 + cs.v.z as f64 * d.1,
        )
    };

    // Per-vertex co-circularity: the circle through (prev, cur, next), gated so a
    // sharp polygon corner classifies as a non-arc vertex (a run separator).
    let arc_vert: Vec<Option<(f64, f64, f64)>> = (0..n)
        .map(|i| {
            let prev = pts[(i + n - 1) % n];
            let cur = pts[i];
            let next = pts[(i + 1) % n];
            if turn_angle_2d(prev, cur, next) > ARC_MAX_TURN {
                return None;
            }
            circumcircle_2d(prev, cur, next)
        })
        .collect();

    let mut edges: Vec<Edge> = Vec::new();

    // Append the arc covering rotated vertices [cs_v ..= ce_v] (ce_v may equal n,
    // i.e. wrap to vertex 0) to `edges`, split into <=MAX_ARC_PIECE sub-arcs.
    let emit_arc = |edges: &mut Vec<Edge>,
                    rp: &[(f64, f64)],
                    cs_v: usize,
                    ce_v: usize,
                    circ: (f64, f64, f64)| {
        let m = rp.len();
        let (cx, cy, r) = circ;
        let start2d = rp[cs_v % m];
        let end2d = rp[ce_v % m];

        // Sense: signed area of the covered chain about the centre picks which
        // way `Circle::point(t)` (CCW about +main) must turn to trace it.
        let mut signed = 0.0;
        for k in cs_v..ce_v {
            let p = rp[k % m];
            let q = rp[(k + 1) % m];
            signed += (p.0 - cx) * (q.1 - cy) - (p.1 - cy) * (q.0 - cx);
        }
        let main = if signed >= 0.0 {
            axis_n
        } else {
            axis_n.reversed()
        };

        let center3d = to_pnt((cx, cy));
        let xd = dir3d((start2d.0 - cx, start2d.1 - cy));
        let mv = GeomVec::from_dir(main);
        let xperp = xd - mv * xd.dot(&mv);
        let Some(xdir) = Dir::from_vec(&xperp) else {
            // Degenerate (start coincident with centre): fall back to chords.
            for k in cs_v..ce_v {
                edges.push(Edge::between_points(
                    to_pnt(rp[k % m]),
                    to_pnt(rp[(k + 1) % m]),
                ));
            }
            return;
        };
        let circle = Circle::new(Ax3::new_axes(center3d, main, xdir), r);
        let ydir = mv.cross(&GeomVec::from_dir(xdir));
        let ang = |p2d: (f64, f64)| -> f64 {
            let w = dir3d((p2d.0 - cx, p2d.1 - cy));
            w.dot(&ydir).atan2(w.dot(&GeomVec::from_dir(xdir)))
        };
        let t0 = ang(start2d);
        let mut t1 = ang(end2d);
        while t1 <= t0 + 1e-9 {
            t1 += 2.0 * std::f64::consts::PI;
        }
        let span = t1 - t0;
        let pieces = ((span / MAX_ARC_PIECE).ceil() as usize).max(1);
        let mut prev_v = Vertex::new(to_pnt(start2d));
        for k in 1..=pieces {
            let ts = t0 + span * ((k - 1) as f64 / pieces as f64);
            let te = t0 + span * (k as f64 / pieces as f64);
            let end_v = if k == pieces {
                Vertex::new(to_pnt(end2d))
            } else {
                Vertex::new(circle.point(te))
            };
            edges.push(Edge::new(
                Some(GeomCurve::circle(circle)),
                ts,
                te,
                prev_v.clone(),
                end_v.clone(),
            ));
            prev_v = end_v;
        }
    };

    // Pick a rotation start at a non-arc (corner) vertex so arc runs never wrap
    // the array boundary. With no corner the loop is a full circle (or an
    // ellipse, whose osculating circles never agree).
    match arc_vert.iter().position(|a| a.is_none()) {
        Some(off) => {
            let rp: Vec<(f64, f64)> = (0..n).map(|i| pts[(i + off) % n]).collect();
            let ra: Vec<Option<(f64, f64, f64)>> =
                (0..n).map(|i| arc_vert[(i + off) % n]).collect();

            // Collect arc runs (covered vertex ranges) over the interior 1..n.
            let mut runs: Vec<(usize, usize, (f64, f64, f64))> = Vec::new();
            let mut i = 1;
            while i < n {
                let Some(ci) = ra[i] else {
                    i += 1;
                    continue;
                };
                let mut j = i;
                while j < n - 1 {
                    match (ra[j], ra[j + 1]) {
                        (Some(a), Some(b)) if same_circle(a, b) => j += 1,
                        _ => break,
                    }
                }
                if j - i + 1 >= ARC_MIN_RUN {
                    let (cs_v, ce_v) = (i - 1, j + 1);
                    let mid = (cs_v + ce_v) / 2;
                    let circ =
                        circumcircle_2d(rp[cs_v % n], rp[mid % n], rp[ce_v % n]).unwrap_or(ci);
                    runs.push((cs_v, ce_v, circ));
                }
                i = j + 1;
            }

            // Walk the loop, emitting arcs for runs and line edges between them.
            let mut cur = 0usize;
            let mut ri = 0usize;
            while cur < n {
                if ri < runs.len() && runs[ri].0 == cur {
                    let (a, b, circ) = runs[ri];
                    emit_arc(&mut edges, &rp, a, b, circ);
                    cur = b;
                    ri += 1;
                } else {
                    edges.push(Edge::between_points(
                        to_pnt(rp[cur % n]),
                        to_pnt(rp[(cur + 1) % n]),
                    ));
                    cur += 1;
                }
            }
        }
        None => {
            // No corners. A full circle has one shared circle; an ellipse does not.
            let avg = {
                let (mut sx, mut sy, mut sr) = (0.0, 0.0, 0.0);
                for c in arc_vert.iter().flatten() {
                    sx += c.0;
                    sy += c.1;
                    sr += c.2;
                }
                (sx / n as f64, sy / n as f64, sr / n as f64)
            };
            let is_circle = arc_vert.iter().flatten().all(|c| same_circle(*c, avg));
            if is_circle {
                let circ = circumcircle_2d(pts[0], pts[n / 3], pts[2 * n / 3]).unwrap_or(avg);
                emit_arc(&mut edges, &pts, 0, n, circ);
            } else {
                for i in 0..n {
                    edges.push(Edge::between_points(
                        to_pnt(pts[i]),
                        to_pnt(pts[(i + 1) % n]),
                    ));
                }
            }
        }
    }

    (edges.len() >= 2).then(|| Wire::from_edges(edges))
}

/// Build a wire directly from the region's sampled polyline. This is the stable
/// path for visible sketch-extrude meshes: cap tessellation sees only straight
/// edges, so mixed rectangle/circle regions cannot produce analytic-arc cap
/// spikes in the preview render.
fn loop_to_polyline_wire(
    loop_pts: &[(f32, f32)],
    cs: &crate::geometry::CoordinateSystem,
) -> Option<Wire> {
    if loop_pts.len() < 3 {
        return None;
    }
    let pts: Vec<Pnt> = loop_pts
        .iter()
        .map(|&(u, v)| {
            let p = cs.unproject(u, v);
            Pnt::new(p.x as f64, p.y as f64, p.z as f64)
        })
        .collect();
    let n = pts.len();
    let edges: Vec<Edge> = (0..n)
        .map(|i| Edge::between_points(pts[i], pts[(i + 1) % n]))
        .collect();
    Some(Wire::from_edges(edges))
}

fn build_extrusion_solid(
    points: &[(f32, f32)],
    holes: &[Vec<(f32, f32)>],
    depth: f64,
    cs: &crate::geometry::CoordinateSystem,
    reconstruct_arcs: bool,
) -> Option<KernelSolid> {
    if points.len() < 3 || depth.abs() < f64::EPSILON {
        return None;
    }

    let to_pnt = |u: f32, v: f32| -> Pnt {
        let p = cs.unproject(u, v);
        Pnt::new(p.x as f64, p.y as f64, p.z as f64)
    };
    // Boolean solids can reconstruct circular arcs so swept circle fragments
    // become smooth cylindrical walls. Visible sketch-extrude meshes keep the
    // sampled polyline: OpenRCAD's cap tessellator is more reliable on compound
    // circle/line regions when the cap boundary contains only straight chords.
    let make_wire = |loop_pts: &[(f32, f32)]| -> Option<Wire> {
        if reconstruct_arcs {
            loop_to_wire(loop_pts, cs)
        } else {
            loop_to_polyline_wire(loop_pts, cs)
        }
    };

    // A planar face on the sketch frame: outer boundary plus holes as inner
    // wires. `prism` orients its caps from the face's declared plane normal vs.
    // the sweep direction, so that normal must agree with the outer loop's actual
    // winding — otherwise the shell comes out with mixed (some inward) face
    // normals. ZeroCAD's XZ/YZ sketch frames are left-handed (u × v = −n), so a
    // CCW-in-(u,v) loop there faces −cs.n, not +cs.n. Derive the plane normal
    // straight from the 3D winding (Newell's method) so it is always consistent,
    // for any frame handedness; the sweep still runs along cs.n·depth, and
    // `prism` reconciles the sign.
    let outer = make_wire(points)?;
    let inners: Vec<Wire> = holes.iter().filter_map(|h| make_wire(h)).collect();
    let pts3: Vec<Pnt> = points.iter().map(|(u, v)| to_pnt(*u, *v)).collect();
    let normal = newell_normal(&pts3)?;
    let plane = GeomSurface::plane(Plane::from_point_normal(pts3[0], normal));
    let face = if inners.is_empty() {
        Face::new(Some(plane), outer)
    } else {
        Face::with_wires(Some(plane), Some(outer), inners, Orientation::Forward)
    };
    let sweep = GeomVec::new(
        cs.n.x as f64 * depth,
        cs.n.y as f64 * depth,
        cs.n.z as f64 * depth,
    );
    prism(&face, sweep).ok()
}

/// Unit normal of a planar 3D loop via Newell's method — robust to the loop's
/// winding and to which axis it spans. `None` for a degenerate (collinear or
/// zero-area) loop.
fn newell_normal(pts: &[Pnt]) -> Option<Dir> {
    let n = pts.len();
    let (mut nx, mut ny, mut nz) = (0.0f64, 0.0, 0.0);
    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        nx += (a.y() - b.y()) * (a.z() + b.z());
        ny += (a.z() - b.z()) * (a.x() + b.x());
        nz += (a.x() - b.x()) * (a.y() + b.y());
    }
    let len = (nx * nx + ny * ny + nz * nz).sqrt();
    (len > 1e-12).then(|| Dir::new(nx / len, ny / len, nz / len))
}

// ---------------------------------------------------------------------------
// Tessellation → flat interleaved vertex buffer
// ---------------------------------------------------------------------------

fn solid_to_flat_mesh(
    solid: &KernelSolid,
    correct_boolean_bevels: bool,
    correct_mixed_triangle_normals: bool,
) -> (Vec<f32>, Vec<u32>, Vec<u32>) {
    // `gpu_mesh` unwelds each triangle into three vertices carrying that
    // triangle's flat face normal, plus a per-triangle source-face id — exactly
    // the interleaved layout (minus the f32 normal smoothing) we want. Each
    // vertex copy belongs to a single triangle, so the per-vertex→face mapping
    // `smooth_vertex_normals` relies on holds.
    let mesh = tessellate(solid, TESS_TOL, TESS_ANGLE);
    let gpu = mesh.gpu_mesh();

    let vcount = gpu.positions.len() / 3;
    let mut vertices: Vec<f32> = Vec::with_capacity(vcount * 6);
    for i in 0..vcount {
        let p = i * 3;
        vertices.extend_from_slice(&[
            gpu.positions[p],
            gpu.positions[p + 1],
            gpu.positions[p + 2],
            gpu.normals[p],
            gpu.normals[p + 1],
            gpu.normals[p + 2],
        ]);
    }
    let mut indices = gpu.indices;
    let face_ids = gpu.face_ids;

    // Normalize the shell to outward-facing normals.
    if correct_boolean_bevels {
        // Boolean / fillet results: `sew` aligns the loop *winding* across faces
        // but the stored plane normals can still diverge, so the shell arrives
        // with a MIX of inward/outward faces (a union drops a face, a cut's hole
        // walls turn invisible). A whole-shell or centroid test can't fix a mix —
        // and a centroid test is plain wrong for a hole wall (which legitimately
        // faces the centroid). Orient robustly by triangle adjacency + signed
        // volume instead, then recompute flat normals from the corrected winding.
        orient_mesh_outward(&mut vertices, &mut indices);
    } else {
        // Analytic primitives / sketch prisms: a per-triangle centroid repair
        // (the inverted cap is local and the geometry is convex enough).
        enforce_outward_normals(&mut vertices, &indices, correct_mixed_triangle_normals);
    }

    // Smooth the normals across shallow creases so a curved surface — an
    // analytic fillet cylinder, or a boolean'd / many-sided extruded cylinder
    // wall — shades as ONE smooth face. Sharp features (90° box corners, 45°
    // chamfers) meet past the crease angle and keep distinct normals, so they
    // stay crisp. Crucially this is *face-aware*: a genuinely flat B-rep face is
    // anchored, so its normal survives unbent right up to a tangent fillet line
    // (a fillet is tangent to its neighbours, so plain crease smoothing would
    // otherwise drag the flat face's edge normals into the round and shade the
    // flat face as a slope). Pairs with the renderer's Gouraud (per-vertex)
    // shading and `mesh_feature_edges`' matching crease filter, which hides the
    // facet-boundary lines.
    apply_analytic_cylinder_normals(solid, &mut vertices, &indices, &face_ids);
    smooth_vertex_normals(&mut vertices, &indices, &face_ids, SHADE_CREASE_COS);

    (vertices, indices, face_ids)
}

/// `cos` of the crease angle (~30°) below which adjacent faces are treated as one
/// smooth surface for shading. Above it (chamfer bevels at 45°, box corners at
/// 90°) the crease is a real edge and the faces keep independent normals.
const SHADE_CREASE_COS: f32 = 0.866;

/// Replace each *curved-face* vertex normal with the average of the normals of
/// all vertices sharing its position whose normal lies within the crease angle
/// (`crease_cos`). This is per-vertex normal smoothing with a crease threshold:
/// a fillet cylinder's tessellation normals (a few degrees apart) blend into a
/// smooth gradient, while a sharp edge — whose two faces' normals diverge past
/// the threshold — keeps each face's own normal, so it still reads as an edge.
///
/// It is **face-aware**: a genuinely flat B-rep face (all its triangles share
/// one normal) is *anchored* — its vertices keep their exact face normal even
/// where they sit on a tangent fillet line. Without this anchor, a fillet (which
/// is tangent to its neighbour faces) would bleed its curving normals into the
/// flat face along that line and shade the flat face as a slope. The round's own
/// vertices are still free to average toward the flat normal there, so the
/// junction stays smooth from the fillet side while the flat face stays flat.
///
/// Operates on the interleaved `[x,y,z,nx,ny,nz]` buffer in place; `face_ids`
/// gives the B-rep face of each triangle in `indices`.
fn smooth_vertex_normals(vertices: &mut [f32], indices: &[u32], face_ids: &[u32], crease_cos: f32) {
    let vcount = vertices.len() / 6;
    if vcount == 0 {
        return;
    }
    // Weld vertices by quantized position (1e-4 mm, as elsewhere).
    let key = |i: usize| -> (i64, i64, i64) {
        let b = i * 6;
        let q = |v: f32| (v as f64 * 10_000.0).round() as i64;
        (q(vertices[b]), q(vertices[b + 1]), q(vertices[b + 2]))
    };
    let nrm = |i: usize| -> [f32; 3] {
        let b = i * 6;
        [vertices[b + 3], vertices[b + 4], vertices[b + 5]]
    };

    // Map each vertex to its B-rep face (a vertex copy is only referenced by
    // triangles of the one face that appended it — see `solid_to_flat_mesh`),
    // then collect each face's vertices.
    let mut vert_face: Vec<Option<u32>> = vec![None; vcount];
    for (t, tri) in indices.chunks_exact(3).enumerate() {
        let fid = face_ids.get(t).copied().unwrap_or(0);
        for &vi in tri {
            vert_face[vi as usize] = Some(fid);
        }
    }
    let mut face_verts: HashMap<u32, Vec<usize>> = HashMap::new();
    for (i, f) in vert_face.iter().enumerate() {
        if let Some(f) = f {
            face_verts.entry(*f).or_default().push(i);
        }
    }
    // A face is flat when all its vertices' normals agree (within the crease
    // angle of the face's first normal). Such faces are anchored: a flat design
    // face stays flat; a faceted-fallback fillet's individual flat facets also
    // anchor (so that path keeps its old per-facet look), while a true analytic
    // fillet cylinder — whose normals genuinely vary — is left smoothable.
    let mut flat_face: HashMap<u32, bool> = HashMap::new();
    for (f, verts) in &face_verts {
        let n0 = nrm(verts[0]);
        let flat = verts.iter().all(|&i| {
            let n = nrm(i);
            (n0[0] * n[0] + n0[1] * n[1] + n0[2] * n[2]) >= crease_cos
        });
        flat_face.insert(*f, flat);
    }

    let mut groups: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
    for i in 0..vcount {
        groups.entry(key(i)).or_default().push(i);
    }
    let mut smoothed = vec![[0.0f32; 3]; vcount];
    for members in groups.values() {
        for &i in members {
            let ni = nrm(i);
            // Anchor: a vertex on a flat face keeps its exact normal.
            let anchored = vert_face[i]
                .and_then(|f| flat_face.get(&f).copied())
                .unwrap_or(false);
            if anchored {
                smoothed[i] = ni;
                continue;
            }
            let (mut sx, mut sy, mut sz) = (0.0f32, 0.0f32, 0.0f32);
            for &j in members {
                let nj = nrm(j);
                let same_face = vert_face[i].is_some() && vert_face[i] == vert_face[j];
                let required_dot = if same_face { crease_cos } else { 0.995 };
                if ni[0] * nj[0] + ni[1] * nj[1] + ni[2] * nj[2] >= required_dot {
                    sx += nj[0];
                    sy += nj[1];
                    sz += nj[2];
                }
            }
            let len = (sx * sx + sy * sy + sz * sz).sqrt();
            smoothed[i] = if len > 1.0e-6 {
                [sx / len, sy / len, sz / len]
            } else {
                ni
            };
        }
    }
    for (i, n) in smoothed.iter().enumerate() {
        let b = i * 6;
        vertices[b + 3] = n[0];
        vertices[b + 4] = n[1];
        vertices[b + 5] = n[2];
    }
}

/// Flip triangle normals in `vertices` (interleaved pos+normal) when they point
/// inward, judged against the direction from the mesh centroid to that triangle.
/// Robustly orient a tessellated **closed** mesh so every triangle winds (and is
/// normalled) outward — correct even for non-convex / holed solids where a
/// centroid test fails. Boolean results arrive orientation-inconsistent (`sew`
/// aligns winding but stored plane normals diverge), which a whole-shell flip
/// can't repair: it leaves a union missing a face and a cut's hole walls
/// back-facing (so the hole — and the whole cut — looks like it never happened).
///
/// Two passes: (1) flood-fill the triangles, flipping any neighbour that
/// traverses a shared edge the *same* way (a consistently-oriented manifold
/// traverses every shared edge in opposite directions); (2) flip the entire
/// shell if its signed volume came out negative (inside-out). Finally each
/// triangle's flat normal is recomputed from its corrected winding so normal and
/// winding always agree. `gpu_mesh` unwelds every triangle (3 private vertices),
/// so flipping one never disturbs a neighbour.
fn orient_mesh_outward(vertices: &mut [f32], indices: &mut [u32]) {
    let tris = indices.len() / 3;
    if tris == 0 {
        return;
    }
    let q = |v: f32| (v as f64 * 10_000.0).round() as i64;
    let vkey = |vi: u32| -> (i64, i64, i64) {
        let b = vi as usize * 6;
        (q(vertices[b]), q(vertices[b + 1]), q(vertices[b + 2]))
    };
    // Per-triangle vertex keys in stored winding order.
    let tkeys: Vec<[(i64, i64, i64); 3]> = (0..tris)
        .map(|t| {
            [
                vkey(indices[t * 3]),
                vkey(indices[t * 3 + 1]),
                vkey(indices[t * 3 + 2]),
            ]
        })
        .collect();
    // Undirected edge -> incident triangles.
    let mut edge_tris: HashMap<((i64, i64, i64), (i64, i64, i64)), Vec<usize>> = HashMap::new();
    for (t, k) in tkeys.iter().enumerate() {
        for ei in 0..3 {
            let (a, b) = (k[ei], k[(ei + 1) % 3]);
            edge_tris
                .entry(if a <= b { (a, b) } else { (b, a) })
                .or_default()
                .push(t);
        }
    }
    // The three directed edges of a triangle given its current flip state.
    let directed = |t: usize, flip: bool| -> [((i64, i64, i64), (i64, i64, i64)); 3] {
        let k = &tkeys[t];
        let o = if flip { [0usize, 2, 1] } else { [0, 1, 2] };
        let v = [k[o[0]], k[o[1]], k[o[2]]];
        [(v[0], v[1]), (v[1], v[2]), (v[2], v[0])]
    };
    let mut flipped = vec![false; tris];
    let mut visited = vec![false; tris];
    // Component id per triangle. A join can leave a body as several disjoint
    // watertight shells (a box with a smooth-cylinder boss fused on top often
    // tessellates as two components that only touch). Winding consistency
    // propagates only WITHIN a component, so the inside/outside sign must be
    // decided per component — a single global flip would orient the larger shell
    // correctly and leave the smaller one inside-out (its faces then back-face
    // cull and "disappear" on screen).
    let mut comp = vec![usize::MAX; tris];
    let mut ncomp = 0usize;
    for seed in 0..tris {
        if visited[seed] {
            continue;
        }
        visited[seed] = true;
        comp[seed] = ncomp;
        let mut queue = std::collections::VecDeque::from([seed]);
        while let Some(t) = queue.pop_front() {
            for &(ta, tb) in &directed(t, flipped[t]) {
                let u = if ta <= tb { (ta, tb) } else { (tb, ta) };
                let Some(adj) = edge_tris.get(&u) else {
                    continue;
                };
                for &t2 in adj {
                    if t2 == t || visited[t2] {
                        continue;
                    }
                    // Consistent ⇔ t2 traverses this edge OPPOSITE to t. If t2
                    // (unflipped) traverses it the SAME way (ta→tb), flip it.
                    flipped[t2] = directed(t2, false).contains(&(ta, tb));
                    visited[t2] = true;
                    comp[t2] = ncomp;
                    queue.push_back(t2);
                }
            }
        }
        ncomp += 1;
    }
    // Signed volume (×6) per component with flips applied; negative ⇒ that shell is
    // inside-out and its whole component must be flipped.
    let pos = |vi: u32| -> [f64; 3] {
        let b = vi as usize * 6;
        [
            vertices[b] as f64,
            vertices[b + 1] as f64,
            vertices[b + 2] as f64,
        ]
    };
    let mut comp_vol = vec![0.0f64; ncomp];
    for t in 0..tris {
        let (i0, i1, i2) = if flipped[t] {
            (indices[t * 3], indices[t * 3 + 2], indices[t * 3 + 1])
        } else {
            (indices[t * 3], indices[t * 3 + 1], indices[t * 3 + 2])
        };
        let (a, b, c) = (pos(i0), pos(i1), pos(i2));
        comp_vol[comp[t]] += a[0] * (b[1] * c[2] - b[2] * c[1])
            - a[1] * (b[0] * c[2] - b[2] * c[0])
            + a[2] * (b[0] * c[1] - b[1] * c[0]);
    }
    // Apply winding flips, then recompute each triangle's flat normal so the
    // normal agrees with the corrected winding (discarding any inconsistent
    // stored normal). Smoothing later blends genuinely-curved faces.
    for t in 0..tris {
        if flipped[t] ^ (comp_vol[comp[t]] < 0.0) {
            indices.swap(t * 3 + 1, t * 3 + 2);
        }
        let (i0, i1, i2) = (
            indices[t * 3] as usize,
            indices[t * 3 + 1] as usize,
            indices[t * 3 + 2] as usize,
        );
        let p = |i: usize| [vertices[i * 6], vertices[i * 6 + 1], vertices[i * 6 + 2]];
        let (a, b, c) = (p(i0), p(i1), p(i2));
        let e1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let e2 = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let n = [
            e1[1] * e2[2] - e1[2] * e2[1],
            e1[2] * e2[0] - e1[0] * e2[2],
            e1[0] * e2[1] - e1[1] * e2[0],
        ];
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        let nn = if len > 1.0e-12 {
            [n[0] / len, n[1] / len, n[2] / len]
        } else {
            [
                vertices[i0 * 6 + 3],
                vertices[i0 * 6 + 4],
                vertices[i0 * 6 + 5],
            ]
        };
        for &vi in &[i0, i1, i2] {
            vertices[vi * 6 + 3] = nn[0];
            vertices[vi * 6 + 4] = nn[1];
            vertices[vi * 6 + 5] = nn[2];
        }
    }
}

fn enforce_outward_normals(vertices: &mut [f32], indices: &[u32], per_triangle: bool) {
    let vcount = vertices.len() / 6;
    if vcount == 0 || indices.is_empty() {
        return;
    }

    let (mut cx, mut cy, mut cz) = (0.0f32, 0.0f32, 0.0f32);
    for v in 0..vcount {
        cx += vertices[v * 6];
        cy += vertices[v * 6 + 1];
        cz += vertices[v * 6 + 2];
    }
    let inv = 1.0 / vcount as f32;
    let (cx, cy, cz) = (cx * inv, cy * inv, cz * inv);

    let flip_triangle = |vertices: &mut [f32], tri: &[u32]| {
        for &vi in tri {
            let b = vi as usize * 6;
            vertices[b + 3] = -vertices[b + 3];
            vertices[b + 4] = -vertices[b + 4];
            vertices[b + 5] = -vertices[b + 5];
        }
    };

    let mut orient = 0.0f32;
    for tri in indices.chunks_exact(3) {
        let i0 = tri[0] as usize * 6;
        let i1 = tri[1] as usize * 6;
        let i2 = tri[2] as usize * 6;
        let tcx = (vertices[i0] + vertices[i1] + vertices[i2]) / 3.0 - cx;
        let tcy = (vertices[i0 + 1] + vertices[i1 + 1] + vertices[i2 + 1]) / 3.0 - cy;
        let tcz = (vertices[i0 + 2] + vertices[i1 + 2] + vertices[i2 + 2]) / 3.0 - cz;
        let dot = vertices[i0 + 3] * tcx + vertices[i0 + 4] * tcy + vertices[i0 + 5] * tcz;
        if per_triangle {
            if dot < 0.0 {
                flip_triangle(vertices, tri);
            }
        } else {
            orient += dot;
        }
    }

    if !per_triangle && orient < 0.0 {
        for v in 0..vcount {
            vertices[v * 6 + 3] = -vertices[v * 6 + 3];
            vertices[v * 6 + 4] = -vertices[v * 6 + 4];
            vertices[v * 6 + 5] = -vertices[v * 6 + 5];
        }
    }
}

fn apply_analytic_cylinder_normals(
    solid: &KernelSolid,
    vertices: &mut [f32],
    indices: &[u32],
    face_ids: &[u32],
) {
    let faces = solid.shell().faces();
    let cylinders: Vec<Option<CylindricalSurface>> = faces
        .iter()
        .map(|face| match face.surface() {
            Some(GeomSurface::Cylinder(cyl)) => Some(*cyl),
            _ => None,
        })
        .collect();

    let current_normal = |vertices: &[f32], vi: u32| -> [f32; 3] {
        let b = vi as usize * 6;
        [vertices[b + 3], vertices[b + 4], vertices[b + 5]]
    };

    let radial_normal = |vertices: &[f32], vi: u32, cyl: &CylindricalSurface| -> Option<[f32; 3]> {
        let b = vi as usize * 6;
        let p = Pnt::new(
            vertices[b] as f64,
            vertices[b + 1] as f64,
            vertices[b + 2] as f64,
        );
        let axis = GeomVec::from_dir(cyl.position().direction());
        let from_axis = p - cyl.position().location();
        let radial = from_axis.subtracted(&axis.multiplied(from_axis.dot(&axis)));
        let len = radial.magnitude();
        if len <= 1.0e-12 {
            return None;
        }
        Some([
            (radial.x() / len) as f32,
            (radial.y() / len) as f32,
            (radial.z() / len) as f32,
        ])
    };

    let mut face_signs = vec![0.0f32; cylinders.len()];
    for (t, tri) in indices.chunks_exact(3).enumerate() {
        let fid = face_ids.get(t).copied().unwrap_or(0) as usize;
        let Some(Some(cyl)) = cylinders.get(fid) else {
            continue;
        };
        if face_signs[fid] != 0.0 {
            continue;
        }

        for &vi in tri {
            let Some(n) = radial_normal(vertices, vi, cyl) else {
                continue;
            };
            let reference = current_normal(vertices, vi);
            let dot = n[0] * reference[0] + n[1] * reference[1] + n[2] * reference[2];
            if dot.abs() > 1.0e-6 {
                face_signs[fid] = if dot < 0.0 { -1.0 } else { 1.0 };
                break;
            }
        }
    }
    for sign in &mut face_signs {
        if *sign == 0.0 {
            *sign = 1.0;
        }
    }

    for (t, tri) in indices.chunks_exact(3).enumerate() {
        let fid = face_ids.get(t).copied().unwrap_or(0) as usize;
        let Some(Some(cyl)) = cylinders.get(fid) else {
            continue;
        };
        let sign = face_signs.get(fid).copied().unwrap_or(1.0);

        for &vi in tri {
            let Some(n) = radial_normal(vertices, vi, cyl) else {
                continue;
            };
            let b = vi as usize * 6;
            vertices[b + 3] = n[0] * sign;
            vertices[b + 4] = n[1] * sign;
            vertices[b + 5] = n[2] * sign;
        }
    }
}

fn add_missing_straight_brep_edges(
    solid: &KernelSolid,
    vertices: &[f32],
    indices: &[u32],
    face_ids: &[u32],
    surface_group: &[u32],
    edge_vertices: &mut Vec<f32>,
    edge_indices: &mut Vec<u32>,
    edge_face_normals: &mut Vec<f32>,
    edge_pairs: &mut Vec<(u32, u32)>,
) {
    let edge_key_from_points = |a: [f32; 3], b: [f32; 3]| {
        let q = |v: f32| (v as f64 * 10_000.0).round() as i64;
        let ka = (q(a[0]), q(a[1]), q(a[2]));
        let kb = (q(b[0]), q(b[1]), q(b[2]));
        if ka <= kb {
            (ka, kb)
        } else {
            (kb, ka)
        }
    };

    let mut existing = HashSet::new();
    for pair in edge_indices.chunks_exact(2) {
        let ia = pair[0] as usize * 3;
        let ib = pair[1] as usize * 3;
        let a = [
            edge_vertices[ia],
            edge_vertices[ia + 1],
            edge_vertices[ia + 2],
        ];
        let b = [
            edge_vertices[ib],
            edge_vertices[ib + 1],
            edge_vertices[ib + 2],
        ];
        existing.insert(edge_key_from_points(a, b));
    }

    let mut face_normal: HashMap<u32, [f32; 3]> = HashMap::new();
    let mut face_count: HashMap<u32, u32> = HashMap::new();
    for (t, tri) in indices.chunks_exact(3).enumerate() {
        let fid = face_ids.get(t).copied().unwrap_or(0);
        let b = tri[0] as usize * 6;
        let n = [vertices[b + 3], vertices[b + 4], vertices[b + 5]];
        let sum = face_normal.entry(fid).or_insert([0.0, 0.0, 0.0]);
        sum[0] += n[0];
        sum[1] += n[1];
        sum[2] += n[2];
        *face_count.entry(fid).or_insert(0) += 1;
    }
    for (fid, n) in &mut face_normal {
        let count = face_count.get(fid).copied().unwrap_or(1) as f32;
        n[0] /= count;
        n[1] /= count;
        n[2] /= count;
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        if len > 1.0e-6 {
            n[0] /= len;
            n[1] /= len;
            n[2] /= len;
        }
    }

    let mut topo_edges: HashMap<
        ((i64, i64, i64), (i64, i64, i64)),
        ([f32; 3], [f32; 3], Vec<u32>),
    > = HashMap::new();
    for (fid, face) in solid.shell().faces().iter().enumerate() {
        for wire in face.wires() {
            for edge in wire.edges() {
                let a = edge.start().point();
                let b = edge.end().point();
                let pa = [a.x() as f32, a.y() as f32, a.z() as f32];
                let pb = [b.x() as f32, b.y() as f32, b.z() as f32];
                let d2 =
                    (pa[0] - pb[0]).powi(2) + (pa[1] - pb[1]).powi(2) + (pa[2] - pb[2]).powi(2);
                if d2 < 1.0e-8 || !edge_is_straight(&edge) {
                    continue;
                }
                let rec = topo_edges
                    .entry(edge_key_from_points(pa, pb))
                    .or_insert_with(|| (pa, pb, Vec::new()));
                if !rec.2.contains(&(fid as u32)) {
                    rec.2.push(fid as u32);
                }
            }
        }
    }

    for (key, (pa, pb, faces)) in topo_edges {
        if existing.contains(&key) || faces.len() < 2 {
            continue;
        }
        // Don't re-introduce a same-surface cylinder seam (see `mesh_feature_edges`):
        // a straight longitudinal edge whose two faces are arc-faces of one cylinder.
        if faces.len() == 2 {
            let g0 = surface_group.get(faces[0] as usize);
            let g1 = surface_group.get(faces[1] as usize);
            if g0.is_some() && g0 == g1 {
                continue;
            }
        }
        let d2 = (pa[0] - pb[0]).powi(2) + (pa[1] - pb[1]).powi(2) + (pa[2] - pb[2]).powi(2);
        if d2 < 1.0e-6 {
            continue;
        }
        let a = (edge_vertices.len() / 3) as u32;
        edge_vertices.extend_from_slice(&pa);
        edge_vertices.extend_from_slice(&pb);
        edge_indices.push(a);
        edge_indices.push(a + 1);

        let n0 = faces
            .get(0)
            .and_then(|fid| face_normal.get(fid).copied())
            .unwrap_or([0.0, 0.0, 1.0]);
        let n1 = faces
            .get(1)
            .and_then(|fid| face_normal.get(fid).copied())
            .unwrap_or(n0);
        edge_face_normals.extend_from_slice(&n0);
        edge_face_normals.extend_from_slice(&n1);
        // Each restored straight edge is one B-Rep edge: tag it with its bordering
        // surface pair so it joins the same grouping pass as the feature edges.
        let f0 = faces.first().copied().unwrap_or(0);
        let f1 = faces.get(1).copied().unwrap_or(f0);
        edge_pairs.push(canonical_surface_pair(f0, f1, surface_group));
    }
}

fn edge_is_straight(edge: &Edge) -> bool {
    let Some(curve) = edge.curve() else {
        return true;
    };
    let first = edge.first();
    let last = edge.last();
    let mid = 0.5 * (first + last);
    let p0 = curve.point(first);
    let p1 = curve.point(last);
    let pm = curve.point(mid);
    let chord = p1 - p0;
    let len = chord.magnitude();
    if len <= 1.0e-9 {
        return false;
    }
    let along = chord / len;
    let d = pm - p0;
    let closest = p0 + along * d.dot(&along);
    pm.distance(&closest) < 1.0e-4
}

// ---------------------------------------------------------------------------
// Analytical wireframes (unchanged behavior from the prior mock kernel)
// ---------------------------------------------------------------------------

fn build_box_wireframe(w: f32, h: f32, d: f32) -> (Vec<f32>, Vec<u32>, Vec<f32>) {
    let pts: [[f32; 3]; 8] = [
        [0.0, 0.0, 0.0],
        [w, 0.0, 0.0],
        [w, h, 0.0],
        [0.0, h, 0.0],
        [0.0, 0.0, d],
        [w, 0.0, d],
        [w, h, d],
        [0.0, h, d],
    ];

    let mut edge_vertices = Vec::with_capacity(24);
    for p in &pts {
        edge_vertices.extend_from_slice(p);
    }

    // Each box edge borders exactly two of the six axis-aligned faces. The
    // adjacent outward normals let the renderer drop edges whose both faces
    // point away from the camera (hidden-line removal).
    const NX: [f32; 3] = [-1.0, 0.0, 0.0];
    const PX: [f32; 3] = [1.0, 0.0, 0.0];
    const NY: [f32; 3] = [0.0, -1.0, 0.0];
    const PY: [f32; 3] = [0.0, 1.0, 0.0];
    const NZ: [f32; 3] = [0.0, 0.0, -1.0];
    const PZ: [f32; 3] = [0.0, 0.0, 1.0];

    // (edge endpoints, faceA normal, faceB normal)
    let edges: [([u32; 2], [f32; 3], [f32; 3]); 12] = [
        ([0, 1], NZ, NY),
        ([1, 2], NZ, PX),
        ([2, 3], NZ, PY),
        ([3, 0], NZ, NX),
        ([4, 5], PZ, NY),
        ([5, 6], PZ, PX),
        ([6, 7], PZ, PY),
        ([7, 4], PZ, NX),
        ([0, 4], NX, NY),
        ([1, 5], PX, NY),
        ([2, 6], PX, PY),
        ([3, 7], NX, PY),
    ];

    let mut edge_indices = Vec::with_capacity(24);
    let mut edge_face_normals = Vec::with_capacity(72);
    for ([a, b], na, nb) in &edges {
        edge_indices.push(*a);
        edge_indices.push(*b);
        edge_face_normals.extend_from_slice(na);
        edge_face_normals.extend_from_slice(nb);
    }

    (edge_vertices, edge_indices, edge_face_normals)
}

fn build_cylinder_wireframe(r: f32, h: f32, segments: u32) -> (Vec<f32>, Vec<u32>, Vec<f32>) {
    let mut edge_vertices = Vec::new();
    let mut edge_indices = Vec::new();
    let mut edge_face_normals = Vec::new();
    let push_vtx = |ev: &mut Vec<f32>, x: f32, y: f32, z: f32| -> u32 {
        ev.extend_from_slice(&[x, y, z]);
        (ev.len() / 3) as u32 - 1
    };
    // Outward radial normal of the curved wall at angle theta.
    let radial = |theta: f32| -> [f32; 3] { [theta.cos(), 0.0, theta.sin()] };
    let seg_mid = |i: u32| -> f32 { ((i as f32 + 0.5) / segments as f32) * std::f32::consts::TAU };
    const CAP_BOT: [f32; 3] = [0.0, -1.0, 0.0];
    const CAP_TOP: [f32; 3] = [0.0, 1.0, 0.0];

    // Bottom ring — borders the bottom cap and the curved wall.
    let mut bot_idx = Vec::with_capacity(segments as usize);
    for i in 0..segments {
        let theta = (i as f32 / segments as f32) * std::f32::consts::TAU;
        bot_idx.push(push_vtx(
            &mut edge_vertices,
            r * theta.cos(),
            0.0,
            r * theta.sin(),
        ));
    }
    for i in 0..segments {
        edge_indices.push(bot_idx[i as usize]);
        edge_indices.push(bot_idx[((i + 1) % segments) as usize]);
        edge_face_normals.extend_from_slice(&CAP_BOT);
        edge_face_normals.extend_from_slice(&radial(seg_mid(i)));
    }

    // Top ring — borders the top cap and the curved wall.
    let mut top_idx = Vec::with_capacity(segments as usize);
    for i in 0..segments {
        let theta = (i as f32 / segments as f32) * std::f32::consts::TAU;
        top_idx.push(push_vtx(
            &mut edge_vertices,
            r * theta.cos(),
            h,
            r * theta.sin(),
        ));
    }
    for i in 0..segments {
        edge_indices.push(top_idx[i as usize]);
        edge_indices.push(top_idx[((i + 1) % segments) as usize]);
        edge_face_normals.extend_from_slice(&CAP_TOP);
        edge_face_normals.extend_from_slice(&radial(seg_mid(i)));
    }

    // Four vertical struts at quadrants — silhouette helpers along the wall, so
    // both adjacent "faces" are the wall at that angle.
    for k in 0..4u32 {
        let theta = (k as f32 / 4.0) * std::f32::consts::TAU;
        let b = push_vtx(&mut edge_vertices, r * theta.cos(), 0.0, r * theta.sin());
        let t = push_vtx(&mut edge_vertices, r * theta.cos(), h, r * theta.sin());
        edge_indices.push(b);
        edge_indices.push(t);
        edge_face_normals.extend_from_slice(&radial(theta));
        edge_face_normals.extend_from_slice(&radial(theta));
    }

    (edge_vertices, edge_indices, edge_face_normals)
}

fn build_extrusion_wireframe(
    points: &[(f32, f32)],
    holes: &[Vec<(f32, f32)>],
    depth: f32,
    cs: &crate::geometry::CoordinateSystem,
) -> (Vec<f32>, Vec<u32>, Vec<f32>) {
    use crate::geometry::Vec3;

    let mut edge_vertices = Vec::new();
    let mut edge_indices = Vec::new();
    let mut edge_face_normals: Vec<f32> = Vec::new();

    // Sweep sign: which axial direction the solid grows. Used to orient the cap
    // normals so they point out of the solid (away from the body interior).
    let sd = if depth < 0.0 { -1.0 } else { 1.0 };
    let cap_bottom = cs.n.mul(-sd); // base cap faces away from the sweep
    let cap_top = cs.n.mul(sd); // far cap faces along the sweep

    // Emit the bottom loop, top loop, and vertical struts for one closed loop,
    // tagging every edge with the two faces it borders so the renderer can hide
    // edges whose both faces point away from the camera.
    let add_loop =
        |loop_pts: &[(f32, f32)], ev: &mut Vec<f32>, ei: &mut Vec<u32>, efn: &mut Vec<f32>| {
            let n = loop_pts.len();
            if n < 2 {
                return;
            }
            let push_vtx = |ev: &mut Vec<f32>, p: Vec3| -> u32 {
                ev.extend_from_slice(&[p.x, p.y, p.z]);
                (ev.len() / 3) as u32 - 1
            };

            // Base-loop 3D points and their centroid (for choosing outward signs).
            let base: Vec<Vec3> = loop_pts.iter().map(|p| cs.unproject(p.0, p.1)).collect();
            let mut centroid = Vec3::ZERO;
            for p in &base {
                centroid = centroid.add(*p);
            }
            centroid = centroid.mul(1.0 / n as f32);

            // Outward normal of each side wall (between vertex i and i+1).
            let wall: Vec<Vec3> = (0..n)
                .map(|i| {
                    let a = base[i];
                    let b = base[(i + 1) % n];
                    let edge_dir = b.sub(a);
                    let mut wn = edge_dir.cross(cs.n).normalize();
                    let mid = a.add(b).mul(0.5);
                    if wn.dot(mid.sub(centroid)) < 0.0 {
                        wn = wn.mul(-1.0);
                    }
                    wn
                })
                .collect();

            let mut bot_idx = Vec::with_capacity(n);
            let mut top_idx = Vec::with_capacity(n);
            for p in &base {
                let p_t = p.add(cs.n.mul(depth));
                bot_idx.push(push_vtx(ev, *p));
                top_idx.push(push_vtx(ev, p_t));
            }

            let push_n = |efn: &mut Vec<f32>, a: Vec3, b: Vec3| {
                efn.extend_from_slice(&[a.x, a.y, a.z, b.x, b.y, b.z]);
            };

            // A vertical strut is a real silhouette edge only at a *true* corner.
            // Along a smooth run — a tessellated fillet arc (~24 pts) or a
            // sketched circle (`CIRCLE_SEGS` pts) — consecutive walls differ by
            // only a few degrees; strutting every such vertex draws a dense fan of
            // near-parallel lines down the curve (the "spray" artifact). Emit a
            // strut only where the two adjacent walls meet past the crease angle,
            // matching `mesh_feature_edges`' crease filter and the cylinder
            // wireframe, so a rounded edge reads as one smooth surface.
            const STRUT_CREASE_COS: f32 = 0.95; // cos(~18°)
            for i in 0..n {
                // Bottom-loop edge: borders the bottom cap and side wall i.
                ei.push(bot_idx[i]);
                ei.push(bot_idx[(i + 1) % n]);
                push_n(efn, cap_bottom, wall[i]);

                // Top-loop edge: borders the top cap and side wall i.
                ei.push(top_idx[i]);
                ei.push(top_idx[(i + 1) % n]);
                push_n(efn, cap_top, wall[i]);

                // Vertical strut at vertex i: borders walls (i-1) and i. Skip it on
                // a smooth run where those walls nearly agree.
                let prev = wall[(i + n - 1) % n];
                let cur = wall[i];
                if prev.dot(cur).clamp(-1.0, 1.0) <= STRUT_CREASE_COS {
                    ei.push(bot_idx[i]);
                    ei.push(top_idx[i]);
                    push_n(efn, prev, cur);
                }
            }
        };

    add_loop(
        points,
        &mut edge_vertices,
        &mut edge_indices,
        &mut edge_face_normals,
    );
    for hole in holes {
        add_loop(
            hole,
            &mut edge_vertices,
            &mut edge_indices,
            &mut edge_face_normals,
        );
    }

    (edge_vertices, edge_indices, edge_face_normals)
}

/// Wireframe for a real cylinder: two smooth rim circles (top + bottom) and a
/// few silhouette struts down the wall, oriented onto the sketch plane. Each
/// edge carries the two adjacent face normals (cap + radial wall) so the
/// renderer's hidden-line removal works exactly as it does for prisms — but the
/// wall reads as one smooth surface instead of a fan of facet edges.
fn build_oriented_cylinder_wireframe(
    cs: &crate::geometry::CoordinateSystem,
    cu: f32,
    cv: f32,
    r: f32,
    depth: f32,
) -> (Vec<f32>, Vec<u32>, Vec<f32>) {
    use crate::geometry::Vec3;
    let mut ev: Vec<f32> = Vec::new();
    let mut ei: Vec<u32> = Vec::new();
    let mut efn: Vec<f32> = Vec::new();

    let center = cs.unproject(cu, cv);
    let axis = cs.n.mul(depth);
    let sd = if depth < 0.0 { -1.0 } else { 1.0 };
    let cap_bot = cs.n.mul(-sd);
    let cap_top = cs.n.mul(sd);

    let rim = |ang: f32, t: Vec3| -> Vec3 {
        center
            .add(cs.u.mul(r * ang.cos()))
            .add(cs.v.mul(r * ang.sin()))
            .add(t)
    };
    let radial = |ang: f32| -> Vec3 { cs.u.mul(ang.cos()).add(cs.v.mul(ang.sin())) };
    let mut push = |p: Vec3| -> u32 {
        ev.extend_from_slice(&[p.x, p.y, p.z]);
        (ev.len() / 3) as u32 - 1
    };
    let push_n = |efn: &mut Vec<f32>, a: Vec3, b: Vec3| {
        efn.extend_from_slice(&[a.x, a.y, a.z, b.x, b.y, b.z]);
    };

    let n = CYL_WIRE_SEGS;
    let ang = |i: usize| (i as f32 / n as f32) * std::f32::consts::TAU;
    let mid = |i: usize| ((i as f32 + 0.5) / n as f32) * std::f32::consts::TAU;

    // Bottom + top rim circles.
    let bot: Vec<u32> = (0..n).map(|i| push(rim(ang(i), Vec3::ZERO))).collect();
    let top: Vec<u32> = (0..n).map(|i| push(rim(ang(i), axis))).collect();
    for i in 0..n {
        let j = (i + 1) % n;
        ei.push(bot[i]);
        ei.push(bot[j]);
        push_n(&mut efn, cap_bot, radial(mid(i)));
        ei.push(top[i]);
        ei.push(top[j]);
        push_n(&mut efn, cap_top, radial(mid(i)));
    }

    // Four silhouette struts at the quadrants (both adjacent "faces" are the
    // wall at that angle, so the strut shows whenever the wall faces the camera).
    for k in 0..4 {
        let a = (k as f32 / 4.0) * std::f32::consts::TAU;
        let b = push(rim(a, Vec3::ZERO));
        let t = push(rim(a, axis));
        ei.push(b);
        ei.push(t);
        push_n(&mut efn, radial(a), radial(a));
    }

    (ev, ei, efn)
}

/// Build a hidden-line-ready wireframe from a tessellated mesh (the interleaved
/// `[x,y,z,nx,ny,nz]` `vertices`, `indices`, and one `face_ids` entry per
/// triangle). Returns `(edge_vertices, edge_indices, edge_face_normals)` in the
/// same layout the analytic box/extrusion wireframes use.
///
/// Only *feature* edges are kept: a triangle edge shared by two triangles with
/// **different** face ids (the crease between two B-Rep faces), or a lone mesh
/// boundary edge. Internal triangulation diagonals — shared by two triangles of
/// the *same* face — are dropped. Each kept edge records its (up to two)
/// adjacent face normals so the renderer hides it when both faces turn away.
///
/// Deriving edges from triangles rather than the raw B-Rep is what fixes the
/// boolean "stray lines": a degenerate zero-area fin face produces no triangle,
/// so it contributes no edge, and back edges now get proper hidden-line removal
/// instead of x-raying through the body.
/// Group B-rep faces that lie on the *same* analytic cylinder, returning one
/// group id per face in `solid.shell().faces()` order (which matches the mesh's
/// `face_id`s). The kernel emits a cylindrical wall — a bored hole, a round boss
/// — as 3 arc-faces (thirds), and the straight longitudinal seams between them
/// are a construction artifact, not design edges: drawn, they make a hole read
/// as a notched/faceted circle. Faces sharing a group are recognised as one
/// surface so those seams can be suppressed. Every non-cylinder face (and each
/// distinct cylinder) gets its own id, so only true co-cylindrical faces match.
fn cylinder_surface_groups(solid: &KernelSolid) -> Vec<u32> {
    // Quantized (axis-foot xyz, axis-dir xyz, radius) — a cylinder's identity.
    type CylSig = (i64, i64, i64, i64, i64, i64, i64);
    let q = |v: f64| (v * 1.0e4).round() as i64;
    // A cylinder's identity is its axis *line* + radius, independent of which
    // generator/location names the axis. Canonicalize the axis point to the foot
    // of the perpendicular from the origin, and the direction to a fixed sign.
    let sig = |s: &CylindricalSurface| -> CylSig {
        let p = s.position();
        let d = p.direction();
        let (mut dx, mut dy, mut dz) = (d.x(), d.y(), d.z());
        // Sign-normalize the direction so +axis and -axis hash the same.
        let lead = if dx.abs() > 1e-9 {
            dx
        } else if dy.abs() > 1e-9 {
            dy
        } else {
            dz
        };
        if lead < 0.0 {
            dx = -dx;
            dy = -dy;
            dz = -dz;
        }
        let loc = p.location();
        let t = loc.x() * dx + loc.y() * dy + loc.z() * dz; // (loc·d)
        let (fx, fy, fz) = (loc.x() - dx * t, loc.y() - dy * t, loc.z() - dz * t);
        (q(fx), q(fy), q(fz), q(dx), q(dy), q(dz), q(s.radius()))
    };

    let mut groups = Vec::with_capacity(solid.shell().faces().len());
    let mut seen: HashMap<CylSig, u32> = HashMap::new();
    let mut next = 0u32;
    for face in solid.shell().faces() {
        let id = match face.surface() {
            Some(GeomSurface::Cylinder(c)) => *seen.entry(sig(c)).or_insert_with(|| {
                let g = next;
                next += 1;
                g
            }),
            // Non-cylinder: a fresh, unshareable id.
            _ => {
                let g = next;
                next += 1;
                g
            }
        };
        groups.push(id);
    }
    groups
}

fn mesh_feature_edges(
    vertices: &[f32],
    indices: &[u32],
    face_ids: &[u32],
    surface_group: &[u32],
) -> (Vec<f32>, Vec<u32>, Vec<f32>, Vec<(u32, u32)>) {
    // Quantize a vertex position so the independent per-face copies of a shared
    // corner collapse to one key (1e-4 mm, matching the watertightness test).
    let key = |idx: usize| -> (i64, i64, i64) {
        let b = idx * 6;
        let q = |v: f32| (v as f64 * 10_000.0).round() as i64;
        (q(vertices[b]), q(vertices[b + 1]), q(vertices[b + 2]))
    };
    let pos = |idx: usize| -> [f32; 3] {
        let b = idx * 6;
        [vertices[b], vertices[b + 1], vertices[b + 2]]
    };
    // The crease test deliberately uses the *smoothed* per-vertex normal
    // (`solid_to_flat_mesh` runs `smooth_vertex_normals`, which blends normals
    // across shallow creases). That makes adjacent facets of a curved/boolean'd
    // surface look nearly identical, so their seams are suppressed and the round
    // reads as one face — while a genuine sharp edge (≥30°, beyond the smoothing
    // cap) keeps each face's distinct normal and still draws. A planar face's
    // vertices share one normal, so the first vertex's is fine.
    let nrm = |idx: usize| -> [f32; 3] {
        let b = idx * 6;
        [vertices[b + 3], vertices[b + 4], vertices[b + 5]]
    };

    struct EdgeRec {
        pa: [f32; 3],
        pb: [f32; 3],
        tris: u32,
        faces: Vec<(u32, [f32; 3])>, // distinct (face id, that face's normal)
    }
    let mut edges: HashMap<((i64, i64, i64), (i64, i64, i64)), EdgeRec> = HashMap::new();

    for (t, tri) in indices.chunks_exact(3).enumerate() {
        let fid = face_ids.get(t).copied().unwrap_or(0);
        let n = nrm(tri[0] as usize);
        for &(i, j) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            let (vi, vj) = (tri[i] as usize, tri[j] as usize);
            let (ka, kb) = (key(vi), key(vj));
            let k = if ka <= kb { (ka, kb) } else { (kb, ka) };
            let rec = edges.entry(k).or_insert_with(|| EdgeRec {
                pa: pos(vi),
                pb: pos(vj),
                tris: 0,
                faces: Vec::new(),
            });
            rec.tris += 1;
            if !rec.faces.iter().any(|(f, _)| *f == fid) {
                rec.faces.push((fid, n));
            }
        }
    }

    // Classify each B-rep face as flat or curved by whether its triangles' stored
    // (smoothed) normals vary. A fillet/cylinder face is curved; box and cap faces
    // are flat. Used below so a fillet's *tangent boundary* — where its curved face
    // meets a flat one with nearly-equal normals — is kept as a real edge (the
    // top/bottom line of the round), while a faceted fallback's flat-facet seams
    // (also shallow) stay suppressed.
    let mut face_ref_n: HashMap<u32, [f32; 3]> = HashMap::new();
    let mut face_curved: HashMap<u32, bool> = HashMap::new();
    for (t, tri) in indices.chunks_exact(3).enumerate() {
        let fid = face_ids.get(t).copied().unwrap_or(0);
        let r = *face_ref_n
            .entry(fid)
            .or_insert_with(|| nrm(tri[0] as usize));
        for &v in tri {
            let n = nrm(v as usize);
            // ~2.5°: a flat B-rep face's vertices share one (anchored) normal, so
            // it never trips this; a curved face's normals fan out and do.
            const CURVE_COS: f32 = 0.999;
            if r[0] * n[0] + r[1] * n[1] + r[2] * n[2] < CURVE_COS {
                face_curved.insert(fid, true);
            }
        }
    }

    let mut edge_vertices: Vec<f32> = Vec::new();
    let mut edge_indices: Vec<u32> = Vec::new();
    let mut edge_face_normals: Vec<f32> = Vec::new();
    // Per kept segment: the canonical pair of surface ids it borders. Aligned with
    // each `edge_indices` pair, this is what `group_edge_segments` chains on so an
    // arc's chords (constant surface pair along the whole arc) become one edge.
    let mut edge_pairs: Vec<(u32, u32)> = Vec::new();
    for rec in edges.values() {
        // Keep only a crease between two *distinct* B-rep faces. This drops both
        // internal triangulation diagonals (two triangles of the SAME face) and
        // lone boundary edges (a single triangle owns the edge). Every ZeroCAD
        // body is a closed solid, so a watertight tessellation has no genuine
        // boundary edge — a single-triangle edge is a crack/sliver left by a
        // fragile boolean, and drawing it is exactly the "stray spray". Real
        // design edges are always shared by ≥2 faces, so they're untouched.
        if rec.faces.len() < 2 {
            continue;
        }
        // Suppress a *same-surface* seam: the straight longitudinal edge where two
        // arc-faces of ONE analytic cylinder meet (the kernel splits a cylindrical
        // wall into thirds). It's a construction artifact, not a design edge — drawn,
        // it makes a bored hole / round boss look notched. Recognised by surface
        // identity rather than normals (the per-face representative normals here are
        // too coarse: each 120° arc-face's sample normal can sit ~60° off the seam).
        if rec.faces.len() == 2 {
            let g0 = surface_group.get(rec.faces[0].0 as usize);
            let g1 = surface_group.get(rec.faces[1].0 as usize);
            if g0.is_some() && g0 == g1 {
                continue;
            }
        }
        // Suppress the *facet-boundary* lines of a curved surface: a crease whose
        // two faces meet at a shallow dihedral (their outward normals nearly
        // agree) is a tessellation seam of a fillet / boolean'd cylinder, not a
        // design edge. Hiding it lets the round read as one smooth face, while
        // genuine edges (box corners at 90°, chamfer bevels at 45°, …) — whose
        // normals differ well past the threshold — still draw. The crease is kept
        // only when the normals diverge by more than ~`CREASE_COS` (≈18°).
        if rec.faces.len() >= 2 {
            let n0 = rec.faces[0].1;
            let n1 = rec.faces[1].1;
            let dot = (n0[0] * n1[0] + n0[1] * n1[1] + n0[2] * n1[2]).clamp(-1.0, 1.0);
            const CREASE_COS: f32 = 0.95; // cos(~18°)
                                          // A curved face (fillet/cylinder) meets its neighbour along a *tangent*
                                          // edge whose normals nearly agree — yet it's a real design edge (the
                                          // top/bottom of a fillet, a cylinder's rim), so any shallow crease that
                                          // touches a curved face is kept. Only a shallow crease between two
                                          // genuinely flat faces is a faceted tessellation seam to hide.
                                          // (A *same-surface* seam — two arc-faces of one cylinder — is handled
                                          // separately above via `surface_group`; here the representative
                                          // per-face normals are too coarse to recognise it.)
            let touches_curved = rec
                .faces
                .iter()
                .any(|(fid, _)| face_curved.get(fid).copied().unwrap_or(false));
            if dot > CREASE_COS && !touches_curved {
                continue;
            }
        }
        // Drop a degenerate zero-length edge (collapsed by a sliver triangle): it
        // would render as a stray dot/spike and never as a real line.
        let d2 = (rec.pa[0] - rec.pb[0]).powi(2)
            + (rec.pa[1] - rec.pb[1]).powi(2)
            + (rec.pa[2] - rec.pb[2]).powi(2);
        if d2 < 1.0e-12 {
            continue;
        }
        let a = (edge_vertices.len() / 3) as u32;
        edge_vertices.extend_from_slice(&rec.pa);
        edge_vertices.extend_from_slice(&rec.pb);
        edge_indices.push(a);
        edge_indices.push(a + 1);
        // Two adjacent face normals; duplicate the lone one on a boundary edge.
        let n0 = rec.faces[0].1;
        let n1 = rec.faces.get(1).map_or(n0, |(_, n)| *n);
        edge_face_normals.extend_from_slice(&n0);
        edge_face_normals.extend_from_slice(&n1);
        // Bordering surface pair, canonicalized so arc-faces of one cylinder match.
        let f0 = rec.faces[0].0;
        let f1 = rec.faces.get(1).map_or(f0, |(f, _)| *f);
        edge_pairs.push(canonical_surface_pair(f0, f1, surface_group));
    }

    (edge_vertices, edge_indices, edge_face_normals, edge_pairs)
}

/// Canonical, order-independent key for the pair of *surfaces* an edge borders.
///
/// Each face id is mapped through `surface_group` so the arc-faces a cylinder is
/// split into collapse to one surface — then the smaller id is placed first.
/// Segments that share this key and touch are chords of the same topological
/// edge (see [`group_edge_segments`]).
fn canonical_surface_pair(f0: u32, f1: u32, surface_group: &[u32]) -> (u32, u32) {
    let g0 = surface_group.get(f0 as usize).copied().unwrap_or(f0);
    let g1 = surface_group.get(f1 as usize).copied().unwrap_or(f1);
    if g0 <= g1 {
        (g0, g1)
    } else {
        (g1, g0)
    }
}

/// Group edge **segments** into topological edges, returning one group id per
/// segment (parallel to the `edge_indices` pairs).
///
/// Two segments that meet at a shared (welded) endpoint are placed in the same
/// group when:
/// * `surface_pairs` is given (B-Rep solids) — they border the same surface pair.
///   An arc keeps one pair along its whole length, so its chords chain into one
///   edge; a corner where the pair changes splits them.
/// * `surface_pairs` is `None` (analytic primitive wireframes, which carry no
///   face provenance) — exactly two segments meet there and pass through nearly
///   straight (tangent-continuous). This chains a rim circle's chords while
///   leaving a box's 90° corners as separate edges.
fn group_edge_segments(
    edge_vertices: &[f32],
    edge_indices: &[u32],
    surface_pairs: Option<&[(u32, u32)]>,
) -> Vec<u32> {
    let n = edge_indices.len() / 2;
    if n == 0 {
        return Vec::new();
    }

    // Weld endpoints by quantized position (1e-4 mm, matching the wireframe build).
    let vkey = |vi: u32| -> (i64, i64, i64) {
        let b = vi as usize * 3;
        let q = |v: f32| (v as f64 * 10_000.0).round() as i64;
        (
            q(edge_vertices[b]),
            q(edge_vertices[b + 1]),
            q(edge_vertices[b + 2]),
        )
    };
    let pos = |vi: u32| -> [f64; 3] {
        let b = vi as usize * 3;
        [
            edge_vertices[b] as f64,
            edge_vertices[b + 1] as f64,
            edge_vertices[b + 2] as f64,
        ]
    };

    // Welded vertex -> the segments incident to it.
    let mut at: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
    for s in 0..n {
        at.entry(vkey(edge_indices[s * 2])).or_default().push(s);
        at.entry(vkey(edge_indices[s * 2 + 1])).or_default().push(s);
    }

    // Union-find over segments.
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(parent: &mut [usize], mut x: usize) -> usize {
        while parent[x] != x {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        x
    }
    let union = |parent: &mut [usize], a: usize, b: usize| {
        let (ra, rb) = (find(parent, a), find(parent, b));
        if ra != rb {
            parent[ra.max(rb)] = ra.min(rb);
        }
    };

    // Direction of segment `s` pointing away from welded vertex `key`.
    let dir_away = |s: usize, key: (i64, i64, i64)| -> Option<[f64; 3]> {
        let (a, b) = (edge_indices[s * 2], edge_indices[s * 2 + 1]);
        let (from, to) = if vkey(a) == key { (a, b) } else { (b, a) };
        let (p, q) = (pos(from), pos(to));
        let d = [q[0] - p[0], q[1] - p[1], q[2] - p[2]];
        let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        (l > 1e-9).then(|| [d[0] / l, d[1] / l, d[2] / l])
    };

    for (key, segs) in &at {
        match surface_pairs {
            Some(pairs) => {
                // Chain every pair of incident segments that border the same surfaces.
                for i in 0..segs.len() {
                    for j in (i + 1)..segs.len() {
                        if pairs[segs[i]] == pairs[segs[j]] {
                            union(&mut parent, segs[i], segs[j]);
                        }
                    }
                }
            }
            None => {
                // Chain any pair of incident segments that pass through nearly
                // straight (tangent-continuous). Their outward dirs point opposite
                // for a smooth pass; ~25° slack chains coarse rim arcs yet splits
                // corners. Pairwise (not degree-2 only) so a rim still chains
                // *through* a silhouette strut's junction — the two rim chords are
                // tangent while the strut, branching off, joins neither.
                for i in 0..segs.len() {
                    for j in (i + 1)..segs.len() {
                        if let (Some(d0), Some(d1)) =
                            (dir_away(segs[i], *key), dir_away(segs[j], *key))
                        {
                            let dot = d0[0] * d1[0] + d0[1] * d1[1] + d0[2] * d1[2];
                            if dot < -0.9 {
                                union(&mut parent, segs[i], segs[j]);
                            }
                        }
                    }
                }
            }
        }
    }

    // Densely renumber roots into stable group ids.
    let mut group = vec![0u32; n];
    let mut remap: HashMap<usize, u32> = HashMap::new();
    let mut next = 0u32;
    for (s, g) in group.iter_mut().enumerate() {
        let r = find(&mut parent, s);
        *g = *remap.entry(r).or_insert_with(|| {
            let id = next;
            next += 1;
            id
        });
    }
    group
}

#[cfg(test)]
mod wireframe_tests {
    use super::*;
    use crate::geometry::CoordinateSystem;

    /// Count edges that run parallel to the sweep axis (i.e. vertical struts):
    /// their endpoints share x/y and differ by `|depth|` along z (CS::XY here).
    fn count_struts(ev: &[f32], ei: &[u32], depth: f32) -> usize {
        ei.chunks_exact(2)
            .filter(|p| {
                let (a, b) = (p[0] as usize * 3, p[1] as usize * 3);
                let dz = (ev[a + 2] - ev[b + 2]).abs();
                let dxy = ((ev[a] - ev[b]).powi(2) + (ev[a + 1] - ev[b + 1]).powi(2)).sqrt();
                (dz - depth.abs()).abs() < 1e-3 && dxy < 1e-3
            })
            .count()
    }

    #[test]
    fn same_face_triangle_diagonal_is_not_a_feature_edge() {
        let vertices = vec![
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, //
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, //
            0.0, 1.0, 0.0, 0.0, 0.0, 1.0, //
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, //
            1.0, 1.0, 0.0, 0.0, 0.0, 1.0, //
            0.0, 1.0, 0.0, 0.0, 0.0, 1.0, //
        ];
        let indices = vec![0, 1, 2, 3, 4, 5];
        let face_ids = vec![0, 0];

        let (_ev, ei, _, _) = mesh_feature_edges(&vertices, &indices, &face_ids, &[]);

        assert_eq!(
            ei.len() / 2,
            0,
            "triangle edges internal to one B-Rep face must not be selectable or drawn"
        );
    }

    #[test]
    fn sharp_polygon_struts_every_true_corner() {
        // A square has four genuine 90° corners → exactly four vertical struts.
        let square = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];
        let (ev, ei, _) = build_extrusion_wireframe(&square, &[], 5.0, &CoordinateSystem::XY);
        assert_eq!(count_struts(&ev, &ei, 5.0), 4);
    }

    #[test]
    fn smooth_circle_extrude_has_no_strut_fan() {
        // A sketched circle is a many-sided polygon whose consecutive walls differ
        // by only a few degrees. Pre-fix this drew one strut per segment (a dense
        // fan); now the smooth wall must produce ZERO struts.
        let n = crate::CIRCLE_SEGS;
        let circle: Vec<(f32, f32)> = (0..n)
            .map(|i| {
                let a = (i as f32 / n as f32) * std::f32::consts::TAU;
                (5.0 * a.cos(), 5.0 * a.sin())
            })
            .collect();
        let (ev, ei, _) = build_extrusion_wireframe(&circle, &[], 8.0, &CoordinateSystem::XY);
        assert_eq!(count_struts(&ev, &ei, 8.0), 0);
    }

    #[test]
    fn bored_hole_wall_has_no_fake_seam_struts() {
        // A 40x20x10 block with a Ø8 pocket bored 6mm into its top face. The kernel
        // builds the pocket wall as a smooth analytic cylinder, but represents it as
        // 3 arc-faces (thirds); the longitudinal seams between those arc-faces are a
        // construction artifact, not design edges. Drawing them made the hole read as
        // a notched/faceted circle from the top. They run straight down the wall, so
        // they're vertical struts of the *pocket* depth (6) — assert none survive,
        // while the box's four real 90° corner edges (height 10) still draw.
        let block = make_box(&Pnt::origin(), 40.0, 20.0, 10.0);
        let drill = make_cylinder(&Ax2::new(Pnt::new(20.0, 10.0, 4.0), Dir::dz()), 4.0, 6.0);
        let body = boolean_checked(&block, &drill, BooleanOp::Cut).expect("bore should cut");
        let mesh = MockMesh::from_solid(&body);
        assert_eq!(
            count_struts(&mesh.edge_vertices, &mesh.edge_indices, 6.0),
            0,
            "the 3-arc cylinder wall's longitudinal seams must be suppressed"
        );
        assert_eq!(
            count_struts(&mesh.edge_vertices, &mesh.edge_indices, 10.0),
            4,
            "the box's four real 90-degree corner edges must still draw"
        );
    }

    #[test]
    fn lone_boundary_edges_are_dropped() {
        // Two coplanar triangles meeting along a diagonal, the four outer edges
        // owned by a single triangle each. On a closed solid such lone edges are
        // tessellation cracks, not design edges — they must be dropped. The
        // shared diagonal is coplanar (same stored normal) so it's suppressed too,
        // leaving no wireframe at all.
        let v = vec![
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, //
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, //
            1.0, 1.0, 0.0, 0.0, 0.0, 1.0, //
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, //
            1.0, 1.0, 0.0, 0.0, 0.0, 1.0, //
            0.0, 1.0, 0.0, 0.0, 0.0, 1.0, //
        ];
        let indices = vec![0, 1, 2, 3, 4, 5];
        let face_ids = vec![0, 1];
        let (_ev, ei, _, _) = mesh_feature_edges(&v, &indices, &face_ids, &[]);
        assert_eq!(ei.len() / 2, 0, "lone boundary/crack edges must be dropped");
    }

    #[test]
    fn genuine_perpendicular_crease_is_kept() {
        // Two triangles sharing the x-axis edge but lying in perpendicular planes
        // (z=0 and y=0). That 90° crease is a real design edge and must survive.
        let v = vec![
            // Triangle A (face 0) in z=0, normal +Z
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, //
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, //
            0.0, 1.0, 0.0, 0.0, 0.0, 1.0, //
            // Triangle B (face 1) in y=0, normal +Y
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0, //
            0.0, 0.0, 1.0, 0.0, 1.0, 0.0, //
            1.0, 0.0, 0.0, 0.0, 1.0, 0.0, //
        ];
        let indices = vec![0, 1, 2, 3, 4, 5];
        let face_ids = vec![0, 1];
        let (ev, ei, _, _) = mesh_feature_edges(&v, &indices, &face_ids, &[]);

        assert_eq!(
            ei.len() / 2,
            1,
            "the 90° crease should be the one kept edge"
        );
        let (a, b) = (ei[0] as usize * 3, ei[1] as usize * 3);
        let on = |k: usize, p: (f32, f32, f32)| {
            (ev[k] - p.0).abs() < 1e-4
                && (ev[k + 1] - p.1).abs() < 1e-4
                && (ev[k + 2] - p.2).abs() < 1e-4
        };
        let shared = (on(a, (0.0, 0.0, 0.0)) && on(b, (1.0, 0.0, 0.0)))
            || (on(a, (1.0, 0.0, 0.0)) && on(b, (0.0, 0.0, 0.0)));
        assert!(shared, "kept edge should be the shared x-axis crease");
    }

    #[test]
    fn fillet_arc_chords_group_into_one_edge() {
        // Fillet a vertical box edge: the blend's two end arcs are circular, so
        // each tessellates into several chords. Those chords must collapse into a
        // single `edge_groups` id apiece, so the viewport selects the whole arc as
        // one curve (the user's request) instead of a lone chord.
        let solid = make_box(&Pnt::origin(), 10.0, 10.0, 10.0);
        let edge = solid
            .edges()
            .into_iter()
            .find(|e| {
                let p0 = e.start().point();
                let p1 = e.end().point();
                p0.x().abs() < 1e-9
                    && p1.x().abs() < 1e-9
                    && p0.y().abs() < 1e-9
                    && p1.y().abs() < 1e-9
                    && (p0.z() - p1.z()).abs() > 9.9
            })
            .expect("box has a vertical origin edge");
        let filleted = fillet_edges(&solid, &[edge], 2.0).expect("fillet the box edge");
        let mesh = MockMesh::from_solid(&filleted);

        let seg_count = mesh.edge_indices.len() / 2;
        assert!(seg_count > 0, "fillet produced no wireframe edges");
        assert_eq!(
            mesh.edge_groups.len(),
            seg_count,
            "one group id per edge segment"
        );

        let mut sizes: HashMap<u32, usize> = HashMap::new();
        for &g in &mesh.edge_groups {
            *sizes.entry(g).or_insert(0) += 1;
        }
        // At least one edge (a fillet end arc) is many chords grouped as one.
        let max_group = sizes.values().copied().max().unwrap_or(0);
        assert!(
            max_group >= 2,
            "no multi-chord arc was grouped into one edge: sizes={sizes:?}"
        );
        // Grouping actually collapsed segments — fewer groups than chords.
        assert!(
            sizes.len() < seg_count,
            "grouping collapsed nothing: {} groups for {seg_count} chords",
            sizes.len()
        );
    }

    #[test]
    fn smooth_circle_rim_groups_into_one_edge() {
        // A sketched circle extrudes to a smooth cylinder whose rim is one circular
        // edge drawn as many chords. The geometric (tangent-continuity) grouping —
        // used where there's no B-Rep face provenance — must chain those chords
        // into a single edge so the rim selects as one curve.
        let n = crate::CIRCLE_SEGS;
        let circle: Vec<(f32, f32)> = (0..n)
            .map(|i| {
                let a = (i as f32 / n as f32) * std::f32::consts::TAU;
                (5.0 * a.cos(), 5.0 * a.sin())
            })
            .collect();
        let mesh = MockMesh::make_extruded_sketch(&circle, &[], 8.0, &CoordinateSystem::XY);

        let seg_count = mesh.edge_indices.len() / 2;
        assert_eq!(
            mesh.edge_groups.len(),
            seg_count,
            "one group id per segment"
        );

        let mut sizes: HashMap<u32, usize> = HashMap::new();
        for &g in &mesh.edge_groups {
            *sizes.entry(g).or_insert(0) += 1;
        }
        // A rim circle chains into one large group (at least half its chords).
        let big = sizes.values().filter(|&&c| c >= n / 2).count();
        assert!(
            big >= 1,
            "rim circle was not chained into one edge: sizes={sizes:?}"
        );
    }
}

#[cfg(test)]
mod arc_reconstruction_tests {
    use super::*;
    use crate::geometry::CoordinateSystem;
    use openrcad::geom::GeomSurface;

    fn cylinder_faces(solid: &KernelSolid) -> usize {
        solid
            .shell()
            .faces()
            .iter()
            .filter(|f| matches!(f.surface(), Some(GeomSurface::Cylinder(_))))
            .count()
    }

    /// A finely-sampled circular arc (a sketch fillet / drawn arc) must rebuild
    /// into cylindrical walls when extruded — not a fan of planar facets.
    #[test]
    fn semicircle_d_profile_extrudes_to_smooth_cylinder_wall() {
        // "D" shape: a right semicircle (radius 5 about (5,5)) closed by three
        // straight edges down the left. 24 arc facets — pre-fix this swept 24
        // planar strips; now it must be smooth cylindrical wall(s).
        let mut pts: Vec<(f32, f32)> = Vec::new();
        let steps = 24;
        for i in 0..=steps {
            let t = -std::f32::consts::FRAC_PI_2 + (i as f32 / steps as f32) * std::f32::consts::PI;
            pts.push((5.0 + 5.0 * t.cos(), 5.0 + 5.0 * t.sin()));
        }
        pts.push((0.0, 10.0));
        pts.push((0.0, 0.0));

        let solid = extruded_region_solid(&pts, &[], 5.0, &CoordinateSystem::XY)
            .expect("D profile should extrude");
        assert!(
            cylinder_faces(&solid) >= 1,
            "the semicircle must become cylindrical wall(s), got {} cylinders / {} faces",
            cylinder_faces(&solid),
            solid.face_count()
        );
        // A 180° arc capped at 135°/piece → 2 cylinder faces + 3 straight laterals
        // + 2 caps. The exact split count isn't contractual, but the shell must be
        // a closed, healthy solid.
        assert!(solid.is_watertight(), "D extrusion must be watertight");
        assert!(
            solid.health_report().is_healthy(),
            "D extrusion must be healthy"
        );
    }

    /// A polygon the user genuinely wants faceted (here a rectangle) has no
    /// co-circular run, so every corner stays a sharp line edge.
    #[test]
    fn rectangle_stays_a_six_face_box() {
        let rect = [(0.0, 0.0), (10.0, 0.0), (10.0, 6.0), (0.0, 6.0)];
        let solid = extruded_region_solid(&rect, &[], 4.0, &CoordinateSystem::XY)
            .expect("rectangle should extrude");
        assert_eq!(
            cylinder_faces(&solid),
            0,
            "a box must have no cylinder faces"
        );
        assert_eq!(solid.face_count(), 6, "a box is 6 faces");
        assert!(solid.is_watertight());
    }

    /// A full sketched circle fed straight to the prism path (the boolean-tool
    /// fallback) rebuilds into a smooth cylinder, not a 48-gon.
    #[test]
    fn full_circle_profile_rebuilds_to_cylinder() {
        let n = crate::CIRCLE_SEGS;
        let circle: Vec<(f32, f32)> = (0..n)
            .map(|i| {
                let a = (i as f32 / n as f32) * std::f32::consts::TAU;
                (5.0 * a.cos(), 5.0 * a.sin())
            })
            .collect();
        let solid = extruded_region_solid(&circle, &[], 8.0, &CoordinateSystem::XY)
            .expect("circle should extrude");
        assert!(
            cylinder_faces(&solid) >= 1,
            "a circle must rebuild to cylindrical wall(s), got {} cylinders",
            cylinder_faces(&solid)
        );
        assert!(solid.is_watertight(), "circle extrusion must be watertight");
    }

    /// An octagon turns 45° per vertex — well past the arc-vs-corner threshold —
    /// so it must stay a faceted prism, never collapse to a cylinder.
    #[test]
    fn octagon_is_not_mistaken_for_a_circle() {
        let octagon: Vec<(f32, f32)> = (0..8)
            .map(|i| {
                let a = (i as f32 / 8.0) * std::f32::consts::TAU;
                (5.0 * a.cos(), 5.0 * a.sin())
            })
            .collect();
        let solid = extruded_region_solid(&octagon, &[], 4.0, &CoordinateSystem::XY)
            .expect("octagon should extrude");
        assert_eq!(
            cylinder_faces(&solid),
            0,
            "an octagon is an intentional polygon — keep it faceted"
        );
        assert!(solid.is_watertight());
    }
}
