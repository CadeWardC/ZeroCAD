//! Geometry kernel — backed by the `truck` B-Rep CAD kernel.
//!
//! The `MockMesh` name and field layout are preserved for now so existing
//! parametric and rendering code keeps working unchanged. Internally each
//! constructor now builds a real `truck_modeling::Solid`, tessellates it via
//! `truck_meshalgo`, and flattens the result into the same interleaved
//! position+normal vertex buffer the egui painter expects.
//!
//! Wireframe edges are still produced analytically (matching the previous
//! procedural output) — extracting them from the B-Rep topology is deferred
//! to the GPU-viewport phase.

use std::collections::HashMap;

use truck_meshalgo::tessellation::MeshableShape;
use truck_modeling::{builder, Point3, Vector3, Vertex, Wire};
use truck_polymesh::PolygonMesh;
use truck_topology::Solid;

/// The kernel's solid type (a `truck` B-Rep solid). Re-exported so the
/// parametric evaluator can hold solids between features and combine them with
/// boolean operations (join/cut) before tessellating to a `MockMesh`.
pub type KernelSolid = truck_modeling::Solid;

/// Tessellation tolerance (in model units / mm). 0.05mm produces a smooth
/// cylinder without explosive triangle counts. Will become a user-facing
/// setting in a later phase.
const TESS_TOL: f64 = 0.05;

/// Tolerance handed to the boolean solver.
const BOOL_TOL: f64 = 0.05;

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

        self.vertices.extend(other.vertices);
        for idx in other.indices {
            self.indices.push(idx + v_offset);
        }
        self.edge_vertices.extend(other.edge_vertices);
        for idx in other.edge_indices {
            self.edge_indices.push(idx + e_offset);
        }
        // Per-edge data, parallel to edge pairs — no index rebasing needed.
        self.edge_face_normals.extend(other.edge_face_normals);
        for fid in other.face_ids {
            self.face_ids.push(fid + f_offset);
        }
    }

    /// Axis-aligned box with one corner at the origin, opposite corner at (w, h, d).
    pub fn make_box(w: f32, h: f32, d: f32) -> Self {
        let solid = box_solid(w, h, d);

        let (vertices, indices, face_ids) = solid_to_flat_mesh(&solid);

        let (edge_vertices, edge_indices, edge_face_normals) = build_box_wireframe(w, h, d);

        Self {
            vertices,
            indices,
            edge_vertices,
            edge_indices,
            edge_face_normals,
            face_ids,
        }
    }

    /// Cylinder along the +Y axis, base centered at origin, radius r, height h.
    /// `segments` is currently advisory — truck tessellates to TESS_TOL.
    pub fn make_cylinder(r: f32, h: f32, segments: u32) -> Self {
        let solid = match build_cylinder_solid(r as f64, h as f64) {
            Some(s) => s,
            None => return Self::empty(),
        };

        let (vertices, indices, face_ids) = solid_to_flat_mesh(&solid);
        let (edge_vertices, edge_indices, edge_face_normals) =
            build_cylinder_wireframe(r, h, segments.max(4));

        Self {
            vertices,
            indices,
            edge_vertices,
            edge_indices,
            edge_face_normals,
            face_ids,
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
            None => build_extrusion_solid(points, holes, depth as f64, cs)
                .or_else(|| build_extrusion_solid(points, &[], depth as f64, cs)),
        };
        let solid = match solid {
            Some(s) => s,
            None => return Self::empty(),
        };

        let (vertices, indices, face_ids) = solid_to_flat_mesh(&solid);

        let (edge_vertices, edge_indices, edge_face_normals) = match circle {
            Some((cu, cv, r)) => build_oriented_cylinder_wireframe(cs, cu, cv, r, depth),
            None => build_extrusion_wireframe(points, holes, depth, cs),
        };

        Self {
            vertices,
            indices,
            edge_vertices,
            edge_indices,
            edge_face_normals,
            face_ids,
        }
    }

    /// Tessellate an arbitrary kernel solid (typically a boolean result) into a
    /// renderable mesh. Unlike the analytic constructors above, the wireframe is
    /// extracted from the solid's B-Rep edges, and hidden-line normals are left
    /// empty (the renderer then shows every edge).
    pub fn from_solid(solid: &KernelSolid) -> Self {
        let (vertices, indices, face_ids) = solid_to_flat_mesh(solid);
        // Derive the wireframe from the tessellation's *feature* edges (borders
        // between two distinct faces), not the raw B-Rep edge list. This gives
        // every edge its two adjacent face normals — so the renderer's
        // hidden-line removal works for boolean results just like it does for
        // primitives, instead of x-raying every edge — and it silently drops the
        // degenerate zero-area "fin" edges a boolean can leave in the B-Rep
        // (they produce no triangle, so no feature edge, so no stray spike).
        let (edge_vertices, edge_indices, edge_face_normals) =
            mesh_feature_edges(&vertices, &indices, &face_ids);
        Self {
            vertices,
            indices,
            edge_vertices,
            edge_indices,
            edge_face_normals,
            face_ids,
        }
    }
}

// ---------------------------------------------------------------------------
// Public solid builders + boolean operations (used by the parametric evaluator
// to compose join/cut features). Each returns a `truck` solid so several
// features can be combined before a single tessellation pass.
// ---------------------------------------------------------------------------

/// Axis-aligned box solid, one corner at the origin, opposite at (w, h, d).
pub fn box_solid(w: f32, h: f32, d: f32) -> KernelSolid {
    let v0 = builder::vertex(Point3::new(0.0, 0.0, 0.0));
    let edge_x = builder::tsweep(&v0, Vector3::new(w as f64, 0.0, 0.0));
    let face_xy = builder::tsweep(&edge_x, Vector3::new(0.0, h as f64, 0.0));
    builder::tsweep(&face_xy, Vector3::new(0.0, 0.0, d as f64))
}

/// Boolean-ready solid for a cylinder primitive: a polygonal prism along +Y,
/// base centered at origin, **not** a true cylinder.
///
/// This is the cylinder counterpart of `extruded_region_solid`'s "always a
/// prism" rule. truck's boolean solver panics on smooth cylindrical faces, so a
/// cylinder primitive fed to a join/cut would make the operation silently do
/// nothing (the boolean returns `None` via `guarded_boolean` and the body is
/// kept intact). Faceting it to a 48-gon — the same discretization sketched
/// circles use, so a cylinder primitive and an extruded circle cut/join
/// identically — lets booleans succeed. The smooth look is preserved for an
/// untouched body by `make_cylinder` (the `pristine` display mesh); only once a
/// boolean clears `pristine` does the body re-tessellate from this prism, at
/// which point a 48-gon reads as a clean cylinder anyway.
pub fn cylinder_solid(r: f32, h: f32) -> Option<KernelSolid> {
    use crate::geometry::{CoordinateSystem, Vec3};
    // Matches the discretization sketched circles use (`crate::CIRCLE_SEGS`),
    // so a cylinder primitive and an extruded circle cut/join identically.
    const SEGS: usize = crate::CIRCLE_SEGS;
    let pts: Vec<(f32, f32)> = (0..SEGS)
        .map(|i| {
            let a = (i as f32 / SEGS as f32) * std::f32::consts::TAU;
            (r * a.cos(), r * a.sin())
        })
        .collect();
    // Extrude the (CCW) circle on a RIGHT-HANDED frame whose normal is +Y
    // (u = Z, v = X, so u × v = +Y), giving the base-at-origin, +Y-axis cylinder
    // the primitive expects — and, critically, the solid orientation truck's
    // booleans accept. Do NOT use `CoordinateSystem::XZ`: that const frame is
    // left-handed (X × Z = −Y ≠ its +Y normal), and truck extrudes left-handed
    // frames inside-out, which makes `difference` *add* the cut tool instead of
    // subtracting it (the body's `enforce_outward_normals` only fixes display
    // normals, not the solid handed to the solver).
    let frame = CoordinateSystem::new(Vec3::ZERO, Vec3::Z, Vec3::X);
    build_extrusion_solid(&pts, &[], h as f64, &frame)
}

/// Solid for one extruded sketch region, as a polygonal prism. Holed profiles
/// try the holed plane first and fall back to the outer boundary alone if the
/// kernel can't attach it.
///
/// Note this is deliberately a *prism* even for circular profiles: `truck`'s
/// boolean solver reliably handles polyhedral solids but fails (and sometimes
/// panics) on the smooth cylindrical faces a true cylinder would introduce —
/// e.g. a blind cylindrical pocket comes back empty. The *display* of a plain
/// extruded circle is still a smooth cylinder (see `make_extruded_sketch`); only
/// the geometry fed to join/cut booleans is faceted, and a fine circle polygon
/// is visually indistinguishable from a cylinder once it's a hole anyway.
pub fn extruded_region_solid(
    points: &[(f32, f32)],
    holes: &[Vec<(f32, f32)>],
    depth: f32,
    cs: &crate::geometry::CoordinateSystem,
) -> Option<KernelSolid> {
    if points.len() < 3 || depth.abs() < f32::EPSILON {
        return None;
    }
    build_extrusion_solid(points, holes, depth as f64, cs)
        .or_else(|| build_extrusion_solid(points, &[], depth as f64, cs))
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
    (0..n)
        .map(|i| {
            let prev = (i + n - 1) % n;
            intersect(off_pt[prev], dir[prev], off_pt[i], dir[i]).unwrap_or(off_pt[i])
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
/// sketch plane, swept `depth` along the plane normal. Built from four quarter
/// arcs so the side is a single smooth cylindrical B-Rep face (not a prism).
fn oriented_cylinder_solid(
    cs: &crate::geometry::CoordinateSystem,
    cu: f32,
    cv: f32,
    r: f32,
    depth: f32,
) -> Option<KernelSolid> {
    use std::f64::consts::PI;
    let center = cs.unproject(cu, cv);
    // A point on the rim at angle `ang`, expressed in 3D via the plane axes.
    let rim = |ang: f64| -> Point3 {
        let (c, s) = (ang.cos() as f32, ang.sin() as f32);
        let p = center.add(cs.u.mul(r * c)).add(cs.v.mul(r * s));
        Point3::new(p.x as f64, p.y as f64, p.z as f64)
    };

    let v0 = builder::vertex(rim(0.0));
    let v1 = builder::vertex(rim(PI * 0.5));
    let v2 = builder::vertex(rim(PI));
    let v3 = builder::vertex(rim(PI * 1.5));
    let a1 = builder::circle_arc(&v0, &v1, rim(PI * 0.25));
    let a2 = builder::circle_arc(&v1, &v2, rim(PI * 0.75));
    let a3 = builder::circle_arc(&v2, &v3, rim(PI * 1.25));
    let a4 = builder::circle_arc(&v3, &v0, rim(PI * 1.75));

    let wire: Wire = vec![a1, a2, a3, a4].into_iter().collect();
    let face = builder::try_attach_plane(&[wire]).ok()?;
    let sweep = Vector3::new(
        cs.n.x as f64 * depth as f64,
        cs.n.y as f64 * depth as f64,
        cs.n.z as f64 * depth as f64,
    );
    Some(builder::tsweep(&face, sweep))
}

/// Run a kernel boolean, swallowing both a `None` result and any panic from
/// inside `truck` (its solver can panic outright on curved geometry — e.g. a
/// cylinder meeting a box). The panic hook is silenced for the duration so a
/// degenerate boolean doesn't spam the log on every drag frame. Either way the
/// caller just sees `None` and degrades gracefully instead of crashing the app.
fn guarded_boolean(op: impl FnOnce() -> Option<KernelSolid>) -> Option<KernelSolid> {
    let prev = std::panic::take_hook();
    std::panic::set_hook(Box::new(|_| {}));
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(op))
        .ok()
        .flatten();
    std::panic::set_hook(prev);
    result
}

/// Boolean union (`a ∪ b`). Returns `None` if the solver can't resolve (or
/// panics on) the configuration — callers decide how to degrade.
pub fn union(a: &KernelSolid, b: &KernelSolid) -> Option<KernelSolid> {
    guarded_boolean(|| truck_shapeops::or(a, b, BOOL_TOL))
}

/// Boolean difference (`a − b`): subtract `b`'s volume from `a`. Implemented as
/// `a ∩ complement(b)` by inverting `b`. Returns `None` on solver failure.
pub fn difference(a: &KernelSolid, b: &KernelSolid) -> Option<KernelSolid> {
    let mut inv = b.clone();
    inv.not();
    guarded_boolean(|| truck_shapeops::and(a, &inv, BOOL_TOL))
}

/// Axis-aligned bounding box of a solid from its B-Rep vertices, as
/// `(min, max)`. Exact for polygonal solids; a conservative-enough estimate for
/// curved ones (used only for cheap overlap pre-tests). `None` if vertexless.
pub fn solid_aabb(solid: &KernelSolid) -> Option<([f32; 3], [f32; 3])> {
    let mut min = [f32::INFINITY; 3];
    let mut max = [f32::NEG_INFINITY; 3];
    let mut any = false;
    for shell in solid.boundaries() {
        for face in shell.face_iter() {
            for wire in face.boundaries() {
                for edge in wire.edge_iter() {
                    for p in [edge.front().point(), edge.back().point()] {
                        any = true;
                        let c = [p.x as f32, p.y as f32, p.z as f32];
                        for k in 0..3 {
                            min[k] = min[k].min(c[k]);
                            max[k] = max[k].max(c[k]);
                        }
                    }
                }
            }
        }
    }
    any.then_some((min, max))
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
// truck Solid builders
// ---------------------------------------------------------------------------

fn build_cylinder_solid(r: f64, h: f64) -> Option<truck_modeling::Solid> {
    // Four quarter-arcs forming a circle in the XZ plane (perpendicular to Y).
    let half = std::f64::consts::FRAC_1_SQRT_2 * r;

    let v_px = builder::vertex(Point3::new(r, 0.0, 0.0));
    let v_pz = builder::vertex(Point3::new(0.0, 0.0, r));
    let v_nx = builder::vertex(Point3::new(-r, 0.0, 0.0));
    let v_nz = builder::vertex(Point3::new(0.0, 0.0, -r));

    let a1 = builder::circle_arc(&v_px, &v_pz, Point3::new(half, 0.0, half));
    let a2 = builder::circle_arc(&v_pz, &v_nx, Point3::new(-half, 0.0, half));
    let a3 = builder::circle_arc(&v_nx, &v_nz, Point3::new(-half, 0.0, -half));
    let a4 = builder::circle_arc(&v_nz, &v_px, Point3::new(half, 0.0, -half));

    let wire: Wire = vec![a1, a2, a3, a4].into_iter().collect();
    let face = builder::try_attach_plane(&[wire]).ok()?;
    Some(builder::tsweep(&face, Vector3::new(0.0, h, 0.0)))
}

fn build_extrusion_solid(
    points: &[(f32, f32)],
    holes: &[Vec<(f32, f32)>],
    depth: f64,
    cs: &crate::geometry::CoordinateSystem,
) -> Option<truck_modeling::Solid> {
    // truck's `tsweep` only yields an *outward-facing* solid — the orientation
    // its boolean solver needs — when the swept profile is wound CCW as seen
    // from the +normal side AND the sweep runs along that +normal, which holds
    // for a right-handed frame swept a positive depth.
    //
    // Two things flip the orientation, each independently turning the prism
    // inside-out:
    //   • a LEFT-handed frame (u × v = −n): the XZ/YZ origin-plane consts are
    //     left-handed, so a CCW profile there faces −n.
    //   • a NEGATIVE depth: the sweep runs opposite the plane normal.
    // Either alone inverts the solid; both together cancel out. Display always
    // survives (`enforce_outward_normals` re-signs render normals), but the
    // boolean solver consumes the raw solid: an inside-out tool makes `union`
    // (join) *subtract* the body and `difference` (cut) *add* the tool. XOR the
    // two conditions so the winding is reversed exactly when needed, leaving an
    // outward solid for any frame and either sweep direction.
    let left_handed = cs.u.cross(cs.v).dot(cs.n) < 0.0;
    let reverse_winding = left_handed ^ (depth < 0.0);

    let make_wire = |loop_pts: &[(f32, f32)], reverse: bool| -> Wire {
        let mut ordered: Vec<(f32, f32)> = loop_pts.to_vec();
        if reverse {
            ordered.reverse();
        }
        let verts: Vec<Vertex> = ordered
            .iter()
            .map(|(u, v)| {
                let p3 = cs.unproject(*u, *v);
                builder::vertex(Point3::new(p3.x as f64, p3.y as f64, p3.z as f64))
            })
            .collect();
        let n = verts.len();
        let edges: Vec<_> = (0..n)
            .map(|i| builder::line(&verts[i], &verts[(i + 1) % n]))
            .collect();
        edges.into_iter().collect()
    };

    // Outer boundary first; holes follow, wound opposite so they cut a pocket.
    // `reverse_winding` flips both together so the solid stays outward-facing
    // for any frame handedness and either sweep direction.
    let mut wires: Vec<Wire> = vec![make_wire(points, reverse_winding)];
    for hole in holes {
        if hole.len() < 3 {
            continue;
        }
        wires.push(make_wire(hole, !reverse_winding));
    }

    let face = builder::try_attach_plane(&wires).ok()?;
    let sweep = Vector3::new(
        cs.n.x as f64 * depth,
        cs.n.y as f64 * depth,
        cs.n.z as f64 * depth,
    );
    Some(builder::tsweep(&face, sweep))
}

// ---------------------------------------------------------------------------
// Tessellation → flat interleaved vertex buffer
// ---------------------------------------------------------------------------

fn solid_to_flat_mesh(solid: &truck_modeling::Solid) -> (Vec<f32>, Vec<u32>, Vec<u32>) {
    let meshed: Solid<Point3, _, Option<PolygonMesh>> = solid.triangulation(TESS_TOL);

    let mut vertices: Vec<f32> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();
    let mut face_ids: Vec<u32> = Vec::new();
    // One id per truck B-rep face, so all triangles of a face share an id.
    let mut next_face_id: u32 = 0;

    for shell in meshed.boundaries() {
        for face in shell.face_iter() {
            // Account for face orientation: tsweep can produce inverted shells.
            let flip = !face.orientation();
            if let Some(pm) = face.surface().as_ref() {
                let before = indices.len() / 3;
                append_polymesh(pm, flip, &mut vertices, &mut indices);
                let added = indices.len() / 3 - before;
                if added > 0 {
                    face_ids.extend(std::iter::repeat(next_face_id).take(added));
                    next_face_id += 1;
                }
            }
        }
    }

    // Normalize the whole shell to outward-facing normals. truck builds some
    // solids inside-out (e.g. extrusions on the left-handed XZ/YZ sketch
    // frames), which would otherwise make the renderer's back-face culling and
    // hidden-line removal disagree. A closed shell's normals are uniformly
    // oriented, so one centroid test decides the sign for the whole mesh.
    enforce_outward_normals(&mut vertices, &indices);

    // Smooth the normals across shallow creases so a faceted curved surface — a
    // fillet, or a boolean'd / many-sided extruded cylinder wall — shades as ONE
    // smooth face. Sharp features (90° box corners, 45° chamfers) meet past the
    // crease angle and keep distinct normals, so they stay crisp. Pristine flat
    // faces are unaffected (their normals already agree). Pairs with the
    // renderer's Gouraud (per-vertex) shading and `mesh_feature_edges`' matching
    // crease filter, which hides the facet-boundary lines.
    smooth_vertex_normals(&mut vertices, SHADE_CREASE_COS);

    (vertices, indices, face_ids)
}

/// `cos` of the crease angle (~30°) below which adjacent faces are treated as one
/// smooth surface for shading. Above it (chamfer bevels at 45°, box corners at
/// 90°) the crease is a real edge and the faces keep independent normals.
const SHADE_CREASE_COS: f32 = 0.866;

/// Replace each vertex normal with the average of the normals of all vertices
/// sharing its position whose normal lies within the crease angle (`crease_cos`).
/// This is per-vertex normal smoothing with a crease threshold: a fillet's facet
/// normals (a few degrees apart) blend into a smooth gradient, while a sharp edge
/// — whose two faces' normals diverge past the threshold — keeps each face's own
/// normal, so it still reads as an edge. Operates on the interleaved
/// `[x,y,z,nx,ny,nz]` buffer in place.
fn smooth_vertex_normals(vertices: &mut [f32], crease_cos: f32) {
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
    let mut groups: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
    for i in 0..vcount {
        groups.entry(key(i)).or_default().push(i);
    }
    let mut smoothed = vec![[0.0f32; 3]; vcount];
    for members in groups.values() {
        for &i in members {
            let ni = nrm(i);
            let (mut sx, mut sy, mut sz) = (0.0f32, 0.0f32, 0.0f32);
            for &j in members {
                let nj = nrm(j);
                if ni[0] * nj[0] + ni[1] * nj[1] + ni[2] * nj[2] >= crease_cos {
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

/// Flip every normal in `vertices` (interleaved pos+normal) if the shell's
/// normals point inward, judged by summing each triangle's normal against the
/// direction from the mesh centroid to that triangle.
fn enforce_outward_normals(vertices: &mut [f32], indices: &[u32]) {
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

    let mut orient = 0.0f32;
    for tri in indices.chunks_exact(3) {
        let i0 = tri[0] as usize * 6;
        let i1 = tri[1] as usize * 6;
        let i2 = tri[2] as usize * 6;
        let tcx = (vertices[i0] + vertices[i1] + vertices[i2]) / 3.0 - cx;
        let tcy = (vertices[i0 + 1] + vertices[i1 + 1] + vertices[i2 + 1]) / 3.0 - cy;
        let tcz = (vertices[i0 + 2] + vertices[i1 + 2] + vertices[i2 + 2]) / 3.0 - cz;
        orient += vertices[i0 + 3] * tcx + vertices[i0 + 4] * tcy + vertices[i0 + 5] * tcz;
    }

    if orient < 0.0 {
        for v in 0..vcount {
            vertices[v * 6 + 3] = -vertices[v * 6 + 3];
            vertices[v * 6 + 4] = -vertices[v * 6 + 4];
            vertices[v * 6 + 5] = -vertices[v * 6 + 5];
        }
    }
}

fn append_polymesh(pm: &PolygonMesh, flip: bool, vertices: &mut Vec<f32>, indices: &mut Vec<u32>) {
    let positions = pm.positions();
    let normals = pm.normals();
    let tri_faces = pm.tri_faces();

    if positions.is_empty() || tri_faces.is_empty() {
        return;
    }

    // Dedupe per (pos, normal) within this face. Reusing across faces would be
    // nice but seam normals differ — keeping faces independent is correct.
    let mut cache: HashMap<(usize, Option<usize>), u32> = HashMap::new();

    for tri in tri_faces {
        let mut local = [0u32; 3];
        for (slot, v) in tri.iter().enumerate() {
            let key = (v.pos, v.nor);
            let idx = *cache.entry(key).or_insert_with(|| {
                let p = &positions[v.pos];
                let n = match v.nor.and_then(|i| normals.get(i)) {
                    Some(n) => {
                        if flip {
                            [-n.x as f32, -n.y as f32, -n.z as f32]
                        } else {
                            [n.x as f32, n.y as f32, n.z as f32]
                        }
                    }
                    None => [0.0, 0.0, 1.0],
                };
                vertices.extend_from_slice(&[p.x as f32, p.y as f32, p.z as f32, n[0], n[1], n[2]]);
                (vertices.len() / 6) as u32 - 1
            });
            local[slot] = idx;
        }
        if flip {
            indices.extend_from_slice(&[local[0], local[2], local[1]]);
        } else {
            indices.extend_from_slice(&[local[0], local[1], local[2]]);
        }
    }
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

            for i in 0..n {
                // Bottom-loop edge: borders the bottom cap and side wall i.
                ei.push(bot_idx[i]);
                ei.push(bot_idx[(i + 1) % n]);
                push_n(efn, cap_bottom, wall[i]);

                // Top-loop edge: borders the top cap and side wall i.
                ei.push(top_idx[i]);
                ei.push(top_idx[(i + 1) % n]);
                push_n(efn, cap_top, wall[i]);

                // Vertical strut at vertex i: borders walls (i-1) and i.
                ei.push(bot_idx[i]);
                ei.push(top_idx[i]);
                push_n(efn, wall[(i + n - 1) % n], wall[i]);
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
fn mesh_feature_edges(
    vertices: &[f32],
    indices: &[u32],
    face_ids: &[u32],
) -> (Vec<f32>, Vec<u32>, Vec<f32>) {
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
    // A planar face's vertices share one normal, so the first vertex's is fine.
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

    let mut edge_vertices: Vec<f32> = Vec::new();
    let mut edge_indices: Vec<u32> = Vec::new();
    let mut edge_face_normals: Vec<f32> = Vec::new();
    for rec in edges.values() {
        // Skip internal diagonals: two triangles of the SAME face (one distinct
        // face id but two triangles). Keep creases (≥2 faces) and true mesh
        // boundary edges (a single triangle).
        if rec.faces.len() < 2 && rec.tris >= 2 {
            continue;
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
            if dot > CREASE_COS {
                continue;
            }
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
    }

    (edge_vertices, edge_indices, edge_face_normals)
}
