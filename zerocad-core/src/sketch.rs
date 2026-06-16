//! 2D sketch curves and planar region (face) detection.
//!
//! A sketch holds a collection of straight line segments and full circles
//! (drawn in 2D plane coordinates). When the sketch is finalized we compute
//! the *planar subdivision* of those curves and return the bounded faces
//! ("regions"). Each region is a closed CCW polygon the user can pick and
//! extrude — this is what makes intersecting shapes in Fusion 360 split
//! into independently selectable pieces.
//!
//! The algorithm is the standard half-edge / DCEL face traversal:
//!
//! 1. Discretize circles to polylines.
//! 2. Compute all pairwise segment intersections; snap-deduplicate vertices.
//! 3. Split each input segment at every intersection that lies on it.
//! 4. For each split sub-segment, emit two directed half-edges (twins).
//! 5. At each vertex, sort outgoing half-edges by polar angle.
//! 6. For half-edge `h` with destination `v` and twin at sorted index `i`,
//!    set `next(h)` to the outgoing half-edge at index `(i - 1) mod k` —
//!    the one immediately clockwise of `twin(h)`. Walking `h → next(h) → …`
//!    traces the face on the LEFT of `h`.
//! 7. Walk all half-edge cycles. Cycles with positive signed area are
//!    bounded faces; the one negative-area cycle is the outer face — drop it.
//!
//! For Phase 2a this only handles line segments and circles (no arcs yet).
//! Collinear/overlapping input segments are not split against each other —
//! sketch tools currently can't produce them in practice. Adding that and
//! arc support are tracked as follow-ups in the plan.

use std::collections::HashMap;

/// Vertex coordinate snap tolerance (in sketch plane units / mm).
const VERTEX_TOL: f64 = 1e-3;
/// Cross-product epsilon for "parallel" classification.
const PARALLEL_EPS: f64 = 1e-9;
/// Discretization for circles when computing planar arrangement. Shared crate
/// constant so sketch arrangement and the kernel's cylinder solids agree.
use crate::CIRCLE_SEGS;
/// Facet count for an ellipse drawn via [`SketchCurves::add_ellipse`] — matches
/// the circle discretization so the two read consistently.
const ELLIPSE_SEGS: usize = CIRCLE_SEGS;
/// Minimum area for a cycle to count as a real region (filters slivers).
const MIN_REGION_AREA: f64 = 1e-2;

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct LineSegment {
    pub a: (f32, f32),
    pub b: (f32, f32),
}

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Circle {
    pub center: (f32, f32),
    pub radius: f32,
}

#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SketchCurves {
    pub segments: Vec<LineSegment>,
    pub circles: Vec<Circle>,
}

impl SketchCurves {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn is_empty(&self) -> bool {
        self.segments.is_empty() && self.circles.is_empty()
    }

    /// Append a rectangle as four segments around the two opposite corners.
    pub fn add_rectangle(&mut self, p0: (f32, f32), p2: (f32, f32)) {
        let p1 = (p2.0, p0.1);
        let p3 = (p0.0, p2.1);
        self.segments.push(LineSegment { a: p0, b: p1 });
        self.segments.push(LineSegment { a: p1, b: p2 });
        self.segments.push(LineSegment { a: p2, b: p3 });
        self.segments.push(LineSegment { a: p3, b: p0 });
    }

    pub fn add_line(&mut self, a: (f32, f32), b: (f32, f32)) {
        self.segments.push(LineSegment { a, b });
    }

    pub fn add_circle(&mut self, center: (f32, f32), radius: f32) {
        if radius > 0.0 {
            self.circles.push(Circle { center, radius });
        }
    }

    /// Append an ellipse as a faceted closed polyline (the same polygon-
    /// approximation strategy circles/cylinders already use for the boolean
    /// kernel — there is no analytic ellipse primitive). `major` is the
    /// half-axis vector from the center to the end of the major axis (its length
    /// is the major radius and its direction sets the rotation); `minor_radius`
    /// is the perpendicular half-axis length. Emits [`ELLIPSE_SEGS`] segments.
    pub fn add_ellipse(&mut self, center: (f32, f32), major: (f32, f32), minor_radius: f32) {
        let rx = (major.0 * major.0 + major.1 * major.1).sqrt();
        if rx <= 1e-4 || minor_radius <= 1e-4 {
            return;
        }
        // Unit major axis and the unit minor axis (90° CCW from it).
        let (ux, uy) = (major.0 / rx, major.1 / rx);
        let (px, py) = (-uy, ux);
        let ry = minor_radius;
        let n = ELLIPSE_SEGS;
        let pts: Vec<(f32, f32)> = (0..n)
            .map(|k| {
                let t = (k as f32) / (n as f32) * std::f32::consts::TAU;
                let (ct, st) = (t.cos(), t.sin());
                (
                    center.0 + rx * ct * ux + ry * st * px,
                    center.1 + rx * ct * uy + ry * st * py,
                )
            })
            .collect();
        for k in 0..n {
            self.segments.push(LineSegment {
                a: pts[k],
                b: pts[(k + 1) % n],
            });
        }
    }

    /// Remove the most recently added primitive (LIFO across segments + circles).
    /// Rectangles count as 4 segments — call 4× to undo one.
    pub fn pop_last(&mut self) -> bool {
        if let Some(_) = self.circles.pop() {
            true
        } else if let Some(_) = self.segments.pop() {
            true
        } else {
            false
        }
    }
}

// ---------------------------------------------------------------------------
// Parametric sketch shapes
// ---------------------------------------------------------------------------

/// A sketch dimension that may be a literal or an expression over the document's
/// variables. `value` is the resolved fallback (the value drawn / last known);
/// `expr`, when set, is re-evaluated against the current variables every build,
/// so editing the variable updates the sketch.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Dimension {
    pub value: f32,
    #[serde(default)]
    pub expr: Option<String>,
}

impl Dimension {
    /// A plain literal dimension (no variable binding).
    pub fn literal(value: f32) -> Self {
        Self { value, expr: None }
    }

    /// Resolve to a number in base units (mm): the expression if it still
    /// evaluates, otherwise the stored fallback value.
    pub fn resolve(&self, vars: &HashMap<String, f64>) -> f32 {
        self.expr
            .as_ref()
            .and_then(|e| crate::expr::eval(e, vars).ok())
            .map(|v| v as f32)
            .unwrap_or(self.value)
    }
}

/// A parametric primitive in a sketch: the construction (anchor points + named
/// dimensions) rather than baked coordinates, so it can be rebuilt against the
/// current variables. Tools without dimension fields (3-point shapes, ellipses)
/// are stored pre-built as [`SketchShape::Raw`].
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum SketchShape {
    /// Axis-aligned rectangle. When `from_center`, `origin` is the center and
    /// `w`/`h` are full extents; otherwise `origin` is a corner and the opposite
    /// corner is `origin + (sx·w, sy·h)` (`sx`/`sy` record the drawn direction).
    Rectangle {
        origin: (f32, f32),
        sx: f32,
        sy: f32,
        w: Dimension,
        h: Dimension,
        from_center: bool,
    },
    Circle {
        center: (f32, f32),
        diameter: Dimension,
    },
    Line {
        start: (f32, f32),
        length: Dimension,
        angle_deg: Dimension,
    },
    /// Pre-built geometry with no variable bindings (3-point rect/circle,
    /// ellipses). Stored as-is and emitted verbatim.
    Raw {
        curves: SketchCurves,
    },
}

impl SketchShape {
    /// Build this shape's curves, resolving any dimension expressions against
    /// `vars` (variable values in base units / mm).
    pub fn build(&self, vars: &HashMap<String, f64>) -> SketchCurves {
        let mut c = SketchCurves::new();
        match self {
            SketchShape::Rectangle {
                origin,
                sx,
                sy,
                w,
                h,
                from_center,
            } => {
                let (wv, hv) = (w.resolve(vars), h.resolve(vars));
                if *from_center {
                    let (hw, hh) = (wv * 0.5, hv * 0.5);
                    c.add_rectangle(
                        (origin.0 - hw, origin.1 - hh),
                        (origin.0 + hw, origin.1 + hh),
                    );
                } else {
                    c.add_rectangle(*origin, (origin.0 + sx * wv, origin.1 + sy * hv));
                }
            }
            SketchShape::Circle { center, diameter } => {
                c.add_circle(*center, (diameter.resolve(vars) * 0.5).max(0.0));
            }
            SketchShape::Line {
                start,
                length,
                angle_deg,
            } => {
                let len = length.resolve(vars);
                let ang = angle_deg.resolve(vars).to_radians();
                c.add_line(*start, (start.0 + len * ang.cos(), start.1 + len * ang.sin()));
            }
            SketchShape::Raw { curves } => c = curves.clone(),
        }
        c
    }
}

/// Build the full set of sketch curves from a parametric shape list, resolving
/// every dimension expression against `vars`. This is the single source of
/// truth for a parametric sketch's geometry (region detection, rendering, and
/// extrusion all consume the result).
pub fn build_sketch_curves(shapes: &[SketchShape], vars: &HashMap<String, f64>) -> SketchCurves {
    let mut out = SketchCurves::new();
    for s in shapes {
        let c = s.build(vars);
        out.segments.extend(c.segments);
        out.circles.extend(c.circles);
    }
    out
}

/// Whether a corner modifier rounds the corner (fillet) or bevels it (chamfer).
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum CornerKind {
    Fillet,
    Chamfer,
}

/// A fillet/chamfer applied to a sketch corner — the vertex where two segments
/// meet. Stored at the sketch level (the corner may join segments from different
/// shapes) and applied after the shapes are built, so the underlying shapes stay
/// parametric. `radius` may itself be variable-bound.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CornerMod {
    /// The corner location in sketch coordinates, snapped to the nearest shared
    /// vertex when applied (so it survives the geometry being rebuilt).
    pub at: (f32, f32),
    pub radius: Dimension,
    pub kind: CornerKind,
}

/// How many straight segments to approximate an arc of `angle` radians and
/// radius `r` so it reads as a *smooth* curve. Combines an angular budget
/// (~3.6°/segment, so the polyline corners are imperceptible at any size) with a
/// chord-tolerance budget (finer for large radii, where 3.6° would still bow
/// visibly), clamped to a sane range so tiny fillets stay cheap and huge ones
/// don't explode region detection.
fn arc_segments(angle: f32, r: f32) -> usize {
    let angle = angle.abs();
    // ~3.6° per segment.
    let by_angle = (angle / 0.063).ceil() as usize;
    // Chord error ≤ 0.01mm: each step subtends ≤ 2·acos(1 − tol/r).
    let tol = 0.01_f32;
    let by_chord = if r > tol {
        let step = 2.0 * (1.0 - tol / r).clamp(-1.0, 1.0).acos();
        if step > 1.0e-4 {
            (angle / step).ceil() as usize
        } else {
            2
        }
    } else {
        2
    };
    by_angle.max(by_chord).clamp(6, 160)
}

/// The live geometry of a sketch: rebuilt from its parametric `shapes` against
/// `vars` when present (else the baked `curves` for legacy documents), then with
/// any fillet/chamfer `corner_mods` applied.
pub fn effective_curves(
    curves: &SketchCurves,
    shapes: &[SketchShape],
    corner_mods: &[CornerMod],
    vars: &HashMap<String, f64>,
) -> SketchCurves {
    let mut c = if shapes.is_empty() {
        curves.clone()
    } else {
        build_sketch_curves(shapes, vars)
    };
    for m in corner_mods {
        let r = m.radius.resolve(vars);
        apply_corner_mod(&mut c, m.at, r, m.kind);
    }
    c
}

fn dist2(a: (f32, f32), b: (f32, f32)) -> f32 {
    let (dx, dy) = (a.0 - b.0, a.1 - b.1);
    dx * dx + dy * dy
}

/// Round or bevel the corner of `curves` nearest `at`. Snaps `at` to the closest
/// segment endpoint, finds the two segments meeting there, trims them back, and
/// inserts a faceted arc (fillet) or a straight bevel (chamfer). No-ops unless
/// exactly two segments share the corner and the radius is usable.
fn apply_corner_mod(curves: &mut SketchCurves, at: (f32, f32), radius: f32, kind: CornerKind) {
    if radius <= 1e-4 {
        return;
    }
    // Snap to the nearest existing vertex.
    let mut v = None;
    let mut best = f32::MAX;
    for s in &curves.segments {
        for &p in &[s.a, s.b] {
            let d = dist2(p, at);
            if d < best {
                best = d;
                v = Some(p);
            }
        }
    }
    let Some(v) = v else {
        return;
    };

    // The (at most two) segments touching that vertex, with which endpoint.
    const TOL2: f32 = 1e-6;
    let mut touching: Vec<(usize, bool)> = Vec::new(); // (index, endpoint_is_a)
    for (i, s) in curves.segments.iter().enumerate() {
        if dist2(s.a, v) < TOL2 {
            touching.push((i, true));
        } else if dist2(s.b, v) < TOL2 {
            touching.push((i, false));
        }
    }
    if touching.len() != 2 {
        return;
    }
    let (i1, a1) = touching[0];
    let (i2, a2) = touching[1];
    let far1 = if a1 {
        curves.segments[i1].b
    } else {
        curves.segments[i1].a
    };
    let far2 = if a2 {
        curves.segments[i2].b
    } else {
        curves.segments[i2].a
    };

    let len1 = dist2(far1, v).sqrt();
    let len2 = dist2(far2, v).sqrt();
    if len1 < 1e-4 || len2 < 1e-4 {
        return;
    }
    let u = ((far1.0 - v.0) / len1, (far1.1 - v.1) / len1);
    let w = ((far2.0 - v.0) / len2, (far2.1 - v.1) / len2);
    let cos_t = (u.0 * w.0 + u.1 * w.1).clamp(-1.0, 1.0);
    let theta = cos_t.acos();
    if theta < 1e-3 || (std::f32::consts::PI - theta) < 1e-3 {
        return; // collinear — nothing to round
    }
    let half = theta * 0.5;
    // Setback distance along each edge; shrink the radius if it won't fit.
    let mut t = radius / half.tan();
    let max_t = len1.min(len2) * 0.95;
    let radius = if t > max_t {
        t = max_t;
        max_t * half.tan()
    } else {
        radius
    };
    let p1 = (v.0 + u.0 * t, v.1 + u.1 * t);
    let p2 = (v.0 + w.0 * t, v.1 + w.1 * t);

    // Trim the two segments back to the setback points.
    if a1 {
        curves.segments[i1].a = p1;
    } else {
        curves.segments[i1].b = p1;
    }
    if a2 {
        curves.segments[i2].a = p2;
    } else {
        curves.segments[i2].b = p2;
    }

    match kind {
        CornerKind::Chamfer => {
            curves.segments.push(LineSegment { a: p1, b: p2 });
        }
        CornerKind::Fillet => {
            // Arc center sits along the angle bisector, distance r/sin(half).
            let bl = ((u.0 + w.0).powi(2) + (u.1 + w.1).powi(2)).sqrt();
            if bl < 1e-5 {
                curves.segments.push(LineSegment { a: p1, b: p2 });
                return;
            }
            let bis = ((u.0 + w.0) / bl, (u.1 + w.1) / bl);
            let cd = radius / half.sin();
            let c = (v.0 + bis.0 * cd, v.1 + bis.1 * cd);
            push_arc(curves, c, p1, p2, radius);
        }
    }
}

/// Append an arc from `p1` to `p2` about center `c` (radius `r`), taking the
/// short way around. Used to draw a fillet. The arc is tessellated finely enough
/// (see [`arc_segments`]) to read as a smooth curve rather than a few flats.
fn push_arc(curves: &mut SketchCurves, c: (f32, f32), p1: (f32, f32), p2: (f32, f32), r: f32) {
    use std::f32::consts::{PI, TAU};
    let a0 = (p1.1 - c.1).atan2(p1.0 - c.0);
    let a1 = (p2.1 - c.1).atan2(p2.0 - c.0);
    let mut da = a1 - a0;
    while da > PI {
        da -= TAU;
    }
    while da < -PI {
        da += TAU;
    }
    let n = arc_segments(da.abs(), r);
    let mut prev = p1;
    for k in 1..=n {
        let a = a0 + da * (k as f32 / n as f32);
        let pt = (c.0 + r * a.cos(), c.1 + r * a.sin());
        curves.segments.push(LineSegment { a: prev, b: pt });
        prev = pt;
    }
}

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Region {
    /// Closed CCW outer boundary polygon in sketch plane coordinates.
    pub boundary: Vec<(f32, f32)>,
    /// Inner boundaries (holes) — e.g. a shape drawn fully inside this face.
    /// Each is a closed loop; the face is the outer area minus the holes.
    #[serde(default)]
    pub holes: Vec<Vec<(f32, f32)>>,
    /// Net area: outer boundary area minus the area of the holes.
    pub area: f32,
}

impl Region {
    /// True if `p` is inside this face: within the outer boundary and not in
    /// any hole.
    pub fn contains(&self, p: (f32, f32)) -> bool {
        if !point_in_polygon(p, &self.boundary) {
            return false;
        }
        !self.holes.iter().any(|h| point_in_polygon(p, h))
    }
}

// ---------------------------------------------------------------------------
// Region detection
// ---------------------------------------------------------------------------

#[derive(Clone, Copy)]
struct P {
    x: f64,
    y: f64,
}

impl P {
    fn from(p: (f32, f32)) -> Self {
        Self {
            x: p.0 as f64,
            y: p.1 as f64,
        }
    }
}

pub fn detect_regions(curves: &SketchCurves) -> Vec<Region> {
    let raw_segs = flatten_curves(curves);
    if raw_segs.is_empty() {
        return Vec::new();
    }

    let mut vertices: Vec<P> = Vec::new();
    let split = split_at_intersections(&raw_segs, &mut vertices);
    if split.is_empty() {
        return Vec::new();
    }

    // Deduplicate undirected sub-segments — collinear/duplicate input would
    // otherwise yield bogus zero-area cycles. Key by sorted endpoint indices.
    let mut seen: HashMap<(usize, usize), ()> = HashMap::new();
    let mut unique: Vec<(usize, usize)> = Vec::new();
    for (u, v) in split {
        if u == v {
            continue;
        }
        let key = if u < v { (u, v) } else { (v, u) };
        if seen.insert(key, ()).is_none() {
            unique.push((u, v));
        }
    }

    let half_edges = build_half_edges(&unique);
    let next = compute_next_pointers(&half_edges, &vertices);
    let cycles = walk_cycles(&half_edges, &next);

    let mut regions: Vec<Region> = Vec::new();
    for cycle in cycles {
        let pts: Vec<(f32, f32)> = cycle
            .iter()
            .map(|&h| {
                let v = vertices[half_edges[h].from];
                (v.x as f32, v.y as f32)
            })
            .collect();
        let area = signed_area_f64(&cycle, &half_edges, &vertices);
        if area > MIN_REGION_AREA {
            regions.push(Region {
                boundary: pts,
                holes: Vec::new(),
                area: area as f32,
            });
        }
    }

    assign_holes(&mut regions);
    regions
}

/// Turn nesting into holes. When one face lies fully inside another (a shape
/// drawn inside another, with no intersecting edges), the inner face becomes a
/// hole of its immediate (smallest) container, and the container's area is
/// reduced accordingly. Every face is kept — so a circle with a rectangle drawn
/// inside it yields BOTH the inner rectangle face AND the annular face around it
/// (rather than the annulus vanishing).
///
/// Adjacent faces produced by intersecting shapes share edges, so neither
/// contains the other's interior point — they are never turned into holes.
fn assign_holes(regions: &mut [Region]) {
    let n = regions.len();
    if n < 2 {
        return;
    }

    // A point guaranteed strictly inside each face's outer boundary.
    let interior_points: Vec<(f32, f32)> = regions
        .iter()
        .map(|r| polygon_interior_point(&r.boundary))
        .collect();
    // Gross outer-boundary areas (before hole subtraction).
    let gross: Vec<f64> = regions.iter().map(|r| r.area as f64).collect();

    // Immediate parent = smallest face that strictly contains this face.
    let mut parent: Vec<Option<usize>> = vec![None; n];
    for j in 0..n {
        let mut best: Option<usize> = None;
        for i in 0..n {
            if i == j {
                continue;
            }
            if gross[i] > gross[j] + MIN_REGION_AREA
                && point_in_polygon(interior_points[j], &regions[i].boundary)
            {
                best = match best {
                    None => Some(i),
                    Some(b) if gross[i] < gross[b] => Some(i),
                    other => other,
                };
            }
        }
        parent[j] = best;
    }

    let boundaries: Vec<Vec<(f32, f32)>> = regions.iter().map(|r| r.boundary.clone()).collect();
    for j in 0..n {
        if let Some(p) = parent[j] {
            regions[p].holes.push(boundaries[j].clone());
            regions[p].area -= gross[j] as f32;
        }
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn flatten_curves(curves: &SketchCurves) -> Vec<(P, P)> {
    let mut out: Vec<(P, P)> = Vec::new();
    for s in &curves.segments {
        let a = P::from(s.a);
        let b = P::from(s.b);
        if (a.x - b.x).abs() > VERTEX_TOL || (a.y - b.y).abs() > VERTEX_TOL {
            out.push((a, b));
        }
    }
    for c in &curves.circles {
        let cx = c.center.0 as f64;
        let cy = c.center.1 as f64;
        let r = c.radius as f64;
        let mut prev = P { x: cx + r, y: cy };
        for i in 1..=CIRCLE_SEGS {
            let theta = (i as f64 / CIRCLE_SEGS as f64) * std::f64::consts::TAU;
            let cur = P {
                x: cx + r * theta.cos(),
                y: cy + r * theta.sin(),
            };
            out.push((prev, cur));
            prev = cur;
        }
    }
    out
}

fn add_vertex(p: P, vertices: &mut Vec<P>) -> usize {
    for (i, q) in vertices.iter().enumerate() {
        if (p.x - q.x).abs() < VERTEX_TOL && (p.y - q.y).abs() < VERTEX_TOL {
            return i;
        }
    }
    vertices.push(p);
    vertices.len() - 1
}

/// For each input segment, compute all interior intersection points with
/// every other segment, sort them along the segment, and emit consecutive
/// sub-segments as index pairs into the vertex pool.
fn split_at_intersections(raw: &[(P, P)], vertices: &mut Vec<P>) -> Vec<(usize, usize)> {
    let mut out: Vec<(usize, usize)> = Vec::new();

    for (i, &(a, b)) in raw.iter().enumerate() {
        // (t-param-on-AB, vertex_index) for each split point including the endpoints.
        let mut pts: Vec<(f64, usize)> = Vec::new();
        pts.push((0.0, add_vertex(a, vertices)));
        pts.push((1.0, add_vertex(b, vertices)));

        for (j, &(c, d)) in raw.iter().enumerate() {
            if i == j {
                continue;
            }
            if let Some((t, _u, p)) = intersect(a, b, c, d) {
                if t > VERTEX_TOL && t < 1.0 - VERTEX_TOL {
                    let idx = add_vertex(p, vertices);
                    pts.push((t, idx));
                }
            }
        }

        pts.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap_or(std::cmp::Ordering::Equal));
        // Emit consecutive pairs, skipping duplicate vertex indices.
        for w in pts.windows(2) {
            let (_, u) = w[0];
            let (_, v) = w[1];
            if u != v {
                out.push((u, v));
            }
        }
    }
    out
}

fn intersect(a: P, b: P, c: P, d: P) -> Option<(f64, f64, P)> {
    let rx = b.x - a.x;
    let ry = b.y - a.y;
    let sx = d.x - c.x;
    let sy = d.y - c.y;
    let denom = rx * sy - ry * sx;
    if denom.abs() < PARALLEL_EPS {
        return None;
    }
    let qpx = c.x - a.x;
    let qpy = c.y - a.y;
    let t = (qpx * sy - qpy * sx) / denom;
    let u = (qpx * ry - qpy * rx) / denom;
    let tol = 1e-7;
    if t >= -tol && t <= 1.0 + tol && u >= -tol && u <= 1.0 + tol {
        Some((
            t,
            u,
            P {
                x: a.x + t * rx,
                y: a.y + t * ry,
            },
        ))
    } else {
        None
    }
}

#[derive(Debug, Clone, Copy)]
struct HEdge {
    from: usize,
    to: usize,
    twin: usize,
}

fn build_half_edges(undirected: &[(usize, usize)]) -> Vec<HEdge> {
    let mut he = Vec::with_capacity(undirected.len() * 2);
    for &(u, v) in undirected {
        let i = he.len();
        he.push(HEdge {
            from: u,
            to: v,
            twin: i + 1,
        });
        he.push(HEdge {
            from: v,
            to: u,
            twin: i,
        });
    }
    he
}

fn compute_next_pointers(half_edges: &[HEdge], vertices: &[P]) -> Vec<usize> {
    // outgoing[v] = sorted list of half-edge indices originating at v
    let mut outgoing: Vec<Vec<usize>> = vec![Vec::new(); vertices.len()];
    for (i, h) in half_edges.iter().enumerate() {
        outgoing[h.from].push(i);
    }
    for v in 0..vertices.len() {
        let p = vertices[v];
        outgoing[v].sort_by(|&a, &b| {
            let pa = vertices[half_edges[a].to];
            let pb = vertices[half_edges[b].to];
            let ang_a = (pa.y - p.y).atan2(pa.x - p.x);
            let ang_b = (pb.y - p.y).atan2(pb.x - p.x);
            ang_a
                .partial_cmp(&ang_b)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
    }

    // pos_in_outgoing[h] = index of h within outgoing[h.from]
    let mut pos_in_outgoing = vec![0usize; half_edges.len()];
    for v in 0..vertices.len() {
        for (idx, &h) in outgoing[v].iter().enumerate() {
            pos_in_outgoing[h] = idx;
        }
    }

    let mut next = vec![0usize; half_edges.len()];
    for h in 0..half_edges.len() {
        let v_dest = half_edges[h].to;
        let twin = half_edges[h].twin;
        let i = pos_in_outgoing[twin];
        let k = outgoing[v_dest].len();
        let prev_i = (i + k - 1) % k;
        next[h] = outgoing[v_dest][prev_i];
    }
    next
}

fn walk_cycles(half_edges: &[HEdge], next: &[usize]) -> Vec<Vec<usize>> {
    let mut visited = vec![false; half_edges.len()];
    let mut cycles: Vec<Vec<usize>> = Vec::new();
    for start in 0..half_edges.len() {
        if visited[start] {
            continue;
        }
        let mut cycle = Vec::new();
        let mut cur = start;
        loop {
            if visited[cur] {
                break;
            }
            visited[cur] = true;
            cycle.push(cur);
            cur = next[cur];
            if cur == start {
                break;
            }
        }
        if !cycle.is_empty() {
            cycles.push(cycle);
        }
    }
    cycles
}

fn signed_area_f64(cycle: &[usize], he: &[HEdge], vertices: &[P]) -> f64 {
    let mut sum = 0.0;
    for &h in cycle {
        let p = vertices[he[h].from];
        let q = vertices[he[h].to];
        sum += p.x * q.y - q.x * p.y;
    }
    sum * 0.5
}

fn centroid(poly: &[(f32, f32)]) -> (f32, f32) {
    let (mut sx, mut sy) = (0.0f32, 0.0f32);
    for p in poly {
        sx += p.0;
        sy += p.1;
    }
    let n = poly.len() as f32;
    (sx / n, sy / n)
}

/// A point guaranteed to lie strictly inside a simple (possibly concave)
/// polygon. The vertex centroid can fall outside a concave polygon, so we find
/// an "ear" (a convex corner whose triangle contains no other vertex) and
/// return that triangle's centroid — which is always interior. Falls back to
/// the vertex centroid only for degenerate input.
fn polygon_interior_point(poly: &[(f32, f32)]) -> (f32, f32) {
    let n = poly.len();
    if n < 3 {
        return centroid(poly);
    }

    let cross = |o: (f32, f32), a: (f32, f32), b: (f32, f32)| -> f64 {
        (a.0 - o.0) as f64 * (b.1 - o.1) as f64 - (a.1 - o.1) as f64 * (b.0 - o.0) as f64
    };
    let in_tri = |p: (f32, f32), a: (f32, f32), b: (f32, f32), c: (f32, f32)| -> bool {
        let d1 = cross(a, b, p);
        let d2 = cross(b, c, p);
        let d3 = cross(c, a, p);
        let has_neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
        let has_pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
        !(has_neg && has_pos)
    };

    // Work on a CCW copy so a convex corner has positive cross product.
    let mut order: Vec<usize> = (0..n).collect();
    let mut area2 = 0.0f64;
    for i in 0..n {
        let (ax, ay) = poly[i];
        let (bx, by) = poly[(i + 1) % n];
        area2 += ax as f64 * by as f64 - bx as f64 * ay as f64;
    }
    if area2 < 0.0 {
        order.reverse();
    }

    let m = order.len();
    for i in 0..m {
        let a = poly[order[(i + m - 1) % m]];
        let b = poly[order[i]];
        let c = poly[order[(i + 1) % m]];
        if cross(a, b, c) <= 0.0 {
            continue; // reflex (or collinear) corner — not an ear
        }
        let mut contains = false;
        for &k in &order {
            let pk = poly[k];
            if pk == a || pk == b || pk == c {
                continue;
            }
            if in_tri(pk, a, b, c) {
                contains = true;
                break;
            }
        }
        if contains {
            continue;
        }
        return ((a.0 + b.0 + c.0) / 3.0, (a.1 + b.1 + c.1) / 3.0);
    }

    centroid(poly)
}

pub fn point_in_polygon(p: (f32, f32), poly: &[(f32, f32)]) -> bool {
    let mut inside = false;
    let n = poly.len();
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = poly[i];
        let (xj, yj) = poly[j];
        let intersect_y = (yi > p.1) != (yj > p.1)
            && (p.0) < (xj - xi) * (p.1 - yi) / ((yj - yi) + f32::EPSILON) + xi;
        if intersect_y {
            inside = !inside;
        }
        j = i;
    }
    inside
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() < tol
    }

    #[test]
    fn single_rectangle_one_region() {
        let mut c = SketchCurves::new();
        c.add_rectangle((0.0, 0.0), (10.0, 8.0));
        let regions = detect_regions(&c);
        assert_eq!(
            regions.len(),
            1,
            "single rect should give one region, got {:?}",
            regions
        );
        assert!(approx(regions[0].area, 80.0, 0.1));
    }

    #[test]
    fn axis_aligned_ellipse_is_one_region_with_right_area() {
        // A faceted ellipse (rx=10, ry=5) should close into one region whose
        // area is close to π·rx·ry (a 48-gon slightly under-estimates it).
        let mut c = SketchCurves::new();
        c.add_ellipse((0.0, 0.0), (10.0, 0.0), 5.0);
        assert_eq!(c.segments.len(), 48, "ellipse should emit 48 facets");
        let regions = detect_regions(&c);
        assert_eq!(regions.len(), 1, "ellipse should be one closed region");
        let expected = std::f32::consts::PI * 10.0 * 5.0;
        assert!(
            (regions[0].area - expected).abs() < expected * 0.02,
            "ellipse area {} should be within 2% of {}",
            regions[0].area,
            expected
        );
    }

    #[test]
    fn rotated_ellipse_closes_into_one_region() {
        // Major axis at 45°, so the polyline is genuinely rotated.
        let mut c = SketchCurves::new();
        c.add_ellipse((3.0, 3.0), (7.07, 7.07), 4.0);
        let regions = detect_regions(&c);
        assert_eq!(regions.len(), 1, "rotated ellipse should still be one region");
    }

    #[test]
    fn fillet_and_chamfer_round_a_square_corner() {
        use std::collections::HashMap;
        let vars: HashMap<String, f64> = HashMap::new();
        let square = vec![SketchShape::Rectangle {
            origin: (0.0, 0.0),
            sx: 1.0,
            sy: 1.0,
            w: Dimension::literal(10.0),
            h: Dimension::literal(10.0),
            from_center: false,
        }];

        // Baseline: sharp square, area 100, one region.
        let base = effective_curves(&SketchCurves::new(), &square, &[], &vars);
        let base_area = detect_regions(&base)[0].area;
        assert!(approx(base_area, 100.0, 0.1));

        // Fillet the (0,0) corner with r=2 — one region, slightly less area.
        let fillet = CornerMod {
            at: (0.0, 0.0),
            radius: Dimension::literal(2.0),
            kind: CornerKind::Fillet,
        };
        let filleted = effective_curves(&SketchCurves::new(), &square, &[fillet], &vars);
        let regions = detect_regions(&filleted);
        assert_eq!(regions.len(), 1, "filleted square is still one region");
        let fa = regions[0].area;
        assert!(
            fa < base_area && fa > base_area - 2.0,
            "fillet trims a small bite: base={base_area} filleted={fa}"
        );

        // Chamfer removes a 45° triangle of area r²/2·tan(45)=2 at a right angle.
        let chamfer = CornerMod {
            at: (0.0, 0.0),
            radius: Dimension::literal(2.0),
            kind: CornerKind::Chamfer,
        };
        let chamfered = effective_curves(&SketchCurves::new(), &square, &[chamfer], &vars);
        let cregions = detect_regions(&chamfered);
        assert_eq!(cregions.len(), 1, "chamfered square is still one region");
        assert!(cregions[0].area < base_area);
    }

    #[test]
    fn degenerate_ellipse_adds_nothing() {
        let mut c = SketchCurves::new();
        c.add_ellipse((0.0, 0.0), (0.0, 0.0), 5.0); // zero major axis
        c.add_ellipse((0.0, 0.0), (10.0, 0.0), 0.0); // zero minor radius
        assert!(c.segments.is_empty(), "degenerate ellipses must be skipped");
    }

    #[test]
    fn two_disjoint_rectangles_two_regions() {
        let mut c = SketchCurves::new();
        c.add_rectangle((0.0, 0.0), (10.0, 10.0));
        c.add_rectangle((20.0, 0.0), (30.0, 10.0));
        let regions = detect_regions(&c);
        assert_eq!(regions.len(), 2);
        let total: f32 = regions.iter().map(|r| r.area).sum();
        assert!(approx(total, 200.0, 0.1));
    }

    #[test]
    fn overlapping_rectangles_split_into_three_regions() {
        // Two unit squares overlapping in a 5x10 strip → 3 regions:
        // left-only, overlap, right-only.
        let mut c = SketchCurves::new();
        c.add_rectangle((0.0, 0.0), (10.0, 10.0));
        c.add_rectangle((5.0, 0.0), (15.0, 10.0));
        let regions = detect_regions(&c);
        assert_eq!(
            regions.len(),
            3,
            "expected 3 sub-regions, got {:?}",
            regions
        );
        // Each sub-region should be 50 area; total = 150
        let mut areas: Vec<f32> = regions.iter().map(|r| r.area).collect();
        areas.sort_by(|a, b| a.partial_cmp(b).unwrap());
        for a in &areas {
            assert!(approx(*a, 50.0, 0.5), "region area {} not ~50", a);
        }
    }

    #[test]
    fn circle_alone_one_region() {
        let mut c = SketchCurves::new();
        c.add_circle((0.0, 0.0), 5.0);
        let regions = detect_regions(&c);
        assert_eq!(regions.len(), 1);
        // Polygon approximation of πr² ≈ 78.5
        assert!(regions[0].area > 70.0 && regions[0].area < 80.0);
    }

    #[test]
    fn circle_inside_rectangle_makes_annulus_and_inner_face() {
        // No edge intersections → two disjoint bounded cycles. With hole support
        // we keep BOTH faces: the inner circle, and the rectangle-with-a-circular
        // hole (the annulus) around it — instead of the outer face vanishing.
        let mut c = SketchCurves::new();
        c.add_rectangle((-10.0, -10.0), (10.0, 10.0)); // area 400
        c.add_circle((0.0, 0.0), 5.0); // area ≈ 78.5
        let regions = detect_regions(&c);
        assert_eq!(regions.len(), 2, "nested rect+circle gave: {:?}", regions);

        let annulus = regions
            .iter()
            .find(|r| !r.holes.is_empty())
            .expect("expected one face with a hole");
        let inner = regions
            .iter()
            .find(|r| r.holes.is_empty())
            .expect("expected one face without holes");

        assert_eq!(annulus.holes.len(), 1);
        assert!(
            approx(annulus.area, 400.0 - 78.5, 6.0),
            "annulus net area {} should be ~321.5",
            annulus.area
        );
        assert!(
            inner.area > 70.0 && inner.area < 82.0,
            "inner circle area {} should be ~78.5",
            inner.area
        );

        // The inner circle's centre is in the inner face, not the annulus.
        assert!(inner.contains((0.0, 0.0)));
        assert!(!annulus.contains((0.0, 0.0)));
        // A point near the rectangle corner is in the annulus, not the inner.
        assert!(annulus.contains((9.0, 9.0)));
        assert!(!inner.contains((9.0, 9.0)));
    }

    #[test]
    fn interior_point_is_inside_concave_polygon() {
        // A chevron whose vertex centroid falls OUTSIDE the polygon (in the
        // notch). This is exactly the situation that made the shell filter drop
        // a valid neighbouring face. `polygon_interior_point` must return a
        // point that is actually inside.
        let poly = vec![(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (2.0, 1.0), (0.0, 4.0)];
        let vc = centroid(&poly);
        assert!(
            !point_in_polygon(vc, &poly),
            "test premise: vertex centroid {:?} should be outside the chevron",
            vc
        );
        let ip = polygon_interior_point(&poly);
        assert!(
            point_in_polygon(ip, &poly),
            "interior point {:?} must be inside the chevron",
            ip
        );
    }

    #[test]
    fn circle_overlapping_two_rectangles_keeps_all_faces() {
        // Reproduces the reported case: two overlapping rectangles plus a circle
        // crossing both. The detected faces tile the union of the shapes, so the
        // sum of their areas must equal the union's area. A missing face (the
        // reported bug) would make the total fall short.
        let mut c = SketchCurves::new();
        c.add_rectangle((-6.0, -2.0), (4.0, 10.0));
        c.add_rectangle((0.0, -8.0), (12.0, 4.0));
        c.add_circle((2.0, 0.0), 6.0);
        let regions = detect_regions(&c);
        assert!(!regions.is_empty());

        let in_union = |x: f32, y: f32| -> bool {
            let in_r1 = x >= -6.0 && x <= 4.0 && y >= -2.0 && y <= 10.0;
            let in_r2 = x >= 0.0 && x <= 12.0 && y >= -8.0 && y <= 4.0;
            let in_c = (x - 2.0).powi(2) + (y - 0.0).powi(2) <= 6.0 * 6.0;
            in_r1 || in_r2 || in_c
        };

        // Monte-Carlo-on-a-grid estimate of the union area.
        let cell = 0.1f32;
        let mut area_union = 0.0f32;
        let mut x = -8.0;
        while x <= 14.0 {
            let mut y = -10.0;
            while y <= 12.0 {
                if in_union(x, y) {
                    area_union += cell * cell;
                }
                y += cell;
            }
            x += cell;
        }

        let area_regions: f32 = regions.iter().map(|r| r.area).sum();
        let tol = 0.05 * area_union; // grid + circle-polygon discretisation slack
        assert!(
            (area_regions - area_union).abs() < tol,
            "regions should tile the union: sum(region areas)={:.1} vs union≈{:.1} (tol {:.1}); \
             a shortfall means a face is missing",
            area_regions,
            area_union,
            tol,
        );
    }

    #[test]
    fn circle_crossing_rectangle_edge_creates_multiple_regions() {
        // A circle straddling one edge of a rectangle DOES intersect that
        // edge, so the planar arrangement now connects the two shapes and
        // produces the full set of sub-regions (no nesting collapse).
        let mut c = SketchCurves::new();
        c.add_rectangle((0.0, 0.0), (10.0, 10.0));
        c.add_circle((10.0, 5.0), 3.0);
        let regions = detect_regions(&c);
        assert!(
            regions.len() >= 2,
            "circle straddling rect edge should split into multiple regions, got {:?}",
            regions
        );
    }
}
