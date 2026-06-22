use std::collections::{HashMap, HashSet};

use crate::TriangleMesh;
use openrcad_foundation::{tolerance::CONFUSION, Pnt, Pnt2d, Vec as GeomVec};
use openrcad_geom::{Curve, GeomCurve, GeomSurface, Surface};
use openrcad_topo::{orientation::Orientation, Face};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tri {
    pub a: usize,
    pub b: usize,
    pub c: usize,
}

#[derive(Clone, Copy, Debug)]
struct BoundarySample {
    uv: Pnt2d,
    point: Pnt,
    /// True when this sample lies on a miter-seam (elliptical) boundary edge.
    /// Such edges are shared by two analytic faces whose surfaces are tangent
    /// along the seam; without interior support the per-face triangulation
    /// collapses to a flat fan over the shared seam vertices (see
    /// [`add_seam_support`]).
    on_seam: bool,
}

/// Robust 2D orientation test from predicates (CCW turn > 0, CW turn < 0, collinear == 0).
fn ccw(a: Pnt2d, b: Pnt2d, c: Pnt2d) -> f64 {
    openrcad_foundation::predicates::orient2d(a, b, c)
}

/// Check if point `p` is inside the circumcircle of counter-clockwise oriented triangle `a`, `b`, `c`.
pub fn in_circumcircle(p: Pnt2d, a: Pnt2d, b: Pnt2d, c: Pnt2d) -> bool {
    let adx = a.x() - p.x();
    let ady = a.y() - p.y();
    let bdx = b.x() - p.x();
    let bdy = b.y() - p.y();
    let cdx = c.x() - p.x();
    let cdy = c.y() - p.y();

    let abdet = adx * bdy - bdx * ady;
    let bcdet = bdx * cdy - cdx * bdy;
    let cadet = cdx * ady - adx * cdy;

    let alift = adx * adx + ady * ady;
    let blift = bdx * bdx + bdy * bdy;
    let clift = cdx * cdx + cdy * cdy;

    let det = alift * bcdet + blift * cadet + clift * abdet;

    let is_ccw = ccw(a, b, c) > 0.0;
    if is_ccw {
        det > 1e-12
    } else {
        det < -1e-12
    }
}

/// Bowyer-Watson algorithm for 2D Delaunay triangulation.
pub fn delaunay_triangulate(points: &[Pnt2d]) -> Vec<Tri> {
    if points.len() < 3 {
        return Vec::new();
    }

    let mut x_min = points[0].x();
    let mut x_max = points[0].x();
    let mut y_min = points[0].y();
    let mut y_max = points[0].y();
    for p in points {
        x_min = f64::min(x_min, p.x());
        x_max = f64::max(x_max, p.x());
        y_min = f64::min(y_min, p.y());
        y_max = f64::max(y_max, p.y());
    }

    let dx = x_max - x_min;
    let dy = y_max - y_min;
    let dmax = f64::max(dx, dy);
    let mid_x = 0.5 * (x_min + x_max);
    let mid_y = 0.5 * (y_min + y_max);

    // Super-triangle enclosing all points
    let sp0 = Pnt2d::new(mid_x - 20.0 * dmax - 1.0, mid_y - 20.0 * dmax - 1.0);
    let sp1 = Pnt2d::new(mid_x, mid_y + 20.0 * dmax + 1.0);
    let sp2 = Pnt2d::new(mid_x + 20.0 * dmax + 1.0, mid_y - 20.0 * dmax - 1.0);

    let mut all_points = points.to_vec();
    let s0_idx = all_points.len();
    all_points.push(sp0);
    let s1_idx = all_points.len();
    all_points.push(sp1);
    let s2_idx = all_points.len();
    all_points.push(sp2);

    let mut triangles = vec![Tri {
        a: s0_idx,
        b: s1_idx,
        c: s2_idx,
    }];

    for i in 0..points.len() {
        let p = all_points[i];
        let mut bad_triangles = Vec::new();

        for (t_idx, &t) in triangles.iter().enumerate() {
            if in_circumcircle(p, all_points[t.a], all_points[t.b], all_points[t.c]) {
                bad_triangles.push(t_idx);
            }
        }

        let mut polygon = Vec::new();
        for &t_idx in &bad_triangles {
            let t = triangles[t_idx];
            let edges = [(t.a, t.b), (t.b, t.c), (t.c, t.a)];
            for &(edge_start, edge_end) in &edges {
                let mut shared = false;
                for &other_idx in &bad_triangles {
                    if other_idx == t_idx {
                        continue;
                    }
                    let ot = triangles[other_idx];
                    let ot_edges = [
                        (ot.a, ot.b),
                        (ot.b, ot.a),
                        (ot.b, ot.c),
                        (ot.c, ot.b),
                        (ot.c, ot.a),
                        (ot.a, ot.c),
                    ];
                    if ot_edges.contains(&(edge_start, edge_end)) {
                        shared = true;
                        break;
                    }
                }
                if !shared {
                    polygon.push((edge_start, edge_end));
                }
            }
        }

        bad_triangles.sort_unstable();
        for &t_idx in bad_triangles.iter().rev() {
            triangles.remove(t_idx);
        }

        for &(edge_start, edge_end) in &polygon {
            triangles.push(Tri {
                a: edge_start,
                b: edge_end,
                c: i,
            });
        }
    }

    triangles.retain(|t| t.a < points.len() && t.b < points.len() && t.c < points.len());

    triangles
}

fn edge_key(a: usize, b: usize) -> (usize, usize) {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

fn tri_has_edge(t: Tri, a: usize, b: usize) -> bool {
    (t.a == a && t.b == b)
        || (t.a == b && t.b == a)
        || (t.b == a && t.c == b)
        || (t.b == b && t.c == a)
        || (t.c == a && t.a == b)
        || (t.c == b && t.a == a)
}

fn mesh_has_edge(tris: &[Tri], a: usize, b: usize) -> bool {
    tris.iter().any(|&t| tri_has_edge(t, a, b))
}

fn third_vertex(t: Tri, a: usize, b: usize) -> Option<usize> {
    if (t.a == a || t.a == b) && (t.b == a || t.b == b) {
        Some(t.c)
    } else if (t.b == a || t.b == b) && (t.c == a || t.c == b) {
        Some(t.a)
    } else if (t.c == a || t.c == b) && (t.a == a || t.a == b) {
        Some(t.b)
    } else {
        None
    }
}

fn make_ccw(a: usize, b: usize, c: usize, points: &[Pnt2d]) -> Option<Tri> {
    let area = ccw(points[a], points[b], points[c]);
    if area > 1e-14 {
        Some(Tri { a, b, c })
    } else if area < -1e-14 {
        Some(Tri { a, b: c, c: b })
    } else {
        None
    }
}

fn segments_intersect_strict(a: Pnt2d, b: Pnt2d, c: Pnt2d, d: Pnt2d) -> bool {
    let ab_c = ccw(a, b, c);
    let ab_d = ccw(a, b, d);
    let cd_a = ccw(c, d, a);
    let cd_b = ccw(c, d, b);

    ab_c * ab_d < -1e-14 && cd_a * cd_b < -1e-14
}

fn build_edge_map(tris: &[Tri]) -> HashMap<(usize, usize), Vec<usize>> {
    let mut edge_map: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
    for (idx, t) in tris.iter().enumerate() {
        for (a, b) in [(t.a, t.b), (t.b, t.c), (t.c, t.a)] {
            edge_map.entry(edge_key(a, b)).or_default().push(idx);
        }
    }
    edge_map
}

fn find_crossing_edge(
    tris: &[Tri],
    points: &[Pnt2d],
    a: usize,
    b: usize,
    locked_edges: &HashSet<(usize, usize)>,
) -> Option<(usize, usize, usize, usize)> {
    let edge_map = build_edge_map(tris);
    for (&(c, d), adj) in &edge_map {
        if c == a || c == b || d == a || d == b || locked_edges.contains(&edge_key(c, d)) {
            continue;
        }

        if adj.len() == 2 && segments_intersect_strict(points[a], points[b], points[c], points[d]) {
            return Some((c, d, adj[0], adj[1]));
        }
    }

    None
}

fn flip_edge(tris: &mut [Tri], points: &[Pnt2d], edge: (usize, usize, usize, usize)) -> bool {
    let (a, b, tri_idx_a, tri_idx_b) = edge;
    let Some(c) = third_vertex(tris[tri_idx_a], a, b) else {
        return false;
    };
    let Some(d) = third_vertex(tris[tri_idx_b], a, b) else {
        return false;
    };

    if c == d || mesh_has_edge(tris, c, d) {
        return false;
    }

    let old_sides = ccw(points[a], points[b], points[c]) * ccw(points[a], points[b], points[d]);
    let new_sides = ccw(points[c], points[d], points[a]) * ccw(points[c], points[d], points[b]);
    if old_sides >= -1e-14 || new_sides >= -1e-14 {
        return false;
    }

    let Some(t1) = make_ccw(c, d, a, points) else {
        return false;
    };
    let Some(t2) = make_ccw(d, c, b, points) else {
        return false;
    };

    tris[tri_idx_a] = t1;
    tris[tri_idx_b] = t2;
    true
}

fn recover_constrained_edges(
    mut tris: Vec<Tri>,
    points: &[Pnt2d],
    constraints: &[(usize, usize)],
) -> Vec<Tri> {
    let mut locked_edges = HashSet::new();
    let max_flips = constraints.len().max(1) * tris.len().max(1) * tris.len().max(1);
    let mut flips = 0;

    for &(a, b) in constraints {
        if a == b {
            continue;
        }

        while !mesh_has_edge(&tris, a, b) {
            if flips >= max_flips {
                break;
            }

            let Some(crossing) = find_crossing_edge(&tris, points, a, b, &locked_edges) else {
                break;
            };

            if !flip_edge(&mut tris, points, crossing) {
                break;
            }

            flips += 1;
        }

        if mesh_has_edge(&tris, a, b) {
            locked_edges.insert(edge_key(a, b));
        }
    }

    tris
}

/// Ray-casting point-in-polygon containment test.
pub fn is_point_in_polygon(p: Pnt2d, loop_pts: &[Pnt2d]) -> bool {
    let q = (p.x(), p.y());
    let poly: Vec<_> = loop_pts.iter().map(|pt| (pt.x(), pt.y())).collect();
    openrcad_topo::containment::point_in_polygon_2d(q, &poly)
}

/// Project a 3D point onto the surface to find its (u, v) coordinates.
///
/// Uses closed-form analytical equations for primitives, falling back to a robust Newton-Raphson solver.
pub fn project_point(surf: &GeomSurface, pt: Pnt, hint: Option<(f64, f64)>) -> (f64, f64) {
    match surf {
        GeomSurface::Plane(plane) => {
            let diff = pt - plane.location();
            let u = diff.dot(&GeomVec::from_dir(plane.position().x_direction()));
            let v = diff.dot(&GeomVec::from_dir(plane.position().y_direction()));
            return (u, v);
        }
        GeomSurface::Cylinder(cyl) => {
            let diff = pt - cyl.position().location();
            let v = diff.dot(&GeomVec::from_dir(cyl.position().direction()));
            let dx = diff.dot(&GeomVec::from_dir(cyl.position().x_direction()));
            let dy = diff.dot(&GeomVec::from_dir(cyl.position().y_direction()));
            let mut u = dy.atan2(dx);
            if u < 0.0 {
                u += 2.0 * std::f64::consts::PI;
            }
            return (u, v);
        }
        GeomSurface::Sphere(sph) => {
            let diff = pt - sph.center();
            let dist = diff.magnitude();
            if dist < CONFUSION {
                return hint.unwrap_or((0.0, 0.0));
            }
            let d_unit = diff / dist;
            let z_comp = d_unit.dot(&GeomVec::from_dir(sph.position().direction()));
            let z_comp_clamped = z_comp.clamp(-1.0, 1.0);
            let v = z_comp_clamped.asin();

            if (1.0 - z_comp_clamped.abs()) < 1e-12 {
                return (hint.map(|h| h.0).unwrap_or(0.0), v);
            }

            let dx = d_unit.dot(&GeomVec::from_dir(sph.position().x_direction()));
            let dy = d_unit.dot(&GeomVec::from_dir(sph.position().y_direction()));
            let mut u = dy.atan2(dx);
            if u < 0.0 {
                u += 2.0 * std::f64::consts::PI;
            }
            return (u, v);
        }
        GeomSurface::Cone(cone) => {
            let diff = pt - cone.position().location();
            let v = diff.dot(&GeomVec::from_dir(cone.position().direction()));
            let r_at_v = cone.radius_at(v);

            if r_at_v.abs() < CONFUSION {
                return (hint.map(|h| h.0).unwrap_or(0.0), v);
            }

            let dx = diff.dot(&GeomVec::from_dir(cone.position().x_direction()));
            let dy = diff.dot(&GeomVec::from_dir(cone.position().y_direction()));
            let mut u = dy.atan2(dx);
            if u < 0.0 {
                u += 2.0 * std::f64::consts::PI;
            }
            return (u, v);
        }
        _ => {}
    }

    // Newton-Raphson fallback for NURBS/B-Spline
    let (u_min, u_max, v_min, v_max) = surf.bounds();
    let (mut u, mut v) = hint.unwrap_or_else(|| {
        let u0 = if u_min.is_infinite() {
            0.0
        } else {
            0.5 * (u_min + u_max)
        };
        let v0 = if v_min.is_infinite() {
            0.0
        } else {
            0.5 * (v_min + v_max)
        };
        (u0, v0)
    });

    for _ in 0..10 {
        let (s, su, sv) = surf.d1(u, v);
        let diff = s - pt;

        let a = su.dot(&su);
        let b = su.dot(&sv);
        let c = sv.dot(&sv);
        let det = a * c - b * b;

        if det.abs() < 1e-12 {
            break;
        }

        let gu = diff.dot(&su);
        let gv = diff.dot(&sv);

        let du = (c * gu - b * gv) / det;
        let dv = (-b * gu + a * gv) / det;

        u -= du;
        v -= dv;

        if !u_min.is_infinite() {
            u = u.clamp(u_min, u_max);
        }
        if !v_min.is_infinite() {
            v = v.clamp(v_min, v_max);
        }

        if du.abs() < 1e-9 && dv.abs() < 1e-9 {
            break;
        }
    }

    (u, v)
}

/// Periodic unwrapping to prevent jumps across seams.
pub fn unwrap_coordinate(val: f64, prev: f64, period: f64) -> f64 {
    let closure = |i: i32| val + i as f64 * period;
    (-2..=2)
        .map(closure)
        .min_by(|a, b| (a - prev).abs().partial_cmp(&(b - prev).abs()).unwrap())
        .unwrap()
}

/// Sample interior (u, v) points to represent surface curvature.
pub fn sample_interior_points(
    surf: &GeomSurface,
    u_min: f64,
    u_max: f64,
    v_min: f64,
    v_max: f64,
    chord_err: f64,
) -> Vec<Pnt2d> {
    let mut points = Vec::new();

    // Cylinder and cone are RULED along v (straight lines), so interior samples add
    // no shape accuracy — the v-boundary curves already pin the surface. But a few
    // interior vertices still matter for *shading* (per-vertex normals along the
    // round) and to give the Delaunay non-collinear points to triangulate the
    // end-cap arcs against. The catch is the old full-width row: an interior point
    // sampled right up against a tangent boundary routes that straight boundary's
    // triangulation through it, bowing the tangent line off true and cracking it
    // against the flat neighbour (the reported fillet "white line"). So for ruled
    // surfaces the u samples are inset to the central band, well clear of both
    // tangents — `u_inset` below.
    let mut u_inset = 0.0;
    let (u_divs, v_divs) = match surf {
        GeomSurface::Plane(_) => (0, 0),
        GeomSurface::Cylinder(cyl) => {
            u_inset = 0.25;
            let r = cyl.radius();
            let theta = 2.0 * (2.0 * chord_err / r).sqrt();
            let span = u_max - u_min;
            let n = f64::max(2.0, (span / theta).ceil()) as usize;
            (n, 0)
        }
        GeomSurface::Sphere(sph) => {
            let r = sph.radius();
            let theta = 2.0 * (2.0 * chord_err / r).sqrt();
            let span_u = u_max - u_min;
            let span_v = v_max - v_min;
            let nu = f64::max(2.0, (span_u / theta).ceil()) as usize;
            let nv = f64::max(2.0, (span_v / theta).ceil()) as usize;
            (nu, nv)
        }
        GeomSurface::Cone(cone) => {
            u_inset = 0.25;
            let r1 = cone.radius_at(v_min).abs();
            let r2 = cone.radius_at(v_max).abs();
            let r = f64::max(r1, r2);
            let theta = if r > CONFUSION {
                2.0 * (2.0 * chord_err / r).sqrt()
            } else {
                std::f64::consts::PI / 2.0
            };
            let span = u_max - u_min;
            let n = f64::max(2.0, (span / theta).ceil()) as usize;
            (n, 0)
        }
        GeomSurface::Torus(tor) => {
            // Curvature in u scales with (R + r); in v with the tube radius r.
            let span_u = u_max - u_min;
            let span_v = v_max - v_min;
            let theta_u =
                2.0 * (2.0 * chord_err / (tor.major_radius() + tor.minor_radius())).sqrt();
            let theta_v = 2.0 * (2.0 * chord_err / tor.minor_radius().max(CONFUSION)).sqrt();
            let nu = f64::max(2.0, (span_u / theta_u).ceil()) as usize;
            let nv = f64::max(2.0, (span_v / theta_v).ceil()) as usize;
            (nu, nv)
        }
        GeomSurface::BSpline(_)
        | GeomSurface::Gregory(_)
        | GeomSurface::Offset(_)
        | GeomSurface::Ruled(_) => (10, 10),
    };

    if u_divs > 0 {
        for i in 1..u_divs {
            // `u_inset` (ruled surfaces) maps the samples into the central band
            // [u_inset, 1-u_inset] so none sits against a tangent boundary.
            let t_raw = i as f64 / u_divs as f64;
            let t_u = u_inset + t_raw * (1.0 - 2.0 * u_inset);
            let u = u_min + t_u * (u_max - u_min);
            if v_divs > 0 {
                for j in 1..v_divs {
                    let t_v = j as f64 / v_divs as f64;
                    let v = v_min + t_v * (v_max - v_min);
                    points.push(Pnt2d::new(u, v));
                }
            } else {
                let v = 0.5 * (v_min + v_max);
                points.push(Pnt2d::new(u, v));
            }
        }
    }

    points
}

/// Discretize an edge's curve to satisfy the chordal error budget.
pub fn discretize_edge_curve(curve: &GeomCurve, first: f64, last: f64, chord_err: f64) -> Vec<f64> {
    let mut params = vec![first, last];

    fn subdivide(
        curve: &GeomCurve,
        t0: f64,
        t1: f64,
        chord_err: f64,
        depth: usize,
        params: &mut Vec<f64>,
    ) {
        if depth > 8 {
            return;
        }
        let tm = 0.5 * (t0 + t1);
        let p0 = curve.point(t0);
        let p1 = curve.point(t1);
        let pm = curve.point(tm);

        let seg_vec = p1 - p0;
        let seg_len = seg_vec.magnitude();
        let dev = if seg_len > CONFUSION {
            let u_vec = seg_vec / seg_len;
            let d_vec = pm - p0;
            let proj = d_vec.dot(&u_vec);
            let perp = d_vec - u_vec * proj;
            perp.magnitude()
        } else {
            pm.distance(&p0)
        };

        if dev > chord_err {
            subdivide(curve, t0, tm, chord_err, depth + 1, params);
            params.push(tm);
            subdivide(curve, tm, t1, chord_err, depth + 1, params);
        }
    }

    subdivide(curve, first, last, chord_err, 0, &mut params);
    params.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap());
    params.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    params
}

/// Tessellates a single Face into a TriangleMesh locally.
///
/// `face_index` is recorded as the source-face id of every emitted triangle, so
/// the combined mesh can map triangles back to their originating face (picking).
pub fn tessellate_face_local(face: &Face, chord_err: f64, face_index: u32) -> TriangleMesh {
    let surface = match face.surface() {
        Some(s) => s,
        None => return TriangleMesh::new(),
    };
    let _outer_wire = match face.outer_wire() {
        Some(w) => w,
        None => return TriangleMesh::new(),
    };

    // 1. Project boundary wires to 2D parameters
    let mut loops_2d = Vec::new();
    let mut all_points_2d = Vec::new();
    let mut all_points_3d = Vec::new();
    // (u, v) of every boundary sample that sits on a miter seam, for the
    // interior-support pass that keeps two tangent faces from collapsing to a
    // shared flat fan over the seam (see step 2b below).
    let mut seam_uv: Vec<Pnt2d> = Vec::new();

    // Mapping to dedup vertices close to each other in 2D
    let mut point_map = HashMap::new();

    let face_wires = face.wires();
    for wire in &face_wires {
        let mut loop_samples = Vec::new();
        let edges = wire.edges();
        let mut prev_hint = None;

        for (edge_idx, edge) in edges.iter().enumerate() {
            let p_start = edge.start().point();
            let p_end = edge.end().point();
            let is_collapsed = p_start.distance(&p_end) <= 1e-5;

            if is_collapsed {
                let (u_start, v_pole) = if let Some((pu, pv)) = prev_hint {
                    (pu, pv)
                } else {
                    let u = find_prev_non_collapsed_u(&edges, edge_idx, surface, p_start)
                        .unwrap_or(0.0);
                    let (_, v) = project_point(surface, p_start, None);
                    (u, v)
                };

                let u_next = find_next_non_collapsed_u(&edges, edge_idx, surface, p_start)
                    .unwrap_or(u_start);
                let u_next = if surface.is_uclosed() {
                    unwrap_coordinate(u_next, u_start, 2.0 * std::f64::consts::PI)
                } else {
                    u_next
                };

                let n_points = 5;
                for i in 0..n_points {
                    let frac = i as f64 / (n_points - 1) as f64;
                    let u = u_start + frac * (u_next - u_start);
                    let v = v_pole;
                    loop_samples.push(BoundarySample {
                        uv: Pnt2d::new(u, v),
                        point: p_start,
                        on_seam: false,
                    });
                    prev_hint = Some((u, v));
                }

                if loop_samples.len() > 1 {
                    loop_samples.pop();
                }
                continue;
            }

            let curve = match edge.curve() {
                Some(c) => c,
                None => {
                    // Fallback degenerate edge
                    let p3d = edge.start().point();
                    let (mut u, mut v) = project_point(surface, p3d, prev_hint);
                    if let Some((pu, pv)) = prev_hint {
                        if surface.is_uclosed() {
                            u = unwrap_coordinate(u, pu, 2.0 * std::f64::consts::PI);
                        }
                        if surface.is_vclosed() {
                            v = unwrap_coordinate(v, pv, 2.0 * std::f64::consts::PI);
                        }
                    }
                    loop_samples.push(BoundarySample {
                        uv: Pnt2d::new(u, v),
                        point: p3d,
                        on_seam: false,
                    });
                    prev_hint = Some((u, v));
                    continue;
                }
            };

            let on_seam = matches!(curve, GeomCurve::Ellipse(_));
            let params = discretize_edge_curve(curve, edge.first(), edge.last(), chord_err);
            let is_reversed = !edge.orientation().is_forward();

            let params_directed: Vec<f64> = if is_reversed {
                params.into_iter().rev().collect()
            } else {
                params
            };

            for &t in &params_directed {
                let p3d = curve.point(t);
                let (mut u, mut v) = project_point(surface, p3d, prev_hint);

                // Periodic Seam coordinate unwrapping
                if let Some((pu, pv)) = prev_hint {
                    if surface.is_uclosed() {
                        u = unwrap_coordinate(u, pu, 2.0 * std::f64::consts::PI);
                    }
                    if surface.is_vclosed() {
                        v = unwrap_coordinate(v, pv, 2.0 * std::f64::consts::PI);
                    }
                }

                let p2d = Pnt2d::new(u, v);
                loop_samples.push(BoundarySample {
                    uv: p2d,
                    point: p3d,
                    on_seam,
                });
                prev_hint = Some((u, v));
            }

            // Remove duplicated adjacent endpoint when moving to next edge
            if loop_samples.len() > 1 {
                loop_samples.pop();
            }
        }

        // Close the loop if not closed
        if loop_samples.len() > 2 {
            let first = loop_samples[0].uv;
            let last = loop_samples.last().unwrap().uv;
            if first.distance(&last) > 1e-6 {
                // Keep it
            } else {
                loop_samples.pop();
            }
        }

        // Add loop points to master coordinates
        let mut loop_indices = Vec::new();
        for sample in &loop_samples {
            let p2d = sample.uv;
            if sample.on_seam {
                seam_uv.push(p2d);
            }
            let key = ((p2d.x() * 1e8) as i64, (p2d.y() * 1e8) as i64);
            let idx = *point_map.entry(key).or_insert_with(|| {
                let id = all_points_2d.len();
                all_points_2d.push(p2d);
                all_points_3d.push(sample.point);
                id
            });
            loop_indices.push(idx);
        }

        loops_2d.push(loop_indices);
    }

    if loops_2d.is_empty() || loops_2d[0].len() < 3 {
        return TriangleMesh::new();
    }

    // 2. Generate and filter interior sampling points
    // Calculate bounding box of the outer loop
    let mut u_min = all_points_2d[loops_2d[0][0]].x();
    let mut u_max = u_min;
    let mut v_min = all_points_2d[loops_2d[0][0]].y();
    let mut v_max = v_min;
    for &idx in &loops_2d[0] {
        let p = all_points_2d[idx];
        u_min = f64::min(u_min, p.x());
        u_max = f64::max(u_max, p.x());
        v_min = f64::min(v_min, p.y());
        v_max = f64::max(v_max, p.y());
    }

    let interior_candidates =
        sample_interior_points(surface, u_min, u_max, v_min, v_max, chord_err);

    // Filter interior points inside loops
    let outer_pts: Vec<Pnt2d> = loops_2d[0].iter().map(|&idx| all_points_2d[idx]).collect();
    let inner_pts_list: Vec<Vec<Pnt2d>> = loops_2d
        .iter()
        .skip(1)
        .map(|l| l.iter().map(|&idx| all_points_2d[idx]).collect())
        .collect();

    for p2d in interior_candidates {
        // Must be inside outer loop
        if !is_point_in_polygon(p2d, &outer_pts) {
            continue;
        }
        // Must not be inside any inner loop (hole)
        let mut inside_hole = false;
        for hole in &inner_pts_list {
            if is_point_in_polygon(p2d, hole) {
                inside_hole = true;
                break;
            }
        }
        if inside_hole {
            continue;
        }

        // Add interior point
        let key = ((p2d.x() * 1e8) as i64, (p2d.y() * 1e8) as i64);
        point_map.entry(key).or_insert_with(|| {
            let id = all_points_2d.len();
            all_points_2d.push(p2d);
            all_points_3d.push(surface.point(p2d.x(), p2d.y()));
            id
        });
    }

    // 2b. Miter-seam interior support.
    //
    // A miter seam (an elliptical edge) is shared by two analytic faces whose
    // surfaces are *tangent* along it. Near the seam's stub-vertex corner the
    // per-face (u, v) region tapers to a thin wedge with no interior sample, so
    // the triangulation degenerates to a flat fan that uses only the shared seam
    // vertices — and since both faces produce the identical fan (same welded
    // vertices), the result is two coincident, oppositely-wound triangle layers:
    // a non-manifold "double membrane" that z-fights on screen at the corner.
    //
    // Seed one interior vertex just *inside* the region next to each seam sample,
    // offset along the seam→region-centroid direction. On each face that vertex
    // lands on that face's own curved surface (the two faces bulge opposite ways),
    // so the triangulations no longer coincide and the membrane is gone. The
    // offset is a small fraction toward the centroid, so it stays clear of the
    // straight tangent contacts the seam meets at its ends (which a support point
    // *on* a tangent would bow — the classic fillet "white line" crack).
    if !seam_uv.is_empty() {
        let cu = outer_pts.iter().map(|p| p.x()).sum::<f64>() / outer_pts.len() as f64;
        let cv = outer_pts.iter().map(|p| p.y()).sum::<f64>() / outer_pts.len() as f64;
        // One support vertex just inside the region next to each seam sample,
        // offset a small fraction toward the centroid. The fraction (0.04) is small
        // on purpose: a larger offset overshoots the *tiny* fan triangle at the
        // seam's stub-vertex corner on elongated faces (where the centroid is far),
        // leaving that one apex triangle coincident. 0.04 keeps the support close to
        // the seam — enough to pull each face's near-seam triangles onto its own
        // (oppositely-bulging) surface so the two faces no longer share identical
        // flat triangles — while staying clear of the straight tangent contacts the
        // seam meets at its ends (a support point *on* a tangent would bow it and
        // crack against the flat neighbour). Verified to leave zero non-manifold and
        // zero crack edges across a wide range of box aspect ratios and radii.
        for s in &seam_uv {
            let p2d = Pnt2d::new(s.x() + 0.04 * (cu - s.x()), s.y() + 0.04 * (cv - s.y()));
            if !is_point_in_polygon(p2d, &outer_pts)
                || inner_pts_list.iter().any(|h| is_point_in_polygon(p2d, h))
            {
                continue;
            }
            let key = ((p2d.x() * 1e8) as i64, (p2d.y() * 1e8) as i64);
            point_map.entry(key).or_insert_with(|| {
                let id = all_points_2d.len();
                all_points_2d.push(p2d);
                all_points_3d.push(surface.point(p2d.x(), p2d.y()));
                id
            });
        }
    }

    // A convex planar face whose boundary is a finely-sampled curve — a circular
    // cap, a bored-hole rim — has many near-cocircular boundary points. Bowyer–
    // Watson on those alone is numerically ambiguous and can leave a sliver "flap"
    // triangle that chords the disc, cracking it against the cylinder wall (two
    // single-referenced edges). For exactly this shape — one convex loop, no
    // holes, many points — fan it from the centroid instead: a plane is flat so
    // the fan is exact, the rim sub-edges stay identical to the wall's (no crack),
    // and the spokes are interior. Everything else keeps the Delaunay path.
    if matches!(surface, GeomSurface::Plane(_)) && loops_2d.len() == 1 && outer_pts.len() >= 10 {
        let n = outer_pts.len();
        let cu = outer_pts.iter().map(|p| p.x()).sum::<f64>() / n as f64;
        let cv = outer_pts.iter().map(|p| p.y()).sum::<f64>() / n as f64;
        // Only a genuinely (near-)cocircular boundary — a real circular cap or
        // hole rim — triggers the Bowyer–Watson degeneracy this works around. A
        // rounded rectangle (a filleted box side face) is convex and may reach 10
        // points too, but its boundary points sit at wildly varying radii, and the
        // Delaunay handles it cleanly; fanning it from the centroid would chord its
        // straight edges and mismatch the neighbours. So require uniform radius.
        let radii: Vec<f64> = outer_pts
            .iter()
            .map(|p| ((p.x() - cu).powi(2) + (p.y() - cv).powi(2)).sqrt())
            .collect();
        let mean_r = radii.iter().sum::<f64>() / n as f64;
        let cocircular = mean_r > 1e-9
            && radii.iter().all(|r| (r - mean_r).abs() <= 0.06 * mean_r);
        if cocircular {
            let c_id = all_points_2d.len();
            all_points_2d.push(Pnt2d::new(cu, cv));
            all_points_3d.push(surface.point(cu, cv));

            let wants_ccw = face.orientation() != Orientation::Reversed;
            let ring = &loops_2d[0];
            let mut triangles = Vec::with_capacity(n);
            for i in 0..n {
                let ia = ring[i];
                let ib = ring[(i + 1) % n];
                let tri_ccw = ccw(all_points_2d[ia], all_points_2d[ib], all_points_2d[c_id]) > 0.0;
                if tri_ccw == wants_ccw {
                    triangles.push([ia as u32, ib as u32, c_id as u32]);
                } else {
                    triangles.push([ia as u32, c_id as u32, ib as u32]);
                }
            }
            let face_ids = vec![face_index; triangles.len()];
            return TriangleMesh::from_buffers_with_faces(all_points_3d, triangles, face_ids);
        }
    }

    // 3. Delaunay Triangulation in 2D, with trimming-loop edges recovered.
    let mut constraints = Vec::new();
    for loop_indices in &loops_2d {
        for i in 0..loop_indices.len() {
            let a = loop_indices[i];
            let b = loop_indices[(i + 1) % loop_indices.len()];
            if a != b {
                constraints.push((a, b));
            }
        }
    }

    let tris = recover_constrained_edges(
        delaunay_triangulate(&all_points_2d),
        &all_points_2d,
        &constraints,
    );

    // 4. Centroid trimming and construction of output TriangleMesh
    let mut triangles = Vec::new();
    for t in tris {
        let pa = all_points_2d[t.a];
        let pb = all_points_2d[t.b];
        let pc = all_points_2d[t.c];

        // Centroid of the triangle
        let centroid = Pnt2d::new(
            (pa.x() + pb.x() + pc.x()) / 3.0,
            (pa.y() + pb.y() + pc.y()) / 3.0,
        );

        if is_point_in_polygon(centroid, &outer_pts) {
            let mut inside_hole = false;
            for hole in &inner_pts_list {
                if is_point_in_polygon(centroid, hole) {
                    inside_hole = true;
                    break;
                }
            }
            if !inside_hole {
                let tri_ccw = ccw(pa, pb, pc) > 0.0;
                let wants_ccw = face.orientation() != Orientation::Reversed;
                if tri_ccw == wants_ccw {
                    triangles.push([t.a as u32, t.b as u32, t.c as u32]);
                } else {
                    triangles.push([t.a as u32, t.c as u32, t.b as u32]);
                }
            }
        }
    }

    let face_ids = vec![face_index; triangles.len()];
    TriangleMesh::from_buffers_with_faces(all_points_3d, triangles, face_ids)
}

/// Combine multiple TriangleMeshes into a single watertight TriangleMesh by welding coincident vertices.
pub fn combine(meshes: &[TriangleMesh]) -> TriangleMesh {
    let mut all_vertices = Vec::new();
    let mut all_triangles = Vec::new();
    let mut all_face_ids = Vec::new();
    let mut vertex_map = HashMap::new();

    for mesh in meshes {
        let mut index_map = Vec::with_capacity(mesh.vertices.len());
        for &p in &mesh.vertices {
            // Weld vertices that are within 1e-9 of each other
            let key = (
                (p.x() * 1e9).round() as i64,
                (p.y() * 1e9).round() as i64,
                (p.z() * 1e9).round() as i64,
            );
            let idx = *vertex_map.entry(key).or_insert_with(|| {
                let id = all_vertices.len();
                all_vertices.push(p);
                id
            });
            index_map.push(idx);
        }
        for (i, tri) in mesh.triangles.iter().enumerate() {
            let a = index_map[tri[0] as usize] as u32;
            let b = index_map[tri[1] as usize] as u32;
            let c = index_map[tri[2] as usize] as u32;
            all_triangles.push([a, b, c]);
            // Carry per-triangle face provenance through the weld.
            all_face_ids.push(mesh.face_ids.get(i).copied().unwrap_or(0));
        }
    }

    TriangleMesh::from_buffers_with_faces(all_vertices, all_triangles, all_face_ids)
}

/// Close tessellation cracks where one face *chords* a shared boundary that its
/// neighbour *subdivides*.
///
/// Adjacent faces sample a shared curved boundary independently. A face whose
/// (u, v) image of that boundary is a straight parameter line — e.g. a cylinder
/// fillet's end cap, where the 3D arc maps to `v = const` — can only chord it
/// (its collinear boundary points triangulate to degenerate slivers that get
/// dropped), while the neighbour that sees the true arc (a planar side face)
/// fans it with the full vertex chain. After welding, the chord is referenced by
/// one triangle and the arc by the other, so the lens between them is a hole — a
/// bright crack on screen and a leak in exported STL.
///
/// For every boundary edge (the *chord*) it finds the complementary chain of
/// boundary edges spanning the same two endpoints (the *arc* the neighbour
/// subdivided) and re-fans the chord's triangle through that chain, so both
/// faces share the same sub-edges. It is purely topological — it threads the
/// already-present, welded boundary vertices — so it adds no geometry and only
/// subdivides an existing triangle; the inserted vertices lie on the true
/// surface (the neighbour put them there), so nothing is distorted.
///
/// The chord is identified geometrically (it is the long edge the arc shortcuts;
/// the arc stays close to it) rather than by face-id bookkeeping, so it is robust
/// to any boundary topology — open arc chains, lens loops where the chord is
/// *also* a single boundary edge, T-junctions where the neighbour's points lie
/// **on** the chord, and cracks on extruded-prism or boolean (boss-union) bodies
/// alike. On a closed solid the result has zero single-referenced edges.
pub fn stitch_boundary_lenses(mesh: &mut TriangleMesh) {
    use std::collections::VecDeque;
    let key = |a: u32, b: u32| if a <= b { (a, b) } else { (b, a) };

    // Shortest boundary path a..b that does NOT use the direct a–b hop — i.e. the
    // arc the chord shortcuts. `None` if a and b are only directly connected.
    let boundary_path = |adj: &HashMap<u32, Vec<u32>>, a: u32, b: u32| -> Option<Vec<u32>> {
        let mut prev: HashMap<u32, u32> = HashMap::new();
        let mut visited: HashSet<u32> = HashSet::new();
        let mut queue: VecDeque<u32> = VecDeque::new();
        visited.insert(a);
        queue.push_back(a);
        while let Some(u) = queue.pop_front() {
            for &w in adj.get(&u).into_iter().flatten() {
                if u == a && w == b {
                    continue; // forbid the direct chord edge
                }
                if w == a || visited.contains(&w) {
                    continue;
                }
                visited.insert(w);
                prev.insert(w, u);
                if w == b {
                    let mut path = vec![b];
                    let mut cur = b;
                    while cur != a {
                        cur = prev[&cur];
                        path.push(cur);
                    }
                    path.reverse();
                    return Some(path);
                }
                queue.push_back(w);
            }
        }
        None
    };

    // One stitch pass can expose another (nested lenses); a few passes converge.
    for _ in 0..6 {
        // Edge -> the triangles using it. On a closed solid a boundary edge (used
        // once) is a crack; the chord that shortcuts it is shared (used 2+).
        let mut edge_tris: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
        for (ti, t) in mesh.triangles.iter().enumerate() {
            for &(a, b) in &[(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                edge_tris.entry(key(a, b)).or_default().push(ti);
            }
        }
        let mut boundary: Vec<(u32, u32)> = edge_tris
            .iter()
            .filter(|(_, v)| v.len() == 1)
            .map(|(&k, _)| k)
            .collect();
        if boundary.is_empty() {
            break;
        }
        // Adjacency over boundary edges only.
        let mut adj: HashMap<u32, Vec<u32>> = HashMap::new();
        for &(a, b) in &boundary {
            adj.entry(a).or_default().push(b);
            adj.entry(b).or_default().push(a);
        }
        // Process longest boundary edges first: a lens chord is longer than any
        // single sub-edge of the arc it shortcuts, so this re-fans real chords
        // before their own arc sub-edges get a chance to be mistaken for one.
        let plen = |e: &(u32, u32)| mesh.vertices[e.0 as usize].distance(&mesh.vertices[e.1 as usize]);
        boundary.sort_by(|x, y| plen(y).partial_cmp(&plen(x)).unwrap_or(std::cmp::Ordering::Equal));

        let mut new_tris: Vec<[u32; 3]> = Vec::new();
        let mut new_fids: Vec<u32> = Vec::new();
        // Chord triangles re-fanned in phase B (replaced, not kept).
        let mut removed: HashSet<usize> = HashSet::new();
        // Vertices consumed by a fill this pass — keeps two overlapping fills from
        // fighting over the same chain (the next pass mops up any leftover).
        let mut touched: HashSet<u32> = HashSet::new();
        let mut progressed = false;

        // Unit flat normal of the single triangle on boundary edge `e`.
        let tri_normal = |e: (u32, u32)| -> Option<[f64; 3]> {
            let ti = *edge_tris.get(&key(e.0, e.1))?.first()?;
            let t = mesh.triangles[ti];
            let p = |i: u32| mesh.vertices[i as usize];
            let (a, b, c) = (p(t[0]), p(t[1]), p(t[2]));
            let u = [b.x() - a.x(), b.y() - a.y(), b.z() - a.z()];
            let v = [c.x() - a.x(), c.y() - a.y(), c.z() - a.z()];
            let n = [
                u[1] * v[2] - u[2] * v[1],
                u[2] * v[0] - u[0] * v[2],
                u[0] * v[1] - u[1] * v[0],
            ];
            let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
            (l > 1e-12).then(|| [n[0] / l, n[1] / l, n[2] / l])
        };

        for &(a, b) in &boundary {
            if edge_tris.get(&key(a, b)).map(Vec::len) != Some(1) {
                continue;
            }
            // The arc the chord (a, b) shortcuts.
            let Some(path) = boundary_path(&adj, a, b) else {
                continue;
            };
            if path.len() < 3 {
                continue; // need >=1 intermediate vertex to bound a lens
            }
            if path.iter().any(|v| touched.contains(v)) {
                continue;
            }
            // The arc must be a modest detour around the chord, not the long way
            // round the whole loop (which means (a, b) is an arc sub-edge, not the
            // chord) nor a genuine large open boundary. The real chord — longest, so
            // processed first — passes; an arc sub-edge's complementary path is the
            // rest of the loop and fails this, as does a true open hole.
            let chord = mesh.vertices[a as usize].distance(&mesh.vertices[b as usize]);
            if chord < 1e-9 {
                continue;
            }
            let arc_len: f64 = path
                .windows(2)
                .map(|w| mesh.vertices[w[0] as usize].distance(&mesh.vertices[w[1] as usize]))
                .sum();
            if arc_len > 8.0 * chord {
                continue;
            }
            // Fill the planar lens (chord + arc) by fanning from the chord endpoint
            // `a`: a circular segment is convex, so the fan tiles it cleanly through
            // the on-surface arc points. The filler belongs to the neighbour that
            // subdivided the arc (a planar end/top face, or a bored hole rim), so it
            // takes that face's id and is oriented to match its normal — seamless,
            // adds no geometry, and leaves every lens edge shared by two triangles.
            let arc_fid = mesh
                .face_ids
                .get(edge_tris[&key(path[0], path[1])][0])
                .copied()
                .unwrap_or(0);
            let narc = tri_normal((path[0], path[1]));
            for i in 1..path.len() - 1 {
                let (p0, p1, p2) = (a, path[i], path[i + 1]);
                if p1 == p0 || p2 == p0 || p1 == p2 {
                    continue;
                }
                let mut tri = [p0, p1, p2];
                if let Some(na) = narc {
                    let q = |i: u32| mesh.vertices[i as usize];
                    let (va, vb, vc) = (q(p0), q(p1), q(p2));
                    let u = [vb.x() - va.x(), vb.y() - va.y(), vb.z() - va.z()];
                    let v = [vc.x() - va.x(), vc.y() - va.y(), vc.z() - va.z()];
                    let fnv = [
                        u[1] * v[2] - u[2] * v[1],
                        u[2] * v[0] - u[0] * v[2],
                        u[0] * v[1] - u[1] * v[0],
                    ];
                    if fnv[0] * na[0] + fnv[1] * na[1] + fnv[2] * na[2] < 0.0 {
                        tri = [p0, p2, p1];
                    }
                }
                new_tris.push(tri);
                new_fids.push(arc_fid);
            }
            for &v in &path {
                touched.insert(v);
            }
            progressed = true;
        }

        // Phase B — OPEN arc chains. The neighbour subdivided a shared boundary
        // that the owner spanned with a single *internal* chord edge, so the
        // chain's two ends reconnect only through the mesh interior and phase A's
        // all-boundary path can't reach them. Walk each open chain (its ends have
        // boundary-degree 1) and re-fan the owner's chord triangle through it, so
        // the owner picks up the neighbour's sub-edges. This is the tangent-line /
        // end-cap lens on an analytic fillet.
        let mut starts: Vec<u32> = adj
            .iter()
            .filter(|(_, ns)| ns.len() == 1)
            .map(|(&v, _)| v)
            .collect();
        starts.sort_unstable();
        let mut walked: HashSet<(u32, u32)> = HashSet::new();
        for start in starts {
            if touched.contains(&start) {
                continue;
            }
            let Some(&first) = adj.get(&start).and_then(|n| n.first()) else {
                continue;
            };
            if walked.contains(&key(start, first)) {
                continue;
            }
            let mut chain = vec![start];
            let (mut prev, mut cur) = (start, first);
            walked.insert(key(prev, cur));
            loop {
                chain.push(cur);
                let nexts: Vec<u32> =
                    adj[&cur].iter().copied().filter(|&w| w != prev).collect();
                if nexts.len() != 1 {
                    break;
                }
                let nxt = nexts[0];
                if walked.contains(&key(cur, nxt)) {
                    break;
                }
                walked.insert(key(cur, nxt));
                prev = cur;
                cur = nxt;
            }
            if chain.len() < 3 {
                continue;
            }
            let (a, b) = (chain[0], *chain.last().unwrap());
            if a == b || chain.iter().any(|v| touched.contains(v)) {
                continue;
            }
            // The arc belongs to one face; the chord that shortcuts it is owned by
            // the *other* face — re-fan that one through the chain.
            let arc_fid = mesh
                .face_ids
                .get(edge_tris[&key(chain[0], chain[1])][0])
                .copied()
                .unwrap_or(0);
            let Some(chord_tris) = edge_tris.get(&key(a, b)) else {
                continue;
            };
            for &ti in chord_tris {
                if removed.contains(&ti)
                    || mesh.face_ids.get(ti).copied().unwrap_or(0) == arc_fid
                {
                    continue;
                }
                let tri = mesh.triangles[ti];
                let opp = if tri[0] != a && tri[0] != b {
                    tri[0]
                } else if tri[1] != a && tri[1] != b {
                    tri[1]
                } else {
                    tri[2]
                };
                let forward = (tri[0] == a && tri[1] == b)
                    || (tri[1] == a && tri[2] == b)
                    || (tri[2] == a && tri[0] == b);
                let ch: Vec<u32> = if forward {
                    chain.clone()
                } else {
                    chain.iter().rev().copied().collect()
                };
                let fid = mesh.face_ids.get(ti).copied().unwrap_or(0);
                for w in ch.windows(2) {
                    if w[0] == opp || w[1] == opp {
                        continue;
                    }
                    new_tris.push([w[0], w[1], opp]);
                    new_fids.push(fid);
                }
                removed.insert(ti);
                for &v in &chain {
                    touched.insert(v);
                }
                progressed = true;
                break;
            }
        }

        if !progressed {
            break;
        }
        if removed.is_empty() {
            mesh.triangles.extend(new_tris);
            mesh.face_ids.extend(new_fids);
        } else {
            let mut out_tris = Vec::with_capacity(mesh.triangles.len() + new_tris.len());
            let mut out_fids = Vec::with_capacity(out_tris.capacity());
            for (ti, t) in mesh.triangles.iter().enumerate() {
                if removed.contains(&ti) {
                    continue;
                }
                out_tris.push(*t);
                out_fids.push(mesh.face_ids.get(ti).copied().unwrap_or(0));
            }
            out_tris.extend(new_tris);
            out_fids.extend(new_fids);
            mesh.triangles = out_tris;
            mesh.face_ids = out_fids;
        }
    }
}

fn find_next_non_collapsed_u(
    edges: &[openrcad_topo::Edge],
    curr_idx: usize,
    surface: &GeomSurface,
    pole_pt: Pnt,
) -> Option<f64> {
    let n = edges.len();
    for offset in 1..=n {
        let idx = (curr_idx + offset) % n;
        let next_edge = &edges[idx];
        if let Some(curve) = next_edge.curve() {
            let t0 = next_edge.first();
            let t1 = next_edge.last();
            let n_samples = 5;
            for i in 0..=n_samples {
                let t = t0 + (t1 - t0) * (i as f64) / (n_samples as f64);
                let p = curve.point(t);
                if p.distance(&pole_pt) > 1e-5 {
                    let (u, _) = project_point(surface, p, None);
                    return Some(u);
                }
            }
        } else {
            let p = next_edge.start().point();
            if p.distance(&pole_pt) > 1e-5 {
                let (u, _) = project_point(surface, p, None);
                return Some(u);
            }
        }
    }
    None
}

fn find_prev_non_collapsed_u(
    edges: &[openrcad_topo::Edge],
    curr_idx: usize,
    surface: &GeomSurface,
    pole_pt: Pnt,
) -> Option<f64> {
    let n = edges.len();
    for offset in 1..=n {
        let idx = (curr_idx + n - offset) % n;
        let prev_edge = &edges[idx];
        if let Some(curve) = prev_edge.curve() {
            let t0 = prev_edge.first();
            let t1 = prev_edge.last();
            let n_samples = 5;
            for i in (0..=n_samples).rev() {
                let t = t0 + (t1 - t0) * (i as f64) / (n_samples as f64);
                let p = curve.point(t);
                if p.distance(&pole_pt) > 1e-5 {
                    let (u, _) = project_point(surface, p, None);
                    return Some(u);
                }
            }
        } else {
            let p = prev_edge.start().point();
            if p.distance(&pole_pt) > 1e-5 {
                let (u, _) = project_point(surface, p, None);
                return Some(u);
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::{Dir, Pnt};
    use openrcad_geom::{GeomSurface, Plane};
    use openrcad_topo::{Edge, Face, Orientation, Wire};

    fn square_face(z: f64, orientation: Orientation) -> Face {
        let wire = Wire::from_edges([
            Edge::between_points(Pnt::new(0.0, 0.0, z), Pnt::new(1.0, 0.0, z)),
            Edge::between_points(Pnt::new(1.0, 0.0, z), Pnt::new(1.0, 1.0, z)),
            Edge::between_points(Pnt::new(1.0, 1.0, z), Pnt::new(0.0, 1.0, z)),
            Edge::between_points(Pnt::new(0.0, 1.0, z), Pnt::new(0.0, 0.0, z)),
        ]);
        Face::with_wires(
            Some(GeomSurface::plane(Plane::from_point_normal(
                Pnt::origin(),
                Dir::dz(),
            ))),
            Some(wire),
            Vec::new(),
            orientation,
        )
    }

    fn normal_z(mesh: &TriangleMesh, tri: [u32; 3]) -> f64 {
        let a = mesh.vertices[tri[0] as usize];
        let b = mesh.vertices[tri[1] as usize];
        let c = mesh.vertices[tri[2] as usize];
        (b - a).cross(&(c - a)).z()
    }

    #[test]
    fn recovers_missing_constrained_edge_by_flipping() {
        let points = vec![
            Pnt2d::new(0.0, 0.0),
            Pnt2d::new(1.0, 0.0),
            Pnt2d::new(1.0, 1.0),
            Pnt2d::new(0.0, 1.0),
        ];
        let tris = vec![Tri { a: 0, b: 1, c: 2 }, Tri { a: 0, b: 2, c: 3 }];

        let recovered = recover_constrained_edges(tris, &points, &[(1, 3)]);

        assert!(mesh_has_edge(&recovered, 1, 3));
    }

    #[test]
    fn tessellated_winding_follows_face_orientation() {
        let forward = tessellate_face_local(&square_face(0.0, Orientation::Forward), 0.01, 0);
        let reversed = tessellate_face_local(&square_face(0.0, Orientation::Reversed), 0.01, 0);

        assert!(!forward.triangles.is_empty());
        assert!(!reversed.triangles.is_empty());
        assert!(forward
            .triangles
            .iter()
            .all(|&tri| normal_z(&forward, tri) > 0.0));
        assert!(reversed
            .triangles
            .iter()
            .all(|&tri| normal_z(&reversed, tri) < 0.0));
    }

    #[test]
    fn boundary_vertices_keep_exact_edge_curve_positions() {
        let mesh = tessellate_face_local(&square_face(0.25, Orientation::Forward), 0.01, 0);

        assert!(!mesh.vertices.is_empty());
        assert!(mesh.vertices.iter().all(|p| (p.z() - 0.25).abs() < 1e-12));
    }
}
