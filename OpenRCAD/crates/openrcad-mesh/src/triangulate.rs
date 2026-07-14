use std::collections::{HashMap, HashSet};

use crate::TriangleMesh;
use openrcad_foundation::{tolerance::CONFUSION, Pnt, Pnt2d, Vec as GeomVec};
use openrcad_geom::{Curve, GeomCurve, GeomSurface, Surface};
use openrcad_topo::{orientation::Orientation, Face, PcurveData};

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

/// A triangle node in the incremental Bowyer-Watson store: CCW vertices plus
/// neighbour adjacency. Edge `e` is `(v[e], v[(e+1)%3])`; `nbr[e]` is the
/// triangle across that edge (`None` on the super-triangle hull).
#[derive(Clone, Copy)]
struct DtNode {
    v: [usize; 3],
    nbr: [Option<usize>; 3],
    alive: bool,
}

/// Walk from `start` toward `p` across neighbour links (Lawson's oriented
/// walk). Triangles are CCW, so `p` strictly right of a directed edge means
/// the containing triangle lies across that edge. Returns a triangle whose
/// interior (or boundary) holds `p`, falling back to `start` if the walk
/// exceeds its step cap (degenerate geometry — the caller re-checks with the
/// circumcircle test anyway).
fn dt_locate(nodes: &[DtNode], pts: &[Pnt2d], start: usize, p: Pnt2d) -> usize {
    let mut cur = start;
    for _ in 0..nodes.len() + 3 {
        let t = &nodes[cur];
        let mut moved = false;
        for e in 0..3 {
            let a = pts[t.v[e]];
            let b = pts[t.v[(e + 1) % 3]];
            if ccw(a, b, p) < -1e-14 {
                if let Some(n) = t.nbr[e] {
                    cur = n;
                    moved = true;
                    break;
                }
            }
        }
        if !moved {
            return cur;
        }
    }
    cur
}

/// Bowyer-Watson 2D Delaunay triangulation — incremental insertion with
/// neighbour adjacency and an oriented point-location walk, so each insertion
/// touches only its local cavity instead of scanning every triangle (the naive
/// full-scan version made large trimmed-cylinder faces quadratic: ~90 ms per
/// call at ~2700 points, re-run each refinement round). Points are inserted in
/// input order and cavities are traversed in sorted index order, so the result
/// is deterministic.
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

    // Super-triangle enclosing all points (CCW: bottom-left, bottom-right, top).
    let sp0 = Pnt2d::new(mid_x - 20.0 * dmax - 1.0, mid_y - 20.0 * dmax - 1.0);
    let sp1 = Pnt2d::new(mid_x + 20.0 * dmax + 1.0, mid_y - 20.0 * dmax - 1.0);
    let sp2 = Pnt2d::new(mid_x, mid_y + 20.0 * dmax + 1.0);

    let mut all_points = points.to_vec();
    let s0_idx = all_points.len();
    all_points.push(sp0);
    all_points.push(sp1);
    all_points.push(sp2);

    let mut nodes: Vec<DtNode> = vec![DtNode {
        v: [s0_idx, s0_idx + 1, s0_idx + 2],
        nbr: [None; 3],
        alive: true,
    }];
    let mut last_alive = 0usize;

    let mut cavity: Vec<usize> = Vec::new();
    let mut stack: Vec<usize> = Vec::new();
    let mut in_cavity: HashSet<usize> = HashSet::new();

    for i in 0..points.len() {
        let p = all_points[i];

        // Seed the cavity at the triangle containing `p`; if the walk's answer
        // fails the circumcircle test (p duplicates an existing vertex, or the
        // walk hit a degenerate cycle), scan for any violated triangle before
        // giving up on the point (matching the old full-scan behaviour).
        let mut seed = dt_locate(&nodes, &all_points, last_alive, p);
        let bad = |t: &DtNode| {
            in_circumcircle(
                p,
                all_points[t.v[0]],
                all_points[t.v[1]],
                all_points[t.v[2]],
            )
        };
        if !nodes[seed].alive || !bad(&nodes[seed]) {
            match nodes.iter().position(|t| t.alive && bad(t)) {
                Some(s) => seed = s,
                None => continue, // duplicate point: no triangle violated
            }
        }

        // Grow the cavity: every connected triangle whose circumcircle holds p.
        cavity.clear();
        stack.clear();
        in_cavity.clear();
        stack.push(seed);
        in_cavity.insert(seed);
        while let Some(t_idx) = stack.pop() {
            cavity.push(t_idx);
            for e in 0..3 {
                if let Some(n) = nodes[t_idx].nbr[e] {
                    if nodes[n].alive && !in_cavity.contains(&n) && bad(&nodes[n]) {
                        in_cavity.insert(n);
                        stack.push(n);
                    }
                }
            }
        }
        // Deterministic boundary order regardless of BFS traversal order.
        cavity.sort_unstable();

        // Boundary edges of the cavity: edge whose neighbour is outside. The
        // cavity is star-shaped around p and triangles are CCW, so each
        // boundary edge (a, b) keeps p to its left → fan triangle (a, b, p)
        // is CCW by construction.
        let mut fan: Vec<usize> = Vec::new();
        let mut open_edges: HashMap<usize, (usize, usize)> = HashMap::new();
        for &t_idx in &cavity {
            let t = nodes[t_idx];
            for e in 0..3 {
                let outside = match t.nbr[e] {
                    Some(n) => !in_cavity.contains(&n),
                    None => true,
                };
                if !outside {
                    continue;
                }
                let a = t.v[e];
                let b = t.v[(e + 1) % 3];
                let new_idx = nodes.len();
                nodes.push(DtNode {
                    v: [a, b, i],
                    nbr: [t.nbr[e], None, None],
                    alive: true,
                });
                // Re-point the outside neighbour at the new fan triangle.
                if let Some(out) = t.nbr[e] {
                    let key = edge_key(a, b);
                    for oe in 0..3 {
                        let ov = nodes[out].v;
                        if edge_key(ov[oe], ov[(oe + 1) % 3]) == key {
                            nodes[out].nbr[oe] = Some(new_idx);
                            break;
                        }
                    }
                }
                // Wire fan siblings: edge 1 is (b, i), edge 2 is (i, a). Two
                // fan triangles meet along the spoke through a shared hull
                // vertex; key each open spoke by that vertex.
                for (edge_slot, hull_v) in [(1usize, b), (2usize, a)] {
                    if let Some((other_idx, other_slot)) = open_edges.remove(&hull_v) {
                        nodes[new_idx].nbr[edge_slot] = Some(other_idx);
                        nodes[other_idx].nbr[other_slot] = Some(new_idx);
                    } else {
                        open_edges.insert(hull_v, (new_idx, edge_slot));
                    }
                }
                fan.push(new_idx);
            }
        }
        for &t_idx in &cavity {
            nodes[t_idx].alive = false;
        }
        if let Some(&f) = fan.last() {
            last_alive = f;
        }
    }

    nodes
        .iter()
        .filter(|t| {
            t.alive && t.v[0] < points.len() && t.v[1] < points.len() && t.v[2] < points.len()
        })
        .map(|t| Tri {
            a: t.v[0],
            b: t.v[1],
            c: t.v[2],
        })
        .collect()
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

fn find_crossing_edges(
    tris: &[Tri],
    points: &[Pnt2d],
    a: usize,
    b: usize,
    locked_edges: &HashSet<(usize, usize)>,
) -> Vec<(usize, usize, usize, usize)> {
    let edge_map = build_edge_map(tris);
    // Scan candidate edges in TRIANGLE order (a stable Vec), not HashMap order.
    // Constraint recovery flips what this returns, so a randomly-seeded HashMap
    // order would recover the same constrained triangulation along a different
    // flip path — a different (still valid) mesh per process, which cascades
    // into the crack-fix point insertions and changes vertex/triangle counts
    // run to run (the flaky circular-bite fillet display). Triangle order is
    // reproducible. ALL crossing edges are returned (deduplicated), because the
    // first one is frequently unflippable near sliver/collinear configurations
    // (a non-convex quad) — the caller must be able to flip a later crossing
    // first and come back, or recovery wedges and leaves a boundary hole.
    let mut seen = HashSet::new();
    let mut out = Vec::new();
    for t in tris {
        for (c, d) in [(t.a, t.b), (t.b, t.c), (t.c, t.a)] {
            if c == a || c == b || d == a || d == b || locked_edges.contains(&edge_key(c, d)) {
                continue;
            }
            if !seen.insert(edge_key(c, d)) {
                continue;
            }
            let Some(adj) = edge_map.get(&edge_key(c, d)) else {
                continue;
            };
            if adj.len() == 2
                && segments_intersect_strict(points[a], points[b], points[c], points[d])
            {
                out.push((c, d, adj[0], adj[1]));
            }
        }
    }
    out
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

            let crossings = find_crossing_edges(&tris, points, a, b, &locked_edges);
            if crossings.is_empty() {
                break;
            }
            // Flip the first flippable crossing. The first crossing alone is
            // not enough: near slivers it is often a non-convex quad, and
            // flipping a later crossing first un-wedges it on the next pass.
            let mut flipped = false;
            for crossing in crossings {
                if flip_edge(&mut tris, points, crossing) {
                    flipped = true;
                    flips += 1;
                    break;
                }
            }
            if !flipped {
                break;
            }
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
        GeomSurface::Torus(tor) => {
            // P = C + (R + r·cos v)·e_r(u) + r·sin v·Z, so u is the angle about
            // the main axis and v the tube angle measured from the outer equator
            // (radial distance ρ − R, axial height z).
            let diff = pt - tor.position().location();
            let dx = diff.dot(&GeomVec::from_dir(tor.position().x_direction()));
            let dy = diff.dot(&GeomVec::from_dir(tor.position().y_direction()));
            let dz = diff.dot(&GeomVec::from_dir(tor.position().direction()));
            let rho = dx.hypot(dy);
            if rho < CONFUSION {
                // On the axis: u is undefined (and v nearly so) — fall back to
                // the caller's hint like the sphere/cone poles do.
                return hint.unwrap_or((0.0, 0.0));
            }
            let mut u = dy.atan2(dx);
            if u < 0.0 {
                u += 2.0 * std::f64::consts::PI;
            }
            let radial = rho - tor.major_radius();
            if radial.abs() < CONFUSION && dz.abs() < CONFUSION {
                // On the tube-centre circle: v is undefined.
                return (u, hint.map(|h| h.1).unwrap_or(0.0));
            }
            let mut v = dz.atan2(radial);
            if v < 0.0 {
                v += 2.0 * std::f64::consts::PI;
            }
            return (u, v);
        }
        GeomSurface::Ruled(r) => {
            // Helical bands (thread walls) span many turns: Newton from the
            // domain midpoint converges to the wrong turn, so project
            // analytically (angle + hint/height turn disambiguation).
            if let Some(uv) = r.helical_uv_hinted(pt, hint) {
                return uv;
            }
            if matches!(r.curve1, GeomCurve::BSpline(_))
                && matches!(r.curve2, GeomCurve::BSpline(_))
            {
                if let Some(uv) = project_compact_ruled(r, pt, hint) {
                    return uv;
                }
            }
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

        u = clamp_to_ordered_bounds(u, u_min, u_max);
        v = clamp_to_ordered_bounds(v, v_min, v_max);

        if du.abs() < 1e-9 && dv.abs() < 1e-9 {
            break;
        }
    }

    (u, v)
}

/// Robust projection for a compact ruled patch with matching finite rail
/// domains. The concave fillet miter uses two exact rational B-spline quarter
/// circles; Newton from the surface midpoint can converge to the wrong boundary
/// branch, so minimize distance to each ruling first and recover its linear `v`.
fn project_compact_ruled(
    ruled: &openrcad_geom::RuledSurface,
    point: Pnt,
    hint: Option<(f64, f64)>,
) -> Option<(f64, f64)> {
    let (a0, a1) = ruled.curve1.bounds();
    let (b0, b1) = ruled.curve2.bounds();
    if ![a0, a1, b0, b1].iter().all(|value| value.is_finite()) {
        return None;
    }
    let lo = a0.max(b0);
    let hi = a1.min(b1);
    if hi - lo <= CONFUSION {
        return None;
    }
    let evaluate = |u: f64| {
        let p0 = ruled.curve1.point(u);
        let p1 = ruled.curve2.point(u);
        let ruling = p1 - p0;
        let len2 = ruling.dot(&ruling);
        let v = if len2 > CONFUSION * CONFUSION {
            ((point - p0).dot(&ruling) / len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let projected = p0 + ruling * v;
        ((projected - point).magnitude_squared(), v)
    };

    const STEPS: usize = 32;
    let step = (hi - lo) / STEPS as f64;
    let mut best_u = hint.map_or(lo, |(u, _)| u.clamp(lo, hi));
    let mut best = evaluate(best_u);
    for index in 0..=STEPS {
        let u = lo + index as f64 * step;
        let candidate = evaluate(u);
        if candidate.0 < best.0 {
            best_u = u;
            best = candidate;
        }
    }

    let mut left = (best_u - step).max(lo);
    let mut right = (best_u + step).min(hi);
    for _ in 0..28 {
        let u1 = left + (right - left) / 3.0;
        let u2 = right - (right - left) / 3.0;
        if evaluate(u1).0 <= evaluate(u2).0 {
            right = u2;
        } else {
            left = u1;
        }
    }
    best_u = 0.5 * (left + right);
    best = evaluate(best_u);
    Some((best_u, best.1))
}

fn clamp_to_ordered_bounds(value: f64, min: f64, max: f64) -> f64 {
    match (min.is_finite(), max.is_finite()) {
        (true, true) => {
            let (lo, hi) = if min <= max { (min, max) } else { (max, min) };
            if value.is_nan() {
                lo
            } else {
                value.max(lo).min(hi)
            }
        }
        (true, false) => {
            if value.is_nan() {
                min
            } else {
                value.max(min)
            }
        }
        (false, true) => {
            if value.is_nan() {
                max
            } else {
                value.min(max)
            }
        }
        (false, false) => value,
    }
}

/// Periodic unwrapping to prevent jumps across seams.
pub fn unwrap_coordinate(val: f64, prev: f64, period: f64) -> f64 {
    let closure = |i: i32| val + i as f64 * period;
    (-2..=2)
        .map(closure)
        .min_by(|a, b| (a - prev).abs().partial_cmp(&(b - prev).abs()).unwrap())
        .unwrap()
}

/// Resolve one boundary sample in the face's parameter space.
///
/// Stored coedge pcurves are the only boundary representation consumed by the
/// tessellator. Legacy topology must first pass through the mesh crate's named
/// compatibility adapter, which attaches and validates pcurves. Periodic values
/// are aligned to the previous sample without destroying an explicitly
/// unwrapped pcurve range.
fn boundary_uv(
    surface: &GeomSurface,
    pcurve: &PcurveData,
    natural_fraction: f64,
    previous: Option<(f64, f64)>,
) -> Pnt2d {
    let uv = pcurve.point_at_fraction(natural_fraction);
    let (mut u, mut v) = (uv.x(), uv.y());

    if let Some((previous_u, previous_v)) = previous {
        let u_period = pcurve
            .periodicity
            .u_period
            .or_else(|| surface.is_uclosed().then_some(core::f64::consts::TAU));
        let v_period = pcurve
            .periodicity
            .v_period
            .or_else(|| surface.is_vclosed().then_some(core::f64::consts::TAU));
        if let Some(period) = u_period {
            u = unwrap_coordinate(u, previous_u, period);
        }
        if let Some(period) = v_period {
            v = unwrap_coordinate(v, previous_v, period);
        }
    }

    Pnt2d::new(u, v)
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
    sample_interior_points_budget(
        surf,
        u_min,
        u_max,
        v_min,
        v_max,
        chord_err,
        std::f64::consts::PI,
    )
}

/// [`sample_interior_points`] with an angular budget: cylinder hoop density is
/// capped at `angle_err` radians per facet so small-radius rounds shade as
/// smoothly as large ones. Axial generators receive only bounded triangulation
/// support because straight directions have no chordal error.
pub fn sample_interior_points_budget(
    surf: &GeomSurface,
    u_min: f64,
    u_max: f64,
    v_min: f64,
    v_max: f64,
    chord_err: f64,
    angle_err: f64,
) -> Vec<Pnt2d> {
    let angle_err = sanitize_angle(angle_err);
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
            u_inset = 0.05;
            let r = cyl.radius();
            let err = chord_err.max(CONFUSION);
            let theta = cylinder_step_angle(r, err).min(angle_err);
            let span = u_max - u_min;
            let nu = f64::max(2.0, (span / theta).ceil()) as usize;
            // A small fixed set of v support rows is sufficient. Cylinder
            // generators are exactly straight, so sizing axial rows from the
            // hoop chord budget made triangle count scale with extrusion length
            // without improving shape accuracy. Eight divisions retain enough
            // support for stable triangulation. Broad cylinder patches retain
            // a slightly denser seed plus axial refinement below; narrow fillet
            // patches remain constant with cylinder length.
            let nv = if span.abs() > 2.0 { 8 } else { 4 };
            (nu, nv)
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
            // A cone is ruled like a cylinder — give it the SAME grid, not a
            // single inset ring. The old `(n, 0)` with `u_inset=0.25` left a
            // sparse mid-height ring clustered into each face's centre, so its
            // Delaunay wove long facets that chorded ~13% inward off the
            // surface near the wide base. A full-u ring (small inset) plus
            // axial support rows fixes it; the ruled-edge refinement above then
            // splits any remaining inward chords.
            u_inset = 0.05;
            let r1 = cone.radius_at(v_min).abs();
            let r2 = cone.radius_at(v_max).abs();
            let r = f64::max(r1, r2);
            let err = chord_err.max(CONFUSION);
            let theta = cylinder_step_angle(r, err).min(angle_err);
            let span = u_max - u_min;
            let nu = f64::max(2.0, (span / theta).ceil()) as usize;
            // Axial support rows sized to the max-radius hoop step (straight
            // generators gain nothing from the angular cap, so use the chordal
            // budget as a target physical edge length).
            let target_len = f64::max(r * cylinder_step_angle(r, err), err);
            let v_span = (v_max - v_min).abs();
            let nv = f64::max(2.0, (v_span / target_len).ceil()) as usize;
            (nu, nv)
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
        GeomSurface::Ruled(ruled)
            if matches!(ruled.curve1, GeomCurve::Helix(_))
                || matches!(ruled.curve2, GeomCurve::Helix(_)) =>
        {
            // A helical band (thread wall) spans many turns, so the 3-point sag
            // probe below aliases across whole turns. Size u by hoop angle like
            // a cylinder; the ruling direction is straight, so a single mid-v
            // support row (v_divs = 0 path) is enough.
            let hoop_r = |c: &GeomCurve| -> f64 {
                match c {
                    GeomCurve::Helix(h) => {
                        f64::max(h.radius_at(u_min).abs(), h.radius_at(u_max).abs())
                    }
                    GeomCurve::Circle(c) => c.radius().abs(),
                    _ => 0.0,
                }
            };
            let r = f64::max(hoop_r(&ruled.curve1), hoop_r(&ruled.curve2));
            let err = chord_err.max(CONFUSION);
            let theta = cylinder_step_angle(r, err).min(angle_err);
            let span = (u_max - u_min).abs();
            let nu = (f64::max(2.0, (span / theta).ceil()) as usize).min(8192);
            (nu, 0)
        }
        GeomSurface::Ruled(_) => {
            // Adaptive, not a fixed 10×10: a skinned solid is thousands of tiny
            // ruled quads, and a blanket grid explodes them into ~200 triangles
            // each (a 40-station threaded cylinder hit 630k triangles). Measure
            // the real mid-chord sag in each direction; a straight-railed quad
            // that already sits within the chordal budget gets NO interior
            // points, while a genuinely curved ruled face subdivides until the
            // per-cell sag (which shrinks as 1/n²) meets the budget.
            let err = chord_err.max(CONFUSION);
            let mid_u = 0.5 * (u_min + u_max);
            let mid_v = 0.5 * (v_min + v_max);
            let sag = |a: Pnt, m: Pnt, b: Pnt| -> f64 {
                let chord_mid = Pnt::new(
                    0.5 * (a.x() + b.x()),
                    0.5 * (a.y() + b.y()),
                    0.5 * (a.z() + b.z()),
                );
                m.distance(&chord_mid)
            };
            let mut sag_u = 0.0_f64;
            for v in [v_min, mid_v, v_max] {
                sag_u = sag_u.max(sag(
                    surf.point(u_min, v),
                    surf.point(mid_u, v),
                    surf.point(u_max, v),
                ));
            }
            let mut sag_v = 0.0_f64;
            for u in [u_min, mid_u, u_max] {
                sag_v = sag_v.max(sag(
                    surf.point(u, v_min),
                    surf.point(u, mid_v),
                    surf.point(u, v_max),
                ));
            }
            let divs = |s: f64| -> usize {
                if s <= err {
                    0
                } else {
                    (((s / err).sqrt().ceil() as usize) + 1).min(10)
                }
            };
            (divs(sag_u), divs(sag_v))
        }
        GeomSurface::BSpline(_) | GeomSurface::Gregory(_) | GeomSurface::Offset(_) => (10, 10),
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

fn cylinder_step_angle(radius: f64, chord_err: f64) -> f64 {
    if radius > CONFUSION {
        2.0 * (2.0 * chord_err / radius).sqrt()
    } else {
        std::f64::consts::PI / 2.0
    }
}

/// The hoop radius of a ruled round surface at axial parameter `v`: constant
/// for a cylinder, `radius_at(v)` for a cone (which shrinks toward the apex).
/// `None` for surfaces that aren't ruled-round, so the ruled-edge refinement
/// leaves them alone.
fn ruled_hoop_radius(surf: &GeomSurface, v: f64) -> Option<f64> {
    match surf {
        GeomSurface::Cylinder(cyl) => Some(cyl.radius().abs()),
        GeomSurface::Cone(cone) => Some(cone.radius_at(v).abs()),
        _ => None,
    }
}

fn cylinder_uv_target_len(surf: &GeomSurface, chord_err: f64) -> Option<f64> {
    // A representative radius for the whole face: the cylinder's radius, or the
    // cone's reference radius (its scale near v=0). Per-edge decisions below
    // refine this with the LOCAL radius, so this only sets the global cap.
    let r = match surf {
        GeomSurface::Cylinder(cyl) => cyl.radius().abs(),
        GeomSurface::Cone(cone) => cone.ref_radius().abs().max(CONFUSION),
        _ => return None,
    };
    let err = chord_err.max(CONFUSION);
    let theta = cylinder_step_angle(r, err);
    Some(f64::max(r * theta, err))
}

fn surface_uv_segment_len(surf: &GeomSurface, a: Pnt2d, b: Pnt2d) -> f64 {
    let mid_v = 0.5 * (a.y() + b.y());
    match ruled_hoop_radius(surf, mid_v) {
        Some(r) => {
            let du = (b.x() - a.x()).abs() * r;
            let dv = (b.y() - a.y()).abs();
            du.hypot(dv)
        }
        None => a.distance(&b),
    }
}

/// `(hoop, axial, surface_len, sagitta, local_radius)` for a ruled-round edge,
/// using the LOCAL radius at the edge's mid-v (so a cone's base edges — larger
/// radius — get finer refinement than its apex edges).
fn cylinder_edge_metrics(
    surf: &GeomSurface,
    a: Pnt2d,
    b: Pnt2d,
) -> Option<(f64, f64, f64, f64, f64)> {
    let mid_v = 0.5 * (a.y() + b.y());
    let r = ruled_hoop_radius(surf, mid_v)?;
    let du = shortest_angle_delta(a.x(), b.x());
    let hoop = du * r;
    let axial = (a.y() - b.y()).abs();
    let surface_len = hoop.hypot(axial);
    let sagitta = if r > CONFUSION {
        r * (1.0 - (0.5 * du).cos())
    } else {
        0.0
    };
    Some((hoop, axial, surface_len, sagitta, r))
}

// The angular budget deliberately does NOT feed this predicate: shading density
// comes from the boundary/interior *sampling* (discretize_edge_curve_budget +
// sample_interior_points_budget); enforcing an angular cap on every mesh edge
// makes the midpoint-insert/re-Delaunay loop thrash on borderline diagonals and
// quadruples the wall for no visual gain (normals are analytic per vertex).
fn cylinder_edge_needs_refinement(
    surf: &GeomSurface,
    a: Pnt2d,
    b: Pnt2d,
    chord_err: f64,
    refine_axial: bool,
) -> bool {
    let Some((_, _, surface_len, sagitta, r)) = cylinder_edge_metrics(surf, a, b) else {
        return false;
    };
    let err = chord_err.max(CONFUSION);
    let target_len = f64::max(r * cylinder_step_angle(r, err), err);
    // Only hoop curvature creates approximation error. Broad cylinder patches
    // retain bounded-aspect support for symmetric mass properties and stable
    // face naming; narrow fillet patches skip axial splitting so a 48 mm
    // extrusion of a 0.25 mm round does not receive hundreds of useless rows.
    sagitta > err || (refine_axial && surface_len > target_len)
}

/// Like [`cylinder_edge_needs_refinement`] but for cones: split ONLY on the
/// inward hoop chord (`sagitta`), never on raw surface length. A tall thin
/// cone's long axial Delaunay edges have huge length but no inward chord, so
/// the length criterion would split them without bound; the facet banding on a
/// channel-cut cone (a thread cutting through a chamfer) is a wide-angle hoop
/// chord, which the sagitta catches with a bounded number of splits.
fn cone_edge_needs_refinement(surf: &GeomSurface, a: Pnt2d, b: Pnt2d, chord_err: f64) -> bool {
    let Some((_, _, _, sagitta, _)) = cylinder_edge_metrics(surf, a, b) else {
        return false;
    };
    sagitta > chord_err.max(CONFUSION)
}

fn uv_midpoint(a: Pnt2d, b: Pnt2d) -> Pnt2d {
    Pnt2d::new(0.5 * (a.x() + b.x()), 0.5 * (a.y() + b.y()))
}

fn shortest_angle_delta(a: f64, b: f64) -> f64 {
    let mut d = (a - b).abs();
    while d > std::f64::consts::TAU {
        d -= std::f64::consts::TAU;
    }
    if d > std::f64::consts::PI {
        std::f64::consts::TAU - d
    } else {
        d
    }
}

fn dist2_point_segment(p: Pnt2d, a: Pnt2d, b: Pnt2d) -> f64 {
    let abx = b.x() - a.x();
    let aby = b.y() - a.y();
    let apx = p.x() - a.x();
    let apy = p.y() - a.y();
    let len2 = abx * abx + aby * aby;
    if len2 <= 1e-24 {
        return apx * apx + apy * apy;
    }
    let t = ((apx * abx + apy * aby) / len2).clamp(0.0, 1.0);
    let dx = apx - t * abx;
    let dy = apy - t * aby;
    dx * dx + dy * dy
}

fn point_on_loop(p: Pnt2d, loop_pts: &[Pnt2d]) -> bool {
    if loop_pts.len() < 2 {
        return false;
    }
    let tol2 = 1e-14;
    for i in 0..loop_pts.len() {
        let a = loop_pts[i];
        let b = loop_pts[(i + 1) % loop_pts.len()];
        if dist2_point_segment(p, a, b) <= tol2 {
            return true;
        }
    }
    false
}

fn point_in_trim_region(p: Pnt2d, outer_pts: &[Pnt2d], inner_pts_list: &[Vec<Pnt2d>]) -> bool {
    if !is_point_in_polygon(p, outer_pts) && !point_on_loop(p, outer_pts) {
        return false;
    }
    for hole in inner_pts_list {
        if is_point_in_polygon(p, hole) && !point_on_loop(p, hole) {
            return false;
        }
    }
    true
}

/// Whether a cylindrical face is an ordinary untrimmed parameter rectangle:
/// two circular rim arcs joined by two straight generators. Such a patch is
/// exactly ruled along its axial direction, so the sampled boundary is enough
/// for stable triangulation and axial edge-length refinement adds no accuracy.
/// Trimmed boolean walls, interrupted cylinders, and faces with holes retain
/// the denser generic path.
fn is_untrimmed_cylinder_patch(
    surface: &GeomSurface,
    outer_pts: &[Pnt2d],
    inner_pts_list: &[Vec<Pnt2d>],
) -> bool {
    if !matches!(surface, GeomSurface::Cylinder(_))
        || !inner_pts_list.is_empty()
        || outer_pts.len() < 4
    {
        return false;
    }
    let (u_min, u_max, v_min, v_max) = outer_pts.iter().fold(
        (
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
        ),
        |(ulo, uhi, vlo, vhi), point| {
            (
                ulo.min(point.x()),
                uhi.max(point.x()),
                vlo.min(point.y()),
                vhi.max(point.y()),
            )
        },
    );
    let u_span = u_max - u_min;
    let v_span = v_max - v_min;
    // Native cylinder primitives use three 120° patches. Other spans commonly
    // belong to fillets or mixed-profile extrusions, where the generic
    // triangulator's support points also drive tangent-seam suppression and
    // must be retained (a capsule's 180° wall is the key regression).
    let native_patch_span = std::f64::consts::TAU / 3.0;
    if (u_span - native_patch_span).abs() > 0.05 || v_span <= 1.0e-9 {
        return false;
    }
    let u_tol = (u_span * 1.0e-7).max(1.0e-8);
    let v_tol = (v_span * 1.0e-7).max(1.0e-8);
    let mut sides = [false; 4];
    for point in outer_pts {
        let on_u_min = (point.x() - u_min).abs() <= u_tol;
        let on_u_max = (point.x() - u_max).abs() <= u_tol;
        let on_v_min = (point.y() - v_min).abs() <= v_tol;
        let on_v_max = (point.y() - v_max).abs() <= v_tol;
        if !(on_u_min || on_u_max || on_v_min || on_v_max) {
            return false;
        }
        sides[0] |= on_u_min;
        sides[1] |= on_u_max;
        sides[2] |= on_v_min;
        sides[3] |= on_v_max;
    }
    sides.into_iter().all(|present| present)
}

/// Triangulate an untrimmed cylinder parameter rectangle as a regular strip
/// between its two already-discretized rim arcs. Long patches reuse the axial
/// samples already established by the edge-first pass, keeping diagonals short
/// without introducing unmatched boundary vertices on adjacent faces.
///
/// Returns `None` if the two rims do not carry matching samples, in which case
/// the caller uses generic constrained Delaunay triangulation.
fn untrimmed_cylinder_strip_tris(
    points_2d: &mut Vec<Pnt2d>,
    points_3d: &mut Vec<Pnt>,
    surface: &GeomSurface,
    outer_indices: &[usize],
    chord_err: f64,
    bound_diagonals: bool,
) -> Option<Vec<Tri>> {
    let (v_min, v_max) = outer_indices
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), &index| {
            (lo.min(points_2d[index].y()), hi.max(points_2d[index].y()))
        });
    let v_span = v_max - v_min;
    if v_span <= 1.0e-9 {
        return None;
    }
    let v_tol = (v_span * 1.0e-7).max(1.0e-8);
    let mut lower: Vec<(f64, usize)> = outer_indices
        .iter()
        .copied()
        .filter(|&index| (points_2d[index].y() - v_min).abs() <= v_tol)
        .map(|index| (points_2d[index].x(), index))
        .collect();
    let mut upper: Vec<(f64, usize)> = outer_indices
        .iter()
        .copied()
        .filter(|&index| (points_2d[index].y() - v_max).abs() <= v_tol)
        .map(|index| (points_2d[index].x(), index))
        .collect();
    lower.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    upper.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    lower.dedup_by_key(|(_, index)| *index);
    upper.dedup_by_key(|(_, index)| *index);
    if lower.len() < 2 || lower.len() != upper.len() {
        return None;
    }
    let u_span = lower.last()?.0 - lower.first()?.0;
    let u_tol = (u_span.abs() * 1.0e-7).max(1.0e-8);
    if lower
        .iter()
        .zip(&upper)
        .any(|((lower_u, _), (upper_u, _))| (lower_u - upper_u).abs() > u_tol)
    {
        return None;
    }

    let use_axial_rows = if bound_diagonals {
        let target_len = cylinder_uv_target_len(surface, chord_err)?;
        // Extra rows are a display safeguard for genuinely long, slender
        // patches. Keeping ordinary cylinders on their two exact rim rows
        // preserves compact meshes and the tangent-seam normal treatment used
        // by short rounds and capsule profiles.
        const LONG_STRIP_MIN_TARGET_SPANS: f64 = 6.0;
        v_span > target_len * LONG_STRIP_MIN_TARGET_SPANS
    } else {
        false
    };

    let lower_row = lower.iter().map(|(_, index)| *index).collect::<Vec<_>>();
    let upper_row = upper.iter().map(|(_, index)| *index).collect::<Vec<_>>();
    let mut rows = vec![lower_row];
    if use_axial_rows {
        let u_min = lower.first()?.0;
        let u_max = lower.last()?.0;
        let mut left: Vec<(f64, usize)> = outer_indices
            .iter()
            .copied()
            .filter(|&index| (points_2d[index].x() - u_min).abs() <= u_tol)
            .map(|index| (points_2d[index].y(), index))
            .collect();
        let mut right: Vec<(f64, usize)> = outer_indices
            .iter()
            .copied()
            .filter(|&index| (points_2d[index].x() - u_max).abs() <= u_tol)
            .map(|index| (points_2d[index].y(), index))
            .collect();
        let by_v = |a: &(f64, usize), b: &(f64, usize)| {
            a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal)
        };
        left.sort_by(by_v);
        right.sort_by(by_v);
        left.dedup_by_key(|(_, index)| *index);
        right.dedup_by_key(|(_, index)| *index);
        if left.len() < 2
            || left.len() != right.len()
            || left
                .iter()
                .zip(&right)
                .any(|((left_v, _), (right_v, _))| (left_v - right_v).abs() > v_tol)
        {
            return None;
        }

        for ((v, left_index), (_, right_index)) in left
            .iter()
            .zip(&right)
            .skip(1)
            .take(left.len().saturating_sub(2))
        {
            let mut indices = Vec::with_capacity(lower.len());
            indices.push(*left_index);
            for &(u, _) in lower.iter().skip(1).take(lower.len().saturating_sub(2)) {
                let index = points_2d.len();
                points_2d.push(Pnt2d::new(u, *v));
                points_3d.push(surface.point(u, *v));
                indices.push(index);
            }
            indices.push(*right_index);
            rows.push(indices);
        }
    }
    rows.push(upper_row);

    let mut tris = Vec::with_capacity((lower.len() - 1) * (rows.len() - 1) * 2);
    for rows in rows.windows(2) {
        for i in 0..lower.len() - 1 {
            let a = rows[0][i];
            let b = rows[0][i + 1];
            let c = rows[1][i];
            let d = rows[1][i + 1];
            tris.push(Tri { a, b, c });
            tris.push(Tri { a: b, b: d, c });
        }
    }
    Some(tris)
}

fn triangle_in_trim_region(
    pa: Pnt2d,
    pb: Pnt2d,
    pc: Pnt2d,
    outer_pts: &[Pnt2d],
    inner_pts_list: &[Vec<Pnt2d>],
    check_edge_midpoints: bool,
) -> bool {
    let centroid = Pnt2d::new(
        (pa.x() + pb.x() + pc.x()) / 3.0,
        (pa.y() + pb.y() + pc.y()) / 3.0,
    );
    if !point_in_trim_region(centroid, outer_pts, inner_pts_list) {
        return false;
    }
    if !check_edge_midpoints {
        return true;
    }
    [
        uv_midpoint(pa, pb),
        uv_midpoint(pb, pc),
        uv_midpoint(pc, pa),
    ]
    .into_iter()
    .all(|p| point_in_trim_region(p, outer_pts, inner_pts_list))
}

fn trimmed_constrained_tris(
    points: &[Pnt2d],
    constraints: &[(usize, usize)],
    outer_pts: &[Pnt2d],
    inner_pts_list: &[Vec<Pnt2d>],
    wants_ccw: bool,
    check_edge_midpoints: bool,
) -> Vec<Tri> {
    let dbg = std::env::var("ORC_DEBUG_TRI").is_ok();
    let tris = recover_constrained_edges(delaunay_triangulate(points), points, constraints);
    if dbg {
        for &(a, b) in constraints {
            if a != b && !mesh_has_edge(&tris, a, b) {
                eprintln!(
                    "tri dbg: UNRECOVERED constraint ({:.4},{:.4})-({:.4},{:.4})",
                    points[a].x(),
                    points[a].y(),
                    points[b].x(),
                    points[b].y()
                );
            }
        }
    }
    let mut out = Vec::new();
    for t in tris {
        let pa = points[t.a];
        let pb = points[t.b];
        let pc = points[t.c];
        if !triangle_in_trim_region(pa, pb, pc, outer_pts, inner_pts_list, check_edge_midpoints) {
            if dbg {
                eprintln!(
                    "tri dbg: CULLED ({:.4},{:.4}) ({:.4},{:.4}) ({:.4},{:.4})",
                    pa.x(),
                    pa.y(),
                    pb.x(),
                    pb.y(),
                    pc.x(),
                    pc.y()
                );
            }
            continue;
        }
        let tri_ccw = ccw(pa, pb, pc) > 0.0;
        if tri_ccw == wants_ccw {
            out.push(t);
        } else {
            out.push(Tri {
                a: t.a,
                b: t.c,
                c: t.b,
            });
        }
    }
    out
}

#[allow(clippy::too_many_arguments)]
fn refine_cylinder_tris(
    surface: &GeomSurface,
    points_2d: &mut Vec<Pnt2d>,
    points_3d: &mut Vec<Pnt>,
    point_map: &mut HashMap<(i64, i64), usize>,
    constraints: &[(usize, usize)],
    outer_pts: &[Pnt2d],
    inner_pts_list: &[Vec<Pnt2d>],
    wants_ccw: bool,
    chord_err: f64,
    // When the face's boundary polylines are shared with its neighbours
    // (shared-edge discretization), constraint edges must never be split —
    // inserting a boundary midpoint on the curved surface would move this
    // face's boundary off the neighbour's and re-open the crack the shared
    // pass exists to prevent.
    allow_constraint_split: bool,
) -> Vec<Tri> {
    const MAX_ITERS: usize = 16;
    const MAX_POINTS: usize = 20_000;

    // Only cylinders get the iterative interior midpoint-refinement pass —
    // cones get their accuracy from a dense ruled grid (below) instead, which
    // is far cheaper than re-Delaunay-ing a growing point set (a cone's
    // apex-ward facets have a vanishing target length that would drive the
    // iterative pass to explode the vertex count).
    let check_edge_midpoints = matches!(surface, GeomSurface::Cylinder(_));
    let (u_min, u_max) = outer_pts
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
            (lo.min(p.x()), hi.max(p.x()))
        });
    let refine_axial = (u_max - u_min).abs() > 2.0
        && !is_untrimmed_cylinder_patch(surface, outer_pts, inner_pts_list);
    let mut constraints = constraints.to_vec();
    let mut tris = trimmed_constrained_tris(
        points_2d,
        &constraints,
        outer_pts,
        inner_pts_list,
        wants_ccw,
        check_edge_midpoints,
    );
    if !check_edge_midpoints {
        return tris;
    }

    for _ in 0..MAX_ITERS {
        if points_2d.len() >= MAX_POINTS {
            break;
        }
        let constraint_edges: HashSet<(usize, usize)> =
            constraints.iter().map(|&(a, b)| edge_key(a, b)).collect();
        let mut seen_edges = HashSet::new();
        let mut candidates = Vec::new();
        for tri in &tris {
            for (a, b) in [(tri.a, tri.b), (tri.b, tri.c), (tri.c, tri.a)] {
                let key = edge_key(a, b);
                if !seen_edges.insert(key) {
                    continue;
                }
                let is_constraint = constraint_edges.contains(&key);
                if is_constraint && !allow_constraint_split {
                    continue;
                }
                let pa = points_2d[a];
                let pb = points_2d[b];
                if cylinder_edge_needs_refinement(surface, pa, pb, chord_err, refine_axial) {
                    let mid = uv_midpoint(pa, pb);
                    if point_in_trim_region(mid, outer_pts, inner_pts_list) {
                        candidates.push((a, b, is_constraint, mid));
                    }
                }
            }
        }
        if candidates.is_empty() {
            break;
        }

        let mut inserted = false;
        for (a, b, split_constraint, p2d) in candidates {
            if points_2d.len() >= MAX_POINTS {
                break;
            }
            let key = ((p2d.x() * 1e8) as i64, (p2d.y() * 1e8) as i64);
            if let Some(&id) = point_map.get(&key) {
                if split_constraint {
                    split_constraint_edge(&mut constraints, a, b, id);
                    inserted = true;
                }
                continue;
            }
            let id = points_2d.len();
            point_map.insert(key, id);
            points_2d.push(p2d);
            points_3d.push(surface.point(p2d.x(), p2d.y()));
            if split_constraint {
                split_constraint_edge(&mut constraints, a, b, id);
            }
            inserted = true;
        }
        if !inserted {
            break;
        }

        tris = trimmed_constrained_tris(
            points_2d,
            &constraints,
            outer_pts,
            inner_pts_list,
            wants_ccw,
            true,
        );
    }

    tris
}

fn split_constraint_edge(constraints: &mut Vec<(usize, usize)>, a: usize, b: usize, mid: usize) {
    for i in 0..constraints.len() {
        if constraints[i] == (a, b) {
            constraints[i] = (a, mid);
            constraints.push((mid, b));
            return;
        }
        if constraints[i] == (b, a) {
            constraints[i] = (b, mid);
            constraints.push((mid, a));
            return;
        }
    }
}

fn mesh_from_uv_tris(
    points_2d: &[Pnt2d],
    surface: &GeomSurface,
    wants_ccw: bool,
    points_3d: Vec<Pnt>,
    tris: Vec<Tri>,
    face_index: u32,
) -> TriangleMesh {
    let triangles = tris
        .into_iter()
        .map(|t| orient_triangle_to_surface(points_2d, &points_3d, surface, wants_ccw, t))
        .collect::<Vec<_>>();
    let face_ids = vec![face_index; triangles.len()];
    TriangleMesh::from_buffers_with_faces(points_3d, triangles, face_ids)
}

fn orient_triangle_to_surface(
    points_2d: &[Pnt2d],
    points_3d: &[Pnt],
    surface: &GeomSurface,
    wants_ccw: bool,
    tri: Tri,
) -> [u32; 3] {
    let mut out = [tri.a as u32, tri.b as u32, tri.c as u32];
    let pa = points_3d[tri.a];
    let pb = points_3d[tri.b];
    let pc = points_3d[tri.c];
    let normal = (pb - pa).cross(&(pc - pa));
    if normal.magnitude() <= 1.0e-12 {
        return out;
    }

    let ua = points_2d[tri.a];
    let ub = points_2d[tri.b];
    let uc = points_2d[tri.c];
    let u = (ua.x() + ub.x() + uc.x()) / 3.0;
    let v = (ua.y() + ub.y() + uc.y()) / 3.0;
    let (_, du, dv) = surface.d1(u, v);
    let mut desired = du.cross(&dv);
    if !wants_ccw {
        desired = -desired;
    }
    if desired.magnitude() > 1.0e-12 && normal.dot(&desired) < 0.0 {
        out.swap(1, 2);
    }
    out
}

pub(crate) fn refine_cylinder_mesh_edges(mesh: &mut TriangleMesh, faces: &[Face], chord_err: f64) {
    const MAX_ITERS: usize = 8;
    const MAX_VERTS: usize = 100_000;

    for _ in 0..MAX_ITERS {
        if mesh.vertices.len() >= MAX_VERTS {
            break;
        }

        let mut edge_tris: EdgeTriUses = HashMap::new();
        for (ti, tri) in mesh.triangles.iter().enumerate() {
            for &(a, b) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
                let ka = point_key_3d(mesh.vertices[a as usize]);
                let kb = point_key_3d(mesh.vertices[b as usize]);
                let key = if ka <= kb { (ka, kb) } else { (kb, ka) };
                edge_tris.entry(key).or_default().push((ti, a, b));
            }
        }

        let mut vertex_map: HashMap<(i64, i64, i64), u32> = mesh
            .vertices
            .iter()
            .enumerate()
            .map(|(i, p)| (point_key_3d(*p), i as u32))
            .collect();
        let mut removed = HashSet::new();
        let mut new_tris = Vec::new();
        let mut new_fids = Vec::new();
        let mut progressed = false;
        let mut edges: Vec<_> = edge_tris.keys().copied().collect();
        edges.sort_unstable();

        for key in edges {
            let Some(adj) = edge_tris.get(&key) else {
                continue;
            };
            if adj.len() < 2 || adj.iter().any(|(ti, _, _)| removed.contains(ti)) {
                continue;
            }
            let fid = mesh.face_ids.get(adj[0].0).copied().unwrap_or(0);
            if adj
                .iter()
                .any(|&(ti, _, _)| mesh.face_ids.get(ti).copied().unwrap_or(0) != fid)
            {
                continue;
            }
            let Some(face) = faces.get(fid as usize) else {
                continue;
            };
            // Cylinder walls AND cones get post-hoc mesh-edge refinement: a
            // channel-cut cone (a thread cutting through a chamfer) has a dense
            // helical boundary against a coarse interior grid, so its Delaunay
            // leaves a few skinny facets that chord inward off the surface — the
            // same inward-chord that `cylinder_edge_needs_refinement` splits,
            // using each edge's LOCAL hoop radius (`ruled_hoop_radius` returns
            // `radius_at(v)` for cones).
            let surface = match face.surface() {
                Some(GeomSurface::Cylinder(cyl)) => GeomSurface::Cylinder(*cyl),
                Some(GeomSurface::Cone(cone)) => GeomSurface::Cone(*cone),
                _ => continue,
            };
            let (_, a, b) = adj[0];
            let pa = mesh.vertices[a as usize];
            let pb = mesh.vertices[b as usize];
            if face_has_miter_seam(face) {
                continue;
            }
            let (ua, va) = project_point(&surface, pa, None);
            let (mut ub, vb) = project_point(&surface, pb, Some((ua, va)));
            ub = unwrap_coordinate(ub, ua, std::f64::consts::TAU);
            let a_uv = Pnt2d::new(ua, va);
            let b_uv = Pnt2d::new(ub, vb);
            let needs = if matches!(surface, GeomSurface::Cone(_)) {
                // Cones only refine on inward hoop chord (the sagitta), NOT on
                // raw surface length: a tall thin cone's long axial Delaunay
                // edges have huge length but zero inward chord, so the
                // length criterion would split them forever. The channel-cut
                // cone's facet banding is a wide-angle hoop chord, which
                // sagitta catches.
                cone_edge_needs_refinement(&surface, a_uv, b_uv, chord_err)
            } else {
                cylinder_edge_needs_refinement(&surface, a_uv, b_uv, chord_err, false)
            };
            if !needs {
                continue;
            }

            let mid_uv = uv_midpoint(a_uv, b_uv);
            let mid_point = surface.point(mid_uv.x(), mid_uv.y());
            let mid_key = point_key_3d(mid_point);
            let mid = if let Some(&idx) = vertex_map.get(&mid_key) {
                idx
            } else {
                if mesh.vertices.len() >= MAX_VERTS {
                    break;
                }
                let idx = mesh.vertices.len() as u32;
                mesh.vertices.push(mid_point);
                vertex_map.insert(mid_key, idx);
                idx
            };
            if mid == a || mid == b {
                continue;
            }

            for &(ti, ea, eb) in adj {
                if removed.contains(&ti) {
                    continue;
                }
                let Some((t0, t1)) = split_triangle_edge(mesh.triangles[ti], ea, eb, mid) else {
                    continue;
                };
                removed.insert(ti);
                new_tris.push(t0);
                new_fids.push(fid);
                new_tris.push(t1);
                new_fids.push(fid);
            }
            progressed = true;
        }

        if !progressed {
            break;
        }

        let mut out_tris = Vec::with_capacity(mesh.triangles.len() + new_tris.len());
        let mut out_fids = Vec::with_capacity(mesh.face_ids.len() + new_fids.len());
        for (ti, tri) in mesh.triangles.iter().enumerate() {
            if removed.contains(&ti) {
                continue;
            }
            out_tris.push(*tri);
            out_fids.push(mesh.face_ids.get(ti).copied().unwrap_or(0));
        }
        out_tris.extend(new_tris);
        out_fids.extend(new_fids);
        mesh.triangles = out_tris;
        mesh.face_ids = out_fids;
    }
}

fn face_has_miter_seam(face: &Face) -> bool {
    face.wires().into_iter().any(|wire| {
        wire.edges()
            .iter()
            .any(|edge| matches!(edge.curve(), Some(GeomCurve::Ellipse(_))))
    })
}

type PointKey3d = (i64, i64, i64);

/// Triangles using each welded (undirected) mesh edge: `(tri index, a, b)`.
type EdgeTriUses = HashMap<(PointKey3d, PointKey3d), Vec<(usize, u32, u32)>>;

fn point_key_3d(p: Pnt) -> PointKey3d {
    (
        (p.x() * 1e9).round() as i64,
        (p.y() * 1e9).round() as i64,
        (p.z() * 1e9).round() as i64,
    )
}

fn split_triangle_edge(tri: [u32; 3], a: u32, b: u32, mid: u32) -> Option<([u32; 3], [u32; 3])> {
    let split = |x, y, o| Some(([x, mid, o], [mid, y, o]));
    if (tri[0] == a && tri[1] == b) || (tri[0] == b && tri[1] == a) {
        split(tri[0], tri[1], tri[2])
    } else if (tri[1] == a && tri[2] == b) || (tri[1] == b && tri[2] == a) {
        split(tri[1], tri[2], tri[0])
    } else if (tri[2] == a && tri[0] == b) || (tri[2] == b && tri[0] == a) {
        split(tri[2], tri[0], tri[1])
    } else {
        None
    }
}

fn refine_surface_edge_params(
    surf: &GeomSurface,
    curve: &GeomCurve,
    params: &[f64],
    chord_err: f64,
) -> Vec<f64> {
    let Some(target_len) = cylinder_uv_target_len(surf, chord_err) else {
        return params.to_vec();
    };
    if params.len() < 2 || !target_len.is_finite() || target_len <= CONFUSION {
        return params.to_vec();
    }

    let mut uvs = Vec::with_capacity(params.len());
    let mut prev_hint = None;
    for &t in params {
        let p = curve.point(t);
        let (mut u, mut v) = project_point(surf, p, prev_hint);
        if let Some((pu, pv)) = prev_hint {
            if surf.is_uclosed() {
                u = unwrap_coordinate(u, pu, 2.0 * std::f64::consts::PI);
            }
            if surf.is_vclosed() {
                v = unwrap_coordinate(v, pv, 2.0 * std::f64::consts::PI);
            }
        }
        prev_hint = Some((u, v));
        uvs.push(Pnt2d::new(u, v));
    }

    let mut refined = Vec::new();
    for i in 0..(params.len() - 1) {
        let t0 = params[i];
        let t1 = params[i + 1];
        if refined
            .last()
            .map_or(true, |last: &f64| (*last - t0).abs() > 1e-12)
        {
            refined.push(t0);
        }

        let len = surface_uv_segment_len(surf, uvs[i], uvs[i + 1]);
        let mut divs = f64::max(1.0, (len / target_len).ceil()) as usize;
        if matches!(surf, GeomSurface::Cylinder(_)) && divs > 64 {
            // This is supplemental support for trimmed-wall triangulation, not
            // curve approximation (the curve sampler above already enforces
            // chord and angle error). Cap only pathological aspect ratios; normal
            // cylinders retain their denser symmetric support, while a 48 mm
            // extrusion of a 0.25 mm round avoids hundreds of straight-edge
            // subdivisions.
            divs = divs.min(8);
        }
        for j in 1..divs {
            let f = j as f64 / divs as f64;
            refined.push(t0 + f * (t1 - t0));
        }
    }

    if let Some(&last) = params.last() {
        refined.push(last);
    }
    refined.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap());
    refined.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    refined
}

/// Discretize an edge's curve to satisfy the chordal error budget.
pub fn discretize_edge_curve(curve: &GeomCurve, first: f64, last: f64, chord_err: f64) -> Vec<f64> {
    discretize_edge_curve_budget(curve, first, last, chord_err, std::f64::consts::PI)
}

/// Angular tolerances at or below this (or non-finite) mean "no angular cap".
fn sanitize_angle(angle_err: f64) -> f64 {
    if angle_err.is_finite() && angle_err > 1.0e-3 {
        angle_err
    } else {
        std::f64::consts::PI
    }
}

/// Discretize an edge's curve to satisfy both the chordal error budget and an
/// angular budget (max tangent turn per segment, radians). The angular budget
/// makes curved-edge density resolution-independent: a 0.5 mm fillet arc gets
/// the same facets-per-degree as a 20 mm bore rim, so small rounds never render
/// segmented just because their chordal sagitta happens to fit the tolerance.
pub fn discretize_edge_curve_budget(
    curve: &GeomCurve,
    first: f64,
    last: f64,
    chord_err: f64,
    angle_err: f64,
) -> Vec<f64> {
    let mut params = vec![first, last];
    let angle_err = sanitize_angle(angle_err);

    // Periodic curves alias the 3-point sag probe: a helix window spanning a
    // whole number of turns has chord endpoints AND midpoint at the same
    // angle (collinear, zero deviation) and identical tangents, so the
    // recursion would terminate immediately on a 6-turn rail. Seed quarter-turn
    // cells so every recursion interval is well under one period.
    let mut seeds = vec![first, last];
    if matches!(curve, GeomCurve::Helix(_)) {
        let step = std::f64::consts::FRAC_PI_2;
        let mut t = (first / step).floor() * step + step;
        while t < last - 1e-9 {
            if t > first + 1e-9 {
                seeds.push(t);
            }
            t += step;
        }
        seeds.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap());
        params.extend(seeds.iter().skip(1).take(seeds.len().saturating_sub(2)));
    }

    fn tangent_turn(curve: &GeomCurve, t0: f64, t1: f64) -> f64 {
        let (_, d0) = curve.d1(t0);
        let (_, d1) = curve.d1(t1);
        let (m0, m1) = (d0.magnitude(), d1.magnitude());
        if m0 <= CONFUSION || m1 <= CONFUSION {
            return 0.0;
        }
        (d0.dot(&d1) / (m0 * m1)).clamp(-1.0, 1.0).acos()
    }

    fn subdivide(
        curve: &GeomCurve,
        t0: f64,
        t1: f64,
        chord_err: f64,
        angle_err: f64,
        depth: usize,
        params: &mut Vec<f64>,
    ) {
        if depth > 10 {
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

        // Treat a segment exactly on budget as accepted. Analytic arcs split
        // into thirds accumulate a few ulps in their parameter endpoints; a
        // strict comparison made nominal 7.5° cells read microscopically over
        // a 7.5° budget and recursively split them to 3.75°.
        let angle_over_budget = tangent_turn(curve, t0, t1) > angle_err * (1.0 + 1.0e-9);
        if dev > chord_err || angle_over_budget {
            subdivide(curve, t0, tm, chord_err, angle_err, depth + 1, params);
            params.push(tm);
            subdivide(curve, tm, t1, chord_err, angle_err, depth + 1, params);
        }
    }

    for w in seeds.windows(2) {
        subdivide(curve, w[0], w[1], chord_err, angle_err, 0, &mut params);
    }
    params.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap());
    params.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    params
}

/// Canonical geometric key for a boundary edge, independent of which face (or
/// which arena edge — the boolean can leave coincident-but-distinct edges) is
/// looking at it: sorted quantized endpoints plus the curve midpoint. The
/// midpoint distinguishes the two arcs that can join one vertex pair.
pub type SharedEdgeKey = ((i64, i64, i64), (i64, i64, i64), (i64, i64, i64));

/// One canonical sample of a solid-wide shared edge polyline.
#[derive(Clone, Copy, Debug)]
pub struct SharedEdgeSample {
    /// Exact 3D boundary point shared by adjacent faces.
    pub point: Pnt,
    /// Normalized progress from the canonical first endpoint to the last.
    pub canonical_fraction: f64,
}

fn shared_key_point(p: Pnt) -> (i64, i64, i64) {
    (
        (p.x() * 1e6).round() as i64,
        (p.y() * 1e6).round() as i64,
        (p.z() * 1e6).round() as i64,
    )
}

fn shared_edge_key(edge: &openrcad_topo::Edge) -> Option<SharedEdgeKey> {
    let curve = edge.curve()?;
    let p0 = edge.start().point();
    let p1 = edge.end().point();
    // Collapsed/closed edges take the pole path in the face tessellator, not
    // the curve-discretization path, so they are never shared.
    if p0.distance(&p1) <= 1e-5 {
        return None;
    }
    let pm = curve.point(0.5 * (edge.first() + edge.last()));
    let (k0, k1) = (shared_key_point(p0), shared_key_point(p1));
    let km = shared_key_point(pm);
    Some(if k0 <= k1 { (k0, k1, km) } else { (k1, k0, km) })
}

/// Discretize every boundary edge of the shell ONCE, at the maximum density any
/// adjacent face requires, so both faces of a shared edge consume the *same*
/// boundary polyline (stored canonically: from the smaller endpoint key).
///
/// This is the "edges first, faces second" meshing order production kernels
/// use: per-face tessellations then agree exactly along shared boundaries, so
/// the combined mesh is crack-free by construction instead of relying on
/// post-hoc lens stitching (which fans, and whose fans the cylinder refinement
/// pass used to amplify into non-manifold soup on tangent seams).
pub fn shared_edge_polylines(
    faces: &[Face],
    chord_err: f64,
    angle_err: f64,
) -> HashMap<SharedEdgeKey, Vec<SharedEdgeSample>> {
    // The native cylinder primitive is exactly two planar caps plus three
    // cylindrical 120° wall patches. Its binary arc subdivision sits on a
    // numerical cliff at common round budgets (0.13 rad is intended as 48
    // facets, but is 0.7% below 2π/48 and therefore doubles to 96). A 1% budget
    // snap is safe for this closed-form primitive and avoids that redundant
    // recursion. Mixed/trimmed solids retain the strict caller budget.
    let native_cylinder = faces.len() == 5
        && faces
            .iter()
            .filter(|face| matches!(face.surface(), Some(GeomSurface::Cylinder(_))))
            .count()
            == 3
        && faces
            .iter()
            .filter(|face| matches!(face.surface(), Some(GeomSurface::Plane(_))))
            .count()
            == 2;
    let mut map: HashMap<SharedEdgeKey, Vec<SharedEdgeSample>> = HashMap::new();
    for face in faces {
        let surface = match face.surface() {
            Some(s) => s,
            None => continue,
        };
        for wire in &face.wires() {
            for edge in wire.edges().iter() {
                let (key, curve) = match (shared_edge_key(edge), edge.curve()) {
                    (Some(k), Some(c)) => (k, c),
                    _ => continue,
                };
                // Miter seams (elliptical tangent edges between two blend faces)
                // keep chordal-only density: they are interior tangent edges, so
                // extra angular density buys no visual smoothness — it only
                // tapers the stub-corner wedge thinner than the seam-support
                // offsets reach, resurrecting the coincident double-membrane.
                let edge_angle = if matches!(curve, GeomCurve::Ellipse(_)) {
                    std::f64::consts::PI
                } else if native_cylinder && matches!(curve, GeomCurve::Circle(_)) {
                    angle_err * 1.01
                } else {
                    angle_err
                };
                let params = discretize_edge_curve_budget(
                    curve,
                    edge.first(),
                    edge.last(),
                    chord_err,
                    edge_angle,
                );
                let params = refine_surface_edge_params(surface, curve, &params, chord_err);
                let span = edge.last() - edge.first();
                let mut pts: Vec<(Pnt, f64)> = params
                    .iter()
                    .map(|&t| (curve.point(t), (t - edge.first()) / span))
                    .collect();
                if pts.len() < 2 {
                    continue;
                }
                // Snap the polyline's ends to the edge's topological vertex
                // points — the one representation every adjacent edge and face
                // agrees on bit-exactly. A curve evaluation can land ~1e-7 off
                // the vertex (f32-sourced sketch geometry): inside B-Rep
                // tolerance but outside combine()'s 1e-9 weld, which would
                // leave hairline index-level cracks at every shared corner.
                let n = pts.len();
                let (vs, ve) = (edge.start().point(), edge.end().point());
                if pts[0].0.distance(&vs) <= pts[0].0.distance(&ve) {
                    pts[0].0 = vs;
                    pts[n - 1].0 = ve;
                } else {
                    pts[0].0 = ve;
                    pts[n - 1].0 = vs;
                }
                let k_first = shared_key_point(pts[0].0);
                let k_last = shared_key_point(pts.last().unwrap().0);
                let reversed = k_first > k_last;
                if reversed {
                    pts.reverse();
                }
                let pts: Vec<SharedEdgeSample> = pts
                    .into_iter()
                    .map(|(point, natural_fraction)| SharedEdgeSample {
                        point,
                        canonical_fraction: if reversed {
                            1.0 - natural_fraction
                        } else {
                            natural_fraction
                        },
                    })
                    .collect();
                match map.entry(key) {
                    std::collections::hash_map::Entry::Occupied(mut e) => {
                        if pts.len() > e.get().len() {
                            e.insert(pts);
                        }
                    }
                    std::collections::hash_map::Entry::Vacant(v) => {
                        v.insert(pts);
                    }
                }
            }
        }
    }
    map
}

/// Tessellates a single Face into a TriangleMesh locally.
///
/// `face_index` is recorded as the source-face id of every emitted triangle, so
/// the combined mesh can map triangles back to their originating face (picking).
pub fn tessellate_face_local(face: &Face, chord_err: f64, face_index: u32) -> TriangleMesh {
    tessellate_face_budget(face, chord_err, std::f64::consts::PI, face_index, None)
}

/// [`tessellate_face_local`] with an angular budget and (optionally) the
/// solid-wide shared boundary polylines from [`shared_edge_polylines`]. When a
/// boundary edge has a shared polyline, its samples are consumed verbatim so
/// both adjacent faces agree exactly along the edge; boundary constraints are
/// then also pinned (never split by the cylinder refinement), keeping that
/// agreement intact.
pub fn tessellate_face_budget(
    face: &Face,
    chord_err: f64,
    angle_err: f64,
    face_index: u32,
    shared: Option<&HashMap<SharedEdgeKey, Vec<SharedEdgeSample>>>,
) -> TriangleMesh {
    tessellate_face_budget_configured(face, chord_err, angle_err, face_index, shared, false)
}

pub(crate) fn tessellate_face_budget_configured(
    face: &Face,
    chord_err: f64,
    angle_err: f64,
    face_index: u32,
    shared: Option<&HashMap<SharedEdgeKey, Vec<SharedEdgeSample>>>,
    bound_cylinder_diagonals: bool,
) -> TriangleMesh {
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
        // On helix-railed ruled surfaces the analytic projection has no height
        // signal when both rails are lead-0 (a thread runout ring), so an
        // un-hinted first point wraps into [0, 2π) while the hint chain later
        // closes the loop at its true (possibly negative) angle — tearing the
        // uv polygon open at the seam. Helix edge params on these faces ARE
        // the surface angle, so seed the chain from the first helix edge's own
        // start parameter.
        let mut prev_hint = None;
        if matches!(surface, GeomSurface::Ruled(_)) {
            for e in edges.iter() {
                if matches!(e.curve(), Some(GeomCurve::Helix(_))) {
                    let t = if e.orientation().is_forward() {
                        e.first()
                    } else {
                        e.last()
                    };
                    prev_hint = Some((t, 0.5));
                    break;
                }
            }
        }

        for (edge_idx, edge) in edges.iter().enumerate() {
            let pcurve = wire
                .pcurve(edge_idx)
                .expect("strict tessellation requires a pcurve on every surface coedge");
            let p_start = edge.start().point();
            let p_end = edge.end().point();
            let is_collapsed = p_start.distance(&p_end) <= 1e-5;

            if is_collapsed {
                let reversed = !edge.orientation().is_forward();
                let n_points = 5;
                for i in 0..n_points {
                    let directed_fraction = i as f64 / (n_points - 1) as f64;
                    let natural_fraction = if reversed {
                        1.0 - directed_fraction
                    } else {
                        directed_fraction
                    };
                    let uv = boundary_uv(surface, pcurve, natural_fraction, prev_hint);
                    loop_samples.push(BoundarySample {
                        uv,
                        point: p_start,
                        on_seam: pcurve.crosses_u_seam() || pcurve.crosses_v_seam(),
                    });
                    prev_hint = Some((uv.x(), uv.y()));
                }
                if loop_samples.len() > 1 {
                    loop_samples.pop();
                }
                continue;
            }

            let curve = match edge.curve() {
                Some(curve) => curve,
                None => {
                    let reversed = !edge.orientation().is_forward();
                    let samples = if reversed {
                        [(p_end, 1.0), (p_start, 0.0)]
                    } else {
                        [(p_start, 0.0), (p_end, 1.0)]
                    };
                    for (point, fraction) in samples {
                        let uv = boundary_uv(surface, pcurve, fraction, prev_hint);
                        loop_samples.push(BoundarySample {
                            uv,
                            point,
                            on_seam: pcurve.crosses_u_seam() || pcurve.crosses_v_seam(),
                        });
                        prev_hint = Some((uv.x(), uv.y()));
                    }
                    if loop_samples.len() > 1 {
                        loop_samples.pop();
                    }
                    continue;
                }
            };

            let on_seam = matches!(curve, GeomCurve::Ellipse(_));
            let is_reversed = !edge.orientation().is_forward();

            // Shared solid-wide polyline for this edge when available (both
            // adjacent faces then sample identical boundary points); otherwise
            // discretize locally as before.
            let shared_pts = shared.and_then(|m| shared_edge_key(edge).and_then(|k| m.get(&k)));
            let samples_directed: Vec<(Pnt, f64)> = if let Some(sp) = shared_pts {
                let t_start = if is_reversed {
                    edge.last()
                } else {
                    edge.first()
                };
                let start_pt = curve.point(t_start);
                let natural_start_is_canonical = sp[0].point.distance(&edge.start().point())
                    <= sp[sp.len() - 1].point.distance(&edge.start().point());
                let directed: Vec<_> = if sp[0].point.distance(&start_pt)
                    <= sp[sp.len() - 1].point.distance(&start_pt)
                {
                    sp.to_vec()
                } else {
                    sp.iter().rev().copied().collect()
                };
                directed
                    .into_iter()
                    .map(|sample| {
                        let fraction = if natural_start_is_canonical {
                            sample.canonical_fraction
                        } else {
                            1.0 - sample.canonical_fraction
                        };
                        (sample.point, fraction)
                    })
                    .collect()
            } else {
                // Same miter-seam exemption as the shared pass: elliptical
                // tangent seams keep chordal-only density.
                let edge_angle = if on_seam {
                    std::f64::consts::PI
                } else {
                    angle_err
                };
                let params = discretize_edge_curve_budget(
                    curve,
                    edge.first(),
                    edge.last(),
                    chord_err,
                    edge_angle,
                );
                let params = refine_surface_edge_params(surface, curve, &params, chord_err);
                let params_directed: Vec<f64> = if is_reversed {
                    params.into_iter().rev().collect()
                } else {
                    params
                };
                let span = edge.last() - edge.first();
                let mut samples: Vec<(Pnt, f64)> = params_directed
                    .iter()
                    .map(|&t| (curve.point(t), (t - edge.first()) / span))
                    .collect();
                // Same vertex snap as the shared pass: adjacent faces meet
                // bit-exactly at topological vertices even without a shared
                // polyline (see shared_edge_polylines).
                if samples.len() >= 2 {
                    let n = samples.len();
                    let (vs, ve) = (edge.start().point(), edge.end().point());
                    if samples[0].0.distance(&vs) <= samples[0].0.distance(&ve) {
                        samples[0].0 = vs;
                        samples[n - 1].0 = ve;
                    } else {
                        samples[0].0 = ve;
                        samples[n - 1].0 = vs;
                    }
                }
                samples
            };

            for &(p3d, natural_fraction) in &samples_directed {
                let p2d = boundary_uv(surface, pcurve, natural_fraction, prev_hint);
                loop_samples.push(BoundarySample {
                    uv: p2d,
                    point: p3d,
                    on_seam,
                });
                prev_hint = Some((p2d.x(), p2d.y()));
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

    // Resolve the trim loops before sampling so a plain cylinder wall can use
    // one axial support row instead of the generic eight-row trimmed-wall grid.
    let outer_pts: Vec<Pnt2d> = loops_2d[0].iter().map(|&idx| all_points_2d[idx]).collect();
    let inner_pts_list: Vec<Vec<Pnt2d>> = loops_2d
        .iter()
        .skip(1)
        .map(|l| l.iter().map(|&idx| all_points_2d[idx]).collect())
        .collect();
    let compact_cylinder = is_untrimmed_cylinder_patch(surface, &outer_pts, &inner_pts_list);
    if compact_cylinder {
        if let Some(tris) = untrimmed_cylinder_strip_tris(
            &mut all_points_2d,
            &mut all_points_3d,
            surface,
            &loops_2d[0],
            chord_err,
            bound_cylinder_diagonals,
        ) {
            let wants_ccw = face.orientation() != Orientation::Reversed;
            return mesh_from_uv_tris(
                &all_points_2d,
                surface,
                wants_ccw,
                all_points_3d,
                tris,
                face_index,
            );
        }
    }
    let interior_candidates =
        sample_interior_points_budget(surface, u_min, u_max, v_min, v_max, chord_err, angle_err);

    // Filter interior points inside loops.
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
            // Shrink the offset until the support point lands inside the region:
            // near the seam's stub-vertex corner the (u, v) wedge tapers so thin
            // that the 0.04 point falls outside and used to be skipped — leaving
            // the apex triangle unsupported, so both tangent faces fanned the
            // identical flat triangle there (a residual double-membrane).
            let Some(p2d) = [0.04, 0.02, 0.01, 0.005, 0.002]
                .iter()
                .map(|f| Pnt2d::new(s.x() + f * (cu - s.x()), s.y() + f * (cv - s.y())))
                .find(|p| {
                    is_point_in_polygon(*p, &outer_pts)
                        && !inner_pts_list.iter().any(|h| is_point_in_polygon(*p, h))
                })
            else {
                continue;
            };
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
        let cocircular = mean_r > 1e-9 && radii.iter().all(|r| (r - mean_r).abs() <= 0.06 * mean_r);
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

    let wants_ccw = face.orientation() != Orientation::Reversed;
    let tris = refine_cylinder_tris(
        surface,
        &mut all_points_2d,
        &mut all_points_3d,
        &mut point_map,
        &constraints,
        &outer_pts,
        &inner_pts_list,
        wants_ccw,
        chord_err,
        shared.is_none(),
    );

    mesh_from_uv_tris(
        &all_points_2d,
        surface,
        wants_ccw,
        all_points_3d,
        tris,
        face_index,
    )
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
        let plen =
            |e: &(u32, u32)| mesh.vertices[e.0 as usize].distance(&mesh.vertices[e.1 as usize]);
        boundary.sort_by(|x, y| {
            plen(y)
                .partial_cmp(&plen(x))
                .unwrap_or(std::cmp::Ordering::Equal)
        });

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
            // A real tessellation lens is THIN: the arc a face chorded deviates
            // from that chord by at most a few chordal tolerances. A large
            // deviation means this "chord" spans a genuine boundary concavity
            // (e.g. a circular bite eating into the face) — filling it would
            // membrane over the void (seen as fillet-validation ghost material).
            let pa = mesh.vertices[a as usize];
            let pb = mesh.vertices[b as usize];
            let ab = [pb.x() - pa.x(), pb.y() - pa.y(), pb.z() - pa.z()];
            let ab_len2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
            let mut max_dev = 0.0_f64;
            if ab_len2 > 1e-18 {
                for &v in &path[1..path.len() - 1] {
                    let p = mesh.vertices[v as usize];
                    let ap = [p.x() - pa.x(), p.y() - pa.y(), p.z() - pa.z()];
                    let t =
                        ((ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / ab_len2).clamp(0.0, 1.0);
                    let d = [ap[0] - t * ab[0], ap[1] - t * ab[1], ap[2] - t * ab[2]];
                    max_dev = max_dev.max((d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt());
                }
            }
            if max_dev > (0.02 * chord).max(0.2) {
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
                let nexts: Vec<u32> = adj[&cur].iter().copied().filter(|&w| w != prev).collect();
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
                if removed.contains(&ti) || mesh.face_ids.get(ti).copied().unwrap_or(0) == arc_fid {
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

        // Phase C - straight closed-loop contact chains. Multi-edge fillets can
        // leave a cylinder contact sampled as many collinear boundary sub-edges
        // while the adjacent face still owns one long chord. In all-four-edge
        // top fillets those chains are part of a closed boundary graph, so the
        // open-chain phase above never sees degree-1 starts. Re-fan the chord
        // owner through each maximal straight chain.
        let boundary_set: HashSet<(u32, u32)> = boundary.iter().copied().collect();
        let mut stitched_edges: HashSet<(u32, u32)> = HashSet::new();
        for &(a0, b0) in &boundary {
            let edge_key = key(a0, b0);
            if stitched_edges.contains(&edge_key) || touched.contains(&a0) || touched.contains(&b0)
            {
                continue;
            }
            let Some(&owner_tri) = edge_tris.get(&edge_key).and_then(|tris| tris.first()) else {
                continue;
            };
            let owner_fid = mesh.face_ids.get(owner_tri).copied().unwrap_or(0);
            let mut chain = vec![a0, b0];
            extend_collinear_boundary_chain(
                mesh,
                &edge_tris,
                &boundary_set,
                &adj,
                &mut chain,
                owner_fid,
                false,
            );
            extend_collinear_boundary_chain(
                mesh,
                &edge_tris,
                &boundary_set,
                &adj,
                &mut chain,
                owner_fid,
                true,
            );
            if chain.len() < 3 || chain.iter().any(|v| touched.contains(v)) {
                continue;
            }
            let (a, b) = (chain[0], *chain.last().unwrap());
            if a == b || !chain_is_straight(mesh, &chain) {
                continue;
            }
            let Some(chord_tris) = edge_tris.get(&key(a, b)) else {
                continue;
            };
            let Some(&ti) = chord_tris.iter().find(|&&ti| {
                !removed.contains(&ti) && mesh.face_ids.get(ti).copied().unwrap_or(0) != owner_fid
            }) else {
                continue;
            };
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
                stitched_edges.insert(key(w[0], w[1]));
            }
            removed.insert(ti);
            for &v in &chain {
                touched.insert(v);
            }
            progressed = true;
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

fn extend_collinear_boundary_chain(
    mesh: &TriangleMesh,
    edge_tris: &HashMap<(u32, u32), Vec<usize>>,
    boundary_set: &HashSet<(u32, u32)>,
    adj: &HashMap<u32, Vec<u32>>,
    chain: &mut Vec<u32>,
    owner_fid: u32,
    reverse: bool,
) {
    loop {
        let (prev, head) = if reverse {
            (chain[1], chain[0])
        } else {
            let n = chain.len();
            (chain[n - 2], chain[n - 1])
        };
        let Some(candidates) = adj.get(&head) else {
            break;
        };
        let mut next = None;
        for &candidate in candidates {
            if candidate == prev || chain.contains(&candidate) {
                continue;
            }
            let edge_key = if head <= candidate {
                (head, candidate)
            } else {
                (candidate, head)
            };
            if !boundary_set.contains(&edge_key) {
                continue;
            }
            let Some(&ti) = edge_tris.get(&edge_key).and_then(|tris| tris.first()) else {
                continue;
            };
            if mesh.face_ids.get(ti).copied().unwrap_or(0) != owner_fid {
                continue;
            }
            if boundary_turn_is_collinear(mesh, prev, head, candidate) {
                next = Some(candidate);
                break;
            }
        }
        let Some(next) = next else {
            break;
        };
        if reverse {
            chain.insert(0, next);
        } else {
            chain.push(next);
        }
    }
}

fn boundary_turn_is_collinear(mesh: &TriangleMesh, a: u32, b: u32, c: u32) -> bool {
    let pa = mesh.vertices[a as usize];
    let pb = mesh.vertices[b as usize];
    let pc = mesh.vertices[c as usize];
    let Some(ba) = (pa - pb).normalized() else {
        return false;
    };
    let Some(bc) = (pc - pb).normalized() else {
        return false;
    };
    GeomVec::from_dir(ba).dot(&GeomVec::from_dir(bc)).abs() > 0.999_999
}

fn chain_is_straight(mesh: &TriangleMesh, chain: &[u32]) -> bool {
    if chain.len() < 3 {
        return false;
    }
    let a = mesh.vertices[chain[0] as usize];
    let b = mesh.vertices[*chain.last().unwrap() as usize];
    let Some(dir) = (b - a).normalized() else {
        return false;
    };
    chain
        .iter()
        .copied()
        .all(|vi| point_line_distance_3d(mesh.vertices[vi as usize], a, dir) <= 1.0e-7)
}

fn point_line_distance_3d(p: Pnt, origin: Pnt, dir: openrcad_foundation::Dir) -> f64 {
    let v = p - origin;
    let along = GeomVec::from_dir(dir) * v.dot(&GeomVec::from_dir(dir));
    (v - along).magnitude()
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::{Ax3, Dir, Dir2d, Pnt, Pnt2d};
    use openrcad_geom::{CylindricalSurface, GeomSurface, Plane};
    use openrcad_geom2d::{GeomCurve2d, Line2d};
    use openrcad_topo::{Edge, Face, Orientation, PcurveData, SurfacePeriodicity, Wire};

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

    fn test_cylinder_surface(radius: f64) -> GeomSurface {
        GeomSurface::cylinder(CylindricalSurface::new(
            Ax3::new(Pnt::origin(), Dir::dz()),
            radius,
        ))
    }

    fn point_key(p: Pnt2d) -> (i64, i64) {
        ((p.x() * 1e8) as i64, (p.y() * 1e8) as i64)
    }

    fn refine_test_points(surface: &GeomSurface, points: Vec<Pnt2d>) -> Vec<Pnt2d> {
        let mut points_2d = points;
        let outer_pts = points_2d.clone();
        let mut points_3d: Vec<Pnt> = points_2d
            .iter()
            .map(|p| surface.point(p.x(), p.y()))
            .collect();
        let mut point_map: HashMap<(i64, i64), usize> = points_2d
            .iter()
            .enumerate()
            .map(|(i, &p)| (point_key(p), i))
            .collect();
        let inner_pts_list = Vec::new();
        let _tris = refine_cylinder_tris(
            surface,
            &mut points_2d,
            &mut points_3d,
            &mut point_map,
            &[],
            &outer_pts,
            &inner_pts_list,
            true,
            0.05,
            true,
        );
        points_2d
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
    fn ordered_bounds_clamp_swaps_reversed_projection_limits() {
        assert_eq!(clamp_to_ordered_bounds(2.12, 2.14, 2.09), 2.12);
        assert_eq!(clamp_to_ordered_bounds(2.00, 2.14, 2.09), 2.09);
        assert_eq!(clamp_to_ordered_bounds(2.20, 2.14, 2.09), 2.14);
    }

    #[test]
    fn boundary_sampling_uses_the_coedge_pcurve_exactly() {
        let surface = GeomSurface::plane(Plane::from_point_normal(Pnt::origin(), Dir::dz()));
        let pcurve = PcurveData::new(
            GeomCurve2d::line(Line2d::from_point_dir(Pnt2d::new(10.0, 20.0), Dir2d::dx())),
            0.0,
            2.0,
        );

        let uv = boundary_uv(&surface, &pcurve, 0.25, None);

        assert_eq!(uv, Pnt2d::new(10.5, 20.0));
    }

    #[test]
    fn boundary_sampling_preserves_an_unwrapped_periodic_range() {
        let surface = test_cylinder_surface(2.0);
        let pcurve = PcurveData::new(
            GeomCurve2d::line(Line2d::from_point_dir(Pnt2d::new(5.8, 3.0), Dir2d::dx())),
            0.0,
            1.0,
        )
        .with_periodicity(SurfacePeriodicity::u_periodic(core::f64::consts::TAU));

        let uv = boundary_uv(
            &surface,
            &pcurve,
            0.8,
            Some((6.4, 3.0)),
        );

        assert!((uv.x() - 6.6).abs() < 1e-12);
        assert_eq!(uv.y(), 3.0);
    }

    #[test]
    fn cylinder_refinement_splits_pure_hoop_chord() {
        let surface = test_cylinder_surface(10.0);
        let points = refine_test_points(
            &surface,
            vec![
                Pnt2d::new(0.0, 0.0),
                Pnt2d::new(1.0, 0.0),
                Pnt2d::new(0.0, 1.0),
            ],
        );

        assert!(
            points
                .iter()
                .any(|p| (p.x() - 0.5).abs() < 1e-9 && p.y().abs() < 1e-9),
            "expected midpoint support on the long pure-hoop edge"
        );
    }

    #[test]
    fn cylinder_refinement_splits_mixed_axial_hoop_chord() {
        let surface = test_cylinder_surface(10.0);
        let points = refine_test_points(
            &surface,
            vec![
                Pnt2d::new(0.0, 0.0),
                Pnt2d::new(0.5, 3.0),
                Pnt2d::new(0.0, 3.0),
            ],
        );

        assert!(
            points
                .iter()
                .any(|p| (p.x() - 0.25).abs() < 1e-9 && (p.y() - 1.5).abs() < 1e-9),
            "expected midpoint support on the long mixed cylinder edge"
        );
    }

    #[test]
    fn uv_triangle_mesh_preserves_source_face_id() {
        let surface = GeomSurface::plane(Plane::from_point_normal(Pnt::origin(), Dir::dz()));
        let points_2d = vec![
            Pnt2d::new(0.0, 0.0),
            Pnt2d::new(1.0, 0.0),
            Pnt2d::new(0.0, 1.0),
            Pnt2d::new(1.0, 1.0),
        ];
        let mesh = mesh_from_uv_tris(
            &points_2d,
            &surface,
            true,
            vec![
                Pnt::origin(),
                Pnt::new(1.0, 0.0, 0.0),
                Pnt::new(0.0, 1.0, 0.0),
                Pnt::new(1.0, 1.0, 0.0),
            ],
            vec![Tri { a: 0, b: 1, c: 2 }, Tri { a: 1, b: 3, c: 2 }],
            42,
        );

        assert_eq!(mesh.face_ids, vec![42, 42]);
    }

    #[test]
    fn post_stitch_cylinder_refinement_skips_exact_axial_edge() {
        let surface = test_cylinder_surface(10.0);
        let pts = [
            surface.point(0.0, 0.0),
            surface.point(0.0, 10.0),
            surface.point(0.2, 0.0),
            surface.point(-0.2, 10.0),
        ];
        let face = Face::with_wires(Some(surface), None, Vec::new(), Orientation::Forward);
        let mut mesh = TriangleMesh::from_buffers_with_faces(
            pts.to_vec(),
            vec![[0, 1, 2], [1, 0, 3]],
            vec![0, 0],
        );

        refine_cylinder_mesh_edges(&mut mesh, &[face], 0.05);

        assert_eq!(mesh.vertices.len(), pts.len());
        assert_eq!(mesh.triangles.len(), 2);
        assert!(mesh.face_ids.iter().all(|&fid| fid == 0));
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
