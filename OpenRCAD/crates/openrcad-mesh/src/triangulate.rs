use std::collections::HashMap;

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

/// Ray-casting point-in-polygon containment test.
pub fn is_point_in_polygon(p: Pnt2d, loop_pts: &[Pnt2d]) -> bool {
    let mut inside = false;
    let n = loop_pts.len();
    if n < 3 {
        return false;
    }
    let mut j = n - 1;
    for i in 0..n {
        let pi = loop_pts[i];
        let pj = loop_pts[j];

        let intersect = ((pi.y() > p.y()) != (pj.y() > p.y()))
            && (p.x() < (pj.x() - pi.x()) * (p.y() - pi.y()) / (pj.y() - pi.y()) + pi.x());
        if intersect {
            inside = !inside;
        }
        j = i;
    }
    inside
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

    let (u_divs, v_divs) = match surf {
        GeomSurface::Plane(_) => (0, 0),
        GeomSurface::Cylinder(cyl) => {
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
            let t_u = i as f64 / u_divs as f64;
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

    // Mapping to dedup vertices close to each other in 2D
    let mut point_map = HashMap::new();

    let face_wires = face.wires();
    for wire in &face_wires {
        let mut loop_pts_2d = Vec::new();
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
                    loop_pts_2d.push(Pnt2d::new(u, v));
                    prev_hint = Some((u, v));
                }

                if loop_pts_2d.len() > 1 {
                    loop_pts_2d.pop();
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
                    loop_pts_2d.push(Pnt2d::new(u, v));
                    prev_hint = Some((u, v));
                    continue;
                }
            };

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
                loop_pts_2d.push(p2d);
                prev_hint = Some((u, v));
            }

            // Remove duplicated adjacent endpoint when moving to next edge
            if loop_pts_2d.len() > 1 {
                loop_pts_2d.pop();
            }
        }

        // Close the loop if not closed
        if loop_pts_2d.len() > 2 {
            let first = loop_pts_2d[0];
            let last = *loop_pts_2d.last().unwrap();
            if first.distance(&last) > 1e-6 {
                // Keep it
            } else {
                loop_pts_2d.pop();
            }
        }

        // Add loop points to master coordinates
        let mut loop_indices = Vec::new();
        for &p2d in &loop_pts_2d {
            let key = ((p2d.x() * 1e8) as i64, (p2d.y() * 1e8) as i64);
            let idx = *point_map.entry(key).or_insert_with(|| {
                let id = all_points_2d.len();
                all_points_2d.push(p2d);
                // Evaluate 3D coordinate corresponding to this 2D coordinate on surface
                all_points_3d.push(surface.point(p2d.x(), p2d.y()));
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

    // 3. Delaunay Triangulation in 2D
    let tris = delaunay_triangulate(&all_points_2d);

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
                // Respect face orientation (reverse CCW if face is Reversed)
                if face.orientation() == Orientation::Reversed {
                    triangles.push([t.a as u32, t.c as u32, t.b as u32]);
                } else {
                    triangles.push([t.a as u32, t.b as u32, t.c as u32]);
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
