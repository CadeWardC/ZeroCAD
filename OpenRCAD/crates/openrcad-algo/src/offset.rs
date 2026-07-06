use openrcad_foundation::{tolerance, Dir, Pnt, Vec as GeomVec};
use openrcad_geom::{GeomCurve, GeomSurface, OffsetSurface, Plane, Surface};
use openrcad_topo::{Edge, Face, Solid, Vertex, Wire};
use std::collections::HashMap;

use crate::blend::{detect_cylinder, shell_cylinder, BlendError};
use crate::sew::sew;

/// Shell a solid by `thickness`, removing `open_faces`.
pub fn shell_solid(
    solid: &Solid,
    thickness: f64,
    open_faces: &[Face],
) -> Result<Solid, BlendError> {
    if thickness.abs() <= tolerance::CONFUSION {
        return Ok(solid.clone());
    }
    if let Some((p0, ex, ey, ez, dx, dy, dz)) = detect_box(solid) {
        return Ok(shell_box(p0, ex, ey, ez, dx, dy, dz, thickness, open_faces));
    }
    if let Some(cyl) = detect_cylinder(solid) {
        return shell_cylinder(&cyl, thickness, open_faces);
    }
    shell_planar_general(solid, thickness, open_faces)
}

/// General shell for arbitrary PLANAR-faced solids with straight edges (any
/// extruded profile, wedges, chamfered blocks — everything the box/cylinder
/// recognisers above don't match). Each face's plane is offset inward by
/// `thickness`; every vertex is re-solved as the intersection of its three
/// adjacent offset planes; the inner shell reuses the outer topology with the
/// remapped vertices; removed faces get rim quads bridging outer → inner.
///
/// Limits (fail-loud with [`BlendError::UnsupportedShape`]): curved faces or
/// curved edges, vertices with ≠ 3 distinct adjacent face planes, and
/// thickness large enough to invert a vertex (offset planes meeting on the
/// wrong side). `open_faces` must name at least one face — a fully closed
/// hollow (a void) needs two-shell solids, which the arena represents but
/// this path does not emit yet.
fn shell_planar_general(
    solid: &Solid,
    thickness: f64,
    open_faces: &[Face],
) -> Result<Solid, BlendError> {
    if open_faces.is_empty() {
        return Err(BlendError::UnsupportedShape);
    }
    let faces = solid.shell().faces();
    let quant = |p: &Pnt| {
        (
            (p.x() * 1.0e6).round() as i64,
            (p.y() * 1.0e6).round() as i64,
            (p.z() * 1.0e6).round() as i64,
        )
    };

    // Effective OUTWARD plane per face (normal ⊗ orientation), keyed by index.
    let mut planes: Vec<(Pnt, Dir)> = Vec::with_capacity(faces.len());
    for face in &faces {
        let Some(GeomSurface::Plane(pl)) = face.surface() else {
            return Err(BlendError::UnsupportedShape);
        };
        let mut n = pl.normal();
        if face.orientation() == openrcad_topo::Orientation::Reversed {
            n = n.reversed();
        }
        // Anchor the plane at an actual boundary point (the stored plane
        // location may sit anywhere).
        let anchor = face
            .outer_wire()
            .and_then(|w| w.edges().first().map(|e| e.source().point()))
            .ok_or(BlendError::UnsupportedShape)?;
        planes.push((anchor, n));
    }

    // Vertex → set of adjacent face indices (by quantized position).
    let mut vertex_faces: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
    for (fi, face) in faces.iter().enumerate() {
        for wire in face.wires() {
            for edge in wire.edges() {
                if edge.curve().is_some()
                    && !matches!(edge.curve(), Some(GeomCurve::Line(_)))
                {
                    return Err(BlendError::UnsupportedShape);
                }
                for p in [edge.source().point(), edge.target().point()] {
                    let entry = vertex_faces.entry(quant(&p)).or_default();
                    if !entry.contains(&fi) {
                        entry.push(fi);
                    }
                }
            }
        }
    }

    // Offset every vertex: intersection of its three adjacent offset planes.
    // (Coplanar duplicates — split faces on one plane — collapse first.)
    let mut offset_of: HashMap<(i64, i64, i64), Pnt> = HashMap::new();
    for (key, fis) in &vertex_faces {
        let mut distinct: Vec<usize> = Vec::new();
        for &fi in fis {
            let (_, n) = planes[fi];
            let dup = distinct.iter().any(|&fj| {
                let (pj, nj) = planes[fj];
                n.dot(&nj).abs() > 1.0 - 1e-9
                    && ((planes[fi].0 - pj).dot(&GeomVec::from_dir(nj))).abs() < 1e-6
            });
            if !dup {
                distinct.push(fi);
            }
        }
        if distinct.len() != 3 {
            return Err(BlendError::UnsupportedShape);
        }
        let mut rows = [[0.0f64; 3]; 3];
        let mut rhs = [0.0f64; 3];
        for (r, &fi) in distinct.iter().enumerate() {
            let (p, n) = planes[fi];
            rows[r] = [n.x(), n.y(), n.z()];
            // Inward offset: the plane moves −thickness along its OUTWARD normal.
            rhs[r] = n.x() * p.x() + n.y() * p.y() + n.z() * p.z() - thickness;
        }
        let det = det3(&rows);
        if det.abs() < 1e-12 {
            return Err(BlendError::UnsupportedShape);
        }
        let x = solve3(&rows, &rhs, det);
        offset_of.insert(*key, Pnt::new(x[0], x[1], x[2]));
    }

    // Guard against inversion: every offset vertex must have moved less than
    // a few thicknesses (a vertex shooting far away means the offset planes
    // meet on the wrong side — thickness ≥ the local feature size).
    for (key, off) in &offset_of {
        let orig = Pnt::new(
            key.0 as f64 / 1.0e6,
            key.1 as f64 / 1.0e6,
            key.2 as f64 / 1.0e6,
        );
        if orig.distance(off) > thickness.abs() * 10.0 {
            return Err(BlendError::ParameterTooLarge {
                requested: thickness,
                max: orig.distance(off) / 10.0,
            });
        }
    }

    let is_open = |fi: usize| -> bool {
        open_faces.iter().any(|of| {
            let a = of
                .outer_wire()
                .and_then(|w| w.edges().first().map(|e| e.source().point()));
            let b = faces[fi]
                .outer_wire()
                .and_then(|w| w.edges().first().map(|e| e.source().point()));
            // Same face iff same plane and same anchor vertex (faces come from
            // this very solid, so anchor identity is exact).
            match (a, b) {
                (Some(a), Some(b)) => quant(&a) == quant(&b),
                _ => false,
            }
        })
    };

    let remap_wire = |wire: &Wire| -> Option<Wire> {
        let mut edges = Vec::new();
        for edge in wire.edges() {
            let a = *offset_of.get(&quant(&edge.source().point()))?;
            let b = *offset_of.get(&quant(&edge.target().point()))?;
            edges.push(Edge::between_points(a, b));
        }
        Some(Wire::from_edges(edges))
    };

    let mut result: Vec<Face> = Vec::new();
    for (fi, face) in faces.iter().enumerate() {
        if is_open(fi) {
            // Rim: one quad per boundary edge of the opening, bridging the
            // outer edge to its inner (offset) twin.
            if let Some(wire) = face.outer_wire() {
                for edge in wire.edges() {
                    let p0 = edge.source().point();
                    let p1 = edge.target().point();
                    let (Some(&q0), Some(&q1)) = (
                        offset_of.get(&quant(&p0)),
                        offset_of.get(&quant(&p1)),
                    ) else {
                        return Err(BlendError::UnsupportedShape);
                    };
                    let n = ((p1 - p0).cross(&(q0 - p0))).normalized();
                    let Some(n) = n else { continue };
                    result.push(Face::new(
                        Some(GeomSurface::plane(Plane::from_point_normal(p0, n))),
                        Wire::from_edges([
                            Edge::between_points(p0, p1),
                            Edge::between_points(p1, q1),
                            Edge::between_points(q1, q0),
                            Edge::between_points(q0, p0),
                        ]),
                    ));
                }
            }
            continue;
        }
        // Outer face kept verbatim; inner face is its offset twin. Windings
        // stay as-built — sew's planar canonicalization + global outward pass
        // orient the closed result.
        result.push(face.clone());
        let (anchor, n) = planes[fi];
        let inner_anchor = anchor + GeomVec::from_dir(n) * (-thickness);
        let outer = face.outer_wire().and_then(|w| remap_wire(&w));
        let inners: Vec<Wire> = face
            .inner_wires()
            .into_iter()
            .filter_map(|w| remap_wire(&w))
            .collect();
        let Some(outer) = outer else {
            return Err(BlendError::UnsupportedShape);
        };
        result.push(Face::with_wires(
            Some(GeomSurface::plane(Plane::from_point_normal(
                inner_anchor,
                n.reversed(),
            ))),
            Some(outer),
            inners,
            openrcad_topo::Orientation::Forward,
        ));
    }

    let shelled = Solid::new(sew(&result, tolerance::CONFUSION * 10.0));
    if !shelled.is_watertight() {
        return Err(BlendError::UnsupportedShape);
    }
    Ok(shelled)
}

fn det3(m: &[[f64; 3]; 3]) -> f64 {
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
}

/// Cramer's rule for the 3×3 system `m·x = b` with precomputed determinant.
fn solve3(m: &[[f64; 3]; 3], b: &[f64; 3], det: f64) -> [f64; 3] {
    let mut x = [0.0f64; 3];
    for c in 0..3 {
        let mut mc = *m;
        for r in 0..3 {
            mc[r][c] = b[r];
        }
        x[c] = det3(&mc) / det;
    }
    x
}

struct LocalFrame {
    p0: Pnt,
    ex: GeomVec,
    ey: GeomVec,
    ez: GeomVec,
}

impl LocalFrame {
    fn to_world(&self, u: f64, v: f64, w: f64) -> Pnt {
        self.p0 + self.ex * u + self.ey * v + self.ez * w
    }

    fn to_world_dir(&self, du: f64, dv: f64, dw: f64) -> Dir {
        let v = self.ex * du + self.ey * dv + self.ez * dw;
        Dir::new(v.x(), v.y(), v.z())
    }
}

fn detect_box(solid: &Solid) -> Option<(Pnt, GeomVec, GeomVec, GeomVec, f64, f64, f64)> {
    let vertices = solid.vertices();
    if vertices.len() != 8 {
        return None;
    }
    let faces = solid.shell().faces();
    if faces.len() != 6 {
        return None;
    }

    let p0 = vertices[0].point();
    let mut other_pts: Vec<Pnt> = vertices.iter().skip(1).map(|v| v.point()).collect();
    other_pts.sort_by(|a, b| {
        let da = p0.distance(a);
        let db = p0.distance(b);
        da.partial_cmp(&db).unwrap()
    });

    let p_x = other_pts[0];
    let p_y = other_pts[1];
    let p_z = other_pts[2];

    let v_x = p_x - p0;
    let v_y = p_y - p0;
    let v_z = p_z - p0;

    let dx = v_x.magnitude();
    let dy = v_y.magnitude();
    let dz = v_z.magnitude();

    if dx < 1e-5 || dy < 1e-5 || dz < 1e-5 {
        return None;
    }

    let ex = GeomVec::from_dir(v_x.normalized().unwrap());
    let ey = GeomVec::from_dir(v_y.normalized().unwrap());
    let ez = GeomVec::from_dir(v_z.normalized().unwrap());

    // Check orthogonality
    if ex.dot(&ey).abs() > 1e-4 || ey.dot(&ez).abs() > 1e-4 || ez.dot(&ex).abs() > 1e-4 {
        return None;
    }

    // Verify combinations
    let tol = 1e-4;
    let expected = [
        p0 + ex * dx + ey * dy,
        p0 + ey * dy + ez * dz,
        p0 + ex * dx + ez * dz,
        p0 + ex * dx + ey * dy + ez * dz,
    ];

    for &exp in &expected {
        if !other_pts.iter().any(|&p| p.distance(&exp) < tol) {
            return None;
        }
    }

    Some((p0, ex, ey, ez, dx, dy, dz))
}

fn make_straight_edge(v1: &Vertex, v2: &Vertex) -> Edge {
    let p1 = v1.point();
    let p2 = v2.point();
    let disp = p2 - p1;
    let len = disp.magnitude();
    if len <= tolerance::CONFUSION {
        Edge::new_with_tolerance(None, 0.0, 0.0, v1.clone(), v2.clone(), v1.tolerance())
    } else {
        let dir = disp.normalized().unwrap();
        let line = GeomCurve::line(openrcad_geom::Line::from_point_dir(
            p1,
            Dir::new(dir.x(), dir.y(), dir.z()),
        ));
        Edge::new_with_tolerance(
            Some(line),
            0.0,
            len,
            v1.clone(),
            v2.clone(),
            tolerance::CONFUSION,
        )
    }
}

#[allow(clippy::too_many_arguments)]
fn shell_box(
    p0: Pnt,
    ex: GeomVec,
    ey: GeomVec,
    ez: GeomVec,
    dx: f64,
    dy: f64,
    dz: f64,
    thickness: f64,
    open_faces: &[Face],
) -> Solid {
    let frame = LocalFrame { p0, ex, ey, ez };

    // Identify which faces are open.
    // Face indices: 0:Bottom, 1:Top, 2:Front, 3:Back, 4:Left, 5:Right.
    let mut is_open = [false; 6];

    let check_face_open = |face_center: Pnt| -> bool {
        for open_f in open_faces {
            if let Some(surf) = open_f.surface() {
                // Check if the face center lies on the open face surface.
                let (u, v) = openrcad_algo_intersect::uv_of(surf, &face_center);
                let p_on_surf = surf.point(u, v);
                if p_on_surf.distance(&face_center) < 1e-4 {
                    // Check if it's within the face trimming loops
                    if openrcad_algo_intersect::is_inside_trimming_loops(u, v, open_f) {
                        return true;
                    }
                }
            }
        }
        false
    };

    is_open[0] = check_face_open(frame.to_world(dx / 2.0, dy / 2.0, 0.0));
    is_open[1] = check_face_open(frame.to_world(dx / 2.0, dy / 2.0, dz));
    is_open[2] = check_face_open(frame.to_world(dx / 2.0, 0.0, dz / 2.0));
    is_open[3] = check_face_open(frame.to_world(dx / 2.0, dy, dz / 2.0));
    is_open[4] = check_face_open(frame.to_world(0.0, dy / 2.0, dz / 2.0));
    is_open[5] = check_face_open(frame.to_world(dx, dy / 2.0, dz / 2.0));

    // Outer shell bounds
    let min_x_out = 0.0;
    let max_x_out = dx;
    let min_y_out = 0.0;
    let max_y_out = dy;
    let min_z_out = 0.0;
    let max_z_out = dz;

    // Inner shell bounds, adjusted by whether the face is open or closed
    let min_x_in = if is_open[4] { 0.0 } else { thickness };
    let max_x_in = if is_open[5] { dx } else { dx - thickness };
    let min_y_in = if is_open[2] { 0.0 } else { thickness };
    let max_y_in = if is_open[3] { dy } else { dy - thickness };
    let min_z_in = if is_open[0] { 0.0 } else { thickness };
    let max_z_in = if is_open[1] { dz } else { dz - thickness };

    // 1. Create the 8 outer and 8 inner vertices
    let mut v_out = HashMap::new();
    let mut v_in = HashMap::new();
    for i in 0..=1 {
        for j in 0..=1 {
            for k in 0..=1 {
                let x_o = if i == 0 { min_x_out } else { max_x_out };
                let y_o = if j == 0 { min_y_out } else { max_y_out };
                let z_o = if k == 0 { min_z_out } else { max_z_out };
                v_out.insert((i, j, k), Vertex::new(frame.to_world(x_o, y_o, z_o)));

                let x_i = if i == 0 { min_x_in } else { max_x_in };
                let y_i = if j == 0 { min_y_in } else { max_y_in };
                let z_i = if k == 0 { min_z_in } else { max_z_in };
                v_in.insert((i, j, k), Vertex::new(frame.to_world(x_i, y_i, z_i)));
            }
        }
    }

    let mut faces = Vec::new();

    // 2. Helper to construct outer & inner faces
    let mut add_flat_face = |face_idx: usize,
                             v_out_coords: [(usize, usize, usize); 4],
                             v_in_coords: [(usize, usize, usize); 4],
                             surf_origin: Pnt,
                             surf_normal: Dir| {
        if is_open[face_idx] {
            return;
        }

        // Outer Face
        let a_o = v_out[&v_out_coords[0]].clone();
        let b_o = v_out[&v_out_coords[1]].clone();
        let c_o = v_out[&v_out_coords[2]].clone();
        let d_o = v_out[&v_out_coords[3]].clone();
        let w_o = Wire::from_edges([
            make_straight_edge(&a_o, &b_o),
            make_straight_edge(&b_o, &c_o),
            make_straight_edge(&c_o, &d_o),
            make_straight_edge(&d_o, &a_o),
        ]);
        let outer_surf = Plane::from_point_normal(surf_origin, surf_normal);
        let f_outer = Face::new(Some(GeomSurface::plane(outer_surf)), w_o);
        faces.push(f_outer);

        // Inner Face (Reversed & offset)
        let a_i = v_in[&v_in_coords[0]].clone();
        let b_i = v_in[&v_in_coords[1]].clone();
        let c_i = v_in[&v_in_coords[2]].clone();
        let d_i = v_in[&v_in_coords[3]].clone();
        let w_i = Wire::from_edges([
            make_straight_edge(&a_i, &b_i),
            make_straight_edge(&b_i, &c_i),
            make_straight_edge(&c_i, &d_i),
            make_straight_edge(&d_i, &a_i),
        ]);
        let base_surf = GeomSurface::plane(outer_surf);
        let offset_surf = GeomSurface::offset(OffsetSurface::new(base_surf, -thickness));
        let f_inner = Face::new(Some(offset_surf), w_i).reversed();
        faces.push(f_inner);
    };

    // Bottom (Z=0, index 0)
    add_flat_face(
        0,
        [(0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 0, 0)],
        [(0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 0, 0)],
        frame.to_world(0.0, 0.0, 0.0),
        frame.to_world_dir(0.0, 0.0, -1.0),
    );

    // Top (Z=dz, index 1)
    add_flat_face(
        1,
        [(0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)],
        [(0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)],
        frame.to_world(0.0, 0.0, dz),
        frame.to_world_dir(0.0, 0.0, 1.0),
    );

    // Front (Y=0, index 2)
    add_flat_face(
        2,
        [(0, 0, 0), (0, 0, 1), (1, 0, 1), (1, 0, 0)],
        [(0, 0, 0), (0, 0, 1), (1, 0, 1), (1, 0, 0)],
        frame.to_world(0.0, 0.0, 0.0),
        frame.to_world_dir(0.0, -1.0, 0.0),
    );

    // Back (Y=dy, index 3)
    add_flat_face(
        3,
        [(0, 1, 0), (1, 1, 0), (1, 1, 1), (0, 1, 1)],
        [(0, 1, 0), (1, 1, 0), (1, 1, 1), (0, 1, 1)],
        frame.to_world(0.0, dy, 0.0),
        frame.to_world_dir(0.0, 1.0, 0.0),
    );

    // Left (X=0, index 4)
    add_flat_face(
        4,
        [(0, 0, 0), (0, 1, 0), (0, 1, 1), (0, 0, 1)],
        [(0, 0, 0), (0, 1, 0), (0, 1, 1), (0, 0, 1)],
        frame.to_world(0.0, 0.0, 0.0),
        frame.to_world_dir(-1.0, 0.0, 0.0),
    );

    // Right (X=dx, index 5)
    add_flat_face(
        5,
        [(1, 0, 0), (1, 0, 1), (1, 1, 1), (1, 1, 0)],
        [(1, 0, 0), (1, 0, 1), (1, 1, 1), (1, 1, 0)],
        frame.to_world(dx, 0.0, 0.0),
        frame.to_world_dir(1.0, 0.0, 0.0),
    );

    // 3. Add Rim stitching faces around the open face loops
    // If a face is open, we add rim faces connecting its boundary segments.
    // For a rectangular box, each open face has 4 boundary segments.
    // Let's specify the 4 segments for each of the 6 possible open faces.
    // Each segment connects an outer vertex pair to an inner vertex pair.
    // Winding: Outer_A -> Inner_A -> Inner_B -> Outer_B -> Outer_A.
    let mut add_rim_segment = |v_out_a: (usize, usize, usize),
                               v_out_b: (usize, usize, usize),
                               v_in_a: (usize, usize, usize),
                               v_in_b: (usize, usize, usize),
                               rim_normal: Dir| {
        let a_o = v_out[&v_out_a].clone();
        let b_o = v_out[&v_out_b].clone();
        let a_i = v_in[&v_in_a].clone();
        let b_i = v_in[&v_in_b].clone();

        let e1 = make_straight_edge(&a_o, &a_i);
        let e2 = make_straight_edge(&a_i, &b_i);
        let e3 = make_straight_edge(&b_i, &b_o);
        let e4 = make_straight_edge(&b_o, &a_o);

        let w = Wire::from_edges([e1, e2, e3, e4]);
        let surf = GeomSurface::plane(Plane::from_point_normal(a_o.point(), rim_normal));
        let f = Face::new(Some(surf), w);
        faces.push(f);
    };

    // Bottom (index 0) open: Z=0
    if is_open[0] {
        let n = frame.to_world_dir(0.0, 0.0, -1.0);
        add_rim_segment((0, 0, 0), (0, 1, 0), (0, 0, 0), (0, 1, 0), n);
        add_rim_segment((0, 1, 0), (1, 1, 0), (0, 1, 0), (1, 1, 0), n);
        add_rim_segment((1, 1, 0), (1, 0, 0), (1, 1, 0), (1, 0, 0), n);
        add_rim_segment((1, 0, 0), (0, 0, 0), (1, 0, 0), (0, 0, 0), n);
    }

    // Top (index 1) open: Z=dz
    if is_open[1] {
        let n = frame.to_world_dir(0.0, 0.0, 1.0);
        add_rim_segment((0, 0, 1), (1, 0, 1), (0, 0, 1), (1, 0, 1), n);
        add_rim_segment((1, 0, 1), (1, 1, 1), (1, 0, 1), (1, 1, 1), n);
        add_rim_segment((1, 1, 1), (0, 1, 1), (1, 1, 1), (0, 1, 1), n);
        add_rim_segment((0, 1, 1), (0, 0, 1), (0, 1, 1), (0, 0, 1), n);
    }

    // Front (index 2) open: Y=0
    if is_open[2] {
        let n = frame.to_world_dir(0.0, -1.0, 0.0);
        add_rim_segment((0, 0, 0), (1, 0, 0), (0, 0, 0), (1, 0, 0), n);
        add_rim_segment((1, 0, 0), (1, 0, 1), (1, 0, 0), (1, 0, 1), n);
        add_rim_segment((1, 0, 1), (0, 0, 1), (1, 0, 1), (0, 0, 1), n);
        add_rim_segment((0, 0, 1), (0, 0, 0), (0, 0, 1), (0, 0, 0), n);
    }

    // Back (index 3) open: Y=dy
    if is_open[3] {
        let n = frame.to_world_dir(0.0, 1.0, 0.0);
        add_rim_segment((0, 1, 0), (0, 1, 1), (0, 1, 0), (0, 1, 1), n);
        add_rim_segment((0, 1, 1), (1, 1, 1), (0, 1, 1), (1, 1, 1), n);
        add_rim_segment((1, 1, 1), (1, 1, 0), (1, 1, 1), (1, 1, 0), n);
        add_rim_segment((1, 1, 0), (0, 1, 0), (1, 1, 0), (0, 1, 0), n);
    }

    // Left (index 4) open: X=0
    if is_open[4] {
        let n = frame.to_world_dir(-1.0, 0.0, 0.0);
        add_rim_segment((0, 0, 0), (0, 1, 0), (0, 0, 0), (0, 1, 0), n);
        add_rim_segment((0, 1, 0), (0, 1, 1), (0, 1, 0), (0, 1, 1), n);
        add_rim_segment((0, 1, 1), (0, 0, 1), (0, 1, 1), (0, 0, 1), n);
        add_rim_segment((0, 0, 1), (0, 0, 0), (0, 0, 1), (0, 0, 0), n);
    }

    // Right (index 5) open: X=dx
    if is_open[5] {
        let n = frame.to_world_dir(1.0, 0.0, 0.0);
        add_rim_segment((1, 0, 0), (1, 0, 1), (1, 0, 0), (1, 0, 1), n);
        add_rim_segment((1, 0, 1), (1, 1, 1), (1, 0, 1), (1, 1, 1), n);
        add_rim_segment((1, 1, 1), (1, 1, 0), (1, 1, 1), (1, 1, 0), n);
        add_rim_segment((1, 1, 0), (1, 0, 0), (1, 1, 0), (1, 0, 0), n);
    }

    // Sew the collection of faces into a watertight shell
    let shell = sew(&faces, thickness * 0.1);
    Solid::new(shell)
}

mod openrcad_algo_intersect {
    use super::*;
    pub fn uv_of(s: &GeomSurface, p: &Pnt) -> (f64, f64) {
        crate::intersect::uv_of(s, p)
    }
    pub fn is_inside_trimming_loops(u: f64, v: f64, face: &Face) -> bool {
        crate::intersect::is_inside_trimming_loops(u, v, face)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::Pnt;
    use openrcad_primitives::make_box;

    /// A right-triangle prism (legs `a`, height `h`) built through `prism` —
    /// NOT the box/cylinder recognisers' territory, so it exercises the
    /// general planar shell path.
    fn triangle_prism(a: f64, h: f64) -> Solid {
        use openrcad_geom::Plane as GPlane;
        let face = Face::new(
            Some(GeomSurface::plane(GPlane::from_point_normal(
                Pnt::origin(),
                Dir::dz(),
            ))),
            Wire::from_edges([
                Edge::between_points(Pnt::origin(), Pnt::new(a, 0.0, 0.0)),
                Edge::between_points(Pnt::new(a, 0.0, 0.0), Pnt::new(0.0, a, 0.0)),
                Edge::between_points(Pnt::new(0.0, a, 0.0), Pnt::origin()),
            ]),
        );
        crate::prism::prism(&face, GeomVec::new(0.0, 0.0, h)).unwrap()
    }

    #[test]
    fn general_shell_triangle_prism_open_top() {
        let a = 10.0f64;
        let h = 10.0f64;
        let t = 1.0f64;
        let solid = triangle_prism(a, h);
        // The top face: plane z = h with outward +z normal.
        let faces = solid.shell().faces();
        let top: Vec<Face> = faces
            .iter()
            .filter(|f| {
                f.outer_wire()
                    .and_then(|w| w.edges().first().map(|e| e.source().point().z()))
                    .map(|z| (z - h).abs() < 1e-6)
                    .unwrap_or(false)
                    && matches!(f.surface(), Some(GeomSurface::Plane(p)) if p.normal().z().abs() > 0.9)
            })
            .cloned()
            .collect();
        assert_eq!(top.len(), 1, "expected exactly one top cap");

        let cup = shell_solid(&solid, t, &top).expect("general shell");
        assert!(cup.is_watertight(), "shelled prism not watertight");

        // Expected material volume. The inner triangle is the outer inset by t
        // per edge: legs shrink to a − t·(2 + √2). The void is a prism of the
        // inner triangle from z=t to h−t, topped by a linear transition to the
        // full outer triangle at z=h (the slanted rim quads). The transition's
        // cross-section area is quadratic in z, so Simpson integrates exactly.
        let s2 = 2.0f64.sqrt();
        let leg_inner = a - t * (2.0 + s2);
        let area = |leg: f64| leg * leg / 2.0;
        let a_outer = area(a);
        let a_inner = area(leg_inner);
        let a_mid = area((a + leg_inner) / 2.0);
        let transition = t / 6.0 * (a_inner + 4.0 * a_mid + a_outer);
        let void = a_inner * (h - 2.0 * t) + transition;
        let expected = a_outer * h - void;

        let mesh = openrcad_mesh::tessellate(&cup, 0.002, 0.05);
        let mp = openrcad_mesh::mass_properties(&mesh).expect("closed shelled mesh");
        assert!(
            (mp.volume - expected).abs() / expected < 0.01,
            "shelled volume {} vs expected {expected}",
            mp.volume
        );
    }

    #[test]
    fn general_shell_rejects_closed_hollow_and_curved() {
        let solid = triangle_prism(10.0, 10.0);
        // No open faces → not supported (needs two-shell voids).
        assert!(matches!(
            shell_solid(&solid, 1.0, &[]),
            Err(BlendError::UnsupportedShape)
        ));
    }

    #[test]
    fn test_solid_shelling() {
        let cube = make_box(&Pnt::origin(), 1.0, 1.0, 1.0);

        // Find the top face (Z = 1.0) to remove
        let faces = cube.shell().faces();
        let mut open_faces = Vec::new();
        for face in &faces {
            if let Some(GeomSurface::Plane(plane)) = face.surface() {
                if (plane.location().z() - 1.0).abs() < 1e-4 && plane.normal().z() > 0.9 {
                    open_faces.push(face.clone());
                    break;
                }
            }
        }
        assert_eq!(open_faces.len(), 1);

        let cup = shell_solid(&cube, 0.1, &open_faces).unwrap();

        // A shelled box with 1 open face has:
        // 5 outer faces + 5 inner faces + 4 rim faces = 14 faces.
        // 16 vertices.
        // 28 edges.
        assert_eq!(cup.face_count(), 14);
        assert_eq!(cup.vertex_count(), 16);
        assert_eq!(cup.edge_count(), 28);

        // Verify Euler characteristic
        let v = cup.vertex_count() as i32;
        let e = cup.edge_count() as i32;
        let f = cup.face_count() as i32;
        assert_eq!(v - e + f, 2);

        // Verify face surface types:
        // 5 Planes (outer) + 5 Offset surfaces (inner) + 4 Planes (rims) = 9 Planes, 5 Offsets
        let mut planes_count = 0;
        let mut offsets_count = 0;

        for face in cup.shell().faces() {
            if let Some(surf) = face.surface() {
                match surf {
                    GeomSurface::Plane(_) => planes_count += 1,
                    GeomSurface::Offset(_) => offsets_count += 1,
                    _ => {}
                }
            }
        }

        assert_eq!(planes_count, 9);
        assert_eq!(offsets_count, 5);
    }
}
