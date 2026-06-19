use openrcad_foundation::{tolerance, Dir, Pnt, Vec as GeomVec};
use openrcad_geom::{GeomCurve, GeomSurface, Plane, RuledSurface};
use openrcad_topo::{Edge, Face, Solid, Vertex, Wire};
use std::collections::HashMap;

use crate::blend::{chamfer_cylinder, detect_cylinder, BlendError};
use crate::sew::sew;

/// Chamfer every edge of `solid` by `distance`.
pub fn chamfer(solid: &Solid, distance: f64) -> Result<Solid, BlendError> {
    if distance <= tolerance::CONFUSION {
        return Ok(solid.clone());
    }
    if let Some((p0, ex, ey, ez, dx, dy, dz)) = detect_box(solid) {
        let max = dx.min(dy).min(dz) / 2.0;
        if distance >= max {
            return Err(BlendError::ParameterTooLarge {
                requested: distance,
                max,
            });
        }
        return Ok(chamfer_box(p0, ex, ey, ez, dx, dy, dz, distance));
    }
    if let Some(cyl) = detect_cylinder(solid) {
        return chamfer_cylinder(&cyl, distance);
    }
    Err(BlendError::UnsupportedShape)
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

fn make_ruled_face(
    v_start_1: &Vertex,
    v_end_1: &Vertex,
    v_start_2: &Vertex,
    v_end_2: &Vertex,
) -> Face {
    let p_start_1 = v_start_1.point();
    let p_end_1 = v_end_1.point();
    let p_start_2 = v_start_2.point();
    let p_end_2 = v_end_2.point();

    let disp1 = p_end_1 - p_start_1;
    let len = disp1.magnitude();
    let dir1 = disp1.normalized().unwrap();
    let c1 = GeomCurve::line(openrcad_geom::Line::from_point_dir(
        p_start_1,
        Dir::new(dir1.x(), dir1.y(), dir1.z()),
    ));

    let disp2 = p_end_2 - p_start_2;
    let dir2 = disp2.normalized().unwrap();
    let c2 = GeomCurve::line(openrcad_geom::Line::from_point_dir(
        p_start_2,
        Dir::new(dir2.x(), dir2.y(), dir2.z()),
    ));

    let e1 = Edge::new_with_tolerance(
        Some(c1.clone()),
        0.0,
        len,
        v_start_1.clone(),
        v_end_1.clone(),
        tolerance::CONFUSION,
    );
    let e2 = make_straight_edge(v_end_1, v_end_2);
    let e3 = Edge::new_with_tolerance(
        Some(c2.clone()),
        0.0,
        len,
        v_start_2.clone(),
        v_end_2.clone(),
        tolerance::CONFUSION,
    )
    .reversed();
    let e4 = make_straight_edge(v_start_2, v_start_1);

    let w = Wire::from_edges([e1, e2, e3, e4]);
    let surf = GeomSurface::ruled(RuledSurface::new(c1, c2));
    Face::new(Some(surf), w)
}

fn make_planar_corner_face(v1: &Vertex, v2: &Vertex, v3: &Vertex, c_corner: Pnt) -> Face {
    let p1 = v1.point();
    let p2 = v2.point();
    let p3 = v3.point();

    let normal_approx = (p2 - p1).cross(&(p3 - p1));
    let to_outside = c_corner - p1;
    let (va, vb, vc) = if normal_approx.dot(&to_outside) > 0.0 {
        (v1, v2, v3)
    } else {
        (v1, v3, v2)
    };

    let e1 = make_straight_edge(va, vb);
    let e2 = make_straight_edge(vb, vc);
    let e3 = make_straight_edge(vc, va);
    let w = Wire::from_edges([e1, e2, e3]);

    let n = (vb.point() - va.point())
        .cross(&(vc.point() - va.point()))
        .normalized()
        .unwrap();
    let surf = GeomSurface::plane(Plane::from_point_normal(
        va.point(),
        Dir::new(n.x(), n.y(), n.z()),
    ));

    Face::new(Some(surf), w)
}

#[allow(clippy::too_many_arguments)] // the box frame is naturally seven scalars/vectors
fn chamfer_box(
    p0: Pnt,
    ex: GeomVec,
    ey: GeomVec,
    ez: GeomVec,
    dx: f64,
    dy: f64,
    dz: f64,
    distance: f64,
) -> Solid {
    let frame = LocalFrame { p0, ex, ey, ez };

    // 1. Generate the 24 vertices
    let mut vertices = HashMap::new();
    for i in 0..=1 {
        for j in 0..=1 {
            for k in 0..=1 {
                let sx = if i == 1 { 1.0 } else { -1.0 };
                let sy = if j == 1 { 1.0 } else { -1.0 };
                let sz = if k == 1 { 1.0 } else { -1.0 };

                // V_z on flat face normal to Z
                let p_z = frame.to_world(
                    i as f64 * dx - sx * distance,
                    j as f64 * dy - sy * distance,
                    k as f64 * dz,
                );
                vertices.insert((i, j, k, 2), Vertex::new(p_z));

                // V_y on flat face normal to Y
                let p_y = frame.to_world(
                    i as f64 * dx - sx * distance,
                    j as f64 * dy,
                    k as f64 * dz - sz * distance,
                );
                vertices.insert((i, j, k, 1), Vertex::new(p_y));

                // V_x on flat face normal to X
                let p_x = frame.to_world(
                    i as f64 * dx,
                    j as f64 * dy - sy * distance,
                    k as f64 * dz - sz * distance,
                );
                vertices.insert((i, j, k, 0), Vertex::new(p_x));
            }
        }
    }

    let mut faces = Vec::new();

    // 2. Add the 6 flat faces
    // Bottom (Z=0)
    let f_bottom = {
        let a = vertices[&(0, 0, 0, 2)].clone();
        let b = vertices[&(0, 1, 0, 2)].clone();
        let c = vertices[&(1, 1, 0, 2)].clone();
        let d = vertices[&(1, 0, 0, 2)].clone();
        let w = Wire::from_edges([
            make_straight_edge(&a, &b),
            make_straight_edge(&b, &c),
            make_straight_edge(&c, &d),
            make_straight_edge(&d, &a),
        ]);
        let surf = GeomSurface::plane(Plane::from_point_normal(
            frame.to_world(0.0, 0.0, 0.0),
            frame.to_world_dir(0.0, 0.0, -1.0),
        ));
        Face::new(Some(surf), w)
    };
    faces.push(f_bottom);

    // Top (Z=dz)
    let f_top = {
        let a = vertices[&(0, 0, 1, 2)].clone();
        let b = vertices[&(1, 0, 1, 2)].clone();
        let c = vertices[&(1, 1, 1, 2)].clone();
        let d = vertices[&(0, 1, 1, 2)].clone();
        let w = Wire::from_edges([
            make_straight_edge(&a, &b),
            make_straight_edge(&b, &c),
            make_straight_edge(&c, &d),
            make_straight_edge(&d, &a),
        ]);
        let surf = GeomSurface::plane(Plane::from_point_normal(
            frame.to_world(0.0, 0.0, dz),
            frame.to_world_dir(0.0, 0.0, 1.0),
        ));
        Face::new(Some(surf), w)
    };
    faces.push(f_top);

    // Front (Y=0)
    let f_front = {
        let a = vertices[&(0, 0, 0, 1)].clone();
        let b = vertices[&(1, 0, 0, 1)].clone();
        let c = vertices[&(1, 0, 1, 1)].clone();
        let d = vertices[&(0, 0, 1, 1)].clone();
        let w = Wire::from_edges([
            make_straight_edge(&a, &b),
            make_straight_edge(&b, &c),
            make_straight_edge(&c, &d),
            make_straight_edge(&d, &a),
        ]);
        let surf = GeomSurface::plane(Plane::from_point_normal(
            frame.to_world(0.0, 0.0, 0.0),
            frame.to_world_dir(0.0, -1.0, 0.0),
        ));
        Face::new(Some(surf), w)
    };
    faces.push(f_front);

    // Back (Y=dy)
    let f_back = {
        let a = vertices[&(0, 1, 0, 1)].clone();
        let b = vertices[&(0, 1, 1, 1)].clone();
        let c = vertices[&(1, 1, 1, 1)].clone();
        let d = vertices[&(1, 1, 0, 1)].clone();
        let w = Wire::from_edges([
            make_straight_edge(&a, &b),
            make_straight_edge(&b, &c),
            make_straight_edge(&c, &d),
            make_straight_edge(&d, &a),
        ]);
        let surf = GeomSurface::plane(Plane::from_point_normal(
            frame.to_world(0.0, dy, 0.0),
            frame.to_world_dir(0.0, 1.0, 0.0),
        ));
        Face::new(Some(surf), w)
    };
    faces.push(f_back);

    // Left (X=0)
    let f_left = {
        let a = vertices[&(0, 0, 0, 0)].clone();
        let b = vertices[&(0, 1, 0, 0)].clone();
        let c = vertices[&(0, 1, 1, 0)].clone();
        let d = vertices[&(0, 0, 1, 0)].clone();
        let w = Wire::from_edges([
            make_straight_edge(&a, &b),
            make_straight_edge(&b, &c),
            make_straight_edge(&c, &d),
            make_straight_edge(&d, &a),
        ]);
        let surf = GeomSurface::plane(Plane::from_point_normal(
            frame.to_world(0.0, 0.0, 0.0),
            frame.to_world_dir(-1.0, 0.0, 0.0),
        ));
        Face::new(Some(surf), w)
    };
    faces.push(f_left);

    // Right (X=dx)
    let f_right = {
        let a = vertices[&(1, 0, 0, 0)].clone();
        let b = vertices[&(1, 0, 1, 0)].clone();
        let c = vertices[&(1, 1, 1, 0)].clone();
        let d = vertices[&(1, 1, 0, 0)].clone();
        let w = Wire::from_edges([
            make_straight_edge(&a, &b),
            make_straight_edge(&b, &c),
            make_straight_edge(&c, &d),
            make_straight_edge(&d, &a),
        ]);
        let surf = GeomSurface::plane(Plane::from_point_normal(
            frame.to_world(dx, 0.0, 0.0),
            frame.to_world_dir(1.0, 0.0, 0.0),
        ));
        Face::new(Some(surf), w)
    };
    faces.push(f_right);

    // 3. Add the 12 ruled chamfers
    // X-axis chamfers
    faces.push(make_ruled_face(
        &vertices[&(0, 0, 0, 1)],
        &vertices[&(1, 0, 0, 1)],
        &vertices[&(0, 0, 0, 2)],
        &vertices[&(1, 0, 0, 2)],
    ));
    faces.push(make_ruled_face(
        &vertices[&(0, 1, 0, 1)],
        &vertices[&(1, 1, 0, 1)],
        &vertices[&(0, 1, 0, 2)],
        &vertices[&(1, 1, 0, 2)],
    ));
    faces.push(make_ruled_face(
        &vertices[&(0, 0, 1, 1)],
        &vertices[&(1, 0, 1, 1)],
        &vertices[&(0, 0, 1, 2)],
        &vertices[&(1, 0, 1, 2)],
    ));
    faces.push(make_ruled_face(
        &vertices[&(0, 1, 1, 1)],
        &vertices[&(1, 1, 1, 1)],
        &vertices[&(0, 1, 1, 2)],
        &vertices[&(1, 1, 1, 2)],
    ));

    // Y-axis chamfers
    faces.push(make_ruled_face(
        &vertices[&(0, 0, 0, 0)],
        &vertices[&(0, 1, 0, 0)],
        &vertices[&(0, 0, 0, 2)],
        &vertices[&(0, 1, 0, 2)],
    ));
    faces.push(make_ruled_face(
        &vertices[&(1, 0, 0, 0)],
        &vertices[&(1, 1, 0, 0)],
        &vertices[&(1, 0, 0, 2)],
        &vertices[&(1, 1, 0, 2)],
    ));
    faces.push(make_ruled_face(
        &vertices[&(0, 0, 1, 0)],
        &vertices[&(0, 1, 1, 0)],
        &vertices[&(0, 0, 1, 2)],
        &vertices[&(0, 1, 1, 2)],
    ));
    faces.push(make_ruled_face(
        &vertices[&(1, 0, 1, 0)],
        &vertices[&(1, 1, 1, 0)],
        &vertices[&(1, 0, 1, 2)],
        &vertices[&(1, 1, 1, 2)],
    ));

    // Z-axis chamfers
    faces.push(make_ruled_face(
        &vertices[&(0, 0, 0, 0)],
        &vertices[&(0, 0, 1, 0)],
        &vertices[&(0, 0, 0, 1)],
        &vertices[&(0, 0, 1, 1)],
    ));
    faces.push(make_ruled_face(
        &vertices[&(1, 0, 0, 0)],
        &vertices[&(1, 0, 1, 0)],
        &vertices[&(1, 0, 0, 1)],
        &vertices[&(1, 0, 1, 1)],
    ));
    faces.push(make_ruled_face(
        &vertices[&(0, 1, 0, 0)],
        &vertices[&(0, 1, 1, 0)],
        &vertices[&(0, 1, 0, 1)],
        &vertices[&(0, 1, 1, 1)],
    ));
    faces.push(make_ruled_face(
        &vertices[&(1, 1, 0, 0)],
        &vertices[&(1, 1, 1, 0)],
        &vertices[&(1, 1, 0, 1)],
        &vertices[&(1, 1, 1, 1)],
    ));

    // 4. Add the 8 planar corner triangles
    for i in 0..=1 {
        for j in 0..=1 {
            for k in 0..=1 {
                let corner_pt = frame.to_world(i as f64 * dx, j as f64 * dy, k as f64 * dz);

                let v1 = vertices[&(i, j, k, 0)].clone();
                let v2 = vertices[&(i, j, k, 1)].clone();
                let v3 = vertices[&(i, j, k, 2)].clone();

                faces.push(make_planar_corner_face(&v1, &v2, &v3, corner_pt));
            }
        }
    }

    // Sew the 26 faces into a single watertight Shell
    let shell = sew(&faces, distance * 0.1);
    Solid::new(shell)
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::Pnt;
    use openrcad_primitives::make_box;

    #[test]
    fn test_box_chamfering() {
        let cube = make_box(&Pnt::origin(), 1.0, 1.0, 1.0);
        let chamfered = chamfer(&cube, 0.1).unwrap();

        // A box chamfered on all edges has:
        // 6 trimmed flat faces + 12 ruled chamfer faces + 8 triangular corner faces = 26 faces.
        // 24 vertices.
        // 48 edges.
        // Euler characteristic: V - E + F = 24 - 48 + 26 = 2.
        assert_eq!(chamfered.face_count(), 26);
        assert_eq!(chamfered.vertex_count(), 24);
        assert_eq!(chamfered.edge_count(), 48);

        // Verify Euler characteristic
        let v = chamfered.vertex_count() as i32;
        let e = chamfered.edge_count() as i32;
        let f = chamfered.face_count() as i32;
        assert_eq!(v - e + f, 2);

        // Verify face surface types:
        // 6 Planes + 8 Planes = 14 Planes, and 12 Ruled surfaces
        let mut planes_count = 0;
        let mut ruled_count = 0;

        for face in chamfered.shell().faces() {
            if let Some(surf) = face.surface() {
                match surf {
                    GeomSurface::Plane(_) => planes_count += 1,
                    GeomSurface::Ruled(_) => ruled_count += 1,
                    _ => {}
                }
            }
        }

        assert_eq!(planes_count, 14);
        assert_eq!(ruled_count, 12);
    }
}
