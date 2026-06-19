use openrcad_foundation::{tolerance, Ax3, Dir, Pnt, Vec as GeomVec};
use openrcad_geom::{Circle, CylindricalSurface, GeomCurve, GeomSurface, Plane, SphericalSurface};
use openrcad_topo::{Edge, Face, Solid, Vertex, Wire};
use std::collections::HashMap;

use crate::blend::{detect_cylinder, fillet_cylinder, BlendError};
use crate::sew::sew;

/// Roll a constant-`radius` fillet along every edge of `solid`.
pub fn fillet(solid: &Solid, radius: f64) -> Result<Solid, BlendError> {
    if radius <= tolerance::CONFUSION {
        return Ok(solid.clone());
    }
    if let Some((p0, ex, ey, ez, dx, dy, dz)) = detect_box(solid) {
        // A fillet rolling in from every face meets in the middle once the radius
        // reaches half the smallest box dimension.
        let max = dx.min(dy).min(dz) / 2.0;
        if radius >= max {
            return Err(BlendError::ParameterTooLarge {
                requested: radius,
                max,
            });
        }
        return Ok(fillet_box(p0, ex, ey, ez, dx, dy, dz, radius));
    }
    if let Some(cyl) = detect_cylinder(solid) {
        return fillet_cylinder(&cyl, radius);
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

fn make_circular_arc(c: Pnt, r: f64, v1: &Vertex, v2: &Vertex) -> Edge {
    let p1 = v1.point();
    let p2 = v2.point();
    let d1 = (p1 - c) / r;
    let d2 = (p2 - c) / r;
    let main = d1.cross(&d2);
    let dir = Dir::new(main.x(), main.y(), main.z());
    let xdir = Dir::new(d1.x(), d1.y(), d1.z());
    let pos = Ax3::new_axes(c, dir, xdir);
    let circle = Circle::new(pos, r);
    Edge::new_with_tolerance(
        Some(GeomCurve::circle(circle)),
        0.0,
        std::f64::consts::FRAC_PI_2,
        v1.clone(),
        v2.clone(),
        tolerance::CONFUSION,
    )
}

fn make_cylindrical_face(
    v_start_1: &Vertex,
    v_end_1: &Vertex,
    v_start_2: &Vertex,
    v_end_2: &Vertex,
    c_start: Pnt,
    c_end: Pnt,
    radius: f64,
) -> Face {
    let axis_dir = (c_end - c_start).normalized().unwrap();
    let x = (v_start_1.point() - c_start).normalized().unwrap();
    let _y = axis_dir.cross(&x);

    let _d2 = (v_start_2.point() - c_start).normalized().unwrap();
    let pos = Ax3::new_axes(
        c_start,
        Dir::new(axis_dir.x(), axis_dir.y(), axis_dir.z()),
        Dir::new(x.x(), x.y(), x.z()),
    );

    // Build the boundary wire in u,v parameter space:
    // A=(0, 0), B=(0, len), C=(u_end, len), D=(u_end, 0)
    let a = v_start_1;
    let b = v_end_1;
    let c = v_end_2;
    let d = v_start_2;

    let e1 = make_straight_edge(a, b);
    let e2 = make_circular_arc(c_end, radius, b, c);
    let e3 = make_straight_edge(c, d);
    let e4 = make_circular_arc(c_start, radius, d, a);

    let w = Wire::from_edges([e1, e2, e3, e4]);
    let surf = GeomSurface::cylinder(CylindricalSurface::new(pos, radius));
    Face::new(Some(surf), w)
}

fn make_spherical_face(v1: &Vertex, v2: &Vertex, v3: &Vertex, c: Pnt, radius: f64) -> Face {
    let p1 = v1.point();
    let p2 = v2.point();
    let p3 = v3.point();

    let normal_approx = (p2 - p1).cross(&(p3 - p1));
    let to_outside = p1 - c;
    let (va, vb, vc) = if normal_approx.dot(&to_outside) > 0.0 {
        (v1, v2, v3)
    } else {
        (v1, v3, v2)
    };

    let arc1 = make_circular_arc(c, radius, va, vb);
    let arc2 = make_circular_arc(c, radius, vb, vc);
    let arc3 = make_circular_arc(c, radius, vc, va);

    let w = Wire::from_edges([arc1, arc2, arc3]);
    let pos = Ax3::new(c, Dir::dz());
    let surf = GeomSurface::sphere(SphericalSurface::new(pos, radius));
    Face::new(Some(surf), w)
}

#[allow(clippy::too_many_arguments)] // the box frame is naturally seven scalars/vectors
fn fillet_box(
    p0: Pnt,
    ex: GeomVec,
    ey: GeomVec,
    ez: GeomVec,
    dx: f64,
    dy: f64,
    dz: f64,
    radius: f64,
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
                    i as f64 * dx - sx * radius,
                    j as f64 * dy - sy * radius,
                    k as f64 * dz,
                );
                vertices.insert((i, j, k, 2), Vertex::new(p_z));

                // V_y on flat face normal to Y
                let p_y = frame.to_world(
                    i as f64 * dx - sx * radius,
                    j as f64 * dy,
                    k as f64 * dz - sz * radius,
                );
                vertices.insert((i, j, k, 1), Vertex::new(p_y));

                // V_x on flat face normal to X
                let p_x = frame.to_world(
                    i as f64 * dx,
                    j as f64 * dy - sy * radius,
                    k as f64 * dz - sz * radius,
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

    // 3. Add the 12 cylindrical fillets
    // X-axis fillets
    faces.push(make_cylindrical_face(
        &vertices[&(0, 0, 0, 1)],
        &vertices[&(1, 0, 0, 1)],
        &vertices[&(0, 0, 0, 2)],
        &vertices[&(1, 0, 0, 2)],
        frame.to_world(radius, radius, radius),
        frame.to_world(dx - radius, radius, radius),
        radius,
    ));
    faces.push(make_cylindrical_face(
        &vertices[&(0, 1, 0, 1)],
        &vertices[&(1, 1, 0, 1)],
        &vertices[&(0, 1, 0, 2)],
        &vertices[&(1, 1, 0, 2)],
        frame.to_world(radius, dy - radius, radius),
        frame.to_world(dx - radius, dy - radius, radius),
        radius,
    ));
    faces.push(make_cylindrical_face(
        &vertices[&(0, 0, 1, 1)],
        &vertices[&(1, 0, 1, 1)],
        &vertices[&(0, 0, 1, 2)],
        &vertices[&(1, 0, 1, 2)],
        frame.to_world(radius, radius, dz - radius),
        frame.to_world(dx - radius, radius, dz - radius),
        radius,
    ));
    faces.push(make_cylindrical_face(
        &vertices[&(0, 1, 1, 1)],
        &vertices[&(1, 1, 1, 1)],
        &vertices[&(0, 1, 1, 2)],
        &vertices[&(1, 1, 1, 2)],
        frame.to_world(radius, dy - radius, dz - radius),
        frame.to_world(dx - radius, dy - radius, dz - radius),
        radius,
    ));

    // Y-axis fillets
    faces.push(make_cylindrical_face(
        &vertices[&(0, 0, 0, 0)],
        &vertices[&(0, 1, 0, 0)],
        &vertices[&(0, 0, 0, 2)],
        &vertices[&(0, 1, 0, 2)],
        frame.to_world(radius, radius, radius),
        frame.to_world(radius, dy - radius, radius),
        radius,
    ));
    faces.push(make_cylindrical_face(
        &vertices[&(1, 0, 0, 0)],
        &vertices[&(1, 1, 0, 0)],
        &vertices[&(1, 0, 0, 2)],
        &vertices[&(1, 1, 0, 2)],
        frame.to_world(dx - radius, radius, radius),
        frame.to_world(dx - radius, dy - radius, radius),
        radius,
    ));
    faces.push(make_cylindrical_face(
        &vertices[&(0, 0, 1, 0)],
        &vertices[&(0, 1, 1, 0)],
        &vertices[&(0, 0, 1, 2)],
        &vertices[&(0, 1, 1, 2)],
        frame.to_world(radius, radius, dz - radius),
        frame.to_world(radius, dy - radius, dz - radius),
        radius,
    ));
    faces.push(make_cylindrical_face(
        &vertices[&(1, 0, 1, 0)],
        &vertices[&(1, 1, 1, 0)],
        &vertices[&(1, 0, 1, 2)],
        &vertices[&(1, 1, 1, 2)],
        frame.to_world(dx - radius, radius, dz - radius),
        frame.to_world(dx - radius, dy - radius, dz - radius),
        radius,
    ));

    // Z-axis fillets
    faces.push(make_cylindrical_face(
        &vertices[&(0, 0, 0, 0)],
        &vertices[&(0, 0, 1, 0)],
        &vertices[&(0, 0, 0, 1)],
        &vertices[&(0, 0, 1, 1)],
        frame.to_world(radius, radius, radius),
        frame.to_world(radius, radius, dz - radius),
        radius,
    ));
    faces.push(make_cylindrical_face(
        &vertices[&(1, 0, 0, 0)],
        &vertices[&(1, 0, 1, 0)],
        &vertices[&(1, 0, 0, 1)],
        &vertices[&(1, 0, 1, 1)],
        frame.to_world(dx - radius, radius, radius),
        frame.to_world(dx - radius, radius, dz - radius),
        radius,
    ));
    faces.push(make_cylindrical_face(
        &vertices[&(0, 1, 0, 0)],
        &vertices[&(0, 1, 1, 0)],
        &vertices[&(0, 1, 0, 1)],
        &vertices[&(0, 1, 1, 1)],
        frame.to_world(radius, dy - radius, radius),
        frame.to_world(radius, dy - radius, dz - radius),
        radius,
    ));
    faces.push(make_cylindrical_face(
        &vertices[&(1, 1, 0, 0)],
        &vertices[&(1, 1, 1, 0)],
        &vertices[&(1, 1, 0, 1)],
        &vertices[&(1, 1, 1, 1)],
        frame.to_world(dx - radius, dy - radius, radius),
        frame.to_world(dx - radius, dy - radius, dz - radius),
        radius,
    ));

    // 4. Add the 8 spherical corners
    for i in 0..=1 {
        for j in 0..=1 {
            for k in 0..=1 {
                let cx = if i == 1 { dx - radius } else { radius };
                let cy = if j == 1 { dy - radius } else { radius };
                let cz = if k == 1 { dz - radius } else { radius };
                let center = frame.to_world(cx, cy, cz);

                let v1 = vertices[&(i, j, k, 0)].clone();
                let v2 = vertices[&(i, j, k, 1)].clone();
                let v3 = vertices[&(i, j, k, 2)].clone();

                faces.push(make_spherical_face(&v1, &v2, &v3, center, radius));
            }
        }
    }

    // Sew the 26 faces into a single watertight Shell
    let shell = sew(&faces, radius * 0.1);
    Solid::new(shell)
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::Pnt;
    use openrcad_primitives::make_box;

    #[test]
    fn test_box_filleting() {
        let cube = make_box(&Pnt::origin(), 1.0, 1.0, 1.0);
        let filleted = fillet(&cube, 0.1).unwrap();

        // A box filleted on all edges should have:
        // 6 trimmed flat faces + 12 cylindrical fillet faces + 8 spherical corner faces = 26 faces.
        // 24 vertices.
        // 48 edges.
        // Euler characteristic: V - E + F = 24 - 48 + 26 = 2.
        assert_eq!(filleted.face_count(), 26);
        assert_eq!(filleted.vertex_count(), 24);
        assert_eq!(filleted.edge_count(), 48);

        // Verify Euler characteristic
        let v = filleted.vertex_count() as i32;
        let e = filleted.edge_count() as i32;
        let f = filleted.face_count() as i32;
        assert_eq!(v - e + f, 2);

        // Verify face surface types:
        // 6 Planes, 12 Cylinders, 8 Spheres
        let mut planes_count = 0;
        let mut cylinders_count = 0;
        let mut spheres_count = 0;

        for face in filleted.shell().faces() {
            if let Some(surf) = face.surface() {
                match surf {
                    GeomSurface::Plane(_) => planes_count += 1,
                    GeomSurface::Cylinder(_) => cylinders_count += 1,
                    GeomSurface::Sphere(_) => spheres_count += 1,
                    _ => {}
                }
            }
        }

        assert_eq!(planes_count, 6);
        assert_eq!(cylinders_count, 12);
        assert_eq!(spheres_count, 8);
    }
}
