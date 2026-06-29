#[allow(unused_imports)]
use super::*;

/// Number of segments used to draw a smooth circular wireframe outline.
pub(crate) const CYL_WIRE_SEGS: usize = crate::CIRCLE_SEGS;

pub(crate) fn build_box_wireframe(w: f32, h: f32, d: f32) -> (Vec<f32>, Vec<u32>, Vec<f32>) {
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

pub(crate) fn build_cylinder_wireframe(
    r: f32,
    h: f32,
    segments: u32,
) -> (Vec<f32>, Vec<u32>, Vec<f32>) {
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

pub(crate) fn build_extrusion_wireframe(
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
pub(crate) fn build_oriented_cylinder_wireframe(
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
