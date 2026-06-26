use core::fmt;

use openrcad_foundation::{tolerance, Dir, Pnt, Vec as GeomVec};
use openrcad_geom::{CylindricalSurface, GeomCurve, GeomSurface, Plane, RuledSurface};
use openrcad_topo::{Edge, Face, FaceId, Solid, Vertex, Wire};
use std::collections::{HashMap, HashSet};

use crate::blend::{chamfer_cylinder, detect_cylinder, BlendError};
use crate::rolling_ball::{
    adjacent_faces, endpoint_cap_faces, farthest_endpoint, is_concave_cut_cylinder,
    line_meets_cylinder, nearest_endpoint, orient_edge_between, planar_outward_normal,
    polyline_edge, relocate_edge, same_face, trim_face_along_spine, trim_face_at_corner,
    RollingBallError,
};
use crate::sew::sew;

/// Errors reported by selected-edge chamfer construction.
#[derive(Clone, Debug, PartialEq)]
pub enum ChamferError {
    /// Distance must be finite and non-negative.
    InvalidDistance { distance: f64 },
    /// The selected edge is degenerate.
    DegenerateSpine,
    /// The selected edge is not shared by exactly two faces in the solid.
    EdgeAdjacency { count: usize },
    /// Native chamfer v1 supports selected planar-planar edges.
    UnsupportedSurfacePair,
    /// The adjacent planes do not define a clean chamferable wedge.
    InvalidDihedral,
    /// The selected edge could not be re-located after earlier chamfers.
    SpineNotOnFace,
    /// Endpoint trimming could not be represented by the current local topology.
    UnsupportedTrimTopology,
    /// The rebuilt shell was not watertight and healthy.
    InvalidTopology,
}

impl fmt::Display for ChamferError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidDistance { distance } => {
                write!(
                    f,
                    "chamfer: distance must be finite and non-negative, got {distance}"
                )
            }
            Self::DegenerateSpine => f.write_str("chamfer: selected edge is degenerate"),
            Self::EdgeAdjacency { count } => write!(
                f,
                "chamfer: selected edge must have exactly two adjacent faces, found {count}"
            ),
            Self::UnsupportedSurfacePair => f.write_str(
                "chamfer: native selected-edge chamfer currently supports planar-planar edges",
            ),
            Self::InvalidDihedral => {
                f.write_str("chamfer: adjacent faces do not form a chamferable wedge")
            }
            Self::SpineNotOnFace => {
                f.write_str("chamfer: selected edge was not found on the current body")
            }
            Self::UnsupportedTrimTopology => {
                f.write_str("chamfer: endpoint trim topology is not supported")
            }
            Self::InvalidTopology => {
                f.write_str("chamfer: rebuilt body is not watertight and healthy")
            }
        }
    }
}

impl std::error::Error for ChamferError {}

impl From<RollingBallError> for ChamferError {
    fn from(value: RollingBallError) -> Self {
        match value {
            RollingBallError::InvalidRadius { radius } => {
                ChamferError::InvalidDistance { distance: radius }
            }
            RollingBallError::DegenerateSpine => ChamferError::DegenerateSpine,
            RollingBallError::EdgeAdjacency { count } => ChamferError::EdgeAdjacency { count },
            RollingBallError::InvalidDihedral => ChamferError::InvalidDihedral,
            RollingBallError::SpineNotOnFace => ChamferError::SpineNotOnFace,
            RollingBallError::UnsupportedTrimTopology => ChamferError::UnsupportedTrimTopology,
            RollingBallError::UnsolvableAdjacency { .. }
            | RollingBallError::NewtonDiverged { .. }
            | RollingBallError::BlendSurfaceBuild(_) => ChamferError::UnsupportedTrimTopology,
            RollingBallError::InvalidTopology => ChamferError::InvalidTopology,
        }
    }
}

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

/// Apply a planar selected-edge chamfer to several edges.
///
/// Edges are chamfered sequentially, matching [`crate::rolling_ball::fillet_edges`]:
/// after each rebuild the next requested edge is relocated in the evolving body.
/// Unsupported local geometry returns a clean error and leaves upstream callers
/// free to keep the original body.
pub fn chamfer_edges(solid: &Solid, edges: &[Edge], distance: f64) -> Result<Solid, ChamferError> {
    if !distance.is_finite() || distance < 0.0 {
        return Err(ChamferError::InvalidDistance { distance });
    }
    if distance <= tolerance::CONFUSION {
        return Ok(solid.clone());
    }

    let mut current = solid.clone();
    for edge in edges {
        let target = relocate_edge(&current, edge).ok_or(ChamferError::SpineNotOnFace)?;
        current = chamfer_planar_edge(&current, &target, distance)?;
    }
    Ok(current)
}

#[derive(Clone)]
struct ChamferBlend {
    spine: Edge,
    face_a: Face,
    face_b: Face,
    contact_a: Edge,
    contact_b: Edge,
    chamfer_face: Face,
    start_edge: Edge,
    end_edge: Edge,
}

#[derive(Clone, Copy)]
enum Endpoint {
    Start,
    End,
}

fn chamfer_planar_edge(solid: &Solid, edge: &Edge, distance: f64) -> Result<Solid, ChamferError> {
    let adjacent = adjacent_faces(solid, edge);
    if adjacent.len() != 2 {
        return Err(ChamferError::EdgeAdjacency {
            count: adjacent.len(),
        });
    }
    if !matches!(adjacent[0].surface(), Some(GeomSurface::Plane(_)))
        || !matches!(adjacent[1].surface(), Some(GeomSurface::Plane(_)))
    {
        return Err(ChamferError::UnsupportedSurfacePair);
    }

    let mut blend = planar_chamfer(edge, &adjacent[0], &adjacent[1], distance)?;
    let start = edge.source().point();
    let end = edge.target().point();
    let start_caps = endpoint_cap_faces(solid, start, &blend.face_a, &blend.face_b);
    let end_caps = endpoint_cap_faces(solid, end, &blend.face_a, &blend.face_b);

    let mut faces = Vec::new();
    let mut skipped_faces = HashSet::new();
    close_chamfer_endpoint(
        solid,
        &mut blend,
        start,
        &start_caps,
        Endpoint::Start,
        &mut faces,
        &mut skipped_faces,
    )?;
    close_chamfer_endpoint(
        solid,
        &mut blend,
        end,
        &end_caps,
        Endpoint::End,
        &mut faces,
        &mut skipped_faces,
    )?;

    let trimmed_a = trim_face_along_spine(&blend.face_a, &blend.spine, &blend.contact_a)?;
    let trimmed_b = trim_face_along_spine(&blend.face_b, &blend.spine, &blend.contact_b)?;

    for face in solid.shell().faces() {
        if same_face(&face, &blend.face_a)
            || same_face(&face, &blend.face_b)
            || skipped_faces.contains(&face.id())
        {
            continue;
        }
        faces.push(face);
    }

    faces.push(trimmed_a);
    faces.push(trimmed_b);
    faces.push(blend.chamfer_face);

    let result = Solid::new(sew(&faces, distance * 0.1));
    let merged = crate::merge::merge_cocylindrical_faces(&crate::merge::merge_coplanar_faces(
        &result,
    ));
    if merged.is_watertight() && merged.health_report().is_healthy() {
        return Ok(merged);
    }
    if result.is_watertight() && result.health_report().is_healthy() {
        return Ok(result);
    }
    Err(ChamferError::InvalidTopology)
}

fn planar_chamfer(
    edge: &Edge,
    face_a: &Face,
    face_b: &Face,
    distance: f64,
) -> Result<ChamferBlend, ChamferError> {
    let p0 = edge.source().point();
    let p1 = edge.target().point();
    let spine_vec = p1 - p0;
    if spine_vec.magnitude() <= tolerance::CONFUSION {
        return Err(ChamferError::DegenerateSpine);
    }

    let n_a = planar_outward_normal(face_a)?;
    let n_b = planar_outward_normal(face_b)?;
    let n_a_vec = GeomVec::from_dir(n_a);
    let n_b_vec = GeomVec::from_dir(n_b);
    if n_a_vec.cross(&n_b_vec).magnitude() <= tolerance::CONFUSION {
        return Err(ChamferError::InvalidDihedral);
    }

    let offset_dir_a = project_onto_plane(-n_b_vec, n_a_vec)
        .normalized()
        .ok_or(ChamferError::InvalidDihedral)?;
    let offset_dir_b = project_onto_plane(-n_a_vec, n_b_vec)
        .normalized()
        .ok_or(ChamferError::InvalidDihedral)?;

    let off_a = GeomVec::from_dir(offset_dir_a) * distance;
    let off_b = GeomVec::from_dir(offset_dir_b) * distance;
    let a0 = p0 + off_a;
    let a1 = p1 + off_a;
    let b0 = p0 + off_b;
    let b1 = p1 + off_b;

    let contact_a = Edge::between_points(a0, a1);
    let contact_b = Edge::between_points(b0, b1);
    let start_edge = Edge::between_points(b0, a0);
    let end_edge = Edge::between_points(a1, b1);
    let chamfer_face = chamfer_face_from_edges(&contact_a, &contact_b, &start_edge, &end_edge)?;

    Ok(ChamferBlend {
        spine: edge.clone(),
        face_a: face_a.clone(),
        face_b: face_b.clone(),
        contact_a,
        contact_b,
        chamfer_face,
        start_edge,
        end_edge,
    })
}

fn project_onto_plane(v: GeomVec, normal: GeomVec) -> GeomVec {
    v - normal * v.dot(&normal)
}

#[allow(clippy::too_many_arguments)]
fn close_chamfer_endpoint(
    solid: &Solid,
    blend: &mut ChamferBlend,
    corner: Pnt,
    caps: &[Face],
    endpoint: Endpoint,
    faces: &mut Vec<Face>,
    skipped: &mut HashSet<FaceId>,
) -> Result<(), ChamferError> {
    if caps.is_empty() {
        let ca = contact_endpoint(&blend.contact_a, endpoint);
        let cb = contact_endpoint(&blend.contact_b, endpoint);
        let trim = Edge::between_points(ca, cb);
        faces.push(chamfer_internal_stop_face(corner, ca, cb)?);
        update_chamfer_endpoint(blend, endpoint, ca, cb, trim)?;
        return Ok(());
    }
    if caps.len() != 1 {
        return Err(ChamferError::UnsupportedTrimTopology);
    }
    let cap = &caps[0];

    let (ca, cb, trim) = if is_concave_cut_cylinder(solid, cap) {
        let cut_cyl = match cap.surface() {
            Some(GeomSurface::Cylinder(c)) => *c,
            _ => return Err(ChamferError::UnsupportedTrimTopology),
        };
        let plane = match blend.chamfer_face.surface() {
            Some(GeomSurface::Plane(p)) => *p,
            _ => return Err(ChamferError::UnsupportedSurfacePair),
        };

        let a_corner = nearest_endpoint(&blend.contact_a, corner);
        let b_corner = nearest_endpoint(&blend.contact_b, corner);
        let a_far = farthest_endpoint(&blend.contact_a, corner);
        let b_far = farthest_endpoint(&blend.contact_b, corner);
        let a_dir = (a_corner - a_far)
            .normalized()
            .ok_or(ChamferError::DegenerateSpine)?;
        let b_dir = (b_corner - b_far)
            .normalized()
            .ok_or(ChamferError::DegenerateSpine)?;
        let ca_real = line_meets_cylinder(a_far, a_dir, &cut_cyl, a_corner)
            .ok_or(ChamferError::UnsupportedTrimTopology)?;
        let cb_real = line_meets_cylinder(b_far, b_dir, &cut_cyl, b_corner)
            .ok_or(ChamferError::UnsupportedTrimTopology)?;
        let trim = plane_cylinder_trim_edge(&plane, &cut_cyl, ca_real, cb_real)
            .ok_or(ChamferError::UnsupportedTrimTopology)?;
        (ca_real, cb_real, trim)
    } else {
        let ca = contact_endpoint(&blend.contact_a, endpoint);
        let cb = contact_endpoint(&blend.contact_b, endpoint);
        (ca, cb, Edge::between_points(ca, cb))
    };

    let trimmed_cap = trim_face_at_corner(cap, corner, ca, cb, &trim)?;
    faces.push(trimmed_cap);
    skipped.insert(cap.id());

    update_chamfer_endpoint(blend, endpoint, ca, cb, trim)?;
    Ok(())
}

fn update_chamfer_endpoint(
    blend: &mut ChamferBlend,
    endpoint: Endpoint,
    ca: Pnt,
    cb: Pnt,
    trim: Edge,
) -> Result<(), ChamferError> {
    if matches!(endpoint, Endpoint::Start) {
        let keep_a = blend.contact_a.end().point();
        let keep_b = blend.contact_b.end().point();
        blend.contact_a = rebuild_contact(&blend.contact_a, keep_a, ca);
        blend.contact_b = rebuild_contact(&blend.contact_b, keep_b, cb);
        blend.start_edge = trim;
    } else {
        let keep_a = blend.contact_a.start().point();
        let keep_b = blend.contact_b.start().point();
        blend.contact_a = rebuild_contact(&blend.contact_a, keep_a, ca);
        blend.contact_b = rebuild_contact(&blend.contact_b, keep_b, cb);
        blend.end_edge = trim;
    }
    blend.chamfer_face = chamfer_face_from_edges(
        &blend.contact_a,
        &blend.contact_b,
        &blend.start_edge,
        &blend.end_edge,
    )?;
    Ok(())
}

fn chamfer_internal_stop_face(corner: Pnt, ca: Pnt, cb: Pnt) -> Result<Face, ChamferError> {
    let normal = (ca - corner)
        .cross(&(cb - corner))
        .normalized()
        .ok_or(ChamferError::InvalidDihedral)
        .map(GeomVec::from_dir)?;
    let surf = GeomSurface::plane(Plane::from_point_normal(
        corner,
        Dir::new(normal.x(), normal.y(), normal.z()),
    ));
    Ok(Face::new(
        Some(surf),
        Wire::from_edges(vec![
            Edge::between_points(corner, ca),
            Edge::between_points(ca, cb),
            Edge::between_points(cb, corner),
        ]),
    ))
}

fn contact_endpoint(edge: &Edge, endpoint: Endpoint) -> Pnt {
    match endpoint {
        Endpoint::Start => edge.start().point(),
        Endpoint::End => edge.end().point(),
    }
}

fn rebuild_contact(edge: &Edge, keep: Pnt, moved: Pnt) -> Edge {
    let s = edge.source().point();
    if s.distance(&keep) <= s.distance(&moved) {
        Edge::between_points(keep, moved)
    } else {
        Edge::between_points(moved, keep)
    }
}

fn chamfer_face_from_edges(
    contact_a: &Edge,
    contact_b: &Edge,
    start_edge: &Edge,
    end_edge: &Edge,
) -> Result<Face, ChamferError> {
    let a0 = contact_a.start().point();
    let a1 = contact_a.end().point();
    let b0 = contact_b.start().point();
    let b1 = contact_b.end().point();
    let end_edge = orient_edge_between(end_edge, a1, b1);
    let start_edge = orient_edge_between(start_edge, b0, a0);
    let normal = (a1 - a0)
        .cross(&(b1 - a1))
        .normalized()
        .ok_or(ChamferError::InvalidDihedral)
        .map(GeomVec::from_dir)?;
    let surf = GeomSurface::plane(Plane::from_point_normal(
        a0,
        Dir::new(normal.x(), normal.y(), normal.z()),
    ));
    let wire = Wire::from_edges([
        contact_a.clone(),
        end_edge,
        contact_b.clone().reversed(),
        start_edge,
    ]);
    Ok(Face::new(Some(surf), wire))
}

fn plane_cylinder_trim_edge(
    plane: &Plane,
    cut: &CylindricalSurface,
    p_a: Pnt,
    p_b: Pnt,
) -> Option<Edge> {
    use core::f64::consts::PI;

    let axis_pt = cut.position().location();
    let z = GeomVec::from_dir(cut.position().direction());
    let x = GeomVec::from_dir(cut.position().x_direction());
    let y = GeomVec::from_dir(cut.position().y_direction());
    let n = GeomVec::from_dir(plane.normal());
    let nz = z.dot(&n);
    let tol = 100.0 * tolerance::CONFUSION;

    if nz.abs() <= tol {
        let chord = p_b - p_a;
        if chord.cross(&z).magnitude() <= 1e-5 * chord.magnitude().max(1.0) {
            return Some(Edge::between_points(p_a, p_b));
        }
        return None;
    }

    let angle_of = |p: Pnt| -> f64 {
        let v = p - axis_pt;
        let radial = v - z * v.dot(&z);
        radial.dot(&y).atan2(radial.dot(&x))
    };
    let theta_a = angle_of(p_a);
    let mut theta_b = angle_of(p_b);
    while theta_b - theta_a > PI {
        theta_b -= 2.0 * PI;
    }
    while theta_a - theta_b > PI {
        theta_b += 2.0 * PI;
    }

    let span = (theta_b - theta_a).abs();
    let steps = ((span / (PI / 32.0)).ceil() as usize).clamp(8, 64);
    let mut pts = Vec::with_capacity(steps + 1);
    for k in 0..=steps {
        if k == 0 {
            pts.push(p_a);
            continue;
        }
        if k == steps {
            pts.push(p_b);
            continue;
        }
        let theta = theta_a + (theta_b - theta_a) * (k as f64) / (steps as f64);
        let radial = x * theta.cos() + y * theta.sin();
        let z_param = ((plane.location() - axis_pt) - radial * cut.radius()).dot(&n) / nz;
        pts.push(axis_pt + radial * cut.radius() + z * z_param);
    }
    Some(polyline_edge(&pts))
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
    use openrcad_geom::Curve;
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

    #[test]
    fn plane_cylinder_trim_samples_when_axis_not_parallel_to_plane() {
        let cyl =
            CylindricalSurface::new(openrcad_foundation::Ax3::new(Pnt::origin(), Dir::dz()), 2.0);
        let n = Dir::new(-1.0, 0.0, 1.0);
        let plane = Plane::from_point_normal(Pnt::origin(), n);
        let p_a = Pnt::new(2.0, 0.0, 2.0);
        let p_b = Pnt::new(0.0, 2.0, 0.0);

        let edge = plane_cylinder_trim_edge(&plane, &cyl, p_a, p_b)
            .expect("non-parallel plane-cylinder intersection should trim");
        let curve = edge.curve().expect("trim edge has a curve");
        for k in 0..=8 {
            let t = edge.first() + (edge.last() - edge.first()) * (k as f64) / 8.0;
            let p = curve.point(t);
            let radial = (p.x() * p.x() + p.y() * p.y()).sqrt();
            assert!((radial - 2.0).abs() < 5e-3, "sample must stay on cylinder");
            assert!((p.z() - p.x()).abs() < 5e-3, "sample must stay on plane");
        }
    }

    #[test]
    fn plane_cylinder_trim_parallel_axis_requires_same_generator() {
        let cyl =
            CylindricalSurface::new(openrcad_foundation::Ax3::new(Pnt::origin(), Dir::dz()), 2.0);
        let plane = Plane::from_point_normal(Pnt::new(2.0, 0.0, 0.0), Dir::dx());
        let p_a = Pnt::new(2.0, 0.0, 0.0);
        let p_b = Pnt::new(2.0, 0.0, 5.0);
        let edge = plane_cylinder_trim_edge(&plane, &cyl, p_a, p_b)
            .expect("same generator should be a straight trim");
        assert!(edge.start().point().distance(&p_a) < 1e-9);
        assert!(edge.end().point().distance(&p_b) < 1e-9);

        let unsupported = plane_cylinder_trim_edge(
            &Plane::from_point_normal(Pnt::origin(), Dir::dx()),
            &cyl,
            Pnt::new(0.0, 2.0, 0.0),
            Pnt::new(0.0, -2.0, 5.0),
        );
        assert!(
            unsupported.is_none(),
            "different generators are unsupported"
        );
    }
}
