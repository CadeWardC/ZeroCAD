//! Rolling-ball edge blend construction.
//!
//! This module is the first local-blend slice: it solves the contact curves for
//! a constant-radius ball rolling along a selected edge shared by two planar
//! faces, then builds the cylindrical blend face bounded by those contacts.

use core::fmt;

use openrcad_foundation::{tolerance, Ax3, Dir, Pnt, Vec as GeomVec};
use openrcad_geom::{Circle, CylindricalSurface, GeomCurve, GeomSurface, ToroidalSurface, GregorySurface, Curve, Surface};
use openrcad_topo::{Edge, Face, Orientation, Solid, Vertex, Wire};

use crate::sew::sew;

/// Reasons the rolling-ball solver could not resolve a face adjacency.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AdjacencyReason {
    /// An adjacent face's surface is neither planar nor analytic-supported.
    NotPlaneOrAnalytic,
    /// Both adjacent faces are non-planar and no analytic path applies.
    BothFacesNonPlanar,
    /// The numeric tangency solver failed to converge.
    NewtonFailed,
    /// The requested radius exceeds what fits the adjacent geometry.
    RadiusTooLarge,
    /// The pair of surface types has no supported solver path.
    UnsupportedSurfacePair,
}

/// Errors reported by the rolling-ball solver.
#[derive(Clone, Debug, PartialEq)]
pub enum RollingBallError {
    /// Radius must be finite and positive.
    InvalidRadius { radius: f64 },
    /// The selected edge is degenerate.
    DegenerateSpine,
    /// The selected edge is not shared by exactly two faces in the solid.
    EdgeAdjacency { count: usize },
    /// The two adjacent faces could not be blended (analytic or numeric).
    UnsolvableAdjacency { reason: AdjacencyReason },
    /// The adjacent faces are parallel or otherwise do not define a convex
    /// rolling-ball wedge.
    InvalidDihedral,
    /// The selected edge was not found in a face that should be trimmed.
    SpineNotOnFace,
    /// This local trim pass only supports simple outer-loop faces.
    UnsupportedTrimTopology,
    /// The numeric tangency solver diverged.
    NewtonDiverged { iterations: usize },
    /// The blend surface could not be constructed.
    BlendSurfaceBuild(&'static str),
}

impl fmt::Display for RollingBallError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidRadius { radius } => {
                write!(f, "rolling ball: radius must be positive, got {radius}")
            }
            Self::DegenerateSpine => f.write_str("rolling ball: selected edge is degenerate"),
            Self::EdgeAdjacency { count } => write!(
                f,
                "rolling ball: selected edge must have exactly two adjacent faces, found {count}"
            ),
            Self::UnsolvableAdjacency { reason } => {
                write!(f, "rolling ball: adjacent faces could not be blended ({reason:?})")
            }
            Self::InvalidDihedral => {
                f.write_str("rolling ball: adjacent faces do not form a blendable wedge")
            }
            Self::SpineNotOnFace => {
                f.write_str("rolling ball: selected edge was not found on an adjacent face")
            }
            Self::UnsupportedTrimTopology => f.write_str(
                "rolling ball: local trimming currently supports simple outer-loop faces only",
            ),
            Self::NewtonDiverged { iterations } => write!(
                f,
                "rolling ball: numeric tangency solver diverged after {iterations} iterations"
            ),
            Self::BlendSurfaceBuild(msg) => {
                write!(f, "rolling ball: blend surface could not be built ({msg})")
            }
        }
    }
}

impl std::error::Error for RollingBallError {}

/// Contact-curve and blend-surface result for one planar rolling-ball edge.
#[derive(Clone, Debug)]
pub struct RollingBallBlend {
    /// The selected spine edge.
    pub spine: Edge,
    /// First adjacent face.
    pub face_a: Face,
    /// Second adjacent face.
    pub face_b: Face,
    /// Contact curve on `face_a`.
    pub contact_a: Edge,
    /// Contact curve on `face_b`.
    pub contact_b: Edge,
    /// Ball-center path.
    pub centerline: Edge,
    /// Cylindrical blend face bounded by both contact curves and endpoint arcs.
    pub blend_face: Face,
    /// Arc on the start endpoint cap, from contact B to contact A.
    pub start_arc: Edge,
    /// Arc on the end endpoint cap, from contact A to contact B.
    pub end_arc: Edge,
    /// Fillet radius.
    pub radius: f64,
}

/// Apply a selected-edge rolling-ball fillet to a simple planar solid.
///
/// This performs the first surgical local B-Rep edit: the two adjacent planar
/// faces are trimmed to the contact curves, the two endpoint cap faces are
/// trimmed to the circular end arcs, the cylindrical blend face is inserted,
/// and the shell is sewn back into a watertight solid.
fn face_outward_normal_at(face: &Face, point: Pnt) -> Result<Dir, RollingBallError> {
    let normal = match face.surface() {
        Some(GeomSurface::Plane(plane)) => plane.normal(),
        Some(GeomSurface::Cylinder(cyl)) => {
            let axis_pt = cyl.position().location();
            let axis_dir = cyl.position().direction();
            let v = point - axis_pt;
            let proj = axis_pt + GeomVec::from_dir(axis_dir) * v.dot(&GeomVec::from_dir(axis_dir));
            let radial = (point - proj)
                .normalized()
                .ok_or(RollingBallError::UnsolvableAdjacency {
                    reason: AdjacencyReason::NotPlaneOrAnalytic,
                })?;
            Dir::new(radial.x(), radial.y(), radial.z())
        }
        Some(surf @ (GeomSurface::Cone(_) | GeomSurface::Sphere(_))) => {
            // General analytic surfaces: the normal is dU × dV at the nearest
            // parameter to `point`, computed from the surface's first derivatives.
            let (u, v) = surface_nearest_uv(surf, point);
            let (_, du, dv) = surf.d1(u, v);
            du.cross(&dv)
                .normalized()
                .map(|n| Dir::new(n.x(), n.y(), n.z()))
                .ok_or(RollingBallError::UnsolvableAdjacency {
                    reason: AdjacencyReason::NotPlaneOrAnalytic,
                })?
        }
        _ => {
            return Err(RollingBallError::UnsolvableAdjacency {
                reason: AdjacencyReason::NotPlaneOrAnalytic,
            })
        }
    };
    Ok(if face.orientation() == Orientation::Reversed {
        normal.reversed()
    } else {
        normal
    })
}

/// Nearest `(u, v)` on a general surface to `point`, via a coarse parameter
/// sweep refined by a short bounded Gauss-Newton search using `d1`.
fn surface_nearest_uv(surface: &GeomSurface, point: Pnt) -> (f64, f64) {
    let (mut u0, mut u1, mut v0, mut v1) = surface.bounds();
    if !u0.is_finite() {
        u0 = -100.0;
    }
    if !u1.is_finite() {
        u1 = 100.0;
    }
    if !v0.is_finite() {
        v0 = -100.0;
    }
    if !v1.is_finite() {
        v1 = 100.0;
    }

    let n = 16;
    let mut best = (u0, v0);
    let mut best_d2 = f64::INFINITY;
    for i in 0..=n {
        let u = u0 + (u1 - u0) * (i as f64) / (n as f64);
        for j in 0..=n {
            let v = v0 + (v1 - v0) * (j as f64) / (n as f64);
            let d2 = surface.point(u, v).distance_squared(&point);
            if d2 < best_d2 {
                best_d2 = d2;
                best = (u, v);
            }
        }
    }

    let (mut u, mut v) = best;
    for _ in 0..16 {
        let (s, du, dv) = surface.d1(u, v);
        let r = s - point;
        let guu = du.dot(&du);
        let gvv = dv.dot(&dv);
        let guv = du.dot(&dv);
        let bu = r.dot(&du);
        let bv = r.dot(&dv);
        let det = guu * gvv - guv * guv;
        if det.abs() <= 1e-14 {
            break;
        }
        let step_u = (bu * gvv - bv * guv) / det;
        let step_v = (guu * bv - guv * bu) / det;
        u = (u - step_u).clamp(u0, u1);
        v = (v - step_v).clamp(v0, v1);
        if step_u.abs() + step_v.abs() <= 1e-12 {
            break;
        }
    }
    (u, v)
}

fn radial(axis: Dir, xref: Dir, u: f64) -> GeomVec {
    let x = GeomVec::from_dir(xref);
    let y = GeomVec::from_dir(axis).cross(&GeomVec::from_dir(xref));
    x * u.cos() + y * u.sin()
}

fn make_gregory_corner_patch(
    corner: Pnt,
    p_a: Pnt,
    p_b: Pnt,
    _radius: f64,
) -> Face {
    let p01 = corner;
    let p02 = corner;
    
    let p10 = corner + (p_a - corner) * 0.33;
    let p20 = corner + (p_a - corner) * 0.66;
    
    let p31 = p_a + (p_b - p_a) * 0.33;
    let p32 = p_a + (p_b - p_a) * 0.66;
    
    let p13 = corner + (p_b - corner) * 0.33;
    let p23 = corner + (p_b - corner) * 0.66;
    
    let p11_u = p10 + (p31 - p10) * 0.5;
    let p11_v = p13 + (p32 - p13) * 0.5;
    let p21_u = p20 + (p31 - p20) * 0.5;
    let p21_v = p23 + (p32 - p23) * 0.5;
    
    let p12_u = p11_u;
    let p12_v = p11_v;
    let p22_u = p21_u;
    let p22_v = p21_v;
    
    let surf = GregorySurface::new(
        corner, p01, p02, corner,
        p10, p20, p_a, p31, p32, p_b,
        p13, p23,
        p11_u, p11_v,
        p21_u, p21_v,
        p12_u, p12_v,
        p22_u, p22_v,
    );
    
    let e1 = Edge::between_points(corner, p_a);
    let e2 = Edge::between_points(p_a, p_b);
    let e3 = Edge::between_points(p_b, corner);
    
    let wire = Wire::from_edges([e1, e2, e3]);
    Face::new(Some(GeomSurface::gregory(surf)), wire)
}

/// Apply a selected-edge rolling-ball fillet to a simple planar or curved solid.
pub fn fillet_planar_edge(
    solid: &Solid,
    edge: &Edge,
    radius: f64,
) -> Result<Solid, RollingBallError> {
    let blend = rolling_ball_fillet_edge(solid, edge, radius)?;
    let start = edge.source().point();
    let end = edge.target().point();

    let start_caps = endpoint_cap_faces(solid, start, &blend.face_a, &blend.face_b);
    let end_caps = endpoint_cap_faces(solid, end, &blend.face_a, &blend.face_b);

    let mut faces = Vec::new();
    let mut skipped_faces = std::collections::HashSet::new();

    let trimmed_start = if start_caps.len() == 1 {
        trim_face_at_corner(
            &start_caps[0],
            start,
            blend.contact_a.start().point(),
            blend.contact_b.start().point(),
            &blend.start_arc,
        )?
    } else {
        let patch = make_gregory_corner_patch(
            start,
            blend.contact_a.start().point(),
            blend.contact_b.start().point(),
            radius,
        );
        faces.push(patch);
        for cap in &start_caps {
            if let Ok(trimmed) = trim_face_at_corner(
                cap,
                start,
                blend.contact_a.start().point(),
                blend.contact_b.start().point(),
                &blend.start_arc,
            ) {
                faces.push(trimmed);
                skipped_faces.insert(cap.id());
            }
        }
        Face::new(None, Wire::from_edges([]))
    };

    let trimmed_end = if end_caps.len() == 1 {
        trim_face_at_corner(
            &end_caps[0],
            end,
            blend.contact_a.end().point(),
            blend.contact_b.end().point(),
            &blend.end_arc,
        )?
    } else {
        let patch = make_gregory_corner_patch(
            end,
            blend.contact_a.end().point(),
            blend.contact_b.end().point(),
            radius,
        );
        faces.push(patch);
        for cap in &end_caps {
            if let Ok(trimmed) = trim_face_at_corner(
                cap,
                end,
                blend.contact_a.end().point(),
                blend.contact_b.end().point(),
                &blend.end_arc,
            ) {
                faces.push(trimmed);
                skipped_faces.insert(cap.id());
            }
        }
        Face::new(None, Wire::from_edges([]))
    };

    let trimmed_a = trim_face_along_spine(&blend.face_a, &blend.spine, &blend.contact_a)?;
    let trimmed_b = trim_face_along_spine(&blend.face_b, &blend.spine, &blend.contact_b)?;

    for face in solid.shell().faces() {
        if same_face(&face, &blend.face_a)
            || same_face(&face, &blend.face_b)
            || (start_caps.len() == 1 && same_face(&face, &start_caps[0]))
            || (end_caps.len() == 1 && same_face(&face, &end_caps[0]))
            || skipped_faces.contains(&face.id())
        {
            continue;
        }
        faces.push(face);
    }
    
    faces.push(trimmed_a);
    faces.push(trimmed_b);
    if start_caps.len() == 1 {
        faces.push(trimmed_start);
    }
    if end_caps.len() == 1 {
        faces.push(trimmed_end);
    }
    faces.push(blend.blend_face);

    // Reject a blend that didn't close: a radius too large for the local geometry
    // (e.g. larger than half the part thickness) collapses trim edges and leaves a
    // non-watertight / degenerate shell. Surface that as an error rather than
    // returning a broken solid the application would cache.
    let result = Solid::new(sew(&faces, radius * 0.1));
    if !result.is_watertight() || !result.health_report().is_healthy() {
        return Err(RollingBallError::InvalidRadius { radius });
    }
    Ok(result)
}

/// Apply a constant-`radius` rolling-ball fillet to several selected `edges`.
///
/// Edges are filleted sequentially: after each blend the solid is rebuilt, so
/// the next selected edge is re-located in the evolving body by matching its
/// endpoint positions (within tolerance). Independent edges — and edges that
/// share a corner, where [`fillet_planar_edge`] inserts a corner cap or Gregory
/// patch — are supported; the order of `edges` does not need to be sorted.
///
/// Returns the first [`RollingBallError`] encountered, or
/// [`RollingBallError::SpineNotOnFace`] if a requested edge can no longer be
/// located after earlier blends consumed it.
pub fn fillet_edges(
    solid: &Solid,
    edges: &[Edge],
    radius: f64,
) -> Result<Solid, RollingBallError> {
    let mut current = solid.clone();
    for edge in edges {
        let target = current
            .edges()
            .into_iter()
            .find(|candidate| same_undirected_edge(candidate, edge))
            .ok_or(RollingBallError::SpineNotOnFace)?;
        current = fillet_planar_edge(&current, &target, radius)?;
    }
    Ok(current)
}

/// Find the two faces adjacent to `edge` in `solid`, solve the
/// rolling-ball contact curves, and build the blend face.
pub fn rolling_ball_fillet_edge(
    solid: &Solid,
    edge: &Edge,
    radius: f64,
) -> Result<RollingBallBlend, RollingBallError> {
    let adjacent = adjacent_faces(solid, edge);
    if adjacent.len() != 2 {
        return Err(RollingBallError::EdgeAdjacency {
            count: adjacent.len(),
        });
    }

    // Both adjacent faces planar → analytic bisector solve. Any curved face
    // present → the curved solver (analytic plane/cylinder paths, else numeric).
    let both_planar = matches!(adjacent[0].surface(), Some(GeomSurface::Plane(_)))
        && matches!(adjacent[1].surface(), Some(GeomSurface::Plane(_)));

    if both_planar {
        // `sew` canonicalizes every sewn shell so each planar face's stored normal
        // agrees with its winding and the shell faces outward, so the face's stored
        // orientation is trustworthy here (no solid-interior re-derivation needed).
        let n_a = planar_outward_normal(&adjacent[0])?;
        let n_b = planar_outward_normal(&adjacent[1])?;
        planar_blend(edge, &adjacent[0], &adjacent[1], n_a, n_b, radius)
    } else {
        rolling_ball_between_curved_faces(edge, &adjacent[0], &adjacent[1], radius)
    }
}

/// Solve a rolling-ball fillet between a plane and a cylinder.
///
/// Two analytic configurations are handled:
/// - **Cap fillet** — a circular edge with the plane normal parallel to the
///   axis; the ball rolls around the rim and the blend is a torus.
/// - **Longitudinal fillet** — the plane normal is perpendicular to the axis,
///   so the shared edge is a straight generator of the wall; the blend is a
///   cylinder with straight-line contacts (see [`rolling_ball_plane_perp_cylinder`]).
pub fn rolling_ball_between_curved_faces(
    edge: &Edge,
    face_a: &Face,
    face_b: &Face,
    radius: f64,
) -> Result<RollingBallBlend, RollingBallError> {
    if !radius.is_finite() || radius <= tolerance::CONFUSION {
        return Err(RollingBallError::InvalidRadius { radius });
    }

    let (plane_face, cyl_face, is_a_plane) = if matches!(face_a.surface(), Some(GeomSurface::Plane(_))) {
        (face_a, face_b, true)
    } else if matches!(face_b.surface(), Some(GeomSurface::Plane(_))) {
        (face_b, face_a, false)
    } else {
        return Err(RollingBallError::UnsolvableAdjacency {
            reason: AdjacencyReason::UnsupportedSurfacePair,
        });
    };

    let _plane = match plane_face.surface() {
        Some(GeomSurface::Plane(p)) => p,
        _ => {
            return Err(RollingBallError::UnsolvableAdjacency {
                reason: AdjacencyReason::NotPlaneOrAnalytic,
            })
        }
    };
    let cyl = match cyl_face.surface() {
        Some(GeomSurface::Cylinder(c)) => c,
        _ => {
            return Err(RollingBallError::UnsolvableAdjacency {
                reason: AdjacencyReason::NotPlaneOrAnalytic,
            })
        }
    };

    let axis_dir = cyl.position().direction();
    let p0 = edge.source().point();
    let n_plane = face_outward_normal_at(plane_face, p0)?;

    // Cap fillet: a circular edge with the plane normal parallel to the axis.
    // The ball rolls around the rim, producing a toroidal blend surface.
    if let Some(GeomCurve::Circle(circle)) = edge.curve() {
        let center = circle.center();
        let spine_r = circle.radius();

        if n_plane.is_parallel(&axis_dir, 1e-4) {
            let major_radius = spine_r - radius;
            if major_radius <= tolerance::CONFUSION {
                return Err(RollingBallError::InvalidRadius { radius });
            }

            let torus_center = center - GeomVec::from_dir(n_plane) * radius;
            let pos = Ax3::new_axes(torus_center, axis_dir, cyl.position().x_direction());
            let torus_surf = GeomSurface::torus(ToroidalSurface::new(pos, major_radius, radius));

            let contact_plane_r = major_radius;
            let contact_cyl_height = radius;

            let c_plane = Circle::new(Ax3::new_axes(center, axis_dir, cyl.position().x_direction()), contact_plane_r);
            let c_cyl = Circle::new(Ax3::new_axes(center - GeomVec::from_dir(n_plane) * contact_cyl_height, axis_dir, cyl.position().x_direction()), spine_r);

            let u0 = edge.first();
            let u1 = edge.last();

            let contact_plane_edge = Edge::new(
                Some(GeomCurve::circle(c_plane)),
                u0,
                u1,
                Vertex::new(c_plane.point(u0)),
                Vertex::new(c_plane.point(u1)),
            );

            let contact_cyl_edge = Edge::new(
                Some(GeomCurve::circle(c_cyl)),
                u0,
                u1,
                Vertex::new(c_cyl.point(u0)),
                Vertex::new(c_cyl.point(u1)),
            );

            let (contact_a, contact_b) = if is_a_plane {
                (contact_plane_edge, contact_cyl_edge)
            } else {
                (contact_cyl_edge, contact_plane_edge)
            };

            let c0 = torus_center + radial(axis_dir, cyl.position().x_direction(), u0) * major_radius;
            let c1 = torus_center + radial(axis_dir, cyl.position().x_direction(), u1) * major_radius;
            let centerline = Edge::between_points(c0, c1);

            let r0 = radial(axis_dir, cyl.position().x_direction(), u0);
            let dir0 = Dir::new(r0.x(), r0.y(), r0.z());
            let r1 = radial(axis_dir, cyl.position().x_direction(), u1);
            let dir1 = Dir::new(r1.x(), r1.y(), r1.z());

            let t0_vec = GeomVec::from_dir(axis_dir).cross(&r0).normalized().unwrap();
            let t0 = Dir::new(t0_vec.x(), t0_vec.y(), t0_vec.z());
            let t1_vec = GeomVec::from_dir(axis_dir).cross(&r1).normalized().unwrap();
            let t1 = Dir::new(t1_vec.x(), t1_vec.y(), t1_vec.z());

            let arc_start = contact_arc(c0, t0, dir0, radius, contact_b.start().point(), contact_a.start().point())?;
            let arc_end = contact_arc(c1, t1, dir1, radius, contact_a.end().point(), contact_b.end().point())?;

            let wire = Wire::from_edges([
                contact_a.clone(),
                arc_end.clone(),
                contact_b.clone().reversed(),
                arc_start.clone(),
            ]);
            let blend_face = Face::new(Some(torus_surf), wire);

            return Ok(RollingBallBlend {
                spine: edge.clone(),
                face_a: face_a.clone(),
                face_b: face_b.clone(),
                contact_a,
                contact_b,
                centerline,
                blend_face,
                start_arc: arc_start,
                end_arc: arc_end,
                radius,
            });
        }
    }

    // Longitudinal fillet: the plane normal is perpendicular to the axis, so the
    // shared edge is a straight generator of the cylinder wall. Convex
    // closed-form solve → a cylindrical blend face with straight-line contacts.
    if !n_plane.is_parallel(&axis_dir, 1e-4) {
        return rolling_ball_plane_perp_cylinder(edge, plane_face, cyl_face, is_a_plane, radius);
    }

    Err(RollingBallError::UnsolvableAdjacency {
        reason: AdjacencyReason::UnsupportedSurfacePair,
    })
}

/// Solve a rolling-ball fillet for a straight generator edge between a plane
/// (whose normal is perpendicular to the cylinder axis) and a cylinder wall.
///
/// The plane is parallel to the cylinder axis, so the shared edge is a straight
/// generator line. The cross-section (perpendicular to the axis) reduces to a
/// 2D problem of a ball tangent to a line (the plane) and a circle (the
/// cylinder wall); solving it in closed form gives straight-line contacts on
/// both faces and a cylindrical blend surface of radius `radius`.
fn rolling_ball_plane_perp_cylinder(
    edge: &Edge,
    plane_face: &Face,
    cyl_face: &Face,
    is_a_plane: bool,
    radius: f64,
) -> Result<RollingBallBlend, RollingBallError> {
    let cyl = match cyl_face.surface() {
        Some(GeomSurface::Cylinder(c)) => c,
        _ => {
            return Err(RollingBallError::UnsolvableAdjacency {
                reason: AdjacencyReason::NotPlaneOrAnalytic,
            })
        }
    };
    let axis = cyl.position().direction();
    let axis_vec = GeomVec::from_dir(axis);
    let axis_loc = cyl.position().location();
    let r_cyl = cyl.radius();

    let p0 = edge.source().point();
    let p1 = edge.target().point();
    let n_plane = face_outward_normal_at(plane_face, p0)?;
    let n_vec = GeomVec::from_dir(n_plane);

    // The plane must contain the axis → its normal is perpendicular to the axis.
    if n_plane.is_parallel(&axis, 1e-4) {
        return Err(RollingBallError::UnsolvableAdjacency {
            reason: AdjacencyReason::UnsupportedSurfacePair,
        });
    }

    // Radial direction at the edge + axial parameter of p0.
    let z0 = (p0 - axis_loc).dot(&axis_vec);
    let o_perp = axis_loc + axis_vec * z0;
    let radial_vec = p0 - o_perp;
    let r_hat = GeomVec::from_dir(
        radial_vec
            .normalized()
            .ok_or(RollingBallError::DegenerateSpine)?,
    );
    // The edge must actually lie on the cylinder wall.
    if (radial_vec.magnitude() - r_cyl).abs() > 1e-6 * r_cyl.max(1.0) + 1e-6 {
        return Err(RollingBallError::UnsolvableAdjacency {
            reason: AdjacencyReason::UnsupportedSurfacePair,
        });
    }

    // Convex cross-section solve. `c = cos(dihedral)` between the cylinder
    // radial and the plane normal at the edge; reduces to the classic
    // `α = √(R_c·(R_c + 2r)) − R_c` offset when the meeting is square (c = 0).
    let c = r_hat.dot(&n_vec);
    if c <= -1.0 + 1e-6 {
        return Err(RollingBallError::InvalidDihedral);
    }
    let a2 = r_cyl * r_cyl + 2.0 * radius * r_cyl / (1.0 + c);
    if a2 <= 0.0 {
        return Err(RollingBallError::UnsolvableAdjacency {
            reason: AdjacencyReason::RadiusTooLarge,
        });
    }
    let a = a2.sqrt();
    let b = radius + c * (r_cyl - a);

    // Cross-section offsets relative to the axis point `o_perp`:
    //   ball center, plane contact (foot of perpendicular), cylinder contact.
    let center_off = r_hat * a + n_vec * b;
    let plane_off = r_hat * a + n_vec * (c * (r_cyl - a));
    let cyl_off = (r_hat * a + n_vec * b) * (r_cyl / (r_cyl + radius));

    let z1 = (p1 - axis_loc).dot(&axis_vec);
    let at = |off: GeomVec, z: f64| o_perp + off + axis_vec * z;

    let c0 = at(center_off, z0);
    let c1 = at(center_off, z1);
    let centerline = Edge::between_points(c0, c1);

    let contact_plane_edge = Edge::between_points(at(plane_off, z0), at(plane_off, z1));
    let contact_cyl_edge = Edge::between_points(at(cyl_off, z0), at(cyl_off, z1));
    let (contact_a, contact_b) = if is_a_plane {
        (contact_plane_edge, contact_cyl_edge)
    } else {
        (contact_cyl_edge, contact_plane_edge)
    };

    // Blend surface: a cylinder of radius `radius` swept along the centerline.
    let blend_surf = GeomSurface::cylinder(CylindricalSurface::new(
        Ax3::new_axes(c0, axis, n_plane),
        radius,
    ));

    // End arcs close the blend at each endpoint, swinging in the cross-section
    // plane (normal = axis) around the centerline endpoints.
    let arc_start = contact_arc(
        c0,
        axis,
        n_plane,
        radius,
        contact_b.start().point(),
        contact_a.start().point(),
    )?;
    let arc_end = contact_arc(
        c1,
        axis,
        n_plane,
        radius,
        contact_a.end().point(),
        contact_b.end().point(),
    )?;

    let wire = Wire::from_edges([
        contact_a.clone(),
        arc_end.clone(),
        contact_b.clone().reversed(),
        arc_start.clone(),
    ]);
    let blend_face = Face::new(Some(blend_surf), wire);

    // Reconstruct the original (face_a, face_b) ordering from the plane/cylinder
    // pair and the `is_a_plane` flag.
    let (face_a_clone, face_b_clone) = if is_a_plane {
        (plane_face.clone(), cyl_face.clone())
    } else {
        (cyl_face.clone(), plane_face.clone())
    };

    Ok(RollingBallBlend {
        spine: edge.clone(),
        face_a: face_a_clone,
        face_b: face_b_clone,
        contact_a,
        contact_b,
        centerline,
        blend_face,
        start_arc: arc_start,
        end_arc: arc_end,
        radius,
    })
}

/// Solve a rolling-ball fillet for `edge` between two known adjacent planar
/// faces.
pub fn rolling_ball_between_planar_faces(
    edge: &Edge,
    face_a: &Face,
    face_b: &Face,
    radius: f64,
) -> Result<RollingBallBlend, RollingBallError> {
    let n_a = planar_outward_normal(face_a)?;
    let n_b = planar_outward_normal(face_b)?;
    planar_blend(edge, face_a, face_b, n_a, n_b, radius)
}

/// Core rolling-ball planar blend with explicit, already-outward face normals.
/// Factored out of [`rolling_ball_between_planar_faces`] so
/// [`rolling_ball_fillet_edge`] can substitute solid-aware outward normals for a
/// face whose *stored* orientation is inward (a sewn prism cap), which
/// `planar_outward_normal` alone would misread.
fn planar_blend(
    edge: &Edge,
    face_a: &Face,
    face_b: &Face,
    n_a: Dir,
    n_b: Dir,
    radius: f64,
) -> Result<RollingBallBlend, RollingBallError> {
    if !radius.is_finite() || radius <= tolerance::CONFUSION {
        return Err(RollingBallError::InvalidRadius { radius });
    }

    let p0 = edge.source().point();
    let p1 = edge.target().point();
    let spine_vec = p1 - p0;
    let spine_dir = spine_vec
        .normalized()
        .ok_or(RollingBallError::DegenerateSpine)?;

    let inward_a = -GeomVec::from_dir(n_a);
    let inward_b = -GeomVec::from_dir(n_b);
    let bisector = inward_a + inward_b;
    let bisector_dir = bisector
        .normalized()
        .ok_or(RollingBallError::InvalidDihedral)?;
    let sin_half =
        inward_a.cross(&inward_b).magnitude() / bisector.magnitude().max(tolerance::CONFUSION);
    if sin_half <= tolerance::CONFUSION {
        return Err(RollingBallError::InvalidDihedral);
    }

    let center_offset = GeomVec::from_dir(bisector_dir) * (radius / sin_half);
    let c0 = p0 + center_offset;
    let c1 = p1 + center_offset;

    let contact_offset_a = center_offset + GeomVec::from_dir(n_a) * radius;
    let contact_offset_b = center_offset + GeomVec::from_dir(n_b) * radius;
    let a0 = p0 + contact_offset_a;
    let a1 = p1 + contact_offset_a;
    let b0 = p0 + contact_offset_b;
    let b1 = p1 + contact_offset_b;

    let contact_a = Edge::between_points(a0, a1);
    let contact_b = Edge::between_points(b0, b1);
    let centerline = Edge::between_points(c0, c1);
    let arc_start = contact_arc(c0, spine_dir, n_b, radius, b0, a0)?;
    let arc_end = contact_arc(c1, spine_dir, n_a, radius, a1, b1)?;

    let wire = Wire::from_edges([
        contact_a.clone(),
        arc_end.clone(),
        contact_b.clone().reversed(),
        arc_start.clone(),
    ]);
    let surface = GeomSurface::cylinder(CylindricalSurface::new(
        Ax3::new_axes(c0, spine_dir, n_a),
        radius,
    ));
    let blend_face = Face::new(Some(surface), wire);

    Ok(RollingBallBlend {
        spine: edge.clone(),
        face_a: face_a.clone(),
        face_b: face_b.clone(),
        contact_a,
        contact_b,
        centerline,
        blend_face,
        start_arc: arc_start,
        end_arc: arc_end,
        radius,
    })
}

fn adjacent_faces(solid: &Solid, edge: &Edge) -> Vec<Face> {
    solid
        .shell()
        .faces()
        .into_iter()
        .filter(|face| {
            face.wires()
                .into_iter()
                .flat_map(|wire| wire.edges())
                .any(|candidate| same_undirected_edge(&candidate, edge))
        })
        .collect()
}

fn same_undirected_edge(a: &Edge, b: &Edge) -> bool {
    let a0 = a.start().point();
    let a1 = a.end().point();
    let b0 = b.start().point();
    let b1 = b.end().point();
    let tol = 10.0 * tolerance::CONFUSION;
    (a0.distance(&b0) <= tol && a1.distance(&b1) <= tol)
        || (a0.distance(&b1) <= tol && a1.distance(&b0) <= tol)
}

fn same_face(a: &Face, b: &Face) -> bool {
    a.id() == b.id()
}

fn endpoint_cap_faces(solid: &Solid, point: Pnt, face_a: &Face, face_b: &Face) -> Vec<Face> {
    solid
        .shell()
        .faces()
        .into_iter()
        .filter(|face| {
            !same_face(face, face_a) && !same_face(face, face_b) && face_contains_point(face, point)
        })
        .collect()
}

fn face_contains_point(face: &Face, point: Pnt) -> bool {
    face.wires().into_iter().any(|wire| {
        wire.edges().into_iter().any(|edge| {
            edge.source().point().distance(&point) <= 10.0 * tolerance::CONFUSION
                || edge.target().point().distance(&point) <= 10.0 * tolerance::CONFUSION
        })
    })
}

fn trim_face_along_spine(
    face: &Face,
    spine: &Edge,
    contact: &Edge,
) -> Result<Face, RollingBallError> {
    ensure_simple_planar_outer_loop(face)?;
    let edges = face
        .outer_wire()
        .ok_or(RollingBallError::UnsupportedTrimTopology)?
        .edges();
    let n = edges.len();
    if n < 3 {
        return Err(RollingBallError::UnsupportedTrimTopology);
    }
    let Some(idx) = edges
        .iter()
        .position(|candidate| same_undirected_edge(candidate, spine))
    else {
        return Err(RollingBallError::SpineNotOnFace);
    };

    let prev_idx = (idx + n - 1) % n;
    let next_idx = (idx + 1) % n;
    let selected = &edges[idx];
    let contact_start = contact_point_for_spine_vertex(spine, contact, selected.source().point());
    let contact_end = contact_point_for_spine_vertex(spine, contact, selected.target().point());

    // --- BLEND OVERFLOW HANDLING / CLAMPING ---
    let mut contact_start_clamped = contact_start;
    let mut contact_end_clamped = contact_end;

    let prev = &edges[prev_idx];
    let next = &edges[next_idx];

    let len_prev = prev.source().point().distance(&prev.target().point());
    let dist_start_to_prev_start = contact_start.distance(&prev.source().point());
    if dist_start_to_prev_start > len_prev * 0.99 {
        contact_start_clamped = prev.source().point();
    }

    let len_next = next.source().point().distance(&next.target().point());
    let dist_end_to_next_end = contact_end.distance(&next.target().point());
    if dist_end_to_next_end > len_next * 0.99 {
        contact_end_clamped = next.target().point();
    }

    let oriented_contact = orient_edge_between(contact, contact_start_clamped, contact_end_clamped);

    let mut new_edges = Vec::with_capacity(n);
    for (i, edge) in edges.iter().enumerate() {
        if i == prev_idx {
            new_edges.push(Edge::between_points(edge.source().point(), contact_start_clamped));
        } else if i == idx {
            new_edges.push(oriented_contact.clone());
        } else if i == next_idx {
            new_edges.push(Edge::between_points(contact_end_clamped, edge.target().point()));
        } else {
            new_edges.push(edge.clone());
        }
    }

    rebuild_face(face, Wire::from_edges(new_edges))
}

fn trim_face_at_corner(
    face: &Face,
    corner: Pnt,
    contact_a: Pnt,
    contact_b: Pnt,
    arc: &Edge,
) -> Result<Face, RollingBallError> {
    ensure_simple_planar_outer_loop(face)?;
    let edges = face
        .outer_wire()
        .ok_or(RollingBallError::UnsupportedTrimTopology)?
        .edges();
    let n = edges.len();
    if n < 3 {
        return Err(RollingBallError::UnsupportedTrimTopology);
    }

    let Some(next_idx) = edges
        .iter()
        .position(|edge| edge.source().point().distance(&corner) <= 10.0 * tolerance::CONFUSION)
    else {
        return Err(RollingBallError::UnsupportedTrimTopology);
    };
    let prev_idx = (next_idx + n - 1) % n;
    if edges[prev_idx].target().point().distance(&corner) > 10.0 * tolerance::CONFUSION {
        return Err(RollingBallError::UnsupportedTrimTopology);
    }

    let prev = &edges[prev_idx];
    let next = &edges[next_idx];
    let da_prev = point_segment_distance(contact_a, prev.source().point(), prev.target().point());
    let db_prev = point_segment_distance(contact_b, prev.source().point(), prev.target().point());
    let (prev_contact, next_contact) = if da_prev <= db_prev {
        (contact_a, contact_b)
    } else {
        (contact_b, contact_a)
    };
    let oriented_arc = orient_edge_between(arc, prev_contact, next_contact);

    let mut new_edges = Vec::with_capacity(n + 1);
    for offset in 0..n {
        let i = (prev_idx + offset) % n;
        if i == prev_idx {
            new_edges.push(Edge::between_points(prev.source().point(), prev_contact));
            new_edges.push(oriented_arc.clone());
        } else if i == next_idx {
            new_edges.push(Edge::between_points(next_contact, next.target().point()));
        } else {
            new_edges.push(edges[i].clone());
        }
    }

    rebuild_face(face, Wire::from_edges(new_edges))
}

fn ensure_simple_planar_outer_loop(face: &Face) -> Result<(), RollingBallError> {
    if !matches!(face.surface(), Some(GeomSurface::Plane(_)) | Some(GeomSurface::Cylinder(_))) {
        return Err(RollingBallError::UnsolvableAdjacency {
            reason: AdjacencyReason::NotPlaneOrAnalytic,
        });
    }
    if face.outer_wire().is_none() {
        return Err(RollingBallError::UnsupportedTrimTopology);
    }
    if !face.inner_wires().is_empty() {
        return Err(RollingBallError::UnsupportedTrimTopology);
    }
    Ok(())
}

fn rebuild_face(face: &Face, outer: Wire) -> Result<Face, RollingBallError> {
    Ok(Face::with_wires(
        face.surface().cloned(),
        Some(outer),
        Vec::new(),
        face.orientation(),
    ))
}

fn contact_point_for_spine_vertex(spine: &Edge, contact: &Edge, point: Pnt) -> Pnt {
    let spine_start = spine.source().point();
    if point.distance(&spine_start) <= 10.0 * tolerance::CONFUSION {
        contact.start().point()
    } else {
        contact.end().point()
    }
}

fn orient_edge_between(edge: &Edge, start: Pnt, end: Pnt) -> Edge {
    let tol = 10.0 * tolerance::CONFUSION;
    if edge.start().point().distance(&start) <= tol && edge.end().point().distance(&end) <= tol {
        edge.clone()
    } else {
        edge.reversed()
    }
}

fn point_segment_distance(point: Pnt, a: Pnt, b: Pnt) -> f64 {
    let ab = b - a;
    let len2 = ab.magnitude_squared();
    if len2 <= tolerance::CONFUSION * tolerance::CONFUSION {
        return point.distance(&a);
    }
    let t = ((point - a).dot(&ab) / len2).clamp(0.0, 1.0);
    point.distance(&(a + ab * t))
}

fn planar_outward_normal(face: &Face) -> Result<Dir, RollingBallError> {
    let normal = match face.surface() {
        Some(GeomSurface::Plane(plane)) => plane.normal(),
        _ => {
            return Err(RollingBallError::UnsolvableAdjacency {
                reason: AdjacencyReason::NotPlaneOrAnalytic,
            })
        }
    };
    Ok(if face.orientation() == Orientation::Reversed {
        normal.reversed()
    } else {
        normal
    })
}

fn contact_arc(
    center: Pnt,
    mut axis: Dir,
    xdir: Dir,
    radius: f64,
    start: Pnt,
    end: Pnt,
) -> Result<Edge, RollingBallError> {
    let start_v = (start - center)
        .normalized()
        .ok_or(RollingBallError::InvalidDihedral)?;
    let end_v = (end - center)
        .normalized()
        .ok_or(RollingBallError::InvalidDihedral)?;
    let mut angle = angle_about_axis(start_v, end_v, axis);
    if angle <= tolerance::CONFUSION {
        axis = axis.reversed();
        angle = angle_about_axis(start_v, end_v, axis);
    }
    if angle <= tolerance::CONFUSION {
        return Err(RollingBallError::InvalidDihedral);
    }

    let circle = Circle::new(Ax3::new_axes(center, axis, xdir), radius);
    Ok(Edge::new(
        Some(GeomCurve::circle(circle)),
        0.0,
        angle,
        Vertex::new(start),
        Vertex::new(end),
    ))
}

fn angle_about_axis(start: Dir, end: Dir, axis: Dir) -> f64 {
    let x = GeomVec::from_dir(start);
    let y = GeomVec::from_dir(axis).cross(&x);
    let end_v = GeomVec::from_dir(end);
    let mut angle = end_v.dot(&y).atan2(end_v.dot(&x));
    if angle < 0.0 {
        angle += core::f64::consts::TAU;
    }
    angle
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::Pnt;
    use openrcad_geom::{Plane, Surface};
    use openrcad_primitives::make_box;

    #[test]
    fn solves_contacts_for_box_edge() {
        let solid = make_box(&Pnt::origin(), 4.0, 5.0, 6.0);
        let edge = origin_vertical_edge(&solid);

        let blend = rolling_ball_fillet_edge(&solid, &edge, 0.5).unwrap();
        assert!((blend.radius - 0.5).abs() < 1e-12);
        assert!(matches!(
            blend.blend_face.surface(),
            Some(GeomSurface::Cylinder(_))
        ));

        let c0 = blend.centerline.start().point();
        assert!((c0.x() - 0.5).abs() < 1e-9);
        assert!((c0.y() - 0.5).abs() < 1e-9);

        let a0 = blend.contact_a.start().point();
        let b0 = blend.contact_b.start().point();
        let on_x_face = a0.x().abs() < 1e-9 || b0.x().abs() < 1e-9;
        let on_y_face = a0.y().abs() < 1e-9 || b0.y().abs() < 1e-9;
        assert!(on_x_face, "one contact curve lies on the X-min face");
        assert!(on_y_face, "one contact curve lies on the Y-min face");
        assert!(blend.blend_face.outer_wire().unwrap().is_closed());
    }

    #[test]
    fn fillets_single_planar_box_edge_into_watertight_solid() {
        let solid = make_box(&Pnt::origin(), 4.0, 5.0, 6.0);
        let edge = origin_vertical_edge(&solid);

        let filleted = fillet_planar_edge(&solid, &edge, 0.5).unwrap();

        assert_eq!(filleted.face_count(), 7);
        assert!(filleted.health_report().is_healthy());
        assert!(filleted.is_watertight());
        let cylinders = filleted
            .shell()
            .faces()
            .iter()
            .filter(|face| matches!(face.surface(), Some(GeomSurface::Cylinder(_))))
            .count();
        assert_eq!(cylinders, 1);
    }

    #[test]
    fn error_variant_set_is_renamed() {
        // Compile-time confirmation that the obsolete variants (`NonPlanarAdjacentFace`,
        // `CornerAdjacency`) are gone and the new vocabulary exists.
        let err = RollingBallError::UnsolvableAdjacency {
            reason: AdjacencyReason::UnsupportedSurfacePair,
        };
        assert!(format!("{err}").contains("could not be blended"));
        let diverged = RollingBallError::NewtonDiverged { iterations: 8 };
        assert!(format!("{diverged}").contains("diverged"));
        let blend = RollingBallError::BlendSurfaceBuild("sweep");
        assert!(format!("{blend}").contains("sweep"));

        // Exhaustiveness guard: every current variant is named, so a future
        // rename/addition cannot silently drop a branch.
        fn _exhaustive(e: &RollingBallError) -> &'static str {
            match e {
                RollingBallError::InvalidRadius { .. } => "r",
                RollingBallError::DegenerateSpine => "d",
                RollingBallError::EdgeAdjacency { .. } => "e",
                RollingBallError::UnsolvableAdjacency { .. } => "u",
                RollingBallError::InvalidDihedral => "i",
                RollingBallError::SpineNotOnFace => "s",
                RollingBallError::UnsupportedTrimTopology => "t",
                RollingBallError::NewtonDiverged { .. } => "n",
                RollingBallError::BlendSurfaceBuild(_) => "b",
            }
        }
    }

    #[test]
    fn rejects_boundary_edge_without_two_faces() {
        let face = Face::new(
            Some(GeomSurface::plane(Plane::from_point_normal(
                Pnt::origin(),
                Dir::dz(),
            ))),
            Wire::from_edges([
                Edge::between_points(Pnt::origin(), Pnt::new(1.0, 0.0, 0.0)),
                Edge::between_points(Pnt::new(1.0, 0.0, 0.0), Pnt::new(0.0, 1.0, 0.0)),
                Edge::between_points(Pnt::new(0.0, 1.0, 0.0), Pnt::origin()),
            ]),
        );
        let solid = Solid::new(openrcad_topo::Shell::from_faces([face.clone()]));
        let edge = face.outer_wire().unwrap().edges()[0].clone();

        assert!(matches!(
            rolling_ball_fillet_edge(&solid, &edge, 0.1),
            Err(RollingBallError::EdgeAdjacency { count: 1 })
        ));
    }

    #[test]
    fn solves_curved_face_blending_cylinder() {
        use openrcad_foundation::Ax2;
        use openrcad_primitives::make_cylinder;

        let solid = make_cylinder(&Ax2::new(Pnt::origin(), Dir::dz()), 2.0, 6.0);
        let edge = solid
            .edges()
            .into_iter()
            .find(|e| {
                e.start().point().z().abs() < 1e-9 && matches!(e.curve(), Some(GeomCurve::Circle(_)))
            })
            .unwrap();

        let filleted = fillet_planar_edge(&solid, &edge, 0.2).unwrap();
        assert!(filleted.is_watertight());
        let tori = filleted
            .shell()
            .faces()
            .iter()
            .filter(|f| matches!(f.surface(), Some(GeomSurface::Torus(_))))
            .count();
        assert!(tori > 0);
    }

    // Goal-test for the longitudinal (plane⊥axis) solver. Currently blocked on
    // the boolean engine: the cylinder-flat `Cut` below is a partial-imprint
    // case (the cut plane only partially crosses the cylinder wall), which the
    // split pass does not yet imprint into a clean D-shape — so the +Y
    // generator edge the solver needs never appears. See the boolean-frontier
    // `#[ignore]`d tests in tests/robustness.rs. Run with `cargo test --ignored`.
    #[test]
    #[ignore = "blocked on boolean cylinder-flat (longitudinal partial-imprint) robustness"]
    fn solves_longitudinal_plane_cylinder_fillet() {
        use crate::{boolean, BooleanOp};
        use openrcad_foundation::Ax2;
        use openrcad_primitives::make_cylinder;

        // Cylinder (R=2, H=6, axis Z) shaved flat at x=0.5. The plane (x=0.5,
        // normal ⊥ axis) meets the cylinder wall along two straight generator
        // edges — the longitudinal (plane⊥axis) config.
        let cyl = make_cylinder(&Ax2::new(Pnt::origin(), Dir::dz()), 2.0, 6.0);
        let cutter = make_box(&Pnt::new(0.5, -3.0, -1.0), 10.0, 6.0, 8.0);
        let dshape = boolean(&cyl, &cutter, BooleanOp::Cut);

        // The +Y generator: a straight Z-line at (0.5, √(4-0.25), z).
        let y_gen = (4.0_f64 - 0.5 * 0.5).sqrt();
        let edge = dshape
            .edges()
            .into_iter()
            .find(|e| {
                let p0 = e.start().point();
                let p1 = e.end().point();
                (p0.x() - 0.5).abs() < 1e-6
                    && (p1.x() - 0.5).abs() < 1e-6
                    && (p0.y() - y_gen).abs() < 1e-4
                    && (p1.y() - y_gen).abs() < 1e-4
                    && (p0.z() - p1.z()).abs() > 5.0
            })
            .expect("D-shape has a +Y generator edge");

        let r = 0.3_f64;
        let blend = rolling_ball_fillet_edge(&dshape, &edge, r).expect("longitudinal solve");

        // Blend surface is a cylinder of the fillet radius.
        let surf_r = match blend.blend_face.surface() {
            Some(GeomSurface::Cylinder(c)) => c.radius(),
            other => panic!("expected cylindrical blend, got {other:?}"),
        };
        assert!((surf_r - r).abs() < 1e-9, "blend cylinder radius {surf_r}");

        // One contact lies on the plane (x=0.5), the other on the wall
        // (distance √(x²+y²) = 2 from the Z-axis).
        let a0 = blend.contact_a.start().point();
        let b0 = blend.contact_b.start().point();
        let on_plane = |p: Pnt| (p.x() - 0.5).abs() < 1e-6;
        let on_cyl = |p: Pnt| ((p.x() * p.x() + p.y() * p.y()).sqrt() - 2.0).abs() < 1e-6;
        assert!(
            (on_plane(a0) && on_cyl(b0)) || (on_cyl(a0) && on_plane(b0)),
            "contacts must lie on the plane and the cylinder: a0={a0:?} b0={b0:?}"
        );

        // Full surgical fillet must remain watertight.
        let filleted = fillet_planar_edge(&dshape, &edge, r).expect("longitudinal fillet");
        assert!(
            filleted.is_watertight(),
            "longitudinal fillet not watertight: {:?}",
            filleted.manifold_report()
        );
    }

    #[test]
    fn solves_gregory_corner_patch() {
        let corner = Pnt::origin();
        let p_a = Pnt::new(1.0, 0.0, 0.0);
        let p_b = Pnt::new(0.0, 1.0, 0.0);
        let patch = make_gregory_corner_patch(corner, p_a, p_b, 0.1);
        assert!(patch.surface().is_some());
        if let Some(GeomSurface::Gregory(surf)) = patch.surface() {
            assert_eq!(surf.point(0.0, 0.0), corner);
            assert_eq!(surf.point(1.0, 0.0), p_a);
            assert_eq!(surf.point(1.0, 1.0), p_b);
        } else {
            panic!("Expected GregorySurface");
        }
    }

    #[test]
    fn handles_blend_overflow_clamping() {
        let solid = make_box(&Pnt::origin(), 1.0, 1.0, 1.0);
        let edge = solid
            .edges()
            .into_iter()
            .find(|edge| {
                let p0 = edge.start().point();
                let p1 = edge.end().point();
                p0.x().abs() < 1e-9
                    && p1.x().abs() < 1e-9
                    && p0.y().abs() < 1e-9
                    && p1.y().abs() < 1e-9
                    && (p0.z() - p1.z()).abs() > 0.9
            })
            .unwrap();

        let filleted = fillet_planar_edge(&solid, &edge, 1.5);
        assert!(filleted.is_ok());
        assert!(filleted.unwrap().is_watertight());
    }

    #[test]
    fn fillet_edges_blends_two_independent_box_edges() {
        // Two diagonally-opposite vertical edges of the box do not share a face
        // corner, so the multi-edge API blends them independently and the result
        // stays watertight with one cylindrical blend face per edge.
        let solid = make_box(&Pnt::origin(), 4.0, 5.0, 6.0);
        let verticals: Vec<Edge> = solid
            .edges()
            .into_iter()
            .filter(|edge| {
                let p0 = edge.start().point();
                let p1 = edge.end().point();
                (p0.x() - p1.x()).abs() < 1e-9
                    && (p0.y() - p1.y()).abs() < 1e-9
                    && (p0.z() - p1.z()).abs() > 5.9
            })
            .collect();
        let pick = |x: f64, y: f64| -> Edge {
            verticals
                .iter()
                .find(|e| {
                    let p = e.start().point();
                    (p.x() - x).abs() < 1e-9 && (p.y() - y).abs() < 1e-9
                })
                .expect("vertical edge at corner")
                .clone()
        };
        let edges = [pick(0.0, 0.0), pick(4.0, 5.0)];

        let filleted = fillet_edges(&solid, &edges, 0.4).expect("multi-edge fillet");
        assert!(filleted.is_watertight(), "{:?}", filleted.manifold_report());
        let cylinders = filleted
            .shell()
            .faces()
            .iter()
            .filter(|f| matches!(f.surface(), Some(GeomSurface::Cylinder(_))))
            .count();
        assert_eq!(cylinders, 2, "one cylindrical blend per filleted edge");
    }

    fn origin_vertical_edge(solid: &Solid) -> Edge {
        solid
            .edges()
            .into_iter()
            .find(|edge| {
                let p0 = edge.start().point();
                let p1 = edge.end().point();
                p0.x().abs() < 1e-9
                    && p1.x().abs() < 1e-9
                    && p0.y().abs() < 1e-9
                    && p1.y().abs() < 1e-9
                    && (p0.z() - p1.z()).abs() > 5.9
            })
            .expect("box has a vertical origin edge")
    }
}
