//! Rolling-ball edge blend construction.
//!
//! This module is the first local-blend slice: it solves the contact curves for
//! a constant-radius ball rolling along a selected edge shared by two planar
//! faces, then builds the cylindrical blend face bounded by those contacts.

use core::fmt;

use openrcad_foundation::{tolerance, Ax2, Ax3, Dir, Pnt, Vec as GeomVec};
use openrcad_geom::{
    Circle, Curve, CylindricalSurface, Ellipse, GeomCurve, GeomSurface, GregorySurface, Plane,
    SphericalSurface, Surface, ToroidalSurface,
};
use openrcad_mesh::tessellate;
use openrcad_primitives::make_cylinder;
use openrcad_topo::{Edge, Face, FaceId, Orientation, Solid, Vertex, Wire};

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
    /// The local edit built faces, but the sewn shell was not watertight/healthy.
    InvalidTopology,
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
                write!(
                    f,
                    "rolling ball: adjacent faces could not be blended ({reason:?})"
                )
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
            Self::InvalidTopology => {
                f.write_str("rolling ball: rebuilt body is not watertight and healthy")
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
            let radial =
                (point - proj)
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
    let (u0, u1) = ordered_bounds(u0, u1);
    let (v0, v1) = ordered_bounds(v0, v1);

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
        u = clamp_ordered(u - step_u, u0, u1);
        v = clamp_ordered(v - step_v, v0, v1);
        if step_u.abs() + step_v.abs() <= 1e-12 {
            break;
        }
    }
    (u, v)
}

fn ordered_bounds(a: f64, b: f64) -> (f64, f64) {
    if a <= b {
        (a, b)
    } else {
        (b, a)
    }
}

fn clamp_ordered(value: f64, min: f64, max: f64) -> f64 {
    let (lo, hi) = ordered_bounds(min, max);
    if value.is_nan() {
        lo
    } else {
        value.max(lo).min(hi)
    }
}

fn radial(axis: Dir, xref: Dir, u: f64) -> GeomVec {
    let x = GeomVec::from_dir(xref);
    let y = GeomVec::from_dir(axis).cross(&GeomVec::from_dir(xref));
    x * u.cos() + y * u.sin()
}

fn make_gregory_corner_patch(corner: Pnt, p_a: Pnt, p_b: Pnt, _radius: f64) -> Face {
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
        corner, p01, p02, corner, p10, p20, p_a, p31, p32, p_b, p13, p23, p11_u, p11_v, p21_u,
        p21_v, p12_u, p12_v, p22_u, p22_v,
    );

    let e1 = Edge::between_points(corner, p_a);
    let e2 = Edge::between_points(p_a, p_b);
    let e3 = Edge::between_points(p_b, corner);

    let wire = Wire::from_edges([e1, e2, e3]);
    Face::new(Some(GeomSurface::gregory(surf)), wire)
}

/// Apply a selected-edge rolling-ball fillet to a simple planar or curved solid.
///
/// When the filleted edge ends at a corner already rounded by an earlier fillet
/// (two blends meeting at a shared vertex), the corner is closed with a smooth
/// spherical patch — the rolling ball pivoting in the corner — instead of the
/// degenerate flat trim that collapses the prior cylinder to its own axis (the
/// "spike" artifact). If that smooth path leaves a non-watertight shell for an
/// unusual corner, the build retries with the legacy corner trim, which always
/// closes (it just creases).
pub fn fillet_planar_edge(
    solid: &Solid,
    edge: &Edge,
    radius: f64,
) -> Result<Solid, RollingBallError> {
    match fillet_planar_edge_inner(solid, edge, radius, true) {
        Ok(solid) => Ok(solid),
        Err(_) => fillet_planar_edge_inner(solid, edge, radius, false),
    }
}

fn fillet_planar_edge_inner(
    solid: &Solid,
    edge: &Edge,
    radius: f64,
    use_sphere: bool,
) -> Result<Solid, RollingBallError> {
    let mut blend = rolling_ball_fillet_edge(solid, edge, radius)?;
    let start = edge.source().point();
    let end = edge.target().point();

    let start_caps = endpoint_cap_faces(solid, start, &blend.face_a, &blend.face_b);
    let end_caps = endpoint_cap_faces(solid, end, &blend.face_a, &blend.face_b);
    let cut_guards = cut_cylinder_guards(solid, &start_caps, &end_caps);

    let mut faces = Vec::new();
    let mut skipped_faces = std::collections::HashSet::new();

    // First pass: smart corner closure, before the spine trims read the contacts
    // (`try_corner_miter` extends the new blend toward the seam by mutating `blend`).
    //
    // - One earlier fillet meets this one and the corner's third edge is still
    //   sharp → the two fillets *miter*, meeting along their mutual seam and leaving
    //   the third edge sharp (`try_corner_miter`).
    // - Two earlier fillets already meet here (mitered) and this is the third edge
    //   at the corner → it closes into a spherical octant (`try_corner_sphere_two_caps`).
    //
    // The two-cap sphere is tried first (a two-cap corner is never a single-cap
    // miter). Both are gated on `use_sphere`, so the retry path (use_sphere = false)
    // falls back to the flat corner trim if either leaves a non-watertight shell.
    // A blend endpoint that runs into a concave cut cylinder is trimmed flush
    // against it (the cut stays a clean vertical cylinder). This is not a sphere
    // path, so it runs regardless of `use_sphere`, and takes priority over the
    // miter/sphere closures.
    let start_cut = try_corner_cut(
        solid,
        &mut blend,
        start,
        &start_caps,
        radius,
        &mut faces,
        &mut skipped_faces,
    )?;
    let start_tangent_wall = if start_cut {
        false
    } else {
        try_tangent_curved_wall_runout(
            solid,
            &mut blend,
            start,
            &start_caps,
            radius,
            &mut faces,
            &mut skipped_faces,
        )?
    };
    let start_mitered = start_cut
        || start_tangent_wall
        || (use_sphere
            && (try_corner_sphere_two_caps(
                solid,
                &blend,
                start,
                &start_caps,
                radius,
                &mut faces,
                &mut skipped_faces,
            ) || try_corner_miter(
                solid,
                &mut blend,
                start,
                &start_caps,
                radius,
                &mut faces,
                &mut skipped_faces,
            )));
    let end_cut = try_corner_cut(
        solid,
        &mut blend,
        end,
        &end_caps,
        radius,
        &mut faces,
        &mut skipped_faces,
    )?;
    let end_tangent_wall = if end_cut {
        false
    } else {
        try_tangent_curved_wall_runout(
            solid,
            &mut blend,
            end,
            &end_caps,
            radius,
            &mut faces,
            &mut skipped_faces,
        )?
    };
    let end_mitered = end_cut
        || end_tangent_wall
        || (use_sphere
            && (try_corner_sphere_two_caps(
                solid,
                &blend,
                end,
                &end_caps,
                radius,
                &mut faces,
                &mut skipped_faces,
            ) || try_corner_miter(
                solid,
                &mut blend,
                end,
                &end_caps,
                radius,
                &mut faces,
                &mut skipped_faces,
            )));

    if !start_mitered {
        handle_corner_endpoint(
            solid,
            &blend,
            start,
            &start_caps,
            &blend.start_arc,
            radius,
            use_sphere,
            &mut faces,
            &mut skipped_faces,
        )?;
    }
    if !end_mitered {
        handle_corner_endpoint(
            solid,
            &blend,
            end,
            &end_caps,
            &blend.end_arc,
            radius,
            use_sphere,
            &mut faces,
            &mut skipped_faces,
        )?;
    }

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
    faces.push(blend.blend_face);

    // Reject a blend that didn't close: a radius too large for the local geometry
    // (e.g. larger than half the part thickness) collapses trim edges and leaves a
    // non-watertight / degenerate shell. Surface that as an error rather than
    // returning a broken solid the application would cache.
    let result = Solid::new(sew(&faces, radius * 0.1));
    let merged =
        crate::merge::merge_cocylindrical_faces(&crate::merge::merge_coplanar_faces(&result));
    if cut_guards.is_empty() {
        if let Some(accepted) = accept_subtractive_blend_result(&merged, &cut_guards) {
            return Ok(accepted);
        }
        if let Some(accepted) = accept_subtractive_blend_result(&result, &cut_guards) {
            return Ok(accepted);
        }
    } else {
        if let Some(accepted) = accept_subtractive_blend_result(&result, &cut_guards) {
            return Ok(accepted);
        }
        if let Some(accepted) = accept_subtractive_blend_result(&merged, &cut_guards) {
            return Ok(accepted);
        }
    }
    Err(RollingBallError::InvalidTopology)
}

#[derive(Clone)]
struct CutCylinderGuard {
    cyl: CylindricalSurface,
    v_min: f64,
    v_max: f64,
}

fn cut_cylinder_guards(
    solid: &Solid,
    start_caps: &[Face],
    end_caps: &[Face],
) -> Vec<CutCylinderGuard> {
    start_caps
        .iter()
        .chain(end_caps.iter())
        .filter(|cap| is_concave_cut_cylinder(solid, cap))
        .filter_map(|cap| {
            let Some(GeomSurface::Cylinder(cyl)) = cap.surface() else {
                return None;
            };
            let mut v_min = f64::INFINITY;
            let mut v_max = f64::NEG_INFINITY;
            let axis_loc = cyl.position().location();
            let axis = GeomVec::from_dir(cyl.position().direction());
            for wire in cap.wires() {
                for edge in wire.edges() {
                    let Some(curve) = edge.curve() else {
                        continue;
                    };
                    let (t0, t1) = (edge.first(), edge.last());
                    for k in 0..=8 {
                        let t = t0 + (t1 - t0) * (k as f64) / 8.0;
                        let p = curve.point(t);
                        let v = (p - axis_loc).dot(&axis);
                        v_min = v_min.min(v);
                        v_max = v_max.max(v);
                    }
                }
            }
            v_min.is_finite().then_some(CutCylinderGuard {
                cyl: *cyl,
                v_min,
                v_max,
            })
        })
        .collect()
}

fn accept_subtractive_blend_result(
    candidate: &Solid,
    cut_guards: &[CutCylinderGuard],
) -> Option<Solid> {
    if !candidate.is_watertight() || !candidate.health_report().is_healthy() {
        return None;
    }
    if cut_guards.is_empty() {
        return Some(candidate.clone());
    }
    let intrudes = solid_surface_intrudes_into_cut(candidate, cut_guards);
    if !intrudes {
        return Some(candidate.clone());
    }

    let mut clipped = candidate.clone();
    for guard in cut_guards {
        if !solid_surface_intrudes_into_cut(&clipped, std::slice::from_ref(guard)) {
            continue;
        }
        let axis = guard.cyl.position();
        let dir = axis.direction();
        let base = axis.location() + GeomVec::from_dir(dir) * (guard.v_min - 0.25);
        let cutter_axis = Ax2::new_axes(base, dir, axis.x_direction());
        let cutter = make_cylinder(
            &cutter_axis,
            guard.cyl.radius(),
            (guard.v_max - guard.v_min).abs() + 0.5,
        );
        clipped = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            crate::boolean_checked(&clipped, &cutter, crate::BooleanOp::Cut)
        }))
        .ok()
        .and_then(Result::ok)?;
        clipped =
            crate::merge::merge_cocylindrical_faces(&crate::merge::merge_coplanar_faces(&clipped));
    }
    (clipped.is_watertight()
        && clipped.health_report().is_healthy()
        && !solid_surface_intrudes_into_cut(&clipped, cut_guards))
    .then_some(clipped)
}

fn solid_surface_intrudes_into_cut(candidate: &Solid, cut_guards: &[CutCylinderGuard]) -> bool {
    solid_surface_intrudes_into_cut_once(candidate, cut_guards)
}

fn solid_surface_intrudes_into_cut_once(
    candidate: &Solid,
    cut_guards: &[CutCylinderGuard],
) -> bool {
    let mesh = tessellate(candidate, 0.05, 0.5);
    let faces = candidate.shell().faces();
    for (i, tri) in mesh.triangles.iter().enumerate() {
        let a = mesh.vertices[tri[0] as usize];
        let b = mesh.vertices[tri[1] as usize];
        let c = mesh.vertices[tri[2] as usize];
        let centroid = Pnt::new(
            (a.x() + b.x() + c.x()) / 3.0,
            (a.y() + b.y() + c.y()) / 3.0,
            (a.z() + b.z() + c.z()) / 3.0,
        );
        let face = mesh
            .face_ids
            .get(i)
            .and_then(|fid| faces.get(*fid as usize));
        for guard in cut_guards {
            if face
                .and_then(|face| face.surface())
                .is_some_and(|surface| match surface {
                    GeomSurface::Cylinder(cyl) => cylinders_same_surface(&cyl, &guard.cyl),
                    _ => false,
                })
            {
                continue;
            }
            if point_inside_cut_guard(centroid, guard, 0.75) {
                return true;
            }
        }
    }
    false
}

fn cylinders_same_surface(a: &CylindricalSurface, b: &CylindricalSurface) -> bool {
    if (a.radius() - b.radius()).abs() > 1.0e-6 {
        return false;
    }
    let ad = GeomVec::from_dir(a.position().direction());
    let bd = GeomVec::from_dir(b.position().direction());
    if ad.dot(&bd).abs() < 0.999_999 {
        return false;
    }
    let delta = b.position().location() - a.position().location();
    (delta - ad * delta.dot(&ad)).magnitude() <= 1.0e-5
}

fn point_inside_cut_guard(p: Pnt, guard: &CutCylinderGuard, margin: f64) -> bool {
    let axis_loc = guard.cyl.position().location();
    let axis = GeomVec::from_dir(guard.cyl.position().direction());
    let v = (p - axis_loc).dot(&axis);
    if v < guard.v_min - 0.05 || v > guard.v_max + 0.05 {
        return false;
    }
    let radial = (p - axis_loc) - axis * v;
    radial.magnitude() < guard.cyl.radius() - margin
}

/// Faces produced for one shared corner: the new faces to add and the original
/// faces (the prior cylinder, the trimmed side face) those replace.
struct CornerSphere {
    new_faces: Vec<Face>,
    skip_ids: Vec<FaceId>,
}

/// Trim/close the faces around one spine endpoint of a blend.
///
/// Three cases:
/// - The endpoint cap is a single prior *blend* surface (a cylinder/torus) — two
///   fillets share this corner. Round it with a [`corner_sphere_blend`].
/// - The cap is a single planar face — trim that face for the blend's end arc.
/// - Several cap faces (an n-valent vertex) — drop in a Gregory corner patch.
#[allow(clippy::too_many_arguments)]
fn handle_corner_endpoint(
    solid: &Solid,
    blend: &RollingBallBlend,
    corner: Pnt,
    caps: &[Face],
    arc: &Edge,
    radius: f64,
    use_sphere: bool,
    faces: &mut Vec<Face>,
    skipped: &mut std::collections::HashSet<FaceId>,
) -> Result<(), RollingBallError> {
    // The blend's two contact points at this corner (nearest endpoint of each
    // contact curve), independent of spine orientation.
    let ca = nearest_endpoint(&blend.contact_a, corner);
    let cb = nearest_endpoint(&blend.contact_b, corner);

    if caps.len() == 1 {
        let cap = &caps[0];
        let is_prior_blend = matches!(
            cap.surface(),
            Some(GeomSurface::Cylinder(_)) | Some(GeomSurface::Torus(_))
        );
        if use_sphere && is_prior_blend {
            if let Some(cs) = corner_sphere_blend(solid, cap, corner, blend, ca, cb, radius) {
                faces.extend(cs.new_faces);
                skipped.extend(cs.skip_ids);
                return Ok(());
            }
        }
        let trimmed = trim_face_at_corner(cap, corner, ca, cb, arc)?;
        faces.push(trimmed);
        skipped.insert(cap.id());
        Ok(())
    } else {
        let patch = make_gregory_corner_patch(corner, ca, cb, radius);
        faces.push(patch);
        for cap in caps {
            if let Ok(trimmed) = trim_face_at_corner(cap, corner, ca, cb, arc) {
                faces.push(trimmed);
                skipped.insert(cap.id());
            }
        }
        Ok(())
    }
}

/// Position tolerance for corner-blend cross-checks, scaled to the fillet radius.
///
/// The corner constructors compare points they derive analytically (sphere/miter
/// tangents `T`/`F`/`R`/`K`) against geometry the blend already built. When the
/// GUI drives a fillet, those "already built" points are re-read from a quantized
/// tessellated mesh, so they drift from the exact analytic values by a few parts
/// in `1e4`. A fixed `1e-4` literal rejects genuine equal-radius perpendicular
/// corners under that drift — they then fall to the flat-trim crease (the user's
/// "becomes flat near the miter"). Scale the tolerance to the radius instead.
fn corner_tol(r: f64) -> f64 {
    (1e-4 * r.max(1.0)).max(10.0 * tolerance::CONFUSION)
}

/// The endpoint of `edge` nearest to `point`.
pub(crate) fn nearest_endpoint(edge: &Edge, point: Pnt) -> Pnt {
    let s = edge.source().point();
    let t = edge.target().point();
    if s.distance(&point) <= t.distance(&point) {
        s
    } else {
        t
    }
}

/// Round a corner where the new blend meets a cylinder left by an earlier fillet.
///
/// Geometry (a box's top-front-right corner with the top-front and top-right
/// edges blended, radius `r`): the rolling ball, pivoting in the corner, sits at
/// the point `C` tangent to all three faces (top, and the two sides). The corner
/// is a spherical octant of radius `r` about `C`, meeting:
/// - the prior cylinder along the great-circle arc T→F (so the prior cylinder is
///   *retracted* to end on that arc instead of overrunning to its own axis),
/// - the new blend cylinder along arc T→R (already built by the normal path),
/// - a small planar closure face along arc F→R, capping the still-sharp third
///   edge's stub.
///
/// `T`, `F`, `R` are the sphere's tangent points on the common (top), prior-side
/// and new-side faces. Returns `None` (caller falls back to the flat trim) unless
/// the corner is the supported convex, mutually-perpendicular, equal-radius case.
fn corner_sphere_blend(
    solid: &Solid,
    cap: &Face,
    corner: Pnt,
    blend: &RollingBallBlend,
    contact_a_corner: Pnt,
    contact_b_corner: Pnt,
    radius: f64,
) -> Option<CornerSphere> {
    let r = radius;
    let near = |p: Pnt, q: Pnt| p.distance(&q) <= 10.0 * tolerance::CONFUSION;

    // Sphere center: the rolling-ball center at this corner.
    let c0 = blend.centerline.start().point();
    let c1 = blend.centerline.end().point();
    let center = if c0.distance(&corner) <= c1.distance(&corner) {
        c0
    } else {
        c1
    };

    // The cap must be a cylinder whose axis passes through the sphere center
    // (true for an equal-radius perpendicular meeting).
    let cyl = match cap.surface() {
        Some(GeomSurface::Cylinder(c)) => *c,
        _ => return None,
    };
    if point_line_distance(
        center,
        cyl.position().location(),
        cyl.position().direction(),
    ) > 1e-5 * r.max(1.0) + 1e-6
    {
        return None;
    }

    // The cap's outer loop must be the simple "two straight contacts + two end
    // arcs" of a single straight-edge fillet.
    let cap_edges = cap.outer_wire()?.edges();
    if cap_edges.len() != 4 {
        return None;
    }
    let is_arc = |e: &Edge| matches!(e.curve(), Some(GeomCurve::Circle(_)));

    // The corner-end arc of the cap: a circular edge with an endpoint at `corner`.
    let corner_arc_idx = cap_edges.iter().position(|e| {
        is_arc(e) && (near(e.source().point(), corner) || near(e.target().point(), corner))
    })?;
    let (arc_s, arc_t) = (
        cap_edges[corner_arc_idx].source().point(),
        cap_edges[corner_arc_idx].target().point(),
    );
    // The arc's other end sits on the cap's "side1" contact (the prior fillet's
    // other planar face); it is also the sharp corner stub's top vertex `K`.
    let other_corner = if near(arc_s, corner) { arc_t } else { arc_s };

    // The two straight contact edges and the planar faces they ride on.
    let straights: Vec<&Edge> = cap_edges.iter().filter(|e| !is_arc(e)).collect();
    if straights.len() != 2 {
        return None;
    }
    let top_contact = *straights
        .iter()
        .find(|e| near(e.source().point(), corner) || near(e.target().point(), corner))?;
    let side1_contact = *straights.iter().find(|e| {
        near(e.source().point(), other_corner) || near(e.target().point(), other_corner)
    })?;
    let common = adjacent_planar_face(solid, top_contact, cap)?;
    let side1 = adjacent_planar_face(solid, side1_contact, cap)?;

    // The common face must be one of the new blend's two faces; the other new
    // blend face is "side2".
    let side2 = if same_face(&common, &blend.face_a) {
        blend.face_b.clone()
    } else if same_face(&common, &blend.face_b) {
        blend.face_a.clone()
    } else {
        return None;
    };

    let n_top = planar_outward_normal(&common).ok()?;
    let n_side1 = planar_outward_normal(&side1).ok()?;
    let n_side2 = planar_outward_normal(&side2).ok()?;

    // Supported only for a convex, mutually-perpendicular trihedral corner.
    let perp = |a: Dir, b: Dir| GeomVec::from_dir(a).dot(&GeomVec::from_dir(b)).abs() < 1e-3;
    if !(perp(n_top, n_side1) && perp(n_top, n_side2) && perp(n_side1, n_side2)) {
        return None;
    }

    // Sphere tangent points on each face, and the sharp corner stub vertex K.
    let tangent = |n: Dir| center + GeomVec::from_dir(n) * r;
    let t = tangent(n_top);
    let f = tangent(n_side1);
    let rr = tangent(n_side2);
    let k = f + (rr - center);

    // Cross-checks against geometry the rest of the build already produced:
    // T/R must coincide with the new blend's two corner contacts, and K with the
    // cap's other corner vertex. Otherwise the corner is not the case we model.
    let xtol = corner_tol(r);
    let matches_a_or_b =
        |x: Pnt| x.distance(&contact_a_corner) < xtol || x.distance(&contact_b_corner) < xtol;
    if !matches_a_or_b(t) || !matches_a_or_b(rr) || k.distance(&other_corner) > xtol {
        return None;
    }

    // Retract the prior cylinder: its corner-end arc becomes the sphere arc T→F,
    // and its two contacts are shortened to T and F.
    let mut new_cap_edges = Vec::with_capacity(4);
    for (i, e) in cap_edges.iter().enumerate() {
        if i == corner_arc_idx {
            let s = if near(e.source().point(), corner) {
                t
            } else {
                f
            };
            let d = if near(e.target().point(), corner) {
                t
            } else {
                f
            };
            let a = arc_on_sphere(center, s, d, r).ok()?;
            new_cap_edges.push(orient_edge_between(&a, s, d));
        } else if is_arc(e) {
            new_cap_edges.push(e.clone());
        } else {
            let remap = |p: Pnt| {
                if near(p, corner) {
                    t
                } else if near(p, other_corner) {
                    f
                } else {
                    p
                }
            };
            new_cap_edges.push(Edge::between_points(
                remap(e.source().point()),
                remap(e.target().point()),
            ));
        }
    }
    let retracted_cap = Face::with_wires(
        cap.surface().cloned(),
        Some(Wire::from_edges(new_cap_edges)),
        cap.inner_wires(),
        cap.orientation(),
    );

    // The spherical octant T→F→R.
    let sphere_face = Face::new(
        Some(GeomSurface::sphere(SphericalSurface::new(
            Ax3::new_axes(center, n_top, n_side2),
            r,
        ))),
        Wire::from_edges([
            arc_on_sphere(center, t, f, r).ok()?,
            arc_on_sphere(center, f, rr, r).ok()?,
            arc_on_sphere(center, rr, t, r).ok()?,
        ]),
    );

    // The planar closure capping the sharp corner stub, bounded by F→K, K→R and
    // the sphere's R→F arc.
    let closure_face = Face::new(
        Some(GeomSurface::plane(Plane::from_point_normal(f, n_top))),
        Wire::from_edges([
            Edge::between_points(f, k),
            Edge::between_points(k, rr),
            orient_edge_between(&arc_on_sphere(center, rr, f, r).ok()?, rr, f),
        ]),
    );

    // Split the prior fillet's side face so its contact edge ends at F (the rest
    // of that edge, F→K, is shared with the closure face).
    let side1_edges = side1.outer_wire()?.edges();
    let mut new_side1 = Vec::with_capacity(side1_edges.len() + 1);
    let mut split = false;
    for e in &side1_edges {
        if !split && same_undirected_edge(e, side1_contact) {
            let (s, d) = (e.source().point(), e.target().point());
            new_side1.push(Edge::between_points(s, f));
            new_side1.push(Edge::between_points(f, d));
            split = true;
        } else {
            new_side1.push(e.clone());
        }
    }
    if !split {
        return None;
    }
    let modified_side1 = Face::with_wires(
        side1.surface().cloned(),
        Some(Wire::from_edges(new_side1)),
        side1.inner_wires(),
        side1.orientation(),
    );

    Some(CornerSphere {
        new_faces: vec![retracted_cap, sphere_face, closure_face, modified_side1],
        skip_ids: vec![cap.id(), side1.id()],
    })
}

/// True when `cap` is a *concave* cylindrical wall (a bored/cut cylinder) whose
/// material lies *outside* the cylinder, as opposed to a *convex* prior-fillet
/// blend cylinder (material inside).
///
/// Face orientation alone does not discriminate (a convex blend cylinder can be
/// stored with the same winding as a cut wall), so we probe the solid directly:
/// at a point in the *interior* of the cap face, step a little radially outward
/// and inward and ask which side is material. A concave cut has material on the
/// *outside* of the cylinder; a convex boss on the inside.
pub(crate) fn is_concave_cut_cylinder(solid: &Solid, cap: &Face) -> bool {
    let cyl = match cap.surface() {
        Some(GeomSurface::Cylinder(c)) => *c,
        _ => return false,
    };
    let Some(wire) = cap.outer_wire() else {
        return false;
    };
    let verts: std::vec::Vec<Pnt> = wire.edges().iter().map(|e| e.source().point()).collect();
    if verts.is_empty() {
        return false;
    }
    let mut sum = GeomVec::new(0.0, 0.0, 0.0);
    for v in &verts {
        sum += *v - Pnt::origin();
    }
    let centroid = Pnt::origin() + sum * (1.0 / verts.len() as f64);
    let axis_pt = cyl.position().location();
    let axis_dir = GeomVec::from_dir(cyl.position().direction());
    let v = centroid - axis_pt;
    let along = axis_dir * v.dot(&axis_dir);
    let radial = match (v - along).normalized() {
        Some(r) => GeomVec::from_dir(r),
        None => return false,
    };
    let on_wall = axis_pt + along + radial * cyl.radius();
    let eps = (cyl.radius() * 0.05).max(10.0 * tolerance::CONFUSION);
    let outside = on_wall + radial * eps;
    let inside = on_wall - radial * eps;
    // Concave cut: material outside, void inside.
    crate::boolean::point_in_solid(&outside, solid)
        && !crate::boolean::point_in_solid(&inside, solid)
}

/// Intersect the infinite line `p0 + t·dir` with `cyl`, returning the hit point
/// nearest `near` (or `None` if the line misses the cylinder).
pub(crate) fn line_meets_cylinder(
    p0: Pnt,
    dir: Dir,
    cyl: &CylindricalSurface,
    near: Pnt,
) -> Option<Pnt> {
    let hits = line_cylinder_intersections(p0, dir, cyl)?;
    Some(if hits[0].distance(&near) <= hits[1].distance(&near) {
        hits[0]
    } else {
        hits[1]
    })
}

fn line_cylinder_intersections(p0: Pnt, dir: Dir, cyl: &CylindricalSurface) -> Option<[Pnt; 2]> {
    let axis_pt = cyl.position().location();
    let w = GeomVec::from_dir(cyl.position().direction());
    let d = GeomVec::from_dir(dir);
    // Components perpendicular to the cylinder axis.
    let e = p0 - axis_pt;
    let e_perp = e - w * e.dot(&w);
    let d_perp = d - w * d.dot(&w);
    let a = d_perp.dot(&d_perp);
    if a <= tolerance::CONFUSION {
        return None; // line parallel to axis: no transverse crossing
    }
    let b = 2.0 * e_perp.dot(&d_perp);
    let c = e_perp.dot(&e_perp) - cyl.radius() * cyl.radius();
    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        return None;
    }
    let sq = disc.max(0.0).sqrt();
    let t0 = (-b - sq) / (2.0 * a);
    let t1 = (-b + sq) / (2.0 * a);
    let p_at = |t: f64| p0 + d * t;
    Some([p_at(t0), p_at(t1)])
}

fn line_meets_cylinder_on_edge(
    p0: Pnt,
    dir: Dir,
    cyl: &CylindricalSurface,
    near: Pnt,
    edge: &Edge,
) -> Option<Pnt> {
    let hits = line_cylinder_intersections(p0, dir, cyl)?;
    let score = |p: Pnt| point_on_edge_score(edge, p) + 1.0e-7 * p.distance(&near);
    Some(if score(hits[0]) <= score(hits[1]) {
        hits[0]
    } else {
        hits[1]
    })
}

fn point_on_edge_score(edge: &Edge, point: Pnt) -> f64 {
    match edge.curve() {
        Some(GeomCurve::Line(_)) | None => {
            point_segment_distance(point, edge.source().point(), edge.target().point())
        }
        Some(GeomCurve::Circle(circle)) => {
            let center = circle.center();
            let axis = GeomVec::from_dir(circle.axis());
            let x = GeomVec::from_dir(circle.position().x_direction());
            let y = GeomVec::from_dir(circle.position().y_direction());
            let v = point - center;
            let axial = v.dot(&axis).abs();
            let radial_vec = v - axis * v.dot(&axis);
            let radial = radial_vec.magnitude();
            let raw = radial_vec.dot(&y).atan2(radial_vec.dot(&x));
            let lo = edge.first().min(edge.last());
            let hi = edge.first().max(edge.last());
            let mut param_penalty = f64::INFINITY;
            for k in -3..=3 {
                let u = raw + (k as f64) * core::f64::consts::TAU;
                let penalty = if u < lo {
                    (lo - u) * circle.radius()
                } else if u > hi {
                    (u - hi) * circle.radius()
                } else {
                    0.0
                };
                param_penalty = param_penalty.min(penalty);
            }
            axial + (radial - circle.radius()).abs() + param_penalty
        }
        Some(curve) => {
            let mut best = f64::INFINITY;
            for k in 0..=24 {
                let t = edge.first() + (edge.last() - edge.first()) * (k as f64) / 24.0;
                best = best.min(point.distance(&curve.point(t)));
            }
            best
        }
    }
}

/// A degree-1 (chorded) B-spline edge through `points`, from the first to the
/// last. Used for the cut-cylinder ∩ blend-cylinder trim curve, which has no
/// closed-form conic representation.
pub(crate) fn polyline_edge(points: &[Pnt]) -> Edge {
    use openrcad_geom::BSplineCurve;
    let n = points.len();
    let mut knots = vec![0.0];
    let mut mults = vec![2usize];
    let mut t = 0.0;
    for i in 1..n {
        t += points[i].distance(&points[i - 1]).max(1e-5);
        knots.push(t);
        mults.push(if i < n - 1 { 1 } else { 2 });
    }
    let curve = GeomCurve::bspline(BSplineCurve::new(1, points.to_vec(), None, knots, mults));
    Edge::new(
        Some(curve),
        0.0,
        t,
        Vertex::new(points[0]),
        Vertex::new(points[n - 1]),
    )
}

/// Sample the intersection of the `blend` cylinder with the `cut` cylinder
/// between contact points `p_a` (on the blend's A-side contact) and `p_b` (B-side),
/// returning a chorded B-spline edge that lies on *both* surfaces.
///
/// Parametrised by the blend cylinder's angle about its own axis (monotonic along
/// this quarter-arc of contact); for each angle the axial position is solved so
/// the point also lands on the cut cylinder.
fn cyl_cyl_trim_edge(
    blend: &CylindricalSurface,
    cut: &CylindricalSurface,
    p_a: Pnt,
    p_b: Pnt,
) -> Option<Edge> {
    let bl_pt = blend.position().location();
    let bl_axis = GeomVec::from_dir(blend.position().direction());
    let bx = GeomVec::from_dir(blend.position().x_direction());
    let by = GeomVec::from_dir(blend.position().y_direction());
    let r_b = blend.radius();

    // Angle of a point about the blend axis.
    let angle_of = |p: Pnt| -> f64 {
        let v = p - bl_pt;
        let vp = v - bl_axis * v.dot(&bl_axis);
        vp.dot(&by).atan2(vp.dot(&bx))
    };
    let mut phi_a = angle_of(p_a);
    let phi_b = angle_of(p_b);
    // Sweep the short way between the two contacts.
    use core::f64::consts::PI;
    while phi_a - phi_b > PI {
        phi_a -= 2.0 * PI;
    }
    while phi_b - phi_a > PI {
        phi_a += 2.0 * PI;
    }

    let cut_pt = cut.position().location();
    let cut_axis = GeomVec::from_dir(cut.position().direction());
    // For a fixed blend angle phi, the blend-cylinder point is base(phi) + s·bl_axis;
    // solve s so its distance from the cut axis equals the cut radius.
    let solve = |phi: f64, prev: Pnt| -> Option<Pnt> {
        let base = bl_pt + (bx * phi.cos() + by * phi.sin()) * r_b;
        let u = base - cut_pt;
        let u_perp = u - cut_axis * u.dot(&cut_axis);
        let d_perp = bl_axis - cut_axis * bl_axis.dot(&cut_axis);
        let a = d_perp.dot(&d_perp);
        if a <= tolerance::CONFUSION {
            return None;
        }
        let b = 2.0 * u_perp.dot(&d_perp);
        let c = u_perp.dot(&u_perp) - cut.radius() * cut.radius();
        let disc = b * b - 4.0 * a * c;
        if disc < 0.0 {
            return None;
        }
        let sq = disc.sqrt();
        let s0 = (-b - sq) / (2.0 * a);
        let s1 = (-b + sq) / (2.0 * a);
        let q0 = base + bl_axis * s0;
        let q1 = base + bl_axis * s1;
        Some(if q0.distance(&prev) <= q1.distance(&prev) {
            q0
        } else {
            q1
        })
    };

    // Sample densely enough that the chorded curve hugs both cylinders to well
    // under a render tolerance (≈0.05 mm chords), so the cut wall reads as a
    // clean cylinder rather than a faceted boundary.
    let steps = ((p_a.distance(&p_b) / 0.05).ceil() as usize).clamp(24, 160);
    let mut pts = Vec::with_capacity(steps + 1);
    pts.push(p_a);
    let mut prev = p_a;
    for k in 1..steps {
        let phi = phi_a + (phi_b - phi_a) * (k as f64) / (steps as f64);
        let p = solve(phi, prev)?;
        prev = p;
        pts.push(p);
    }
    pts.push(p_b);
    Some(polyline_edge(&pts))
}

/// Trim a selected-edge fillet into an extruded sketch arc that is tangent to
/// one of the selected edge's planar side faces.
///
/// This is distinct from [`try_corner_cut`]: the wall is convex material
/// boundary, not a void. One blend contact may therefore shorten back along the
/// selected edge until it reaches the wall, while the other contact can end at
/// the original tangent point. The shared endpoint edge is still the true
/// cylinder-cylinder intersection between the blend cylinder and the wall
/// cylinder, so both surfaces keep analytic support and sew without cracks.
#[allow(clippy::too_many_arguments)]
fn try_tangent_curved_wall_runout(
    solid: &Solid,
    blend: &mut RollingBallBlend,
    corner: Pnt,
    caps: &[Face],
    radius: f64,
    faces: &mut Vec<Face>,
    skipped: &mut std::collections::HashSet<FaceId>,
) -> Result<bool, RollingBallError> {
    if caps.len() != 1 {
        return Ok(false);
    }
    let cap = &caps[0];
    if is_concave_cut_cylinder(solid, cap) {
        return Ok(false);
    }
    let wall_cyl = match cap.surface() {
        Some(GeomSurface::Cylinder(c)) => *c,
        _ => return Ok(false),
    };
    let blend_cyl = match blend.blend_face.surface() {
        Some(GeomSurface::Cylinder(c)) => *c,
        _ => return Ok(false),
    };

    // Prior fillets are also cylindrical endpoint caps. They are handled by the
    // miter/sphere paths when the cap axis passes through the rolling-ball center
    // at this corner. A tangent sketch wall sits away from that center.
    let center = nearest_endpoint(&blend.centerline, corner);
    if point_line_distance(
        center,
        wall_cyl.position().location(),
        wall_cyl.position().direction(),
    ) <= 1.0e-5 * radius.max(1.0) + 1.0e-6
    {
        return Ok(false);
    }
    if !wall_is_tangent_to_selected_side(cap, &wall_cyl, blend, corner)? {
        return Ok(false);
    }

    let a_corner = nearest_endpoint(&blend.contact_a, corner);
    let b_corner = nearest_endpoint(&blend.contact_b, corner);
    let a_far = farthest_endpoint(&blend.contact_a, corner);
    let b_far = farthest_endpoint(&blend.contact_b, corner);
    let a_dir = match (a_corner - a_far).normalized() {
        Some(v) => Dir::new(v.x(), v.y(), v.z()),
        None => return Err(RollingBallError::DegenerateSpine),
    };
    let b_dir = match (b_corner - b_far).normalized() {
        Some(v) => Dir::new(v.x(), v.y(), v.z()),
        None => return Err(RollingBallError::DegenerateSpine),
    };

    let (prev_edge, next_edge) = cap_edges_at_corner(cap, corner)?;
    let da_prev = point_on_edge_score(&prev_edge, a_corner);
    let db_prev = point_on_edge_score(&prev_edge, b_corner);
    let (a_trim_edge, b_trim_edge) = if da_prev <= db_prev {
        (&prev_edge, &next_edge)
    } else {
        (&next_edge, &prev_edge)
    };

    let ca_real = match line_meets_cylinder_on_edge(a_far, a_dir, &wall_cyl, a_corner, a_trim_edge)
    {
        Some(p) => p,
        None => return Err(RollingBallError::UnsupportedTrimTopology),
    };
    let cb_real = match line_meets_cylinder_on_edge(b_far, b_dir, &wall_cyl, b_corner, b_trim_edge)
    {
        Some(p) => p,
        None => return Err(RollingBallError::UnsupportedTrimTopology),
    };
    if ca_real.distance(&cb_real) <= tolerance::CONFUSION {
        return Err(RollingBallError::UnsupportedTrimTopology);
    }

    let trim = match cyl_cyl_trim_edge(&blend_cyl, &wall_cyl, ca_real, cb_real) {
        Some(e) => e,
        None => {
            return Err(RollingBallError::BlendSurfaceBuild(
                "tangent wall trim curve",
            ))
        }
    };
    let trimmed_cap = trim_face_at_corner(cap, corner, ca_real, cb_real, &trim)?;

    let new_a = rebuild_contact(&blend.contact_a, a_far, ca_real);
    let new_b = rebuild_contact(&blend.contact_b, b_far, cb_real);
    let is_end = new_a.end().point().distance(&ca_real) <= new_a.start().point().distance(&ca_real);
    let new_blend_face = build_blend_face_with_trim(blend, &new_a, &new_b, &trim, is_end);

    blend.contact_a = new_a;
    blend.contact_b = new_b;
    blend.blend_face = new_blend_face;
    faces.push(trimmed_cap);
    skipped.insert(cap.id());
    Ok(true)
}

fn cap_edges_at_corner(face: &Face, corner: Pnt) -> Result<(Edge, Edge), RollingBallError> {
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
    Ok((edges[prev_idx].clone(), edges[next_idx].clone()))
}

fn wall_is_tangent_to_selected_side(
    cap: &Face,
    wall_cyl: &CylindricalSurface,
    blend: &RollingBallBlend,
    corner: Pnt,
) -> Result<bool, RollingBallError> {
    let axis_pt = wall_cyl.position().location();
    let axis = GeomVec::from_dir(wall_cyl.position().direction());
    let v = corner - axis_pt;
    let radial = v - axis * v.dot(&axis);
    let Some(radial_dir) = radial.normalized() else {
        return Ok(false);
    };
    let radial_vec = GeomVec::from_dir(radial_dir);
    if (radial.magnitude() - wall_cyl.radius()).abs() > 1.0e-4 * wall_cyl.radius().max(1.0) {
        return Ok(false);
    }

    let tangent_to = |face: &Face| -> Result<bool, RollingBallError> {
        let n = planar_outward_normal(face)?;
        Ok(radial_vec.dot(&GeomVec::from_dir(n)).abs() > 0.999)
    };

    let tangent = tangent_to(&blend.face_a)? || tangent_to(&blend.face_b)?;
    if !tangent {
        return Ok(false);
    }

    // Require the cap to actually own the endpoint; otherwise a same-support
    // cylinder elsewhere in the shell could be mistaken for the runout wall.
    Ok(face_contains_point(cap, corner))
}

/// Trim the new blend flush against a concave cut cylinder it runs into at
/// `corner`, leaving the cut a clean full-height vertical cylinder.
///
/// The fillet's quarter-circle end cap (which lies on the *blend* cylinder, not
/// the cut) is replaced by the true blend ∩ cut intersection curve. The blend's
/// two contact curves are extended to where they actually meet the cut wall, and
/// the cut face is re-trimmed to share that same intersection edge — so its
/// boundary stays on the cylinder and the seam is watertight by construction.
/// Mutates `blend` (extended contacts + rebuilt blend face) so the later spine
/// trims pick up the extension, mirroring [`try_corner_miter`]. Returns
/// `Ok(false)` before a concave cut cylinder is recognized. Once one is
/// recognized, failures are reported as clean errors instead of falling back to a
/// flat endpoint cap that cannot share a valid edge with the cut wall.
#[allow(clippy::too_many_arguments)]
fn try_corner_cut(
    solid: &Solid,
    blend: &mut RollingBallBlend,
    corner: Pnt,
    caps: &[Face],
    _radius: f64,
    faces: &mut Vec<Face>,
    skipped: &mut std::collections::HashSet<FaceId>,
) -> Result<bool, RollingBallError> {
    if caps.len() != 1 {
        return Ok(false);
    }
    let cap = &caps[0];
    if !is_concave_cut_cylinder(solid, cap) {
        return Ok(false);
    }
    let cut_cyl = match cap.surface() {
        Some(GeomSurface::Cylinder(c)) => *c,
        _ => return Err(RollingBallError::UnsupportedTrimTopology),
    };
    let blend_cyl = match blend.blend_face.surface() {
        Some(GeomSurface::Cylinder(c)) => *c,
        _ => {
            return Err(RollingBallError::BlendSurfaceBuild(
                "cut trim requires a cylindrical blend",
            ))
        }
    };

    // Which end of each contact runs into the cut.
    let a_corner = nearest_endpoint(&blend.contact_a, corner);
    let b_corner = nearest_endpoint(&blend.contact_b, corner);
    let a_far = farthest_endpoint(&blend.contact_a, corner);
    let b_far = farthest_endpoint(&blend.contact_b, corner);
    let a_dir = match (a_corner - a_far).normalized() {
        Some(v) => Dir::new(v.x(), v.y(), v.z()),
        None => return Err(RollingBallError::DegenerateSpine),
    };
    let b_dir = match (b_corner - b_far).normalized() {
        Some(v) => Dir::new(v.x(), v.y(), v.z()),
        None => return Err(RollingBallError::DegenerateSpine),
    };

    // Where each contact line actually meets the cut wall (may extend past the
    // spine endpoint when the cut boundary curves away).
    let ca_real = match line_meets_cylinder(a_far, a_dir, &cut_cyl, a_corner) {
        Some(p) => p,
        None => return Err(RollingBallError::UnsupportedTrimTopology),
    };
    let cb_real = match line_meets_cylinder(b_far, b_dir, &cut_cyl, b_corner) {
        Some(p) => p,
        None => return Err(RollingBallError::UnsupportedTrimTopology),
    };

    // The shared trim curve, lying on both cylinders.
    let trim = match cyl_cyl_trim_edge(&blend_cyl, &cut_cyl, ca_real, cb_real) {
        Some(e) => e,
        None => return Err(RollingBallError::BlendSurfaceBuild("cut trim curve")),
    };

    // Re-trim the cut face: shorten its rim/wall edges at `corner` to the real
    // contacts and splice in the trim curve (reusing the corner trimmer).
    let trimmed_cap = trim_face_at_corner(cap, corner, ca_real, cb_real, &trim)?;

    // Extend the blend's contacts to the real meet points and rebuild the blend
    // face with the trim curve replacing this end's cap arc. Build everything as
    // locals first, then commit atomically so a fallback can't see a half-mutated
    // blend.
    let new_a = rebuild_contact(&blend.contact_a, a_far, ca_real);
    let new_b = rebuild_contact(&blend.contact_b, b_far, cb_real);
    // The cut is at the contacts' *end* if ca_real sits at their end vertex.
    let is_end = new_a.end().point().distance(&ca_real) <= new_a.start().point().distance(&ca_real);
    let new_blend_face = build_blend_face_with_trim(blend, &new_a, &new_b, &trim, is_end);

    blend.contact_a = new_a;
    blend.contact_b = new_b;
    blend.blend_face = new_blend_face;
    faces.push(trimmed_cap);
    skipped.insert(cap.id());
    Ok(true)
}

/// The endpoint of `edge` farthest from `point`.
pub(crate) fn farthest_endpoint(edge: &Edge, point: Pnt) -> Pnt {
    let s = edge.source().point();
    let t = edge.target().point();
    if s.distance(&point) >= t.distance(&point) {
        s
    } else {
        t
    }
}

/// Rebuild a straight contact edge keeping its `keep` endpoint and moving the
/// other to `moved`.
fn rebuild_contact(edge: &Edge, keep: Pnt, moved: Pnt) -> Edge {
    // `keep`/`moved` are passed as the far/real points; map them onto the edge's
    // current source→target order so the contact direction is preserved.
    let s = edge.source().point();
    if s.distance(&keep) <= s.distance(&moved) {
        Edge::between_points(keep, moved)
    } else {
        Edge::between_points(moved, keep)
    }
}

/// Build the blend face's wire from the given contacts, replacing the end
/// (`is_end`) or start cap arc with the `trim` curve. The blend cylinder surface
/// is unchanged. The blend wire is always
/// `[contact_a, end_arc, contact_b.reversed(), start_arc]`.
fn build_blend_face_with_trim(
    blend: &RollingBallBlend,
    ca: &Edge,
    cb: &Edge,
    trim: &Edge,
    is_end: bool,
) -> Face {
    let end_src = if is_end { trim } else { &blend.end_arc };
    let start_src = if is_end { &blend.start_arc } else { trim };
    let end_edge = orient_edge_between(end_src, ca.end().point(), cb.end().point());
    let start_edge = orient_edge_between(start_src, cb.start().point(), ca.start().point());
    let wire = Wire::from_edges([ca.clone(), end_edge, cb.clone().reversed(), start_edge]);
    rebuild_face(&blend.blend_face, wire).expect("blend face rebuild is infallible")
}

/// Round a corner where the new blend meets exactly *one* earlier fillet's
/// cylinder and the corner's third edge is still sharp — the Fusion-style miter.
///
/// Two equal-radius perpendicular fillets meeting at a corner whose third edge is
/// NOT filleted must meet along their mutual intersection *seam* (a quarter-ellipse
/// running from the shared top tangent point `T` down to the sharp corner stub
/// vertex `K`), leaving that third edge a crisp edge running down from `K`. This
/// is distinct from the spherical ball-corner, which only belongs when all three
/// edges at the corner are rounded.
///
/// On the supported convex, mutually-perpendicular, equal-radius case this:
/// - retracts the prior cylinder cap so its corner end follows the seam (its top
///   contact shortens to `T`; its side contact already ends at `K`), pushing the
///   retracted cap and skipping the original,
/// - extends the new blend toward `K` and swaps its corner end arc for the seam,
///   mutating `blend` so the later spine trims pick up the extension,
/// - leaves the third edge's two side faces untouched (no closure, no sphere),
///
/// and returns `true`. Returns `false` (caller falls back to the sphere / flat
/// corner trim) for any unsupported corner.
#[allow(clippy::too_many_arguments)]
fn try_corner_miter(
    solid: &Solid,
    blend: &mut RollingBallBlend,
    corner: Pnt,
    caps: &[Face],
    radius: f64,
    faces: &mut Vec<Face>,
    skipped: &mut std::collections::HashSet<FaceId>,
) -> bool {
    let r = radius;
    let near = |p: Pnt, q: Pnt| p.distance(&q) <= 10.0 * tolerance::CONFUSION;

    if caps.len() != 1 {
        return false;
    }
    let cap = &caps[0];
    if !matches!(
        cap.surface(),
        Some(GeomSurface::Cylinder(_)) | Some(GeomSurface::Torus(_))
    ) {
        return false;
    }

    // The corner's two new-blend contacts (nearest endpoint of each contact curve).
    let contact_a_corner = nearest_endpoint(&blend.contact_a, corner);
    let contact_b_corner = nearest_endpoint(&blend.contact_b, corner);

    // --- Corner frame: the rolling-ball center at this corner and the cap's
    // contact/arc layout (the same analysis `corner_sphere_blend` performs). ---
    let c0 = blend.centerline.start().point();
    let c1 = blend.centerline.end().point();
    let center = if c0.distance(&corner) <= c1.distance(&corner) {
        c0
    } else {
        c1
    };

    let cyl = match cap.surface() {
        Some(GeomSurface::Cylinder(c)) => *c,
        _ => return false,
    };
    if point_line_distance(
        center,
        cyl.position().location(),
        cyl.position().direction(),
    ) > 1e-5 * r.max(1.0) + 1e-6
    {
        return false;
    }

    let Some(cap_wire) = cap.outer_wire() else {
        return false;
    };
    let cap_edges = cap_wire.edges();
    if cap_edges.len() != 4 {
        return false;
    }
    let is_arc = |e: &Edge| matches!(e.curve(), Some(GeomCurve::Circle(_)));

    let Some(corner_arc_idx) = cap_edges.iter().position(|e| {
        is_arc(e) && (near(e.source().point(), corner) || near(e.target().point(), corner))
    }) else {
        return false;
    };
    let (arc_s, arc_t) = (
        cap_edges[corner_arc_idx].source().point(),
        cap_edges[corner_arc_idx].target().point(),
    );
    // The arc's far end is the sharp corner stub vertex `K`.
    let other_corner = if near(arc_s, corner) { arc_t } else { arc_s };

    // The cap's two contacts are its straight (line) edges. Select Lines
    // explicitly: a cap already mitered at its *other* end carries an elliptical
    // seam edge, which `!is_arc` (= "not a circle") would wrongly count here.
    let straights: Vec<&Edge> = cap_edges
        .iter()
        .filter(|e| matches!(e.curve(), Some(GeomCurve::Line(_))))
        .collect();
    if straights.len() != 2 {
        return false;
    }
    let Some(&top_contact) = straights
        .iter()
        .find(|e| near(e.source().point(), corner) || near(e.target().point(), corner))
    else {
        return false;
    };
    let Some(&side1_contact) = straights
        .iter()
        .find(|e| near(e.source().point(), other_corner) || near(e.target().point(), other_corner))
    else {
        return false;
    };
    let Some(common) = adjacent_planar_face(solid, top_contact, cap) else {
        return false;
    };
    let Some(side1) = adjacent_planar_face(solid, side1_contact, cap) else {
        return false;
    };

    let side2 = if same_face(&common, &blend.face_a) {
        blend.face_b.clone()
    } else if same_face(&common, &blend.face_b) {
        blend.face_a.clone()
    } else {
        return false;
    };

    let (Ok(n_top), Ok(n_side1), Ok(n_side2)) = (
        planar_outward_normal(&common),
        planar_outward_normal(&side1),
        planar_outward_normal(&side2),
    ) else {
        return false;
    };

    let perp = |a: Dir, b: Dir| GeomVec::from_dir(a).dot(&GeomVec::from_dir(b)).abs() < 1e-3;
    if !(perp(n_top, n_side1) && perp(n_top, n_side2) && perp(n_side1, n_side2)) {
        return false;
    }

    // Seam endpoints: `T` (top tangent), `R` (new-blend side tangent), `K` (stub).
    let t = center + GeomVec::from_dir(n_top) * r;
    let rr = center + GeomVec::from_dir(n_side2) * r;
    let k = center + (GeomVec::from_dir(n_side1) + GeomVec::from_dir(n_side2)) * r;

    // Cross-check against the geometry the rest of the build already produced: `T`
    // and `R` are the new blend's two corner contacts, and `K` the cap stub vertex.
    let xtol = corner_tol(r);
    let matches_a_or_b =
        |x: Pnt| x.distance(&contact_a_corner) < xtol || x.distance(&contact_b_corner) < xtol;
    if !matches_a_or_b(t) || !matches_a_or_b(rr) || k.distance(&other_corner) > xtol {
        return false;
    }

    // The miter seam: cylinder cap ∩ new blend, a quarter-ellipse `K -> T`.
    let Some(seam) = miter_seam_edge(center, n_top, n_side1, n_side2, r) else {
        return false;
    };

    // Retract the prior cap: corner arc -> seam (T -> K), top contact -> T, side
    // contact already ends at K. Curve edges other than the corner arc (the far
    // end arc, or an elliptical seam from an earlier miter at the cap's other end)
    // are preserved verbatim — only the straight contacts are remapped.
    let mut new_cap_edges = Vec::with_capacity(4);
    for (i, e) in cap_edges.iter().enumerate() {
        if i == corner_arc_idx {
            let s = if near(e.source().point(), corner) {
                t
            } else {
                k
            };
            let d = if near(e.target().point(), corner) {
                t
            } else {
                k
            };
            new_cap_edges.push(orient_edge_between(&seam, s, d));
        } else if !matches!(e.curve(), Some(GeomCurve::Line(_))) {
            new_cap_edges.push(e.clone());
        } else {
            let remap = |p: Pnt| if near(p, corner) { t } else { p };
            new_cap_edges.push(Edge::between_points(
                remap(e.source().point()),
                remap(e.target().point()),
            ));
        }
    }
    let retracted_cap = Face::with_wires(
        cap.surface().cloned(),
        Some(Wire::from_edges(new_cap_edges)),
        cap.inner_wires(),
        cap.orientation(),
    );

    // Extend the new blend toward `K`: the contact on `side2` (corner point == R)
    // is lengthened so it reaches the stub vertex; the contact on `common` (corner
    // point == T) is unchanged.
    let a_is_side2 = contact_a_corner.distance(&rr) < 1e-4;
    if a_is_side2 {
        blend.contact_a = extend_contact_corner(&blend.contact_a, corner, k);
    } else {
        blend.contact_b = extend_contact_corner(&blend.contact_b, corner, k);
    }

    // Swap the new blend's corner-side end arc for the seam, keeping the blend_face
    // wire [contact_a, end_arc, contact_b.reversed(), start_arc] contiguous.
    let corner_is_source = corner.distance(&blend.spine.source().point())
        <= corner.distance(&blend.spine.target().point());
    if corner_is_source {
        let b0 = blend.contact_b.source().point();
        let a0 = blend.contact_a.source().point();
        blend.start_arc = orient_edge_between(&seam, b0, a0);
    } else {
        let a1 = blend.contact_a.target().point();
        let b1 = blend.contact_b.target().point();
        blend.end_arc = orient_edge_between(&seam, a1, b1);
    }
    blend.blend_face = Face::new(
        blend.blend_face.surface().cloned(),
        Wire::from_edges([
            blend.contact_a.clone(),
            blend.end_arc.clone(),
            blend.contact_b.clone().reversed(),
            blend.start_arc.clone(),
        ]),
    );

    faces.push(retracted_cap);
    skipped.insert(cap.id());
    true
}

/// The miter seam between two equal-radius perpendicular fillets meeting at a
/// corner with center `center`: the quarter-ellipse where their two cylinders
/// intersect, running from the stub vertex `K = center + r(n_side1 + n_side2)`
/// (parameter 0) to the shared top tangent `T = center + r·n_top` (parameter π/2).
///
/// The intersection of two equal-radius cylinders whose axes cross at right angles
/// is a planar ellipse with semi-axes `r√2` (along `n_side1 + n_side2`) and `r`
/// (along `n_top`).
fn miter_seam_edge(center: Pnt, n_top: Dir, n_side1: Dir, n_side2: Dir, r: f64) -> Option<Edge> {
    use core::f64::consts::{FRAC_PI_2, SQRT_2};
    let to_dir = |v: GeomVec| v.normalized().map(|d| Dir::new(d.x(), d.y(), d.z()));
    let major_vec = GeomVec::from_dir(n_side1) + GeomVec::from_dir(n_side2);
    let major_dir = to_dir(major_vec)?;
    let normal = to_dir(GeomVec::from_dir(major_dir).cross(&GeomVec::from_dir(n_top)))?;
    let pos = Ax3::new_axes(center, normal, major_dir);
    let ell = Ellipse::new(pos, r * SQRT_2, r);
    let k = center + major_vec * r;
    let t = center + GeomVec::from_dir(n_top) * r;
    Some(Edge::new(
        Some(GeomCurve::ellipse(ell)),
        0.0,
        FRAC_PI_2,
        Vertex::new(k),
        Vertex::new(t),
    ))
}

/// Rebuild straight contact `contact` with the endpoint nearest `corner` moved to
/// `new_pt` (lengthening the new blend's side contact to reach the stub vertex).
fn extend_contact_corner(contact: &Edge, corner: Pnt, new_pt: Pnt) -> Edge {
    let s = contact.source().point();
    let t = contact.target().point();
    if s.distance(&corner) <= t.distance(&corner) {
        Edge::between_points(new_pt, t)
    } else {
        Edge::between_points(s, new_pt)
    }
}

/// Whether `a` and `b` share at least one (undirected) boundary edge.
fn faces_share_edge(a: &Face, b: &Face) -> bool {
    let b_edges: Vec<Edge> = b.wires().into_iter().flat_map(|w| w.edges()).collect();
    a.wires()
        .into_iter()
        .flat_map(|w| w.edges())
        .any(|ea| b_edges.iter().any(|eb| same_undirected_edge(&ea, eb)))
}

/// Close a corner where the new blend is the *third* edge rounded at a corner —
/// the two earlier fillets already meet there (two prior cylinder caps mitered
/// along their seam). With all three edges now rounded the corner becomes a
/// spherical octant (the rolling ball pivoting in the corner), tangent to all
/// three faces.
///
/// This retracts BOTH prior caps so each ends on a great-circle arc of the sphere
/// (`T -> F` for the side-1 cap, `T -> R` for the side-2 cap, replacing the miter
/// seam they shared), inserts the spherical octant `T-F-R`, and skips the two
/// originals. The new blend's own corner end arc (`F -> R`, in the plane through
/// the two side tangents) already lies on the sphere, so the new blend face is
/// left untouched. Returns `true` on the supported convex, mutually-perpendicular,
/// equal-radius corner; `false` otherwise (caller falls back).
#[allow(clippy::too_many_arguments)]
fn try_corner_sphere_two_caps(
    solid: &Solid,
    blend: &RollingBallBlend,
    corner: Pnt,
    caps: &[Face],
    radius: f64,
    faces: &mut Vec<Face>,
    skipped: &mut std::collections::HashSet<FaceId>,
) -> bool {
    let _ = solid;
    let r = radius;
    let near = |p: Pnt, q: Pnt| p.distance(&q) <= 10.0 * tolerance::CONFUSION;

    if caps.len() != 2 {
        return false;
    }
    if !caps
        .iter()
        .all(|c| matches!(c.surface(), Some(GeomSurface::Cylinder(_))))
    {
        return false;
    }

    // Sphere center: the new blend's rolling-ball center at this corner.
    let c0 = blend.centerline.start().point();
    let c1 = blend.centerline.end().point();
    let center = if c0.distance(&corner) <= c1.distance(&corner) {
        c0
    } else {
        c1
    };

    // The new blend's two corner tangents (on its two side faces).
    let side_a = nearest_endpoint(&blend.contact_a, corner);
    let side_b = nearest_endpoint(&blend.contact_b, corner);

    // The shared top tangent `T` is the far end of each cap's miter seam (the
    // ellipse edge meeting this corner). Both caps must agree on it.
    let seam_far = |cap: &Face| -> Option<Pnt> {
        let w = cap.outer_wire()?;
        for e in w.edges() {
            if matches!(e.curve(), Some(GeomCurve::Ellipse(_))) {
                let (s, t) = (e.source().point(), e.target().point());
                if near(s, corner) {
                    return Some(t);
                }
                if near(t, corner) {
                    return Some(s);
                }
            }
        }
        None
    };
    let (Some(t0), Some(t1)) = (seam_far(&caps[0]), seam_far(&caps[1])) else {
        return false;
    };
    let xtol = corner_tol(r);
    if t0.distance(&t1) > xtol {
        return false;
    }
    let t = t0;

    // All three tangent points must sit on the sphere of radius r about C.
    let on_sphere = |p: Pnt| (p.distance(&center) - r).abs() < xtol;
    if !(on_sphere(t) && on_sphere(side_a) && on_sphere(side_b)) {
        return false;
    }

    // Map each cap to the side tangent it must retract to (the side it borders).
    let side_pt_for = |cap: &Face| -> Option<Pnt> {
        if faces_share_edge(cap, &blend.face_a) {
            Some(side_a)
        } else if faces_share_edge(cap, &blend.face_b) {
            Some(side_b)
        } else {
            None
        }
    };
    let (Some(side0), Some(side1)) = (side_pt_for(&caps[0]), side_pt_for(&caps[1])) else {
        return false;
    };
    // The two caps must border different sides.
    if side0.distance(&side1) < 1e-6 {
        return false;
    }

    // Spherical octant T-F-R (F = side_a, R = side_b).
    let n_top = match (t - center).normalized() {
        Some(d) => Dir::new(d.x(), d.y(), d.z()),
        None => return false,
    };
    let n_x = match (side_b - center).normalized() {
        Some(d) => Dir::new(d.x(), d.y(), d.z()),
        None => return false,
    };
    let (Ok(arc_tf), Ok(arc_fr), Ok(arc_rt)) = (
        arc_on_sphere(center, t, side_a, r),
        arc_on_sphere(center, side_a, side_b, r),
        arc_on_sphere(center, side_b, t, r),
    ) else {
        return false;
    };
    let sphere_face = Face::new(
        Some(GeomSurface::sphere(SphericalSurface::new(
            Ax3::new_axes(center, n_top, n_x),
            r,
        ))),
        Wire::from_edges([arc_tf, arc_fr, arc_rt]),
    );

    // Retract each cap: its miter seam becomes the sphere arc `T <-> side`, and its
    // side contact shortens from the stub vertex (corner) to that side tangent.
    for cap in caps {
        let Some(side_pt) = side_pt_for(cap) else {
            return false;
        };
        let Ok(sphere_arc) = arc_on_sphere(center, t, side_pt, r) else {
            return false;
        };
        let Some(w) = cap.outer_wire() else {
            return false;
        };
        let touches_corner =
            |e: &Edge| near(e.source().point(), corner) || near(e.target().point(), corner);
        let mut new_edges = Vec::with_capacity(4);
        for e in w.edges() {
            if matches!(e.curve(), Some(GeomCurve::Ellipse(_))) && touches_corner(&e) {
                // This cap's seam at *this* corner becomes the sphere arc T <-> side.
                let s = if near(e.source().point(), corner) {
                    side_pt
                } else {
                    t
                };
                let d = if near(e.target().point(), corner) {
                    side_pt
                } else {
                    t
                };
                new_edges.push(orient_edge_between(&sphere_arc, s, d));
            } else if !matches!(e.curve(), Some(GeomCurve::Line(_))) {
                // Far-end arcs and any seam from another corner are preserved.
                new_edges.push(e.clone());
            } else {
                let remap = |p: Pnt| if near(p, corner) { side_pt } else { p };
                new_edges.push(Edge::between_points(
                    remap(e.source().point()),
                    remap(e.target().point()),
                ));
            }
        }
        faces.push(Face::with_wires(
            cap.surface().cloned(),
            Some(Wire::from_edges(new_edges)),
            cap.inner_wires(),
            cap.orientation(),
        ));
        skipped.insert(cap.id());
    }

    faces.push(sphere_face);
    true
}

/// A minor great-circle arc of the sphere centered at `center`, radius `r`, from
/// `start` to `end` (both must lie on the sphere).
fn arc_on_sphere(center: Pnt, start: Pnt, end: Pnt, r: f64) -> Result<Edge, RollingBallError> {
    let axis_v = (start - center).cross(&(end - center));
    let axis = axis_v
        .normalized()
        .map(|d| Dir::new(d.x(), d.y(), d.z()))
        .ok_or(RollingBallError::InvalidDihedral)?;
    let xref = (start - center)
        .normalized()
        .map(|d| Dir::new(d.x(), d.y(), d.z()))
        .ok_or(RollingBallError::InvalidDihedral)?;
    contact_arc(center, axis, xref, r, start, end)
}

/// The planar face of `solid`, other than `exclude`, whose boundary contains
/// `edge` (matched undirected by endpoints).
fn adjacent_planar_face(solid: &Solid, edge: &Edge, exclude: &Face) -> Option<Face> {
    solid.shell().faces().into_iter().find(|f| {
        !same_face(f, exclude)
            && matches!(f.surface(), Some(GeomSurface::Plane(_)))
            && f.wires()
                .into_iter()
                .flat_map(|w| w.edges())
                .any(|c| same_undirected_edge(&c, edge))
    })
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
pub fn fillet_edges(solid: &Solid, edges: &[Edge], radius: f64) -> Result<Solid, RollingBallError> {
    let mut current = solid.clone();
    for edge in edges {
        // Re-locate the edge in the evolving body. `relocate_edge` tolerates the
        // endpoint drift an earlier blend leaves when two requested edges share a
        // corner (the shared vertex is consumed, shortening the survivor) — an
        // exact endpoint match alone would fail there with `SpineNotOnFace`.
        let target = relocate_edge(&current, edge).ok_or(RollingBallError::SpineNotOnFace)?;
        current = fillet_planar_edge(&current, &target, radius)?;
    }
    Ok(current)
}

/// Fillet a logical circular edge that is represented in the B-Rep as several
/// co-circular edge fragments.
///
/// Cylinder booleans often split a circular rim at construction seams. Those
/// split points are not design corners, so sequentially filleting each fragment
/// asks the corner code to cap fake endpoints and produces invalid topology.
/// This treats the fragments as one selected edge: contacts and adjacent faces
/// are trimmed across the whole chain, while only the requested spine endpoints
/// are closed against real cap faces.
pub fn fillet_circular_edge_chain(
    solid: &Solid,
    chain_edges: &[Edge],
    spine: &Edge,
    radius: f64,
) -> Result<Solid, RollingBallError> {
    if chain_edges.is_empty() {
        return Err(RollingBallError::SpineNotOnFace);
    }

    let (plane_face, cyl_faces) = circular_chain_support_faces(solid, chain_edges)?;
    let mut blend =
        rolling_ball_between_curved_faces(solid, spine, &plane_face, &cyl_faces[0], radius)?;
    split_blend_face_for_spine_chain(&mut blend, spine, chain_edges)?;
    let start = spine.source().point();
    let end = spine.target().point();

    let start_caps = endpoint_cap_faces(solid, start, &blend.face_a, &blend.face_b);
    let end_caps = endpoint_cap_faces(solid, end, &blend.face_a, &blend.face_b);
    let cut_guards = cut_cylinder_guards(solid, &start_caps, &end_caps);

    let mut faces = Vec::new();
    let mut skipped_faces = std::collections::HashSet::new();

    let start_cut = try_corner_cut(
        solid,
        &mut blend,
        start,
        &start_caps,
        radius,
        &mut faces,
        &mut skipped_faces,
    )?;
    if !start_cut {
        handle_corner_endpoint(
            solid,
            &blend,
            start,
            &start_caps,
            &blend.start_arc,
            radius,
            false,
            &mut faces,
            &mut skipped_faces,
        )?;
    }

    let end_cut = try_corner_cut(
        solid,
        &mut blend,
        end,
        &end_caps,
        radius,
        &mut faces,
        &mut skipped_faces,
    )?;
    if !end_cut {
        handle_corner_endpoint(
            solid,
            &blend,
            end,
            &end_caps,
            &blend.end_arc,
            radius,
            false,
            &mut faces,
            &mut skipped_faces,
        )?;
    }

    let trimmed_plane =
        trim_face_along_spine_segments(&plane_face, chain_edges, spine, &blend.contact_a)?;
    let mut trimmed_cyls = Vec::new();
    for cyl in &cyl_faces {
        trimmed_cyls.push(trim_face_along_spine_segments(
            cyl,
            chain_edges,
            spine,
            &blend.contact_b,
        )?);
    }

    for face in solid.shell().faces() {
        if same_face(&face, &plane_face)
            || cyl_faces.iter().any(|cyl| same_face(&face, cyl))
            || skipped_faces.contains(&face.id())
        {
            continue;
        }
        faces.push(face);
    }

    faces.push(trimmed_plane);
    faces.extend(trimmed_cyls);
    faces.push(blend.blend_face);

    let result = Solid::new(sew(&faces, radius * 0.1));
    let merged =
        crate::merge::merge_cocylindrical_faces(&crate::merge::merge_coplanar_faces(&result));
    if cut_guards.is_empty() {
        if let Some(accepted) = accept_subtractive_blend_result(&merged, &cut_guards) {
            return Ok(accepted);
        }
        if let Some(accepted) = accept_subtractive_blend_result(&result, &cut_guards) {
            return Ok(accepted);
        }
    } else {
        if let Some(accepted) = accept_subtractive_blend_result(&result, &cut_guards) {
            return Ok(accepted);
        }
        if let Some(accepted) = accept_subtractive_blend_result(&merged, &cut_guards) {
            return Ok(accepted);
        }
    }
    Err(RollingBallError::InvalidTopology)
}

fn circular_chain_support_faces(
    solid: &Solid,
    edges: &[Edge],
) -> Result<(Face, Vec<Face>), RollingBallError> {
    let mut plane: Option<Face> = None;
    for face in adjacent_faces(solid, &edges[0]) {
        if !matches!(face.surface(), Some(GeomSurface::Plane(_))) {
            continue;
        }
        if edges.iter().skip(1).all(|edge| {
            adjacent_faces(solid, edge)
                .into_iter()
                .any(|candidate| same_face(&candidate, &face))
        }) {
            plane = Some(face);
            break;
        }
    }
    let plane = plane.ok_or(RollingBallError::EdgeAdjacency { count: 1 })?;

    let mut cyls: Vec<Face> = Vec::new();
    for edge in edges {
        for face in adjacent_faces(solid, edge) {
            if same_face(&face, &plane) || !matches!(face.surface(), Some(GeomSurface::Cylinder(_)))
            {
                continue;
            }
            if cyls.iter().any(|seen| same_face(seen, &face)) {
                continue;
            }
            if let Some(first) = cyls.first() {
                if !cylinders_same_support(first, &face) {
                    return Err(RollingBallError::UnsolvableAdjacency {
                        reason: AdjacencyReason::UnsupportedSurfacePair,
                    });
                }
            }
            cyls.push(face);
        }
    }
    if cyls.is_empty() {
        return Err(RollingBallError::EdgeAdjacency { count: 1 });
    }
    Ok((plane, cyls))
}

fn cylinders_same_support(a: &Face, b: &Face) -> bool {
    let (Some(GeomSurface::Cylinder(a)), Some(GeomSurface::Cylinder(b))) =
        (a.surface(), b.surface())
    else {
        return false;
    };
    let ac = a.position().location();
    let bc = b.position().location();
    ac.distance(&bc) <= 1.0e-6
        && a.position()
            .direction()
            .dot(&b.position().direction())
            .abs()
            > 0.999_999
        && (a.radius() - b.radius()).abs() <= 1.0e-6
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
        rolling_ball_between_curved_faces(solid, edge, &adjacent[0], &adjacent[1], radius)
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
    solid: &Solid,
    edge: &Edge,
    face_a: &Face,
    face_b: &Face,
    radius: f64,
) -> Result<RollingBallBlend, RollingBallError> {
    if !radius.is_finite() || radius <= tolerance::CONFUSION {
        return Err(RollingBallError::InvalidRadius { radius });
    }

    let (plane_face, cyl_face, is_a_plane) =
        if matches!(face_a.surface(), Some(GeomSurface::Plane(_))) {
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
            let concave_cut = is_concave_cut_cylinder(solid, cyl_face);
            let major_radius = if concave_cut {
                spine_r + radius
            } else {
                spine_r - radius
            };
            if major_radius <= tolerance::CONFUSION {
                return Err(RollingBallError::InvalidRadius { radius });
            }

            // Parametrise the contacts in the SPINE CIRCLE's own frame, not the
            // cylinder's. The edge's `first()/last()` params are angles in the spine
            // circle's frame; a reconstructed sketch-arc rim has a different
            // x-direction than the cylinder it bounds, so using `cyl`'s frame here
            // swept the blend over the wrong arc range (an extruded sketch arc would
            // fillet a different quarter of the rim than the selected edge).
            let cax = circle.axis();
            let cxr = circle.position().x_direction();

            let torus_center = center - GeomVec::from_dir(n_plane) * radius;
            let pos = Ax3::new_axes(torus_center, cax, cxr);
            let torus_surf = GeomSurface::torus(ToroidalSurface::new(pos, major_radius, radius));

            let contact_plane_r = major_radius;
            let contact_cyl_height = radius;

            let c_plane = Circle::new(Ax3::new_axes(center, cax, cxr), contact_plane_r);
            let c_cyl = Circle::new(
                Ax3::new_axes(
                    center - GeomVec::from_dir(n_plane) * contact_cyl_height,
                    cax,
                    cxr,
                ),
                spine_r,
            );

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

            let c0 = torus_center + radial(cax, cxr, u0) * major_radius;
            let c1 = torus_center + radial(cax, cxr, u1) * major_radius;
            let centerline = Edge::between_points(c0, c1);

            let r0 = radial(cax, cxr, u0);
            let dir0 = Dir::new(r0.x(), r0.y(), r0.z());
            let r1 = radial(cax, cxr, u1);
            let dir1 = Dir::new(r1.x(), r1.y(), r1.z());

            let t0_vec = GeomVec::from_dir(cax).cross(&r0).normalized().unwrap();
            let t0 = Dir::new(t0_vec.x(), t0_vec.y(), t0_vec.z());
            let t1_vec = GeomVec::from_dir(cax).cross(&r1).normalized().unwrap();
            let t1 = Dir::new(t1_vec.x(), t1_vec.y(), t1_vec.z());

            let arc_start = contact_arc(
                c0,
                t0,
                dir0,
                radius,
                contact_b.start().point(),
                contact_a.start().point(),
            )?;
            let arc_end = contact_arc(
                c1,
                t1,
                dir1,
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

pub(crate) fn adjacent_faces(solid: &Solid, edge: &Edge) -> Vec<Face> {
    solid
        .shell()
        .faces()
        .into_iter()
        .filter(|face| {
            face.wires()
                .into_iter()
                .flat_map(|wire| wire.edges())
                .any(|candidate| {
                    same_undirected_edge(&candidate, edge)
                        || edge_contains_requested_span(&candidate, edge)
                })
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

fn edge_contains_requested_span(container: &Edge, requested: &Edge) -> bool {
    let c0 = container.start().point();
    let c1 = container.end().point();
    let r0 = requested.start().point();
    let r1 = requested.end().point();
    let c_len = c0.distance(&c1);
    let r_len = r0.distance(&r1);
    let tol = 10.0 * tolerance::CONFUSION;
    if c_len <= tol || r_len <= tol || r_len > c_len + tol {
        return false;
    }

    let Some(dir) = (c1 - c0).normalized() else {
        return false;
    };
    if point_line_distance(r0, c0, dir) > tol || point_line_distance(r1, c0, dir) > tol {
        return false;
    }

    let dir_vec = GeomVec::from_dir(dir);
    let t0 = (r0 - c0).dot(&dir_vec);
    let t1 = (r1 - c0).dot(&dir_vec);
    t0 >= -tol && t0 <= c_len + tol && t1 >= -tol && t1 <= c_len + tol
}

/// Perpendicular distance from `p` to the infinite line through `origin` with
/// unit direction `dir`.
fn point_line_distance(p: Pnt, origin: Pnt, dir: Dir) -> f64 {
    let v = p - origin;
    let along = GeomVec::from_dir(dir) * v.dot(&GeomVec::from_dir(dir));
    (v - along).magnitude()
}

/// Locate the current edge of `solid` that corresponds to `requested`, tolerating
/// endpoint drift left by earlier blends.
///
/// First tries an exact undirected endpoint match (the common case). Failing
/// that — e.g. when a prior fillet trimmed back the shared corner so one endpoint
/// moved — it matches the current straight edge that is **collinear** with the
/// request and **overlaps** its span the most: the surviving sub-segment of the
/// originally-selected edge. Returns that edge with its *actual* current
/// endpoints, so the blend runs along the surviving spine (the consumed end is
/// already rounded by the earlier fillet). Without this, multi-edge fillets that
/// share a corner fail with [`RollingBallError::SpineNotOnFace`].
pub(crate) fn relocate_edge(solid: &Solid, requested: &Edge) -> Option<Edge> {
    let current = solid.edges();
    if let Some(e) = current.iter().find(|c| same_undirected_edge(c, requested)) {
        return Some(e.clone());
    }
    if let Some(candidate) = current
        .iter()
        .find(|candidate| edge_contains_requested_span(candidate, requested))
    {
        return Some(snap_requested_span_to_container(candidate, requested));
    }

    let r0 = requested.start().point();
    let r1 = requested.end().point();
    let len = r0.distance(&r1);
    if len <= tolerance::CONFUSION {
        return None;
    }
    let dir = (r1 - r0).normalized()?;
    let dir_vec = GeomVec::from_dir(dir);
    let tol = 10.0 * tolerance::CONFUSION;

    let mut best: Option<(f64, Edge)> = None;
    for c in &current {
        let c0 = c.start().point();
        let c1 = c.end().point();
        // Both endpoints must lie on the request's infinite line (collinear).
        if point_line_distance(c0, r0, dir) > tol || point_line_distance(c1, r0, dir) > tol {
            continue;
        }
        // 1-D spans along the request direction; keep the largest overlap with
        // [0, len]. A surviving sub-segment overlaps almost the whole request.
        let t0 = (c0 - r0).dot(&dir_vec);
        let t1 = (c1 - r0).dot(&dir_vec);
        let overlap = t0.max(t1).min(len) - t0.min(t1).max(0.0);
        if overlap <= tol {
            continue;
        }
        if best.as_ref().map_or(true, |(o, _)| overlap > *o) {
            best = Some((overlap, c.clone()));
        }
    }
    best.map(|(_, e)| e)
}

fn snap_requested_span_to_container(container: &Edge, requested: &Edge) -> Edge {
    let c0 = container.start().point();
    let c1 = container.end().point();
    let r0 = requested.start().point();
    let r1 = requested.end().point();
    let c_len = c0.distance(&c1);
    let snap_tol = (c_len * 5.0e-4).clamp(10.0 * tolerance::CONFUSION, 0.01);

    let start = snap_point_to_edge_endpoint(r0, c0, c1, snap_tol);
    let end = snap_point_to_edge_endpoint(r1, c0, c1, snap_tol);
    if (start.distance(&c0) <= tolerance::CONFUSION && end.distance(&c1) <= tolerance::CONFUSION)
        || (start.distance(&c1) <= tolerance::CONFUSION
            && end.distance(&c0) <= tolerance::CONFUSION)
    {
        return container.clone();
    }
    if start.distance(&r0) > tolerance::CONFUSION || end.distance(&r1) > tolerance::CONFUSION {
        return Edge::between_points(start, end);
    }
    requested.clone()
}

fn snap_point_to_edge_endpoint(point: Pnt, a: Pnt, b: Pnt, tol: f64) -> Pnt {
    if point.distance(&a) <= tol {
        a
    } else if point.distance(&b) <= tol {
        b
    } else {
        point
    }
}

pub(crate) fn same_face(a: &Face, b: &Face) -> bool {
    a.id() == b.id()
}

pub(crate) fn endpoint_cap_faces(
    solid: &Solid,
    point: Pnt,
    face_a: &Face,
    face_b: &Face,
) -> Vec<Face> {
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

/// Shorten `edge` so the endpoint at `keep` stays put and its other endpoint moves
/// to `moved`, PRESERVING the edge's curve.
///
/// `Edge::between_points` always builds a straight line. Using it to shorten an
/// *arc* boundary edge — which happens when a fillet's end is trimmed against a
/// cylindrical cap face (e.g. an extruded sketch-arc wall) — would replace that
/// arc with an off-surface chord, so the cap face's wire no longer lies on its
/// cylinder and the sewn solid fails the watertight/health gate. For a circular
/// edge this rebuilds the true sub-arc; for any other curve it falls back to the
/// straight segment (the previous behaviour, correct for the planar caps).
fn shorten_edge_keep_curve(edge: &Edge, keep: Pnt, moved: Pnt) -> Edge {
    let Some(GeomCurve::Circle(circle)) = edge.curve() else {
        let source = edge.source().point();
        if source.distance(&keep) <= source.distance(&moved) {
            return Edge::between_points(keep, moved);
        }
        return Edge::between_points(moved, keep);
    };
    let circle = *circle;
    let center = circle.center();
    let x = GeomVec::from_dir(circle.position().x_direction());
    let y = GeomVec::from_dir(circle.axis()).cross(&x);
    let raw_angle = {
        let v = moved - center;
        v.dot(&y).atan2(v.dot(&x))
    };
    let keep_is_source =
        keep.distance(&edge.source().point()) <= keep.distance(&edge.target().point());
    let keep_param = if keep_is_source {
        edge.first()
    } else {
        edge.last()
    };
    // Unwrap the moved endpoint's angle onto the same branch as the kept end so the
    // sub-arc travels the original short way around, not the reflex complement.
    let mut moved_param = raw_angle;
    use core::f64::consts::PI;
    while moved_param - keep_param > PI {
        moved_param -= 2.0 * PI;
    }
    while keep_param - moved_param > PI {
        moved_param += 2.0 * PI;
    }
    let curve = Some(GeomCurve::circle(circle));
    if keep_is_source {
        Edge::new(
            curve,
            edge.first(),
            moved_param,
            edge.source(),
            Vertex::new(moved),
        )
    } else {
        Edge::new(
            curve,
            moved_param,
            edge.last(),
            Vertex::new(moved),
            edge.target(),
        )
    }
}

fn move_edge_endpoint_keep_curve(edge: &Edge, old: Pnt, new: Pnt) -> Edge {
    let tol = 10.0 * tolerance::CONFUSION;
    let source = edge.source().point();
    let target = edge.target().point();
    if source.distance(&old) <= tol {
        return rebuild_edge_with_endpoints(edge, new, target);
    }
    if target.distance(&old) <= tol {
        return rebuild_edge_with_endpoints(edge, source, new);
    }
    edge.clone()
}

fn rebuild_edge_with_endpoints(edge: &Edge, start: Pnt, end: Pnt) -> Edge {
    let Some(GeomCurve::Circle(circle)) = edge.curve() else {
        return Edge::between_points(start, end);
    };
    let circle = *circle;
    let start_param = circle_parameter_for_point(&circle, start, edge.first(), edge.last());
    let end_param = circle_parameter_for_point(&circle, end, edge.first(), edge.last());
    Edge::new(
        Some(GeomCurve::circle(circle)),
        start_param,
        end_param,
        Vertex::new(start),
        Vertex::new(end),
    )
}

fn circle_parameter_for_point(circle: &Circle, point: Pnt, first: f64, last: f64) -> f64 {
    let center = circle.center();
    let x = GeomVec::from_dir(circle.position().x_direction());
    let y = GeomVec::from_dir(circle.axis()).cross(&x);
    let v = point - center;
    let raw = v.dot(&y).atan2(v.dot(&x));
    let lo = first.min(last);
    let hi = first.max(last);
    let mut best = raw;
    let mut best_score = f64::INFINITY;
    for k in -3..=3 {
        let candidate = raw + (k as f64) * core::f64::consts::TAU;
        let score = if candidate < lo {
            lo - candidate
        } else if candidate > hi {
            candidate - hi
        } else {
            0.0
        };
        if score < best_score {
            best = candidate;
            best_score = score;
        }
    }
    best
}

pub(crate) fn trim_face_along_spine(
    face: &Face,
    spine: &Edge,
    contact: &Edge,
) -> Result<Face, RollingBallError> {
    ensure_trimmable_face(face)?;
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
        let Some(idx) = edges
            .iter()
            .position(|candidate| edge_contains_requested_span(candidate, spine))
        else {
            return Err(RollingBallError::SpineNotOnFace);
        };
        return trim_face_along_subspine(face, &edges, idx, spine, contact);
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
            new_edges.push(shorten_edge_keep_curve(
                edge,
                edge.source().point(),
                contact_start_clamped,
            ));
        } else if i == idx {
            new_edges.push(oriented_contact.clone());
        } else if i == next_idx {
            new_edges.push(shorten_edge_keep_curve(
                edge,
                edge.target().point(),
                contact_end_clamped,
            ));
        } else {
            new_edges.push(edge.clone());
        }
    }

    // A mitered corner extends the new blend's side contact onto the prior fillet's
    // own corner vertex, which collapses this face's pre-corner edge to zero length
    // (e.g. the side face's old fillet-end arc shrinks to a point). Drop any such
    // degenerate edges so the trimmed loop stays valid.
    new_edges.retain(|e| e.source().point().distance(&e.target().point()) > tolerance::CONFUSION);

    rebuild_face(face, Wire::from_edges(new_edges))
}

fn trim_face_along_subspine(
    face: &Face,
    edges: &[Edge],
    idx: usize,
    spine: &Edge,
    contact: &Edge,
) -> Result<Face, RollingBallError> {
    let selected = &edges[idx];
    let s0 = selected.source().point();
    let s1 = selected.target().point();
    let len2 = (s1 - s0).magnitude_squared();
    if len2 <= tolerance::CONFUSION * tolerance::CONFUSION {
        return Err(RollingBallError::DegenerateSpine);
    }

    let a = spine.source().point();
    let b = spine.target().point();
    let ta = (a - s0).dot(&(s1 - s0)) / len2;
    let tb = (b - s0).dot(&(s1 - s0)) / len2;
    let (run_start, run_end) = if ta <= tb { (a, b) } else { (b, a) };
    let contact_start = contact_point_for_spine_vertex(spine, contact, run_start);
    let contact_end = contact_point_for_spine_vertex(spine, contact, run_end);
    let oriented_contact = orient_edge_between(contact, contact_start, contact_end);

    let mut new_edges = Vec::with_capacity(edges.len() + 4);
    for (i, edge) in edges.iter().enumerate() {
        if i != idx {
            new_edges.push(edge.clone());
            continue;
        }

        push_nonzero_edge(&mut new_edges, shorten_edge_keep_curve(edge, s0, run_start));
        push_nonzero_edge(
            &mut new_edges,
            Edge::between_points(run_start, contact_start),
        );
        push_nonzero_edge(&mut new_edges, oriented_contact.clone());
        push_nonzero_edge(&mut new_edges, Edge::between_points(contact_end, run_end));
        push_nonzero_edge(&mut new_edges, shorten_edge_keep_curve(edge, s1, run_end));
    }

    rebuild_face(face, Wire::from_edges(new_edges))
}

fn push_nonzero_edge(edges: &mut Vec<Edge>, edge: Edge) {
    if edge.source().point().distance(&edge.target().point()) > tolerance::CONFUSION {
        edges.push(edge);
    }
}

fn split_blend_face_for_spine_chain(
    blend: &mut RollingBallBlend,
    spine: &Edge,
    spine_edges: &[Edge],
) -> Result<(), RollingBallError> {
    let mut spans: Vec<(f64, f64)> = spine_edges
        .iter()
        .map(|edge| {
            let a = spine_parameter_for_point(spine, edge.source().point())?;
            let b = spine_parameter_for_point(spine, edge.target().point())?;
            Ok(if spine.last() >= spine.first() {
                (a.min(b), a.max(b))
            } else {
                (a.max(b), a.min(b))
            })
        })
        .collect::<Result<_, RollingBallError>>()?;
    spans.sort_by(|a, b| {
        a.0.min(a.1)
            .partial_cmp(&b.0.min(b.1))
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut contact_a = Vec::new();
    let mut contact_b = Vec::new();
    for &(a, b) in &spans {
        contact_a.push(edge_on_contact_between_params(&blend.contact_a, a, b)?);
        contact_b.push(edge_on_contact_between_params(&blend.contact_b, a, b)?);
    }

    let mut wire_edges = Vec::with_capacity(contact_a.len() + contact_b.len() + 2);
    wire_edges.extend(contact_a);
    wire_edges.push(blend.end_arc.clone());
    wire_edges.extend(contact_b.into_iter().rev().map(|edge| edge.reversed()));
    wire_edges.push(blend.start_arc.clone());

    blend.blend_face = Face::new(
        blend.blend_face.surface().cloned(),
        Wire::from_edges(wire_edges),
    );
    Ok(())
}

fn trim_face_along_spine_segments(
    face: &Face,
    spine_edges: &[Edge],
    spine: &Edge,
    contact: &Edge,
) -> Result<Face, RollingBallError> {
    ensure_trimmable_face(face)?;
    let edges = face
        .outer_wire()
        .ok_or(RollingBallError::UnsupportedTrimTopology)?
        .edges();
    let n = edges.len();
    if n < 3 || spine_edges.is_empty() || spine_edges.len() >= n {
        return Err(RollingBallError::UnsupportedTrimTopology);
    }

    let selected: std::collections::HashSet<usize> = edges
        .iter()
        .enumerate()
        .filter_map(|(i, candidate)| {
            spine_edges
                .iter()
                .any(|spine_edge| same_undirected_edge(candidate, spine_edge))
                .then_some(i)
        })
        .collect();
    if selected.is_empty() {
        return Err(RollingBallError::SpineNotOnFace);
    }

    let run_start = selected
        .iter()
        .copied()
        .find(|&i| !selected.contains(&((i + n - 1) % n)))
        .ok_or(RollingBallError::UnsupportedTrimTopology)?;
    let mut run = Vec::new();
    let mut i = run_start;
    loop {
        if !selected.contains(&i) {
            break;
        }
        run.push(i);
        i = (i + 1) % n;
        if i == run_start {
            return Err(RollingBallError::UnsupportedTrimTopology);
        }
    }
    if run.len() != selected.len() {
        return Err(RollingBallError::UnsupportedTrimTopology);
    }

    let run_end = *run
        .last()
        .ok_or(RollingBallError::UnsupportedTrimTopology)?;
    let prev_idx = (run_start + n - 1) % n;
    let next_idx = (run_end + 1) % n;
    if selected.contains(&prev_idx) || selected.contains(&next_idx) || prev_idx == next_idx {
        return Err(RollingBallError::UnsupportedTrimTopology);
    }

    let run_start_point = edges[run_start].source().point();
    let run_end_point = edges[run_end].target().point();
    let contact_start = contact_point_for_spine_point(spine, contact, run_start_point)?;
    let contact_end = contact_point_for_spine_point(spine, contact, run_end_point)?;

    let mut new_edges = Vec::with_capacity(n);
    for (i, edge) in edges.iter().enumerate() {
        if selected.contains(&i) {
            new_edges.push(contact_subedge_for_spine_edge(spine, contact, edge)?);
        } else {
            let moved_start = move_edge_endpoint_keep_curve(edge, run_start_point, contact_start);
            let moved = move_edge_endpoint_keep_curve(&moved_start, run_end_point, contact_end);
            new_edges.push(moved);
        }
    }

    new_edges.retain(|e| e.source().point().distance(&e.target().point()) > tolerance::CONFUSION);
    rebuild_face(face, Wire::from_edges(new_edges))
}

pub(crate) fn trim_face_at_corner(
    face: &Face,
    corner: Pnt,
    contact_a: Pnt,
    contact_b: Pnt,
    arc: &Edge,
) -> Result<Face, RollingBallError> {
    ensure_trimmable_face(face)?;
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
            new_edges.push(shorten_edge_keep_curve(
                prev,
                prev.source().point(),
                prev_contact,
            ));
            new_edges.push(oriented_arc.clone());
        } else if i == next_idx {
            new_edges.push(shorten_edge_keep_curve(
                next,
                next.target().point(),
                next_contact,
            ));
        } else {
            new_edges.push(edges[i].clone());
        }
    }

    rebuild_face(face, Wire::from_edges(new_edges))
}

fn ensure_trimmable_face(face: &Face) -> Result<(), RollingBallError> {
    if !matches!(
        face.surface(),
        Some(GeomSurface::Plane(_)) | Some(GeomSurface::Cylinder(_))
    ) {
        return Err(RollingBallError::UnsolvableAdjacency {
            reason: AdjacencyReason::NotPlaneOrAnalytic,
        });
    }
    if face.outer_wire().is_none() {
        return Err(RollingBallError::UnsupportedTrimTopology);
    }
    // Inner wires (holes) are allowed: only the outer loop is re-trimmed for the
    // blend, and `rebuild_face` carries the holes through untouched. A hole that
    // intrudes on the trimmed boundary is caught by the final watertight gate.
    Ok(())
}

fn rebuild_face(face: &Face, outer: Wire) -> Result<Face, RollingBallError> {
    // Preserve any inner (hole) loops. A face being trimmed for a blend may own
    // holes — e.g. a top face with a bored pocket — that sit in the interior, far
    // from the edge being filleted. Dropping them would re-fill the holes (and
    // break watertightness against the bore wall). If a hole *does* reach the
    // trimmed boundary, the result fails the watertight/health gate in
    // `fillet_planar_edge` and surfaces as a clean error rather than bad geometry.
    Ok(Face::with_wires(
        face.surface().cloned(),
        Some(outer),
        face.inner_wires(),
        face.orientation(),
    ))
}

pub(crate) fn contact_point_for_spine_vertex(spine: &Edge, contact: &Edge, point: Pnt) -> Pnt {
    let spine_start = spine.source().point();
    if point.distance(&spine_start) <= 10.0 * tolerance::CONFUSION {
        contact.start().point()
    } else {
        contact.end().point()
    }
}

fn contact_point_for_spine_point(
    spine: &Edge,
    contact: &Edge,
    point: Pnt,
) -> Result<Pnt, RollingBallError> {
    let Some(curve) = contact.curve() else {
        return Ok(contact_point_for_spine_vertex(spine, contact, point));
    };
    let t = spine_parameter_for_point(spine, point)?;
    Ok(curve.point(t))
}

fn contact_subedge_for_spine_edge(
    spine: &Edge,
    contact: &Edge,
    edge: &Edge,
) -> Result<Edge, RollingBallError> {
    let t0 = spine_parameter_for_point(spine, edge.source().point())?;
    let t1 = spine_parameter_for_point(spine, edge.target().point())?;
    edge_on_contact_between_params(contact, t0, t1)
}

fn edge_on_contact_between_params(
    contact: &Edge,
    t0: f64,
    t1: f64,
) -> Result<Edge, RollingBallError> {
    let Some(curve) = contact.curve().cloned() else {
        return Ok(Edge::between_points(
            contact.start().point(),
            contact.end().point(),
        ));
    };
    let p0 = curve.point(t0);
    let p1 = curve.point(t1);
    Ok(Edge::new(
        Some(curve),
        t0,
        t1,
        Vertex::new(p0),
        Vertex::new(p1),
    ))
}

fn spine_parameter_for_point(spine: &Edge, point: Pnt) -> Result<f64, RollingBallError> {
    let Some(GeomCurve::Circle(circle)) = spine.curve() else {
        let a = spine.start().point();
        let b = spine.end().point();
        let ab = b - a;
        let len2 = ab.magnitude_squared();
        if len2 <= tolerance::CONFUSION * tolerance::CONFUSION {
            return Err(RollingBallError::DegenerateSpine);
        }
        let t = (point - a).dot(&ab) / len2;
        return Ok(spine.first() + (spine.last() - spine.first()) * t);
    };

    let center = circle.center();
    let x = GeomVec::from_dir(circle.position().x_direction());
    let y = GeomVec::from_dir(circle.axis()).cross(&x);
    let v = point - center;
    let raw = v.dot(&y).atan2(v.dot(&x));
    let lo = spine.first().min(spine.last());
    let hi = spine.first().max(spine.last());
    let mut best = raw;
    let mut best_score = f64::INFINITY;
    for k in -3..=3 {
        let candidate = raw + (k as f64) * core::f64::consts::TAU;
        let score = if candidate < lo {
            lo - candidate
        } else if candidate > hi {
            candidate - hi
        } else {
            0.0
        };
        if score < best_score {
            best = candidate;
            best_score = score;
        }
    }
    Ok(best)
}

pub(crate) fn orient_edge_between(edge: &Edge, start: Pnt, end: Pnt) -> Edge {
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

pub(crate) fn planar_outward_normal(face: &Face) -> Result<Dir, RollingBallError> {
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
    // A rolling-ball blend always sweeps the minor (convex) arc between its two
    // contact points. `angle_about_axis` returns a value in [0, TAU) measured in
    // one fixed rotational sense, so callers that feed `start`/`end` in opposite
    // order get complementary angles — one the 90°-ish convex arc, the other its
    // 270°-ish reflex complement. A `> PI` result means we measured the long way
    // around: flip the axis to swing the short way instead (this also re-points
    // the circle so the curve still runs start -> end). Without this clamp one of
    // the two end arcs of a planar edge blend sweeps the reflex side, producing a
    // cylinder face that renders round at one end and flat (chamfer-like) at the
    // other.
    let mut angle = angle_about_axis(start_v, end_v, axis);
    if angle > core::f64::consts::PI {
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
    fn ordered_clamp_accepts_reversed_surface_bounds() {
        assert_eq!(clamp_ordered(2.12, 2.14, 2.09), 2.12);
        assert_eq!(clamp_ordered(2.00, 2.14, 2.09), 2.09);
        assert_eq!(clamp_ordered(2.20, 2.14, 2.09), 2.14);
    }

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
    fn box_edge_blend_takes_the_minor_arc_on_both_ends() {
        // Regression: `planar_blend` feeds `contact_arc` the two end arcs in
        // opposite point-order (`b -> a` vs `a -> b`). `angle_about_axis` measures
        // in one fixed rotational sense, so without the minor-arc clamp one end
        // sweeps the convex ~90° arc and the other its ~270° reflex complement —
        // a single cylinder face that renders round at one end and flat
        // (chamfer-like) at the other. A convex blend must take the minor
        // (<= PI) arc on *both* ends.
        use core::f64::consts::PI;

        let solid = make_box(&Pnt::origin(), 4.0, 5.0, 6.0);
        let edge = origin_vertical_edge(&solid);
        let blend = rolling_ball_fillet_edge(&solid, &edge, 0.5).unwrap();

        // The arc edges store the sweep as their parameter span (first = 0).
        let start_sweep = blend.start_arc.last() - blend.start_arc.first();
        let end_sweep = blend.end_arc.last() - blend.end_arc.first();
        assert!(
            start_sweep <= PI + 1e-9,
            "start arc swept the reflex side: {start_sweep} rad"
        );
        assert!(
            end_sweep <= PI + 1e-9,
            "end arc swept the reflex side: {end_sweep} rad"
        );
        // A square box edge blends a clean quarter-round on each end.
        assert!(
            (start_sweep - PI / 2.0).abs() < 1e-9,
            "start sweep {start_sweep}"
        );
        assert!((end_sweep - PI / 2.0).abs() < 1e-9, "end sweep {end_sweep}");
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
                RollingBallError::InvalidTopology => "h",
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
                e.start().point().z().abs() < 1e-9
                    && matches!(e.curve(), Some(GeomCurve::Circle(_)))
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
