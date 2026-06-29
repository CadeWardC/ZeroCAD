use super::*;

/// Recognise a boundary that is (within tolerance) a circle, returning its
/// centre `(u, v)` and radius in sketch-plane coordinates. Requires enough
/// points that it can't be mistaken for a coarse regular polygon the user
/// actually wants faceted, and every point within 2% of the mean radius.
pub(crate) fn circle_profile(points: &[(f32, f32)]) -> Option<(f32, f32, f32)> {
    let n = points.len();
    if n < 12 {
        return None;
    }
    let (mut cx, mut cy) = (0.0f32, 0.0f32);
    for p in points {
        cx += p.0;
        cy += p.1;
    }
    cx /= n as f32;
    cy /= n as f32;
    let dists: Vec<f32> = points
        .iter()
        .map(|p| ((p.0 - cx).powi(2) + (p.1 - cy).powi(2)).sqrt())
        .collect();
    let r = dists.iter().sum::<f32>() / n as f32;
    if r < 1.0e-3 {
        return None;
    }
    dists
        .iter()
        .all(|d| (d - r).abs() <= 0.02 * r)
        .then_some((cx, cy, r))
}

pub(crate) fn loop_bounds_2d(points: &[(f32, f32)]) -> Option<((f32, f32), (f32, f32))> {
    let mut min_x = f32::INFINITY;
    let mut min_y = f32::INFINITY;
    let mut max_x = f32::NEG_INFINITY;
    let mut max_y = f32::NEG_INFINITY;
    let mut any = false;
    for &(x, y) in points {
        any = true;
        min_x = min_x.min(x);
        min_y = min_y.min(y);
        max_x = max_x.max(x);
        max_y = max_y.max(y);
    }
    any.then_some(((min_x, min_y), (max_x, max_y)))
}

pub(crate) fn circle_from_three_points_2d(
    a: (f32, f32),
    b: (f32, f32),
    c: (f32, f32),
) -> Option<(f32, f32, f32)> {
    let ax = a.0 as f64;
    let ay = a.1 as f64;
    let bx = b.0 as f64;
    let by = b.1 as f64;
    let cx = c.0 as f64;
    let cy = c.1 as f64;
    let d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
    if d.abs() <= 1.0e-9 {
        return None;
    }
    let a2 = ax * ax + ay * ay;
    let b2 = bx * bx + by * by;
    let c2 = cx * cx + cy * cy;
    let ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d;
    let uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d;
    let r = ((ux - ax).powi(2) + (uy - ay).powi(2)).sqrt();
    Some((ux as f32, uy as f32, r as f32))
}

pub(crate) fn circle_loop_2d(center: (f32, f32), radius: f32) -> Vec<(f32, f32)> {
    (0..crate::CIRCLE_SEGS)
        .map(|i| {
            let a = (i as f32 / crate::CIRCLE_SEGS as f32) * std::f32::consts::TAU;
            (center.0 + radius * a.cos(), center.1 + radius * a.sin())
        })
        .collect()
}

/// A real cylinder: a circular face of radius `r` centred at `(cu, cv)` on the
/// sketch plane, swept `depth` along the plane normal. Uses the native cylinder
/// primitive so the side is a smooth analytic cylindrical surface (not a prism).
pub(crate) fn oriented_cylinder_solid(
    cs: &crate::geometry::CoordinateSystem,
    cu: f32,
    cv: f32,
    r: f32,
    depth: f32,
) -> Option<KernelSolid> {
    if r <= 0.0 || depth.abs() < f32::EPSILON {
        return None;
    }
    let center = cs.unproject(cu, cv);
    // `make_cylinder` builds the wall along +axis from the base; for a negative
    // sweep, base the cylinder at the far (lower) rim and use the positive height.
    let base = if depth >= 0.0 {
        center
    } else {
        center.add(cs.n.mul(depth))
    };
    let axis = Ax2::new(
        Pnt::new(base.x as f64, base.y as f64, base.z as f64),
        Dir::new(cs.n.x as f64, cs.n.y as f64, cs.n.z as f64),
    );
    Some(make_cylinder(&axis, r as f64, depth.abs() as f64))
}

/// The **smooth native-cylinder** boolean tool for a circular, hole-free region:
/// the same swept volume [`extruded_region_solid`] builds as a 48-gon prism, but
/// with a true analytic cylindrical wall. Returns `None` for non-circular or
/// holed profiles (those stay prisms).
///
/// OpenRCAD's boolean engine resolves smooth cylinder cuts, blind pockets and
/// coplanar boss-unions natively and watertight (see the kernel's
/// `repro_cylinder` tests), so feeding the smooth tool — tried *before* the
/// faceted prism fallback in [`crate::parametric`]'s join/cut assembler — yields
/// a clean round hole / boss instead of the striped, sectioned facet result the
/// old "always a prism" rule produced.
pub fn circular_cylinder_tool(
    boundary: &[(f32, f32)],
    holes: &[Vec<(f32, f32)>],
    depth: f32,
    cs: &crate::geometry::CoordinateSystem,
) -> Option<KernelSolid> {
    if !holes.is_empty() || depth.abs() < f32::EPSILON {
        return None;
    }
    let (cu, cv, r) = circle_profile(boundary)?;
    oriented_cylinder_solid(cs, cu, cv, r, depth)
}

/// Circumcircle `(cx, cy, r)` of three 2D points, or `None` when (near) collinear.
pub(crate) fn circumcircle_2d(
    a: (f64, f64),
    b: (f64, f64),
    c: (f64, f64),
) -> Option<(f64, f64, f64)> {
    let d = 2.0 * (a.0 * (b.1 - c.1) + b.0 * (c.1 - a.1) + c.0 * (a.1 - b.1));
    if d.abs() < 1e-12 {
        return None;
    }
    let a2 = a.0 * a.0 + a.1 * a.1;
    let b2 = b.0 * b.0 + b.1 * b.1;
    let c2 = c.0 * c.0 + c.1 * c.1;
    let ux = (a2 * (b.1 - c.1) + b2 * (c.1 - a.1) + c2 * (a.1 - b.1)) / d;
    let uy = (a2 * (c.0 - b.0) + b2 * (a.0 - c.0) + c2 * (b.0 - a.0)) / d;
    let r = ((ux - a.0).powi(2) + (uy - a.1).powi(2)).sqrt();
    Some((ux, uy, r))
}

/// Exterior turn angle (radians) at `cur` between the incoming and outgoing
/// segments. ~0 on a straight run, large at a polygon corner.
pub(crate) fn turn_angle_2d(prev: (f64, f64), cur: (f64, f64), next: (f64, f64)) -> f64 {
    let v1 = (cur.0 - prev.0, cur.1 - prev.1);
    let v2 = (next.0 - cur.0, next.1 - cur.1);
    let l1 = v1.0.hypot(v1.1);
    let l2 = v2.0.hypot(v2.1);
    if l1 < 1e-12 || l2 < 1e-12 {
        return 0.0;
    }
    (((v1.0 * v2.0 + v1.1 * v2.1) / (l1 * l2)).clamp(-1.0, 1.0)).acos()
}

/// Whether two per-vertex circumcircles describe the same underlying circle
/// (so the vertices between them lie on one arc).
pub(crate) fn same_circle(a: (f64, f64, f64), b: (f64, f64, f64)) -> bool {
    let dc = (a.0 - b.0).hypot(a.1 - b.1);
    let r = a.2.max(b.2);
    dc <= 0.05 * r + 1e-2 && (a.2 - b.2).abs() <= 0.07 * r + 1e-2
}
