use super::*;

/// Build the **cutter solid** for a 3D edge fillet/chamfer: subtract it from a
/// body and the sharp edge from `p0` to `p1` (with adjacent outward face normals
/// `n1`, `n2`) becomes a rounded (`fillet`) or beveled corner of size `dist`.
///
/// The cross-section perpendicular to the edge is the corner sliver to remove —
/// a right triangle for a chamfer, or that triangle minus a circular segment
/// (faceted into `segments` chords) for a fillet — swept the length of the edge.
/// It is built by [`extruded_region_solid`] on an edge-aligned frame, so it
/// reuses the same tested, outward-facing prism path the extrude cut uses.
///
/// Two robustness offsets dodge truck's coplanar/tangent-face boolean failures,
/// the same hazards the extrude cut fights (see `directional_cut` / `grow_loop`):
/// * `grow` inflates the whole cross-section outward about its centroid, so the
///   tangent points lift *off* the body faces and the cutter slices through them
///   transversally instead of lying tangent — the configuration truck's solver
///   chokes on. It costs ~`grow`mm of size, so it's used as the fallback cutter.
/// * `end_overshoot` extends the prism past both ends of the edge, so its end
///   caps clear the body's perpendicular faces.
///
/// Returns `None` for a degenerate edge (zero length, `dist <= 0`, or
/// near-parallel face normals that don't define a corner).
#[allow(clippy::too_many_arguments)]
pub fn edge_corner_cutter(
    p0: [f32; 3],
    p1: [f32; 3],
    n1: [f32; 3],
    n2: [f32; 3],
    dist: f32,
    fillet: bool,
    segments: usize,
    grow: f32,
    end_overshoot: f32,
) -> Option<KernelSolid> {
    let profile =
        edge_corner_cutter_profile(p0, p1, n1, n2, dist, fillet, segments, grow, end_overshoot)?;
    // Edge-mod cutters must remain faceted for boolean robustness. Sketch
    // extrusion solids reconstruct arcs into analytic cylinders, which is right
    // for circular cutout bodies but too fragile for subtracting the cutter back
    // into tangent curved walls.
    build_extrusion_solid(
        &profile.loop_pts,
        &[],
        profile.depth as f64,
        &profile.cs,
        false,
    )
}

/// Build a fillet fallback as a fan of convex triangular cutters. Subtracting
/// the pieces one by one is more robust than subtracting one rounded cutter when
/// the selected edge runs into a tangent curved wall.
#[allow(clippy::too_many_arguments)]
pub fn edge_corner_cutter_pieces(
    p0: [f32; 3],
    p1: [f32; 3],
    n1: [f32; 3],
    n2: [f32; 3],
    dist: f32,
    fillet: bool,
    segments: usize,
    grow: f32,
    end_overshoot: f32,
) -> Option<Vec<KernelSolid>> {
    let profile =
        edge_corner_cutter_profile(p0, p1, n1, n2, dist, fillet, segments, grow, end_overshoot)?;

    if !fillet || profile.loop_pts.len() <= 3 {
        return edge_corner_cutter(p0, p1, n1, n2, dist, fillet, segments, grow, end_overshoot)
            .map(|solid| vec![solid]);
    }

    let corner = profile.loop_pts[0];
    let mut pieces = Vec::new();
    for pair in profile.loop_pts[1..].windows(2) {
        let tri = [corner, pair[0], pair[1]];
        let area = ((tri[0].0 * (tri[1].1 - tri[2].1)
            + tri[1].0 * (tri[2].1 - tri[0].1)
            + tri[2].0 * (tri[0].1 - tri[1].1))
            * 0.5)
            .abs();
        if area <= 1.0e-6 {
            continue;
        }
        if let Some(solid) =
            build_extrusion_solid(&tri, &[], profile.depth as f64, &profile.cs, false)
        {
            pieces.push(solid);
        }
    }

    (!pieces.is_empty()).then_some(pieces)
}

pub(crate) struct EdgeCutterProfile {
    pub(crate) loop_pts: Vec<(f32, f32)>,
    pub(crate) cs: crate::geometry::CoordinateSystem,
    pub(crate) depth: f32,
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn edge_corner_cutter_profile(
    p0: [f32; 3],
    p1: [f32; 3],
    n1: [f32; 3],
    n2: [f32; 3],
    dist: f32,
    fillet: bool,
    segments: usize,
    grow: f32,
    end_overshoot: f32,
) -> Option<EdgeCutterProfile> {
    use crate::geometry::{CoordinateSystem, Vec3};

    if dist <= 1.0e-4 {
        return None;
    }
    let p0 = Vec3::new(p0[0], p0[1], p0[2]);
    let p1 = Vec3::new(p1[0], p1[1], p1[2]);
    let n1 = Vec3::new(n1[0], n1[1], n1[2]).normalize();
    let n2 = Vec3::new(n2[0], n2[1], n2[2]).normalize();

    let edge = p1.sub(p0);
    let len = edge.length();
    if len < 1.0e-4 {
        return None;
    }
    let t = edge.mul(1.0 / len);

    // The two face normals must define a real corner (not the same face).
    if n1.cross(n2).length() < 1.0e-3 {
        return None;
    }

    // Into-body directions along each face: the component of the *other* face's
    // inward normal that lies in this face. For a 90° box corner these reduce to
    // `-n2` and `-n1`.
    let f1 = n2.mul(-1.0).sub(n1.mul(n2.mul(-1.0).dot(n1))).normalize();
    let f2 = n1.mul(-1.0).sub(n2.mul(n1.mul(-1.0).dot(n2))).normalize();
    if f1.length() < 0.5 || f2.length() < 0.5 {
        return None;
    }

    // Edge-aligned frame: u = n1 (already ⊥ t, since the edge lies in face 1),
    // v = t × u, so u × v = t = the sweep normal. Start one overshoot behind p0.
    let u_axis = n1.sub(t.mul(n1.dot(t))).normalize();
    let v_axis = t.cross(u_axis).normalize();
    let origin = p0.sub(t.mul(end_overshoot));
    let cs = CoordinateSystem::new(origin, u_axis, v_axis);

    // 2D cross-section coordinates, taken relative to the corner point p0 (the
    // along-edge offset of `origin` is ⊥ u/v, so it doesn't affect these).
    let proj = |pt: Vec3| -> (f32, f32) {
        let d = pt.sub(p0);
        (d.dot(u_axis), d.dot(v_axis))
    };

    let t1 = p0.add(f1.mul(dist)); // tangent point on face 1
    let t2 = p0.add(f2.mul(dist)); // tangent point on face 2
    let t1_2d = proj(t1);
    let t2_2d = proj(t2);
    let corner_2d = (0.0f32, 0.0f32); // the edge itself, projected

    let mut loop_pts: Vec<(f32, f32)> = Vec::new();
    loop_pts.push(corner_2d);
    loop_pts.push(t1_2d);
    if fillet {
        // Faceted quarter-ish arc from T1 to T2, bulging toward the corner. The
        // centre sits one `dist` off each face (exact for a right-angle corner).
        let center = p0.add(f1.mul(dist)).add(f2.mul(dist));
        let c_2d = proj(center);
        let a0 = (t1_2d.1 - c_2d.1).atan2(t1_2d.0 - c_2d.0);
        let a1 = (t2_2d.1 - c_2d.1).atan2(t2_2d.0 - c_2d.0);
        // Sweep the short way (|Δ| ≤ π) so the arc hugs the corner.
        let mut delta = a1 - a0;
        while delta > std::f32::consts::PI {
            delta -= std::f32::consts::TAU;
        }
        while delta < -std::f32::consts::PI {
            delta += std::f32::consts::TAU;
        }
        let r = ((t1_2d.0 - c_2d.0).powi(2) + (t1_2d.1 - c_2d.1).powi(2)).sqrt();
        // Tessellate to ~3.6°/segment so the round reads smooth, capped by
        // `segments` to keep the boolean cutter's face count (and so truck's
        // solver cost/fragility) bounded.
        let steps = ((delta.abs() / 0.063).ceil() as usize).clamp(6, segments.max(6));
        // Interior arc points only (endpoints are T1/T2, already placed).
        for k in 1..steps {
            let a = a0 + delta * (k as f32 / steps as f32);
            loop_pts.push((c_2d.0 + r * a.cos(), c_2d.1 + r * a.sin()));
        }
    }
    loop_pts.push(t2_2d);

    // Wind CCW as seen from +n (= +t) *first*, so the extrusion builder yields an
    // outward-facing solid the boolean accepts — and so the outward edge-offset
    // below pushes the right way.
    let area: f32 = (0..loop_pts.len())
        .map(|i| {
            let (x0, y0) = loop_pts[i];
            let (x1, y1) = loop_pts[(i + 1) % loop_pts.len()];
            x0 * y1 - x1 * y0
        })
        .sum::<f32>()
        * 0.5;
    if area.abs() < 1.0e-6 {
        return None;
    }
    if area < 0.0 {
        loop_pts.reverse();
    }

    // Fallback robustness: inflate the section outward so the tangent points lift
    // off the body faces (no tangency) and the cutter slices through them
    // transversally — the configuration truck's boolean accepts. This is a proper
    // per-edge polygon offset (each edge slid out by `grow`, new vertices at the
    // intersections of consecutive offset edges), NOT a radial scale about the
    // centroid: the fillet's cross-section is *concave* (the arc bulges toward
    // the corner), and a radial scale collapses/self-intersects the arc vertices
    // that sit near the centroid — which is what made filleted bodies come out
    // garbled while chamfers (a convex triangle) were fine.
    if grow > 1.0e-6 {
        loop_pts = offset_polygon_outward(&loop_pts, grow);
    }

    Some(EdgeCutterProfile {
        loop_pts,
        cs,
        depth: len + 2.0 * end_overshoot,
    })
}

/// Offset a simple **CCW** polygon outward by `grow`, the robust way: slide every
/// edge out along its outward normal, then place each new vertex at the
/// intersection of the two consecutive offset edges. Unlike a radial scale about
/// the centroid this stays valid for concave polygons (e.g. the fillet cutter),
/// so it never folds the arc back over its legs.
pub(crate) fn offset_polygon_outward(pts: &[(f32, f32)], grow: f32) -> Vec<(f32, f32)> {
    let n = pts.len();
    if n < 3 {
        return pts.to_vec();
    }
    // Per edge i (pts[i] → pts[i+1]): a point slid out along the outward normal,
    // plus the (unit) edge direction. For a CCW loop the outward (right-hand)
    // normal of direction (dx, dy) is (dy, -dx).
    let mut off_pt = Vec::with_capacity(n);
    let mut dir = Vec::with_capacity(n);
    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        let (mut dx, mut dy) = (b.0 - a.0, b.1 - a.1);
        let l = (dx * dx + dy * dy).sqrt();
        if l < 1.0e-9 {
            dx = 1.0;
            dy = 0.0;
        } else {
            dx /= l;
            dy /= l;
        }
        off_pt.push((a.0 + dy * grow, a.1 - dx * grow));
        dir.push((dx, dy));
    }
    // Each output vertex i is the intersection of offset edge (i-1) and edge i.
    let intersect = |p1: (f32, f32), d1: (f32, f32), p2: (f32, f32), d2: (f32, f32)| {
        let denom = d1.0 * d2.1 - d1.1 * d2.0;
        if denom.abs() < 1.0e-9 {
            return None; // parallel (a straight run) — caller falls back
        }
        let t = ((p2.0 - p1.0) * d2.1 - (p2.1 - p1.1) * d2.0) / denom;
        Some((p1.0 + t * d1.0, p1.1 + t * d1.1))
    };
    // Sanity bound: when two consecutive edges are *nearly* (but not exactly)
    // parallel, `denom` is tiny-but-finite, so `t` blows up and the miter vertex
    // shoots astronomically far away — a spike that corrupts the cutter and
    // leaves stray vertices (and lines) in the boolean result. Only such
    // degenerate (or non-finite) intersections are rejected; every legitimate
    // miter, even on a sharp corner, stays far inside this bound and is untouched.
    const SPIKE_LIMIT: f32 = 1.0e4; // mm
    (0..n)
        .map(|i| {
            let prev = (i + n - 1) % n;
            match intersect(off_pt[prev], dir[prev], off_pt[i], dir[i]) {
                Some(p)
                    if p.0.is_finite()
                        && p.1.is_finite()
                        && p.0.abs() < SPIKE_LIMIT
                        && p.1.abs() < SPIKE_LIMIT =>
                {
                    p
                }
                _ => off_pt[i],
            }
        })
        .collect()
}
