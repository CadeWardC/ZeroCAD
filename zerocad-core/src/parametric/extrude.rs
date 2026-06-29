use super::*;

pub(crate) fn stamp_sketch_extrude_edge_refs(
    mesh: &mut MockMesh,
    body_id: &str,
    region_index: usize,
    provenance: Option<&RegionProvenance>,
    cs: &CoordinateSystem,
    depth: f32,
) {
    let Some(provenance) = provenance else {
        return;
    };
    if provenance.fragments.is_empty() {
        return;
    }

    let mut occurrences: HashMap<String, usize> = HashMap::new();
    for edge_ref in &mut mesh.edge_refs {
        let role = sketch_extrude_edge_role(edge_ref.p0, edge_ref.p1, cs, depth);
        let fragment_id = sketch_extrude_edge_fragment_id(edge_ref, provenance, cs)
            .unwrap_or_else(|| "unknown".to_string());
        let base_id =
            format!("sketch:{body_id}:region:{region_index}:fragment:{fragment_id}:role:{role}");
        let occurrence = occurrences.entry(base_id.clone()).or_insert(0);
        let edge_id = if *occurrence == 0 {
            base_id
        } else {
            format!("{base_id}:occ:{occurrence}")
        };
        *occurrence += 1;

        let curve_kind = match edge_ref.curve {
            Some(EdgeCurveHint::Circle { .. }) => Some("circle".to_string()),
            Some(EdgeCurveHint::Line) => Some("line".to_string()),
            None => None,
        };
        edge_ref.topology = Some(MeshTopologyEdgeRef {
            body_id: Some(body_id.to_string()),
            topology_version: Some(0),
            edge_id: Some(edge_id),
            curve_kind,
            adjacent_surface_kinds: Vec::new(),
            adjacent_face_ids: Vec::new(),
        });
    }
}

pub(crate) fn sketch_extrude_edge_role(
    p0: [f32; 3],
    p1: [f32; 3],
    cs: &CoordinateSystem,
    depth: f32,
) -> &'static str {
    let offset = |p: [f32; 3]| {
        Vec3::new(p[0], p[1], p[2])
            .sub(cs.origin)
            .dot(cs.n.normalize())
    };
    let a = offset(p0);
    let b = offset(p1);
    let tol = (depth.abs() * 1.0e-3).max(0.02);
    if a.abs() <= tol && b.abs() <= tol {
        "bottom"
    } else if (a - depth).abs() <= tol && (b - depth).abs() <= tol {
        "top"
    } else {
        "side"
    }
}

pub(crate) fn sketch_extrude_edge_fragment_id(
    edge_ref: &crate::mock_kernel::MeshEdgeRef,
    provenance: &RegionProvenance,
    cs: &CoordinateSystem,
) -> Option<String> {
    match edge_ref.curve {
        Some(EdgeCurveHint::Circle { center, radius, .. }) => {
            let center_2d = cs.project(Vec3::new(center[0], center[1], center[2]));
            provenance
                .fragments
                .iter()
                .enumerate()
                .find_map(|(i, fragment)| match fragment {
                    RegionProvenanceFragment::CircleArc {
                        shape_id,
                        center,
                        radius: source_radius,
                    } if (center.0 - center_2d.0).hypot(center.1 - center_2d.1) <= 0.05
                        && (*source_radius - radius).abs() <= 0.05 =>
                    {
                        Some(provenance_fragment_stable_id(i, fragment, *shape_id))
                    }
                    _ => None,
                })
        }
        _ => sketch_extrude_linear_fragment_id(edge_ref, provenance, cs),
    }
}

pub(crate) fn sketch_extrude_linear_fragment_id(
    edge_ref: &crate::mock_kernel::MeshEdgeRef,
    provenance: &RegionProvenance,
    cs: &CoordinateSystem,
) -> Option<String> {
    let mid = [
        (edge_ref.p0[0] + edge_ref.p1[0]) * 0.5,
        (edge_ref.p0[1] + edge_ref.p1[1]) * 0.5,
        (edge_ref.p0[2] + edge_ref.p1[2]) * 0.5,
    ];
    let mid_2d = cs.project(Vec3::new(mid[0], mid[1], mid[2]));

    let mut best_rect: Option<(f32, String)> = None;
    let mut best_circle: Option<(f32, String)> = None;
    let mut raw: Option<String> = None;
    for (i, fragment) in provenance.fragments.iter().enumerate() {
        match fragment {
            RegionProvenanceFragment::RectangleEdge {
                shape_id,
                edge_index,
                rect_min,
                rect_max,
            } => {
                let dist = distance_to_rect_edge(mid_2d, *edge_index, *rect_min, *rect_max);
                if best_rect.as_ref().map_or(true, |(best, _)| dist < *best) {
                    best_rect = Some((dist, provenance_fragment_stable_id(i, fragment, *shape_id)));
                }
            }
            RegionProvenanceFragment::CircleArc {
                shape_id,
                center,
                radius,
            } => {
                let dist = ((mid_2d.0 - center.0).hypot(mid_2d.1 - center.1) - radius).abs();
                if best_circle.as_ref().map_or(true, |(best, _)| dist < *best) {
                    best_circle =
                        Some((dist, provenance_fragment_stable_id(i, fragment, *shape_id)));
                }
            }
            RegionProvenanceFragment::RawPolyline { shape_id }
            | RegionProvenanceFragment::SketchFilletArc { shape_id }
            | RegionProvenanceFragment::SketchChamferEdge { shape_id }
            | RegionProvenanceFragment::Slot { shape_id }
            | RegionProvenanceFragment::RoundedRectangle { shape_id } => {
                raw.get_or_insert_with(|| provenance_fragment_stable_id(i, fragment, *shape_id));
            }
        }
    }

    if let Some((dist, id)) = best_circle {
        if dist <= 0.08 {
            return Some(id);
        }
    }
    if let Some((dist, id)) = best_rect {
        if dist <= 0.08 {
            return Some(id);
        }
    }
    raw
}

pub(crate) fn distance_to_rect_edge(
    p: (f32, f32),
    edge_index: usize,
    rect_min: (f32, f32),
    rect_max: (f32, f32),
) -> f32 {
    match edge_index {
        0 => {
            let x = p.0.clamp(rect_min.0, rect_max.0);
            (p.0 - x).hypot(p.1 - rect_min.1)
        }
        1 => {
            let y = p.1.clamp(rect_min.1, rect_max.1);
            (p.0 - rect_max.0).hypot(p.1 - y)
        }
        2 => {
            let x = p.0.clamp(rect_min.0, rect_max.0);
            (p.0 - x).hypot(p.1 - rect_max.1)
        }
        _ => {
            let y = p.1.clamp(rect_min.1, rect_max.1);
            (p.0 - rect_min.0).hypot(p.1 - y)
        }
    }
}

pub(crate) fn provenance_fragment_stable_id(
    fallback_index: usize,
    fragment: &RegionProvenanceFragment,
    shape_id: Option<usize>,
) -> String {
    let owner = shape_id
        .map(|id| format!("shape:{id}"))
        .unwrap_or_else(|| format!("fragment:{fallback_index}"));
    match fragment {
        RegionProvenanceFragment::RectangleEdge { edge_index, .. } => {
            format!("{owner}:rectangle-edge:{edge_index}")
        }
        RegionProvenanceFragment::CircleArc { .. } => format!("{owner}:circle"),
        RegionProvenanceFragment::SketchFilletArc { .. } => format!("{owner}:sketch-fillet"),
        RegionProvenanceFragment::SketchChamferEdge { .. } => format!("{owner}:sketch-chamfer"),
        RegionProvenanceFragment::Slot { .. } => format!("{owner}:slot"),
        RegionProvenanceFragment::RoundedRectangle { .. } => {
            format!("{owner}:rounded-rectangle")
        }
        RegionProvenanceFragment::RawPolyline { .. } => format!("{owner}:raw-polyline"),
    }
}

/// A body being assembled during evaluation. `parts` are the kernel solids that
/// make it up (more than one only when disjoint lumps share a node); `pristine`
/// holds the analytic mesh while the body is untouched by any boolean, so plain
/// bodies keep their nice hidden-line wireframes. A boolean clears it, forcing a
/// fresh tessellation from `parts`.
#[derive(Debug, Clone)]
pub(crate) struct LiveBody {
    pub(crate) id: String,
    pub(crate) parts: Vec<KernelSolid>,
    pub(crate) pristine: Option<MockMesh>,
    pub(crate) sketch_source: Option<SketchExtrudeSource>,
    pub(crate) cut_tools: Vec<KernelSolid>,
}

#[derive(Debug, Clone)]
pub(crate) struct SketchExtrudeSource {
    pub(crate) regions: Vec<SketchExtrudeRegionSource>,
}

#[derive(Debug, Clone)]
pub(crate) struct SketchExtrudeRegionSource {
    pub(crate) boundary: Vec<(f32, f32)>,
    pub(crate) holes: Vec<Vec<(f32, f32)>>,
    pub(crate) depth: f32,
    pub(crate) cs: CoordinateSystem,
    pub(crate) rect_circle: Option<RectCircleCanonicalSource>,
}

#[derive(Debug, Clone)]
pub(crate) struct RectCircleCanonicalSource {
    pub(crate) base: KernelSolid,
    pub(crate) cutter: KernelSolid,
    pub(crate) body: Option<KernelSolid>,
}

pub(crate) fn rect_circle_region_base_and_cutter_from_sketch(
    curves: &SketchCurves,
    region: &Region,
    depth: f32,
    cs: &CoordinateSystem,
    radius_grow: f32,
) -> Option<(KernelSolid, KernelSolid)> {
    if region.holes.len() > 0 {
        return None;
    }
    let (rect_min, rect_max) = rectangle_bounds_from_source_curves(curves)?;
    let circle = curves.circles.first().copied()?;
    if curves.circles.len() != 1 {
        return None;
    }
    if !circle_intersects_rect_boundary(rect_min, rect_max, circle.center, circle.radius) {
        return None;
    }
    if !region_is_rect_minus_circle_material(
        region,
        rect_min,
        rect_max,
        circle.center,
        circle.radius,
    ) {
        return None;
    }
    crate::mock_kernel::rect_circle_base_and_cutter_from_primitives(
        rect_min,
        rect_max,
        circle.center,
        circle.radius,
        depth,
        cs,
        radius_grow,
    )
}

pub(crate) fn rect_circle_region_base_and_cutter_from_provenance(
    provenance: &RegionProvenance,
    region: &Region,
    depth: f32,
    cs: &CoordinateSystem,
    radius_grow: f32,
) -> Option<(KernelSolid, KernelSolid)> {
    if !region.holes.is_empty() {
        return None;
    }
    let mut rect: Option<((f32, f32), (f32, f32))> = None;
    let mut circle: Option<((f32, f32), f32)> = None;
    for fragment in &provenance.fragments {
        match fragment {
            RegionProvenanceFragment::RectangleEdge {
                rect_min, rect_max, ..
            } => {
                let candidate = (*rect_min, *rect_max);
                if rect.is_none() {
                    rect = Some(candidate);
                } else if rect != Some(candidate) {
                    return None;
                }
            }
            RegionProvenanceFragment::CircleArc { center, radius, .. } => {
                let candidate = (*center, *radius);
                if circle.is_none() {
                    circle = Some(candidate);
                } else if circle != Some(candidate) {
                    return None;
                }
            }
            RegionProvenanceFragment::RawPolyline { .. } => return None,
            _ => {}
        }
    }
    let (rect_min, rect_max) = rect?;
    let (circle_center, circle_radius) = circle?;
    if !circle_intersects_rect_boundary(rect_min, rect_max, circle_center, circle_radius) {
        return None;
    }
    if !region_is_rect_minus_circle_material(
        region,
        rect_min,
        rect_max,
        circle_center,
        circle_radius,
    ) {
        return None;
    }
    crate::mock_kernel::rect_circle_base_and_cutter_from_primitives(
        rect_min,
        rect_max,
        circle_center,
        circle_radius,
        depth,
        cs,
        radius_grow,
    )
}

pub(crate) fn sketch_source_after_circle_cut(
    source: &SketchExtrudeSource,
    circle: Circle,
) -> Option<SketchExtrudeSource> {
    let mut next = source.clone();
    let mut any = false;
    for region in &mut next.regions {
        if !region.holes.is_empty() || region.rect_circle.is_some() {
            continue;
        }
        let (rect_min, rect_max) = loop_bounds_2d(&region.boundary)?;
        if !circle_intersects_rect_boundary(rect_min, rect_max, circle.center, circle.radius) {
            continue;
        }
        let mut provenance_curves = SketchCurves::new();
        provenance_curves.add_rectangle(rect_min, rect_max);
        provenance_curves.add_circle(circle.center, circle.radius);
        if let Some(material_region) = detect_regions(&provenance_curves).into_iter().find(|r| {
            region_is_rect_minus_circle_material(
                r,
                rect_min,
                rect_max,
                circle.center,
                circle.radius,
            )
        }) {
            region.boundary = material_region.boundary;
            region.holes = material_region.holes;
        }
        let Some((base, cutter)) = crate::mock_kernel::rect_circle_base_and_cutter_from_primitives(
            rect_min,
            rect_max,
            circle.center,
            circle.radius,
            region.depth,
            &region.cs,
            0.0,
        ) else {
            continue;
        };
        region.rect_circle = Some(RectCircleCanonicalSource {
            body: crate::mock_kernel::difference(&base, &cutter),
            base,
            cutter,
        });
        any = true;
    }
    any.then_some(next)
}

pub(crate) fn rectangle_bounds_from_source_curves(
    curves: &SketchCurves,
) -> Option<((f32, f32), (f32, f32))> {
    if curves.segments.len() != 4 {
        return None;
    }
    let mut pts: Vec<(f32, f32)> = Vec::new();
    for seg in &curves.segments {
        for p in [seg.a, seg.b] {
            if !pts.iter().any(|q| (q.0 - p.0).hypot(q.1 - p.1) <= 1.0e-4) {
                pts.push(p);
            }
        }
    }
    if pts.len() != 4 {
        return None;
    }

    let (mut min_x, mut min_y) = (f32::INFINITY, f32::INFINITY);
    let (mut max_x, mut max_y) = (f32::NEG_INFINITY, f32::NEG_INFINITY);
    for (x, y) in pts {
        min_x = min_x.min(x);
        min_y = min_y.min(y);
        max_x = max_x.max(x);
        max_y = max_y.max(y);
    }
    if max_x - min_x <= 1.0e-3 || max_y - min_y <= 1.0e-3 {
        return None;
    }

    let has_corner = |p: (f32, f32)| {
        curves
            .segments
            .iter()
            .flat_map(|s| [s.a, s.b])
            .any(|q| (q.0 - p.0).abs() <= 1.0e-4 && (q.1 - p.1).abs() <= 1.0e-4)
    };
    for corner in [
        (min_x, min_y),
        (max_x, min_y),
        (max_x, max_y),
        (min_x, max_y),
    ] {
        if !has_corner(corner) {
            return None;
        }
    }

    let mut sides = [false; 4];
    for seg in &curves.segments {
        let horizontal = (seg.a.1 - seg.b.1).abs() <= 1.0e-4;
        let vertical = (seg.a.0 - seg.b.0).abs() <= 1.0e-4;
        if horizontal {
            if (seg.a.1 - min_y).abs() <= 1.0e-4 {
                sides[0] = true;
            } else if (seg.a.1 - max_y).abs() <= 1.0e-4 {
                sides[2] = true;
            } else {
                return None;
            }
        } else if vertical {
            if (seg.a.0 - max_x).abs() <= 1.0e-4 {
                sides[1] = true;
            } else if (seg.a.0 - min_x).abs() <= 1.0e-4 {
                sides[3] = true;
            } else {
                return None;
            }
        } else {
            return None;
        }
    }
    sides
        .iter()
        .all(|s| *s)
        .then_some(((min_x, min_y), (max_x, max_y)))
}

pub(crate) fn circle_intersects_rect_boundary(
    rect_min: (f32, f32),
    rect_max: (f32, f32),
    center: (f32, f32),
    radius: f32,
) -> bool {
    let ((min_x, min_y), (max_x, max_y)) = ordered_rect(rect_min, rect_max);
    let mut hits = 0usize;
    let eps = 1.0e-4;
    for y in [min_y, max_y] {
        let dy = y - center.1;
        if dy.abs() <= radius + eps {
            let dx2 = radius * radius - dy * dy;
            if dx2 >= -eps {
                let dx = dx2.max(0.0).sqrt();
                for x in [center.0 - dx, center.0 + dx] {
                    if x >= min_x - eps && x <= max_x + eps {
                        hits += 1;
                    }
                }
            }
        }
    }
    for x in [min_x, max_x] {
        let dx = x - center.0;
        if dx.abs() <= radius + eps {
            let dy2 = radius * radius - dx * dx;
            if dy2 >= -eps {
                let dy = dy2.max(0.0).sqrt();
                for y in [center.1 - dy, center.1 + dy] {
                    if y >= min_y - eps && y <= max_y + eps {
                        hits += 1;
                    }
                }
            }
        }
    }
    hits >= 2
}

pub(crate) fn region_is_rect_minus_circle_material(
    region: &Region,
    rect_min: (f32, f32),
    rect_max: (f32, f32),
    center: (f32, f32),
    radius: f32,
) -> bool {
    if region.contains(center) {
        return false;
    }
    let ((min_x, min_y), (max_x, max_y)) = ordered_rect(rect_min, rect_max);
    let rect_area = (max_x - min_x) * (max_y - min_y);
    if region.area <= 1.0e-3 || region.area >= rect_area - 1.0e-3 {
        return false;
    }

    let mut has_material_sample = false;
    let mut has_removed_circle_sample = false;
    for ix in 1..5 {
        for iy in 1..5 {
            let x = min_x + (max_x - min_x) * (ix as f32 / 5.0);
            let y = min_y + (max_y - min_y) * (iy as f32 / 5.0);
            let inside_circle = (x - center.0).hypot(y - center.1) < radius - 0.05;
            let inside_region = region.contains((x, y));
            if inside_region && !inside_circle {
                has_material_sample = true;
            }
            if inside_region && inside_circle {
                has_removed_circle_sample = true;
            }
        }
    }
    has_material_sample && !has_removed_circle_sample
}

pub(crate) fn ordered_rect(a: (f32, f32), b: (f32, f32)) -> ((f32, f32), (f32, f32)) {
    ((a.0.min(b.0), a.1.min(b.1)), (a.0.max(b.0), a.1.max(b.1)))
}

// ---------------------------------------------------------------------------
// Boolean extrude of overlapping sketch shapes
// ---------------------------------------------------------------------------

/// Absolute area of a closed 2D loop (shoelace).
fn loop_area(poly: &[(f32, f32)]) -> f32 {
    let n = poly.len();
    if n < 3 {
        return 0.0;
    }
    let mut s = 0.0f32;
    for i in 0..n {
        let (x0, y0) = poly[i];
        let (x1, y1) = poly[(i + 1) % n];
        s += x0 * y1 - x1 * y0;
    }
    (s * 0.5).abs()
}

/// A point inside a region's actual material — inside the outer boundary and not
/// in any hole. `polygon_interior_point` only sees the outer boundary, so for a
/// holed region (an annulus) its ear-centroid can land in the hole; this falls
/// back to boundary-edge midpoints nudged inward until one is truly inside.
pub(crate) fn region_material_point(region: &Region) -> (f32, f32) {
    let p = crate::sketch::polygon_interior_point(&region.boundary);
    if region.holes.is_empty() || region.contains(p) {
        return p;
    }
    let b = &region.boundary;
    let n = b.len();
    let (mut cx, mut cy) = (0.0f32, 0.0f32);
    for &(x, y) in b {
        cx += x;
        cy += y;
    }
    cx /= n as f32;
    cy /= n as f32;
    for i in 0..n {
        let a = b[i];
        let c = b[(i + 1) % n];
        let mid = ((a.0 + c.0) * 0.5, (a.1 + c.1) * 0.5);
        let q = (mid.0 + (cx - mid.0) * 0.02, mid.1 + (cy - mid.1) * 0.02);
        if region.contains(q) {
            return q;
        }
    }
    p
}

/// Every shape loop whose boundary contains `interior` (all shapes the region
/// belongs to; >1 for an overlap region / lens).
pub(crate) fn region_containing_shapes(interior: (f32, f32), loops: &[ShapeLoop]) -> Vec<usize> {
    loops
        .iter()
        .enumerate()
        .filter(|(_, l)| crate::sketch::point_in_polygon(interior, &l.boundary))
        .map(|(s, _)| s)
        .collect()
}

/// Which shapes the user kept (the base material). Two passes so a shared overlap
/// region doesn't accidentally flip the base onto the wrong (smaller) shape:
///
/// 1. A selected region lying in exactly one shape marks that shape as base.
/// 2. A selected region shared by several shapes marks the **smallest** of them
///    only when none of its shapes is already a base — so selecting the
///    rectangle (whose body region is exclusive) keeps the rectangle the base
///    even if the small overlap lens is also picked, while selecting the inner
///    disk of a circle-in-rectangle still makes the circle the base.
///
/// An empty `region_indices` (whole-sketch extrude) selects every region.
pub(crate) fn selected_shape_mask(
    regions: &[Region],
    region_indices: &[usize],
    loops: &[ShapeLoop],
) -> Vec<bool> {
    let mut mask = vec![false; loops.len()];
    let take_all = region_indices.is_empty();
    let mut shared: Vec<Vec<usize>> = Vec::new();
    for (i, r) in regions.iter().enumerate() {
        if !take_all && !region_indices.contains(&i) {
            continue;
        }
        let interior = region_material_point(r);
        let containing = region_containing_shapes(interior, loops);
        match containing.len() {
            0 => {}
            1 => mask[containing[0]] = true,
            _ => shared.push(containing),
        }
    }
    for containing in shared {
        if containing.iter().any(|&s| mask[s]) {
            continue;
        }
        if let Some(&s) = containing.iter().min_by(|&&a, &&b| {
            loop_area(&loops[a].boundary)
                .partial_cmp(&loop_area(&loops[b].boundary))
                .unwrap_or(std::cmp::Ordering::Equal)
        }) {
            mask[s] = true;
        }
    }
    mask
}

/// Fuse a body's parts so that adjacent/overlapping kept regions of a boolean
/// cluster read as one solid (a unioned cluster), while genuinely disjoint lumps
/// stay separate. A union that fails or that would drop material is skipped —
/// the parts simply stay separate (visually identical for a New Body).
pub(crate) fn fuse_overlapping_solids(parts: Vec<KernelSolid>) -> Vec<KernelSolid> {
    let mut out: Vec<KernelSolid> = Vec::new();
    for part in parts {
        let mut merged = false;
        for existing in out.iter_mut() {
            // Only fuse parts whose AABBs touch — disjoint lumps must not be
            // folded into one (false) solid.
            let touch = match (
                crate::mock_kernel::solid_aabb(existing),
                crate::mock_kernel::solid_aabb(&part),
            ) {
                (Some(a), Some(b)) => crate::mock_kernel::aabbs_overlap(&a, &b, 0.05),
                _ => false,
            };
            if touch {
                if let Some(u) = crate::mock_kernel::union(existing, &part) {
                    *existing = u;
                    merged = true;
                    break;
                }
            }
        }
        if !merged {
            out.push(part);
        }
    }
    out
}
