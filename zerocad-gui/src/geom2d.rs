//! Standalone 2D geometry helpers and sketch rendering. Pure functions with no
//! dependency on the app state, factored out of `main.rs` for readability.

use std::collections::{HashMap, HashSet};

use eframe::egui;
use zerocad_core::{Region, SketchCurves};

/// Draw a sketch's faces, curves, and vertex dots using a caller supplied
/// 2D→screen projection. Shared by the active sketch, finished 2D objects, and
/// the face picker.
///
/// `interactive` brightens the curves (used for the sketch being drawn). Faces
/// in `selected` are highlighted blue and edges in `selected_edges` are
/// highlighted orange; everything else stays faint so picking one element never
/// recolors the whole sketch. Edge indices are `segment i` for i < segment
/// count, else `circle (i - segment count)`.
pub fn draw_sketch_geometry(
    painter: &egui::Painter,
    curves: &SketchCurves,
    regions: &[Region],
    selected: &HashSet<usize>,
    selected_edges: &HashSet<usize>,
    to_screen: &dyn Fn((f32, f32)) -> egui::Pos2,
    interactive: bool,
) {
    // Face fills first so curves overlay them. Concave faces (common once shapes
    // intersect) must be triangulated — `convex_polygon` fans from one vertex
    // and produces the stray-triangle artifacts otherwise.
    for (i, region) in regions.iter().enumerate() {
        let is_sel = selected.contains(&i);
        let (fill, border) = if is_sel {
            (
                egui::Color32::from_rgba_unmultiplied(0, 160, 240, 95),
                egui::Stroke::new(1.8, egui::Color32::from_rgb(0, 120, 210)),
            )
        } else if interactive {
            // Active drawing: light, uniform blue tint.
            (
                egui::Color32::from_rgba_unmultiplied(0, 160, 240, 45),
                egui::Stroke::new(0.8, egui::Color32::from_rgba_unmultiplied(0, 100, 160, 120)),
            )
        } else {
            // Finished, unselected: faint neutral so it reads as "white".
            (
                egui::Color32::from_rgba_unmultiplied(150, 170, 190, 22),
                egui::Stroke::new(0.8, egui::Color32::from_rgba_unmultiplied(90, 110, 130, 90)),
            )
        };
        let outer: Vec<egui::Pos2> = region.boundary.iter().map(|&p| to_screen(p)).collect();
        let holes: Vec<Vec<egui::Pos2>> = region
            .holes
            .iter()
            .map(|h| h.iter().map(|&p| to_screen(p)).collect())
            .collect();

        fill_polygon_with_holes(painter, &outer, &holes, fill);

        // Borders: outer loop plus each hole loop.
        let draw_loop = |loop_pts: &[egui::Pos2]| {
            if loop_pts.len() >= 2 {
                for k in 0..loop_pts.len() {
                    painter.line_segment([loop_pts[k], loop_pts[(k + 1) % loop_pts.len()]], border);
                }
            }
        };
        draw_loop(&outer);
        for h in &holes {
            draw_loop(h);
        }
    }

    let seg_color = if interactive {
        egui::Color32::from_rgb(0, 160, 240)
    } else {
        egui::Color32::from_rgb(90, 110, 130)
    };
    let seg_stroke = egui::Stroke::new(2.0, seg_color);
    // Highlight for a selected edge (orange, thicker).
    let sel_edge_stroke = egui::Stroke::new(3.0, egui::Color32::from_rgb(255, 140, 0));

    let seg_count = curves.segments.len();
    for (i, seg) in curves.segments.iter().enumerate() {
        let stroke = if selected_edges.contains(&i) {
            sel_edge_stroke
        } else {
            seg_stroke
        };
        painter.line_segment([to_screen(seg.a), to_screen(seg.b)], stroke);
    }

    for (j, c) in curves.circles.iter().enumerate() {
        let stroke = if selected_edges.contains(&(seg_count + j)) {
            sel_edge_stroke
        } else {
            seg_stroke
        };
        let mut prev: Option<egui::Pos2> = None;
        for k in 0..=48 {
            let theta = (k as f32 / 48.0) * std::f32::consts::TAU;
            let p = (
                c.center.0 + c.radius * theta.cos(),
                c.center.1 + c.radius * theta.sin(),
            );
            let s = to_screen(p);
            if let Some(last) = prev {
                painter.line_segment([last, s], stroke);
            }
            prev = Some(s);
        }
    }

    let dot_fill = egui::Color32::WHITE;
    let dot_stroke = egui::Stroke::new(1.3, seg_color);
    let draw_dot = |p: (f32, f32)| {
        let s = to_screen(p);
        painter.circle_filled(s, 3.5, dot_fill);
        painter.circle_stroke(s, 3.5, dot_stroke);
    };

    // Vertex handles: draw a dot only at *real* vertices, not at every segment
    // endpoint. A fillet (and an ellipse) is a polyline of many tiny segments, so
    // dotting each one beads the curve and hides its smoothness. Group segment
    // ends by position and skip any vertex where exactly two segments meet
    // near-straight (an arc sample, or the tangent point where a fillet blends
    // into a line) — keeping dots on open ends, junctions, and genuine corners.
    let qkey = |p: (f32, f32)| ((p.0 * 1000.0).round() as i32, (p.1 * 1000.0).round() as i32);
    let mut nodes: HashMap<(i32, i32), ((f32, f32), Vec<(f32, f32)>)> = HashMap::new();
    let mut add_end = |p: (f32, f32), other: (f32, f32)| {
        let d = (other.0 - p.0, other.1 - p.1);
        let len = (d.0 * d.0 + d.1 * d.1).sqrt();
        if len > 1e-6 {
            nodes
                .entry(qkey(p))
                .or_insert((p, Vec::new()))
                .1
                .push((d.0 / len, d.1 / len));
        }
    };
    for seg in &curves.segments {
        add_end(seg.a, seg.b);
        add_end(seg.b, seg.a);
    }
    for (_, (p, dirs)) in &nodes {
        let is_handle = if dirs.len() == 2 {
            // Two outgoing directions: a straight pass-through has them opposite
            // (dot ≈ −1). Keep a dot only once the turn exceeds ~15° (a corner).
            let d = dirs[0].0 * dirs[1].0 + dirs[0].1 * dirs[1].1;
            d > -0.966
        } else {
            true // open endpoint (1) or junction (3+)
        };
        if is_handle {
            draw_dot(*p);
        }
    }
    for c in &curves.circles {
        draw_dot(c.center);
    }
}

/// The circle through three points (center, radius), or `None` if they are
/// (near-)collinear. Used by the 3-point circle tool.
pub fn circumcircle(
    a: (f32, f32),
    b: (f32, f32),
    c: (f32, f32),
) -> Option<((f32, f32), f32)> {
    let d = 2.0 * (a.0 * (b.1 - c.1) + b.0 * (c.1 - a.1) + c.0 * (a.1 - b.1));
    if d.abs() < 1e-6 {
        return None;
    }
    let a2 = a.0 * a.0 + a.1 * a.1;
    let b2 = b.0 * b.0 + b.1 * b.1;
    let c2 = c.0 * c.0 + c.1 * c.1;
    let ux = (a2 * (b.1 - c.1) + b2 * (c.1 - a.1) + c2 * (a.1 - b.1)) / d;
    let uy = (a2 * (c.0 - b.0) + b2 * (a.0 - c.0) + c2 * (b.0 - a.0)) / d;
    let center = (ux, uy);
    let r = ((center.0 - a.0).powi(2) + (center.1 - a.1).powi(2)).sqrt();
    Some((center, r))
}

/// Closest point on segment AB to P (clamped to the segment).
pub fn project_point_on_segment(p: (f32, f32), a: (f32, f32), b: (f32, f32)) -> (f32, f32) {
    let abx = b.0 - a.0;
    let aby = b.1 - a.1;
    let len2 = abx * abx + aby * aby;
    if len2 <= f32::EPSILON {
        return a;
    }
    let t = (((p.0 - a.0) * abx + (p.1 - a.1) * aby) / len2).clamp(0.0, 1.0);
    (a.0 + t * abx, a.1 + t * aby)
}

/// Distance in screen space from point `p` to segment `a`-`b`.
pub fn dist_point_to_segment(p: egui::Pos2, a: egui::Pos2, b: egui::Pos2) -> f32 {
    let proj = project_point_on_segment((p.x, p.y), (a.x, a.y), (b.x, b.y));
    ((p.x - proj.0).powi(2) + (p.y - proj.1).powi(2)).sqrt()
}

fn poly_cross(o: egui::Pos2, a: egui::Pos2, b: egui::Pos2) -> f32 {
    (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)
}

fn point_in_tri(p: egui::Pos2, a: egui::Pos2, b: egui::Pos2, c: egui::Pos2) -> bool {
    let d1 = poly_cross(a, b, p);
    let d2 = poly_cross(b, c, p);
    let d3 = poly_cross(c, a, p);
    let has_neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
    let has_pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
    !(has_neg && has_pos)
}

fn poly_signed_area(pts: &[egui::Pos2]) -> f32 {
    let n = pts.len();
    let mut a = 0.0;
    for i in 0..n {
        let p = pts[i];
        let q = pts[(i + 1) % n];
        a += p.x * q.y - q.x * p.y;
    }
    a * 0.5
}

/// Ear-clip a simple polygon (CCW or CW) into triangles. Robust to mild
/// degeneracy: if no valid ear is found in a pass it force-clips the most
/// convex corner so it always terminates and roughly fills the area.
fn triangulate_simple(pts: &[egui::Pos2]) -> Vec<[egui::Pos2; 3]> {
    let n = pts.len();
    let mut tris = Vec::new();
    if n < 3 {
        return tris;
    }
    let mut idx: Vec<usize> = (0..n).collect();
    if poly_signed_area(pts) < 0.0 {
        idx.reverse();
    }

    let mut guard = 0;
    while idx.len() > 3 && guard < 20000 {
        guard += 1;
        let m = idx.len();
        let mut clipped = false;
        let mut best_convex: Option<(usize, f32)> = None;
        for i in 0..m {
            let ia = idx[(i + m - 1) % m];
            let ib = idx[i];
            let ic = idx[(i + 1) % m];
            let (a, b, c) = (pts[ia], pts[ib], pts[ic]);
            let cr = poly_cross(a, b, c);
            if cr <= 0.0 {
                continue; // reflex/collinear
            }
            // Track the sharpest convex corner as a fallback.
            if best_convex.map_or(true, |(_, bc)| cr > bc) {
                best_convex = Some((i, cr));
            }
            let mut contains = false;
            for &j in &idx {
                if j == ia || j == ib || j == ic {
                    continue;
                }
                if point_in_tri(pts[j], a, b, c) {
                    contains = true;
                    break;
                }
            }
            if contains {
                continue;
            }
            tris.push([a, b, c]);
            idx.remove(i);
            clipped = true;
            break;
        }
        if !clipped {
            // No clean ear (degenerate input). Force progress on the most
            // convex corner to avoid stalling.
            if let Some((i, _)) = best_convex {
                let m = idx.len();
                let a = pts[idx[(i + m - 1) % m]];
                let b = pts[idx[i]];
                let c = pts[idx[(i + 1) % m]];
                tris.push([a, b, c]);
                idx.remove(i);
            } else {
                break;
            }
        }
    }
    if idx.len() == 3 {
        tris.push([pts[idx[0]], pts[idx[1]], pts[idx[2]]]);
    }
    tris
}

/// Splice holes into an outer boundary, producing a single simple polygon with
/// zero-width bridge edges (so it can be ear-clipped). `outer` is forced CCW and
/// holes CW. Standard right-most-vertex bridging.
fn merge_holes(outer: &[egui::Pos2], holes: &[Vec<egui::Pos2>]) -> Vec<egui::Pos2> {
    let mut poly: Vec<egui::Pos2> = outer.to_vec();
    if poly_signed_area(&poly) < 0.0 {
        poly.reverse();
    }

    // Holes as CW loops, processed right-to-left (largest max-x first). The
    // max-x sort key is precomputed once per hole (Schwartzian transform);
    // computing it inside `sort_by` would redo the O(n) fold for every compare.
    let mut hs: Vec<(f32, Vec<egui::Pos2>)> = holes
        .iter()
        .filter(|h| h.len() >= 3)
        .map(|h| {
            let mut hv = h.clone();
            if poly_signed_area(&hv) > 0.0 {
                hv.reverse();
            }
            let max_x = hv.iter().map(|p| p.x).fold(f32::MIN, f32::max);
            (max_x, hv)
        })
        .collect();
    hs.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));

    for (_, hole) in hs {
        let m = match (0..hole.len()).max_by(|&i, &j| {
            hole[i]
                .x
                .partial_cmp(&hole[j].x)
                .unwrap_or(std::cmp::Ordering::Equal)
        }) {
            Some(v) => v,
            None => continue,
        };
        let mp = hole[m];

        // Cast a +x ray from the hole's right-most vertex; find the nearest
        // outer edge crossing and a visible bridge vertex.
        let n = poly.len();
        let mut best_x = f32::INFINITY;
        let mut best_edge: Option<usize> = None;
        let mut hit = egui::pos2(0.0, 0.0);
        for i in 0..n {
            let a = poly[i];
            let b = poly[(i + 1) % n];
            if (a.y > mp.y) != (b.y > mp.y) {
                let t = (mp.y - a.y) / (b.y - a.y);
                let x = a.x + t * (b.x - a.x);
                if x >= mp.x - 1e-3 && x < best_x {
                    best_x = x;
                    best_edge = Some(i);
                    hit = egui::pos2(x, mp.y);
                }
            }
        }
        let e = match best_edge {
            Some(e) => e,
            None => continue, // hole not actually inside; skip
        };
        let a = poly[e];
        let b = poly[(e + 1) % n];
        let (mut bridge, p) = if a.x > b.x { (e, a) } else { ((e + 1) % n, b) };

        // If a reflex vertex sits inside triangle (M, hit, P), bridge to the
        // one closest to the +x axis instead (keeps the bridge inside).
        let mut best_ang = f32::INFINITY;
        for i in 0..n {
            let v = poly[i];
            let prev = poly[(i + n - 1) % n];
            let nxt = poly[(i + 1) % n];
            if poly_cross(prev, v, nxt) >= 0.0 {
                continue; // convex (outer is CCW)
            }
            if point_in_tri(v, mp, hit, p) {
                let ang = (v.y - mp.y).atan2(v.x - mp.x).abs();
                if ang < best_ang {
                    best_ang = ang;
                    bridge = i;
                }
            }
        }

        // Splice: outer[..=bridge] + hole(from m, around, back to m) + outer[bridge..]
        let mut merged = Vec::with_capacity(poly.len() + hole.len() + 2);
        merged.extend_from_slice(&poly[..=bridge]);
        for k in 0..hole.len() {
            merged.push(hole[(m + k) % hole.len()]);
        }
        merged.push(hole[m]);
        merged.push(poly[bridge]);
        merged.extend_from_slice(&poly[bridge + 1..]);
        poly = merged;
    }
    poly
}

/// Fill a polygon that may have holes (e.g. an annulus from a shape drawn inside
/// another). Holes are bridged into the outer boundary, then ear-clipped.
fn fill_polygon_with_holes(
    painter: &egui::Painter,
    outer: &[egui::Pos2],
    holes: &[Vec<egui::Pos2>],
    color: egui::Color32,
) {
    if outer.len() < 3 {
        return;
    }
    let poly = if holes.is_empty() {
        outer.to_vec()
    } else {
        merge_holes(outer, holes)
    };
    // Emit all triangles as ONE mesh. Drawing them as separate `convex_polygon`s
    // makes egui anti-alias (feather) every triangle's edges; along the shared
    // diagonals the translucent fill then blends twice, producing seams that
    // shimmer as the view changes. A single mesh has no internal feathering.
    let mut mesh = egui::Mesh::default();
    for tri in triangulate_simple(&poly) {
        let base = mesh.vertices.len() as u32;
        for v in tri {
            mesh.colored_vertex(v, color);
        }
        mesh.add_triangle(base, base + 1, base + 2);
    }
    if !mesh.is_empty() {
        painter.add(egui::Shape::mesh(mesh));
    }
}

#[cfg(test)]
mod tests {
    use super::circumcircle;

    #[test]
    fn circumcircle_of_unit_axis_points() {
        // Points (1,0), (0,1), (-1,0) lie on the unit circle centered at origin.
        let (c, r) = circumcircle((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0)).unwrap();
        assert!(c.0.abs() < 1e-4 && c.1.abs() < 1e-4, "center should be origin, got {c:?}");
        assert!((r - 1.0).abs() < 1e-4, "radius should be 1, got {r}");
    }

    #[test]
    fn circumcircle_collinear_is_none() {
        assert!(circumcircle((0.0, 0.0), (1.0, 0.0), (2.0, 0.0)).is_none());
    }
}

/// Standalone geometric helper to check if a point lies inside a convex 2D quad
pub fn is_point_in_quad(p: egui::Pos2, quad: &[egui::Pos2; 4]) -> bool {
    let side_test = |p1: egui::Pos2, p2: egui::Pos2, p: egui::Pos2| -> bool {
        (p2.x - p1.x) * (p.y - p1.y) - (p2.y - p1.y) * (p.x - p1.x) >= 0.0
    };

    let b0 = side_test(quad[0], quad[1], p);
    let b1 = side_test(quad[1], quad[2], p);
    let b2 = side_test(quad[2], quad[3], p);
    let b3 = side_test(quad[3], quad[0], p);

    (b0 && b1 && b2 && b3) || (!b0 && !b1 && !b2 && !b3)
}
