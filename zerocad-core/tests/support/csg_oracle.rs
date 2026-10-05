//! Analytic CSG oracle for chains of swept-profile joins and cuts.
//!
//! Every step is a sketch profile (rectangle, circle, polygon, or slot) swept
//! along the normal of a sketch plane, optionally with wall draft. Expected
//! material is evaluated from those primitives directly, so the reference is
//! independent of every kernel path, and a model's result is compared point
//! by point.
#![allow(dead_code)]

use openrcad::foundation::Pnt;
use zerocad_core::*;

/// Keep samples this far from any primitive boundary: membership exactly on a
/// face is tolerance-dependent and not what these tests measure.
pub const BOUNDARY_CLEARANCE: f64 = 0.05;

/// Most vertices a [`Profile::Polygon`] holds.
pub const MAX_POLYGON: usize = 8;

type V3 = [f64; 3];

fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// The sketch plane's orientation. Profile coordinates `(s, t)` are along
/// `(u, v)` and the sweep runs along `u x v`: `(x, y)` for `Z`, `(z, x)` for
/// `Y`, `(y, z)` for `X`, so every frame is right-handed about +axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Axis {
    Z,
    Y,
    X,
    /// An arbitrary plane: `u`, `v` orthonormal, sweeping along `u x v`.
    Tilted {
        u: V3,
        v: V3,
    },
}

impl Axis {
    /// `(u, v, n)` with `n = u x v`.
    pub fn basis(self) -> (V3, V3, V3) {
        let (u, v) = match self {
            Axis::Z => ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
            Axis::Y => ([0.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
            Axis::X => ([0.0, 1.0, 0.0], [0.0, 0.0, 1.0]),
            Axis::Tilted { u, v } => (u, v),
        };
        (u, v, cross(u, v))
    }

    /// The XY frame rotated by `tilt` radians about the horizontal direction
    /// perpendicular to `azimuth`, so the sweep leans toward `azimuth`.
    pub fn tilted(azimuth: f64, tilt: f64) -> Self {
        let axis = [-azimuth.sin(), azimuth.cos(), 0.0];
        let (sin, cos) = tilt.sin_cos();
        // Rodrigues' rotation about the unit `axis`.
        let rotate = |p: V3| {
            let k_cross = cross(axis, p);
            let k_dot = dot(axis, p);
            [0, 1, 2].map(|i| p[i] * cos + k_cross[i] * sin + axis[i] * k_dot * (1.0 - cos))
        };
        Axis::Tilted {
            u: rotate([1.0, 0.0, 0.0]),
            v: rotate([0.0, 1.0, 0.0]),
        }
    }

    /// `(s, t, axial)` coordinates of a world point.
    pub fn split(self, p: V3) -> (f64, f64, f64) {
        let (u, v, n) = self.basis();
        (dot(p, u), dot(p, v), dot(p, n))
    }

    /// The world point at profile coordinates `(s, t)` and height `axial`.
    pub fn point(self, s: f64, t: f64, axial: f64) -> V3 {
        let (u, v, n) = self.basis();
        [0, 1, 2].map(|i| u[i] * s + v[i] * t + n[i] * axial)
    }

    fn frame(self, offset: f64) -> CoordinateSystem {
        let (u, v, n) = self.basis();
        // `+ 0.0` turns -0.0 into 0.0: signed zeros once split sketch faces.
        let vec = |a: V3, scale: f64| {
            Vec3::new(
                (a[0] * scale + 0.0) as f32,
                (a[1] * scale + 0.0) as f32,
                (a[2] * scale + 0.0) as f32,
            )
        };
        CoordinateSystem::new(vec(n, offset), vec(u, 1.0), vec(v, 1.0))
    }
}

#[derive(Clone, Copy, Debug)]
pub enum Profile {
    Rect {
        min: (f64, f64),
        max: (f64, f64),
    },
    Circle {
        center: (f64, f64),
        radius: f64,
    },
    /// A simple counter-clockwise polygon; the first `count` points are used.
    Polygon {
        points: [(f64, f64); MAX_POLYGON],
        count: usize,
    },
    /// The stadium around segment `a`-`b`: two lines and two half circles.
    Slot {
        a: (f64, f64),
        b: (f64, f64),
        radius: f64,
    },
}

fn segment_distance(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let length_squared = dx * dx + dy * dy;
    let along = if length_squared > 0.0 {
        (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / length_squared).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (p.0 - a.0 - dx * along).hypot(p.1 - a.1 - dy * along)
}

impl Profile {
    /// A polygon from any simple vertex list, stored counter-clockwise.
    pub fn polygon(vertices: &[(f64, f64)]) -> Self {
        assert!((3..=MAX_POLYGON).contains(&vertices.len()));
        let mut points = [(0.0, 0.0); MAX_POLYGON];
        points[..vertices.len()].copy_from_slice(vertices);
        let area: f64 = (0..vertices.len())
            .map(|i| {
                let (a, b) = (vertices[i], vertices[(i + 1) % vertices.len()]);
                a.0 * b.1 - b.0 * a.1
            })
            .sum();
        if area < 0.0 {
            points[..vertices.len()].reverse();
        }
        Profile::Polygon {
            points,
            count: vertices.len(),
        }
    }

    fn vertices(&self) -> &[(f64, f64)] {
        match self {
            Profile::Polygon { points, count } => &points[..*count],
            _ => &[],
        }
    }

    /// `((s_min, t_min), (s_max, t_max))` of the undrafted section.
    pub fn bounds(&self) -> ((f64, f64), (f64, f64)) {
        match *self {
            Profile::Rect { min, max } => (min, max),
            Profile::Circle { center, radius } => (
                (center.0 - radius, center.1 - radius),
                (center.0 + radius, center.1 + radius),
            ),
            Profile::Polygon { .. } => self.vertices().iter().fold(
                (
                    (f64::INFINITY, f64::INFINITY),
                    (f64::NEG_INFINITY, f64::NEG_INFINITY),
                ),
                |(lo, hi), p| {
                    (
                        (lo.0.min(p.0), lo.1.min(p.1)),
                        (hi.0.max(p.0), hi.1.max(p.1)),
                    )
                },
            ),
            Profile::Slot { a, b, radius } => (
                (a.0.min(b.0) - radius, a.1.min(b.1) - radius),
                (a.0.max(b.0) + radius, a.1.max(b.1) + radius),
            ),
        }
    }

    /// Whether drafted walls can be modelled: polygonal and convex.
    pub fn supports_draft(&self) -> bool {
        match self {
            Profile::Rect { .. } => true,
            Profile::Polygon { .. } => {
                let vertices = self.vertices();
                (0..vertices.len()).all(|i| {
                    let (a, b, c) = (
                        vertices[i],
                        vertices[(i + 1) % vertices.len()],
                        vertices[(i + 2) % vertices.len()],
                    );
                    (b.0 - a.0) * (c.1 - b.1) - (b.1 - a.1) * (c.0 - b.0) > 0.0
                })
            }
            _ => false,
        }
    }

    /// Signed in-plane distance to the section `inset` inside the profile
    /// (negative inside). Polygons with `inset != 0` (convex only) use the
    /// half-plane form, as rectangles always do: exact inside, at most the
    /// true distance outside, so boundary clearance stays conservative.
    fn signed_distance(&self, s: f64, t: f64, inset: f64) -> f64 {
        match *self {
            Profile::Rect { min, max } => {
                (min.0 - s).max(s - max.0).max(min.1 - t).max(t - max.1) + inset
            }
            Profile::Circle { center, radius } => (s - center.0).hypot(t - center.1) - radius,
            Profile::Slot { a, b, radius } => segment_distance((s, t), a, b) - radius,
            Profile::Polygon { .. } => {
                let vertices = self.vertices();
                let edges =
                    (0..vertices.len()).map(|i| (vertices[i], vertices[(i + 1) % vertices.len()]));
                if inset != 0.0 {
                    // The outward normal of a counter-clockwise edge is (dy, -dx).
                    return edges
                        .map(|(a, b)| {
                            let (dx, dy) = (b.0 - a.0, b.1 - a.1);
                            ((s - a.0) * dy - (t - a.1) * dx) / dx.hypot(dy)
                        })
                        .fold(f64::NEG_INFINITY, f64::max)
                        + inset;
                }
                let mut inside = false;
                let mut nearest = f64::INFINITY;
                for (a, b) in edges {
                    nearest = nearest.min(segment_distance((s, t), a, b));
                    if (a.1 > t) != (b.1 > t) && s < a.0 + (t - a.1) * (b.0 - a.0) / (b.1 - a.1) {
                        inside = !inside;
                    }
                }
                if inside {
                    -nearest
                } else {
                    nearest
                }
            }
        }
    }

    /// Append this profile's curves in sketch (f32) coordinates.
    fn add_to(&self, curves: &mut SketchCurves) {
        let f = |p: (f64, f64)| (p.0 as f32, p.1 as f32);
        match *self {
            Profile::Rect { min, max } => curves.add_rectangle(f(min), f(max)),
            Profile::Circle { center, radius } => curves.add_circle(f(center), radius as f32),
            Profile::Polygon { .. } => {
                let vertices = self.vertices();
                for i in 0..vertices.len() {
                    curves.add_line(f(vertices[i]), f(vertices[(i + 1) % vertices.len()]));
                }
            }
            Profile::Slot { a, b, radius } => {
                let length = (b.0 - a.0).hypot(b.1 - a.1);
                let (dx, dy) = ((b.0 - a.0) / length, (b.1 - a.1) / length);
                // Left of the a -> b direction.
                let side = (-dy * radius, dx * radius);
                let at = |p: (f64, f64), sign: f64| f((p.0 + side.0 * sign, p.1 + side.1 * sign));
                curves.add_line(at(a, -1.0), at(b, -1.0));
                curves.add_line(at(b, 1.0), at(a, 1.0));
                // Each end cap is two quarter arcs through its tip: an exact
                // semicircle's "short way" is ambiguous to `Arc`.
                let tip = |p: (f64, f64), sign: f64| {
                    f((p.0 + dx * radius * sign, p.1 + dy * radius * sign))
                };
                for (center, start, end, outward) in [(b, -1.0, 1.0, 1.0), (a, 1.0, -1.0, -1.0)] {
                    for (from, to) in [
                        (at(center, start), tip(center, outward)),
                        (tip(center, outward), at(center, end)),
                    ] {
                        curves.arcs.push(sketch::Arc {
                            center: f(center),
                            radius: radius as f32,
                            start: from,
                            end: to,
                            clockwise: false,
                        });
                    }
                }
            }
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Step {
    pub label: &'static str,
    pub profile: Profile,
    pub axis: Axis,
    /// Position of the sketch plane along `axis`.
    pub offset: f64,
    /// Signed extrusion along +axis.
    pub depth: f64,
    pub mode: ExtrudeMode,
    /// Wall draft in degrees: positive contracts the far section, as the
    /// extrude feature defines it. Only for [`Profile::supports_draft`].
    pub draft_deg: f64,
}

impl Step {
    pub fn along_z(
        label: &'static str,
        profile: Profile,
        z: f64,
        depth: f64,
        mode: ExtrudeMode,
    ) -> Self {
        Self {
            label,
            profile,
            axis: Axis::Z,
            offset: z,
            depth,
            mode,
            draft_deg: 0.0,
        }
    }

    /// Signed distance to the swept tool: negative inside, positive outside.
    /// Drafted walls are tilted planes, so their in-plane distance is scaled
    /// by the draft's cosine to give the distance to the wall itself.
    pub fn signed_distance(&self, p: [f64; 3]) -> f64 {
        let (s, t, axial) = self.axis.split(p);
        let end = self.offset + self.depth;
        let (lo, hi) = (self.offset.min(end), self.offset.max(end));
        let along = (lo - axial).max(axial - hi);
        let across = if self.draft_deg == 0.0 {
            self.profile.signed_distance(s, t, 0.0)
        } else {
            let (sin, cos) = self.draft_deg.to_radians().sin_cos();
            let height = (axial - self.offset).abs();
            cos * self.profile.signed_distance(s, t, height * sin / cos)
        };
        along.max(across)
    }

    /// The extrude node id this step is built as.
    pub fn extrude_id(index: usize) -> String {
        format!("extrude_{}", 2 * index + 2)
    }
}

/// Expected material at `p`, or `None` when `p` is too close to a boundary of
/// an applied step. Steps listed in `skipped` were rejected and change nothing.
pub fn oracle(steps: &[Step], skipped: &[usize], p: [f64; 3]) -> Option<bool> {
    let mut material = false;
    for (index, step) in steps.iter().enumerate() {
        if skipped.contains(&index) {
            continue;
        }
        let distance = step.signed_distance(p);
        if distance.abs() < BOUNDARY_CLEARANCE {
            return None;
        }
        let inside = distance < 0.0;
        material = match step.mode {
            ExtrudeMode::Cut => material && !inside,
            _ => material || inside,
        };
    }
    Some(material)
}

/// Whether step `index`'s tool shares volume with the material present just
/// before it. A Join into empty space or a Cut through air may be rejected;
/// any other rejection is a missing capability. Samples a lattice over the
/// tool's axis-aligned box, keeping points strictly inside the tool.
pub fn tool_overlaps_material(steps: &[Step], rejected: &[usize], index: usize) -> bool {
    let step = &steps[index];
    let (min, max) = step.profile.bounds();
    // Positive draft only shrinks the section; negative draft grows it by at
    // most the far end's offset.
    let grow = (-step.draft_deg.to_radians().tan() * step.depth.abs()).max(0.0);
    let (s_range, t_range) = ((min.0 - grow, max.0 + grow), (min.1 - grow, max.1 + grow));
    let end = step.offset + step.depth;
    let axial = (step.offset.min(end), step.offset.max(end));
    const N: u32 = 12;
    let at = |(lo, hi): (f64, f64), i: u32| lo + (hi - lo) * (f64::from(i) + 0.5) / f64::from(N);
    for i in 0..N {
        for j in 0..N {
            for k in 0..N {
                let p = step
                    .axis
                    .point(at(s_range, i), at(t_range, j), at(axial, k));
                if step.signed_distance(p) < -BOUNDARY_CLEARANCE
                    && oracle(&steps[..index], rejected, p) == Some(true)
                {
                    return true;
                }
            }
        }
    }
    false
}

pub fn build(steps: &[Step]) -> ParametricGraph {
    let mut graph = ParametricGraph::new();
    for (index, step) in steps.iter().enumerate() {
        let mut curves = SketchCurves::new();
        step.profile.add_to(&mut curves);
        let sketch = format!("sketch_{}", 2 * index + 1);
        let extrude = Step::extrude_id(index);
        graph.add_feature(FeatureNode {
            id: sketch.clone(),
            name: sketch.clone(),
            feature: FeatureType::Sketch {
                entity_ids: vec![],
                next_entity_id: 0,
                solver: None,
                cs: step.axis.frame(step.offset),
                curves,
                shapes: vec![],
                corner_mods: vec![],
                mirrors: vec![],
                on_face: false,
            },
        });
        graph.add_feature(FeatureNode {
            id: extrude.clone(),
            name: extrude.clone(),
            feature: FeatureType::Extrude {
                target: None,
                depth: step.depth as f32,
                region_indices: vec![],
                mode: step.mode,
                depth_expr: None,
                draft_angle_deg: step.draft_deg as f32,
                draft_angle_expr: None,
            },
        });
        graph.add_dependency(&sketch, &extrude);
    }
    graph
}

/// Sampling lattice: `counts` points per axis from `origin` in `spacing` steps.
#[derive(Clone, Copy, Debug)]
pub struct Lattice {
    pub origin: [f64; 3],
    pub spacing: [f64; 3],
    pub counts: [u32; 3],
}

/// What evaluating a chain produced, judged against the oracle.
#[derive(Debug, Default)]
pub struct Outcome {
    /// Steps a warning attributes a rejection to.
    pub rejected: Vec<usize>,
    /// Warnings not attributable to one step.
    pub unattributed: Vec<String>,
    /// Every warning, verbatim, for failure reports.
    pub warnings: Vec<String>,
    pub parts: usize,
    /// Every way the result is wrong: invalid solids or oracle disagreement.
    pub problems: Vec<String>,
    /// Samples where the kernel's `point_in_solid` disagreed with the oracle
    /// but the tessellated boundary agreed: a classifier fault, not geometry.
    pub classifier_disagreements: Vec<[f64; 3]>,
}

/// Generalized winding number of a closed triangle mesh about `p`: about 1
/// inside, about 0 outside (Van Oosterom-Strackee solid angles).
pub fn winding_number(mesh: &MockMesh, p: [f64; 3]) -> f64 {
    let vertex = |index: u32| {
        let offset = index as usize * 6;
        [
            f64::from(mesh.vertices[offset]) - p[0],
            f64::from(mesh.vertices[offset + 1]) - p[1],
            f64::from(mesh.vertices[offset + 2]) - p[2],
        ]
    };
    let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let norm = |a: [f64; 3]| dot(a, a).sqrt();
    let mut total = 0.0;
    for triangle in mesh.indices.chunks_exact(3) {
        let (a, b, c) = (
            vertex(triangle[0]),
            vertex(triangle[1]),
            vertex(triangle[2]),
        );
        let (la, lb, lc) = (norm(a), norm(b), norm(c));
        let cross = [
            b[1] * c[2] - b[2] * c[1],
            b[2] * c[0] - b[0] * c[2],
            b[0] * c[1] - b[1] * c[0],
        ];
        let numerator = dot(a, cross);
        let denominator = la * lb * lc + dot(a, b) * lc + dot(b, c) * la + dot(c, a) * lb;
        total += 2.0 * numerator.atan2(denominator);
    }
    total / (4.0 * std::f64::consts::PI)
}

/// Evaluate `steps` and compare the result with the oracle, treating steps a
/// warning rejects as not applied (a rejected Join or Cut preserves material).
pub fn judge(steps: &[Step], lattice: Lattice) -> Outcome {
    let graph = build(steps);
    let mut outcome = Outcome::default();
    let warnings = match graph.evaluate_bodies_with_warnings(&Default::default()) {
        Ok((_, warnings)) => warnings,
        Err(error) => {
            outcome.problems.push(format!("evaluation failed: {error}"));
            return outcome;
        }
    };
    outcome.warnings = warnings.clone();
    for warning in warnings {
        match (0..steps.len())
            .rev()
            .find(|&index| warning.contains(&format!("'{}'", Step::extrude_id(index))))
        {
            Some(index) => {
                if !outcome.rejected.contains(&index) {
                    outcome.rejected.push(index);
                }
            }
            None => outcome.unattributed.push(warning),
        }
    }
    outcome.rejected.sort_unstable();
    // The export accessor refuses a model with warnings, but a rejected step
    // must still leave correct geometry, so judge the raw kernel solids.
    let bodies = match graph.debug_kernel_solids(&Default::default()) {
        Ok(bodies) => bodies,
        Err(error) => {
            outcome
                .problems
                .push(format!("kernel bodies unavailable: {error}"));
            return outcome;
        }
    };
    let parts: Vec<_> = bodies.iter().flat_map(|(_, parts)| parts).collect();
    outcome.parts = parts.len();
    let policy = Default::default();
    for (index, part) in parts.iter().enumerate() {
        if !part.is_watertight() {
            outcome
                .problems
                .push(format!("part {index} is not watertight"));
        }
        if !part.health_report().is_healthy() {
            outcome
                .problems
                .push(format!("part {index}: {:?}", part.health_report()));
        }
        if let Err(error) = part.validate_strict_with_policy(&policy) {
            outcome
                .problems
                .push(format!("part {index} fails strict validation: {error:?}"));
        }
    }

    // Exact point classification dominates the runtime; a point outside a
    // part's enclosing box cannot be inside that part. `bounding_box` covers
    // vertices only (an arc can bulge past it), so use the conservative box.
    let boxes: Vec<_> = parts
        .iter()
        .map(|part| part.conservative_bounding_box().corners())
        .collect();
    let in_box = |index: usize, p: [f64; 3]| {
        boxes[index].is_none_or(|(lo, hi)| {
            (lo.x()..=hi.x()).contains(&p[0])
                && (lo.y()..=hi.y()).contains(&p[1])
                && (lo.z()..=hi.z()).contains(&p[2])
        })
    };
    // Independent tiebreaker: the generalized winding number of each part's
    // mesh. Chord error only matters within the boundary clearance.
    let meshes: Vec<_> = parts
        .iter()
        .map(|part| MockMesh::from_solid(part))
        .collect();
    let (mut mismatches, mut sampled, mut first) = (0, 0, None);
    for i in 0..lattice.counts[0] {
        for j in 0..lattice.counts[1] {
            for k in 0..lattice.counts[2] {
                let p = [
                    lattice.origin[0] + lattice.spacing[0] * f64::from(i),
                    lattice.origin[1] + lattice.spacing[1] * f64::from(j),
                    lattice.origin[2] + lattice.spacing[2] * f64::from(k),
                ];
                let Some(expected) = oracle(steps, &outcome.rejected, p) else {
                    continue;
                };
                sampled += 1;
                let point = Pnt::new(p[0], p[1], p[2]);
                let actual = parts.iter().enumerate().any(|(index, part)| {
                    in_box(index, p) && openrcad::algo::boolean::point_in_solid(&point, part)
                });
                if actual != expected {
                    let mesh_inside = meshes
                        .iter()
                        .any(|mesh| winding_number(mesh, p).abs() > 0.5);
                    if mesh_inside == expected {
                        outcome.classifier_disagreements.push(p);
                    } else {
                        mismatches += 1;
                        first.get_or_insert((p, expected));
                    }
                }
            }
        }
    }
    if let Some((p, expected)) = first {
        outcome.problems.push(format!(
            "{mismatches}/{sampled} samples disagree with the oracle; first at {p:?} \
             (expected material: {expected})"
        ));
    }
    outcome
}
