use crate::*;

fn next_id_after<'a>(ids: impl Iterator<Item = &'a str>) -> usize {
    ids.filter_map(|id| id.rsplit('_').next()?.parse::<usize>().ok())
        .max()
        .unwrap_or(0)
        .saturating_add(1)
}

impl ZeroCadApp {
    /// Apply one body hit to the persistent selection. A modified double-click
    /// promotes that body's transient first-click face/edge pick to `Whole`
    /// without clearing other bodies; double-clicking an already-selected body
    /// toggles it off.
    pub(crate) fn select_body_hit(
        &mut self,
        node: String,
        pick: BodyPick,
        is_double: bool,
        multi_select: bool,
    ) {
        if is_double {
            let whole = (node.clone(), BodyPick::Whole);
            let was_selected = self.selected_body.contains(&whole);
            // Egui reports the first half of a double-click as a normal click.
            // Remove that body's temporary face/edge selection before promoting
            // it, otherwise Shift+double-click would leave three selections.
            self.selected_body.retain(|(id, _)| id != &node);
            if !multi_select {
                self.selected_body.clear();
            }
            if !multi_select || !was_selected {
                self.selected_body.insert(whole);
            }
            self.status_msg = if multi_select {
                format!("{} body/bodies selected.", self.selected_body.len())
            } else {
                format!("Selected whole body {node}.")
            };
        } else if multi_select {
            let key = (node, pick);
            if !self.selected_body.insert(key.clone()) {
                self.selected_body.remove(&key);
            }
            self.status_msg = format!("{} element(s) selected.", self.selected_body.len());
        } else {
            self.selected_body.clear();
            self.selected_body.insert((node.clone(), pick));
            self.status_msg = match pick {
                BodyPick::Whole => format!("Selected whole body {node}."),
                BodyPick::Face(face) => {
                    format!("Selected face {face} of {node} (Draw Sketch to sketch on it).")
                }
                BodyPick::Edge(edge) => format!("Selected edge {edge} of {node}."),
                BodyPick::Vertex(vertex) => format!("Selected point {vertex} of {node}."),
            };
        }
    }

    /// The selected body **edges** of a single body, as `(node_id, [edge_index,…])`,
    /// or `None` when no edge is selected. With edges of more than one body
    /// selected, only the first body's edges are returned (a fillet/chamfer feature
    /// targets one body); non-edge picks (faces, points) are ignored. Gates the 3D
    /// fillet/chamfer affordance and drives a multi-edge fillet.
    pub(crate) fn selected_body_edges(&self) -> Option<(String, Vec<u32>)> {
        let mut edges: Vec<(String, u32)> = self
            .selected_body
            .iter()
            .filter_map(|(nid, pick)| match pick {
                BodyPick::Edge(e) => Some((nid.clone(), *e)),
                _ => None,
            })
            .collect();
        // Deterministic: pick the lowest-id body, then its edges in id order, so the
        // primary (anchor) edge and the apply order are stable across frames.
        edges.sort();
        let node = edges.first()?.0.clone();
        let ids: Vec<u32> = edges
            .into_iter()
            .filter(|(n, _)| *n == node)
            .map(|(_, e)| e)
            .collect();
        Some((node, ids))
    }

    /// Expand selected body-edge groups through tangent-continuous neighbors.
    /// This is the CAD "tangent chain" behavior: an analytic sketch fillet arc
    /// pulls in its straight tangent runs, while sharp corners stop the walk.
    pub(crate) fn tangent_edge_chain(&self, node_id: &str, seeds: &[u32]) -> Vec<u32> {
        let Some((_, mesh)) = self.evaluated_scene.find(node_id) else {
            return seeds.to_vec();
        };
        let mut groups: Vec<u32> = mesh.edge_refs.iter().map(|edge| edge.group).collect();
        groups.extend(mesh.edge_groups.iter().copied());
        groups.sort_unstable();
        groups.dedup();
        let candidates: Vec<(u32, EdgeRef)> = groups
            .into_iter()
            .filter_map(|group| {
                Self::edge_ref_from_mesh(node_id, mesh, group).map(|edge| (group, edge))
            })
            .collect();
        expand_tangent_edge_groups(&candidates, seeds)
    }

    /// Read a body edge's world-space geometry (endpoints + the two adjacent
    /// face normals) straight from its wireframe, packaged for an [`EdgeRef`].
    ///
    /// `e` is a topological **edge group** id (see [`BodyPick::Edge`]): the chord
    /// segments of one whole edge. The endpoints returned are the chain's two free
    /// ends — for a straight edge that's its own two corners; for a multi-chord
    /// fillet arc, the arc's ends.
    pub(crate) fn edge_ref_from(&self, node_id: &str, e: u32) -> Option<EdgeRef> {
        let (instance, mesh) = self.evaluated_scene.find(node_id)?;
        let transformed;
        let world_mesh = if instance.placement().is_identity() {
            mesh
        } else {
            transformed = instance.placement().transform_mesh(mesh);
            &transformed
        };
        Self::edge_ref_from_mesh(node_id, world_mesh, e)
    }

    /// Mesh-level body of [`edge_ref_from`], separated so tests can drive the
    /// exact GUI selection pipeline on a real evaluated mesh.
    pub(crate) fn edge_ref_from_mesh(node_id: &str, mesh: &MockMesh, e: u32) -> Option<EdgeRef> {
        if let Some(edge_ref) = mesh.edge_refs.iter().find(|edge_ref| edge_ref.group == e) {
            let topology = edge_ref.topology.as_ref().map(|topology| {
                let mut topology = zerocad_core::TopologyEdgeRef {
                    body_id: topology.body_id.clone(),
                    topology_version: topology.topology_version,
                    edge_id: topology.edge_id.clone(),
                    adjacent_face_ids: topology.adjacent_face_ids.clone(),
                    curve_kind: topology.curve_kind.clone(),
                    adjacent_surface_kinds: topology.adjacent_surface_kinds.clone(),
                    producer_feature_id: topology.producer_feature_id.clone(),
                    source_entity_id: topology.source_entity_id.clone(),
                };
                if topology.body_id.is_none() {
                    topology.body_id = Some(node_id.to_string());
                }
                topology
            });
            return Some(EdgeRef {
                p0: edge_ref.p0,
                p1: edge_ref.p1,
                n1: edge_ref.n1,
                n2: edge_ref.n2,
                curve: edge_ref.curve.clone(),
                topology,
            });
        }
        let seg_count = mesh.edge_indices.len() / 2;

        // Gather the group's chord segments. A legacy mesh without grouping treats
        // `e` as a single raw segment index.
        let segs: Vec<usize> = if mesh.edge_groups.is_empty() {
            if (e as usize) < seg_count {
                vec![e as usize]
            } else {
                return None;
            }
        } else {
            (0..seg_count)
                .filter(|&s| mesh.edge_groups.get(s).copied() == Some(e))
                .collect()
        };
        let &first = segs.first()?;

        let vpos = |seg: usize, which: usize| -> [f32; 3] {
            let vi = mesh.edge_indices[seg * 2 + which] as usize * 3;
            [
                mesh.edge_vertices[vi],
                mesh.edge_vertices[vi + 1],
                mesh.edge_vertices[vi + 2],
            ]
        };
        // A welded endpoint touched by exactly one of the group's chords is a free
        // end of the chain. Two of them bound the edge; a closed loop has none, so
        // fall back to the first chord's endpoints.
        let qkey = |p: [f32; 3]| -> (i64, i64, i64) {
            let q = |v: f32| (v as f64 * 10_000.0).round() as i64;
            (q(p[0]), q(p[1]), q(p[2]))
        };
        let mut uses: HashMap<(i64, i64, i64), (u32, [f32; 3])> = HashMap::new();
        for &s in &segs {
            for w in 0..2 {
                let p = vpos(s, w);
                uses.entry(qkey(p)).or_insert((0, p)).0 += 1;
            }
        }
        let mut ends: Vec<[f32; 3]> = uses
            .values()
            .filter(|(c, _)| *c == 1)
            .map(|(_, p)| *p)
            .collect();
        // Deterministic order so the fillet's speculative precompute key is stable.
        ends.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let closed = ends.len() < 2;
        let (p0, p1) = if ends.len() >= 2 {
            (ends[0], ends[1])
        } else {
            (vpos(first, 0), vpos(first, 1))
        };

        // Adjacent face normals from the first chord (constant along a straight
        // edge; representative for an arc). Absent on legacy meshes → can't orient
        // a cutter, so bail (the user sees the action do nothing).
        let fo = first * 6;
        if mesh.edge_face_normals.len() < fo + 6 {
            return None;
        }
        let mut n1 = [
            mesh.edge_face_normals[fo],
            mesh.edge_face_normals[fo + 1],
            mesh.edge_face_normals[fo + 2],
        ];
        let mut n2 = [
            mesh.edge_face_normals[fo + 3],
            mesh.edge_face_normals[fo + 4],
            mesh.edge_face_normals[fo + 5],
        ];
        // On analytic (smooth-cylinder) tessellations the stored pair can be
        // face-REPRESENTATIVE normals (the wall third's midpoint direction),
        // not local to this chord — which points a rim's "wall" normal at the
        // wrong angle and flips the fillet/chamfer ribbon outward. Prefer the
        // chord's actual two adjacent triangles when they can be found.
        if let Some((t1, t2)) =
            Self::chord_adjacent_triangle_normals(mesh, vpos(first, 0), vpos(first, 1))
        {
            n1 = t1;
            n2 = t2;
        }
        let curve = Self::edge_curve_hint_from_group(mesh, &segs, p0, p1, n1, n2, closed);
        Some(EdgeRef {
            p0,
            p1,
            n1,
            n2,
            curve,
            topology: None,
        })
    }

    /// The geometric normals of the (up to) two triangles sharing the chord
    /// `a`–`b`, matched by quantized vertex position. `None` unless exactly two
    /// distinct-normal triangles touch the chord (open/degenerate meshes fall
    /// back to the stored per-segment normals).
    fn chord_adjacent_triangle_normals(
        mesh: &MockMesh,
        a: [f32; 3],
        b: [f32; 3],
    ) -> Option<([f32; 3], [f32; 3])> {
        let qkey = |p: [f32; 3]| -> (i64, i64, i64) {
            let q = |v: f32| (v as f64 * 10_000.0).round() as i64;
            (q(p[0]), q(p[1]), q(p[2]))
        };
        let (ka, kb) = (qkey(a), qkey(b));
        let vert = |vi: u32| -> [f32; 3] {
            let o = vi as usize * 6;
            [mesh.vertices[o], mesh.vertices[o + 1], mesh.vertices[o + 2]]
        };
        let mut normals: Vec<[f32; 3]> = Vec::new();
        for tri in mesh.indices.chunks_exact(3) {
            let ps = [vert(tri[0]), vert(tri[1]), vert(tri[2])];
            let ks = [qkey(ps[0]), qkey(ps[1]), qkey(ps[2])];
            if !(ks.contains(&ka) && ks.contains(&kb)) {
                continue;
            }
            let u = Vec3::new(
                ps[1][0] - ps[0][0],
                ps[1][1] - ps[0][1],
                ps[1][2] - ps[0][2],
            );
            let v = Vec3::new(
                ps[2][0] - ps[0][0],
                ps[2][1] - ps[0][1],
                ps[2][2] - ps[0][2],
            );
            let n = u.cross(v);
            if n.length() <= 1.0e-9 {
                continue;
            }
            let n = n.normalize();
            let n = [n.x, n.y, n.z];
            // Coplanar neighbors (two cap triangles) count once.
            let dup = normals
                .iter()
                .any(|m| m[0] * n[0] + m[1] * n[1] + m[2] * n[2] > 0.999);
            if !dup {
                normals.push(n);
            }
            if normals.len() > 2 {
                return None;
            }
        }
        (normals.len() == 2).then(|| (normals[0], normals[1]))
    }

    pub(crate) fn edge_curve_hint_from_group(
        mesh: &MockMesh,
        segs: &[usize],
        p0: [f32; 3],
        p1: [f32; 3],
        n1: [f32; 3],
        n2: [f32; 3],
        closed: bool,
    ) -> Option<EdgeCurveHint> {
        let vpos = |seg: usize, which: usize| -> Vec3 {
            let vi = mesh.edge_indices[seg * 2 + which] as usize * 3;
            Vec3::new(
                mesh.edge_vertices[vi],
                mesh.edge_vertices[vi + 1],
                mesh.edge_vertices[vi + 2],
            )
        };
        let mut pts: Vec<Vec3> = Vec::new();
        for &s in segs {
            for w in 0..2 {
                let p = vpos(s, w);
                if !pts.iter().any(|q| q.sub(p).length() <= 1.0e-4) {
                    pts.push(p);
                }
            }
        }
        if pts.len() < 3 {
            return Some(EdgeCurveHint::Line);
        }

        let p0v = Vec3::new(p0[0], p0[1], p0[2]);
        let p1v = Vec3::new(p1[0], p1[1], p1[2]);
        let mut axes = vec![
            Vec3::new(n1[0], n1[1], n1[2]).normalize(),
            Vec3::new(n2[0], n2[1], n2[2]).normalize(),
        ];
        let mut chord_axis = Vec3::ZERO;
        for i in 1..pts.len() {
            for j in (i + 1)..pts.len() {
                let axis = pts[i].sub(pts[0]).cross(pts[j].sub(pts[0]));
                if axis.length() > chord_axis.length() {
                    chord_axis = axis;
                }
            }
        }
        let chord_axis = chord_axis.normalize();
        axes.push(chord_axis);

        let mut best: Option<(f32, EdgeCurveHint)> = None;
        for axis in axes {
            if axis.length() < 0.5 {
                continue;
            }
            if let Some((score, hint)) = Self::fit_circle_hint_on_axis(&pts, p0v, p1v, axis, closed)
            {
                if best
                    .as_ref()
                    .is_none_or(|(best_score, _)| score < *best_score)
                {
                    best = Some((score, hint));
                }
            }
        }
        best.map(|(_, hint)| hint).or(Some(EdgeCurveHint::Line))
    }

    pub(crate) fn fit_circle_hint_on_axis(
        pts: &[Vec3],
        p0: Vec3,
        p1: Vec3,
        axis: Vec3,
        closed: bool,
    ) -> Option<(f32, EdgeCurveHint)> {
        let axis = axis.normalize();
        let base = if axis.dot(Vec3::X).abs() < 0.9 {
            Vec3::X
        } else {
            Vec3::Y
        };
        let u = base.sub(axis.mul(base.dot(axis))).normalize();
        if u.length() < 0.5 {
            return None;
        }
        let v = axis.cross(u).normalize();
        let origin = pts[0];
        let project = |p: Vec3| -> (f32, f32, f32) {
            let d = p.sub(origin);
            (d.dot(u), d.dot(v), d.dot(axis))
        };
        let projected: Vec<(f32, f32)> = pts
            .iter()
            .map(|&p| {
                let (x, y, _) = project(p);
                (x, y)
            })
            .collect();
        if pts
            .iter()
            .map(|&p| project(p).2.abs())
            .fold(0.0f32, f32::max)
            > 0.05
        {
            return None;
        }

        let mut circle = None;
        'outer: for i in 0..projected.len() {
            for j in (i + 1)..projected.len() {
                for k in (j + 1)..projected.len() {
                    if let Some(c) = circle_from_three_2d(projected[i], projected[j], projected[k])
                    {
                        circle = Some(c);
                        break 'outer;
                    }
                }
            }
        }
        let (cx, cy, radius) = circle?;
        if radius <= 1.0e-4 {
            return None;
        }
        let residual = projected
            .iter()
            .map(|&(x, y)| ((x - cx).hypot(y - cy) - radius).abs())
            .fold(0.0f32, f32::max);
        if residual > (0.01 * radius).max(0.05) {
            return None;
        }

        let center = origin.add(u.mul(cx)).add(v.mul(cy));
        let mut x_dir = p0.sub(center).normalize();
        if x_dir.length() < 0.5 {
            x_dir = pts[0].sub(center).normalize();
        }
        x_dir = x_dir.sub(axis.mul(x_dir.dot(axis))).normalize();
        if x_dir.length() < 0.5 {
            return None;
        }
        let y_dir = axis.cross(x_dir).normalize();
        let angle = |p: Vec3| {
            let d = p.sub(center);
            d.dot(y_dir).atan2(d.dot(x_dir))
        };
        let end = if closed {
            std::f32::consts::TAU
        } else {
            let raw_end = angle(p1);
            let forward = normalize_positive(raw_end);
            let reverse = forward - std::f32::consts::TAU;
            let contains = |span_end: f32| -> bool {
                pts.iter()
                    .all(|&p| angle_in_span_f32(angle(p), 0.0, span_end, 0.08))
            };
            if contains(forward) {
                forward
            } else if contains(reverse) {
                reverse
            } else {
                raw_end
            }
        };

        Some((
            residual,
            EdgeCurveHint::Circle {
                center: [center.x, center.y, center.z],
                axis: [axis.x, axis.y, axis.z],
                x_dir: [x_dir.x, x_dir.y, x_dir.z],
                radius,
                start: 0.0,
                end,
                closed,
            },
        ))
    }

    /// Display name for the next 3D fillet/chamfer (Fillet_1, Chamfer_2, …).
    pub(crate) fn next_edge_mod_name(&self, kind: CornerKind) -> String {
        let prefix = match kind {
            CornerKind::Fillet => "Fillet",
            CornerKind::Chamfer => "Chamfer",
        };
        let n = self
            .document
            .graph
            .node_indices()
            .filter(|&i| {
                matches!(
                    self.document.graph[i].feature,
                    FeatureType::EdgeMod { kind: k, .. }
                        | FeatureType::EdgeBlend { kind: k, .. }
                        if k == kind
                )
            })
            .count()
            + 1;
        format!("{}_{}", prefix, n)
    }

    /// Discard the half-drawn shape (placed points + dimension dialog) without
    /// touching the curves already committed to the sketch.
    pub(crate) fn cancel_in_progress_shape(&mut self) {
        self.sketch_temp_start = None;
        self.sketch_points.clear();
        self.sketch_trim_preview = None;
        self.dim_input = None;
        self.dim_screen_positions.clear();
        self.snap_guides.clear();
        self.cursor_snap_guides.clear();
    }

    /// Allocate a fresh unique id suffix and bump the counter.
    pub(crate) fn next_id(&mut self) -> usize {
        let id = self.id_counter;
        self.id_counter += 1;
        id
    }

    /// Advance the shared feature-id counter past every numeric suffix already
    /// present in the current graph. Loaded documents do not persist the GUI
    /// counter, but evaluation uses these suffixes as creation order.
    pub(crate) fn reseed_id_counter_from_graph(&mut self) {
        let next = next_id_after(
            self.document
                .graph
                .node_indices()
                .map(|i| self.document.graph[i].id.as_str()),
        );
        // Never move backwards: undo can restore a graph from before a document
        // load, and ids allocated earlier in this session must remain reserved.
        self.id_counter = self.id_counter.max(next);
    }

    /// True when the viewport is locked to the 2D drawing plane (sketching).
    pub(crate) fn is_planar_view(&self) -> bool {
        self.is_sketch_mode
    }

    /// Animate the camera back to the pre-sketch 3D state.
    pub(crate) fn restore_camera(&mut self, ctx: &egui::Context) {
        self.camera_anim_active = true;
        self.camera_anim_start_pitch = self.camera_pitch;
        self.camera_anim_start_yaw = self.camera_yaw;
        self.camera_anim_target_pitch = self.pre_sketch_pitch;
        self.camera_anim_target_yaw = self.pre_sketch_yaw;
        self.camera_anim_start_time = ctx.input(|i| i.time);
        self.is_perspective = self.pre_sketch_perspective;
    }

    /// The selected region indices belonging to one sketch.
    pub(crate) fn selected_regions_for(&self, sketch_id: &str) -> HashSet<usize> {
        self.selected_faces
            .iter()
            .filter(|(sid, _)| sid == sketch_id)
            .map(|(_, ri)| *ri)
            .collect()
    }

    /// The selected edge indices belonging to one sketch.
    pub(crate) fn selected_edges_for(&self, sketch_id: &str) -> HashSet<usize> {
        self.selected_edges
            .iter()
            .filter(|(sid, _)| sid == sketch_id)
            .map(|(_, ei)| *ei)
            .collect()
    }

    /// The selected visible point-handle indices belonging to one sketch.
    pub(crate) fn selected_sketch_points_for(&self, sketch_id: &str) -> HashSet<usize> {
        self.selected_sketch_points
            .iter()
            .filter(|(sid, _)| sid == sketch_id)
            .map(|(_, point_index)| *point_index)
            .collect()
    }

    /// Pick a finished sketch element under the pointer in Model mode. Point
    /// handles take priority over edges, and edges over filled profiles, matching
    /// the click behavior and the subtle preselection highlight.
    pub(crate) fn pick_finished_sketch_element(
        &self,
        click: egui::Pos2,
        project: &dyn Fn(f32, f32, f32) -> (f32, f32, f32),
    ) -> Option<(String, SketchPick)> {
        const POINT_TOLERANCE: f32 = 8.0;
        const EDGE_TOLERANCE: f32 = 6.0;

        let mut best_point: Option<(String, usize, f32)> = None;
        let mut best_edge: Option<(String, usize, f32)> = None;
        let mut best_face: Option<(String, usize, f32)> = None;
        let variables = self.document.variable_map();

        for index in self.document.graph.node_indices() {
            let node = &self.document.graph[index];
            if self.hidden_nodes.contains(&node.id) {
                continue;
            }
            let FeatureType::Sketch {
                cs,
                curves,
                shapes,
                corner_mods,
                mirrors,
                solver,
                ..
            } = &node.feature
            else {
                continue;
            };
            let curves = zerocad_core::effective_curves_solved(
                curves,
                shapes,
                corner_mods,
                mirrors,
                solver.as_ref(),
                &variables,
            );
            let to_screen = |point: (f32, f32)| {
                let world = cs.unproject(point.0, point.1);
                let projected = project(world.x, world.y, world.z);
                egui::pos2(projected.0, projected.1)
            };

            for (point_index, point) in crate::geom2d::selectable_sketch_points(&curves)
                .into_iter()
                .enumerate()
            {
                let distance = click.distance(to_screen(point));
                if distance < POINT_TOLERANCE
                    && best_point
                        .as_ref()
                        .is_none_or(|candidate| distance < candidate.2)
                {
                    best_point = Some((node.id.clone(), point_index, distance));
                }
            }

            for (edge_index, segment) in curves.segments.iter().enumerate() {
                let distance =
                    dist_point_to_segment(click, to_screen(segment.a), to_screen(segment.b));
                if distance < EDGE_TOLERANCE
                    && best_edge
                        .as_ref()
                        .is_none_or(|candidate| distance < candidate.2)
                {
                    best_edge = Some((node.id.clone(), edge_index, distance));
                }
            }

            let circle_offset = curves.segments.len();
            for (circle_index, circle) in curves.circles.iter().enumerate() {
                let distance = (0..=48)
                    .map(|sample| {
                        let angle = std::f32::consts::TAU * sample as f32 / 48.0;
                        to_screen((
                            circle.center.0 + circle.radius * angle.cos(),
                            circle.center.1 + circle.radius * angle.sin(),
                        ))
                    })
                    .collect::<Vec<_>>()
                    .windows(2)
                    .map(|pair| dist_point_to_segment(click, pair[0], pair[1]))
                    .fold(f32::INFINITY, f32::min);
                if distance < EDGE_TOLERANCE
                    && best_edge
                        .as_ref()
                        .is_none_or(|candidate| distance < candidate.2)
                {
                    best_edge = Some((node.id.clone(), circle_offset + circle_index, distance));
                }
            }

            let arc_offset = circle_offset + curves.circles.len();
            for (arc_index, arc) in curves.arcs.iter().enumerate() {
                let distance = crate::geom2d::sample_arc_points(arc, 48)
                    .windows(2)
                    .map(|pair| {
                        dist_point_to_segment(click, to_screen(pair[0]), to_screen(pair[1]))
                    })
                    .fold(f32::INFINITY, f32::min);
                if distance < EDGE_TOLERANCE
                    && best_edge
                        .as_ref()
                        .is_none_or(|candidate| distance < candidate.2)
                {
                    best_edge = Some((node.id.clone(), arc_offset + arc_index, distance));
                }
            }

            let spline_offset = arc_offset + curves.arcs.len();
            for (spline_index, spline) in curves.splines.iter().enumerate() {
                let distance = spline
                    .sampled_points(0.01)
                    .windows(2)
                    .map(|pair| {
                        dist_point_to_segment(click, to_screen(pair[0]), to_screen(pair[1]))
                    })
                    .fold(f32::INFINITY, f32::min);
                if distance < EDGE_TOLERANCE
                    && best_edge
                        .as_ref()
                        .is_none_or(|candidate| distance < candidate.2)
                {
                    best_edge = Some((node.id.clone(), spline_offset + spline_index, distance));
                }
            }

            let mut region_curves = curves;
            if let Some(boundary) = self.document.sketch_face_boundaries.get(node.id.as_str()) {
                region_curves.extend_curves(boundary);
            }
            for (region_index, region) in detect_regions(&region_curves).iter().enumerate() {
                let screen_boundary: Vec<_> =
                    region.boundary.iter().copied().map(to_screen).collect();
                if screen_boundary.len() < 3
                    || !zerocad_core::sketch::point_in_polygon(
                        (click.x, click.y),
                        &screen_boundary
                            .iter()
                            .map(|point| (point.x, point.y))
                            .collect::<Vec<_>>(),
                    )
                {
                    continue;
                }
                let inside_hole = region.holes.iter().any(|hole| {
                    let screen_hole: Vec<_> = hole
                        .iter()
                        .copied()
                        .map(to_screen)
                        .map(|point| (point.x, point.y))
                        .collect();
                    screen_hole.len() >= 3
                        && zerocad_core::sketch::point_in_polygon((click.x, click.y), &screen_hole)
                });
                if inside_hole {
                    continue;
                }
                let depth = region
                    .boundary
                    .iter()
                    .map(|point| {
                        let world = cs.unproject(point.0, point.1);
                        project(world.x, world.y, world.z).2
                    })
                    .sum::<f32>()
                    / region.boundary.len().max(1) as f32;
                if best_face
                    .as_ref()
                    .is_none_or(|candidate| depth > candidate.2)
                {
                    best_face = Some((node.id.clone(), region_index, depth));
                }
            }
        }

        if let Some((node, point, _)) = best_point {
            Some((node, SketchPick::Point(point)))
        } else if let Some((node, edge, _)) = best_edge {
            Some((node, SketchPick::Edge(edge)))
        } else if let Some((node, face, _)) = best_face {
            Some((node, SketchPick::Face(face)))
        } else {
            None
        }
    }

    /// Pick the visible body element under `click`, in priority vertex > edge >
    /// face. `proj` maps world (x,y,z) to (screen_x, screen_y, depth) — larger
    /// depth is nearer the camera. The `sin/cos` are the camera angles, used to
    /// cull back-facing triangles so only visible geometry is pickable.
    ///
    /// `gpu_face` is the GPU pick buffer's answer for this pixel (see
    /// [`ZeroCadApp::gpu_pick_face`]): when it's authoritative
    /// ([`GpuFacePick::Hit`]) the CPU face-id scan is skipped. Vertex and edge
    /// picks keep their CPU pixel-tolerance search, then pass a solid-surface
    /// depth test so an edge hidden behind a nearer face cannot win.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn pick_body_element(
        &self,
        click: egui::Pos2,
        proj: &dyn Fn(f32, f32, f32) -> (f32, f32, f32),
        sin_p: f32,
        cos_p: f32,
        sin_y: f32,
        cos_y: f32,
        gpu_face: gpu_viewport::GpuFacePick,
    ) -> Option<(String, BodyPick)> {
        const VERT_TOL_PX: f32 = 7.0;
        const EDGE_TOL_PX: f32 = 6.0;

        struct ProximityCandidate {
            node: String,
            pick: BodyPick,
            priority: u8,
            distance: f32,
            screen: egui::Pos2,
            depth: f32,
        }

        let mut proximity_candidates = Vec::new();
        let mut best_face: Option<(String, u32, f32)> = None; // (node, face, depth)
        let scan_faces = match gpu_face {
            gpu_viewport::GpuFacePick::Hit(hit) => {
                best_face = hit.map(|(node, fid)| (node, fid, 0.0));
                false
            }
            gpu_viewport::GpuFacePick::Unavailable => true,
        };

        let faces_camera = |n: (f32, f32, f32)| -> bool {
            let rz_n = sin_y * n.0 + cos_y * n.2;
            sin_p * n.1 + cos_p * rz_n > 0.0
        };
        let triangle_depth_at = |p: egui::Pos2,
                                 a: (f32, f32, f32),
                                 b: (f32, f32, f32),
                                 c: (f32, f32, f32)|
         -> Option<f32> {
            let denominator = (b.1 - c.1) * (a.0 - c.0) + (c.0 - b.0) * (a.1 - c.1);
            if denominator.abs() < 1.0e-9 {
                return None;
            }
            let wa = ((b.1 - c.1) * (p.x - c.0) + (c.0 - b.0) * (p.y - c.1)) / denominator;
            let wb = ((c.1 - a.1) * (p.x - c.0) + (a.0 - c.0) * (p.y - c.1)) / denominator;
            let wc = 1.0 - wa - wb;
            (wa >= -1.0e-4 && wb >= -1.0e-4 && wc >= -1.0e-4)
                .then_some(wa * a.2 + wb * b.2 + wc * c.2)
        };
        let closest_segment_sample =
            |p: egui::Pos2, a: (f32, f32, f32), b: (f32, f32, f32)| -> (f32, egui::Pos2, f32) {
                let a2 = egui::pos2(a.0, a.1);
                let b2 = egui::pos2(b.0, b.1);
                let ab = b2 - a2;
                let length_sq = ab.length_sq();
                let t = if length_sq > 1.0e-12 {
                    ((p - a2).dot(ab) / length_sq).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                let screen = a2 + ab * t;
                ((p - screen).length(), screen, a.2 + (b.2 - a.2) * t)
            };

        // Gather every nearby edge and topological vertex first. Visibility is
        // decided after the front surfaces of all bodies have been considered.
        for instance in self.evaluated_scene.instances() {
            let node_id = instance.entity_id();
            let mesh = self.evaluated_scene.mesh(instance);
            let placement = instance.placement();
            if self.hidden_nodes.contains(node_id) {
                continue;
            }

            // Vertices — but only *true topological endpoints*. Within one edge
            // group (a whole fillet arc, a full rim) every interior tessellation
            // chord point is shared by two segments; only a real B-Rep vertex is
            // used once. A closed circle therefore offers no pickable points at
            // all — professional CAD does not snap to phantom points along a
            // smooth rim. Legacy meshes without grouping keep every corner.
            let endpoint_ok: Option<std::collections::HashSet<u32>> =
                if mesh.edge_groups.len() == mesh.edge_indices.len() / 2 {
                    let q = |i: u32| {
                        let b = i as usize * 3;
                        (
                            (mesh.edge_vertices[b] * 1.0e4).round() as i64,
                            (mesh.edge_vertices[b + 1] * 1.0e4).round() as i64,
                            (mesh.edge_vertices[b + 2] * 1.0e4).round() as i64,
                        )
                    };
                    let mut uses: std::collections::HashMap<(u32, (i64, i64, i64)), u32> =
                        std::collections::HashMap::new();
                    for (s, g) in mesh.edge_groups.iter().enumerate() {
                        for k in 0..2 {
                            let vi = mesh.edge_indices[s * 2 + k];
                            *uses.entry((*g, q(vi))).or_insert(0) += 1;
                        }
                    }
                    let mut ok = std::collections::HashSet::new();
                    for (s, g) in mesh.edge_groups.iter().enumerate() {
                        for k in 0..2 {
                            let vi = mesh.edge_indices[s * 2 + k];
                            if uses.get(&(*g, q(vi))).copied().unwrap_or(0) == 1 {
                                ok.insert(vi);
                            }
                        }
                    }
                    Some(ok)
                } else {
                    None
                };
            let vcount = mesh.edge_vertices.len() / 3;
            for v in 0..vcount {
                if let Some(ok) = &endpoint_ok {
                    if !ok.contains(&(v as u32)) {
                        continue;
                    }
                }
                let point = placement.transform_point([
                    mesh.edge_vertices[v * 3],
                    mesh.edge_vertices[v * 3 + 1],
                    mesh.edge_vertices[v * 3 + 2],
                ]);
                let p = proj(point[0], point[1], point[2]);
                let d = (egui::pos2(p.0, p.1) - click).length();
                if d < VERT_TOL_PX {
                    proximity_candidates.push(ProximityCandidate {
                        node: node_id.to_string(),
                        pick: BodyPick::Vertex(v as u32),
                        priority: 0,
                        distance: d,
                        screen: egui::pos2(p.0, p.1),
                        depth: p.2,
                    });
                }
            }

            // Edges (wireframe segments).
            let ecount = mesh.edge_indices.len() / 2;
            for e in 0..ecount {
                let i0 = mesh.edge_indices[e * 2] as usize * 3;
                let i1 = mesh.edge_indices[e * 2 + 1] as usize * 3;
                let point_a = placement.transform_point([
                    mesh.edge_vertices[i0],
                    mesh.edge_vertices[i0 + 1],
                    mesh.edge_vertices[i0 + 2],
                ]);
                let point_b = placement.transform_point([
                    mesh.edge_vertices[i1],
                    mesh.edge_vertices[i1 + 1],
                    mesh.edge_vertices[i1 + 2],
                ]);
                let a = proj(point_a[0], point_a[1], point_a[2]);
                let b = proj(point_b[0], point_b[1], point_b[2]);
                let (distance, screen, depth) = closest_segment_sample(click, a, b);
                if distance < EDGE_TOL_PX {
                    // Map the hit chord to its topological edge group, so the whole
                    // curve (a fillet arc, a circular rim) selects as one. Legacy
                    // meshes without grouping fall back to the raw segment index.
                    let g = mesh.edge_groups.get(e).copied().unwrap_or(e as u32);
                    proximity_candidates.push(ProximityCandidate {
                        node: node_id.to_string(),
                        pick: BodyPick::Edge(g),
                        priority: 1,
                        distance,
                        screen,
                        depth,
                    });
                }
            }
        }

        // Scan front-facing triangles once. Besides the CPU face fallback, this
        // supplies projected surface depth at each nearby edge/vertex sample.
        let mut surface_depths = vec![f32::NEG_INFINITY; proximity_candidates.len()];
        let (mut depth_min, mut depth_max) = (f32::INFINITY, f32::NEG_INFINITY);
        if scan_faces || !proximity_candidates.is_empty() {
            for instance in self.evaluated_scene.instances() {
                let node_id = instance.entity_id();
                if self.hidden_nodes.contains(node_id) {
                    continue;
                }
                let mesh = self.evaluated_scene.mesh(instance);
                let placement = instance.placement();
                for t in 0..mesh.indices.len() / 3 {
                    let i0 = mesh.indices[t * 3] as usize * 6;
                    let i1 = mesh.indices[t * 3 + 1] as usize * 6;
                    let i2 = mesh.indices[t * 3 + 2] as usize * 6;
                    let n0 = placement.transform_vector([
                        mesh.vertices[i0 + 3],
                        mesh.vertices[i0 + 4],
                        mesh.vertices[i0 + 5],
                    ]);
                    let n1 = placement.transform_vector([
                        mesh.vertices[i1 + 3],
                        mesh.vertices[i1 + 4],
                        mesh.vertices[i1 + 5],
                    ]);
                    let n2 = placement.transform_vector([
                        mesh.vertices[i2 + 3],
                        mesh.vertices[i2 + 4],
                        mesh.vertices[i2 + 5],
                    ]);
                    let normal = (
                        (n0[0] + n1[0] + n2[0]) / 3.0,
                        (n0[1] + n1[1] + n2[1]) / 3.0,
                        (n0[2] + n1[2] + n2[2]) / 3.0,
                    );
                    if !faces_camera(normal) {
                        continue;
                    }
                    let point0 = placement.transform_point([
                        mesh.vertices[i0],
                        mesh.vertices[i0 + 1],
                        mesh.vertices[i0 + 2],
                    ]);
                    let point1 = placement.transform_point([
                        mesh.vertices[i1],
                        mesh.vertices[i1 + 1],
                        mesh.vertices[i1 + 2],
                    ]);
                    let point2 = placement.transform_point([
                        mesh.vertices[i2],
                        mesh.vertices[i2 + 1],
                        mesh.vertices[i2 + 2],
                    ]);
                    let p0 = proj(point0[0], point0[1], point0[2]);
                    let p1 = proj(point1[0], point1[1], point1[2]);
                    let p2 = proj(point2[0], point2[1], point2[2]);
                    for projected in [p0, p1, p2] {
                        depth_min = depth_min.min(projected.2);
                        depth_max = depth_max.max(projected.2);
                    }

                    if scan_faces {
                        if let Some(depth) = triangle_depth_at(click, p0, p1, p2) {
                            if best_face.as_ref().is_none_or(|b| depth > b.2) {
                                let fid = mesh.face_ids.get(t).copied().unwrap_or(0);
                                best_face = Some((node_id.to_string(), fid, depth));
                            }
                        }
                    }

                    for (candidate, surface_depth) in
                        proximity_candidates.iter().zip(surface_depths.iter_mut())
                    {
                        if let Some(depth) = triangle_depth_at(candidate.screen, p0, p1, p2) {
                            *surface_depth = surface_depth.max(depth);
                        }
                    }
                }
            }
        }

        // Permit self-contact within the same scale-aware tolerance used by the
        // hidden-line renderer, but reject candidates covered by a nearer face.
        let depth_bias = if depth_min.is_finite() && depth_max.is_finite() {
            ((depth_max - depth_min) * 0.01).max(0.02)
        } else {
            0.02
        };
        let visible_candidate = proximity_candidates
            .into_iter()
            .zip(surface_depths)
            .filter(|(candidate, surface_depth)| {
                !surface_depth.is_finite() || *surface_depth <= candidate.depth + depth_bias
            })
            .min_by(|(a, _), (b, _)| {
                a.priority
                    .cmp(&b.priority)
                    .then_with(|| a.distance.total_cmp(&b.distance))
                    .then_with(|| b.depth.total_cmp(&a.depth))
            });

        if let Some((candidate, _)) = visible_candidate {
            Some((candidate.node, candidate.pick))
        } else if let Some((n, f, _)) = best_face {
            Some((n, BodyPick::Face(f)))
        } else {
            None
        }
    }

    /// A short human label for a sketch plane, by its normal.
    pub(crate) fn cs_label(cs: &CoordinateSystem) -> &'static str {
        let n = cs.n;
        let near = |a: f32, b: f32| (a - b).abs() < 1e-3;
        let on_origin =
            cs.origin.x.abs() < 1e-3 && cs.origin.y.abs() < 1e-3 && cs.origin.z.abs() < 1e-3;
        let axis = if near(n.x.abs(), 1.0) {
            "Right (YZ)"
        } else if near(n.y.abs(), 1.0) {
            "Top (XZ)"
        } else if near(n.z.abs(), 1.0) {
            "Front (XY)"
        } else {
            "Face"
        };
        if on_origin {
            axis
        } else {
            "Face"
        }
    }

    /// Whether the body face `(node_id, fid)` is planar — every triangle of the
    /// face shares one normal and every vertex lies on the face's centroid plane.
    /// A sketch needs a flat plane, and co-cylindrical face-id merging means the
    /// stored `surface_kind` can't be trusted (GUI-reconstructed refs have topology
    /// `None`), so this decides purely from the tessellation geometry.
    pub(crate) fn face_is_planar(&self, node_id: &str, fid: u32) -> bool {
        let Some((_, mesh)) = self.evaluated_scene.find(node_id) else {
            return false;
        };
        let ntris = mesh.indices.len() / 3;
        let tri_vertex = |t: usize, k: usize| -> Vec3 {
            let i = mesh.indices[t * 3 + k] as usize * 6;
            Vec3::new(mesh.vertices[i], mesh.vertices[i + 1], mesh.vertices[i + 2])
        };

        // Geometric triangle normals + centroid over this face's triangles.
        let mut normals: Vec<Vec3> = Vec::new();
        let mut centroid = Vec3::ZERO;
        let mut vcount = 0.0f32;
        for t in 0..ntris {
            if mesh.face_ids.get(t).copied() != Some(fid) {
                continue;
            }
            let (a, b, c) = (tri_vertex(t, 0), tri_vertex(t, 1), tri_vertex(t, 2));
            let n = b.sub(a).cross(c.sub(a));
            if n.length() > 1e-9 {
                normals.push(n.normalize());
            }
            for k in 0..3 {
                centroid = centroid.add(tri_vertex(t, k));
                vcount += 1.0;
            }
        }
        if normals.is_empty() || vcount == 0.0 {
            return false;
        }
        centroid = centroid.mul(1.0 / vcount);
        let mut avg = Vec3::ZERO;
        for n in &normals {
            avg = avg.add(*n);
        }
        if avg.length() < 1e-6 {
            return false;
        }
        let avg = avg.normalize();

        // 1) Every triangle normal is nearly parallel to the average (~0.8°).
        if normals.iter().any(|n| n.dot(avg) < 0.9999) {
            return false;
        }
        // 2) Every face vertex lies on the centroid plane.
        for t in 0..ntris {
            if mesh.face_ids.get(t).copied() != Some(fid) {
                continue;
            }
            for k in 0..3 {
                if tri_vertex(t, k).sub(centroid).dot(avg).abs() > 1e-3 {
                    return false;
                }
            }
        }
        true
    }

    /// Build a sketch coordinate system from a body face: origin at the face
    /// centroid, normal = the face's outward normal, with in-plane axes derived
    /// so `u × v == n`. Returns `None` if the face/body isn't found.
    pub(crate) fn face_cs(&self, node_id: &str, fid: u32) -> Option<CoordinateSystem> {
        let (instance, mesh) = self.evaluated_scene.find(node_id)?;
        let placement = instance.placement();
        let ntris = mesh.indices.len() / 3;
        let (mut cx, mut cy, mut cz) = (0.0f32, 0.0f32, 0.0f32);
        let (mut nx, mut ny, mut nz) = (0.0f32, 0.0f32, 0.0f32);
        let mut count = 0.0f32;
        for t in 0..ntris {
            if mesh.face_ids.get(t).copied() != Some(fid) {
                continue;
            }
            for k in 0..3 {
                let i = mesh.indices[t * 3 + k] as usize * 6;
                cx += mesh.vertices[i];
                cy += mesh.vertices[i + 1];
                cz += mesh.vertices[i + 2];
                count += 1.0;
            }
            let i0 = mesh.indices[t * 3] as usize * 6;
            nx += mesh.vertices[i0 + 3];
            ny += mesh.vertices[i0 + 4];
            nz += mesh.vertices[i0 + 5];
        }
        if count == 0.0 {
            return None;
        }
        let origin = placement.transform_point([cx / count, cy / count, cz / count]);
        let normal = placement.transform_vector([nx, ny, nz]);
        let origin = Vec3::new(origin[0], origin[1], origin[2]);
        let n = Vec3::new(normal[0], normal[1], normal[2]).normalize();
        // In-plane axes: u perpendicular to both world-up and n (fall back to
        // world-X if the face is horizontal), v completes the right-handed frame.
        let mut u = Vec3::Y.cross(n);
        if u.length() < 1e-4 {
            u = Vec3::X.cross(n);
        }
        let u = u.normalize();
        let v = n.cross(u).normalize();
        Some(CoordinateSystem::new(origin, u, v))
    }

    /// The boundary loops of body face `(node_id, fid)` — its outer wire plus
    /// any hole rims — projected into `cs`'s 2D plane as line segments, ready
    /// to join a sketch's region detection as reference geometry. Thin wrapper
    /// over the shared kernel extraction (`mesh_face_boundary_2d`) — the SAME
    /// code the evaluator uses to re-derive the outline when the body changes,
    /// so a just-captured boundary and an eval-refreshed one are bit-identical
    /// for an unchanged face. Empty when the face/body is missing.
    pub(crate) fn face_boundary_curves(
        &self,
        node_id: &str,
        fid: u32,
        cs: &CoordinateSystem,
    ) -> SketchCurves {
        let Some((instance, mesh)) = self.evaluated_scene.find(node_id) else {
            return SketchCurves::new();
        };
        let transformed;
        let world_mesh = if instance.placement().is_identity() {
            mesh
        } else {
            transformed = instance.placement().transform_mesh(mesh);
            &transformed
        };
        zerocad_core::mock_kernel::mesh_face_boundary_2d(world_mesh, fid, cs)
    }

    /// The durable [`FaceRef`] for a picked body face, so a sketch placed on it can
    /// re-derive its plane from wherever the face is after the body changes.
    pub(crate) fn face_ref(
        &self,
        node_id: &str,
        fid: u32,
    ) -> Option<zerocad_core::parametric::FaceRef> {
        let (_, mesh) = self.body_meshes.iter().find(|(id, _)| id == node_id)?;
        let f = mesh.face_refs.iter().find(|f| f.face_id == fid)?;
        Some(zerocad_core::parametric::FaceRef {
            centroid: f.centroid,
            normal: f.normal,
            topology: Some(zerocad_core::parametric::TopologyFaceRef {
                body_id: Some(node_id.to_string()),
                component_id: f.topology.as_ref().and_then(|t| t.component_id.clone()),
                topology_version: f.topology.as_ref().and_then(|t| t.topology_version),
                face_id: f.topology.as_ref().and_then(|t| t.face_id.clone()),
                surface_kind: f.topology.as_ref().and_then(|t| t.surface_kind.clone()),
                producer_feature_id: f
                    .topology
                    .as_ref()
                    .and_then(|t| t.producer_feature_id.clone()),
                source_entity_id: f.topology.as_ref().and_then(|t| t.source_entity_id.clone()),
            }),
        })
    }
}

fn normalized(v: [f32; 3]) -> Option<[f32; 3]> {
    let length = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    (length > 1.0e-6).then(|| [v[0] / length, v[1] / length, v[2] / length])
}

fn endpoint_tangent(edge: &EdgeRef, point: [f32; 3]) -> Option<[f32; 3]> {
    match edge.curve.as_ref() {
        Some(EdgeCurveHint::Circle {
            center,
            axis,
            closed,
            ..
        }) if !closed => {
            let radial = [
                point[0] - center[0],
                point[1] - center[1],
                point[2] - center[2],
            ];
            normalized([
                axis[1] * radial[2] - axis[2] * radial[1],
                axis[2] * radial[0] - axis[0] * radial[2],
                axis[0] * radial[1] - axis[1] * radial[0],
            ])
        }
        Some(EdgeCurveHint::Circle { closed: true, .. }) => None,
        _ => normalized([
            edge.p1[0] - edge.p0[0],
            edge.p1[1] - edge.p0[1],
            edge.p1[2] - edge.p0[2],
        ]),
    }
}

fn dist_sq(a: [f32; 3], b: [f32; 3]) -> f32 {
    (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)
}

fn shared_endpoint(a: &EdgeRef, b: &EdgeRef) -> Option<[f32; 3]> {
    const ENDPOINT_TOL_SQ: f32 = 1.0e-6;
    [a.p0, a.p1].into_iter().find(|&pa| {
        [b.p0, b.p1]
            .into_iter()
            .any(|pb| dist_sq(pa, pb) <= ENDPOINT_TOL_SQ)
    })
}

fn shares_named_face(a: &EdgeRef, b: &EdgeRef) -> bool {
    let Some(a_topology) = a.topology.as_ref() else {
        return true;
    };
    let Some(b_topology) = b.topology.as_ref() else {
        return true;
    };
    if a_topology.adjacent_face_ids.is_empty() || b_topology.adjacent_face_ids.is_empty() {
        return true;
    }
    a_topology
        .adjacent_face_ids
        .iter()
        .any(|face| b_topology.adjacent_face_ids.contains(face))
}

fn edges_are_tangent(a: &EdgeRef, b: &EdgeRef) -> bool {
    const COS_TANGENT_LIMIT: f32 = 0.9995; // about 1.8 degrees
    let Some(point) = shared_endpoint(a, b) else {
        return false;
    };
    if !shares_named_face(a, b) {
        return false;
    }
    let (Some(ta), Some(tb)) = (endpoint_tangent(a, point), endpoint_tangent(b, point)) else {
        return false;
    };
    (ta[0] * tb[0] + ta[1] * tb[1] + ta[2] * tb[2]).abs() >= COS_TANGENT_LIMIT
}

fn expand_tangent_edge_groups(candidates: &[(u32, EdgeRef)], seeds: &[u32]) -> Vec<u32> {
    let mut selected: std::collections::HashSet<u32> = seeds.iter().copied().collect();
    // Tangent propagation is contour assistance for an analytic arc. A straight
    // edge is a complete, unambiguous selection by itself; using it as a walk
    // seed can unexpectedly sweep around a rounded pocket and preview several
    // fillets when the user clicked only one rim edge. Explicit multi-selection
    // still selects several straight edges, and selecting the arc still expands
    // through its G1-continuous straight runs.
    let mut frontier: Vec<u32> = seeds
        .iter()
        .copied()
        .filter(|seed| {
            candidates.iter().any(|(id, edge)| {
                id == seed
                    && matches!(
                        edge.curve,
                        Some(EdgeCurveHint::Circle { closed: false, .. })
                    )
            })
        })
        .collect();
    let mut result = Vec::new();
    for &seed in seeds {
        if !result.contains(&seed) {
            result.push(seed);
        }
    }
    while let Some(group) = frontier.pop() {
        let Some((_, edge)) = candidates.iter().find(|(id, _)| *id == group) else {
            continue;
        };
        for (candidate_group, candidate) in candidates {
            if !selected.contains(candidate_group) && edges_are_tangent(edge, candidate) {
                selected.insert(*candidate_group);
                frontier.push(*candidate_group);
                result.push(*candidate_group);
            }
        }
    }
    result
}

#[cfg(test)]
mod id_counter_tests {
    use super::next_id_after;

    #[test]
    fn loaded_graph_ids_reseed_the_shared_counter() {
        let ids = ["origin", "sketch_1", "extrude_2", "datum_plane_17"];
        assert_eq!(next_id_after(ids.into_iter()), 18);
    }

    #[test]
    fn graph_without_numbered_features_starts_at_one() {
        assert_eq!(next_id_after(["origin"].into_iter()), 1);
    }
}

#[cfg(test)]
mod body_selection_tests {
    use crate::{BodyPick, ZeroCadApp};

    #[test]
    fn shift_double_click_keeps_the_first_whole_body_and_promotes_the_second() {
        let mut app = ZeroCadApp::new();
        app.select_body_hit("body_1".to_string(), BodyPick::Face(0), true, false);

        // Egui reports the first click before it reports the completed double-click.
        app.select_body_hit("body_2".to_string(), BodyPick::Face(3), false, true);
        app.select_body_hit("body_2".to_string(), BodyPick::Face(3), true, true);

        assert_eq!(app.selected_body.len(), 2);
        assert!(app
            .selected_body
            .contains(&("body_1".to_string(), BodyPick::Whole)));
        assert!(app
            .selected_body
            .contains(&("body_2".to_string(), BodyPick::Whole)));
    }

    #[test]
    fn shift_double_click_toggles_an_already_selected_whole_body() {
        let mut app = ZeroCadApp::new();
        app.select_body_hit("body_1".to_string(), BodyPick::Face(0), true, false);
        app.select_body_hit("body_2".to_string(), BodyPick::Face(0), true, true);
        app.select_body_hit("body_2".to_string(), BodyPick::Face(0), false, true);
        app.select_body_hit("body_2".to_string(), BodyPick::Face(0), true, true);

        assert_eq!(
            app.selected_body,
            [("body_1".to_string(), BodyPick::Whole)]
                .into_iter()
                .collect()
        );
    }
}

#[cfg(test)]
mod placed_scene_picking_tests {
    use crate::{egui, gpu_viewport, BodyPick, MockMesh, ZeroCadApp};
    use zerocad_core::{EvaluatedScene, SceneInstance, ScenePlacement};

    fn front_square(z: f32) -> MockMesh {
        let mut mesh = MockMesh::empty();
        mesh.vertices = vec![
            -20.0, -20.0, z, 0.0, 0.0, 1.0, 20.0, -20.0, z, 0.0, 0.0, 1.0, 20.0, 20.0, z, 0.0, 0.0,
            1.0, -20.0, 20.0, z, 0.0, 0.0, 1.0,
        ];
        mesh.indices = vec![0, 1, 2, 0, 2, 3];
        mesh.face_ids = vec![7, 7];
        mesh
    }

    fn horizontal_edge(z: f32) -> MockMesh {
        let mut mesh = MockMesh::empty();
        mesh.edge_vertices = vec![-10.0, 0.0, z, 10.0, 0.0, z];
        mesh.edge_indices = vec![0, 1];
        mesh
    }

    #[test]
    fn cpu_fallback_picks_transformed_scene_geometry_not_local_geometry() {
        let mut app = ZeroCadApp::new();
        app.set_body_meshes(vec![("box".to_string(), MockMesh::make_box(2.0, 2.0, 2.0))]);
        let placement = ScenePlacement::from_rotation_translation(
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            [100.0, 0.0, 0.0],
        )
        .unwrap();
        app.replace_evaluated_scene(std::sync::Arc::new(
            EvaluatedScene::from_instances(
                app.body_meshes.clone(),
                vec![SceneInstance::new("placed-box", 0, placement)],
            )
            .unwrap(),
        ));
        let project = |x: f32, y: f32, z: f32| (x, y, z);

        let placed = app.pick_body_element(
            egui::pos2(100.0, 0.0),
            &project,
            0.0,
            1.0,
            0.0,
            1.0,
            gpu_viewport::GpuFacePick::Unavailable,
        );
        assert!(matches!(
            placed,
            Some((ref id, BodyPick::Vertex(_))) if id == "placed-box"
        ));

        let local = app.pick_body_element(
            egui::pos2(0.0, 0.0),
            &project,
            0.0,
            1.0,
            0.0,
            1.0,
            gpu_viewport::GpuFacePick::Unavailable,
        );
        assert_eq!(local, None);
    }

    #[test]
    fn nearer_face_blocks_an_edge_pick_through_the_body() {
        let mut app = ZeroCadApp::new();
        app.set_body_meshes(vec![
            ("hidden-edge".to_string(), horizontal_edge(0.0)),
            ("front-face".to_string(), front_square(10.0)),
        ]);
        let project = |x: f32, y: f32, z: f32| (x, y, z);

        let cpu_pick = app.pick_body_element(
            egui::pos2(0.0, 0.0),
            &project,
            0.0,
            1.0,
            0.0,
            1.0,
            gpu_viewport::GpuFacePick::Unavailable,
        );
        assert_eq!(
            cpu_pick,
            Some(("front-face".to_string(), BodyPick::Face(7)))
        );

        let gpu_pick = app.pick_body_element(
            egui::pos2(0.0, 0.0),
            &project,
            0.0,
            1.0,
            0.0,
            1.0,
            gpu_viewport::GpuFacePick::Hit(Some(("front-face".to_string(), 7))),
        );

        assert_eq!(
            gpu_pick,
            Some(("front-face".to_string(), BodyPick::Face(7)))
        );
    }

    #[test]
    fn edge_in_front_of_the_surface_remains_pickable() {
        let mut app = ZeroCadApp::new();
        app.set_body_meshes(vec![
            ("visible-edge".to_string(), horizontal_edge(12.0)),
            ("front-face".to_string(), front_square(10.0)),
        ]);
        let project = |x: f32, y: f32, z: f32| (x, y, z);

        let pick = app.pick_body_element(
            egui::pos2(0.0, 0.0),
            &project,
            0.0,
            1.0,
            0.0,
            1.0,
            gpu_viewport::GpuFacePick::Hit(Some(("front-face".to_string(), 7))),
        );

        assert_eq!(pick, Some(("visible-edge".to_string(), BodyPick::Edge(0))));
    }
}

#[cfg(test)]
mod tangent_chain_tests {
    use super::expand_tangent_edge_groups;
    use crate::{EdgeCurveHint, EdgeRef};

    fn line(p0: [f32; 3], p1: [f32; 3]) -> EdgeRef {
        EdgeRef {
            p0,
            p1,
            n1: [0.0, 0.0, 1.0],
            n2: [0.0, 1.0, 0.0],
            curve: Some(EdgeCurveHint::Line),
            topology: None,
        }
    }

    fn quarter_circle() -> EdgeRef {
        EdgeRef {
            p0: [1.0, 0.0, 0.0],
            p1: [0.0, 1.0, 0.0],
            n1: [0.0, 0.0, 1.0],
            n2: [0.0, 1.0, 0.0],
            curve: Some(EdgeCurveHint::Circle {
                center: [0.0, 0.0, 0.0],
                axis: [0.0, 0.0, 1.0],
                x_dir: [1.0, 0.0, 0.0],
                radius: 1.0,
                start: 0.0,
                end: std::f32::consts::FRAC_PI_2,
                closed: false,
            }),
            topology: None,
        }
    }

    #[test]
    fn rounded_segment_pulls_in_tangent_runs_but_stops_at_sharp_corner() {
        let candidates = vec![
            (0, quarter_circle()),
            (1, line([1.0, -2.0, 0.0], [1.0, 0.0, 0.0])),
            (2, line([0.0, 1.0, 0.0], [-2.0, 1.0, 0.0])),
            (3, line([-2.0, 1.0, 0.0], [-2.0, 3.0, 0.0])),
        ];
        assert_eq!(expand_tangent_edge_groups(&candidates, &[0]), [0, 1, 2]);
    }

    #[test]
    fn straight_segment_does_not_expand_around_rounded_pocket_contour() {
        let candidates = vec![
            (0, quarter_circle()),
            (1, line([1.0, -2.0, 0.0], [1.0, 0.0, 0.0])),
            (2, line([0.0, 1.0, 0.0], [-2.0, 1.0, 0.0])),
        ];
        assert_eq!(expand_tangent_edge_groups(&candidates, &[1]), [1]);
    }
}
