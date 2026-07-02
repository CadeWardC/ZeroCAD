use crate::*;

impl ZeroCadApp {
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

    /// Read a body edge's world-space geometry (endpoints + the two adjacent
    /// face normals) straight from its wireframe, packaged for an [`EdgeRef`].
    ///
    /// `e` is a topological **edge group** id (see [`BodyPick::Edge`]): the chord
    /// segments of one whole edge. The endpoints returned are the chain's two free
    /// ends — for a straight edge that's its own two corners; for a multi-chord
    /// fillet arc, the arc's ends.
    pub(crate) fn edge_ref_from(&self, node_id: &str, e: u32) -> Option<EdgeRef> {
        let (_, mesh) = self.body_meshes.iter().find(|(id, _)| id == node_id)?;
        if let Some(edge_ref) = mesh.edge_refs.iter().find(|edge_ref| edge_ref.group == e) {
            let topology = edge_ref.topology.as_ref().map(|topology| {
                let mut topology = zerocad_core::TopologyEdgeRef {
                    body_id: topology.body_id.clone(),
                    topology_version: topology.topology_version,
                    edge_id: topology.edge_id.clone(),
                    adjacent_face_ids: topology.adjacent_face_ids.clone(),
                    curve_kind: topology.curve_kind.clone(),
                    adjacent_surface_kinds: topology.adjacent_surface_kinds.clone(),
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
        let n1 = [
            mesh.edge_face_normals[fo],
            mesh.edge_face_normals[fo + 1],
            mesh.edge_face_normals[fo + 2],
        ];
        let n2 = [
            mesh.edge_face_normals[fo + 3],
            mesh.edge_face_normals[fo + 4],
            mesh.edge_face_normals[fo + 5],
        ];
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
            .graph
            .graph
            .node_indices()
            .filter(|&i| matches!(self.graph.graph[i].feature, FeatureType::EdgeMod { kind: k, .. } if k == kind))
            .count()
            + 1;
        format!("{}_{}", prefix, n)
    }

    /// Discard the half-drawn shape (placed points + dimension dialog) without
    /// touching the curves already committed to the sketch.
    pub(crate) fn cancel_in_progress_shape(&mut self) {
        self.sketch_temp_start = None;
        self.sketch_points.clear();
        self.dim_input = None;
        self.dim_screen_positions.clear();
    }

    /// Allocate a fresh unique id suffix and bump the counter.
    pub(crate) fn next_id(&mut self) -> usize {
        let id = self.id_counter;
        self.id_counter += 1;
        id
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

    /// Pick the body element under `click`, in priority vertex > edge > face.
    /// `proj` maps world (x,y,z) to (screen_x, screen_y, depth) — larger depth is
    /// nearer the camera. The `sin/cos` are the camera angles, used to cull
    /// back-facing triangles so only visible faces are pickable. Returns the
    /// body node id and which element was hit.
    pub(crate) fn pick_body_element(
        &self,
        click: egui::Pos2,
        proj: &dyn Fn(f32, f32, f32) -> (f32, f32, f32),
        sin_p: f32,
        cos_p: f32,
        sin_y: f32,
        cos_y: f32,
    ) -> Option<(String, BodyPick)> {
        const VERT_TOL_PX: f32 = 7.0;
        const EDGE_TOL_PX: f32 = 6.0;

        let mut best_vertex: Option<(String, u32, f32)> = None; // (node, vert, px)
        let mut best_edge: Option<(String, u32, f32)> = None; // (node, edge GROUP, px)
        let mut best_face: Option<(String, u32, f32)> = None; // (node, face, depth)

        let faces_camera = |n: (f32, f32, f32)| -> bool {
            let rz_n = sin_y * n.0 + cos_y * n.2;
            sin_p * n.1 + cos_p * rz_n > 0.0
        };
        // 2D point-in-triangle via consistent winding sign.
        let point_in_tri = |p: egui::Pos2, a: egui::Pos2, b: egui::Pos2, c: egui::Pos2| -> bool {
            let s = |u: egui::Pos2, v: egui::Pos2, w: egui::Pos2| {
                (v.x - u.x) * (w.y - u.y) - (v.y - u.y) * (w.x - u.x)
            };
            let d1 = s(a, b, p);
            let d2 = s(b, c, p);
            let d3 = s(c, a, p);
            let has_neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
            let has_pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
            !(has_neg && has_pos)
        };

        for (node_id, mesh) in &self.body_meshes {
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
                let p = proj(
                    mesh.edge_vertices[v * 3],
                    mesh.edge_vertices[v * 3 + 1],
                    mesh.edge_vertices[v * 3 + 2],
                );
                let d = (egui::pos2(p.0, p.1) - click).length();
                if d < VERT_TOL_PX && best_vertex.as_ref().map_or(true, |b| d < b.2) {
                    best_vertex = Some((node_id.clone(), v as u32, d));
                }
            }

            // Edges (wireframe segments).
            let ecount = mesh.edge_indices.len() / 2;
            for e in 0..ecount {
                let i0 = mesh.edge_indices[e * 2] as usize * 3;
                let i1 = mesh.edge_indices[e * 2 + 1] as usize * 3;
                let a = proj(
                    mesh.edge_vertices[i0],
                    mesh.edge_vertices[i0 + 1],
                    mesh.edge_vertices[i0 + 2],
                );
                let b = proj(
                    mesh.edge_vertices[i1],
                    mesh.edge_vertices[i1 + 1],
                    mesh.edge_vertices[i1 + 2],
                );
                let d = dist_point_to_segment(click, egui::pos2(a.0, a.1), egui::pos2(b.0, b.1));
                if d < EDGE_TOL_PX && best_edge.as_ref().map_or(true, |b| d < b.2) {
                    // Map the hit chord to its topological edge group, so the whole
                    // curve (a fillet arc, a circular rim) selects as one. Legacy
                    // meshes without grouping fall back to the raw segment index.
                    let g = mesh.edge_groups.get(e).copied().unwrap_or(e as u32);
                    best_edge = Some((node_id.clone(), g, d));
                }
            }

            // Faces (front-facing triangles under the cursor; nearest wins).
            let tcount = mesh.indices.len() / 3;
            for t in 0..tcount {
                let i0 = mesh.indices[t * 3] as usize * 6;
                let i1 = mesh.indices[t * 3 + 1] as usize * 6;
                let i2 = mesh.indices[t * 3 + 2] as usize * 6;
                let n0 = (
                    mesh.vertices[i0 + 3],
                    mesh.vertices[i0 + 4],
                    mesh.vertices[i0 + 5],
                );
                let n1 = (
                    mesh.vertices[i1 + 3],
                    mesh.vertices[i1 + 4],
                    mesh.vertices[i1 + 5],
                );
                let n2 = (
                    mesh.vertices[i2 + 3],
                    mesh.vertices[i2 + 4],
                    mesh.vertices[i2 + 5],
                );
                let normal = (
                    (n0.0 + n1.0 + n2.0) / 3.0,
                    (n0.1 + n1.1 + n2.1) / 3.0,
                    (n0.2 + n1.2 + n2.2) / 3.0,
                );
                if !faces_camera(normal) {
                    continue;
                }
                let p0 = proj(
                    mesh.vertices[i0],
                    mesh.vertices[i0 + 1],
                    mesh.vertices[i0 + 2],
                );
                let p1 = proj(
                    mesh.vertices[i1],
                    mesh.vertices[i1 + 1],
                    mesh.vertices[i1 + 2],
                );
                let p2 = proj(
                    mesh.vertices[i2],
                    mesh.vertices[i2 + 1],
                    mesh.vertices[i2 + 2],
                );
                if point_in_tri(
                    click,
                    egui::pos2(p0.0, p0.1),
                    egui::pos2(p1.0, p1.1),
                    egui::pos2(p2.0, p2.1),
                ) {
                    let depth = (p0.2 + p1.2 + p2.2) / 3.0;
                    if best_face.as_ref().map_or(true, |b| depth > b.2) {
                        let fid = mesh.face_ids.get(t).copied().unwrap_or(0);
                        best_face = Some((node_id.clone(), fid, depth));
                    }
                }
            }
        }

        if let Some((n, v, _)) = best_vertex {
            Some((n, BodyPick::Vertex(v)))
        } else if let Some((n, e, _)) = best_edge {
            Some((n, BodyPick::Edge(e)))
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
        let Some((_, mesh)) = self.body_meshes.iter().find(|(id, _)| id == node_id) else {
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
        let (_, mesh) = self.body_meshes.iter().find(|(id, _)| id == node_id)?;
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
        let origin = Vec3::new(cx / count, cy / count, cz / count);
        let n = Vec3::new(nx, ny, nz).normalize();
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
                topology_version: f.topology.as_ref().and_then(|t| t.topology_version),
                face_id: f.topology.as_ref().and_then(|t| t.face_id.clone()),
                surface_kind: f.topology.as_ref().and_then(|t| t.surface_kind.clone()),
            }),
        })
    }
}
