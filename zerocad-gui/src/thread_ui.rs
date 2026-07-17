//! Thread tool UI: select a cylindrical body face → choose a standard (Metric /
//! Unified / Custom) + size, internal/external, handedness, and length → commit
//! a `FeatureType::Thread` node. The thread is modeled directly as analytic
//! helical faces (external rods AND tapped holes / bosses); if the wall can't
//! be replaced cleanly the body is left intact (cosmetic fallback).

use crate::*;
use zerocad_core::parametric::{closest_preset, default_depth_mm, FaceRef, ThreadStandard};

/// State of the in-progress thread tool.
#[derive(Debug, Clone)]
pub(crate) struct ThreadOp {
    pub(crate) target: String,
    pub(crate) face: FaceRef,
    /// Geometric frame captured from the selected mesh face. This powers the
    /// cheap cosmetic preview and retains a real point on the picked wall.
    pub(crate) surface: ThreadSurfaceFrame,
    pub(crate) standard: ThreadStandard,
    pub(crate) preset_idx: usize,
    pub(crate) class: String,
    pub(crate) internal: bool,
    pub(crate) right_handed: bool,
    pub(crate) full_length: bool,
    pub(crate) flip: bool,
    pub(crate) length_text: String,
    pub(crate) pitch_text: String,
    pub(crate) depth_text: String,
    pub(crate) angle_text: String,
}

/// A fitted frame for one complete cylindrical mesh face.
#[derive(Debug, Clone)]
pub(crate) struct ThreadSurfaceFrame {
    origin: [f32; 3],
    axis: [f32; 3],
    radial_u: [f32; 3],
    radial_v: [f32; 3],
    radius: f32,
    axial_min: f32,
    axial_max: f32,
    /// Direction from the material surface into empty space: +1 on an outside
    /// wall and -1 on a hole wall.
    empty_side: f32,
    anchor: [f32; 3],
}

impl ThreadOp {
    /// Resolve the numeric fields (pitch, depth, angle, designation) from the
    /// chosen standard + size, or the custom text fields.
    fn resolved(&self) -> Option<(f32, f32, f32, String)> {
        if self.standard == ThreadStandard::Custom {
            let pitch: f32 = self.pitch_text.trim().parse().ok()?;
            let depth: f32 = self.depth_text.trim().parse().ok()?;
            let angle: f32 = self.angle_text.trim().parse().ok()?;
            (pitch > 0.0 && depth > 0.0 && angle > 0.0 && angle < 180.0)
                .then(|| (pitch, depth, angle, "Custom".to_string()))
        } else {
            let p = self.standard.presets().get(self.preset_idx)?;
            Some((
                p.pitch_mm,
                default_depth_mm(p.pitch_mm),
                self.standard.angle_deg(),
                p.designation.to_string(),
            ))
        }
    }
}

impl ZeroCadApp {
    /// The face the thread tool would cut into: exactly one selected body face,
    /// and only when it reads as cylindrical (or its kind is unknown/legacy).
    pub(crate) fn thread_face_candidate(&self) -> Option<(String, u32)> {
        let faces: Vec<(String, u32)> = self
            .selected_body
            .iter()
            .filter_map(|(id, pick)| match pick {
                BodyPick::Face(f) => Some((id.clone(), *f)),
                _ => None,
            })
            .collect();
        let (node, fid) = (faces.len() == 1).then(|| faces.into_iter().next().unwrap())?;
        // Only offer threads on a cylindrical face (unknown kind = allow, and let
        // eval report if the selection isn't cylindrical).
        let cyl = self
            .face_ref(&node, fid)
            .and_then(|f| f.topology.and_then(|t| t.surface_kind))
            .is_none_or(|k| k.contains("cylinder"));
        cyl.then_some((node, fid))
    }

    pub(crate) fn begin_thread(&mut self, node: String, fid: u32) {
        if self.thread_op.is_some() {
            return;
        }
        let Some(mut face) = self.face_ref(&node, fid) else {
            self.status_msg = "Couldn't resolve the selected face for a thread.".to_string();
            return;
        };
        let Some(surface) = self.selected_thread_surface(&node, fid) else {
            self.status_msg = "Couldn't fit a cylinder to the selected face.".to_string();
            return;
        };
        // A wrapping cylinder's area centroid lies on its axis. Coaxial inner
        // and outer walls consequently save nearly the same point, so the old
        // radius resolver chose the smaller (inside) wall every time. Preserve
        // the public FaceRef shape but capture a real point on the picked wall.
        face.centroid = surface.anchor;
        // Default to the metric size nearest the face's radius (if we can read it
        // off the mesh), else M6.
        let dia = Some(surface.radius * 2.0);
        let standard = ThreadStandard::MetricCoarse;
        let internal = surface.empty_side < 0.0;
        let preset_idx = dia
            .and_then(|d| {
                closest_preset(standard, d).and_then(|p| {
                    standard
                        .presets()
                        .iter()
                        .position(|q| q.designation == p.designation)
                })
            })
            .unwrap_or_else(|| {
                standard
                    .presets()
                    .iter()
                    .position(|p| p.designation == "M6×1")
                    .unwrap_or(0)
            });
        let seed = standard.presets().get(preset_idx).copied();
        let pitch = seed.map(|p| p.pitch_mm).unwrap_or(1.0);
        self.thread_op = Some(ThreadOp {
            target: node,
            face,
            internal,
            surface,
            standard,
            preset_idx,
            class: zerocad_core::parametric::thread_classes(standard, internal)
                .get(1)
                .copied()
                .unwrap_or("6H")
                .to_string(),
            right_handed: true,
            full_length: true,
            flip: false,
            length_text: "10".to_string(),
            pitch_text: format!("{pitch:.3}"),
            depth_text: format!("{:.3}", default_depth_mm(pitch)),
            angle_text: "60".to_string(),
        });
        self.status_msg =
            "Thread: pick a standard + size (or Custom), set internal/external, then OK."
                .to_string();
    }

    fn selected_thread_surface(&self, node_id: &str, fid: u32) -> Option<ThreadSurfaceFrame> {
        let (_, mesh) = self.body_meshes.iter().find(|(id, _)| id == node_id)?;
        thread_surface_frame(mesh, fid)
    }

    /// Build a thin helical ribbon on the selected wall. This is cosmetic only:
    /// the graph and B-Rep stay untouched until OK is clicked.
    pub(crate) fn thread_preview_mesh(&self) -> Option<MockMesh> {
        let op = self.thread_op.as_ref()?;
        let (pitch, _, _, _) = op.resolved()?;
        let face_len = op.surface.axial_max - op.surface.axial_min;
        let requested_len = if op.full_length {
            face_len
        } else {
            op.length_text.trim().parse::<f32>().ok()?
        };
        if pitch <= 1.0e-3 || requested_len <= 1.0e-3 || face_len <= 1.0e-3 {
            return None;
        }
        let length = requested_len.min(face_len);
        let (lo, hi) = if op.full_length {
            (op.surface.axial_min, op.surface.axial_max)
        } else if op.flip {
            (op.surface.axial_min, op.surface.axial_min + length)
        } else {
            (op.surface.axial_max - length, op.surface.axial_max)
        };
        Some(thread_preview_ribbon(
            &op.surface,
            pitch,
            lo,
            hi,
            op.right_handed,
        ))
    }

    /// The floating Thread dialog (drawn every frame while an op is active).
    pub(crate) fn show_thread_dialog(&mut self, ctx: &egui::Context) {
        let Some(op) = self.thread_op.clone() else {
            return;
        };
        let mut commit = false;
        let mut cancel = false;
        let mut op_new = op.clone();
        egui::Area::new(egui::Id::new("thread_dialog"))
            .anchor(egui::Align2::CENTER_TOP, egui::vec2(0.0, 60.0))
            .show(ctx, |ui| {
                egui::Frame::window(&ctx.style()).show(ui, |ui| {
                    ui.set_min_width(260.0);
                    ui.label(
                        egui::RichText::new("Thread")
                            .strong()
                            .size(13.5)
                            .color(self.pal().text_strong),
                    );
                    ui.add_space(6.0);

                    // Standard picker.
                    ui.horizontal_wrapped(|ui| {
                        for std in [
                            ThreadStandard::MetricCoarse,
                            ThreadStandard::UnifiedCoarse,
                            ThreadStandard::UnifiedFine,
                            ThreadStandard::Custom,
                        ] {
                            if ui
                                .selectable_label(op_new.standard == std, std.label())
                                .clicked()
                            {
                                op_new.standard = std;
                                op_new.preset_idx = 0;
                                op_new.class =
                                    zerocad_core::parametric::thread_classes(std, op_new.internal)
                                        .get(1)
                                        .copied()
                                        .unwrap_or("")
                                        .to_string();
                                // Seed the custom fields from the new selection.
                                if let Some(p) = std.presets().first() {
                                    op_new.pitch_text = format!("{:.3}", p.pitch_mm);
                                    op_new.depth_text =
                                        format!("{:.3}", default_depth_mm(p.pitch_mm));
                                    op_new.angle_text = format!("{:.0}", std.angle_deg());
                                }
                            }
                        }
                    });
                    if op_new.standard != ThreadStandard::Custom {
                        ui.label(
                            egui::RichText::new(format!(
                                "{} v{}",
                                zerocad_core::parametric::STANDARDS_LIBRARY_ID,
                                zerocad_core::parametric::STANDARDS_LIBRARY_VERSION
                            ))
                            .small()
                            .weak(),
                        );
                    }
                    ui.add_space(4.0);

                    // Size (preset) or custom numeric fields.
                    if op_new.standard == ThreadStandard::Custom {
                        ui.horizontal(|ui| {
                            ui.label("Pitch");
                            ui.add(
                                egui::TextEdit::singleline(&mut op_new.pitch_text)
                                    .desired_width(46.0),
                            );
                            ui.label("Depth");
                            ui.add(
                                egui::TextEdit::singleline(&mut op_new.depth_text)
                                    .desired_width(46.0),
                            );
                            ui.label("Angle");
                            ui.add(
                                egui::TextEdit::singleline(&mut op_new.angle_text)
                                    .desired_width(38.0),
                            );
                            ui.label("°");
                        });
                    } else {
                        let presets = op_new.standard.presets();
                        let cur = presets
                            .get(op_new.preset_idx)
                            .map(|p| p.designation)
                            .unwrap_or("—");
                        ui.horizontal(|ui| {
                            ui.label("Size");
                            egui::ComboBox::from_id_salt("thread_size")
                                .selected_text(cur)
                                .show_ui(ui, |ui| {
                                    for (i, p) in presets.iter().enumerate() {
                                        ui.selectable_value(
                                            &mut op_new.preset_idx,
                                            i,
                                            p.designation,
                                        );
                                    }
                                });
                        });
                        let classes = zerocad_core::parametric::thread_classes(
                            op_new.standard,
                            op_new.internal,
                        );
                        ui.horizontal(|ui| {
                            ui.label("Class");
                            egui::ComboBox::from_id_salt("thread_class")
                                .selected_text(&op_new.class)
                                .show_ui(ui, |ui| {
                                    for class in classes {
                                        ui.selectable_value(
                                            &mut op_new.class,
                                            (*class).to_string(),
                                            *class,
                                        );
                                    }
                                });
                        });
                    }
                    ui.add_space(4.0);

                    // Internal / external and handedness.
                    ui.horizontal(|ui| {
                        if ui.selectable_label(!op_new.internal, "External").clicked() {
                            op_new.internal = false;
                            op_new.class =
                                zerocad_core::parametric::thread_classes(op_new.standard, false)
                                    .get(1)
                                    .copied()
                                    .unwrap_or("")
                                    .to_string();
                        }
                        if ui.selectable_label(op_new.internal, "Internal").clicked() {
                            op_new.internal = true;
                            op_new.class =
                                zerocad_core::parametric::thread_classes(op_new.standard, true)
                                    .get(1)
                                    .copied()
                                    .unwrap_or("")
                                    .to_string();
                        }
                        ui.separator();
                        if ui.selectable_label(op_new.right_handed, "RH").clicked() {
                            op_new.right_handed = true;
                        }
                        if ui.selectable_label(!op_new.right_handed, "LH").clicked() {
                            op_new.right_handed = false;
                        }
                    });
                    ui.add_space(4.0);

                    // Length.
                    ui.horizontal(|ui| {
                        ui.checkbox(&mut op_new.full_length, "Full length");
                        if !op_new.full_length {
                            ui.label("Length");
                            ui.add(
                                egui::TextEdit::singleline(&mut op_new.length_text)
                                    .desired_width(50.0),
                            );
                            ui.label("mm");
                        }
                    });
                    if !op_new.full_length {
                        ui.checkbox(&mut op_new.flip, "From opposite end");
                    }

                    ui.add_space(6.0);
                    ui.horizontal(|ui| {
                        if ui.button("OK").clicked() {
                            commit = true;
                        }
                        if ui.button("Cancel").clicked() {
                            cancel = true;
                        }
                    });
                });
            });

        if cancel {
            self.thread_op = None;
            self.status_msg = "Thread cancelled.".to_string();
            return;
        }
        self.thread_op = Some(op_new.clone());
        if commit {
            self.commit_thread_op(op_new);
        }
    }

    fn commit_thread_op(&mut self, op: ThreadOp) {
        let Some((pitch, depth, angle_deg, designation)) = op.resolved() else {
            self.status_msg =
                "Thread: enter a positive pitch/depth and an angle in (0, 180).".to_string();
            return;
        };
        let length = if op.full_length {
            None
        } else {
            match op.length_text.trim().parse::<f32>() {
                Ok(l) if l > 0.0 => Some(l),
                _ => {
                    self.status_msg =
                        "Thread length must be positive (or use Full length).".to_string();
                    return;
                }
            }
        };
        self.push_undo();
        let n = self.next_id();
        let id = format!("thread_{n}");
        let standard = (op.standard != ThreadStandard::Custom)
            .then(|| op.standard.presets().get(op.preset_idx))
            .flatten()
            .and_then(|preset| {
                zerocad_core::parametric::thread_reference_with_class(
                    op.standard,
                    preset,
                    op.internal,
                    &op.class,
                )
            });
        self.document.add_feature(FeatureNode {
            id: id.clone(),
            name: format!("Thread {n} ({designation})"),
            feature: FeatureType::Thread {
                target: op.target.clone(),
                face: op.face.clone(),
                internal: op.internal,
                pitch,
                depth,
                angle_deg,
                right_handed: op.right_handed,
                starts: 1,
                length,
                flip: op.flip,
                designation,
                standard,
            },
        });
        self.document.add_dependency(&op.target, &id);
        self.selected_body.clear();
        self.selected_node_id = Some(id);
        self.thread_op = None;
        self.reevaluate_geometry();
        self.status_msg = "Thread created.".to_string();
    }
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn normalized(v: [f32; 3]) -> Option<[f32; 3]> {
    let len = dot(v, v).sqrt();
    (len > 1.0e-5).then(|| [v[0] / len, v[1] / len, v[2] / len])
}

fn thread_surface_frame(mesh: &MockMesh, fid: u32) -> Option<ThreadSurfaceFrame> {
    let mut samples: Vec<([f32; 3], [f32; 3])> = Vec::new();
    for (triangle, tri) in mesh.indices.chunks_exact(3).enumerate() {
        if mesh.face_ids.get(triangle).copied() != Some(fid) {
            continue;
        }
        for &vertex in tri {
            let offset = vertex as usize * 6;
            if offset + 5 < mesh.vertices.len() {
                samples.push((
                    [
                        mesh.vertices[offset],
                        mesh.vertices[offset + 1],
                        mesh.vertices[offset + 2],
                    ],
                    [
                        mesh.vertices[offset + 3],
                        mesh.vertices[offset + 4],
                        mesh.vertices[offset + 5],
                    ],
                ));
            }
        }
    }
    if samples.len() < 6 {
        return None;
    }

    // Cylinder normals span its radial plane. The largest cross product of any
    // two sampled normals is the most stable estimate of its axis.
    let normals: Vec<[f32; 3]> = samples
        .iter()
        .filter_map(|(_, normal)| normalized(*normal))
        .collect();
    let mut best_axis = None;
    let mut best_len2 = 0.0;
    for (i, &a) in normals.iter().enumerate() {
        for &b in normals.iter().skip(i + 1) {
            let candidate = cross(a, b);
            let len2 = dot(candidate, candidate);
            if len2 > best_len2 {
                best_axis = Some(candidate);
                best_len2 = len2;
            }
        }
    }
    let mut axis = normalized(best_axis?)?;
    // Canonical sign makes preview handedness deterministic across rebuilds.
    let major = if axis[0].abs() >= axis[1].abs() && axis[0].abs() >= axis[2].abs() {
        0
    } else if axis[1].abs() >= axis[2].abs() {
        1
    } else {
        2
    };
    if axis[major] < 0.0 {
        axis = [-axis[0], -axis[1], -axis[2]];
    }

    let sample_count = samples.len() as f32;
    let origin_sum = samples.iter().fold([0.0; 3], |sum, (point, _)| {
        [sum[0] + point[0], sum[1] + point[1], sum[2] + point[2]]
    });
    let origin = [
        origin_sum[0] / sample_count,
        origin_sum[1] / sample_count,
        origin_sum[2] / sample_count,
    ];
    let mut axial_min = f32::INFINITY;
    let mut axial_max = f32::NEG_INFINITY;
    let mut radius_sum = 0.0;
    let mut radial_samples = 0usize;
    let mut empty_side_sum = 0.0;
    let mut radial_u = None;
    let mut anchor = None;
    for &(point, normal) in &samples {
        let rel = [
            point[0] - origin[0],
            point[1] - origin[1],
            point[2] - origin[2],
        ];
        let axial = dot(rel, axis);
        let radial = [
            rel[0] - axis[0] * axial,
            rel[1] - axis[1] * axial,
            rel[2] - axis[2] * axial,
        ];
        let radial_len = dot(radial, radial).sqrt();
        if radial_len <= 1.0e-4 {
            continue;
        }
        let radial_dir = [
            radial[0] / radial_len,
            radial[1] / radial_len,
            radial[2] / radial_len,
        ];
        axial_min = axial_min.min(axial);
        axial_max = axial_max.max(axial);
        radius_sum += radial_len;
        radial_samples += 1;
        empty_side_sum += normalized(normal).map_or(0.0, |n| dot(n, radial_dir));
        radial_u.get_or_insert(radial_dir);
        anchor.get_or_insert(point);
    }
    let radial_u = radial_u?;
    let radial_v = normalized(cross(axis, radial_u))?;
    let radius = radius_sum / radial_samples as f32;
    if radius <= 0.1 || axial_max - axial_min <= 1.0e-3 {
        return None;
    }
    Some(ThreadSurfaceFrame {
        origin,
        axis,
        radial_u,
        radial_v,
        radius,
        axial_min,
        axial_max,
        empty_side: if empty_side_sum < 0.0 { -1.0 } else { 1.0 },
        anchor: anchor?,
    })
}

fn thread_preview_ribbon(
    surface: &ThreadSurfaceFrame,
    pitch: f32,
    lo: f32,
    hi: f32,
    right_handed: bool,
) -> MockMesh {
    let mut mesh = MockMesh::empty();
    let span = (hi - lo).max(0.0);
    let samples = ((span / pitch * 24.0).ceil() as usize).clamp(24, 2400);
    let half_width = (pitch * 0.055).clamp(0.015, 0.10);
    let display_radius =
        surface.radius + surface.empty_side * (surface.radius * 0.003).clamp(0.02, 0.06);
    let handed = if right_handed { 1.0 } else { -1.0 };

    for i in 0..=samples {
        let fraction = i as f32 / samples as f32;
        let z = lo + span * fraction;
        let angle = handed * std::f32::consts::TAU * (z - lo) / pitch;
        let (sin, cos) = angle.sin_cos();
        let radial = [
            surface.radial_u[0] * cos + surface.radial_v[0] * sin,
            surface.radial_u[1] * cos + surface.radial_v[1] * sin,
            surface.radial_u[2] * cos + surface.radial_v[2] * sin,
        ];
        let normal = [
            radial[0] * surface.empty_side,
            radial[1] * surface.empty_side,
            radial[2] * surface.empty_side,
        ];
        for axial_offset in [-half_width, half_width] {
            let zz = (z + axial_offset).clamp(lo, hi);
            let point = [
                surface.origin[0] + surface.axis[0] * zz + radial[0] * display_radius,
                surface.origin[1] + surface.axis[1] * zz + radial[1] * display_radius,
                surface.origin[2] + surface.axis[2] * zz + radial[2] * display_radius,
            ];
            mesh.vertices.extend_from_slice(&[
                point[0], point[1], point[2], normal[0], normal[1], normal[2],
            ]);
        }
        if i > 0 {
            let a = (i * 2 - 2) as u32;
            let b = a + 1;
            let c = a + 2;
            let d = a + 3;
            mesh.indices.extend_from_slice(&[a, c, b, b, c, d]);
            mesh.face_ids.extend_from_slice(&[0, 0]);

            let center = |pair: u32| {
                let left = pair as usize * 6;
                let right = (pair as usize + 1) * 6;
                [
                    (mesh.vertices[left] + mesh.vertices[right]) * 0.5,
                    (mesh.vertices[left + 1] + mesh.vertices[right + 1]) * 0.5,
                    (mesh.vertices[left + 2] + mesh.vertices[right + 2]) * 0.5,
                ]
            };
            let p0 = center(a);
            let p1 = center(c);
            let edge_base = (mesh.edge_vertices.len() / 3) as u32;
            mesh.edge_vertices.extend_from_slice(&p0);
            mesh.edge_vertices.extend_from_slice(&p1);
            mesh.edge_indices
                .extend_from_slice(&[edge_base, edge_base + 1]);
            mesh.edge_face_normals.extend_from_slice(&[
                normal[0], normal[1], normal[2], normal[0], normal[1], normal[2],
            ]);
            mesh.edge_groups.push(0);
        }
    }
    mesh
}

#[cfg(test)]
mod tests {
    use super::*;
    use zerocad_core::{mock_kernel, CoordinateSystem};

    fn tube() -> (zerocad_core::mock_kernel::KernelSolid, MockMesh) {
        let circle = |radius: f32| {
            (0..48)
                .map(|i| {
                    let angle = std::f32::consts::TAU * i as f32 / 48.0;
                    (radius * angle.cos(), radius * angle.sin())
                })
                .collect::<Vec<_>>()
        };
        let solid = mock_kernel::extruded_region_solid(
            &circle(6.0),
            &[circle(5.0)],
            10.0,
            &CoordinateSystem::XY,
        )
        .expect("hollow cylinder");
        let mesh = MockMesh::from_solid(&solid);
        (solid, mesh)
    }

    #[test]
    fn coaxial_inside_and_outside_faces_capture_distinct_wall_points() {
        let (solid, mesh) = tube();
        let mut ids = mesh.face_ids.clone();
        ids.sort_unstable();
        ids.dedup();
        let mut cylinders: Vec<ThreadSurfaceFrame> = ids
            .into_iter()
            .filter_map(|fid| thread_surface_frame(&mesh, fid))
            .collect();
        cylinders.sort_by(|a, b| a.radius.partial_cmp(&b.radius).unwrap());

        assert_eq!(
            cylinders.len(),
            2,
            "tube should expose inner and outer walls"
        );
        assert!((cylinders[0].radius - 5.0).abs() < 0.15);
        assert!((cylinders[1].radius - 6.0).abs() < 0.15);
        assert!(cylinders[0].empty_side < 0.0, "hole wall points inward");
        assert!(cylinders[1].empty_side > 0.0, "outer wall points outward");

        let inner = mock_kernel::cylinder_face_near(&solid, cylinders[0].anchor)
            .expect("inner wall resolves");
        let outer = mock_kernel::cylinder_face_near(&solid, cylinders[1].anchor)
            .expect("outer wall resolves");
        assert!((inner.radius - 5.0).abs() < 0.05);
        assert!((outer.radius - 6.0).abs() < 0.05);
    }

    #[test]
    fn cosmetic_preview_is_a_helix_without_body_geometry() {
        let (_, mesh) = tube();
        let mut ids = mesh.face_ids.clone();
        ids.sort_unstable();
        ids.dedup();
        let surface = ids
            .into_iter()
            .filter_map(|fid| thread_surface_frame(&mesh, fid))
            .max_by(|a, b| a.radius.partial_cmp(&b.radius).unwrap())
            .expect("outer cylindrical wall");
        let preview =
            thread_preview_ribbon(&surface, 1.25, surface.axial_min, surface.axial_max, true);

        assert!(!preview.indices.is_empty());
        assert_eq!(preview.face_ids.len(), preview.indices.len() / 3);
        assert!(!preview.edge_indices.is_empty());
        assert!(
            preview.face_refs.is_empty(),
            "preview is not selectable model geometry"
        );
    }
}
