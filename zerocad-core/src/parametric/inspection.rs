//! Read-only inspection operations over evaluated kernel bodies.

use super::*;
use crate::mock_kernel::{common_bodies_with_history, CommonBodiesError};

#[derive(Debug, Clone, PartialEq)]
pub struct BodyInspection {
    pub body_id: String,
    pub part_count: usize,
    pub volume_mm3: f64,
    pub surface_area_mm2: f64,
    pub centroid: [f64; 3],
    pub bounds_min: [f64; 3],
    pub bounds_max: [f64; 3],
    /// Display/material density selected by the caller. Geometry remains in
    /// millimetres, so `mass_g = volume_mm3 / 1000 * density_g_cm3`.
    pub density_g_cm3: f64,
    pub mass_g: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct InterferencePair {
    pub first_body: String,
    pub second_body: String,
    pub volume_mm3: f64,
    pub centroid: [f64; 3],
}

#[derive(Debug, Clone, PartialEq)]
pub struct FaceInspection {
    pub body_id: String,
    pub face_id: u32,
    pub area_mm2: f64,
}

impl ParametricGraph {
    /// Measure one evaluated body from strict, fine tessellations of its B-Rep
    /// parts. This path never reads the viewport mesh and therefore remains
    /// stable when display quality changes.
    pub fn inspect_body(&self, body_id: &str) -> Result<BodyInspection, String> {
        self.inspect_body_with_density(body_id, 1.0)
    }

    pub fn inspect_body_with_density(
        &self,
        body_id: &str,
        density_g_cm3: f64,
    ) -> Result<BodyInspection, String> {
        if !density_g_cm3.is_finite() || density_g_cm3 < 0.0 {
            return Err("material density must be finite and non-negative".to_string());
        }
        let hidden = std::collections::HashSet::new();
        let (live, _) = self.build_live(&hidden, false)?;
        let body = live
            .iter()
            .find(|body| body.id == body_id)
            .ok_or_else(|| format!("body '{body_id}' does not exist"))?;
        inspect_live_body(body, density_g_cm3)
    }

    /// Measure one picked display face from a fresh strict kernel
    /// tessellation. Face ids use the same analytic-surface grouping and
    /// multipart rebasing as the viewport mesh.
    pub fn inspect_face(&self, body_id: &str, face_id: u32) -> Result<FaceInspection, String> {
        let hidden = std::collections::HashSet::new();
        let (live, _) = self.build_live(&hidden, false)?;
        let body = live
            .iter()
            .find(|body| body.id == body_id)
            .ok_or_else(|| format!("body '{body_id}' does not exist"))?;
        if body.parts.is_empty() {
            let mesh = body
                .pristine
                .as_deref()
                .ok_or_else(|| format!("mesh body '{body_id}' has no display geometry"))?;
            let mut area = 0.0;
            let mut found = false;
            for (triangle, &triangle_face) in mesh.indices.chunks_exact(3).zip(&mesh.face_ids) {
                if triangle_face != face_id {
                    continue;
                }
                let Some([a, b, c]) = mesh_triangle(mesh, triangle) else {
                    continue;
                };
                found = true;
                area += triangle_area(a, b, c);
            }
            return found
                .then_some(FaceInspection {
                    body_id: body_id.to_string(),
                    face_id,
                    area_mm2: area,
                })
                .ok_or_else(|| format!("face {face_id} does not exist on mesh body '{body_id}'"));
        }
        let mut face_offset = 0_u32;
        let mut area = 0.0;
        let mut found = false;
        for part in &body.parts {
            let mesh = openrcad::mesh::tessellate_checked(part, 0.005, 0.05)
                .map_err(|error| format!("strict face tessellation failed: {error}"))?;
            let surface_groups = crate::mock_kernel::cylinder_surface_groups(part);
            let mut canonical = std::collections::HashMap::new();
            for (index, group) in surface_groups.iter().copied().enumerate() {
                canonical.entry(group).or_insert(index as u32);
            }
            let display_face = |local: u32| {
                surface_groups
                    .get(local as usize)
                    .and_then(|group| canonical.get(group))
                    .copied()
                    .unwrap_or(local)
            };
            let local_max = mesh.face_ids.iter().copied().map(display_face).max();
            for (triangle, local_face) in mesh.triangles.iter().zip(&mesh.face_ids) {
                if display_face(*local_face) + face_offset != face_id {
                    continue;
                }
                found = true;
                let [a, b, c] = triangle.map(|index| mesh.vertices[index as usize]);
                let ab = b - a;
                let ac = c - a;
                area += 0.5 * ab.cross(&ac).magnitude();
            }
            if let Some(local_max) = local_max {
                face_offset += local_max + 1;
            }
        }
        if !found {
            return Err(format!("face {face_id} does not exist on body '{body_id}'"));
        }
        Ok(FaceInspection {
            body_id: body_id.to_string(),
            face_id,
            area_mm2: area,
        })
    }

    /// Exact Common-based interference check. Contact-only pairs are omitted;
    /// every returned row has strictly more than `minimum_volume_mm3` common
    /// volume. Inputs are sorted/deduplicated for deterministic pair order.
    pub fn inspect_interference(
        &self,
        body_ids: &[String],
        minimum_volume_mm3: f64,
    ) -> Result<Vec<InterferencePair>, String> {
        if !minimum_volume_mm3.is_finite() || minimum_volume_mm3 < 0.0 {
            return Err("minimum interference volume must be finite and non-negative".to_string());
        }
        let hidden = std::collections::HashSet::new();
        let (live, _) = self.build_live(&hidden, false)?;
        let mut ids = body_ids.to_vec();
        ids.sort();
        ids.dedup();
        if ids.len() < 2 {
            return Err("interference checking needs at least two bodies".to_string());
        }
        let mut reports = Vec::new();
        for first_index in 0..ids.len() {
            for second_index in first_index + 1..ids.len() {
                let first = live
                    .iter()
                    .find(|body| body.id == ids[first_index])
                    .ok_or_else(|| format!("body '{}' does not exist", ids[first_index]))?;
                let second = live
                    .iter()
                    .find(|body| body.id == ids[second_index])
                    .ok_or_else(|| format!("body '{}' does not exist", ids[second_index]))?;
                if first.parts.is_empty() || second.parts.is_empty() {
                    return Err(format!(
                        "interference checking requires B-Rep solids; '{}' or '{}' is a mesh body",
                        first.id, second.id
                    ));
                }
                let mut volume = 0.0;
                let mut centroid_sum = [0.0_f64; 3];
                for first_part in &first.parts {
                    for second_part in &second.parts {
                        let common = match common_bodies_with_history(first_part, second_part, None)
                        {
                            Ok(common) => common,
                            Err(CommonBodiesError::Empty) => continue,
                            Err(CommonBodiesError::Failed(error)) => return Err(error),
                        };
                        for solid in common.bodies {
                            let properties = solid_mass_properties(&solid)?;
                            if properties.volume <= minimum_volume_mm3 {
                                continue;
                            }
                            volume += properties.volume;
                            for (sum, coordinate) in
                                centroid_sum.iter_mut().zip(properties.centroid)
                            {
                                *sum += coordinate * properties.volume;
                            }
                        }
                    }
                }
                if volume > minimum_volume_mm3 {
                    reports.push(InterferencePair {
                        first_body: first.id.clone(),
                        second_body: second.id.clone(),
                        volume_mm3: volume,
                        centroid: centroid_sum.map(|component| component / volume),
                    });
                }
            }
        }
        Ok(reports)
    }
}

fn inspect_live_body(body: &LiveBody, density_g_cm3: f64) -> Result<BodyInspection, String> {
    if body.parts.is_empty() {
        let mesh = body
            .pristine
            .as_deref()
            .ok_or_else(|| format!("mesh body '{}' has no geometry", body.id))?;
        return inspect_mesh_body(&body.id, mesh, density_g_cm3);
    }
    let mut volume = 0.0;
    let mut surface_area = 0.0;
    let mut centroid_sum = [0.0_f64; 3];
    let mut bounds_min = [f64::INFINITY; 3];
    let mut bounds_max = [f64::NEG_INFINITY; 3];
    for part in &body.parts {
        let properties = solid_mass_properties(part)?;
        volume += properties.volume;
        surface_area += properties.surface_area;
        for (sum, coordinate) in centroid_sum.iter_mut().zip(properties.centroid) {
            *sum += coordinate * properties.volume;
        }
        if let Some((min, max)) = crate::mock_kernel::solid_aabb(part) {
            for axis in 0..3 {
                bounds_min[axis] = bounds_min[axis].min(min[axis] as f64);
                bounds_max[axis] = bounds_max[axis].max(max[axis] as f64);
            }
        }
    }
    if volume <= 0.0 {
        return Err(format!(
            "body '{}' has no positive measurable volume",
            body.id
        ));
    }
    Ok(BodyInspection {
        body_id: body.id.clone(),
        part_count: body.parts.len(),
        volume_mm3: volume,
        surface_area_mm2: surface_area,
        centroid: centroid_sum.map(|component| component / volume),
        bounds_min,
        bounds_max,
        density_g_cm3,
        mass_g: volume / 1_000.0 * density_g_cm3,
    })
}

fn inspect_mesh_body(
    body_id: &str,
    mesh: &MockMesh,
    density_g_cm3: f64,
) -> Result<BodyInspection, String> {
    let mut surface_area = 0.0;
    let mut area_centroid = [0.0_f64; 3];
    let mut bounds_min = [f64::INFINITY; 3];
    let mut bounds_max = [f64::NEG_INFINITY; 3];
    for vertex in mesh.vertices.chunks_exact(6) {
        for axis in 0..3 {
            let coordinate = vertex[axis] as f64;
            bounds_min[axis] = bounds_min[axis].min(coordinate);
            bounds_max[axis] = bounds_max[axis].max(coordinate);
        }
    }
    for triangle in mesh.indices.chunks_exact(3) {
        let Some([a, b, c]) = mesh_triangle(mesh, triangle) else {
            continue;
        };
        let area = triangle_area(a, b, c);
        surface_area += area;
        for axis in 0..3 {
            area_centroid[axis] += (a[axis] + b[axis] + c[axis]) / 3.0 * area;
        }
    }
    if surface_area <= 0.0 || !bounds_min[0].is_finite() {
        return Err(format!("mesh body '{body_id}' has no measurable triangles"));
    }
    let mass = mesh.mass_properties();
    let (volume, centroid) = mass.map_or_else(
        || (0.0, area_centroid.map(|component| component / surface_area)),
        |properties| (properties.volume, properties.centroid),
    );
    Ok(BodyInspection {
        body_id: body_id.to_string(),
        part_count: 1,
        volume_mm3: volume,
        surface_area_mm2: surface_area,
        centroid,
        bounds_min,
        bounds_max,
        density_g_cm3,
        mass_g: volume / 1_000.0 * density_g_cm3,
    })
}

fn mesh_triangle(mesh: &MockMesh, indices: &[u32]) -> Option<[[f64; 3]; 3]> {
    let point = |index: u32| {
        let start = index as usize * 6;
        Some([
            *mesh.vertices.get(start)? as f64,
            *mesh.vertices.get(start + 1)? as f64,
            *mesh.vertices.get(start + 2)? as f64,
        ])
    };
    Some([
        point(*indices.first()?)?,
        point(*indices.get(1)?)?,
        point(*indices.get(2)?)?,
    ])
}

fn triangle_area(a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let cross = [
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    ];
    0.5 * (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt()
}

fn solid_mass_properties(solid: &KernelSolid) -> Result<openrcad::mesh::MassProperties, String> {
    let mesh = openrcad::mesh::tessellate_checked(solid, 0.005, 0.05)
        .map_err(|error| format!("strict inspection tessellation failed: {error}"))?;
    openrcad::mesh::mass_properties(&mesh)
        .ok_or_else(|| "strict inspection mesh is open or degenerate".to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn body_measurement_is_independent_of_viewport_mesh_quality() {
        let mut graph = ParametricGraph::new();
        graph.add_feature(FeatureNode {
            id: "box_1".to_string(),
            name: "Box".to_string(),
            feature: FeatureType::Box {
                w: 10.0,
                h: 20.0,
                d: 30.0,
            },
        });
        let measured = graph.inspect_body("box_1").unwrap();
        assert!((measured.volume_mm3 - 6_000.0).abs() < 1.0e-6);
        assert!((measured.surface_area_mm2 - 2_200.0).abs() < 1.0e-6);
        assert_eq!(measured.centroid, [5.0, 10.0, 15.0]);
        let steel = graph.inspect_body_with_density("box_1", 7.85).unwrap();
        assert!((steel.mass_g - 47.1).abs() < 1.0e-9);
    }

    #[test]
    fn common_based_interference_omits_disjoint_and_reports_overlap() {
        let mut graph = ParametricGraph::new();
        graph.add_feature(FeatureNode {
            id: "box_1".to_string(),
            name: "Box A".to_string(),
            feature: FeatureType::Box {
                w: 10.0,
                h: 10.0,
                d: 10.0,
            },
        });
        graph.add_feature(FeatureNode {
            id: "box_2".to_string(),
            name: "Box B".to_string(),
            feature: FeatureType::Box {
                w: 10.0,
                h: 10.0,
                d: 10.0,
            },
        });
        let report = graph
            .inspect_interference(&["box_1".to_string(), "box_2".to_string()], 1.0e-6)
            .unwrap();
        assert_eq!(report.len(), 1);
        assert!((report[0].volume_mm3 - 1_000.0).abs() < 1.0e-6);
    }

    #[test]
    fn picked_face_area_is_measured_from_strict_kernel_geometry() {
        let mut graph = ParametricGraph::new();
        graph.add_feature(FeatureNode {
            id: "box_1".to_string(),
            name: "Box".to_string(),
            feature: FeatureType::Box {
                w: 10.0,
                h: 20.0,
                d: 30.0,
            },
        });
        let mut areas = (0..6)
            .map(|face| graph.inspect_face("box_1", face).unwrap().area_mm2)
            .collect::<Vec<_>>();
        areas.sort_by(f64::total_cmp);
        assert_eq!(areas, [200.0, 200.0, 300.0, 300.0, 600.0, 600.0]);
    }
}
