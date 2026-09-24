//! Proven branch identity for the two straight intersections of a named plane
//! and cylinder. Unsupported or multiply fragmented branches remain unnamed.
use super::*;
use openrcad::geom::{Curve, GeomCurve, GeomSurface};
use std::collections::HashMap;

pub(crate) fn populate_cylinder_plane_edge_names(mesh: &mut MockMesh, solid: &KernelSolid) {
    let faces = solid.shell().faces();
    if !faces
        .iter()
        .any(|face| matches!(face.surface(), Some(GeomSurface::Cylinder(_))))
    {
        return;
    }
    let groups = cylinder_surface_groups(solid);
    let mut canonical = HashMap::new();
    for (index, group) in groups.iter().enumerate() {
        canonical.entry(*group).or_insert(index as u32);
    }
    let names: Vec<_> = groups
        .iter()
        .map(|group| {
            let id = canonical.get(group)?;
            mesh.face_refs
                .iter()
                .find(|face| face.face_id == *id)?
                .topology
                .as_ref()?
                .face_id
                .clone()
        })
        .collect();
    let mut proposals: HashMap<String, Vec<(usize, Vec<String>)>> = HashMap::new();
    for (index, selectable) in mesh.edge_refs.iter().enumerate() {
        // Preserve existing design identities (including older saved sketch refs).
        if selectable
            .topology
            .as_ref()
            .and_then(|t| t.edge_id.as_deref())
            .is_some_and(|id| crate::parametric::topo_name::TopoName::parse(id).is_durable())
        {
            continue;
        }
        let p0 = Pnt::new(
            selectable.p0[0] as f64,
            selectable.p0[1] as f64,
            selectable.p0[2] as f64,
        );
        let p1 = Pnt::new(
            selectable.p1[0] as f64,
            selectable.p1[1] as f64,
            selectable.p1[2] as f64,
        );
        let scale = selectable
            .p0
            .iter()
            .chain(selectable.p1.iter())
            .map(|v| v.abs() as f64)
            .fold(1.0_f64, f64::max);
        let tolerance = scale * f32::EPSILON as f64 * 8.0;
        let adjacent: Vec<_> = faces
            .iter()
            .enumerate()
            .filter(|(_, face)| {
                face.wires()
                    .iter()
                    .flat_map(|wire| wire.edges())
                    .any(|edge| {
                        let Some(curve @ GeomCurve::Line(_)) = edge.curve() else {
                            return false;
                        };
                        let a = curve.point(edge.first());
                        let b = curve.point(edge.last());
                        let direction = b - a;
                        let length_squared = direction.dot(&direction);
                        if length_squared <= tolerance * tolerance {
                            return false;
                        }
                        let on_span = |point: Pnt| {
                            let parameter =
                                ((point - a).dot(&direction) / length_squared).clamp(0., 1.);
                            point.distance(&(a + direction * parameter)) <= tolerance
                        };
                        on_span(p0) && on_span(p1)
                    })
            })
            .collect();
        if adjacent.len() != 2 {
            continue;
        }
        let (a, b) = (adjacent[0], adjacent[1]);
        let (cylinder, plane) = match (a.1.surface(), b.1.surface()) {
            (Some(GeomSurface::Cylinder(c)), Some(GeomSurface::Plane(p)))
            | (Some(GeomSurface::Plane(p)), Some(GeomSurface::Cylinder(c))) => (c, p),
            _ => continue,
        };
        let (Some(name_a), Some(name_b)) = (&names[a.0], &names[b.0]) else {
            continue;
        };
        if name_a == name_b {
            continue;
        }
        let axis = cylinder.position().direction();
        let normal = plane.normal();
        let axis = GeomVec::new(axis.x(), axis.y(), axis.z());
        let normal = GeomVec::new(normal.x(), normal.y(), normal.z());
        if axis.dot(&normal).abs() > 1e-8 {
            continue;
        }
        let side = (p0 - cylinder.position().location()).dot(&axis.cross(&normal));
        if side.abs() <= tolerance {
            continue;
        }
        let mut pair = vec![name_a.clone(), name_b.clone()];
        pair.sort();
        let mut hash = blake3::Hasher::new();
        for name in &pair {
            hash.update(&(name.len() as u64).to_le_bytes());
            hash.update(name.as_bytes());
        }
        let id = format!(
            "edge-pair:{}:side:{}",
            hash.finalize().to_hex(),
            if side > 0. { "+" } else { "-" }
        );
        proposals.entry(id).or_default().push((index, pair));
    }
    for (id, entries) in proposals {
        if entries.len() != 1 {
            continue;
        }
        let (index, pair) = entries.into_iter().next().unwrap();
        // Exact bounded B-Rep lines take precedence over a noisy display circle fit.
        mesh.edge_refs[index].curve = Some(EdgeCurveHint::Line);
        let topology = mesh.edge_refs[index]
            .topology
            .get_or_insert_with(Default::default);
        topology.curve_kind = Some("line".into());
        topology.edge_id = Some(id);
        topology.adjacent_face_ids = pair;
    }
}
