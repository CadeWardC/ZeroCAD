//! A closed hollow needs inward void faces, not just watertight topology.
use openrcad_foundation::{Ax1, Dir, Pnt, Trsf, Vec as GeomVec};

#[test]
fn closed_box_shell_preserves_material_moments_and_directed_boundaries() {
    let rotation = Trsf::rotation(&Ax1::new(Pnt::origin(), Dir::new(1.0, 2.0, 3.0)), 0.7);
    let placement = Trsf::translation(GeomVec::new(80.0, -45.0, 120.0)).multiply(&rotation);
    for (width, height, depth, thickness) in [(60.0, 40.0, 20.0, 2.0), (10.0, 10.0, 10.0, 1.0)] {
        let source = openrcad_primitives::make_box_operation(&Pnt::origin(), width, height, depth)
            .unwrap()
            .value;
        let hollow = openrcad_algo::shell_solid_operation_with_policy(
            &source,
            thickness,
            &[],
            &Default::default(),
        )
        .unwrap()
        .value;
        let inner = [width, height, depth].map(|size| size - 2.0 * thickness);
        let expected_volume = width * height * depth - inner.iter().product::<f64>();
        let expected_area = 2.0
            * (width * height
                + width * depth
                + height * depth
                + inner[0] * inner[1]
                + inner[0] * inner[2]
                + inner[1] * inner[2]);
        for transform in [Trsf::default(), placement] {
            let solid = hollow.transformed(&transform);
            let center =
                transform.transform_point(&Pnt::new(width / 2.0, height / 2.0, depth / 2.0));
            let outer_faces = solid.shell().len() as u32;
            let mesh = openrcad_mesh::tessellate_checked(&solid, 0.005, 0.05).unwrap();
            let properties = openrcad_mesh::mass_properties(&mesh).unwrap();
            assert!(
                (properties.volume - expected_volume).abs() < 1e-6,
                "{properties:?}"
            );
            assert!(
                (properties.surface_area - expected_area).abs() < 1e-6,
                "{properties:?}"
            );
            for (actual, expected) in
                properties
                    .centroid
                    .into_iter()
                    .zip([center.x(), center.y(), center.z()])
            {
                assert!((actual - expected).abs() < 1e-7, "{properties:?}");
            }
            let mut directed = std::collections::HashMap::<(u32, u32), i32>::new();
            for (triangle, face) in mesh.triangles.iter().zip(&mesh.face_ids) {
                let [a, b, c] = triangle.map(|index| mesh.vertices[index as usize]);
                let normal = (b - a).cross(&(c - a));
                let outward = normal.dot(&(a - center));
                assert!(if *face < outer_faces {
                    outward > 0.0
                } else {
                    outward < 0.0
                });
                for (start, end) in [
                    (triangle[0], triangle[1]),
                    (triangle[1], triangle[2]),
                    (triangle[2], triangle[0]),
                ] {
                    *directed
                        .entry((start.min(end), start.max(end)))
                        .or_default() += if start < end { 1 } else { -1 };
                }
            }
            assert!(directed.values().all(|balance| *balance == 0));
        }
    }
}
