use openrcad_algo::{boolean_operation, BooleanOp};
use openrcad_foundation::{Ax1, Ax2, Dir, Pnt, Trsf, Vec as GeomVec};

#[test]
fn circular_cut_touching_support_preserves_its_disk_winding() {
    use openrcad_topo::{Edge, Face, Wire};
    let points = [
        (0.0, 0.0),
        (16.0, 0.0),
        (16.0, 12.0),
        (13.0, 12.0),
        (13.0, 2.0),
        (3.0, 2.0),
        (3.0, 12.0),
        (0.0, 12.0),
    ];
    let face = Face::new(
        Some(openrcad_geom::GeomSurface::plane(
            openrcad_geom::Plane::from_point_normal(Pnt::origin(), Dir::dz()),
        )),
        Wire::from_edges((0..points.len()).map(|i| {
            let (a, b) = (points[i], points[(i + 1) % points.len()]);
            Edge::between_points(Pnt::new(a.0, a.1, 0.0), Pnt::new(b.0, b.1, 0.0))
        })),
    );
    let base = openrcad_algo::prism_operation(&face, GeomVec::new(0.0, 0.0, 12.0))
        .unwrap()
        .value;
    let forward = openrcad_primitives::make_cylinder_operation(
        &Ax2::new(Pnt::new(3.0, 6.0, 7.0), Dir::dx()),
        2.0,
        20.0,
    )
    .unwrap()
    .value;
    let reverse = openrcad_primitives::make_cylinder_operation(
        &Ax2::new(Pnt::new(-5.0, 6.0, 7.0), Dir::dx()),
        2.0,
        8.0,
    )
    .unwrap()
    .value;
    let placement = Trsf::translation(GeomVec::new(35.0, -20.0, 12.0)).multiply(&Trsf::rotation(
        &Ax1::new(Pnt::origin(), Dir::new(1.0, 2.0, 3.0)),
        0.7,
    ));
    for transform in [Trsf::default(), placement] {
        let base = base.transformed(&transform);
        let mut result = base;
        for (index, tool) in [
            forward.transformed(&transform),
            reverse.transformed(&transform),
        ]
        .iter()
        .enumerate()
        {
            result = boolean_operation(&result, tool, BooleanOp::Cut)
                .unwrap()
                .value;
            let mesh = openrcad_mesh::tessellate_checked(&result, 0.001, 0.05).unwrap();
            let volume = openrcad_mesh::mass_properties(&mesh).unwrap().volume;
            let expected = 1104.0 - (index + 1) as f64 * 12.0 * std::f64::consts::PI;
            assert!((volume - expected).abs() < 0.06, "{volume} != {expected}");
            let support_probe = transform.transform_point(&Pnt::new(2.99, 6.0, 7.0));
            assert_eq!(
                openrcad_algo::boolean::point_in_solid(&support_probe, &result),
                index == 0
            );
        }
    }
}
