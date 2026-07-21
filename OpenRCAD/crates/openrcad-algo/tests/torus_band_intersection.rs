use openrcad_algo::band_topology::{BandTopologyError, GeometryWorkBudget, GeometryWorkStage};
use openrcad_algo::intersect::{surface_surface_with_budget, uv_of};
use openrcad_foundation::{Ax3, Dir, Pnt, ToleranceContext, TolerancePolicy, Vec as GeomVec};
use openrcad_geom::{
    ConicalSurface, Curve, CylindricalSurface, GeomCurve, GeomSurface, Plane, Surface,
    ToroidalSurface,
};

fn frame(scale: f64, far_origin: bool) -> Ax3 {
    let origin = if far_origin {
        Pnt::new(1.0e6 * scale, -2.0e6 * scale, 3.0e6 * scale)
    } else {
        Pnt::new(2.0 * scale, -3.0 * scale, 5.0 * scale)
    };
    let axis = Dir::new(0.36, 0.48, 0.8);
    let x_direction = Dir::new(0.8, -0.6, 0.0);
    Ax3::new_axes(origin, axis, x_direction)
}

fn context(scale: f64, far_origin: bool) -> ToleranceContext {
    let mut bounds = openrcad_foundation::BndBox::new();
    let origin = frame(scale, far_origin).location();
    bounds.add(&origin);
    bounds.add(&(origin + GeomVec::new(12.0 * scale, 9.0 * scale, 7.0 * scale)));
    ToleranceContext::derive(&TolerancePolicy::STANDARD, &[bounds], Some(scale), 1.0).unwrap()
}

fn assert_curve_on_both(
    curve: &GeomCurve,
    first: &GeomSurface,
    second: &GeomSurface,
    tolerance: f64,
) {
    let (start, end) = curve.bounds();
    for fraction in [0.0, 0.13, 0.37, 0.71, 1.0] {
        let parameter = start + (end - start) * fraction;
        let point = curve.point(parameter);
        for surface in [first, second] {
            let (u, v) = uv_of(surface, &point);
            let lifted = surface.point(u, v);
            assert!(
                point.distance(&lifted) <= tolerance,
                "point {:?} missed {:?} by {}",
                point,
                surface,
                point.distance(&lifted)
            );
        }
    }
}

fn assert_pair(first: GeomSurface, second: GeomSurface, expected: usize, scale: f64, far: bool) {
    let context = context(scale, far);
    let mut budget = GeometryWorkBudget::new(64);
    let curves =
        surface_surface_with_budget(&first, &second, context.policy.intersection, &mut budget)
            .unwrap();
    assert_eq!(curves.len(), expected);
    let certificate_tolerance = context.policy.pcurve_consistency
        + context.arithmetic_floor * 64.0
        + scale * f64::EPSILON * 256.0;
    for curve in &curves {
        assert_curve_on_both(curve, &first, &second, certificate_tolerance);
    }
}

#[test]
fn exact_torus_transition_matrix_survives_scale_rotation_and_far_origin() {
    for scale in [1.0e-3, 1.0, 1.0e3] {
        for far in [false, true] {
            let frame = frame(scale, far);
            let axis = GeomVec::from_dir(frame.direction());
            let torus = GeomSurface::torus(ToroidalSurface::new(frame, 4.0 * scale, scale));

            let plane = GeomSurface::plane(Plane::from_point_normal(
                frame.location() + axis * (0.5 * scale),
                frame.direction(),
            ));
            assert_pair(plane, torus.clone(), 2, scale, far);

            let meridian_normal = frame.x_direction();
            let meridian_plane =
                GeomSurface::plane(Plane::from_point_normal(frame.location(), meridian_normal));
            assert_pair(meridian_plane, torus.clone(), 2, scale, far);

            let offset_meridian_plane = GeomSurface::plane(Plane::from_point_normal(
                frame.location() + GeomVec::from_dir(meridian_normal) * (0.5 * scale),
                meridian_normal,
            ));
            assert_pair(offset_meridian_plane, torus.clone(), 2, scale, far);

            let cylinder = GeomSurface::cylinder(CylindricalSurface::new(
                Ax3::new_axes(
                    frame.location() + axis * (7.0 * scale),
                    frame.direction(),
                    frame.x_direction(),
                ),
                4.5 * scale,
            ));
            assert_pair(cylinder, torus.clone(), 2, scale, far);

            let cone = GeomSurface::cone(ConicalSurface::new(frame, 4.0 * scale, 0.35));
            assert_pair(cone, torus.clone(), 2, scale, far);

            let second_torus = GeomSurface::torus(ToroidalSurface::new(
                Ax3::new_axes(
                    frame.location() + axis * scale,
                    frame.direction(),
                    frame.x_direction(),
                ),
                4.0 * scale,
                scale,
            ));
            assert_pair(torus, second_torus, 2, scale, far);
        }
    }
}

#[test]
fn generic_intersection_stops_at_the_exact_work_budget() {
    let frame = frame(1.0, false);
    let torus = GeomSurface::torus(ToroidalSurface::new(frame, 4.0, 1.0));
    let oblique_plane = GeomSurface::plane(Plane::from_point_normal(
        frame.location(),
        Dir::new(0.0, 0.0, 1.0),
    ));
    let mut budget = GeometryWorkBudget::new(1);
    let error =
        surface_surface_with_budget(&torus, &oblique_plane, 1.0e-7, &mut budget).unwrap_err();
    assert_eq!(
        error,
        BandTopologyError::OperationBudgetExhausted {
            stage: GeometryWorkStage::Intersection,
            limit: 1,
            consumed: 1,
        }
    );
}
