//! Shared homogeneous B-spline derivative evaluation.
//!
//! Curves and surfaces use the same local de Boor and derivative-control-net
//! machinery.  Coordinates are expressed relative to a nearby control point
//! before they become homogeneous, avoiding cancellation when a small shape is
//! modeled far from the global origin.

use openrcad_foundation::{Pnt, Vec as GeomVec};

pub(crate) type Homogeneous = [f64; 4];

const ZERO_H: Homogeneous = [0.0; 4];

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct HomogeneousSurfaceDerivatives {
    pub(crate) value: Homogeneous,
    pub(crate) du: Homogeneous,
    pub(crate) dv: Homogeneous,
    pub(crate) duu: Homogeneous,
    pub(crate) duv: Homogeneous,
    pub(crate) dvv: Homogeneous,
}

/// Locate the active span, clamping to the B-spline's active parameter range.
pub(crate) fn find_span(
    degree: usize,
    flat_knots: &[f64],
    control_count: usize,
    parameter: f64,
) -> usize {
    let parameter = parameter.clamp(flat_knots[degree], flat_knots[control_count]);
    if parameter >= flat_knots[control_count] {
        return control_count - 1;
    }

    let mut low = degree;
    let mut high = control_count;
    while low + 1 < high {
        let mid = (low + high) / 2;
        if parameter < flat_knots[mid] {
            high = mid;
        } else {
            low = mid;
        }
    }
    low
}

/// Convert a weighted point to homogeneous coordinates in a local frame.
pub(crate) fn homogenize(point: Pnt, weight: f64, origin: Pnt) -> Homogeneous {
    [
        (point.x() - origin.x()) * weight,
        (point.y() - origin.y()) * weight,
        (point.z() - origin.z()) * weight,
        weight,
    ]
}

/// Evaluate value through second derivative from an active homogeneous control
/// polygon. `active` must contain the `degree + 1` controls ending at `span`.
pub(crate) fn evaluate_active_curve(
    degree: usize,
    flat_knots: &[f64],
    span: usize,
    active: &[Homogeneous],
    parameter: f64,
    max_order: usize,
) -> [Homogeneous; 3] {
    debug_assert_eq!(active.len(), degree + 1);
    let max_order = max_order.min(2);
    let parameter = parameter.clamp(
        flat_knots[degree],
        flat_knots[flat_knots.len() - degree - 1],
    );

    let mut result = [ZERO_H; 3];
    result[0] = de_boor_active(degree, flat_knots, span, active, parameter);

    let base_control = span - degree;
    let mut derivative_controls = active.to_vec();
    for order in 1..=max_order.min(degree) {
        let current_degree = degree + 1 - order;
        let mut next = Vec::with_capacity(derivative_controls.len() - 1);
        for local_index in 0..derivative_controls.len() - 1 {
            let control_index = base_control + local_index;
            let denominator =
                flat_knots[control_index + degree + 1] - flat_knots[control_index + order];
            let mut derivative = ZERO_H;
            if denominator != 0.0 {
                let factor = current_degree as f64 / denominator;
                for coordinate in 0..4 {
                    derivative[coordinate] = factor
                        * (derivative_controls[local_index + 1][coordinate]
                            - derivative_controls[local_index][coordinate]);
                }
            }
            next.push(derivative);
        }
        derivative_controls = next;

        let derivative_degree = degree - order;
        let derivative_knots = &flat_knots[order..flat_knots.len() - order];
        result[order] = de_boor_active(
            derivative_degree,
            derivative_knots,
            span - order,
            &derivative_controls,
            parameter,
        );
    }

    result
}

fn de_boor_active(
    degree: usize,
    flat_knots: &[f64],
    span: usize,
    active: &[Homogeneous],
    parameter: f64,
) -> Homogeneous {
    if degree == 0 {
        return active[0];
    }

    let mut values = active.to_vec();
    for level in 1..=degree {
        for local_index in (level..=degree).rev() {
            let knot_index = span - degree + local_index;
            let denominator = flat_knots[knot_index + degree + 1 - level] - flat_knots[knot_index];
            let alpha = if denominator == 0.0 {
                0.0
            } else {
                (parameter - flat_knots[knot_index]) / denominator
            };
            for coordinate in 0..4 {
                values[local_index][coordinate] = (1.0 - alpha)
                    * values[local_index - 1][coordinate]
                    + alpha * values[local_index][coordinate];
            }
        }
    }
    values[degree]
}

pub(crate) fn project_curve(origin: Pnt, homogeneous: [Homogeneous; 3]) -> (Pnt, GeomVec, GeomVec) {
    let value = homogeneous[0];
    let weight = value[3];
    debug_assert!(weight > 0.0 && weight.is_finite());

    let relative_point = [value[0] / weight, value[1] / weight, value[2] / weight];
    let point = Pnt::new(
        origin.x() + relative_point[0],
        origin.y() + relative_point[1],
        origin.z() + relative_point[2],
    );

    let first = quotient_first(homogeneous[1], value[3], relative_point);
    let second = quotient_second(
        homogeneous[2],
        homogeneous[1][3],
        first,
        value[3],
        relative_point,
    );
    (
        point,
        GeomVec::new(first[0], first[1], first[2]),
        GeomVec::new(second[0], second[1], second[2]),
    )
}

pub(crate) fn project_surface(
    origin: Pnt,
    homogeneous: HomogeneousSurfaceDerivatives,
) -> (Pnt, GeomVec, GeomVec, GeomVec, GeomVec, GeomVec) {
    let weight = homogeneous.value[3];
    debug_assert!(weight > 0.0 && weight.is_finite());
    let relative_point = [
        homogeneous.value[0] / weight,
        homogeneous.value[1] / weight,
        homogeneous.value[2] / weight,
    ];
    let point = Pnt::new(
        origin.x() + relative_point[0],
        origin.y() + relative_point[1],
        origin.z() + relative_point[2],
    );

    let du = quotient_first(homogeneous.du, weight, relative_point);
    let dv = quotient_first(homogeneous.dv, weight, relative_point);
    let duu = quotient_second(
        homogeneous.duu,
        homogeneous.du[3],
        du,
        weight,
        relative_point,
    );
    let dvv = quotient_second(
        homogeneous.dvv,
        homogeneous.dv[3],
        dv,
        weight,
        relative_point,
    );

    let mut duv = [0.0; 3];
    for coordinate in 0..3 {
        duv[coordinate] = (homogeneous.duv[coordinate]
            - homogeneous.du[3] * dv[coordinate]
            - homogeneous.dv[3] * du[coordinate]
            - homogeneous.duv[3] * relative_point[coordinate])
            / weight;
    }

    (
        point,
        GeomVec::new(du[0], du[1], du[2]),
        GeomVec::new(dv[0], dv[1], dv[2]),
        GeomVec::new(duu[0], duu[1], duu[2]),
        GeomVec::new(duv[0], duv[1], duv[2]),
        GeomVec::new(dvv[0], dvv[1], dvv[2]),
    )
}

fn quotient_first(numerator_derivative: Homogeneous, weight: f64, point: [f64; 3]) -> [f64; 3] {
    let mut derivative = [0.0; 3];
    for coordinate in 0..3 {
        derivative[coordinate] = (numerator_derivative[coordinate]
            - numerator_derivative[3] * point[coordinate])
            / weight;
    }
    derivative
}

fn quotient_second(
    numerator_second: Homogeneous,
    weight_first: f64,
    first: [f64; 3],
    weight: f64,
    point: [f64; 3],
) -> [f64; 3] {
    let mut second = [0.0; 3];
    for coordinate in 0..3 {
        second[coordinate] = (numerator_second[coordinate]
            - 2.0 * weight_first * first[coordinate]
            - numerator_second[3] * point[coordinate])
            / weight;
    }
    second
}
