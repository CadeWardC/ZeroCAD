#![forbid(unsafe_code)]
//! STEP AP242 (ISO 10303-21) B-Rep Writer.

use std::collections::{HashMap, HashSet};
use std::fs::File;
use std::io::{self, Write};

use openrcad_foundation::{Ax22d, Ax3, Dir, Dir2d, Pnt, Pnt2d, TolerancePolicy};
use openrcad_geom::{Curve, GeomCurve, GeomSurface};
use openrcad_geom2d::{BSplineCurve2d, Curve2d, GeomCurve2d};
use openrcad_topo::{PcurveData, Solid};

struct StepWriter {
    next_id: u32,
    lines: Vec<String>,
    points_3d: HashMap<[u64; 3], u32>,
    directions_3d: HashMap<[u64; 3], u32>,
    vectors_3d: HashMap<(u32, u64), u32>,
    points_2d: HashMap<[u64; 2], u32>,
    directions_2d: HashMap<[u64; 2], u32>,
    vectors_2d: HashMap<u32, u32>,
    pcurve_representations: Vec<(PcurveData, u32)>,
}

impl StepWriter {
    fn new() -> Self {
        Self {
            next_id: 1,
            lines: Vec::new(),
            points_3d: HashMap::new(),
            directions_3d: HashMap::new(),
            vectors_3d: HashMap::new(),
            points_2d: HashMap::new(),
            directions_2d: HashMap::new(),
            vectors_2d: HashMap::new(),
            pcurve_representations: Vec::new(),
        }
    }

    fn alloc_id(&mut self) -> u32 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    fn write_line(&mut self, id: u32, content: String) {
        self.lines.push(format!("#{}={};", id, content));
    }

    fn write_point(&mut self, p: Pnt) -> u32 {
        let key = [float_key(p.x()), float_key(p.y()), float_key(p.z())];
        if let Some(&id) = self.points_3d.get(&key) {
            return id;
        }
        let id = self.alloc_id();
        self.write_line(
            id,
            format!(
                "CARTESIAN_POINT('', ({}, {}, {}))",
                f(p.x()),
                f(p.y()),
                f(p.z())
            ),
        );
        self.points_3d.insert(key, id);
        id
    }

    fn write_direction(&mut self, d: Dir) -> u32 {
        let key = [float_key(d.x()), float_key(d.y()), float_key(d.z())];
        if let Some(&id) = self.directions_3d.get(&key) {
            return id;
        }
        let id = self.alloc_id();
        self.write_line(
            id,
            format!("DIRECTION('', ({}, {}, {}))", f(d.x()), f(d.y()), f(d.z())),
        );
        self.directions_3d.insert(key, id);
        id
    }

    fn write_vector(&mut self, d: Dir, mag: f64) -> u32 {
        let dir_id = self.write_direction(d);
        let key = (dir_id, float_key(mag));
        if let Some(&id) = self.vectors_3d.get(&key) {
            return id;
        }
        let id = self.alloc_id();
        self.write_line(id, format!("VECTOR('', #{}, {})", dir_id, f(mag)));
        self.vectors_3d.insert(key, id);
        id
    }

    fn write_axis2_placement_3d(&mut self, pos: &Ax3) -> u32 {
        let loc_id = self.write_point(pos.location());
        let axis_id = self.write_direction(pos.direction());
        let ref_dir_id = self.write_direction(pos.x_direction());
        let id = self.alloc_id();
        self.write_line(
            id,
            format!(
                "AXIS2_PLACEMENT_3D('', #{}, #{}, #{})",
                loc_id, axis_id, ref_dir_id
            ),
        );
        id
    }

    fn write_point2d(&mut self, point: Pnt2d) -> u32 {
        let key = [float_key(point.x()), float_key(point.y())];
        if let Some(&id) = self.points_2d.get(&key) {
            return id;
        }
        let id = self.alloc_id();
        self.write_line(
            id,
            format!("CARTESIAN_POINT('', ({}, {}))", f(point.x()), f(point.y())),
        );
        self.points_2d.insert(key, id);
        id
    }

    fn write_direction2d(&mut self, direction: Dir2d) -> u32 {
        let key = [float_key(direction.x()), float_key(direction.y())];
        if let Some(&id) = self.directions_2d.get(&key) {
            return id;
        }
        let id = self.alloc_id();
        self.write_line(
            id,
            format!(
                "DIRECTION('', ({}, {}))",
                f(direction.x()),
                f(direction.y())
            ),
        );
        self.directions_2d.insert(key, id);
        id
    }

    fn write_vector2d(&mut self, direction: Dir2d) -> u32 {
        let direction_id = self.write_direction2d(direction);
        if let Some(&id) = self.vectors_2d.get(&direction_id) {
            return id;
        }
        let id = self.alloc_id();
        self.write_line(id, format!("VECTOR('', #{direction_id}, 1.0)"));
        self.vectors_2d.insert(direction_id, id);
        id
    }

    fn write_axis2_placement_2d(&mut self, position: Ax22d) -> u32 {
        let location = self.write_point2d(position.location());
        let reference = self.write_direction2d(position.x_direction());
        let id = self.alloc_id();
        self.write_line(
            id,
            format!("AXIS2_PLACEMENT_2D('', #{location}, #{reference})"),
        );
        id
    }

    fn write_bspline_curve2d(&mut self, curve: &BSplineCurve2d) -> u32 {
        let poles = curve
            .poles()
            .iter()
            .map(|point| self.write_point2d(*point))
            .map(|id| format!("#{id}"))
            .collect::<Vec<_>>()
            .join(",");
        let multiplicities = curve
            .multiplicities()
            .iter()
            .map(usize::to_string)
            .collect::<Vec<_>>()
            .join(",");
        let knots = curve
            .knots()
            .iter()
            .map(|value| f(*value))
            .collect::<Vec<_>>()
            .join(",");
        let id = self.alloc_id();
        if let Some(weights) = curve.weights() {
            let weights = weights
                .iter()
                .map(|value| f(*value))
                .collect::<Vec<_>>()
                .join(",");
            self.write_line(
                id,
                format!(
                    "(B_SPLINE_CURVE({}, ({}), .UNSPECIFIED., .F., .F.) B_SPLINE_CURVE_WITH_KNOTS(({}), ({}), .UNSPECIFIED.) BOUNDED_CURVE() CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE(({})) REPRESENTATION_ITEM())",
                    curve.degree(), poles, multiplicities, knots, weights
                ),
            );
        } else {
            self.write_line(
                id,
                format!(
                    "B_SPLINE_CURVE_WITH_KNOTS('', {}, ({}), .UNSPECIFIED., .F., .F., ({}), ({}), .UNSPECIFIED.)",
                    curve.degree(), poles, multiplicities, knots
                ),
            );
        }
        id
    }

    fn write_curve2d(&mut self, curve: &GeomCurve2d, range: (f64, f64)) -> (u32, f64) {
        match curve {
            GeomCurve2d::Line(line) => {
                let location = self.write_point2d(line.location());
                let vector = self.write_vector2d(line.direction());
                let id = self.alloc_id();
                self.write_line(id, format!("LINE('', #{location}, #{vector})"));
                (id, 1.0)
            }
            GeomCurve2d::Circle(circle) => {
                let axis = self.write_axis2_placement_2d(circle.position());
                let id = self.alloc_id();
                self.write_line(id, format!("CIRCLE('', #{axis}, {})", f(circle.radius())));
                let canonical_y = circle.position().x_direction().rotated_90();
                let parameter_scale = if canonical_y.dot(&circle.position().y_direction()) >= 0.0 {
                    1.0
                } else {
                    -1.0
                };
                (id, parameter_scale)
            }
            GeomCurve2d::Ellipse(ellipse) => {
                let axis = self.write_axis2_placement_2d(ellipse.position());
                let id = self.alloc_id();
                self.write_line(
                    id,
                    format!(
                        "ELLIPSE('', #{axis}, {}, {})",
                        f(ellipse.major_radius()),
                        f(ellipse.minor_radius())
                    ),
                );
                let canonical_y = ellipse.position().x_direction().rotated_90();
                let parameter_scale = if canonical_y.dot(&ellipse.position().y_direction()) >= 0.0 {
                    1.0
                } else {
                    -1.0
                };
                (id, parameter_scale)
            }
            GeomCurve2d::BSpline(curve) => (self.write_bspline_curve2d(curve), 1.0),
            // STEP readers in the supported subset do not agree on analytic
            // 2D conics beyond circles/ellipses. Preserve the stored interval
            // as a deterministic degree-1 curve instead.
            GeomCurve2d::Parabola(_)
            | GeomCurve2d::Hyperbola(_)
            | GeomCurve2d::TorusPlaneSection(_)
            | GeomCurve2d::PlaneTorusSection(_)
            | GeomCurve2d::CylinderPlaneSection(_)
            | GeomCurve2d::SphereGreatCircle(_)
            | GeomCurve2d::EndpointCorrected(_) => {
                // These section pcurves are exact inside OpenRCAD but have no
                // portable AP242 analytic entity. Export a denser,
                // deterministic representation without changing the durable
                // in-memory B-Rep curve.
                let count = if matches!(
                    curve,
                    GeomCurve2d::TorusPlaneSection(_)
                        | GeomCurve2d::PlaneTorusSection(_)
                        | GeomCurve2d::CylinderPlaneSection(_)
                        | GeomCurve2d::SphereGreatCircle(_)
                        | GeomCurve2d::EndpointCorrected(_)
                ) {
                    257
                } else {
                    65
                };
                let poles = (0..count)
                    .map(|index| {
                        let fraction = index as f64 / (count - 1) as f64;
                        curve.point(range.0 + fraction * (range.1 - range.0))
                    })
                    .collect::<Vec<_>>();
                let knots = (0..count).map(|index| index as f64).collect::<Vec<_>>();
                let multiplicities = (0..count)
                    .map(|index| {
                        if index == 0 || index == count - 1 {
                            2
                        } else {
                            1
                        }
                    })
                    .collect::<Vec<_>>();
                (
                    self.write_bspline_curve2d(&BSplineCurve2d::new(
                        1,
                        poles,
                        None,
                        knots,
                        multiplicities,
                    )),
                    1.0,
                )
            }
        }
    }

    fn write_pcurve(&mut self, surface: u32, pcurve: &PcurveData) -> u32 {
        let representation = if let Some(representation) = self
            .pcurve_representations
            .iter()
            .find_map(|(stored, id)| (stored == pcurve).then_some(*id))
        {
            representation
        } else {
            let curve = match &pcurve.curve {
                GeomCurve2d::Line(line) => {
                    // A degree-one B-spline carries its finite parameter bounds in
                    // its knot vector, so a separate (and very verbose)
                    // TRIMMED_CURVE is unnecessary. This is especially valuable
                    // for planar solids, whose many linear coedges remain fully
                    // authoritative while producing compact STEP payloads.
                    let first = pcurve.first.min(pcurve.last);
                    let last = pcurve.first.max(pcurve.last);
                    self.write_bspline_curve2d(&BSplineCurve2d::new(
                        1,
                        vec![line.point(first), line.point(last)],
                        None,
                        vec![first, last],
                        vec![2, 2],
                    ))
                }
                _ => {
                    let (basis, parameter_scale) =
                        self.write_curve2d(&pcurve.curve, (pcurve.first, pcurve.last));
                    let trimmed = self.alloc_id();
                    self.write_line(
                        trimmed,
                        format!(
                            "TRIMMED_CURVE('', #{basis}, (PARAMETER_VALUE({})), (PARAMETER_VALUE({})), .T., .PARAMETER.)",
                            f(parameter_scale * pcurve.first),
                            f(parameter_scale * pcurve.last)
                        ),
                    );
                    trimmed
                }
            };
            let representation = self.alloc_id();
            self.write_line(
                representation,
                format!("DEFINITIONAL_REPRESENTATION('', (#{curve}), $)"),
            );
            self.pcurve_representations
                .push((pcurve.clone(), representation));
            representation
        };
        let id = self.alloc_id();
        self.write_line(id, format!("PCURVE('', #{surface}, #{representation})"));
        id
    }

    fn write_curve_ranged(&mut self, curve: &GeomCurve, range: Option<(f64, f64)>) -> u32 {
        match curve {
            GeomCurve::Line(l) => {
                let loc_id = self.write_point(l.location());
                let vec_id = self.write_vector(l.direction(), 1.0);
                let id = self.alloc_id();
                self.write_line(id, format!("LINE('', #{}, #{})", loc_id, vec_id));
                id
            }
            GeomCurve::Circle(c) => {
                let axis_id = self.write_axis2_placement_3d(&c.position());
                let id = self.alloc_id();
                self.write_line(id, format!("CIRCLE('', #{}, {})", axis_id, f(c.radius())));
                id
            }
            GeomCurve::Ellipse(e) => {
                let axis_id = self.write_axis2_placement_3d(&e.position());
                let id = self.alloc_id();
                self.write_line(
                    id,
                    format!(
                        "ELLIPSE('', #{}, {}, {})",
                        axis_id,
                        f(e.major_radius()),
                        f(e.minor_radius())
                    ),
                );
                id
            }
            GeomCurve::Parabola(p) => {
                let axis_id = self.write_axis2_placement_3d(&p.position());
                let id = self.alloc_id();
                self.write_line(id, format!("PARABOLA('', #{}, {})", axis_id, f(p.focal())));
                id
            }
            GeomCurve::Hyperbola(h) => {
                let axis_id = self.write_axis2_placement_3d(&h.position());
                let id = self.alloc_id();
                self.write_line(
                    id,
                    format!(
                        "HYPERBOLA('', #{}, {}, {})",
                        axis_id,
                        f(h.major_radius()),
                        f(h.minor_radius())
                    ),
                );
                id
            }
            GeomCurve::BSpline(b) => {
                let pole_ids: Vec<u32> = b.poles().iter().map(|p| self.write_point(*p)).collect();
                let pole_str = pole_ids
                    .iter()
                    .map(|id: &u32| format!("#{}", id))
                    .collect::<Vec<String>>()
                    .join(",");
                let mult_str = b
                    .multiplicities()
                    .iter()
                    .map(|m: &usize| m.to_string())
                    .collect::<Vec<String>>()
                    .join(",");
                let knot_str = b
                    .knots()
                    .iter()
                    .map(|k: &f64| f(*k))
                    .collect::<Vec<String>>()
                    .join(",");

                let id = self.alloc_id();
                if let Some(weights) = b.weights() {
                    let weight_str = weights
                        .iter()
                        .map(|w: &f64| f(*w))
                        .collect::<Vec<String>>()
                        .join(",");
                    let content = format!(
                        "(\n\
                        B_SPLINE_CURVE({}, ({}), .UNSPECIFIED., .F., .F.)\n\
                        B_SPLINE_CURVE_WITH_KNOTS(({}), ({}), .UNSPECIFIED.)\n\
                        BOUNDED_CURVE()\n\
                        CURVE()\n\
                        GEOMETRIC_REPRESENTATION_ITEM()\n\
                        RATIONAL_B_SPLINE_CURVE(({}))\n\
                        REPRESENTATION_ITEM()\n\
                        )",
                        b.degree(),
                        pole_str,
                        mult_str,
                        knot_str,
                        weight_str
                    );
                    self.write_line(id, content);
                } else {
                    let content = format!(
                        "B_SPLINE_CURVE_WITH_KNOTS('', {}, ({}), .UNSPECIFIED., .F., .F., ({}), ({}), .UNSPECIFIED.)",
                        b.degree(),
                        pole_str,
                        mult_str,
                        knot_str
                    );
                    self.write_line(id, content);
                }
                id
            }
            // STEP has no helix entity: approximate over the edge's parameter
            // range (one turn if unknown) as a polyline-degree B-spline.
            GeomCurve::Helix(h) => {
                use openrcad_geom::Curve as _;
                let (t0, t1) = range.unwrap_or((0.0, 2.0 * std::f64::consts::PI));
                let turns = ((t1 - t0).abs() / (2.0 * std::f64::consts::PI)).max(1.0);
                let n = ((turns * 24.0).ceil() as usize).clamp(8, 512);
                let mut poles = Vec::with_capacity(n);
                let mut knots = Vec::with_capacity(n);
                let mut mults = Vec::with_capacity(n);
                for i in 0..n {
                    let t = t0 + (t1 - t0) * (i as f64) / ((n - 1) as f64);
                    poles.push(h.point(t));
                    knots.push(t);
                    mults.push(if i == 0 || i == n - 1 { 2 } else { 1 });
                }
                let b = openrcad_geom::BSplineCurve::new(1, poles, None, knots, mults);
                self.write_curve_ranged(&GeomCurve::BSpline(b), None)
            }
            GeomCurve::Reparametrized(curve) => {
                let (first, last) = range.unwrap_or_else(|| curve.bounds());
                self.write_curve_ranged(
                    curve.curve(),
                    Some((curve.target_parameter(first), curve.target_parameter(last))),
                )
            }
            // AP242 has no portable analytic entities for these exact torus
            // boundary families. Approximate only the exchange representation;
            // the operation-owned OpenRCAD B-Rep remains exact.
            GeomCurve::TorusPlaneSection(_) | GeomCurve::TorusSurfaceCurve(_) => {
                let (t0, t1) = range.unwrap_or_else(|| curve.bounds());
                let count = 257;
                let poles = (0..count)
                    .map(|index| {
                        let parameter = t0 + (t1 - t0) * index as f64 / (count - 1) as f64;
                        curve.point(parameter)
                    })
                    .collect::<Vec<_>>();
                let knots = (0..count)
                    .map(|index| t0 + (t1 - t0) * index as f64 / (count - 1) as f64)
                    .collect::<Vec<_>>();
                let multiplicities = (0..count)
                    .map(|index| {
                        if index == 0 || index == count - 1 {
                            2
                        } else {
                            1
                        }
                    })
                    .collect::<Vec<_>>();
                let approximation =
                    openrcad_geom::BSplineCurve::new(1, poles, None, knots, multiplicities);
                self.write_curve_ranged(&GeomCurve::BSpline(approximation), None)
            }
        }
    }

    fn write_surface(&mut self, surface: &GeomSurface) -> u32 {
        match surface {
            GeomSurface::Plane(p) => {
                let axis_id = self.write_axis2_placement_3d(&p.position());
                let id = self.alloc_id();
                self.write_line(id, format!("PLANE('', #{})", axis_id));
                id
            }
            GeomSurface::Cylinder(c) => {
                let axis_id = self.write_axis2_placement_3d(&c.position());
                let id = self.alloc_id();
                self.write_line(
                    id,
                    format!("CYLINDRICAL_SURFACE('', #{}, {})", axis_id, f(c.radius())),
                );
                id
            }
            GeomSurface::Cone(co) => {
                let axis_id = self.write_axis2_placement_3d(&co.position());
                let id = self.alloc_id();
                self.write_line(
                    id,
                    format!(
                        "CONICAL_SURFACE('', #{}, {}, {})",
                        axis_id,
                        f(co.ref_radius()),
                        f(co.semi_angle())
                    ),
                );
                id
            }
            GeomSurface::Sphere(s) => {
                let axis_id = self.write_axis2_placement_3d(&s.position());
                let id = self.alloc_id();
                self.write_line(
                    id,
                    format!("SPHERICAL_SURFACE('', #{}, {})", axis_id, f(s.radius())),
                );
                id
            }
            GeomSurface::Torus(t) => {
                let axis_id = self.write_axis2_placement_3d(&t.position());
                let id = self.alloc_id();
                self.write_line(
                    id,
                    format!(
                        "TOROIDAL_SURFACE('', #{}, {}, {})",
                        axis_id,
                        f(t.major_radius()),
                        f(t.minor_radius())
                    ),
                );
                id
            }
            GeomSurface::BSpline(b) => {
                let pole_ids: Vec<Vec<u32>> = b
                    .poles()
                    .iter()
                    .map(|row: &Vec<Pnt>| {
                        row.iter()
                            .map(|p: &Pnt| self.write_point(*p))
                            .collect::<Vec<u32>>()
                    })
                    .collect::<Vec<Vec<u32>>>();
                let pole_str = pole_ids
                    .iter()
                    .map(|row: &Vec<u32>| {
                        format!(
                            "({})",
                            row.iter()
                                .map(|id: &u32| format!("#{}", id))
                                .collect::<Vec<String>>()
                                .join(",")
                        )
                    })
                    .collect::<Vec<String>>()
                    .join(",");
                let u_mult_str = b
                    .u_multiplicities()
                    .iter()
                    .map(|m: &usize| m.to_string())
                    .collect::<Vec<String>>()
                    .join(",");
                let v_mult_str = b
                    .v_multiplicities()
                    .iter()
                    .map(|m: &usize| m.to_string())
                    .collect::<Vec<String>>()
                    .join(",");
                let u_knot_str = b
                    .u_knots()
                    .iter()
                    .map(|k: &f64| f(*k))
                    .collect::<Vec<String>>()
                    .join(",");
                let v_knot_str = b
                    .v_knots()
                    .iter()
                    .map(|k: &f64| f(*k))
                    .collect::<Vec<String>>()
                    .join(",");

                let id = self.alloc_id();
                if let Some(weights) = b.weights() {
                    let weight_str = weights
                        .iter()
                        .map(|row: &Vec<f64>| {
                            format!(
                                "({})",
                                row.iter()
                                    .map(|w: &f64| f(*w))
                                    .collect::<Vec<String>>()
                                    .join(",")
                            )
                        })
                        .collect::<Vec<String>>()
                        .join(",");
                    let content = format!(
                        "(\n\
                        BOUNDED_SURFACE()\n\
                        B_SPLINE_SURFACE({}, {}, ({}), .UNSPECIFIED., .F., .F., .F.)\n\
                        B_SPLINE_SURFACE_WITH_KNOTS(({}), ({}), ({}), ({}), .UNSPECIFIED.)\n\
                        GEOMETRIC_REPRESENTATION_ITEM()\n\
                        RATIONAL_B_SPLINE_SURFACE(({}))\n\
                        REPRESENTATION_ITEM()\n\
                        SURFACE()\n\
                        )",
                        b.u_degree(),
                        b.v_degree(),
                        pole_str,
                        u_mult_str,
                        v_mult_str,
                        u_knot_str,
                        v_knot_str,
                        weight_str
                    );
                    self.write_line(id, content);
                } else {
                    let content = format!(
                        "B_SPLINE_SURFACE_WITH_KNOTS('', {}, {}, ({}), .UNSPECIFIED., .F., .F., .F., ({}), ({}), ({}), ({}), .UNSPECIFIED.)",
                        b.u_degree(),
                        b.v_degree(),
                        pole_str,
                        u_mult_str,
                        v_mult_str,
                        u_knot_str,
                        v_knot_str
                    );
                    self.write_line(id, content);
                }
                id
            }
            GeomSurface::Gregory(_) | GeomSurface::Offset(_) | GeomSurface::Ruled(_) => {
                if let Some(bspline) = surface.to_bspline() {
                    self.write_surface(&GeomSurface::BSpline(bspline))
                } else {
                    0
                }
            }
        }
    }
}

#[inline]
fn float_key(value: f64) -> u64 {
    if value == 0.0 {
        0
    } else {
        value.to_bits()
    }
}

/// Helper to format float to standard scientific/decimal form.
fn f(val: f64) -> String {
    if val.is_nan() {
        return "0.0".to_string();
    }
    let s = format!("{:.12}", val);
    let trimmed = s.trim_end_matches('0');
    if trimmed.ends_with('.') {
        format!("{}0", trimmed)
    } else {
        trimmed.to_string()
    }
}

/// Write `solid` to `path` as a STEP file (AP242 B-Rep).
pub fn write_step(solid: &Solid, path: &str) -> io::Result<()> {
    solid
        .validate_strict_with_policy(&TolerancePolicy::STANDARD)
        .map_err(|error| io::Error::new(io::ErrorKind::InvalidInput, error.to_string()))?;
    let health = solid.health_report_with_policy(&TolerancePolicy::STANDARD);
    if !health.is_healthy() || !solid.is_watertight_with_policy(&TolerancePolicy::STANDARD) {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            format!("STEP output requires a healthy watertight solid: {health:?}"),
        ));
    }
    let mut writer = StepWriter::new();
    let brep = solid.brep();
    let solid_data = &brep.solids[solid.id()];

    // Traverse only this solid's reachable topology. B-Rep arenas may retain
    // construction intermediates, which are not part of the exported shape.
    let shell_ids = solid_data.shells.clone();
    let mut seen_faces = HashSet::new();
    let face_ids = shell_ids
        .iter()
        .flat_map(|shell_id| brep.shells[*shell_id].faces.iter().copied())
        .filter(|face_id| seen_faces.insert(*face_id))
        .collect::<Vec<_>>();
    let mut seen_loops = HashSet::new();
    let loop_ids = face_ids
        .iter()
        .flat_map(|face_id| {
            let face = &brep.faces[*face_id];
            face.outer_wire
                .into_iter()
                .chain(face.inner_wires.iter().copied())
        })
        .filter(|loop_id| seen_loops.insert(*loop_id))
        .collect::<Vec<_>>();
    let mut seen_edges = HashSet::new();
    let edge_ids = loop_ids
        .iter()
        .flat_map(|loop_id| brep.loops[*loop_id].edges.iter().map(|coedge| coedge.id))
        .filter(|edge_id| seen_edges.insert(*edge_id))
        .collect::<Vec<_>>();

    // The topology layer permits two face-local edge records to represent one
    // physical shared boundary. STEP models that as one EDGE_CURVE carrying
    // independent PCURVEs. Group by the same endpoint-plus-midpoint identity
    // used by strict manifold validation, and remember whether each local
    // record runs opposite to the chosen representative.
    let grid = 1.0 / TolerancePolicy::STANDARD.approximation;
    let quantize = |point: Pnt| {
        (
            (point.x() * grid).round() as i64,
            (point.y() * grid).round() as i64,
            (point.z() * grid).round() as i64,
        )
    };
    let mut representative_by_key = HashMap::new();
    let mut edge_group = HashMap::new();
    let mut representative_edges = Vec::new();
    for &edge_id in &edge_ids {
        let edge = &brep.edges[edge_id];
        let start = brep.vertices[edge.start].point;
        let end = brep.vertices[edge.end].point;
        let midpoint = edge.curve.as_ref().map_or_else(
            || {
                Pnt::new(
                    0.5 * (start.x() + end.x()),
                    0.5 * (start.y() + end.y()),
                    0.5 * (start.z() + end.z()),
                )
            },
            |curve| curve.point(0.5 * (edge.first + edge.last)),
        );
        let natural_start = quantize(start);
        let natural_end = quantize(end);
        let middle = quantize(midpoint);
        let key = if natural_start <= natural_end {
            (natural_start, natural_end, middle)
        } else {
            (natural_end, natural_start, middle)
        };
        if let Some(&(representative, representative_start, representative_end)) =
            representative_by_key.get(&key)
        {
            let reversed = natural_start == representative_end
                && natural_end == representative_start
                && natural_start != natural_end;
            edge_group.insert(edge_id, (representative, reversed));
        } else {
            representative_by_key.insert(key, (edge_id, natural_start, natural_end));
            edge_group.insert(edge_id, (edge_id, false));
            representative_edges.push(edge_id);
        }
    }

    let mut seen_vertices = HashSet::new();
    let vertex_ids = representative_edges
        .iter()
        .flat_map(|edge_id| {
            let edge = &brep.edges[*edge_id];
            [edge.start, edge.end]
        })
        .filter(|vertex_id| seen_vertices.insert(*vertex_id))
        .collect::<Vec<_>>();

    let mut vertex_map = HashMap::new();
    let mut edge_map = HashMap::new();
    let mut loop_map = HashMap::new();
    let mut face_map = HashMap::new();
    let mut shell_map = HashMap::new();
    let mut surface_map = HashMap::new();

    // 1. Write vertices
    for v_id in vertex_ids {
        let v_data = &brep.vertices[v_id];
        let pt_id = writer.write_point(v_data.point);
        let v_step_id = writer.alloc_id();
        writer.write_line(v_step_id, format!("VERTEX_POINT('', #{})", pt_id));
        vertex_map.insert(v_id, v_step_id);
    }

    // 2. Write carrying surfaces first so each edge can reference every
    // face-local PCURVE associated with its 3D curve.
    for &face_id in &face_ids {
        let face = &brep.faces[face_id];
        let surface = face.surface.as_ref().ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("STEP face {face_id:?} has no carrying surface"),
            )
        })?;
        surface_map.insert(face_id, writer.write_surface(surface));
    }

    let mut edge_pcurves = HashMap::<_, Vec<(u32, PcurveData)>>::new();
    for &face_id in &face_ids {
        let face = &brep.faces[face_id];
        let surface = surface_map[&face_id];
        for loop_id in face
            .outer_wire
            .into_iter()
            .chain(face.inner_wires.iter().copied())
        {
            for coedge in &brep.loops[loop_id].edges {
                let pcurve = coedge.pcurve.ok_or_else(|| {
                    io::Error::new(
                        io::ErrorKind::InvalidInput,
                        format!("STEP coedge {:?} has no pcurve", coedge.id),
                    )
                })?;
                let (representative, reversed) = edge_group[&coedge.id];
                let mut pcurve = brep.pcurves[pcurve].clone();
                if reversed {
                    pcurve = pcurve.reversed();
                }
                edge_pcurves
                    .entry(representative)
                    .or_default()
                    .push((surface, pcurve));
            }
        }
    }

    // 3. Write 3D edges with their stored 2D representations.
    let mut representative_step_edges = HashMap::new();
    for e_id in representative_edges {
        let e_data = &brep.edges[e_id];
        let start_v = vertex_map[&e_data.start];
        let end_v = vertex_map[&e_data.end];
        let curve_3d = if let Some(ref c) = e_data.curve {
            writer.write_curve_ranged(c, Some((e_data.first, e_data.last)))
        } else {
            // A curve-less topological edge is not necessarily geometrically
            // collapsed: apex primitives intentionally store their straight
            // seams by endpoints. Emit that exact chord so a strict reader can
            // project the authoritative pcurve back to the same 3D boundary.
            let start = brep.vertices[e_data.start].point;
            let end = brep.vertices[e_data.end].point;
            let chord = end - start;
            let magnitude = chord.magnitude();
            let direction = chord
                .normalized()
                .unwrap_or_else(|| Dir::new(1.0, 0.0, 0.0));
            let loc_id = writer.write_point(start);
            let vec_id = writer.write_vector(direction, magnitude);
            let line_id = writer.alloc_id();
            writer.write_line(line_id, format!("LINE('', #{}, #{})", loc_id, vec_id));
            line_id
        };
        let associations = edge_pcurves.get(&e_id).cloned().unwrap_or_default();
        let curve_id = if associations.is_empty() {
            curve_3d
        } else {
            let pcurves = associations
                .iter()
                .map(|(surface, pcurve)| writer.write_pcurve(*surface, pcurve))
                .map(|id| format!("#{id}"))
                .collect::<Vec<_>>()
                .join(",");
            let is_seam = associations.iter().enumerate().any(|(index, item)| {
                associations[..index]
                    .iter()
                    .any(|previous| previous.0 == item.0)
            });
            let id = writer.alloc_id();
            writer.write_line(
                id,
                format!(
                    "{}('', #{curve_3d}, ({pcurves}), .PCURVE_S1.)",
                    if is_seam {
                        "SEAM_CURVE"
                    } else {
                        "SURFACE_CURVE"
                    }
                ),
            );
            id
        };

        // EDGE_CURVE same_sense reflects whether the edge runs along the curve's
        // increasing parameter (start -> end with first <= last). Per-use loop
        // orientation is carried separately by each ORIENTED_EDGE below.
        let same_sense = if e_data.first <= e_data.last {
            ".T."
        } else {
            ".F."
        };
        let edge_step_id = writer.alloc_id();
        writer.write_line(
            edge_step_id,
            format!(
                "EDGE_CURVE('', #{}, #{}, #{}, {})",
                start_v, end_v, curve_id, same_sense
            ),
        );
        representative_step_edges.insert(e_id, edge_step_id);
    }
    for (edge_id, (representative, reversed)) in edge_group {
        edge_map.insert(
            edge_id,
            (representative_step_edges[&representative], reversed),
        );
    }

    // 4. Write loops
    for l_id in loop_ids {
        let l_data = &brep.loops[l_id];
        let mut oriented_edge_ids = Vec::new();
        for oe in &l_data.edges {
            let (edge_step_id, representative_reversed) = edge_map[&oe.id];
            let same_sense = if oe.orientation.is_forward() ^ representative_reversed {
                ".T."
            } else {
                ".F."
            };
            let oe_id = writer.alloc_id();
            writer.write_line(
                oe_id,
                format!("ORIENTED_EDGE('', *, *, #{}, {})", edge_step_id, same_sense),
            );
            oriented_edge_ids.push(oe_id);
        }

        let loop_step_id = writer.alloc_id();
        let oe_list = oriented_edge_ids
            .iter()
            .map(|id| format!("#{}", id))
            .collect::<Vec<_>>()
            .join(",");
        writer.write_line(loop_step_id, format!("EDGE_LOOP('', ({}))", oe_list));
        loop_map.insert(l_id, loop_step_id);
    }

    // 5. Write faces
    for f_id in face_ids {
        let f_data = &brep.faces[f_id];
        let surface_id = surface_map[&f_id];

        let mut bound_ids = Vec::new();
        if let Some(outer_l) = f_data.outer_wire {
            let loop_step_id = loop_map[&outer_l];
            let fob_id = writer.alloc_id();
            writer.write_line(
                fob_id,
                format!("FACE_OUTER_BOUND('', #{}, .T.)", loop_step_id),
            );
            bound_ids.push(fob_id);
        }
        for &inner_l in &f_data.inner_wires {
            let loop_step_id = loop_map[&inner_l];
            let fb_id = writer.alloc_id();
            writer.write_line(fb_id, format!("FACE_BOUND('', #{}, .T.)", loop_step_id));
            bound_ids.push(fb_id);
        }

        let same_sense = if f_data.orientation.is_forward() {
            ".T."
        } else {
            ".F."
        };
        let bound_list = bound_ids
            .iter()
            .map(|id| format!("#{}", id))
            .collect::<Vec<_>>()
            .join(",");
        let face_step_id = writer.alloc_id();
        writer.write_line(
            face_step_id,
            format!(
                "ADVANCED_FACE('', ({}), #{}, {})",
                bound_list, surface_id, same_sense
            ),
        );
        face_map.insert(f_id, face_step_id);
    }

    // 6. Write shells
    for sh_id in shell_ids {
        let sh_data = &brep.shells[sh_id];
        let face_list = sh_data
            .faces
            .iter()
            .map(|f_id| format!("#{}", face_map[f_id]))
            .collect::<Vec<_>>()
            .join(",");
        let shell_step_id = writer.alloc_id();
        writer.write_line(shell_step_id, format!("CLOSED_SHELL('', ({}))", face_list));
        shell_map.insert(sh_id, shell_step_id);
    }

    // 7. Write solids
    let shell_step_id = shell_map[&solid_data.shells[0]];
    let solid_step_id = writer.alloc_id();
    writer.write_line(
        solid_step_id,
        format!("MANIFOLD_SOLID_BREP('', #{})", shell_step_id),
    );

    // Save output
    let mut file = File::create(path)?;
    writeln!(file, "ISO-10303-21;")?;
    writeln!(file, "HEADER;")?;
    writeln!(file, "FILE_DESCRIPTION(('OpenRCAD'),'2;1');")?;
    writeln!(
        file,
        "FILE_NAME('OpenRCAD.step','2026-06-19T00:00:00',('OpenRCAD'),(''),'OpenRCAD','OpenRCAD','');"
    )?;
    writeln!(
        file,
        "FILE_SCHEMA(('AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF'));"
    )?;
    writeln!(file, "ENDSEC;")?;
    writeln!(file, "DATA;")?;
    for line in &writer.lines {
        writeln!(file, "{}", line)?;
    }
    writeln!(file, "ENDSEC;")?;
    writeln!(file, "END-ISO-10303-21;")?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use openrcad_foundation::{Ax3, Vec2d};
    use openrcad_geom::TorusPlaneSection;
    use openrcad_geom2d::{PlaneTorusSection2d, TorusPlaneSection2d};

    fn torus_section() -> GeomCurve {
        GeomCurve::torus_plane_section(TorusPlaneSection::new(
            Ax3::new(Pnt::new(3.0, -2.0, 7.0), Dir::dz()),
            8.0,
            2.0,
            Dir::dx(),
            1.0,
            true,
        ))
    }

    #[test]
    fn exact_torus_boundaries_have_deterministic_step_exchange_approximations() {
        let curve = torus_section();
        let torus_pcurve = GeomCurve2d::torus_plane_section(
            TorusPlaneSection2d::new(0.0, 8.0, 2.0, 1.0, true).unwrap(),
        );
        let plane_pcurve = GeomCurve2d::plane_torus_section(
            PlaneTorusSection2d::new(
                Pnt2d::new(3.0, -2.0),
                Vec2d::new(1.0, 0.0),
                Vec2d::new(0.0, 1.0),
                8.0,
                2.0,
                1.0,
                true,
            )
            .unwrap(),
        );
        let range = (0.0, 2.0 * std::f64::consts::PI);

        let serialize = || {
            let mut writer = StepWriter::new();
            writer.write_curve_ranged(&curve, Some(range));
            writer.write_curve2d(&torus_pcurve, range);
            writer.write_curve2d(&plane_pcurve, range);
            writer.lines
        };

        let first = serialize();
        let second = serialize();
        assert_eq!(
            first, second,
            "STEP approximation must be byte deterministic"
        );
        assert_eq!(
            first
                .iter()
                .filter(|line| line.contains("B_SPLINE_CURVE_WITH_KNOTS"))
                .count(),
            3,
            "the exact 3D boundary and both exact pcurves need exchange curves"
        );
        assert!(
            first
                .iter()
                .filter(|line| line.contains("CARTESIAN_POINT"))
                .count()
                >= 3 * 250,
            "torus exchange curves retain their deterministic dense sampling"
        );
        assert!(matches!(curve, GeomCurve::TorusPlaneSection(_)));
        assert!(matches!(torus_pcurve, GeomCurve2d::TorusPlaneSection(_)));
        assert!(matches!(plane_pcurve, GeomCurve2d::PlaneTorusSection(_)));
    }
}
