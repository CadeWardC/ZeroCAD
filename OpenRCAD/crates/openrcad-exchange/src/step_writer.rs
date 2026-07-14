#![forbid(unsafe_code)]
//! STEP AP242 (ISO 10303-21) B-Rep Writer.

use std::collections::HashMap;
use std::fs::File;
use std::io::{self, Write};

use openrcad_foundation::{Ax22d, Ax3, Dir, Dir2d, Pnt, Pnt2d, TolerancePolicy};
use openrcad_geom::{GeomCurve, GeomSurface};
use openrcad_geom2d::{BSplineCurve2d, Curve2d, GeomCurve2d};
use openrcad_topo::{PcurveData, Solid};

struct StepWriter {
    next_id: u32,
    lines: Vec<String>,
}

impl StepWriter {
    fn new() -> Self {
        Self {
            next_id: 1,
            lines: Vec::new(),
        }
    }

    fn alloc_id(&mut self) -> u32 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    fn write_line(&mut self, id: u32, content: String) {
        self.lines.push(format!("#{} = {};", id, content));
    }

    fn write_point(&mut self, p: Pnt) -> u32 {
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
        id
    }

    fn write_direction(&mut self, d: Dir) -> u32 {
        let id = self.alloc_id();
        self.write_line(
            id,
            format!("DIRECTION('', ({}, {}, {}))", f(d.x()), f(d.y()), f(d.z())),
        );
        id
    }

    fn write_vector(&mut self, d: Dir, mag: f64) -> u32 {
        let dir_id = self.write_direction(d);
        let id = self.alloc_id();
        self.write_line(id, format!("VECTOR('', #{}, {})", dir_id, f(mag)));
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
        let id = self.alloc_id();
        self.write_line(
            id,
            format!("CARTESIAN_POINT('', ({}, {}))", f(point.x()), f(point.y())),
        );
        id
    }

    fn write_direction2d(&mut self, direction: Dir2d) -> u32 {
        let id = self.alloc_id();
        self.write_line(
            id,
            format!(
                "DIRECTION('', ({}, {}))",
                f(direction.x()),
                f(direction.y())
            ),
        );
        id
    }

    fn write_vector2d(&mut self, direction: Dir2d) -> u32 {
        let direction_id = self.write_direction2d(direction);
        let id = self.alloc_id();
        self.write_line(id, format!("VECTOR('', #{direction_id}, 1.0)"));
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
            .join(", ");
        let multiplicities = curve
            .multiplicities()
            .iter()
            .map(usize::to_string)
            .collect::<Vec<_>>()
            .join(", ");
        let knots = curve
            .knots()
            .iter()
            .map(|value| f(*value))
            .collect::<Vec<_>>()
            .join(", ");
        let id = self.alloc_id();
        if let Some(weights) = curve.weights() {
            let weights = weights
                .iter()
                .map(|value| f(*value))
                .collect::<Vec<_>>()
                .join(", ");
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
                self.write_line(
                    id,
                    format!("CIRCLE('', #{axis}, {})", f(circle.radius())),
                );
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
                let parameter_scale =
                    if canonical_y.dot(&ellipse.position().y_direction()) >= 0.0 {
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
            GeomCurve2d::Parabola(_) | GeomCurve2d::Hyperbola(_) => {
                let count = 65;
                let poles = (0..count)
                    .map(|index| {
                        let fraction = index as f64 / (count - 1) as f64;
                        curve.point(range.0 + fraction * (range.1 - range.0))
                    })
                    .collect::<Vec<_>>();
                let knots = (0..count).map(|index| index as f64).collect::<Vec<_>>();
                let multiplicities = (0..count)
                    .map(|index| if index == 0 || index == count - 1 { 2 } else { 1 })
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
        let representation = self.alloc_id();
        self.write_line(
            representation,
            format!("DEFINITIONAL_REPRESENTATION('', (#{trimmed}), $)"),
        );
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
                    .join(", ");
                let mult_str = b
                    .multiplicities()
                    .iter()
                    .map(|m: &usize| m.to_string())
                    .collect::<Vec<String>>()
                    .join(", ");
                let knot_str = b
                    .knots()
                    .iter()
                    .map(|k: &f64| f(*k))
                    .collect::<Vec<String>>()
                    .join(", ");

                let id = self.alloc_id();
                if let Some(weights) = b.weights() {
                    let weight_str = weights
                        .iter()
                        .map(|w: &f64| f(*w))
                        .collect::<Vec<String>>()
                        .join(", ");
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
                                .join(", ")
                        )
                    })
                    .collect::<Vec<String>>()
                    .join(", ");
                let u_mult_str = b
                    .u_multiplicities()
                    .iter()
                    .map(|m: &usize| m.to_string())
                    .collect::<Vec<String>>()
                    .join(", ");
                let v_mult_str = b
                    .v_multiplicities()
                    .iter()
                    .map(|m: &usize| m.to_string())
                    .collect::<Vec<String>>()
                    .join(", ");
                let u_knot_str = b
                    .u_knots()
                    .iter()
                    .map(|k: &f64| f(*k))
                    .collect::<Vec<String>>()
                    .join(", ");
                let v_knot_str = b
                    .v_knots()
                    .iter()
                    .map(|k: &f64| f(*k))
                    .collect::<Vec<String>>()
                    .join(", ");

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
                                    .join(", ")
                            )
                        })
                        .collect::<Vec<String>>()
                        .join(", ");
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

    let mut vertex_map = HashMap::new();
    let mut edge_map = HashMap::new();
    let mut loop_map = HashMap::new();
    let mut face_map = HashMap::new();
    let mut shell_map = HashMap::new();
    let mut surface_map = HashMap::new();

    // 1. Write vertices
    for (v_id, v_data) in &brep.vertices {
        let pt_id = writer.write_point(v_data.point);
        let v_step_id = writer.alloc_id();
        writer.write_line(v_step_id, format!("VERTEX_POINT('', #{})", pt_id));
        vertex_map.insert(v_id, v_step_id);
    }

    // 2. Write carrying surfaces first so each edge can reference every
    // face-local PCURVE associated with its 3D curve.
    for (face_id, face) in &brep.faces {
        let surface = face.surface.as_ref().ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("STEP face {face_id:?} has no carrying surface"),
            )
        })?;
        surface_map.insert(face_id, writer.write_surface(surface));
    }

    let mut edge_pcurves = HashMap::<_, Vec<(u32, _)>>::new();
    for (face_id, face) in &brep.faces {
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
                edge_pcurves
                    .entry(coedge.id)
                    .or_default()
                    .push((surface, pcurve));
            }
        }
    }

    // 3. Write 3D edges with their stored 2D representations.
    for (e_id, e_data) in &brep.edges {
        let start_v = vertex_map[&e_data.start];
        let end_v = vertex_map[&e_data.end];
        let curve_3d = if let Some(ref c) = e_data.curve {
            writer.write_curve_ranged(c, Some((e_data.first, e_data.last)))
        } else {
            // Degenerate edge: write a dummy line at start point
            let p = brep.vertices[e_data.start].point;
            let loc_id = writer.write_point(p);
            let vec_id = writer.write_vector(Dir::new(1.0, 0.0, 0.0), 0.0);
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
                .map(|(surface, pcurve)| writer.write_pcurve(*surface, &brep.pcurves[*pcurve]))
                .map(|id| format!("#{id}"))
                .collect::<Vec<_>>()
                .join(", ");
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
                    if is_seam { "SEAM_CURVE" } else { "SURFACE_CURVE" }
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
        edge_map.insert(e_id, edge_step_id);
    }

    // 4. Write loops
    for (l_id, l_data) in &brep.loops {
        let mut oriented_edge_ids = Vec::new();
        for oe in &l_data.edges {
            let edge_step_id = edge_map[&oe.id];
            let same_sense = if oe.orientation.is_forward() {
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
            .join(", ");
        writer.write_line(loop_step_id, format!("EDGE_LOOP('', ({}))", oe_list));
        loop_map.insert(l_id, loop_step_id);
    }

    // 5. Write faces
    for (f_id, f_data) in &brep.faces {
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
            .join(", ");
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
    for (sh_id, sh_data) in &brep.shells {
        let face_list = sh_data
            .faces
            .iter()
            .map(|f_id| format!("#{}", face_map[f_id]))
            .collect::<Vec<_>>()
            .join(", ");
        let shell_step_id = writer.alloc_id();
        writer.write_line(shell_step_id, format!("CLOSED_SHELL('', ({}))", face_list));
        shell_map.insert(sh_id, shell_step_id);
    }

    // 7. Write solids
    let solid_data = &brep.solids[solid.id()];
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
    writeln!(file, "FILE_DESCRIPTION(('OpenRCAD STEP Model'),'2;1');")?;
    writeln!(
        file,
        "FILE_NAME('{}','2026-06-19T00:00:00',('OpenRCAD'),('OpenRCAD team'),'OpenRCAD','OpenRCAD','');",
        path
    )?;
    writeln!(
        file,
        "FILE_SCHEMA(('AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF {{ 1 0 10303 242 1 1 1 }}'));"
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
