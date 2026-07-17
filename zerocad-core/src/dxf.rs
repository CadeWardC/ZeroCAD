//! Planar ASCII DXF to sketch import.
//!
//! The importer deliberately targets the entity subset used for machining and
//! Part Design profiles. Coordinates are converted to ZeroCAD's millimetre base
//! unit, while the source unit code and layer name remain attached to each
//! imported [`crate::sketch::SketchShape`]. Unsupported entities are reported
//! and skipped; malformed supported entities produce diagnostics instead of
//! partially invented geometry.

use crate::sketch::{
    ImportedSketchMetadata, SketchCurves, SketchImportFormat, SketchShape, Spline, SplineContinuity,
};
use std::collections::BTreeMap;
use std::path::Path;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DxfDiagnosticSeverity {
    Info,
    Warning,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DxfDiagnostic {
    pub severity: DxfDiagnosticSeverity,
    pub message: String,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DxfUnit {
    pub code: u16,
    pub name: &'static str,
    pub scale_to_mm: f64,
}

impl DxfUnit {
    pub fn from_insunits(code: u16) -> Self {
        let (name, scale_to_mm) = match code {
            0 => ("Unitless", 1.0),
            1 => ("Inches", 25.4),
            2 => ("Feet", 304.8),
            3 => ("Miles", 1_609_344.0),
            4 => ("Millimeters", 1.0),
            5 => ("Centimeters", 10.0),
            6 => ("Meters", 1_000.0),
            7 => ("Kilometers", 1_000_000.0),
            8 => ("Microinches", 0.000_025_4),
            9 => ("Mils", 0.0254),
            10 => ("Yards", 914.4),
            11 => ("Angstroms", 0.000_000_1),
            12 => ("Nanometers", 0.000_001),
            13 => ("Micrometers", 0.001),
            14 => ("Decimeters", 100.0),
            15 => ("Decameters", 10_000.0),
            16 => ("Hectometers", 100_000.0),
            17 => ("Gigameters", 1_000_000_000_000.0),
            18 => ("Astronomical units", 149_597_870_700_000.0),
            19 => ("Light years", 9.460_730_472_580_8e18),
            20 => ("Parsecs", 3.085_677_581_491_367e19),
            _ => ("Unknown", 1.0),
        };
        Self {
            code,
            name,
            scale_to_mm,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct DxfImport {
    pub shapes: Vec<SketchShape>,
    pub unit: DxfUnit,
    pub diagnostics: Vec<DxfDiagnostic>,
}

impl DxfImport {
    pub fn imported_curve_count(&self) -> usize {
        self.shapes
            .iter()
            .map(|shape| match shape {
                SketchShape::Imported { curves, .. } => {
                    curves.segments.len()
                        + curves.circles.len()
                        + curves.arcs.len()
                        + curves.splines.len()
                }
                _ => 0,
            })
            .sum()
    }
}

#[derive(Debug)]
pub struct DxfError {
    pub line: Option<usize>,
    pub message: String,
}

impl std::fmt::Display for DxfError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if let Some(line) = self.line {
            write!(formatter, "DXF line {line}: {}", self.message)
        } else {
            write!(formatter, "DXF: {}", self.message)
        }
    }
}

impl std::error::Error for DxfError {}

#[derive(Debug, Clone)]
struct Pair {
    code: i16,
    value: String,
    line: usize,
}

pub fn read_dxf_file(path: impl AsRef<Path>) -> Result<DxfImport, DxfError> {
    let path = path.as_ref();
    let bytes = std::fs::read(path).map_err(|error| DxfError {
        line: None,
        message: error.to_string(),
    })?;
    let text = std::str::from_utf8(&bytes).map_err(|error| DxfError {
        line: None,
        message: format!("only ASCII/UTF-8 DXF is supported ({error})"),
    })?;
    read_dxf_str(
        path.file_name()
            .and_then(|name| name.to_str())
            .unwrap_or("import.dxf"),
        text,
    )
}

pub fn read_dxf_str(source_name: &str, text: &str) -> Result<DxfImport, DxfError> {
    let pairs = parse_pairs(text)?;
    let unit = find_unit(&pairs);
    let (start, end) = entities_range(&pairs)?;
    let mut layers: BTreeMap<String, SketchCurves> = BTreeMap::new();
    let mut diagnostics = Vec::new();
    let mut unsupported: BTreeMap<String, usize> = BTreeMap::new();
    let mut index = start;
    while index < end {
        if pairs[index].code != 0 {
            index += 1;
            continue;
        }
        let kind = pairs[index].value.to_ascii_uppercase();
        let record_start = index + 1;
        index = record_start;
        while index < end && pairs[index].code != 0 {
            index += 1;
        }
        let record = &pairs[record_start..index];
        if kind == "POLYLINE" {
            let mut vertices = Vec::new();
            while index < end && pairs[index].code == 0 {
                let nested_kind = pairs[index].value.to_ascii_uppercase();
                if nested_kind == "SEQEND" {
                    index += 1;
                    while index < end && pairs[index].code != 0 {
                        index += 1;
                    }
                    break;
                }
                if nested_kind != "VERTEX" {
                    break;
                }
                let vertex_start = index + 1;
                index = vertex_start;
                while index < end && pairs[index].code != 0 {
                    index += 1;
                }
                vertices.push(&pairs[vertex_start..index]);
            }
            import_polyline(record, &vertices, unit, &mut layers, &mut diagnostics);
            continue;
        }
        if !import_record(&kind, record, unit, &mut layers, &mut diagnostics) {
            *unsupported.entry(kind).or_default() += 1;
        }
    }

    for (kind, count) in unsupported {
        diagnostics.push(DxfDiagnostic {
            severity: DxfDiagnosticSeverity::Warning,
            message: format!(
                "Skipped {count} unsupported {kind} entit{}",
                if count == 1 { "y" } else { "ies" }
            ),
        });
    }
    if unit.code == 0 {
        diagnostics.push(DxfDiagnostic {
            severity: DxfDiagnosticSeverity::Info,
            message: "DXF declares no insertion units; coordinates were interpreted as millimeters"
                .to_string(),
        });
    } else if unit.name == "Unknown" {
        diagnostics.push(DxfDiagnostic {
            severity: DxfDiagnosticSeverity::Warning,
            message: format!(
                "DXF uses unknown INSUNITS code {}; coordinates were interpreted as millimeters",
                unit.code
            ),
        });
    }

    let shapes = layers
        .into_iter()
        .filter(|(_, curves)| !curves.is_empty())
        .map(|(layer, curves)| SketchShape::Imported {
            curves,
            metadata: ImportedSketchMetadata {
                format: SketchImportFormat::Dxf,
                source_name: source_name.to_string(),
                layer,
                source_unit_code: unit.code,
                source_unit_name: unit.name.to_string(),
                source_scale_to_mm: unit.scale_to_mm,
            },
        })
        .collect::<Vec<_>>();
    if shapes.is_empty() {
        return Err(DxfError {
            line: None,
            message: "ENTITIES contains no supported planar profile geometry".to_string(),
        });
    }
    Ok(DxfImport {
        shapes,
        unit,
        diagnostics,
    })
}

fn parse_pairs(text: &str) -> Result<Vec<Pair>, DxfError> {
    let lines: Vec<&str> = text.lines().collect();
    if !lines.len().is_multiple_of(2) {
        return Err(DxfError {
            line: Some(lines.len()),
            message: "group code is missing its value line".to_string(),
        });
    }
    let mut pairs = Vec::with_capacity(lines.len() / 2);
    for index in (0..lines.len()).step_by(2) {
        let code = lines[index].trim().parse::<i16>().map_err(|_| DxfError {
            line: Some(index + 1),
            message: format!("invalid group code {:?}", lines[index].trim()),
        })?;
        pairs.push(Pair {
            code,
            value: lines[index + 1].trim().to_string(),
            line: index + 2,
        });
    }
    Ok(pairs)
}

fn find_unit(pairs: &[Pair]) -> DxfUnit {
    for (index, pair) in pairs.iter().enumerate() {
        if pair.code == 9 && pair.value.eq_ignore_ascii_case("$INSUNITS") {
            if let Some(value) = pairs[index + 1..]
                .iter()
                .take_while(|candidate| candidate.code != 9 && candidate.code != 0)
                .find(|candidate| candidate.code == 70)
                .and_then(|candidate| candidate.value.parse::<u16>().ok())
            {
                return DxfUnit::from_insunits(value);
            }
        }
    }
    DxfUnit::from_insunits(0)
}

fn entities_range(pairs: &[Pair]) -> Result<(usize, usize), DxfError> {
    let mut index = 0;
    while index + 1 < pairs.len() {
        if pairs[index].code == 0
            && pairs[index].value.eq_ignore_ascii_case("SECTION")
            && pairs[index + 1].code == 2
            && pairs[index + 1].value.eq_ignore_ascii_case("ENTITIES")
        {
            let start = index + 2;
            let end = pairs[start..]
                .iter()
                .position(|pair| pair.code == 0 && pair.value.eq_ignore_ascii_case("ENDSEC"))
                .map(|offset| start + offset)
                .unwrap_or(pairs.len());
            return Ok((start, end));
        }
        index += 1;
    }
    Err(DxfError {
        line: None,
        message: "missing ENTITIES section".to_string(),
    })
}

fn import_record(
    kind: &str,
    record: &[Pair],
    unit: DxfUnit,
    layers: &mut BTreeMap<String, SketchCurves>,
    diagnostics: &mut Vec<DxfDiagnostic>,
) -> bool {
    let layer = text_value(record, 8).unwrap_or("0").to_string();
    let curves = layers.entry(layer.clone()).or_default();
    let scale = unit.scale_to_mm as f32;
    let imported = match kind {
        "LINE" => match (point(record, 10, 20, scale), point(record, 11, 21, scale)) {
            (Some(start), Some(end)) if distance(start, end) > 1.0e-6 => {
                curves.add_line(start, end);
                true
            }
            _ => false,
        },
        "CIRCLE" => match (point(record, 10, 20, scale), number(record, 40)) {
            (Some(center), Some(radius)) if radius > 0.0 => {
                curves.add_circle(center, radius * scale);
                true
            }
            _ => false,
        },
        "ARC" => import_arc(record, scale, curves),
        "ELLIPSE" => import_ellipse(record, scale, curves),
        "LWPOLYLINE" => import_lwpolyline(record, scale, curves),
        "SPLINE" => import_spline(record, scale, curves),
        _ => return false,
    };
    if !imported {
        diagnostics.push(DxfDiagnostic {
            severity: DxfDiagnosticSeverity::Warning,
            message: format!(
                "Skipped malformed {kind} on layer {layer} near line {}",
                record.first().map_or(0, |pair| pair.line)
            ),
        });
    }
    true
}

fn import_arc(record: &[Pair], scale: f32, curves: &mut SketchCurves) -> bool {
    let (Some(center), Some(radius), Some(start), Some(end)) = (
        point(record, 10, 20, scale),
        number(record, 40).map(|value| value * scale),
        number(record, 50),
        number(record, 51),
    ) else {
        return false;
    };
    if radius <= 0.0 {
        return false;
    }
    let mut sweep = (end - start).to_radians();
    while sweep <= 0.0 {
        sweep += std::f32::consts::TAU;
    }
    append_elliptic_arc(
        curves,
        center,
        (radius, 0.0),
        radius,
        start.to_radians(),
        sweep,
    );
    true
}

fn import_ellipse(record: &[Pair], scale: f32, curves: &mut SketchCurves) -> bool {
    let (Some(center), Some(major), Some(ratio)) = (
        point(record, 10, 20, scale),
        point(record, 11, 21, scale),
        number(record, 40),
    ) else {
        return false;
    };
    let major_radius = major.0.hypot(major.1);
    if major_radius <= 1.0e-6 || ratio <= 0.0 {
        return false;
    }
    let start = number(record, 41).unwrap_or(0.0);
    let mut sweep = number(record, 42).unwrap_or(std::f32::consts::TAU) - start;
    while sweep <= 0.0 {
        sweep += std::f32::consts::TAU;
    }
    append_elliptic_arc(curves, center, major, major_radius * ratio, start, sweep);
    true
}

fn append_elliptic_arc(
    curves: &mut SketchCurves,
    center: (f32, f32),
    major: (f32, f32),
    minor_radius: f32,
    start: f32,
    sweep: f32,
) {
    let major_radius = major.0.hypot(major.1);
    let ux = major.0 / major_radius;
    let uy = major.1 / major_radius;
    let px = -uy;
    let py = ux;
    let segments = ((sweep.abs() / std::f32::consts::TAU * 96.0).ceil() as usize).clamp(2, 192);
    let at = |parameter: f32| {
        (
            center.0 + major_radius * parameter.cos() * ux + minor_radius * parameter.sin() * px,
            center.1 + major_radius * parameter.cos() * uy + minor_radius * parameter.sin() * py,
        )
    };
    let mut previous = at(start);
    for index in 1..=segments {
        let current = at(start + sweep * index as f32 / segments as f32);
        curves.add_line(previous, current);
        previous = current;
    }
}

#[derive(Clone, Copy)]
struct PolyVertex {
    point: (f32, f32),
    bulge: f32,
}

fn import_lwpolyline(record: &[Pair], scale: f32, curves: &mut SketchCurves) -> bool {
    let mut vertices = Vec::new();
    for pair in record {
        match pair.code {
            10 => {
                if let Ok(x) = pair.value.parse::<f32>() {
                    vertices.push(PolyVertex {
                        point: (x * scale, 0.0),
                        bulge: 0.0,
                    });
                }
            }
            20 => {
                if let (Some(vertex), Ok(y)) = (vertices.last_mut(), pair.value.parse::<f32>()) {
                    vertex.point.1 = y * scale;
                }
            }
            42 => {
                if let (Some(vertex), Ok(bulge)) = (vertices.last_mut(), pair.value.parse::<f32>())
                {
                    vertex.bulge = bulge;
                }
            }
            _ => {}
        }
    }
    let closed = integer(record, 70).unwrap_or(0) & 1 != 0;
    append_polyline(curves, &vertices, closed)
}

fn import_polyline(
    header: &[Pair],
    records: &[&[Pair]],
    unit: DxfUnit,
    layers: &mut BTreeMap<String, SketchCurves>,
    diagnostics: &mut Vec<DxfDiagnostic>,
) {
    let layer = text_value(header, 8).unwrap_or("0").to_string();
    let scale = unit.scale_to_mm as f32;
    let mut vertices = Vec::new();
    for record in records {
        if let Some(point) = point(record, 10, 20, scale) {
            vertices.push(PolyVertex {
                point,
                bulge: number(record, 42).unwrap_or(0.0),
            });
        }
    }
    let closed = integer(header, 70).unwrap_or(0) & 1 != 0;
    if !append_polyline(layers.entry(layer.clone()).or_default(), &vertices, closed) {
        diagnostics.push(DxfDiagnostic {
            severity: DxfDiagnosticSeverity::Warning,
            message: format!("Skipped malformed POLYLINE on layer {layer}"),
        });
    }
}

fn append_polyline(curves: &mut SketchCurves, vertices: &[PolyVertex], closed: bool) -> bool {
    if vertices.len() < 2 {
        return false;
    }
    let span_count = if closed {
        vertices.len()
    } else {
        vertices.len() - 1
    };
    for index in 0..span_count {
        let start = vertices[index];
        let end = vertices[(index + 1) % vertices.len()];
        if start.bulge.abs() <= 1.0e-7 {
            curves.add_line(start.point, end.point);
        } else {
            append_bulge(curves, start.point, end.point, start.bulge);
        }
    }
    true
}

fn append_bulge(curves: &mut SketchCurves, start: (f32, f32), end: (f32, f32), bulge: f32) {
    let dx = end.0 - start.0;
    let dy = end.1 - start.1;
    let chord = dx.hypot(dy);
    if chord <= 1.0e-7 {
        return;
    }
    let center_offset = chord * (1.0 - bulge * bulge) / (4.0 * bulge);
    let midpoint = ((start.0 + end.0) * 0.5, (start.1 + end.1) * 0.5);
    let center = (
        midpoint.0 - dy / chord * center_offset,
        midpoint.1 + dx / chord * center_offset,
    );
    let start_angle = (start.1 - center.1).atan2(start.0 - center.0);
    let sweep = 4.0 * bulge.atan();
    let radius = distance(start, center);
    let segments = ((sweep.abs() / std::f32::consts::TAU * 96.0).ceil() as usize).clamp(2, 192);
    let mut previous = start;
    for index in 1..=segments {
        let angle = start_angle + sweep * index as f32 / segments as f32;
        let current = if index == segments {
            end
        } else {
            (
                center.0 + radius * angle.cos(),
                center.1 + radius * angle.sin(),
            )
        };
        curves.add_line(previous, current);
        previous = current;
    }
}

fn import_spline(record: &[Pair], scale: f32, curves: &mut SketchCurves) -> bool {
    let degree = integer(record, 71).unwrap_or(3).clamp(1, u8::MAX as i32) as u8;
    let flags = integer(record, 70).unwrap_or(0);
    let control_points = repeated_points(record, 10, 20, scale);
    let fit_points = repeated_points(record, 11, 21, scale);
    let (kind, points) = if fit_points.len() >= 2 {
        (crate::sketch::SplineKind::FitPoint, fit_points)
    } else {
        (crate::sketch::SplineKind::ControlPoint, control_points)
    };
    let spline = Spline {
        kind,
        points,
        degree,
        knots: repeated_numbers(record, 40),
        weights: repeated_numbers(record, 41),
        closed: flags & 1 != 0,
        periodic: flags & 2 != 0,
        continuity: SplineContinuity::Curvature,
        trim: None,
    };
    if !spline.is_valid() {
        return false;
    }
    curves.add_spline(spline);
    true
}

fn repeated_points(record: &[Pair], x_code: i16, y_code: i16, scale: f32) -> Vec<(f32, f32)> {
    let mut points = Vec::new();
    for pair in record {
        if pair.code == x_code {
            if let Ok(x) = pair.value.parse::<f32>() {
                points.push((x * scale, 0.0));
            }
        } else if pair.code == y_code {
            if let (Some(point), Ok(y)) = (points.last_mut(), pair.value.parse::<f32>()) {
                point.1 = y * scale;
            }
        }
    }
    points
}

fn repeated_numbers(record: &[Pair], code: i16) -> Vec<f32> {
    record
        .iter()
        .filter(|pair| pair.code == code)
        .filter_map(|pair| pair.value.parse::<f32>().ok())
        .collect()
}

fn text_value(record: &[Pair], code: i16) -> Option<&str> {
    record
        .iter()
        .find(|pair| pair.code == code)
        .map(|pair| pair.value.as_str())
}

fn number(record: &[Pair], code: i16) -> Option<f32> {
    text_value(record, code)?.parse().ok()
}

fn integer(record: &[Pair], code: i16) -> Option<i32> {
    text_value(record, code)?.parse().ok()
}

fn point(record: &[Pair], x_code: i16, y_code: i16, scale: f32) -> Option<(f32, f32)> {
    Some((
        number(record, x_code)? * scale,
        number(record, y_code)? * scale,
    ))
}

fn distance(a: (f32, f32), b: (f32, f32)) -> f32 {
    (b.0 - a.0).hypot(b.1 - a.1)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sketch::{detect_regions, SplineKind};

    const HEADER: &str =
        "0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n1\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n";

    #[test]
    fn imports_units_layers_and_closed_profile() {
        let text = format!(
            "{HEADER}0\nLWPOLYLINE\n8\nCUT\n70\n1\n10\n0\n20\n0\n10\n1\n20\n0\n10\n1\n20\n1\n10\n0\n20\n1\n0\nENDSEC\n0\nEOF\n"
        );
        let imported = read_dxf_str("inch-profile.dxf", &text).unwrap();
        assert_eq!(imported.unit.code, 1);
        assert_eq!(imported.unit.scale_to_mm, 25.4);
        assert_eq!(imported.shapes.len(), 1);
        let SketchShape::Imported { curves, metadata } = &imported.shapes[0] else {
            panic!("DXF must retain import metadata")
        };
        assert_eq!(metadata.layer, "CUT");
        assert_eq!(metadata.source_name, "inch-profile.dxf");
        let regions = detect_regions(curves);
        assert_eq!(regions.len(), 1);
        assert!((regions[0].area - 25.4 * 25.4).abs() < 0.5);
    }

    #[test]
    fn imports_bulges_and_nurbs_without_flattening_the_spline_record() {
        let text = format!(
            "{HEADER}0\nLWPOLYLINE\n8\nARC\n70\n0\n10\n0\n20\n0\n42\n1\n10\n2\n20\n0\n0\nSPLINE\n8\nSPLINES\n70\n0\n71\n2\n10\n0\n20\n0\n10\n1\n20\n2\n10\n2\n20\n0\n40\n0\n40\n0\n40\n0\n40\n1\n40\n1\n40\n1\n0\nENDSEC\n0\nEOF\n"
        );
        let imported = read_dxf_str("curves.dxf", &text).unwrap();
        assert_eq!(imported.shapes.len(), 2);
        let spline = imported
            .shapes
            .iter()
            .find_map(|shape| match shape {
                SketchShape::Imported { curves, .. } => curves.splines.first(),
                _ => None,
            })
            .unwrap();
        assert_eq!(spline.kind, SplineKind::ControlPoint);
        assert_eq!(spline.degree, 2);
        assert_eq!(spline.knots.len(), 6);
        assert!(spline.sampled_points(0.01).len() > 10);
    }

    #[test]
    fn reports_unsupported_entities_but_keeps_supported_geometry() {
        let text = format!(
            "{HEADER}0\nTEXT\n8\nNOTES\n1\nhello\n0\nLINE\n8\nCUT\n10\n0\n20\n0\n11\n1\n21\n0\n0\nENDSEC\n0\nEOF\n"
        );
        let imported = read_dxf_str("mixed.dxf", &text).unwrap();
        assert!(imported
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.message.contains("unsupported TEXT")));
        assert_eq!(imported.imported_curve_count(), 1);
    }
}
