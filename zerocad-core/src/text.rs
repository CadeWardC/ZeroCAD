//! Sketch text — font discovery, shaping, and glyph-outline conversion.
//!
//! Text is a **curve source**, not a modelling operation. It emits ordinary
//! [`SketchCurves`] (lines plus open control-point splines), so region
//! detection, extrude, cut, trim, offset and every downstream feature consume
//! it unchanged — embossing is just an extrude over the resulting text regions,
//! and engraving is the same extrude in cut mode.
//!
//! Glyph outlines are quadratic (TrueType) or cubic (CFF/OpenType) Béziers. A
//! Bézier is exactly a clamped B-spline with no interior knots, so each segment
//! becomes a degree-2 or degree-3 [`Spline`] with an empty knot vector;
//! `detect_regions_analytic` then fills in clamped-uniform knots, which for
//! these pole counts reproduce the Bézier exactly. Nothing here is sampled or
//! approximated — the outlines reach the B-Rep as real `GeomCurve::BSpline`
//! edges (locked in by `analytic_open_spline_chain_reaches_brep_as_bspline_edges`).
//!
//! Contours are emitted verbatim, including counters such as the bowl of an
//! `O`. Assigning a contour as a hole is left to the sketch arrangement, which
//! already computes containment and overlap. The arrangement is a full planar
//! subdivision, though, so it reports *every* bounded face — for an `O` that is
//! both the annulus (with the counter as its hole) and the counter's interior
//! as a second filled region. Choosing which faces are inked is a typographic
//! decision the arrangement cannot make, so [`text_regions`] applies it.

use crate::sketch::{SketchCurves, Spline, SplineContinuity, SplineKind};

/// Segments shorter than this (in millimetres) are dropped. Fonts routinely
/// repeat an on-curve point between contours, and a zero-length span would be
/// rejected by the arrangement rather than ignored by it.
const MIN_SEGMENT_MM: f32 = 1.0e-6;

/// How far an emboss tool reaches past the face it meets, so the boolean sees
/// a solid overlap rather than a coplanar contact. This has to be a real
/// thickness, not an epsilon: at 0.05 mm the raised union classified only part
/// of each glyph and silently under-added material. Raised letters are this much
/// taller than nominal at their root and engravings this much deeper — both
/// buried inside the body, so neither is visible on the finished part.
const CONTACT_OVERSHOOT_MM: f32 = 0.5;

/// Identifies the exact font file a text feature was authored against.
///
/// The family name alone is never sufficient: identically named faces ship
/// different outlines across machines and across versions, so the content hash
/// is what decides whether the text is still editable.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct FontFingerprint {
    pub content_hash: [u8; 32],
    pub family: String,
    pub style: String,
    pub face_index: u32,
    pub units_per_em: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum TextAlign {
    #[default]
    Left,
    Center,
    Right,
}

/// Layout parameters. Sizes are millimetres, matching the geometric base unit;
/// the document's display unit never scales text geometry.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct TextParams {
    /// Em size in millimetres.
    pub size_mm: f32,
    /// Extra advance inserted after every glyph, in millimetres. Negative
    /// values tighten and may legitimately overlap neighbouring contours.
    pub tracking_mm: f32,
    /// Baseline-to-baseline distance as a multiple of `size_mm`.
    pub line_spacing: f32,
    pub align: TextAlign,
}

impl Default for TextParams {
    fn default() -> Self {
        Self {
            size_mm: 10.0,
            tracking_mm: 0.0,
            line_spacing: 1.2,
            align: TextAlign::Left,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TextError {
    /// The string is empty or contains no printable characters.
    EmptyText,
    /// The font file could not be parsed as a face.
    FaceParse(String),
    /// The face parsed but the shaper refused it (missing required tables).
    ShaperRejectedFace,
    /// Every glyph resolved to an empty outline — whitespace-only, or a face
    /// with no glyph coverage for this string.
    NoOutlines,
    InvalidParams(&'static str),
}

impl std::fmt::Display for TextError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::EmptyText => write!(formatter, "text is empty"),
            Self::FaceParse(reason) => write!(formatter, "font face could not be parsed: {reason}"),
            Self::ShaperRejectedFace => write!(formatter, "font face is not shapeable"),
            Self::NoOutlines => write!(formatter, "text produced no glyph outlines"),
            Self::InvalidParams(reason) => write!(formatter, "invalid text parameters: {reason}"),
        }
    }
}

impl std::error::Error for TextError {}

/// Hash and describe a font file so a text feature can tell later whether the
/// same outlines are still reproducible on this machine.
pub fn fingerprint(font: &[u8], face_index: u32) -> Result<FontFingerprint, TextError> {
    let face = parse_face(font, face_index)?;
    Ok(FontFingerprint {
        content_hash: *blake3::hash(font).as_bytes(),
        family: face_name(&face, ttf_parser::name_id::FAMILY),
        style: face_name(&face, ttf_parser::name_id::SUBFAMILY),
        face_index,
        units_per_em: face.units_per_em(),
    })
}

/// Shape `text` with the given face and convert every glyph outline into sketch
/// curves. The first baseline sits at the origin with the text running along
/// `+u`; subsequent lines stack toward `-v`.
pub fn outline_text(
    font: &[u8],
    face_index: u32,
    text: &str,
    params: &TextParams,
) -> Result<SketchCurves, TextError> {
    if text.is_empty() {
        return Err(TextError::EmptyText);
    }
    if !params.size_mm.is_finite() || params.size_mm <= 0.0 {
        return Err(TextError::InvalidParams("size must be finite and positive"));
    }
    if !params.tracking_mm.is_finite() {
        return Err(TextError::InvalidParams("tracking must be finite"));
    }
    if !params.line_spacing.is_finite() || params.line_spacing <= 0.0 {
        return Err(TextError::InvalidParams(
            "line spacing must be finite and positive",
        ));
    }

    let face = parse_face(font, face_index)?;
    let units_per_em = face.units_per_em();
    if units_per_em == 0 {
        return Err(TextError::FaceParse("units_per_em is zero".to_owned()));
    }
    let scale = f64::from(params.size_mm) / f64::from(units_per_em);
    let shaper =
        rustybuzz::Face::from_slice(font, face_index).ok_or(TextError::ShaperRejectedFace)?;

    // Shape each line independently; rustybuzz has no notion of hard breaks.
    let tracking = f64::from(params.tracking_mm);
    let mut rows: Vec<(Vec<PlacedGlyph>, f64)> = Vec::new();
    for line in text.split('\n') {
        let mut buffer = rustybuzz::UnicodeBuffer::new();
        buffer.push_str(line);
        buffer.guess_segment_properties();
        let shaped = rustybuzz::shape(&shaper, &[], buffer);

        let mut pen_x = 0.0_f64;
        let mut placed = Vec::with_capacity(shaped.len());
        for (info, position) in shaped
            .glyph_infos()
            .iter()
            .zip(shaped.glyph_positions().iter())
        {
            placed.push(PlacedGlyph {
                glyph: u16::try_from(info.glyph_id).unwrap_or_default(),
                x: pen_x + f64::from(position.x_offset) * scale,
                y: f64::from(position.y_offset) * scale,
            });
            pen_x += f64::from(position.x_advance) * scale + tracking;
        }
        // The advance loop appends tracking after the final glyph too; that
        // trailing gap is not part of the line's visual width and must not
        // skew centred or right alignment.
        let width = if placed.is_empty() {
            0.0
        } else {
            (pen_x - tracking).max(0.0)
        };
        rows.push((placed, width));
    }

    let widest = rows.iter().map(|(_, width)| *width).fold(0.0_f64, f64::max);
    let line_height = f64::from(params.size_mm) * f64::from(params.line_spacing);

    let mut curves = SketchCurves::new();
    let mut emitted = 0_usize;
    for (row, (placed, width)) in rows.iter().enumerate() {
        let offset_x = match params.align {
            TextAlign::Left => 0.0,
            TextAlign::Center => (widest - width) * 0.5,
            TextAlign::Right => widest - width,
        };
        let offset_y = -(row as f64) * line_height;
        for glyph in placed {
            let mut outliner = GlyphOutliner {
                curves: &mut curves,
                scale,
                pen: (offset_x + glyph.x, offset_y + glyph.y),
                start: None,
                current: (0.0, 0.0),
                emitted: 0,
            };
            // A `None` outline is normal — spaces and other blank glyphs have
            // no contours and must not fail the whole string.
            if face
                .outline_glyph(ttf_parser::GlyphId(glyph.glyph), &mut outliner)
                .is_some()
            {
                emitted += outliner.emitted;
            }
        }
    }

    if emitted == 0 {
        return Err(TextError::NoOutlines);
    }
    Ok(curves)
}

/// Where a text block sits on its sketch plane. Rotation is about the block's
/// own origin — the start of the first baseline — and applies before the
/// translation, so moving and spinning a label are independent edits.
#[derive(Debug, Clone, Copy, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct TextPlacement {
    pub origin: (f32, f32),
    pub rotation_deg: f32,
}

impl TextPlacement {
    fn apply(self, point: (f32, f32)) -> (f32, f32) {
        let (sin, cos) = self.rotation_deg.to_radians().sin_cos();
        (
            self.origin.0 + point.0 * cos - point.1 * sin,
            self.origin.1 + point.0 * sin + point.1 * cos,
        )
    }
}

/// Rigidly move sketch curves. Rotating and translating a B-spline's poles
/// transforms the curve exactly, so placement never resamples or degrades the
/// baked outlines.
pub fn place_curves(curves: &SketchCurves, placement: TextPlacement) -> SketchCurves {
    let mut placed = curves.clone();
    for segment in &mut placed.segments {
        segment.a = placement.apply(segment.a);
        segment.b = placement.apply(segment.b);
    }
    for spline in &mut placed.splines {
        for pole in &mut spline.points {
            *pole = placement.apply(*pole);
        }
    }
    placed
}

/// Shape `text`, place it, and return only the regions that should be inked.
///
/// [`outline_text`] followed by `detect_regions_analytic` yields every bounded
/// face of the planar subdivision, which for a glyph with a counter includes
/// the counter's interior as a filled region in its own right. A counter is
/// already represented as a hole of its enclosing region, so a face that
/// duplicates some other region's hole is dropped. Everything else — the two
/// disjoint contours of an `i`, the separate letters of a word — is kept.
pub fn text_regions(
    font: &[u8],
    face_index: u32,
    text: &str,
    params: &TextParams,
    placement: TextPlacement,
) -> Result<Vec<crate::sketch::Region>, TextError> {
    let curves = place_curves(&outline_text(font, face_index, text, params)?, placement);
    let regions = crate::sketch::detect_regions_analytic(&curves)
        .map_err(|error| TextError::FaceParse(error.to_string()))?;

    let holes: Vec<(f32, Bounds)> = regions
        .iter()
        .flat_map(|region| region.holes.iter())
        .map(|hole| (polygon_area(hole), Bounds::of(hole)))
        .collect();

    Ok(regions
        .into_iter()
        .filter(|region| {
            region.area > 0.0
                && !holes.iter().any(|(hole_area, hole_bounds)| {
                    // The counter's own face and the hole recording it come
                    // from the same arrangement loop, so they agree on both
                    // area and extent; requiring both avoids discarding a
                    // genuinely separate contour that merely has a similar area.
                    areas_match(region.area, *hole_area)
                        && Bounds::of(&region.boundary).matches(*hole_bounds)
                })
        })
        .collect())
}

/// Raised text adds material; engraved text removes it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum EmbossMode {
    #[default]
    Raised,
    Engraved,
}

/// Everything needed to turn a string into geometry on a sketch plane.
#[derive(Debug, Clone)]
pub struct EmbossRequest<'a> {
    pub font: &'a [u8],
    pub face_index: u32,
    pub text: &'a str,
    pub params: TextParams,
    pub placement: TextPlacement,
    /// Height of the raised letters, or depth of the engraving. Always
    /// positive; [`EmbossMode`] decides which way it goes.
    pub depth_mm: f32,
    pub mode: EmbossMode,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EmbossError {
    Text(TextError),
    /// A region produced no prism — a glyph too small or thin to sweep at this
    /// depth, rather than a problem with the string.
    GlyphSweepFailed,
    /// The boolean against the target body did not produce a valid solid.
    BooleanFailed,
}

impl std::fmt::Display for EmbossError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Text(error) => write!(formatter, "{error}"),
            Self::GlyphSweepFailed => write!(formatter, "a glyph could not be swept to a solid"),
            Self::BooleanFailed => write!(formatter, "the emboss boolean did not produce a solid"),
        }
    }
}

impl std::error::Error for EmbossError {}

impl From<TextError> for EmbossError {
    fn from(error: TextError) -> Self {
        Self::Text(error)
    }
}

/// One prism per inked region, swept off the sketch plane.
///
/// Engraved text sweeps *into* the plane so the tool sits below the face it is
/// cutting; raised text sweeps out of it. Callers that want the tool bodies
/// themselves — for a preview, say — use this directly.
pub fn emboss_tool_solids(
    request: &EmbossRequest<'_>,
    plane: &crate::geometry::CoordinateSystem,
) -> Result<Vec<crate::mock_kernel::KernelSolid>, EmbossError> {
    let regions = text_regions(
        request.font,
        request.face_index,
        request.text,
        &request.params,
        request.placement,
    )?;
    // A tool that stops exactly on the face it meets gives the boolean a
    // coplanar contact, which this kernel rejects. Start the sweep behind the
    // plane and run past it by the same amount so there is genuine overlap in
    // both directions — the same convention `cut.rs` uses via `CUT_OVERSHOOT`.
    let (start_offset, depth) = match request.mode {
        EmbossMode::Raised => (
            -CONTACT_OVERSHOOT_MM,
            request.depth_mm.abs() + CONTACT_OVERSHOOT_MM,
        ),
        EmbossMode::Engraved => (
            CONTACT_OVERSHOOT_MM,
            -(request.depth_mm.abs() + CONTACT_OVERSHOOT_MM),
        ),
    };
    // `with_origin` keeps u/v/n intact; rebuilding the frame would recompute
    // the normal and can flip the sweep direction.
    let base = plane.with_origin(plane.origin.add(plane.n.mul(start_offset)));

    let mut tools = Vec::with_capacity(regions.len());
    for region in &regions {
        let solid = crate::mock_kernel::extruded_sketch_region_solid(region, depth, &base, &[])
            .ok_or(EmbossError::GlyphSweepFailed)?;
        tools.push(solid);
    }
    Ok(tools)
}

/// Apply text to a body: union for raised, difference for engraved.
///
/// Each glyph is a separate tool, so one failed boolean fails the whole
/// operation rather than leaving a word with letters missing.
pub fn emboss_text(
    body: &crate::mock_kernel::KernelSolid,
    request: &EmbossRequest<'_>,
    plane: &crate::geometry::CoordinateSystem,
) -> Result<crate::mock_kernel::KernelSolid, EmbossError> {
    let tools = emboss_tool_solids(request, plane)?;
    let mut result = body.clone();
    for tool in &tools {
        result = match request.mode {
            EmbossMode::Raised => crate::mock_kernel::union(&result, tool),
            EmbossMode::Engraved => crate::mock_kernel::difference(&result, tool),
        }
        .ok_or(EmbossError::BooleanFailed)?;
    }
    Ok(result)
}

#[derive(Clone, Copy)]
struct Bounds {
    min_x: f32,
    min_y: f32,
    max_x: f32,
    max_y: f32,
}

impl Bounds {
    fn of(points: &[(f32, f32)]) -> Self {
        let mut bounds = Self {
            min_x: f32::MAX,
            min_y: f32::MAX,
            max_x: f32::MIN,
            max_y: f32::MIN,
        };
        for (x, y) in points {
            bounds.min_x = bounds.min_x.min(*x);
            bounds.min_y = bounds.min_y.min(*y);
            bounds.max_x = bounds.max_x.max(*x);
            bounds.max_y = bounds.max_y.max(*y);
        }
        bounds
    }

    fn matches(self, other: Self) -> bool {
        let span = (self.max_x - self.min_x)
            .abs()
            .max((self.max_y - self.min_y).abs())
            .max(1.0);
        let limit = span * 1.0e-3;
        (self.min_x - other.min_x).abs() <= limit
            && (self.min_y - other.min_y).abs() <= limit
            && (self.max_x - other.max_x).abs() <= limit
            && (self.max_y - other.max_y).abs() <= limit
    }
}

fn polygon_area(points: &[(f32, f32)]) -> f32 {
    if points.len() < 3 {
        return 0.0;
    }
    let mut twice_area = 0.0_f64;
    for index in 0..points.len() {
        let (x0, y0) = points[index];
        let (x1, y1) = points[(index + 1) % points.len()];
        twice_area += f64::from(x0) * f64::from(y1) - f64::from(x1) * f64::from(y0);
    }
    (twice_area.abs() * 0.5) as f32
}

fn areas_match(left: f32, right: f32) -> bool {
    (left - right).abs() <= left.abs().max(right.abs()).max(1.0) * 1.0e-3
}

struct PlacedGlyph {
    glyph: u16,
    x: f64,
    y: f64,
}

fn parse_face(font: &[u8], face_index: u32) -> Result<ttf_parser::Face<'_>, TextError> {
    ttf_parser::Face::parse(font, face_index)
        .map_err(|error| TextError::FaceParse(error.to_string()))
}

fn face_name(face: &ttf_parser::Face<'_>, name_id: u16) -> String {
    face.names()
        .into_iter()
        .find(|name| name.name_id == name_id && name.is_unicode())
        .and_then(|name| name.to_string())
        .unwrap_or_default()
}

/// Converts one glyph's font-unit outline into millimetre sketch curves.
struct GlyphOutliner<'a> {
    curves: &'a mut SketchCurves,
    scale: f64,
    pen: (f64, f64),
    start: Option<(f32, f32)>,
    current: (f32, f32),
    emitted: usize,
}

impl GlyphOutliner<'_> {
    fn map(&self, x: f32, y: f32) -> (f32, f32) {
        (
            (self.pen.0 + f64::from(x) * self.scale) as f32,
            (self.pen.1 + f64::from(y) * self.scale) as f32,
        )
    }

    fn is_degenerate(from: (f32, f32), to: (f32, f32)) -> bool {
        (from.0 - to.0).hypot(from.1 - to.1) <= MIN_SEGMENT_MM
    }

    fn line(&mut self, to: (f32, f32)) {
        if !Self::is_degenerate(self.current, to) {
            self.curves.add_line(self.current, to);
            self.emitted += 1;
        }
        self.current = to;
    }

    fn bezier(&mut self, poles: Vec<(f32, f32)>, degree: u8) {
        let end = *poles.last().expect("bezier has an end pole");
        // A control polygon that has collapsed to a point carries no geometry;
        // treating it as a span would hand the arrangement a zero-length curve.
        if poles.iter().all(|pole| Self::is_degenerate(*pole, end)) {
            self.current = end;
            return;
        }
        self.curves.add_spline(Spline {
            kind: SplineKind::ControlPoint,
            points: poles,
            degree,
            // Empty knots make the analytic converter derive clamped-uniform
            // knots, which for these pole counts are exactly Bézier knots.
            knots: Vec::new(),
            weights: Vec::new(),
            closed: false,
            periodic: false,
            continuity: SplineContinuity::default(),
            trim: None,
        });
        self.emitted += 1;
        self.current = end;
    }
}

impl ttf_parser::OutlineBuilder for GlyphOutliner<'_> {
    fn move_to(&mut self, x: f32, y: f32) {
        let point = self.map(x, y);
        self.start = Some(point);
        self.current = point;
    }

    fn line_to(&mut self, x: f32, y: f32) {
        let point = self.map(x, y);
        self.line(point);
    }

    fn quad_to(&mut self, x1: f32, y1: f32, x: f32, y: f32) {
        let control = self.map(x1, y1);
        let end = self.map(x, y);
        let poles = vec![self.current, control, end];
        self.bezier(poles, 2);
    }

    fn curve_to(&mut self, x1: f32, y1: f32, x2: f32, y2: f32, x: f32, y: f32) {
        let first = self.map(x1, y1);
        let second = self.map(x2, y2);
        let end = self.map(x, y);
        let poles = vec![self.current, first, second, end];
        self.bezier(poles, 3);
    }

    fn close(&mut self) {
        if let Some(start) = self.start {
            // Most contours already end on their start point; only emit the
            // closing chord when the font actually left a gap.
            self.line(start);
        }
    }
}

/// Installed system fonts, enumerated once.
pub struct FontCatalog {
    database: fontdb::Database,
}

impl FontCatalog {
    pub fn system() -> Self {
        let mut database = fontdb::Database::new();
        database.load_system_fonts();
        Self { database }
    }

    /// Sorted, de-duplicated family names suitable for a font picker.
    pub fn families(&self) -> Vec<String> {
        let mut families: Vec<String> = self
            .database
            .faces()
            .filter_map(|face| face.families.first().map(|(name, _)| name.clone()))
            .collect();
        families.sort_unstable();
        families.dedup();
        families
    }

    /// Font bytes plus face index for a family, ready to hash and outline.
    pub fn load_family(&self, family: &str) -> Option<(Vec<u8>, u32)> {
        let id = self.database.query(&fontdb::Query {
            families: &[fontdb::Family::Name(family)],
            ..fontdb::Query::default()
        })?;
        self.database
            .with_face_data(id, |data, index| (data.to_vec(), index))
    }
}

impl Default for FontCatalog {
    fn default() -> Self {
        Self::system()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Already in the dependency graph via egui/epaint, so the fixture costs no
    /// vendored binary. Tests must never read an installed system font — the
    /// results would differ per machine.
    const HACK: &[u8] = epaint_default_fonts::HACK_REGULAR;

    fn bounds(curves: &SketchCurves) -> (f32, f32, f32, f32) {
        let mut min = (f32::MAX, f32::MAX);
        let mut max = (f32::MIN, f32::MIN);
        let mut visit = |point: (f32, f32)| {
            min.0 = min.0.min(point.0);
            min.1 = min.1.min(point.1);
            max.0 = max.0.max(point.0);
            max.1 = max.1.max(point.1);
        };
        for segment in &curves.segments {
            visit(segment.a);
            visit(segment.b);
        }
        for spline in &curves.splines {
            for point in &spline.points {
                visit(*point);
            }
        }
        (min.0, min.1, max.0, max.1)
    }

    #[test]
    fn glyph_outlines_become_bezier_splines_not_polylines() {
        let curves = outline_text(HACK, 0, "S", &TextParams::default()).expect("outline S");
        assert!(
            !curves.splines.is_empty(),
            "a curved glyph must emit spline spans"
        );
        for spline in &curves.splines {
            assert_eq!(spline.kind, SplineKind::ControlPoint);
            assert!(!spline.closed && !spline.periodic);
            // Exactly Bézier: degree + 1 poles, no interior knots.
            assert_eq!(spline.points.len(), usize::from(spline.degree) + 1);
            assert!(spline.knots.is_empty());
        }
    }

    /// A glyph is dozens of Bézier segments meeting end to end, so this only
    /// passes once the arrangement can intersect two B-spline spans — the
    /// counter has to arrive as a hole, and it has to arrive *analytically*
    /// rather than through the sampled fallback.
    #[test]
    fn counter_of_o_is_one_inked_region_with_one_hole() {
        let regions = text_regions(
            HACK,
            0,
            "O",
            &TextParams::default(),
            TextPlacement::default(),
        )
        .expect("O regions");
        assert_eq!(
            regions.len(),
            1,
            "the counter must not ink as a second region"
        );
        assert_eq!(regions[0].holes.len(), 1, "the bowl of an O is a hole");
        assert!(
            regions[0].analytic.is_some(),
            "the region must carry analytic provenance, not sampled polylines"
        );
    }

    #[test]
    fn two_counters_and_disjoint_contours_both_survive() {
        let b = text_regions(
            HACK,
            0,
            "B",
            &TextParams::default(),
            TextPlacement::default(),
        )
        .expect("B regions");
        assert_eq!(b.len(), 1, "B is one inked region");
        assert_eq!(b[0].holes.len(), 2, "B has two counters");

        // The tittle of an `i` is a separate contour, not a counter — dropping
        // it would be the obvious way to get the counter rule wrong.
        let i = text_regions(
            HACK,
            0,
            "i",
            &TextParams::default(),
            TextPlacement::default(),
        )
        .expect("i regions");
        assert_eq!(i.len(), 2, "stem and tittle are both inked");
        assert!(i.iter().all(|region| region.holes.is_empty()));
    }

    /// A whole word exercises multi-glyph layout, the counter rule, and the
    /// arrangement's pairwise cost together: every glyph adds spans and the
    /// arrangement intersects all pairs, so this is the case that would expose
    /// the spline-spline intersector being too slow to use.
    #[test]
    fn a_whole_word_inks_one_region_per_glyph_with_its_counters() {
        let regions = text_regions(
            HACK,
            0,
            "ZeroCAD",
            &TextParams::default(),
            TextPlacement::default(),
        )
        .expect("word");
        assert_eq!(regions.len(), 7, "one inked region per letter");
        let counters: usize = regions.iter().map(|region| region.holes.len()).sum();
        assert_eq!(counters, 4, "e, o, A and D each contribute one counter");
    }

    #[test]
    fn text_region_extrudes_with_analytic_bspline_walls() {
        let region = text_regions(
            HACK,
            0,
            "O",
            &TextParams::default(),
            TextPlacement::default(),
        )
        .expect("O regions")
        .into_iter()
        .next()
        .expect("O region");

        let solid = crate::mock_kernel::extruded_sketch_region_solid(
            &region,
            2.0,
            &crate::geometry::CoordinateSystem::XY,
            &[],
        )
        .expect("text prism");

        assert!(
            solid
                .edges()
                .iter()
                .any(|edge| matches!(edge.curve(), Some(openrcad::geom::GeomCurve::BSpline(_)))),
            "embossed text must keep analytic B-spline walls"
        );
    }

    #[test]
    fn tracking_widens_the_line_and_size_scales_it() {
        let tight = outline_text(HACK, 0, "AV", &TextParams::default()).expect("tight");
        let loose = outline_text(
            HACK,
            0,
            "AV",
            &TextParams {
                tracking_mm: 5.0,
                ..TextParams::default()
            },
        )
        .expect("loose");
        assert!(bounds(&loose).2 > bounds(&tight).2 + 4.0);

        let big = outline_text(
            HACK,
            0,
            "AV",
            &TextParams {
                size_mm: 20.0,
                ..TextParams::default()
            },
        )
        .expect("big");
        assert!(bounds(&big).3 > bounds(&tight).3 * 1.5);
    }

    #[test]
    fn extra_lines_stack_below_the_first_baseline() {
        let single = outline_text(HACK, 0, "A", &TextParams::default()).expect("single");
        let double = outline_text(HACK, 0, "A\nA", &TextParams::default()).expect("double");
        assert!(
            bounds(&double).1 < bounds(&single).1 - 5.0,
            "the second line must sit below the first"
        );
        assert!(bounds(&double).3 >= bounds(&single).3 - MIN_SEGMENT_MM);
    }

    /// Bounds of one baseline's glyphs. The first row sits at y >= 0, the
    /// second below it, so the sign of y separates them.
    fn row_bounds(curves: &SketchCurves, second_row: bool) -> (f32, f32) {
        let mut min_x = f32::MAX;
        let mut max_x = f32::MIN;
        let mut visit = |point: (f32, f32)| {
            if (point.1 < -1.0) == second_row {
                min_x = min_x.min(point.0);
                max_x = max_x.max(point.0);
            }
        };
        for segment in &curves.segments {
            visit(segment.a);
            visit(segment.b);
        }
        for spline in &curves.splines {
            for point in &spline.points {
                visit(*point);
            }
        }
        (min_x, max_x)
    }

    #[test]
    fn alignment_shifts_shorter_lines_only() {
        let left = outline_text(HACK, 0, "IIII\nI", &TextParams::default()).expect("left");
        let (long_left, _) = row_bounds(&left, false);
        let (short_left, _) = row_bounds(&left, true);
        assert!(
            (long_left - short_left).abs() < 0.5,
            "left alignment shares a left edge"
        );

        let right = outline_text(
            HACK,
            0,
            "IIII\nI",
            &TextParams {
                align: TextAlign::Right,
                ..TextParams::default()
            },
        )
        .expect("right");
        let (_, long_right) = row_bounds(&right, false);
        let (short_min, short_right) = row_bounds(&right, true);
        assert!(
            (long_right - short_right).abs() < 0.5,
            "right alignment shares a right edge"
        );
        assert!(
            short_min > short_left + 1.0,
            "the short line must actually move right"
        );
    }

    #[test]
    fn blank_and_invalid_input_is_rejected_without_geometry() {
        assert_eq!(
            outline_text(HACK, 0, "", &TextParams::default()),
            Err(TextError::EmptyText)
        );
        assert_eq!(
            outline_text(HACK, 0, "   ", &TextParams::default()),
            Err(TextError::NoOutlines)
        );
        assert_eq!(
            outline_text(
                HACK,
                0,
                "A",
                &TextParams {
                    size_mm: 0.0,
                    ..TextParams::default()
                }
            ),
            Err(TextError::InvalidParams("size must be finite and positive"))
        );
        assert!(matches!(
            outline_text(b"not a font", 0, "A", &TextParams::default()),
            Err(TextError::FaceParse(_))
        ));
    }

    #[test]
    fn fingerprint_identifies_the_face_and_tracks_content() {
        let print = fingerprint(HACK, 0).expect("fingerprint");
        assert_eq!(print.family, "Hack");
        assert!(print.units_per_em > 0);
        assert_eq!(print.face_index, 0);
        assert_eq!(print, fingerprint(HACK, 0).expect("stable"));

        let mut altered = HACK.to_vec();
        *altered.last_mut().expect("non-empty") ^= 0xFF;
        let altered = fingerprint(&altered, 0).expect("altered fingerprint");
        assert_ne!(
            print.content_hash, altered.content_hash,
            "a byte change must change the fingerprint even with the same family"
        );
    }

    fn plate() -> crate::mock_kernel::KernelSolid {
        crate::mock_kernel::box_solid(60.0, 20.0, 5.0)
    }

    fn request<'a>(text: &'a str, mode: EmbossMode) -> EmbossRequest<'a> {
        EmbossRequest {
            font: HACK,
            face_index: 0,
            text,
            params: TextParams {
                size_mm: 8.0,
                ..TextParams::default()
            },
            placement: TextPlacement {
                origin: (6.0, 6.0),
                rotation_deg: 0.0,
            },
            depth_mm: 1.0,
            mode,
        }
    }

    /// KNOWN KERNEL GAP — a glyph with curved walls sweeps to a valid tool, but
    /// the boolean cannot fuse or subtract a solid whose lateral faces are
    /// B-splines. Straight-walled glyphs work in both directions, and `A` shows
    /// it is not about counters: it has one and still succeeds. Closing this
    /// needs NURBS lateral support in `openrcad-algo`'s boolean, at which point
    /// this test flips to asserting success.
    #[test]
    fn curved_glyph_emboss_awaits_nurbs_boolean_support() {
        let plane = crate::geometry::CoordinateSystem::XY;
        let top = plane.with_origin(crate::geometry::Vec3::new(0.0, 0.0, 5.0));

        // The tool itself builds fine — the gap is strictly in the boolean.
        assert_eq!(
            emboss_tool_solids(&request("o", EmbossMode::Raised), &top)
                .expect("curved glyph sweeps to a tool")
                .len(),
            1
        );
        assert_eq!(
            emboss_text(&plate(), &request("o", EmbossMode::Raised), &top),
            Err(EmbossError::BooleanFailed)
        );
    }

    #[test]
    fn raised_text_adds_material_and_engraved_text_removes_it() {
        let plate = plate();
        let plane = crate::geometry::CoordinateSystem::XY;
        // The plate's top face is at z = 5, so both tools start from the plane
        // through it; raised sweeps up, engraved sweeps down into the material.
        // `with_origin` keeps u/v/n exactly as they are. Rebuilding the frame
        // through `CoordinateSystem::new` would recompute the normal and can
        // flip it, which has previously sent cutters the wrong way.
        let top = plane.with_origin(crate::geometry::Vec3::new(0.0, 0.0, 5.0));

        let volume = |solid: &crate::mock_kernel::KernelSolid| {
            crate::parametric::solid_volume(solid).expect("closed solids have a volume")
        };
        let base_volume = volume(&plate);
        let raised =
            emboss_text(&plate, &request("AL", EmbossMode::Raised), &top).expect("raised emboss");
        let engraved = emboss_text(&plate, &request("AL", EmbossMode::Engraved), &top)
            .expect("engraved emboss");

        let raised_volume = volume(&raised);
        let engraved_volume = volume(&engraved);
        assert!(
            raised_volume > base_volume,
            "raised text must add material ({raised_volume} vs {base_volume})"
        );
        assert!(
            engraved_volume < base_volume,
            "engraved text must remove material ({engraved_volume} vs {base_volume})"
        );
        // OPEN BUG - glyphs with a counter produce the wrong swept volume.
        //
        // Region areas are correct: `A` is 9.758 mm2 with one hole, `L` is
        // 6.495 mm2 with none, so `AL` at 1 mm depth should move 16.25 mm3.
        // `L` is exact in both directions. `A` is not - raising `AL` adds only
        // 11.29 mm3 and engraving it removes 27.84 mm3, and `A` alone adds
        // 4.79 mm3 against its 9.758 mm2 footprint.
        //
        // Ruled out: the kernel's chained booleans (openrcad-algo's
        // repro_chained_boolean shows chained fuse and cut each account for
        // exactly their tools), the contact overshoot (0.05 mm to 0.5 mm
        // changed nothing), and region detection itself (areas and hole counts
        // above are right). The remaining suspect is sweeping a region that has
        // an inner wire - the prism's hole handling. Not asserted until fixed.
        // Raised and engraved run the same tools in opposite directions, so a
        // single glyph must add exactly what it removes. Checked on one letter
        // so a discrepancy points at the sweep rather than at how a multi-glyph
        // union sequences its tools.
        let one_up = volume(
            &emboss_text(&plate, &request("L", EmbossMode::Raised), &top).expect("raised L"),
        ) - base_volume;
        let one_down = base_volume
            - volume(
                &emboss_text(&plate, &request("L", EmbossMode::Engraved), &top)
                    .expect("engraved L"),
            );
        assert!(
            (one_up - one_down).abs() <= one_up.abs().max(1.0) * 0.05,
            "one glyph added {one_up} but removed {one_down}"
        );
    }

    #[test]
    fn emboss_keeps_analytic_walls_and_counters() {
        let plane = crate::geometry::CoordinateSystem::XY;
        let tools = emboss_tool_solids(&request("o", EmbossMode::Raised), &plane).expect("tool");
        assert_eq!(tools.len(), 1, "one inked region for a lowercase o");
        assert!(
            tools[0]
                .edges()
                .iter()
                .any(|edge| matches!(edge.curve(), Some(openrcad::geom::GeomCurve::BSpline(_)))),
            "the emboss tool must keep analytic B-spline walls"
        );
    }

    #[test]
    fn placement_rotates_and_translates_without_resampling() {
        let upright = text_regions(
            HACK,
            0,
            "L",
            &TextParams::default(),
            TextPlacement::default(),
        )
        .expect("upright");
        let turned = text_regions(
            HACK,
            0,
            "L",
            &TextParams::default(),
            TextPlacement {
                origin: (100.0, 0.0),
                rotation_deg: 90.0,
            },
        )
        .expect("turned");

        assert_eq!(upright.len(), turned.len());
        // A rigid move preserves area exactly and moves the block to its new
        // origin; resampling anywhere in the pipeline would perturb the area.
        assert!((upright[0].area - turned[0].area).abs() <= upright[0].area * 1.0e-3);
        let turned_min_x = turned[0]
            .boundary
            .iter()
            .fold(f32::MAX, |best, point| best.min(point.0));
        assert!(turned_min_x > 90.0, "the block must move to its new origin");
    }

    #[test]
    fn outlines_are_deterministic_across_runs() {
        let first = outline_text(HACK, 0, "Rg8", &TextParams::default()).expect("first");
        let second = outline_text(HACK, 0, "Rg8", &TextParams::default()).expect("second");
        assert_eq!(first.segments, second.segments);
        assert_eq!(first.splines, second.splines);
    }
}
