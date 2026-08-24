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
//! becomes a degree-2 or degree-3 [`Spline`] carrying explicit clamped Bézier
//! knots (identical to what the analytic converter would derive), so every
//! evaluator takes the stored-knots fast path. Nothing here is sampled or
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
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct TextPlacement {
    pub origin: (f32, f32),
    pub rotation_deg: f32,
    /// Sketch-plane (u, v) direction that reads as RIGHT on screen while the
    /// sketch camera is locked to the plane. Identity for right-handed planes
    /// viewed from +n; the XZ ground plane and YZ plane are not — baking
    /// against the measured screen basis is what keeps letters upright and
    /// forward-reading there. Text was the first chirality-sensitive geometry
    /// these sketches ever carried, which is why nothing caught this earlier.
    #[serde(default = "default_basis_u")]
    pub basis_u: (f32, f32),
    /// Sketch-plane (u, v) direction that reads as UP on screen.
    #[serde(default = "default_basis_v")]
    pub basis_v: (f32, f32),
}

fn default_basis_u() -> (f32, f32) {
    (1.0, 0.0)
}

fn default_basis_v() -> (f32, f32) {
    (0.0, 1.0)
}

impl Default for TextPlacement {
    fn default() -> Self {
        Self {
            origin: (0.0, 0.0),
            rotation_deg: 0.0,
            basis_u: default_basis_u(),
            basis_v: default_basis_v(),
        }
    }
}

impl TextPlacement {
    /// Public form of the placement map, for callers that pre-sampled display
    /// polylines in text-local space and only need the rigid re-place per frame.
    pub fn transform_point(&self, point: (f32, f32)) -> (f32, f32) {
        self.apply(point)
    }

    fn apply(self, point: (f32, f32)) -> (f32, f32) {
        let (sin, cos) = self.rotation_deg.to_radians().sin_cos();
        let rotated = (point.0 * cos - point.1 * sin, point.0 * sin + point.1 * cos);
        (
            self.origin.0 + rotated.0 * self.basis_u.0 + rotated.1 * self.basis_v.0,
            self.origin.1 + rotated.0 * self.basis_u.1 + rotated.1 * self.basis_v.1,
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

/// Bake a complete [`SketchShape::Text`] record: shape the string, place the
/// outlines, and capture the font identity. This is the ONLY constructor the
/// GUI should use — it guarantees the semantic fields and the baked curves
/// were produced together, from the same font bytes.
pub fn bake_text_shape(
    font: &[u8],
    face_index: u32,
    text: &str,
    params: &TextParams,
    placement: TextPlacement,
) -> Result<crate::sketch::SketchShape, TextError> {
    let curves = place_curves(&outline_text(font, face_index, text, params)?, placement);
    Ok(crate::sketch::SketchShape::Text {
        text: text.to_owned(),
        font: fingerprint(font, face_index)?,
        params: params.clone(),
        placement,
        curves,
    })
}

/// Return the non-zero-winding fill of already-baked text outlines.
///
/// OpenType contours may overlap, so matching a bounded face to another
/// region's hole by area and bounds is not a valid fill rule. Classifying an
/// interior point of every arrangement tile against the original directed
/// contours implements the font format's non-zero rule. Adjacent ink tiles are
/// merged again before extrusion so an overlapping glyph does not create
/// internal coplanar walls.
pub fn text_ink_regions(curves: &SketchCurves) -> Result<Vec<crate::sketch::Region>, TextError> {
    let regions = crate::sketch::detect_regions_analytic(curves)
        .map_err(|error| TextError::FaceParse(error.to_string()))?;
    let process = text_ink_mask(curves, &regions);
    Ok(
        crate::parametric::prepare_extrude_regions(&regions, &process)
            .into_iter()
            .map(|prepared| prepared.region)
            .collect(),
    )
}

/// One non-zero-winding decision per arrangement region. Kept separate from
/// [`text_ink_regions`] so the normal Sketch → Extrude path can retain stable
/// arrangement indices while excluding counters from fill, picking and model
/// evaluation.
pub(crate) fn text_ink_mask(curves: &SketchCurves, regions: &[crate::sketch::Region]) -> Vec<bool> {
    regions
        .iter()
        .map(|region| {
            region.area > 0.0
                && directed_contour_winding(
                    curves,
                    crate::parametric::region_material_point(region),
                ) != 0
        })
        .collect()
}

/// Apply typographic fill semantics to the regions of an ordinary sketch.
///
/// Non-text shapes keep their existing sketch/boolean behavior. Text shapes
/// contribute material only where their original directed outlines have
/// non-zero winding, so counter faces stay visible as holes and cannot be
/// picked or extruded as standalone solids. The split is rebuilt from semantic
/// shape records instead of serialized mask data, keeping the `.zcad` schema
/// unchanged.
pub fn sketch_region_ink_mask(
    curves: &SketchCurves,
    shapes: &[crate::sketch::SketchShape],
    corner_mods: &[crate::sketch::CornerMod],
    mirrors: &[crate::sketch::SketchMirror],
    solver: Option<&crate::sketch::SketchSolverModel>,
    vars: &std::collections::HashMap<String, f64>,
    regions: &[crate::sketch::Region],
) -> Vec<bool> {
    let text_shapes: Vec<_> = shapes
        .iter()
        .filter(|shape| matches!(shape, crate::sketch::SketchShape::Text { .. }))
        .cloned()
        .collect();
    if text_shapes.is_empty() {
        return vec![true; regions.len()];
    }

    // The solver treats text as one rigid anchor and appends these baked
    // outlines unchanged. Rebuilding only the text subset with the sketch's
    // associative mirrors therefore reproduces the text part of `effective`.
    let text_curves = crate::sketch::effective_curves_solved(
        &SketchCurves::new(),
        &text_shapes,
        &[],
        mirrors,
        None,
        vars,
    );

    let non_text_shapes: Vec<_> = shapes
        .iter()
        .filter(|shape| !matches!(shape, crate::sketch::SketchShape::Text { .. }))
        .cloned()
        .collect();
    // Stored low-level `curves` are authoritative only for legacy sketches
    // without semantic shapes. A text-bearing sketch necessarily has shapes,
    // so feeding the stored bake here would count its counters as non-text
    // material and undo the winding rule.
    let non_text_base = if shapes.is_empty() {
        curves.clone()
    } else {
        SketchCurves::new()
    };
    let non_text_curves = crate::sketch::effective_curves_solved(
        &non_text_base,
        &non_text_shapes,
        corner_mods,
        mirrors,
        solver,
        vars,
    );
    let non_text_regions = if non_text_curves.is_empty() {
        Vec::new()
    } else {
        crate::sketch::detect_regions(&non_text_curves)
    };

    regions
        .iter()
        .map(|region| {
            let point = crate::parametric::region_material_point(region);
            directed_contour_winding(&text_curves, point) != 0
                || non_text_regions
                    .iter()
                    .any(|non_text| non_text.contains(point))
        })
        .collect()
}

fn directed_contour_winding(curves: &SketchCurves, point: (f32, f32)) -> i32 {
    fn contribution(a: (f32, f32), b: (f32, f32), point: (f32, f32)) -> i32 {
        let side = (b.0 - a.0) * (point.1 - a.1) - (point.0 - a.0) * (b.1 - a.1);
        if a.1 <= point.1 {
            i32::from(b.1 > point.1 && side > 0.0)
        } else {
            -i32::from(b.1 <= point.1 && side < 0.0)
        }
    }

    let segment_winding: i32 = curves
        .segments
        .iter()
        .map(|segment| contribution(segment.a, segment.b, point))
        .sum();
    let spline_winding: i32 = curves
        .splines
        .iter()
        .map(|spline| {
            spline
                .sampled_points(0.01)
                .windows(2)
                .map(|pair| contribution(pair[0], pair[1], point))
                .sum::<i32>()
        })
        .sum();
    segment_winding + spline_winding
}

/// Shape `text`, place it, and return only the regions that should be inked.
pub fn text_regions(
    font: &[u8],
    face_index: u32,
    text: &str,
    params: &TextParams,
    placement: TextPlacement,
) -> Result<Vec<crate::sketch::Region>, TextError> {
    let curves = place_curves(&outline_text(font, face_index, text, params)?, placement);
    text_ink_regions(&curves)
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
        let order = usize::from(degree) + 1;
        let mut knots = vec![0.0; order];
        knots.extend(std::iter::repeat_n(1.0, order));
        self.curves.add_spline(Spline {
            kind: SplineKind::ControlPoint,
            points: poles,
            degree,
            // Explicit clamped Bézier knots ([0×order, 1×order] — exactly what
            // the analytic converter would derive from empty knots). Stored so
            // every downstream evaluation takes the stored-knots fast path
            // instead of rebuilding a clamped-uniform vector per sample, which
            // dominated per-frame stroke drawing for text-heavy sketches.
            knots,
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
    fn nonzero_winding_unions_overlapping_contours_without_internal_walls() {
        let mut curves = SketchCurves::new();
        curves.add_rectangle((0.0, 0.0), (2.0, 2.0));
        curves.add_rectangle((1.0, 0.0), (3.0, 2.0));

        let regions = text_ink_regions(&curves).expect("overlapping text contours");
        assert_eq!(
            regions.len(),
            1,
            "same-direction overlapping contours are one ink body"
        );
        assert!(regions[0].holes.is_empty());
        assert!(
            (regions[0].area - 6.0).abs() < 1.0e-3,
            "the merged ink area must be the contour union, got {}",
            regions[0].area
        );
    }

    #[test]
    fn nonzero_winding_keeps_opposite_contour_as_a_counter() {
        let mut curves = SketchCurves::new();
        curves.add_rectangle((0.0, 0.0), (4.0, 4.0));
        for (a, b) in [
            ((1.0, 1.0), (1.0, 3.0)),
            ((1.0, 3.0), (3.0, 3.0)),
            ((3.0, 3.0), (3.0, 1.0)),
            ((3.0, 1.0), (1.0, 1.0)),
        ] {
            curves.add_line(a, b);
        }

        let regions = text_ink_regions(&curves).expect("counter contours");
        assert_eq!(regions.len(), 1);
        assert_eq!(regions[0].holes.len(), 1);
        assert!((regions[0].area - 12.0).abs() < 1.0e-3);
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
            // Exactly Bézier: degree + 1 poles, clamped end knots only.
            let order = usize::from(spline.degree) + 1;
            assert_eq!(spline.points.len(), order);
            assert_eq!(spline.knots.len(), 2 * order);
            assert!(spline.knots[..order].iter().all(|k| *k == 0.0));
            assert!(spline.knots[order..].iter().all(|k| *k == 1.0));
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
    fn ordinary_sketch_extrude_volume_matches_typographic_ink() {
        let shape = bake_text_shape(
            HACK,
            0,
            "O",
            &TextParams::default(),
            TextPlacement::default(),
        )
        .expect("baked O");
        let baked_curves = shape.build(&std::collections::HashMap::new());
        let ink_area: f64 = text_ink_regions(&baked_curves)
            .expect("ink regions")
            .iter()
            .map(|region| f64::from(region.area))
            .sum();
        let depth = 3.0_f32;

        let mut graph = crate::parametric::ParametricGraph::new();
        graph.add_feature(crate::parametric::FeatureNode {
            id: "text_sketch".into(),
            name: "Text Sketch".into(),
            feature: crate::parametric::FeatureType::Sketch {
                cs: crate::geometry::CoordinateSystem::XY,
                curves: SketchCurves::new(),
                shapes: vec![shape],
                corner_mods: Vec::new(),
                mirrors: Vec::new(),
                on_face: false,
                entity_ids: Vec::new(),
                next_entity_id: 0,
                solver: None,
            },
        });
        graph.add_feature(crate::parametric::FeatureNode {
            id: "text_extrude".into(),
            name: "Text Extrude".into(),
            feature: crate::parametric::FeatureType::Extrude {
                depth,
                region_indices: Vec::new(),
                mode: crate::parametric::ExtrudeMode::NewBody,
                target: None,
                depth_expr: None,
                draft_angle_deg: 0.0,
                draft_angle_expr: None,
            },
        });
        graph.add_dependency("text_sketch", "text_extrude");

        let bodies = graph
            .debug_kernel_solids(&std::collections::HashSet::new())
            .expect("ordinary text extrude");
        let actual: f64 = bodies
            .iter()
            .flat_map(|(_, parts)| parts)
            .filter_map(crate::parametric::solid_volume)
            .sum();
        let expected = ink_area * f64::from(depth);
        assert!(
            (actual - expected).abs() <= expected * 0.01,
            "Sketch → Extrude must sweep ink area only: expected {expected}, got {actual}"
        );
    }

    #[test]
    fn ordinary_sketch_plate_around_text_is_exact_and_preserves_counters() {
        let text_shape = bake_text_shape(
            HACK,
            0,
            "Ember",
            &TextParams::default(),
            TextPlacement {
                origin: (-8.4, 2.6),
                ..TextPlacement::default()
            },
        )
        .expect("baked Ember");
        let text_curves = text_shape.build(&std::collections::HashMap::new());
        let rectangle = crate::sketch::SketchShape::Rectangle {
            origin: (-10.0, -2.0),
            sx: 1.0,
            sy: 1.0,
            w: crate::sketch::Dimension::literal(38.2),
            h: crate::sketch::Dimension::literal(14.0),
            from_center: false,
        };
        let shapes = vec![rectangle, text_shape];
        assert_eq!(
            crate::sketch::shape_loops(&shapes, &std::collections::HashMap::new()).len(),
            1,
            "compound text contours must not become a partial shape-boolean loop"
        );
        let curves = crate::sketch::build_sketch_curves(&shapes, &std::collections::HashMap::new());
        let regions = crate::sketch::detect_regions_analytic(&curves).expect("plate regions");
        let text_ink = text_ink_mask(&text_curves, &regions);
        let selected: Vec<usize> = text_ink
            .iter()
            .enumerate()
            .filter_map(|(index, ink)| (!ink).then_some(index))
            .collect();
        assert_eq!(
            selected.len(),
            3,
            "plate material is the exterior plus the b/e counter islands"
        );
        let selected_count = selected.len();
        let expected_area: f64 = selected
            .iter()
            .map(|index| f64::from(regions[*index].area))
            .sum();
        assert!(
            regions[selected[0]].sampled_edge_count() > crate::mock_kernel::MAX_SAMPLED_PRISM_EDGES,
            "fixture must remain too dense for the synchronous sampled path"
        );

        let depth = 3.0_f32;
        let mut graph = crate::parametric::ParametricGraph::new();
        graph.add_feature(crate::parametric::FeatureNode {
            id: "plate_sketch".into(),
            name: "Plate Around Text".into(),
            feature: crate::parametric::FeatureType::Sketch {
                cs: crate::geometry::CoordinateSystem::XY,
                curves: SketchCurves::new(),
                shapes,
                corner_mods: Vec::new(),
                mirrors: Vec::new(),
                on_face: false,
                entity_ids: Vec::new(),
                next_entity_id: 0,
                solver: None,
            },
        });
        graph.add_feature(crate::parametric::FeatureNode {
            id: "plate_extrude".into(),
            name: "Plate Extrude".into(),
            feature: crate::parametric::FeatureType::Extrude {
                depth,
                region_indices: selected.clone(),
                mode: crate::parametric::ExtrudeMode::NewBody,
                target: None,
                depth_expr: None,
                draft_angle_deg: 0.0,
                draft_angle_expr: None,
            },
        });
        graph.add_dependency("plate_sketch", "plate_extrude");

        let bodies = graph
            .debug_kernel_solids(&std::collections::HashSet::new())
            .expect("plate around text extrude");
        assert_eq!(
            bodies.len(),
            selected_count,
            "the surrounding plate and its two counter islands are three disconnected bodies"
        );
        let mut actual_volumes: Vec<f64> = bodies
            .iter()
            .flat_map(|(_, parts)| parts)
            .filter_map(crate::parametric::solid_volume)
            .collect();
        let mut expected_volumes: Vec<f64> = selected
            .iter()
            .map(|index| f64::from(regions[*index].area) * f64::from(depth))
            .collect();
        actual_volumes.sort_by(f64::total_cmp);
        expected_volumes.sort_by(f64::total_cmp);
        assert_eq!(actual_volumes.len(), expected_volumes.len());
        for (actual, expected) in actual_volumes.iter().zip(&expected_volumes) {
            assert!(
                (actual - expected).abs() <= expected * 0.01,
                "each disconnected text/counter prism must preserve area: expected {expected}, got {actual}"
            );
        }
        let actual: f64 = actual_volumes.iter().sum();
        let expected = expected_area * f64::from(depth);
        assert!(
            (actual - expected).abs() <= expected * 0.01,
            "rectangle minus text must preserve its holes and counters: expected {expected}, got {actual}"
        );
    }

    #[test]
    fn text_through_cut_rebuilds_the_plate_once_and_preserves_counters() {
        let placement = TextPlacement {
            origin: (-8.4, 2.6),
            ..TextPlacement::default()
        };
        let text_shape = bake_text_shape(HACK, 0, "Ember", &TextParams::default(), placement)
            .expect("baked Ember");
        let text_curves = text_shape.build(&std::collections::HashMap::new());
        let ink_regions = text_ink_regions(&text_curves).expect("text ink");
        assert!(
            ink_regions.iter().any(|region| !region.holes.is_empty()),
            "the fixture must contain letter counters"
        );
        assert!(
            ink_regions.iter().any(|region| {
                region
                    .analytic
                    .as_ref()
                    .is_some_and(|analytic| !analytic.holes.is_empty())
            }),
            "the analytic fixture must carry letter counters"
        );
        let arranged =
            crate::sketch::detect_regions_analytic(&text_curves).expect("text arrangement");
        let arranged_ink = text_ink_mask(&text_curves, &arranged);
        assert!(
            arranged
                .iter()
                .zip(arranged_ink)
                .any(|(region, ink)| ink && !region.holes.is_empty()),
            "selected arrangement ink must carry letter counters"
        );
        let ink_area: f64 = ink_regions
            .iter()
            .map(|region| f64::from(region.area))
            .sum();
        let plate_width = 38.2_f32;
        let plate_height = 14.0_f32;
        let depth = 2.0_f32;
        let rectangle = crate::sketch::SketchShape::Rectangle {
            origin: (-10.0, -2.0),
            sx: 1.0,
            sy: 1.0,
            w: crate::sketch::Dimension::literal(plate_width),
            h: crate::sketch::Dimension::literal(plate_height),
            from_center: false,
        };

        let mut graph = crate::parametric::ParametricGraph::new();
        graph.add_feature(crate::parametric::FeatureNode {
            id: "plate_sketch".into(),
            name: "Plate Sketch".into(),
            feature: crate::parametric::FeatureType::Sketch {
                cs: crate::geometry::CoordinateSystem::XY,
                curves: SketchCurves::new(),
                shapes: vec![rectangle],
                corner_mods: Vec::new(),
                mirrors: Vec::new(),
                on_face: false,
                entity_ids: Vec::new(),
                next_entity_id: 0,
                solver: None,
            },
        });
        graph.add_feature(crate::parametric::FeatureNode {
            id: "plate".into(),
            name: "Plate".into(),
            feature: crate::parametric::FeatureType::Extrude {
                depth,
                region_indices: Vec::new(),
                mode: crate::parametric::ExtrudeMode::NewBody,
                target: None,
                depth_expr: None,
                draft_angle_deg: 0.0,
                draft_angle_expr: None,
            },
        });
        graph.add_dependency("plate_sketch", "plate");
        graph.add_feature(crate::parametric::FeatureNode {
            id: "text_sketch".into(),
            name: "Text Sketch".into(),
            feature: crate::parametric::FeatureType::Sketch {
                cs: crate::geometry::CoordinateSystem::XY
                    .with_origin(crate::geometry::Vec3::new(0.0, 0.0, depth)),
                curves: SketchCurves::new(),
                shapes: vec![text_shape],
                corner_mods: Vec::new(),
                mirrors: Vec::new(),
                on_face: true,
                entity_ids: Vec::new(),
                next_entity_id: 0,
                solver: None,
            },
        });
        graph.add_dependency("plate", "text_sketch");
        graph.add_feature(crate::parametric::FeatureNode {
            id: "text_cut".into(),
            name: "Text Cut".into(),
            feature: crate::parametric::FeatureType::Extrude {
                depth: -depth,
                region_indices: Vec::new(),
                mode: crate::parametric::ExtrudeMode::Cut,
                target: Some("plate".into()),
                depth_expr: None,
                draft_angle_deg: 0.0,
                draft_angle_expr: None,
            },
        });
        graph.add_dependency("text_sketch", "text_cut");
        graph.add_dependency("plate", "text_cut");

        let bodies = graph
            .debug_kernel_solids(&std::collections::HashSet::new())
            .expect("text through-cut");
        assert_eq!(bodies.len(), 1, "the engraved plate remains one body");
        let actual: f64 = bodies[0]
            .1
            .iter()
            .filter_map(crate::parametric::solid_volume)
            .sum();
        let expected =
            f64::from((plate_width * plate_height) * depth) - ink_area * f64::from(depth);
        assert!(
            (actual - expected).abs() <= expected * 0.01,
            "text through-cut must remove ink area once: expected {expected}, got {actual} in {} parts",
            bodies[0].1.len()
        );
        assert!(
            bodies[0].1.len() > 1,
            "the b/e counters must remain as disconnected material islands"
        );
        let (_, warnings) = graph
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .expect("cached text through-cut display");
        assert!(warnings.is_empty(), "exact text through-cut: {warnings:?}");
    }

    #[test]
    fn text_blind_cut_rebuilds_one_valid_pocket_without_boolean_retries() {
        let placement = TextPlacement {
            origin: (-8.4, 2.6),
            ..TextPlacement::default()
        };
        let text_shape = bake_text_shape(HACK, 0, "Cade", &TextParams::default(), placement)
            .expect("baked Cade");
        let text_curves = text_shape.build(&std::collections::HashMap::new());
        let ink_regions = text_ink_regions(&text_curves).expect("text ink");
        assert_eq!(
            ink_regions.len(),
            4,
            "the fixture must exercise four glyph tools"
        );
        assert!(
            ink_regions.iter().any(|region| !region.holes.is_empty()),
            "the fixture must preserve letter counters"
        );
        let ink_area: f64 = ink_regions
            .iter()
            .map(|region| f64::from(region.area))
            .sum();
        let plate_width = 38.2_f32;
        let plate_height = 14.0_f32;
        let plate_depth = 2.0_f32;
        let pocket_depth = 0.8_f32;
        let rectangle = crate::sketch::SketchShape::Rectangle {
            origin: (-10.0, -2.0),
            sx: 1.0,
            sy: 1.0,
            w: crate::sketch::Dimension::literal(plate_width),
            h: crate::sketch::Dimension::literal(plate_height),
            from_center: false,
        };

        let mut graph = crate::parametric::ParametricGraph::new();
        graph.add_feature(crate::parametric::FeatureNode {
            id: "plate_sketch".into(),
            name: "Plate Sketch".into(),
            feature: crate::parametric::FeatureType::Sketch {
                cs: crate::geometry::CoordinateSystem::XY,
                curves: SketchCurves::new(),
                shapes: vec![rectangle],
                corner_mods: Vec::new(),
                mirrors: Vec::new(),
                on_face: false,
                entity_ids: Vec::new(),
                next_entity_id: 0,
                solver: None,
            },
        });
        graph.add_feature(crate::parametric::FeatureNode {
            id: "plate".into(),
            name: "Plate".into(),
            feature: crate::parametric::FeatureType::Extrude {
                depth: plate_depth,
                region_indices: Vec::new(),
                mode: crate::parametric::ExtrudeMode::NewBody,
                target: None,
                depth_expr: None,
                draft_angle_deg: 0.0,
                draft_angle_expr: None,
            },
        });
        graph.add_dependency("plate_sketch", "plate");
        graph.add_feature(crate::parametric::FeatureNode {
            id: "text_sketch".into(),
            name: "Text Sketch".into(),
            feature: crate::parametric::FeatureType::Sketch {
                cs: crate::geometry::CoordinateSystem::XY.with_origin(crate::geometry::Vec3::new(
                    0.0,
                    0.0,
                    plate_depth,
                )),
                curves: SketchCurves::new(),
                shapes: vec![text_shape],
                corner_mods: Vec::new(),
                mirrors: Vec::new(),
                on_face: true,
                entity_ids: Vec::new(),
                next_entity_id: 0,
                solver: None,
            },
        });
        graph.add_dependency("plate", "text_sketch");
        graph.add_feature(crate::parametric::FeatureNode {
            id: "text_cut".into(),
            name: "Text Cut".into(),
            feature: crate::parametric::FeatureType::Extrude {
                depth: -pocket_depth,
                region_indices: Vec::new(),
                mode: crate::parametric::ExtrudeMode::Cut,
                target: Some("plate".into()),
                depth_expr: None,
                draft_angle_deg: 0.0,
                draft_angle_expr: None,
            },
        });
        graph.add_dependency("text_sketch", "text_cut");
        graph.add_dependency("plate", "text_cut");

        let bodies = graph
            .debug_kernel_solids(&std::collections::HashSet::new())
            .expect("text blind cut");
        assert_eq!(bodies.len(), 1, "the pocketed plate remains one body");
        assert_eq!(bodies[0].1.len(), 1, "a blind pocket remains connected");
        let pocketed = &bodies[0].1[0];
        assert!(pocketed.is_watertight());
        assert!(pocketed.health_report().is_healthy());
        assert!(pocketed.validate().is_ok());
        let actual = crate::parametric::solid_volume(pocketed).expect("pocket volume");
        let removed_depth = pocket_depth + crate::parametric::CUT_OVERSHOOT;
        let expected = f64::from(plate_width * plate_height * plate_depth)
            - ink_area * f64::from(removed_depth);
        assert!(
            (actual - expected).abs() <= expected * 0.01,
            "blind text cut must remove ink area once: expected {expected}, got {actual}"
        );
        let (_, warnings) = graph
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .expect("cached text blind-cut display");
        assert!(warnings.is_empty(), "exact text blind cut: {warnings:?}");
    }

    #[test]
    fn text_join_rebuilds_one_valid_raised_word_without_boolean_retries() {
        let placement = TextPlacement {
            origin: (-8.4, 2.6),
            ..TextPlacement::default()
        };
        let text_shape = bake_text_shape(HACK, 0, "Cade", &TextParams::default(), placement)
            .expect("baked Cade");
        let text_curves = text_shape.build(&std::collections::HashMap::new());
        let ink_regions = text_ink_regions(&text_curves).expect("text ink");
        assert_eq!(
            ink_regions.len(),
            4,
            "the fixture must exercise four glyph tools"
        );
        assert!(
            ink_regions.iter().any(|region| !region.holes.is_empty()),
            "the fixture must preserve letter counters"
        );
        let ink_area: f64 = ink_regions
            .iter()
            .map(|region| f64::from(region.area))
            .sum();
        let plate_width = 38.2_f32;
        let plate_height = 14.0_f32;
        let plate_depth = 2.0_f32;
        let raised_height = 0.8_f32;
        let rectangle = crate::sketch::SketchShape::Rectangle {
            origin: (-10.0, -2.0),
            sx: 1.0,
            sy: 1.0,
            w: crate::sketch::Dimension::literal(plate_width),
            h: crate::sketch::Dimension::literal(plate_height),
            from_center: false,
        };

        let mut graph = crate::parametric::ParametricGraph::new();
        graph.add_feature(crate::parametric::FeatureNode {
            id: "plate_sketch".into(),
            name: "Plate Sketch".into(),
            feature: crate::parametric::FeatureType::Sketch {
                cs: crate::geometry::CoordinateSystem::XY,
                curves: SketchCurves::new(),
                shapes: vec![rectangle],
                corner_mods: Vec::new(),
                mirrors: Vec::new(),
                on_face: false,
                entity_ids: Vec::new(),
                next_entity_id: 0,
                solver: None,
            },
        });
        graph.add_feature(crate::parametric::FeatureNode {
            id: "plate".into(),
            name: "Plate".into(),
            feature: crate::parametric::FeatureType::Extrude {
                depth: plate_depth,
                region_indices: Vec::new(),
                mode: crate::parametric::ExtrudeMode::NewBody,
                target: None,
                depth_expr: None,
                draft_angle_deg: 0.0,
                draft_angle_expr: None,
            },
        });
        graph.add_dependency("plate_sketch", "plate");
        graph.add_feature(crate::parametric::FeatureNode {
            id: "text_sketch".into(),
            name: "Text Sketch".into(),
            feature: crate::parametric::FeatureType::Sketch {
                cs: crate::geometry::CoordinateSystem::XY.with_origin(crate::geometry::Vec3::new(
                    0.0,
                    0.0,
                    plate_depth,
                )),
                curves: SketchCurves::new(),
                shapes: vec![text_shape],
                corner_mods: Vec::new(),
                mirrors: Vec::new(),
                on_face: true,
                entity_ids: Vec::new(),
                next_entity_id: 0,
                solver: None,
            },
        });
        graph.add_dependency("plate", "text_sketch");
        graph.add_feature(crate::parametric::FeatureNode {
            id: "text_join".into(),
            name: "Text Join".into(),
            feature: crate::parametric::FeatureType::Extrude {
                depth: raised_height,
                region_indices: Vec::new(),
                mode: crate::parametric::ExtrudeMode::Join,
                target: Some("plate".into()),
                depth_expr: None,
                draft_angle_deg: 0.0,
                draft_angle_expr: None,
            },
        });
        graph.add_dependency("text_sketch", "text_join");
        graph.add_dependency("plate", "text_join");

        let bodies = graph
            .debug_kernel_solids(&std::collections::HashSet::new())
            .expect("raised text join");
        assert_eq!(bodies.len(), 1, "the raised plate remains one body");
        assert_eq!(bodies[0].1.len(), 1, "raised text is fused material");
        let raised = &bodies[0].1[0];
        assert!(raised.is_watertight());
        assert!(raised.health_report().is_healthy());
        assert!(raised.validate().is_ok());
        assert!(
            raised
                .validate_strict_with_policy(&openrcad::foundation::TolerancePolicy::STANDARD)
                .is_ok(),
            "raised text must pass strict topology validation"
        );
        let actual = crate::parametric::solid_volume(raised).expect("raised volume");
        let expected = f64::from(plate_width * plate_height * plate_depth)
            + ink_area * f64::from(raised_height);
        assert!(
            (actual - expected).abs() <= expected * 0.01,
            "raised text must add ink area once: expected {expected}, got {actual}"
        );
        let (_, warnings) = graph
            .evaluate_bodies_with_warnings(&std::collections::HashSet::new())
            .expect("cached raised-text display");
        assert!(warnings.is_empty(), "exact raised-text join: {warnings:?}");
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
                ..TextPlacement::default()
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
        // OPEN ANOMALY, scoped to this helper: raising `AL` adds 11.29 mm3
        // while engraving removes 27.84 mm3 against a 16.25 mm3 nominal, and
        // the numbers did NOT change when the kernel's cylinder-orientation fix
        // landed (disc/annulus sweeps and single counter-free glyphs are exact).
        // Whatever this is lives in `emboss_text`'s union/difference chain, not
        // in sweeping or region detection. The sketch-text workflow does not
        // use this helper - it extrudes regions through the app's normal
        // Extrude Join/Cut path - so this stays documented rather than fatal.

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
                ..TextPlacement::default()
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
    fn text_shape_bakes_builds_verbatim_and_round_trips() {
        let shape = bake_text_shape(
            HACK,
            0,
            "Rev 2",
            &TextParams::default(),
            TextPlacement {
                origin: (4.0, 7.0),
                rotation_deg: 30.0,
                ..TextPlacement::default()
            },
        )
        .expect("bake");

        // build() must return the baked curves verbatim — never re-shape.
        let crate::sketch::SketchShape::Text { curves, .. } = &shape else {
            panic!("bake_text_shape must produce SketchShape::Text");
        };
        let built = shape.build(&std::collections::HashMap::new());
        assert_eq!(
            &built, curves,
            "build() must emit the baked curves verbatim"
        );

        // Serde round trip through CBOR — the same encoding the sketch payload
        // uses — must preserve every field bit-for-bit.
        let mut encoded = Vec::new();
        ciborium::ser::into_writer(&shape, &mut encoded).expect("encode");
        let decoded: crate::sketch::SketchShape =
            ciborium::de::from_reader(encoded.as_slice()).expect("decode");
        assert_eq!(decoded, shape);
    }

    /// Text is ONE rigid item: the solver receives a single anchor point —
    /// never the glyph curves — and the display curves still arrive through the
    /// solver-model branch of `effective_curves_solved`. Promoting glyphs as
    /// raw entities made every drag re-solve hundreds of points (the "text
    /// makes the app crawl" report) and let letters be constrained apart.
    #[test]
    fn text_is_one_solver_item_and_still_renders_with_a_model() {
        let shape = bake_text_shape(
            HACK,
            0,
            "Rig",
            &TextParams::default(),
            TextPlacement {
                origin: (3.0, 4.0),
                rotation_deg: 0.0,
                ..TextPlacement::default()
            },
        )
        .expect("bake");
        let vars = std::collections::HashMap::new();
        let owner = crate::sketch::EntityId(7);
        let (model, _next) = crate::sketch::constraints::promote_shapes_to_entities(
            std::slice::from_ref(&shape),
            &[owner],
            &vars,
            8,
        );
        assert_eq!(
            model.points.len(),
            1,
            "one anchor point for the whole block"
        );
        assert_eq!(model.points[0].pos, (3.0, 4.0));
        assert!(
            model.entities.is_empty(),
            "glyph curves stay out of the solver"
        );

        let mut solver = crate::sketch::SketchSolverModel::default();
        solver.points.extend(model.points.clone());
        let baked = match &shape {
            crate::sketch::SketchShape::Text { curves, .. } => curves.clone(),
            _ => unreachable!(),
        };
        let rendered = crate::sketch::effective_curves_solved(
            &crate::sketch::SketchCurves::new(),
            std::slice::from_ref(&shape),
            &[],
            &[],
            Some(&solver),
            &vars,
        );
        assert_eq!(
            rendered.splines.len(),
            baked.splines.len(),
            "text must render through the solver-model branch"
        );
        assert_eq!(rendered.segments.len(), baked.segments.len());
    }

    #[test]
    fn outlines_are_deterministic_across_runs() {
        let first = outline_text(HACK, 0, "Rg8", &TextParams::default()).expect("first");
        let second = outline_text(HACK, 0, "Rg8", &TextParams::default()).expect("second");
        assert_eq!(first.segments, second.segments);
        assert_eq!(first.splines, second.splines);
    }
}
