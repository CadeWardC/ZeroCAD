//! Thread standard presets. A modeled thread ([`crate::parametric::FeatureType::Thread`])
//! stores only numeric fields (pitch, depth, angle, …); the *standard* is just a
//! convenient way to fill them. Metric and Unified are lookup tables keyed by
//! nominal size; Custom lets the user type the fields directly. All three feed
//! the same `ThreadSpec` at build time, so there is one geometry path.

/// One named thread size, resolvable to the numeric fields a Thread feature stores.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThreadPreset {
    /// Display designation, e.g. `"M6×1"` or `"1/4-20 UNC"`.
    pub designation: &'static str,
    /// Nominal major (crest) diameter in millimeters.
    pub major_dia_mm: f32,
    /// Axial pitch in millimeters (Unified rows convert from TPI).
    pub pitch_mm: f32,
}

/// Which family of preset sizes the thread dialog is offering.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThreadStandard {
    /// ISO metric coarse (M) — 60° V-thread.
    MetricCoarse,
    /// Unified coarse (UNC) — 60° V-thread, imperial.
    UnifiedCoarse,
    /// Unified fine (UNF) — 60° V-thread, imperial.
    UnifiedFine,
    /// User-defined pitch/depth/angle.
    Custom,
}

impl ThreadStandard {
    /// Short label for the standard picker.
    pub fn label(self) -> &'static str {
        match self {
            ThreadStandard::MetricCoarse => "Metric (M)",
            ThreadStandard::UnifiedCoarse => "Unified coarse (UNC)",
            ThreadStandard::UnifiedFine => "Unified fine (UNF)",
            ThreadStandard::Custom => "Custom",
        }
    }

    /// The preset rows for this standard (empty for [`ThreadStandard::Custom`]).
    pub fn presets(self) -> &'static [ThreadPreset] {
        match self {
            ThreadStandard::MetricCoarse => METRIC_COARSE,
            ThreadStandard::UnifiedCoarse => UNIFIED_COARSE,
            ThreadStandard::UnifiedFine => UNIFIED_FINE,
            ThreadStandard::Custom => &[],
        }
    }

    /// The included thread angle in degrees (60° for every standard here).
    pub fn angle_deg(self) -> f32 {
        60.0
    }
}

/// Default radial thread depth (crest to root) for a 60° V-thread of the given
/// pitch: 5⁄8 of the theoretical triangle height H = (√3/2)·pitch, i.e. the
/// standard external engagement depth ≈ 0.6134·pitch.
pub fn default_depth_mm(pitch_mm: f32) -> f32 {
    0.613_43 * pitch_mm
}

/// Convert Unified threads-per-inch to a metric pitch (mm).
const fn tpi(t: f32) -> f32 {
    25.4 / t
}
/// Convert an inch dimension to millimeters.
const fn inch(i: f32) -> f32 {
    i * 25.4
}

/// ISO metric coarse pitch series (M1.6–M24).
pub static METRIC_COARSE: &[ThreadPreset] = &[
    ThreadPreset {
        designation: "M2×0.4",
        major_dia_mm: 2.0,
        pitch_mm: 0.4,
    },
    ThreadPreset {
        designation: "M2.5×0.45",
        major_dia_mm: 2.5,
        pitch_mm: 0.45,
    },
    ThreadPreset {
        designation: "M3×0.5",
        major_dia_mm: 3.0,
        pitch_mm: 0.5,
    },
    ThreadPreset {
        designation: "M4×0.7",
        major_dia_mm: 4.0,
        pitch_mm: 0.7,
    },
    ThreadPreset {
        designation: "M5×0.8",
        major_dia_mm: 5.0,
        pitch_mm: 0.8,
    },
    ThreadPreset {
        designation: "M6×1",
        major_dia_mm: 6.0,
        pitch_mm: 1.0,
    },
    ThreadPreset {
        designation: "M8×1.25",
        major_dia_mm: 8.0,
        pitch_mm: 1.25,
    },
    ThreadPreset {
        designation: "M10×1.5",
        major_dia_mm: 10.0,
        pitch_mm: 1.5,
    },
    ThreadPreset {
        designation: "M12×1.75",
        major_dia_mm: 12.0,
        pitch_mm: 1.75,
    },
    ThreadPreset {
        designation: "M16×2",
        major_dia_mm: 16.0,
        pitch_mm: 2.0,
    },
    ThreadPreset {
        designation: "M20×2.5",
        major_dia_mm: 20.0,
        pitch_mm: 2.5,
    },
    ThreadPreset {
        designation: "M24×3",
        major_dia_mm: 24.0,
        pitch_mm: 3.0,
    },
];

/// Unified coarse series (UNC).
pub static UNIFIED_COARSE: &[ThreadPreset] = &[
    ThreadPreset {
        designation: "#4-40 UNC",
        major_dia_mm: inch(0.112),
        pitch_mm: tpi(40.0),
    },
    ThreadPreset {
        designation: "#6-32 UNC",
        major_dia_mm: inch(0.138),
        pitch_mm: tpi(32.0),
    },
    ThreadPreset {
        designation: "#8-32 UNC",
        major_dia_mm: inch(0.164),
        pitch_mm: tpi(32.0),
    },
    ThreadPreset {
        designation: "#10-24 UNC",
        major_dia_mm: inch(0.190),
        pitch_mm: tpi(24.0),
    },
    ThreadPreset {
        designation: "1/4-20 UNC",
        major_dia_mm: inch(0.25),
        pitch_mm: tpi(20.0),
    },
    ThreadPreset {
        designation: "5/16-18 UNC",
        major_dia_mm: inch(0.3125),
        pitch_mm: tpi(18.0),
    },
    ThreadPreset {
        designation: "3/8-16 UNC",
        major_dia_mm: inch(0.375),
        pitch_mm: tpi(16.0),
    },
    ThreadPreset {
        designation: "1/2-13 UNC",
        major_dia_mm: inch(0.5),
        pitch_mm: tpi(13.0),
    },
];

/// Unified fine series (UNF).
pub static UNIFIED_FINE: &[ThreadPreset] = &[
    ThreadPreset {
        designation: "#4-48 UNF",
        major_dia_mm: inch(0.112),
        pitch_mm: tpi(48.0),
    },
    ThreadPreset {
        designation: "#6-40 UNF",
        major_dia_mm: inch(0.138),
        pitch_mm: tpi(40.0),
    },
    ThreadPreset {
        designation: "#8-36 UNF",
        major_dia_mm: inch(0.164),
        pitch_mm: tpi(36.0),
    },
    ThreadPreset {
        designation: "#10-32 UNF",
        major_dia_mm: inch(0.190),
        pitch_mm: tpi(32.0),
    },
    ThreadPreset {
        designation: "1/4-28 UNF",
        major_dia_mm: inch(0.25),
        pitch_mm: tpi(28.0),
    },
    ThreadPreset {
        designation: "5/16-24 UNF",
        major_dia_mm: inch(0.3125),
        pitch_mm: tpi(24.0),
    },
    ThreadPreset {
        designation: "3/8-24 UNF",
        major_dia_mm: inch(0.375),
        pitch_mm: tpi(24.0),
    },
    ThreadPreset {
        designation: "1/2-20 UNF",
        major_dia_mm: inch(0.5),
        pitch_mm: tpi(20.0),
    },
];

/// Pick the preset whose major diameter is closest to `dia_mm` (for defaulting
/// the dialog to the size that matches a just-selected cylindrical face).
pub fn closest_preset(standard: ThreadStandard, dia_mm: f32) -> Option<&'static ThreadPreset> {
    standard.presets().iter().min_by(|a, b| {
        (a.major_dia_mm - dia_mm)
            .abs()
            .partial_cmp(&(b.major_dia_mm - dia_mm).abs())
            .unwrap_or(std::cmp::Ordering::Equal)
    })
}
