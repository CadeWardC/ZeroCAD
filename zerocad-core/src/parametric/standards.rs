//! Versioned engineering standards used by Hole and Thread features.
//!
//! A feature stores both a [`StandardReference`] and its resolved numeric
//! geometry. Table updates therefore affect new selections only; rebuilding an
//! existing document never consults the live table.

pub const STANDARDS_LIBRARY_ID: &str = "org.zerocad.part-design-standards";
pub const STANDARDS_LIBRARY_VERSION: u32 = 2;

/// Provenance of the bundled append-only data packs. A document stores the
/// selected version plus resolved dimensions; retaining prior pack identities
/// makes upgrades auditable without allowing a table refresh to mutate an
/// existing feature.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StandardsDataPack {
    pub version: u32,
    pub coverage: &'static str,
}

pub const STANDARDS_DATA_PACKS: &[StandardsDataPack] = &[
    StandardsDataPack {
        version: 1,
        coverage: "curated ISO/ANSI M3-M12 and #10-3/8 inch",
    },
    StandardsDataPack {
        version: 2,
        coverage: "expanded ISO M3-M24 and ANSI #2-1/2 inch",
    },
];

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum StandardsFamily {
    Iso,
    Ansi,
}

impl StandardsFamily {
    pub fn label(self) -> &'static str {
        match self {
            Self::Iso => "ISO",
            Self::Ansi => "ANSI",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum HoleApplication {
    Clearance,
    Tapped,
    Counterbore,
    Countersink,
}

impl HoleApplication {
    pub fn label(self) -> &'static str {
        match self {
            Self::Clearance => "Clearance",
            Self::Tapped => "Tapped",
            Self::Counterbore => "Counterbore",
            Self::Countersink => "Countersink",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum HoleFit {
    Close,
    Normal,
    Loose,
}

/// Manufacturing intent that does not need to be rediscovered from dimensions.
/// The drill point is modeled for blind holes; tapped-hole thread markings stay
/// cosmetic until a separate modeled Thread feature is requested.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct HoleManufacturingMetadata {
    pub application: HoleApplication,
    pub drill_point_angle_deg: Option<f32>,
    pub cosmetic_thread: bool,
    pub thread_designation: Option<String>,
    pub thread_class: Option<String>,
    pub tap_pitch_mm: Option<f32>,
}

impl HoleFit {
    pub fn label(self) -> &'static str {
        match self {
            Self::Close => "Close",
            Self::Normal => "Normal",
            Self::Loose => "Loose",
        }
    }
}

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum ResolvedStandardGeometry {
    Hole {
        bore_diameter_mm: f32,
        head_diameter_mm: Option<f32>,
        head_depth_mm: Option<f32>,
        head_angle_deg: Option<f32>,
        tap_pitch_mm: Option<f32>,
    },
    Thread {
        major_diameter_mm: f32,
        pitch_mm: f32,
        radial_depth_mm: f32,
        angle_deg: f32,
    },
}

/// Stable selection identity plus the numeric values resolved when the feature
/// was created. `library_version` is intentionally independent of the `.zcad`
/// container version.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct StandardReference {
    pub library_id: String,
    pub library_version: u32,
    pub family: StandardsFamily,
    pub table: String,
    pub designation: String,
    pub class: Option<String>,
    pub fit: Option<HoleFit>,
    pub resolved: ResolvedStandardGeometry,
}

impl StandardReference {
    fn new(
        family: StandardsFamily,
        table: &str,
        designation: &str,
        class: Option<&str>,
        fit: Option<HoleFit>,
        resolved: ResolvedStandardGeometry,
    ) -> Self {
        Self {
            library_id: STANDARDS_LIBRARY_ID.to_string(),
            library_version: STANDARDS_LIBRARY_VERSION,
            family,
            table: table.to_string(),
            designation: designation.to_string(),
            class: class.map(str::to_string),
            fit,
            resolved,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HoleStandardPreset {
    pub family: StandardsFamily,
    pub table: &'static str,
    pub designation: &'static str,
    pub application: HoleApplication,
    pub fit: HoleFit,
    pub class: Option<&'static str>,
    pub bore_diameter_mm: f32,
    pub head_diameter_mm: Option<f32>,
    pub head_depth_mm: Option<f32>,
    pub head_angle_deg: Option<f32>,
    pub tap_pitch_mm: Option<f32>,
}

impl HoleStandardPreset {
    pub fn reference_with_resolved(
        self,
        bore_diameter_mm: f32,
        head_diameter_mm: Option<f32>,
        head_depth_mm: Option<f32>,
        head_angle_deg: Option<f32>,
    ) -> StandardReference {
        StandardReference::new(
            self.family,
            self.table,
            self.designation,
            self.class,
            Some(self.fit),
            ResolvedStandardGeometry::Hole {
                bore_diameter_mm,
                head_diameter_mm,
                head_depth_mm,
                head_angle_deg,
                tap_pitch_mm: self.tap_pitch_mm,
            },
        )
    }

    pub fn manufacturing_metadata(
        self,
        drill_point_angle_deg: Option<f32>,
    ) -> HoleManufacturingMetadata {
        let tapped = self.application == HoleApplication::Tapped;
        HoleManufacturingMetadata {
            application: self.application,
            drill_point_angle_deg,
            cosmetic_thread: tapped,
            thread_designation: tapped.then(|| self.designation.to_string()),
            thread_class: self.class.map(str::to_string),
            tap_pitch_mm: self.tap_pitch_mm,
        }
    }
}

const fn inch(value: f32) -> f32 {
    value * 25.4
}

macro_rules! hole {
    ($family:ident, $table:literal, $designation:literal, $application:ident, $fit:ident,
     $class:expr, $bore:expr, $head_dia:expr, $head_depth:expr, $head_angle:expr, $pitch:expr) => {
        HoleStandardPreset {
            family: StandardsFamily::$family,
            table: $table,
            designation: $designation,
            application: HoleApplication::$application,
            fit: HoleFit::$fit,
            class: $class,
            bore_diameter_mm: $bore,
            head_diameter_mm: $head_dia,
            head_depth_mm: $head_depth,
            head_angle_deg: $head_angle,
            tap_pitch_mm: $pitch,
        }
    };
}

/// Curated Phase-4 v1 tables. Values are resolved dimensions in millimeters;
/// table identifiers identify the source convention and are persisted.
pub static HOLE_STANDARD_PRESETS: &[HoleStandardPreset] = &[
    // ISO 273 medium-series clearance holes.
    hole!(Iso, "ISO 273", "M3", Clearance, Normal, None, 3.4, None, None, None, None),
    hole!(Iso, "ISO 273", "M4", Clearance, Normal, None, 4.5, None, None, None, None),
    hole!(Iso, "ISO 273", "M5", Clearance, Normal, None, 5.5, None, None, None, None),
    hole!(Iso, "ISO 273", "M6", Clearance, Normal, None, 6.6, None, None, None, None),
    hole!(Iso, "ISO 273", "M8", Clearance, Normal, None, 9.0, None, None, None, None),
    hole!(Iso, "ISO 273", "M10", Clearance, Normal, None, 11.0, None, None, None, None),
    hole!(Iso, "ISO 273", "M12", Clearance, Normal, None, 13.5, None, None, None, None),
    // ISO 273 fine and coarse series, exposed as Close and Loose fits.
    hole!(Iso, "ISO 273", "M3", Clearance, Close, None, 3.2, None, None, None, None),
    hole!(Iso, "ISO 273", "M4", Clearance, Close, None, 4.3, None, None, None, None),
    hole!(Iso, "ISO 273", "M5", Clearance, Close, None, 5.3, None, None, None, None),
    hole!(Iso, "ISO 273", "M6", Clearance, Close, None, 6.4, None, None, None, None),
    hole!(Iso, "ISO 273", "M8", Clearance, Close, None, 8.4, None, None, None, None),
    hole!(Iso, "ISO 273", "M10", Clearance, Close, None, 10.5, None, None, None, None),
    hole!(Iso, "ISO 273", "M12", Clearance, Close, None, 13.0, None, None, None, None),
    hole!(Iso, "ISO 273", "M3", Clearance, Loose, None, 3.6, None, None, None, None),
    hole!(Iso, "ISO 273", "M4", Clearance, Loose, None, 4.8, None, None, None, None),
    hole!(Iso, "ISO 273", "M5", Clearance, Loose, None, 5.8, None, None, None, None),
    hole!(Iso, "ISO 273", "M6", Clearance, Loose, None, 7.0, None, None, None, None),
    hole!(Iso, "ISO 273", "M8", Clearance, Loose, None, 10.0, None, None, None, None),
    hole!(Iso, "ISO 273", "M10", Clearance, Loose, None, 12.0, None, None, None, None),
    hole!(Iso, "ISO 273", "M12", Clearance, Loose, None, 15.0, None, None, None, None),
    // ISO metric coarse tap drills (6H internal thread class).
    hole!(
        Iso,
        "ISO 261 / tap drill",
        "M3×0.5",
        Tapped,
        Normal,
        Some("6H"),
        2.5,
        None,
        None,
        None,
        Some(0.5)
    ),
    hole!(
        Iso,
        "ISO 261 / tap drill",
        "M4×0.7",
        Tapped,
        Normal,
        Some("6H"),
        3.3,
        None,
        None,
        None,
        Some(0.7)
    ),
    hole!(
        Iso,
        "ISO 261 / tap drill",
        "M5×0.8",
        Tapped,
        Normal,
        Some("6H"),
        4.2,
        None,
        None,
        None,
        Some(0.8)
    ),
    hole!(
        Iso,
        "ISO 261 / tap drill",
        "M6×1",
        Tapped,
        Normal,
        Some("6H"),
        5.0,
        None,
        None,
        None,
        Some(1.0)
    ),
    hole!(
        Iso,
        "ISO 261 / tap drill",
        "M8×1.25",
        Tapped,
        Normal,
        Some("6H"),
        6.8,
        None,
        None,
        None,
        Some(1.25)
    ),
    hole!(
        Iso,
        "ISO 261 / tap drill",
        "M10×1.5",
        Tapped,
        Normal,
        Some("6H"),
        8.5,
        None,
        None,
        None,
        Some(1.5)
    ),
    hole!(
        Iso,
        "ISO 261 / tap drill",
        "M12×1.75",
        Tapped,
        Normal,
        Some("6H"),
        10.2,
        None,
        None,
        None,
        Some(1.75)
    ),
    // ISO 4762 socket-head counterbores.
    hole!(
        Iso,
        "ISO 4762",
        "M3",
        Counterbore,
        Normal,
        None,
        3.4,
        Some(6.5),
        Some(3.3),
        None,
        None
    ),
    hole!(
        Iso,
        "ISO 4762",
        "M4",
        Counterbore,
        Normal,
        None,
        4.5,
        Some(8.0),
        Some(4.4),
        None,
        None
    ),
    hole!(
        Iso,
        "ISO 4762",
        "M5",
        Counterbore,
        Normal,
        None,
        5.5,
        Some(10.0),
        Some(5.4),
        None,
        None
    ),
    hole!(
        Iso,
        "ISO 4762",
        "M6",
        Counterbore,
        Normal,
        None,
        6.6,
        Some(11.0),
        Some(6.5),
        None,
        None
    ),
    hole!(
        Iso,
        "ISO 4762",
        "M8",
        Counterbore,
        Normal,
        None,
        9.0,
        Some(15.0),
        Some(8.6),
        None,
        None
    ),
    hole!(
        Iso,
        "ISO 4762",
        "M10",
        Counterbore,
        Normal,
        None,
        11.0,
        Some(18.0),
        Some(10.6),
        None,
        None
    ),
    // ISO 10642 90-degree flat-head clearance.
    hole!(
        Iso,
        "ISO 10642",
        "M3",
        Countersink,
        Normal,
        None,
        3.4,
        Some(6.0),
        None,
        Some(90.0),
        None
    ),
    hole!(
        Iso,
        "ISO 10642",
        "M4",
        Countersink,
        Normal,
        None,
        4.5,
        Some(8.0),
        None,
        Some(90.0),
        None
    ),
    hole!(
        Iso,
        "ISO 10642",
        "M5",
        Countersink,
        Normal,
        None,
        5.5,
        Some(10.0),
        None,
        Some(90.0),
        None
    ),
    hole!(
        Iso,
        "ISO 10642",
        "M6",
        Countersink,
        Normal,
        None,
        6.6,
        Some(12.0),
        None,
        Some(90.0),
        None
    ),
    hole!(
        Iso,
        "ISO 10642",
        "M8",
        Countersink,
        Normal,
        None,
        9.0,
        Some(16.0),
        None,
        Some(90.0),
        None
    ),
    // ANSI inch clearance / tap / head styles.
    hole!(
        Ansi,
        "ASME B18.2.8",
        "#10",
        Clearance,
        Normal,
        None,
        inch(0.201),
        None,
        None,
        None,
        None
    ),
    hole!(
        Ansi,
        "ASME B18.2.8",
        "1/4",
        Clearance,
        Normal,
        None,
        inch(0.266),
        None,
        None,
        None,
        None
    ),
    hole!(
        Ansi,
        "ASME B18.2.8",
        "5/16",
        Clearance,
        Normal,
        None,
        inch(0.332),
        None,
        None,
        None,
        None
    ),
    hole!(
        Ansi,
        "ASME B18.2.8",
        "3/8",
        Clearance,
        Normal,
        None,
        inch(0.397),
        None,
        None,
        None,
        None
    ),
    hole!(
        Ansi,
        "ASME B18.2.8",
        "#10",
        Clearance,
        Close,
        None,
        inch(0.196),
        None,
        None,
        None,
        None
    ),
    hole!(
        Ansi,
        "ASME B18.2.8",
        "1/4",
        Clearance,
        Close,
        None,
        inch(0.257),
        None,
        None,
        None,
        None
    ),
    hole!(
        Ansi,
        "ASME B18.2.8",
        "5/16",
        Clearance,
        Close,
        None,
        inch(0.323),
        None,
        None,
        None,
        None
    ),
    hole!(
        Ansi,
        "ASME B18.2.8",
        "3/8",
        Clearance,
        Close,
        None,
        inch(0.386),
        None,
        None,
        None,
        None
    ),
    hole!(
        Ansi,
        "ASME B18.2.8",
        "#10",
        Clearance,
        Loose,
        None,
        inch(0.213),
        None,
        None,
        None,
        None
    ),
    hole!(
        Ansi,
        "ASME B18.2.8",
        "1/4",
        Clearance,
        Loose,
        None,
        inch(0.281),
        None,
        None,
        None,
        None
    ),
    hole!(
        Ansi,
        "ASME B18.2.8",
        "5/16",
        Clearance,
        Loose,
        None,
        inch(0.348),
        None,
        None,
        None,
        None
    ),
    hole!(
        Ansi,
        "ASME B18.2.8",
        "3/8",
        Clearance,
        Loose,
        None,
        inch(0.413),
        None,
        None,
        None,
        None
    ),
    hole!(
        Ansi,
        "ASME B1.1 / tap drill",
        "#10-24 UNC",
        Tapped,
        Normal,
        Some("2B"),
        inch(0.1495),
        None,
        None,
        None,
        Some(inch(1.0 / 24.0))
    ),
    hole!(
        Ansi,
        "ASME B1.1 / tap drill",
        "1/4-20 UNC",
        Tapped,
        Normal,
        Some("2B"),
        inch(0.201),
        None,
        None,
        None,
        Some(inch(1.0 / 20.0))
    ),
    hole!(
        Ansi,
        "ASME B1.1 / tap drill",
        "5/16-18 UNC",
        Tapped,
        Normal,
        Some("2B"),
        inch(0.257),
        None,
        None,
        None,
        Some(inch(1.0 / 18.0))
    ),
    hole!(
        Ansi,
        "ASME B18.3",
        "#10",
        Counterbore,
        Normal,
        None,
        inch(0.201),
        Some(inch(0.375)),
        Some(inch(0.200)),
        None,
        None
    ),
    hole!(
        Ansi,
        "ASME B18.3",
        "1/4",
        Counterbore,
        Normal,
        None,
        inch(0.266),
        Some(inch(0.438)),
        Some(inch(0.250)),
        None,
        None
    ),
    hole!(
        Ansi,
        "ASME B18.3",
        "5/16",
        Counterbore,
        Normal,
        None,
        inch(0.332),
        Some(inch(0.531)),
        Some(inch(0.312)),
        None,
        None
    ),
    hole!(
        Ansi,
        "ASME B18.6.3",
        "#10",
        Countersink,
        Normal,
        None,
        inch(0.201),
        Some(inch(0.385)),
        None,
        Some(82.0),
        None
    ),
    hole!(
        Ansi,
        "ASME B18.6.3",
        "1/4",
        Countersink,
        Normal,
        None,
        inch(0.266),
        Some(inch(0.477)),
        None,
        Some(82.0),
        None
    ),
    hole!(
        Ansi,
        "ASME B18.6.3",
        "5/16",
        Countersink,
        Normal,
        None,
        inch(0.332),
        Some(inch(0.597)),
        None,
        Some(82.0),
        None
    ),
    // Phase-6 pack v2: larger metric clearance and coarse-thread coverage.
    hole!(Iso, "ISO 273", "M14", Clearance, Close, None, 15.0, None, None, None, None),
    hole!(Iso, "ISO 273", "M16", Clearance, Close, None, 17.0, None, None, None, None),
    hole!(Iso, "ISO 273", "M18", Clearance, Close, None, 19.0, None, None, None, None),
    hole!(Iso, "ISO 273", "M20", Clearance, Close, None, 21.0, None, None, None, None),
    hole!(Iso, "ISO 273", "M22", Clearance, Close, None, 23.0, None, None, None, None),
    hole!(Iso, "ISO 273", "M24", Clearance, Close, None, 25.0, None, None, None, None),
    hole!(Iso, "ISO 273", "M14", Clearance, Normal, None, 15.5, None, None, None, None),
    hole!(Iso, "ISO 273", "M16", Clearance, Normal, None, 17.5, None, None, None, None),
    hole!(Iso, "ISO 273", "M18", Clearance, Normal, None, 20.0, None, None, None, None),
    hole!(Iso, "ISO 273", "M20", Clearance, Normal, None, 22.0, None, None, None, None),
    hole!(Iso, "ISO 273", "M22", Clearance, Normal, None, 24.0, None, None, None, None),
    hole!(Iso, "ISO 273", "M24", Clearance, Normal, None, 26.0, None, None, None, None),
    hole!(Iso, "ISO 273", "M14", Clearance, Loose, None, 17.0, None, None, None, None),
    hole!(Iso, "ISO 273", "M16", Clearance, Loose, None, 18.0, None, None, None, None),
    hole!(Iso, "ISO 273", "M18", Clearance, Loose, None, 21.0, None, None, None, None),
    hole!(Iso, "ISO 273", "M20", Clearance, Loose, None, 24.0, None, None, None, None),
    hole!(Iso, "ISO 273", "M22", Clearance, Loose, None, 26.0, None, None, None, None),
    hole!(Iso, "ISO 273", "M24", Clearance, Loose, None, 28.0, None, None, None, None),
    hole!(
        Iso,
        "ISO 261 / tap drill",
        "M14×2",
        Tapped,
        Normal,
        Some("6H"),
        12.0,
        None,
        None,
        None,
        Some(2.0)
    ),
    hole!(
        Iso,
        "ISO 261 / tap drill",
        "M16×2",
        Tapped,
        Normal,
        Some("6H"),
        14.0,
        None,
        None,
        None,
        Some(2.0)
    ),
    hole!(
        Iso,
        "ISO 261 / tap drill",
        "M18×2.5",
        Tapped,
        Normal,
        Some("6H"),
        15.5,
        None,
        None,
        None,
        Some(2.5)
    ),
    hole!(
        Iso,
        "ISO 261 / tap drill",
        "M20×2.5",
        Tapped,
        Normal,
        Some("6H"),
        17.5,
        None,
        None,
        None,
        Some(2.5)
    ),
    hole!(
        Iso,
        "ISO 261 / tap drill",
        "M22×2.5",
        Tapped,
        Normal,
        Some("6H"),
        19.5,
        None,
        None,
        None,
        Some(2.5)
    ),
    hole!(
        Iso,
        "ISO 261 / tap drill",
        "M24×3",
        Tapped,
        Normal,
        Some("6H"),
        21.0,
        None,
        None,
        None,
        Some(3.0)
    ),
    // Small and larger unified-inch sizes omitted from the v1 curated pack.
    hole!(
        Ansi,
        "ASME B18.2.8",
        "#2",
        Clearance,
        Normal,
        None,
        inch(0.089),
        None,
        None,
        None,
        None
    ),
    hole!(
        Ansi,
        "ASME B18.2.8",
        "#4",
        Clearance,
        Normal,
        None,
        inch(0.116),
        None,
        None,
        None,
        None
    ),
    hole!(
        Ansi,
        "ASME B18.2.8",
        "#6",
        Clearance,
        Normal,
        None,
        inch(0.144),
        None,
        None,
        None,
        None
    ),
    hole!(
        Ansi,
        "ASME B18.2.8",
        "#8",
        Clearance,
        Normal,
        None,
        inch(0.169),
        None,
        None,
        None,
        None
    ),
    hole!(
        Ansi,
        "ASME B18.2.8",
        "1/2",
        Clearance,
        Normal,
        None,
        inch(0.531),
        None,
        None,
        None,
        None
    ),
    hole!(
        Ansi,
        "ASME B1.1 / tap drill",
        "#2-56 UNC",
        Tapped,
        Normal,
        Some("2B"),
        inch(0.070),
        None,
        None,
        None,
        Some(inch(1.0 / 56.0))
    ),
    hole!(
        Ansi,
        "ASME B1.1 / tap drill",
        "#4-40 UNC",
        Tapped,
        Normal,
        Some("2B"),
        inch(0.089),
        None,
        None,
        None,
        Some(inch(1.0 / 40.0))
    ),
    hole!(
        Ansi,
        "ASME B1.1 / tap drill",
        "#6-32 UNC",
        Tapped,
        Normal,
        Some("2B"),
        inch(0.1065),
        None,
        None,
        None,
        Some(inch(1.0 / 32.0))
    ),
    hole!(
        Ansi,
        "ASME B1.1 / tap drill",
        "#8-32 UNC",
        Tapped,
        Normal,
        Some("2B"),
        inch(0.136),
        None,
        None,
        None,
        Some(inch(1.0 / 32.0))
    ),
    hole!(
        Ansi,
        "ASME B1.1 / tap drill",
        "3/8-16 UNC",
        Tapped,
        Normal,
        Some("2B"),
        inch(0.3125),
        None,
        None,
        None,
        Some(inch(1.0 / 16.0))
    ),
    hole!(
        Ansi,
        "ASME B1.1 / tap drill",
        "1/2-13 UNC",
        Tapped,
        Normal,
        Some("2B"),
        inch(0.4219),
        None,
        None,
        None,
        Some(inch(1.0 / 13.0))
    ),
];

pub fn hole_presets(
    family: StandardsFamily,
    application: HoleApplication,
    fit: HoleFit,
) -> impl Iterator<Item = &'static HoleStandardPreset> {
    HOLE_STANDARD_PRESETS.iter().filter(move |preset| {
        preset.family == family && preset.application == application && preset.fit == fit
    })
}

pub fn thread_reference(
    standard: super::thread::ThreadStandard,
    preset: &super::thread::ThreadPreset,
    internal: bool,
) -> Option<StandardReference> {
    let class = thread_classes(standard, internal).get(1).copied()?;
    thread_reference_with_class(standard, preset, internal, class)
}

/// Available tolerance classes for a modeled thread. Index 1 is the common
/// general-purpose default retained by [`thread_reference`].
pub fn thread_classes(
    standard: super::thread::ThreadStandard,
    internal: bool,
) -> &'static [&'static str] {
    use super::thread::ThreadStandard;
    match (standard, internal) {
        (ThreadStandard::MetricCoarse, true) => &["5H", "6H", "7H"],
        (ThreadStandard::MetricCoarse, false) => &["4g6g", "6g", "8g"],
        (ThreadStandard::UnifiedCoarse | ThreadStandard::UnifiedFine, true) => &["1B", "2B", "3B"],
        (ThreadStandard::UnifiedCoarse | ThreadStandard::UnifiedFine, false) => &["1A", "2A", "3A"],
        (ThreadStandard::Custom, _) => &[],
    }
}

pub fn thread_reference_with_class(
    standard: super::thread::ThreadStandard,
    preset: &super::thread::ThreadPreset,
    internal: bool,
    class: &str,
) -> Option<StandardReference> {
    use super::thread::ThreadStandard;
    if !thread_classes(standard, internal).contains(&class) {
        return None;
    }
    let (family, table) = match standard {
        ThreadStandard::MetricCoarse => (StandardsFamily::Iso, "ISO 261"),
        ThreadStandard::UnifiedCoarse => (StandardsFamily::Ansi, "ASME B1.1 UNC"),
        ThreadStandard::UnifiedFine => (StandardsFamily::Ansi, "ASME B1.1 UNF"),
        ThreadStandard::Custom => return None,
    };
    Some(StandardReference::new(
        family,
        table,
        preset.designation,
        Some(class),
        None,
        ResolvedStandardGeometry::Thread {
            major_diameter_mm: preset.major_dia_mm,
            pitch_mm: preset.pitch_mm,
            radial_depth_mm: super::thread::default_depth_mm(preset.pitch_mm),
            angle_deg: standard.angle_deg(),
        },
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn library_identity_and_resolved_values_are_frozen_into_references() {
        let preset = hole_presets(
            StandardsFamily::Iso,
            HoleApplication::Counterbore,
            HoleFit::Normal,
        )
        .find(|preset| preset.designation == "M6")
        .unwrap();
        let reference = preset.reference_with_resolved(6.8, Some(11.2), Some(6.5), None);
        assert_eq!(reference.library_id, STANDARDS_LIBRARY_ID);
        assert_eq!(reference.library_version, STANDARDS_LIBRARY_VERSION);
        assert!(matches!(
            reference.resolved,
            ResolvedStandardGeometry::Hole {
                bore_diameter_mm: 6.8,
                head_diameter_mm: Some(11.2),
                ..
            }
        ));
    }

    #[test]
    fn every_table_key_is_unique_within_its_selection_domain() {
        let mut keys = std::collections::HashSet::new();
        for preset in HOLE_STANDARD_PRESETS {
            assert!(keys.insert((
                preset.family,
                preset.table,
                preset.designation,
                preset.application,
                preset.fit,
            )));
        }
    }

    #[test]
    fn phase6_data_pack_is_append_only_and_covers_large_metric_and_inch_sizes() {
        assert_eq!(
            STANDARDS_DATA_PACKS
                .iter()
                .map(|pack| pack.version)
                .collect::<Vec<_>>(),
            vec![1, 2]
        );
        assert_eq!(
            STANDARDS_DATA_PACKS.last().unwrap().version,
            STANDARDS_LIBRARY_VERSION
        );
        for (family, application, designation) in [
            (StandardsFamily::Iso, HoleApplication::Clearance, "M24"),
            (StandardsFamily::Iso, HoleApplication::Tapped, "M24×3"),
            (StandardsFamily::Ansi, HoleApplication::Clearance, "#2"),
            (StandardsFamily::Ansi, HoleApplication::Tapped, "1/2-13 UNC"),
        ] {
            assert!(
                HOLE_STANDARD_PRESETS.iter().any(|preset| {
                    preset.family == family
                        && preset.application == application
                        && preset.designation == designation
                }),
                "missing {family:?} {designation} {application:?}"
            );
        }
    }
}
