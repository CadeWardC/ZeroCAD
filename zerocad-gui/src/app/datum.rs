//! Datum GUI support: resolving the graph's datum features for display and
//! hit-testing, and creating new datum nodes from the toolbar.

use crate::*;
use zerocad_core::{DatumAxisDef, DatumPlaneDef, DatumPointDef, DatumValue, PlaneBase};

/// Half-extent of a datum plane's displayed sheet (world units). Centered on
/// the datum's origin, unlike the origin sheets' one-quadrant style, so an
/// offset plane reads as a plane wherever its base geometry sits.
const DATUM_SHEET_HALF: f32 = 12.0;

impl ZeroCadApp {
    /// Every visible datum PLANE, resolved: `(node_id, display_name, frame)`.
    /// Sorted by id for stable draw order.
    pub(crate) fn resolved_datum_planes(&self) -> Vec<(String, String, CoordinateSystem)> {
        let mut out: Vec<(String, String, CoordinateSystem)> = self
            .resolved_datums()
            .into_iter()
            .filter_map(|(id, name, v)| match v {
                DatumValue::Plane(cs) => Some((id, name, cs)),
                _ => None,
            })
            .collect();
        out.sort_by(|a, b| a.0.cmp(&b.0));
        out
    }

    /// Every visible datum AXIS or POINT, resolved.
    pub(crate) fn resolved_datum_axes_points(&self) -> Vec<(String, String, DatumValue)> {
        self.resolved_datums()
            .into_iter()
            .filter(|(_, _, v)| !matches!(v, DatumValue::Plane(_)))
            .collect()
    }

    fn resolved_datums(&self) -> Vec<(String, String, DatumValue)> {
        let vars = self.document.variable_map();
        let mut warnings = Vec::new();
        let datums = self.document.resolve_datums(&vars, &mut warnings);
        datums
            .into_iter()
            .filter(|(id, _)| !self.hidden_nodes.contains(id))
            .map(|(id, v)| {
                let name = self
                    .document
                    .graph
                    .node_indices()
                    .find(|&i| self.document.graph[i].id == id)
                    .map(|i| self.document.graph[i].name.clone())
                    .unwrap_or_else(|| id.clone());
                (id, name, v)
            })
            .collect()
    }

    /// World corners of a datum plane's display sheet, CCW in the plane's own
    /// (u, v), centered on its origin. Shared by the renderer and the viewport
    /// hover test so what you see is exactly what you can click.
    pub(crate) fn datum_plane_world_corners(cs: &CoordinateSystem) -> [[f32; 3]; 4] {
        let h = DATUM_SHEET_HALF;
        let corner = |u: f32, v: f32| {
            let p = cs.unproject(u, v);
            [p.x, p.y, p.z]
        };
        [corner(-h, -h), corner(h, -h), corner(h, h), corner(-h, h)]
    }

    /// Create a datum feature with sensible defaults (undoable) and select it
    /// so the Properties panel is ready for tuning the numbers.
    pub(crate) fn create_datum(&mut self, kind: DatumKind) {
        self.push_undo();
        let (id, name, feature) = match kind {
            DatumKind::OffsetPlane(base) => {
                let id = format!("datum_{}", self.next_id());
                let base_label = match base {
                    PlaneBase::XY => "XY",
                    PlaneBase::XZ => "XZ",
                    PlaneBase::YZ => "YZ",
                    PlaneBase::Datum(_) => "datum",
                };
                (
                    id,
                    format!("Offset Plane ({base_label})"),
                    FeatureType::DatumPlane {
                        def: DatumPlaneDef::Offset {
                            base,
                            distance: 10.0,
                            distance_expr: None,
                        },
                    },
                )
            }
            DatumKind::AnglePlane => (
                format!("datum_{}", self.next_id()),
                "Angle Plane".to_string(),
                FeatureType::DatumPlane {
                    def: DatumPlaneDef::Angle {
                        base: PlaneBase::XY,
                        axis: zerocad_core::AxisBase::X,
                        angle_deg: 45.0,
                        angle_expr: None,
                    },
                },
            ),
            DatumKind::ThreePointPlane => (
                format!("datum_{}", self.next_id()),
                "3-Point Plane".to_string(),
                FeatureType::DatumPlane {
                    def: DatumPlaneDef::ThreePoints {
                        a: [0.0, 0.0, 0.0],
                        b: [10.0, 0.0, 0.0],
                        c: [0.0, 10.0, 0.0],
                    },
                },
            ),
            DatumKind::Axis => (
                format!("datumaxis_{}", self.next_id()),
                "Axis".to_string(),
                FeatureType::DatumAxis {
                    def: DatumAxisDef::TwoPoints {
                        a: [0.0, 0.0, 0.0],
                        b: [0.0, 10.0, 0.0],
                    },
                },
            ),
            DatumKind::Point => (
                format!("datumpt_{}", self.next_id()),
                "Point".to_string(),
                FeatureType::DatumPoint {
                    def: DatumPointDef::Coords { p: [0.0, 0.0, 0.0] },
                },
            ),
        };
        self.document.add_feature(FeatureNode {
            id: id.clone(),
            name: name.clone(),
            feature,
        });
        self.selected_node_id = Some(id);
        self.status_msg = format!("{name} created — tune it in the Properties panel.");
    }
}
