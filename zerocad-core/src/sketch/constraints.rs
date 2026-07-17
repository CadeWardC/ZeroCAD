//! Sketch entity identity and the constraint-solver data model.
//!
//! [`EntityId`] is the shared prerequisite of two workstreams: the constraint
//! solver addresses points/entities/constraints by id, and persistent
//! topological naming keys sketch provenance by id instead of `shapes` Vec
//! position (a deleted shape must not re-key every reference downstream of its
//! neighbours). Ids are sketch-scoped, allocated monotonically, and **never
//! reused or renumbered** — deleting an entity leaves a hole in the sequence.
//!
//! [`SketchSolverModel`] is the solver's source of truth when present: shared
//! points (no duplicated coordinates), entities referencing them by id, and the
//! constraints between them. Legacy sketches carry `None` and keep the exact
//! `shapes`/`curves` path. Everything here is serde-visible on purpose: the
//! evaluator's prefix cache hashes the serialized feature, so any hidden field
//! would let a constraint edit reuse a stale mesh. Never `#[serde(skip)]`
//! anything in this module.

/// A durable, sketch-scoped identity for one sketch element (a shape in the
/// legacy `shapes` list, or a point/entity/constraint in the solver model).
#[derive(
    Debug,
    Clone,
    Copy,
    PartialEq,
    Eq,
    PartialOrd,
    Ord,
    Hash,
    Default,
    serde::Serialize,
    serde::Deserialize,
)]
pub struct EntityId(pub u32);

impl EntityId {
    /// The ids `0..n` in order — the legacy backfill for a sketch whose shapes
    /// predate entity ids. Position `i` gets id `i`, which makes the id-keyed
    /// provenance grammar string-identical to the old Vec-index grammar, so
    /// every reference captured before ids existed still resolves.
    pub fn sequence(n: usize) -> Vec<EntityId> {
        (0..n as u32).map(EntityId).collect()
    }
}

/// The ids for a sketch's `shapes` list, backfilling legacy documents. When
/// `entity_ids` parallels `shapes` it is authoritative; when it is missing or
/// desynced (a legacy or hand-edited document), position `i` gets id `i`,
/// reproducing the old Vec-index identity exactly.
pub fn effective_shape_ids(shape_count: usize, entity_ids: &[EntityId]) -> Vec<EntityId> {
    if entity_ids.len() == shape_count {
        entity_ids.to_vec()
    } else {
        EntityId::sequence(shape_count)
    }
}

/// One solver point: shared coordinates addressed by id. Entities reference
/// points by id, so coincidence is structural (one point, many users) rather
/// than recovered by tolerance snapping. Positions are `f64` — the solve runs
/// in double precision and bakes to `f32` curves at the end.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SketchPoint {
    pub id: EntityId,
    pub pos: (f64, f64),
}

/// Associative source record for construction geometry projected from a body
/// edge. `entity_ids` and `point_ids` identify the generated sketch elements;
/// they are locked by the solver and can be rebuilt in place when the named
/// source edge changes upstream.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ProjectedEdgeReference {
    pub source_body: String,
    pub source: crate::parametric::EdgeRef,
    #[serde(default)]
    pub entity_ids: Vec<EntityId>,
    #[serde(default)]
    pub point_ids: Vec<EntityId>,
}

/// One solver entity. `derived_from` records the legacy shape (its
/// [`EntityId`]) this entity was promoted from, so region provenance keeps
/// emitting the pre-promotion fragment ids and references captured before the
/// promotion still resolve.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum SketchEntity {
    Line {
        id: EntityId,
        p0: EntityId,
        p1: EntityId,
        #[serde(default)]
        derived_from: Option<EntityId>,
    },
    Circle {
        id: EntityId,
        center: EntityId,
        radius: f64,
        #[serde(default)]
        derived_from: Option<EntityId>,
    },
    Arc {
        id: EntityId,
        center: EntityId,
        start: EntityId,
        end: EntityId,
        radius: f64,
        #[serde(default)]
        derived_from: Option<EntityId>,
    },
    /// An analytic ellipse or elliptical arc. `major_axis` and `minor_axis`
    /// are the two world-in-sketch-plane vectors of the conic; the parameter
    /// interval evaluates `center + major*cos(t) + minor*sin(t)`. Keeping the
    /// vectors instead of a faceted outline makes projected circular edges
    /// associative and exact even when viewed obliquely.
    Ellipse {
        id: EntityId,
        center: EntityId,
        major_axis: [f64; 2],
        minor_axis: [f64; 2],
        start_parameter: f64,
        end_parameter: f64,
        closed: bool,
        #[serde(default)]
        derived_from: Option<EntityId>,
    },
    Spline {
        id: EntityId,
        points: Vec<EntityId>,
        kind: crate::sketch::SplineKind,
        degree: u8,
        knots: Vec<f32>,
        weights: Vec<f32>,
        closed: bool,
        periodic: bool,
        continuity: crate::sketch::SplineContinuity,
        trim: Option<(f32, f32)>,
        #[serde(default)]
        derived_from: Option<EntityId>,
    },
}

impl SketchEntity {
    pub fn id(&self) -> EntityId {
        match self {
            SketchEntity::Line { id, .. }
            | SketchEntity::Circle { id, .. }
            | SketchEntity::Arc { id, .. }
            | SketchEntity::Ellipse { id, .. }
            | SketchEntity::Spline { id, .. } => *id,
        }
    }

    pub fn derived_from(&self) -> Option<EntityId> {
        match self {
            SketchEntity::Line { derived_from, .. }
            | SketchEntity::Circle { derived_from, .. }
            | SketchEntity::Arc { derived_from, .. }
            | SketchEntity::Ellipse { derived_from, .. }
            | SketchEntity::Spline { derived_from, .. } => *derived_from,
        }
    }
}

/// A geometric constraint between solver entities. Dimensional constraints
/// (`Distance`, `Radius`) reuse [`crate::sketch::Dimension`], so a constraint
/// value can be a literal or a variable-bound expression re-resolved against
/// the document's variables every build — exactly like a rectangle's `w`/`h`
/// today. The enum is append-only: `.zcad` forward compatibility relies on
/// never reordering or removing variants.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Constraint {
    /// Two points share a location (their residual drives them together; they
    /// remain distinct points so the constraint can be deleted).
    Coincident {
        id: EntityId,
        a: EntityId,
        b: EntityId,
    },
    /// A line's endpoints share a v-coordinate.
    Horizontal { id: EntityId, line: EntityId },
    /// A line's endpoints share a u-coordinate.
    Vertical { id: EntityId, line: EntityId },
    /// Driving distance between two points.
    Distance {
        id: EntityId,
        a: EntityId,
        b: EntityId,
        d: crate::sketch::Dimension,
    },
    /// Driving radius of a circle or arc.
    Radius {
        id: EntityId,
        circle: EntityId,
        r: crate::sketch::Dimension,
    },
    /// Two lines with parallel directions.
    Parallel {
        id: EntityId,
        a: EntityId,
        b: EntityId,
    },
    /// Two lines with perpendicular directions.
    Perpendicular {
        id: EntityId,
        a: EntityId,
        b: EntityId,
    },
    /// A line tangent to a circle/arc.
    Tangent {
        id: EntityId,
        line: EntityId,
        circle: EntityId,
    },
    /// Equal lengths (two lines) or equal radii (two circles/arcs).
    Equal {
        id: EntityId,
        a: EntityId,
        b: EntityId,
    },
    /// A point anchored at its current position (kills rigid-body freedom).
    Fixed { id: EntityId, p: EntityId },
    /// Signed horizontal separation between two points.
    DistanceX {
        id: EntityId,
        a: EntityId,
        b: EntityId,
        d: crate::sketch::Dimension,
    },
    /// Signed vertical separation between two points.
    DistanceY {
        id: EntityId,
        a: EntityId,
        b: EntityId,
        d: crate::sketch::Dimension,
    },
    /// Included angle between two lines in degrees.
    Angle {
        id: EntityId,
        a: EntityId,
        b: EntityId,
        angle_deg: crate::sketch::Dimension,
    },
    /// Two circles/arcs share a center.
    Concentric {
        id: EntityId,
        a: EntityId,
        b: EntityId,
    },
    /// A point lies at a line's midpoint.
    Midpoint {
        id: EntityId,
        point: EntityId,
        line: EntityId,
    },
    /// A point lies on a line or circle/arc.
    PointOnObject {
        id: EntityId,
        point: EntityId,
        object: EntityId,
    },
    /// Two lines lie on the same infinite line.
    Collinear {
        id: EntityId,
        a: EntityId,
        b: EntityId,
    },
    /// Two points are mirror-symmetric about a line.
    Symmetric {
        id: EntityId,
        a: EntityId,
        b: EntityId,
        axis: EntityId,
    },
    /// Driving circle/arc diameter.
    Diameter {
        id: EntityId,
        circle: EntityId,
        d: crate::sketch::Dimension,
    },
}

impl Constraint {
    pub fn id(&self) -> EntityId {
        match self {
            Constraint::Coincident { id, .. }
            | Constraint::Horizontal { id, .. }
            | Constraint::Vertical { id, .. }
            | Constraint::Distance { id, .. }
            | Constraint::Radius { id, .. }
            | Constraint::Parallel { id, .. }
            | Constraint::Perpendicular { id, .. }
            | Constraint::Tangent { id, .. }
            | Constraint::Equal { id, .. }
            | Constraint::Fixed { id, .. }
            | Constraint::DistanceX { id, .. }
            | Constraint::DistanceY { id, .. }
            | Constraint::Angle { id, .. }
            | Constraint::Concentric { id, .. }
            | Constraint::Midpoint { id, .. }
            | Constraint::PointOnObject { id, .. }
            | Constraint::Collinear { id, .. }
            | Constraint::Symmetric { id, .. }
            | Constraint::Diameter { id, .. } => *id,
        }
    }
}

/// The constraint solver's model of a sketch. When present on a sketch feature
/// it is the source of truth for geometry: the solver solves the constrained
/// point positions and bakes them into the same `SketchCurves` the rest of the
/// pipeline already consumes. Stored positions are simultaneously the next
/// solve's warm start and the last-valid bake when a solve fails, so a bad
/// constraint edit degrades the sketch, never blanks it.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SketchSolverModel {
    #[serde(default)]
    pub points: Vec<SketchPoint>,
    #[serde(default)]
    pub entities: Vec<SketchEntity>,
    #[serde(default)]
    pub constraints: Vec<Constraint>,
    /// Entity ids used as reference/construction geometry. They remain in the
    /// solver and render in the sketch, but do not participate in region
    /// detection or Part Design profiles.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub construction: Vec<EntityId>,
    /// Dimensional constraints that report the live geometric value without
    /// contributing a solver equation. The durable constraint id survives a
    /// driving/reference toggle.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub driven_dimensions: Vec<EntityId>,
    /// Body-edge projections are construction geometry, but unlike ordinary
    /// construction entities they retain a stable source and refresh after an
    /// upstream rebuild.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub projected_edges: Vec<ProjectedEdgeReference>,
}

impl SketchSolverModel {
    pub fn is_empty(&self) -> bool {
        self.points.is_empty() && self.entities.is_empty() && self.constraints.is_empty()
    }

    pub fn point(&self, id: EntityId) -> Option<&SketchPoint> {
        self.points.iter().find(|p| p.id == id)
    }

    pub fn is_driven_dimension(&self, id: EntityId) -> bool {
        self.driven_dimensions.contains(&id)
    }

    pub fn is_projected_point(&self, id: EntityId) -> bool {
        self.projected_edges
            .iter()
            .any(|projection| projection.point_ids.contains(&id))
    }

    pub fn is_projected_entity(&self, id: EntityId) -> bool {
        self.projected_edges
            .iter()
            .any(|projection| projection.entity_ids.contains(&id))
    }
}

/// Allocates monotonically increasing [`EntityId`]s for one sketch. Ids are
/// never reused: the counter only moves forward, so a deleted entity's id
/// stays retired and every surviving reference stays valid.
pub struct IdAllocator {
    next: u32,
}

impl IdAllocator {
    pub fn new(next: u32) -> Self {
        Self { next }
    }

    pub fn alloc(&mut self) -> EntityId {
        let id = EntityId(self.next);
        self.next += 1;
        id
    }

    pub fn next_value(&self) -> u32 {
        self.next
    }
}

/// Bake a solver model into the [`crate::sketch::SketchCurves`] the rest of
/// the pipeline consumes. Entities emit **in stored order** (creation order,
/// monotone in id for promoted models) — region detection and the
/// `region:{i}` name grammar are order-sensitive, so the bake must never
/// iterate a hash map or re-sort.
pub fn bake_entities_to_curves(model: &SketchSolverModel) -> crate::sketch::SketchCurves {
    let mut curves = crate::sketch::SketchCurves::new();
    let pos = |id: EntityId| -> Option<(f32, f32)> {
        model.point(id).map(|p| (p.pos.0 as f32, p.pos.1 as f32))
    };
    for entity in &model.entities {
        if model.construction.contains(&entity.id()) {
            continue;
        }
        match entity {
            SketchEntity::Line { p0, p1, .. } => {
                if let (Some(a), Some(b)) = (pos(*p0), pos(*p1)) {
                    curves.add_line(a, b);
                }
            }
            SketchEntity::Circle { center, radius, .. } => {
                if let Some(c) = pos(*center) {
                    curves.add_circle(c, *radius as f32);
                }
            }
            SketchEntity::Arc {
                center,
                start,
                end,
                radius,
                ..
            } => {
                if let (Some(c), Some(s), Some(e)) = (pos(*center), pos(*start), pos(*end)) {
                    curves.arcs.push(crate::sketch::Arc {
                        center: c,
                        radius: *radius as f32,
                        start: s,
                        end: e,
                    });
                }
            }
            SketchEntity::Ellipse {
                center,
                major_axis,
                minor_axis,
                start_parameter,
                end_parameter,
                closed,
                ..
            } => {
                if let Some(center) = pos(*center) {
                    let segments = if *closed { 64 } else { 32 };
                    let sweep = if *closed {
                        std::f64::consts::TAU
                    } else {
                        end_parameter - start_parameter
                    };
                    let point = |parameter: f64| {
                        let (sin, cos) = parameter.sin_cos();
                        (
                            center.0
                                + (major_axis[0] * cos + minor_axis[0] * sin) as f32,
                            center.1
                                + (major_axis[1] * cos + minor_axis[1] * sin) as f32,
                        )
                    };
                    for index in 0..segments {
                        let a = *start_parameter + sweep * index as f64 / segments as f64;
                        let b = *start_parameter + sweep * (index + 1) as f64 / segments as f64;
                        curves.add_line(point(a), point(b));
                    }
                }
            }
            SketchEntity::Spline {
                points,
                kind,
                degree,
                knots,
                weights,
                closed,
                periodic,
                continuity,
                trim,
                ..
            } => {
                let positions: Option<Vec<(f32, f32)>> = points.iter().map(|id| pos(*id)).collect();
                if let Some(points) = positions {
                    curves.add_spline(crate::sketch::Spline {
                        kind: *kind,
                        points,
                        degree: *degree,
                        knots: knots.clone(),
                        weights: weights.clone(),
                        closed: *closed,
                        periodic: *periodic,
                        continuity: *continuity,
                        trim: *trim,
                    });
                }
            }
        }
    }
    curves
}

/// Bake only reference/construction entities for dedicated sketch rendering.
pub fn bake_construction_curves(model: &SketchSolverModel) -> crate::sketch::SketchCurves {
    let mut construction = model.clone();
    construction
        .entities
        .retain(|entity| model.construction.contains(&entity.id()));
    construction.construction.clear();
    bake_entities_to_curves(&construction)
}

/// Promote a legacy `shapes` sketch into the solver's entity model — the
/// explicit, one-way migration run when the user first applies a constraint
/// tool to an old sketch (never on load).
///
/// Guarantees:
/// - **Geometry-lossless**: point positions are read from each shape's own
///   `build()` output, so a promoted-then-baked sketch is `f32`-identical to
///   the legacy path.
/// - **Provenance-preserving**: every entity records `derived_from` = the
///   source shape's durable id, so region provenance keeps emitting the
///   pre-promotion fragment ids and captured references still resolve.
/// - Rectangle/Circle keep their driving dimensions as constraints (a
///   variable-bound `w` stays variable-bound); a `Line`'s length becomes a
///   `Distance`. A `Line`'s `angle_deg` binding and `Raw` geometry promote as
///   free (unconstrained) geometry.
///
/// Returns the model plus the advanced id counter.
pub fn promote_shapes_to_entities(
    shapes: &[crate::sketch::SketchShape],
    entity_ids: &[EntityId],
    vars: &std::collections::HashMap<String, f64>,
    next_entity_id: u32,
) -> (SketchSolverModel, u32) {
    use crate::sketch::SketchShape;

    let shape_ids = effective_shape_ids(shapes.len(), entity_ids);
    // Never allocate below the ids already in use (legacy sketches store 0).
    let floor = shape_ids.iter().map(|id| id.0 + 1).max().unwrap_or(0);
    let mut ids = IdAllocator::new(next_entity_id.max(floor));
    let mut model = SketchSolverModel::default();

    for (shape, &owner) in shapes.iter().zip(&shape_ids) {
        let built = shape.build(vars);
        match shape {
            SketchShape::Rectangle { w, h, .. } => {
                if built.segments.len() != 4 {
                    promote_raw(&built, owner, &mut ids, &mut model);
                    continue;
                }
                // `add_rectangle` emits p0→p1→p2→p3→p0; share the corner
                // points so adjacency is structural, not snapped.
                let corners = [
                    built.segments[0].a,
                    built.segments[1].a,
                    built.segments[2].a,
                    built.segments[3].a,
                ];
                let pts: Vec<EntityId> = corners
                    .iter()
                    .map(|c| {
                        let id = ids.alloc();
                        model.points.push(SketchPoint {
                            id,
                            pos: (c.0 as f64, c.1 as f64),
                        });
                        id
                    })
                    .collect();
                let mut lines = Vec::with_capacity(4);
                for k in 0..4 {
                    let id = ids.alloc();
                    model.entities.push(SketchEntity::Line {
                        id,
                        p0: pts[k],
                        p1: pts[(k + 1) % 4],
                        derived_from: Some(owner),
                    });
                    lines.push(id);
                }
                // Bottom/top horizontal, right/left vertical, w/h driving dims.
                for (line, horizontal) in [(0, true), (1, false), (2, true), (3, false)] {
                    let id = ids.alloc();
                    model.constraints.push(if horizontal {
                        Constraint::Horizontal {
                            id,
                            line: lines[line],
                        }
                    } else {
                        Constraint::Vertical {
                            id,
                            line: lines[line],
                        }
                    });
                }
                let w_id = ids.alloc();
                model.constraints.push(Constraint::Distance {
                    id: w_id,
                    a: pts[0],
                    b: pts[1],
                    d: w.clone(),
                });
                let h_id = ids.alloc();
                model.constraints.push(Constraint::Distance {
                    id: h_id,
                    a: pts[1],
                    b: pts[2],
                    d: h.clone(),
                });
                // Anchor the origin corner: the legacy shape grows from its
                // origin (w extends +x, h extends +y), and an unanchored solve
                // would instead split a dimension change symmetrically between
                // both sides (the minimum-motion solution). The user can delete
                // the anchor to float the rectangle.
                let anchor = ids.alloc();
                model.constraints.push(Constraint::Fixed {
                    id: anchor,
                    p: pts[0],
                });
            }
            SketchShape::Circle { diameter, .. } => {
                let Some(circle) = built.circles.first() else {
                    continue;
                };
                let center = ids.alloc();
                model.points.push(SketchPoint {
                    id: center,
                    pos: (circle.center.0 as f64, circle.center.1 as f64),
                });
                let entity = ids.alloc();
                model.entities.push(SketchEntity::Circle {
                    id: entity,
                    center,
                    radius: circle.radius as f64,
                    derived_from: Some(owner),
                });
                let rc = ids.alloc();
                model.constraints.push(Constraint::Radius {
                    id: rc,
                    circle: entity,
                    r: crate::sketch::Dimension {
                        value: circle.radius,
                        expr: diameter.expr.as_ref().map(|e| format!("({e})/2")),
                    },
                });
            }
            SketchShape::Line { length, .. } => {
                let Some(seg) = built.segments.first() else {
                    continue;
                };
                let p0 = ids.alloc();
                model.points.push(SketchPoint {
                    id: p0,
                    pos: (seg.a.0 as f64, seg.a.1 as f64),
                });
                let p1 = ids.alloc();
                model.points.push(SketchPoint {
                    id: p1,
                    pos: (seg.b.0 as f64, seg.b.1 as f64),
                });
                let line = ids.alloc();
                model.entities.push(SketchEntity::Line {
                    id: line,
                    p0,
                    p1,
                    derived_from: Some(owner),
                });
                let dc = ids.alloc();
                model.constraints.push(Constraint::Distance {
                    id: dc,
                    a: p0,
                    b: p1,
                    d: length.clone(),
                });
            }
            SketchShape::RegularPolygon { .. }
            | SketchShape::Spline { .. }
            | SketchShape::Imported { .. }
            | SketchShape::Raw { .. } => promote_raw(&built, owner, &mut ids, &mut model),
        }
    }
    let next = ids.next_value();
    (model, next)
}

/// Promote pre-built curves as free geometry: exact-coordinate-shared points,
/// no constraints. Used for `Raw` shapes and any shape whose build output
/// doesn't match its expected form.
fn promote_raw(
    built: &crate::sketch::SketchCurves,
    owner: EntityId,
    ids: &mut IdAllocator,
    model: &mut SketchSolverModel,
) {
    fn point_at(ids: &mut IdAllocator, model: &mut SketchSolverModel, p: (f32, f32)) -> EntityId {
        let pos = (p.0 as f64, p.1 as f64);
        if let Some(existing) = model
            .points
            .iter()
            .find(|q| q.pos.0 == pos.0 && q.pos.1 == pos.1)
        {
            return existing.id;
        }
        let id = ids.alloc();
        model.points.push(SketchPoint { id, pos });
        id
    }
    for seg in &built.segments {
        let p0 = point_at(ids, model, seg.a);
        let p1 = point_at(ids, model, seg.b);
        let id = ids.alloc();
        model.entities.push(SketchEntity::Line {
            id,
            p0,
            p1,
            derived_from: Some(owner),
        });
    }
    for circle in &built.circles {
        let center = point_at(ids, model, circle.center);
        let id = ids.alloc();
        model.entities.push(SketchEntity::Circle {
            id,
            center,
            radius: circle.radius as f64,
            derived_from: Some(owner),
        });
    }
    for arc in &built.arcs {
        let center = point_at(ids, model, arc.center);
        let start = point_at(ids, model, arc.start);
        let end = point_at(ids, model, arc.end);
        let id = ids.alloc();
        model.entities.push(SketchEntity::Arc {
            id,
            center,
            start,
            end,
            radius: arc.radius as f64,
            derived_from: Some(owner),
        });
    }
    for spline in &built.splines {
        let points = spline
            .points
            .iter()
            .map(|point| point_at(ids, model, *point))
            .collect();
        let id = ids.alloc();
        model.entities.push(SketchEntity::Spline {
            id,
            points,
            kind: spline.kind,
            degree: spline.degree,
            knots: spline.knots.clone(),
            weights: spline.weights.clone(),
            closed: spline.closed,
            periodic: spline.periodic,
            continuity: spline.continuity,
            trim: spline.trim,
            derived_from: Some(owner),
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sketch::Dimension;

    #[test]
    fn effective_shape_ids_backfills_legacy_by_position() {
        assert_eq!(
            effective_shape_ids(3, &[]),
            vec![EntityId(0), EntityId(1), EntityId(2)]
        );
    }

    #[test]
    fn effective_shape_ids_uses_stored_ids_when_synced() {
        let stored = vec![EntityId(0), EntityId(4)];
        assert_eq!(effective_shape_ids(2, &stored), stored);
    }

    #[test]
    fn effective_shape_ids_falls_back_when_desynced() {
        // A hand-edited or partially-migrated document whose ids don't parallel
        // shapes gets positional identity — the pre-ids behavior — rather than
        // a panic or misalignment.
        assert_eq!(
            effective_shape_ids(2, &[EntityId(7)]),
            vec![EntityId(0), EntityId(1)]
        );
    }

    #[test]
    fn solver_model_serde_round_trips_every_variant() {
        let model = SketchSolverModel {
            points: vec![
                SketchPoint {
                    id: EntityId(0),
                    pos: (0.0, 0.0),
                },
                SketchPoint {
                    id: EntityId(1),
                    pos: (10.0, 0.25),
                },
            ],
            entities: vec![
                SketchEntity::Line {
                    id: EntityId(2),
                    p0: EntityId(0),
                    p1: EntityId(1),
                    derived_from: Some(EntityId(0)),
                },
                SketchEntity::Circle {
                    id: EntityId(3),
                    center: EntityId(0),
                    radius: 4.0,
                    derived_from: None,
                },
                SketchEntity::Arc {
                    id: EntityId(4),
                    center: EntityId(0),
                    start: EntityId(1),
                    end: EntityId(1),
                    radius: 4.0,
                    derived_from: None,
                },
                SketchEntity::Spline {
                    id: EntityId(15),
                    points: vec![EntityId(0), EntityId(1)],
                    kind: crate::sketch::SplineKind::ControlPoint,
                    degree: 1,
                    knots: vec![0.0, 0.0, 1.0, 1.0],
                    weights: vec![1.0, 1.0],
                    closed: false,
                    periodic: false,
                    continuity: crate::sketch::SplineContinuity::Tangent,
                    trim: Some((0.1, 0.9)),
                    derived_from: None,
                },
            ],
            constraints: vec![
                Constraint::Coincident {
                    id: EntityId(5),
                    a: EntityId(0),
                    b: EntityId(1),
                },
                Constraint::Horizontal {
                    id: EntityId(6),
                    line: EntityId(2),
                },
                Constraint::Vertical {
                    id: EntityId(7),
                    line: EntityId(2),
                },
                Constraint::Distance {
                    id: EntityId(8),
                    a: EntityId(0),
                    b: EntityId(1),
                    d: Dimension {
                        value: 10.0,
                        expr: Some("w".to_string()),
                    },
                },
                Constraint::Radius {
                    id: EntityId(9),
                    circle: EntityId(3),
                    r: Dimension::literal(4.0),
                },
                Constraint::Parallel {
                    id: EntityId(10),
                    a: EntityId(2),
                    b: EntityId(2),
                },
                Constraint::Perpendicular {
                    id: EntityId(11),
                    a: EntityId(2),
                    b: EntityId(2),
                },
                Constraint::Tangent {
                    id: EntityId(12),
                    line: EntityId(2),
                    circle: EntityId(3),
                },
                Constraint::Equal {
                    id: EntityId(13),
                    a: EntityId(2),
                    b: EntityId(2),
                },
                Constraint::Fixed {
                    id: EntityId(14),
                    p: EntityId(0),
                },
                Constraint::DistanceX {
                    id: EntityId(16),
                    a: EntityId(0),
                    b: EntityId(1),
                    d: Dimension::literal(10.0),
                },
                Constraint::DistanceY {
                    id: EntityId(17),
                    a: EntityId(0),
                    b: EntityId(1),
                    d: Dimension::literal(0.25),
                },
                Constraint::Angle {
                    id: EntityId(18),
                    a: EntityId(2),
                    b: EntityId(2),
                    angle_deg: Dimension::literal(0.0),
                },
                Constraint::Concentric {
                    id: EntityId(19),
                    a: EntityId(3),
                    b: EntityId(4),
                },
                Constraint::Midpoint {
                    id: EntityId(20),
                    point: EntityId(0),
                    line: EntityId(2),
                },
                Constraint::PointOnObject {
                    id: EntityId(21),
                    point: EntityId(1),
                    object: EntityId(2),
                },
                Constraint::Collinear {
                    id: EntityId(22),
                    a: EntityId(2),
                    b: EntityId(2),
                },
                Constraint::Symmetric {
                    id: EntityId(23),
                    a: EntityId(0),
                    b: EntityId(1),
                    axis: EntityId(2),
                },
                Constraint::Diameter {
                    id: EntityId(24),
                    circle: EntityId(3),
                    d: Dimension::literal(8.0),
                },
            ],
            construction: vec![EntityId(2)],
            driven_dimensions: vec![EntityId(24)],
            projected_edges: vec![ProjectedEdgeReference {
                source_body: "box_1".to_string(),
                source: crate::parametric::EdgeRef {
                    p0: [0.0, 0.0, 0.0],
                    p1: [10.0, 0.0, 0.0],
                    n1: [0.0, 1.0, 0.0],
                    n2: [0.0, 0.0, 1.0],
                    curve: Some(crate::mock_kernel::EdgeCurveHint::Line),
                    topology: None,
                },
                entity_ids: vec![EntityId(2)],
                point_ids: vec![EntityId(0), EntityId(1)],
            }],
        };
        let json = serde_json::to_string(&model).expect("serialize");
        let back: SketchSolverModel = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(back, model);

        // CBOR is the `.zcad` wire format — round-trip that too.
        let mut cbor = Vec::new();
        ciborium::into_writer(&model, &mut cbor).expect("cbor serialize");
        let back: SketchSolverModel =
            ciborium::from_reader(cbor.as_slice()).expect("cbor deserialize");
        assert_eq!(back, model);
    }

    #[test]
    fn promotion_bakes_f32_identical_curves() {
        use crate::sketch::{effective_curves, SketchCurves, SketchShape};
        let mut vars = std::collections::HashMap::new();
        vars.insert("w".to_string(), 20.0);
        let shapes = vec![
            SketchShape::Rectangle {
                origin: (0.0, 0.0),
                sx: 1.0,
                sy: 1.0,
                w: Dimension {
                    value: 20.0,
                    expr: Some("w".to_string()),
                },
                h: Dimension::literal(12.0),
                from_center: false,
            },
            SketchShape::Circle {
                center: (32.0, 6.0),
                diameter: Dimension::literal(6.0),
            },
        ];
        let ids = EntityId::sequence(shapes.len());

        let legacy = effective_curves(&SketchCurves::new(), &shapes, &[], &vars);
        let (model, next) = promote_shapes_to_entities(&shapes, &ids, &vars, 2);
        let baked = bake_entities_to_curves(&model);

        assert_eq!(baked, legacy, "promotion must bake byte-identical geometry");
        // Rectangle corners are SHARED points (4, not 8) + the circle center.
        assert_eq!(model.points.len(), 5);
        assert_eq!(model.entities.len(), 5);
        // 2×Horizontal + 2×Vertical + w/h Distance + origin anchor + circle Radius.
        assert_eq!(model.constraints.len(), 8);
        // Driving dims carried through, still variable-bound.
        assert!(model.constraints.iter().any(|c| matches!(
            c,
            Constraint::Distance { d, .. } if d.expr.as_deref() == Some("w")
        )));
        // Every entity remembers its source shape for provenance.
        assert!(model.entities.iter().all(|e| e.derived_from().is_some()));
        // Counter advanced past everything allocated, monotonic.
        assert!(next as usize >= model.points.len() + model.entities.len());
    }

    #[test]
    fn promotion_never_reuses_existing_shape_ids() {
        use crate::sketch::SketchShape;
        let vars = std::collections::HashMap::new();
        let shapes = vec![SketchShape::Circle {
            center: (0.0, 0.0),
            diameter: Dimension::literal(6.0),
        }];
        // Legacy sketch: stored next_entity_id is 0, but the shape id 0 is in
        // use via positional backfill — allocation must start above it.
        let (model, _) = promote_shapes_to_entities(&shapes, &[], &vars, 0);
        assert!(
            model.points.iter().all(|p| p.id.0 >= 1)
                && model.entities.iter().all(|e| e.id().0 >= 1),
            "promoted ids must not collide with backfilled shape ids"
        );
    }

    #[test]
    fn legacy_sketch_solver_field_defaults_to_none() {
        // A pre-solver serialized Sketch feature has no `solver` field; serde's
        // default must produce `None`, keeping the legacy path byte-identical.
        #[derive(serde::Deserialize)]
        struct Probe {
            #[serde(default)]
            solver: Option<SketchSolverModel>,
            #[serde(default)]
            entity_ids: Vec<EntityId>,
            #[serde(default)]
            next_entity_id: u32,
        }
        let probe: Probe = serde_json::from_str("{}").expect("deserialize empty");
        assert!(probe.solver.is_none());
        assert!(probe.entity_ids.is_empty());
        assert_eq!(probe.next_entity_id, 0);
    }

    #[test]
    fn construction_entities_are_editable_but_do_not_form_profiles() {
        let mut model = SketchSolverModel::default();
        model.points.extend([
            SketchPoint {
                id: EntityId(0),
                pos: (0.0, 0.0),
            },
            SketchPoint {
                id: EntityId(1),
                pos: (10.0, 0.0),
            },
        ]);
        model.entities.push(SketchEntity::Line {
            id: EntityId(2),
            p0: EntityId(0),
            p1: EntityId(1),
            derived_from: None,
        });
        model.construction.push(EntityId(2));
        model.constraints.push(Constraint::Horizontal {
            id: EntityId(3),
            line: EntityId(2),
        });

        assert!(bake_entities_to_curves(&model).is_empty());
        let reference = bake_construction_curves(&model);
        assert_eq!(reference.segments.len(), 1);
        assert_eq!(reference.segments[0].a, (0.0, 0.0));
        assert_eq!(reference.segments[0].b, (10.0, 0.0));
    }
}
