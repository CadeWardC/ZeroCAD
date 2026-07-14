//! Operation-independent topology lineage.
//!
//! Arena IDs are local to one B-Rep and are re-keyed by sewing and healing, so
//! public history uses deterministic entity positions at an operation boundary.
//! A [`TopologyRef`] identifies the entity kind and its position in that
//! boundary's canonical enumeration; [`InputTopologyRef`] additionally names
//! the operand. This keeps history serializable and meaningful after an arena is
//! rebuilt.

use serde::{Deserialize, Serialize};

use crate::Solid;

/// Kind of topological entity participating in operation history.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TopologyKind {
    Vertex,
    Edge,
    Wire,
    Face,
    Shell,
    Solid,
}

/// An entity at one operation boundary, addressed by canonical position.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct TopologyRef {
    pub kind: TopologyKind,
    pub index: usize,
}

impl TopologyRef {
    #[inline]
    pub const fn new(kind: TopologyKind, index: usize) -> Self {
        Self { kind, index }
    }

    #[inline]
    pub const fn face(index: usize) -> Self {
        Self::new(TopologyKind::Face, index)
    }
}

/// Result entities not yet represented by an operation's history.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct HistoryCoverage {
    pub missing_results: Vec<TopologyRef>,
}

impl HistoryCoverage {
    #[inline]
    pub fn is_complete(&self) -> bool {
        self.missing_results.is_empty()
    }
}

/// A source entity together with the zero-based input operand that owns it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct InputTopologyRef {
    pub operand: usize,
    pub entity: TopologyRef,
}

impl InputTopologyRef {
    #[inline]
    pub const fn new(operand: usize, entity: TopologyRef) -> Self {
        Self { operand, entity }
    }

    #[inline]
    pub const fn face(operand: usize, index: usize) -> Self {
        Self::new(operand, TopologyRef::face(index))
    }
}

/// One explicit lineage event emitted by a modeling operation.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum TopologyChange {
    /// New result topology constructed from zero or more source entities.
    Generated {
        sources: Vec<InputTopologyRef>,
        result: TopologyRef,
    },
    /// One source entity survived as one result entity.
    Modified {
        source: InputTopologyRef,
        result: TopologyRef,
    },
    /// One source entity became several result entities.
    Split {
        source: InputTopologyRef,
        results: Vec<TopologyRef>,
    },
    /// Several source entities became one result entity.
    Merged {
        sources: Vec<InputTopologyRef>,
        result: TopologyRef,
    },
    /// A source entity has no descendant in the result.
    Deleted { source: InputTopologyRef },
}

/// Complete topology lineage emitted by one operation.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct TopologyHistory {
    pub changes: Vec<TopologyChange>,
}

impl TopologyHistory {
    /// History for a newly-created solid. Every result entity is explicitly
    /// recorded as generated rather than being implied by an empty history.
    pub fn generated_solid(solid: &Solid) -> Self {
        let mut history = Self::default();
        for result in result_inventory(solid) {
            history.generated([], result);
        }
        history
    }

    /// Report result entities absent from this history.
    pub fn coverage_for_solid(&self, solid: &Solid) -> HistoryCoverage {
        let mut represented = std::collections::HashSet::new();
        for change in &self.changes {
            match change {
                TopologyChange::Generated { result, .. }
                | TopologyChange::Modified { result, .. }
                | TopologyChange::Merged { result, .. } => {
                    represented.insert(*result);
                }
                TopologyChange::Split { results, .. } => {
                    represented.extend(results.iter().copied());
                }
                TopologyChange::Deleted { .. } => {}
            }
        }
        HistoryCoverage {
            missing_results: result_inventory(solid)
                .into_iter()
                .filter(|result| !represented.contains(result))
                .collect(),
        }
    }

    /// Account for currently-unattributed result entities as generated.
    ///
    /// This is the conservative Phase 1 behavior: an entity without proven
    /// lineage is explicitly new, never silently missing from history.
    pub fn complete_unattributed_results(&mut self, solid: &Solid) -> Vec<TopologyRef> {
        let missing = self.coverage_for_solid(solid).missing_results;
        for &result in &missing {
            self.generated([], result);
        }
        missing
    }

    #[inline]
    pub fn generated(
        &mut self,
        sources: impl IntoIterator<Item = InputTopologyRef>,
        result: TopologyRef,
    ) {
        self.changes.push(TopologyChange::Generated {
            sources: sources.into_iter().collect(),
            result,
        });
    }

    #[inline]
    pub fn modified(&mut self, source: InputTopologyRef, result: TopologyRef) {
        self.changes
            .push(TopologyChange::Modified { source, result });
    }

    #[inline]
    pub fn split(
        &mut self,
        source: InputTopologyRef,
        results: impl IntoIterator<Item = TopologyRef>,
    ) {
        self.changes.push(TopologyChange::Split {
            source,
            results: results.into_iter().collect(),
        });
    }

    #[inline]
    pub fn merged(
        &mut self,
        sources: impl IntoIterator<Item = InputTopologyRef>,
        result: TopologyRef,
    ) {
        self.changes.push(TopologyChange::Merged {
            sources: sources.into_iter().collect(),
            result,
        });
    }

    #[inline]
    pub fn deleted(&mut self, source: InputTopologyRef) {
        self.changes.push(TopologyChange::Deleted { source });
    }

    /// Direct result descendants of one source entity, in operation order.
    pub fn descendants(&self, source: InputTopologyRef) -> Vec<TopologyRef> {
        let mut out = Vec::new();
        for change in &self.changes {
            match change {
                TopologyChange::Generated { sources, result }
                | TopologyChange::Merged { sources, result }
                    if sources.contains(&source) =>
                {
                    out.push(*result);
                }
                TopologyChange::Modified {
                    source: candidate,
                    result,
                } if *candidate == source => out.push(*result),
                TopologyChange::Split {
                    source: candidate,
                    results,
                } if *candidate == source => out.extend(results.iter().copied()),
                _ => {}
            }
        }
        out.sort_unstable_by_key(|entity| (kind_order(entity.kind), entity.index));
        out.dedup();
        out
    }

    /// Source entities attributed to one result entity.
    pub fn sources_of(&self, result: TopologyRef) -> Vec<InputTopologyRef> {
        let mut out = Vec::new();
        for change in &self.changes {
            match change {
                TopologyChange::Generated {
                    sources,
                    result: candidate,
                }
                | TopologyChange::Merged {
                    sources,
                    result: candidate,
                } if *candidate == result => out.extend(sources.iter().copied()),
                TopologyChange::Modified {
                    source,
                    result: candidate,
                } if *candidate == result => out.push(*source),
                TopologyChange::Split { source, results } if results.contains(&result) => {
                    out.push(*source);
                }
                _ => {}
            }
        }
        out.sort_unstable_by_key(|source| {
            (
                source.operand,
                kind_order(source.entity.kind),
                source.entity.index,
            )
        });
        out.dedup();
        out
    }

    /// Validate event cardinality and same-kind invariants.
    pub fn validate(&self) -> Result<(), TopologyHistoryError> {
        for (change_index, change) in self.changes.iter().enumerate() {
            match change {
                TopologyChange::Modified { source, result }
                    if source.entity.kind != result.kind =>
                {
                    return Err(TopologyHistoryError::KindMismatch { change_index });
                }
                TopologyChange::Split { source, results } => {
                    if results.len() < 2 {
                        return Err(TopologyHistoryError::TooFewEntities { change_index });
                    }
                    if results
                        .iter()
                        .any(|result| result.kind != source.entity.kind)
                    {
                        return Err(TopologyHistoryError::KindMismatch { change_index });
                    }
                }
                TopologyChange::Merged { sources, result } => {
                    if sources.len() < 2 {
                        return Err(TopologyHistoryError::TooFewEntities { change_index });
                    }
                    if sources
                        .iter()
                        .any(|source| source.entity.kind != result.kind)
                    {
                        return Err(TopologyHistoryError::KindMismatch { change_index });
                    }
                }
                _ => {}
            }
        }
        Ok(())
    }

    /// Start a composable history chain with `next`. `carried_operand` names the
    /// operand of `next` that consumes this operation's result.
    pub fn compose(self, next: Self, carried_operand: usize) -> TopologyHistoryChain {
        TopologyHistoryChain {
            first: self,
            following: vec![HistoryStage {
                history: next,
                carried_operand,
            }],
        }
    }
}

fn result_inventory(solid: &Solid) -> Vec<TopologyRef> {
    let wire_count = solid
        .shell()
        .faces()
        .iter()
        .map(|face| face.wires().len())
        .sum::<usize>();
    let shell_count = solid
        .brep
        .solids
        .get(solid.id)
        .map_or(0, |data| data.shells.len());
    let counts = [
        (TopologyKind::Vertex, solid.vertex_count()),
        (TopologyKind::Edge, solid.edge_count()),
        (TopologyKind::Wire, wire_count),
        (TopologyKind::Face, solid.face_count()),
        (TopologyKind::Shell, shell_count),
        (TopologyKind::Solid, 1),
    ];
    counts
        .into_iter()
        .flat_map(|(kind, count)| (0..count).map(move |index| TopologyRef::new(kind, index)))
        .collect()
}

/// A later operation and the operand through which prior result topology flows.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct HistoryStage {
    pub history: TopologyHistory,
    pub carried_operand: usize,
}

/// Lossless composition of operation histories.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TopologyHistoryChain {
    pub first: TopologyHistory,
    pub following: Vec<HistoryStage>,
}

impl TopologyHistoryChain {
    /// Append another operation that consumes the current result through
    /// `carried_operand`.
    pub fn then(mut self, history: TopologyHistory, carried_operand: usize) -> Self {
        self.following.push(HistoryStage {
            history,
            carried_operand,
        });
        self
    }

    /// Trace one entity from the first operation's input to all descendants at
    /// the end of the chain.
    pub fn descendants(&self, source: InputTopologyRef) -> Vec<TopologyRef> {
        let mut current = self.first.descendants(source);
        for stage in &self.following {
            current = current
                .into_iter()
                .flat_map(|entity| {
                    stage
                        .history
                        .descendants(InputTopologyRef::new(stage.carried_operand, entity))
                })
                .collect();
            current.sort_unstable_by_key(|entity| (kind_order(entity.kind), entity.index));
            current.dedup();
        }
        current
    }
}

/// A malformed history event.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TopologyHistoryError {
    KindMismatch { change_index: usize },
    TooFewEntities { change_index: usize },
}

impl core::fmt::Display for TopologyHistoryError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::KindMismatch { change_index } => {
                write!(
                    f,
                    "topology history change {change_index} mixes entity kinds"
                )
            }
            Self::TooFewEntities { change_index } => write!(
                f,
                "topology history change {change_index} has too few entities"
            ),
        }
    }
}

impl std::error::Error for TopologyHistoryError {}

const fn kind_order(kind: TopologyKind) -> u8 {
    match kind {
        TopologyKind::Vertex => 0,
        TopologyKind::Edge => 1,
        TopologyKind::Wire => 2,
        TopologyKind::Face => 3,
        TopologyKind::Shell => 4,
        TopologyKind::Solid => 5,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn history_queries_split_merge_and_deletion() {
        let source = InputTopologyRef::face(0, 4);
        let mut history = TopologyHistory::default();
        history.split(source, [TopologyRef::face(2), TopologyRef::face(3)]);
        history.deleted(InputTopologyRef::face(1, 0));

        assert!(history.validate().is_ok());
        assert_eq!(
            history.descendants(source),
            vec![TopologyRef::face(2), TopologyRef::face(3)]
        );
        assert_eq!(history.sources_of(TopologyRef::face(3)), vec![source]);
        assert!(history.descendants(InputTopologyRef::face(1, 0)).is_empty());
    }

    #[test]
    fn history_chain_traces_the_carried_operand() {
        let original = InputTopologyRef::face(0, 4);
        let mut first = TopologyHistory::default();
        first.modified(original, TopologyRef::face(2));

        let mut second = TopologyHistory::default();
        second.split(
            InputTopologyRef::face(1, 2),
            [TopologyRef::face(7), TopologyRef::face(8)],
        );

        let chain = first.compose(second, 1);
        assert_eq!(
            chain.descendants(original),
            vec![TopologyRef::face(7), TopologyRef::face(8)]
        );
    }
}
