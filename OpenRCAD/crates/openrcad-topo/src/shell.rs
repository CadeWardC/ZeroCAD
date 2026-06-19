//! A topological shell — a set of connected [`Face`]s (OCCT `TopoDS_Shell`).
//!
//! A shell is the 2-manifold (or near-manifold) boundary of a [`Solid`]. In this
//! first representation the faces are stored as a flat list; the README's roadmap
//! (Point 1: *generational arenas*) upgrades this to shared, index-based storage
//! so adjacent faces reference the same edge/vertex handles.

use openrcad_foundation::Trsf;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

use crate::arena::{BRep, ShellData, ShellId};
use crate::face::Face;

/// A collection of [`Face`]s forming (part of) a solid boundary.
#[derive(Clone, Debug)]
pub struct Shell {
    pub(crate) brep: Arc<BRep>,
    pub(crate) id: ShellId,
}

impl Shell {
    /// Construct a Shell handle from a shared BRep storage and Shell ID.
    #[inline]
    pub fn from_id(brep: Arc<BRep>, id: ShellId) -> Self {
        Self { brep, id }
    }

    /// Get the Shell ID in the arena.
    #[inline]
    pub fn id(&self) -> ShellId {
        self.id
    }

    /// Get the underlying BRep storage.
    #[inline]
    pub fn brep(&self) -> &Arc<BRep> {
        &self.brep
    }

    /// A shell from a list of faces.
    #[inline]
    pub fn new(faces: Vec<Face>) -> Self {
        Self::from_faces(faces)
    }

    /// Build from anything iterable of faces.
    pub fn from_faces<I: IntoIterator<Item = Face>>(faces: I) -> Self {
        let mut brep = BRep::new();
        let mut merged: std::collections::HashMap<usize, crate::arena::MergeMap> =
            std::collections::HashMap::new();
        let mut new_faces = Vec::new();
        for face in faces {
            let ptr = Arc::as_ptr(&face.brep) as usize;
            let map = merged.entry(ptr).or_insert_with(|| brep.merge(&face.brep));
            let new_face_id = map.faces[&face.id];

            // Sync face's orientation in BRep data to match handle
            let merged_face_data = brep.faces.get_mut(new_face_id).unwrap();
            merged_face_data.orientation = face.orientation;

            new_faces.push(new_face_id);
        }
        let id = brep.shells.insert(ShellData { faces: new_faces });
        Self {
            brep: Arc::new(brep),
            id,
        }
    }

    /// The faces.
    #[inline]
    pub fn faces(&self) -> Vec<Face> {
        let shell_data = &self.brep.shells[self.id];
        shell_data
            .faces
            .iter()
            .map(|&face_id| Face {
                brep: self.brep.clone(),
                id: face_id,
                orientation: self.brep.faces[face_id].orientation,
            })
            .collect()
    }

    /// Number of faces.
    #[inline]
    pub fn len(&self) -> usize {
        self.brep.shells[self.id].faces.len()
    }

    /// True if there are no faces.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Apply `t` to every face.
    pub fn transformed(&self, t: &Trsf) -> Self {
        let transformed_brep = self.brep.transformed(t);
        Self {
            brep: Arc::new(transformed_brep),
            id: self.id,
        }
    }
}

impl Default for Shell {
    fn default() -> Self {
        let mut brep = BRep::new();
        let id = brep.shells.insert(ShellData { faces: Vec::new() });
        Self {
            brep: Arc::new(brep),
            id,
        }
    }
}

impl PartialEq for Shell {
    fn eq(&self, other: &Self) -> bool {
        self.faces() == other.faces()
    }
}

impl Serialize for Shell {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        (&*self.brep, self.id).serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Shell {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let (brep, id) = <(BRep, ShellId)>::deserialize(deserializer)?;
        Ok(Self {
            brep: Arc::new(brep),
            id,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_shell_is_default() {
        let s = Shell::default();
        assert!(s.is_empty());
        assert_eq!(s.len(), 0);
    }
}
