//! The durable topological-name grammar, parsed.
//!
//! Persistent naming stores entity identity as strings (serde-friendly, no
//! kernel-type leakage). This module is the single authority on that grammar:
//! every form a producer emits parses here, and `Display` reproduces the input
//! byte-for-byte (the golden test pins it). Resolvers can match on structure —
//! owner, role, occurrence — instead of ad-hoc `contains`/`ends_with` string
//! probes, and new arms (a constraint-solver entity, a discriminator) are
//! added here once instead of scattered format strings.
//!
//! Grammar (one variant per creation path):
//! - `sketch:{body}:region:{i}:face:{role}[:occ:{n}]` — extruded region face.
//! - `sketch:{body}:region:{i}:fragment:{frag}:role:{role}[:occ:{n}]` —
//!   extruded region edge; `{frag}` is itself `shape:{id}:{kind}[:{k}]` or
//!   `fragment:{i}:{kind}` (see `provenance_fragment_stable_id`).
//! - `box_{node}:face:{+x|-x|+y|-y|+z|-z}` — primitive box face.
//! - `cyl_{node}:face:{lateral|top|bottom}` — primitive cylinder face.
//! - `cut:{node}:tool-face:{i}` / `join:{node}:tool-face:{i}` — a boolean's
//!   GENERATED face, named from the input tool face that produced it (exact
//!   kernel history, `BooleanFaceHistory`).
//! - `entity:{id}:{role}` — reserved for solver-created sketch entities.
//! - `mesh:{group}` — the reconstructed fallback identity (not durable; never
//!   trusted without a geometric match).

/// A parsed durable name. `Unrecognized` carries anything outside the grammar
/// verbatim so `Display` is total and lossless.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TopoName {
    /// `sketch:{body}:region:{i}:face:{role}[:occ:{n}]`
    SketchFace {
        body: String,
        region: usize,
        role: String,
        occ: Option<usize>,
    },
    /// `sketch:{body}:region:{i}:fragment:{frag}:role:{role}[:occ:{n}]`
    SketchEdge {
        body: String,
        region: usize,
        fragment: String,
        role: String,
        occ: Option<usize>,
    },
    /// `box_{node}:face:{role}` / `cyl_{node}:face:{role}`
    PrimitiveFace {
        kind: PrimitiveKind,
        node: String,
        role: String,
    },
    /// `cut:{node}:tool-face:{i}` / `join:{node}:tool-face:{i}`
    GeneratedFace {
        op: BooleanOpKind,
        node: String,
        tool_face: usize,
    },
    /// `entity:{id}:{role}` — solver-created sketch entity provenance.
    Entity { id: u32, role: String },
    /// `mesh:{group}` — reconstructed (non-durable) identity.
    MeshGroup(u32),
    /// Anything else, preserved verbatim.
    Unrecognized(String),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PrimitiveKind {
    Box,
    Cylinder,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BooleanOpKind {
    Cut,
    Join,
}

impl TopoName {
    pub fn parse(s: &str) -> TopoName {
        parse_name(s).unwrap_or_else(|| TopoName::Unrecognized(s.to_string()))
    }

    /// True when this is a durable design identity (safe to resolve by name);
    /// false for the reconstructed `mesh:{group}` ids and unknown forms, which
    /// must always be confirmed geometrically.
    pub fn is_durable(&self) -> bool {
        !matches!(self, TopoName::MeshGroup(_) | TopoName::Unrecognized(_))
    }

    /// The owner (feature/node) this name is anchored to, when it has one.
    pub fn owner_node(&self) -> Option<&str> {
        match self {
            TopoName::SketchFace { body, .. } | TopoName::SketchEdge { body, .. } => Some(body),
            TopoName::PrimitiveFace { node, .. } | TopoName::GeneratedFace { node, .. } => {
                Some(node)
            }
            _ => None,
        }
    }
}

impl std::fmt::Display for TopoName {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TopoName::SketchFace {
                body,
                region,
                role,
                occ,
            } => {
                write!(f, "sketch:{body}:region:{region}:face:{role}")?;
                if let Some(n) = occ {
                    write!(f, ":occ:{n}")?;
                }
                Ok(())
            }
            TopoName::SketchEdge {
                body,
                region,
                fragment,
                role,
                occ,
            } => {
                write!(
                    f,
                    "sketch:{body}:region:{region}:fragment:{fragment}:role:{role}"
                )?;
                if let Some(n) = occ {
                    write!(f, ":occ:{n}")?;
                }
                Ok(())
            }
            TopoName::PrimitiveFace { kind, node, role } => match kind {
                PrimitiveKind::Box => write!(f, "box_{node}:face:{role}"),
                PrimitiveKind::Cylinder => write!(f, "cyl_{node}:face:{role}"),
            },
            TopoName::GeneratedFace {
                op,
                node,
                tool_face,
            } => match op {
                BooleanOpKind::Cut => write!(f, "cut:{node}:tool-face:{tool_face}"),
                BooleanOpKind::Join => write!(f, "join:{node}:tool-face:{tool_face}"),
            },
            TopoName::Entity { id, role } => write!(f, "entity:{id}:{role}"),
            TopoName::MeshGroup(g) => write!(f, "mesh:{g}"),
            TopoName::Unrecognized(s) => f.write_str(s),
        }
    }
}

fn parse_name(s: &str) -> Option<TopoName> {
    // Trailing `:occ:{n}` applies to the sketch face/edge forms.
    fn split_occ(s: &str) -> (&str, Option<usize>) {
        if let Some(idx) = s.rfind(":occ:") {
            if let Ok(n) = s[idx + 5..].parse() {
                return (&s[..idx], Some(n));
            }
        }
        (s, None)
    }

    if let Some(rest) = s.strip_prefix("sketch:") {
        let (core, occ) = split_occ(rest);
        // {body}:region:{i}:face:{role}  |  {body}:region:{i}:fragment:{frag}:role:{role}
        let region_idx = core.find(":region:")?;
        let body = &core[..region_idx];
        let after = &core[region_idx + 8..];
        let sep = after.find(':')?;
        let region: usize = after[..sep].parse().ok()?;
        let tail = &after[sep + 1..];
        if let Some(role) = tail.strip_prefix("face:") {
            if body.is_empty() || role.is_empty() {
                return None;
            }
            return Some(TopoName::SketchFace {
                body: body.to_string(),
                region,
                role: role.to_string(),
                occ,
            });
        }
        if let Some(frag_and_role) = tail.strip_prefix("fragment:") {
            let role_idx = frag_and_role.rfind(":role:")?;
            let fragment = &frag_and_role[..role_idx];
            let role = &frag_and_role[role_idx + 6..];
            if body.is_empty() || fragment.is_empty() || role.is_empty() {
                return None;
            }
            return Some(TopoName::SketchEdge {
                body: body.to_string(),
                region,
                fragment: fragment.to_string(),
                role: role.to_string(),
                occ,
            });
        }
        return None;
    }
    for (prefix, kind) in [
        ("box_", PrimitiveKind::Box),
        ("cyl_", PrimitiveKind::Cylinder),
    ] {
        if let Some(rest) = s.strip_prefix(prefix) {
            if let Some(idx) = rest.find(":face:") {
                let node = &rest[..idx];
                let role = &rest[idx + 6..];
                if !node.is_empty() && !role.is_empty() && !role.contains(':') {
                    return Some(TopoName::PrimitiveFace {
                        kind,
                        node: node.to_string(),
                        role: role.to_string(),
                    });
                }
            }
        }
    }
    for (prefix, op) in [("cut:", BooleanOpKind::Cut), ("join:", BooleanOpKind::Join)] {
        if let Some(rest) = s.strip_prefix(prefix) {
            if let Some(idx) = rest.find(":tool-face:") {
                let node = &rest[..idx];
                if let Ok(tool_face) = rest[idx + 11..].parse() {
                    if !node.is_empty() {
                        return Some(TopoName::GeneratedFace {
                            op,
                            node: node.to_string(),
                            tool_face,
                        });
                    }
                }
            }
        }
    }
    if let Some(rest) = s.strip_prefix("entity:") {
        let sep = rest.find(':')?;
        let id: u32 = rest[..sep].parse().ok()?;
        let role = &rest[sep + 1..];
        if !role.is_empty() {
            return Some(TopoName::Entity {
                id,
                role: role.to_string(),
            });
        }
        return None;
    }
    if let Some(rest) = s.strip_prefix("mesh:") {
        if let Ok(g) = rest.parse() {
            return Some(TopoName::MeshGroup(g));
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The grammar's golden corpus: every producer form must parse to the
    /// expected structure AND display back byte-identically — serialized refs
    /// depend on it.
    #[test]
    fn every_grammar_form_round_trips_byte_identically() {
        let corpus = [
            "sketch:extrude_3:region:0:face:top",
            "sketch:extrude_3:region:0:face:side:occ:2",
            "sketch:extrude_3:region:1:face:bottom",
            "sketch:extrude_3:region:0:fragment:shape:1:rectangle-edge:2:role:top",
            "sketch:extrude_3:region:0:fragment:shape:0:circle:role:side:occ:1",
            "sketch:extrude_3:region:0:fragment:fragment:4:raw-polyline:role:bottom",
            "box_box_1:face:+x",
            "box_box_1:face:-z",
            "cyl_cyl_2:face:lateral",
            "cyl_cyl_2:face:top",
            "cut:extrude_5:tool-face:3",
            "join:extrude_7:tool-face:0",
            "entity:12:line",
            "mesh:4",
        ];
        for s in corpus {
            let parsed = TopoName::parse(s);
            assert!(
                !matches!(parsed, TopoName::Unrecognized(_)),
                "grammar form must be recognized: {s} -> {parsed:?}"
            );
            assert_eq!(parsed.to_string(), s, "display must be byte-identical");
        }
    }

    #[test]
    fn unrecognized_names_pass_through_verbatim() {
        for s in [
            "",
            "weird",
            "sketch:broken",
            "mesh:not-a-number",
            "box_:face:+x",
        ] {
            let parsed = TopoName::parse(s);
            assert_eq!(parsed.to_string(), s, "lossless for out-of-grammar input");
            assert!(!parsed.is_durable());
        }
    }

    #[test]
    fn durability_and_ownership_classification() {
        assert!(TopoName::parse("cut:extrude_5:tool-face:3").is_durable());
        assert_eq!(
            TopoName::parse("cut:extrude_5:tool-face:3").owner_node(),
            Some("extrude_5")
        );
        assert!(!TopoName::parse("mesh:9").is_durable());
        assert_eq!(
            TopoName::parse("sketch:extrude_3:region:0:face:top").owner_node(),
            Some("extrude_3")
        );
        assert_eq!(
            TopoName::parse("box_box_1:face:+x").owner_node(),
            Some("box_1")
        );
    }
}
