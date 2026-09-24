#![allow(dead_code)]
use zerocad_core::*;

pub fn part(width: f32) -> PreparedPartDefinition {
    let mut graph = ParametricGraph::new();
    graph.add_feature(FeatureNode {
        id: "plate".into(),
        name: "Mounting plate".into(),
        feature: FeatureType::Box {
            w: width,
            h: 8.0,
            d: 2.0,
        },
    });
    let bytes = write_project_document_to_vec(
        &ProjectDocument::Part(Document::from_graph(graph, Unit::Millimeter)),
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .unwrap();
    prepare_part_definition(&bytes, Some("plate.zcad"), &LoadOptions::default()).unwrap()
}

pub fn entity(id: u64) -> AssemblyEntityRef {
    AssemblyEntityRef {
        occurrence_id: id,
        local_body_id: "plate".into(),
        local_selector: AssemblyLocalSelector::Origin,
    }
}

/// Repeated plates separated by 12 mm. Dense adds short closed constraint loops,
/// matching the existing release stress topology (not an all-to-all graph).
pub fn corpus(count: u64, dense: bool) -> (AssemblyDocument, MateSolveContext) {
    let prepared = part(10.0);
    let mut assembly = AssemblyDocument::new();
    for id in 1..=count {
        insert_prepared_occurrence(
            &mut assembly,
            prepared.clone(),
            RigidPlacement::new([0.0, 0.0, 12.0 * (id - 1) as f64], [1.0, 0.0, 0.0, 0.0]).unwrap(),
            id == 1,
        )
        .unwrap();
    }
    let mut mates = AssemblyMateSet::default();
    let mut context = MateSolveContext {
        assembly_diagonal_mm: 12.0 * count as f64,
        ..Default::default()
    };
    for id in 1..=count {
        context.characteristic_lengths_mm.insert(id, 10.0);
    }
    for second in 2..=count {
        let firsts = if dense && second > 2 {
            vec![second - 1, second - 2]
        } else {
            vec![second - 1]
        };
        for first in firsts {
            let id = mates.next_mate_id;
            mates.next_mate_id += 1;
            mates.mates.insert(
                id,
                AssemblyMate {
                    id,
                    name: format!("Spacing {id}"),
                    suppressed: false,
                    first: entity(first),
                    second: Some(entity(second)),
                    kind: AssemblyMateKind::SignedDistance {
                        millimeters: 12.0 * (second - first) as f64,
                    },
                    sense: MateSense::Aligned,
                },
            );
            context.frames.insert(
                id,
                ResolvedMateFrames {
                    first: MateFrame {
                        origin: [0.0; 3],
                        axis: [0.0, 0.0, 1.0],
                        radial: [1.0, 0.0, 0.0],
                    },
                    second: Some(MateFrame {
                        origin: [0.0; 3],
                        axis: [0.0, 0.0, 1.0],
                        radial: [1.0, 0.0, 0.0],
                    }),
                },
            );
        }
    }
    assembly.mates = Some(mates);
    (assembly, context)
}

pub fn roundtrip(assembly: &AssemblyDocument) -> AssemblyDocument {
    let bytes = write_project_document_to_vec(
        &ProjectDocument::Assembly(assembly.clone()),
        &SaveOptions::default(),
        &HydrationBundle::default(),
    )
    .unwrap();
    let loaded = read_project_document_from_slice(&bytes, &LoadOptions::default()).unwrap();
    let ProjectDocument::Assembly(result) = loaded.document else {
        panic!("expected assembly")
    };
    result
}
