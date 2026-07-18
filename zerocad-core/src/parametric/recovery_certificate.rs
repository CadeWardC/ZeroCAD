use super::*;
use openrcad::foundation::Pnt;
use std::fmt;

const BOUNDS_EPSILON: f32 = 0.05;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RecoveryKind {
    DippedJoin,
    ExpandedCut,
}

/// Evidence collected before a dimensional boolean fallback may replace an
/// exact operation. The certificate is deliberately runtime-only: it records
/// why the orchestrator accepted this candidate, not document history.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct RecoveryCertificate {
    pub(crate) kind: RecoveryKind,
    pub(crate) source_volume: f64,
    pub(crate) result_volume: f64,
    pub(crate) reference_volume: f64,
    pub(crate) fallback_volume: f64,
    pub(crate) containment_probes: usize,
}

impl RecoveryCertificate {
    pub(crate) fn summary(self) -> String {
        format!(
            "{:?}: source={:.9}, result={:.9}, reference={:.9}, fallback={:.9}, probes={}",
            self.kind,
            self.source_volume,
            self.result_volume,
            self.reference_volume,
            self.fallback_volume,
            self.containment_probes
        )
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct RecoveryCertificateError(&'static str);

impl fmt::Display for RecoveryCertificateError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(self.0)
    }
}

#[derive(Clone, Copy)]
struct SolidMeasure {
    volume: f64,
    probe: Pnt,
}

pub(crate) fn certify_dipped_join(
    source_parts: &[KernelSolid],
    exact_tool: &KernelSolid,
    dipped_tool: &KernelSolid,
    result_parts: &[KernelSolid],
) -> Result<RecoveryCertificate, RecoveryCertificateError> {
    if source_parts.is_empty() || result_parts.is_empty() {
        return Err(RecoveryCertificateError(
            "join certificate has no solid geometry",
        ));
    }

    let exact = measure(exact_tool)?;
    let dipped = measure(dipped_tool)?;
    let source = measures(source_parts)?;
    let result = measures(result_parts)?;
    let source_volume = total_volume(&source)?;
    let result_volume = total_volume(&result)?;
    let tolerance = volume_tolerance(&[source_volume, result_volume, exact.volume, dipped.volume]);

    let exact_bounds = bounds(exact_tool)?;
    let dipped_bounds = bounds(dipped_tool)?;
    if !crate::mock_kernel::aabb_contains(&dipped_bounds, &exact_bounds, BOUNDS_EPSILON)
        || !openrcad::algo::boolean::point_in_solid(&exact.probe, dipped_tool)
    {
        return Err(RecoveryCertificateError(
            "dipped join tool does not contain the exact tool",
        ));
    }

    let result_bounds = combined_bounds(result_parts)?;
    for part in source_parts {
        if !crate::mock_kernel::aabb_contains(&result_bounds, &bounds(part)?, BOUNDS_EPSILON) {
            return Err(RecoveryCertificateError(
                "dipped join result does not contain every source bound",
            ));
        }
    }
    if !crate::mock_kernel::aabb_contains(&result_bounds, &exact_bounds, BOUNDS_EPSILON) {
        return Err(RecoveryCertificateError(
            "dipped join result does not contain the exact tool bound",
        ));
    }

    let mut containment_probes = 0;
    for probe in source
        .iter()
        .map(|measure| measure.probe)
        .chain([exact.probe])
    {
        if !contains_probe(result_parts, &probe) {
            return Err(RecoveryCertificateError(
                "dipped join containment proof is ambiguous",
            ));
        }
        containment_probes += 1;
    }

    if result_volume + tolerance < source_volume
        || result_volume + tolerance < exact.volume
        || result_volume > source_volume + dipped.volume + tolerance
    {
        return Err(RecoveryCertificateError(
            "dipped join result violates certified volume bounds",
        ));
    }

    Ok(RecoveryCertificate {
        kind: RecoveryKind::DippedJoin,
        source_volume,
        result_volume,
        reference_volume: exact.volume,
        fallback_volume: dipped.volume,
        containment_probes,
    })
}

pub(crate) fn certify_expanded_cut(
    source: &KernelSolid,
    exact_tool: &KernelSolid,
    expanded_tool: &KernelSolid,
    result_parts: &[KernelSolid],
) -> Result<RecoveryCertificate, RecoveryCertificateError> {
    let source_measure = measure(source)?;
    let exact = measure(exact_tool)?;
    let expanded = measure(expanded_tool)?;
    let result = measures(result_parts)?;
    let result_volume = total_volume(&result)?;
    let tolerance = volume_tolerance(&[
        source_measure.volume,
        result_volume,
        exact.volume,
        expanded.volume,
    ]);

    let source_bounds = bounds(source)?;
    let exact_bounds = bounds(exact_tool)?;
    let expanded_bounds = bounds(expanded_tool)?;
    if !crate::mock_kernel::aabb_contains(&expanded_bounds, &exact_bounds, BOUNDS_EPSILON)
        || !openrcad::algo::boolean::point_in_solid(&exact.probe, expanded_tool)
        || expanded.volume + tolerance < exact.volume
    {
        return Err(RecoveryCertificateError(
            "expanded cut tool does not contain the exact tool",
        ));
    }

    let mut containment_probes = 0;
    for (part, measured) in result_parts.iter().zip(&result) {
        if !crate::mock_kernel::aabb_contains(&source_bounds, &bounds(part)?, BOUNDS_EPSILON)
            || !openrcad::algo::boolean::point_in_solid(&measured.probe, source)
        {
            return Err(RecoveryCertificateError(
                "expanded cut result is not contained by its source",
            ));
        }
        containment_probes += 1;
    }

    let removed_probe = overlap_probe(source, exact_tool, source_measure.probe, exact.probe)?;
    if contains_probe(result_parts, &removed_probe) {
        return Err(RecoveryCertificateError(
            "expanded cut did not remove a proven exact-overlap point",
        ));
    }
    containment_probes += 1;

    if result_volume + tolerance >= source_measure.volume
        || result_volume + expanded.volume + tolerance < source_measure.volume
    {
        return Err(RecoveryCertificateError(
            "expanded cut result violates certified volume bounds",
        ));
    }

    Ok(RecoveryCertificate {
        kind: RecoveryKind::ExpandedCut,
        source_volume: source_measure.volume,
        result_volume,
        reference_volume: exact.volume,
        fallback_volume: expanded.volume,
        containment_probes,
    })
}

fn measure(solid: &KernelSolid) -> Result<SolidMeasure, RecoveryCertificateError> {
    let mesh = MockMesh::try_from_solid(solid)
        .map_err(|_| RecoveryCertificateError("recovery solid could not be tessellated"))?;
    let properties = mesh.mass_properties().ok_or(RecoveryCertificateError(
        "recovery solid has no finite volume",
    ))?;
    if !properties.volume.is_finite() || properties.volume <= 0.0 {
        return Err(RecoveryCertificateError(
            "recovery solid has invalid volume",
        ));
    }

    let centroid = Pnt::new(
        properties.centroid[0],
        properties.centroid[1],
        properties.centroid[2],
    );
    let probe = if openrcad::algo::boolean::point_in_solid(&centroid, solid) {
        centroid
    } else {
        let (lo, hi) = bounds(solid)?;
        let center = Pnt::new(
            f64::from(lo[0] + hi[0]) * 0.5,
            f64::from(lo[1] + hi[1]) * 0.5,
            f64::from(lo[2] + hi[2]) * 0.5,
        );
        if !openrcad::algo::boolean::point_in_solid(&center, solid) {
            return Err(RecoveryCertificateError(
                "recovery solid has no unambiguous interior probe",
            ));
        }
        center
    };

    Ok(SolidMeasure {
        volume: properties.volume,
        probe,
    })
}

fn measures(solids: &[KernelSolid]) -> Result<Vec<SolidMeasure>, RecoveryCertificateError> {
    solids.iter().map(measure).collect()
}

fn total_volume(measures: &[SolidMeasure]) -> Result<f64, RecoveryCertificateError> {
    let volume: f64 = measures.iter().map(|measure| measure.volume).sum();
    if volume.is_finite() {
        Ok(volume)
    } else {
        Err(RecoveryCertificateError(
            "recovery volume accumulation is not finite",
        ))
    }
}

fn volume_tolerance(volumes: &[f64]) -> f64 {
    volumes.iter().copied().map(f64::abs).fold(0.0, f64::max) * 1.0e-6 + 1.0e-9
}

fn bounds(solid: &KernelSolid) -> Result<([f32; 3], [f32; 3]), RecoveryCertificateError> {
    crate::mock_kernel::solid_aabb(solid).ok_or(RecoveryCertificateError(
        "recovery solid has no finite bounds",
    ))
}

fn combined_bounds(
    solids: &[KernelSolid],
) -> Result<([f32; 3], [f32; 3]), RecoveryCertificateError> {
    let mut combined = ([f32::INFINITY; 3], [f32::NEG_INFINITY; 3]);
    for solid in solids {
        let solid_bounds = bounds(solid)?;
        for axis in 0..3 {
            combined.0[axis] = combined.0[axis].min(solid_bounds.0[axis]);
            combined.1[axis] = combined.1[axis].max(solid_bounds.1[axis]);
        }
    }
    if combined
        .0
        .iter()
        .chain(&combined.1)
        .all(|value| value.is_finite())
    {
        Ok(combined)
    } else {
        Err(RecoveryCertificateError(
            "recovery result has no finite combined bounds",
        ))
    }
}

fn contains_probe(solids: &[KernelSolid], probe: &Pnt) -> bool {
    solids
        .iter()
        .any(|solid| openrcad::algo::boolean::point_in_solid(probe, solid))
}

fn overlap_probe(
    source: &KernelSolid,
    exact_tool: &KernelSolid,
    source_probe: Pnt,
    exact_probe: Pnt,
) -> Result<Pnt, RecoveryCertificateError> {
    let source_bounds = bounds(source)?;
    let exact_bounds = bounds(exact_tool)?;
    let overlap_center = Pnt::new(
        f64::from(
            source_bounds.0[0].max(exact_bounds.0[0]) + source_bounds.1[0].min(exact_bounds.1[0]),
        ) * 0.5,
        f64::from(
            source_bounds.0[1].max(exact_bounds.0[1]) + source_bounds.1[1].min(exact_bounds.1[1]),
        ) * 0.5,
        f64::from(
            source_bounds.0[2].max(exact_bounds.0[2]) + source_bounds.1[2].min(exact_bounds.1[2]),
        ) * 0.5,
    );

    [overlap_center, source_probe, exact_probe]
        .into_iter()
        .find(|probe| {
            openrcad::algo::boolean::point_in_solid(probe, source)
                && openrcad::algo::boolean::point_in_solid(probe, exact_tool)
        })
        .ok_or(RecoveryCertificateError(
            "exact cut overlap has no unambiguous interior probe",
        ))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn box_at(x: f64, y: f64, z: f64, dx: f64, dy: f64, dz: f64) -> KernelSolid {
        openrcad::primitives::make_box_operation(&Pnt::new(x, y, z), dx, dy, dz)
            .expect("box")
            .value
    }

    #[test]
    fn dipped_join_requires_volume_and_containment_evidence() {
        let source = box_at(0.0, 0.0, 0.0, 10.0, 10.0, 10.0);
        let exact = box_at(9.0, 2.0, 2.0, 4.0, 6.0, 6.0);
        let dipped = box_at(8.9, 2.0, 2.0, 4.1, 6.0, 6.0);
        let result = crate::mock_kernel::union(&source, &dipped).expect("union");

        let certificate = certify_dipped_join(
            std::slice::from_ref(&source),
            &exact,
            &dipped,
            std::slice::from_ref(&result),
        )
        .expect("certified fallback");
        assert_eq!(certificate.kind, RecoveryKind::DippedJoin);
        assert!(certificate.containment_probes >= 2);

        assert!(certify_dipped_join(
            std::slice::from_ref(&source),
            &exact,
            &dipped,
            std::slice::from_ref(&source),
        )
        .is_err());
    }

    #[test]
    fn expanded_cut_requires_a_removed_exact_overlap_probe() {
        let source = box_at(0.0, 0.0, 0.0, 10.0, 10.0, 10.0);
        let exact = box_at(4.0, -1.0, -1.0, 2.0, 12.0, 12.0);
        let expanded = box_at(3.9, -1.1, -1.1, 2.2, 12.2, 12.2);
        let result = crate::mock_kernel::difference_bodies(&source, &expanded).expect("difference");

        let certificate =
            certify_expanded_cut(&source, &exact, &expanded, &result).expect("certified fallback");
        assert_eq!(certificate.kind, RecoveryKind::ExpandedCut);
        assert!(certificate.result_volume < certificate.source_volume);

        assert!(
            certify_expanded_cut(&source, &exact, &expanded, std::slice::from_ref(&source),)
                .is_err()
        );
    }
}
