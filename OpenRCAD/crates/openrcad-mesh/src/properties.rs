//! Mass properties of a triangle mesh: volume, surface area, centroid, and
//! moments of inertia, computed by the divergence theorem over the closed
//! surface (each triangle contributes a signed tetrahedron against the
//! origin). Exact for the mesh; the mesh's chordal error is the only
//! approximation against the analytic solid.
//!
//! The mesh must be closed and consistently outward-wound for the volumetric
//! quantities to be meaningful (ZeroCAD meshes are — booleans and prisms are
//! sewn watertight). Surface area is winding-independent.

use crate::TriangleMesh;
use openrcad_foundation::Pnt;

/// Volume, area, centroid, and inertia of a closed mesh at uniform density.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MassProperties {
    /// Enclosed volume (mm³ in ZeroCAD's base units).
    pub volume: f64,
    /// Total surface area (mm²).
    pub surface_area: f64,
    /// Centroid (center of mass at uniform density).
    pub centroid: [f64; 3],
    /// Inertia tensor about the CENTROID at unit density, row-major
    /// `[[Ixx, Ixy, Ixz], [Ixy, Iyy, Iyz], [Ixz, Iyz, Izz]]`.
    /// Multiply by the material density for physical units.
    pub inertia: [[f64; 3]; 3],
}

/// Compute [`MassProperties`] for `mesh`. Returns `None` for an empty mesh or
/// one whose signed volume is (near-)zero — an open or degenerate surface has
/// no meaningful volumetric properties.
pub fn mass_properties(mesh: &TriangleMesh) -> Option<MassProperties> {
    if mesh.triangles.is_empty() {
        return None;
    }

    // Accumulated signed integrals over the volume: 1, x, y, z, x², y², z²,
    // xy, yz, zx — everything volume/centroid/inertia need, via the canonical
    // tetrahedron construction (Mirtich-style, specialised to triangles
    // against the origin).
    let mut vol = 0.0f64;
    let mut sx = 0.0f64;
    let mut sy = 0.0f64;
    let mut sz = 0.0f64;
    let mut sxx = 0.0f64;
    let mut syy = 0.0f64;
    let mut szz = 0.0f64;
    let mut sxy = 0.0f64;
    let mut syz = 0.0f64;
    let mut szx = 0.0f64;
    let mut area = 0.0f64;

    let p = |i: u32| -> Option<[f64; 3]> {
        let v: &Pnt = mesh.vertices.get(i as usize)?;
        Some([v.x(), v.y(), v.z()])
    };

    for t in &mesh.triangles {
        let (a, b, c) = (p(t[0])?, p(t[1])?, p(t[2])?);

        // Surface area from the cross product (winding-independent).
        let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let w = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let cr = [
            u[1] * w[2] - u[2] * w[1],
            u[2] * w[0] - u[0] * w[2],
            u[0] * w[1] - u[1] * w[0],
        ];
        area += 0.5 * (cr[0] * cr[0] + cr[1] * cr[1] + cr[2] * cr[2]).sqrt();

        // Signed volume of the origin tetrahedron: det(a, b, c) / 6.
        let det = a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
            + a[2] * (b[0] * c[1] - b[1] * c[0]);
        let v6 = det; // 6 × signed tetra volume
        vol += v6 / 6.0;

        // First moments: ∫x dV over the tetra = v6/24 · (a+b+c) (origin
        // vertex contributes 0).
        sx += v6 * (a[0] + b[0] + c[0]) / 24.0;
        sy += v6 * (a[1] + b[1] + c[1]) / 24.0;
        sz += v6 * (a[2] + b[2] + c[2]) / 24.0;

        // Second moments over the tetra with vertices (0, a, b, c):
        // ∫ xᵢxⱼ dV = v6/120 · (Σₖ aᵢₖ aⱼₖ + (Σₖ aᵢₖ)(Σₖ aⱼₖ)) where the sums
        // run over the three non-origin vertices.
        let f = |i: usize, j: usize| -> f64 {
            let (ai, bi, ci) = (a[i], b[i], c[i]);
            let (aj, bj, cj) = (a[j], b[j], c[j]);
            v6 / 120.0 * (ai * aj + bi * bj + ci * cj + (ai + bi + ci) * (aj + bj + cj))
        };
        sxx += f(0, 0);
        syy += f(1, 1);
        szz += f(2, 2);
        sxy += f(0, 1);
        syz += f(1, 2);
        szx += f(2, 0);
    }

    // A cut that inverts the winding yields negative volume; normalize so the
    // caller always sees positive quantities (orientation is a mesh detail).
    let sign = if vol < 0.0 { -1.0 } else { 1.0 };
    let volume = vol * sign;
    if volume < 1e-12 {
        return None;
    }
    let (sx, sy, sz) = (sx * sign, sy * sign, sz * sign);
    let (sxx, syy, szz) = (sxx * sign, syy * sign, szz * sign);
    let (sxy, syz, szx) = (sxy * sign, syz * sign, szx * sign);

    let centroid = [sx / volume, sy / volume, sz / volume];

    // Inertia about the ORIGIN at unit density...
    let ixx_o = syy + szz;
    let iyy_o = sxx + szz;
    let izz_o = sxx + syy;
    let ixy_o = -sxy;
    let iyz_o = -syz;
    let izx_o = -szx;
    // ...shifted to the centroid by the (reverse) parallel-axis theorem.
    let [cx, cy, cz] = centroid;
    let m = volume; // unit density: mass = volume
    let ixx = ixx_o - m * (cy * cy + cz * cz);
    let iyy = iyy_o - m * (cx * cx + cz * cz);
    let izz = izz_o - m * (cx * cx + cy * cy);
    let ixy = ixy_o + m * cx * cy;
    let iyz = iyz_o + m * cy * cz;
    let izx = izx_o + m * cz * cx;

    Some(MassProperties {
        volume,
        surface_area: area,
        centroid,
        inertia: [[ixx, ixy, izx], [ixy, iyy, iyz], [izx, iyz, izz]],
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol * b.abs().max(1.0)
    }

    /// An axis-aligned box mesh from (x0,y0,z0) to (x1,y1,z1), outward wound.
    fn box_mesh(x0: f64, y0: f64, z0: f64, x1: f64, y1: f64, z1: f64) -> TriangleMesh {
        let mut m = TriangleMesh::new();
        let v = [
            [x0, y0, z0],
            [x1, y0, z0],
            [x1, y1, z0],
            [x0, y1, z0],
            [x0, y0, z1],
            [x1, y0, z1],
            [x1, y1, z1],
            [x0, y1, z1],
        ];
        m.vertices = v.iter().map(|p| Pnt::new(p[0], p[1], p[2])).collect();
        // 12 triangles, outward.
        let quads: [[u32; 4]; 6] = [
            [0, 3, 2, 1], // -z
            [4, 5, 6, 7], // +z
            [0, 1, 5, 4], // -y
            [2, 3, 7, 6], // +y
            [1, 2, 6, 5], // +x
            [3, 0, 4, 7], // -x
        ];
        for q in quads {
            m.triangles.push([q[0], q[1], q[2]]);
            m.triangles.push([q[0], q[2], q[3]]);
        }
        m
    }

    #[test]
    fn box_volume_area_centroid_inertia() {
        // 2×3×4 box with a corner at (1, 2, 3).
        let (w, h, d) = (2.0, 3.0, 4.0);
        let mesh = box_mesh(1.0, 2.0, 3.0, 1.0 + w, 2.0 + h, 3.0 + d);
        let mp = mass_properties(&mesh).expect("closed box");
        assert!(close(mp.volume, w * h * d, 1e-9), "vol {}", mp.volume);
        assert!(
            close(mp.surface_area, 2.0 * (w * h + h * d + w * d), 1e-9),
            "area {}",
            mp.surface_area
        );
        assert!(close(mp.centroid[0], 2.0, 1e-9));
        assert!(close(mp.centroid[1], 3.5, 1e-9));
        assert!(close(mp.centroid[2], 5.0, 1e-9));
        // Box inertia about its centroid: m/12 · (h²+d²) etc.
        let m = w * h * d;
        assert!(close(mp.inertia[0][0], m / 12.0 * (h * h + d * d), 1e-9));
        assert!(close(mp.inertia[1][1], m / 12.0 * (w * w + d * d), 1e-9));
        assert!(close(mp.inertia[2][2], m / 12.0 * (w * w + h * h), 1e-9));
        // Axis-aligned box: products of inertia vanish.
        assert!(mp.inertia[0][1].abs() < 1e-9);
        assert!(mp.inertia[1][2].abs() < 1e-9);
        assert!(mp.inertia[0][2].abs() < 1e-9);
    }

    #[test]
    fn inverted_winding_still_positive() {
        let mut mesh = box_mesh(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        for t in &mut mesh.triangles {
            t.swap(1, 2);
        }
        let mp = mass_properties(&mesh).expect("closed box");
        assert!(close(mp.volume, 1.0, 1e-9));
        assert!(close(mp.centroid[0], 0.5, 1e-9));
    }

    #[test]
    fn empty_and_open_meshes_return_none() {
        assert!(mass_properties(&TriangleMesh::new()).is_none());
        let mut open = TriangleMesh::new();
        open.vertices = vec![
            Pnt::new(0.0, 0.0, 0.0),
            Pnt::new(1.0, 0.0, 0.0),
            Pnt::new(0.0, 1.0, 0.0),
        ];
        open.triangles = vec![[0, 1, 2]];
        // A single flat triangle encloses no volume.
        assert!(mass_properties(&open).is_none());
    }

    #[test]
    fn tessellated_cylinder_matches_analytic() {
        use openrcad_foundation::{Ax2, Dir};
        let r = 1.5f64;
        let h = 5.0f64;
        let solid = openrcad_primitives::make_cylinder(
            &Ax2::new(Pnt::origin(), Dir::new(0.0, 0.0, 1.0)),
            r,
            h,
        );
        let mesh = crate::tessellate(&solid, 0.005, 0.1);
        let mp = mass_properties(&mesh).expect("closed cylinder");
        let exact = std::f64::consts::PI * r * r * h;
        // The chordal polygon under-fills the circle slightly; 1% is generous.
        assert!(
            (mp.volume - exact).abs() / exact < 0.01,
            "volume {} vs analytic {exact}",
            mp.volume
        );
        assert!((mp.centroid[2] - h / 2.0).abs() < 1e-6);
    }

    #[test]
    fn tessellated_cone_is_robust_across_aspect_ratios() {
        use openrcad_foundation::{Ax2, Dir};
        // The cone fix (dense ruled grid) must hold its <2% accuracy AND a bounded
        // vertex count across extreme aspect ratios — not just the one revolve
        // test's shape. Cases: normal frustum, tall+thin, flat+wide, near-apex.
        for &(r1, r2, h) in &[
            (2.0, 0.5, 4.0),
            (1.0, 0.2, 40.0),
            (40.0, 8.0, 1.0),
            (5.0, 0.02, 5.0),
        ] {
            let solid = openrcad_primitives::make_cone(
                &Ax2::new(Pnt::origin(), Dir::new(0.0, 0.0, 1.0)),
                r1,
                r2,
                h,
            );
            let mesh = crate::tessellate(&solid, 0.005, 0.1);
            let mp = mass_properties(&mesh).expect("closed cone");
            // Frustum volume: π·h/3·(r1² + r1·r2 + r2²).
            let exact = std::f64::consts::PI * h / 3.0 * (r1 * r1 + r1 * r2 + r2 * r2);
            assert!(
                (mp.volume - exact).abs() / exact < 0.02,
                "cone r1={r1} r2={r2} h={h}: volume {} vs analytic {exact}",
                mp.volume
            );
            // No pathological vertex blowup (the grid must not over-generate
            // axial rows on extreme aspect ratios).
            assert!(
                mesh.vertices.len() < 40_000,
                "cone r1={r1} r2={r2} h={h}: {} verts — grid blew up",
                mesh.vertices.len()
            );
        }
    }
}
