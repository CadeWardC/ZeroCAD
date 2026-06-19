//! Matrices — [`Mat`] (3×3) and [`Mat2d`] (2×2).
//!
//! These back the linear part of a [`Trsf`](crate::trsf::Trsf) and are
//! occasionally useful directly. Storage is row-major; indexing is 0-based.

use serde::{Deserialize, Serialize};

use crate::xyz::{Xy, Xyz};

/// A 3×3 matrix (OCCT `gp_Mat`).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Mat {
    rows: [[f64; 3]; 3],
}

impl Mat {
    /// The 3×3 identity.
    pub const IDENTITY: Self = Self {
        rows: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    };

    /// The zero matrix.
    pub const ZERO: Self = Self {
        rows: [[0.0; 3]; 3],
    };

    /// The identity matrix.
    #[inline]
    pub const fn identity() -> Self {
        Self::IDENTITY
    }

    /// Build from three rows.
    #[inline]
    pub const fn from_rows(rows: [[f64; 3]; 3]) -> Self {
        Self { rows }
    }

    /// Element at `(row, col)`, 0-based.
    #[inline]
    pub fn at(&self, row: usize, col: usize) -> f64 {
        self.rows[row][col]
    }

    /// Set the element at `(row, col)`, 0-based.
    #[inline]
    pub fn set_at(&mut self, row: usize, col: usize, value: f64) {
        self.rows[row][col] = value;
    }

    /// `self · v` — the matrix applied to a column vector.
    #[inline]
    pub fn multiply_xyz(&self, v: &Xyz) -> Xyz {
        Xyz::new(
            self.rows[0][0] * v.x() + self.rows[0][1] * v.y() + self.rows[0][2] * v.z(),
            self.rows[1][0] * v.x() + self.rows[1][1] * v.y() + self.rows[1][2] * v.z(),
            self.rows[2][0] * v.x() + self.rows[2][1] * v.y() + self.rows[2][2] * v.z(),
        )
    }

    /// `self · other` — standard matrix product.
    #[inline]
    pub fn multiplied(&self, other: &Self) -> Self {
        let mut out = [[0.0; 3]; 3];
        for (i, row) in out.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                *cell = self.rows[i][0] * other.rows[0][j]
                    + self.rows[i][1] * other.rows[1][j]
                    + self.rows[i][2] * other.rows[2][j];
            }
        }
        Self::from_rows(out)
    }

    /// The transpose.
    #[inline]
    pub fn transposed(&self) -> Self {
        Self::from_rows([
            [self.rows[0][0], self.rows[1][0], self.rows[2][0]],
            [self.rows[0][1], self.rows[1][1], self.rows[2][1]],
            [self.rows[0][2], self.rows[1][2], self.rows[2][2]],
        ])
    }

    /// The determinant.
    #[inline]
    pub fn determinant(&self) -> f64 {
        let r = self.rows;
        r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
            - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
            + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0])
    }

    /// The inverse, or `None` if (near) singular.
    #[inline]
    pub fn inverted(&self) -> Option<Self> {
        let det = self.determinant();
        if det.abs() < crate::tolerance::CONFUSION {
            return None;
        }
        let r = self.rows;
        let inv_det = 1.0 / det;
        // The two indices left after deleting `idx`, in ascending order. (Must be
        // sorted: swapping the two rows of a 2x2 flips its determinant's sign.)
        let remaining = |idx: usize| -> [usize; 2] {
            match idx {
                0 => [1, 2],
                1 => [0, 2],
                _ => [0, 1],
            }
        };
        // Cofactor at (i,j): the minor of the 2x2 left by deleting row i, col j.
        let c = |i: usize, j: usize| -> f64 {
            let rows_idx = remaining(i);
            let cols_idx = remaining(j);
            let minor = r[rows_idx[0]][cols_idx[0]] * r[rows_idx[1]][cols_idx[1]]
                - r[rows_idx[0]][cols_idx[1]] * r[rows_idx[1]][cols_idx[0]];
            // sign (-1)^(i+j)
            if (i + j) % 2 == 0 {
                minor
            } else {
                -minor
            }
        };
        // inverse = adjugate / det = transpose(cofactor) / det
        let mut out = [[0.0; 3]; 3];
        for (i, row) in out.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                *cell = c(j, i) * inv_det; // transposed cofactor
            }
        }
        Some(Self::from_rows(out))
    }

    /// True when the rows are orthonormal (unit length and mutually
    /// perpendicular) within `tol`.
    pub fn is_orthonormal(&self, tol: f64) -> bool {
        for i in 0..3 {
            let ri = Xyz::new(self.rows[i][0], self.rows[i][1], self.rows[i][2]);
            if (ri.modulus() - 1.0).abs() > tol {
                return false;
            }
            for j in (i + 1)..3 {
                let rj = Xyz::new(self.rows[j][0], self.rows[j][1], self.rows[j][2]);
                if ri.dot(&rj).abs() > tol {
                    return false;
                }
            }
        }
        true
    }
}

impl Default for Mat {
    #[inline]
    fn default() -> Self {
        Self::IDENTITY
    }
}

/// A 2×2 matrix (OCCT `gp_Mat2d`).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Mat2d {
    rows: [[f64; 2]; 2],
}

impl Mat2d {
    /// The 2×2 identity.
    pub const IDENTITY: Self = Self {
        rows: [[1.0, 0.0], [0.0, 1.0]],
    };

    /// Build from two rows.
    #[inline]
    pub const fn from_rows(rows: [[f64; 2]; 2]) -> Self {
        Self { rows }
    }

    /// Element at `(row, col)`, 0-based.
    #[inline]
    pub fn at(&self, row: usize, col: usize) -> f64 {
        self.rows[row][col]
    }

    /// `self · v`.
    #[inline]
    pub fn multiply_xy(&self, v: &Xy) -> Xy {
        Xy::new(
            self.rows[0][0] * v.x() + self.rows[0][1] * v.y(),
            self.rows[1][0] * v.x() + self.rows[1][1] * v.y(),
        )
    }

    /// The determinant.
    #[inline]
    pub fn determinant(&self) -> f64 {
        self.rows[0][0] * self.rows[1][1] - self.rows[0][1] * self.rows[1][0]
    }
}

impl Default for Mat2d {
    #[inline]
    fn default() -> Self {
        Self::IDENTITY
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_is_orthonormal_and_own_inverse() {
        assert!(Mat::IDENTITY.is_orthonormal(1e-9));
        assert_eq!(Mat::IDENTITY.inverted().unwrap(), Mat::IDENTITY);
    }

    #[test]
    fn multiply_then_inverse_round_trips() {
        let m = Mat::from_rows([[2.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);
        let inv = m.inverted().unwrap();
        assert_eq!(m.multiplied(&inv), Mat::IDENTITY);
        assert_eq!(inv.at(0, 0), 0.5);
    }

    #[test]
    fn singular_has_no_inverse() {
        let m = Mat::from_rows([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]);
        assert!(m.inverted().is_none());
    }

    #[test]
    fn rotation_matrix_is_orthonormal() {
        // 90° about Z, from Rodrigues (computed in trsf too).
        let r = Mat::from_rows([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]);
        assert!(r.is_orthonormal(1e-9));
        assert!((r.determinant() - 1.0).abs() < 1e-9);
        assert_eq!(
            r.multiply_xyz(&Xyz::new(1.0, 0.0, 0.0)),
            Xyz::new(0.0, 1.0, 0.0)
        );
    }
}
