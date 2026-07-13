//! Minimal dense linear algebra for the sketch constraint solver.
//!
//! Deliberately hand-rolled: sketches are dozens of unknowns, so a dense
//! Cholesky solve and a column-pivoted QR rank are microseconds — far below
//! the threshold where an external linear-algebra dependency would earn its
//! place (the crate's rule is serde-only). Everything is `f64` and
//! deterministic: fixed iteration order, no randomness, no parallelism.

/// Dense row-major matrix.
#[derive(Debug, Clone)]
pub struct Mat {
    pub rows: usize,
    pub cols: usize,
    pub data: Vec<f64>,
}

impl Mat {
    pub fn zeros(rows: usize, cols: usize) -> Self {
        Self {
            rows,
            cols,
            data: vec![0.0; rows * cols],
        }
    }

    #[inline]
    pub fn at(&self, r: usize, c: usize) -> f64 {
        self.data[r * self.cols + c]
    }

    #[inline]
    pub fn set(&mut self, r: usize, c: usize, v: f64) {
        self.data[r * self.cols + c] = v;
    }

    #[inline]
    pub fn add_at(&mut self, r: usize, c: usize, v: f64) {
        self.data[r * self.cols + c] += v;
    }

    /// `Aᵀ·A` (n×n, symmetric positive semi-definite).
    pub fn ata(&self) -> Mat {
        let n = self.cols;
        let mut out = Mat::zeros(n, n);
        for i in 0..n {
            for j in i..n {
                let mut s = 0.0;
                for r in 0..self.rows {
                    s += self.at(r, i) * self.at(r, j);
                }
                out.set(i, j, s);
                out.set(j, i, s);
            }
        }
        out
    }

    /// `Aᵀ·v` (length n).
    pub fn atv(&self, v: &[f64]) -> Vec<f64> {
        let mut out = vec![0.0; self.cols];
        for r in 0..self.rows {
            let vr = v[r];
            for c in 0..self.cols {
                out[c] += self.at(r, c) * vr;
            }
        }
        out
    }
}

/// Solve `A·x = b` for symmetric positive-definite `A` via Cholesky
/// (`A = L·Lᵀ`). Returns `None` when `A` is not positive definite (a
/// singular/indefinite normal system — the caller raises damping).
pub fn cholesky_solve(a: &Mat, b: &[f64]) -> Option<Vec<f64>> {
    let n = a.rows;
    debug_assert_eq!(a.cols, n);
    debug_assert_eq!(b.len(), n);
    let mut l = vec![0.0f64; n * n];
    for i in 0..n {
        for j in 0..=i {
            let mut s = a.at(i, j);
            for k in 0..j {
                s -= l[i * n + k] * l[j * n + k];
            }
            if i == j {
                if s <= 0.0 || !s.is_finite() {
                    return None;
                }
                l[i * n + i] = s.sqrt();
            } else {
                l[i * n + j] = s / l[j * n + j];
            }
        }
    }
    // Forward substitution L·y = b.
    let mut y = vec![0.0f64; n];
    for i in 0..n {
        let mut s = b[i];
        for k in 0..i {
            s -= l[i * n + k] * y[k];
        }
        y[i] = s / l[i * n + i];
    }
    // Back substitution Lᵀ·x = y.
    let mut x = vec![0.0f64; n];
    for i in (0..n).rev() {
        let mut s = y[i];
        for k in i + 1..n {
            s -= l[k * n + i] * x[k];
        }
        x[i] = s / l[i * n + i];
    }
    Some(x)
}

/// Numerical rank of `a` via column-pivoted Householder QR: the number of
/// diagonal R entries above `tol` relative to the largest. This is the DOF
/// analysis workhorse — `dof = n_params − rank(J)`.
pub fn qr_rank(a: &Mat, tol: f64) -> usize {
    let m = a.rows;
    let n = a.cols;
    if m == 0 || n == 0 {
        return 0;
    }
    let mut r = a.clone();
    let kmax = m.min(n);
    let mut perm: Vec<usize> = (0..n).collect();
    let mut diag = Vec::with_capacity(kmax);

    for k in 0..kmax {
        // Pivot: move the column with the largest remaining norm to position k.
        let mut best = k;
        let mut best_norm = 0.0;
        for c in k..n {
            let mut s = 0.0;
            for row in k..m {
                let v = r.at(row, c);
                s += v * v;
            }
            if s > best_norm {
                best_norm = s;
                best = c;
            }
        }
        if best != k {
            for row in 0..m {
                let tmp = r.at(row, k);
                r.set(row, k, r.at(row, best));
                r.set(row, best, tmp);
            }
            perm.swap(k, best);
        }
        // Householder reflector for column k.
        let mut norm = 0.0;
        for row in k..m {
            norm += r.at(row, k) * r.at(row, k);
        }
        let norm = norm.sqrt();
        if norm == 0.0 {
            diag.push(0.0);
            continue;
        }
        let alpha = if r.at(k, k) >= 0.0 { -norm } else { norm };
        let mut v = vec![0.0f64; m - k];
        v[0] = r.at(k, k) - alpha;
        for (i, item) in v.iter_mut().enumerate().skip(1) {
            *item = r.at(k + i, k);
        }
        let vnorm2: f64 = v.iter().map(|x| x * x).sum();
        if vnorm2 > 0.0 {
            for c in k..n {
                let mut dot = 0.0;
                for (i, &vi) in v.iter().enumerate() {
                    dot += vi * r.at(k + i, c);
                }
                let scale = 2.0 * dot / vnorm2;
                for (i, &vi) in v.iter().enumerate() {
                    r.add_at(k + i, c, -scale * vi);
                }
            }
        }
        diag.push(r.at(k, k).abs());
    }

    let max_diag = diag.iter().cloned().fold(0.0f64, f64::max);
    if max_diag == 0.0 {
        return 0;
    }
    diag.iter().filter(|&&d| d > tol * max_diag).count()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cholesky_solves_a_known_spd_system() {
        // A = [[4,2],[2,3]], b = [10, 9] -> x = [1.5, 2].
        let mut a = Mat::zeros(2, 2);
        a.set(0, 0, 4.0);
        a.set(0, 1, 2.0);
        a.set(1, 0, 2.0);
        a.set(1, 1, 3.0);
        let x = cholesky_solve(&a, &[10.0, 9.0]).expect("SPD system");
        assert!(
            (x[0] - 1.5).abs() < 1e-12 && (x[1] - 2.0).abs() < 1e-12,
            "{x:?}"
        );
    }

    #[test]
    fn cholesky_rejects_singular_systems() {
        let mut a = Mat::zeros(2, 2);
        a.set(0, 0, 1.0);
        a.set(0, 1, 1.0);
        a.set(1, 0, 1.0);
        a.set(1, 1, 1.0);
        assert!(cholesky_solve(&a, &[1.0, 1.0]).is_none());
    }

    #[test]
    fn qr_rank_detects_dependent_columns() {
        // Third column = col0 + col1 -> rank 2.
        let mut a = Mat::zeros(4, 3);
        for r in 0..4 {
            let c0 = (r + 1) as f64;
            let c1 = (r * r) as f64 + 1.0;
            a.set(r, 0, c0);
            a.set(r, 1, c1);
            a.set(r, 2, c0 + c1);
        }
        assert_eq!(qr_rank(&a, 1e-10), 2);
        // Full-rank identity-ish.
        let mut b = Mat::zeros(3, 3);
        for i in 0..3 {
            b.set(i, i, 1.0);
        }
        assert_eq!(qr_rank(&b, 1e-10), 3);
        // Zero matrix.
        assert_eq!(qr_rank(&Mat::zeros(3, 3), 1e-10), 0);
    }

    #[test]
    fn ata_and_atv_match_hand_computation() {
        // A = [[1,2],[3,4],[5,6]]
        let mut a = Mat::zeros(3, 2);
        a.set(0, 0, 1.0);
        a.set(0, 1, 2.0);
        a.set(1, 0, 3.0);
        a.set(1, 1, 4.0);
        a.set(2, 0, 5.0);
        a.set(2, 1, 6.0);
        let ata = a.ata();
        assert_eq!(
            [ata.at(0, 0), ata.at(0, 1), ata.at(1, 0), ata.at(1, 1)],
            [35.0, 44.0, 44.0, 56.0]
        );
        assert_eq!(a.atv(&[1.0, 1.0, 1.0]), vec![9.0, 12.0]);
    }
}
