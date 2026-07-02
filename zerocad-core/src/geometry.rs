#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    pub const X: Self = Self {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    };
    pub const Y: Self = Self {
        x: 0.0,
        y: 1.0,
        z: 0.0,
    };
    pub const Z: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 1.0,
    };

    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub fn add(self, other: Self) -> Self {
        Self::new(self.x + other.x, self.y + other.y, self.z + other.z)
    }

    pub fn sub(self, other: Self) -> Self {
        Self::new(self.x - other.x, self.y - other.y, self.z - other.z)
    }

    pub fn mul(self, scalar: f32) -> Self {
        Self::new(self.x * scalar, self.y * scalar, self.z * scalar)
    }

    pub fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn cross(self, other: Self) -> Self {
        Self::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }

    pub fn length(self) -> f32 {
        self.dot(self).sqrt()
    }

    pub fn normalize(self) -> Self {
        let len = self.length();
        if len > 0.0 {
            self.mul(1.0 / len)
        } else {
            Self::ZERO
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum SketchPlane {
    XY,
    XZ,
    YZ,
}

/// Represents a 3D coordinate system (Local Plane) for 2D Sketching.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CoordinateSystem {
    pub origin: Vec3,
    pub u: Vec3, // Horizontal local axis
    pub v: Vec3, // Vertical local axis
    pub n: Vec3, // Normal axis (U x V)
}

impl CoordinateSystem {
    pub const XY: Self = Self {
        origin: Vec3::ZERO,
        u: Vec3::X,
        v: Vec3::Y,
        n: Vec3::Z,
    };

    pub const XZ: Self = Self {
        origin: Vec3::ZERO,
        u: Vec3::X,
        v: Vec3::Z,
        n: Vec3::Y,
    };

    pub const YZ: Self = Self {
        origin: Vec3::ZERO,
        u: Vec3::Y,
        v: Vec3::Z,
        n: Vec3::X,
    };

    pub fn new(origin: Vec3, u: Vec3, v: Vec3) -> Self {
        let u = u.normalize();
        let v = v.normalize();
        let n = u.cross(v).normalize();
        // Zero-length or parallel `u`/`v` collapse the normal to zero, which
        // would silently turn every projected/unprojected coordinate into
        // garbage. Fall back to the canonical XY frame (keeping the requested
        // origin) and flag it instead of building a corrupt plane.
        if n == Vec3::ZERO {
            log::warn!(
                "CoordinateSystem::new: degenerate axes (u={u:?}, v={v:?}); \
                 falling back to the XY plane at {origin:?}"
            );
            return Self { origin, ..Self::XY };
        }
        Self { origin, u, v, n }
    }

    /// The same plane frame relocated to `origin`, KEEPING the stored axes.
    ///
    /// Never rebuild a shifted frame with [`CoordinateSystem::new`]: that
    /// recomputes `n = u × v`, and the ground/top plane constant ([`Self::XZ`])
    /// is LEFT-handed (stored `n = +Y` but `u × v = −Y`) — the recompute flips
    /// the sweep direction, placing extruded tools on the wrong side of the
    /// plane (a cutter that misses the body entirely).
    pub fn with_origin(&self, origin: Vec3) -> Self {
        Self { origin, ..*self }
    }

    /// Project a 3D global coordinate onto this 2D local plane coordinate.
    pub fn project(&self, point_3d: Vec3) -> (f32, f32) {
        let diff = point_3d.sub(self.origin);
        let u_coord = diff.dot(self.u);
        let v_coord = diff.dot(self.v);
        (u_coord, v_coord)
    }

    /// Unproject a 2D local plane coordinate back to a 3D global coordinate.
    pub fn unproject(&self, u_coord: f32, v_coord: f32) -> Vec3 {
        self.origin
            .add(self.u.mul(u_coord))
            .add(self.v.mul(v_coord))
    }
}
