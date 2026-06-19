//! Geometric horizon ray-caster.
//!
//! From a viewpoint, sweep one ray per 0.1° of azimuth out across the DEM and
//! record the greatest *apparent elevation angle* of the terrain along each
//! ray — that maximum is the skyline. Earth curvature and atmospheric
//! refraction are folded into an effective radius `R_eff = R / (1 - k)` (the
//! standard surveyor's approximation; `k ≈ 0.13`).
//!
//! This is the CPU reference implementation (the heart of AirHorizon); a WGSL
//! compute-shader port will follow and must match it within float tolerance.
//!
//! Apparent elevation of terrain height `h_t` at horizontal distance `d`, eye
//! at height `h_e`:  `atan( (h_t − h_e − d²/(2·R_eff)) / d )`.

use std::f64::consts::PI;

use dem::Dem;
use geodesy::LatLon;

/// 3600 buckets = 0.1° resolution over the full 360°.
pub const AZIMUTH_BUCKETS: usize = 3600;

const EARTH_RADIUS_M: f64 = 6_371_000.0;

#[derive(Debug, Clone, Copy)]
pub struct HorizonParams {
    /// Observer eye height above the ground at the viewpoint (m).
    pub eye_height_m: f64,
    /// Maximum ray distance (m).
    pub max_range_m: f64,
    /// Refraction coefficient k (0.13 textbook; ~0.17 hot, 0.0 cold/dense).
    pub refraction_k: f64,
}

impl Default for HorizonParams {
    fn default() -> Self {
        Self { eye_height_m: 1.6, max_range_m: 40_000.0, refraction_k: 0.13 }
    }
}

/// A computed skyline.
pub struct Horizon {
    /// Apparent elevation angle (radians) per 0.1° azimuth bucket, index 0 =
    /// due north, increasing clockwise. Buckets the ray never hit hold the
    /// "looking at nothing" floor of −π/2.
    pub elev_rad: Vec<f32>,
    /// Ground elevation sampled at the viewpoint (m).
    pub eye_ground_m: f64,
}

impl Horizon {
    /// Apparent elevation (radians) for a compass bearing in degrees.
    pub fn at_bearing_deg(&self, deg: f64) -> f32 {
        let i = (deg.rem_euclid(360.0) * 10.0).round() as usize % AZIMUTH_BUCKETS;
        self.elev_rad[i]
    }

    /// The highest skyline point: (bearing °, elevation °).
    pub fn highest(&self) -> (f64, f64) {
        let mut bi = 0usize;
        for i in 1..AZIMUTH_BUCKETS {
            if self.elev_rad[i] > self.elev_rad[bi] {
                bi = i;
            }
        }
        (bi as f64 * 0.1, (self.elev_rad[bi] as f64).to_degrees())
    }
}

/// Cast the horizon from `viewpoint`. Returns `None` if the viewpoint itself is
/// outside DEM coverage.
pub fn cast(dem: &Dem, viewpoint: LatLon, params: &HorizonParams) -> Option<Horizon> {
    let (eye_e, eye_n) = geodesy::wgs84_to_bng(viewpoint.lat, viewpoint.lon);
    let ground = dem.elevation_bng(eye_e, eye_n)?;
    let h_eye = ground + params.eye_height_m;
    let r_eff = EARTH_RADIUS_M / (1.0 - params.refraction_k);

    let mut elev = vec![(-PI / 2.0) as f32; AZIMUTH_BUCKETS];
    for (i, slot) in elev.iter_mut().enumerate() {
        let az = i as f64 * 0.1 * PI / 180.0;
        let (dx, dy) = (az.sin(), az.cos()); // BNG east, north
        let mut max_elev = -PI / 2.0;
        let mut d = 50.0;
        while d <= params.max_range_m {
            let h_t = dem.elevation_bng(eye_e + dx * d, eye_n + dy * d);
            if let Some(h_t) = h_t {
                let curve_drop = d * d / (2.0 * r_eff);
                let elev_angle = ((h_t - h_eye - curve_drop) / d).atan();
                if elev_angle > max_elev {
                    max_elev = elev_angle;
                }
            }
            // Step coarsens with distance: fine near the eye (DEM is 50 m), up
            // to 600 m far out where the skyline subtends a tiny angle anyway.
            d += (d * 0.015).clamp(30.0, 600.0);
        }
        *slot = max_elev as f32;
    }

    Some(Horizon { elev_rad: elev, eye_ground_m: ground })
}
