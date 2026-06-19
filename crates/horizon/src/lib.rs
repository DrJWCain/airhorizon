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
use peaks::{Peak, Peaks};

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
    /// Horizontal distance (m) at which each bucket's skyline point sits — i.e.
    /// how far away the ridge forming the horizon in that direction is.
    pub dist_m: Vec<f32>,
    /// Per azimuth bucket: (elevation radians, distance m) of *occlusion edges*
    /// below the skyline — nearer ridge crests superseded by a much farther,
    /// higher fell behind them (Wainwright's "edge of one fell reveals the next").
    pub edges: Vec<Vec<(f32, f32)>>,
    /// Ground elevation sampled at the viewpoint (m).
    pub eye_ground_m: f64,
}

impl Horizon {
    /// Apparent elevation (radians) for a compass bearing in degrees.
    pub fn at_bearing_deg(&self, deg: f64) -> f32 {
        self.elev_rad[Self::bucket(deg)]
    }

    /// Distance (m) of the skyline ridge at a compass bearing.
    pub fn dist_at_bearing_deg(&self, deg: f64) -> f32 {
        self.dist_m[Self::bucket(deg)]
    }

    fn bucket(deg: f64) -> usize {
        (deg.rem_euclid(360.0) * 10.0).round() as usize % AZIMUTH_BUCKETS
    }

    /// Trace the per-azimuth occlusion edge points into polylines by greedily
    /// chaining edges that continue (similar elevation and reveal-distance) into
    /// the next azimuth. Returns lines of `(azimuth_deg, elev_rad)`; short noisy
    /// chains are dropped.
    pub fn edge_polylines(&self) -> Vec<Vec<(f32, f32)>> {
        struct Chain {
            pts: Vec<(f32, f32)>, // (az_deg, elev_rad)
            elev: f32,
            dist: f32,
            last_az: i32,
        }
        const ELEV_TOL: f32 = 0.015; // rad (~0.86°) allowed step between azimuths
        const DIST_TOL_FRAC: f32 = 0.30;
        const MAX_AZ_GAP: i32 = 4; // buckets (0.4°)
        const MIN_LEN: usize = 3;

        let mut chains: Vec<Chain> = Vec::new();
        let mut done: Vec<Vec<(f32, f32)>> = Vec::new();

        for b in 0..AZIMUTH_BUCKETS {
            let az = b as f32 * 0.1;
            // Only match against chains that existed before this bucket (new ones
            // appended below mustn't be matched again, or re-indexed, this pass).
            let n0 = chains.len();
            let mut used = vec![false; n0];
            for &(elev, dist) in &self.edges[b] {
                // Best matching active chain.
                let mut best: Option<usize> = None;
                let mut best_score = f32::MAX;
                for (ci, ch) in chains.iter().enumerate().take(n0) {
                    if used[ci] || b as i32 - ch.last_az > MAX_AZ_GAP {
                        continue;
                    }
                    let de = (ch.elev - elev).abs();
                    let dd = (ch.dist - dist).abs() / ch.dist.max(1.0);
                    if de < ELEV_TOL && dd < DIST_TOL_FRAC {
                        let score = de / ELEV_TOL + dd / DIST_TOL_FRAC;
                        if score < best_score {
                            best_score = score;
                            best = Some(ci);
                        }
                    }
                }
                match best {
                    Some(ci) => {
                        chains[ci].pts.push((az, elev));
                        chains[ci].elev = elev;
                        chains[ci].dist = dist;
                        chains[ci].last_az = b as i32;
                        used[ci] = true;
                    }
                    None => chains.push(Chain { pts: vec![(az, elev)], elev, dist, last_az: b as i32 }),
                }
            }
            // Retire chains that didn't continue.
            let cutoff = b as i32 - MAX_AZ_GAP;
            let mut i = 0;
            while i < chains.len() {
                if chains[i].last_az < cutoff {
                    let ch = chains.remove(i);
                    if ch.pts.len() >= MIN_LEN {
                        done.push(ch.pts);
                    }
                } else {
                    i += 1;
                }
            }
        }
        for ch in chains {
            if ch.pts.len() >= MIN_LEN {
                done.push(ch.pts);
            }
        }
        done
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

    // A near ridge crest is recorded as an occlusion edge when it's superseded
    // by a higher crest at least this much farther out (you see past it to a
    // distant fell). Tune for how "Wainwright" the edges look.
    const EDGE_DEPTH_JUMP_M: f64 = 2000.0;

    let mut elev = vec![(-PI / 2.0) as f32; AZIMUTH_BUCKETS];
    let mut dist = vec![0.0f32; AZIMUTH_BUCKETS];
    let mut edges: Vec<Vec<(f32, f32)>> = vec![Vec::new(); AZIMUTH_BUCKETS];
    for i in 0..AZIMUTH_BUCKETS {
        let az = i as f64 * 0.1 * PI / 180.0;
        let (dx, dy) = (az.sin(), az.cos()); // BNG east, north
        let mut max_elev = -PI / 2.0;
        let mut best_d = 0.0;
        let mut d = 50.0;
        while d <= params.max_range_m {
            let h_t = dem.elevation_bng(eye_e + dx * d, eye_n + dy * d);
            if let Some(h_t) = h_t {
                let curve_drop = d * d / (2.0 * r_eff);
                let elev_angle = ((h_t - h_eye - curve_drop) / d).atan();
                if elev_angle > max_elev {
                    // The crest we're leaving behind is a visible edge if the new,
                    // higher crest sits well beyond it.
                    if best_d > 0.0 && d - best_d > EDGE_DEPTH_JUMP_M {
                        edges[i].push((max_elev as f32, best_d as f32));
                    }
                    max_elev = elev_angle;
                    best_d = d;
                }
            }
            // Step coarsens with distance: fine near the eye (DEM is 50 m), up
            // to 600 m far out where the skyline subtends a tiny angle anyway.
            d += (d * 0.015).clamp(30.0, 600.0);
        }
        elev[i] = max_elev as f32;
        dist[i] = best_d as f32;
    }

    Some(Horizon { elev_rad: elev, dist_m: dist, edges, eye_ground_m: ground })
}

/// How much of a fell can be seen from the viewpoint.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Visibility {
    /// The summit itself clears the skyline.
    Summit,
    /// The summit is hidden behind a nearer ridge, but that ridge sits close to
    /// the peak (its own shoulder) — so the fell's slopes are in view.
    Slopes,
}

/// A peak that is at least partly visible from the viewpoint.
#[derive(Debug, Clone)]
pub struct VisiblePeak<'a> {
    pub peak: &'a Peak,
    /// Compass bearing to the peak (degrees, 0 = north, clockwise).
    pub bearing_deg: f64,
    /// Apparent elevation angle of the summit (degrees; may be negative).
    pub elev_deg: f64,
    /// Horizontal distance (m).
    pub dist_m: f64,
    pub visibility: Visibility,
}

/// Which DoBIH summits within range are visible above the cast skyline.
///
/// A peak is visible when its own apparent elevation (using its DoBIH summit
/// height) reaches the horizon at its bearing — i.e. it isn't hidden behind a
/// nearer, higher ridge. A small tolerance absorbs DEM-vs-DoBIH height
/// differences and the 0.1° bearing quantisation. Must use the same eye height
/// and refraction as the `horizon` was cast with.
pub fn visible_peaks<'a>(
    horizon: &Horizon,
    viewpoint: LatLon,
    peaks: &'a Peaks,
    params: &HorizonParams,
) -> Vec<VisiblePeak<'a>> {
    let (eye_e, eye_n) = geodesy::wgs84_to_bng(viewpoint.lat, viewpoint.lon);
    let h_eye = horizon.eye_ground_m + params.eye_height_m;
    let r_eff = EARTH_RADIUS_M / (1.0 - params.refraction_k);
    let tol = 0.1_f64.to_radians();

    // An obscured summit still counts as "slopes visible" when the ridge hiding
    // it sits within this distance of the summit — i.e. it's the fell's own near
    // shoulder, so its slopes are in view. A *separate* ridge blocking from much
    // farther forward hides the whole fell, so that's dropped. Using the absolute
    // gap (not a ratio) keeps distant fells hidden when a foreground ridge blocks
    // them, while still catching near fells like Lingmell or Yewbarrow.
    const MASSIF_GAP_M: f64 = 1500.0;

    let mut out = Vec::new();
    for pk in peaks.within_range(viewpoint.lat, viewpoint.lon, params.max_range_m) {
        let (pe, pn) = geodesy::wgs84_to_bng(pk.lat, pk.lon);
        let (de, dn) = (pe - eye_e, pn - eye_n);
        let dist = (de * de + dn * dn).sqrt();
        if dist < 30.0 {
            continue; // the viewpoint's own summit
        }
        let bearing = de.atan2(dn).to_degrees().rem_euclid(360.0);
        let curve_drop = dist * dist / (2.0 * r_eff);
        let elev = ((pk.height_m - h_eye - curve_drop) / dist).atan();
        let skyline = horizon.at_bearing_deg(bearing) as f64;

        let visibility = if elev + tol >= skyline {
            Visibility::Summit
        } else if dist - horizon.dist_at_bearing_deg(bearing) as f64 <= MASSIF_GAP_M {
            Visibility::Slopes // summit just behind the fell's own near shoulder
        } else {
            continue; // fully hidden behind a separate, nearer fell
        };
        out.push(VisiblePeak {
            peak: pk,
            bearing_deg: bearing,
            elev_deg: elev.to_degrees(),
            dist_m: dist,
            visibility,
        });
    }
    out
}
