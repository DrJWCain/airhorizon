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

/// A boolean "is water here" raster over a British National Grid area, built by
/// rasterising water-body polygons. Lets the horizon cast tell water apart from
/// ordinary low ground (the DEM can't).
pub struct WaterMask {
    e0: f64,
    n0: f64,
    cell: f64,
    nx: usize,
    ny: usize,
    bits: Vec<bool>,
}

impl WaterMask {
    /// Rasterise BNG polygons (rings of [e, n]; holes not handled) into a grid
    /// with south-west corner (e0, n0), `cell` m, `nx`×`ny` cells.
    pub fn from_polygons(
        e0: f64,
        n0: f64,
        cell: f64,
        nx: usize,
        ny: usize,
        polys: &[Vec<[f64; 2]>],
    ) -> Self {
        let mut bits = vec![false; nx * ny];
        for poly in polys {
            if poly.len() < 4 {
                continue;
            }
            let (mut emin, mut emax, mut nmin, mut nmax) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
            for p in poly {
                emin = emin.min(p[0]);
                emax = emax.max(p[0]);
                nmin = nmin.min(p[1]);
                nmax = nmax.max(p[1]);
            }
            let cx0 = (((emin - e0) / cell).floor().max(0.0)) as usize;
            let cy0 = (((nmin - n0) / cell).floor().max(0.0)) as usize;
            let cx1 = (((emax - e0) / cell).ceil().max(0.0) as usize).min(nx);
            let cy1 = (((nmax - n0) / cell).ceil().max(0.0) as usize).min(ny);
            for cy in cy0..cy1 {
                for cx in cx0..cx1 {
                    let e = e0 + (cx as f64 + 0.5) * cell;
                    let n = n0 + (cy as f64 + 0.5) * cell;
                    if point_in_poly(e, n, poly) {
                        bits[cy * nx + cx] = true;
                    }
                }
            }
        }
        WaterMask { e0, n0, cell, nx, ny, bits }
    }

    /// Stamp a BNG polyline (e.g. a river centreline) into the mask, marking
    /// every cell it passes through plus `buf` cells either side.
    pub fn add_line(&mut self, line: &[[f64; 2]], buf: i64) {
        for w in line.windows(2) {
            let (ax, ay) = (w[0][0], w[0][1]);
            let (bx, by) = (w[1][0], w[1][1]);
            let len = (bx - ax).hypot(by - ay).max(1.0);
            let steps = (len / (self.cell * 0.5)).ceil() as i64;
            for s in 0..=steps {
                let t = s as f64 / steps as f64;
                let cx = (((ax + (bx - ax) * t) - self.e0) / self.cell).floor() as i64;
                let cy = (((ay + (by - ay) * t) - self.n0) / self.cell).floor() as i64;
                for dy in -buf..=buf {
                    for dx in -buf..=buf {
                        let (gx, gy) = (cx + dx, cy + dy);
                        if gx >= 0 && gy >= 0 && (gx as usize) < self.nx && (gy as usize) < self.ny {
                            self.bits[gy as usize * self.nx + gx as usize] = true;
                        }
                    }
                }
            }
        }
    }

    pub fn is_water(&self, e: f64, n: f64) -> bool {
        let cx = ((e - self.e0) / self.cell).floor();
        let cy = ((n - self.n0) / self.cell).floor();
        if cx < 0.0 || cy < 0.0 {
            return false;
        }
        let (cx, cy) = (cx as usize, cy as usize);
        cx < self.nx && cy < self.ny && self.bits[cy * self.nx + cx]
    }
}

fn point_in_poly(e: f64, n: f64, ring: &[[f64; 2]]) -> bool {
    let mut inside = false;
    let mut j = ring.len() - 1;
    for i in 0..ring.len() {
        let (xi, yi) = (ring[i][0], ring[i][1]);
        let (xj, yj) = (ring[j][0], ring[j][1]);
        if (yi > n) != (yj > n) && e < (xj - xi) * (n - yi) / (yj - yi) + xi {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Per azimuth bucket, the *contiguous segments* of visible water as
/// apparent-elevation bands `(top, bottom)` (radians). Each segment is one
/// stretch of unoccluded water surface along the ray; separate lakes (or a lake
/// then a farther one across intervening land) stay separate segments so the
/// renderer doesn't fill the gap. Water sits below eye level (angle rises with
/// distance); a sample shows when not occluded by nearer, higher terrain.
pub fn visible_water(
    dem: &Dem,
    viewpoint: LatLon,
    params: &HorizonParams,
    mask: &WaterMask,
) -> Vec<Vec<(f32, f32)>> {
    let (eye_e, eye_n) = geodesy::wgs84_to_bng(viewpoint.lat, viewpoint.lon);
    let h_eye = dem.elevation_bng(eye_e, eye_n).unwrap_or(0.0) + params.eye_height_m;
    let r_eff = EARTH_RADIUS_M / (1.0 - params.refraction_k);

    let mut out: Vec<Vec<(f32, f32)>> = vec![Vec::new(); AZIMUTH_BUCKETS];
    for i in 0..AZIMUTH_BUCKETS {
        let az = i as f64 * 0.1 * PI / 180.0;
        let (dx, dy) = (az.sin(), az.cos());
        // Bridge short gaps (occlusion/sampling noise, common when a lake is seen
        // edge-on from near its own level) so a run stays one band, not dashes.
        const MAX_GAP: i32 = 5;
        let mut run_max = -PI / 2.0;
        let mut seg: Option<(f32, f32)> = None; // (top, bottom) of the current run
        let mut gap = 0;
        let mut d = 50.0;
        while d <= params.max_range_m {
            let (e, n) = (eye_e + dx * d, eye_n + dy * d);
            if let Some(h_t) = dem.elevation_bng(e, n) {
                let curve_drop = d * d / (2.0 * r_eff);
                let ang = (((h_t - h_eye - curve_drop) / d).atan()) as f32;
                let here = ang >= run_max as f32 && mask.is_water(e, n);
                if here {
                    seg = Some(match seg {
                        Some((t, b)) => (t.max(ang), b.min(ang)),
                        None => (ang, ang),
                    });
                    gap = 0;
                } else if seg.is_some() {
                    gap += 1;
                    if gap > MAX_GAP {
                        out[i].push(seg.take().unwrap()); // run really ended
                        gap = 0;
                    }
                }
                if ang as f64 > run_max {
                    run_max = ang as f64;
                }
            }
            d += (d * 0.015).clamp(30.0, 600.0);
        }
        if let Some(s) = seg.take() {
            out[i].push(s);
        }
    }
    out
}
