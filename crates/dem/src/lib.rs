//! UK Digital Elevation Model: OS Terrain 50 ASCII Grid lookup.
//!
//! Ported from qct-viewer's `dem` crate, with two changes for AirHorizon:
//! BNG conversion comes from the shared [`geodesy`] crate (not a local Helmert
//! copy), and [`Dem::elevation_bng`] samples directly by easting/northing so the
//! horizon ray-caster — which samples millions of points already in BNG — skips
//! the per-sample lat/lon conversion. The tile cache is large enough that a
//! wide horizon cast doesn't thrash (a 75 km cast spans ~200 of the 10 km tiles).

pub mod asc;
pub mod lidar;

use geodesy::LatLon;
use std::cell::RefCell;
use std::collections::VecDeque;
use std::path::{Path, PathBuf};
use walkdir::WalkDir;

pub use asc::{AscHeader, AscTile};
pub use lidar::Lidar;

#[derive(thiserror::Error, Debug)]
pub enum DemError {
    #[error("io: {0}")]
    Io(#[from] std::io::Error),
    #[error("dem directory does not exist: {0}")]
    DirMissing(PathBuf),
    #[error("no .asc files found under {0}")]
    Empty(PathBuf),
}

/// A directory of OS Terrain 50 `.asc` tiles, indexed by their south-west
/// corner so a BNG point routes to the right tile in O(1).
pub struct Dem {
    tiles: std::collections::HashMap<(i32, i32), PathBuf>,
    cache: RefCell<TileCache>,
    /// Optional 1 m LIDAR overlay, sampled in preference to Terrain 50 where it
    /// has coverage (sharpens near crags/cliffs around Wasdale).
    lidar: Option<Lidar>,
}

struct TileCache {
    cap: usize,
    order: VecDeque<(i32, i32)>,
    by_key: std::collections::HashMap<(i32, i32), AscTile>,
}

impl TileCache {
    fn new(cap: usize) -> Self {
        Self { cap, order: VecDeque::new(), by_key: std::collections::HashMap::new() }
    }

    fn get_or_load(&mut self, key: (i32, i32), path: &Path) -> std::io::Result<&AscTile> {
        if !self.by_key.contains_key(&key) {
            let tile = asc::read_tile(path)?;
            if self.by_key.len() >= self.cap {
                if let Some(old) = self.order.pop_front() {
                    self.by_key.remove(&old);
                }
            }
            self.by_key.insert(key, tile);
            self.order.push_back(key);
        }
        Ok(self.by_key.get(&key).unwrap())
    }
}

/// OS Terrain 50 is a 10 km grid.
const TILE_SIZE_M: f64 = 10_000.0;

impl Dem {
    /// Walk `dir`, index every `.asc` tile by its SW corner. Tile bodies load
    /// lazily as queries touch them.
    pub fn open(dir: &Path) -> Result<Self, DemError> {
        if !dir.exists() {
            return Err(DemError::DirMissing(dir.to_path_buf()));
        }
        let mut tiles = std::collections::HashMap::new();
        for entry in WalkDir::new(dir).follow_links(false) {
            let Ok(entry) = entry else { continue };
            if !entry.file_type().is_file() {
                continue;
            }
            let is_asc = entry
                .path()
                .extension()
                .and_then(|e| e.to_str())
                .map(|e| e.eq_ignore_ascii_case("asc"))
                .unwrap_or(false);
            if !is_asc {
                continue;
            }
            match asc::read_header(entry.path()) {
                Ok(h) => {
                    tiles.insert(tile_key(h.xllcorner, h.yllcorner), entry.path().to_path_buf());
                }
                Err(e) => eprintln!("dem: skipping {} ({e})", entry.path().display()),
            }
        }
        if tiles.is_empty() {
            return Err(DemError::Empty(dir.to_path_buf()));
        }
        // Large cache: a wide horizon cast revisits ~200 tiles many times.
        Ok(Dem { tiles, cache: RefCell::new(TileCache::new(1024)), lidar: None })
    }

    /// Attach a 1 m LIDAR overlay (e.g. `data/lidar`). Sampled in preference to
    /// Terrain 50 wherever it has coverage. No-op if the directory is absent or
    /// empty. Returns the number of LIDAR tiles indexed.
    pub fn attach_lidar(&mut self, dir: &Path) -> usize {
        self.lidar = Lidar::open(dir);
        self.lidar.as_ref().map(|l| l.tile_count()).unwrap_or(0)
    }

    /// Pre-warm the LIDAR overlay around a BNG point (parallel decode + disk
    /// cache) so the first cast there doesn't stall. No-op without LIDAR.
    pub fn preload_lidar_around(&self, e: f64, n: f64, radius: f64) {
        if let Some(lidar) = &self.lidar {
            lidar.preload_around(e, n, radius);
        }
    }

    /// True if a 1 m LIDAR overlay is attached (gills/ravines are only worth
    /// detecting at LIDAR resolution).
    pub fn has_lidar(&self) -> bool {
        self.lidar.is_some()
    }

    pub fn tile_count(&self) -> usize {
        self.tiles.len()
    }

    /// Elevation (m) at a BNG easting/northing, or None outside coverage / on
    /// NODATA. The ray-caster's hot path.
    pub fn elevation_bng(&self, e: f64, n: f64) -> Option<f64> {
        // Prefer 1 m LIDAR where it covers the point; fall back to Terrain 50.
        if let Some(lidar) = &self.lidar {
            if let Some(v) = lidar.elevation_bng(e, n) {
                return Some(v);
            }
        }
        let key = tile_key(e, n);
        let path = self.tiles.get(&key)?;
        let mut cache = self.cache.borrow_mut();
        let parsed = cache.get_or_load(key, path).ok()?;
        parsed.sample_bilinear(e, n)
    }

    /// Elevation (m) at a WGS-84 lat/lon, or None outside coverage.
    pub fn elevation_m(&self, ll: LatLon) -> Option<f64> {
        let (e, n) = geodesy::wgs84_to_bng(ll.lat, ll.lon);
        self.elevation_bng(e, n)
    }
}

fn tile_key(xll: f64, yll: f64) -> (i32, i32) {
    ((xll / TILE_SIZE_M).floor() as i32, (yll / TILE_SIZE_M).floor() as i32)
}
