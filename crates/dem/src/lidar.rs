//! 1 m LIDAR Composite DTM overlay (Environment Agency / DEFRA).
//!
//! Each source file is a 5 km × 5 km GeoTIFF of float32 heights at 1 m spacing,
//! shipped zipped (with an ESRI world file `.tfw` for georeferencing). We read
//! the TIFF straight out of the zip on demand — no extraction — and decimate to
//! [`DECIMATE`] m on load so a cast over the Wasdale tiles stays within a sane
//! memory budget. Tiles are handed out as [`AscTile`]s so the sampler and the
//! ray-caster reuse the exact bilinear path used for OS Terrain 50.

use crate::asc::{AscHeader, AscTile};
use std::cell::RefCell;
use std::collections::{HashMap, VecDeque};
use std::io::{self, Read};
use std::path::{Path, PathBuf};
use walkdir::WalkDir;

/// DEFRA LIDAR composite tiles are 5 km squares.
const TILE_M: f64 = 5_000.0;
/// Stored resolution (m). 1 m source decimated by this factor to bound RAM;
/// 2 m is still far finer than the 50 m base and resolves crags/cliffs well.
const DECIMATE: usize = 2;
const NODATA: f64 = -9999.0;
/// Magic for the decoded-grid disk cache (so re-launches skip TIFF/zip decode).
const CACHE_MAGIC: &[u8; 8] = b"AHLDTM01";
/// Max LIDAR tiles decoded concurrently when pre-warming (bounds memory spike).
const PRELOAD_CONCURRENCY: usize = 6;

/// One indexed tile: where its raster lives and its georeferencing (from `.tfw`).
struct Entry {
    zip: PathBuf,
    tif: String,
    xll: f64,       // SW corner easting
    north_top: f64, // north edge northing
}

/// A directory of zipped LIDAR tiles, indexed by 5 km cell. Raster bodies load
/// (and decimate) lazily on first touch and are cached LRU.
pub struct Lidar {
    tiles: HashMap<(i32, i32), Entry>,
    cache: RefCell<Cache>,
}

struct Cache {
    cap: usize,
    order: VecDeque<(i32, i32)>,
    by_key: HashMap<(i32, i32), AscTile>,
}

impl Cache {
    fn new(cap: usize) -> Self {
        Self { cap, order: VecDeque::new(), by_key: HashMap::new() }
    }
}

impl Lidar {
    /// Index every `*.zip` LIDAR tile under `dir` by reading its `.tfw`. Returns
    /// `None` if the directory is absent or holds no readable tiles, so callers
    /// can treat LIDAR as simply unavailable.
    pub fn open(dir: &Path) -> Option<Lidar> {
        if !dir.exists() {
            return None;
        }
        let mut tiles = HashMap::new();
        for entry in WalkDir::new(dir).follow_links(false) {
            let Ok(entry) = entry else { continue };
            if !entry.file_type().is_file() {
                continue;
            }
            let is_zip = entry
                .path()
                .extension()
                .and_then(|e| e.to_str())
                .map(|e| e.eq_ignore_ascii_case("zip"))
                .unwrap_or(false);
            if !is_zip {
                continue;
            }
            match index_zip(entry.path()) {
                Ok((key, e)) => {
                    tiles.insert(key, e);
                }
                Err(err) => eprintln!("lidar: skipping {} ({err})", entry.path().display()),
            }
        }
        if tiles.is_empty() {
            return None;
        }
        // Hold all the Wasdale-area tiles a cast can sweep without eviction.
        Some(Lidar { tiles, cache: RefCell::new(Cache::new(16)) })
    }

    pub fn tile_count(&self) -> usize {
        self.tiles.len()
    }

    /// Decode (in parallel) every indexed tile within `radius` of a BNG point
    /// that isn't already cached, so the first cast doesn't stall loading them
    /// one by one. Decoded grids are also written to a disk cache, making later
    /// launches near-instant. No-op away from LIDAR coverage.
    pub fn preload_around(&self, e: f64, n: f64, radius: f64) {
        let want: Vec<(i32, i32)> = {
            let cache = self.cache.borrow();
            self.tiles
                .keys()
                .copied()
                .filter(|k| {
                    let cx = (k.0 as f64 + 0.5) * TILE_M;
                    let cy = (k.1 as f64 + 0.5) * TILE_M;
                    (cx - e).hypot(cy - n) <= radius + TILE_M && !cache.by_key.contains_key(k)
                })
                .collect()
        };
        if want.is_empty() {
            return;
        }
        for chunk in want.chunks(PRELOAD_CONCURRENCY) {
            let decoded: Vec<((i32, i32), io::Result<AscTile>)> = std::thread::scope(|s| {
                let handles: Vec<_> = chunk
                    .iter()
                    .map(|&k| {
                        let entry = &self.tiles[&k];
                        s.spawn(move || (k, load_tile(entry)))
                    })
                    .collect();
                handles.into_iter().map(|h| h.join().unwrap()).collect()
            });
            let mut cache = self.cache.borrow_mut();
            for (k, res) in decoded {
                match res {
                    Ok(tile) => {
                        if cache.by_key.len() >= cache.cap {
                            if let Some(old) = cache.order.pop_front() {
                                cache.by_key.remove(&old);
                            }
                        }
                        cache.by_key.insert(k, tile);
                        cache.order.push_back(k);
                    }
                    Err(e) => eprintln!("lidar: preload tile {k:?} failed ({e})"),
                }
            }
        }
    }

    /// Bilinear elevation at a BNG point if a LIDAR tile covers it, else None.
    pub fn elevation_bng(&self, e: f64, n: f64) -> Option<f64> {
        let key = (
            (e / TILE_M).floor() as i32,
            (n / TILE_M).floor() as i32,
        );
        let entry = self.tiles.get(&key)?;
        let mut cache = self.cache.borrow_mut();
        if !cache.by_key.contains_key(&key) {
            let tile = load_tile(entry).ok()?;
            if cache.by_key.len() >= cache.cap {
                if let Some(old) = cache.order.pop_front() {
                    cache.by_key.remove(&old);
                }
            }
            cache.by_key.insert(key, tile);
            cache.order.push_back(key);
        }
        cache.by_key.get(&key).unwrap().sample_bilinear(e, n)
    }
}

/// Read just the `.tfw` from a tile zip to learn its footprint and key.
fn index_zip(path: &Path) -> io::Result<((i32, i32), Entry)> {
    let file = std::fs::File::open(path)?;
    let mut ar = zip::ZipArchive::new(file).map_err(zip_err)?;
    let (mut tif, mut tfw) = (None, None);
    for i in 0..ar.len() {
        let f = ar.by_index(i).map_err(zip_err)?;
        let name = f.name().to_string();
        let lower = name.to_ascii_lowercase();
        if lower.ends_with(".tif") || lower.ends_with(".tiff") {
            tif = Some(name);
        } else if lower.ends_with(".tfw") {
            tfw = Some(name);
        }
    }
    let tif = tif.ok_or_else(|| bad("no .tif in zip"))?;
    let tfw = tfw.ok_or_else(|| bad("no .tfw in zip"))?;
    let mut text = String::new();
    ar.by_name(&tfw).map_err(zip_err)?.read_to_string(&mut text)?;
    // World file: x-pixel, y-row, x-col, y-pixel(neg), originX, originY where the
    // origin is the centre of the upper-left pixel.
    let nums: Vec<f64> = text.split_whitespace().filter_map(|t| t.parse().ok()).collect();
    if nums.len() < 6 {
        return Err(bad("short .tfw"));
    }
    let (px, py, ox, oy) = (nums[0], nums[3], nums[4], nums[5]);
    let xll = ox - px / 2.0;
    let north_top = oy - py / 2.0; // py is negative, so this is the top edge
    let key = (
        (xll / TILE_M).floor() as i32,
        ((north_top - TILE_M) / TILE_M).floor() as i32,
    );
    Ok((key, Entry { zip: path.to_path_buf(), tif, xll, north_top }))
}

/// Load a tile: from the fast decoded-grid disk cache if present, otherwise
/// decode the GeoTIFF and write the cache for next time.
fn load_tile(entry: &Entry) -> io::Result<AscTile> {
    let cache = cache_path(entry);
    if let Ok(tile) = read_cache(&cache) {
        return Ok(tile);
    }
    let tile = decode_tif(entry)?;
    if let Err(e) = write_cache(&cache, &tile) {
        eprintln!("lidar: could not write cache {}: {e}", cache.display());
    }
    Ok(tile)
}

/// Decode a tile's GeoTIFF from its zip and decimate it to an [`AscTile`].
fn decode_tif(entry: &Entry) -> io::Result<AscTile> {
    let file = std::fs::File::open(&entry.zip)?;
    let mut ar = zip::ZipArchive::new(file).map_err(zip_err)?;
    let mut buf = Vec::new();
    ar.by_name(&entry.tif).map_err(zip_err)?.read_to_end(&mut buf)?;

    let mut dec = tiff::decoder::Decoder::new(io::Cursor::new(buf)).map_err(tiff_err)?;
    let (w, h) = dec.dimensions().map_err(tiff_err)?;
    let (w, h) = (w as usize, h as usize);
    let src = match dec.read_image().map_err(tiff_err)? {
        tiff::decoder::DecodingResult::F32(v) => v,
        other => return Err(bad(&format!("unexpected LIDAR sample type {other:?}"))),
    };
    if src.len() != w * h {
        return Err(bad("LIDAR raster size mismatch"));
    }

    let ncols = w / DECIMATE;
    let nrows = h / DECIMATE;
    let mut data = Vec::with_capacity(ncols * nrows);
    for r in 0..nrows {
        let sr = r * DECIMATE;
        for c in 0..ncols {
            let v = src[sr * w + c * DECIMATE];
            // Tidy NODATA / spurious values so bilinear treats them as missing.
            data.push(if v.is_finite() && v > -1.0e4 { v } else { NODATA as f32 });
        }
    }
    let cell = DECIMATE as f64; // 1 m source * decimation
    let header = AscHeader {
        ncols,
        nrows,
        xllcorner: entry.xll,
        yllcorner: entry.north_top - nrows as f64 * cell,
        cellsize: cell,
        nodata: NODATA,
    };
    Ok(AscTile { header, data })
}

/// Disk-cache path for a tile's decoded grid (next to its zip).
fn cache_path(entry: &Entry) -> PathBuf {
    let mut p = entry.zip.clone();
    p.set_extension(format!("dec{DECIMATE}.bin"));
    p
}

/// Read a decoded grid from the disk cache. Errors (absent / stale / corrupt)
/// just mean "decode the TIFF instead".
fn read_cache(path: &Path) -> io::Result<AscTile> {
    let bytes = std::fs::read(path)?;
    if bytes.len() < 52 || &bytes[0..8] != CACHE_MAGIC {
        return Err(bad("bad cache header"));
    }
    let u32_at = |o: usize| u32::from_le_bytes(bytes[o..o + 4].try_into().unwrap());
    let f64_at = |o: usize| f64::from_le_bytes(bytes[o..o + 8].try_into().unwrap());
    if u32_at(8) != DECIMATE as u32 {
        return Err(bad("stale cache (decimation changed)"));
    }
    let ncols = u32_at(12) as usize;
    let nrows = u32_at(16) as usize;
    let header = AscHeader {
        ncols,
        nrows,
        xllcorner: f64_at(20),
        yllcorner: f64_at(28),
        cellsize: f64_at(36),
        nodata: f64_at(44),
    };
    let body = &bytes[52..];
    if body.len() != ncols * nrows * 4 {
        return Err(bad("cache body size mismatch"));
    }
    let data = body
        .chunks_exact(4)
        .map(|c| f32::from_le_bytes(c.try_into().unwrap()))
        .collect();
    Ok(AscTile { header, data })
}

/// Write a decoded grid to the disk cache (temp file + rename, best effort).
fn write_cache(path: &Path, tile: &AscTile) -> io::Result<()> {
    let h = &tile.header;
    let mut buf = Vec::with_capacity(52 + tile.data.len() * 4);
    buf.extend_from_slice(CACHE_MAGIC);
    buf.extend_from_slice(&(DECIMATE as u32).to_le_bytes());
    buf.extend_from_slice(&(h.ncols as u32).to_le_bytes());
    buf.extend_from_slice(&(h.nrows as u32).to_le_bytes());
    buf.extend_from_slice(&h.xllcorner.to_le_bytes());
    buf.extend_from_slice(&h.yllcorner.to_le_bytes());
    buf.extend_from_slice(&h.cellsize.to_le_bytes());
    buf.extend_from_slice(&h.nodata.to_le_bytes());
    for v in &tile.data {
        buf.extend_from_slice(&v.to_le_bytes());
    }
    let tmp = path.with_extension("tmp");
    std::fs::write(&tmp, &buf)?;
    std::fs::rename(&tmp, path)?;
    Ok(())
}

fn zip_err(e: zip::result::ZipError) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, format!("zip: {e}"))
}
fn tiff_err(e: tiff::TiffError) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, format!("tiff: {e}"))
}
fn bad(msg: &str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, msg.to_string())
}
