//! Read OS Open Zoomstack (or any vector MBTiles) and decode Mapbox Vector
//! Tiles to typed geometry in tile-local coordinates.
//!
//! MBTiles is a SQLite database: a `tiles(zoom_level, tile_column, tile_row,
//! tile_data)` table plus a `metadata(name, value)` table. Rows are stored
//! TMS-flipped (bottom-up), so we go through [`geodesy::Tile::tms_y`]. Each
//! `tile_data` blob is a gzip-compressed MVT (protobuf).
//!
//! Geometry comes back in tile-local integer space (0..=`extent`, usually 4096);
//! the renderer maps it to Web Mercator via [`geodesy::Tile::mvt_to_mercator`].
//! Like qct-viewer's `qct` crate, this stays GPU-unaware.

use std::io::Read;
use std::path::Path;

use geodesy::Tile;
use prost::Message;
use rusqlite::{Connection, OpenFlags, OptionalExtension};

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("sqlite: {0}")]
    Sqlite(#[from] rusqlite::Error),
    #[error("io: {0}")]
    Io(#[from] std::io::Error),
    #[error("mvt decode: {0}")]
    Decode(#[from] prost::DecodeError),
}

pub type Result<T> = std::result::Result<T, Error>;

/// A read-only handle to an MBTiles archive.
pub struct Mbtiles {
    conn: Connection,
}

/// A few useful `metadata` values (whatever the archive provides).
#[derive(Debug, Default, Clone)]
pub struct Metadata {
    pub name: Option<String>,
    pub format: Option<String>,
    pub minzoom: Option<u8>,
    pub maxzoom: Option<u8>,
    pub bounds: Option<String>,
}

/// Geometry kind of a decoded feature.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GeomKind {
    Point,
    Line,
    Polygon,
    Unknown,
}

impl GeomKind {
    fn from_mvt(t: i32) -> Self {
        match t {
            1 => GeomKind::Point,
            2 => GeomKind::Line,
            3 => GeomKind::Polygon,
            _ => GeomKind::Unknown,
        }
    }
}

/// One feature: its geometry kind and parts. Each "part" is a ring (polygon),
/// a polyline (line), or a run of points, in tile-local coordinates.
#[derive(Debug, Clone)]
pub struct Feature {
    pub kind: GeomKind,
    pub parts: Vec<Vec<[f32; 2]>>,
}

/// One layer's decoded features plus its attribute-key dictionary.
#[derive(Debug, Clone)]
pub struct Layer {
    pub name: String,
    pub version: u32,
    pub extent: u32,
    pub keys: Vec<String>,
    pub features: Vec<Feature>,
}

impl Layer {
    pub fn count(&self, kind: GeomKind) -> usize {
        self.features.iter().filter(|f| f.kind == kind).count()
    }
}

/// A fully decoded vector tile.
#[derive(Debug, Clone)]
pub struct VectorTile {
    pub layers: Vec<Layer>,
}

impl VectorTile {
    pub fn total_features(&self) -> usize {
        self.layers.iter().map(|l| l.features.len()).sum()
    }

    pub fn layer(&self, name: &str) -> Option<&Layer> {
        self.layers.iter().find(|l| l.name == name)
    }
}

impl Mbtiles {
    /// Open an MBTiles file read-only.
    pub fn open(path: impl AsRef<Path>) -> Result<Self> {
        let conn = Connection::open_with_flags(
            path,
            OpenFlags::SQLITE_OPEN_READ_ONLY | OpenFlags::SQLITE_OPEN_URI,
        )?;
        Ok(Self { conn })
    }

    fn meta_value(&self, key: &str) -> Result<Option<String>> {
        let v = self
            .conn
            .query_row("SELECT value FROM metadata WHERE name = ?1", [key], |r| {
                r.get::<_, String>(0)
            })
            .optional()?;
        Ok(v)
    }

    pub fn metadata(&self) -> Result<Metadata> {
        Ok(Metadata {
            name: self.meta_value("name")?,
            format: self.meta_value("format")?,
            minzoom: self.meta_value("minzoom")?.and_then(|s| s.trim().parse().ok()),
            maxzoom: self.meta_value("maxzoom")?.and_then(|s| s.trim().parse().ok()),
            bounds: self.meta_value("bounds")?,
        })
    }

    /// Raw (still possibly gzipped) tile blob, or `None` if the tile is absent.
    pub fn raw_tile(&self, tile: Tile) -> Result<Option<Vec<u8>>> {
        let blob = self
            .conn
            .query_row(
                "SELECT tile_data FROM tiles \
                 WHERE zoom_level = ?1 AND tile_column = ?2 AND tile_row = ?3",
                rusqlite::params![tile.z, tile.x, tile.tms_y()],
                |r| r.get::<_, Vec<u8>>(0),
            )
            .optional()?;
        Ok(blob)
    }

    /// Fetch, decompress and MVT-decode a tile into typed geometry.
    /// `None` if the tile is absent.
    pub fn decode_tile(&self, tile: Tile) -> Result<Option<VectorTile>> {
        let Some(raw) = self.raw_tile(tile)? else {
            return Ok(None);
        };
        let bytes = maybe_gunzip(&raw)?;
        let mvt = geozero::mvt::Tile::decode(bytes.as_slice())?;

        let mut layers = Vec::with_capacity(mvt.layers.len());
        for l in &mvt.layers {
            let extent = l.extent.unwrap_or(geodesy::TILE_EXTENT);
            let features = l
                .features
                .iter()
                .map(|f| Feature {
                    kind: GeomKind::from_mvt(f.r#type.unwrap_or(0)),
                    parts: decode_geometry(&f.geometry),
                })
                .collect();
            layers.push(Layer {
                name: l.name.clone(),
                version: l.version,
                extent,
                keys: l.keys.clone(),
                features,
            });
        }
        Ok(Some(VectorTile { layers }))
    }
}

/// Decode an MVT geometry command stream (MVT spec §4.3) into parts of
/// tile-local points. MoveTo starts a new part; LineTo extends it; ClosePath
/// closes a polygon ring back to its start.
fn decode_geometry(geom: &[u32]) -> Vec<Vec<[f32; 2]>> {
    let mut parts: Vec<Vec<[f32; 2]>> = Vec::new();
    let mut cur: Vec<[f32; 2]> = Vec::new();
    let (mut x, mut y) = (0i32, 0i32);
    let mut i = 0usize;
    while i < geom.len() {
        let cmd = geom[i] & 0x7;
        let count = (geom[i] >> 3) as usize;
        i += 1;
        match cmd {
            1 => {
                // MoveTo: each point begins a new part.
                for _ in 0..count {
                    if i + 1 >= geom.len() {
                        break;
                    }
                    x += zigzag(geom[i]);
                    y += zigzag(geom[i + 1]);
                    i += 2;
                    if !cur.is_empty() {
                        parts.push(std::mem::take(&mut cur));
                    }
                    cur.push([x as f32, y as f32]);
                }
            }
            2 => {
                // LineTo: extend the current part.
                for _ in 0..count {
                    if i + 1 >= geom.len() {
                        break;
                    }
                    x += zigzag(geom[i]);
                    y += zigzag(geom[i + 1]);
                    i += 2;
                    cur.push([x as f32, y as f32]);
                }
            }
            7 => {
                // ClosePath: close the ring back to its first vertex (no params).
                if let Some(&first) = cur.first() {
                    cur.push(first);
                }
            }
            _ => break, // unknown command — stop to avoid desync
        }
    }
    if !cur.is_empty() {
        parts.push(cur);
    }
    parts
}

/// MVT zig-zag decode: maps an unsigned parameter back to a signed delta.
fn zigzag(n: u32) -> i32 {
    ((n >> 1) as i32) ^ (-((n & 1) as i32))
}

/// Gzip-decompress if the blob carries the gzip magic (1f 8b); otherwise return
/// it as-is (some MBTiles store raw, uncompressed PBF).
fn maybe_gunzip(bytes: &[u8]) -> Result<Vec<u8>> {
    if bytes.len() >= 2 && bytes[0] == 0x1f && bytes[1] == 0x8b {
        let mut out = Vec::new();
        flate2::read::GzDecoder::new(bytes).read_to_end(&mut out)?;
        Ok(out)
    } else {
        Ok(bytes.to_vec())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    #[test]
    fn gunzips_gzip_blobs_and_passes_through_raw() {
        let payload = b"vector tile bytes";
        let mut enc = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::default());
        enc.write_all(payload).unwrap();
        let gz = enc.finish().unwrap();
        assert_eq!(&gz[0..2], &[0x1f, 0x8b]);
        assert_eq!(maybe_gunzip(&gz).unwrap(), payload);
        assert_eq!(maybe_gunzip(payload).unwrap(), payload);
    }

    #[test]
    fn decodes_mvt_point_example() {
        // MVT spec worked example: a point at (25, 17): [MoveTo(1), 25, 17].
        let parts = decode_geometry(&[9, 50, 34]);
        assert_eq!(parts, vec![vec![[25.0, 17.0]]]);
    }

    #[test]
    fn decodes_mvt_linestring_example() {
        // MVT spec: MoveTo (2,2) then LineTo (2,10),(10,10).
        // [MoveTo(1), +2,+2, LineTo(2), +0,+8, +8,+0]
        let parts = decode_geometry(&[9, 4, 4, 18, 0, 16, 16, 0]);
        assert_eq!(parts, vec![vec![[2.0, 2.0], [2.0, 10.0], [10.0, 10.0]]]);
    }

    #[test]
    fn closepath_closes_ring() {
        // Triangle: MoveTo(0,0) LineTo(8,0),(0,8) ClosePath -> first vertex re-added.
        let parts = decode_geometry(&[9, 0, 0, 18, 16, 0, 0, 16, 15]);
        assert_eq!(
            parts,
            vec![vec![[0.0, 0.0], [8.0, 0.0], [8.0, 8.0], [0.0, 0.0]]]
        );
    }

    #[test]
    fn multiple_moveto_split_into_parts() {
        // Two separate single-point parts.
        let parts = decode_geometry(&[9, 2, 2, 9, 2, 2]);
        assert_eq!(parts.len(), 2);
    }
}
