//! Read OS Open Zoomstack (or any vector MBTiles) and decode Mapbox Vector
//! Tiles to a typed-feature summary.
//!
//! MBTiles is a SQLite database: a `tiles(zoom_level, tile_column, tile_row,
//! tile_data)` table plus a `metadata(name, value)` table. Rows are stored
//! TMS-flipped (bottom-up), so we go through [`geodesy::Tile::tms_y`]. Each
//! `tile_data` blob is a gzip-compressed MVT (protobuf).
//!
//! Like qct-viewer's `qct` crate, this stays GPU-unaware: it hands back
//! geometry/attributes; tessellation and upload live in the renderer.

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

/// Per-layer summary of a decoded tile.
#[derive(Debug, Clone)]
pub struct LayerSummary {
    pub name: String,
    pub version: u32,
    pub extent: u32,
    pub features: usize,
    pub points: usize,
    pub lines: usize,
    pub polygons: usize,
    /// Attribute keys present in the layer's dictionary.
    pub keys: Vec<String>,
}

/// A decoded tile reduced to per-layer summaries.
#[derive(Debug, Clone)]
pub struct DecodedTile {
    pub layers: Vec<LayerSummary>,
}

impl DecodedTile {
    pub fn total_features(&self) -> usize {
        self.layers.iter().map(|l| l.features).sum()
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

    /// Fetch, decompress and MVT-decode a tile into a per-layer summary.
    /// `None` if the tile is absent.
    pub fn decode_tile(&self, tile: Tile) -> Result<Option<DecodedTile>> {
        let Some(raw) = self.raw_tile(tile)? else {
            return Ok(None);
        };
        let bytes = maybe_gunzip(&raw)?;
        let mvt = geozero::mvt::Tile::decode(bytes.as_slice())?;

        let mut layers = Vec::with_capacity(mvt.layers.len());
        for l in &mvt.layers {
            let (mut points, mut lines, mut polygons) = (0usize, 0usize, 0usize);
            for f in &l.features {
                // GeomType: 1 = Point, 2 = LineString, 3 = Polygon.
                match f.r#type.unwrap_or(0) {
                    1 => points += 1,
                    2 => lines += 1,
                    3 => polygons += 1,
                    _ => {}
                }
            }
            layers.push(LayerSummary {
                name: l.name.clone(),
                version: l.version,
                extent: l.extent.unwrap_or(geodesy::TILE_EXTENT),
                features: l.features.len(),
                points,
                lines,
                polygons,
                keys: l.keys.clone(),
            });
        }
        Ok(Some(DecodedTile { layers }))
    }
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
    use super::maybe_gunzip;
    use std::io::Write;

    #[test]
    fn gunzips_gzip_blobs_and_passes_through_raw() {
        let payload = b"vector tile bytes";
        // Round-trip through gzip -> maybe_gunzip recovers the payload.
        let mut enc = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::default());
        enc.write_all(payload).unwrap();
        let gz = enc.finish().unwrap();
        assert_eq!(gz[0], 0x1f);
        assert_eq!(gz[1], 0x8b);
        assert_eq!(maybe_gunzip(&gz).unwrap(), payload);
        // A non-gzip blob (no magic) is returned untouched.
        assert_eq!(maybe_gunzip(payload).unwrap(), payload);
    }
}
