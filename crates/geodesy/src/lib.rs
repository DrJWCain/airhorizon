//! Coordinate transforms for AirHorizon.
//!
//! Three frames are in play, and this crate is the single bridge between them
//! (the rest of the workspace should not roll its own conversions):
//!
//! * [`LatLon`] — WGS84 geographic degrees. GPS input; also ~ETRS89 for UK work.
//! * [`Mercator`] — Web Mercator (EPSG:3857) metres. The internal display frame
//!   and the tiling scheme used by OS Open Zoomstack / slippy maps.
//! * [`Bng`] — OSGB36 British National Grid easting/northing metres. The frame
//!   of OS Terrain DEMs, LIDAR, and peak grid refs — used for elevation, the
//!   horizon ray-caster, and peak geometry.
//!
//! Display path: `LatLon <-> Mercator` (cheap, closed-form spherical mercator).
//! DEM/horizon path: `LatLon -> Bng` (Helmert 7-parameter datum shift in B1a,
//! ~5 m; an OSTN15 grid backend for ~10 cm lands in B1b behind the same API).
//! Tiling: [`tile`] maps lon/lat to slippy `z/x/y` and back, plus the TMS row
//! flip that MBTiles uses.

mod bng;
mod mercator;
pub mod tile;

pub use bng::wgs84_to_bng;
pub use mercator::{EARTH_RADIUS_M, MERCATOR_MAX};
pub use tile::{Tile, TILE_EXTENT};

/// WGS84 geographic position, degrees (positive north / east).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LatLon {
    pub lat: f64,
    pub lon: f64,
}

impl LatLon {
    pub fn new(lat: f64, lon: f64) -> Self {
        Self { lat, lon }
    }

    /// Project to Web Mercator (EPSG:3857) metres.
    pub fn to_mercator(self) -> Mercator {
        mercator::forward(self)
    }

    /// Convert to OSGB36 British National Grid easting/northing (metres).
    pub fn to_bng(self) -> Bng {
        let (e, n) = bng::wgs84_to_bng(self.lat, self.lon);
        Bng { e, n }
    }
}

/// Web Mercator (EPSG:3857) position, metres. Valid latitude band is
/// roughly ±85.0511°; x,y are bounded by ±[`MERCATOR_MAX`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mercator {
    pub x: f64,
    pub y: f64,
}

impl Mercator {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    /// Inverse projection back to WGS84 degrees.
    pub fn to_latlon(self) -> LatLon {
        mercator::inverse(self)
    }
}

/// OSGB36 British National Grid easting/northing, metres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bng {
    pub e: f64,
    pub n: f64,
}

impl Bng {
    pub fn new(e: f64, n: f64) -> Self {
        Self { e, n }
    }
}
