//! Slippy-map tile math (XYZ scheme, Web Mercator). Maps lon/lat to `z/x/y`
//! and gives tile bounds in both WGS84 and Web Mercator. Includes the TMS row
//! flip used by MBTiles (Zoomstack ships as MBTiles, which stores rows
//! bottom-up while XYZ counts top-down).

use crate::mercator::{MAX_LATITUDE, MERCATOR_MAX};
use crate::{LatLon, Mercator};

/// MVT geometry is expressed in this integer tile-local extent (0..=4096).
pub const TILE_EXTENT: u32 = 4096;

/// A slippy-map tile address. `x` counts east from the antimeridian, `y` counts
/// south from the north (XYZ convention).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Tile {
    pub z: u8,
    pub x: u32,
    pub y: u32,
}

impl Tile {
    pub fn new(z: u8, x: u32, y: u32) -> Self {
        Self { z, x, y }
    }

    /// Number of tiles per axis at this zoom: `2^z`.
    pub fn count_per_axis(z: u8) -> u32 {
        1u32 << z
    }

    /// The tile containing `ll` at zoom `z`.
    pub fn containing(ll: LatLon, z: u8) -> Tile {
        let n = Self::count_per_axis(z) as f64;
        let lat = ll.lat.clamp(-MAX_LATITUDE, MAX_LATITUDE).to_radians();
        let xf = (ll.lon + 180.0) / 360.0 * n;
        let yf = (1.0 - (lat.tan() + 1.0 / lat.cos()).ln() / std::f64::consts::PI) / 2.0 * n;
        let max = Self::count_per_axis(z) - 1;
        Tile {
            z,
            x: (xf.floor() as i64).clamp(0, max as i64) as u32,
            y: (yf.floor() as i64).clamp(0, max as i64) as u32,
        }
    }

    /// North-west (top-left) corner of the tile in WGS84 degrees.
    pub fn nw_corner(self) -> LatLon {
        let n = Self::count_per_axis(self.z) as f64;
        let lon = self.x as f64 / n * 360.0 - 180.0;
        let lat_rad =
            (std::f64::consts::PI * (1.0 - 2.0 * self.y as f64 / n)).sinh().atan();
        LatLon { lat: lat_rad.to_degrees(), lon }
    }

    /// Tile bounds in Web Mercator metres as `(min_x, min_y, max_x, max_y)`
    /// (min = south-west, max = north-east). Linear in tile space, so this is
    /// what the renderer uses to place tile-local MVT geometry on screen.
    pub fn mercator_bounds(self) -> (f64, f64, f64, f64) {
        let n = Self::count_per_axis(self.z) as f64;
        let span = 2.0 * MERCATOR_MAX / n;
        let min_x = -MERCATOR_MAX + self.x as f64 * span;
        let max_x = min_x + span;
        // y counts south from the top, Mercator y counts north from the equator.
        let max_y = MERCATOR_MAX - self.y as f64 * span;
        let min_y = max_y - span;
        (min_x, min_y, max_x, max_y)
    }

    /// Web Mercator coordinate of a tile-local MVT point (0..=`TILE_EXTENT`).
    pub fn mvt_to_mercator(self, px: f64, py: f64) -> Mercator {
        let (min_x, min_y, max_x, max_y) = self.mercator_bounds();
        let fx = px / TILE_EXTENT as f64;
        let fy = py / TILE_EXTENT as f64; // MVT y is top-down within the tile
        Mercator {
            x: min_x + fx * (max_x - min_x),
            y: max_y - fy * (max_y - min_y),
        }
    }

    /// Convert this XYZ row to the TMS row MBTiles stores it under: `2^z-1 - y`.
    pub fn tms_y(self) -> u32 {
        Self::count_per_axis(self.z) - 1 - self.y
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() < tol, "{a} != {b} (tol {tol})");
    }

    #[test]
    fn zoom0_is_one_world_tile() {
        assert_eq!(Tile::count_per_axis(0), 1);
        let t = Tile::containing(LatLon::new(54.6, -3.14), 0);
        assert_eq!(t, Tile::new(0, 0, 0));
        // Its NW corner is the top-left of the Mercator world.
        let nw = t.nw_corner();
        close(nw.lon, -180.0, 1e-9);
        close(nw.lat, MAX_LATITUDE, 1e-6);
    }

    #[test]
    fn known_keswick_tile_z12() {
        // Independently computed from the slippy formula for (54.6012, -3.1399).
        let t = Tile::containing(LatLon::new(54.6012, -3.1399), 12);
        assert_eq!(t, Tile::new(12, 2012, 1303));
    }

    #[test]
    fn nw_corner_round_trips_to_same_tile() {
        let t = Tile::new(14, 8021, 5210);
        // A point just inside the NW corner must map back to the same tile.
        let nw = t.nw_corner();
        let inside = LatLon::new(nw.lat - 1e-6, nw.lon + 1e-6);
        assert_eq!(Tile::containing(inside, 14), t);
    }

    #[test]
    fn mercator_bounds_tile_local_corners() {
        let t = Tile::new(12, 2012, 1304);
        let (min_x, min_y, max_x, max_y) = t.mercator_bounds();
        // MVT (0,0) is the tile's NW corner -> (min_x, max_y).
        let nw = t.mvt_to_mercator(0.0, 0.0);
        close(nw.x, min_x, 1e-6);
        close(nw.y, max_y, 1e-6);
        // MVT (extent, extent) is the SE corner -> (max_x, min_y).
        let se = t.mvt_to_mercator(TILE_EXTENT as f64, TILE_EXTENT as f64);
        close(se.x, max_x, 1e-6);
        close(se.y, min_y, 1e-6);
    }

    #[test]
    fn nw_corner_agrees_with_mercator_bounds() {
        // The geographic NW corner, projected, equals the mercator-bounds NW.
        let t = Tile::new(13, 4005, 2603);
        let (min_x, _min_y, _max_x, max_y) = t.mercator_bounds();
        let m = t.nw_corner().to_mercator();
        close(m.x, min_x, 1e-3);
        close(m.y, max_y, 1e-3);
    }

    #[test]
    fn tms_flip_is_involutive() {
        let t = Tile::new(12, 2012, 1304);
        let flipped = Tile::new(12, t.x, t.tms_y());
        assert_eq!(flipped.tms_y(), t.y);
    }
}
