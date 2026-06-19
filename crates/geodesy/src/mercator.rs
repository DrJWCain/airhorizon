//! Web Mercator (EPSG:3857) — the spherical projection used by slippy-map tiles
//! and OS Open Zoomstack. Closed-form and cheap enough to run per-frame.

use crate::{LatLon, Mercator};

/// WGS84 semi-major axis, used as the Web Mercator sphere radius (EPSG:3857
/// treats the earth as a sphere of this radius).
pub const EARTH_RADIUS_M: f64 = 6_378_137.0;

/// Half the Web Mercator world extent in metres: `pi * EARTH_RADIUS_M`.
/// x and y both span ±this value.
pub const MERCATOR_MAX: f64 = std::f64::consts::PI * EARTH_RADIUS_M;

/// The latitude (deg) at which Web Mercator is clipped to a square world,
/// `2*atan(e^pi) - pi/2`.
pub const MAX_LATITUDE: f64 = 85.051_128_779_806_59;

pub fn forward(ll: LatLon) -> Mercator {
    let lat = ll.lat.clamp(-MAX_LATITUDE, MAX_LATITUDE).to_radians();
    let lon = ll.lon.to_radians();
    let x = EARTH_RADIUS_M * lon;
    let y = EARTH_RADIUS_M * (std::f64::consts::FRAC_PI_4 + lat / 2.0).tan().ln();
    Mercator { x, y }
}

pub fn inverse(m: Mercator) -> LatLon {
    let lon = (m.x / EARTH_RADIUS_M).to_degrees();
    let lat = (2.0 * (m.y / EARTH_RADIUS_M).exp().atan() - std::f64::consts::FRAC_PI_2).to_degrees();
    LatLon { lat, lon }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() < tol, "{a} != {b} (tol {tol})");
    }

    #[test]
    fn origin_maps_to_zero() {
        let m = forward(LatLon::new(0.0, 0.0));
        close(m.x, 0.0, 1e-6);
        close(m.y, 0.0, 1e-6);
    }

    #[test]
    fn extent_at_clip_latitude() {
        // The world is square: at the clip latitude, y reaches +MERCATOR_MAX,
        // and lon 180 reaches +MERCATOR_MAX in x.
        close(forward(LatLon::new(MAX_LATITUDE, 180.0)).x, MERCATOR_MAX, 1e-3);
        close(forward(LatLon::new(MAX_LATITUDE, 180.0)).y, MERCATOR_MAX, 1e-3);
    }

    #[test]
    fn round_trip_keswick() {
        let ll = LatLon::new(54.6012, -3.1399);
        let back = forward(ll).to_latlon();
        close(back.lat, ll.lat, 1e-9);
        close(back.lon, ll.lon, 1e-9);
    }

    #[test]
    fn latitude_is_clamped_not_infinite() {
        // Beyond the clip latitude the projection must not blow up to inf/NaN.
        let y = forward(LatLon::new(89.9, 0.0)).y;
        assert!(y.is_finite());
        close(y, MERCATOR_MAX, 1e-3);
    }
}
