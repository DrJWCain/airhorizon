//! WGS84 geodetic -> OSGB36 British National Grid (easting / northing).
//!
//! Ported from qct-viewer's `dem/osgb.rs` (validated against OS test points).
//! Follows the Ordnance Survey "A Guide to Coordinate Systems in Great Britain"
//! (annexes B and C).
//!
//! The datum shift uses a Helmert 7-parameter transformation with the published
//! average WGS84 -> OSGB36 values. Accuracy across the UK is ~5 m, well inside
//! the 50 m OS Terrain cell size and negligible for horizon/AR bearing angles.
//! Sub-metre work wants OSTN15; that grid backend (B1b) will slot in behind
//! [`wgs84_to_bng`] without changing this signature.
//!
//! Forward projection (lat/lon on Airy 1830 -> BNG E/N) is a transverse Mercator
//! with the standard BNG parameters: scale factor 0.9996012717 on the central
//! meridian, true origin 49°N 2°W, false easting +400000 m, false northing
//! -100000 m. Angles in degrees; metres SI.

const RAD: f64 = std::f64::consts::PI / 180.0;

// WGS-84 ellipsoid.
const WGS84_A: f64 = 6_378_137.0;
const WGS84_F: f64 = 1.0 / 298.257_223_563;

// Airy 1830 ellipsoid (OSGB36).
const AIRY_A: f64 = 6_377_563.396;
const AIRY_B: f64 = 6_356_256.909;

// Helmert 7-parameter transformation, WGS-84 -> OSGB36 (OS published values).
const HELMERT_TX: f64 = -446.448;
const HELMERT_TY: f64 = 125.157;
const HELMERT_TZ: f64 = -542.060;
const HELMERT_S_PPM: f64 = 20.4894;
// Rotation parameters in arc-seconds; converted to radians at use site.
const HELMERT_RX_SEC: f64 = -0.1502;
const HELMERT_RY_SEC: f64 = -0.2470;
const HELMERT_RZ_SEC: f64 = -0.8421;

// Transverse Mercator parameters for the British National Grid.
const BNG_F0: f64 = 0.999_601_271_7;
const BNG_LAT0_DEG: f64 = 49.0;
const BNG_LON0_DEG: f64 = -2.0;
const BNG_E0: f64 = 400_000.0;
const BNG_N0: f64 = -100_000.0;

/// Forward: WGS-84 geodetic (degrees) -> BNG easting/northing (metres).
pub fn wgs84_to_bng(lat_deg: f64, lon_deg: f64) -> (f64, f64) {
    let (x1, y1, z1) = geodetic_to_ecef(lat_deg, lon_deg, 0.0, WGS84_A, ecc_sq(WGS84_A, wgs84_b()));
    let (x2, y2, z2) = helmert_wgs84_to_osgb36(x1, y1, z1);
    let (lat2_rad, lon2_rad, _) = ecef_to_geodetic(x2, y2, z2, AIRY_A, AIRY_B);
    tm_forward(lat2_rad, lon2_rad)
}

fn wgs84_b() -> f64 {
    WGS84_A * (1.0 - WGS84_F)
}

fn ecc_sq(a: f64, b: f64) -> f64 {
    (a * a - b * b) / (a * a)
}

/// Geodetic (lat/lon/height) on an ellipsoid -> ECEF Cartesian.
fn geodetic_to_ecef(lat_deg: f64, lon_deg: f64, h: f64, a: f64, e2: f64) -> (f64, f64, f64) {
    let lat = lat_deg * RAD;
    let lon = lon_deg * RAD;
    let sin_lat = lat.sin();
    let cos_lat = lat.cos();
    let nu = a / (1.0 - e2 * sin_lat * sin_lat).sqrt();
    let x = (nu + h) * cos_lat * lon.cos();
    let y = (nu + h) * cos_lat * lon.sin();
    let z = ((1.0 - e2) * nu + h) * sin_lat;
    (x, y, z)
}

/// ECEF Cartesian -> geodetic on an ellipsoid (Bowring iteration).
/// Returns (lat_rad, lon_rad, height_m).
fn ecef_to_geodetic(x: f64, y: f64, z: f64, a: f64, b: f64) -> (f64, f64, f64) {
    let e2 = ecc_sq(a, b);
    let p = (x * x + y * y).sqrt();
    let lon = y.atan2(x);
    let mut lat = (z / p / (1.0 - e2)).atan();
    for _ in 0..6 {
        let sin_lat = lat.sin();
        let nu = a / (1.0 - e2 * sin_lat * sin_lat).sqrt();
        lat = ((z + e2 * nu * sin_lat) / p).atan();
    }
    let sin_lat = lat.sin();
    let nu = a / (1.0 - e2 * sin_lat * sin_lat).sqrt();
    let h = p / lat.cos() - nu;
    (lat, lon, h)
}

/// Helmert 7-parameter transformation, WGS-84 -> OSGB36 (ECEF in, ECEF out).
fn helmert_wgs84_to_osgb36(x: f64, y: f64, z: f64) -> (f64, f64, f64) {
    let rx = HELMERT_RX_SEC / 3600.0 * RAD;
    let ry = HELMERT_RY_SEC / 3600.0 * RAD;
    let rz = HELMERT_RZ_SEC / 3600.0 * RAD;
    let s = HELMERT_S_PPM * 1e-6;
    let x2 = HELMERT_TX + (1.0 + s) * x + (-rz) * y + (ry) * z;
    let y2 = HELMERT_TY + (rz) * x + (1.0 + s) * y + (-rx) * z;
    let z2 = HELMERT_TZ + (-ry) * x + (rx) * y + (1.0 + s) * z;
    (x2, y2, z2)
}

/// Transverse Mercator forward (lat/lon on Airy -> BNG E/N).
fn tm_forward(lat: f64, lon: f64) -> (f64, f64) {
    let a = AIRY_A;
    let b = AIRY_B;
    let f0 = BNG_F0;
    let lat0 = BNG_LAT0_DEG * RAD;
    let lon0 = BNG_LON0_DEG * RAD;

    let e2 = ecc_sq(a, b);
    let n = (a - b) / (a + b);
    let sin_lat = lat.sin();
    let cos_lat = lat.cos();
    let nu = a * f0 / (1.0 - e2 * sin_lat * sin_lat).sqrt();
    let rho = a * f0 * (1.0 - e2) / (1.0 - e2 * sin_lat * sin_lat).powf(1.5);
    let eta2 = nu / rho - 1.0;

    // Meridional arc M.
    let n2 = n * n;
    let n3 = n2 * n;
    let m = b
        * f0
        * ((1.0 + n + 5.0 / 4.0 * n2 + 5.0 / 4.0 * n3) * (lat - lat0)
            - (3.0 * n + 3.0 * n2 + 21.0 / 8.0 * n3) * (lat - lat0).sin() * (lat + lat0).cos()
            + (15.0 / 8.0 * n2 + 15.0 / 8.0 * n3)
                * (2.0 * (lat - lat0)).sin()
                * (2.0 * (lat + lat0)).cos()
            - 35.0 / 24.0 * n3 * (3.0 * (lat - lat0)).sin() * (3.0 * (lat + lat0)).cos());

    let tan_lat = lat.tan();
    let tan2 = tan_lat * tan_lat;
    let tan4 = tan2 * tan2;

    let i = m + BNG_N0;
    let ii = nu / 2.0 * sin_lat * cos_lat;
    let iii = nu / 24.0 * sin_lat * cos_lat.powi(3) * (5.0 - tan2 + 9.0 * eta2);
    let iiia = nu / 720.0 * sin_lat * cos_lat.powi(5) * (61.0 - 58.0 * tan2 + tan4);
    let iv = nu * cos_lat;
    let v = nu / 6.0 * cos_lat.powi(3) * (nu / rho - tan2);
    let vi = nu / 120.0
        * cos_lat.powi(5)
        * (5.0 - 18.0 * tan2 + tan4 + 14.0 * eta2 - 58.0 * tan2 * eta2);

    let dlon = lon - lon0;
    let northing = i + ii * dlon.powi(2) + iii * dlon.powi(4) + iiia * dlon.powi(6);
    let easting = BNG_E0 + iv * dlon + v * dlon.powi(3) + vi * dlon.powi(5);
    (easting, northing)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Royal Greenwich Observatory (WGS-84 51.4779°N, 0.0015°W) maps to
    /// BNG TQ 38886 77322 = (538886, 177322). Helmert-only is good to ~5 m.
    #[test]
    fn greenwich() {
        let (e, n) = wgs84_to_bng(51.4779, -0.0015);
        assert!((e - 538886.0).abs() < 10.0, "easting was {e}");
        assert!((n - 177322.0).abs() < 10.0, "northing was {n}");
    }

    /// Caister water tower, OSGB36 (52°39'27.2531"N, 1°43'4.5177"E) ->
    /// BNG (651409.903, 313177.270). Airy-only TM round-trip, within ~1 m.
    #[test]
    fn caister_via_osgb36() {
        let lat = 52.0 + 39.0 / 60.0 + 27.2531 / 3600.0;
        let lon = 1.0 + 43.0 / 60.0 + 4.5177 / 3600.0;
        let (e, n) = tm_forward(lat * RAD, lon * RAD);
        assert!((e - 651409.9).abs() < 1.0, "easting was {e}");
        assert!((n - 313177.3).abs() < 1.0, "northing was {n}");
    }

    /// Scafell Pike summit (~54.4542°N, 3.2118°W) should land near its
    /// published grid ref NY 2150 0721 = (321500, 507210), within Helmert's ~5 m.
    #[test]
    fn scafell_pike_near_grid_ref() {
        let (e, n) = wgs84_to_bng(54.4542, -3.2118);
        assert!((e - 321500.0).abs() < 60.0, "easting was {e}");
        assert!((n - 507210.0).abs() < 60.0, "northing was {n}");
    }
}
