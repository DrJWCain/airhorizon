//! Cast the horizon from a viewpoint, then list the named summits visible above
//! it — the A3 sanity check.
//!
//!   cargo run -p horizon --example visible --release --offline
//!   cargo run -p horizon --example visible --release --offline -- 54.4543 -3.2117

use dem::Dem;
use geodesy::LatLon;
use horizon::{cast, visible_peaks, HorizonParams};
use peaks::Peaks;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut a = std::env::args().skip(1);
    let lat: f64 = a.next().and_then(|s| s.parse().ok()).unwrap_or(54.4543);
    let lon: f64 = a.next().and_then(|s| s.parse().ok()).unwrap_or(-3.2117);
    let vp = LatLon::new(lat, lon);

    let dem = Dem::open(std::path::Path::new(r"C:\maps\OS Terrain 50"))?;
    let peaks = Peaks::load_csv(r"C:\maps\airhorizon\data\DoBIH_v18_4.csv")?;
    let params = HorizonParams::default();

    let h = cast(&dem, vp, &params).ok_or("viewpoint outside DEM")?;
    let mut vis: Vec<_> = visible_peaks(&h, vp, &peaks, &params)
        .into_iter()
        .filter(|v| v.peak.prominence_m >= 100.0) // Marilyns+ to cut clutter
        .collect();
    vis.sort_by(|a, b| b.elev_deg.partial_cmp(&a.elev_deg).unwrap());

    println!(
        "from ({lat}, {lon}), ground {:.0} m: {} prominent summits visible\n",
        h.eye_ground_m,
        vis.len()
    );
    println!("{:<24} {:>5} {:>7} {:>6}", "name", "bear", "elev", "dist");
    for v in vis.iter().take(20) {
        println!(
            "{:<24} {:>4.0}° {:>+6.1}° {:>5.1}km",
            v.peak.name,
            v.bearing_deg,
            v.elev_deg,
            v.dist_m / 1000.0
        );
    }
    Ok(())
}
