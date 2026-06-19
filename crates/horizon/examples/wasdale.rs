//! Debug: from Wasdale Head Inn, print the full visibility geometry for named
//! fells, so we can see why each is/ isn't labelled.
//!   cargo run -p horizon --example wasdale --release --offline

use dem::Dem;
use geodesy::LatLon;
use horizon::{cast, HorizonParams};
use peaks::Peaks;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let dem = Dem::open(std::path::Path::new(r"C:\maps\OS Terrain 50"))?;
    let peaks = Peaks::load_csv(r"C:\maps\airhorizon\data\DoBIH_v18_4.csv")?;

    // lat/lon closest to Wasdale Head Inn, NY ~1870 0879.
    let target = (318700.0, 508790.0);
    let (mut best, mut bd) = ((0.0, 0.0), f64::MAX);
    let mut lat = 54.44;
    while lat < 54.50 {
        let mut lon = -3.33;
        while lon < -3.24 {
            let (e, n) = geodesy::wgs84_to_bng(lat, lon);
            let d = (e - target.0).hypot(n - target.1);
            if d < bd {
                bd = d;
                best = (lat, lon);
            }
            lon += 0.0003;
        }
        lat += 0.0003;
    }
    let vp = LatLon::new(best.0, best.1);
    let params = HorizonParams::default();
    let h = cast(&dem, vp, &params).ok_or("outside DEM")?;
    let (eye_e, eye_n) = geodesy::wgs84_to_bng(vp.lat, vp.lon);
    let h_eye = h.eye_ground_m + params.eye_height_m;
    let r_eff = 6_371_000.0 / (1.0 - params.refraction_k);
    println!("inn ({:.4}, {:.4}) ground {:.0} m\n", vp.lat, vp.lon, h.eye_ground_m);
    println!("{:<14} {:>4} {:>6} {:>7}  {:>8} {:>8}  {:>8}", "fell", "bear", "dist", "sum_el", "sky_el", "sky_dist", "verdict");

    let in_range = peaks.within_range(vp.lat, vp.lon, params.max_range_m);
    for name in ["Lingmell", "Yewbarrow", "Great Gable", "Kirk Fell", "Pillar", "Red Pike (Wasdale)"] {
        // Nearest fell of this name (several DoBIH hills share names).
        let Some(pk) = in_range
            .iter()
            .filter(|p| p.name == name)
            .min_by(|a, b| {
                let da = { let (e, n) = geodesy::wgs84_to_bng(a.lat, a.lon); (e - eye_e).hypot(n - eye_n) };
                let db = { let (e, n) = geodesy::wgs84_to_bng(b.lat, b.lon); (e - eye_e).hypot(n - eye_n) };
                da.partial_cmp(&db).unwrap()
            })
        else {
            println!("{name:<14} (not within range)");
            continue;
        };
        let (pe, pn) = geodesy::wgs84_to_bng(pk.lat, pk.lon);
        let (de, dn) = (pe - eye_e, pn - eye_n);
        let dist = de.hypot(dn);
        let bearing = de.atan2(dn).to_degrees().rem_euclid(360.0);
        let drop = dist * dist / (2.0 * r_eff);
        let elev = ((pk.height_m - h_eye - drop) / dist).atan().to_degrees();
        let sky = (h.at_bearing_deg(bearing) as f64).to_degrees();
        let sky_d = h.dist_at_bearing_deg(bearing) as f64;
        let verdict = if elev + 0.1 >= sky {
            "SUMMIT"
        } else if dist - sky_d <= 1500.0 {
            "slopes"
        } else {
            "HIDDEN"
        };
        println!(
            "{name:<14} {bearing:>4.0} {:>5.1}km {elev:>+6.1}°  {sky:>+6.1}° {:>6.1}km  {verdict}  (sum-sky_dist {:.0}m)",
            dist / 1000.0,
            sky_d / 1000.0,
            dist - sky_d
        );
    }
    Ok(())
}
