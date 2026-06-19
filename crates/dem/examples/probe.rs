//! Probe DEM elevation at given lat/lon points (debugging).
//!   cargo run -p dem --example probe --release --offline

use dem::Dem;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let dem = Dem::open(std::path::Path::new(r"C:\maps\OS Terrain 50"))?;
    let pts = [
        ("Scafell Pike (978)", 54.4543, -3.2117),
        ("Wasdale Head Inn (~75)", 54.4877, -3.2967),
        ("Wast Water shore (~61)", 54.4585, -3.2917),
        ("Wasdale Head NGR NY187088", 54.4878, -3.2961),
        ("Keswick (~80)", 54.6005, -3.1347),
    ];
    for (name, lat, lon) in pts {
        let (e, n) = geodesy::wgs84_to_bng(lat, lon);
        let el = dem.elevation_bng(e, n);
        println!("{name:<26} ({lat},{lon}) -> BNG ({e:.0}, {n:.0}) -> {el:?} m");
    }
    Ok(())
}
