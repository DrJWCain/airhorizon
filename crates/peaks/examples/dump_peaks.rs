//! Load DoBIH and run a couple of queries — smoke test for B7.
//!
//!   cargo run -p peaks --example dump_peaks --release --offline

use peaks::Peaks;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| r"C:\maps\airhorizon\data\DoBIH_v18_4.csv".to_string());

    let p = Peaks::load_csv(&path)?;
    println!("loaded {} hills with coordinates", p.len());

    // Scafell Pike should resolve at ~54.454, -3.212, 978 m.
    if let Some(sp) = p
        .in_bbox(-3.25, 54.43, -3.18, 54.47)
        .into_iter()
        .find(|pk| pk.name.contains("Scafell Pike"))
    {
        println!(
            "Scafell Pike: ({:.4}, {:.4}) {} m, prominence {} m, class {:?}",
            sp.lat, sp.lon, sp.height_m, sp.prominence_m, sp.classification
        );
    }

    // Wasdale skyline: peaks within 6 km of Wasdale Head, prominence >= 30 m.
    let mut near: Vec<_> = p
        .within_range(54.4878, -3.2961, 6000.0)
        .into_iter()
        .filter(|pk| pk.prominence_m >= 30.0)
        .collect();
    near.sort_by(|a, b| b.height_m.partial_cmp(&a.height_m).unwrap());
    println!("\n{} hills within 6 km of Wasdale Head (prom >= 30 m); tallest:", near.len());
    for pk in near.iter().take(8) {
        println!("  {:<22} {:>4} m (prom {:>4} m)", pk.name, pk.height_m as i32, pk.prominence_m as i32);
    }
    Ok(())
}
