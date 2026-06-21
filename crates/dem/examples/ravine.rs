//! Does our elevation data resolve a narrow ravine (Piers Gill, above Wasdale)?
//! Scan the area in both Terrain 50 and 1 m LIDAR and report where LIDAR drops
//! far below Terrain 50 — i.e. the incised gill that the 50 m grid smooths away.
//!   cargo run -p dem --example ravine --release --offline

use dem::Dem;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let t50 = Dem::open(std::path::Path::new(r"C:\maps\OS Terrain 50"))?;
    let mut lid = Dem::open(std::path::Path::new(r"C:\maps\OS Terrain 50"))?;
    let n = lid.attach_lidar(std::path::Path::new(r"C:\maps\airhorizon\data\lidar"));
    println!("attached {n} LIDAR tiles");

    // Piers Gill sits around NY 209 094.
    let (e0, e1) = (320_650.0, 321_250.0);
    let (n0, n1) = (509_000.0, 509_900.0);
    let step = 5.0;

    let mut worst: Vec<(f64, f64, f64, f64)> = Vec::new(); // e, n, lidar, t50
    let mut e = e0;
    while e <= e1 {
        let mut nn = n0;
        while nn <= n1 {
            if let (Some(l), Some(t)) = (lid.elevation_bng(e, nn), t50.elevation_bng(e, nn)) {
                worst.push((e, nn, l, t));
            }
            nn += step;
        }
        e += step;
    }
    // Sort by how far LIDAR sits below Terrain 50 (deepest incision first).
    worst.sort_by(|a, b| (a.2 - a.3).partial_cmp(&(b.2 - b.3)).unwrap());

    println!("\nMost incised points (LIDAR far below Terrain 50 = the gill):");
    println!("{:>9} {:>9} {:>8} {:>8} {:>7}", "easting", "northing", "LIDAR", "T50", "drop");
    for &(e, nn, l, t) in worst.iter().take(12) {
        println!("{e:>9.0} {nn:>9.0} {l:>8.1} {t:>8.1} {:>7.1}", l - t);
    }

    // A west-east transect through the deepest point, to see the V in the LIDAR.
    if let Some(&(_, gn, _, _)) = worst.first() {
        println!("\nW-E LIDAR transect at N={gn:.0} (1 m relief across the slot):");
        let mut e = e0;
        while e <= e1 {
            if let Some(l) = lid.elevation_bng(e, gn) {
                let bar = "#".repeat(((l - 380.0).max(0.0) / 3.0) as usize);
                println!("  E{e:>7.0}  {l:>6.1}  {bar}");
            }
            e += 10.0;
        }
    }
    Ok(())
}
