//! What does the ravine detector return toward Piers Gill (ENE of Wasdale Head)?
//!   cargo run -p horizon --example gilldebug --release --offline

use dem::Dem;
use geodesy::LatLon;
use horizon::{cast, ravines, HorizonParams};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut dem = Dem::open(std::path::Path::new(r"C:\maps\OS Terrain 50"))?;
    let n = dem.attach_lidar(std::path::Path::new(r"C:\maps\airhorizon\data\lidar"));
    println!("lidar tiles: {n}");
    let vp = LatLon::new(54.4877, -3.2967); // Wasdale Head
    let params = HorizonParams::default();
    let _h = cast(&dem, vp, &params).ok_or("outside DEM")?;

    let r = ravines(&dem, vp, &params, 8_000.0, 5.0, 15.0);
    println!("total ravine points: {}", r.len());

    // Piers Gill is roughly ENE (~60-78 deg) of Wasdale Head.
    let mut near: Vec<_> = r.iter().filter(|&&(az, _, _, _)| az >= 55.0 && az <= 82.0).collect();
    near.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap()); // deepest first
    println!("points in bearing 55-82 deg: {} (deepest first)", near.len());
    for &&(az, elev, depth, dist) in near.iter().take(40) {
        println!(
            "  az {az:>5.1}  elev {:>5.1} deg  depth {depth:>4.1} m  dist {:>5.0} m",
            elev.to_degrees(),
            dist
        );
    }
    Ok(())
}
