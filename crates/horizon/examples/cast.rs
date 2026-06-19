//! Cast the horizon from a couple of known viewpoints and print the skyline —
//! the A1 sanity check.
//!
//!   cargo run -p horizon --example cast --release --offline
//!
//! Expectations:
//!   * Scafell Pike (978 m, England's highest): skyline almost entirely AT or
//!     BELOW eye level — max apparent elevation near 0°, slightly negative.
//!   * Keswick (valley town ~80 m): Skiddaw (931 m) ~5 km north — skyline well
//!     ABOVE eye level to the north, low to the south down the Borrowdale vale.

use dem::Dem;
use geodesy::LatLon;
use horizon::{cast, HorizonParams};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let dir = std::env::args().nth(1).unwrap_or_else(|| r"C:\maps\OS Terrain 50".to_string());
    let dem = Dem::open(std::path::Path::new(&dir))?;
    println!("DEM: {} tiles indexed\n", dem.tile_count());

    let params = HorizonParams::default();
    let compass = [
        (0.0, "N"), (45.0, "NE"), (90.0, "E"), (135.0, "SE"),
        (180.0, "S"), (225.0, "SW"), (270.0, "W"), (315.0, "NW"),
    ];

    for (name, vp) in [
        ("Scafell Pike", LatLon::new(54.4543, -3.2117)),
        ("Keswick", LatLon::new(54.6005, -3.1347)),
    ] {
        let t = std::time::Instant::now();
        let Some(h) = cast(&dem, vp, &params) else {
            println!("{name}: viewpoint outside DEM coverage");
            continue;
        };
        let (b, el) = h.highest();
        println!("== {name} ==  ground {:.0} m  (cast {:.2}s)", h.eye_ground_m, t.elapsed().as_secs_f32());
        println!("   highest skyline: {:.1}° elevation, bearing {:.0}°", el, b);
        print!("   by compass:");
        for (deg, lbl) in compass {
            print!("  {lbl} {:+.1}°", (h.at_bearing_deg(deg) as f64).to_degrees());
        }
        println!("\n");
    }
    Ok(())
}
