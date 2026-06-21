//! Verify the 1 m LIDAR overlay decodes and georeferences correctly by
//! comparing LIDAR-vs-Terrain50 at known Wasdale points.
//!   cargo run -p dem --example lidarprobe --release --offline

use dem::Dem;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let base = Dem::open(std::path::Path::new(r"C:\maps\OS Terrain 50"))?;
    let mut lid = Dem::open(std::path::Path::new(r"C:\maps\OS Terrain 50"))?;
    let n = lid.attach_lidar(std::path::Path::new(r"C:\maps\airhorizon\data\lidar"));
    println!("attached {n} LIDAR tiles\n");

    let pts = [
        ("Wasdale Head Inn (~75)", 54.4877, -3.2967),
        ("Wast Water shore (~60)", 54.4585, -3.2917),
        ("Scafell Pike (978)", 54.4543, -3.2117),
        ("Great Gable (899)", 54.4820, -3.2192),
        ("Lingmell (807)", 54.4640, -3.2230),
    ];
    println!("{:<26} {:>10} {:>10}  {:>8}", "point", "Terrain50", "LIDAR", "diff");
    for (name, lat, lon) in pts {
        let (e, n) = geodesy::wgs84_to_bng(lat, lon);
        let b = base.elevation_bng(e, n);
        let l = lid.elevation_bng(e, n);
        let diff = match (b, l) {
            (Some(b), Some(l)) => format!("{:+.1}", l - b),
            _ => "-".into(),
        };
        println!(
            "{name:<26} {:>10} {:>10}  {diff:>8}",
            b.map(|v| format!("{v:.1}")).unwrap_or("None".into()),
            l.map(|v| format!("{v:.1}")).unwrap_or("None".into()),
        );
    }
    Ok(())
}
