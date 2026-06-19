//! Extract walker paths from the Cumbria OSM extract and summarise — smoke test
//! for B6a.
//!
//!   cargo run -p mapdata --example dump_paths --release --offline

use mapdata::{load_paths, PathKind};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| r"C:\maps\airhorizon\data\cumbria-latest.osm.pbf".to_string());

    println!("reading {path} ...");
    let paths = load_paths(&path)?;

    let (mut foot, mut bridle, mut track, mut pts) = (0usize, 0usize, 0usize, 0usize);
    let (mut min_lat, mut max_lat, mut min_lon, mut max_lon) =
        (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
    for w in &paths {
        match w.kind {
            PathKind::Foot => foot += 1,
            PathKind::Bridleway => bridle += 1,
            PathKind::Track => track += 1,
        }
        pts += w.points.len();
        for &(la, lo) in &w.points {
            min_lat = min_lat.min(la);
            max_lat = max_lat.max(la);
            min_lon = min_lon.min(lo);
            max_lon = max_lon.max(lo);
        }
    }
    // Longest single segment (metres): after the gap-split this should be small
    // (<= the 2 km cap), confirming no straight shortcuts across terrain remain.
    let mut longest = 0.0f64;
    for w in &paths {
        for s in w.points.windows(2) {
            let (la1, lo1) = s[0];
            let (la2, lo2) = s[1];
            let r = 6_371_000.0;
            let dlat = (la2 - la1).to_radians();
            let dlon = (lo2 - lo1).to_radians();
            let h = (dlat / 2.0).sin().powi(2)
                + la1.to_radians().cos() * la2.to_radians().cos() * (dlon / 2.0).sin().powi(2);
            longest = longest.max(2.0 * r * h.sqrt().asin());
        }
    }
    println!(
        "{} polylines ({foot} foot, {bridle} bridleway, {track} track), {pts} points total",
        paths.len()
    );
    println!("bounds: lat {min_lat:.3}..{max_lat:.3}  lon {min_lon:.3}..{max_lon:.3}");
    println!("longest segment: {longest:.0} m");
    if let Some(first) = paths.iter().find(|w| w.points.len() >= 3) {
        println!("sample {:?}: {} pts, first {:?}", first.kind, first.points.len(), first.points[0]);
    }
    Ok(())
}
