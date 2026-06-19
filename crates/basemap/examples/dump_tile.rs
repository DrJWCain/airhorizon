//! Decode one Zoomstack tile and print its layers — a smoke test for B2/B3a.
//!
//!   cargo run -p basemap --example dump_tile --offline
//!   cargo run -p basemap --example dump_tile --offline -- <mbtiles> <lat> <lon> <zoom>
//!
//! Defaults to the downloaded Zoomstack pack centred on Keswick.

use basemap::{GeomKind, Mbtiles};
use geodesy::{LatLon, Tile};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let path = args
        .next()
        .unwrap_or_else(|| r"C:\maps\airhorizon\data\OS_Open_Zoomstack.mbtiles".to_string());
    let lat: f64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(54.6012);
    let lon: f64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(-3.1399);
    let want_zoom: Option<u8> = args.next().and_then(|s| s.parse().ok());

    let mbt = Mbtiles::open(&path)?;
    let meta = mbt.metadata()?;
    println!("== {path}");
    println!(
        "metadata: name={:?} format={:?} zoom={:?}..{:?}",
        meta.name, meta.format, meta.minzoom, meta.maxzoom
    );

    let zoom = want_zoom
        .or(meta.maxzoom)
        .unwrap_or(14)
        .min(meta.maxzoom.unwrap_or(14));
    let tile = Tile::containing(LatLon::new(lat, lon), zoom);
    println!(
        "\ntile for ({lat}, {lon}) @ z{zoom}: x={} y={} (tms_row={})",
        tile.x,
        tile.y,
        tile.tms_y()
    );

    match mbt.decode_tile(tile)? {
        None => println!("(no tile stored here — try a different zoom or location)"),
        Some(vt) => {
            println!("{} layers, {} features total:", vt.layers.len(), vt.total_features());
            for l in &vt.layers {
                // Sanity: confirm decoded coords sit within the tile extent.
                let mut max_c = 0f32;
                for f in &l.features {
                    for part in &f.parts {
                        for p in part {
                            max_c = max_c.max(p[0].abs()).max(p[1].abs());
                        }
                    }
                }
                println!(
                    "  {:<16} extent={} feats={:<5} (pt {}, line {}, poly {})  max|coord|={:.0}  keys: {}",
                    l.name,
                    l.extent,
                    l.features.len(),
                    l.count(GeomKind::Point),
                    l.count(GeomKind::Line),
                    l.count(GeomKind::Polygon),
                    max_c,
                    l.keys.join(", ")
                );
                // For the names layer, show a few resolved labels (verifies B5a).
                if l.name == "names" {
                    for f in l.features.iter().take(6) {
                        if let Some(n) = f.attr("name1") {
                            println!("      name1={:?} type={:?}", n, f.attr("type"));
                        }
                    }
                }
            }
        }
    }
    Ok(())
}
