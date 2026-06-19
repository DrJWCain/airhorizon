//! Decode one Zoomstack tile and print its layers — a smoke test for B2.
//!
//!   cargo run -p basemap --example dump_tile --offline
//!   cargo run -p basemap --example dump_tile --offline -- <mbtiles> <lat> <lon> <zoom>
//!
//! Defaults to the downloaded Zoomstack pack centred on Keswick.

use basemap::Mbtiles;
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
        "metadata: name={:?} format={:?} zoom={:?}..{:?} bounds={:?}",
        meta.name, meta.format, meta.minzoom, meta.maxzoom, meta.bounds
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
        Some(dec) => {
            println!("{} layers, {} features total:", dec.layers.len(), dec.total_features());
            for l in &dec.layers {
                println!(
                    "  {:<16} v{} extent={} feats={:<5} (pt {}, line {}, poly {})  keys: {}",
                    l.name,
                    l.version,
                    l.extent,
                    l.features,
                    l.points,
                    l.lines,
                    l.polygons,
                    l.keys.join(", ")
                );
            }
        }
    }
    Ok(())
}
