//! Extract walker paths from an OpenStreetMap `.pbf` extract.
//!
//! OS Open Zoomstack has no Public Rights of Way, so footpaths/bridleways come
//! from OSM (Geofabrik Cumbria extract). Geofabrik files don't embed node
//! coordinates on ways, so we make two passes: pass 1 collects the path ways
//! and the node ids they reference; pass 2 resolves those node coordinates.
//!
//! Output is plain WGS84 lat/lon polylines — coordinate-frame-agnostic, like the
//! `basemap` decoder. The viewer projects them to Web Mercator itself.

use std::collections::{HashMap, HashSet};
use std::path::Path;

use osmpbf::{Element, ElementReader};

/// What kind of path a way is, for styling.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PathKind {
    /// Footway, path, steps, pedestrian — on-foot routes.
    Foot,
    /// Bridleway (horse/foot/cycle).
    Bridleway,
    /// Track (farm/forestry, walkable).
    Track,
}

/// A single path as an ordered WGS84 polyline.
pub struct PathWay {
    pub kind: PathKind,
    /// (lat, lon) points in order.
    pub points: Vec<(f64, f64)>,
}

/// Decide whether an OSM way is a walker path, from its tags.
fn classify<'a>(tags: impl Iterator<Item = (&'a str, &'a str)>) -> Option<PathKind> {
    let mut highway: Option<&str> = None;
    let mut designation: Option<&str> = None;
    for (k, v) in tags {
        match k {
            "highway" => highway = Some(v),
            "designation" => designation = Some(v),
            _ => {}
        }
    }
    match highway {
        Some("footway") | Some("path") | Some("steps") | Some("pedestrian")
        | Some("cycleway") => Some(PathKind::Foot),
        Some("bridleway") => Some(PathKind::Bridleway),
        Some("track") => Some(PathKind::Track),
        // A designated Right of Way without a path-y highway tag still counts.
        _ if designation.is_some() => Some(PathKind::Foot),
        _ => None,
    }
}

/// Read `pbf` and return every walker path as a WGS84 polyline.
pub fn load_paths(pbf: impl AsRef<Path>) -> Result<Vec<PathWay>, osmpbf::Error> {
    let pbf = pbf.as_ref();

    // Pass 1: path ways (kind + node-id refs) and the set of node ids we need.
    let mut ways: Vec<(PathKind, Vec<i64>)> = Vec::new();
    let mut needed: HashSet<i64> = HashSet::new();
    ElementReader::from_path(pbf)?.for_each(|el| {
        if let Element::Way(w) = el {
            if let Some(kind) = classify(w.tags()) {
                let refs: Vec<i64> = w.refs().collect();
                for &r in &refs {
                    needed.insert(r);
                }
                ways.push((kind, refs));
            }
        }
    })?;

    // Pass 2: coordinates for just those nodes.
    let mut coords: HashMap<i64, (f64, f64)> = HashMap::with_capacity(needed.len());
    ElementReader::from_path(pbf)?.for_each(|el| match el {
        Element::Node(n) => {
            if needed.contains(&n.id()) {
                coords.insert(n.id(), (n.lat(), n.lon()));
            }
        }
        Element::DenseNode(n) => {
            if needed.contains(&n.id()) {
                coords.insert(n.id(), (n.lat(), n.lon()));
            }
        }
        _ => {}
    })?;

    // Stitch node ids into coordinate polylines.
    let mut out = Vec::with_capacity(ways.len());
    for (kind, refs) in ways {
        let points: Vec<(f64, f64)> =
            refs.iter().filter_map(|r| coords.get(r).copied()).collect();
        if points.len() >= 2 {
            out.push(PathWay { kind, points });
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classify_picks_path_highways() {
        assert_eq!(classify([("highway", "footway")].into_iter()), Some(PathKind::Foot));
        assert_eq!(classify([("highway", "bridleway")].into_iter()), Some(PathKind::Bridleway));
        assert_eq!(classify([("highway", "track")].into_iter()), Some(PathKind::Track));
        assert_eq!(classify([("highway", "motorway")].into_iter()), None);
        // A Right of Way designation without a path highway still counts.
        assert_eq!(
            classify([("designation", "public_footpath")].into_iter()),
            Some(PathKind::Foot)
        );
    }
}
