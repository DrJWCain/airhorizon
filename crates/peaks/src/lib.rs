//! Database of British & Irish Hills (DoBIH): load the CSV and query summits.
//!
//! Used both for map peak labels and (later) AirHorizon's horizon engine, which
//! needs every summit within range of a viewpoint filtered by prominence. Peaks
//! are held in memory with an `rstar` R-tree over (lon, lat) — the CSV is small
//! (~21k rows), so no on-disk database is needed.

use std::path::Path;

use rstar::primitives::GeomWithData;
use rstar::{RTree, AABB};

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("csv: {0}")]
    Csv(#[from] csv::Error),
    #[error("io: {0}")]
    Io(#[from] std::io::Error),
    #[error("DoBIH CSV missing expected column: {0}")]
    MissingColumn(&'static str),
}

/// One summit.
#[derive(Debug, Clone)]
pub struct Peak {
    pub name: String,
    pub lat: f64,
    pub lon: f64,
    /// Summit height in metres.
    pub height_m: f64,
    /// Prominence ("drop") in metres — the key importance filter.
    pub prominence_m: f64,
    /// DoBIH classification codes (e.g. contains "W" for Wainwrights).
    pub classification: String,
}

impl Peak {
    /// True if this hill carries DoBIH classification code `code` (e.g. "W").
    pub fn has_class(&self, code: &str) -> bool {
        self.classification.split(',').any(|c| c.trim() == code)
    }
}

type IndexedPoint = GeomWithData<[f64; 2], usize>; // [lon, lat] -> index into `peaks`

pub struct Peaks {
    peaks: Vec<Peak>,
    rtree: RTree<IndexedPoint>,
}

impl Peaks {
    pub fn len(&self) -> usize {
        self.peaks.len()
    }
    pub fn is_empty(&self) -> bool {
        self.peaks.is_empty()
    }

    /// Load from a DoBIH CSV file (columns accessed by header name).
    pub fn load_csv(path: impl AsRef<Path>) -> Result<Self, Error> {
        let mut rdr = csv::ReaderBuilder::new().flexible(true).from_path(path)?;
        let headers = rdr.headers()?.clone();
        let col = |name: &'static str| headers.iter().position(|h| h == name).ok_or(Error::MissingColumn(name));
        let (i_name, i_lat, i_lon, i_h, i_drop, i_class) = (
            col("Name")?,
            col("Latitude")?,
            col("Longitude")?,
            col("Metres")?,
            col("Drop")?,
            col("Classification")?,
        );

        let mut peaks = Vec::new();
        for rec in rdr.records() {
            let r = rec?;
            let get = |i: usize| r.get(i).unwrap_or("").trim();
            let (lat, lon) = (get(i_lat).parse::<f64>(), get(i_lon).parse::<f64>());
            let (Ok(lat), Ok(lon)) = (lat, lon) else { continue }; // skip rows without coords
            peaks.push(Peak {
                name: get(i_name).to_string(),
                lat,
                lon,
                height_m: get(i_h).parse().unwrap_or(0.0),
                prominence_m: get(i_drop).parse().unwrap_or(0.0),
                classification: get(i_class).to_string(),
            });
        }

        let nodes: Vec<IndexedPoint> =
            peaks.iter().enumerate().map(|(i, p)| GeomWithData::new([p.lon, p.lat], i)).collect();
        let rtree = RTree::bulk_load(nodes);
        Ok(Peaks { peaks, rtree })
    }

    /// Peaks whose (lon, lat) falls inside the bounding box.
    pub fn in_bbox(&self, min_lon: f64, min_lat: f64, max_lon: f64, max_lat: f64) -> Vec<&Peak> {
        let aabb = AABB::from_corners([min_lon, min_lat], [max_lon, max_lat]);
        self.rtree
            .locate_in_envelope_intersecting(&aabb)
            .map(|g| &self.peaks[g.data])
            .collect()
    }

    /// Peaks within `range_m` great-circle metres of (lat, lon). For the horizon
    /// engine; queries a lat/lon envelope then refines by true distance.
    pub fn within_range(&self, lat: f64, lon: f64, range_m: f64) -> Vec<&Peak> {
        let dlat = range_m / 111_320.0;
        let dlon = range_m / (111_320.0 * lat.to_radians().cos().abs().max(0.01));
        let aabb = AABB::from_corners([lon - dlon, lat - dlat], [lon + dlon, lat + dlat]);
        self.rtree
            .locate_in_envelope_intersecting(&aabb)
            .map(|g| &self.peaks[g.data])
            .filter(|p| haversine_m(lat, lon, p.lat, p.lon) <= range_m)
            .collect()
    }
}

fn haversine_m(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let r = 6_371_000.0;
    let (a1, a2) = (lat1.to_radians(), lat2.to_radians());
    let dlat = (lat2 - lat1).to_radians();
    let dlon = (lon2 - lon1).to_radians();
    let h = (dlat / 2.0).sin().powi(2) + a1.cos() * a2.cos() * (dlon / 2.0).sin().powi(2);
    2.0 * r * h.sqrt().asin()
}
