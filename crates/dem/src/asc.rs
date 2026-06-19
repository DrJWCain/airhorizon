//! Reader for ESRI ASCII Grid (`.asc`) files as used by OS Terrain 50.
//!
//! Each file is a 10 km × 10 km tile holding a 200 × 200 grid of elevations
//! at 50 m spacing, anchored to a BNG easting/northing south-west corner.
//! Header is six text lines; the data is whitespace-separated floats laid
//! out row-major from the *top* row (highest northing) downward.
//!
//! Memory model: parsing the full ~700 KB grid takes ~10 ms once. We hand
//! out parsed tiles by value and let the caller cache them.

use std::fs;
use std::io;
use std::path::Path;

/// Header metadata for an ASCII Grid tile.
#[derive(Debug, Clone, Copy)]
pub struct AscHeader {
    pub ncols: usize,
    pub nrows: usize,
    pub xllcorner: f64,
    pub yllcorner: f64,
    pub cellsize: f64,
    pub nodata: f64,
}

impl AscHeader {
    /// True if this tile's footprint includes the given BNG point.
    pub fn contains(&self, e: f64, n: f64) -> bool {
        let w = self.ncols as f64 * self.cellsize;
        let h = self.nrows as f64 * self.cellsize;
        e >= self.xllcorner
            && e < self.xllcorner + w
            && n >= self.yllcorner
            && n < self.yllcorner + h
    }
}

/// A parsed tile: header plus all cells. Elevation in metres; cells equal to
/// `header.nodata` are treated as missing (return None from `sample_bilinear`).
#[derive(Debug)]
pub struct AscTile {
    pub header: AscHeader,
    /// Row-major, top-to-bottom (row 0 is the highest northing).
    pub data: Vec<f32>,
}

impl AscTile {
    /// Bilinear-interpolated elevation at BNG point (e, n). Returns None if
    /// the point falls outside the tile's footprint or hits a NODATA cell.
    ///
    /// Cells are treated as samples at their centres (ESRI `xllcorner`
    /// convention). Points inside the tile but within ½ cell of an edge clamp
    /// to the nearest cell rather than degenerating to NODATA.
    pub fn sample_bilinear(&self, e: f64, n: f64) -> Option<f64> {
        let h = &self.header;
        let cs = h.cellsize;
        let w = h.ncols as f64 * cs;
        let height = h.nrows as f64 * cs;
        if e < h.xllcorner || e > h.xllcorner + w || n < h.yllcorner || n > h.yllcorner + height {
            return None;
        }
        let max_col = (h.ncols - 1) as f64;
        let max_row = (h.nrows - 1) as f64;
        let col_f = ((e - h.xllcorner) / cs - 0.5).clamp(0.0, max_col);
        let row_f = (h.nrows as f64 - 0.5 - (n - h.yllcorner) / cs).clamp(0.0, max_row);
        let c0 = col_f.floor() as usize;
        let r0 = row_f.floor() as usize;
        let c1 = (c0 + 1).min(h.ncols - 1);
        let r1 = (r0 + 1).min(h.nrows - 1);
        let fx = col_f - c0 as f64;
        let fy = row_f - r0 as f64;
        let g = |r: usize, c: usize| -> Option<f64> {
            let v = self.data[r * h.ncols + c] as f64;
            if v == h.nodata {
                None
            } else {
                Some(v)
            }
        };
        let v00 = g(r0, c0)?;
        let v10 = g(r0, c1)?;
        let v01 = g(r1, c0)?;
        let v11 = g(r1, c1)?;
        let top = v00 * (1.0 - fx) + v10 * fx;
        let bot = v01 * (1.0 - fx) + v11 * fx;
        Some(top * (1.0 - fy) + bot * fy)
    }
}

/// Read just the six header lines without parsing the grid body.
pub fn read_header(path: &Path) -> io::Result<AscHeader> {
    let text = fs::read_to_string(path)?;
    parse_header(&text)
}

/// Parse a full tile (header + grid).
pub fn read_tile(path: &Path) -> io::Result<AscTile> {
    let text = fs::read_to_string(path)?;
    let (header, data_start) = parse_header_with_offset(&text)?;
    let body = &text[data_start..];
    let mut data: Vec<f32> = Vec::with_capacity(header.ncols * header.nrows);
    for tok in body.split_ascii_whitespace() {
        let v: f32 = tok.parse().map_err(|e| {
            io::Error::new(io::ErrorKind::InvalidData, format!("bad cell {tok:?}: {e}"))
        })?;
        data.push(v);
    }
    let expected = header.ncols * header.nrows;
    if data.len() != expected {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("expected {expected} cells, got {}", data.len()),
        ));
    }
    Ok(AscTile { header, data })
}

fn parse_header(text: &str) -> io::Result<AscHeader> {
    parse_header_with_offset(text).map(|(h, _)| h)
}

/// Parse the header section and report the byte offset where data begins.
fn parse_header_with_offset(text: &str) -> io::Result<(AscHeader, usize)> {
    let mut ncols = None;
    let mut nrows = None;
    let mut xll = None;
    let mut yll = None;
    let mut cell = None;
    let mut nodata = -9999.0_f64;
    let mut byte_offset = 0usize;
    for line in text.lines() {
        let line_len_with_nl = line.len()
            + if text[byte_offset + line.len()..].starts_with("\r\n") {
                2
            } else if text[byte_offset + line.len()..].starts_with('\n') {
                1
            } else {
                0
            };
        let mut it = line.split_ascii_whitespace();
        let key = it.next().unwrap_or("").to_ascii_lowercase();
        let val = it.next().unwrap_or("");
        let recognised = match key.as_str() {
            "ncols" => {
                ncols = val.parse().ok();
                true
            }
            "nrows" => {
                nrows = val.parse().ok();
                true
            }
            "xllcorner" | "xllcenter" => {
                xll = val.parse().ok();
                true
            }
            "yllcorner" | "yllcenter" => {
                yll = val.parse().ok();
                true
            }
            "cellsize" => {
                cell = val.parse().ok();
                true
            }
            "nodata_value" => {
                if let Ok(v) = val.parse() {
                    nodata = v;
                }
                true
            }
            _ => false,
        };
        if !recognised {
            break;
        }
        byte_offset += line_len_with_nl;
    }
    let header = AscHeader {
        ncols: ncols.ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "missing ncols"))?,
        nrows: nrows.ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "missing nrows"))?,
        xllcorner: xll.ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "missing xllcorner"))?,
        yllcorner: yll.ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "missing yllcorner"))?,
        cellsize: cell.ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "missing cellsize"))?,
        nodata,
    };
    Ok((header, byte_offset))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn synthetic_tile() -> AscTile {
        AscTile {
            header: AscHeader {
                ncols: 2,
                nrows: 2,
                xllcorner: 1000.0,
                yllcorner: 2000.0,
                cellsize: 50.0,
                nodata: -9999.0,
            },
            data: vec![10.0, 20.0, 30.0, 40.0],
        }
    }

    #[test]
    fn bilinear_centre() {
        let t = synthetic_tile();
        let v = t.sample_bilinear(1050.0, 2050.0).unwrap();
        assert!((v - 25.0).abs() < 1e-9, "got {v}");
    }

    #[test]
    fn bilinear_cell_centre() {
        let t = synthetic_tile();
        let v = t.sample_bilinear(1025.0, 2025.0).unwrap();
        assert!((v - 30.0).abs() < 1e-9, "got {v}");
    }

    #[test]
    fn outside_returns_none() {
        let t = synthetic_tile();
        assert!(t.sample_bilinear(5000.0, 5000.0).is_none());
    }

    #[test]
    fn parses_header() {
        let text = "ncols 4\nnrows 3\nxllcorner 100\nyllcorner 200\ncellsize 50\nNODATA_value -9999\n";
        let h = parse_header(text).unwrap();
        assert_eq!(h.ncols, 4);
        assert_eq!(h.cellsize, 50.0);
        assert_eq!(h.nodata, -9999.0);
    }
}
