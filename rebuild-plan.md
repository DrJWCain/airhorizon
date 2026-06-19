# qct-viewer open-data rebuild -> AirHorizon

Detailed build plan. Rebuild the QCT raster viewer on free/open **vector** data,
then grow it into **AirHorizon** (offline horizon + AR). Architecture decided
2026-06-18; revised for offline-first 2026-06-19.

Existing code: Rust workspace at `C:\maps\qct-viewer` (winit 0.30 + wgpu 23).
Builds fully offline today. Data pack + warmed cargo cache already on disk
(see memory `opendata-rebuild`).

-----

## 0. Architecture (DECIDED)

**Base map = live vector.** Render OS Open Zoomstack Mapbox Vector Tiles (MVT)
directly at runtime. No prep-time rasteriser, no frozen style, nothing to
install -- buildable entirely offline now (all crates cached).

**Foreground = vector overlays** drawn on top of the base, the pattern the
viewer ALREADY uses for routes/GPS (`route.wgsl`, `overlay.wgsl`): OSM footpaths,
peak labels, the GPS dot, routes, and later the AirHorizon horizon line + AR
billboards. This is where dynamic/toggleable/queryable behaviour lives, and it
is exactly the geometry+text stack AirHorizon needs anyway.

**Raster bake = optional fallback, not a dependency.** If live vector proves too
heavy on the Surface GPU, bake Zoomstack -> XYZ raster tiles at home (needs a
one-time rasteriser install = the ONLY residual wifi touch) and render them
through the existing tile pipeline. Demoted to a perf escape hatch.

**Everything at runtime is 100% offline.** Disk reads only; nothing phones home.

**Coordinate model.** Internal frame becomes **Web Mercator (EPSG:3857)** -- the
Zoomstack/slippy-map tiling scheme. Three transforms, all owned by one crate:
- WGS84 <-> Web Mercator: closed-form spherical mercator (cheap, per-frame ok).
- WGS84/ETRS89 <-> OSGB36 BNG: **OSTN15** NTv2 grid (the downloaded .tif),
  bilinear -- for DEM sampling, horizon, peaks (all BNG). Replaces the Helmert
  ~5 m approximation in `dem/osgb.rs`.
- lon/lat <-> tile z/x/y and tile -> mercator/screen bounds.

-----

## 1. Workspace layout

```
crates/
  geodesy/   NEW  Web Mercator + OSTN15 BNG + tile math. Pure, no GPU. (B1)
  basemap/   NEW  MBTiles (rusqlite) + MVT decode (flate2+geozero) -> typed
                  features by z/x/y. Pure, no GPU. Mirrors the qct decoupling rule. (B2)
  mapdata/   NEW  OSM .pbf (osmpbf) -> footpaths/bridleways/walls/crags as
                  styled vector layers. Prep-time pre-extract to a compact file. (B6)
  peaks/     NEW  DoBIH CSV -> SQLite + R-tree (rstar). Range/prominence queries.
                  SHARED with AirHorizon peak labels. (B7 / A3)
  style/     NEW  layer+zoom -> draw style (fill/line/label) map. A "fell" style.
                  Tiny; may live inside render at first. (B3-B5)
  prep/      NEW  pre-extract OSM, build peaks DB, write manifest.json; later the
                  optional raster bake. Runs at home. (B6/B7, B8)
  dem/       KEEP Terrain 50 now; add LIDAR 1 m (Wasdale tiles on disk) + mip
                  access for AirHorizon foreground. Re-point osgb.rs at geodesy.
  sensor/    KEEP GPS now; attitude packet later (AirHorizon A4/iOS).
  view/      EVOLVE qct-view -> vector renderer (tessellation + SDF labels +
                  tile streaming) + existing pan/zoom/touch/GPS/routes/profile.
  qct, qct2png, catalog, qct-catalog  KEEP building (legacy raster); off the main
                  app path. Option: keep QCT as a selectable raster base layer.
```

Cached deps backing the new crates (offline-ready): `rusqlite`(bundled, +R-tree),
`geozero`(with-mvt), `prost`, `flate2`, `osmpbf`, `csv`, `tiff`, `geo`, `rstar`,
`earcutr`, `lyon`, `fontdue`.

-----

## 2. Rendering pipeline (the new core in `view/`)

Keep the proven streaming design from the QCT renderer -- it transfers from
textures to geometry unchanged:
- **Candidate tiles**: z/x/y around the view centre for the current zoom.
- **Centre-out sort + small per-frame budget** (the concentric-fill the user
  values -- preserve `MAX_UPLOADS_PER_FRAME`-style throttle; see feedback memory).
- **Tile cache** keyed by z/x/y; keep `CACHE_SIZE > iteration-window` invariant.
  Now caches built GPU vertex/index buffers, not textures.

Per tile, once decoded by `basemap`:
1. **Polygons** (woodland/water/buildings/foreshore): `earcutr` triangulation
   -> indexed triangles, colour per `style`. Drawn first (bottom).
2. **Lines** (roads/rail/contours): `lyon` stroke tessellation with screen-space
   width -> triangles. Drawn over fills.
3. **Labels** (names): collect point/line label features; place text from a
   `fontdue` SDF atlas as billboards. Drawn last (top). Hardest part -- defer to B5.

MVT mechanics: tile geometry is integer tile-local (0..4096 extent). Transform
tile coords -> Web Mercator -> screen with the view matrix. Use f64 for the
transform chain, f32 for GPU buffers; watch precision at high zoom.

Overlays share the same tessellation/label machinery, drawn after the base:
footpaths (lines), peaks (labels), GPS dot + routes (already implemented), and
later the AirHorizon horizon polyline + AR billboards.

-----

## 3. Build order (each step ships something runnable; all offline unless noted)

**B1 - geodesy.** Web Mercator <-> WGS84; OSTN15 NTv2 reader (parse the .tif grid,
bilinear) for WGS84 <-> OSGB36; tile math. Validate against the existing
`osgb.rs` tests (Greenwich, Caister) + one HeyWhatsThat viewpoint. Re-point
`dem/osgb.rs` at it. *Decision-independent; do first.*

**B2 - basemap reader.** Open `OS_Open_Zoomstack.mbtiles` (rusqlite; note TMS
y-flip), fetch tile by z/x/y, gunzip (flate2), decode MVT (geozero) -> typed
features (layer, geometry, attrs). Read layer names from the mbtiles `metadata`
`json`. Deliverable: an example that dumps a tile's layers + feature counts for a
known spot (e.g. Keswick) to confirm decode. No GPU yet.

**B3 - vector render MVP (lines only).** Window renders Zoomstack roads/water/
contours as tessellated lines in Web Mercator, reusing pan/zoom/touch/inertia and
the centre-out tile streaming. *This replaces QCT as the live map.* No fills/labels.

**B4 - polygon fills.** Woodland/water/buildings/foreshore via `earcutr`, styled
by `style`. Now a recognisable topographic base map.

**B5 - labels (SDF).** Place-name labels via `fontdue` SDF atlas + a basic
placement pass. *This is AirHorizon's label system, not a detour.*

**B6 - footpath overlay.** `prep` pre-extracts OSM paths/bridleways/walls/crags
from `cumbria-latest.osm.pbf` (osmpbf) to a compact file; `view` draws them as a
toggleable styled line overlay. The walker-critical layer OS OpenData lacks.

**B7 - peaks + parity.** `peaks` crate (DoBIH CSV -> SQLite/R-tree); label visible
peaks. Port GPS-follow, routes, elevation profile, and `[`/`]` detail-switch onto
the new renderer. Open-data viewer reaches QCT parity (and beats it on paths).

**B8 - raster-bake fallback (only if needed, needs wifi once).** Bake Zoomstack ->
XYZ raster tiles at home; add a raster base path to the renderer. Reach for this
only if B3-B4 live vector is too slow on the Surface.

**AirHorizon track** (reuses geodesy/peaks/dem/labels/render directly):
- **A1** horizon ray-caster: CPU first over Terrain 50 + Wasdale LIDAR 1 m, then
  WGSL compute (3600 azimuth buckets, mip-walk, R_eff=R/(1-k)). Validate vs
  HeyWhatsThat from Scafell Pike / Great Gable.
- **A2** panorama renderer (horizon strip; scrub azimuth) -- reuses `view`.
- **A3** peak labels above the horizon (visibility test vs the horizon buffer;
  reuse `peaks` + the SDF atlas from B5).
- **A4** AR mode: needs device **attitude** -> custom iOS sender (GPS2IP gives
  position only). Bearing-nudge gesture for magnetometer error.
- **A5** "look from here": tap the map -> panorama at that point. Cache Wainwright
  horizons.
- **A6** LIDAR foreground detail in `dem` for near-field horizon (Wasdale tiles
  already downloaded).

-----

## 4. Risks & sub-decisions
- **GPU perf of live vector on the Surface** -- biggest unknown. Mitigations:
  tile cache + small per-frame budget + B8 raster fallback.
- **Label placement/collision** -- the hard part of any vector map; iterate, keep
  it simple first (point labels, prominence-ranked, drop on overlap).
- **Zoomstack style authoring** -- read actual layer names from mbtiles metadata;
  author a minimal fell style; not pixel-matching OS Explorer (copyrighted).
- **Mercator vs BNG separation** -- `geodesy` is the single bridge; keep DEM/
  horizon/peaks strictly in BNG, display strictly in Mercator.
- **OSM parse cost** -- pre-extract in `prep` rather than parse 43 MB at startup.
- **Keep the qct/render decoupling rule**: `basemap`/`geodesy`/`peaks` stay
  GPU-unaware; tessellation/upload lives only in `view`.

-----

## 5. Data pack (on disk, `C:\maps\airhorizon\data\`)
Zoomstack mbtiles (2.7 GB), OSM Cumbria (43 MB), DoBIH peaks, OS Open Names,
OSTN15 grid, 15x Wasdale LIDAR 1 m tiles (1.2 GB), OS Terrain 50 at
`C:\maps\OS Terrain 50`. Licences: OS = OGL (attribute "Contains OS data (c)
Crown copyright"); OSM = ODbL ("(c) OpenStreetMap contributors"). Deferred: EA
LIDAR beyond Wasdale, Natural England CRoW access land.
