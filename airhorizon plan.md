# AirHorizon — Project Plan

An offline mountain-identifier and AR horizon viewer for the Lake District (and any region with OS open data). Runs on a Surface Pro 12 tablet that lacks GPS and IMU sensors; uses a tethered iPhone over USB-C for position and orientation.

## Context & existing assets

- **Tablet**: Surface Pro 12 (fast, Windows). No GPS, no compass, no IMU.
- **Phone**: iPhone with USB-C (15-series or newer). Supplies GPS + fused attitude.
- **Existing code**: Interactive map viewer in Rust + wgpu, already rendering tiles and handling input on the tablet.
- **Skills available**: strong C++ and Rust; comfortable with async Tokio; codec/binary work; coordinate-system maths is fine.
- **Connectivity assumption**: zero. Must work entirely offline in the fells. Cellular and Wi-Fi are unreliable on Lakeland summits.

## Goal

Stand on a fell, point the tablet at the skyline, and have it draw:

1. The computed horizon line from the user’s position (geometric, not from a camera).
1. Labels for every named summit visible above that horizon, oriented to match the way the device is pointing.
1. A “synthetic panorama” mode for when you’d rather scrub through 360° than hold the tablet up.
1. The existing moving-map view, with a “look from here” gesture that jumps into horizon mode.

-----

## Architecture

```
┌────────────────────────┐        USB-C        ┌──────────────────────────────┐
│ iPhone                 │ ◄────────────────►  │ Surface Pro 12 (tablet)      │
│ ─ CoreLocation (GPS)   │  Personal Hotspot   │ ─ Rust + wgpu app            │
│ ─ CoreMotion (attitude)│  over USB (local    │ ─ Tokio UDP listener         │
│ ─ Streamer app         │  network, no cell   │ ─ Pose state (Arc<ArcSwap>)  │
│   sends UDP packets    │  needed)            │ ─ Map viewer (existing)      │
└────────────────────────┘                     │ ─ Horizon compute pass (new) │
                                               │ ─ Panorama renderer (new)    │
                                               │ ─ AR camera mode (new)       │
                                               │ ─ Offline data pack on disk  │
                                               └──────────────────────────────┘
```

Three big subsystems to add to the existing map viewer:

- **Sensor ingest**: receive GPS + attitude from the phone, expose to render loop.
- **Horizon engine**: GPU ray-caster over the DEM that produces a 1D `elevation[azimuth]` buffer.
- **Renderers**: panorama strip and AR camera mode, both consuming that buffer plus a peak list.

-----

## iPhone over USB-C — sensor streaming

### Why USB-C and not Bluetooth/Wi-Fi

- **Power**: streaming GPS + IMU at 10–20 Hz with the screen on will drain the phone in a few hours. USB-C charges it.
- **Latency**: USB tethering is consistently 5–20 ms; Bluetooth LE is jittery.
- **Reliability**: ad-hoc Wi-Fi between phone and tablet can drop in cold/wet conditions; the wired link does not.
- **Offline**: USB tethering’s local network works even with no cellular signal at all. The phone does not need internet.

### How the USB-C link works

1. Plug iPhone (USB-C) directly into Surface Pro 12 (USB-C).
1. On Windows, install **Apple Devices** from the Microsoft Store (this installs Apple Mobile Device Support). This is what makes Windows see the iPhone as more than a camera.
1. On the iPhone: Settings → Personal Hotspot → toggle **Allow Others to Join** on.
- Cellular data is *not* required for the local network to come up.
1. Windows sees the iPhone as a new network adapter (“Apple Mobile Device Ethernet” or similar) and assigns an IP via DHCP from the phone.
1. The phone is now at a known address on that interface (typically `172.20.10.1`); the tablet gets `172.20.10.2` or similar.
1. UDP packets sent from the phone to the tablet’s IP travel over the USB link at low latency.

### iPhone-side streamer — three options

**Option A: off-the-shelf app (fastest start)**

Apps like **Sensorstream IMU+GPS**, **Sensor Logger**, or **SensorLog** stream a configurable mix of CoreLocation + CoreMotion data over UDP/TCP to a target IP and port. Pick one that exposes:

- Location: lat, lon, altitude (ellipsoidal or orthometric — note which), horizontal accuracy
- Attitude: quaternion or roll/pitch/yaw from `CMDeviceMotion`
- Rate: configurable, target 10 Hz

Document the exact packet format the chosen app uses and write a parser on the tablet side.

**Option B: custom iOS companion app (best long-term)**

Small SwiftUI app with two services:

- `CLLocationManager` with `desiredAccuracy = kCLLocationAccuracyBest`, deliver via delegate.
- `CMMotionManager.startDeviceMotionUpdates(using: .xMagneticNorthZVertical)` — gives a *fused* attitude quaternion in a north-referenced frame, with gravity already separated from user acceleration. This is dramatically better than raw IMU.

Background entitlements needed so it doesn’t get killed when the screen locks: `UIBackgroundModes` = `location`.

Packet format (suggested, little-endian, fixed 64 bytes):

|Offset|Type |Field                               |
|------|-----|------------------------------------|
|0     |u32  |magic = `0x41484F52` (“AHOR”)       |
|4     |u32  |sequence counter                    |
|8     |u64  |timestamp µs since boot             |
|16    |f64  |latitude (WGS84, degrees)           |
|24    |f64  |longitude (WGS84, degrees)          |
|32    |f32  |altitude (m, ellipsoidal)           |
|36    |f32  |horizontal accuracy (m, 1σ)         |
|40    |f32×4|attitude quaternion (w, x, y, z)    |
|56    |f32  |heading accuracy (degrees)          |
|60    |u32  |flags (bit 0: GPS fix, bit 1: cal’d)|

Send at 10 Hz. Lossy is fine — newer packet always wins.

**Option C: NMEA over virtual COM (interop only)**

Use **GPS2IP** for NMEA GPS streaming if you ever want to use third-party Windows GIS software. Not needed for the bespoke app — skip unless you want that interop.

**Recommendation**: start with Option A (Sensorstream IMU+GPS) to get the tablet side built, then move to Option B once the protocol stabilises. The tablet code is unchanged — only the parser shim differs.

### Magnetometer reality check

The fused attitude quaternion’s yaw component comes from the magnetometer. Expect:

- ±5–10° absolute bearing error in clean conditions.
- Much worse near the tablet itself (Surface speakers, pen, kickstand magnets), near vehicles, near rebar in summit cairns.
- Drift over time if the user spins around a lot.

**Mitigation**: a one-finger drag gesture on the panorama that lets the user nudge the horizon left/right to align with a known peak. Store the offset, apply it to all subsequent bearings until the user clears it. This is what PeakFinder does and it works very well in practice.

-----

## Offline data pack

All data must be on disk before leaving the house. Build a `prep` tool (Rust) that runs at home on Wi-Fi and produces a single regional pack.

### Sources (all free, mostly OGL)

|Dataset                                  |Purpose                               |Source                      |Format                   |Approx size (Lakes + 50km buffer)                        |
|-----------------------------------------|--------------------------------------|----------------------------|-------------------------|---------------------------------------------------------|
|OS Terrain 5                             |Horizon DEM                           |OS Data Hub (OpenData)      |ASC or GeoTIFF, 5km tiles|~300 MB packed                                           |
|EA National LIDAR DTM 1m                 |Foreground detail (optional)          |DEFRA Data Services Platform|GeoTIFF                  |~10 GB raw; skip globally, use only round popular summits|
|DoBIH (Database of British & Irish Hills)|Peak names, prominence, classification|hills-database.co.uk        |CSV                      |<5 MB                                                    |
|OS Open Names                            |Place labels (towns, lakes)           |OS Data Hub                 |CSV/GPKG                 |~50 MB GB-wide                                           |
|OS Open Zoomstack                        |Vector base map for moving-map view   |OS Data Hub                 |MBTiles (vector)         |~700 MB GB-wide                                          |
|OSTN15 transformation grid               |WGS84 ↔ OSGB36 accurate transform     |OS                          |Binary grid              |<10 MB                                                   |

Total: comfortably under 2 GB for the full pack.

### Repacking step

The data as supplied is not the data you want to ship.

1. **DEM**: convert all OS Terrain 5 ASC tiles to a single Cloud-Optimised GeoTIFF (COG), R32F, with internal tiling (256×256) and full mip overview pyramid built (`gdaladdo -r average` to about 1/64 resolution). The mip pyramid is essential — the ray-caster samples coarser mips at distance, which is the single biggest perf win. Final file is mmap-friendly.
1. **Peaks**: import DoBIH into SQLite + SpatiaLite. Create an R-tree index on (lon, lat). Add columns for `prominence_m`, `height_m`, `classification` (Wainwright, Birkett, Marilyn, etc.) so you can filter labels by importance at runtime.
1. **Base map**: OS Open Zoomstack ships as vector MBTiles — usable directly by MapLibre or your own vector renderer in wgpu.
1. **OSTN15**: ship as-is, load at startup.

### Pack layout on disk

```
/data/lakes-v1/
  terrain.tif          # COG with overviews
  peaks.sqlite         # SpatiaLite with R-tree
  basemap.mbtiles      # OS Open Zoomstack
  ostn15.gsb           # OSTN15 transformation grid
  manifest.json        # version, bounds, build date, source attributions
```

Make this versioned and swappable — a future “Snowdonia v1” pack should drop into the same slot.

-----

## Coordinate systems

Three frames in play:

|Frame                                                                |When used                          |
|---------------------------------------------------------------------|-----------------------------------|
|**WGS84** (lat, lon, h ellipsoidal)                                  |GPS input from phone               |
|**OSGB36 / British National Grid** (easting, northing, h orthometric)|OS Terrain 5 DEM, all OS data      |
|**Local ENU** (east, north, up; metres, viewpoint-centred)           |Ray-caster maths, screen projection|

### Transforms

- **WGS84 → OSGB36**: use OSTN15 grid (bilinear interpolation between grid nodes). Accurate to ~10 cm. Cheap Helmert approximation is only good to ~5 m and not worth using when the grid is small enough to ship.
- **OSGB36 → local ENU**: pick the GPS viewpoint as origin, treat the area as flat-Earth tangent plane (fine over 50 km), east = ΔE, north = ΔN, up = Δh.
- The **`proj` crate** (PROJ bindings) handles both directly if you’d rather not roll your own OSTN15 reader.

The DEM is in OSGB36 BNG coordinates with an associated affine transform — you sample by `(easting, northing)`, not lat/lon. Convert the viewpoint once on each GPS update; samples along the ray stay in BNG.

-----

## Horizon ray-caster (compute shader)

This is the heart of the project. A WGSL compute shader running on the Surface’s GPU.

### Inputs

- DEM as a 2D `texture_storage_2d<r32float, read>` (or sampled texture with mips) — the COG terrain pyramid.
- Uniform buffer with: viewpoint BNG easting/northing/eye height, eye height above ground (e.g. 1.6 m), max range (default 75 km), refraction factor `k` (default 0.13), DEM affine transform.

### Output

- Storage buffer `horizon_elevation: array<f32, 3600>` — apparent elevation angle (radians) at each 0.1° azimuth bucket.

### Algorithm per thread (one thread per azimuth)

```wgsl
@compute @workgroup_size(64)
fn horizon_cast(@builtin(global_invocation_id) gid: vec3<u32>) {
    let az_idx = gid.x;
    if (az_idx >= 3600u) { return; }
    let azimuth = f32(az_idx) * 0.1 * PI / 180.0;
    let dir = vec2<f32>(sin(azimuth), cos(azimuth)); // BNG east, north

    let eye_e = uniforms.viewpoint.x;
    let eye_n = uniforms.viewpoint.y;
    let h_eye = uniforms.eye_height;            // ground + observer height
    let R_eff = 6_371_000.0 / (1.0 - uniforms.refraction_k);

    var max_elev: f32 = -PI / 2.0;
    var d: f32 = 50.0; // start a little out from the viewpoint

    loop {
        if (d > uniforms.max_range) { break; }

        // Pick mip level proportional to distance: coarser when far
        let mip = clamp(floor(log2(d / 5.0) - 2.0), 0.0, 8.0);
        let step = 5.0 * pow(2.0, mip);

        let e = eye_e + dir.x * d;
        let n = eye_n + dir.y * d;
        let h_terrain = sample_dem_mip(e, n, mip);

        // Earth curvature drop + refraction baked into R_eff
        let curve_drop = (d * d) / (2.0 * R_eff);
        let dh = h_terrain - h_eye - curve_drop;
        let elev = atan2(dh, d);

        if (elev > max_elev) { max_elev = elev; }

        d = d + step;
    }

    horizon_elevation[az_idx] = max_elev;
}
```

### Notes on the algorithm

- **Mip walking**: at 50 km out you don’t need 5 m DEM resolution — sample mip 4 (80 m effective) and you’ll miss nothing the eye could resolve. This drops sample count from ~10,000 per ray to ~1,500 and is the biggest perf knob.
- **Curvature + refraction**: combined into `R_eff = R / (1 - k)`, the standard surveyor’s approximation. `k = 0.13` is the textbook value; consider 0.17 for hot summer days, 0.0 for cold dense air. Could expose as a debug slider.
- **Eye height above ground**: when GPS gives you a position, you need the ground elevation there (sample the DEM at the viewpoint itself) plus a small observer offset (say 1.6 m). Without this, you’ll get bizarre results when GPS altitude disagrees with the DEM by ±20 m, which it routinely does.
- **Workgroup size 64**: 3600 azimuths ÷ 64 = 57 workgroups. Easy.

### Timing budget

Roughly 5–15 ms on the Surface’s GPU for a full pass. Don’t recompute every frame — only when GPS has moved >50 m or eye height changed. Cache the buffer.

-----

## Peak visibility & labels

After the horizon is computed:

1. R-tree query against `peaks.sqlite` for all peaks within `max_range` of the viewpoint. Filter by prominence (e.g. `prominence_m >= 30` for Birketts and above).
1. For each candidate:
- Compute BNG offset from viewpoint, distance `d`, bearing (`atan2(ΔE, ΔN)`).
- Compute apparent elevation: `atan2(h_peak - h_eye - d²/(2·R_eff), d)`.
- Azimuth bucket = `round(bearing_deg * 10)`.
- **Visible if** `apparent_elev > horizon_elevation[bucket] + tiny epsilon`.
1. For visible peaks, store `{ name, az, el, dist, prominence }` in a list for the renderer.

Hundreds of peaks at most; do this CPU-side per pose update, it’s trivial.

### Label rendering

- Each visible peak becomes an instanced billboard.
- Instance buffer: `[az_deg, el_deg, dist_m, name_atlas_uv...]`.
- Vertex shader projects `(az, el)` through the current view/camera matrix.
- Fragment shader samples a pre-rasterised text atlas.
- Fade by distance; cluster nearby peaks; show only top-N by prominence when zoomed out.

-----

## Renderers — two camera modes sharing one buffer

### Mode 1: synthetic panorama (no orientation needed)

- A horizontal triangle strip across the screen, N vertices wide (say 720, two samples per degree).
- Vertex `y` = `horizon_elevation[az_idx]` mapped to screen space with a vertical FOV (start ~15°).
- User scrubs horizontally to pan azimuth — no phone orientation involved.
- Above the horizon line: sky gradient. Below: a hatched terrain fill or just dark.
- Peak labels float above the horizon line at the right azimuth/elevation.

This mode is the most useful one in poor visibility — works with the tablet flat on your knee.

### Mode 2: AR camera

- Build view matrix from the phone’s attitude quaternion (north-referenced, so yaw is bearing).
- Apply manual bearing offset (the magnetometer-nudge gesture).
- Project peaks and a slice of the horizon polyline through it with a sensible FOV.
- No actual camera feed needed — Surface front/rear cameras aren’t useful here.

Both modes share: the same `horizon_elevation` buffer, the same peak list, the same label atlas. Only the projection differs.

### Mode 3 (later): integration with the existing map view

A “look from here” tap on the map jumps into panorama mode with the tap point as the viewpoint. Lets the user preview the view from any location before walking there. Cache horizons for major Wainwright summits so these are instant.

-----

## Sensor integration (Tokio side)

```
[ Tokio runtime ]
        │
        ├── UDP socket on 0.0.0.0:46000
        │     │
        │     └── parse packet → Pose struct
        │
        └── Arc<ArcSwap<Pose>>  ←── render loop reads each frame
```

- One background task owns the UDP socket, parses incoming packets, and `store`s the latest `Pose` into an `ArcSwap`.
- Render loop does `pose_state.load()` once per frame — wait-free, no contention.
- `Pose` carries: `wgs84_lat_lon_h`, `bng_easting_northing_h`, `attitude_quat`, `timestamp_us`, `gps_accuracy_m`, `flags`.
- WGS84 → BNG conversion happens in the UDP task, not on the render thread.
- Low-pass filter on attitude yaw with a small time constant (~100 ms) to kill magnetometer jitter; pass through pitch/roll untouched (the gyro is excellent).
- If no packet for >2 s, set a `stale` flag — the UI should show a warning.

-----

## Project layout (Rust workspace)

```
airhorizon/
├── Cargo.toml          # workspace
├── crates/
│   ├── prep/           # offline data prep tool (downloads, repacks)
│   ├── geodesy/        # OSTN15, WGS84↔BNG, local ENU
│   ├── dem/            # COG loader, mip access
│   ├── peaks/          # SpatiaLite wrapper, R-tree queries
│   ├── sensor/         # UDP listener, Pose state
│   ├── horizon/        # compute pipeline + WGSL
│   └── render/         # wgpu renderers (extends existing viewer)
└── app/                # main binary, ties it all together
```

The existing map viewer becomes the `render` crate (or sits alongside as `render-map`).

-----

## Build order (suggested milestones)

1. **M0 — Sensor link**: get UDP packets flowing from iPhone over USB-C, displayed as raw text on the tablet. Validate the local network is up without cellular. Verify with off-the-shelf streaming app first.
1. **M1 — Data pack**: build the `prep` tool. Download Lakes region OS Terrain 5, repack to COG, build SpatiaLite peak DB from DoBIH. Verify by loading and querying on the tablet.
1. **M2 — Horizon kernel (CPU first)**: write the ray-caster in plain Rust first, single-threaded, no GPU. Validate against HeyWhatsThat from known viewpoints (e.g. Helvellyn summit, Castle Crag). Rayon-parallelise for sanity.
1. **M3 — Horizon kernel (GPU)**: port to WGSL compute shader. Verify outputs match the CPU version within float tolerance.
1. **M4 — Panorama renderer**: draw the horizon strip, scrub horizontally. No labels yet.
1. **M5 — Peak labels**: R-tree query, visibility test, billboard rendering with a text atlas.
1. **M6 — AR mode**: consume phone attitude, project peaks through view matrix, add the bearing-nudge gesture.
1. **M7 — Integration**: “look from here” tap on existing map view jumps to panorama. Cache horizons for Wainwright summits.
1. **M8 — Custom iOS app**: replace off-the-shelf streamer with a bespoke companion. Better background reliability, fixed protocol.
1. **M9 — Field test**: walk up a Wainwright, see what breaks. The list is always longer than expected.

-----

## Key decisions deferred to implementation

- **Vector vs raster base map**: OS Open Zoomstack is vector — does the existing viewer render vector tiles, or are you on raster? Affects the basemap branch of the prep tool.
- **Text atlas tooling**: signed-distance-field text via `fontdue` + atlas pack, or just rasterise per-zoom labels?
- **Eye height policy**: trust phone GPS altitude (noisy, ±20 m) or always re-ground to DEM + 1.6 m? Probably the latter unless on a structure.
- **Refraction value**: fixed 0.13, or expose a slider/auto-tune from temperature if available?
- **What “visible” means at the noise floor**: peaks just barely above horizon may be unreliable. Add a configurable margin (default 0.05°) before labelling.

-----

## Risks and mitigations

|Risk                                    |Mitigation                                                                  |
|----------------------------------------|----------------------------------------------------------------------------|
|Magnetometer wildly wrong               |Manual nudge gesture; always available                                      |
|iPhone packet stream stops in background|Use `location` background entitlement in custom app; show stale-data warning|
|GPS altitude inaccurate                 |Re-ground to DEM at viewpoint, don’t trust the phone’s vertical             |
|DEM/peak disagreement on summit position|Snap candidate peaks to local DEM maximum within 50 m radius during prep    |
|OSGB36 transform error if Helmert used  |Ship OSTN15 grid, do it properly                                            |
|Power runs out                          |USB-C charges the phone; bring an external battery for the tablet           |

-----

## References to keep on hand

- **HeyWhatsThat** (`heywhatsthat.com`) — sanity-check oracle for horizon outputs. Plug in a viewpoint and compare. Uses the same `R_eff = R / (1 - k)` refraction model.
- **OS Data Hub** (`osdatahub.os.uk`) — OS Terrain 5, OS Open Names, OS Open Zoomstack.
- **DEFRA Data Services Platform** — EA National LIDAR Programme DTM.
- **hills-database.co.uk** — DoBIH download.
- **PROJ** + the **`proj` Rust crate** — coordinate transforms including OSTN15.
- **GDAL** for prep-time tile handling; consider `gdal` crate.
- **wgpu** (already in use) for compute + render.

-----

## Quick reference — the maths

**Apparent elevation angle of a point at distance `d`, terrain height `h_t`, observer eye at `h_e`:**

```
elev = atan2(h_t - h_e - d² / (2 · R_eff), d)
R_eff = R_earth / (1 - k)   ;   R_earth ≈ 6 371 km   ;   k ≈ 0.13
```

**Curvature drop alone:**

```
drop(d) ≈ d² / (2 · R_earth) ≈ 0.0785 m per (km)²
       ≈ 7.85 m at 10 km, 196 m at 50 km
```

**Refraction reduces apparent drop by ~14%**, hence `R_eff > R_earth`.

**Bearing from viewpoint (BNG) to target (BNG):**

```
bearing = atan2(ΔE, ΔN)
```

**Azimuth bucket from bearing in degrees:**

```
bucket = round(bearing_deg * 10) mod 3600
```