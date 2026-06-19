#!/usr/bin/env bash
# AirHorizon / qct-viewer free-data pack downloader.
# Re-runnable: -C - resumes partial files, so safe to re-launch if interrupted.
set -u
cd "$(dirname "$0")"
log() { echo "[$(date '+%H:%M:%S')] $*"; }

get() {
  local name="$1" url="$2" out="$3"
  log "START $name -> $out"
  if curl -L --fail --retry 5 --retry-delay 3 -C - -o "$out" "$url"; then
    log "DONE  $name  ($(du -h "$out" | cut -f1))"
  else
    log "FAIL  $name (curl exit $?)"
  fi
}

# 1. OS Open Zoomstack — vector basemap (MBTiles / MVT), GB-wide. ~2.85 GB.
get "OS Open Zoomstack (MBTiles)" \
  "https://api.os.uk/downloads/v1/products/OpenZoomstack/downloads?area=GB&format=Vector+Tiles&subformat=%28MBTiles%29&redirect" \
  "OS_Open_Zoomstack.mbtiles"

# 2. OS Open Names — gazetteer CSV, GB-wide. ~103 MB.
get "OS Open Names (CSV)" \
  "https://api.os.uk/downloads/v1/products/OpenNames/downloads?area=GB&format=CSV&redirect" \
  "opname_csv_gb.zip"

# 3. OSTN15/OSGM15 NTv2 grid (WGS84/ETRS89 <-> OSGB36, ~10 cm). ~3 MB. (PROJ CDN)
get "OSTN15 NTv2 grid" \
  "https://cdn.proj.org/uk_os_OSTN15_NTv2_OSGBtoETRS.tif" \
  "uk_os_OSTN15_NTv2_OSGBtoETRS.tif"

# 4. OpenStreetMap — Cumbria extract (footpaths, walls, crags, POIs). ~60 MB. (Geofabrik, ODbL)
get "OSM Cumbria (Lake District)" \
  "https://download.geofabrik.de/europe/united-kingdom/england/cumbria-latest.osm.pbf" \
  "cumbria-latest.osm.pbf"

# 5. DoBIH — Database of British & Irish Hills, full CSV (peaks). v18.4. ~3 MB.
get "DoBIH peaks (CSV)" \
  "http://www.hill-bagging.co.uk/dobih-downloads/hillcsv.zip" \
  "dobih_hillcsv.zip"

log "ALL DONE"
