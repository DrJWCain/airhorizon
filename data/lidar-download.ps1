<#
.SYNOPSIS
  Download Environment Agency LIDAR 1 m DTM tiles for Wasdale / the Lake District
  from the DEFRA Data Services Platform tile API (the backend the
  environment.data.gov.uk/survey portal actually uses). No account needed;
  the portal's public subscription key 'dspui' is used. OGL licence.

.DESCRIPTION
  One POST to the survey "search" endpoint with a GeoJSON polygon returns every
  available tile (product / year / resolution / download uri) for that area.
  We filter to the chosen product at 1 m, keep the latest year per 5 km tile,
  and stream each one to disk as a .zip (GeoTIFF/ASC inside, EPSG:27700).

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File .\lidar-download.ps1
  powershell -ExecutionPolicy Bypass -File .\lidar-download.ps1 -Scope lakes
  powershell -ExecutionPolicy Bypass -File .\lidar-download.ps1 -Discover
  powershell -ExecutionPolicy Bypass -File .\lidar-download.ps1 -Product national_lidar_programme_dtm

.NOTES
  WARNING: tiles are ~20-70 MB each. Wasdale is ~30 tiles (~1-2 GB); whole
  Cumbria is tens of GB. Verified working 2026-06-17.
#>
[CmdletBinding()]
param(
  [string] $Dest    = 'C:\maps\airhorizon\data\lidar',
  [ValidateSet('wasdale','lakes','cumbria')] [string] $Scope = 'wasdale',
  [string] $Product = 'lidar_composite_dtm',   # or 'national_lidar_programme_dtm'
  [string] $ResId   = '1',                      # resolution id: '1' = 1 m, '2' = 2 m
  [string] $GeoJsonFile,                         # optional: path to your own GeoJSON polygon (WGS84)
  [switch] $Discover,                            # list available products/years/res for the area, then stop
  [int]    $TimeoutSec = 120
)

$ErrorActionPreference = 'Stop'
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12

$searchUrl = 'https://environment.data.gov.uk/tiles/collections/survey/search'
$key       = 'dspui'
$headers   = @{ 'Ocp-Apim-Subscription-Key' = $key }

# Area polygons (WGS84 lon/lat). Literal strings to avoid any locale/number issues.
$polys = @{
  wasdale = '{"type":"Polygon","coordinates":[[[-3.40,54.40],[-3.15,54.40],[-3.15,54.52],[-3.40,54.52],[-3.40,54.40]]]}'
  lakes   = '{"type":"Polygon","coordinates":[[[-3.45,54.35],[-2.75,54.35],[-2.75,54.75],[-3.45,54.75],[-3.45,54.35]]]}'
  cumbria = '{"type":"Polygon","coordinates":[[[-3.70,54.00],[-2.30,54.00],[-2.30,55.20],[-3.70,55.20],[-3.70,54.00]]]}'
}
if ($GeoJsonFile) { $geojson = Get-Content -Raw -Path $GeoJsonFile } else { $geojson = $polys[$Scope] }

New-Item -ItemType Directory -Force -Path $Dest | Out-Null
$log = Join-Path $Dest 'lidar-download.log'
function Log($m) { $line = '[{0}] {1}' -f (Get-Date -Format HH:mm:ss), $m; Write-Host $line; Add-Content -Path $log -Value $line }

Log ("Scope={0} Product={1} ResId={2} dest={3}" -f $Scope, $Product, $ResId, $Dest)

# 1. Search: POST the polygon, get the tile catalogue for the area.
try {
  $resp = Invoke-RestMethod -Uri $searchUrl -Method Post -Headers $headers `
            -ContentType 'application/geo+json' -Body $geojson -TimeoutSec $TimeoutSec
} catch {
  Log "FATAL search failed: $($_.Exception.Message)"
  throw
}
$all = @($resp.results)
Log ("search returned {0} tile-records" -f $all.Count)

if ($Discover) {
  Log 'Available products / resolutions / years in this area:'
  $all | Group-Object { '{0,-34} {1,-3} {2}' -f $_.product.id, ($_.resolution.label), $_.year.id } |
    Sort-Object Name | ForEach-Object { Log ('  {0}  x{1}' -f $_.Name, $_.Count) }
  Log 'Discover done. Pick -Product / -ResId and re-run without -Discover.'
  return
}

# 2. Filter to the wanted product + resolution; keep the latest year per tile.
$want = $all | Where-Object { $_.product.id -eq $Product -and $_.resolution.id -eq $ResId }
if (-not $want) {
  Log "No tiles matched product='$Product' resId='$ResId'. Run with -Discover to see what's available."
  return
}
$tiles = $want | Group-Object { $_.tile.id } |
  ForEach-Object { $_.Group | Sort-Object { [int]$_.year.id } -Descending | Select-Object -First 1 }
Log ("{0} distinct tiles to fetch (latest year each)" -f @($tiles).Count)

# 3. Download each tile (streamed). Some tiles are slow server-side and exceed
# the default 100 s WebClient timeout, so use a subclass with a long timeout
# and retry a couple of times before giving up.
Add-Type -TypeDefinition @"
using System;
using System.Net;
public class LongWebClient : WebClient {
  public int TimeoutMs = 600000;
  protected override WebRequest GetWebRequest(Uri address) {
    WebRequest w = base.GetWebRequest(address);
    if (w != null) { w.Timeout = TimeoutMs; ((HttpWebRequest)w).ReadWriteTimeout = TimeoutMs; }
    return w;
  }
}
"@
$ok = 0; $skip = 0; $fail = 0
foreach ($t in $tiles) {
  $out = Join-Path $Dest ($t.label + '.zip')
  if ((Test-Path $out) -and ((Get-Item $out).Length -gt 0)) { $skip++; continue }
  $url = $t.uri + '?subscription-key=' + $key
  $done = $false
  for ($try = 1; $try -le 3 -and -not $done; $try++) {
    $wc = New-Object LongWebClient
    try {
      Log ("GET {0}{1}" -f $t.label, $(if ($try -gt 1) { " (attempt $try)" } else { "" }))
      $wc.DownloadFile($url, $out)
      Log ("OK  {0}  ({1} MB)" -f $t.label, [math]::Round((Get-Item $out).Length/1MB,1))
      $ok++; $done = $true
    } catch {
      Log ("WARN {0} attempt {1}: {2}" -f $t.label, $try, $_.Exception.Message)
      if (Test-Path $out) { Remove-Item $out -Force }
    } finally { $wc.Dispose() }
  }
  if (-not $done) { Log ("FAIL {0} (gave up after 3 tries)" -f $t.label); $fail++ }
}
Log ("DONE downloaded={0} skipped(existing)={1} failed={2}  -> {3}" -f $ok, $skip, $fail, $Dest)
