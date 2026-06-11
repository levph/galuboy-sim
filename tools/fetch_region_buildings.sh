#!/usr/bin/env bash
# Build a region's building GeoJSON (footprints + nDSM-derived heights).
#
#   tools/fetch_region_buildings.sh NAME CENTER_LAT CENTER_LON SPAN_LAT SPAN_LON
#
# Example (relocated region 3):
#   tools/fetch_region_buildings.sh urban_suburban_03 33.171427 35.521378 0.050050 0.059820
#
# Pipeline (matches the documented mapping prep):
#   1. Overture building footprints for the bbox        -> footprints.geojson
#   2. Copernicus GLO-30 DSM for the covering 1-deg tiles (public S3, no-sign)
#   3. FABDEM bare-earth from resources/terrain/*_fabdem.dt2 (already local)
#   4. nDSM = DSM - FABDEM on a 1-arcsec, -tap-aligned grid
#   5. per-footprint zonal-MAX height (build_buildings_heights.py), floor 3 / cap 60
# Output: resources/buildings/<NAME>.geojson
#
# Terrain (DTM) is NOT downloaded here - the FABDEM .dt2 tiles already cover the
# AOI and are the sim's terrain.
set -euo pipefail

NAME=$1; CLAT=$2; CLON=$3; SLAT=$4; SLON=$5
REPO="$(cd "$(dirname "$0")/.." && pwd)"
RES=0.0002777777778            # 1 arcsec (~30 m), GLO-30 native
REL=2026-05-20.0               # Overture release
W="$(mktemp -d)"
export AWS_NO_SIGN_REQUEST=YES

XMIN=$(python3 -c "print($CLON-$SLON/2)"); XMAX=$(python3 -c "print($CLON+$SLON/2)")
YMIN=$(python3 -c "print($CLAT-$SLAT/2)"); YMAX=$(python3 -c "print($CLAT+$SLAT/2)")
echo "[$NAME] bbox lon[$XMIN,$XMAX] lat[$YMIN,$YMAX]  work=$W"

echo "[1/5] Overture footprints"
AWS_REGION=us-west-2 ogr2ogr -f GeoJSON "$W/footprints.geojson" \
  "/vsis3/overturemaps-us-west-2/release/$REL/theme=buildings/type=building/" \
  -spat "$XMIN" "$YMIN" "$XMAX" "$YMAX" \
  -select height,num_floors,subtype,class -nln buildings

echo "[2/5] GLO-30 DSM (covering 1-deg tiles -> VRT)"
dsm_tiles=()
for la in $(seq "$(python3 -c "import math;print(math.floor($YMIN))")" "$(python3 -c "import math;print(math.floor($YMAX))")"); do
  for lo in $(seq "$(python3 -c "import math;print(math.floor($XMIN))")" "$(python3 -c "import math;print(math.floor($XMAX))")"); do
    t=$(printf "Copernicus_DSM_COG_10_N%02d_00_E%03d_00_DEM" "$la" "$lo")
    dsm_tiles+=("/vsis3/copernicus-dem-30m/$t/$t.tif")
  done
done
gdalbuildvrt -q "$W/dsm.vrt" "${dsm_tiles[@]}"
gdalwarp -overwrite -q -te "$XMIN" "$YMIN" "$XMAX" "$YMAX" -tr "$RES" "$RES" -tap -r bilinear "$W/dsm.vrt" "$W/dsm.tif"

echo "[3/5] FABDEM bare-earth (local tiles -> VRT)"
gdalbuildvrt -q "$W/fab.vrt" "$REPO"/resources/terrain/*_fabdem.dt2
gdalwarp -overwrite -q -te "$XMIN" "$YMIN" "$XMAX" "$YMAX" -tr "$RES" "$RES" -tap -r bilinear "$W/fab.vrt" "$W/fab.tif"

echo "[4/5] nDSM = DSM - FABDEM"
gdal_calc.py --quiet -A "$W/dsm.tif" -B "$W/fab.tif" --calc="A-B" \
  --outfile="$W/ndsm.tif" --type=Float32 --NoDataValue=-9999 --overwrite

echo "[5/5] per-footprint heights -> resources/buildings/$NAME.geojson"
python3 "$REPO/tools/build_buildings_heights.py" \
  "$W/footprints.geojson" "$W/ndsm.tif" "$REPO/resources/buildings/$NAME.geojson" \
  "$XMIN" "$YMIN" "$XMAX" "$YMAX" "$RES"

echo "[$NAME] done."
