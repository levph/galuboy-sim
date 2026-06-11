#!/usr/bin/env python3
"""Derive per-building heights for a region from an nDSM raster.

Zonal MAX of nDSM over each footprint (rasterized all-touched ~ "+1-cell buffer"),
with a num_floors*3 fallback prior, floored at 3 m and capped at 60 m. Writes a
GeoJSON with `height` (m) + `height_source` ("ndsm" | "num_floors" | "floored"),
matching the format of the other region building files.

Uses only the GDAL CLI (gdal_rasterize / gdal_translate) + numpy + json -- this
Python has no osgeo/shapely/rasterio bindings.

Usage:
  build_buildings_heights.py FOOTPRINTS.geojson NDSM.tif OUT.geojson \
      XMIN YMIN XMAX YMAX RES
(the bbox/RES must match the grid NDSM was built on; -tap aligned)
"""
import json, os, subprocess, sys, tempfile, shutil
from collections import Counter
import numpy as np

fp_in, ndsm, out = sys.argv[1], sys.argv[2], sys.argv[3]
xmin, ymin, xmax, ymax, res = map(float, sys.argv[4:9])
os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
# Intermediates go in a scratch dir, NOT next to the output (which is
# resources/buildings/) - otherwise we litter that dir with _id.tif/_fp_id etc.
W = tempfile.mkdtemp(prefix="bldgheights_")

# 1. tag each footprint with an integer id (1..N)
d = json.load(open(fp_in))
feats = d["features"]
for i, f in enumerate(feats):
    f["properties"]["bid"] = i + 1
fid = os.path.join(W, "_fp_id.geojson")
json.dump(d, open(fid, "w"))

# 2. rasterize ids onto the nDSM grid (all-touched), then ASCII for numpy
idtif = os.path.join(W, "_id.tif")
subprocess.run(["gdal_rasterize", "-q", "-a", "bid", "-init", "0", "-a_nodata", "0",
                "-te", repr(xmin), repr(ymin), repr(xmax), repr(ymax),
                "-tr", repr(res), repr(res), "-tap", "-at", "-ot", "Int32",
                fid, idtif], check=True)

def to_arr(src):
    asc = os.path.splitext(src)[0] + ".asc"
    for ext in (".asc", ".asc.aux.xml", ".prj"):
        p = os.path.splitext(src)[0] + ext
        if os.path.exists(p):
            os.remove(p)
    subprocess.run(["gdal_translate", "-q", "-of", "AAIGrid", src, asc], check=True)
    return np.loadtxt(asc, skiprows=6).ravel()

N = to_arr(ndsm)
I = to_arr(idtif).astype(int)

# 3. zonal max nDSM per footprint id (id 0 = nodata, lands in mx[0] and is ignored)
mx = np.full(len(feats) + 1, -1e9)
np.maximum.at(mx, I, N)

# 4. height per footprint: max(nDSM, num_floors*3); floored 3, capped 60
for i, f in enumerate(feats):
    v = mx[i + 1]
    nf = f["properties"].get("num_floors")
    cands = []
    if np.isfinite(v) and v > -1e8:
        cands.append(("ndsm", float(v)))
    if nf:
        cands.append(("num_floors", float(nf) * 3.0))
    if cands:
        src, h = max(cands, key=lambda t: t[1])
    else:
        src, h = "floored", 3.0
    if h < 3.0:
        src, h = "floored", 3.0
    h = min(h, 60.0)
    f["properties"]["height"] = round(h, 2)
    f["properties"]["height_source"] = src
    del f["properties"]["bid"]

json.dump(d, open(out, "w"))
shutil.rmtree(W, ignore_errors=True)   # drop scratch intermediates
hs = [f["properties"]["height"] for f in feats]
print(f"features={len(feats)}  height min={min(hs):.1f} max={max(hs):.1f}  "
      f"sources={dict(Counter(f['properties']['height_source'] for f in feats))}")
