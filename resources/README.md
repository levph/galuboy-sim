# Resources

Runtime data the simulation reads at startup.

```
resources/
  antennas/   antenna pattern CSVs (committed; small reference patterns)
  osm/        OpenStreetMap region files (NOT committed; large)
  terrain/    DTED2 / SRTM tiles (NOT committed; large)
```

## `osm/`

Drop one OSM file per region you want to simulate. Bounds are read from the
file's `<bounds>` tag by `utils/osmReadBounds.m`. Default region in
`config/buildConfig.m` is `yarka.osm`.

## `terrain/`

Drop the DT2 (or DT1 / GeoTIFF) tiles that cover your regions. The standard
SRTM 1-arc-second naming `n{LAT}_e{LON}_1arc_v3.dt2` is auto-discovered by
`terrain/discoverTerrainTiles.m`. Yarka (~32.85N, 35.21E) needs at minimum
`n32_e035_1arc_v3.dt2`; bringing surrounding tiles improves edge propagation.

GeoTIFF files that do **not** follow the SRTM naming convention are also
supported — `discoverTerrainTiles` reads their bounding box via `geotiffinfo`
(requires Mapping Toolbox). They must be in a geographic CRS (e.g. WGS84 /
EPSG:4326). The seven region-specific TIFs below use this path.

The first run registers all discovered tiles via `addCustomTerrain` and tiles
them under `<repo>/terrain_cache/galuboy_terrain/`. Subsequent runs reuse the
cache.

## Region Maps

Seven high-resolution terrain cutouts (~10 m/px, WGS84/EPSG:4326, float32)
exported from a DSM raster. **Terrain elevation only — no building data.**
The companion `.npy` files from the export tool are raw NumPy arrays with no
georeference and are not included or used by the simulation.

Each region is defined in `config/buildConfig.m` with `latlim_override` /
`lonlim_override` (no OSM file needed).

### Urban / Suburban  (~5 × 5 km)

| File | Center | Radius | Elev range | Mean | Roughness RMS | Coverage |
|------|--------|--------|-----------|------|---------------|----------|
| `urban_suburban_01.tif` | 33.020 N, 35.124 E | 2620 m | 3–116 m | 31 m | 2.1 m | 78 % |
| `urban_suburban_02.tif` | 33.269 N, 35.226 E | 2542 m | 0–122 m | 30 m | 2.8 m | 78 % |
| `urban_suburban_03.tif` | 33.201 N, 35.613 E | 2786 m | 71–158 m | 85 m | 1.9 m | 78 % |

### Forest  (~10 × 10 km)

| File | Center | Radius | Elev range | Mean | Roughness RMS | Coverage |
|------|--------|--------|-----------|------|---------------|----------|
| `forest_01.tif` | 32.998 N, 35.401 E | 5197 m | 590–1215 m | 831 m | 2.8 m | 78 % |

### Mountain  (~10 × 10 km)

| File | Center | Radius | Elev range | Mean | Roughness RMS | Coverage |
|------|--------|--------|-----------|------|---------------|----------|
| `mountain_01.tif` | 33.407 N, 35.627 E | 5944 m | 308–1239 m | 716 m | 3.2 m | 78 % |
| `mountain_02.tif` | 33.497 N, 35.854 E | 5790 m | 840–1925 m | 1247 m | 2.4 m | 78 % |

### Rural / Open  (~10 × 10 km)

| File | Center | Radius | Elev range | Mean | Roughness RMS | Coverage |
|------|--------|--------|-----------|------|---------------|----------|
| `rural_open_01.tif` | 33.504 N, 35.423 E | 4700 m | 23–489 m | 266 m | 3.4 m | 78 % |

A machine-readable summary of all exports is in `resources/summary.csv`.

## `antennas/`

CSV pattern files; one per antenna model. Two formats:

**Isotropic sentinel** (single non-comment line):

```
isotropic,7.0
```

**Regular grid** (header + rows):

```
az_deg,el_deg,gain_dbi
0,-90,2.5
0,-60,3.5
...
```

- Comments start with `#`.
- Az/el are **offsets from the antenna boresight in degrees**.
- The grid must be rectangular (every az repeats the same el list).
- Out-of-range queries clamp to the nearest grid point — no NaN.

Boresight orientation is set per-device in `config/buildConfig.m`
(`config.tx.boresight_*`, `config.rx.infantry.boresight_*`, etc.).
Default TX boresight is nadir (`el = -π/2`); default RX boresight is
zenith (`el = +π/2`).

Three example patterns are committed:

| File                       | Description                              |
|----------------------------|------------------------------------------|
| `tx_omni.csv`              | 7 dBi isotropic (placeholder)            |
| `rx_infantry_omni.csv`     | Handheld vertical dipole (peak 6.5 dBi)  |
| `rx_vehicular_dipole.csv`  | Roof whip with horizon peak / zenith null |
