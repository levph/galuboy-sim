# galuboy-sim

Air-to-ground RF propagation Monte Carlo. An aerial transmitter flies a
rotated figure-8 (lemniscate of Bernoulli) above a defined map region;
ground receivers (infantry + vehicular) are dropped uniformly per
rotation; per-RX percentile-based link availability is computed against
real terrain via Longley-Rice.

> *"Galuboy"* — internal project name. The codebase used to be a
> single 350-line script; v4.0 splits it into modular components,
> registers DT2 terrain, supports multiple regions and two RX device
> types per rotation, parallelizes the propagation sweep, and adds a
> uihtml UI.

---

## Requirements

- **MATLAB R2021b+** (R2025a verified)
- Toolboxes:
  - **Antenna Toolbox** — `propagationModel`, `pathloss`, `txsite`, `rxsite`
  - **Mapping Toolbox** — `addCustomTerrain`, `siteviewer`
  - **Parallel Computing Toolbox** — `parfor`, `parpool`
  - **Phased Array System Toolbox**
  - **Statistics and Machine Learning Toolbox** — `prctile`
- ~250 MB free under `<repo>/terrain_cache/` per registered terrain
  (the Tiler builds a tiled pyramid).

---

## First-time setup

### 1. Drop your map

```
resources/osm/<region>.osm
```

The default config expects `resources/osm/yarka.osm`. Region bounds are
read from the file's `<bounds>` tag (`utils/osmReadBounds.m`).

### 2. Drop SRTM terrain tiles

```
resources/terrain/n{LL}_e{LLL}_1arc_v3.dt2     # standard SRTM 1-arc names
```

`terrain/discoverTerrainTiles.m` auto-finds the tiles whose 1° squares
intersect the configured regions' union bbox (with a 0.05° buffer).
Tiles that don't fit a strict rectangle are filled with elevation 0
via `'FillMissing', true`.

For the default Yarka region, drop at least `n32_e035_1arc_v3.dt2`.
For larger regions or runs that include the bbox edges, also drop
neighboring tiles (`n32_e034`, `n33_e035`, etc.).

### 3. (Optional) Replace the example antennas

Three reference patterns live in `resources/antennas/` and are committed:

| File                       | Description                                |
|----------------------------|--------------------------------------------|
| `tx_omni.csv`              | 7 dBi isotropic placeholder                |
| `rx_infantry_omni.csv`     | Vertical dipole, peak 6.5 dBi at horizon   |
| `rx_vehicular_dipole.csv`  | Horizon peak / zenith null (deep at ±π/2)  |

Replace with measured patterns when available. Format options:

**Isotropic sentinel** (one non-comment line):

```csv
isotropic,7.0
```

**Regular grid** (rectangular az×el table):

```csv
az_deg,el_deg,gain_dbi
0,-90,2.5
0,-60,3.5
...
```

- `#` lines are comments.
- Az/el are *offsets from antenna boresight* in degrees.
- Out-of-range queries clamp to the nearest grid point (no NaN).

Boresight orientation is set per-device in `config/buildConfig.m`
(`config.tx.boresight_*`, `config.rx.{infantry,vehicular}.boresight_*`).
Defaults: TX boresight = nadir (`el = -π/2`); RX boresight = zenith
(`el = +π/2`).

---

## Running

### Headless

From the repo root in MATLAB:

```matlab
run_galuboy
```

This:
1. Loads default config (`buildConfig`).
2. Resolves regions + registers DT2 terrain (cache hit on subsequent runs).
3. Launches a parallel pool.
4. For each region, sweeps rotations 0° → 180° in `step_degrees` increments.
5. Per rotation: drops fresh receivers, computes per-step `pathloss` (parfor
   over flight steps), applies TX/RX antenna gains, reduces to per-RX
   percentile P_rx, marks availability vs MDS threshold.
6. Saves `results/scenario_<timestamp>.mat` plus `results/latest.mat`
   (`-v7.3`).
7. Plots range histogram + availability map.

### UI (R2021b+)

```matlab
galuboySimApp
```

A `uifigure` window opens with:

- **Left form** — region selector, `n_inf` / `n_veh` counts, frequency, power,
  altitude, percentile, fade margin type, three antenna selectors, "use DT2
  terrain" checkbox.
- **Center** — uihtml progress panel: live region/rotation bars + log.
- **Right tabs** — Range histogram and Availability map, rendered after
  completion.
- **Bottom row** — Run, Save config (JSON), Load config (JSON).

The simulation runs synchronously on the UI thread with `drawnow` inside
the progress callback to keep the app responsive — no `backgroundPool`
required (so it works on every R2021b release).

### Custom config from the prompt

```matlab
cfg = buildConfig();

% Override anything you like:
cfg.flight.num_rotations    = 4;
cfg.flight.num_flight_steps = 30;
cfg.rx.infantry.count       = 20;
cfg.rx.vehicular.count      = 10;
cfg.propagation.use_terrain = false;     % skip DT2, use default GMTED2010
cfg.analysis.percentile     = 95;        % availability percentile
cfg.tx.power_dbm            = 40;

results = runScenario(cfg, @(varargin) fprintf('%s\n', plotProgress(varargin{:})));

plotRangeHistogram(results, cfg);
plotAvailabilityMap(results, cfg);
```

### Adding a new region

1. Drop `resources/osm/<name>.osm`.
2. Drop DT2 tiles covering it into `resources/terrain/`.
3. In `buildConfig.m`, append to `config.regions`:

   ```matlab
   config.regions(end+1) = struct( ...
       'name',            '<name>', ...
       'osm_path',        'resources/osm/<name>.osm', ...
       'latlim_override', [], ...
       'lonlim_override', []);
   ```

4. Run. The single `addCustomTerrain` registration covers the *union*
   of all enabled regions' bboxes — one shared cache.

---

## Config reference

`config/buildConfig.m` returns one grouped struct:

| Group               | Key fields                                                            |
|---------------------|-----------------------------------------------------------------------|
| `tx`                | `frequency_hz`, `power_dbm`, `altitude_m`, `antenna_name`, `boresight_az_rad`, `boresight_el_rad` |
| `rx.infantry`       | `count`, `height_m` (default 1.5), `antenna_name`, `boresight_*`      |
| `rx.vehicular`      | `count`, `height_m` (default 2.5), `antenna_name`, `boresight_*`      |
| `propagation`       | `model` (`'longley-rice'`/`'freespace'`/...), `time_variability`, `situation_variability`, `bandwidth_hz`, `terrain_name`, `use_terrain` |
| `flight`            | `num_flight_steps`, `step_degrees`, `num_rotations`, `lemniscate_scale_deg` |
| `analysis`          | `threshold_dbm` (-95), `percentile` (99), `fade_margin_type`, `fade_margin_db` |
| `regions`           | struct array `{name, osm_path, latlim_override, lonlim_override}`     |
| `parallel`          | `enabled`, `num_workers` (`[]` = MATLAB default)                      |
| `io`                | `results_dir` (`results`), `terrain_cache_dir` (`terrain_cache`)      |
| `viz`               | `show_siteviewer` (false default)                                     |
| `hist`              | `n_bins` (50), `min_samples_per_bin` (1)                              |

Fade-margin presets: `minimal` (3 dB), `moderate` (10), `conservative` (15),
`urban` (20), `custom` (uses `fade_margin_custom_db`).

---

## Results structure

`runScenario` returns a struct array (one entry per region):

| Field             | Type / shape                  | Meaning                                       |
|-------------------|-------------------------------|-----------------------------------------------|
| `region`          | struct                        | `name, osm_path, latlim, lonlim, center_*`    |
| `available`       | `[M × n_rx]` logical          | true = link available for this (rotation, RX) |
| `prx_pctile`      | `[M × n_rx]` single           | percentile P_rx (dBm)                         |
| `dists_m`         | `[M × n_rx]` single           | distance from region centroid                 |
| `rx_locations`    | `[M × n_rx × 2]` single       | (lon, lat) per (rotation, RX)                 |
| `rx_types`        | cell `{M × 1}`                | categorical column per rotation               |
| `rx_heights`      | `[M × n_rx]` single           | m AGL                                         |
| `elapsed_s`       | scalar                        | wall-clock for that region                    |

The full per-step `[Nf × n_rx]` path-loss / az / el cube is **not**
persisted — reduction to per-RX percentile happens inside
`runRotation` so memory stays flat regardless of `num_flight_steps`.

---

## Tests

```matlab
% From the repo root:
runtests('tests')
```

Six test classes under `tests/`:

| File                                  | Verifies                                        |
|---------------------------------------|-------------------------------------------------|
| `test_buildConfig_schema.m`           | required fields, deprecated fields gone         |
| `test_loadAntennaPattern.m`           | isotropic sentinel, boresight peak, OOR clamp   |
| `test_applyAntennaGain.m`             | boresight subtraction, vectorization, single-precision |
| `test_setupTerrain.m`                 | persistent cache under repo, manifest fingerprint, registration survives |
| `test_runRotation_freespace.m`        | Friis sanity check, no NaN, link-budget sign    |
| `test_runScenario_smoke.m`            | end-to-end (1 region × 2 rotations × 5+5 RX)    |

Tests requiring toolboxes/tiles auto-skip via `assumeTrue` when prerequisites
are missing.

---

## Repo layout

```
run_galuboy.m         Headless entry point (thin driver)
config/               buildConfig, resolveRegion
sim/                  generateTrajectory, placeReceivers, runRotation, runScenario
terrain/              setupTerrain (DT2 registration), discoverTerrainTiles
antennas/             loadAntennaPattern (CSV → griddedInterpolant), applyAntennaGain
visualization/        plotRangeHistogram, plotAvailabilityMap, plotProgress
ui/                   galuboySimApp (uihtml), runFromUI, progress.html
utils/                osmReadBounds, haversineDistance, distanceFromCentroid
resources/            antennas/ (committed), osm/ (you supply), terrain/ (you supply)
tests/                runtests('tests')
legacy/               pre-v4 modules — reference only
results/              .mat outputs (gitignored)
terrain_cache/        on-disk Tiler cache (gitignored)
```

---

## Troubleshooting

**"Region 'yarka' expects an OSM file at: …/resources/osm/yarka.osm"**
Drop the OSM file at that exact path, or override `config.regions(1).osm_path`.

**"No tiles in resources/terrain/ cover lat=[…], lon=[…]"**
Drop SRTM 1-arc tiles covering the bbox. Filename must match
`n{LAT}_e{LON}_*.{dt1,dt2,tif}` (or `s/w` for southern/western
hemispheres).

**"Unable to access terrain '…'" on a previously-working run**
This is the macOS-tempdir-cleanup symptom. `setupTerrain` writes the
cache to `<repo>/terrain_cache/` instead of `tempname` so registration
survives. If the error persists, delete `terrain_cache/<name>/` and
re-run — the manifest fingerprint will trigger a fresh re-tile.

**First run takes ~50 s per terrain tile**
That's the Tiler building the cache pyramid. Subsequent runs hit the
manifest fast path and reuse it (~10 ms).

**`pathloss` errors with "object cannot be sent to workers"**
You're mutating a `txsite` *inside* `parfor`. Build the array outside
the loop and slice into it (see `runRotation.m` for the pattern).

**`propagationModel('freespace')` rejects `TimeVariabilityTolerance`**
Those parameters are Longley-Rice-only. `runScenario` already branches
on model name; if you add a new model, mirror that switch.

---

## Authors / version

Tomer Antebi & Lev Panov — v4.0 (May 2026).
