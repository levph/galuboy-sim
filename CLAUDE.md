# galuboy-sim — Claude Code project guide

Air-to-ground RF propagation Monte Carlo. An aerial transmitter flies a
rotated figure-8 (lemniscate of Bernoulli) above a defined map region;
ground receivers (infantry + vehicular) are dropped uniformly per
rotation; per-RX percentile-based link availability is computed against
real terrain via Longley-Rice. Target MATLAB R2021b+.

## Repo layout

```
run_galuboy.m         Headless entry point (thin driver)
config/               buildConfig.m, resolveRegion.m
sim/                  generateTrajectory, placeReceivers, runRotation, runScenario
terrain/              setupTerrain (DT2 registration), discoverTerrainTiles
antennas/             loadAntennaPattern (CSV -> griddedInterpolant), applyAntennaGain
visualization/        plotRangeHistogram, plotAvailabilityMap, plotProgress
ui/                   galuboySimApp (uihtml), runFromUI, progress.html
utils/                osmReadBounds, haversineDistance, distanceFromCentroid
resources/            antennas/ (committed CSVs), osm/ (user), terrain/ (user)
tests/                runtests('tests')
legacy/               pre-v4 modules - reference only
```

## Resources you must supply

`resources/osm/<region>.osm` — OpenStreetMap export for each region. Bounds
are read from the file's `<bounds>` tag. The default config expects
`yarka.osm`.

`resources/terrain/*.dt2` — SRTM 1-arc tiles in standard
`n{LL}_e{LLL}_1arc_v3.dt2` naming. `discoverTerrainTiles` auto-finds the
ones covering the union of all configured regions' bboxes (with a 0.05°
buffer).

`resources/antennas/*.csv` — three example patterns are committed
(`tx_omni.csv`, `rx_infantry_omni.csv`, `rx_vehicular_dipole.csv`). Format
documented in `resources/README.md`. Either an `isotropic,<gain_dbi>`
sentinel or a regular `az_deg,el_deg,gain_dbi` grid.

## How to run

```matlab
% Headless:
run_galuboy

% UI (R2021b+):
galuboySimApp

% Custom config:
cfg = buildConfig();
cfg.flight.num_rotations    = 4;
cfg.propagation.use_terrain = false;   % skip DT2, use default GMTED
results = runScenario(cfg, @(varargin) fprintf('%s\n', plotProgress(varargin{:})));
```

## Skills to invoke during implementation

| Skill                            | When to use                                          |
|----------------------------------|------------------------------------------------------|
| `matlab-performance-optimizer`   | parfor pass in `runRotation`, vectorization audits   |
| `matlab-test-creator`            | New tests under `tests/`                              |
| `matlab-test-execution`          | Run `runtests('tests')`                               |
| `matlab-uihtml-app-builder`      | Anything under `ui/`                                  |

Skip `matlab-live-script` (no live-script outputs), `matlab-digital-filter-design`
(no DSP filters in scope).

## Hard MATLAB constraints (do not regress)

- **Terrain registration** must use `setupTerrain` (which sets an explicit
  persistent `WriteLocation` under `<repo>/terrain_cache/`). The default
  `addCustomTerrain` `WriteLocation` is `tempname` under `tempdir`, which
  macOS periodically cleans — registration survives in user prefs but the
  tile data on disk does not, causing `"Unable to access terrain"` errors
  on every subsequent query.
- **`parfor` over flight steps** in `runRotation`. Build `tx_steps =
  repmat(tx_template, 1, Nf)` *outside* the loop. Mutating `tx.Longitude`
  *inside* `parfor` errors with "object cannot be sent to workers".
- **`pathloss(...,'Map',name)`** only when `use_terrain == true` and
  `name` is the registered terrain name. Passing `'Map',''` is invalid;
  `runRotation` switches between the two overloads with an `if`.
- **Default RX heights:** 1.5 m infantry / 2.5 m vehicular. Set in
  `config.rx.{infantry,vehicular}.height_m`. R2021b's `rxsite` accepts
  vector `'AntennaHeight'` for a single array — `placeReceivers` relies on
  this.
- **Antenna pattern frame:** CSV columns are *offsets from boresight in
  degrees*. `loadAntennaPattern` builds a `griddedInterpolant` keyed on
  *radians*. `applyAntennaGain` does the boresight subtraction +
  `wrapToPi` + el clamp. TX gain uses az/el from `angle(rx,tx)`; RX gain
  flips to `mod(az+pi,2*pi)` and `-el` (`runRotation` does this).
- **Single precision** is preserved through the hot path (`PL`, `AZ`, `EL`,
  `G_*`, `Prx`); `prctile` accepts `single`.
- **Save with `-v7.3`.** `results/scenario_<timestamp>.mat` plus
  `results/latest.mat` (overwritten).
- **TX boresight default = nadir** (`-pi/2`) — fits a target-circling
  figure-8 above a small area. Override per scenario if needed.

## Reference patterns mirrored from freq-planner

`/Users/lev/freq-planner/matlab/code/terrain/setupTerrain.m` is the
canonical implementation of the terrain-registration cache strategy.
`computeDirectionalGain.m` and `applyDirectionalPattern.m` over there are
the az-only analogues of our az+el `applyAntennaGain`. The parfor shape
in `computePathLossMatrix.m` (build `txsite` array outside, slice inside)
is what `runRotation` follows.

## Adding a new region

1. Drop `resources/osm/<name>.osm`.
2. Append a struct to `config.regions` in `buildConfig.m`:
   ```matlab
   config.regions(end+1) = struct('name','<name>', ...
       'osm_path','resources/osm/<name>.osm', ...
       'latlim_override',[],'lonlim_override',[]);
   ```
3. Make sure `resources/terrain/` has DT2 tiles covering the new bbox.
4. Run.

The single `addCustomTerrain` registration uses the *union* of all
regions' bboxes, so all enabled regions share one cache.
