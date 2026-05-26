function results = runScenario(cfg, progress_cb)
%RUNSCENARIO  Top-level engine: register terrain, sweep rotations across regions.
%
%   results = runScenario(cfg)
%   results = runScenario(cfg, progress_cb)
%
%   cfg          struct from buildConfig (possibly user-modified)
%   progress_cb  optional function handle:
%                  progress_cb(stage, region_idx, n_regions, rot_idx, n_rots, msg)
%                stage is one of: "setup" | "region" | "rotation" | "done"
%
%   Returns results - struct array, one entry per region:
%     results(r).region              (struct from resolveRegion)
%     results(r).available_dl        [M x n_rx] logical  (DL  link availability)
%     results(r).available_ul        [M x n_rx] logical  (UL  link availability)
%     results(r).available_bidi      [M x n_rx] logical  (DL AND UL)
%     results(r).available           [M x n_rx] logical  (alias for available_dl)
%     results(r).frac_above_dl       [M x n_rx] single   (DL fraction of rotation > MDS)
%     results(r).frac_above_ul       [M x n_rx] single   (UL fraction of rotation > MDS)
%     results(r).frac_above          [M x n_rx] single   (alias for frac_above_dl)
%     results(r).prx_floor_dbm_dl    [M x n_rx] single   (DL lower-tail P_rx percentile)
%     results(r).prx_floor_dbm_ul    [M x n_rx] single   (UL lower-tail P_rx percentile)
%     results(r).prx_floor_dbm       [M x n_rx] single   (alias for prx_floor_dbm_dl)
%     results(r).dists_m             [M x n_rx] single   (distance from region centroid)
%     results(r).rx_locations        [M x n_rx x 2] single  (lon, lat per rx per rotation)
%     results(r).rx_types            cell {M x 1}   (categorical column per rotation)
%     results(r).rx_heights          [M x n_rx]    (m AGL)
%     results(r).elapsed_s           scalar
%
%   The full [Nf x n_rx] path-loss / az / el cube is NOT persisted -
%   reduction to per-RX percentile happens inside runRotation.
%
%   Saves results/scenario_<timestamp>.mat (and overwrites results/latest.mat)
%   with cfg + results.

    arguments
        cfg         (1,1) struct
        progress_cb = []
    end
    has_cb = ~isempty(progress_cb);

    repo_root = fileparts(fileparts(mfilename('fullpath')));   % <repo>/sim -> <repo>

    % --- Resolve regions --------------------------------------------------
    region_cfgs = cfg.regions;
    n_regions   = numel(region_cfgs);
    if n_regions == 0
        error('runScenario:noRegions', 'config.regions is empty.');
    end
    regions(1, n_regions) = struct( ...
        'name', '', 'osm_path', '', 'latlim', [], 'lonlim', [], ...
        'center_lat', 0, 'center_lon', 0, 'buildings', {{}});
    for r = 1:n_regions
        rg = resolveRegion(region_cfgs(r), repo_root);
        % OSM-less regions: osmReadBuildings returns {} for missing/empty paths
        rg.buildings = osmReadBuildings(rg.osm_path);
        regions(r) = rg;
    end

    % --- Terrain registration (single union-bbox terrain) ----------------
    map_name = '';
    if cfg.propagation.use_terrain
        if has_cb, progress_cb("setup", 0, n_regions, 0, 0, "discovering terrain tiles"); end
        all_lats = arrayfun(@(rg) rg.latlim, regions, 'UniformOutput', false);
        all_lons = arrayfun(@(rg) rg.lonlim, regions, 'UniformOutput', false);
        union_latlim = [min(cellfun(@(v) v(1), all_lats)), max(cellfun(@(v) v(2), all_lats))];
        union_lonlim = [min(cellfun(@(v) v(1), all_lons)), max(cellfun(@(v) v(2), all_lons))];

        terrain_dir = fullfile(repo_root, 'resources', 'terrain');
        tile_files  = discoverTerrainTiles(terrain_dir, union_latlim, union_lonlim);
        if has_cb, progress_cb("setup", 0, n_regions, 0, 0, ...
                sprintf("registering %d terrain tile(s)", numel(tile_files))); end
        setupTerrain(cfg.propagation.terrain_name, tile_files);
        map_name = cfg.propagation.terrain_name;
    end

    % --- Antenna patterns (loaded once) -----------------------------------
    antennas_dir = fullfile(repo_root, 'resources', 'antennas');
    ant.tx_air_pat  = loadAntennaPattern(cfg.tx.airborne.antenna_name,         antennas_dir);
    ant.tx_gnd_pat  = loadAntennaPattern(cfg.tx.ground.antenna_name,           antennas_dir);
    ant.rx_air_pat  = loadAntennaPattern(cfg.rx.airborne.antenna_name_ul,      antennas_dir);
    ant.rx_gnd1_pat = loadAntennaPattern(cfg.rx.infantry.antenna_name_dl,      antennas_dir);
    ant.rx_gnd2_pat = loadAntennaPattern(cfg.rx.vehicular.antenna_name_dl,     antennas_dir);

    % --- Propagation model + TX template ---------------------------------
    % TimeVariabilityTolerance / SituationVariabilityTolerance are only
    % accepted by Longley-Rice; FreeSpace and other models reject them.
    if strcmpi(cfg.propagation.model, 'longley-rice')
        pm = propagationModel(cfg.propagation.model, ...
            'TimeVariabilityTolerance',      cfg.propagation.time_variability, ...
            'SituationVariabilityTolerance', cfg.propagation.situation_variability);
    else
        pm = propagationModel(cfg.propagation.model);
    end

    % TX template uses the airborne TX power as a reference. The actual DL/UL
    % link budgets reapply per-direction powers inside runRotation; MATLAB's
    % pathloss() helper uses the TX power only as a reference for its rxsignal
    % output, which we don't consume.
    tx_template = txsite('Name',                 'Aerial_Tx', ...
                         'Latitude',             0, ...
                         'Longitude',            0, ...
                         'AntennaHeight',        cfg.tx.altitude_m, ...
                         'TransmitterFrequency', cfg.tx.frequency_hz, ...
                         'TransmitterPower',     10^((cfg.tx.power_air_dbm - 30) / 10));

    % --- Parallel pool ----------------------------------------------------
    if cfg.parallel.enabled && isempty(gcp('nocreate'))
        if has_cb, progress_cb("setup", 0, n_regions, 0, 0, "starting parallel pool"); end
        if isempty(cfg.parallel.num_workers)
            parpool('local');
        else
            parpool('local', cfg.parallel.num_workers);
        end
    end

    % --- Per-region sweep -------------------------------------------------
    M = cfg.flight.num_rotations;
    n_inf = cfg.rx.infantry.count;
    n_veh = cfg.rx.vehicular.count;
    n_rx  = n_inf + n_veh;

    results(1, n_regions) = struct( ...
        'region', struct(), ...
        'available_dl', logical([]), 'available_ul', logical([]), ...
        'available_bidi', logical([]), 'available', logical([]), ...
        'frac_above_dl', [], 'frac_above_ul', [], 'frac_above', [], ...
        'prx_floor_dbm_dl', [], 'prx_floor_dbm_ul', [], 'prx_floor_dbm', [], ...
        'dists_m', [], 'rx_locations', [], 'rx_types', {{}}, ...
        'rx_heights', [], 'elapsed_s', 0);

    for r = 1:n_regions
        region = regions(r);
        if has_cb
            progress_cb("region", r, n_regions, 0, M, ...
                sprintf("region %d/%d (%s)", r, n_regions, region.name));
        end

        avail_dl_mat   = false(M, n_rx);
        avail_ul_mat   = false(M, n_rx);
        avail_bidi_mat = false(M, n_rx);
        frac_dl_mat    = zeros(M, n_rx, 'single');
        frac_ul_mat    = zeros(M, n_rx, 'single');
        floor_dl_mat   = zeros(M, n_rx, 'single');
        floor_ul_mat   = zeros(M, n_rx, 'single');
        dists_mat      = zeros(M, n_rx, 'single');
        loc_mat        = zeros(M, n_rx, 2, 'single');
        types_cell     = cell(M, 1);
        height_mat     = zeros(M, n_rx, 'single');

        t_start = tic;
        for m = 1:M
            rotation_deg = (m - 1) * cfg.flight.step_degrees;
            [flight_lons, flight_lats, flight_tilts, flight_headings] = ...
                generateTrajectory(region, cfg.flight, rotation_deg);
            [rx_array,    rx_table]    = placeReceivers(region, cfg.rx, region.buildings);

            rot = runRotation(tx_template, rx_array, rx_table, pm, map_name, ...
                              flight_lons, flight_lats, ant, cfg, ...
                              flight_tilts, flight_headings);

            avail_dl_mat(m, :)   = rot.available_dl(:).';
            avail_ul_mat(m, :)   = rot.available_ul(:).';
            avail_bidi_mat(m, :) = rot.available_bidi(:).';
            frac_dl_mat(m, :)    = single(rot.frac_above_dl(:).');
            frac_ul_mat(m, :)    = single(rot.frac_above_ul(:).');
            floor_dl_mat(m, :)   = single(rot.prx_floor_dbm_dl(:).');
            floor_ul_mat(m, :)   = single(rot.prx_floor_dbm_ul(:).');
            loc_mat(m, :, 1)     = single(rx_table.lon(:).');
            loc_mat(m, :, 2)     = single(rx_table.lat(:).');
            dists_mat(m, :)      = single(distanceFromCentroid( ...
                region.lonlim, region.latlim, [rx_table.lon, rx_table.lat]));
            types_cell{m}        = rx_table.type;
            height_mat(m, :)     = single(rx_table.height_m(:).');

            if has_cb
                progress_cb("rotation", r, n_regions, m, M, ...
                    sprintf("region %d/%d  rotation %d/%d  (DL=%.1f%%  UL=%.1f%%  both=%.1f%%)", ...
                        r, n_regions, m, M, ...
                        100 * mean(rot.available_dl), ...
                        100 * mean(rot.available_ul), ...
                        100 * mean(rot.available_bidi)));
            end
        end

        results(r).region            = region;
        results(r).available_dl      = avail_dl_mat;
        results(r).available_ul      = avail_ul_mat;
        results(r).available_bidi    = avail_bidi_mat;
        results(r).available         = avail_dl_mat;       % back-compat alias
        results(r).frac_above_dl     = frac_dl_mat;
        results(r).frac_above_ul     = frac_ul_mat;
        results(r).frac_above        = frac_dl_mat;        % back-compat alias
        results(r).prx_floor_dbm_dl  = floor_dl_mat;
        results(r).prx_floor_dbm_ul  = floor_ul_mat;
        results(r).prx_floor_dbm     = floor_dl_mat;       % back-compat alias
        results(r).dists_m           = dists_mat;
        results(r).rx_locations      = loc_mat;
        results(r).rx_types          = types_cell;
        results(r).rx_heights        = height_mat;
        results(r).elapsed_s         = toc(t_start);
    end

    % --- Save -------------------------------------------------------------
    results_dir = fullfile(repo_root, cfg.io.results_dir);
    if ~exist(results_dir, 'dir'), mkdir(results_dir); end
    timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    out_path = fullfile(results_dir, ['scenario_', timestamp, '.mat']);
    latest   = fullfile(results_dir, 'latest.mat');
    save(out_path, 'cfg', 'results', '-v7.3');
    save(latest,   'cfg', 'results', '-v7.3');

    if has_cb
        progress_cb("done", n_regions, n_regions, M, M, ...
            sprintf("saved %s", out_path));
    end
end
