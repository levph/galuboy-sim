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
%     results(r).region          (struct from resolveRegion)
%     results(r).available       [M x n_rx] logical
%     results(r).prx_pctile      [M x n_rx] single
%     results(r).dists_m         [M x n_rx] single  (distance from region centroid)
%     results(r).rx_locations    [M x n_rx x 2] single  (lon, lat per rx per rotation)
%     results(r).rx_types        cell {M x 1}   (categorical column per rotation)
%     results(r).rx_heights      [M x n_rx]    (m AGL)
%     results(r).elapsed_s       scalar
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
        'center_lat', 0, 'center_lon', 0);
    for r = 1:n_regions
        regions(r) = resolveRegion(region_cfgs(r), repo_root);
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
    ant.tx_pat  = loadAntennaPattern(cfg.tx.antenna_name,             antennas_dir);
    ant.inf_pat = loadAntennaPattern(cfg.rx.infantry.antenna_name,    antennas_dir);
    ant.veh_pat = loadAntennaPattern(cfg.rx.vehicular.antenna_name,   antennas_dir);

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

    tx_template = txsite('Name',                 'Aerial_Tx', ...
                         'Latitude',             0, ...
                         'Longitude',            0, ...
                         'AntennaHeight',        cfg.tx.altitude_m, ...
                         'TransmitterFrequency', cfg.tx.frequency_hz, ...
                         'TransmitterPower',     10^((cfg.tx.power_dbm - 30) / 10));

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
        'region', struct(), 'available', logical([]), 'prx_pctile', [], ...
        'dists_m', [], 'rx_locations', [], 'rx_types', {{}}, ...
        'rx_heights', [], 'elapsed_s', 0);

    for r = 1:n_regions
        region = regions(r);
        if has_cb
            progress_cb("region", r, n_regions, 0, M, ...
                sprintf("region %d/%d (%s)", r, n_regions, region.name));
        end

        avail_mat   = false(M, n_rx);
        prx_mat     = zeros(M, n_rx, 'single');
        dists_mat   = zeros(M, n_rx, 'single');
        loc_mat     = zeros(M, n_rx, 2, 'single');
        types_cell  = cell(M, 1);
        height_mat  = zeros(M, n_rx, 'single');

        t_start = tic;
        for m = 1:M
            rotation_deg = (m - 1) * cfg.flight.step_degrees;
            [flight_lons, flight_lats, flight_tilts, flight_headings] = ...
                generateTrajectory(region, cfg.flight, rotation_deg);
            [rx_array,    rx_table]    = placeReceivers(region, cfg.rx);

            rot = runRotation(tx_template, rx_array, rx_table, pm, map_name, ...
                              flight_lons, flight_lats, ant, cfg, ...
                              flight_tilts, flight_headings);

            avail_mat(m, :)   = rot.available(:).';
            prx_mat(m, :)     = single(rot.prx_pctile(:).');
            loc_mat(m, :, 1)  = single(rx_table.lon(:).');
            loc_mat(m, :, 2)  = single(rx_table.lat(:).');
            dists_mat(m, :)   = single(distanceFromCentroid( ...
                region.lonlim, region.latlim, [rx_table.lon, rx_table.lat]));
            types_cell{m}     = rx_table.type;
            height_mat(m, :)  = single(rx_table.height_m(:).');

            if has_cb
                progress_cb("rotation", r, n_regions, m, M, ...
                    sprintf("region %d/%d  rotation %d/%d  (avail=%.1f%%)", ...
                        r, n_regions, m, M, 100 * mean(rot.available)));
            end
        end

        results(r).region       = region;
        results(r).available    = avail_mat;
        results(r).prx_pctile   = prx_mat;
        results(r).dists_m      = dists_mat;
        results(r).rx_locations = loc_mat;
        results(r).rx_types     = types_cell;
        results(r).rx_heights   = height_mat;
        results(r).elapsed_s    = toc(t_start);
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
