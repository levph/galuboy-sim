function info = runPartA(cfg, region_idx)
%RUNPARTA  Part A driver: raytrace all regions, write a normalized xlsx workbook.
%
%   info = runPartA()                 % default config, all 7 regions
%   info = runPartA(cfg)              % custom config
%   info = runPartA(cfg, region_idx)  % subset of regions (indices into cfg.regions)
%
%   Registers the shared FABDEM terrain once, then for each region builds a
%   (hidden) Site Viewer, raytraces the constant-bank circle (raytraceRotation),
%   and writes results/partA_<ts>.xlsx (+ partA_latest.xlsx) with 3 sheets:
%
%     params   global scalars (freq, altitude, circle, tilt, dt, num_samples,
%              raytracing R/D, device counts + heights, ...)  [name/value]
%     regions  one row per region: centroid, bbox, disc, category, terrain
%              elevation, device-id range, and the tx trajectory (tx_lon/tx_lat/
%              tx_alt_msl_m) as comma-joined per-step arrays
%     devices  one row per device per region: id, region, type, height, lon, lat,
%              dist_m (from centroid), and pl_db / steer_gnd_deg / steer_air_deg
%              as comma-joined per-flight-point arrays
%
%   Requires R2025a (raytracing + readgeotable + Site Viewer) and the FABDEM
%   .dt2 tiles in resources/terrain.
%
%   Returns info: .devices, .regions, .file, .num_samples, .elapsed_s.

    if nargin < 1 || isempty(cfg),        cfg = buildConfigA();              end
    repo = fileparts(fileparts(mfilename('fullpath')));
    if nargin < 2 || isempty(region_idx), region_idx = 1:numel(cfg.regions); end

    samp = circularFlightSampling(cfg.flight);   % Nf etc. (same for all regions)
    Nf   = samp.num_samples;

    fprintf('=== Part A: %d region(s), %d flight steps each, SBR R=%d D=%d (coherent) ===\n', ...
        numel(region_idx), Nf, cfg.propagation.max_reflections, cfg.propagation.max_diffractions);

    % --- Resolve regions + register the union terrain once ---------------
    fprintf('[setup] resolving regions + registering union FABDEM terrain...\n');
    rgs = arrayfun(@(rc) resolveRegion(rc, repo), cfg.regions(region_idx));
    ulat = [min(arrayfun(@(g) g.latlim(1), rgs)), max(arrayfun(@(g) g.latlim(2), rgs))];
    ulon = [min(arrayfun(@(g) g.lonlim(1), rgs)), max(arrayfun(@(g) g.lonlim(2), rgs))];
    tiles = discoverTerrainTiles(fullfile(repo, cfg.io.terrain_dir), ulat, ulon);
    setupTerrain(cfg.propagation.terrain_name, tiles);
    fprintf('[setup] terrain "%s" registered (%d tile(s))\n', ...
        cfg.propagation.terrain_name, numel(tiles));

    pm = propagationModel('raytracing', 'Method', cfg.propagation.rt_method, ...
        'MaxNumReflections',  cfg.propagation.max_reflections, ...
        'MaxNumDiffractions', cfg.propagation.max_diffractions);

    % --- Per-region raytrace ---------------------------------------------
    t0 = tic;
    dev_parts = cell(1, numel(rgs));
    reg_parts = cell(1, numel(rgs));
    offset = 0;
    for r = 1:numel(rgs)
        region = rgs(r);
        rid    = region_idx(r);
        fprintf('\n[region %d/%d] %s (id %d): building scene (terrain + buildings)...\n', ...
            r, numel(rgs), region.name, rid);
        t_reg = tic;
        rng(cfg.flight.rng_seed + rid);              % reproducible RX placement
        sv = setupRegionScene(region, cfg);          % hidden viewer (single instance)
        clean = onCleanup(@() closeIfValid(sv));
        rot = raytraceRotation(region, cfg, sv, pm);

        n_rx = size(rot.pl_db, 2);
        dev_parts{r} = rotationToDeviceRows(rot, rid, offset);

        reg_parts{r} = table(rid, string(region.name), region.center_lat, region.center_lon, ...
            region.latlim(1), region.latlim(2), region.lonlim(1), region.lonlim(2), ...
            region.placement_diameter_km, string(region.category), rot.terrain_centroid_m, Nf, ...
            offset+1, offset+n_rx, ...
            numArrayToStr(rot.tx_lon), numArrayToStr(rot.tx_lat), numArrayToStr(rot.tx_alt_msl_m), ...
            'VariableNames', {'region_id','name','center_lat','center_lon', ...
            'lat_lo','lat_hi','lon_lo','lon_hi','placement_diameter_km','category', ...
            'terrain_centroid_m','num_samples','device_id_start','device_id_end', ...
            'tx_lon','tx_lat','tx_alt_msl_m'});

        offset = offset + n_rx;
        fprintf('[region %d/%d] %s done: %d devices (ids %d-%d) in %.1f s\n', ...
                r, numel(rgs), region.name, n_rx, offset-n_rx+1, offset, toc(t_reg));
        clear clean;                                  % closes the viewer
    end
    devicesTbl = vertcat(dev_parts{:});
    regionsTbl = vertcat(reg_parts{:});
    paramsTbl  = buildParamsTable(cfg, samp);

    % --- Write workbook (3 sheets) ---------------------------------------
    fprintf('\n[write] assembling %d device rows -> xlsx (3 sheets)...\n', height(devicesTbl));
    outdir = fullfile(repo, cfg.io.results_dir);
    if ~exist(outdir, 'dir'), mkdir(outdir); end
    ts = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    f1 = fullfile(outdir, sprintf('partA_%s.xlsx', ts));
    f2 = fullfile(outdir, 'partA_latest.xlsx');
    if exist(f1, 'file'), delete(f1); end
    writetable(paramsTbl,  f1, 'Sheet', 'params');
    writetable(regionsTbl, f1, 'Sheet', 'regions');
    writetable(devicesTbl, f1, 'Sheet', 'devices');
    copyfile(f1, f2);

    info = struct('devices', height(devicesTbl), 'regions', height(regionsTbl), ...
                  'num_samples', Nf, 'file', f1, 'elapsed_s', toc(t0));
    fprintf('Part A done: %d devices x %d points, %d region(s), %.1f s -> %s\n', ...
            info.devices, Nf, info.regions, info.elapsed_s, f1);
end

% -------------------------------------------------------------------------
function P = buildParamsTable(cfg, samp)
    name = [ "frequency_hz"; "tx_altitude_m_agl"; "circle_radius_m"; "max_tilt_deg"; ...
             "tilt_direction"; "max_rotation_deg"; "delta_t_s"; "num_samples"; ...
             "dt_effective_s"; "airspeed_mps"; "rt_method"; "max_reflections"; ...
             "max_diffractions"; "pl_combine"; "n_infantry"; "n_vehicular"; ...
             "infantry_height_m"; "vehicular_height_m"; "generated" ];
    value = [ string(cfg.tx.frequency_hz); string(cfg.tx.altitude_m); ...
              string(cfg.flight.circle_radius_m); string(cfg.flight.max_tilt_deg); ...
              string(cfg.flight.tilt_direction); string(cfg.flight.max_rotation_deg); ...
              string(cfg.flight.delta_t_s); string(samp.num_samples); ...
              string(samp.dt_effective_s); string(samp.v_mps); ...
              string(cfg.propagation.rt_method); string(cfg.propagation.max_reflections); ...
              string(cfg.propagation.max_diffractions); "coherent"; ...
              string(cfg.rx.infantry.count); string(cfg.rx.vehicular.count); ...
              string(cfg.rx.infantry.height_m); string(cfg.rx.vehicular.height_m); ...
              string(datetime('now')) ];
    P = table(name, value);
end

% -------------------------------------------------------------------------
function closeIfValid(sv)
    if ~isempty(sv) && isvalid(sv)
        close(sv);
    end
end
