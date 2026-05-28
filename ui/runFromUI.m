function [results, cfg] = runFromUI(form_state, html_panel, axes_hist, axes_map, axes_traj, axes_ccdf, ccdf_opts)
%RUNFROMUI  Run a scenario driven by uihtml form state, stream progress, plot.
%
%   results = runFromUI(form_state, html_panel, axes_hist, axes_map)
%   results = runFromUI(form_state, html_panel, axes_hist, axes_map, axes_traj)
%
%   form_state   struct with fields gathered by galuboySimApp.
%   html_panel   uihtml component receiving live progress (Data property)
%   axes_hist    uiaxes target for the range histogram
%   axes_map     uiaxes target for the availability map
%   axes_traj    optional uiaxes target for the rotation-1 trajectory
%                + RX layout preview (rendered before runScenario starts)
%
%   Synchronous: blocks until runScenario returns. The progress callback
%   updates html_panel.Data and calls drawnow each tick to keep the UI
%   responsive without backgroundPool.

    if nargin < 5, axes_traj = []; end
    if nargin < 6, axes_ccdf = []; end
    if nargin < 7
        ccdf_opts = struct('show_inf', true, 'show_veh', true, 'show_tot', true);
    end

    cfg = buildCfgFromUI(form_state);

    link_hist = pickLink(form_state, 'link_hist');
    link_ccdf = pickLink(form_state, 'link_ccdf');

    % --- Pre-run trajectory preview ---------------------------------------
    % Resolve the selected region, draw rot-1's figure-8 plus a fresh RX
    % placement. The placement here is illustrative - runScenario calls
    % placeReceivers again per rotation, so the points used in the link
    % budget will differ statistically. The geometry is what matters.
    if ~isempty(axes_traj)
        repo_root = fileparts(fileparts(mfilename('fullpath')));
        region    = resolveRegion(cfg.regions(1), repo_root);
        preview_rx_cfg = cfg.rx;
        if isfield(region, 'placement_diameter_km') && ~isempty(region.placement_diameter_km)
            preview_rx_cfg.placement_diameter_km = region.placement_diameter_km;
        end
        [lons, lats] = generateTrajectory(region, cfg.flight, 0);
        [~, rx_table] = placeReceivers(region, preview_rx_cfg);
        plotTrajectoryMap(region, lons, lats, rx_table, axes_traj, ...
            preview_rx_cfg.placement_diameter_km);
        drawnow;
    end

    % Progress callback: post a struct into html_panel.Data each tick.
    cb = @(stage, ridx, nreg, ritx, nrots, msg) ...
        postProgress(html_panel, stage, ridx, nreg, ritx, nrots, msg);

    results = runScenario(cfg, cb);

    % Render final plots into the supplied axes
    plotRangeHistogram(results, cfg, axes_hist, 'link', link_hist);
    plotAvailabilityMap(results, cfg, axes_map);
    if ~isempty(axes_ccdf)
        plotAvailabilityCCDF(results, cfg, axes_ccdf, ...
            'show_inf',   logical(ccdf_opts.show_inf), ...
            'show_veh',   logical(ccdf_opts.show_veh), ...
            'show_total', logical(ccdf_opts.show_tot), ...
            'link',       link_ccdf);
    end
end


function link = pickLink(s, field)
    link = "dl";
    if isfield(s, field) && ~isempty(s.(field))
        v = lower(string(s.(field)));
        if ismember(v, ["dl","ul","bidi"])
            link = v;
        end
    end
end


% -------------------------------------------------------------------------
function cfg = buildCfgFromUI(s)
    cfg = buildConfig();

    cfg.tx.frequency_hz   = s.frequency_hz;
    cfg.tx.power_air_dbm  = s.power_air_dbm;
    cfg.tx.power_gnd_dbm  = s.power_gnd_dbm;
    cfg.tx.altitude_m     = s.altitude_m;
    cfg.tx.airborne.antenna_name = s.tx_air_antenna_name;
    cfg.tx.ground.antenna_name   = s.tx_gnd_antenna_name;

    cfg.rx.infantry.count            = s.n_inf;
    cfg.rx.infantry.antenna_name_dl  = s.inf_antenna_name_dl;
    cfg.rx.vehicular.count           = s.n_veh;
    cfg.rx.vehicular.antenna_name_dl = s.veh_antenna_name_dl;
    cfg.rx.airborne.antenna_name_ul  = s.rx_air_antenna_name_ul;

    if isfield(s, 'inf_height_m'), cfg.rx.infantry.height_m  = s.inf_height_m; end
    if isfield(s, 'veh_height_m'), cfg.rx.vehicular.height_m = s.veh_height_m; end

    if isfield(s, 'threshold_dbm'),         cfg.analysis.threshold_dbm     = s.threshold_dbm;     end
    cfg.analysis.percentile        = s.percentile;
    if isfield(s, 'fade_margin_db'), cfg.analysis.fade_margin_db = s.fade_margin_db; end

    if isfield(s, 'bandwidth_hz'),         cfg.propagation.bandwidth_hz    = s.bandwidth_hz;        end
    cfg.propagation.use_terrain = logical(s.use_terrain);

    if isfield(s, 'num_flight_steps'),     cfg.flight.num_flight_steps     = s.num_flight_steps;     end
    if isfield(s, 'num_rotations'),        cfg.flight.num_rotations        = s.num_rotations;        end
    if isfield(s, 'lemniscate_length_m'),  cfg.flight.lemniscate_length_m  = s.lemniscate_length_m;  end
    if isfield(s, 'lemniscate_scale_deg'), cfg.flight.lemniscate_scale_deg = s.lemniscate_scale_deg; end
    if isfield(s, 'max_tilt_deg'),         cfg.flight.max_tilt_deg         = s.max_tilt_deg;         end
    % Per-region RX-disk diameters live on cfg.regions; no global UI override.

    % Filter regions to the selected subset. Accept the new region_names
    % cellstr (from the multi-select listbox) or the legacy single
    % region_name (from older saved JSON form state).
    if isfield(s, 'region_names') && ~isempty(s.region_names)
        wanted = s.region_names;
    elseif isfield(s, 'region_name') && ~isempty(s.region_name)
        wanted = {s.region_name};
    else
        error('runFromUI:noRegion', 'No region selected in UI form state.');
    end
    if isstring(wanted), wanted = cellstr(wanted); end
    if ischar(wanted),   wanted = {wanted};        end

    all_names = arrayfun(@(rg) string(rg.name), cfg.regions);
    keep = false(numel(cfg.regions), 1);
    for k = 1:numel(cfg.regions)
        if any(strcmp(cfg.regions(k).name, wanted))
            keep(k) = true;
        end
    end
    unknown = setdiff(string(wanted), all_names);
    if ~isempty(unknown)
        error('runFromUI:unknownRegion', ...
              'Selected region(s) not in config.regions: %s', ...
              strjoin(cellstr(unknown), ', '));
    end
    cfg.regions = cfg.regions(keep);
end


function postProgress(html_panel, stage, region_idx, n_regions, rot_idx, n_rots, msg)
    % Write the progress event into html_panel.Data, which the JS side
    % renders via its DataChanged listener.
    payload = struct( ...
        'stage',      char(stage), ...
        'region_idx', region_idx, ...
        'n_regions',  n_regions, ...
        'rot_idx',    rot_idx, ...
        'n_rots',     n_rots, ...
        'msg',        char(msg), ...
        'tick',       posixtime(datetime('now')));   % bust DataChanged dedupe
    html_panel.Data = payload;
    drawnow;
end


