function config = buildConfig()
%BUILDCONFIG  Return the default simulation configuration struct.
%
%   config = buildConfig()
%
%   Edit the values here to change simulation parameters.
%   Override individual fields after calling buildConfig() rather than
%   editing this file for one-off experiments.
%
%   Version: 3.0

    config = struct();

    % ---- Geographic boundaries (read from OSM, or set manually) ----------
    config.viz.osm_path = '../resources/yarka.osm';  % map.osm | yarka.osm | north_25km2.osm
    % config.geo.lonlim and config.geo.latlim are populated by run_galuboy.m
    % from osmReadBounds(); they are left empty here as sentinels.
    config.geo.lonlim = [];
    config.geo.latlim = [];

    % ---- Transmitter parameters ------------------------------------------
    config.tx.frequency_hz  = 4e9;    % Hz
    config.tx.power_dbm     = 47;     % dBm
    config.tx.altitude_m    = 1500;   % m AGL

    % ---- Propagation model -----------------------------------------------
    config.prop.model                 = 'longley-rice';
    config.prop.time_variability      = 0.9;
    config.prop.situation_variability = 0.9;
    config.prop.bandwidth_hz          = 25e3;  % Hz

    % Fade margin
    config.prop.fade_margin_type      = 'moderate';  % minimal|moderate|conservative|urban|custom
    config.prop.fade_margin_custom_db = 10;
    config.prop.fade_margin_db        = resolveFadeMargin(config.prop.fade_margin_type, ...
                                                           config.prop.fade_margin_custom_db);

    % ---- Simulation parameters -------------------------------------------
    config.sim.n_rx             = 60;    % random ground receivers per rotation
    config.sim.num_flight_steps = 100;   % trajectory samples per rotation

    % Trajectory rotation sweep [0, 180) deg
    config.sim.step_degrees  = 10;   % degrees per rotation step
    config.sim.num_rotations = [];   % [] → auto-derive as floor(180 / step_degrees)

    if isempty(config.sim.num_rotations)
        config.sim.num_rotations = floor(180 / config.sim.step_degrees);
    end

    % ---- Analysis parameters ---------------------------------------------
    config.sim.availability_threshold_dbm = -95;   % MDS (dBm)
    config.link.percentile                = 99;    % X-th percentile for availability decision

    % ---- Group / Monte Carlo ---------------------------------------------
    config.group.size                = 80;          % battalion size
    config.group.antenna_distribution = [1/3, 2/3]; % fraction per antenna type
    config.group.num_trials          = 1e2;         % Monte Carlo trials

    % ---- Histogram (range analysis) --------------------------------------
    config.hist.n_bins             = 50;
    config.hist.min_samples_per_bin = 1;

    % ---- Flow control ----------------------------------------------------
    % Set run_analysis = false to stop after Phase 1 (data collection).
    config.sim.run_analysis = true;

    % ---- I/O -------------------------------------------------------------
    config.io.results_file = 'simulation_results.mat';

end

% -------------------------------------------------------------------------
function fm_db = resolveFadeMargin(fade_type, custom_db)
    switch lower(fade_type)
        case 'minimal',       fm_db = 3;
        case 'moderate',      fm_db = 10;
        case 'conservative',  fm_db = 15;
        case 'urban',         fm_db = 20;
        case 'custom',        fm_db = custom_db;
        otherwise
            error('buildConfig:unknownFadeMarginType', ...
                  'Unknown fade_margin_type: ''%s''. Use minimal|moderate|conservative|urban|custom.', ...
                  fade_type);
    end
end
