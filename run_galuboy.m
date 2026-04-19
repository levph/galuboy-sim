%% run_galuboy.m — Air-to-Ground Path Loss Simulation  v3.0
%
% Simulates signal propagation from an aerial transmitter flying a rotated
% figure-8 (Lemniscate of Bernoulli) trajectory to randomly placed ground
% receivers, then optionally performs link-budget and availability analysis.
%
% PHASE 1 — Data collection
%   Outer loop: rotates figure-8 from 0 to ~180° in step_degrees increments.
%   Inner loop: flies the full trajectory; records path loss, azimuth, and
%               elevation for every (rotation, step, receiver) triplet.
%   Result: raw_data struct saved to config.io.results_file (.mat).
%
% PHASE 2 — Analysis  (skipped when config.sim.run_analysis = false)
%   Applies link budget (TX/RX antenna gains + fade margin) to compute P_rx,
%   evaluates percentile-based availability, and plots a range histogram.
%
% Usage:
%   config = buildConfig();                  % load defaults
%   config.sim.run_analysis = false;         % Phase 1 only (optional)
%   run_galuboy                              % execute
%
% Authors: Tomer Antebi & Lev Panov
% Version: 3.0
% Date:    2026-04

%% ========================================================================
%  SETUP
%% ========================================================================
addpath(genpath(fileparts(mfilename('fullpath'))));

config = buildConfig();

% Resolve geographic bounds from OSM file
[config.geo.lonlim, config.geo.latlim, ~] = osmReadBounds(config.viz.osm_path);

% Antenna patterns (shared by both phases)
[tx_gain_func, rx_gain_func, n_rx_antenna_types] = setupAntennaPatterns();

% Propagation model
pm = propagationModel(config.prop.model, ...
    'TimeVariabilityTolerance',      config.prop.time_variability, ...
    'SituationVariabilityTolerance', config.prop.situation_variability);

% Site viewer
viewer = siteviewer('Buildings', config.viz.osm_path, ...
                    'Basemap',   'openstreetmap');

% Transmitter template (position updated each step)
tx = txsite('Name',                 'Aerial_Tx', ...
            'Latitude',             0, ...
            'Longitude',            0, ...
            'AntennaHeight',        config.tx.altitude_m, ...
            'TransmitterFrequency', config.tx.frequency_hz, ...
            'TransmitterPower',     10^((config.tx.power_dbm - 30) / 10));

%% ========================================================================
%  PHASE 1 — SIMULATION
%% ========================================================================
M  = config.sim.num_rotations;
Nf = config.sim.num_flight_steps;
N  = config.sim.n_rx;

fprintf('=== Phase 1: Simulation ===\n');
fprintf('Rotations       : %d  (step = %.1f deg, range = [0, %.1f) deg)\n', ...
        M, config.sim.step_degrees, M * config.sim.step_degrees);
fprintf('Flight steps    : %d\n', Nf);
fprintf('Receivers/rot.  : %d\n', N);
fprintf('Propagation     : %s\n\n', config.prop.model);

% Pre-allocate
raw_data.path_loss_db          = zeros(M, Nf, N, 'single');
raw_data.azimuth_rad           = zeros(M, Nf, N, 'single');
raw_data.elevation_rad         = zeros(M, Nf, N, 'single');
raw_data.dists_from_centroid_m = zeros(M, N,  'single');
raw_data.locations             = zeros(M, N, 2, 'single');

for m = 1:M
    rotation_deg = (m - 1) * config.sim.step_degrees;
    fprintf('--- Rotation %d/%d  (%.1f deg) ---\n', m, M, rotation_deg);

    [flight_lons, flight_lats] = generateTrajectory(config, rotation_deg);
    [rx_array,    rx_points]   = placeReceivers(N, config);

    raw_data.locations(m, :, :)          = rx_points;
    raw_data.dists_from_centroid_m(m, :) = distanceFromCentroid( ...
        config.geo.lonlim, config.geo.latlim, rx_points);

    slice = runFlightLoop(tx, rx_array, pm, flight_lons, flight_lats, config);

    raw_data.path_loss_db(m, :, :)   = slice.path_loss_db;
    raw_data.azimuth_rad(m, :, :)    = slice.azimuth_rad;
    raw_data.elevation_rad(m, :, :)  = slice.elevation_rad;
end

fprintf('\n=== Phase 1 Complete ===\n');

save(config.io.results_file, 'raw_data', 'config');
fprintf('Results saved → %s\n\n', config.io.results_file);

% --- Optional exit point --------------------------------------------------
if ~config.sim.run_analysis
    fprintf('config.sim.run_analysis = false — stopping after Phase 1.\n');
    return;
end

%% ========================================================================
%  PHASE 2 — ANALYSIS
%% ========================================================================
fprintf('=== Phase 2: Analysis ===\n');
fprintf('Fade margin     : %.1f dB  (%s)\n', config.prop.fade_margin_db, config.prop.fade_margin_type);
fprintf('Threshold (MDS) : %.1f dBm\n', config.sim.availability_threshold_dbm);
fprintf('Percentile      : %d-th\n\n', config.link.percentile);

prx = computeLinkBudget(raw_data, tx_gain_func, rx_gain_func, ...
                        n_rx_antenna_types, config);

available = computeAvailability(prx, config);

fprintf('Overall availability: %.1f %%  (%d / %d station×rotation pairs)\n\n', ...
        100 * mean(available(:)), sum(available(:)), numel(available));

all_distances = raw_data.dists_from_centroid_m(:);
all_available = available(:);

plotRangeHistogram(all_distances, all_available, config);

fprintf('=== Analysis Complete ===\n');
