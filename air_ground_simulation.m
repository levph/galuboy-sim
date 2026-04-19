%% Air-to-Ground Path Loss Simulation
% Simulates signal propagation between an aerial transmitter following a
% rotated figure-8 trajectory and ground-based receivers with varying antenna types.
%
% Outer loop: Rotates the figure-8 trajectory from 0 to <180 degrees in
%             step_degrees increments (M = 180 / step_degrees rotations total,
%             or an explicit M can be set in config).
% Inner loop: For each rotation, flies the full trajectory and computes
%             path loss, azimuth, elevation, and distance for every Rx point.
%
% Authors: 
% Date: [25/2/26]
% Version: 2.0

%% ========================================================================
%  INITIALIZATION
%  ========================================================================
clear all; close all; clc;

%% Configuration Structure
config = struct();

config.viz.osm_path = "../resources/yarka.osm"; % map.osm, north_25km2.osm, yarka.osm
[config.geo.lonlim, config.geo.latlim, ~] = osmReadBounds(config.viz.osm_path);

% Transmitter parameters
config.tx.frequency_hz  = 4e9;   % Hz
config.tx.power_dbm     = 47;      % dBm
config.tx.altitude_m    = 1500;    % Flight altitude (m)

% Propagation model
config.prop.model                  = "longley-rice";
config.prop.time_variability       = 0.9;
config.prop.situation_variability  = 0.9;
config.prop.bandwidth_hz           = 25e3;   % Hz


% Simulation parameters
config.sim.n_rx                          = 60;
config.sim.num_flight_steps              = 100;    % Trajectory samples per rotation

% ---- Trajectory rotation parameters --------------------------------
% The figure-8 is tested at M evenly-spaced rotations over [0, 180) deg.
% Set step_degrees and leave num_rotations = [] to auto-derive M,
% OR set num_rotations explicitly (it takes priority).
config.sim.step_degrees  = 10;   % degrees per rotation step
config.sim.num_rotations = [];   % [] → auto-derive as floor(180/step_degrees)
                                 % set an integer to override

% Resolve M
if isempty(config.sim.num_rotations)
    config.sim.num_rotations = floor(180 / config.sim.step_degrees);
end
fprintf("Trajectory rotations: %d  (step = %.1f deg, range = [0, %.1f) deg)\n\n", ...
        config.sim.num_rotations, config.sim.step_degrees, ...
        config.sim.num_rotations * config.sim.step_degrees);

% Monte-Carlo / group analysis parameters
config.group.size                = 80;         % Battalion size
config.group.antenna_distribution = [1/3, 2/3]; % Fraction per antenna type
config.group.num_trials          = 1e2;         % Monte Carlo trials


%% ========================================================================
%  ANTENNA PATTERN SETUP
%  ========================================================================
[tx_gain_func, rx_gain_func, n_rx_antenna_types] = setupAntennaPatterns();

%% ========================================================================
%  PROPAGATION MODEL & ENVIRONMENT
%  ========================================================================
pm = propagationModel(config.prop.model, ...
    "TimeVariabilityTolerance",  config.prop.time_variability, ...
    "SituationVariabilityTolerance", config.prop.situation_variability);

viewer = siteviewer("Buildings", config.viz.osm_path, ...
                    "Basemap",   "openstreetmap");



%% ========================================================================
%  TRANSMITTER TEMPLATE  (position updated inside the loop)
%  ========================================================================
tx = txsite("Name", "Aerial_Tx", ...
            "Latitude",            0, ...   % placeholder
            "Longitude",           0, ...   % placeholder
            "AntennaHeight",       config.tx.altitude_m, ...
            "TransmitterFrequency",config.tx.frequency_hz, ...
            "TransmitterPower",    10^((config.tx.power_dbm - 30) / 10));

%% ========================================================================
%  PRE-ALLOCATE RESULTS
%
%  raw_data.(field) is  [num_rotations × num_flight_steps × n_rx]
%  availability_cube    [num_rotations × n_rx]   — fraction of steps above threshold
%  ========================================================================
M  = config.sim.num_rotations;
Nf = config.sim.num_flight_steps;
n_rx = config.sim.n_rx;
raw_data = struct();
raw_data.path_loss_db           = zeros(M, Nf, n_rx, 'single');
raw_data.azimuth_rad            = zeros(M, Nf, n_rx, 'single');
raw_data.elevation_rad          = zeros(M, Nf, n_rx, 'single');
raw_data.dists_from_centroid_m  = zeros(M, n_rx, 'single'); 

fprintf("=== Starting Rotated-Trajectory Simulation ===\n");
fprintf("Rotations : %d\n", M);
fprintf("Flight steps per rotation: %d\n", Nf);
fprintf("Receiver points: %d\n", n_rx);
fprintf("Propagation model: %s\n\n", config.prop.model);

%% ========================================================================
%  OUTER LOOP — trajectory rotations
%  ========================================================================


for m = 1:M
    rotation_deg = (m - 1) * config.sim.step_degrees;

    fprintf("--- Rotation %d/%d  (%.1f deg) ---\n", m, M, rotation_deg);

    % Generate trajectory for this rotation angle
    [flight_lons, flight_lats] = generateFigure8Trajectory(config, rotation_deg);
    
    % ----------------------------------------------------------------
    %  INNER LOOP — flight steps
    % ----------------------------------------------------------------
    
    [rx_array, rx_points] = randomNRx(n_rx, config);
    raw_data.locations(m, :, :) = rx_points;
    raw_data.dists_from_centroid_m(m,:) = distanceFromCentroid(config.geo.lonlim,config.geo.latlim, rx_points);
              
    for i = 1:Nf
        % Update transmitter position
        tx.Longitude = flight_lons(i);
        tx.Latitude  = flight_lats(i);

        % ---- Path loss via Longley-Rice (sigstrength, then invert) ------
        pl_db = pathloss(pm,rx_array,tx);
        raw_data.path_loss_db(m, i, :) = pl_db;

        % ---- Geometry ---------------------------------------------------
        [az_deg, el_deg] = angle(rx_array, tx);   % MATLAB returns degrees
        raw_data.azimuth_rad(m, i, :)   = mod(deg2rad(az_deg) + 2*pi, 2*pi);
        raw_data.elevation_rad(m, i, :) = deg2rad(el_deg);

        if mod(i, 10) == 0
            fprintf("  Step %d/%d completed.\n", i, Nf);
        end
    end  % inner flight-step loop

    

end  % outer rotation loop

fprintf("\n=== Simulation Complete ===\n");

%% ========================================================================
%  SAVE RESULTS
%  ========================================================================
save('simulation_results.mat', 'raw_data', 'config');
% Save results as CSV
writetable(struct2table(raw_data), 'simulation_results.csv');
fprintf("Results saved to simulation_results.mat\n");




%% ========================================================================
%  HELPER FUNCTIONS
%  ========================================================================

% -------------------------------------------------------------------------
function [tx_gain_func, rx_gain_func, n_rx_types] = setupAntennaPatterns()
% Isotropic TX (7 dBi) and two RX types (2.5 dBi and 7 dBi)

    tx_gain_dbi = repmat(7, [2, 2]);
    tx_az  = linspace(0,    2*pi, size(tx_gain_dbi, 1));
    tx_el  = linspace(-pi,    pi, size(tx_gain_dbi, 2));
    [tx_X, tx_Y] = meshgrid(tx_az, tx_el);

    rx_gain_dbi = repmat(shiftdim([2.5; 7], -2), [2, 2]);
    n_rx_types  = size(rx_gain_dbi, 3);
    rx_az  = linspace(0,    2*pi, size(rx_gain_dbi, 1));
    rx_el  = linspace(-pi,    pi, size(rx_gain_dbi, 2));
    [rx_X, rx_Y] = meshgrid(rx_az, rx_el);

    tx_gain_func = @(az, el) interp2(tx_X, tx_Y, tx_gain_dbi, az, el, 'linear');
    rx_gain_func = @(idx, az, el) interp2(rx_X, rx_Y, rx_gain_dbi(:,:,idx), az, el, 'linear');
end

% -------------------------------------------------------------------------
function [lons, lats] = generateFigure8Trajectory(config, rotation_deg)
% Lemniscate of Bernoulli rotated by rotation_deg degrees around its centre

    center_lon = mean(config.geo.lonlim);
    center_lat = mean(config.geo.latlim);
    scale = 0.005;   % ~1 km pattern

    t = linspace(0, 2*pi, config.sim.num_flight_steps);

    % Base parametric lemniscate (normalised)
    x_norm = cos(t) ./ (1 + sin(t).^2);
    y_norm = sin(t) .* cos(t) ./ (1 + sin(t).^2);

    % Rotate in the local (x,y) plane
    theta = deg2rad(rotation_deg);
    x_rot = x_norm * cos(theta) - y_norm * sin(theta);
    y_rot = x_norm * sin(theta) + y_norm * cos(theta);

    % Map to geographic coordinates
    lons = center_lon + x_rot * scale;
    lats = center_lat + y_rot * scale;
end

% -------------------------------------------------------------------------

function [rx_array, rx_points] = randomNRx(N, config)
    % Choose N random (x,y) point in [lonlim(1),lonlim(2)],
    % [latlim(1),latlim(2)]:
    limits = [config.geo.lonlim; config.geo.latlim];      
    rx_points = limits(:,1)' + rand(N,2) .* diff(limits,1,2)'; % Nx2
    rx_array  = rxsite('Latitude',     rx_points(:,2), ...
                       'Longitude',    rx_points(:,1), ...
                       'AntennaHeight', 1.5);
end

% -------------------------------------------------------------------------
function gnd_dist_m = distanceFromCentroid(lonlim,latlim, rx_points)
    center_lon = mean(lonlim);
    center_lat = mean(latlim);
    gnd_dist_m = haversineDistance( ...
            center_lat, center_lon, ...
            rx_points(:,2).', rx_points(:,1).');
end
% -------------------------------------------------------------------------
function dist_m = haversineDistance(lat1_deg, lon1_deg, lat2_deg, lon2_deg)
% Great-circle distance in metres between one point and an array of points.
% Inputs may be scalars or [1 × N] arrays (lat2/lon2 should be the array).

    R = 6371000;   % Earth radius (m)
    dlat = deg2rad(lat2_deg - lat1_deg);
    dlon = deg2rad(lon2_deg - lon1_deg);
    a = sin(dlat/2).^2 + cos(deg2rad(lat1_deg)) .* cos(deg2rad(lat2_deg)) .* sin(dlon/2).^2;
    dist_m = 2 * R * asin(sqrt(a));
end

% -------------------------------------------------------------------------
function visualizeAvailabilityMap(availability_map, flight_lons, flight_lats, ...
                                  lon_grid, lat_grid, config)

    figure('Name', 'Signal Availability Map (mean over rotations)', ...
           'Position', [100, 100, 800, 600]);

    lon_space = lon_grid(1, :);
    lat_space = lat_grid(:, 1);

    imagesc(lon_space, lat_space, availability_map);
    set(gca, 'YDir', 'normal');
    colormap(parula);
    cb = colorbar;
    ylabel(cb, 'Mean Availability (%)');

    xlabel('Longitude (°)');
    ylabel('Latitude (°)');
    title(sprintf('Signal Availability — mean over %d trajectory rotations', ...
                  config.sim.num_rotations));

    hold on;
    plot(flight_lons, flight_lats, 'r-', 'LineWidth', 2, 'DisplayName', 'Flight Path (0°)');
    legend('Location', 'best');
    hold off;
    grid on;
end

% -------------------------------------------------------------------------
function visualizeGroupStatistics(stats, config)
    n_bins = 20;

    figure('Name', 'Group Availability Distribution', 'Position', [100, 100, 1000, 700]);

    subplot(2,2,1);
    histogram(stats.min_per_trial,  n_bins, 'Normalization', 'probability');
    title('Minimum Availability within Groups');
    xlabel('Availability (fraction)'); ylabel('Probability'); grid on;

    subplot(2,2,2);
    histogram(stats.mean_per_trial, n_bins, 'Normalization', 'probability');
    title('Mean Availability within Groups');
    xlabel('Availability (fraction)'); ylabel('Probability'); grid on;

    subplot(2,2,3);
    histogram(stats.max_per_trial,  n_bins, 'Normalization', 'probability');
    title('Maximum Availability within Groups');
    xlabel('Availability (fraction)'); ylabel('Probability'); grid on;

    subplot(2,2,4);
    histogram(stats.all_devices,    n_bins, 'Normalization', 'probability');
    title('Per-Device Availability (All Trials)');
    xlabel('Availability (fraction)'); ylabel('Probability'); grid on;

    sgtitle(sprintf('Group Operability Analysis (N=%d, Trials=%d, Rotations=%d)', ...
            config.group.size, config.group.num_trials, config.sim.num_rotations));
end
