%% Air-to-Ground Path Loss Simulation
% Simulates signal propagation between an aerial transmitter following a
% figure-8 trajectory and ground-based receivers with varying antenna types
%
% Authors: 
% Date: [26/1/25]
% Version: ?

%% ========================================================================
%  INITIALIZATION
%  ========================================================================
clear all; close all; clc;

%% Configuration Structure
config = struct();

% Geographic boundaries
config.geo.lonlim = [34.8196200, 34.8458900];
config.geo.latlim = [31.9335200, 31.9553700];

% Transmitter parameters
config.tx.frequency_hz = 400e6;        % Hz
config.tx.power_dbm = 27;              % dBm
config.tx.altitude_m = 1500;            % Flight altitude

% Propagation model
config.prop.model = "longley-rice";
config.prop.time_variability = 0.9;
config.prop.situation_variability = 0.9;
config.prop.bandwidth_hz = 25e3;       % Hz

% Fade margin options based on environment/scenario
config.prop.fade_margin_type = "moderate";  % Options: 'minimal', 'moderate', 'conservative', 'urban', 'custom'
config.prop.fade_margin_custom_db = 10;     % Used only if fade_margin_type = 'custom'

fade_margins_dict = dictionary(["minimal", "moderate", "conservative", "urban", "custom"], ...
                                [3,10,15,20,10]);
% Determine fade margin based on type
config.prop.fade_margin_db = fade_margins_dict(config.prop.fade_margin_type);

fprintf("Using %s fade margin: %.1f dB\n", config.prop.fade_margin_type, config.prop.fade_margin_db);

% Simulation parameters
config.sim.num_flight_steps = 60;      % Trajectory samples
config.sim.grid_resolution = 40;       % 40x40 receiver grid
config.sim.availability_threshold_dbm = -95;

% Group analysis parameters
config.group.size = 80;                % Battalion size
config.group.antenna_distribution = [1/3, 2/3];  % Fraction per antenna type
config.group.num_trials = 1e2;         % Monte Carlo trials

% Visualization
config.viz.osm_path = "../resources/map.osm";

%% ========================================================================
%  ANTENNA PATTERN SETUP
%  ========================================================================

% Define antenna gain patterns
[tx_gain_func, rx_gain_func, n_rx_antenna_types] = setupAntennaPatterns();

%% ========================================================================
%  PROPAGATION MODEL & ENVIRONMENT
%  ========================================================================

% Initialize propagation model
pm = propagationModel(config.prop.model, ...
    "TimeVariabilityTolerance", config.prop.time_variability, ...
    "SituationVariabilityTolerance", config.prop.situation_variability);

% Setup site viewer
viewer = siteviewer("Buildings", config.viz.osm_path, ...
                    "Basemap", "openstreetmap");

%% ========================================================================
%  TRAJECTORY GENERATION
%  ========================================================================

[flight_lons, flight_lats] = generateFigure8Trajectory(config);
% flight_orientation_angles = TBD % Time varying antenna orientation driven by aircraft roll, pitch, and yaw

% Create initial transmitter site
tx = txsite("Name", "Aerial_Tx", ...
            "Latitude", flight_lats(1), ...
            "Longitude", flight_lons(1), ...
            "AntennaHeight", config.tx.altitude_m, ...
            "TransmitterFrequency", config.tx.frequency_hz, ...
            "TransmitterPower", 10^((config.tx.power_dbm - 30) / 10));

%% ========================================================================
%  RECEIVER GRID SETUP
%  ========================================================================

[rx_array, rx_points, lon_grid, lat_grid] = createReceiverGrid(config);
n_rx = length(rx_array);

%% ========================================================================
%  MAIN SIMULATION: PATH LOSS CALCULATION
%  ========================================================================

fprintf("=== Starting Flight Simulation ===\n");
fprintf("Flight steps: %d\n", config.sim.num_flight_steps);
fprintf("Receiver points: %d\n", n_rx);
fprintf("Propagation model: %s\n\n", config.prop.model);

[availability_matrix, flight_data] = simulateFlightPath(...
    tx, rx_array, flight_lons, flight_lats, pm, config);

% Calculate availability percentage
availability_percent = (availability_matrix / config.sim.num_flight_steps) * 100;
availability_map = reshape(availability_percent, size(lat_grid));

fprintf("\n=== Simulation Complete ===\n");

%% ========================================================================
%  VISUALIZATION: AVAILABILITY MAP
%  ========================================================================

visualizeAvailabilityMap(availability_map, flight_lons, flight_lats, ...
                         lon_grid, lat_grid, config);


% Save the current workspace to a .mat file for later analysis
save('simulation_results.mat');

%% ========================================================================
%  GROUP OPERABILITY ANALYSIS
%  ========================================================================

fprintf("\n=== Starting Group Operability Analysis ===\n");
fprintf("Group size: %d units\n", config.group.size);
fprintf("Monte Carlo trials: %d\n\n", config.group.num_trials);

[availability_stats] = analyzeGroupOperability(...
    flight_data, rx_points, tx_gain_func, rx_gain_func, ...
    n_rx_antenna_types, config);

% Visualize results
visualizeGroupStatistics(availability_stats, config);

fprintf("\n=== Analysis Complete ===\n");

%% ========================================================================
%  HELPER FUNCTIONS
%  ========================================================================

function [tx_gain_func, rx_gain_func, n_rx_types] = setupAntennaPatterns()
    % Setup transmitter antenna pattern (isotropic with 7 dBi gain)
    tx_gain_dbi = repmat(7, [2, 2]);
    tx_az_range = linspace(0, 2*pi, size(tx_gain_dbi, 1));
    tx_el_range = linspace(-pi, pi, size(tx_gain_dbi, 2));
    [tx_X, tx_Y] = meshgrid(tx_az_range, tx_el_range);
    
    % Setup receiver antenna patterns (two types: 2.5 dBi and 7 dBi)
    rx_gain_dbi = repmat(shiftdim([2.5; 7], -2), [2, 2]);
    n_rx_types = size(rx_gain_dbi, 3);
    rx_az_range = linspace(0, 2*pi, size(rx_gain_dbi, 1));
    rx_el_range = linspace(-pi, pi, size(rx_gain_dbi, 2));
    [rx_X, rx_Y] = meshgrid(rx_az_range, rx_el_range);
    
    % Create interpolation functions
    tx_gain_func = @(az, el) interp2(tx_X, tx_Y, tx_gain_dbi, az, el, 'linear');
    rx_gain_func = @(idx, az, el) interp2(rx_X, rx_Y, rx_gain_dbi(:,:,idx), az, el, 'linear');
end

function [lons, lats] = generateFigure8Trajectory(config)
    % Generate Lemniscate of Bernoulli (figure-8) trajectory
    center_lon = mean(config.geo.lonlim);
    center_lat = mean(config.geo.latlim);
    scale = 0.005;  % ~1km pattern size
    
    t = linspace(0, 2*pi, config.sim.num_flight_steps);
    
    % Parametric equations for lemniscate
    x_norm = cos(t) ./ (1 + sin(t).^2);
    y_norm = sin(t) .* cos(t) ./ (1 + sin(t).^2);
    
    % Map to geographic coordinates
    lons = center_lon + (x_norm * scale);
    lats = center_lat + (y_norm * scale);
end

function [rx_array, rx_points, lon_grid, lat_grid] = createReceiverGrid(config)
    % Create uniform grid of receiver points
    lon_space = linspace(config.geo.lonlim(1), config.geo.lonlim(2), ...
                         config.sim.grid_resolution);
    lat_space = linspace(config.geo.latlim(1), config.geo.latlim(2), ...
                         config.sim.grid_resolution);
    [lon_grid, lat_grid] = meshgrid(lon_space, lat_space);
    
    rx_points = [lon_grid(:), lat_grid(:)];
    rx_array = rxsite('Latitude', lat_grid(:), ...
                      'Longitude', lon_grid(:), ...
                      'AntennaHeight', 1.5);
end

function [availability_matrix, flight_data] = simulateFlightPath(...
    tx, rx_array, flight_lons, flight_lats, pm, config)
    
    n_rx = length(rx_array);
    n_steps = config.sim.num_flight_steps;
    
    availability_matrix = zeros(n_rx, 1);
    
    % Initialize flight data struct with descriptive field names
    flight_data = struct();
    flight_data.azimuth_rad = zeros(n_steps, n_rx);      % Azimuth angle (radians)
    flight_data.elevation_rad = zeros(n_steps, n_rx);    % Elevation angle (radians)
    flight_data.path_loss_db = zeros(n_steps, n_rx);     % Path loss (dB)
    
    for i = 1:n_steps
        % Update transmitter position
        tx.Longitude = flight_lons(i);
        tx.Latitude = flight_lats(i);
        
        % Calculate signal strength and apply fade margin
        signal_strength_dbm = sigstrength(rx_array, tx, pm).' - config.prop.fade_margin_db;
        
        % Calculate angles and path loss
        [azimuth_deg, elevation_deg] = angle(rx_array, tx);
        flight_data.azimuth_rad(i, :) = mod(2*pi + deg2rad(azimuth_deg), 2*pi);
        flight_data.elevation_rad(i, :) = deg2rad(elevation_deg);
        flight_data.path_loss_db(i, :) = config.tx.power_dbm - signal_strength_dbm;
        
        % Check availability threshold (using signal with fade margin)
        is_available = signal_strength_dbm > config.sim.availability_threshold_dbm;
        availability_matrix = availability_matrix + double(is_available);
        
        if mod(i, 10) == 0
            fprintf("Step %d/%d completed.\n", i, n_steps);
        end
    end
end

function visualizeAvailabilityMap(availability_map, flight_lons, flight_lats, ...
                                  lon_grid, lat_grid, config)
    figure('Name', 'Signal Availability Map', 'Position', [100, 100, 800, 600]);
    
    lon_space = lon_grid(1, :);
    lat_space = lat_grid(:, 1);
    
    imagesc(lon_space, lat_space, availability_map);
    set(gca, 'YDir', 'normal');
    colormap(parula);
    cb = colorbar;
    ylabel(cb, 'Availability (%)');
    
    xlabel('Longitude (°)');
    ylabel('Latitude (°)');
    title('Signal Availability with Figure-8 Flight Path');
    
    hold on;
    plot(flight_lons, flight_lats, 'r-', 'LineWidth', 2, 'DisplayName', 'Flight Path');
    legend('Location', 'best');
    hold off;
    
    grid on;
end

function [stats] = analyzeGroupOperability(flight_data, rx_points, ...
    tx_gain_func, rx_gain_func, n_rx_types, config)
    
    group_size = config.group.size;
    n_trials = config.group.num_trials;
    n_rx = size(rx_points, 1);
    
    % Determine antenna type assignment
    antenna_dist = config.group.antenna_distribution / sum(config.group.antenna_distribution);
    antenna_counts = round(group_size * antenna_dist);
    antenna_counts(end) = group_size - sum(antenna_counts(1:end-1));
    cumsum_counts = [0, cumsum(antenna_counts)];
    antenna_per_rx = repelem(1:n_rx_types, antenna_counts);
    
    availability_mat = nan(group_size, n_trials);
    
    for trial = 1:n_trials
        % Randomly select receivers
        rx_indices = datasample(1:n_rx, group_size, 'Replace', false);
        
        % Extract data for selected receivers - much more readable!
        azimuth = flight_data.azimuth_rad(:, rx_indices);
        elevation = flight_data.elevation_rad(:, rx_indices);
        path_loss = flight_data.path_loss_db(:, rx_indices);
        
        % Calculate received power with TX antenna gain
        prx_mat = config.tx.power_dbm + ...
                  tx_gain_func(mod(azimuth + pi, 2*pi), -elevation) - ...
                  path_loss;
        
        % Add RX antenna gain per type
        for rx_type = 1:n_rx_types
            rx_idx = (cumsum_counts(rx_type) + 1):cumsum_counts(rx_type + 1);
            prx_mat(:, rx_idx) = prx_mat(:, rx_idx) + ...
                rx_gain_func(rx_type, mod(azimuth(:, rx_idx) + pi, 2*pi), ...
                            -elevation(:, rx_idx));
        end
        
        % Apply fade margin to received power before availability check
        prx_with_margin = prx_mat - config.prop.fade_margin_db;
        
        % Calculate availability
        group_availability = prx_with_margin > config.sim.availability_threshold_dbm;
        availability_mat(:, trial) = mean(group_availability, 1).';
        
        if mod(trial, 10) == 0
            fprintf("Trial %d/%d completed.\n", trial, n_trials);
        end
    end
    
    % Calculate statistics
    stats.min_per_trial = min(availability_mat, [], 1);
    stats.mean_per_trial = mean(availability_mat, 1);
    stats.max_per_trial = max(availability_mat, [], 1);
    stats.all_devices = availability_mat(:);
end

function visualizeGroupStatistics(stats, config)
    n_bins = 20;
    
    figure('Name', 'Group Availability Distribution', 'Position', [100, 100, 1000, 700]);
    
    subplot(2, 2, 1);
    histogram(stats.min_per_trial, n_bins, 'Normalization', 'probability');
    title('Minimum Availability within Groups');
    xlabel('Availability (fraction)');
    ylabel('Probability');
    grid on;
    
    subplot(2, 2, 2);
    histogram(stats.mean_per_trial, n_bins, 'Normalization', 'probability');
    title('Mean Availability within Groups');
    xlabel('Availability (fraction)');
    ylabel('Probability');
    grid on;
    
    subplot(2, 2, 3);
    histogram(stats.max_per_trial, n_bins, 'Normalization', 'probability');
    title('Maximum Availability within Groups');
    xlabel('Availability (fraction)');
    ylabel('Probability');
    grid on;
    
    subplot(2, 2, 4);
    histogram(stats.all_devices, n_bins, 'Normalization', 'probability');
    title('Per-Device Availability (All Trials)');
    xlabel('Availability (fraction)');
    ylabel('Probability');
    grid on;
    
    sgtitle(sprintf('Group Operability Analysis (N=%d, Trials=%d)', ...
            config.group.size, config.group.num_trials));
end