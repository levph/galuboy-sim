%% Aerial Tx Range Analysis — Distance vs. Availability
%
%  Loads pre-computed propagation data across M concentric trajectories,
%  computes P_rx for every (trajectory, sample, rx-unit) triplet, then
%  builds a histogram showing the fraction of ground units that achieve
%  link availability as a function of their distance from the trajectory
%  centroid.
%
%  Input data dimensions (inside raw_data struct):
%    path_loss_db          [M, Nf, n_rx]   – path loss per trajectory / sample / rx
%    azimuth_rad           [M, Nf, n_rx]   – azimuth  TX→RX  (radians)
%    elevation_rad         [M, Nf, n_rx]   – elevation TX→RX  (radians)
%    dists_from_centroid_m [M, n_rx]       – ground-distance from centroid
%
% Authors: [Tomer Antebi & Lev Panov]
% Date: [26/1/25]

%% ========================================================================
%  INITIALIZATION
%% ========================================================================
clear all; close all; clc;

%% -----------------------------------------------------------------------
%  USER CONFIGURATION  (edit here)
%% -----------------------------------------------------------------------
config = struct();

% --- Link budget ---
config.tx.power_dbm            = 27;      % Transmitter power (dBm)
config.prop.fade_margin_db     = 10;      % Fade margin (dB)
config.link.mds_dbm            = -60;     % Minimum discernible signal / availability threshold (dBm)

% --- Availability criterion ---
config.link.percentile         = 99;      % X-th percentile used for availability decision
%   A station is "available" in a trajectory when its (100-percentile)-th
%   worst-case P_rx (i.e. the value exceeded in ≥ percentile % of samples)
%   is above MDS.  For percentile = 90 → link must be up ≥ 90 % of flight.

% --- Histogram ---
config.hist.n_bins             = 50;      % Number of distance bins
config.hist.min_samples_per_bin = 1;      % Bins with fewer samples are greyed out

% --- Data source ---
%   Set data_source to:
%     'mat'  – load from a .mat file  (set data_file accordingly)
%     'csv'  – load from CSV files    (set csv_* paths accordingly)
%     'manual' – data already in workspace; just needs to be wrapped below
config.data.source             = 'mat';
config.data.mat_file           = 'simulation_results.mat';   % .mat path

% CSV option – one file per variable, rows = M*Nf or M depending on var
config.data.csv_path_loss      = 'path_loss_db.csv';
config.data.csv_azimuth        = 'azimuth_rad.csv';
config.data.csv_elevation      = 'elevation_rad.csv';
config.data.csv_dists          = 'dists_from_centroid_m.csv';

%% ========================================================================
%  ANTENNA PATTERNS  (same helper as original script)
%% ========================================================================
[tx_gain_func, rx_gain_func, n_rx_antenna_types] = setupAntennaPatterns();

% Antenna type assignment is kept proportional (1/3 type-1, 2/3 type-2)
antenna_distribution = [1/3, 2/3];

%% ========================================================================
%  LOAD DATA
%% ========================================================================
raw_data = loadData(config);

[M, Nf, n_rx] = size(raw_data.path_loss_db);
fprintf("Data loaded:  M=%d trajectories | Nf=%d samples | n_rx=%d receivers\n\n", ...
        M, Nf, n_rx);

%% ========================================================================
%  ANTENNA TYPE ASSIGNMENT  (fixed per receiver index across all runs)
%% ========================================================================
antenna_dist   = antenna_distribution / sum(antenna_distribution);
antenna_counts = round(n_rx * antenna_dist);
antenna_counts(end) = n_rx - sum(antenna_counts(1:end-1));
cumsum_counts  = [0, cumsum(antenna_counts)];
% antenna_type(k) ∈ {1,2}  for receiver index k
antenna_type   = repelem(1:n_rx_antenna_types, antenna_counts);  % [1, n_rx]

%% ========================================================================
%  P_RX CALCULATION  (vectorised over M, Nf, n_rx)
%% ========================================================================
fprintf("Computing P_rx ...\n");

pl  = raw_data.path_loss_db;          % [M, Nf, n_rx]
az  = raw_data.azimuth_rad;           % [M, Nf, n_rx]
el  = raw_data.elevation_rad;         % [M, Nf, n_rx]

% TX antenna gain  — angle from RX back toward TX is az+π, elevation flips
tx_gain = tx_gain_func(mod(az + pi, 2*pi), -el);   % [M, Nf, n_rx]

% P_rx before RX antenna gain and fade margin
prx = config.tx.power_dbm + tx_gain - pl;           % [M, Nf, n_rx]

% Add RX antenna gain per type
for rx_type = 1:n_rx_antenna_types
    rx_idx = (cumsum_counts(rx_type) + 1):cumsum_counts(rx_type + 1);
    prx(:, :, rx_idx) = prx(:, :, rx_idx) + ...
        rx_gain_func(rx_type, ...
                     mod(az(:, :, rx_idx) + pi, 2*pi), ...
                     -el(:, :, rx_idx));             % [M, Nf, |rx_idx|]
end

% Apply fade margin
prx = prx - config.prop.fade_margin_db;             % [M, Nf, n_rx]

fprintf("P_rx range: [%.1f, %.1f] dBm\n", min(prx(:)), max(prx(:)));

%% ========================================================================
%  PERCENTILE-BASED AVAILABILITY  (per trajectory × per receiver)
%% ========================================================================
fprintf("Evaluating %d-th percentile availability (MDS = %.1f dBm) ...\n", ...
        config.link.percentile, config.link.mds_dbm);

% prx_percentile(m, k) = X-th percentile of P_rx over Nf samples
%   i.e. P_rx was ≥ this value in (100-X)% of the flight samples, and
%   the station is considered "available" if this exceeds MDS.
prx_percentile = squeeze(prctile(prx, config.link.percentile, 2));  % [M, n_rx]

available = prx_percentile > config.link.mds_dbm;    % logical [M, n_rx]

fprintf("Overall availability: %.1f %%  (%d / %d station×trajectory pairs)\n\n", ...
        100 * mean(available(:)), sum(available(:)), numel(available));

%% ========================================================================
%  FLATTEN  → one row per (trajectory, receiver) pair
%% ========================================================================
% dists_from_centroid_m is [M, n_rx]
all_distances  = raw_data.dists_from_centroid_m(:);   % [M*n_rx, 1]
all_available  = available(:);                         % [M*n_rx, 1]  logical

%% ========================================================================
%  HISTOGRAM:  distance bin  →  fraction of available stations
%% ========================================================================
fprintf("Building availability-vs-distance histogram ...\n");

dist_edges = linspace(min(all_distances), max(all_distances), ...
                      config.hist.n_bins + 1);

n_total     = histcounts(all_distances, dist_edges);           % samples per bin
n_available = histcounts(all_distances(all_available), dist_edges);  % available per bin

% Fraction (NaN for bins with too few samples)
frac_available = n_available ./ n_total;
insufficient   = n_total < config.hist.min_samples_per_bin;
frac_available(insufficient) = NaN;

bin_centers = 0.5 * (dist_edges(1:end-1) + dist_edges(2:end));

%% ========================================================================
%  PLOT
%% ========================================================================
fig = figure('Name', 'Range Analysis — Availability vs Distance', ...
             'Position', [100, 100, 900, 550]);

% --- Bar chart ---
ax = axes('Parent', fig);
b  = bar(ax, bin_centers, frac_available * 100, 1.0);
b.FaceColor = [0.20, 0.45, 0.75];
b.EdgeColor = 'none';
b.FaceAlpha = 0.85;

hold(ax, 'on');

% --- 50 % availability reference line ---
yline(ax, 50, '--r', 'LineWidth', 1.5, ...
      'Label', '50% availability', 'LabelHorizontalAlignment', 'left');

% --- Overlay sample-count as text on each bar (optional readability aid) ---
for k = 1:numel(bin_centers)
    if ~isnan(frac_available(k))
        text(ax, bin_centers(k), frac_available(k) * 100 + 1.5, ...
             sprintf('n=%d', n_total(k)), ...
             'HorizontalAlignment', 'center', 'FontSize', 7, 'Color', [0.3 0.3 0.3]);
    end
end

hold(ax, 'off');

% --- Labels & formatting ---
xlabel(ax, 'Distance from Trajectory Centroid (m)', 'FontSize', 12);
ylabel(ax, sprintf('Availability (%%)\n[P_{rx}(p%d) > MDS]', config.link.percentile), ...
       'FontSize', 12);
title(ax, sprintf(['Aerial Tx Range Analysis\n' ...
       'P_{tx}=%.0f dBm | MDS=%.0f dBm | Fade margin=%.0f dB | %d-th pct | M=%d trajectories'], ...
       config.tx.power_dbm, config.link.mds_dbm, config.prop.fade_margin_db, ...
       config.link.percentile, M), 'FontSize', 11);

ylim(ax, [0, 110]);
xlim(ax, [dist_edges(1), dist_edges(end)]);
grid(ax, 'on');
ax.GridAlpha = 0.3;
ax.Box = 'off';

fprintf("Plot complete.\n");

%% ========================================================================
%  OPTIONAL: print summary table to console
%% ========================================================================
fprintf("\n%-20s  %-10s  %-10s  %-15s\n", ...
        "Dist range (m)", "n_total", "n_avail", "Availability (%)");
fprintf("%s\n", repmat('-', 1, 60));
for k = 1:numel(bin_centers)
    if ~isnan(frac_available(k))
        fprintf("[%6.0f – %6.0f]    %-10d  %-10d  %.1f %%\n", ...
                dist_edges(k), dist_edges(k+1), ...
                n_total(k), n_available(k), frac_available(k)*100);
    else
        fprintf("[%6.0f – %6.0f]    %-10d  %-10d  (insufficient data)\n", ...
                dist_edges(k), dist_edges(k+1), n_total(k), n_available(k));
    end
end

%% ========================================================================
%  HELPER FUNCTIONS
%% ========================================================================

function raw_data = loadData(config)
    % Loads raw_data struct from .mat, CSV, or returns stub for 'manual' mode.
    switch lower(config.data.source)

        case 'mat'
            fprintf("Loading data from: %s\n", config.data.mat_file);
            loaded = load(config.data.mat_file, 'raw_data');
            raw_data = loaded.raw_data;

        case 'csv'
            fprintf("Loading data from CSV files ...\n");
            % Each CSV is expected to be a 2-D matrix saved row-major.
            % path_loss_db / azimuth / elevation: reshape to [M, Nf, n_rx] after loading.
            % dists_from_centroid_m:              shape [M, n_rx].
            %
            % Adjust the reshape call below once M, Nf, n_rx are known.
            pl_flat = readmatrix(config.data.csv_path_loss);
            az_flat = readmatrix(config.data.csv_azimuth);
            el_flat = readmatrix(config.data.csv_elevation);

            % Infer dimensions: rows = M*Nf, cols = n_rx
            [MNf, n_rx] = size(pl_flat);
            % *** USER: set M and Nf explicitly if auto-detection is ambiguous ***
            % For now we treat each row as one (trajectory, sample) pair and
            % leave M=MNf, Nf=1 as a safe default until clarified.
            M  = MNf;
            Nf = 1;
            warning("CSV loader: assuming M=%d, Nf=%d, n_rx=%d. " + ...
                    "Edit the reshape() calls if this is wrong.", M, Nf, n_rx);

            raw_data.path_loss_db         = reshape(pl_flat, M, Nf, n_rx);
            raw_data.azimuth_rad          = reshape(az_flat, M, Nf, n_rx);
            raw_data.elevation_rad        = reshape(el_flat, M, Nf, n_rx);
            raw_data.dists_from_centroid_m = readmatrix(config.data.csv_dists);

        case 'manual'
            % raw_data is expected to already exist in the caller's workspace.
            % This branch is a no-op; just validate the fields exist.
            if ~exist('raw_data', 'var')
                error("'manual' mode selected but raw_data not found in workspace.");
            end

        otherwise
            error("Unknown data source: '%s'. Use 'mat', 'csv', or 'manual'.", ...
                  config.data.source);
    end

    % Validate required fields
    required = {'path_loss_db', 'azimuth_rad', 'elevation_rad', 'dists_from_centroid_m'};
    for f = required
        if ~isfield(raw_data, f{1})
            error("raw_data is missing required field: '%s'", f{1});
        end
    end
end

function [tx_gain_func, rx_gain_func, n_rx_types] = setupAntennaPatterns()
    % Isotropic TX antenna (7 dBi) and two RX antenna types (2.5 dBi / 7 dBi).
    % Identical to the pattern used in the original simulation.

    tx_gain_dbi  = repmat(7, [2, 2]);
    tx_az_range  = linspace(0, 2*pi, size(tx_gain_dbi, 1));
    tx_el_range  = linspace(-pi, pi, size(tx_gain_dbi, 2));
    [tx_X, tx_Y] = meshgrid(tx_az_range, tx_el_range);

    rx_gain_dbi  = repmat(shiftdim([2.5; 7], -2), [2, 2]);
    n_rx_types   = size(rx_gain_dbi, 3);
    rx_az_range  = linspace(0, 2*pi, size(rx_gain_dbi, 1));
    rx_el_range  = linspace(-pi, pi, size(rx_gain_dbi, 2));
    [rx_X, rx_Y] = meshgrid(rx_az_range, rx_el_range);

    tx_gain_func = @(az, el) interp2(tx_X, tx_Y, tx_gain_dbi, az, el, 'linear');
    rx_gain_func = @(idx, az, el) interp2(rx_X, rx_Y, rx_gain_dbi(:,:,idx), az, el, 'linear');
end
