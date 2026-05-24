%MAKE_FIGURES  Run the simulation over 7 terrain regions and export the
%   5 PNGs (300 dpi) consumed by the Hebrew engineering report.
%
%   Outputs:
%       results/report_run.mat                 (cfg + results struct array)
%       report/figures/fig1_trajectory.png
%       report/figures/fig2_availability_map.png
%       report/figures/fig3_range_histogram.png
%       report/figures/fig4_ccdf.png
%       report/figures/fig5_antenna_patterns.png
%
%   On-prem: edit the `cfg.flight.num_flight_steps` override below
%   (currently 50 for tractable runtime) before running with real parameters.

repo_root = fileparts(fileparts(mfilename('fullpath')));   % <repo>/report -> <repo>
addpath(genpath(repo_root));

fig_dir = fullfile(repo_root, 'report', 'figures');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

% =========================================================================
% STAGE 1 - Run the simulation over the 7 non-Yarka terrain regions
% =========================================================================
cfg = buildConfig();

% Drop 'yarka' (OSM-driven default) - keep the 7 terrain regions:
% urban_suburban_01..03, forest_01, mountain_01..02, rural_open_01
cfg.regions = cfg.regions(~strcmp({cfg.regions.name}, 'yarka'));

% Reduce flight steps for tractable runtime (50 instead of 100).
% **Remove this line for the on-prem real-parameter run.**
cfg.flight.num_flight_steps = 50;

fprintf('[make_figures] Running %d regions x %d rotations x %d steps\n', ...
        numel(cfg.regions), cfg.flight.num_rotations, cfg.flight.num_flight_steps);
fprintf('[make_figures] Regions: %s\n', strjoin({cfg.regions.name}, ', '));

t0 = tic;
results = runScenario(cfg, @(varargin) fprintf('  %s\n', plotProgress(varargin{:})));
fprintf('[make_figures] Simulation elapsed: %.1f min\n', toc(t0) / 60);

out_path = fullfile(repo_root, cfg.io.results_dir, 'report_run.mat');
save(out_path, 'cfg', 'results', '-v7.3');
fprintf('[make_figures] Saved %s\n', out_path);

% =========================================================================
% STAGE 2 - Render the 5 figures
% =========================================================================
region_names = arrayfun(@(r) string(r.region.name), results);
fprintf('[make_figures] Rendering figures for: %s\n', strjoin(region_names, ', '));

% Pick mountain_01 as the focal region for figs 1+2 (most pronounced terrain)
focal_idx = find(region_names == "mountain_01", 1);
if isempty(focal_idx), focal_idx = 1; end
focal_region = results(focal_idx).region;

% ----- Figure 1 - Flight trajectory + RX layout (focal region, rotation 1)
fprintf('[make_figures] fig 1: trajectory map (%s)\n', focal_region.name);
[flight_lons, flight_lats, ~, ~] = generateTrajectory(focal_region, cfg.flight, 0);
[~, rx_table_focal] = placeReceivers(focal_region, cfg.rx);

f = makeFig([100 100 1100 700]);
ax = axes('Parent', f);
plotTrajectoryMap(focal_region, flight_lons, flight_lats, rx_table_focal, ax, 5);
set(ax, 'FontSize', 11);
title(ax, sprintf('Figure-8 trajectory + RX layout  (%s, rotation 1)', focal_region.name), ...
      'FontSize', 13, 'FontWeight', 'normal');
exportgraphics(f, fullfile(fig_dir, 'fig1_trajectory.png'), ...
               'Resolution', 300, 'BackgroundColor', 'white');
close(f);

% ----- Figure 2 - Availability map (focal region only)
fprintf('[make_figures] fig 2: availability map (%s)\n', focal_region.name);
f = makeFig([100 100 1100 700]);
ax = axes('Parent', f);
plotAvailabilityMap(results(focal_idx), cfg, ax);
xlabel(ax, 'Longitude (deg)', 'FontSize', 12);
ylabel(ax, 'Latitude (deg)',  'FontSize', 12);
overall = 100 * mean(results(focal_idx).available(:));
title(ax, sprintf('Availability map  -  %s  (overall %.1f%%)', ...
      focal_region.name, overall), 'FontSize', 13, 'FontWeight', 'normal');
set(ax, 'FontSize', 11);
exportgraphics(f, fullfile(fig_dir, 'fig2_availability_map.png'), ...
               'Resolution', 300, 'BackgroundColor', 'white');
close(f);

% ----- Figure 3 - Range histogram (aggregate across 7 regions)
fprintf('[make_figures] fig 3: range histogram (aggregate)\n');
f = makeFig([100 100 1100 600]);
ax = axes('Parent', f);
plotRangeHistogram(results, cfg, ax);
set(ax, 'FontSize', 11);
title(ax, 'Availability vs distance from centroid  (aggregate, 7 regions)', ...
      'FontSize', 13, 'FontWeight', 'normal');
exportgraphics(f, fullfile(fig_dir, 'fig3_range_histogram.png'), ...
               'Resolution', 300, 'BackgroundColor', 'white');
close(f);

% ----- Figure 4 - Availability CCDF (aggregate, polished)
fprintf('[make_figures] fig 4: availability CCDF (aggregate)\n');
f = makeFig([100 100 1000 600]);
ax = axes('Parent', f);
plotAvailabilityCCDF(results, cfg, ax);
exportgraphics(f, fullfile(fig_dir, 'fig4_ccdf.png'), ...
               'Resolution', 300, 'BackgroundColor', 'white');
close(f);

% ----- Figure 5 - Antenna patterns (3 polar plots side by side)
fprintf('[make_figures] fig 5: antenna polar patterns\n');
ant_dir = fullfile(repo_root, 'resources', 'antennas');
patterns = { ...
    'tx_omni',              'TX omni (aerial)'; ...
    'rx_infantry_omni',     'RX infantry omni'; ...
    'rx_vehicular_dipole',  'RX vehicular dipole'};

f = makeFig([100 100 1500 500]);
tl = tiledlayout(f, 1, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
title(tl, 'Antenna elevation patterns  (gain dBi vs elevation, az = 0)', ...
      'FontSize', 13, 'FontWeight', 'normal');

el_grid = deg2rad(linspace(-90, 90, 181));
for k = 1:size(patterns, 1)
    pat = loadAntennaPattern(patterns{k, 1}, ant_dir);
    if pat.is_isotropic
        gains = pat.flat_gain_dbi * ones(size(el_grid));
    else
        gains = pat.gain_func(zeros(size(el_grid)), el_grid);
    end

    pax = polaraxes(tl);
    pax.Layout.Tile = k;
    % Map elevation [-90, +90] to polar theta with 0=horizon, +90=zenith,
    % -90=nadir (so the pattern reads naturally for an air-to-ground link).
    polarplot(pax, el_grid, gains, '-', 'LineWidth', 2.0, 'Color', [0.12 0.47 0.71]);
    pax.ThetaZeroLocation = 'right';
    pax.ThetaDir = 'counterclockwise';
    pax.ThetaLim = [-90, 90];
    pax.RLim = [-15, 10];
    pax.FontSize = 10;
    title(pax, patterns{k, 2}, 'FontSize', 12, 'FontWeight', 'normal');
end

exportgraphics(f, fullfile(fig_dir, 'fig5_antenna_patterns.png'), ...
               'Resolution', 300, 'BackgroundColor', 'white');
close(f);

fprintf('[make_figures] All 5 figures written to %s\n', fig_dir);


% -------------------------------------------------------------------------
function f = makeFig(pos)
%MAKEFIG  Create a figure that always renders in classic light-mode colors,
%   regardless of MATLAB R2025a's theme setting.
    f = figure('Visible', 'off', 'Position', pos, 'Color', 'w');
    try
        % R2025a introduced figure themes; force the classic light theme so
        % exported PNGs are not dark-mode.
        f.Theme = 'light';
    catch
        % older releases: nothing to do
    end
    set(f, 'InvertHardcopy', 'off', 'PaperPositionMode', 'auto');
end
