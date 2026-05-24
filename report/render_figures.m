%RENDER_FIGURES  Render the report figures from results/report_run.mat.
%   Pure consumer of pre-computed results — does not re-run the sim.
%
%   Produces (32 PNGs + antenna diagrams):
%     report/figures/fig_<region>_trajectory.png
%     report/figures/fig_<region>_availability_map.png    (uses available_bidi)
%     report/figures/fig_<region>_range_hist.png
%     report/figures/fig_<region>_ccdf.png
%       — for each of 7 regions (urban_suburban_01/02/03 + forest_01 +
%         mountain_01/02 + rural_open_01)
%     report/figures/fig_aggregate_suburban_range_hist.png
%     report/figures/fig_aggregate_suburban_ccdf.png
%     report/figures/fig_aggregate_non_suburban_range_hist.png
%     report/figures/fig_aggregate_non_suburban_ccdf.png
%     report/figures/fig_antenna_<name>.png  (one per antenna CSV)
%
%   Availability-map plots use the bidi (DL AND UL) flag, since that is
%   the strict definition of "the link works" under the v4 architecture.

repo_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(repo_root));

fig_dir = fullfile(repo_root, 'report', 'figures');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

S = load(fullfile(repo_root, 'results', 'report_run.mat'));
cfg     = S.cfg;
results = S.results;

region_names = arrayfun(@(r) string(r.region.name), results);
fprintf('[render_figures] Loaded %d regions: %s\n', ...
        numel(results), strjoin(region_names, ', '));

% Helper: extract per-region placement radius (km)
get_radius_km = @(r) double(r.placement_radius_km);

% Suburban vs non-suburban partition matches the run_for_report.m split
is_suburban = startsWith(region_names, "urban_suburban");

% =========================================================================
% Per-region figures
% =========================================================================
for k = 1:numel(results)
    r = results(k);
    region = r.region;
    radius = get_radius_km(r);
    name   = char(region.name);

    fprintf('[render_figures] %s (%.0f km)\n', name, radius);

    % --- Trajectory + RX layout --------------------------------------
    [flight_lons, flight_lats, ~, ~] = generateTrajectory(region, cfg.flight, 0);
    cfg_focal = cfg;
    cfg_focal.rx.placement_radius_km = radius;
    [~, rx_table] = placeReceivers(region, cfg_focal.rx);

    f = makeFig([100 100 1100 700]);
    ax = axes('Parent', f);
    plotTrajectoryMap(region, flight_lons, flight_lats, rx_table, ax, radius);
    set(ax, 'FontSize', 11);
    title(ax, sprintf('Trajectory + RX layout  (%s, %.0f km radius)', ...
          name, radius), 'FontSize', 13, 'FontWeight', 'normal');
    exportgraphics(f, fullfile(fig_dir, sprintf('fig_%s_trajectory.png', name)), ...
                   'Resolution', 300, 'BackgroundColor', 'white');
    close(f);

    % --- Availability map (bidi) -------------------------------------
    % Swap in available_bidi as the .available view (plotAvailabilityMap
    % reads .available). Restore afterwards so subsequent plots use the
    % default (DL alias) for back-compat.
    saved_available = r.available;
    r.available     = r.available_bidi;

    f = makeFig([100 100 1100 700]);
    ax = axes('Parent', f);
    plotAvailabilityMap(r, cfg, ax);
    xlabel(ax, 'Longitude (deg)', 'FontSize', 12);
    ylabel(ax, 'Latitude (deg)',  'FontSize', 12);
    overall_bidi = 100 * mean(r.available_bidi(:));
    title(ax, sprintf('Bi-directional availability - %s  (%.0f km, %.1f%% DL\\cap UL)', ...
          name, radius, overall_bidi), 'FontSize', 13, 'FontWeight', 'normal');
    set(ax, 'FontSize', 11);
    exportgraphics(f, fullfile(fig_dir, sprintf('fig_%s_availability_map.png', name)), ...
                   'Resolution', 300, 'BackgroundColor', 'white');
    close(f);

    r.available = saved_available;

    % --- Range histogram (per region) --------------------------------
    f = makeFig([100 100 1100 600]);
    ax = axes('Parent', f);
    plotRangeHistogram(r, cfg, ax);
    set(ax, 'FontSize', 11);
    title(ax, sprintf('Availability vs distance - %s', name), ...
          'FontSize', 13, 'FontWeight', 'normal');
    exportgraphics(f, fullfile(fig_dir, sprintf('fig_%s_range_hist.png', name)), ...
                   'Resolution', 300, 'BackgroundColor', 'white');
    close(f);

    % --- CCDF (per region) -------------------------------------------
    f = makeFig([100 100 1000 600]);
    ax = axes('Parent', f);
    plotAvailabilityCCDF(r, cfg, ax);
    exportgraphics(f, fullfile(fig_dir, sprintf('fig_%s_ccdf.png', name)), ...
                   'Resolution', 300, 'BackgroundColor', 'white');
    close(f);
end

% =========================================================================
% Aggregate per terrain category
% =========================================================================
groups = {
    'suburban',     is_suburban
    'non_suburban', ~is_suburban
};

for g = 1:size(groups, 1)
    label = groups{g, 1};
    mask  = groups{g, 2};
    if ~any(mask), continue; end
    subset = results(mask);

    fprintf('[render_figures] aggregate %s (%d regions)\n', label, numel(subset));

    f = makeFig([100 100 1100 600]);
    ax = axes('Parent', f);
    plotRangeHistogram(subset, cfg, ax);
    set(ax, 'FontSize', 11);
    title(ax, sprintf('Availability vs distance - aggregate (%s, n=%d regions)', ...
          strrep(label, '_', ' '), numel(subset)), ...
          'FontSize', 13, 'FontWeight', 'normal');
    exportgraphics(f, fullfile(fig_dir, sprintf('fig_aggregate_%s_range_hist.png', label)), ...
                   'Resolution', 300, 'BackgroundColor', 'white');
    close(f);

    f = makeFig([100 100 1000 600]);
    ax = axes('Parent', f);
    plotAvailabilityCCDF(subset, cfg, ax);
    exportgraphics(f, fullfile(fig_dir, sprintf('fig_aggregate_%s_ccdf.png', label)), ...
                   'Resolution', 300, 'BackgroundColor', 'white');
    close(f);
end

% =========================================================================
% Antenna diagrams (one per CSV)
% =========================================================================
antennas_dir = fullfile(repo_root, 'resources', 'antennas');
ant_files = dir(fullfile(antennas_dir, '*.csv'));
for k = 1:numel(ant_files)
    name = erase(ant_files(k).name, ".csv");
    fprintf('[render_figures] antenna %s\n', name);
    pat = loadAntennaPattern(char(name), antennas_dir);

    f = makeFig([100 100 900 600]);
    switch pat.kind
        case 'isotropic'
            ax = axes('Parent', f);
            text(ax, 0.5, 0.5, sprintf('%s\nisotropic %.2f dBi', name, pat.flat_gain_dbi), ...
                'HorizontalAlignment', 'center', 'FontSize', 18);
            axis(ax, 'off');
        case '1d'
            ax = axes('Parent', f);
            plot(ax, rad2deg(pat.el_grid_rad), pat.gain_grid_dbi, '-o', 'LineWidth', 1.5);
            grid(ax, 'on');
            xlabel(ax, 'Elevation offset from boresight (deg)');
            ylabel(ax, 'Gain (dBi)');
            title(ax, sprintf('%s — 1D pattern', name), 'Interpreter', 'none', ...
                  'FontWeight', 'normal');
        case '2d'
            ax = axes('Parent', f);
            imagesc(ax, rad2deg(pat.el_grid_rad), rad2deg(pat.az_grid_rad), pat.gain_grid_dbi);
            set(ax, 'YDir', 'normal');
            colormap(ax, 'turbo');
            cb = colorbar(ax);
            ylabel(cb, 'Gain (dBi)');
            xlabel(ax, 'Elevation offset (deg)');
            ylabel(ax, 'Azimuth offset (deg)');
            title(ax, sprintf('%s — 2D pattern', name), 'Interpreter', 'none', ...
                  'FontWeight', 'normal');
    end
    exportgraphics(f, fullfile(fig_dir, sprintf('fig_antenna_%s.png', name)), ...
                   'Resolution', 300, 'BackgroundColor', 'white');
    close(f);
end

fprintf('[render_figures] All figures written to %s\n', fig_dir);


% -------------------------------------------------------------------------
function f = makeFig(pos)
    f = figure('Visible', 'off', 'Position', pos, 'Color', 'w');
    try
        f.Theme = 'light';
    catch
    end
    set(f, 'InvertHardcopy', 'off', 'PaperPositionMode', 'auto');
end
