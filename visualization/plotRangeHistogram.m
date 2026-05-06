function plotRangeHistogram(results, cfg, target_axes)
%PLOTRANGEHISTOGRAM  Availability vs distance-from-centroid, per region + aggregate.
%
%   plotRangeHistogram(results, cfg)
%   plotRangeHistogram(results, cfg, target_axes)
%
%   results      struct array from runScenario
%   cfg          full config struct
%   target_axes  optional axes handle (UI use). When given, draws into it
%                instead of creating a new figure. With multiple regions
%                and a single target axes, plots only the aggregate.
%
%   When called without target_axes:
%     - 1 region: single bar chart in a fresh figure.
%     - 2+ regions: tiledlayout with one tile per region plus an aggregate.

    arguments
        results
        cfg
        target_axes = []
    end

    n_regions = numel(results);

    % Aggregate sample pool (used for the single-region case AND the
    % aggregate panel when there are multiple regions). Pre-seed all_a as
    % logical so vertcat doesn't promote it to double - downstream
    % indexing dists(avail) needs logical indexing, not numeric.
    all_d = zeros(0, 1, 'single');
    all_a = false(0, 1);
    for r = 1:n_regions
        all_d = [all_d; results(r).dists_m(:)];                  %#ok<AGROW>
        all_a = [all_a; logical(results(r).available(:))];       %#ok<AGROW>
    end

    if ~isempty(target_axes)
        cla(target_axes);
        plotOne(target_axes, all_d, all_a, cfg, 'aggregate');
        return;
    end

    fig = figure('Name', 'Range Analysis - Availability vs Distance', ...
                 'Position', [100, 100, 1100, 600]);

    if n_regions == 1
        ax = axes('Parent', fig);
        plotOne(ax, all_d, all_a, cfg, results(1).region.name);
        return;
    end

    % Multi-region: tiled layout
    n_tiles = n_regions + 1;
    tcols = min(3, n_tiles);
    trows = ceil(n_tiles / tcols);
    tl = tiledlayout(fig, trows, tcols, 'Padding', 'compact', 'TileSpacing', 'compact');
    title(tl, sprintf(['Range analysis  |  P_{tx}=%.0f dBm  |  MDS=%.0f dBm  |  ' ...
        'fade=%.0f dB  |  p%d  |  M=%d'], ...
        cfg.tx.power_dbm, cfg.analysis.threshold_dbm, ...
        cfg.analysis.fade_margin_db, cfg.analysis.percentile, ...
        cfg.flight.num_rotations));

    for r = 1:n_regions
        ax = nexttile(tl);
        plotOne(ax, results(r).dists_m(:), results(r).available(:), cfg, results(r).region.name);
    end
    ax = nexttile(tl);
    plotOne(ax, all_d, all_a, cfg, 'aggregate');
end


% -------------------------------------------------------------------------
function plotOne(ax, dists, avail, cfg, label)
    n_bins = cfg.hist.n_bins;
    avail = logical(avail);   % defensive: dists(avail) needs logical index
    dists = double(dists);
    if isempty(dists) || all(~isfinite(dists))
        title(ax, sprintf('%s (no data)', label));
        return;
    end

    dist_edges  = linspace(min(dists), max(dists), n_bins + 1);
    n_total     = histcounts(dists,            dist_edges);
    n_available = histcounts(dists(avail),     dist_edges);

    frac = n_available ./ max(n_total, 1);
    insufficient = n_total < cfg.hist.min_samples_per_bin;
    frac(insufficient) = NaN;

    bin_centers = 0.5 * (dist_edges(1:end-1) + dist_edges(2:end));

    b = bar(ax, bin_centers, frac * 100, 1.0);
    b.FaceColor = [0.20, 0.45, 0.75];
    b.EdgeColor = 'none';
    b.FaceAlpha = 0.85;

    hold(ax, 'on');
    yline(ax, 50, '--r', 'LineWidth', 1.0);
    hold(ax, 'off');

    xlabel(ax, 'Distance from centroid (m)');
    ylabel(ax, sprintf('Availability (%%)  [P_{rx}(p%d) > MDS]', cfg.analysis.percentile));
    overall = 100 * mean(avail);
    title(ax, sprintf('%s  (overall %.1f%%, n=%d)', label, overall, numel(avail)));
    ylim(ax, [0, 110]);
    xlim(ax, [dist_edges(1), dist_edges(end)]);
    grid(ax, 'on');
    ax.GridAlpha = 0.3;
    ax.Box = 'off';
end
