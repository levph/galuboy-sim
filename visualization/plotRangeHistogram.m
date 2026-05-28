function plotRangeHistogram(results, cfg, target_axes, opts)
%PLOTRANGEHISTOGRAM  Availability vs distance-from-centroid, per region + 2 aggregates.
%
%   plotRangeHistogram(results, cfg)
%   plotRangeHistogram(results, cfg, target_axes)
%   plotRangeHistogram(results, cfg, target_axes, 'link', 'dl'|'ul'|'bidi')
%
%   results      struct array from runScenario
%   cfg          full config struct
%   target_axes  optional axes handle (UI use). When given, draws two
%                aggregate availability-vs-distance curves into it
%                (urban_suburban group + 'other' group) instead of opening
%                a fresh figure.
%   opts.link    'dl' (default) | 'ul' | 'bidi' - which link-direction
%                availability to plot.
%
%   Aggregation: regions are grouped by region.category ('urban_suburban'
%   vs anything else => 'other'). Sample pools are pooled within each
%   group; bin edges and titles reflect each pool independently.

    arguments
        results
        cfg
        target_axes      = []
        opts.link string = "dl"
    end

    n_regions = numel(results);
    link = lower(string(opts.link));
    if ~ismember(link, ["dl", "ul", "bidi"])
        error('plotRangeHistogram:badLink', ...
              'link must be one of dl/ul/bidi, got "%s".', opts.link);
    end

    % Per-region availability vector (chosen link) and distance vector.
    per_d = cell(n_regions, 1);
    per_a = cell(n_regions, 1);
    per_cat = strings(n_regions, 1);
    for r = 1:n_regions
        per_d{r}   = results(r).dists_m(:);
        % pickAvail returns an [M x n_rx] matrix - flatten so mean/numel
        % reduce to scalars (sprintf-friendly) and poolGroup vertcat works.
        per_a{r}   = logical(reshape(pickAvail(results(r), link), [], 1));
        per_cat(r) = regionCategory(results(r).region);
    end

    % Two aggregate pools by category
    is_urban = per_cat == "urban_suburban";
    [urb_d, urb_a] = poolGroup(per_d(is_urban), per_a(is_urban));
    [oth_d, oth_a] = poolGroup(per_d(~is_urban), per_a(~is_urban));

    link_label = linkDisplayName(link);

    if ~isempty(target_axes)
        cla(target_axes);
        plotTwoAggregateLines(target_axes, urb_d, urb_a, oth_d, oth_a, cfg, link_label);
        return;
    end

    fig = figure('Name', sprintf('Range Analysis - %s', link_label), ...
                 'Position', [100, 100, 1100, 600]);

    if n_regions == 1
        ax = axes('Parent', fig);
        plotOne(ax, per_d{1}, per_a{1}, cfg, results(1).region.name, link_label);
        return;
    end

    % Multi-region: tiled layout. n_regions tiles + up to 2 aggregate tiles
    % (drop a group's aggregate tile if it has no member regions).
    agg_tiles = double(any(is_urban)) + double(any(~is_urban));
    n_tiles = n_regions + agg_tiles;
    tcols = min(3, n_tiles);
    trows = ceil(n_tiles / tcols);
    tl = tiledlayout(fig, trows, tcols, 'Padding', 'compact', 'TileSpacing', 'compact');
    title(tl, sprintf(['Range analysis [%s]  |  P_{air}=%.0f dBm / P_{gnd}=%.0f dBm  |  ' ...
        'MDS=%.0f dBm  |  fade=%.0f dB  |  avail\\geq%g%%  |  M=%d'], ...
        link_label, ...
        cfg.tx.power_air_dbm, cfg.tx.power_gnd_dbm, ...
        cfg.analysis.threshold_dbm, ...
        cfg.analysis.fade_margin_db, cfg.analysis.percentile, ...
        cfg.flight.num_rotations));

    for r = 1:n_regions
        ax = nexttile(tl);
        plotOne(ax, per_d{r}, per_a{r}, cfg, results(r).region.name, link_label);
    end
    if any(is_urban)
        ax = nexttile(tl);
        plotOne(ax, urb_d, urb_a, cfg, 'aggregate: urban_suburban', link_label);
    end
    if any(~is_urban)
        ax = nexttile(tl);
        plotOne(ax, oth_d, oth_a, cfg, 'aggregate: other', link_label);
    end
end


% -------------------------------------------------------------------------
function plotTwoAggregateLines(ax, d_urb, a_urb, d_oth, a_oth, cfg, link_label)
    cla(ax);
    hold(ax, 'on');
    n_bins = cfg.hist.n_bins;

    [bx_u, by_u, n_u] = computeRangeFrac(d_urb, a_urb, n_bins, cfg.hist.min_samples_per_bin);
    [bx_o, by_o, n_o] = computeRangeFrac(d_oth, a_oth, n_bins, cfg.hist.min_samples_per_bin);

    has_urb = ~isempty(bx_u);
    has_oth = ~isempty(bx_o);

    if has_urb
        plot(ax, bx_u, by_u, '-o', 'Color', [0.84 0.39 0.18], ...
             'LineWidth', 1.8, 'MarkerFaceColor', [0.84 0.39 0.18], ...
             'MarkerSize', 4, ...
             'DisplayName', sprintf('urban_suburban (overall %.1f%%, n=%d)', ...
                                    100 * mean(a_urb), numel(a_urb)));
    end
    if has_oth
        plot(ax, bx_o, by_o, '-s', 'Color', [0.12 0.47 0.71], ...
             'LineWidth', 1.8, 'MarkerFaceColor', [0.12 0.47 0.71], ...
             'MarkerSize', 4, ...
             'DisplayName', sprintf('other (overall %.1f%%, n=%d)', ...
                                    100 * mean(a_oth), numel(a_oth)));
    end

    yline(ax, 50, '--r', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    hold(ax, 'off');

    xlabel(ax, 'Distance from centroid (m)');
    ylabel(ax, sprintf('Availability (%%)  [fraction(P_{rx} > MDS) \\geq %g%%]', ...
        cfg.analysis.percentile));
    title(ax, sprintf('Range histogram [%s] - urban_suburban vs other (%d / %d samples)', ...
        link_label, sum(n_u), sum(n_o)));
    ylim(ax, [0, 110]);
    all_x = [bx_u(:); bx_o(:)];
    if ~isempty(all_x)
        xlim(ax, [0, max(all_x) * 1.02]);
    end
    grid(ax, 'on');
    ax.GridAlpha = 0.3;
    ax.Box = 'off';
    if has_urb || has_oth
        legend(ax, 'Location', 'best', 'Interpreter', 'none');
    end
end


% -------------------------------------------------------------------------
function plotOne(ax, dists, avail, cfg, label, link_label)
    n_bins = cfg.hist.n_bins;
    avail  = logical(avail);
    dists  = double(dists);
    if isempty(dists) || all(~isfinite(dists))
        title(ax, sprintf('%s (no data)', label), 'Interpreter', 'none');
        return;
    end

    [bin_centers, frac_pct, ~] = computeRangeFrac(dists, avail, n_bins, cfg.hist.min_samples_per_bin);

    b = bar(ax, bin_centers, frac_pct, 1.0);
    b.FaceColor = [0.20, 0.45, 0.75];
    b.EdgeColor = 'none';
    b.FaceAlpha = 0.85;

    hold(ax, 'on');
    yline(ax, 50, '--r', 'LineWidth', 1.0);
    hold(ax, 'off');

    xlabel(ax, 'Distance from centroid (m)');
    ylabel(ax, sprintf('Availability (%%)  [fraction(P_{rx} > MDS) \\geq %g%%]', cfg.analysis.percentile));
    overall = 100 * mean(avail);
    title(ax, sprintf('%s [%s]  (overall %.1f%%, n=%d)', label, link_label, overall, numel(avail)), ...
        'Interpreter', 'none');
    ylim(ax, [0, 110]);
    xlim(ax, [min(dists), max(dists)]);
    grid(ax, 'on');
    ax.GridAlpha = 0.3;
    ax.Box = 'off';
end


% -------------------------------------------------------------------------
function [bin_centers, frac_pct, n_total] = computeRangeFrac(dists, avail, n_bins, min_per_bin)
    if isempty(dists)
        bin_centers = []; frac_pct = []; n_total = [];
        return;
    end
    dists = double(dists);
    avail = logical(avail);
    dist_edges  = linspace(min(dists), max(dists), n_bins + 1);
    n_total     = histcounts(dists,            dist_edges);
    n_available = histcounts(dists(avail),     dist_edges);
    frac = n_available ./ max(n_total, 1);
    frac(n_total < min_per_bin) = NaN;
    bin_centers = 0.5 * (dist_edges(1:end-1) + dist_edges(2:end));
    frac_pct    = frac * 100;
end


% -------------------------------------------------------------------------
function v = pickAvail(res, link)
    switch link
        case "dl",   v = res.available_dl;
        case "ul",   v = res.available_ul;
        case "bidi", v = res.available_bidi;
    end
end


function s = linkDisplayName(link)
    switch link
        case "dl",   s = 'DL';
        case "ul",   s = 'UL';
        case "bidi", s = 'Bidi';
    end
end


function cat_str = regionCategory(region)
    if isstruct(region) && isfield(region, 'category') && ~isempty(region.category)
        cat_str = string(region.category);
    elseif isstruct(region) && isfield(region, 'name') && startsWith(string(region.name), "urban_suburban")
        cat_str = "urban_suburban";
    else
        cat_str = "other";
    end
end


function [d, a] = poolGroup(d_cells, a_cells)
    d = zeros(0, 1, 'single');
    a = false(0, 1);
    for k = 1:numel(d_cells)
        d = [d; d_cells{k}]; %#ok<AGROW>
        a = [a; a_cells{k}]; %#ok<AGROW>
    end
end
