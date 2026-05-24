function plotAvailabilityCCDF(results, cfg, target_axes, opts)
%PLOTAVAILABILITYCCDF  Availability vs distance-from-centroid CCDF.
%
%   plotAvailabilityCCDF(results, cfg, target_axes)
%   plotAvailabilityCCDF(results, cfg, target_axes, opts)
%
%   For each (rotation x RX) sample we have a distance-from-centroid and
%   an availability flag. This plot shows, for each distance d on the
%   x-axis, the *availability rate among receivers at range >= d*. Reads
%   like a CCDF of coverage vs range: y(d) = P(available | dist >= d).
%
%   Distances are aggregated into fixed-width bins (cfg.ccdf.bin_km,
%   default 0.2 km) so the curve has stable visual resolution and the
%   Wilson 95% confidence bands are evaluated at well-defined sample
%   counts. Tail bins with fewer than cfg.ccdf.min_samples receivers
%   in the d_bin..d_max range are masked to NaN.
%
%   opts (optional struct):
%     .show_inf   (default true)  infantry-only curve
%     .show_veh   (default true)  vehicular-only curve
%     .show_total (default true)  combined curve
%     .no_title   (default false) suppress the figure title (for export)

    arguments
        results
        cfg
        target_axes
        opts.show_inf   (1,1) logical = true
        opts.show_veh   (1,1) logical = true
        opts.show_total (1,1) logical = true
        opts.no_title   (1,1) logical = false
    end

    if isfield(cfg, 'ccdf') && isfield(cfg.ccdf, 'min_samples')
        min_samples = cfg.ccdf.min_samples;
    else
        min_samples = 20;
    end
    if isfield(cfg, 'ccdf') && isfield(cfg.ccdf, 'bin_km')
        bin_km = cfg.ccdf.bin_km;
    else
        bin_km = 0.2;
    end

    ax = target_axes;
    cla(ax);
    hold(ax, 'on');

    [d_km, av, is_inf] = flattenAll(results);
    if isempty(d_km)
        title(ax, 'No data');
        hold(ax, 'off');
        return;
    end

    d_max = max(d_km);

    if opts.show_total
        drawCCDF(ax, d_km,          av,          'total', ...
                 [0.10 0.10 0.10], 2.5, 'none', min_samples, bin_km, d_max);
    end
    if opts.show_inf && any(is_inf)
        drawCCDF(ax, d_km(is_inf),  av(is_inf),  'infantry', ...
                 [0.84 0.39 0.18], 1.8, 'o',    min_samples, bin_km, d_max);
    end
    if opts.show_veh && any(~is_inf)
        drawCCDF(ax, d_km(~is_inf), av(~is_inf), 'vehicular', ...
                 [0.12 0.47 0.71], 1.8, 's',    min_samples, bin_km, d_max);
    end

    hold(ax, 'off');
    grid(ax, 'on');
    ax.GridAlpha           = 0.25;
    ax.Box                 = 'off';
    ax.FontSize            = 11;
    ax.LineWidth           = 1.0;
    xlabel(ax, 'Distance from centroid d (km)', 'FontSize', 12);
    ylabel(ax, 'P(available | range \geq d)',   'FontSize', 12);
    ylim(ax, [0, 1.05]);
    xlim(ax, [0, d_max]);
    if ~opts.no_title
        title(ax, 'Availability CCDF vs distance from centroid', ...
              'FontSize', 13, 'FontWeight', 'normal');
    end
    lg = legend(ax, 'Location', 'best');
    lg.Box      = 'off';
    lg.FontSize = 11;
end


% -------------------------------------------------------------------------
function drawCCDF(ax, d_km, av, label, color, lw, marker, min_samples, bin_km, d_max)
    if isempty(d_km), return; end

    % Bin edges spaced by bin_km from 0 up to d_max (inclusive last edge).
    edges = 0:bin_km:(d_max + bin_km);
    if edges(end) < d_max + 0.5*bin_km
        edges(end+1) = edges(end) + bin_km;
    end
    centres = edges(1:end-1) + 0.5*bin_km;
    nbins   = numel(centres);

    avs = double(av(:));
    [counts, ~, bin_idx] = histcounts(d_km(:), edges);
    avail_per_bin = accumarray(bin_idx(bin_idx > 0), avs(bin_idx > 0), [nbins, 1], @sum, 0);
    counts        = counts(:);

    % Reverse-cumulative aggregation: n(d_i) = total samples with d >= d_i,
    % k(d_i) = number of those that are available.
    n = flipud(cumsum(flipud(counts)));
    k = flipud(cumsum(flipud(avail_per_bin)));

    rate = k ./ max(n, 1);

    % 95% Wilson score interval — well-behaved at small n and extreme rates
    z      = 1.96;
    denom  = 1 + z^2 ./ n;
    centre = (rate + z^2 ./ (2*n)) ./ denom;
    margin = z * sqrt(rate .* (1 - rate) ./ n + z^2 ./ (4 * n.^2)) ./ denom;
    lo = max(0, centre - margin);
    hi = min(1, centre + margin);

    sparse = n < min_samples;
    rate(sparse) = NaN;
    lo(sparse)   = NaN;
    hi(sparse)   = NaN;

    x = centres(:);

    valid = ~sparse;
    if any(valid)
        x_v  = x(valid);
        lo_v = lo(valid);
        hi_v = hi(valid);
        fill(ax, [x_v; flipud(x_v)], [lo_v; flipud(hi_v)], color, ...
             'FaceAlpha', 0.18, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    end

    plot(ax, x, rate, '-', 'Color', color, 'LineWidth', lw, ...
         'Marker', marker, 'MarkerFaceColor', color, 'MarkerEdgeColor', color, ...
         'MarkerSize', 5, 'DisplayName', label);
end


% -------------------------------------------------------------------------
function [d_km, av, is_inf] = flattenAll(results)
    d_km   = [];
    av     = false(0, 1);
    is_inf = false(0, 1);
    for r = 1:numel(results)
        res = results(r);
        if isempty(res.available), continue; end
        M = size(res.available, 1);

        d_r  = double(reshape(res.dists_m,  [], 1)) / 1000;
        av_r = reshape(res.available, [], 1);

        % placeReceivers always emits rx_types in the order
        % [infantry x n_inf; vehicular x n_veh], so the per-rx type column
        % is constant across rotations. reshape() above is COLUMN-MAJOR,
        % so to align is_inf with the flattened vector, we replicate the
        % per-rx mask across M rows then column-major flatten.
        col1 = res.rx_types{1};
        if iscategorical(col1)
            is_inf_per_rx = (col1 == 'infantry');
        else
            is_inf_per_rx = strcmp(string(col1), "infantry");
        end
        is_inf_per_rx = is_inf_per_rx(:).';                       % [1 x n_rx]
        if M >= 2 && ~isequal(res.rx_types{2}, col1)
            warning('plotAvailabilityCCDF:rxTypeDrift', ...
                'rx_types differ between rotations - mask assumes constant order.');
        end
        is_inf_r = reshape(repmat(is_inf_per_rx, M, 1), [], 1);   % column-major flatten

        d_km   = [d_km;   d_r];       %#ok<AGROW>
        av     = [av;     av_r];      %#ok<AGROW>
        is_inf = [is_inf; is_inf_r];  %#ok<AGROW>
    end
end
