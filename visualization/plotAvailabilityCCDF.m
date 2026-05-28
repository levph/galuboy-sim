function plotAvailabilityCCDF(results, cfg, target_axes, opts)
%PLOTAVAILABILITYCCDF  Availability vs distance-from-centroid CCDF, two aggregates.
%
%   plotAvailabilityCCDF(results, cfg, target_axes)
%   plotAvailabilityCCDF(results, cfg, target_axes, opts)
%
%   For each (rotation x RX) sample we have a distance-from-centroid and
%   an availability flag for the selected link direction. The plot shows
%   y(d) = P(available | dist >= d) - reads like a CCDF of coverage.
%
%   Regions are pooled into two groups by region.category:
%     - urban_suburban  (orange)
%     - other            (blue)
%   For each group, up to three curves are drawn (toggled by opts):
%     - total      solid line, no marker
%     - infantry   line with circle markers
%     - vehicular  line with square markers
%
%   opts (optional struct or name-value):
%     .show_inf   (default true)
%     .show_veh   (default true)
%     .show_total (default true)
%     .link       'dl' (default) | 'ul' | 'bidi'
%     .no_title   (default false)

    arguments
        results
        cfg
        target_axes
        opts.show_inf   (1,1) logical = true
        opts.show_veh   (1,1) logical = true
        opts.show_total (1,1) logical = true
        opts.no_title   (1,1) logical = false
        opts.link       string  = "dl"
    end

    link = lower(string(opts.link));
    if ~ismember(link, ["dl", "ul", "bidi"])
        error('plotAvailabilityCCDF:badLink', ...
              'link must be one of dl/ul/bidi, got "%s".', opts.link);
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

    [d_km, av, is_inf, is_urban] = flattenAll(results, link);
    if isempty(d_km)
        title(ax, 'No data');
        hold(ax, 'off');
        return;
    end

    d_max = max(d_km);

    % Color = group; marker = type
    colors = struct('urban_suburban', [0.84 0.39 0.18], ...
                    'other',          [0.12 0.47 0.71]);
    groups = {'urban_suburban', 'other'};
    group_mask = struct('urban_suburban', is_urban, 'other', ~is_urban);

    for gk = 1:numel(groups)
        gname = groups{gk};
        gmask = group_mask.(gname);
        if ~any(gmask), continue; end
        gcolor = colors.(gname);

        if opts.show_total
            drawCCDF(ax, d_km(gmask), av(gmask), ...
                     sprintf('%s total', gname), ...
                     gcolor, 2.5, 'none', min_samples, bin_km, d_max);
        end
        if opts.show_inf
            m = gmask & is_inf;
            if any(m)
                drawCCDF(ax, d_km(m), av(m), ...
                         sprintf('%s infantry', gname), ...
                         gcolor, 1.6, 'o', min_samples, bin_km, d_max);
            end
        end
        if opts.show_veh
            m = gmask & ~is_inf;
            if any(m)
                drawCCDF(ax, d_km(m), av(m), ...
                         sprintf('%s vehicular', gname), ...
                         gcolor, 1.6, 's', min_samples, bin_km, d_max);
            end
        end
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
        title(ax, sprintf('Availability CCDF [%s] - urban_suburban vs other', ...
              linkDisplayName(link)), ...
              'FontSize', 13, 'FontWeight', 'normal', 'Interpreter', 'none');
    end
    lg = legend(ax, 'Location', 'best', 'Interpreter', 'none');
    lg.Box      = 'off';
    lg.FontSize = 10;
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

    % Reverse-cumulative: n(d_i) = total samples with d >= d_i, k = available among those
    n = flipud(cumsum(flipud(counts)));
    k = flipud(cumsum(flipud(avail_per_bin)));

    rate = k ./ max(n, 1);

    % Mask sparse tail bins
    rate(n < min_samples) = NaN;

    x = centres(:);

    plot(ax, x, rate, '-', 'Color', color, 'LineWidth', lw, ...
         'Marker', marker, 'MarkerFaceColor', color, 'MarkerEdgeColor', color, ...
         'MarkerSize', 5, 'DisplayName', label);
end


% -------------------------------------------------------------------------
function [d_km, av, is_inf, is_urban] = flattenAll(results, link)
    d_km     = [];
    av       = false(0, 1);
    is_inf   = false(0, 1);
    is_urban = false(0, 1);
    for r = 1:numel(results)
        res = results(r);
        av_field = pickAvailField(res, link);
        if isempty(av_field), continue; end
        M = size(av_field, 1);

        d_r  = double(reshape(res.dists_m, [], 1)) / 1000;
        av_r = reshape(av_field, [], 1);

        % placeReceivers always emits rx_types in the order
        % [infantry x n_inf; vehicular x n_veh] - constant across rotations.
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
        is_inf_r = reshape(repmat(is_inf_per_rx, M, 1), [], 1);

        is_urb_region = regionIsUrban(res.region);
        is_urban_r    = true(numel(d_r), 1) & is_urb_region;

        d_km     = [d_km;     d_r];        %#ok<AGROW>
        av       = [av;       av_r];       %#ok<AGROW>
        is_inf   = [is_inf;   is_inf_r];   %#ok<AGROW>
        is_urban = [is_urban; is_urban_r]; %#ok<AGROW>
    end
end


function v = pickAvailField(res, link)
    switch link
        case "dl"
            if isfield(res, 'available_dl'),   v = res.available_dl;
            else,                              v = res.available;
            end
        case "ul"
            if isfield(res, 'available_ul'),   v = res.available_ul;
            else,                              v = [];
            end
        case "bidi"
            if isfield(res, 'available_bidi'), v = res.available_bidi;
            else,                              v = [];
            end
    end
end


function tf = regionIsUrban(region)
    if isstruct(region) && isfield(region, 'category') && ~isempty(region.category)
        tf = strcmp(region.category, 'urban_suburban');
    elseif isstruct(region) && isfield(region, 'name')
        tf = startsWith(string(region.name), "urban_suburban");
    else
        tf = false;
    end
end


function s = linkDisplayName(link)
    switch link
        case "dl",   s = 'DL';
        case "ul",   s = 'UL';
        case "bidi", s = 'Bidi';
    end
end
