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
%   Includes 95% Wilson score confidence bands. Tail bins where the
%   denominator falls below cfg.ccdf.min_samples are masked to NaN to
%   suppress noisy far-range artefacts.
%
%   opts (optional struct, all default true):
%     .show_inf    plot the infantry-only curve
%     .show_veh    plot the vehicular-only curve
%     .show_total  plot the combined curve

    arguments
        results
        cfg
        target_axes
        opts.show_inf   (1,1) logical = true
        opts.show_veh   (1,1) logical = true
        opts.show_total (1,1) logical = true
    end

    % Fallback if ccdf config block is absent (backward compat)
    if isfield(cfg, 'ccdf') && isfield(cfg.ccdf, 'min_samples')
        min_samples = cfg.ccdf.min_samples;
    else
        min_samples = 20;
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

    if opts.show_total
        drawCCDF(ax, d_km,          av,          'total',    [0.20 0.20 0.20], 2.0, min_samples);
    end
    if opts.show_inf && any(is_inf)
        drawCCDF(ax, d_km(is_inf),  av(is_inf),  'infantry', [0.93 0.69 0.13], 1.5, min_samples);
    end
    if opts.show_veh && any(~is_inf)
        drawCCDF(ax, d_km(~is_inf), av(~is_inf), 'vehicular',[0.30 0.60 0.90], 1.5, min_samples);
    end

    hold(ax, 'off');
    grid(ax, 'on');
    ax.GridAlpha = 0.3;
    ax.Box       = 'off';
    xlabel(ax, 'Distance from centroid d (km)');
    ylabel(ax, 'P(available | range \geq d)');
    ylim(ax, [0, 1.05]);
    xlim(ax, [0, max(d_km)]);
    title(ax, 'Availability CCDF vs distance from centroid');
    legend(ax, 'Location', 'southwest');
end


% -------------------------------------------------------------------------
function drawCCDF(ax, d_km, av, label, color, lw, min_samples)
    if isempty(d_km), return; end
    [ds, idx] = sort(d_km(:));
    avs = double(av(idx));
    N   = numel(avs);

    % Reverse-cumulative availability rate: at each d=ds(i), fraction of
    % samples in ds(i:end) that are available.
    k = flipud(cumsum(flipud(avs)));
    n = (N:-1:1).';

    rate = k ./ n;

    % 95% Wilson score interval — well-behaved at small n and extreme rates
    z      = 1.96;
    denom  = 1 + z^2 ./ n;
    centre = (rate + z^2 ./ (2*n)) ./ denom;
    margin = z * sqrt(rate .* (1 - rate) ./ n + z^2 ./ (4 * n.^2)) ./ denom;
    lo = max(0, centre - margin);
    hi = min(1, centre + margin);

    % Mask tail where too few samples remain for reliable estimation
    sparse = n < min_samples;
    rate(sparse) = NaN;
    lo(sparse)   = NaN;
    hi(sparse)   = NaN;

    % Shaded Wilson confidence band (drawn first so line sits on top)
    valid = ~sparse;
    if any(valid)
        ds_v = ds(valid);
        fill(ax, [ds_v; flipud(ds_v)], [lo(valid); flipud(hi(valid))], color, ...
             'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    end

    plot(ax, ds, rate, '-', 'Color', color, 'LineWidth', lw, 'DisplayName', label);
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
