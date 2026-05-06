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
%   opts (optional struct, all default true):
%     .show_inf    plot the infantry-only curve
%     .show_veh    plot the vehicular-only curve
%     .show_total  plot the combined curve

    arguments
        results
        cfg %#ok<INUSA>
        target_axes
        opts.show_inf   (1,1) logical = true
        opts.show_veh   (1,1) logical = true
        opts.show_total (1,1) logical = true
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
        drawCCDF(ax, d_km,         av,         'total',     [0.20 0.20 0.20], 1.6);
    end
    if opts.show_inf && any(is_inf)
        drawCCDF(ax, d_km(is_inf), av(is_inf), 'infantry',  [0.95 0.55 0.10], 1.4);
    end
    if opts.show_veh && any(~is_inf)
        drawCCDF(ax, d_km(~is_inf), av(~is_inf), 'vehicular', [0.20 0.55 0.85], 1.4);
    end

    hold(ax, 'off');
    grid(ax, 'on');
    xlabel(ax, 'Distance from centroid d (km)');
    ylabel(ax, 'P(available | range \geq d)');
    ylim(ax, [0 1]);
    title(ax, 'Availability CCDF vs distance from centroid');
    legend(ax, 'Location', 'best');
end


% -------------------------------------------------------------------------
function drawCCDF(ax, d_km, av, label, color, lw)
    if isempty(d_km), return; end
    [ds, idx] = sort(d_km(:));
    avs = double(av(idx));
    N = numel(avs);
    % Reverse-cumulative availability rate: at each d=ds(i), fraction of
    % samples in ds(i:end) that are available.
    rev_count_avail = flipud(cumsum(flipud(avs)));
    denom = (N:-1:1).';
    rate = rev_count_avail ./ denom;
    stairs(ax, ds, rate, '-', 'Color', color, 'LineWidth', lw, ...
           'DisplayName', label);
end


function [d_km, av, is_inf] = flattenAll(results)
    d_km   = [];
    av     = false(0, 1);
    is_inf = false(0, 1);
    for r = 1:numel(results)
        res = results(r);
        if isempty(res.available), continue; end
        M = size(res.available, 1);
        n_rx = size(res.available, 2);

        d_r  = double(reshape(res.dists_m, [], 1)) / 1000;
        av_r = reshape(res.available,  [], 1);

        is_inf_r = false(numel(av_r), 1);
        for m = 1:M
            col = res.rx_types{m};
            if iscategorical(col)
                row_mask = (col == 'infantry');
            else
                row_mask = strcmp(string(col), "infantry");
            end
            idx_r = (m-1)*n_rx + (1:n_rx);
            is_inf_r(idx_r) = row_mask(:);
        end

        d_km   = [d_km;   d_r];   %#ok<AGROW>
        av     = [av;     av_r];  %#ok<AGROW>
        is_inf = [is_inf; is_inf_r]; %#ok<AGROW>
    end
end
