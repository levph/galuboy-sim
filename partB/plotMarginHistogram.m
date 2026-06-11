function plotMarginHistogram(ax, series, opts)
%PLOTMARGINHISTOGRAM  Distribution of margin-from-required-SNR vs distance.
%
%   plotMarginHistogram(ax, series, opts)
%
%   For each series, per distance bin, plots the MEDIAN device margin (dB above
%   required SNR) as a solid line and the 10th/90th-percentile spread as dashed
%   lines of the same colour. The spread shows how much headroom/uncertainty
%   there is - whether devices clear their requirement comfortably or sit right
%   at the edge. margin = pSNR - req_snr (per device, from computeLinkAvailability).
%
%   opts (struct, optional): bin_m (default 100), min_samples (default 5),
%                            pct (default [10 50 90]).

    if nargin < 3, opts = struct(); end
    bin_m = getdef(opts, 'bin_m', 100);
    min_n = getdef(opts, 'min_samples', 5);
    pct   = getdef(opts, 'pct', [10 50 90]);

    hold(ax, 'on');
    edges = binEdges(series, bin_m);
    centers = (edges(1:end-1) + edges(2:end)) / 2;
    for s = series
        m = s.margin_db(:);
        m(~isfinite(m)) = NaN;                  % drop outage devices (pSNR=-Inf) from the
                                                %   margin band -> prctile ignores NaN, no holes.
                                                %   (outage is shown by the availability plot)
        [~, q, cnt] = distanceBinStats(s.dist_m, m, edges, @(v) prctile(v, pct));
        q(cnt < min_n, :) = NaN;
        hMed = plot(ax, centers, q(:,2), '-', 'LineWidth', 1.6, 'DisplayName', s.label);
        col  = get(hMed, 'Color');
        plot(ax, centers, q(:,1), '--', 'Color', col, 'HandleVisibility', 'off');
        plot(ax, centers, q(:,3), '--', 'Color', col, 'HandleVisibility', 'off');
    end
    yline(ax, 0, ':k', 'required SNR', 'HandleVisibility', 'off');
    xlabel(ax, 'distance from centre (m)');
    ylabel(ax, 'margin above required SNR (dB)');
    grid(ax, 'on');
    legend(ax, 'show', 'Location', 'best');
    title(ax, sprintf('Margin (median + %g/%g pct) vs distance', pct(1), pct(end)));
end
