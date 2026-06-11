function plotAvailabilityHistogram(ax, series, opts)
%PLOTAVAILABILITYHISTOGRAM  Availability fraction vs distance, in fixed bins.
%
%   plotAvailabilityHistogram(ax, series, opts)
%
%   For each series (a link x terrain subset from buildSeries) plots the
%   fraction of devices that are available within each distance bin.
%
%   opts (struct, optional): bin_m (default 100), min_samples (default 5).

    if nargin < 3, opts = struct(); end
    bin_m = getdef(opts, 'bin_m', 100);
    min_n = getdef(opts, 'min_samples', 5);

    hold(ax, 'on');
    edges = binEdges(series, bin_m);
    centers = (edges(1:end-1) + edges(2:end)) / 2;
    for s = series
        [~, av, cnt] = distanceBinStats(s.dist_m, double(s.available), edges, @mean);
        av(cnt < min_n) = NaN;
        plot(ax, centers, 100*av, '-o', 'LineWidth', 1.3, 'DisplayName', s.label);
    end
    xlabel(ax, 'distance from centre (m)');
    ylabel(ax, 'availability (%)');
    ylim(ax, [0 101]); grid(ax, 'on');
    legend(ax, 'show', 'Location', 'best');
    title(ax, sprintf('Availability vs distance (%d m bins)', bin_m));
end
