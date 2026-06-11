function plotAvailabilityCCDF(ax, series, ~)
%PLOTAVAILABILITYCCDF  Complementary cumulative availability vs distance.
%
%   plotAvailabilityCCDF(ax, series)
%
%   For each series, plots availability among the devices at range >= d, as d
%   sweeps outward: y(d) = mean(available for devices with dist >= d). At d=0
%   this is the overall availability; it shows how coverage holds up as you
%   require devices farther out to be served.

    hold(ax, 'on');
    for s = series
        d  = s.dist_m(:);
        av = double(s.available(:));
        [d, ord] = sort(d);
        av = av(ord);
        n  = numel(d);
        if n == 0, continue; end
        sufsum = flipud(cumsum(flipud(av)));     % sum(av(i:n))
        cnt    = (n:-1:1).';
        rc     = sufsum ./ cnt;                  % mean(av(i:n))
        plot(ax, d, 100*rc, 'LineWidth', 1.5, 'DisplayName', s.label);
    end
    xlabel(ax, 'range d (m)');
    ylabel(ax, 'availability for devices at range \geq d (%)');
    ylim(ax, [0 101]); grid(ax, 'on');
    legend(ax, 'show', 'Location', 'best');
    title(ax, 'Availability CCDF vs distance');
end
