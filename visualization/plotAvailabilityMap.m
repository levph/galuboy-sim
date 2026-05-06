function plotAvailabilityMap(results, cfg, target_axes)
%PLOTAVAILABILITYMAP  Map of receiver positions colored by availability.
%
%   plotAvailabilityMap(results, cfg)
%   plotAvailabilityMap(results, cfg, target_axes)
%
%   Each receiver placement (one per rotation per RX) is rendered as a
%   point at its (lon, lat). Color encodes availability (red = below
%   threshold, green = above). Marker shape encodes RX type:
%     - circle ('o') = infantry
%     - square ('s') = vehicular
%
%   results      struct array from runScenario
%   cfg          full config struct (used for title/threshold context)
%   target_axes  optional axes handle (UI use). When given, plots all
%                regions into that one axes.

    arguments
        results
        cfg %#ok<INUSA>
        target_axes = []
    end

    n_regions = numel(results);

    if ~isempty(target_axes)
        cla(target_axes);
        plotAllRegionsInto(target_axes, results);
        return;
    end

    fig = figure('Name', 'Availability Map', ...
                 'Position', [120, 120, 1100, 700]);

    if n_regions == 1
        ax = axes('Parent', fig);
        plotRegion(ax, results(1));
    else
        tcols = min(3, n_regions);
        trows = ceil(n_regions / tcols);
        tl = tiledlayout(fig, trows, tcols, 'Padding', 'compact', 'TileSpacing', 'compact');
        title(tl, 'Availability per region');
        for r = 1:n_regions
            ax = nexttile(tl);
            plotRegion(ax, results(r));
        end
    end
end


% -------------------------------------------------------------------------
function plotRegion(ax, res)
    [lons, lats, avail, is_infantry] = flattenRegion(res);

    % Plot non-available first (red), then available (green) on top so
    % positive points dominate visually.
    hold(ax, 'on');
    drawGroup(ax,  is_infantry & ~avail, lons, lats, 'o', [0.85 0.20 0.20], 'inf  unavail');
    drawGroup(ax, ~is_infantry & ~avail, lons, lats, 's', [0.85 0.20 0.20], 'veh  unavail');
    drawGroup(ax,  is_infantry &  avail, lons, lats, 'o', [0.20 0.65 0.30], 'inf  available');
    drawGroup(ax, ~is_infantry &  avail, lons, lats, 's', [0.20 0.65 0.30], 'veh  available');

    % Region bounding box
    rg = res.region;
    plot(ax, [rg.lonlim([1 2 2 1 1])], [rg.latlim([1 1 2 2 1])], ...
         '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 0.75, 'HandleVisibility', 'off');

    hold(ax, 'off');
    grid(ax, 'on');
    axis(ax, 'equal');
    xlabel(ax, 'Longitude (deg)');
    ylabel(ax, 'Latitude (deg)');

    overall = 100 * mean(avail);
    title(ax, sprintf('%s  (overall %.1f%%, n=%d points)', ...
        rg.name, overall, numel(avail)));
    legend(ax, 'Location', 'bestoutside');
end


function plotAllRegionsInto(ax, results)
    hold(ax, 'on');
    seen = struct('inf_un', false, 'veh_un', false, 'inf_av', false, 'veh_av', false);
    for r = 1:numel(results)
        [lons, lats, avail, is_infantry] = flattenRegion(results(r));
        seen.inf_un = drawGroupOnce(ax,  is_infantry & ~avail, lons, lats, 'o', [0.85 0.20 0.20], 'inf  unavail',   seen.inf_un);
        seen.veh_un = drawGroupOnce(ax, ~is_infantry & ~avail, lons, lats, 's', [0.85 0.20 0.20], 'veh  unavail',   seen.veh_un);
        seen.inf_av = drawGroupOnce(ax,  is_infantry &  avail, lons, lats, 'o', [0.20 0.65 0.30], 'inf  available', seen.inf_av);
        seen.veh_av = drawGroupOnce(ax, ~is_infantry &  avail, lons, lats, 's', [0.20 0.65 0.30], 'veh  available', seen.veh_av);
    end
    hold(ax, 'off');
    grid(ax, 'on');
    axis(ax, 'equal');
    xlabel(ax, 'Longitude (deg)');
    ylabel(ax, 'Latitude (deg)');
    legend(ax, 'Location', 'bestoutside');
end


function shown = drawGroupOnce(ax, mask, lons, lats, marker, color, label, already_shown)
    % Like drawGroup, but only the first occurrence of a label registers
    % with the legend - subsequent groups with the same label are drawn
    % with HandleVisibility off so the legend shows each entry once.
    if ~any(mask)
        shown = already_shown;
        return;
    end
    if already_shown
        plot(ax, lons(mask), lats(mask), 'LineStyle','none', ...
             'Marker', marker, 'MarkerSize', 4, ...
             'MarkerEdgeColor', color, 'MarkerFaceColor', color, ...
             'HandleVisibility','off');
    else
        plot(ax, lons(mask), lats(mask), 'LineStyle','none', ...
             'Marker', marker, 'MarkerSize', 4, ...
             'MarkerEdgeColor', color, 'MarkerFaceColor', color, ...
             'DisplayName', label);
    end
    shown = true;
end


function drawGroup(ax, mask, lons, lats, marker, color, label)
    if ~any(mask)
        return;
    end
    if isempty(label)
        plot(ax, lons(mask), lats(mask), 'LineStyle','none', ...
             'Marker', marker, 'MarkerSize', 4, ...
             'MarkerEdgeColor', color, 'MarkerFaceColor', color, ...
             'HandleVisibility','off');
    else
        plot(ax, lons(mask), lats(mask), 'LineStyle','none', ...
             'Marker', marker, 'MarkerSize', 4, ...
             'MarkerEdgeColor', color, 'MarkerFaceColor', color, ...
             'DisplayName', label);
    end
end


function [lons, lats, avail, is_infantry] = flattenRegion(res)
    M = size(res.available, 1);

    lons   = double(reshape(res.rx_locations(:,:,1), [], 1));
    lats   = double(reshape(res.rx_locations(:,:,2), [], 1));
    avail  = reshape(res.available, [], 1);

    is_infantry = false(size(avail));
    n_rx = size(res.available, 2);
    for m = 1:M
        col = res.rx_types{m};
        if iscategorical(col)
            row_mask = (col == 'infantry');
        else
            row_mask = strcmp(string(col), "infantry");
        end
        idx = (m-1)*n_rx + (1:n_rx);
        is_infantry(idx) = row_mask(:);
    end
end
