function plotTrajectoryMap(region, lons, lats, rx_table, target_axes, diameter_km)
%PLOTTRAJECTORYMAP  Render the figure-8 flight path and receiver positions.
%
%   plotTrajectoryMap(region, lons, lats, rx_table, target_axes)
%   plotTrajectoryMap(..., diameter_km)
%
%   region        struct from resolveRegion (uses lonlim, latlim, name)
%   lons, lats    trajectory waypoints (1 x Nf, deg)
%   rx_table      table from placeReceivers (columns: lon, lat, type)
%   target_axes   axes handle to draw into (UI use)
%   diameter_km   optional DIAMETER (km) of the RX placement disk drawn
%                 as a reference circle around the centroid. Pass [] or 0
%                 to skip. The circle's radius on the map is diameter/2.

    if nargin < 6, diameter_km = []; end

    ax = target_axes;
    cla(ax);
    hold(ax, 'on');

    % --- OSM highways (visual basemap) ----------------------------------
    % Drawn first so the trajectory and RX markers render on top. Skipped
    % silently if the OSM file is missing.
    if isfield(region, 'osm_path') && ~isempty(region.osm_path) && isfile(region.osm_path)
        try
            highways = osmReadHighways(region.osm_path);
            drawOsmHighways(ax, highways, region);
        catch ME
            warning('plotTrajectoryMap:osmParseFailed', ...
                    'Could not render OSM highways: %s', ME.message);
        end
    end

    % Region bounding box
    plot(ax, region.lonlim([1 2 2 1 1]), region.latlim([1 1 2 2 1]), ...
         '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 0.75, ...
         'DisplayName', 'region bbox');

    % Reference circle (RX placement disk) around centroid
    if ~isempty(diameter_km) && diameter_km > 0
        clat = 0.5 * (region.latlim(1) + region.latlim(2));
        clon = 0.5 * (region.lonlim(1) + region.lonlim(2));
        if isfield(region, 'center_lat'), clat = region.center_lat; end
        if isfield(region, 'center_lon'), clon = region.center_lon; end
        m_per_deg_lat = 111320;
        m_per_deg_lon = 111320 * cosd(clat);
        radius_m = (diameter_km / 2) * 1000;
        th = linspace(0, 2*pi, 180);
        circ_lon = clon + (radius_m / m_per_deg_lon) * cos(th);
        circ_lat = clat + (radius_m / m_per_deg_lat) * sin(th);
        plot(ax, circ_lon, circ_lat, '--', 'Color', [0.85 0.20 0.20], ...
             'LineWidth', 1.0, 'DisplayName', sprintf('%g km diameter', diameter_km));
        plot(ax, clon, clat, '+', 'MarkerSize', 8, 'LineWidth', 1.2, ...
             'Color', [0.85 0.20 0.20], 'HandleVisibility', 'off');
    end

    % Figure-8 trajectory
    plot(ax, lons, lats, '-', 'Color', [0.20 0.45 0.75], 'LineWidth', 1.4, ...
         'DisplayName', 'flight (rot 1)');
    plot(ax, lons(1), lats(1), 'o', 'MarkerSize', 6, ...
         'MarkerEdgeColor', [0.20 0.45 0.75], 'MarkerFaceColor', [1 1 1], ...
         'HandleVisibility', 'off');

    % Receivers
    type_col = rx_table.type;
    if iscategorical(type_col)
        is_inf = (type_col == 'infantry');
    else
        is_inf = strcmp(string(type_col), "infantry");
    end
    if any(is_inf)
        plot(ax, rx_table.lon(is_inf), rx_table.lat(is_inf), ...
             'LineStyle', 'none', 'Marker', 'o', 'MarkerSize', 5, ...
             'MarkerEdgeColor', [0.10 0.10 0.10], ...
             'MarkerFaceColor', [0.95 0.75 0.10], ...
             'DisplayName', 'infantry');
    end
    if any(~is_inf)
        plot(ax, rx_table.lon(~is_inf), rx_table.lat(~is_inf), ...
             'LineStyle', 'none', 'Marker', 's', 'MarkerSize', 6, ...
             'MarkerEdgeColor', [0.10 0.10 0.10], ...
             'MarkerFaceColor', [0.40 0.70 0.95], ...
             'DisplayName', 'vehicular');
    end

    hold(ax, 'off');
    grid(ax, 'on');
    axis(ax, 'equal');
    xlabel(ax, 'Longitude (deg)');
    ylabel(ax, 'Latitude (deg)');
    title(ax, sprintf('%s  -  trajectory (rot 1) + RX layout', region.name));
    legend(ax, 'Location', 'bestoutside');
end


% -------------------------------------------------------------------------
function drawOsmHighways(ax, highways, region)
    if isempty(highways), return; end
    % Concatenate every polyline into one (X,Y) vector with NaN separators
    % so a single plot() call renders the whole basemap (much faster than
    % one plot per way).
    counts = cellfun(@(p) size(p,1), highways);
    total  = sum(counts) + numel(highways);   % +1 NaN per polyline
    X = nan(total, 1); Y = nan(total, 1);
    cur = 1;
    for k = 1:numel(highways)
        n = counts(k);
        X(cur:cur+n-1) = highways{k}(:,1);
        Y(cur:cur+n-1) = highways{k}(:,2);
        cur = cur + n + 1;   % leave one NaN as a break
    end
    % Clip to a slightly padded bbox to avoid drawing way out of view
    pad_lon = 0.1 * (region.lonlim(2) - region.lonlim(1));
    pad_lat = 0.1 * (region.latlim(2) - region.latlim(1));
    out = X < region.lonlim(1) - pad_lon | X > region.lonlim(2) + pad_lon | ...
          Y < region.latlim(1) - pad_lat | Y > region.latlim(2) + pad_lat;
    X(out) = NaN; Y(out) = NaN;

    plot(ax, X, Y, '-', 'Color', [0.65 0.65 0.65], 'LineWidth', 0.5, ...
         'DisplayName', 'OSM roads');
end
