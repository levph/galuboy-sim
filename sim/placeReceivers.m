function [rx_array, rx_table] = placeReceivers(region, rx_cfg)
%PLACERECEIVERS  Drop infantry + vehicular RXs uniformly within a region.
%
%   [rx_array, rx_table] = placeReceivers(region, rx_cfg)
%
%   region  struct from resolveRegion (latlim, lonlim, ...)
%   rx_cfg  config.rx struct with fields:
%             infantry.count, infantry.height_m, infantry.antenna_name,
%             vehicular.count, vehicular.height_m, vehicular.antenna_name
%
%   Returns:
%     rx_array  rxsite array, length n_inf + n_veh, with VECTOR
%               'AntennaHeight' (R2021b+ supports this).
%     rx_table  table with columns: lon, lat, type, height_m, antenna_name
%
%   Order: infantry first, vehicular second. type is a categorical
%   ("infantry" | "vehicular").

    n_inf = rx_cfg.infantry.count;
    n_veh = rx_cfg.vehicular.count;
    n_rx  = n_inf + n_veh;

    if n_rx <= 0
        error('placeReceivers:noReceivers', ...
              'rx.infantry.count + rx.vehicular.count must be > 0.');
    end

    % Placement: uniform inside a disk of placement_radius_km around the
    % centroid when configured; otherwise uniform inside the bbox. Disk
    % sampling: r = R*sqrt(U), theta = 2*pi*V → uniform over area.
    if isfield(rx_cfg, 'placement_radius_km') && ~isempty(rx_cfg.placement_radius_km) ...
            && rx_cfg.placement_radius_km > 0
        clat = region.center_lat;
        clon = region.center_lon;
        R_m  = rx_cfg.placement_radius_km * 1000;
        m_per_deg_lat = 111320;
        m_per_deg_lon = 111320 * cosd(clat);
        u = rand(n_rx, 1);
        v = rand(n_rx, 1);
        r_m = R_m * sqrt(u);
        th  = 2*pi * v;
        lons = clon + (r_m .* cos(th)) / m_per_deg_lon;
        lats = clat + (r_m .* sin(th)) / m_per_deg_lat;
    else
        lon_lo = region.lonlim(1); lon_hi = region.lonlim(2);
        lat_lo = region.latlim(1); lat_hi = region.latlim(2);
        lons = lon_lo + (lon_hi - lon_lo) * rand(n_rx, 1);
        lats = lat_lo + (lat_hi - lat_lo) * rand(n_rx, 1);
    end

    type = categorical( ...
        [repmat("infantry",  n_inf, 1); repmat("vehicular", n_veh, 1)], ...
        ["infantry", "vehicular"]);

    height_m     = [repmat(rx_cfg.infantry.height_m, n_inf, 1);
                    repmat(rx_cfg.vehicular.height_m, n_veh, 1)];
    antenna_name = [repmat(string(rx_cfg.infantry.antenna_name),  n_inf, 1);
                    repmat(string(rx_cfg.vehicular.antenna_name), n_veh, 1)];

    rx_table = table(lons, lats, type, height_m, antenna_name, ...
                     'VariableNames', {'lon','lat','type','height_m','antenna_name'});

    % Single rxsite array with vector AntennaHeight - same pattern as
    % freq-planner's gnd_h construction. rxsite requires ROW vectors.
    rx_array = rxsite('Latitude',      lats(:).', ...
                      'Longitude',     lons(:).', ...
                      'AntennaHeight', height_m(:).');
end
