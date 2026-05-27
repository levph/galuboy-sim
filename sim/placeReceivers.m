function [rx_array, rx_table] = placeReceivers(region, rx_cfg, buildings)
%PLACERECEIVERS  Drop infantry + vehicular RXs uniformly within a region.
%
%   [rx_array, rx_table] = placeReceivers(region, rx_cfg)
%   [rx_array, rx_table] = placeReceivers(region, rx_cfg, buildings)
%
%   region     struct from resolveRegion (latlim, lonlim, ...)
%   rx_cfg     config.rx struct with fields:
%                infantry.count, infantry.height_m, infantry.antenna_name_dl,
%                vehicular.count, vehicular.height_m, vehicular.antenna_name_dl
%   buildings  optional cell array of [N x 2] polygons (lon, lat) used to
%              reject samples that fall inside a building footprint. {} or
%              omitted disables the check.
%
%   Returns:
%     rx_array  rxsite array, length n_inf + n_veh, with VECTOR
%               'AntennaHeight' (R2021b+ supports this).
%     rx_table  table with columns: lon, lat, type, height_m, antenna_name_dl
%
%   Order: infantry first, vehicular second. type is a categorical
%   ("infantry" | "vehicular"). The antenna_name_dl column carries the
%   DL receiver antenna name; the ground TX antenna is shared across
%   types (config.tx.ground.antenna_name).

    if nargin < 3 || isempty(buildings)
        buildings = {};
    end

    n_inf = rx_cfg.infantry.count;
    n_veh = rx_cfg.vehicular.count;
    n_rx  = n_inf + n_veh;

    if n_rx <= 0
        error('placeReceivers:noReceivers', ...
              'rx.infantry.count + rx.vehicular.count must be > 0.');
    end

    % Decide sampling mode once (disk vs bbox) so the per-sample helper
    % only does the math. placement_diameter_km is the user-facing knob;
    % the disk radius is half of that.
    use_disk = isfield(rx_cfg, 'placement_diameter_km') ...
            && ~isempty(rx_cfg.placement_diameter_km) ...
            && rx_cfg.placement_diameter_km > 0;
    if use_disk
        sampler_ctx.clat = region.center_lat;
        sampler_ctx.clon = region.center_lon;
        sampler_ctx.R_m  = (rx_cfg.placement_diameter_km / 2) * 1000;
        sampler_ctx.m_per_deg_lat = 111320;
        sampler_ctx.m_per_deg_lon = 111320 * cosd(sampler_ctx.clat);
    else
        sampler_ctx.lon_lo = region.lonlim(1);
        sampler_ctx.lon_hi = region.lonlim(2);
        sampler_ctx.lat_lo = region.latlim(1);
        sampler_ctx.lat_hi = region.latlim(2);
    end

    n_buildings  = numel(buildings);
    max_attempts = 50;
    lons = zeros(n_rx, 1);
    lats = zeros(n_rx, 1);
    for k = 1:n_rx
        accepted = false;
        lon_k = 0; lat_k = 0;   % keep defined for the fallback path
        for attempt = 1:max_attempts
            [lon_k, lat_k] = sampleOne(use_disk, sampler_ctx);
            if n_buildings == 0 || ~insideAnyBuilding(lon_k, lat_k, buildings)
                lons(k) = lon_k; lats(k) = lat_k;
                accepted = true;
                break;
            end
        end
        if ~accepted
            warning('placeReceivers:buildingReject', ...
                'rx %d: %d rejections, accepting last sample (likely on a building)', ...
                k, max_attempts);
            lons(k) = lon_k; lats(k) = lat_k;
        end
    end

    type = categorical( ...
        [repmat("infantry",  n_inf, 1); repmat("vehicular", n_veh, 1)], ...
        ["infantry", "vehicular"]);

    height_m        = [repmat(rx_cfg.infantry.height_m, n_inf, 1);
                       repmat(rx_cfg.vehicular.height_m, n_veh, 1)];
    antenna_name_dl = [repmat(string(rx_cfg.infantry.antenna_name_dl),  n_inf, 1);
                       repmat(string(rx_cfg.vehicular.antenna_name_dl), n_veh, 1)];

    rx_table = table(lons, lats, type, height_m, antenna_name_dl, ...
                     'VariableNames', {'lon','lat','type','height_m','antenna_name_dl'});

    % Single rxsite array with vector AntennaHeight - same pattern as
    % freq-planner's gnd_h construction. rxsite requires ROW vectors.
    rx_array = rxsite('Latitude',      lats(:).', ...
                      'Longitude',     lons(:).', ...
                      'AntennaHeight', height_m(:).');
end


% -------------------------------------------------------------------------
function [lon_k, lat_k] = sampleOne(use_disk, ctx)
    if use_disk
        u = rand();
        v = rand();
        r_m = ctx.R_m * sqrt(u);
        th  = 2*pi * v;
        lon_k = ctx.clon + (r_m * cos(th)) / ctx.m_per_deg_lon;
        lat_k = ctx.clat + (r_m * sin(th)) / ctx.m_per_deg_lat;
    else
        lon_k = ctx.lon_lo + (ctx.lon_hi - ctx.lon_lo) * rand();
        lat_k = ctx.lat_lo + (ctx.lat_hi - ctx.lat_lo) * rand();
    end
end


function tf = insideAnyBuilding(lon, lat, buildings)
    tf = false;
    for b = 1:numel(buildings)
        poly = buildings{b};
        if isempty(poly), continue; end
        if inpolygon(lon, lat, poly(:, 1), poly(:, 2))
            tf = true;
            return;
        end
    end
end
