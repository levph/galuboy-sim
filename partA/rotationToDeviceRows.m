function T = rotationToDeviceRows(rot, region_id, id_offset)
%ROTATIONTODEVICEROWS  One row per device, per-point data as array strings.
%
%   T = rotationToDeviceRows(rot, region_id, id_offset)
%
%   Collapses raytraceRotation's [Nf x n_rx] matrices to ONE row per receiver:
%   the static per-device fields plus pl_db / steer_gnd_deg / steer_air_deg as
%   comma-joined per-flight-point arrays (numArrayToStr). device_id is global
%   (id_offset + local index), so regions map to contiguous device-id ranges.
%
%   Columns: device_id, region, region_id, type, height_m, lon, lat, dist_m,
%            pl_db, steer_gnd_deg, steer_air_deg   (last three are array strings)

    n_rx = size(rot.pl_db, 2);

    device_id = (id_offset + (1:n_rx)).';
    region    = repmat(string(rot.region_name), n_rx, 1);
    region_id = repmat(region_id, n_rx, 1);
    type      = string(rot.rx_table.type);
    height_m  = rot.rx_table.height_m;
    lon       = rot.rx_table.lon;
    lat       = rot.rx_table.lat;
    dist_m    = rot.dist_m(:);

    pl_db         = strings(n_rx, 1);
    steer_gnd_deg = strings(n_rx, 1);
    steer_air_deg = strings(n_rx, 1);
    for j = 1:n_rx
        pl_db(j)         = numArrayToStr(rot.pl_db(:, j));
        steer_gnd_deg(j) = numArrayToStr(rot.steer_gnd_deg(:, j));
        steer_air_deg(j) = numArrayToStr(rot.steer_air_deg(:, j));
    end

    T = table(device_id, region, region_id, type, height_m, lon, lat, dist_m, ...
              pl_db, steer_gnd_deg, steer_air_deg);
end
