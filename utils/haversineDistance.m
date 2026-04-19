function dist_m = haversineDistance(lat1_deg, lon1_deg, lat2_deg, lon2_deg)
%HAVERSINEDISTANCE  Great-circle distance in metres.
%
%   dist_m = haversineDistance(lat1_deg, lon1_deg, lat2_deg, lon2_deg)
%
%   Inputs may be scalars or [1 x N] arrays (lat2/lon2 should be the array).
%   Returns distances in metres.

    R    = 6371000;   % Earth radius (m)
    dlat = deg2rad(lat2_deg - lat1_deg);
    dlon = deg2rad(lon2_deg - lon1_deg);
    a    = sin(dlat/2).^2 + ...
           cos(deg2rad(lat1_deg)) .* cos(deg2rad(lat2_deg)) .* sin(dlon/2).^2;
    dist_m = 2 * R * asin(sqrt(a));
end
