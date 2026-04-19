function gnd_dist_m = distanceFromCentroid(lonlim, latlim, rx_points)
%DISTANCEFROMCENTROID  Haversine distances from the geographic centroid.
%
%   gnd_dist_m = distanceFromCentroid(lonlim, latlim, rx_points)
%
%   lonlim    [1x2]  [west east] longitude limits (degrees)
%   latlim    [1x2]  [south north] latitude limits (degrees)
%   rx_points [Nx2]  columns: [longitude, latitude] (degrees)
%
%   Returns gnd_dist_m [1xN] distances in metres.

    center_lon = mean(lonlim);
    center_lat = mean(latlim);
    gnd_dist_m = haversineDistance(center_lat, center_lon, ...
                                   rx_points(:, 2).', rx_points(:, 1).');
end
