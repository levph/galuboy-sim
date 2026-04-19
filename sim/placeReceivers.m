function [rx_array, rx_points] = placeReceivers(n_rx, config)
%PLACERECEIVERS  Randomly place N ground receivers within the simulation area.
%
%   [rx_array, rx_points] = placeReceivers(n_rx, config)
%
%   n_rx      number of receiver points to generate
%   config    struct from buildConfig() — uses geo.lonlim, geo.latlim
%
%   rx_array   MATLAB rxsite array (n_rx x 1), antenna height = 1.5 m
%   rx_points  [n_rx x 2] matrix of [longitude, latitude] in degrees

    limits    = [config.geo.lonlim; config.geo.latlim];   % [2x2]
    rx_points = limits(:, 1).' + rand(n_rx, 2) .* diff(limits, 1, 2).';  % [n_rx x 2]

    rx_array = rxsite('Latitude',      rx_points(:, 2), ...
                      'Longitude',     rx_points(:, 1), ...
                      'AntennaHeight', 1.5);
end
