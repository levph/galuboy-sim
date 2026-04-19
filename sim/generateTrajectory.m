function [lons, lats] = generateTrajectory(config, rotation_deg)
%GENERATETRAJECTORY  Figure-8 (Lemniscate of Bernoulli) flight trajectory.
%
%   [lons, lats] = generateTrajectory(config, rotation_deg)
%
%   Generates a lemniscate centred on the geographic midpoint of the
%   simulation area, scaled to ~1 km, and rotated by rotation_deg degrees.
%
%   config        struct from buildConfig() — uses geo.lonlim, geo.latlim,
%                 sim.num_flight_steps
%   rotation_deg  rotation angle in degrees (0 = unrotated figure-8)
%
%   Returns lons, lats — [1 x num_flight_steps] vectors (degrees).

    center_lon = mean(config.geo.lonlim);
    center_lat = mean(config.geo.latlim);
    scale      = 0.005;   % ~1 km pattern

    t = linspace(0, 2*pi, config.sim.num_flight_steps);

    % Parametric lemniscate (normalised)
    x_norm = cos(t) ./ (1 + sin(t).^2);
    y_norm = sin(t) .* cos(t) ./ (1 + sin(t).^2);

    % Rotate in the local (x, y) plane
    theta  = deg2rad(rotation_deg);
    x_rot  = x_norm * cos(theta) - y_norm * sin(theta);
    y_rot  = x_norm * sin(theta) + y_norm * cos(theta);

    % Map to geographic coordinates
    lons = center_lon + x_rot * scale;
    lats = center_lat + y_rot * scale;
end
