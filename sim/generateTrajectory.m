function [lons, lats, tilts_rad, headings_rad] = generateTrajectory(region, flight_cfg, rotation_deg)
%GENERATETRAJECTORY  Figure-8 (Lemniscate of Bernoulli) flight trajectory.
%
%   [lons, lats, tilts_rad, headings_rad] = generateTrajectory(region, ...
%                                                flight_cfg, rotation_deg)
%
%   Generates a lemniscate centred on the geographic midpoint of the
%   region, scaled to flight_cfg.lemniscate_scale_deg, and rotated by
%   rotation_deg degrees.
%
%   region        struct from resolveRegion (uses center_lon, center_lat)
%   flight_cfg    config.flight struct (uses num_flight_steps,
%                 lemniscate_scale_deg, max_tilt_deg)
%   rotation_deg  rotation angle in degrees (0 = unrotated figure-8)
%
%   Returns:
%     lons, lats     [1 x num_flight_steps] geographic waypoints (deg)
%     tilts_rad      [1 x num_flight_steps] signed bank angle (rad);
%                    sign encodes turn direction. Magnitude is normalized
%                    so max(|tilt|) == deg2rad(flight_cfg.max_tilt_deg).
%     headings_rad   [1 x num_flight_steps] flight heading (rad), CW from
%                    north (navigation convention) — same frame as the az
%                    returned by rxsite/txsite angle().

    center_lon = region.center_lon;
    center_lat = region.center_lat;
    Nf         = flight_cfg.num_flight_steps;
    max_tilt   = deg2rad(flight_cfg.max_tilt_deg);

    % Lemniscate scale: prefer length_m (full E-W extent in meters) when
    % it is provided and positive; fall back to scale_deg otherwise.
    % Normalised lemniscate spans x in [-1, 1], so full extent = 2*scale.
    if isfield(flight_cfg, 'lemniscate_length_m') ...
            && ~isempty(flight_cfg.lemniscate_length_m) ...
            && flight_cfg.lemniscate_length_m > 0
        meters_per_deg_lon = 111320 * cosd(center_lat);
        scale = flight_cfg.lemniscate_length_m / (2 * meters_per_deg_lon);
    else
        scale = flight_cfg.lemniscate_scale_deg;
    end

    t = linspace(0, 2*pi, Nf);

    % Parametric lemniscate (normalised)
    x_norm = cos(t) ./ (1 + sin(t).^2);
    y_norm = sin(t) .* cos(t) ./ (1 + sin(t).^2);

    % Rotate in the local (x, y) plane
    theta = deg2rad(rotation_deg);
    x_rot = x_norm * cos(theta) - y_norm * sin(theta);
    y_rot = x_norm * sin(theta) + y_norm * cos(theta);

    % Map to geographic coordinates (x -> lon-east, y -> lat-north).
    lons = center_lon + x_rot * scale;
    lats = center_lat + y_rot * scale;

    % --- Heading (nav frame: CW from north) ------------------------------
    % gradient() handles endpoints with one-sided differences; the closed
    % curve has t=0 == t=2*pi, so the duplicated endpoint is benign.
    dx = gradient(x_rot);
    dy = gradient(y_rot);
    headings_rad = atan2(dx, dy);   % CW from north (east=+x, north=+y)

    % --- Signed curvature (planar, normalized space) ---------------------
    ddx = gradient(dx);
    ddy = gradient(dy);
    speed_sq = dx.^2 + dy.^2;
    kappa    = (dx .* ddy - dy .* ddx) ./ (speed_sq .^ 1.5 + eps);

    peak = max(abs(kappa));
    if peak > 0
        tilts_rad = max_tilt * kappa / peak;
    else
        tilts_rad = zeros(1, Nf);
    end
end
