function [lons, lats, headings_rad, tilts_rad] = generateCircularTrajectory(region, flight_cfg)
%GENERATECIRCULARTRAJECTORY  Co-centred circular flight path with constant bank.
%
%   [lons, lats, headings_rad, tilts_rad] = generateCircularTrajectory(region, flight_cfg)
%
%   The aircraft flies an arc of max_rotation_deg around the region centroid
%   (region.center_lat/lon) on a circle of radius flight_cfg.circle_radius_m at
%   constant altitude and a constant bank tilt (flight_cfg.max_tilt_deg). The
%   NUMBER of samples is derived from the requested sample period
%   (flight_cfg.delta_t_s) and the coordinated-turn airspeed - see
%   circularFlightSampling. Samples are evenly spaced over the arc, CCW.
%
%   Returns (all 1 x num_samples):
%     lons, lats     geographic waypoints (deg)
%     headings_rad   flight heading, CW from north (navigation), tangent to the
%                    circle in the direction of travel
%     tilts_rad      constant bank-tilt magnitude (rad). Sign is unused here; the
%                    OUTWARD boresight (tilt_direction='outward') is built where
%                    the TX/RX boresight is formed, from the radial bearing
%                    centre->aircraft (see raytraceRotation).
%
%   Replaces the retired figure-8 lemniscate (legacy sim/generateTrajectory.m is
%   left untouched for the old pipeline).

    samp = circularFlightSampling(flight_cfg);
    t    = samp.sample_angles_rad;          % 1 x Nf, in [0, theta_max)

    clat = region.center_lat;
    clon = region.center_lon;
    m_per_deg_lat = 111320;
    m_per_deg_lon = 111320 * cosd(clat);

    % Local ENU offsets (metres) and geographic waypoints.
    R_m  = flight_cfg.circle_radius_m;
    x = R_m * cos(t);          % east
    y = R_m * sin(t);          % north
    lons = clon + x / m_per_deg_lon;
    lats = clat + y / m_per_deg_lat;

    % Velocity direction (CCW tangent): d/dt (cos,sin) = (-sin, cos).
    dx = -sin(t);              % east component
    dy =  cos(t);              % north component
    headings_rad = atan2(dx, dy);   % CW from north (east=+x, north=+y)

    % Constant bank tilt.
    tilts_rad = deg2rad(flight_cfg.max_tilt_deg) * ones(1, numel(t));
end
