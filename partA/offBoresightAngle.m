function theta_rad = offBoresightAngle(az_rad, el_rad, boresight_az_rad, boresight_el_rad)
%OFFBORESIGHTANGLE  Angular distance between a link direction and a boresight.
%
%   theta_rad = offBoresightAngle(az_rad, el_rad, boresight_az_rad, boresight_el_rad)
%
%   The 3D angle between the (az,el) link direction and the antenna boresight,
%   via the spherical law of cosines:
%
%       cos(theta) = cos(el)cos(el_b)cos(az-az_b) + sin(el)sin(el_b)
%
%   This is the steering angle Part A stores per link side (steer_gnd against
%   the ground boresight = zenith; steer_air against the air boresight =
%   outward-tilted nadir). It is robust at the nadir/zenith poles where the
%   naive (az-az_b, el-el_b) decomposition is degenerate.
%
%   All inputs in radians; output in [0, pi], shape of the az/el inputs.

    dot_qb = cos(el_rad) .* cos(boresight_el_rad) ...
          .* cos(az_rad  -  boresight_az_rad) ...
          +  sin(el_rad) .* sin(boresight_el_rad);

    one    = ones(1, 1, 'like', dot_qb);
    dot_qb = max(-one, min(one, dot_qb));   % clamp before acos
    theta_rad = acos(dot_qb);
end
