function s = circularFlightSampling(flight_cfg)
%CIRCULARFLIGHTSAMPLING  Derive speed / sample count for a constant-bank orbit.
%
%   s = circularFlightSampling(flight_cfg)
%
%   In a level coordinated turn the bank angle and radius fix the true airspeed
%   (the horizontal lift component supplies the centripetal force):
%
%       v     = sqrt(g * R * tan(phi))      (m/s)
%       omega = v / R                        (rad/s, angular rate about centre)
%
%   The user specifies the arc to fly (max_rotation_deg), the sample period
%   (delta_t_s) and the radius (circle_radius_m); the NUMBER OF SAMPLES is
%   derived so samples are ~delta_t_s apart:
%
%       num_samples = round( theta_max / (omega * delta_t_s) )
%
%   Samples are then spaced evenly over the arc (dtheta = theta_max/num_samples),
%   so the effective sample period (s.dt_effective_s) equals the requested
%   delta_t_s up to integer rounding. For a full 360 deg pass the closing point
%   coincides with the start and is therefore excluded (angles span [0,theta_max)).
%
%   Required flight_cfg fields:
%     max_rotation_deg, delta_t_s, circle_radius_m, max_tilt_deg
%
%   Output struct s:
%     v_mps, omega_radps, period_s, num_samples,
%     sample_angles_rad (1 x num_samples), dt_effective_s

    g   = 9.81;                              % m/s^2
    phi = deg2rad(flight_cfg.max_tilt_deg);
    R   = flight_cfg.circle_radius_m;

    v     = sqrt(g * R * tan(phi));
    omega = v / R;
    theta_max = deg2rad(flight_cfg.max_rotation_deg);

    Nf     = max(2, round(theta_max / (omega * flight_cfg.delta_t_s)));
    dtheta = theta_max / Nf;                 % even angular spacing

    s.v_mps             = v;
    s.omega_radps       = omega;
    s.period_s          = theta_max / omega;
    s.num_samples       = Nf;
    s.sample_angles_rad = (0:Nf-1) * dtheta; % [0, theta_max)
    s.dt_effective_s    = dtheta / omega;

end
