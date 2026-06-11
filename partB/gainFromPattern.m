function g_dbi = gainFromPattern(pat, theta_deg)
%GAINFROMPATTERN  Interpolate antenna gain at an off-boresight angle (Part B).
%
%   g_dbi = gainFromPattern(pat, theta_deg)
%
%   pat        from loadGainPattern (azimuth-independent).
%   theta_deg  off-boresight angle(s) in degrees (the steer_gnd_deg /
%              steer_air_deg Part A stored); scalar or array.
%
%   Folds to |theta| and clamps to the pattern's tabulated range (gain holds at
%   the last value beyond max_deg). Output matches the shape of theta_deg.

    t = abs(double(theta_deg));
    t = min(t, pat.max_deg);          % clamp beyond tabulated range
    g = pat.interp(t);
    g_dbi = reshape(g, size(theta_deg));
end
