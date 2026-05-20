function G_dbi = applyAntennaGain(pat, az_rad, el_rad, boresight_az_rad, boresight_el_rad)
%APPLYANTENNAGAIN  Look up antenna gain at link az/el (vectorized).
%
%   G_dbi = applyAntennaGain(pat, az_rad, el_rad, boresight_az_rad, boresight_el_rad)
%
%   pat                 struct from loadAntennaPattern
%   az_rad, el_rad      link az/el in the WORLD frame (radians)
%   boresight_az_rad    antenna boresight azimuth (radians)
%   boresight_el_rad    antenna boresight elevation (radians)
%
%   Returns G_dbi (same shape as az_rad / el_rad).
%
%   Frame conventions:
%     - angle(rx, tx) returns az/el at RX looking toward TX (RX->TX).
%     - For RX: pass az/el as returned by angle(rx, tx) directly.
%       RX boresight default = +pi/2 (zenith).
%     - For TX: pass the reciprocity-flipped direction (az+pi, -el) so the
%       lookup is in the TX->RX direction. TX boresight default = -pi/2 (nadir).
%     The caller (runRotation) is responsible for applying that flip.
%
%   Az is folded into [0, 2*pi) modulo the boresight, matching the pattern
%   table's az axis (which represents OFFSETS from boresight in the same
%   frame and is extended to be periodic). Elevation is clamped to
%   [-pi/2, pi/2].

    if pat.is_isotropic
        G_dbi = pat.flat_gain_dbi * ones(size(az_rad), 'like', az_rad);
        return;
    end

    az_off = mod(az_rad - boresight_az_rad, 2*pi);   % match grid range [0, 2*pi]
    el_off = el_rad - boresight_el_rad;
    el_off = max(min(el_off, pi/2), -pi/2);

    % griddedInterpolant accepts double; cast and cast back to preserve
    % the input numeric type (single in the hot path). Linearize the
    % query arrays so MATLAB doesn't issue the spurious "MESHGRID format"
    % performance hint - our queries are scattered, not gridded.
    sz = size(az_rad);
    out = pat.gain_func(double(az_off(:)), double(el_off(:)));
    G_dbi = reshape(cast(out, 'like', az_rad), sz);
end
