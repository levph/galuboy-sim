function pl_db = coherentCombineDb(ray_pl_db, ray_phase_rad)
%COHERENTCOMBINEDB  Coherently combine multipath rays into one path loss.
%
%   pl_db = coherentCombineDb(ray_pl_db, ray_phase_rad)
%
%   Sums the rays as complex field amplitudes (voltage), preserving phase, so
%   constructive/destructive interference is captured:
%
%       a_i = 10^(-PL_i/20)                     (linear field amplitude)
%       E   = sum_i a_i * exp(1j * phi_i)       (coherent field sum)
%       PL  = -20*log10(|E|)                     (combined path loss, dB)
%
%   Unlike an incoherent power sum, this can yield a *higher* loss than the
%   strongest ray when rays cancel. Empty input (no ray) or total cancellation
%   collapses to Inf (outage).
%
%   Inputs are per-ray vectors (dB and radians); both must be the same length.

    if isempty(ray_pl_db)
        pl_db = Inf;
        return;
    end

    a = 10 .^ (-double(ray_pl_db(:)) / 20);
    E = sum(a .* exp(1j * double(ray_phase_rad(:))));

    if abs(E) == 0
        pl_db = Inf;
    else
        pl_db = -20 * log10(abs(E));
    end
end
