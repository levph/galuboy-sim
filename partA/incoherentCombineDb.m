function pl_db = incoherentCombineDb(ray_pl_db)
%INCOHERENTCOMBINEDB  Band-averaged (wideband) path loss from multipath rays.
%
%   pl_db = incoherentCombineDb(ray_pl_db)
%
%   Sums the rays' received POWER (ignoring phase):
%
%       PL = -10*log10( sum_i 10^(-PL_i/10) )
%
%   For a wideband OFDM signal whose bandwidth is large compared to the
%   inverse delay spread, the phase cross-terms average out across the band,
%   so the band-averaged received power equals this incoherent power sum. It
%   is the large-scale (fast-fading-averaged) path loss; small-scale fading is
%   handled statistically downstream (Part B fade margin).
%
%   Empty input (no ray / full blockage) -> Inf (outage).
%
%   Input  ray_pl_db : per-ray path loss vector (dB)
%   Output pl_db      : scalar combined path loss (dB)

    if isempty(ray_pl_db)
        pl_db = Inf;
        return;
    end
    pl_db = -10 * log10(sum(10 .^ (-double(ray_pl_db(:)) / 10)));
end
