function out = computeLinkAvailability(SNR, req_snr_db, percentile)
%COMPUTELINKAVAILABILITY  Per-device p% SNR, availability, and margin (Part B).
%
%   out = computeLinkAvailability(SNR, req_snr_db, percentile)
%
%   SNR         [n x Nf] per-(device, flight-point) SNR (dB).
%   req_snr_db  required SNR for the link (scalar dB).
%   percentile  availability percentile p (e.g. 99).
%
%   For each device the p% SNR is the level it stays ABOVE for p% of the
%   trajectory = the (100-p)th percentile of its SNR samples:
%       pSNR = prctile(SNR, 100 - p, 2)
%   A device is available if pSNR >= req_snr_db; margin = pSNR - req_snr_db.
%   Outage points (SNR = -Inf) are kept, so a device with > (100-p)% outage
%   gets pSNR = -Inf (unavailable), as intended.
%
%   Returns out with [n x 1] fields: pSNR_db, available (logical), margin_db.
%
%   (Named computeLinkAvailability, not computeAvailability, to avoid the
%   legacy/computeAvailability.m on the path.)

    pSNR = prctile(SNR, 100 - percentile, 2);   % per device, over flight points
    out.pSNR_db   = pSNR;
    out.available = pSNR >= req_snr_db;
    out.margin_db = pSNR - req_snr_db;
end
