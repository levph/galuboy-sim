function T = mcsTable()
%MCSTABLE  MCS index -> minimum required SNR (dB) lookup for Part B.
%
%   T = mcsTable()
%
%   Returns a table with columns:
%     mcs         integer MCS index
%     name        display label
%     snr_min_db  minimum SNR (dB) for that MCS to be decodable
%
%   Part B marks a link "available" for a given MCS when its p%-availability
%   SNR meets or exceeds snr_min_db. Selecting several MCS rows overlays one
%   availability curve per threshold.
%
%   PLACEHOLDER VALUES - the real MCS->SNR mapping is TBD (Step 8). The
%   numbers below are a monotone stand-in so the pipeline can be exercised
%   end to end; replace once the agreed thresholds are provided.

    mcs        = (0:9)';
    name       = "MCS" + string(mcs);
    snr_min_db = [ 2; 4; 7; 10; 13; 16; 19; 22; 25; 28 ];   % TBD placeholder

    T = table(mcs, name, snr_min_db);
end
