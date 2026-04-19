function available = computeAvailability(prx, config)
%COMPUTEAVAILABILITY  Percentile-based link availability from P_rx cube.
%
%   available = computeAvailability(prx, config)
%
%   For each (trajectory m, receiver k) pair, the link is declared
%   "available" if the config.link.percentile-th percentile of P_rx over
%   all Nf flight samples exceeds the availability threshold.
%
%   prx       [M x Nf x n_rx] received power (dBm) from computeLinkBudget()
%   config    struct from buildConfig() — uses link.percentile and
%             sim.availability_threshold_dbm
%
%   Returns available — logical [M x n_rx]
%     true  → link is available for this trajectory/receiver pair
%     false → link is below threshold

    % prx_pctile(m, k) = X-th percentile of P_rx over Nf samples
    prx_pctile = squeeze(prctile(prx, config.link.percentile, 2));  % [M x n_rx]

    available = prx_pctile > config.sim.availability_threshold_dbm;
end
