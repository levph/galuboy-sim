function prx = computeLinkBudget(raw_data, tx_gain_func, rx_gain_func, n_rx_types, config)
%COMPUTELINKBUDGET  Apply TX/RX antenna gains and fade margin to path-loss data.
%
%   prx = computeLinkBudget(raw_data, tx_gain_func, rx_gain_func, n_rx_types, config)
%
%   raw_data       struct from run_galuboy.m with fields:
%                    path_loss_db  [M x Nf x n_rx]
%                    azimuth_rad   [M x Nf x n_rx]
%                    elevation_rad [M x Nf x n_rx]
%   tx_gain_func   function handle from setupAntennaPatterns()
%   rx_gain_func   function handle from setupAntennaPatterns()
%   n_rx_types     number of RX antenna types (from setupAntennaPatterns)
%   config         struct from buildConfig()
%
%   Returns prx [M x Nf x n_rx] received power in dBm after:
%     P_rx = P_tx + G_tx(az, el) - PL + G_rx(type, az, el) - fade_margin

    pl  = raw_data.path_loss_db;    % [M x Nf x n_rx]
    az  = raw_data.azimuth_rad;     % [M x Nf x n_rx]
    el  = raw_data.elevation_rad;   % [M x Nf x n_rx]

    n_rx = size(pl, 3);

    % TX antenna gain — looking from RX back toward TX: az+π, elevation flips
    tx_gain = tx_gain_func(mod(az + pi, 2*pi), -el);  % [M x Nf x n_rx]

    % Base received power (before RX antenna gain and fade margin)
    prx = config.tx.power_dbm + tx_gain - pl;

    % Add RX antenna gain per type
    antenna_dist   = config.group.antenna_distribution / sum(config.group.antenna_distribution);
    antenna_counts = round(n_rx * antenna_dist);
    antenna_counts(end) = n_rx - sum(antenna_counts(1:end-1));
    cumsum_counts  = [0, cumsum(antenna_counts)];

    for rx_type = 1:n_rx_types
        idx = (cumsum_counts(rx_type) + 1):cumsum_counts(rx_type + 1);
        prx(:, :, idx) = prx(:, :, idx) + ...
            rx_gain_func(rx_type, ...
                         mod(az(:, :, idx) + pi, 2*pi), ...
                         -el(:, :, idx));
    end

    % Apply fade margin
    prx = prx - config.prop.fade_margin_db;
end
