function rot = runRotation(tx_template, rx_array, rx_table, pm, map_name, ...
                            flight_lons, flight_lats, ant, cfg, ...
                            flight_tilts, flight_headings)
%RUNROTATION  Fly one trajectory, compute path-loss + link budget + availability.
%
%   rot = runRotation(tx_template, rx_array, rx_table, pm, map_name, ...
%                      flight_lons, flight_lats, ant, cfg)
%
%   Per-rotation streaming compute - no raw [Nf x n_rx] cube is returned.
%   Path loss and antenna gains are evaluated, link budget is reduced to
%   a per-RX percentile P_rx, and a logical availability flag is produced.
%
%   Inputs:
%     tx_template   txsite (position + antenna_height + freq + power
%                   already set; lat/lon overwritten per step)
%     rx_array      rxsite array (length n_rx)
%     rx_table      table with column 'type' (categorical or string),
%                   used to dispatch RX antenna patterns
%     pm            propagationModel object
%     map_name      char/string. '' to use the model's default terrain
%                   (GMTED2010); a registered terrain name to use that.
%     flight_lons   [1 x Nf] longitude waypoints (deg)
%     flight_lats   [1 x Nf] latitude waypoints (deg)
%     ant           struct with fields tx_pat, inf_pat, veh_pat
%                   (each from loadAntennaPattern)
%     cfg           full config struct
%
%   Returns rot:
%     rot.prx_pctile  [n_rx x 1]  percentile P_rx (dBm) over flight steps
%     rot.available   [n_rx x 1]  logical, prx_pctile > threshold_dbm
%     rot.rx_table    echo of input rx_table (convenience for downstream)
%
%   Tricky bits:
%     - tx_steps is built OUTSIDE the parfor; mutating txsite properties
%       inside parfor errors with "object cannot be sent to workers".
%     - map_name must be a constant string within the parfor body.
%     - pathloss(...,'Map',...) requires a registered terrain name; pass
%       '' to skip the Name-Value (default GMTED).

    Nf   = numel(flight_lons);
    n_rx = numel(rx_array);

    if nargin < 10 || isempty(flight_tilts),    flight_tilts    = zeros(1, Nf); end
    if nargin < 11 || isempty(flight_headings), flight_headings = zeros(1, Nf); end

    use_terrain = ~isempty(char(map_name));
    map_name_local = char(map_name);   % stable broadcast var for parfor

    % --- Pre-build per-step txsite array (sliced broadcast variable) -----
    tx_steps = repmat(tx_template, 1, Nf);
    for i = 1:Nf
        tx_steps(i).Latitude  = flight_lats(i);
        tx_steps(i).Longitude = flight_lons(i);
    end

    % --- Path-loss + geometry sweep --------------------------------------
    PL = zeros(Nf, n_rx, 'single');
    AZ = zeros(Nf, n_rx, 'single');
    EL = zeros(Nf, n_rx, 'single');

    parfor i = 1:Nf
        if use_terrain
            pl_row = pathloss(pm, rx_array, tx_steps(i), 'Map', map_name_local);
        else
            pl_row = pathloss(pm, rx_array, tx_steps(i));
        end
        [az_deg, el_deg] = angle(rx_array, tx_steps(i));
        PL(i,:) = single(pl_row(:).');
        AZ(i,:) = single(mod(deg2rad(az_deg(:).') + 2*pi, 2*pi));
        EL(i,:) = single(deg2rad(el_deg(:).'));
    end

    % --- Per-step TX boresight (heading + bank-tilt) ---------------------
    % Configured tx.boresight_{az,el}_rad is treated as a body-frame offset
    % from (heading, nadir). Banking by signed angle phi rolls a nadir-
    % pointing antenna off-nadir by |phi| toward azimuth heading + sign(phi)*pi/2
    % (right of heading for phi>0). This is a small-angle perturbation around
    % a nominally-nadir configured boresight.
    abs_tilt = abs(flight_tilts(:));            % [Nf x 1]
    sgn      = sign(flight_tilts(:));           % [Nf x 1]
    tx_az_b  = cfg.tx.boresight_az_rad + flight_headings(:) + sgn * (pi/2);
    tx_el_b  = cfg.tx.boresight_el_rad + abs_tilt;

    % --- Antenna gains (vectorized) --------------------------------------
    % TX gain: lookup in TX frame (same az/el as angle(rx,tx) returned).
    G_tx = applyAntennaGain(ant.tx_pat, AZ, EL, tx_az_b, tx_el_b);

    % RX gain: viewed from RX -> TX, az shifts by pi, el flips sign.
    AZ_rx = mod(AZ + single(pi), single(2*pi));
    EL_rx = -EL;

    % Per-RX dispatch mask: which RX columns are infantry vs vehicular.
    % (Not to be confused with "infinite path loss" - this is purely about
    % which antenna pattern to apply per device type.)
    % rx_table.type may be categorical or string.
    type_col = rx_table.type;
    if iscategorical(type_col)
        is_infantry = (type_col == 'infantry');
    else
        is_infantry = strcmp(string(type_col), "infantry");
    end
    is_infantry = is_infantry(:).';   % row vector of length n_rx

    G_rx = zeros(Nf, n_rx, 'single');
    if any(is_infantry)
        G_rx(:,  is_infantry) = applyAntennaGain(ant.inf_pat, ...
            AZ_rx(:,  is_infantry), EL_rx(:,  is_infantry), ...
            cfg.rx.infantry.boresight_az_rad, cfg.rx.infantry.boresight_el_rad);
    end
    if any(~is_infantry)
        G_rx(:, ~is_infantry) = applyAntennaGain(ant.veh_pat, ...
            AZ_rx(:, ~is_infantry), EL_rx(:, ~is_infantry), ...
            cfg.rx.vehicular.boresight_az_rad, cfg.rx.vehicular.boresight_el_rad);
    end

    % --- Link budget + availability reduction ----------------------------
    Pt_dbm  = single(cfg.tx.power_dbm);
    fade_db = single(cfg.analysis.fade_margin_db);
    Prx     = Pt_dbm + G_tx - PL + G_rx - fade_db;        % [Nf x n_rx]

    % Per-RX percentile over flight steps. prctile(...,1) reduces dim 1.
    rot.prx_pctile = prctile(Prx, cfg.analysis.percentile, 1).';   % [n_rx x 1]
    rot.available  = rot.prx_pctile > single(cfg.analysis.threshold_dbm);
    rot.rx_table   = rx_table;
end
