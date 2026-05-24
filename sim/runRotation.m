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
%   a per-RX availability fraction and a logical availability flag.
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
%     rot.frac_above    [n_rx x 1]  single, mean(Prx > MDS) over flight steps
%                                   (continuous mission-availability metric)
%     rot.available     [n_rx x 1]  logical, frac_above >= percentile/100
%     rot.prx_floor_dbm [n_rx x 1]  single, lower-tail P_rx percentile
%                                   (kept for downstream colormaps / debug;
%                                    equivalent gate: prx_floor_dbm > MDS)
%     rot.rx_table      echo of input rx_table (convenience for downstream)
%
%   Tricky bits:
%     - tx_steps is built OUTSIDE the parfor; mutating txsite properties
%       inside parfor errors with "object cannot be sent to workers".
%     - map_name must be a constant string within the parfor body.
%     - pathloss(...,'Map',...) requires a registered terrain name; pass
%       '' to skip the Name-Value (default GMTED).
%     - TX is flown at constant MSL: each waypoint's AntennaHeight is
%       set to (target_msl - terrain_at_tx). target_msl is either
%       cfg.tx.altitude_msl_m (when provided) or derived from
%       cfg.tx.altitude_m treated as AGL at the trajectory centroid.

    Nf   = numel(flight_lons);
    n_rx = numel(rx_array);

    if nargin < 10 || isempty(flight_tilts),    flight_tilts    = zeros(1, Nf); end
    if nargin < 11 || isempty(flight_headings), flight_headings = zeros(1, Nf); end

    use_terrain = ~isempty(char(map_name));
    map_name_local = char(map_name);   % stable broadcast var for parfor

    % --- Pre-build per-step txsite array (sliced broadcast variable) -----
    tx_steps = repmat(tx_template, 1, Nf);

    % --- TX altitude: hold constant MSL across the trajectory ------------
    % Probe terrain elevation under each TX waypoint (AntennaHeight=0 so
    % elevation() returns the ground MSL, not the aircraft MSL).
    probe = txsite('Latitude', flight_lats, 'Longitude', flight_lons, ...
                   'AntennaHeight', 0);
    if use_terrain
        terrain_at_tx = elevation(probe, 'Map', map_name_local);
    else
        terrain_at_tx = elevation(probe);   % default GMTED2010
    end
    terrain_at_tx = double(terrain_at_tx(:).');     % [1 x Nf]

    % Resolve target MSL. Prefer explicit altitude_msl_m; otherwise treat
    % the legacy altitude_m as AGL at the trajectory centroid.
    if isfield(cfg.tx, 'altitude_msl_m') && ~isempty(cfg.tx.altitude_msl_m)
        target_msl = double(cfg.tx.altitude_msl_m);
    else
        centroid_site = txsite('Latitude', mean(flight_lats), ...
                               'Longitude', mean(flight_lons), 'AntennaHeight', 0);
        if use_terrain
            centroid_el = elevation(centroid_site, 'Map', map_name_local);
        else
            centroid_el = elevation(centroid_site);
        end
        target_msl = double(centroid_el) + double(cfg.tx.altitude_m);
    end

    agl = target_msl - terrain_at_tx;
    min_safe_agl = 50;   % m - refuse to fly below this AGL
    if any(agl < min_safe_agl)
        error('runRotation:terrainConflict', ...
            'MSL target %.0f m hits terrain (min AGL %.0f m) at %d waypoint(s)', ...
            target_msl, min(agl), sum(agl < min_safe_agl));
    end

    for i = 1:Nf
        tx_steps(i).Latitude      = flight_lats(i);
        tx_steps(i).Longitude     = flight_lons(i);
        tx_steps(i).AntennaHeight = agl(i);
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
        % MATLAB angle() returns CCW-from-east; convert to compass
        % (CW-from-north) to match heading + boresight conventions.
        az_compass = mod(single(pi)/2 - single(deg2rad(az_deg(:).')), single(2*pi));
        AZ(i,:) = az_compass;
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
    % angle(rx, tx) returns the direction at RX looking toward TX (RX->TX).
    % RX gain uses that geometry directly. TX gain needs the reciprocal
    % direction (TX->RX): flip azimuth by pi and negate elevation.
    AZ_tx = mod(AZ + single(pi), single(2*pi));
    EL_tx = -EL;
    G_tx  = applyAntennaGain(ant.tx_pat, AZ_tx, EL_tx, tx_az_b, tx_el_b);

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
            AZ(:,  is_infantry), EL(:,  is_infantry), ...
            cfg.rx.infantry.boresight_az_rad, cfg.rx.infantry.boresight_el_rad);
    end
    if any(~is_infantry)
        G_rx(:, ~is_infantry) = applyAntennaGain(ant.veh_pat, ...
            AZ(:, ~is_infantry), EL(:, ~is_infantry), ...
            cfg.rx.vehicular.boresight_az_rad, cfg.rx.vehicular.boresight_el_rad);
    end

    % --- Link budget + availability reduction ----------------------------
    Pt_dbm  = single(cfg.tx.power_dbm);
    fade_db = single(cfg.analysis.fade_margin_db);
    Prx     = Pt_dbm + G_tx - PL + G_rx - fade_db;        % [Nf x n_rx]

    % Mission availability per RX: fraction of flight steps clearing MDS,
    % gated by the configured availability target (cfg.analysis.percentile
    % is the target in % of the rotation, e.g. 99 = "link up >=99% of time").
    threshold      = single(cfg.analysis.threshold_dbm);
    above          = Prx > threshold;                                       % [Nf x n_rx] logical
    rot.frac_above = single(mean(above, 1)).';                              % [n_rx x 1] in [0,1]
    target_frac    = single(cfg.analysis.percentile) / single(100);         % e.g. 0.99
    rot.available  = rot.frac_above >= target_frac;                         % [n_rx x 1] logical

    % Lower-tail Prx percentile, kept for downstream colormaps / debug.
    % Equivalent gate: rot.available == (rot.prx_floor_dbm > threshold).
    rot.prx_floor_dbm = prctile(Prx, 100 - cfg.analysis.percentile, 1).';   % [n_rx x 1]
    rot.rx_table      = rx_table;
end
