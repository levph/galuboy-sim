function rot = runRotation(tx_template, rx_array, rx_table, pm, map_name, ...
                            flight_lons, flight_lats, ant, cfg, ...
                            flight_tilts, flight_headings)
%RUNROTATION  Fly one trajectory, compute path-loss + DL/UL link budget + availability.
%
%   rot = runRotation(tx_template, rx_array, rx_table, pm, map_name, ...
%                      flight_lons, flight_lats, ant, cfg)
%
%   Per-rotation streaming compute - no raw [Nf x n_rx] cube is returned.
%   Path loss and antenna gains are evaluated, link budgets are reduced to
%   per-RX mission-availability fractions and gates in BOTH directions:
%
%     Prx_dl = Ptx_air + Gtx_air(angles_air->gnd) - PL + Grx_gnd{1|2}(angles_gnd->air) - margin
%     Prx_ul = Ptx_gnd + Gtx_gnd(angles_gnd->air) - PL + Grx_air(angles_air->gnd) - margin
%
%   Path loss is reciprocal; antenna heights and frequency are identical
%   between DL and UL, so PL is computed once per (TX_air, RX_gnd) pair.
%
%   Inputs:
%     tx_template   txsite (position + antenna_height + freq + power
%                   already set; lat/lon/AntennaHeight overwritten per step).
%                   Used as the airborne TX (DL) and as the geometry source.
%     rx_array      rxsite array (length n_rx) for the ground RXs (DL).
%     rx_table      table with column 'type' (categorical or string),
%                   used to dispatch ground RX antenna patterns
%                   (infantry -> ground1_rx, vehicular -> ground2_rx).
%     pm            propagationModel object
%     map_name      char/string. '' to use the model's default terrain
%                   (GMTED2010); a registered terrain name to use that.
%     flight_lons   [1 x Nf] longitude waypoints (deg)
%     flight_lats   [1 x Nf] latitude waypoints (deg)
%     ant           struct with fields (each from loadAntennaPattern):
%                     ant.tx_air_pat   airborne TX pattern (DL transmitter)
%                     ant.tx_gnd_pat   ground   TX pattern (UL transmitter)
%                     ant.rx_air_pat   airborne RX pattern (UL receiver)
%                     ant.rx_gnd1_pat  ground   RX pattern, infantry  (DL)
%                     ant.rx_gnd2_pat  ground   RX pattern, vehicular (DL)
%     cfg           full config struct
%
%   Returns rot:
%     rot.frac_above_dl    [n_rx x 1]  DL mean(Prx > MDS) over flight steps
%     rot.frac_above_ul    [n_rx x 1]  UL mean(Prx > MDS) over flight steps
%     rot.available_dl     [n_rx x 1]  logical, frac_above_dl >= percentile/100
%     rot.available_ul     [n_rx x 1]  logical, frac_above_ul >= percentile/100
%     rot.available_bidi   [n_rx x 1]  logical, available_dl AND available_ul
%     rot.prx_floor_dbm_dl [n_rx x 1]  DL lower-tail Prx percentile (dBm)
%     rot.prx_floor_dbm_ul [n_rx x 1]  UL lower-tail Prx percentile (dBm)
%     rot.frac_above       alias for rot.frac_above_dl    (back-compat)
%     rot.available        alias for rot.available_dl     (back-compat)
%     rot.prx_floor_dbm    alias for rot.prx_floor_dbm_dl (back-compat)
%     rot.rx_table         echo of input rx_table (convenience for downstream)
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
    % elevation() with no 'Map' arg silently falls back to MATLAB's GMTED2010
    % service over HTTPS, which is unusable on air-gapped on-prem boxes. So
    % when use_terrain=false (no registered terrain) we assume FLAT EARTH:
    % terrain elevation = 0 everywhere, and altitude_m is treated as MSL.
    % This is the right semantic for free-space runs and never touches the
    % network. For Longley-Rice runs you must use_terrain=true; runScenario
    % refuses the LR + no-terrain combination up front.
    probe = txsite('Latitude', flight_lats, 'Longitude', flight_lons, ...
                   'AntennaHeight', 0);
    if use_terrain
        terrain_at_tx = elevation(probe, 'Map', map_name_local);
    else
        terrain_at_tx = zeros(1, numel(flight_lats));   % flat earth, offline
    end
    terrain_at_tx = double(terrain_at_tx(:).');     % [1 x Nf]

    if isfield(cfg.tx, 'altitude_msl_m') && ~isempty(cfg.tx.altitude_msl_m)
        target_msl = double(cfg.tx.altitude_msl_m);
    else
        if use_terrain
            centroid_site = txsite('Latitude', mean(flight_lats), ...
                                   'Longitude', mean(flight_lons), 'AntennaHeight', 0);
            centroid_el = elevation(centroid_site, 'Map', map_name_local);
        else
            centroid_el = 0;   % flat earth, offline
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
    % Geometry is angle(rx_array, tx_steps(i)) = direction at RX looking
    % toward TX (RX->TX). PL is reciprocal so the same matrix is used for
    % both DL and UL.
    PL = zeros(Nf, n_rx, 'single');
    AZ = zeros(Nf, n_rx, 'single');
    EL = zeros(Nf, n_rx, 'single');

    parfor i = 1:Nf
        if use_terrain
            pl_row = pathloss(pm, rx_array, tx_steps(i), 'Map', map_name_local);
        else
            % No-terrain branch: only safe for propagation models that don't
            % consult a terrain raster (e.g. free-space). Longley-Rice without
            % a Map would silently pull GMTED2010 over HTTPS - runScenario
            % refuses that combination at startup, so this line is only
            % reached for terrain-free models.
            pl_row = pathloss(pm, rx_array, tx_steps(i));
        end
        % angle() internally queries terrain to compute absolute MSL positions
        % from the per-site AGL AntennaHeight values. Without 'Map' it defaults
        % to gmted2010 (network fetch), which fails on air-gapped boxes with
        % "Unable to access terrain data 'gmted2010'". The 'Map' arg IS honored
        % by angle() in R2021b+ - same pattern as pathloss() above.
        if use_terrain
            [az_deg, el_deg] = angle(rx_array, tx_steps(i), 'Map', map_name_local);
        else
            [az_deg, el_deg] = angle(rx_array, tx_steps(i));
        end
        PL(i,:) = single(pl_row(:).');
        % MATLAB angle() returns CCW-from-east; convert to compass
        % (CW-from-north) to match heading + boresight conventions.
        az_compass = mod(single(pi)/2 - single(deg2rad(az_deg(:).')), single(2*pi));
        AZ(i,:) = az_compass;
        EL(i,:) = single(deg2rad(el_deg(:).'));
    end

    % --- Per-step TX boresight (heading + bank-tilt) ---------------------
    abs_tilt = abs(flight_tilts(:));            % [Nf x 1]
    sgn      = sign(flight_tilts(:));           % [Nf x 1]
    tx_az_b  = cfg.tx.boresight_az_rad + flight_headings(:) + sgn * (pi/2);
    tx_el_b  = cfg.tx.boresight_el_rad + abs_tilt;

    % UL RX (airborne) shares the airframe; its boresight follows the same
    % heading + bank perturbation.
    rx_air_az_b = cfg.rx.airborne.boresight_az_rad + flight_headings(:) + sgn * (pi/2);
    rx_air_el_b = cfg.rx.airborne.boresight_el_rad + abs_tilt;

    % --- Antenna gains (vectorized) --------------------------------------
    % Antenna gain pattern G(theta) is a function of direction in the
    % antenna's own body frame -- "where is the OTHER endpoint, as seen
    % from me". By reciprocity this is the same function whether the
    % antenna is acting as TX or RX. So the right query direction is
    % determined by WHERE the antenna physically sits, not by its role:
    %
    %   angle(rx_array, tx) returns the direction at the GROUND site
    %   looking toward the AIR site = "where is air, viewed from ground"
    %   = (AZ, EL).
    %
    %   The reciprocal "where is ground, viewed from air" = (AZ+pi, -EL)
    %   = (AZ_tx, EL_tx).
    %
    % Therefore:
    %   - Any antenna on the AIRBORNE side (DL TX, UL RX) queries at
    %     (AZ_tx, EL_tx).
    %   - Any antenna on the GROUND   side (DL RX, UL TX) queries at
    %     (AZ,    EL).
    %
    % The earlier rule "TX queries flip / RX queries don't" was wrong in
    % general -- it happened to be right for DL only because air==TX
    % there. Locked by test_runRotation_ul_frame.
    AZ_tx = mod(AZ + single(pi), single(2*pi));
    EL_tx = -EL;

    % DL TX (airborne) -- airborne side -> flipped direction.
    G_tx_air = applyAntennaGain(ant.tx_air_pat, AZ_tx, EL_tx, tx_az_b, tx_el_b);

    % UL RX (airborne) -- airborne side -> flipped direction.
    G_rx_air = applyAntennaGain(ant.rx_air_pat, AZ_tx, EL_tx, rx_air_az_b, rx_air_el_b);

    % Per-RX type dispatch mask
    type_col = rx_table.type;
    if iscategorical(type_col)
        is_infantry = (type_col == 'infantry');
    else
        is_infantry = strcmp(string(type_col), "infantry");
    end
    is_infantry = is_infantry(:).';   % row vector of length n_rx

    % DL RX (ground) -- ground side -> direct direction.
    G_rx_gnd = zeros(Nf, n_rx, 'single');
    if any(is_infantry)
        G_rx_gnd(:,  is_infantry) = applyAntennaGain(ant.rx_gnd1_pat, ...
            AZ(:,  is_infantry), EL(:,  is_infantry), ...
            cfg.rx.infantry.boresight_az_rad, cfg.rx.infantry.boresight_el_rad);
    end
    if any(~is_infantry)
        G_rx_gnd(:, ~is_infantry) = applyAntennaGain(ant.rx_gnd2_pat, ...
            AZ(:, ~is_infantry), EL(:, ~is_infantry), ...
            cfg.rx.vehicular.boresight_az_rad, cfg.rx.vehicular.boresight_el_rad);
    end

    % UL TX (ground) -- ground side -> direct direction. Pattern is
    % shared across infantry/vehicular; boresight is per-type (both
    % default to zenith).
    G_tx_gnd = zeros(Nf, n_rx, 'single');
    if any(is_infantry)
        G_tx_gnd(:,  is_infantry) = applyAntennaGain(ant.tx_gnd_pat, ...
            AZ(:,  is_infantry), EL(:,  is_infantry), ...
            cfg.rx.infantry.boresight_az_rad, cfg.rx.infantry.boresight_el_rad);
    end
    if any(~is_infantry)
        G_tx_gnd(:, ~is_infantry) = applyAntennaGain(ant.tx_gnd_pat, ...
            AZ(:, ~is_infantry), EL(:, ~is_infantry), ...
            cfg.rx.vehicular.boresight_az_rad, cfg.rx.vehicular.boresight_el_rad);
    end

    % --- Link budgets + availability reduction ---------------------------
    Pt_air  = single(cfg.tx.power_air_dbm);
    Pt_gnd  = single(cfg.tx.power_gnd_dbm);
    fade_db = single(cfg.analysis.fade_margin_db);
    threshold   = single(cfg.analysis.threshold_dbm);
    target_frac = single(cfg.analysis.percentile) / single(100);

    Prx_dl = Pt_air + G_tx_air - PL + G_rx_gnd - fade_db;        % [Nf x n_rx]
    Prx_ul = Pt_gnd + G_tx_gnd - PL + G_rx_air - fade_db;        % [Nf x n_rx]

    rot.frac_above_dl    = single(mean(Prx_dl > threshold, 1)).';   % [n_rx x 1]
    rot.frac_above_ul    = single(mean(Prx_ul > threshold, 1)).';
    rot.available_dl     = rot.frac_above_dl >= target_frac;
    rot.available_ul     = rot.frac_above_ul >= target_frac;
    rot.available_bidi   = rot.available_dl & rot.available_ul;

    % Lower-tail Prx percentile (kept for downstream colormaps / debug)
    rot.prx_floor_dbm_dl = prctile(Prx_dl, 100 - cfg.analysis.percentile, 1).';
    rot.prx_floor_dbm_ul = prctile(Prx_ul, 100 - cfg.analysis.percentile, 1).';

    % Back-compat aliases: DL is the operationally limiting direction
    % the report and most existing callers expect to read.
    rot.frac_above    = rot.frac_above_dl;
    rot.available     = rot.available_dl;
    rot.prx_floor_dbm = rot.prx_floor_dbm_dl;

    rot.rx_table      = rx_table;
end
