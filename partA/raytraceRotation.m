function rot = raytraceRotation(region, cfg, sv, pm)
%RAYTRACEROTATION  Raytrace one circular pass: PL + steering angles per (step, rx).
%
%   rot = raytraceRotation(region, cfg, sv, pm)
%
%   Flies the constant-bank circle for one region and, at every flight step,
%   raytraces the aerial TX against all ground RX (terrain + buildings via the
%   Site Viewer sv) and records:
%     - the combined (coherent multipath) path loss per link
%     - the off-boresight STEERING ANGLE on each side:
%         steer_gnd : ground RX/TX boresight = zenith
%         steer_air : aircraft boresight = nadir tilted OUTWARD by max_tilt_deg,
%                     azimuth = outward radial bearing (centre -> aircraft)
%   No antenna gains, powers, noise, or availability - those are Part B. DL and
%   UL share this geometry (PL reciprocal; same two steering angles), so one
%   row per (step, rx) serves both directions.
%
%   Sequential over flight steps - the Site Viewer is single-instance, so no
%   parfor here.
%
%   Inputs:
%     region  resolved region struct (resolveRegion)
%     cfg     buildConfigA() config
%     sv      Site Viewer with terrain + this region's buildings (setupRegionScene)
%     pm      raytracing propagationModel (SBR)
%
%   Output struct rot (Nf = number of flight steps, n_rx receivers):
%     pl_db          [Nf x n_rx] single  combined path loss (Inf = outage)
%     steer_gnd_deg  [Nf x n_rx] single  ground-side off-boresight angle (deg)
%     steer_air_deg  [Nf x n_rx] single  air-side  off-boresight angle (deg)
%     rx_table       table (lon, lat, type, height_m, ...)
%     dist_m         [1 x n_rx] single   RX distance from region centroid (m)
%     tx_lon/tx_lat  [1 x Nf]   single   TX waypoints (deg)
%     tx_alt_msl_m   [1 x Nf]   single   TX MSL altitude per step (m)
%     headings_rad   [1 x Nf]            flight heading
%     region_name    char

    tn = cfg.propagation.terrain_name;
    log = @(varargin) fprintf('      %s\n', sprintf(varargin{:}));

    % --- Trajectory -------------------------------------------------------
    [lons, lats, headings, ~] = generateCircularTrajectory(region, cfg.flight);
    Nf = numel(lons);
    log('trajectory: %d flight steps on %.0f m circle', Nf, cfg.flight.circle_radius_m);

    % --- TX altitude: constant MSL -> AGL per step ------------------------
    clat = region.center_lat;  clon = region.center_lon;
    probe = txsite('Latitude', lats, 'Longitude', lons, 'AntennaHeight', zeros(1, Nf));
    terrain_at_tx = elevation(probe, 'Map', tn);          % 1 x Nf (m MSL)
    ter_centroid = elevation(txsite('Latitude', clat, 'Longitude', clon, 'AntennaHeight', 0), 'Map', tn);
    if ~isempty(cfg.tx.altitude_msl_m)
        target_msl = cfg.tx.altitude_msl_m;
    else
        target_msl = ter_centroid + cfg.tx.altitude_m;
    end
    agl = target_msl - terrain_at_tx(:).';                % 1 x Nf
    if any(agl < 50)
        error('raytraceRotation:lowAGL', ...
              'TX AGL drops below 50 m (min %.1f m) over %s terrain.', min(agl), region.name);
    end

    % --- Receivers (placed once; per-region disc) -------------------------
    rx_cfg = cfg.rx;
    if isfield(region, 'placement_diameter_km') && ~isempty(region.placement_diameter_km)
        rx_cfg.placement_diameter_km = region.placement_diameter_km;
    end
    % Reject RX that land inside a building footprint (config-gated). Build the
    % fast point-in-buildings tester ONCE per region from the same GeoJSON the
    % Site Viewer scene uses, then hand it to placeReceivers.
    reject = isfield(cfg.rx, 'reject_in_buildings') && cfg.rx.reject_in_buildings;
    if reject
        repo_root = fileparts(fileparts(mfilename('fullpath')));   % partA/ -> repo
        geojson   = fullfile(repo_root, cfg.io.buildings_dir, [region.name '.geojson']);
        bldg_tester = buildingRejectMask(geojson);
    else
        bldg_tester = {};   % {} disables the check
    end
    [rx_array, rx_table] = placeReceivers(region, rx_cfg, bldg_tester);
    n_rx = numel(rx_array);
    log('placed %d receivers (reject-in-buildings=%d)', n_rx, reject);
    log('TX altitude: %.0f-%.0f m AGL over terrain', min(agl), max(agl));

    % --- Aircraft (air) boresight per step: outward-tilted nadir ----------
    mlat = 111320;  mlon = 111320 * cosd(clat);
    e_off = (lons - clon) * mlon;     % east offset of aircraft from centre (m)
    n_off = (lats - clat) * mlat;     % north offset
    az_out = mod(atan2(e_off, n_off), 2*pi);              % 1 x Nf outward radial bearing
    el_out = -pi/2 + deg2rad(cfg.flight.max_tilt_deg);    % nadir raised outward by tilt

    % Ground boresight = zenith (shared by infantry + vehicular)
    g_az = 0;  g_el = pi/2;

    % --- Per-step raytrace ------------------------------------------------
    pl_db         = zeros(Nf, n_rx, 'single');
    steer_gnd_deg = zeros(Nf, n_rx, 'single');
    steer_air_deg = zeros(Nf, n_rx, 'single');
    log('raytracing %d steps x %d RX (SBR R=%d D=%d)...', Nf, n_rx, ...
        cfg.propagation.max_reflections, cfg.propagation.max_diffractions);
    t_loop = tic;
    for i = 1:Nf
        tx = txsite('Latitude', lats(i), 'Longitude', lons(i), ...
                    'AntennaHeight', agl(i), 'TransmitterFrequency', cfg.tx.frequency_hz);

        % Path loss: coherent combine of the per-ray fields (needs raytrace's
        % PathLoss + PhaseShift; pathloss() returns no phase).
        rays = raytrace(tx, rx_array, pm, 'Map', sv);
        pl_db(i, :) = single(combineRayPathloss(rays));

        % Geometry: direction at each RX (ground) toward TX (air), world compass.
        [az, el] = angle(rx_array, tx, 'Map', tn);
        az_g = mod(pi/2 - deg2rad(az(:).'), 2*pi);   % compass (CW from N)
        el_g = deg2rad(el(:).');

        % Ground steering: link vs zenith boresight.
        steer_gnd_deg(i, :) = single(rad2deg(offBoresightAngle(az_g, el_g, g_az, g_el)));

        % Air steering: reciprocal direction (air->ground) vs outward boresight.
        az_air = mod(az_g + pi, 2*pi);
        el_air = -el_g;
        steer_air_deg(i, :) = single(rad2deg(offBoresightAngle(az_air, el_air, az_out(i), el_out)));

        % --- progress / ETA (print first step, then every ~10%) ----------
        if i == 1 || mod(i, max(1, round(Nf/10))) == 0 || i == Nf
            el_s   = toc(t_loop);
            per    = el_s / i;
            n_out  = nnz(~isfinite(pl_db(i, :)));
            log('  step %3d/%-3d  %5.1fs elapsed  ~%4.1fs/step  ETA %4.0fs  (%d/%d RX outage)', ...
                i, Nf, el_s, per, per * (Nf - i), n_out, n_rx);
        end
    end
    log('raytrace loop done in %.1f s (%.1f s/step)', toc(t_loop), toc(t_loop)/Nf);

    % --- Metadata ---------------------------------------------------------
    rot.pl_db         = pl_db;
    rot.steer_gnd_deg = steer_gnd_deg;
    rot.steer_air_deg = steer_air_deg;
    rot.rx_table      = rx_table;
    rot.dist_m        = single(distanceFromCentroid(region.lonlim, region.latlim, ...
                                                    [rx_table.lon, rx_table.lat]));
    rot.tx_lon        = single(lons);
    rot.tx_lat        = single(lats);
    rot.tx_alt_msl_m  = single(agl + terrain_at_tx(:).');
    rot.terrain_centroid_m = single(ter_centroid);   % terrain elevation at region centroid
    rot.headings_rad  = headings;
    rot.region_name   = region.name;
end
