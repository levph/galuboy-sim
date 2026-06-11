function [sv, tx, rxs] = viewRegion(region_sel, varargin)
%VIEWREGION  Open a visible Site Viewer to inspect a region's propagation scene.
%
%   sv = viewRegion(region_sel)
%   [sv, tx, rxs] = viewRegion(region_sel, 'Name', value, ...)
%
%   Builds the same scene Part A raytraces against - the shared FABDEM terrain
%   plus the region's building footprints - and drops in an aerial TX at one
%   flight waypoint, a few random ground RXs, and a ring of markers tracing the
%   RX-placement disc. Sets an oblique, terrain-aware camera so the geometry is
%   readable (TX up high, RXs on the ground).
%
%   region_sel : region index (1..N) OR region name (e.g. "mountain_02").
%
%   Name/value options:
%     'Config'      config struct (default buildConfigA())
%     'Infantry'    # infantry RXs to drop (default 3)
%     'Vehicular'   # vehicular RXs to drop (default 2)
%     'Waypoint'    flight-step index for the TX (default 1)
%     'Rays'        raytrace TX->RXs and draw the rays (default false)
%     'Boresight'   draw the aircraft outward-tilt boresight arrow (default = Rays)
%     'Reflections' SBR max reflections when Rays/Boresight (default 2)
%
%   Returns the Site Viewer handle plus the TX and RX sites so you can poke at
%   them (e.g. raytrace(tx, rxs, pm, 'Map', sv) yourself). Requires R2025a.
%
%   Examples:
%     viewRegion(5)                         % mountain_01, scene only
%     viewRegion("urban_suburban_02", 'Rays', true)
%     [sv,tx,rxs] = viewRegion(2, 'Infantry', 10, 'Vehicular', 10);

    p = inputParser;
    p.addParameter('Config', []);
    p.addParameter('Infantry', 3);
    p.addParameter('Vehicular', 2);
    p.addParameter('Waypoint', 1);
    p.addParameter('Rays', false);
    p.addParameter('Boresight', []);
    p.addParameter('Reflections', 2);
    p.parse(varargin{:});
    o = p.Results;
    if isempty(o.Config), o.Config = buildConfigA(); end
    if isempty(o.Boresight), o.Boresight = o.Rays; end
    cfg = o.Config;

    repo = fileparts(fileparts(mfilename('fullpath')));
    tn   = cfg.propagation.terrain_name;

    % --- Resolve which region ---------------------------------------------
    ridx = resolveSelector(region_sel, cfg.regions);
    region = resolveRegion(cfg.regions(ridx), repo);

    % --- Register terrain (cached) + build the visible scene --------------
    tiles = discoverTerrainTiles(fullfile(repo, cfg.io.terrain_dir), ...
        [region.latlim(1)-0.1, region.latlim(2)+0.1], ...
        [region.lonlim(1)-0.1, region.lonlim(2)+0.1]);
    setupTerrain(tn, tiles);
    sv = setupRegionScene(region, cfg, true);   % visible

    % --- Aerial TX at one flight waypoint (proper AGL) --------------------
    [lons, lats] = generateCircularTrajectory(region, cfg.flight);
    clat = region.center_lat; clon = region.center_lon;
    target_msl = elevation(txsite('Latitude',clat,'Longitude',clon,'AntennaHeight',0), 'Map', tn) ...
               + cfg.tx.altitude_m;
    k = min(max(1, o.Waypoint), numel(lons));
    ter_k = elevation(txsite('Latitude',lats(k),'Longitude',lons(k),'AntennaHeight',0), 'Map', tn);
    tx = txsite('Name','Aerial_TX', 'Latitude',lats(k), 'Longitude',lons(k), ...
                'AntennaHeight',target_msl-ter_k, 'TransmitterFrequency',cfg.tx.frequency_hz);
    show(tx, 'Map', sv);

    % --- A few random RXs in the placement disc ---------------------------
    rc = cfg.rx;
    if ~isempty(region.placement_diameter_km)
        rc.placement_diameter_km = region.placement_diameter_km;
    end
    rc.infantry.count = o.Infantry; rc.vehicular.count = o.Vehicular;
    rng(cfg.flight.rng_seed + ridx);
    [rxs, ~] = placeReceivers(region, rc, {});
    show(rxs, 'Map', sv);

    % --- Placement-disc boundary ring (ground markers) --------------------
    Rb = (region.placement_diameter_km/2) * 1000;
    phi = linspace(0, 2*pi, 73); phi(end) = [];
    mlat = 111320; mlon = 111320 * cosd(clat);
    blat = clat + (Rb*sin(phi))/mlat;
    blon = clon + (Rb*cos(phi))/mlon;
    ring = txsite('Name', "b"+string(1:numel(phi)), 'Latitude',blat, 'Longitude',blon, 'AntennaHeight',0);
    show(ring, 'Map', sv, 'ShowAntennaHeight', false);

    % --- Optional rays + outward boresight arrow --------------------------
    if o.Rays || o.Boresight
        pm = propagationModel('raytracing', 'Method','sbr', ...
            'MaxNumReflections',o.Reflections, 'MaxNumDiffractions',0);
        if o.Rays
            raytrace(tx, rxs, pm, 'Map', sv);
        end
        if o.Boresight
            drawBoresight(sv, tx, region, cfg);
        end
    end

    % --- Oblique, terrain-aware camera ------------------------------------
    back_deg = (Rb*1.1 + 800) / 111320;          % pull camera south, scaled to disc
    cam_lat  = tx.Latitude - back_deg;
    ter_cam  = elevation(txsite('Latitude',cam_lat,'Longitude',tx.Longitude,'AntennaHeight',0), 'Map', tn);
    campos(sv, cam_lat, tx.Longitude);
    camheight(sv, ter_cam + (Rb*0.4 + 700));      % above local terrain
    camheading(sv, 0);                            % face north toward scene
    campitch(sv, 7);

    fprintf('%s  center=%.5f,%.5f  disc R=%.0f m  terrain@centroid=%.0f m  TX AGL=%.0f m\n', ...
        region.name, clat, clon, Rb, target_msl-cfg.tx.altitude_m, target_msl-ter_k);
end

% -------------------------------------------------------------------------
function ridx = resolveSelector(sel, regions)
    if isnumeric(sel)
        ridx = sel;
        assert(ridx>=1 && ridx<=numel(regions), 'region index %d out of range', ridx);
    else
        names = string({regions.name});
        ridx = find(names == string(sel), 1);
        assert(~isempty(ridx), 'no region named "%s"', string(sel));
    end
end

% -------------------------------------------------------------------------
function drawBoresight(sv, tx, region, cfg)
% Line from the TX along the aircraft's outward-tilted nadir boresight.
    clat = region.center_lat; clon = region.center_lon;
    mlat = 111320; mlon = 111320*cosd(clat);
    e = (tx.Longitude-clon)*mlon; n = (tx.Latitude-clat)*mlat;
    az_out = mod(atan2(e, n), 2*pi);
    el_out = -pi/2 + deg2rad(cfg.flight.max_tilt_deg);
    bE = cos(el_out)*sin(az_out); bN = cos(el_out)*cos(az_out); bU = sin(el_out);
    L = 2500;
    tip = rxsite('Name','boresight_tip', ...
        'Latitude',  tx.Latitude  + (bN*L)/mlat, ...
        'Longitude', tx.Longitude + (bE*L)/mlon, ...
        'AntennaHeight', tx.AntennaHeight + bU*L);
    los(tx, tip, 'Map', sv);
end
