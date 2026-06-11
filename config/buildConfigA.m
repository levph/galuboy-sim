function config = buildConfigA()
%BUILDCONFIGA  Part A (raytracing) configuration for galuboy-sim.
%
%   config = buildConfigA()
%
%   Part A is the online, MATLAB R2025a stage: it raytraces (SBR, R=2, D=0)
%   the aerial TX against terrain + buildings for every region and rotation
%   and stores raw path loss + geometric steering angles. It applies NO
%   antenna gains, powers, noise, or thresholds - those are Part B knobs and
%   live in buildConfigB.m.
%
%   This config therefore holds only what Part A actually uses:
%     - link geometry (frequency, aircraft altitude)
%     - antenna *mounting orientations* (boresights) - needed to compute the
%       off-boresight steering angle; these are platform geometry, not the
%       antenna pattern choice (which is Part B)
%     - receiver placement (counts, heights, discs)
%     - the raytracing model + terrain
%     - the trajectory / rotation sweep
%     - regions + IO
%

    config = struct();

    % ---- Link geometry ---------------------------------------------------
    config.tx.frequency_hz   = 4.2e9;     % 4 GHz (drives the raytracer)
    config.tx.altitude_m     = 4572;    % m AGL at trajectory centroid
    config.tx.altitude_msl_m = [];      % [] -> derive from terrain at centroid
    % Airframe TX mounting orientation (boresight): nadir-looking.
    config.tx.boresight_az_rad = 0;
    config.tx.boresight_el_rad = -pi/2;

    % ---- Receivers: 100 infantry / 100 vehicular per region --------------
    config.rx.placement_diameter_km = 5;   % fallback only; per-region discs
                                           %   come from defineRegions()

    config.rx.infantry.count            = 20;
    config.rx.infantry.height_m         = 1.5;
    config.rx.infantry.boresight_az_rad = 0;
    config.rx.infantry.boresight_el_rad = pi/2;     % zenith

    config.rx.vehicular.count            = 20;
    config.rx.vehicular.height_m         = 2.5;
    config.rx.vehicular.boresight_az_rad = 0;
    config.rx.vehicular.boresight_el_rad = pi/2;    % zenith

    % Reject RX samples that fall inside a building footprint (rejection
    % sampling against the region's footprints). Default on; set false to
    % allow RX anywhere within the placement disc.
    config.rx.reject_in_buildings = true;

    % Airframe RX (UL) mounting orientation: nadir-looking.
    config.rx.airborne.boresight_az_rad = 0;
    config.rx.airborne.boresight_el_rad = -pi/2;

    % ---- Propagation: SBR raytracing -------------------------------------
    config.propagation.model            = 'raytracing';
    config.propagation.rt_method        = 'sbr';
    config.propagation.max_reflections  = 2;     % R = 2
    config.propagation.max_diffractions = 2;     % D = 2 (enable diffraction / NLOS rescue)
    config.propagation.terrain_name     = 'galuboy_fabdem';
    config.propagation.use_terrain      = true;

    % ---- Flight (co-centred circle, constant bank) -----------------------
    % The figure-8 lemniscate is retired. The aircraft flies an arc of
    % max_rotation_deg around the region centroid on a circle of radius
    % circle_radius_m at constant altitude and a constant bank tilt. A circle is
    % rotationally symmetric, so there are no "rotations" - one pass per region,
    % with RX drawn once.
    %
    % User sets max_rotation_deg + delta_t_s + circle_radius_m (+ tilt); the
    % number of samples is DERIVED (circularFlightSampling): a coordinated turn
    % fixes airspeed v = sqrt(g*R*tan(tilt)), omega = v/R, and
    % num_samples = round(max_rotation / (omega*delta_t_s)).
    config.flight.max_rotation_deg = 360;        % arc swept per pass (deg)
    config.flight.delta_t_s        = 1;         % time between samples (s)
    config.flight.circle_radius_m  = 500;        % circle radius (m)
    config.flight.max_tilt_deg     = 23;         % constant bank tilt (deg)
    config.flight.tilt_direction   = 'outward';  % belly/nadir antenna boresight
                                                  %   tilts radially OUTWARD (a
                                                  %   coordinated inward bank points
                                                  %   the downward antenna outward)
    config.flight.rng_seed         = 20260601;   % reproducible RX placement

    % ---- Regions ---------------------------------------------------------
    config.regions = defineRegions();

    % ---- I/O -------------------------------------------------------------
    config.io.results_dir       = 'results';
    config.io.terrain_cache_dir = 'terrain_cache';
    config.io.buildings_dir     = 'resources/buildings';   % <region>.geojson
    config.io.terrain_dir       = 'resources/terrain';     % FABDEM .dt2 tiles

end
