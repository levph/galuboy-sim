function config = buildConfig()
%BUILDCONFIG  Default simulation configuration for galuboy-sim v4.
%
%   config = buildConfig()
%
%   Returns a single struct with grouped subfields. Override individual
%   fields after calling buildConfig() rather than editing this file for
%   one-off experiments.
%
%   Top-level fields:
%     tx           Aerial transmitter parameters
%     rx.infantry  Ground RX type 1 (handheld)
%     rx.vehicular Ground RX type 2 (roof whip)
%     propagation  Propagation model + terrain options
%     flight       Trajectory and rotation sweep
%     analysis     Threshold, percentile, fade margin
%     regions      Struct array - one entry per map region to simulate
%     parallel     Parpool toggle and worker count
%     io           Output paths
%     viz          Visualization toggles
%     hist         Range-histogram parameters
%
%   Antenna patterns are loaded from resources/antennas/<name>.csv at
%   runtime. RX height defaults: 1.5 m infantry / 2.5 m vehicular.

    config = struct();

    % ---- Transmitters (DL: airborne -> ground.  UL: ground -> airborne) --
    % Path loss is reciprocal so the same PL is reused for both directions.
    config.tx.frequency_hz     = 4e9;            % Hz (shared)
    config.tx.power_air_dbm    = 47;             % aerial TX power (DL)
    config.tx.power_gnd_dbm    = 30;             % ground  TX power (UL)
    config.tx.altitude_m       = 1500;           % m AGL at trajectory centroid
                                                  %   (used as fallback when
                                                  %    altitude_msl_m is empty)
    config.tx.altitude_msl_m   = [];             % m MSL. [] -> derive from altitude_m
                                                  %   using terrain at the
                                                  %   trajectory centroid.
    % Airborne TX antenna (DL) + ground TX antenna (UL, shared by all yk"tzim).
    config.tx.airborne.antenna_name = 'airborne_tx';
    config.tx.ground.antenna_name   = 'ground_tx';
    config.tx.boresight_az_rad = 0;              % azimuth (rad); aerial TX body frame
    config.tx.boresight_el_rad = -pi/2;          % elevation (rad); -pi/2 = nadir

    % ---- Receivers (two ground device types + one airborne RX) ----------
    % Per-region RX-placement disk diameters live on each cfg.regions entry
    % (urban_suburban = 5 km, others = 10 km). This default is only used as
    % a fallback if a region struct doesn't carry placement_diameter_km.
    config.rx.placement_diameter_km     = 5;     % km (fallback only)

    config.rx.infantry.count            = 40;
    config.rx.infantry.height_m         = 1.5;
    config.rx.infantry.antenna_name_dl  = 'ground1_rx';   % 8-element patch (DL receiver)
    config.rx.infantry.boresight_az_rad = 0;
    config.rx.infantry.boresight_el_rad = pi/2;           % zenith

    config.rx.vehicular.count            = 20;
    config.rx.vehicular.height_m         = 2.5;
    config.rx.vehicular.antenna_name_dl  = 'ground2_rx';  % 8-element patch (DL receiver)
    config.rx.vehicular.boresight_az_rad = 0;
    config.rx.vehicular.boresight_el_rad = pi/2;

    % Airborne RX (UL): 16-element patch on the aircraft, boresight = nadir
    config.rx.airborne.antenna_name_ul    = 'airborne_rx';
    config.rx.airborne.boresight_az_rad   = 0;
    config.rx.airborne.boresight_el_rad   = -pi/2;        % nadir

    % Patch-array generator parameters (used by generatePatchArrayCsv.m and
    % surfaced in the Antennas tab in the UI for one-click regeneration).
    config.rx.patch.fov_in_deg  = 50;
    config.rx.patch.fov_out_deg = 70;
    config.rx.patch.floor_db    = -20;

    % ---- Propagation -----------------------------------------------------
    config.propagation.model                 = 'longley-rice';
    config.propagation.time_variability      = 0.9;
    config.propagation.situation_variability = 0.9;
    config.propagation.bandwidth_hz          = 25e3;     % Hz
    config.propagation.terrain_name          = 'galuboy_terrain';
    config.propagation.use_terrain           = true;     % false -> default GMTED2010

    % ---- Flight (trajectory + rotation sweep) ----------------------------
    config.flight.num_flight_steps     = 100;    % samples along trajectory
    config.flight.step_degrees         = 10;     % rotation step in deg
    config.flight.num_rotations        = floor(180 / config.flight.step_degrees);
    config.flight.lemniscate_scale_deg = 0.005;  % fallback if length_m is empty
    config.flight.lemniscate_length_m  = 1000;   % full E-W extent of figure-8 (m); [] -> use scale_deg
    config.flight.max_tilt_deg         = 23;     % peak bank angle in turns (deg)

    % ---- Analysis --------------------------------------------------------
    config.analysis.threshold_dbm        = -95;          % MDS in dBm
    config.analysis.percentile           = 99;           % availability percentile
    config.analysis.fade_margin_db       = 10;           % user-supplied fade margin (dB)

    % ---- Regions (struct array; 7 northern-Israel regions) ---------------
    %   name                    short ident used in logs and result files
    %   osm_path                absolute or repo-relative .osm file (empty => no buildings)
    %   latlim_override         [lo, hi] in degrees; [] means read from OSM
    %   lonlim_override         [lo, hi] in degrees; [] means read from OSM
    %   placement_diameter_km   per-region RX disk diameter (km)
    %   category                'urban_suburban' or 'other' - drives the
    %                            two-aggregate split in histograms / CCDFs
    % Urban/Suburban regions (~5x5 km bboxes; OSM provides building polygons).
    config.regions = struct('name','urban_suburban_01', ...
        'osm_path','resources/osm/urban_suburban_01.osm', ...
        'latlim_override',[32.99607, 33.04314], ...
        'lonlim_override',[35.09591, 35.15205], ...
        'placement_diameter_km', 5, ...
        'category', 'urban_suburban');

    config.regions(end+1) = struct('name','urban_suburban_02', ...
        'osm_path','resources/osm/urban_suburban_02.osm', ...
        'latlim_override',[33.24621, 33.29188], ...
        'lonlim_override',[35.19915, 35.25377], ...
        'placement_diameter_km', 5, ...
        'category', 'urban_suburban');

    config.regions(end+1) = struct('name','urban_suburban_03', ...
        'osm_path','resources/osm/urban_suburban_03.osm', ...
        'latlim_override',[33.17575, 33.22580], ...
        'lonlim_override',[35.58279, 35.64261], ...
        'placement_diameter_km', 5, ...
        'category', 'urban_suburban');

    % Forest region (~10x10 km)
    config.regions(end+1) = struct('name','forest_01', ...
        'osm_path','', ...
        'latlim_override',[32.95132, 33.04470], ...
        'lonlim_override',[35.34572, 35.45705], ...
        'placement_diameter_km', 10, ...
        'category', 'other');

    % Mountain regions (~10x10 km)
    config.regions(end+1) = struct('name','mountain_01', ...
        'osm_path','', ...
        'latlim_override',[33.35397, 33.46077], ...
        'lonlim_override',[35.56332, 35.69125], ...
        'placement_diameter_km', 10, ...
        'category', 'other');

    config.regions(end+1) = struct('name','mountain_02', ...
        'osm_path','', ...
        'latlim_override',[33.44502, 33.54904], ...
        'lonlim_override',[35.79151, 35.91625], ...
        'placement_diameter_km', 10, ...
        'category', 'other');

    % Rural/Open region (~10x10 km)
    config.regions(end+1) = struct('name','rural_open_01', ...
        'osm_path','', ...
        'latlim_override',[33.46139, 33.54583], ...
        'lonlim_override',[35.37238, 35.47365], ...
        'placement_diameter_km', 10, ...
        'category', 'other');

    % ---- Parallel pool ---------------------------------------------------
    config.parallel.enabled     = true;
    config.parallel.num_workers = [];   % [] = MATLAB default

    % ---- I/O -------------------------------------------------------------
    config.io.results_dir       = 'results';
    config.io.terrain_cache_dir = 'terrain_cache';

    % ---- Visualization ---------------------------------------------------
    config.viz.show_siteviewer    = false;   % true -> 3D map + buildings overlay

    % ---- Range histogram -------------------------------------------------
    config.hist.n_bins              = 50;
    config.hist.min_samples_per_bin = 1;

    % ---- Availability CCDF -----------------------------------------------
    config.ccdf.min_samples = 20;   % mask tail bins with fewer samples
    config.ccdf.bin_km      = 0.2;  % distance bin width for the CCDF curve

end
