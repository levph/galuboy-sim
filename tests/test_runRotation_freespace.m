classdef test_runRotation_freespace < matlab.unittest.TestCase
%TEST_RUNROTATION_FREESPACE  Synthetic free-space sanity check.
%
%   With:
%     - free-space propagation (no terrain)
%     - all-isotropic antennas
%     - zero fade margin
%   the link budget reduces to  Prx = Pt - PL_fs  per (step, RX).
%   This test runs a single rotation and verifies that the resulting
%   prx_floor_dbm values are finite and match the analytic Friis
%   prediction within a generous tolerance.
%
%   Skipped when the Communications / Phased-Array stack is missing.

    methods (TestClassSetup)
        function checkToolboxes(tc)
            tc.assumeTrue(exist('propagationModel', 'file') == 2, ...
                'propagationModel not available - skipping');
            tc.assumeTrue(exist('txsite', 'file') == 2, ...
                'txsite not available - skipping');
        end
    end

    methods (Test)
        function freeSpaceMatchesFriis(tc)
            cfg = buildConfig();
            cfg.propagation.use_terrain = false;
            cfg.propagation.model       = 'freespace';
            cfg.analysis.fade_margin_db = 0;
            cfg.analysis.percentile     = 50;
            cfg.flight.num_flight_steps = 8;
            cfg.tx.altitude_m           = 1500;
            cfg.tx.altitude_msl_m       = [];
            cfg.tx.power_air_dbm        = 30;
            cfg.tx.power_gnd_dbm        = 30;
            cfg.tx.frequency_hz         = 4e9;

            % All isotropic for predictability (tx_omni is 7 dBi isotropic
            % per the legacy fixture). The DL link path uses tx_air_pat +
            % rx_gnd1_pat (infantry) / rx_gnd2_pat (vehicular).
            cfg.tx.airborne.antenna_name      = 'tx_omni';
            cfg.tx.ground.antenna_name        = 'tx_omni';
            cfg.rx.airborne.antenna_name_ul   = 'tx_omni';
            cfg.rx.infantry.count             = 5;
            cfg.rx.infantry.antenna_name_dl   = 'tx_omni';
            cfg.rx.vehicular.count            = 0;
            cfg.rx.vehicular.antenna_name_dl  = 'tx_omni';

            % Single region, single rotation
            this_dir = fileparts(mfilename('fullpath'));
            repo     = fileparts(this_dir);
            antennas_dir = fullfile(repo, 'resources', 'antennas');
            ant.tx_air_pat  = loadAntennaPattern(cfg.tx.airborne.antenna_name,         antennas_dir);
            ant.tx_gnd_pat  = loadAntennaPattern(cfg.tx.ground.antenna_name,           antennas_dir);
            ant.rx_air_pat  = loadAntennaPattern(cfg.rx.airborne.antenna_name_ul,      antennas_dir);
            ant.rx_gnd1_pat = loadAntennaPattern(cfg.rx.infantry.antenna_name_dl,      antennas_dir);
            ant.rx_gnd2_pat = loadAntennaPattern(cfg.rx.vehicular.antenna_name_dl,     antennas_dir);

            region = struct('name','test','osm_path','', ...
                'latlim', [32.85, 32.87], 'lonlim', [35.20, 35.22], ...
                'center_lat', 32.86, 'center_lon', 35.21);

            [rx_array, rx_table] = placeReceivers(region, cfg.rx);

            pm = propagationModel('freespace');
            tx_template = txsite('Name', 'Aerial_Tx', ...
                'Latitude', 0, 'Longitude', 0, ...
                'AntennaHeight', cfg.tx.altitude_m, ...
                'TransmitterFrequency', cfg.tx.frequency_hz, ...
                'TransmitterPower', 10^((cfg.tx.power_air_dbm - 30) / 10));

            [flight_lons, flight_lats] = generateTrajectory(region, cfg.flight, 0);

            rot = runRotation(tx_template, rx_array, rx_table, pm, '', ...
                              flight_lons, flight_lats, ant, cfg);

            tc.verifyEqual(numel(rot.prx_floor_dbm_dl), 5);
            tc.verifyEqual(numel(rot.frac_above_dl),    5);
            tc.verifyTrue(all(isfinite(rot.prx_floor_dbm_dl)), 'No NaN P_rx_dl');
            tc.verifyTrue(all(rot.prx_floor_dbm_dl < cfg.tx.power_air_dbm + 7 + 7), ...
                'Prx_dl should be below Pt_air + Gtx + Grx (sanity ceiling)');

            % Friis floor: even at 100 km @ 4 GHz, FSPL is ~145 dB.
            tc.verifyTrue(all(rot.prx_floor_dbm_dl > -150), ...
                'Prx unreasonably low - likely wrong link-budget sign');
            tc.verifyTrue(all(rot.frac_above_dl >= 0 & rot.frac_above_dl <= 1), ...
                'frac_above_dl must be in [0,1]');

            % Back-compat aliases
            tc.verifyEqual(rot.prx_floor_dbm, rot.prx_floor_dbm_dl);
            tc.verifyEqual(rot.frac_above,    rot.frac_above_dl);
            tc.verifyEqual(rot.available,     rot.available_dl);
        end
    end
end
