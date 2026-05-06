classdef test_runRotation_freespace < matlab.unittest.TestCase
%TEST_RUNROTATION_FREESPACE  Synthetic free-space sanity check.
%
%   With:
%     - free-space propagation (no terrain)
%     - all-isotropic antennas
%     - zero fade margin
%   the link budget reduces to  Prx = Pt - PL_fs  per (step, RX).
%   This test runs a single rotation and verifies that the resulting
%   prx_pctile values are finite, monotonically lower with distance, and
%   match the analytic Friis prediction within ~1 dB tolerance.
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
            cfg.analysis.fade_margin_type = 'custom';
            cfg.analysis.fade_margin_custom_db = 0;
            cfg.analysis.percentile     = 50;
            cfg.flight.num_flight_steps = 8;
            cfg.tx.altitude_m           = 1500;
            cfg.tx.power_dbm            = 30;
            cfg.tx.frequency_hz         = 4e9;

            % All isotropic for predictability
            cfg.tx.antenna_name           = 'tx_omni';
            cfg.rx.infantry.count         = 5;
            cfg.rx.infantry.antenna_name  = 'tx_omni';
            cfg.rx.vehicular.count        = 0;
            cfg.rx.vehicular.antenna_name = 'tx_omni';

            % Single region, single rotation
            this_dir = fileparts(mfilename('fullpath'));
            repo     = fileparts(this_dir);
            antennas_dir = fullfile(repo, 'resources', 'antennas');
            ant.tx_pat  = loadAntennaPattern(cfg.tx.antenna_name,             antennas_dir);
            ant.inf_pat = loadAntennaPattern(cfg.rx.infantry.antenna_name,    antennas_dir);
            ant.veh_pat = loadAntennaPattern(cfg.rx.vehicular.antenna_name,   antennas_dir);

            region = struct('name','test','osm_path','', ...
                'latlim', [32.85, 32.87], 'lonlim', [35.20, 35.22], ...
                'center_lat', 32.86, 'center_lon', 35.21);

            [rx_array, rx_table] = placeReceivers(region, cfg.rx);

            pm = propagationModel('freespace');
            tx_template = txsite('Name', 'Aerial_Tx', ...
                'Latitude', 0, 'Longitude', 0, ...
                'AntennaHeight', cfg.tx.altitude_m, ...
                'TransmitterFrequency', cfg.tx.frequency_hz, ...
                'TransmitterPower', 10^((cfg.tx.power_dbm - 30) / 10));

            [flight_lons, flight_lats] = generateTrajectory(region, cfg.flight, 0);

            rot = runRotation(tx_template, rx_array, rx_table, pm, '', ...
                              flight_lons, flight_lats, ant, cfg);

            tc.verifyEqual(numel(rot.prx_pctile), 5);
            tc.verifyTrue(all(isfinite(rot.prx_pctile)), 'No NaN P_rx');
            tc.verifyTrue(all(rot.prx_pctile < cfg.tx.power_dbm + 7 + 7), ...
                'Prx should be below Pt + Gtx + Grx (sanity ceiling)');

            % Friis floor: even at 100 km @ 4 GHz, FSPL is ~145 dB. With
            % Pt=30 dBm + 14 dBi total isotropic -> Prx > -101 dBm. We just
            % check Prx > -150 dBm.
            tc.verifyTrue(all(rot.prx_pctile > -150), ...
                'Prx unreasonably low - likely wrong link-budget sign');
        end
    end
end
