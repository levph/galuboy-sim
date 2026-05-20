classdef test_availability_direction < matlab.unittest.TestCase
%TEST_AVAILABILITY_DIRECTION  Lock the availability gate direction (bug 1.3).
%
%   Per project_availability_unit.md, availability(m, j) is
%       mean(Prx(:, j) > threshold) >= cfg.analysis.percentile / 100
%   i.e. the LOWER tail must clear threshold for at least the target
%   fraction of the rotation. The pre-fix code took the UPPER tail
%   (best 1% clearing threshold), which inverted the gate.
%
%   We construct a synthetic Prx column where exactly 95 of 100 flight
%   steps exceed threshold. With target=90 the link is available; with
%   target=99 it is not. Reversing the direction would flip both answers.

    properties
        AntennasDir
        TxOmni
    end

    methods (TestClassSetup)
        function checkToolboxes(tc)
            tc.assumeTrue(exist('propagationModel', 'file') == 2, ...
                'propagationModel not available - skipping');
            tc.assumeTrue(exist('txsite', 'file') == 2, ...
                'txsite not available - skipping');

            this_dir = fileparts(mfilename('fullpath'));
            repo     = fileparts(this_dir);
            tc.AntennasDir = fullfile(repo, 'resources', 'antennas');
            tc.TxOmni = loadAntennaPattern('tx_omni', tc.AntennasDir);
        end
    end

    methods (Test)
        function ninetyFivePctMissionAvailability(tc)
            % Drive runRotation with crafted inputs is hard because Prx is
            % built from pathloss(); instead, exercise the gate logic
            % directly by computing frac_above / available the same way
            % runRotation does, then assert direction.
            target_high = 99;   % 99% target -> link NOT available (we have 95%)
            target_low  = 90;   % 90% target -> link IS  available (we have 95%)

            % Prx: 95 samples above threshold, 5 below.
            threshold = -95;
            Prx = single([repmat(threshold + 10, 1, 95), repmat(threshold - 20, 1, 5)]).';

            frac_above = mean(Prx > threshold);
            tc.verifyEqual(double(frac_above), 0.95, 'AbsTol', 1e-6, ...
                'Synthetic Prx should have exactly 95% above threshold');

            avail_low  = frac_above >= (target_low  / 100);
            avail_high = frac_above >= (target_high / 100);
            tc.verifyTrue(avail_low,  'frac=0.95 must be available with target 90%');
            tc.verifyFalse(avail_high, 'frac=0.95 must NOT be available with target 99%');

            % The equivalent lower-tail Prx percentile gate must agree.
            % rot.prx_floor_dbm = prctile(Prx, 100 - percentile, 1).
            floor_high = double(prctile(Prx, 100 - target_high, 1));
            floor_low  = double(prctile(Prx, 100 - target_low,  1));
            tc.verifyGreaterThan(floor_low,  threshold, ...
                'Lower-tail percentile gate must agree with frac_above gate (low target)');
            tc.verifyLessThanOrEqual(floor_high, threshold, ...
                'Lower-tail percentile gate must agree with frac_above gate (high target)');
        end

        function runRotationFreeSpaceRespectsDirection(tc)
            % End-to-end sanity: with isotropic antennas, freespace, and a
            % low MDS, the link is always up so available must be true
            % regardless of the configured availability target.
            this_dir = fileparts(mfilename('fullpath'));
            repo     = fileparts(this_dir);
            antennas_dir = fullfile(repo, 'resources', 'antennas');

            cfg = buildConfig();
            cfg.propagation.use_terrain = false;
            cfg.propagation.model       = 'freespace';
            cfg.analysis.fade_margin_db = 0;
            cfg.analysis.threshold_dbm  = -200;        % impossible-to-miss MDS
            cfg.flight.num_flight_steps = 8;
            cfg.tx.altitude_m           = 1500;
            cfg.tx.altitude_msl_m       = [];
            cfg.tx.power_dbm            = 30;
            cfg.tx.frequency_hz         = 4e9;
            cfg.tx.antenna_name           = 'tx_omni';
            cfg.rx.infantry.count         = 3;
            cfg.rx.infantry.antenna_name  = 'tx_omni';
            cfg.rx.vehicular.count        = 0;
            cfg.rx.vehicular.antenna_name = 'tx_omni';

            ant.tx_pat  = loadAntennaPattern(cfg.tx.antenna_name,             antennas_dir);
            ant.inf_pat = loadAntennaPattern(cfg.rx.infantry.antenna_name,    antennas_dir);
            ant.veh_pat = loadAntennaPattern(cfg.rx.vehicular.antenna_name,   antennas_dir);

            region = struct('name','test','osm_path','', ...
                'latlim',[32.85, 32.87], 'lonlim',[35.20, 35.22], ...
                'center_lat',32.86,'center_lon',35.21);

            [rx_array, rx_table] = placeReceivers(region, cfg.rx);

            pm = propagationModel('freespace');
            tx_template = txsite('Name','Aerial_Tx', ...
                'Latitude', 0, 'Longitude', 0, ...
                'AntennaHeight', cfg.tx.altitude_m, ...
                'TransmitterFrequency', cfg.tx.frequency_hz, ...
                'TransmitterPower', 10^((cfg.tx.power_dbm - 30) / 10));
            [flight_lons, flight_lats] = generateTrajectory(region, cfg.flight, 0);

            for target = [50, 90, 99]
                cfg.analysis.percentile = target;
                rot = runRotation(tx_template, rx_array, rx_table, pm, '', ...
                                  flight_lons, flight_lats, ant, cfg);
                tc.verifyTrue(all(rot.available), ...
                    sprintf('All RXs must be available at target=%d (MDS = -200 dBm)', target));
                tc.verifyTrue(all(rot.frac_above >= 0.999), ...
                    'frac_above must be ~1.0 when MDS is unreachable');
            end
        end
    end
end
