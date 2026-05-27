classdef test_runRotation_ul_frame < matlab.unittest.TestCase
%TEST_RUNROTATION_UL_FRAME  Lock the UL antenna-frame convention.
%
%   The DL/UL link budgets share PL and fade, and the only asymmetry
%   between the two directions is the TX power. So in a symmetric setup
%   where:
%     - air-side TX antenna == air-side RX antenna (same pattern + boresight)
%     - gnd-side TX antenna == gnd-side RX antenna (same pattern + boresight)
%     - Pt_air == Pt_gnd
%   the two directions must produce IDENTICAL Prx per (step, RX). Any
%   frame-swap bug (e.g. UL TX using (AZ+pi,-EL) instead of (AZ,EL), or
%   UL RX using (AZ,EL) instead of (AZ+pi,-EL)) breaks this equality.
%
%   We use the directional rx_vehicular_dipole pattern (zenith peak,
%   nadir notch ~17 dB lower) so any wrong-direction lookup lands in the
%   notch and the assertion fires with large margin.

    methods (TestClassSetup)
        function checkToolboxes(tc)
            tc.assumeTrue(exist('propagationModel', 'file') == 2, ...
                'propagationModel not available - skipping');
            tc.assumeTrue(exist('txsite', 'file') == 2, ...
                'txsite not available - skipping');
            tc.assumeTrue(exist('rxsite', 'file') == 2, ...
                'rxsite not available - skipping');
        end
    end

    methods (Test)
        function symmetricSetupGivesPrxDlEqualsPrxUl(tc)
            this_dir = fileparts(mfilename('fullpath'));
            repo     = fileparts(this_dir);
            antennas_dir = fullfile(repo, 'resources', 'antennas');

            cfg = buildConfig();
            cfg.propagation.use_terrain = false;
            cfg.propagation.model       = 'freespace';
            cfg.analysis.fade_margin_db = 0;
            cfg.analysis.percentile     = 50;
            cfg.analysis.threshold_dbm  = -200;
            cfg.flight.num_flight_steps = 1;
            cfg.tx.altitude_m           = 1500;
            cfg.tx.altitude_msl_m       = [];

            % EQUAL powers: any Prx_dl != Prx_ul is from frame, not power.
            cfg.tx.power_air_dbm = 30;
            cfg.tx.power_gnd_dbm = 30;

            % Air side: use rx_vehicular_dipole for BOTH air-side antennas.
            % Boresight nadir for both -- the airborne antennas physically
            % point down at the ground.
            cfg.tx.airborne.antenna_name      = 'rx_vehicular_dipole';
            cfg.rx.airborne.antenna_name_ul   = 'rx_vehicular_dipole';
            cfg.tx.boresight_az_rad           = 0;
            cfg.tx.boresight_el_rad           = -pi/2;     % nadir
            cfg.rx.airborne.boresight_az_rad  = 0;
            cfg.rx.airborne.boresight_el_rad  = -pi/2;     % nadir

            % Ground side: use rx_vehicular_dipole for BOTH ground-side
            % antennas. Boresight zenith for both -- they point up.
            cfg.tx.ground.antenna_name        = 'rx_vehicular_dipole';
            cfg.rx.infantry.count             = 1;
            cfg.rx.infantry.height_m          = 1.5;
            cfg.rx.infantry.antenna_name_dl   = 'rx_vehicular_dipole';
            cfg.rx.infantry.boresight_az_rad  = 0;
            cfg.rx.infantry.boresight_el_rad  = pi/2;      % zenith
            cfg.rx.vehicular.count            = 0;
            cfg.rx.vehicular.antenna_name_dl  = 'rx_vehicular_dipole';
            cfg.rx.vehicular.boresight_az_rad = 0;
            cfg.rx.vehicular.boresight_el_rad = pi/2;

            ant.tx_air_pat  = loadAntennaPattern(cfg.tx.airborne.antenna_name,         antennas_dir);
            ant.tx_gnd_pat  = loadAntennaPattern(cfg.tx.ground.antenna_name,           antennas_dir);
            ant.rx_air_pat  = loadAntennaPattern(cfg.rx.airborne.antenna_name_ul,      antennas_dir);
            ant.rx_gnd1_pat = loadAntennaPattern(cfg.rx.infantry.antenna_name_dl,      antennas_dir);
            ant.rx_gnd2_pat = loadAntennaPattern(cfg.rx.vehicular.antenna_name_dl,     antennas_dir);

            % One ground RX, TX directly overhead.
            rx_lat = 32.86;
            rx_lon = 35.21;
            rx_table = table(rx_lon, rx_lat, ...
                categorical("infantry", ["infantry","vehicular"]), ...
                cfg.rx.infantry.height_m, ...
                string(cfg.rx.infantry.antenna_name_dl), ...
                'VariableNames', {'lon','lat','type','height_m','antenna_name_dl'});
            rx_array = rxsite('Latitude', rx_lat, 'Longitude', rx_lon, ...
                              'AntennaHeight', cfg.rx.infantry.height_m);

            pm = propagationModel('freespace');
            tx_template = txsite('Name','Aerial_Tx', ...
                'Latitude', rx_lat, 'Longitude', rx_lon, ...
                'AntennaHeight', cfg.tx.altitude_m, ...
                'TransmitterFrequency', cfg.tx.frequency_hz, ...
                'TransmitterPower', 10^((cfg.tx.power_air_dbm - 30) / 10));

            rot = runRotation(tx_template, rx_array, rx_table, pm, '', ...
                              rx_lon, rx_lat, ant, cfg);

            % With the correct frame convention, Prx_dl == Prx_ul exactly
            % (modulo single-precision noise) because the only physical
            % asymmetry between DL and UL was the TX power, which we equalised.
            dl = double(rot.prx_floor_dbm_dl);
            ul = double(rot.prx_floor_dbm_ul);
            tc.verifyEqual(dl, ul, 'AbsTol', 0.1, sprintf( ...
                ['Symmetric setup (matched antennas, equal powers) should ' ...
                 'give Prx_dl == Prx_ul; got DL=%.2f dBm, UL=%.2f dBm, ' ...
                 'delta=%.2f dB. A delta of ~17 dB or more indicates ' ...
                 'the UL frame is swapped (one of the UL gain lookups is ' ...
                 'in the dipole notch instead of the zenith peak).'], ...
                dl, ul, dl - ul));
        end

        function powerDeltaShowsUpExactlyInPrxDelta(tc)
            % Same symmetric setup, but with Pt_air = Pt_gnd + 17 dB.
            % Prx_dl - Prx_ul should equal exactly 17 dB (any deviation is
            % from a frame mishap, not power).
            this_dir = fileparts(mfilename('fullpath'));
            repo     = fileparts(this_dir);
            antennas_dir = fullfile(repo, 'resources', 'antennas');

            cfg = buildConfig();
            cfg.propagation.use_terrain = false;
            cfg.propagation.model       = 'freespace';
            cfg.analysis.fade_margin_db = 0;
            cfg.analysis.percentile     = 50;
            cfg.analysis.threshold_dbm  = -200;
            cfg.flight.num_flight_steps = 1;
            cfg.tx.altitude_m           = 1500;
            cfg.tx.altitude_msl_m       = [];
            cfg.tx.power_air_dbm        = 47;
            cfg.tx.power_gnd_dbm        = 30;       % delta = 17 dB

            cfg.tx.airborne.antenna_name      = 'rx_vehicular_dipole';
            cfg.rx.airborne.antenna_name_ul   = 'rx_vehicular_dipole';
            cfg.tx.boresight_el_rad           = -pi/2;
            cfg.rx.airborne.boresight_el_rad  = -pi/2;
            cfg.tx.ground.antenna_name        = 'rx_vehicular_dipole';
            cfg.rx.infantry.count             = 1;
            cfg.rx.infantry.antenna_name_dl   = 'rx_vehicular_dipole';
            cfg.rx.infantry.boresight_el_rad  = pi/2;
            cfg.rx.vehicular.count            = 0;
            cfg.rx.vehicular.antenna_name_dl  = 'rx_vehicular_dipole';

            ant.tx_air_pat  = loadAntennaPattern(cfg.tx.airborne.antenna_name,         antennas_dir);
            ant.tx_gnd_pat  = loadAntennaPattern(cfg.tx.ground.antenna_name,           antennas_dir);
            ant.rx_air_pat  = loadAntennaPattern(cfg.rx.airborne.antenna_name_ul,      antennas_dir);
            ant.rx_gnd1_pat = loadAntennaPattern(cfg.rx.infantry.antenna_name_dl,      antennas_dir);
            ant.rx_gnd2_pat = loadAntennaPattern(cfg.rx.vehicular.antenna_name_dl,     antennas_dir);

            rx_lat = 32.86; rx_lon = 35.21;
            rx_table = table(rx_lon, rx_lat, ...
                categorical("infantry", ["infantry","vehicular"]), ...
                1.5, "rx_vehicular_dipole", ...
                'VariableNames', {'lon','lat','type','height_m','antenna_name_dl'});
            rx_array = rxsite('Latitude', rx_lat, 'Longitude', rx_lon, ...
                              'AntennaHeight', 1.5);

            pm = propagationModel('freespace');
            tx_template = txsite('Name','Aerial_Tx', ...
                'Latitude', rx_lat, 'Longitude', rx_lon, ...
                'AntennaHeight', cfg.tx.altitude_m, ...
                'TransmitterFrequency', cfg.tx.frequency_hz, ...
                'TransmitterPower', 10^((cfg.tx.power_air_dbm - 30) / 10));

            rot = runRotation(tx_template, rx_array, rx_table, pm, '', ...
                              rx_lon, rx_lat, ant, cfg);

            delta = double(rot.prx_floor_dbm_dl) - double(rot.prx_floor_dbm_ul);
            expected = cfg.tx.power_air_dbm - cfg.tx.power_gnd_dbm;     % 17
            tc.verifyEqual(delta, expected, 'AbsTol', 0.1, sprintf( ...
                ['Prx_dl - Prx_ul should equal the power delta (%.1f dB); ' ...
                 'got %.2f dB. Difference indicates UL frame mishap.'], ...
                expected, delta));
        end
    end
end
