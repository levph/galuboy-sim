classdef test_runRotation_directional < matlab.unittest.TestCase
%TEST_RUNROTATION_DIRECTIONAL  Directional-pattern link budget at overhead pass.
%
%   Builds a synthetic single-flight-step rotation where the TX sits
%   exactly above one RX. With:
%     - free-space propagation
%     - isotropic TX (7 dBi)
%     - vehicular dipole on the RX (peak 7 dBi at el offset 0, boresight
%       at zenith -> peak when signal arrives from straight up)
%     - zero fade margin
%   the link budget should be:
%       Prx = Pt + 7 dBi (TX) - FSPL(d, f) + 7 dBi (RX peak) - 0
%
%   This catches bugs 1.1 (TX/RX frame swap), 1.4 (compass vs math az), and
%   1.5 (az grid wrap) in combination: any of them would steer the RX
%   lookup into the dipole notch (~-10 dBi) instead of the peak, dropping
%   Prx by ~17 dB.

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
        function overheadHitsBoresightPeak(tc)
            this_dir = fileparts(mfilename('fullpath'));
            repo     = fileparts(this_dir);
            antennas_dir = fullfile(repo, 'resources', 'antennas');

            cfg = buildConfig();
            cfg.propagation.use_terrain   = false;
            cfg.propagation.model         = 'freespace';
            cfg.analysis.fade_margin_db   = 0;
            cfg.analysis.percentile       = 50;
            cfg.analysis.threshold_dbm    = -200;
            cfg.flight.num_flight_steps   = 1;
            cfg.tx.altitude_m             = 1500;
            cfg.tx.altitude_msl_m         = [];
            cfg.tx.power_dbm              = 30;
            cfg.tx.frequency_hz           = 4e9;
            cfg.tx.antenna_name           = 'tx_omni';        % isotropic 7 dBi
            cfg.rx.infantry.count         = 1;
            cfg.rx.infantry.height_m      = 1.5;
            cfg.rx.infantry.antenna_name  = 'rx_vehicular_dipole';
            cfg.rx.infantry.boresight_az_rad = 0;
            cfg.rx.infantry.boresight_el_rad = pi/2;          % zenith
            cfg.rx.vehicular.count        = 0;
            cfg.rx.vehicular.antenna_name = 'rx_vehicular_dipole';

            ant.tx_pat  = loadAntennaPattern(cfg.tx.antenna_name,             antennas_dir);
            ant.inf_pat = loadAntennaPattern(cfg.rx.infantry.antenna_name,    antennas_dir);
            ant.veh_pat = loadAntennaPattern(cfg.rx.vehicular.antenna_name,   antennas_dir);

            % Manual single-RX placement at a known coordinate. We
            % bypass placeReceivers to put the RX exactly at the
            % "centroid" and the TX directly overhead at the same lat/lon.
            rx_lat = 32.86;
            rx_lon = 35.21;

            rx_table = table(rx_lon, rx_lat, ...
                categorical("infantry", ["infantry","vehicular"]), ...
                cfg.rx.infantry.height_m, ...
                string(cfg.rx.infantry.antenna_name), ...
                'VariableNames', {'lon','lat','type','height_m','antenna_name'});
            rx_array = rxsite('Latitude', rx_lat, 'Longitude', rx_lon, ...
                              'AntennaHeight', cfg.rx.infantry.height_m);

            pm = propagationModel('freespace');
            tx_template = txsite('Name', 'Aerial_Tx', ...
                'Latitude', rx_lat, 'Longitude', rx_lon, ...
                'AntennaHeight', cfg.tx.altitude_m, ...
                'TransmitterFrequency', cfg.tx.frequency_hz, ...
                'TransmitterPower', 10^((cfg.tx.power_dbm - 30) / 10));

            % Single-step trajectory: TX coincident with RX in lat/lon.
            flight_lons = rx_lon;
            flight_lats = rx_lat;

            rot = runRotation(tx_template, rx_array, rx_table, pm, '', ...
                              flight_lons, flight_lats, ant, cfg);

            % Compute expected Prx. The MSL-aware runRotation probes GMTED
            % terrain at the centroid and the waypoint; both probes hit the
            % same point so AGL collapses to cfg.tx.altitude_m exactly.
            c       = 299792458;
            lambda  = c / cfg.tx.frequency_hz;
            d       = cfg.tx.altitude_m;          % slant range = TX altitude
            fspl_db = 20 * log10(4 * pi * d / lambda);

            % Peak gains from the pattern at el offset 0 (vehicular dipole)
            % and isotropic TX (tx_omni reports flat 7 dBi).
            g_tx_peak = ant.tx_pat.flat_gain_dbi;                          % 7
            g_rx_peak = ant.inf_pat.gain_func(0, 0);                       % 7

            expected_prx = cfg.tx.power_dbm + g_tx_peak - fspl_db + g_rx_peak;

            tc.verifyEqual(double(rot.prx_floor_dbm), expected_prx, ...
                'AbsTol', 0.5, ...
                sprintf(['Prx at overhead should match boresight peak '...
                         '(expected %.2f dBm; got %.2f dBm). A ~17 dB ' ...
                         'deficit indicates the RX lookup is in the dipole notch.'], ...
                        expected_prx, double(rot.prx_floor_dbm)));
        end
    end
end
