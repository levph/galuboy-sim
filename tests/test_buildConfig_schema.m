classdef test_buildConfig_schema < matlab.unittest.TestCase
%TEST_BUILDCONFIG_SCHEMA  Validate the v4 (DL/UL) config struct schema.
%
%   Confirms the required field tree is present and that retired
%   single-direction fields are gone.

    methods (Test)

        function requiredFieldsPresent(tc)
            cfg = buildConfig();

            tc.verifyTrue(isfield(cfg, 'tx'));
            tc.verifyTrue(isfield(cfg, 'rx'));
            tc.verifyTrue(isfield(cfg, 'propagation'));
            tc.verifyTrue(isfield(cfg, 'flight'));
            tc.verifyTrue(isfield(cfg, 'analysis'));
            tc.verifyTrue(isfield(cfg, 'regions'));
            tc.verifyTrue(isfield(cfg, 'parallel'));
            tc.verifyTrue(isfield(cfg, 'io'));
            tc.verifyTrue(isfield(cfg, 'viz'));
            tc.verifyTrue(isfield(cfg, 'hist'));

            % rx subtree: infantry + vehicular share antenna_name_dl;
            % airborne RX is the UL receiver; patch.* drives the helper.
            tc.verifyTrue(all(isfield(cfg.rx, {'infantry', 'vehicular', 'airborne', 'patch'})));
            for which = ["infantry", "vehicular"]
                sub = cfg.rx.(which);
                tc.verifyTrue(all(isfield(sub, {'count','height_m','antenna_name_dl', ...
                                                'boresight_az_rad','boresight_el_rad'})), ...
                    sprintf('rx.%s missing required field', which));
            end
            tc.verifyTrue(all(isfield(cfg.rx.airborne, ...
                {'antenna_name_ul','boresight_az_rad','boresight_el_rad'})));
            tc.verifyTrue(all(isfield(cfg.rx.patch, {'fov_in_deg','fov_out_deg','floor_db'})));

            % tx: DL/UL split powers, airborne + ground antenna names
            tc.verifyTrue(all(isfield(cfg.tx, {'frequency_hz','power_air_dbm','power_gnd_dbm', ...
                'altitude_m','altitude_msl_m','airborne','ground', ...
                'boresight_az_rad','boresight_el_rad'})));
            tc.verifyTrue(isfield(cfg.tx.airborne, 'antenna_name'));
            tc.verifyTrue(isfield(cfg.tx.ground,   'antenna_name'));

            % flight + analysis + propagation
            tc.verifyTrue(all(isfield(cfg.flight, {'num_flight_steps','step_degrees', ...
                'num_rotations','lemniscate_scale_deg'})));
            tc.verifyTrue(all(isfield(cfg.analysis, {'threshold_dbm','percentile', ...
                'fade_margin_db'})));
            tc.verifyTrue(all(isfield(cfg.propagation, {'model','time_variability', ...
                'situation_variability','terrain_name','use_terrain'})));

            % regions: 6 northern-Israel bboxes
            % (3 urban_suburban + forest + 2 mountain). rural_open was dropped.
            tc.verifyEqual(numel(cfg.regions), 6);
            tc.verifyTrue(any(strcmp({cfg.regions.name}, 'urban_suburban_01')));
            tc.verifyFalse(any(strcmp({cfg.regions.name}, 'rural_open_01')), ...
                'rural_open_01 was dropped from the region set');
            tc.verifyFalse(any(strcmp({cfg.regions.name}, 'yarka')), ...
                'yarka should no longer be in the default region set');
        end

        function retiredFieldsAbsent(tc)
            cfg = buildConfig();

            % v3 names that were retired
            tc.verifyFalse(isfield(cfg, 'group'),  'config.group.* should be dropped');
            tc.verifyFalse(isfield(cfg, 'geo'),    'config.geo.* should be dropped');
            tc.verifyFalse(isfield(cfg, 'link'),   'config.link.* should be dropped');

            tc.verifyFalse(isfield(cfg.tx,  'frequency'),  'use frequency_hz, not frequency');

            % v4 DL/UL split retired the single-direction names.
            tc.verifyFalse(isfield(cfg.tx, 'power_dbm'),    'replaced by power_air_dbm + power_gnd_dbm');
            tc.verifyFalse(isfield(cfg.tx, 'antenna_name'), 'replaced by tx.airborne/ground.antenna_name');
            tc.verifyFalse(isfield(cfg.rx.infantry,  'antenna_name'), 'replaced by antenna_name_dl');
            tc.verifyFalse(isfield(cfg.rx.vehicular, 'antenna_name'), 'replaced by antenna_name_dl');
            tc.verifyFalse(isfield(cfg.viz, 'osm_path'),    'osm_path moved to regions(*).osm_path');
        end

        function fadeMarginIsCustomDb(tc)
            cfg = buildConfig();
            tc.verifyEqual(cfg.analysis.fade_margin_db, 10, ...
                'default custom fade margin = 10 dB');
            tc.verifyFalse(isfield(cfg.analysis, 'fade_margin_type'), ...
                'fade_margin_type was removed');
        end

        function rotationCountAutoDerived(tc)
            cfg = buildConfig();
            tc.verifyEqual(cfg.flight.num_rotations, ...
                floor(180 / cfg.flight.step_degrees));
        end
    end
end
