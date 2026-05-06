classdef test_buildConfig_schema < matlab.unittest.TestCase
%TEST_BUILDCONFIG_SCHEMA  Validate the v4 config struct schema.
%
%   Confirms the required field tree is present and that v3 deprecated
%   fields are gone.

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

            % rx subtree
            tc.verifyTrue(all(isfield(cfg.rx, {'infantry', 'vehicular'})));
            for which = ["infantry", "vehicular"]
                sub = cfg.rx.(which);
                tc.verifyTrue(all(isfield(sub, {'count','height_m','antenna_name', ...
                                                'boresight_az_rad','boresight_el_rad'})), ...
                    sprintf('rx.%s missing required field', which));
            end

            % tx
            tc.verifyTrue(all(isfield(cfg.tx, {'frequency_hz','power_dbm','altitude_m', ...
                'antenna_name','boresight_az_rad','boresight_el_rad'})));

            % flight + analysis + propagation
            tc.verifyTrue(all(isfield(cfg.flight, {'num_flight_steps','step_degrees', ...
                'num_rotations','lemniscate_scale_deg'})));
            tc.verifyTrue(all(isfield(cfg.analysis, {'threshold_dbm','percentile', ...
                'fade_margin_db'})));
            tc.verifyTrue(all(isfield(cfg.propagation, {'model','time_variability', ...
                'situation_variability','terrain_name','use_terrain'})));

            % regions defaults to a single Yarka entry
            tc.verifyEqual(numel(cfg.regions), 1);
            tc.verifyEqual(cfg.regions(1).name, 'yarka');
        end

        function deprecatedFieldsAbsent(tc)
            cfg = buildConfig();

            % v3 names that were retired
            tc.verifyFalse(isfield(cfg, 'group'),  'config.group.* should be dropped');
            tc.verifyFalse(isfield(cfg, 'geo'),    'config.geo.* should be dropped');
            tc.verifyFalse(isfield(cfg, 'link'),   'config.link.* should be dropped');

            tc.verifyFalse(isfield(cfg.tx,  'frequency'),  'use frequency_hz, not frequency');
            tc.verifyFalse(isfield(cfg.viz, 'osm_path'),   'osm_path moved to regions(*).osm_path');

            % availability_threshold and percentile moved under analysis.*
            if isfield(cfg, 'sim')
                tc.verifyFalse(isfield(cfg.sim, 'availability_threshold_dbm'));
                tc.verifyFalse(isfield(cfg.sim, 'n_rx'));
            end
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
