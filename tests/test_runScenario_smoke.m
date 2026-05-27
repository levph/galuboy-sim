classdef test_runScenario_smoke < matlab.unittest.TestCase
%TEST_RUNSCENARIO_SMOKE  End-to-end smoke run with reduced sizes.

    methods (TestClassSetup)
        function checkToolboxes(tc)
            tc.assumeTrue(exist('propagationModel', 'file') == 2, ...
                'propagationModel not available - skipping');
        end
    end

    methods (Test)
        function smokeRun(tc)
            cfg = buildConfig();
            % Run against a single override-bounded region (no OSM file
            % required) so the smoke test is portable across worktrees.
            override_mask = arrayfun(@(rg) ~isempty(rg.latlim_override), cfg.regions);
            override_idx  = find(override_mask, 1);
            tc.assertNotEmpty(override_idx, 'No override-bounded region in config');
            cfg.regions                 = cfg.regions(override_idx);
            cfg.regions.osm_path        = '';            % skip OSM building load
            cfg.flight.num_rotations    = 2;
            cfg.flight.num_flight_steps = 10;
            cfg.rx.infantry.count       = 5;
            cfg.rx.vehicular.count      = 5;
            cfg.propagation.use_terrain = false;         % skip DT2 in smoke
            cfg.propagation.model       = 'freespace';
            cfg.parallel.enabled        = false;         % avoid parpool startup
            cfg.io.results_dir          = fullfile(tempdir, 'galuboy_smoke');

            t0 = tic;
            results = runScenario(cfg);
            elapsed = toc(t0);

            tc.verifyLessThan(elapsed, 60, 'Smoke should run in under 60 s');
            tc.verifyEqual(numel(results), 1);

            r = results(1);
            tc.verifyEqual(size(r.available_dl),       [2, 10]);
            tc.verifyEqual(size(r.available_ul),       [2, 10]);
            tc.verifyEqual(size(r.available_bidi),     [2, 10]);
            tc.verifyEqual(size(r.prx_floor_dbm_dl),   [2, 10]);
            tc.verifyEqual(size(r.prx_floor_dbm_ul),   [2, 10]);
            tc.verifyEqual(size(r.frac_above_dl),      [2, 10]);
            tc.verifyEqual(size(r.frac_above_ul),      [2, 10]);
            tc.verifyTrue(all(isfinite(r.prx_floor_dbm_dl(:))), 'No NaN P_rx_dl');
            tc.verifyTrue(all(isfinite(r.prx_floor_dbm_ul(:))), 'No NaN P_rx_ul');
            tc.verifyTrue(islogical(r.available_dl));
            tc.verifyTrue(islogical(r.available_ul));
            tc.verifyTrue(islogical(r.available_bidi));

            % Back-compat aliases
            tc.verifyEqual(r.available,     r.available_dl);
            tc.verifyEqual(r.frac_above,    r.frac_above_dl);
            tc.verifyEqual(r.prx_floor_dbm, r.prx_floor_dbm_dl);
        end
    end
end
