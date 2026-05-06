classdef test_runScenario_smoke < matlab.unittest.TestCase
%TEST_RUNSCENARIO_SMOKE  End-to-end smoke run with reduced sizes.

    methods (TestClassSetup)
        function checkToolboxes(tc)
            tc.assumeTrue(exist('propagationModel', 'file') == 2, ...
                'propagationModel not available - skipping');

            this_dir = fileparts(mfilename('fullpath'));
            repo     = fileparts(this_dir);
            tc.assumeTrue(isfile(fullfile(repo, 'resources', 'osm', 'yarka.osm')), ...
                'yarka.osm not present - skipping smoke test');
        end
    end

    methods (Test)
        function smokeRun(tc)
            cfg = buildConfig();
            cfg.flight.num_rotations    = 2;
            cfg.flight.num_flight_steps = 10;
            cfg.rx.infantry.count       = 5;
            cfg.rx.vehicular.count      = 5;
            cfg.propagation.use_terrain = false;   % skip DT2 in smoke
            cfg.propagation.model       = 'freespace';
            cfg.parallel.enabled        = false;   % avoid parpool startup
            cfg.io.results_dir          = fullfile(tempdir, 'galuboy_smoke');

            t0 = tic;
            results = runScenario(cfg);
            elapsed = toc(t0);

            tc.verifyLessThan(elapsed, 60, 'Smoke should run in under 60 s');
            tc.verifyEqual(numel(results), 1);

            r = results(1);
            tc.verifyEqual(size(r.available),  [2, 10]);
            tc.verifyEqual(size(r.prx_pctile), [2, 10]);
            tc.verifyTrue(all(isfinite(r.prx_pctile(:))), 'No NaN P_rx');
            tc.verifyTrue(islogical(r.available));
        end
    end
end
