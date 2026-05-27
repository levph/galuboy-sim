classdef test_runRotation_dl_ul < matlab.unittest.TestCase
%TEST_RUNROTATION_DL_UL  Smoke test for the DL/UL split link budget.
%
%   1 region x 2 rotations x 10 steps x 5 infantry + 5 vehicular,
%   free-space propagation. Asserts:
%     - results(r).available_dl / .available_ul / .available_bidi exist
%       with shape [M x n_rx] and the documented logical/single types
%     - mean(available_dl(:)) >= mean(available_ul(:))  (DL has the net
%       advantage of higher TX power + larger air-side patch)
%     - available_bidi == available_dl & available_ul
%     - Back-compat aliases: results(r).available == results(r).available_dl,
%       .frac_above == .frac_above_dl, .prx_floor_dbm == .prx_floor_dbm_dl

    methods (TestClassSetup)
        function checkToolboxes(tc)
            tc.assumeTrue(exist('propagationModel', 'file') == 2, ...
                'propagationModel not available - skipping');
            tc.assumeTrue(exist('txsite', 'file') == 2, ...
                'txsite not available - skipping');
        end
    end

    methods (Test)
        function dlBeatsUl(tc)
            cfg = buildConfig();
            cfg.propagation.use_terrain = false;
            cfg.propagation.model       = 'freespace';
            cfg.parallel.enabled        = false;
            cfg.flight.num_rotations    = 2;
            cfg.flight.num_flight_steps = 10;
            cfg.rx.infantry.count       = 5;
            cfg.rx.vehicular.count      = 5;

            % Use one of the override-bounded regions (no OSM file required)
            % and lower the placement diameter so we stay inside the
            % dynamic range of the link budget.
            override_mask = arrayfun(@(rg) ~isempty(rg.latlim_override), cfg.regions);
            override_idx  = find(override_mask, 1);
            tc.assumeNotEmpty(override_idx, 'No override-bounded region in config');
            cfg.regions                  = cfg.regions(override_idx);
            cfg.regions.osm_path         = '';            % skip OSM building load
            cfg.rx.placement_diameter_km = 6;             % 3 km radius equivalent
            cfg.io.results_dir           = fullfile(tempdir, 'galuboy_dl_ul');

            results = runScenario(cfg);

            tc.verifyEqual(numel(results), 1);
            r = results(1);

            % Shape + type checks
            M = cfg.flight.num_rotations;
            n_rx = cfg.rx.infantry.count + cfg.rx.vehicular.count;
            tc.verifyEqual(size(r.available_dl),   [M, n_rx]);
            tc.verifyEqual(size(r.available_ul),   [M, n_rx]);
            tc.verifyEqual(size(r.available_bidi), [M, n_rx]);
            tc.verifyClass(r.frac_above_dl,    'single');
            tc.verifyClass(r.frac_above_ul,    'single');
            tc.verifyClass(r.prx_floor_dbm_dl, 'single');
            tc.verifyClass(r.prx_floor_dbm_ul, 'single');

            % DL >= UL on average. With air TX = 47 dBm (vs ground TX = 30 dBm)
            % and air RX = 16-elem patch (vs ground RX = 8-elem patch), DL is
            % the net stronger direction (~14 dB advantage at boresight).
            md = mean(r.available_dl(:));
            mu = mean(r.available_ul(:));
            tc.verifyGreaterThanOrEqual(md, mu, ...
                'DL availability should equal or exceed UL availability');

            % Back-compat aliases
            tc.verifyEqual(r.available,     r.available_dl);
            tc.verifyEqual(r.frac_above,    r.frac_above_dl);
            tc.verifyEqual(r.prx_floor_dbm, r.prx_floor_dbm_dl);

            % bidi = AND of the two directions
            tc.verifyEqual(r.available_bidi, r.available_dl & r.available_ul);
        end
    end
end
