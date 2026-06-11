classdef test_raytraceRotation < matlab.unittest.TestCase
%TEST_RAYTRACEROTATION  Integration smoke test for the Part A rotation engine.
%
%   Slow: builds a (hidden) Site Viewer + raytraces a few steps. Skips unless
%   R2025a RF/Mapping toolboxes and the FABDEM .dt2 tiles are present. Reduced
%   scale (coarse delta_t, 3+3 RX) keeps it to a handful of seconds.

    properties
        region; cfg; sv; pm;
    end

    methods (TestClassSetup)
        function setup(tc)
            tc.assumeTrue(exist('propagationModel','file')==2 && exist('readgeotable','file')==2, ...
                'needs R2025a RF Propagation + Mapping');
            repo = fileparts(fileparts(mfilename('fullpath')));
            tdir = fullfile(repo,'resources','terrain');
            tc.assumeTrue(isfolder(tdir) && ~isempty(dir(fullfile(tdir,'*_fabdem.dt2'))), ...
                'FABDEM .dt2 tiles not present in resources/terrain');

            c = buildConfigA();
            c.flight.delta_t_s   = 15;     % coarse -> few steps
            c.rx.infantry.count  = 3;
            c.rx.vehicular.count = 3;
            tc.cfg = c;

            tiles = discoverTerrainTiles(tdir, [32.9 33.55], [35.09 35.92]);
            setupTerrain(c.propagation.terrain_name, tiles);
            tc.region = resolveRegion(c.regions(2), repo);   % urban_suburban_02
            tc.pm = propagationModel('raytracing','Method','sbr', ...
                'MaxNumReflections',2,'MaxNumDiffractions',0);
            tc.sv = setupRegionScene(tc.region, c);          % hidden viewer
        end
    end

    methods (TestClassTeardown)
        function teardown(tc)
            if ~isempty(tc.sv) && isvalid(tc.sv), close(tc.sv); end
        end
    end

    methods (Test)
        function producesSaneArrays(tc)
            rng(tc.cfg.flight.rng_seed);
            rot = raytraceRotation(tc.region, tc.cfg, tc.sv, tc.pm);
            Nf  = size(rot.pl_db, 1);
            nrx = size(rot.pl_db, 2);

            tc.verifyEqual(nrx, 6);
            tc.verifyGreaterThan(Nf, 1);
            tc.verifyClass(rot.pl_db, 'single');
            tc.verifySize(rot.steer_gnd_deg, [Nf nrx]);
            tc.verifySize(rot.steer_air_deg, [Nf nrx]);

            fin = isfinite(rot.pl_db);
            tc.verifyTrue(all(rot.pl_db(fin) > 60 & rot.pl_db(fin) < 300), 'PL out of plausible range');
            tc.verifyTrue(all(rot.steer_gnd_deg(:) >= 0 & rot.steer_gnd_deg(:) <= 180));
            tc.verifyTrue(all(rot.steer_air_deg(:) >= 0 & rot.steer_air_deg(:) <= 180));
            tc.verifyEqual(numel(rot.tx_lon), Nf);
            tc.verifyTrue(all(rot.tx_alt_msl_m > 3000));     % ~4 km MSL
        end
    end
end
