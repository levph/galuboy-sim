classdef test_setupTerrain < matlab.unittest.TestCase
%TEST_SETUPTERRAIN  Terrain registration, persistent cache, manifest match.
%
%   Skipped when the Mapping Toolbox is unavailable or no DT2 tiles are
%   present under resources/terrain/.

    properties
        TerrainDir
        TileFiles
        TerrainName
    end

    methods (TestClassSetup)
        function discover(tc)
            this_dir = fileparts(mfilename('fullpath'));
            repo     = fileparts(this_dir);
            tc.TerrainDir = fullfile(repo, 'resources', 'terrain');

            tc.assumeTrue(exist(tc.TerrainDir, 'dir') > 0, ...
                'No resources/terrain/ - skipping terrain tests');
            tc.assumeTrue(exist('addCustomTerrain', 'file') == 2, ...
                'Mapping Toolbox not available');

            d = [dir(fullfile(tc.TerrainDir, '*.dt2')); ...
                 dir(fullfile(tc.TerrainDir, '*.dt1'))];
            tc.assumeNotEmpty(d, 'No DT2/DT1 tiles in resources/terrain/');
            % Use just ONE tile to keep the tiling step fast - each tile
            % takes ~30 s to ingest. The point of these tests is the cache
            % wrapper logic, not throughput.
            tc.TileFiles = {fullfile(d(1).folder, d(1).name)};

            tc.TerrainName = 'galuboy_test_terrain';
        end
    end

    methods (TestClassTeardown)
        function cleanupTerrain(tc)
            try
                removeCustomTerrain(tc.TerrainName);
            catch
                % ignore - may not be registered
            end
        end
    end

    methods (Test)

        function coldRegisterThenCacheHit(tc)
            % First call: registers (slow). Second call: cache reuse (fast).
            t1 = tic; setupTerrain(tc.TerrainName, tc.TileFiles); cold = toc(t1);
            t2 = tic; setupTerrain(tc.TerrainName, tc.TileFiles); warm = toc(t2);

            tc.verifyLessThan(warm, 0.5, ...
                sprintf('Warm reuse should be < 0.5 s; got %.2f', warm));
            tc.verifyLessThan(warm, max(0.5, cold), ...
                'Warm reuse should be faster than cold register');
        end

        function writeLocationUnderRepoCache(tc)
            setupTerrain(tc.TerrainName, tc.TileFiles);
            s = settings;
            opts = properties(s.shared.terrain);
            tc.assertTrue(ismember(lower(tc.TerrainName), lower(opts)));
            g = s.shared.terrain.(tc.TerrainName);
            this_dir = fileparts(mfilename('fullpath'));
            repo     = fileparts(this_dir);
            expected_root = fullfile(repo, 'terrain_cache');
            tc.verifyTrue(startsWith(g.Location.ActiveValue, expected_root), ...
                sprintf('Cache should live under %s, got %s', ...
                        expected_root, g.Location.ActiveValue));
        end

        function manifestMismatchTriggersRetile(tc)
            setupTerrain(tc.TerrainName, tc.TileFiles);

            % Manually corrupt the manifest -> next call must re-tile.
            this_dir = fileparts(mfilename('fullpath'));
            repo     = fileparts(this_dir);
            manifest = fullfile(repo, 'terrain_cache', tc.TerrainName, ...
                                'tile_manifest.txt');
            tc.assertTrue(isfile(manifest));
            fid = fopen(manifest, 'w'); fwrite(fid, 'BOGUS'); fclose(fid);

            % Re-register: should fall through cache check and re-tile.
            t = tic; setupTerrain(tc.TerrainName, tc.TileFiles); elapsed = toc(t);
            tc.verifyGreaterThan(elapsed, 0.2, ...
                'Manifest mismatch should force re-tile (slower than fast path)');
        end
    end
end
