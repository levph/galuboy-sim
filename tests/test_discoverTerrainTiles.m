classdef test_discoverTerrainTiles < matlab.unittest.TestCase
%TEST_DISCOVERTERRAINTILES  Verify SRTM tile discovery by bounding box.

    properties
        tmpDir;
    end

    methods (TestMethodSetup)
        function createTmpDir(tc)
            tc.tmpDir = [tempname '_tiles'];
            mkdir(tc.tmpDir);
        end
    end

    methods (TestMethodTeardown)
        function removeTmpDir(tc)
            if isfolder(tc.tmpDir)
                rmdir(tc.tmpDir, 's');
            end
        end
    end

    methods (Test)

        function findsMatchingTile(tc)
            fclose(fopen(fullfile(tc.tmpDir, 'n33_e035_1arc_v3.dt2'), 'w'));
            tiles = discoverTerrainTiles(tc.tmpDir, [33.2 33.8], [35.1 35.9]);
            tc.verifyEqual(numel(tiles), 1);
            tc.verifySubstring(tiles{1}, 'n33_e035');
        end

        function excludesNonOverlappingTile(tc)
            fclose(fopen(fullfile(tc.tmpDir, 'n33_e035_1arc_v3.dt2'), 'w'));
            fclose(fopen(fullfile(tc.tmpDir, 'n40_e020_1arc_v3.dt2'), 'w'));
            tiles = discoverTerrainTiles(tc.tmpDir, [33.2 33.8], [35.1 35.9]);
            tc.verifyEqual(numel(tiles), 1);
        end

        function multipleMatchingTiles(tc)
            fclose(fopen(fullfile(tc.tmpDir, 'n33_e035_1arc_v3.dt2'), 'w'));
            fclose(fopen(fullfile(tc.tmpDir, 'n33_e036_1arc_v3.dt2'), 'w'));
            tiles = discoverTerrainTiles(tc.tmpDir, [33.2 33.8], [35.5 36.5]);
            tc.verifyEqual(numel(tiles), 2);
        end

        function bufferIncludesEdgeTiles(tc)
            % Tile n32_e035 covers [32,33] x [35,36]. A query with latlim
            % starting at 33.03 would miss without the 0.05-deg buffer.
            fclose(fopen(fullfile(tc.tmpDir, 'n32_e035_1arc_v3.dt2'), 'w'));
            tiles = discoverTerrainTiles(tc.tmpDir, [33.03 33.5], [35.1 35.5]);
            tc.verifyEqual(numel(tiles), 1);
        end

        function emptyDirErrors(tc)
            empty = [tempname '_empty'];
            mkdir(empty);
            cleanup = onCleanup(@() rmdir(empty, 's'));
            tc.verifyError(@() discoverTerrainTiles(empty, [33 34], [35 36]), ...
                'discoverTerrainTiles:noTiles');
        end

        function noCoverageErrors(tc)
            fclose(fopen(fullfile(tc.tmpDir, 'n50_e010_1arc_v3.dt2'), 'w'));
            tc.verifyError(@() discoverTerrainTiles(tc.tmpDir, [33 34], [35 36]), ...
                'discoverTerrainTiles:noCoverage');
        end

        function missingDirErrors(tc)
            tc.verifyError(@() discoverTerrainTiles('/nonexistent_dir_xyz', [33 34], [35 36]), ...
                'discoverTerrainTiles:dirMissing');
        end

        function southernHemisphereParsed(tc)
            fclose(fopen(fullfile(tc.tmpDir, 's10_w045_1arc_v3.dt2'), 'w'));
            tiles = discoverTerrainTiles(tc.tmpDir, [-10.5 -9.5], [-45.5 -44.5]);
            tc.verifyEqual(numel(tiles), 1);
        end

    end
end
