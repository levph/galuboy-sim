classdef test_loadBuildings < matlab.unittest.TestCase
%TEST_LOADBUILDINGS  Verify the GeoJSON -> raytracing geotable loader (Part A).
%
%   Part A only; requires R2025a (readgeotable + geopolyshape geotables).

    properties
        geojson
    end

    methods (TestClassSetup)
        function locate(tc)
            tc.assumeTrue(exist('readgeotable', 'file') == 2, ...
                'readgeotable not available (needs R2025a Mapping Toolbox)');
            root = fileparts(fileparts(mfilename('fullpath')));
            tc.geojson = fullfile(root, 'resources', 'buildings', 'urban_suburban_02.geojson');
            tc.assumeTrue(exist(tc.geojson, 'file') == 2, 'sample geojson missing');
        end
    end

    methods (Test)

        function addsExtrusionHeights(tc)
            T = loadBuildings(tc.geojson);
            tc.verifyTrue(istable(T) || isa(T, 'table'));
            tc.verifyGreaterThan(height(T), 0);
            tc.verifyTrue(all(ismember({'MinHeight','MaxHeight'}, T.Properties.VariableNames)));
            tc.verifyEqual(class(T.Shape), 'geopolyshape');
        end

        function minHeightZeroMaxHeightMatchesHeight(tc)
            T = loadBuildings(tc.geojson);
            tc.verifyEqual(T.MinHeight, zeros(height(T),1));
            tc.verifyEqual(T.MaxHeight, double(T.height));
            tc.verifyTrue(all(T.MaxHeight >= 3));     % floored at 3 m
            tc.verifyTrue(all(isfinite(T.MaxHeight)));
        end

        function missingFileErrors(tc)
            tc.verifyError(@() loadBuildings('/no/such/file.geojson'), ...
                'loadBuildings:fileNotFound');
        end

    end
end
