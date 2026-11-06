classdef test_distanceFromCentroid < matlab.unittest.TestCase
%TEST_DISTANCEFROMCENTROID  Verify distance-from-bbox-centroid helper.

    methods (Test)

        function centroidIsZero(tc)
            lonlim = [35.0, 35.5];
            latlim = [33.0, 33.5];
            center = [mean(lonlim), mean(latlim)];
            d = distanceFromCentroid(lonlim, latlim, center);
            tc.verifyEqual(d, 0, 'AbsTol', 1e-6);
        end

        function outputIsRowVector(tc)
            pts = [35.1 33.1; 35.2 33.2; 35.3 33.3];
            d = distanceFromCentroid([35 36], [33 34], pts);
            tc.verifySize(d, [1 3]);
        end

        function distancesArePositive(tc)
            pts = [35.1 33.1; 35.9 33.9];
            d = distanceFromCentroid([35 36], [33 34], pts);
            tc.verifyGreaterThan(d, 0);
        end

        function consistentWithHaversine(tc)
            lonlim = [35.0, 35.4];
            latlim = [33.0, 33.4];
            pt = [35.3, 33.1];
            d = distanceFromCentroid(lonlim, latlim, pt);
            expected = haversineDistance(mean(latlim), mean(lonlim), pt(2), pt(1));
            tc.verifyEqual(d, expected, 'AbsTol', 1e-6);
        end

    end
end
