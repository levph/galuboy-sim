classdef test_haversineDistance < matlab.unittest.TestCase
%TEST_HAVERSINEDISTANCE  Verify great-circle distance computation.

    methods (Test)

        function zeroDistanceForSamePoint(tc)
            tc.verifyEqual(haversineDistance(33.0, 35.0, 33.0, 35.0), 0, 'AbsTol', 1e-9);
        end

        function knownOneDegreeLatitude(tc)
            % 1 degree of latitude ~ 111.32 km at any longitude.
            d = haversineDistance(0, 0, 1, 0);
            tc.verifyEqual(d, 111320, 'RelTol', 0.001);
        end

        function knownOneDegreeAtEquator(tc)
            % 1 degree of longitude at the equator ~ 111.32 km.
            d = haversineDistance(0, 0, 0, 1);
            tc.verifyEqual(d, 111320, 'RelTol', 0.001);
        end

        function longitudeShrinkesWithLatitude(tc)
            d_equator = haversineDistance(0, 0, 0, 1);
            d_60      = haversineDistance(60, 0, 60, 1);
            tc.verifyEqual(d_60 / d_equator, cosd(60), 'RelTol', 0.01);
        end

        function antipodalDistance(tc)
            % Antipodal points: half the Earth circumference ~ 20,015 km.
            d = haversineDistance(0, 0, 0, 180);
            tc.verifyEqual(d, pi * 6371000, 'RelTol', 0.001);
        end

        function vectorInput(tc)
            d = haversineDistance(33, 35, [33 34], [35 36]);
            tc.verifySize(d, [1 2]);
            tc.verifyEqual(d(1), 0, 'AbsTol', 1e-9);
            tc.verifyGreaterThan(d(2), 100000);
        end

        function symmetry(tc)
            d1 = haversineDistance(33.0, 35.0, 34.5, 36.2);
            d2 = haversineDistance(34.5, 36.2, 33.0, 35.0);
            tc.verifyEqual(d1, d2, 'AbsTol', 1e-6);
        end

    end
end
