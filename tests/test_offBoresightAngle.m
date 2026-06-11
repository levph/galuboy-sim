classdef test_offBoresightAngle < matlab.unittest.TestCase
%TEST_OFFBORESIGHTANGLE  Verify the steering-angle (spherical) computation.

    methods (Test)

        function alignedIsZero(tc)
            % Link exactly along boresight -> 0.
            tc.verifyEqual(offBoresightAngle(0.7, 0.3, 0.7, 0.3), 0, 'AbsTol', 1e-9);
        end

        function oppositeIsPi(tc)
            % Zenith boresight vs nadir link -> pi.
            tc.verifyEqual(offBoresightAngle(0, -pi/2, 0, pi/2), pi, 'AbsTol', 1e-9);
        end

        function zenithBoresightGivesPolarAngle(tc)
            % Against a zenith boresight, theta = 90deg - elevation, independent
            % of azimuth (pole is azimuth-degenerate but this formula is robust).
            el = deg2rad(30);
            for az = [0 1 2 3 4 5]
                tc.verifyEqual(offBoresightAngle(az, el, 0, pi/2), pi/2 - el, 'AbsTol', 1e-9);
            end
        end

        function shapePreserved(tc)
            az = [0 1; 2 3]; el = [0.1 0.2; 0.3 0.4];
            tc.verifySize(offBoresightAngle(az, el, 0, pi/2), [2 2]);
        end

    end
end
