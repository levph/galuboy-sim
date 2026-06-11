classdef test_generateCircularTrajectory < matlab.unittest.TestCase
%TEST_GENERATECIRCULARTRAJECTORY  Verify the circular flight path geometry.

    properties
        region
        flight
    end

    methods (TestClassSetup)
        function setup(tc)
            tc.region = struct('center_lat', 33.0, 'center_lon', 35.1);
            tc.flight = struct('max_rotation_deg', 360, 'delta_t_s', 1.0, ...
                               'circle_radius_m', 1000, 'max_tilt_deg', 23);
        end
    end

    methods (Test)

        function correctLengthsAndNoDuplicateClose(tc)
            [lo, la, hd, ti] = generateCircularTrajectory(tc.region, tc.flight);
            Nf = circularFlightSampling(tc.flight).num_samples;
            tc.verifyGreaterThan(Nf, 1);
            tc.verifySize(lo, [1 Nf]);
            tc.verifySize(la, [1 Nf]);
            tc.verifySize(hd, [1 Nf]);
            tc.verifySize(ti, [1 Nf]);
            % first and last points must differ (open sampling of the circle)
            tc.verifyGreaterThan(hypot(lo(1)-lo(end), la(1)-la(end)), 0);
        end

        function pointsLieOnCircle(tc)
            [lo, la] = generateCircularTrajectory(tc.region, tc.flight);
            d = haversineDistance(tc.region.center_lat, tc.region.center_lon, la, lo);
            tc.verifyEqual(d, tc.flight.circle_radius_m * ones(1, numel(lo)), ...
                'AbsTol', 5);     % planar->geo approx; within 5 m at R=1 km
        end

        function headingIsTangent(tc)
            % Heading (CW-from-north unit vector) must be perpendicular to the
            % radial direction from centre to each waypoint.
            [lo, la, hd] = generateCircularTrajectory(tc.region, tc.flight);
            mlon = 111320 * cosd(tc.region.center_lat);
            rx = (lo - tc.region.center_lon) * mlon;     % east
            ry = (la - tc.region.center_lat) * 111320;   % north
            rnorm = hypot(rx, ry);
            % heading unit vector in (east, north)
            he = sin(hd); hn = cos(hd);
            dotrad = (rx.*he + ry.*hn) ./ rnorm;
            tc.verifyEqual(dotrad, zeros(1, numel(lo)), 'AbsTol', 1e-2);
        end

        function tiltIsConstant(tc)
            [~, ~, ~, ti] = generateCircularTrajectory(tc.region, tc.flight);
            tc.verifyEqual(ti, deg2rad(23) * ones(1, numel(ti)), 'AbsTol', 1e-12);
        end

    end
end
