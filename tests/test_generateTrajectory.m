classdef test_generateTrajectory < matlab.unittest.TestCase
%TEST_GENERATETRAJECTORY  Verify lemniscate figure-8 trajectory generation.

    properties
        region;
        flight;
    end

    methods (TestClassSetup)
        function setup(tc)
            tc.region = struct('center_lat', 33.25, 'center_lon', 35.25, ...
                'latlim', [33.0 33.5], 'lonlim', [35.0 35.5]);
            tc.flight = struct('num_flight_steps', 50, ...
                'lemniscate_scale_deg', 0.005, ...
                'lemniscate_length_m', 1000, ...
                'max_tilt_deg', 23);
        end
    end

    methods (Test)

        function outputSizes(tc)
            [lons, lats, tilts, headings] = generateTrajectory(tc.region, tc.flight, 0);
            tc.verifySize(lons, [1 50]);
            tc.verifySize(lats, [1 50]);
            tc.verifySize(tilts, [1 50]);
            tc.verifySize(headings, [1 50]);
        end

        function centeredOnRegion(tc)
            [lons, lats, ~, ~] = generateTrajectory(tc.region, tc.flight, 0);
            tc.verifyEqual(mean(lons), tc.region.center_lon, 'AbsTol', 0.01);
            tc.verifyEqual(mean(lats), tc.region.center_lat, 'AbsTol', 0.01);
        end

        function tiltBounded(tc)
            [~, ~, tilts, ~] = generateTrajectory(tc.region, tc.flight, 0);
            tc.verifyLessThanOrEqual(max(abs(tilts)), deg2rad(23) + 1e-9);
        end

        function rotationChangesOrientation(tc)
            [lons0, ~, ~, ~] = generateTrajectory(tc.region, tc.flight, 0);
            [lons90, ~, ~, ~] = generateTrajectory(tc.region, tc.flight, 90);
            tc.verifyFalse(isequal(lons0, lons90));
        end

        function headingsAreDefined(tc)
            [~, ~, ~, headings] = generateTrajectory(tc.region, tc.flight, 0);
            tc.verifyFalse(any(isnan(headings)));
        end

        function scaleFromLengthM(tc)
            f = tc.flight;
            f.lemniscate_length_m = 2000;
            [lons1, ~, ~, ~] = generateTrajectory(tc.region, f, 0);
            f.lemniscate_length_m = 1000;
            [lons2, ~, ~, ~] = generateTrajectory(tc.region, f, 0);
            span1 = max(lons1) - min(lons1);
            span2 = max(lons2) - min(lons2);
            tc.verifyEqual(span1 / span2, 2.0, 'RelTol', 0.05);
        end

        function fallbackToScaleDeg(tc)
            f = tc.flight;
            f.lemniscate_length_m = [];
            [lons, ~, ~, ~] = generateTrajectory(tc.region, f, 0);
            span = max(lons) - min(lons);
            tc.verifyGreaterThan(span, 0);
        end

    end
end
