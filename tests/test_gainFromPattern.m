classdef test_gainFromPattern < matlab.unittest.TestCase
%TEST_GAINFROMPATTERN  Verify off-boresight gain interpolation (Part B).

    properties
        pat;
    end

    methods (TestClassSetup)
        function setup(tc)
            ang = [0; 30; 60; 90; 120; 150; 180];
            g   = [10; 8; 4; 0; -5; -10; -15];
            tc.pat = struct('name','test', 'ang_deg',ang, 'gain_dbi',g, ...
                'max_deg',180, 'interp', griddedInterpolant(ang, g, 'linear', 'nearest'));
        end
    end

    methods (Test)

        function boresightGain(tc)
            g = gainFromPattern(tc.pat, 0);
            tc.verifyEqual(g, 10, 'AbsTol', 1e-9);
        end

        function interpolatedValue(tc)
            g = gainFromPattern(tc.pat, 15);
            tc.verifyEqual(g, 9, 'AbsTol', 0.5);
        end

        function negativeAngleFolded(tc)
            g_pos = gainFromPattern(tc.pat, 30);
            g_neg = gainFromPattern(tc.pat, -30);
            tc.verifyEqual(g_neg, g_pos, 'AbsTol', 1e-9);
        end

        function beyondMaxDegClamped(tc)
            g_at_max  = gainFromPattern(tc.pat, 180);
            g_beyond  = gainFromPattern(tc.pat, 200);
            tc.verifyEqual(g_beyond, g_at_max, 'AbsTol', 1e-9);
        end

        function arrayInput(tc)
            g = gainFromPattern(tc.pat, [0 90 180]);
            tc.verifySize(g, [1 3]);
            tc.verifyEqual(g(1), 10, 'AbsTol', 1e-9);
            tc.verifyEqual(g(2), 0, 'AbsTol', 1e-9);
            tc.verifyEqual(g(3), -15, 'AbsTol', 1e-9);
        end

        function matrixShapePreserved(tc)
            theta = [0 30; 60 90];
            g = gainFromPattern(tc.pat, theta);
            tc.verifySize(g, [2 2]);
        end

    end
end
