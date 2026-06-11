classdef test_combineRayPathloss < matlab.unittest.TestCase
%TEST_COMBINERAYPATHLOSS  Verify per-link COHERENT reduction of raytrace rays.
%
%   Uses struct-array stand-ins for comm.Ray (combineRayPathloss only reads the
%   PathLoss / PhaseShift fields, which [s.Field] concatenation exposes
%   identically for struct arrays).

    methods (Static)
        function s = rayArray(pl, ph)
            s = struct('PathLoss', num2cell(pl), 'PhaseShift', num2cell(ph));
        end
    end

    methods (Test)

        function singleLinkSingleRay(tc)
            out = combineRayPathloss({test_combineRayPathloss.rayArray(100, 0.5)});
            tc.verifyEqual(out, 100, 'AbsTol', 1e-9);     % phase irrelevant for one ray
        end

        function emptyLinkIsOutage(tc)
            tc.verifyEqual(combineRayPathloss({comm.Ray.empty}), Inf);
            tc.verifyEqual(combineRayPathloss({[]}), Inf);
        end

        function shapePreservedAndPerLink(tc)
            rays = {test_combineRayPathloss.rayArray([100 100], [0 0]), ...   % in phase -> +6 dB
                    [], ...
                    test_combineRayPathloss.rayArray(110, 0)};
            out = combineRayPathloss(rays);
            tc.verifyEqual(size(out), [1 3]);
            tc.verifyEqual(out(1), 100 - 20*log10(2), 'AbsTol', 1e-9);   % coherent voltage sum
            tc.verifyEqual(out(2), Inf);
            tc.verifyEqual(out(3), 110, 'AbsTol', 1e-9);
        end

        function antiPhasePartialCancel(tc)
            % two equal rays in anti-phase -> deep cancellation (>> dominant loss)
            out = combineRayPathloss({test_combineRayPathloss.rayArray([100 100], [0 pi])});
            tc.verifyGreaterThan(out, 100);
        end

    end
end
