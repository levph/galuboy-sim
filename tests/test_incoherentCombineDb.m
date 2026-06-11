classdef test_incoherentCombineDb < matlab.unittest.TestCase
%TEST_INCOHERENTCOMBINEDB  Verify wideband (power-sum) ray combination.

    methods (Test)

        function singleRayUnchanged(tc)
            tc.verifyEqual(incoherentCombineDb(100), 100, 'AbsTol', 1e-9);
        end

        function emptyIsOutage(tc)
            tc.verifyEqual(incoherentCombineDb([]), Inf);
        end

        function twoEqualAddThreeDb(tc)
            % Equal powers add 3.01 dB of power -> 3.01 dB lower path loss.
            tc.verifyEqual(incoherentCombineDb([100 100]), 100 - 10*log10(2), 'AbsTol', 1e-9);
        end

        function neverWorseThanDominant(tc)
            % Power sum is always <= the strongest ray (adding power helps).
            v = incoherentCombineDb([116.865 124.953]);
            tc.verifyLessThanOrEqual(v, 116.865 + 1e-9);
        end

    end
end
