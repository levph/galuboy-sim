classdef test_coherentCombineDb < matlab.unittest.TestCase
%TEST_COHERENTCOMBINEDB  Verify coherent multipath path-loss combination.

    methods (Test)

        function singleRayUnchanged(tc)
            tc.verifyEqual(coherentCombineDb(100, 1.234), 100, 'AbsTol', 1e-9);
        end

        function emptyIsOutage(tc)
            tc.verifyEqual(coherentCombineDb([], []), Inf);
        end

        function inPhaseAddsSixDb(tc)
            % Two equal-amplitude rays in phase: voltage doubles -> 6.02 dB lower.
            tc.verifyEqual(coherentCombineDb([100 100], [0 0]), 100 - 20*log10(2), 'AbsTol', 1e-9);
        end

        function antiPhaseCancels(tc)
            % Equal amplitude, opposite phase -> near-total cancellation. Exact
            % Inf only at abs(E)==0; floating-point exp(j*pi) leaves a tiny
            % residual, so we just require an effectively-outage loss.
            tc.verifyGreaterThan(coherentCombineDb([100 100], [0 pi]), 300);
        end

        function higherThanDominantWhenPartialCancel(tc)
            % Reproduces the live two-ray case: phases 4.26891 / 1.60584 rad,
            % loss 116.865 / 124.953 dB -> ~120.28 dB (worse than dominant).
            v = coherentCombineDb([116.865 124.953], [4.26891 1.60584]);
            tc.verifyEqual(v, 120.279, 'AbsTol', 1e-2);
            tc.verifyGreaterThan(v, 116.865);   % worse than the strongest ray
        end

    end
end
