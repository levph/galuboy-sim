classdef test_distanceBinStats < matlab.unittest.TestCase
%TEST_DISTANCEBINSTATS  Verify distance binning + reducer (pure helper).

    methods (Test)
        function meanReducerAndCounts(tc)
            dist  = [50 150 250 250];
            vals  = [1 0 1 0];
            edges = 0:100:300;
            [centers, out, counts] = distanceBinStats(dist, vals, edges, @mean);
            tc.verifyEqual(centers, [50 150 250]);
            tc.verifyEqual(out, [1; 0; 0.5], 'AbsTol', 1e-12);
            tc.verifyEqual(counts, [1; 1; 2]);
        end
        function emptyBinsAreNaN(tc)
            [~, out, counts] = distanceBinStats([50 50], [1 0], 0:100:300, @mean);
            tc.verifyEqual(counts, [2; 0; 0]);
            tc.verifyEqual(out(1), 0.5, 'AbsTol', 1e-12);
            tc.verifyTrue(all(isnan(out(2:3))));
        end
        function multiOutputReducer(tc)
            [~, out] = distanceBinStats([250 250], [1 0], 0:100:300, @(v) prctile(v,[0 50 100]));
            tc.verifyEqual(out(3,:), [0 0.5 1], 'AbsTol', 1e-12);
            tc.verifyTrue(all(isnan(out(1,:))));
        end
    end
end
