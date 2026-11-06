classdef test_binEdges < matlab.unittest.TestCase
%TEST_BINEDGES  Verify distance bin edge generation (Part B helper).

    methods (Test)

        function singleSeriesSpansToMax(tc)
            s = struct('dist_m', [100; 250; 490], 'available', [1;1;0], ...
                'margin_db', [], 'pSNR_db', [], 'key','', 'label','', ...
                'link','', 'dir','', 'ground','', 'category','', ...
                'req_snr_db',0, 'mcs_index',0, 'mcs_name','', 'fingers',0, 'rate_bps',0);
            edges = binEdges(s, 100);
            tc.verifyEqual(edges(1), 0);
            tc.verifyGreaterThanOrEqual(edges(end), 490);
            tc.verifyTrue(numel(edges) >= 2);
        end

        function multipleSeriesUsesGlobalMax(tc)
            s1 = makeMinSeries(300);
            s2 = makeMinSeries(700);
            edges = binEdges([s1 s2], 200);
            tc.verifyGreaterThanOrEqual(edges(end), 700);
        end

        function emptySeriesGivesMinimalEdges(tc)
            s = makeMinSeries([]);
            edges = binEdges(s, 100);
            tc.verifyTrue(numel(edges) >= 2);
        end

        function edgesAreMultiplesOfBinWidth(tc)
            s = makeMinSeries(550);
            edges = binEdges(s, 200);
            diffs = diff(edges);
            tc.verifyTrue(all(abs(diffs - 200) < 1e-9));
        end

    end
end

function s = makeMinSeries(dist)
    s = struct('dist_m',dist(:), 'available',[], 'margin_db',[], 'pSNR_db',[], ...
        'key','', 'label','', 'link','', 'dir','', 'ground','', 'category','', ...
        'req_snr_db',0, 'mcs_index',0, 'mcs_name','', 'fingers',0, 'rate_bps',0);
end
