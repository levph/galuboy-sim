classdef test_buildSeries < matlab.unittest.TestCase
%TEST_BUILDSERIES  Verify the link x terrain series splitter (Part B).

    methods (Test)

        function emptyCategorySkipped(tc)
            R = makeR('DL_infantry', 'DL', 'infantry', ...
                [100;200;300], strings(3,1) + "urban_suburban", ...
                [1;1;0], [5;3;-2], [15;13;8]);
            S = buildSeries(R);
            tc.verifyEqual(numel(S), 1);
            tc.verifyEqual(S(1).category, 'urban_suburban');
        end

        function bothCategoriesProduceTwoSeries(tc)
            cats = ["urban_suburban";"urban_suburban";"other";"other"];
            R = makeR('DL_infantry', 'DL', 'infantry', ...
                [100;200;300;400], cats, [1;1;0;1], [5;3;-2;4], [15;13;8;14]);
            S = buildSeries(R);
            tc.verifyEqual(numel(S), 2);
            tc.verifyTrue(any(strcmp({S.category}, 'urban_suburban')));
            tc.verifyTrue(any(strcmp({S.category}, 'other')));
        end

        function fieldsPopulated(tc)
            R = makeR('UL_vehicular', 'UL', 'vehicular', ...
                [500], "other", true, 6, 16);
            S = buildSeries(R);
            tc.verifyEqual(S(1).link, 'UL_vehicular');
            tc.verifyEqual(S(1).dir, 'UL');
            tc.verifyEqual(S(1).ground, 'vehicular');
            tc.verifyFalse(isempty(S(1).label));
            tc.verifyFalse(isempty(S(1).key));
        end

        function multipleLinksMultiplied(tc)
            R1 = makeR('DL_infantry', 'DL', 'infantry', ...
                [100;200], ["urban_suburban";"other"], [1;0], [5;-2], [15;8]);
            R2 = makeR('UL_infantry', 'UL', 'infantry', ...
                [100;200], ["urban_suburban";"other"], [1;1], [3;4], [13;14]);
            S = buildSeries([R1 R2]);
            tc.verifyEqual(numel(S), 4);
        end

    end
end

function R = makeR(name, dir, ground, dist_m, category, available, margin_db, pSNR_db)
    R = struct('name',name, 'dir',dir, 'ground',ground, ...
        'req_snr_db',10, 'mcs_index',6, 'mcs_name','QPSK 1/2', ...
        'fingers',1, 'rate_bps',10e6, ...
        'dist_m',dist_m(:), 'category',category(:), ...
        'available',logical(available(:)), ...
        'margin_db',margin_db(:), 'pSNR_db',pSNR_db(:));
end
