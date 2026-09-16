classdef test_loadMcsBook < matlab.unittest.TestCase
%TEST_LOADMCSBOOK  loadMcsBook reads the DL/UL MCS lookup sheets.

    methods (Test)
        function readsBothSheets(tc)
            xlsx = [tempname '.xlsx'];
            cleanup = onCleanup(@() delete(xlsx));
            idx = (1:3)';
            mk = @(mul) table(idx, "MCS"+string(idx), idx*mul, idx+10, idx*5, ...
                'VariableNames', {'mcs_index','mcs','ibo_db','req_snr_db','rate_1finger_mbps'});
            writetable(mk(1), xlsx, 'Sheet', 'DL');
            writetable(mk(2), xlsx, 'Sheet', 'UL');

            mcsbook = loadMcsBook(xlsx);
            tc.verifyTrue(all(isfield(mcsbook, {'DL','UL'})));
            tc.verifyEqual(height(mcsbook.DL), 3);
            tc.verifyEqual(mcsbook.DL.ibo_db, [1;2;3], 'AbsTol', 1e-9);
            tc.verifyEqual(mcsbook.UL.ibo_db, [2;4;6], 'AbsTol', 1e-9);
        end

        function missingFileErrors(tc)
            tc.verifyError(@() loadMcsBook('/no/such/mcs_tables.xlsx'), 'loadMcsBook:notFound');
        end

        function missingColumnErrors(tc)
            xlsx = [tempname '.xlsx'];
            cleanup = onCleanup(@() delete(xlsx));
            T = table((1:3)', 'VariableNames', {'mcs_index'});
            writetable(T, xlsx, 'Sheet', 'DL');
            writetable(T, xlsx, 'Sheet', 'UL');
            tc.verifyError(@() loadMcsBook(xlsx), 'loadMcsBook:badSheet');
        end
    end
end
