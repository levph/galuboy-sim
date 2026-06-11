classdef test_numArrayToStr < matlab.unittest.TestCase
%TEST_NUMARRAYTOSTR  Verify numeric-array <-> comma-string round-trip.

    methods (Test)

        function roundTrip(tc)
            v = [116.234 117.0 25.5 0.1];
            s = numArrayToStr(v);
            back = str2double(split(s, ",")).';
            tc.verifyEqual(back, v, 'AbsTol', 1e-6);
        end

        function preservesInfOutage(tc)
            s = numArrayToStr([116.2 Inf 90]);
            back = str2double(split(s, ",")).';
            tc.verifyTrue(isinf(back(2)));
            tc.verifyEqual(back([1 3]), [116.2 90], 'AbsTol', 1e-6);
        end

        function emptyGivesEmptyString(tc)
            tc.verifyEqual(numArrayToStr([]), "");
        end

        function isScalarString(tc)
            s = numArrayToStr([1 2 3]);
            tc.verifyClass(s, 'string');
            tc.verifyEqual(numel(s), 1);
            tc.verifyEqual(s, "1.000000,2.000000,3.000000");
        end

    end
end
