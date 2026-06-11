classdef test_computeLinkAvailability < matlab.unittest.TestCase
%TEST_COMPUTELINKAVAILABILITY  Verify per-device pSNR / availability / margin.

    methods (Test)
        function medianAvailabilityMargin(tc)
            SNR = [0 10 20 30; 5 5 5 5];     % n=2, Nf=4
            out = computeLinkAvailability(SNR, 10, 50);   % p=50 -> median
            tc.verifyEqual(out.pSNR_db, [15; 5], 'AbsTol', 1e-9);
            tc.verifyEqual(out.available, [true; false]);
            tc.verifyEqual(out.margin_db, [5; -5], 'AbsTol', 1e-9);
        end
        function outageGivesUnavailable(tc)
            SNR = [-Inf -Inf 10 20];         % >50% outage -> median -Inf
            out = computeLinkAvailability(SNR, 0, 50);
            tc.verifyEqual(out.pSNR_db, -Inf);
            tc.verifyFalse(out.available);
        end
        function higherPercentileIsStricter(tc)
            SNR = 0:99;                       % 1 device, 100 pts
            o90 = computeLinkAvailability(SNR, 0, 90);
            o99 = computeLinkAvailability(SNR, 0, 99);
            tc.verifyLessThan(o99.pSNR_db, o90.pSNR_db);
        end
    end
end
