classdef test_resolveLinkParams < matlab.unittest.TestCase
%TEST_RESOLVELINKPARAMS  MCS index/fingers/antenna-name -> resolved link params.

    properties
        mcsbook; finger; book; link;
    end

    methods (TestClassSetup)
        function setup(tc)
            idx = (1:3)';
            mkT = @(ibo, snr, rate) table(idx, ["A";"B";"C"], ibo, snr, rate, ...
                'VariableNames', {'mcs_index','mcs','ibo_db','req_snr_db','rate_1finger_mbps'});
            tc.mcsbook = struct( ...
                'DL', mkT([1;2;3], [10;20;30], [5;10;15]), ...
                'UL', mkT([4;5;6], [40;50;60], [1;2;3]));
            tc.finger = struct('bw_per_finger_hz', struct('DL',10e6,'UL',5e6));

            mk = @(nm) struct('name',nm,'ang_deg',[0;180],'gain_dbi',[0;0], ...
                'max_deg',180,'interp',griddedInterpolant([0 180],[0 0],'linear','nearest'));
            tc.book = [mk('tx1') mk('rx1')];

            tc.link = struct('name','L','dir','DL','ground','infantry', ...
                'tx_dev','air','rx_dev','infantry','tx_ant','tx1','rx_ant','rx1', ...
                'tx_side','air','rx_side','gnd','mcs_index',2,'fingers',3);
        end
    end

    methods (Test)
        function resolvesDL(tc)
            L = resolveLinkParams(tc.link, tc.mcsbook, tc.finger, tc.book);
            tc.verifyEqual(L.papr_db, 2);
            tc.verifyEqual(L.req_snr_db, 20);
            tc.verifyEqual(L.mcs_name, 'B');
            tc.verifyEqual(L.bw_hz, 3*10e6, 'AbsTol', 1e-6);
            tc.verifyEqual(L.rate_bps, 3*10e6, 'AbsTol', 1e-6);   % 3 fingers * 10 Mbps
            tc.verifyEqual(L.tx_ant_idx, 1);
            tc.verifyEqual(L.rx_ant_idx, 2);
        end

        function resolvesUL(tc)
            link = tc.link; link.dir = 'UL'; link.mcs_index = 1; link.fingers = 2;
            L = resolveLinkParams(link, tc.mcsbook, tc.finger, tc.book);
            tc.verifyEqual(L.papr_db, 4);
            tc.verifyEqual(L.req_snr_db, 40);
            tc.verifyEqual(L.bw_hz, 2*5e6, 'AbsTol', 1e-6);
            tc.verifyEqual(L.rate_bps, 2*1e6, 'AbsTol', 1e-6);
        end

        function unknownMcsErrors(tc)
            link = tc.link; link.mcs_index = 99;
            tc.verifyError(@() resolveLinkParams(link, tc.mcsbook, tc.finger, tc.book), ...
                'resolveLinkParams:mcs');
        end

        function unknownAntennaErrors(tc)
            link = tc.link; link.tx_ant = 'nope';
            tc.verifyError(@() resolveLinkParams(link, tc.mcsbook, tc.finger, tc.book), ...
                'resolveLinkParams:antenna');
        end
    end
end
