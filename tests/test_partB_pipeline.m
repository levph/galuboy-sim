classdef test_partB_pipeline < matlab.unittest.TestCase
%TEST_PARTB_PIPELINE  loadPartA + analyzeLinks on a synthetic Part A workbook.

    properties
        xlsx; antbook; cfg; book;
    end

    methods (TestClassSetup)
        function setup(tc)
            % --- synthetic 3-sheet Part A workbook (Nf=3, 4 devices) ------
            tc.xlsx = [tempname '.xlsx'];
            P = table(["frequency_hz";"num_samples"], ["4e9";"3"], ...
                      'VariableNames', {'name','value'});
            Rg = table([1;2], ["r1";"r2"], ["urban_suburban";"other"], ...
                      'VariableNames', {'region_id','name','category'});
            D = table((1:4)', ["r1";"r1";"r2";"r2"], [1;1;2;2], ...
                      ["infantry";"infantry";"vehicular";"vehicular"], ...
                      [1.5;1.5;2.5;2.5], [35;35;35.1;35.1], [33;33;33.1;33.1], ...
                      [100;500;200;800], ...
                      ["100,101,102";"110,111,112";"120,121,122";"130,131,132"], ...
                      ["10,11,12";"13,14,15";"16,17,18";"19,20,21"], ...
                      ["20,21,22";"23,24,25";"26,27,28";"29,30,31"], ...
                      'VariableNames', {'device_id','region','region_id','type', ...
                      'height_m','lon','lat','dist_m','pl_db','steer_gnd_deg','steer_air_deg'});
            writetable(P,  tc.xlsx, 'Sheet', 'params');
            writetable(Rg, tc.xlsx, 'Sheet', 'regions');
            writetable(D,  tc.xlsx, 'Sheet', 'devices');

            % --- temp antenna workbook: Angle + >=6 antenna columns -------
            tc.cfg = buildConfigB();
            nant = max([tc.cfg.links.tx_ant_idx, tc.cfg.links.rx_ant_idx]);
            tc.antbook = [tempname '.xlsx'];
            ang = [0; 90; 180];
            cols = num2cell(repmat(5, numel(ang), nant), 1);   % const 5 dBi each
            T = table(ang, cols{:}, ...
                'VariableNames', ['Angle', "ant"+string(1:nant)]);
            writetable(T, tc.antbook);
            tc.book = loadAntennaBook(tc.antbook);
        end
    end
    methods (TestClassTeardown)
        function cleanup(tc)
            if exist(tc.xlsx,'file'), delete(tc.xlsx); end
            if exist(tc.antbook,'file'), delete(tc.antbook); end
        end
    end

    methods (Test)
        function loadsAndExpandsArrays(tc)
            S = loadPartA(tc.xlsx);
            tc.verifyEqual(S.Nf, 3);
            tc.verifyEqual(numel(S.device.id), 4);
            tc.verifyEqual(S.device.pl_db(1,:), [100 101 102], 'AbsTol', 1e-9);
            tc.verifyEqual(S.device.category(1), "urban_suburban");   % via region_id map
            tc.verifyEqual(S.device.category(3), "other");
        end

        function analyzeAllFourLinks(tc)
            S = loadPartA(tc.xlsx);
            R = analyzeLinks(S, tc.cfg, tc.book);
            tc.verifyEqual(numel(R), 4);
            % DL_infantry / UL_infantry select the 2 infantry devices
            inf_links = R(strcmp({R.ground},'infantry'));
            for L = inf_links
                tc.verifyEqual(numel(L.dist_m), 2);
                tc.verifyEqual(numel(L.pSNR_db), 2);
                tc.verifyTrue(all(isfinite(L.pSNR_db)));     % no outage here
                tc.verifyEqual(numel(L.available), 2);
            end
            veh_links = R(strcmp({R.ground},'vehicular'));
            for L = veh_links
                tc.verifyEqual(numel(L.dist_m), 2);
            end
        end
    end
end
