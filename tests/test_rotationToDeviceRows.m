classdef test_rotationToDeviceRows < matlab.unittest.TestCase
%TEST_ROTATIONTODEVICEROWS  Verify one-row-per-device flattening with arrays.

    methods (Static)
        function rot = fakeRot()
            rot.pl_db         = [1 2 3; 4 5 6];        % Nf=2, n_rx=3 (cols = devices)
            rot.steer_gnd_deg = [10 20 30; 40 50 60];
            rot.steer_air_deg = [11 21 31; 41 51 61];
            rot.rx_table = table([35;36;37], [33.0;33.1;33.2], ...
                categorical(["infantry";"vehicular";"infantry"]), [1.5;2.5;1.5], ...
                'VariableNames', {'lon','lat','type','height_m'});
            rot.dist_m      = [100 200 300];
            rot.region_name = 'testreg';
        end
    end

    methods (Test)

        function oneRowPerDevice(tc)
            T = rotationToDeviceRows(test_rotationToDeviceRows.fakeRot(), 5, 10);
            tc.verifyEqual(height(T), 3);
            tc.verifyEqual(T.device_id, [11;12;13]);     % id_offset = 10
            tc.verifyTrue(all(T.region_id == 5));
            tc.verifyTrue(all(T.region == "testreg"));
        end

        function arrayColumnsHoldPerStepData(tc)
            T = rotationToDeviceRows(test_rotationToDeviceRows.fakeRot(), 5, 0);
            % device 1 = column 1 of each matrix -> [1;4], [10;40], [11;41]
            tc.verifyEqual(str2double(split(T.pl_db(1), ",")).',         [1 4],   'AbsTol', 1e-9);
            tc.verifyEqual(str2double(split(T.steer_gnd_deg(1), ",")).', [10 40], 'AbsTol', 1e-9);
            tc.verifyEqual(str2double(split(T.steer_air_deg(1), ",")).', [11 41], 'AbsTol', 1e-9);
            % device 3 = column 3 -> [3;6]
            tc.verifyEqual(str2double(split(T.pl_db(3), ",")).', [3 6], 'AbsTol', 1e-9);
        end

        function staticFieldsCarried(tc)
            T = rotationToDeviceRows(test_rotationToDeviceRows.fakeRot(), 5, 0);
            tc.verifyEqual(string(T.type(2)), "vehicular");
            tc.verifyEqual(T.height_m(2), 2.5);
            tc.verifyEqual(T.lon(2), 36);
            tc.verifyEqual(T.dist_m(2), 200);
        end

    end
end
