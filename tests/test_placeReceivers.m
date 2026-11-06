classdef test_placeReceivers < matlab.unittest.TestCase
%TEST_PLACERECEIVERS  Verify RX placement within disk/bbox.

    properties
        region;
        rx_cfg;
    end

    methods (TestClassSetup)
        function setup(tc)
            tc.region = struct('center_lat', 33.25, 'center_lon', 35.25, ...
                'latlim', [33.0 33.5], 'lonlim', [35.0 35.5]);
            tc.rx_cfg = struct( ...
                'placement_diameter_km', 5, ...
                'infantry', struct('count', 10, 'height_m', 1.5, 'antenna_name_dl', 'inf_rx'), ...
                'vehicular', struct('count', 5, 'height_m', 2.5, 'antenna_name_dl', 'veh_rx'));
        end
    end

    methods (Test)

        function correctTotalCount(tc)
            [rx_array, rx_table] = placeReceivers(tc.region, tc.rx_cfg);
            tc.verifyEqual(numel(rx_array), 15);
            tc.verifyEqual(height(rx_table), 15);
        end

        function infantryFirst(tc)
            [~, rx_table] = placeReceivers(tc.region, tc.rx_cfg);
            types = rx_table.type;
            tc.verifyTrue(all(types(1:10) == 'infantry'));
            tc.verifyTrue(all(types(11:15) == 'vehicular'));
        end

        function correctHeights(tc)
            [~, rx_table] = placeReceivers(tc.region, tc.rx_cfg);
            tc.verifyTrue(all(rx_table.height_m(1:10) == 1.5));
            tc.verifyTrue(all(rx_table.height_m(11:15) == 2.5));
        end

        function withinDisk(tc)
            rng(42);
            [~, rx_table] = placeReceivers(tc.region, tc.rx_cfg);
            d = haversineDistance(tc.region.center_lat, tc.region.center_lon, ...
                rx_table.lat(:).', rx_table.lon(:).');
            radius_m = (tc.rx_cfg.placement_diameter_km / 2) * 1000;
            tc.verifyLessThanOrEqual(max(d), radius_m + 1);
        end

        function bboxFallback(tc)
            cfg = tc.rx_cfg;
            cfg.placement_diameter_km = 0;
            rng(42);
            [~, rx_table] = placeReceivers(tc.region, cfg);
            tc.verifyGreaterThanOrEqual(min(rx_table.lon), tc.region.lonlim(1));
            tc.verifyLessThanOrEqual(max(rx_table.lon), tc.region.lonlim(2));
            tc.verifyGreaterThanOrEqual(min(rx_table.lat), tc.region.latlim(1));
            tc.verifyLessThanOrEqual(max(rx_table.lat), tc.region.latlim(2));
        end

        function zeroCountErrors(tc)
            cfg = tc.rx_cfg;
            cfg.infantry.count = 0;
            cfg.vehicular.count = 0;
            tc.verifyError(@() placeReceivers(tc.region, cfg), ...
                'placeReceivers:noReceivers');
        end

        function antennaNameColumn(tc)
            [~, rx_table] = placeReceivers(tc.region, tc.rx_cfg);
            tc.verifyTrue(ismember('antenna_name_dl', rx_table.Properties.VariableNames));
            tc.verifyEqual(string(rx_table.antenna_name_dl(1)), "inf_rx");
            tc.verifyEqual(string(rx_table.antenna_name_dl(11)), "veh_rx");
        end

        function noBuildingRejectionByDefault(tc)
            rng(1);
            [~, t1] = placeReceivers(tc.region, tc.rx_cfg);
            rng(1);
            [~, t2] = placeReceivers(tc.region, tc.rx_cfg, {});
            tc.verifyEqual(t1.lon, t2.lon);
        end

    end
end
