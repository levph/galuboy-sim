classdef test_buildingRejectMask < matlab.unittest.TestCase
%TEST_BUILDINGREJECTMASK  Point-in-buildings tester + RX building rejection.
%
%   Covers buildingRejectMask (fast tester) on a synthetic square, its
%   vectorised behaviour, the legacy polygon-cell path through placeReceivers,
%   and an end-to-end check that placeReceivers with rejection enabled never
%   drops a real RX inside a real region's footprint (urban_suburban_03).
%
%   No Site Viewer / raytracing here - point-in-polygon only, so it's fast.

    methods (Test)

        % --- buildingRejectMask on a known square (legacy cell input) ------
        function squareInsideOutside(tc)
            sq = [0 0; 1 0; 1 1; 0 1; 0 0];     % unit square (lon,lat)
            tester = buildingRejectMask({sq});
            tc.verifyTrue(tester(0.5, 0.5));     % centre -> inside
            tc.verifyFalse(tester(2, 2));        % far away -> outside
            tc.verifyFalse(tester(-0.1, 0.5));   % just left -> outside
        end

        function vectorisedShapePreserved(tc)
            sq = [0 0; 2 0; 2 2; 0 2; 0 0];
            tester = buildingRejectMask({sq});
            lon = [1 5; -1 1];                   % 2x2, two inside two outside
            lat = [1 1;  1 5];
            tf  = tester(lon, lat);
            tc.verifyEqual(size(tf), size(lon));
            tc.verifyEqual(tf, logical([1 0; 0 0]));
        end

        function multipleBuildings(tc)
            a = [0 0; 1 0; 1 1; 0 1; 0 0];
            b = [10 10; 11 10; 11 11; 10 11; 10 10];
            tester = buildingRejectMask({a, b});
            tc.verifyTrue(tester(0.5, 0.5));
            tc.verifyTrue(tester(10.5, 10.5));
            tc.verifyFalse(tester(5, 5));
        end

        function emptyBuildingsNoReject(tc)
            tester = buildingRejectMask({});
            tc.verifyFalse(tester(0.5, 0.5));
            tc.verifyEqual(tester([1 2 3], [1 2 3]), false(1,3));
        end

        % --- placeReceivers honours a function-handle tester --------------
        function placeReceiversRejectsHandle(tc)
            % Disc fully covered by one big "building": every sample is inside,
            % so every RX hits the max-attempts fallback (warning) but the call
            % still returns the requested count.
            region = tc.fakeRegion();
            big = [region.lonlim(1)-1 region.latlim(1)-1;
                   region.lonlim(2)+1 region.latlim(1)-1;
                   region.lonlim(2)+1 region.latlim(2)+1;
                   region.lonlim(1)-1 region.latlim(2)+1;
                   region.lonlim(1)-1 region.latlim(1)-1];
            tester = buildingRejectMask({big});
            rx_cfg = tc.fakeRxCfg(3, 3);
            warning('off', 'placeReceivers:buildingReject');
            c = onCleanup(@() warning('on', 'placeReceivers:buildingReject'));
            [~, T] = placeReceivers(region, rx_cfg, tester);
            tc.verifyEqual(height(T), 6);
        end

        % --- end-to-end on a REAL region ----------------------------------
        function realRegionNoneInsideBuilding(tc)
            tc.assumeTrue(exist('readgeotable','file') == 2, ...
                'needs R2025a Mapping Toolbox (geojson parse is fine, but gate consistently)');
            root = fileparts(fileparts(mfilename('fullpath')));
            g = fullfile(root, 'resources', 'buildings', 'urban_suburban_03.geojson');
            tc.assumeTrue(exist(g, 'file') == 2, 'urban_suburban_03.geojson missing');

            tester = buildingRejectMask(g);

            % Region disc centred on the dense footprint cluster so rejection
            % actually fires. Bounds from defineRegions(urban_suburban_03).
            region = struct('name','urban_suburban_03', ...
                'latlim',[33.146402 33.196452], 'lonlim',[35.491468 35.551288], ...
                'center_lat', 33.1503, 'center_lon', 35.4970, ...
                'placement_diameter_km', 1);
            rx_cfg = tc.fakeRxCfg(15, 15);   % 30 RX, modest for speed

            warning('off', 'placeReceivers:buildingReject');
            c = onCleanup(@() warning('on', 'placeReceivers:buildingReject'));
            [~, T] = placeReceivers(region, rx_cfg, tester);

            % Re-test the placed points: none should be inside any footprint.
            inside = tester(T.lon, T.lat);
            tc.verifyFalse(any(inside), ...
                sprintf('%d of %d placed RX landed inside a building', sum(inside), numel(inside)));
        end

    end

    methods (Static)
        function region = fakeRegion()
            region = struct('name','fake', ...
                'latlim',[0 0.01], 'lonlim',[0 0.01], ...
                'center_lat', 0.005, 'center_lon', 0.005, ...
                'placement_diameter_km', 0.5);
        end

        function rx_cfg = fakeRxCfg(n_inf, n_veh)
            rx_cfg.placement_diameter_km = 1;
            rx_cfg.infantry.count    = n_inf;
            rx_cfg.infantry.height_m = 1.5;
            rx_cfg.vehicular.count    = n_veh;
            rx_cfg.vehicular.height_m = 2.5;
        end
    end
end
