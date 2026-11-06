classdef test_resolveRegion < matlab.unittest.TestCase
%TEST_RESOLVEREGION  Verify region config -> resolved region struct.

    methods (Test)

        function overridesUsedWhenPresent(tc)
            rc = struct('name','test_region', 'osm_path','', ...
                'latlim_override',[33.0 33.5], 'lonlim_override',[35.0 35.5], ...
                'placement_diameter_km', 8, 'category', 'other');
            r = resolveRegion(rc, '/dummy');
            tc.verifyEqual(r.latlim, [33.0 33.5]);
            tc.verifyEqual(r.lonlim, [35.0 35.5]);
            tc.verifyEqual(r.center_lat, 33.25, 'AbsTol', 1e-9);
            tc.verifyEqual(r.center_lon, 35.25, 'AbsTol', 1e-9);
        end

        function nameAndCategoryPassedThrough(tc)
            rc = struct('name','my_region', 'osm_path','', ...
                'latlim_override',[33.0 33.5], 'lonlim_override',[35.0 35.5], ...
                'placement_diameter_km', 5, 'category', 'urban_suburban');
            r = resolveRegion(rc, '/dummy');
            tc.verifyEqual(r.name, 'my_region');
            tc.verifyEqual(r.category, 'urban_suburban');
        end

        function placementDiameterPassedThrough(tc)
            rc = struct('name','r', 'osm_path','', ...
                'latlim_override',[33 34], 'lonlim_override',[35 36], ...
                'placement_diameter_km', 12, 'category', '');
            r = resolveRegion(rc, '/dummy');
            tc.verifyEqual(r.placement_diameter_km, 12);
        end

        function missingPlacementDiameterIsEmpty(tc)
            rc = struct('name','r', 'osm_path','', ...
                'latlim_override',[33 34], 'lonlim_override',[35 36]);
            r = resolveRegion(rc, '/dummy');
            tc.verifyTrue(isempty(r.placement_diameter_km));
        end

        function missingCategoryIsEmpty(tc)
            rc = struct('name','r', 'osm_path','', ...
                'latlim_override',[33 34], 'lonlim_override',[35 36]);
            r = resolveRegion(rc, '/dummy');
            tc.verifyEqual(r.category, '');
        end

        function missingOsmWithNoOverridesErrors(tc)
            rc = struct('name','bad', 'osm_path','nonexistent.osm', ...
                'latlim_override',[], 'lonlim_override',[], ...
                'placement_diameter_km', 5, 'category', '');
            tc.verifyError(@() resolveRegion(rc, '/dummy'), ...
                'resolveRegion:osmMissing');
        end

    end
end
