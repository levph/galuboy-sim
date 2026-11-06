classdef test_defineRegions < matlab.unittest.TestCase
%TEST_DEFINEREGIONS  Verify the region definition struct array schema.

    properties
        regions;
    end

    methods (TestClassSetup)
        function loadRegions(tc)
            tc.regions = defineRegions();
        end
    end

    methods (Test)

        function correctCount(tc)
            tc.verifyEqual(numel(tc.regions), 6);
        end

        function requiredFieldsPresent(tc)
            req = {'name','osm_path','latlim_override','lonlim_override', ...
                   'placement_diameter_km','category'};
            for f = req
                tc.verifyTrue(isfield(tc.regions, f{1}), ...
                    sprintf('Missing field: %s', f{1}));
            end
        end

        function uniqueNames(tc)
            names = {tc.regions.name};
            tc.verifyEqual(numel(names), numel(unique(names)));
        end

        function validCategories(tc)
            valid = {'urban_suburban', 'other'};
            for r = tc.regions
                tc.verifyTrue(ismember(r.category, valid), ...
                    sprintf('%s has invalid category "%s"', r.name, r.category));
            end
        end

        function boundsAreValid(tc)
            for r = tc.regions
                if ~isempty(r.latlim_override)
                    tc.verifyTrue(r.latlim_override(2) > r.latlim_override(1), ...
                        sprintf('%s: latlim not ascending', r.name));
                end
                if ~isempty(r.lonlim_override)
                    tc.verifyTrue(r.lonlim_override(2) > r.lonlim_override(1), ...
                        sprintf('%s: lonlim not ascending', r.name));
                end
            end
        end

        function urbanSuburbanSmallDisk(tc)
            for r = tc.regions
                if strcmp(r.category, 'urban_suburban')
                    tc.verifyEqual(r.placement_diameter_km, 5, ...
                        sprintf('%s: urban_suburban should have 5 km disk', r.name));
                end
            end
        end

        function otherLargeDisk(tc)
            for r = tc.regions
                if strcmp(r.category, 'other')
                    tc.verifyEqual(r.placement_diameter_km, 10, ...
                        sprintf('%s: "other" category should have 10 km disk', r.name));
                end
            end
        end

        function threeUrbanThreeOther(tc)
            cats = {tc.regions.category};
            tc.verifyEqual(sum(strcmp(cats, 'urban_suburban')), 3);
            tc.verifyEqual(sum(strcmp(cats, 'other')), 3);
        end

    end
end
