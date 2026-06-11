classdef test_partB_plots < matlab.unittest.TestCase
%TEST_PARTB_PLOTS  buildSeries + the 3 per-axes plotters run without error.

    methods (Static)
        function R = fakeR()
            rng(1);
            mk = @(name,dir,grnd) struct('name',name,'dir',dir,'ground',grnd, ...
                'req_snr_db',5, ...
                'dist_m',[ (1:60)'*40 ], ...
                'category', [repmat("urban_suburban",30,1); repmat("other",30,1)], ...
                'pSNR_db', (20 - (1:60)'*0.2 + randn(60,1)), ...
                'available', ((20 - (1:60)'*0.2 + randn(60,1)) >= 5), ...
                'margin_db', (20 - (1:60)'*0.2 + randn(60,1)) - 5);
            R = [mk('DL_infantry','DL','infantry'), mk('DL_vehicular','DL','vehicular'), ...
                 mk('UL_infantry','UL','infantry'), mk('UL_vehicular','UL','vehicular')];
        end
    end

    methods (Test)
        function buildSeriesSplitsByTerrain(tc)
            S = buildSeries(test_partB_plots.fakeR());
            tc.verifyEqual(numel(S), 8);                 % 4 links x 2 terrains
            tc.verifyTrue(any(contains({S.label}, 'urban')));
            tc.verifyTrue(any(contains({S.label}, 'open/mtn')));
        end

        function plottersDrawLines(tc)
            S = buildSeries(test_partB_plots.fakeR());
            sub = S(strcmp({S.link},'DL_infantry'));     % 2 terrain series
            f = figure('Visible','off'); c = onCleanup(@() close(f));
            for fn = {@plotAvailabilityCCDF, @plotAvailabilityHistogram, @plotMarginHistogram}
                ax = axes('Parent', figure('Visible','off'));
                fc = onCleanup(@() close(ancestor(ax,'figure')));
                fn{1}(ax, sub, struct('bin_m',100,'min_samples',3));
                tc.verifyGreaterThan(numel(findobj(ax,'Type','line')), 0);
                clear fc;
            end
        end
    end
end
