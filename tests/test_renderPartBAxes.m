classdef test_renderPartBAxes < matlab.unittest.TestCase
%TEST_RENDERPARTBAXES  The UI render core draws the selected series into an axes.

    properties
        S; book; cfg;
    end

    methods (TestClassSetup)
        function setup(tc)
            rng(2);
            n = 40; Nf = 6;
            d.type     = [repmat("infantry",n/2,1);  repmat("vehicular",n/2,1)];
            d.category = repmat(["urban_suburban";"other"], n/2, 1);
            d.dist_m   = (1:n)' * 60;
            d.pl_db        = 100 + 8*rand(n, Nf);
            d.steer_air_deg = 30 * rand(n, Nf);
            d.steer_gnd_deg = 20 * rand(n, Nf);
            tc.S = struct('Nf', Nf, 'device', d);

            mk = @(g) struct('name','x','ang_deg',[0;180],'gain_dbi',[g;g], ...
                'max_deg',180,'interp',griddedInterpolant([0 180],[g g],'linear','nearest'));
            tc.book = repmat(mk(5), 1, 6);     % 6 constant patterns (indices 1..6)
            tc.cfg  = buildConfigB();
        end
    end

    methods (Test)
        function eachGraphDrawsLines(tc)
            for g = {'ccdf','avail','margin'}
                ax = axes('Parent', figure('Visible','off'));
                co = onCleanup(@() close(ancestor(ax,'figure')));
                renderPartBAxes(ax, tc.S, tc.book, tc.cfg, g{1}, []);
                tc.verifyGreaterThan(numel(findobj(ax,'Type','line')), 0);
                clear co;
            end
        end

        function selectionFiltersSeries(tc)
            ax = axes('Parent', figure('Visible','off'));
            co = onCleanup(@() close(ancestor(ax,'figure')));
            renderPartBAxes(ax, tc.S, tc.book, tc.cfg, 'ccdf', {'DL_infantry / urban'});
            tc.verifyEqual(numel(findobj(ax,'Type','line')), 1);   % just the one series
        end
    end
end
