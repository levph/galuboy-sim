classdef test_renderPartBAxes < matlab.unittest.TestCase
%TEST_RENDERPARTBAXES  The UI render core draws the selected series into an axes.

    properties
        S; book; cfg; mcsbook;
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

            tc.cfg = buildConfigB();
            names = unique([{tc.cfg.links.tx_ant}, {tc.cfg.links.rx_ant}], 'stable');
            mk = @(nm) struct('name',nm,'ang_deg',[0;180],'gain_dbi',[5;5], ...
                'max_deg',180,'interp',griddedInterpolant([0 180],[5 5],'linear','nearest'));
            tc.book = arrayfun(@(i) mk(names{i}), 1:numel(names));

            idx = (1:11)';
            mkT = @() table(idx, "MCS"+string(idx), idx+0.5, idx+1, idx*2, ...
                'VariableNames', {'mcs_index','mcs','ibo_db','req_snr_db','rate_1finger_mbps'});
            tc.mcsbook = struct('DL', mkT(), 'UL', mkT());
        end
    end

    methods (Test)
        function eachGraphDrawsLines(tc)
            for g = {'ccdf','avail','margin'}
                ax = axes('Parent', figure('Visible','off'));
                co = onCleanup(@() close(ancestor(ax,'figure')));
                renderPartBAxes(ax, tc.S, tc.book, tc.cfg, g{1}, [], tc.mcsbook);
                tc.verifyGreaterThan(numel(findobj(ax,'Type','line')), 0);
                clear co;
            end
        end

        function selectionFiltersSeries(tc)
            ax = axes('Parent', figure('Visible','off'));
            co = onCleanup(@() close(ancestor(ax,'figure')));
            renderPartBAxes(ax, tc.S, tc.book, tc.cfg, 'ccdf', {'DL_infantry / urban'}, tc.mcsbook);
            tc.verifyEqual(numel(findobj(ax,'Type','line')), 1);   % just the one series
        end
    end
end
