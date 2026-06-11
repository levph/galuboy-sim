function figs = plotPartB(R, cfg)
%PLOTPARTB  Generate the Part B graph suite from analyzeLinks output.
%
%   figs = plotPartB(R, cfg)
%
%   R from analyzeLinks; cfg from buildConfigB (uses cfg.plot.*). Produces, for
%   each direction (DL/UL) and each of the 3 graph types, one figure laid out as
%   one tile per ground-device link type, each tile overlaying the 2 terrain
%   aggregates (urban vs open/mountainous):
%
%     graph types: availability CCDF vs distance
%                  availability histogram (fixed distance bins)
%                  margin (median + spread) vs distance
%
%   => 2 directions x 3 graph types = 6 figures. Returns the figure handles.
%   (The Step 7 UI reuses the same per-axes plotters with tick-box series
%   selection; this driver is the no-UI batch view.)

    S = buildSeries(R);

    opts = struct('bin_m', getdef(cfg.plot,'dist_bin_m',100), ...
                  'min_samples', getdef(cfg.plot,'hist_min_samples',5));

    graphs = { 'Availability CCDF vs distance', @plotAvailabilityCCDF; ...
               'Availability vs distance',      @plotAvailabilityHistogram; ...
               'Margin vs distance',            @plotMarginHistogram };
    dirs = {'DL','UL'};

    figs = gobjects(0);
    for di = 1:numel(dirs)
        d = dirs{di};
        links = unique({S(strcmp({S.dir}, d)).link}, 'stable');
        if isempty(links), continue; end
        for gi = 1:size(graphs,1)
            fn = graphs{gi,2};
            f  = figure('Name', sprintf('%s — %s', d, graphs{gi,1}), 'Color', 'w');
            t  = tiledlayout(f, 1, numel(links), 'TileSpacing','compact','Padding','compact');
            for li = 1:numel(links)
                ax  = nexttile(t);
                sub = S(strcmp({S.link}, links{li}));    % the terrain series for this link
                fn(ax, sub, opts);
                title(ax, links{li});                    % per-tile = link type
            end
            title(t, sprintf('%s — %s', d, graphs{gi,1}));
            figs(end+1) = f; %#ok<AGROW>
        end
    end
end
