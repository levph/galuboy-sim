function renderPartBAxes(ax, S, book, cfg, graphType, selectedLabels)
%RENDERPARTBAXES  Draw one Part B graph (selected series) into an axes.
%
%   renderPartBAxes(ax, S, book, cfg, graphType, selectedLabels)
%
%   The compute+select+draw core shared by the batch driver and the UI:
%     - analyzeLinks(S,cfg,book) -> buildSeries -> keep only selectedLabels
%     - draw the chosen graphType into ax.
%
%   graphType      'ccdf' | 'avail' | 'margin'
%   selectedLabels cellstr/string of series labels to show (from buildSeries,
%                  e.g. "DL_infantry / urban"); [] / omitted = all series.
%
%   Axes-in so it works in a uiaxes (UI) or a normal axes (tests/batch).

    cla(ax, 'reset');

    R  = analyzeLinks(S, cfg, book);
    Sr = buildSeries(R);

    if nargin < 6 || isempty(selectedLabels)
        keep = true(1, numel(Sr));
    else
        keep = ismember({Sr.label}, cellstr(selectedLabels));
    end
    sub = Sr(keep);
    if isempty(sub)
        title(ax, '(no series selected)');
        return;
    end

    opts = struct('bin_m',       getdef(cfg.plot, 'dist_bin_m', 100), ...
                  'min_samples', getdef(cfg.plot, 'hist_min_samples', 5));

    switch lower(char(graphType))
        case 'ccdf',   plotAvailabilityCCDF(ax, sub, opts);
        case 'avail',  plotAvailabilityHistogram(ax, sub, opts);
        case 'margin', plotMarginHistogram(ax, sub, opts);
        otherwise
            error('renderPartBAxes:graph', 'unknown graphType "%s"', char(graphType));
    end
end
