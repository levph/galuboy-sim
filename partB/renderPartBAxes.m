function renderPartBAxes(ax, S, book, cfg, graphType, selectedLabels, mcsbook)
%RENDERPARTBAXES  Draw one Part B graph (selected series) into an axes.
%
%   renderPartBAxes(ax, S, book, cfg, graphType, selectedLabels, mcsbook)
%
%   The compute+select+draw core shared by the batch driver and the UI:
%     - analyzeLinks(S,cfg,book,mcsbook) -> buildSeries -> keep only selectedLabels
%     - draw the chosen graphType into ax.
%
%   graphType      'ccdf' | 'avail' | 'margin'
%   selectedLabels cellstr/string of series KEYS to show (from buildSeries,
%                  e.g. "DL_infantry / urban" - stable, no MCS/rate);
%                  [] / omitted = all series.
%   mcsbook        from loadMcsBook; defaults to loadMcsBook(cfg.mcs_book).
%
%   Axes-in so it works in a uiaxes (UI) or a normal axes (tests/batch).

    cla(ax, 'reset');

    if nargin < 7 || isempty(mcsbook)
        mcsbook = loadMcsBook(cfg.mcs_book);
    end

    R  = analyzeLinks(S, cfg, book, mcsbook);
    Sr = buildSeries(R);

    if nargin < 6 || isempty(selectedLabels)
        keep = true(1, numel(Sr));
    else
        keep = ismember({Sr.key}, cellstr(selectedLabels));
    end
    sub = Sr(keep);
    if isempty(sub)
        title(ax, '(no series selected)', 'FontSize', 36, 'FontWeight', 'bold');
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
