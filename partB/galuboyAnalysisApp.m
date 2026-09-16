function fig = galuboyAnalysisApp(partA_path, antenna_path, mcs_path)
%GALUBOYANALYSISAPP  Interactive Part B analysis UI (R2021b+).
%
%   galuboyAnalysisApp()
%   galuboyAnalysisApp(partA_xlsx)
%   galuboyAnalysisApp(partA_xlsx, antenna_xlsx)
%   galuboyAnalysisApp(partA_xlsx, antenna_xlsx, mcs_xlsx)
%
%   Loads a Part A workbook + the antenna workbook + the MCS workbook and lets
%   you explore the link analysis: pick the graph (availability CCDF /
%   availability histogram / margin), set the availability percentile, tick
%   which (link x terrain) series to overlay, and edit per-link MCS / finger
%   count / TX+RX antenna (all as dropdowns) in a table. Everything recomputes
%   through analyzeLinks and redraws via renderPartBAxes (the same per-axes
%   plotters as the batch view).
%
%   IBO and required-SNR come from the MCS + link direction (same across
%   ground device types); BW and rate scale with finger count off the
%   direction's organic (1-finger) baseline (config.finger, buildConfigB).
%   The non-link constants (tx powers, losses, noise figures) come from
%   buildConfigB - edit those for the rest.
%
%   "Save plot..." exports the current axes to <repo>/results/plots/ (created
%   if missing) under an auto-generated name encoding the graph type, the
%   link(s) shown, the availability percentile, and a timestamp - no dialog,
%   so it's a one-click save while exploring.

    if nargin < 1, partA_path   = ''; end
    if nargin < 2, antenna_path = ''; end
    if nargin < 3, mcs_path     = ''; end

    cfg  = buildConfigB();
    repo = fileparts(fileparts(mfilename('fullpath')));   % partB/ -> repo

    bookPath = resolvePath(antenna_path, cfg.antenna_book, repo);
    mcsPath  = resolvePath(mcs_path,     cfg.mcs_book,     repo);

    book = [];
    if exist(bookPath,'file') == 2, book = loadAntennaBook(bookPath); end
    mcsbook = [];
    if exist(mcsPath,'file') == 2, mcsbook = loadMcsBook(mcsPath); end

    S = [];
    if nargin >= 1 && ~isempty(partA_path) && exist(partA_path,'file') == 2
        S = loadPartA(partA_path);
    end

    graphNames = {'Availability CCDF','Availability histogram','Margin vs distance'};
    graphKeys  = {'ccdf','avail','margin'};
    labels     = seriesLabels(cfg);

    fig = uifigure('Name','galuboy — Part B analysis','Position',[80 80 1200 720]);
    G = uigridlayout(fig,[1 2]); G.ColumnWidth = {420,'1x'}; G.RowHeight = {'1x'};

    L = uigridlayout(G,[14 1]);
    L.RowHeight = {30,22,30,22,30,32,32,22,150,22,170,34,34,22};
    L.Layout.Row = 1; L.Layout.Column = 1;

    uibutton(L,'Text','Load Part A workbook…','ButtonPushedFcn',@onLoadA);
    lblFile = uilabel(L,'Text', fileLabel(partA_path), 'FontColor',[.35 .35 .35]);
    uibutton(L,'Text','Load antenna workbook…','ButtonPushedFcn',@onLoadAnt);
    lblAnt  = uilabel(L,'Text', fileLabel(bookPath), 'FontColor',[.35 .35 .35]);
    uibutton(L,'Text','Load MCS workbook…','ButtonPushedFcn',@onLoadMcs);

    gG = uigridlayout(L,[1 2]); gG.ColumnWidth = {110,'1x'}; gG.Padding = [0 0 0 0];
    uilabel(gG,'Text','Graph');
    ddGraph = uidropdown(gG,'Items',graphNames,'ValueChangedFcn',@(~,~)doPlot());

    pG = uigridlayout(L,[1 2]); pG.ColumnWidth = {110,'1x'}; pG.Padding = [0 0 0 0];
    uilabel(pG,'Text','Availability %');
    pctField = uieditfield(pG,'numeric','Value',cfg.percentile,'Limits',[1 100], ...
                           'ValueChangedFcn',@(~,~)doPlot());

    uilabel(L,'Text','Series to show:','FontWeight','bold');
    cbGrid = uigridlayout(L,[ceil(numel(labels)/2) 2]); cbGrid.Padding = [0 0 0 0];
    cbx = gobjects(1,numel(labels));
    for i = 1:numel(labels)
        cbx(i) = uicheckbox(cbGrid,'Text',labels{i},'Value',true,'ValueChangedFcn',@(~,~)doPlot());
    end

    uilabel(L,'Text','Link parameters (editable):','FontWeight','bold');
    tbl = uitable(L);
    tbl.ColumnName     = {'link','MCS','fingers','tx antenna','rx antenna'};
    tbl.ColumnEditable = [false true true true true];
    setTableFormats();
    tbl.Data           = linksToTable(cfg, mcsbook);
    tbl.CellEditCallback = @(~,~)doPlot();

    uibutton(L,'Text','Update plot','ButtonPushedFcn',@(~,~)doPlot());

    uibutton(L,'Text','Save plot…','ButtonPushedFcn',@onSavePlot);
    lblSave = uilabel(L,'Text','','FontColor',[.35 .35 .35]);

    ax = uiaxes(G); ax.Layout.Row = 1; ax.Layout.Column = 2;

    doPlot();
    if nargout == 0, clear fig; end

    % ---------------- nested callbacks ----------------
    function onLoadA(~,~)
        [f,p] = uigetfile({'*.xlsx','Part A workbook'},'Select Part A output');
        if isequal(f,0), return; end
        S = loadPartA(fullfile(p,f)); lblFile.Text = f; doPlot();
    end
    function onLoadAnt(~,~)
        [f,p] = uigetfile({'*.xlsx;*.csv','Antenna workbook'},'Select antenna patterns');
        if isequal(f,0), return; end
        book = loadAntennaBook(fullfile(p,f)); lblAnt.Text = f;
        setTableFormats(); tbl.Data = linksToTable(cfg, mcsbook); doPlot();
    end
    function onLoadMcs(~,~)
        [f,p] = uigetfile({'*.xlsx','MCS workbook'},'Select MCS table');
        if isequal(f,0), return; end
        mcsbook = loadMcsBook(fullfile(p,f));
        tbl.Data = linksToTable(cfg, mcsbook); doPlot();
    end
    function setTableFormats()
        antNames = {}; if ~isempty(book), antNames = {book.name}; end
        fingerOpts = arrayfun(@num2str, 1:8, 'UniformOutput', false);
        tbl.ColumnFormat = {[], mcsNameList(), fingerOpts, antNames, antNames};
    end
    function names = mcsNameList()
        if isempty(mcsbook), names = {}; return; end
        names = cellstr(mcsbook.DL.mcs)';
    end
    function onSavePlot(~,~)
        if isempty(S) || isempty(book) || isempty(mcsbook)
            lblSave.Text = 'Nothing to save yet.';
            return;
        end
        outDir = fullfile(repo, 'results', 'plots');
        if exist(outDir,'dir') ~= 7, mkdir(outDir); end

        graphKey  = graphKeys{ strcmp(graphNames, ddGraph.Value) };
        ticked    = labels(arrayfun(@(c) logical(c.Value), cbx));
        linkNames = unique(cellfun(@linkNameOf, ticked, 'UniformOutput', false), 'stable');
        allLinks  = unique({cfg.links.name}, 'stable');
        if isempty(linkNames)
            linkPart = 'none';
        elseif isequal(sort(linkNames), sort(allLinks))
            linkPart = 'all';
        else
            linkPart = strjoin(linkNames, '+');
        end

        stamp = char(datetime('now','Format','yyyyMMdd_HHmmss'));
        fname = sprintf('partB_%s_%s_p%d_%s.png', graphKey, linkPart, round(pctField.Value), stamp);
        fpath = fullfile(outDir, fname);

        exportgraphics(ax, fpath, 'Resolution', 200, 'BackgroundColor', 'white');
        lblSave.Text = ['Saved: ' fname];
        fprintf('Saved plot: %s\n', fpath);
    end
    function doPlot()
        if isempty(S),       cla(ax,'reset'); title(ax,'Load a Part A workbook');   return; end
        if isempty(book),    cla(ax,'reset'); title(ax,'Load an antenna workbook'); return; end
        if isempty(mcsbook), cla(ax,'reset'); title(ax,'Load an MCS workbook');     return; end
        cfg.percentile = pctField.Value;
        cfg = tableToLinks(cfg, tbl.Data, mcsbook);
        sel = labels(arrayfun(@(c) logical(c.Value), cbx));
        key = graphKeys{ strcmp(graphNames, ddGraph.Value) };
        renderPartBAxes(ax, S, book, cfg, key, sel, mcsbook);
    end
end

% ---------------- local helpers ----------------
function p = resolvePath(argPath, cfgPath, repo)
    if nargin >= 1 && ~isempty(argPath)
        p = argPath;
    else
        p = cfgPath;
        if exist(p,'file') ~= 2, p = fullfile(repo, cfgPath); end
    end
end

function s = fileLabel(p)
    if nargin < 1 || isempty(p), s = '(none loaded)';
    else, [~,n,e] = fileparts(p); s = [n e]; end
end

function name = linkNameOf(seriesLabel)
    parts = strsplit(seriesLabel, ' / ');
    name  = parts{1};
end

function labs = seriesLabels(cfg)
    catlab = {'urban','open/mtn'};
    labs = {};
    for k = 1:numel(cfg.links)
        for c = 1:2
            labs{end+1} = sprintf('%s / %s', cfg.links(k).name, catlab{c}); %#ok<AGROW>
        end
    end
end

function D = linksToTable(cfg, mcsbook)
    n = numel(cfg.links); D = cell(n,5);
    for k = 1:n
        L = cfg.links(k);
        mcsName = '';
        if ~isempty(mcsbook)
            row = mcsbook.DL.mcs_index == L.mcs_index;
            if any(row), mcsName = char(mcsbook.DL.mcs(row)); end
        end
        D(k,:) = {L.name, mcsName, num2str(L.fingers), L.tx_ant, L.rx_ant};
    end
end

function cfg = tableToLinks(cfg, D, mcsbook)
    for k = 1:size(D,1)
        row = strcmp(cellstr(mcsbook.DL.mcs), D{k,2});
        if any(row), cfg.links(k).mcs_index = mcsbook.DL.mcs_index(row); end
        cfg.links(k).fingers = str2double(D{k,3});
        cfg.links(k).tx_ant  = D{k,4};
        cfg.links(k).rx_ant  = D{k,5};
    end
end
