function fig = galuboyAnalysisApp(partA_path, antenna_path)
%GALUBOYANALYSISAPP  Interactive Part B analysis UI (R2021b+).
%
%   galuboyAnalysisApp()
%   galuboyAnalysisApp(partA_xlsx)
%   galuboyAnalysisApp(partA_xlsx, antenna_xlsx)
%
%   Loads a Part A workbook + the antenna workbook and lets you explore the
%   link analysis: pick the graph (availability CCDF / availability histogram /
%   margin), set the availability percentile, tick which (link x terrain) series
%   to overlay, and edit per-link BW / PAPR / required-SNR / antenna indices in a
%   table. Everything recomputes through analyzeLinks and redraws via
%   renderPartBAxes (the same per-axes plotters as the batch view).
%
%   Antenna gains and the non-link constants (tx powers, losses, noise figures)
%   come from buildConfigB / the antenna workbook - edit those for the rest.

    cfg  = buildConfigB();
    repo = fileparts(fileparts(mfilename('fullpath')));   % partB/ -> repo

    % resolve a default antenna workbook
    if nargin >= 2 && ~isempty(antenna_path)
        bookPath = antenna_path;
    else
        bookPath = cfg.antenna_book;
        if exist(bookPath,'file') ~= 2, bookPath = fullfile(repo, cfg.antenna_book); end
    end
    book = [];
    if exist(bookPath,'file') == 2, book = loadAntennaBook(bookPath); end
    S = [];
    if nargin >= 1 && ~isempty(partA_path) && exist(partA_path,'file') == 2
        S = loadPartA(partA_path);
    end

    graphNames = {'Availability CCDF','Availability histogram','Margin vs distance'};
    graphKeys  = {'ccdf','avail','margin'};
    labels     = seriesLabels(cfg);

    fig = uifigure('Name','galuboy — Part B analysis','Position',[80 80 1150 700]);
    G = uigridlayout(fig,[1 2]); G.ColumnWidth = {380,'1x'}; G.RowHeight = {'1x'};

    L = uigridlayout(G,[10 1]);
    L.RowHeight = {30,22,30,32,32,22,160,22,150,34};
    L.Layout.Row = 1; L.Layout.Column = 1;

    uibutton(L,'Text','Load Part A workbook…','ButtonPushedFcn',@onLoadA);
    lblFile = uilabel(L,'Text', fileLabel(partA_path), 'FontColor',[.35 .35 .35]);
    uibutton(L,'Text','Load antenna workbook…','ButtonPushedFcn',@onLoadAnt);

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
    tbl.ColumnName     = {'link','BW (MHz)','PAPR dB','req SNR dB','tx idx','rx idx'};
    tbl.ColumnEditable = [false true true true true true];
    tbl.Data           = linksToTable(cfg);
    tbl.CellEditCallback = @(~,~)doPlot();

    uibutton(L,'Text','Update plot','ButtonPushedFcn',@(~,~)doPlot());

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
        book = loadAntennaBook(fullfile(p,f)); doPlot();
    end
    function doPlot()
        if isempty(S),    cla(ax,'reset'); title(ax,'Load a Part A workbook');   return; end
        if isempty(book), cla(ax,'reset'); title(ax,'Load an antenna workbook'); return; end
        cfg.percentile = pctField.Value;
        cfg = tableToLinks(cfg, tbl.Data);
        sel = labels(arrayfun(@(c) logical(c.Value), cbx));
        key = graphKeys{ strcmp(graphNames, ddGraph.Value) };
        renderPartBAxes(ax, S, book, cfg, key, sel);
    end
end

% ---------------- local helpers ----------------
function s = fileLabel(p)
    if nargin < 1 || isempty(p), s = '(no Part A loaded)';
    else, [~,n,e] = fileparts(p); s = [n e]; end
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

function D = linksToTable(cfg)
    n = numel(cfg.links); D = cell(n,6);
    for k = 1:n
        L = cfg.links(k);
        D(k,:) = {L.name, L.bw_hz/1e6, L.papr_db, L.req_snr_db, L.tx_ant_idx, L.rx_ant_idx};
    end
end

function cfg = tableToLinks(cfg, D)
    for k = 1:size(D,1)
        cfg.links(k).bw_hz      = D{k,2} * 1e6;
        cfg.links(k).papr_db    = D{k,3};
        cfg.links(k).req_snr_db = D{k,4};
        cfg.links(k).tx_ant_idx = D{k,5};
        cfg.links(k).rx_ant_idx = D{k,6};
    end
end
