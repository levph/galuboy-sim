function galuboySimApp()
%GALUBOYSIMAPP  uihtml-based UI for galuboy-sim.
%
%   galuboySimApp()  opens a uifigure with:
%     - left pane: native MATLAB form controls for the most-edited
%                  config fields (counts, freq, DL/UL powers, percentile,
%                  fade margin, 5-way antenna selection, region multi-select).
%                  Per-region RX-disk diameters live on cfg.regions, not in
%                  the UI (urban_suburban=5km, others=10km).
%     - center: uihtml progress panel (live region/rotation bars + log)
%     - right: tab group with Trajectory, Range histogram, Availability
%              map, Availability CCDF, and an Antennas inspector tab.
%              Range histogram and CCDF tabs each carry their own
%              DL/UL/Bidi dropdown.
%     - bottom: Run + Save config + Load config + Export HTML buttons

    repo_root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(repo_root, 'utils'));
    addProjectPaths(repo_root);

    cfg0 = buildConfig();
    region_names = arrayfun(@(rg) string(rg.name), cfg0.regions);

    antennas_dir = fullfile(repo_root, 'resources', 'antennas');
    ant_files    = dir(fullfile(antennas_dir, '*.csv'));
    ant_names    = string({ant_files.name});
    ant_names    = arrayfun(@(s) erase(s, ".csv"), ant_names);
    if isempty(ant_names)
        ant_names = ["airborne_tx", "airborne_rx", "ground_tx", "ground1_rx", "ground2_rx"];
    end

    fig = uifigure('Name', 'galuboy-sim', 'Position', [80, 80, 1280, 760]);
    g   = uigridlayout(fig, [3, 3]);
    g.RowHeight   = {'1x', '1x', 56};
    g.ColumnWidth = {300, 360, '1x'};
    g.Padding     = [10 10 10 10];
    g.RowSpacing    = 8;
    g.ColumnSpacing = 10;

    formPanel = uipanel(g, 'Title', 'Configuration', 'Scrollable', 'on');
    formPanel.Layout.Row    = [1 2];
    formPanel.Layout.Column = 1;
    nRows = 23;
    fg = uigridlayout(formPanel, [nRows, 2]);
    fg.RowHeight    = repmat({26}, 1, nRows);
    fg.RowHeight{1} = 120;                              % taller row for region listbox
    fg.ColumnWidth  = {160, '1x'};
    fg.Padding      = [8 8 8 8];
    fg.RowSpacing   = 4;
    fg.Scrollable   = 'on';

    % Multi-select region listbox. Hold Cmd/Ctrl to add to selection;
    % Shift to range-select. At least one region must remain selected.
    handles.regionDD = addRow(fg, 1,  'Regions (multi)', @(p) ...
        uilistbox(p, 'Items', cellstr(region_names), ...
                     'Value', cellstr(region_names(1)), ...
                     'Multiselect', 'on'));
    handles.nInf     = addRow(fg, 2,  'Infantry RX count', @(p) ...
        uispinner(p, 'Limits', [0, 1000], 'Value', cfg0.rx.infantry.count, 'Step', 5));
    handles.infH     = addRow(fg, 3,  'Infantry RX height (m)', @(p) ...
        uieditfield(p, 'numeric', 'Limits', [0, 20], 'Value', cfg0.rx.infantry.height_m));
    handles.nVeh     = addRow(fg, 4,  'Vehicular RX count', @(p) ...
        uispinner(p, 'Limits', [0, 1000], 'Value', cfg0.rx.vehicular.count, 'Step', 5));
    handles.vehH     = addRow(fg, 5,  'Vehicular RX height (m)', @(p) ...
        uieditfield(p, 'numeric', 'Limits', [0, 20], 'Value', cfg0.rx.vehicular.height_m));
    handles.freqMHz  = addRow(fg, 6,  'TX frequency (MHz)', @(p) ...
        uieditfield(p, 'numeric', 'Limits', [30, 60000], 'Value', cfg0.tx.frequency_hz / 1e6));
    handles.powerAirDbm = addRow(fg, 7, 'Air TX power (dBm)', @(p) ...
        uispinner(p, 'Limits', [0, 80], 'Value', cfg0.tx.power_air_dbm, 'Step', 1));
    handles.powerGndDbm = addRow(fg, 8, 'Ground TX power (dBm)', @(p) ...
        uispinner(p, 'Limits', [0, 80], 'Value', cfg0.tx.power_gnd_dbm, 'Step', 1));
    handles.altM     = addRow(fg, 9,  'TX altitude (m AGL)', @(p) ...
        uieditfield(p, 'numeric', 'Limits', [0, 20000], 'Value', cfg0.tx.altitude_m));
    handles.txAirDD  = addRow(fg, 10, 'Air TX antenna', @(p) ...
        uidropdown(p, 'Items', cellstr(ant_names), 'Value', resolveDD(cfg0.tx.airborne.antenna_name, ant_names)));
    handles.txGndDD  = addRow(fg, 11, 'Ground TX antenna', @(p) ...
        uidropdown(p, 'Items', cellstr(ant_names), 'Value', resolveDD(cfg0.tx.ground.antenna_name, ant_names)));
    handles.rxAirDD  = addRow(fg, 12, 'Air RX antenna (UL)', @(p) ...
        uidropdown(p, 'Items', cellstr(ant_names), 'Value', resolveDD(cfg0.rx.airborne.antenna_name_ul, ant_names)));
    handles.infAntDD = addRow(fg, 13, 'Infantry RX antenna (DL)', @(p) ...
        uidropdown(p, 'Items', cellstr(ant_names), 'Value', resolveDD(cfg0.rx.infantry.antenna_name_dl, ant_names)));
    handles.vehAntDD = addRow(fg, 14, 'Vehicular RX antenna (DL)', @(p) ...
        uidropdown(p, 'Items', cellstr(ant_names), 'Value', resolveDD(cfg0.rx.vehicular.antenna_name_dl, ant_names)));
    handles.bwKHz    = addRow(fg, 15, 'Bandwidth (kHz)', @(p) ...
        uieditfield(p, 'numeric', 'Limits', [0.1, 1e6], 'Value', cfg0.propagation.bandwidth_hz / 1e3));
    handles.useTerr  = addRow(fg, 16, 'Use DT2 terrain', @(p) ...
        uicheckbox(p, 'Value', cfg0.propagation.use_terrain, 'Text', ''));
    handles.threshDbm = addRow(fg, 17, 'Threshold (dBm)', @(p) ...
        uispinner(p, 'Limits', [-200, 30], 'Value', cfg0.analysis.threshold_dbm, 'Step', 1));
    handles.percent  = addRow(fg, 18, 'Availability target (%)', @(p) ...
        uispinner(p, 'Limits', [50, 100], 'Value', cfg0.analysis.percentile, 'Step', 1));
    handles.fadeDb   = addRow(fg, 19, 'Fade margin (dB)', @(p) ...
        uispinner(p, 'Limits', [0, 60], 'Value', cfg0.analysis.fade_margin_db, 'Step', 1));
    handles.flightSteps = addRow(fg, 20, 'Flight steps', @(p) ...
        uispinner(p, 'Limits', [4, 10000], 'Value', cfg0.flight.num_flight_steps, 'Step', 10));
    handles.numRot   = addRow(fg, 21, 'Rotations', @(p) ...
        uispinner(p, 'Limits', [1, 360], 'Value', cfg0.flight.num_rotations, 'Step', 1));
    handles.lemLenM  = addRow(fg, 22, 'Lemniscate length (m)', @(p) ...
        uieditfield(p, 'numeric', 'Limits', [10, 1e6], 'Value', cfg0.flight.lemniscate_length_m));
    handles.maxTilt  = addRow(fg, 23, 'Max tilt (deg)', @(p) ...
        uispinner(p, 'Limits', [0, 89], 'Value', cfg0.flight.max_tilt_deg, 'Step', 1));

    progPanel = uipanel(g, 'Title', 'Progress');
    progPanel.Layout.Row    = [1 2];
    progPanel.Layout.Column = 2;
    pg = uigridlayout(progPanel, [1, 1]);
    pg.Padding = [4 4 4 4];
    handles.html = uihtml(pg);
    handles.html.HTMLSource = fullfile(repo_root, 'ui', 'progress.html');
    handles.html.Data = struct( ...
        'stage', 'idle', 'region_idx', 0, 'n_regions', 0, ...
        'rot_idx', 0, 'n_rots', 0, ...
        'msg', 'Ready. Adjust config on the left, then click Run.', ...
        'tick', 0);

    plotsPanel = uipanel(g, 'Title', 'Results');
    plotsPanel.Layout.Row    = [1 2];
    plotsPanel.Layout.Column = 3;
    tg = uigridlayout(plotsPanel, [1, 1]);
    tg.Padding = [4 4 4 4];
    tabs = uitabgroup(tg);
    tabTraj = uitab(tabs, 'Title', 'Trajectory');
    tabHist = uitab(tabs, 'Title', 'Range histogram');
    tabMap  = uitab(tabs, 'Title', 'Availability map');
    tabCcdf = uitab(tabs, 'Title', 'Availability CCDF');
    tabAnt  = uitab(tabs, 'Title', 'Antennas');
    handles.axTraj = uiaxes(tabTraj);
    handles.axTraj.Position = [10 10 880 580];

    histGrid = uigridlayout(tabHist, [2, 1]);
    histGrid.RowHeight = {30, '1x'};
    histGrid.Padding   = [4 4 4 4];
    histTopRow = uigridlayout(histGrid, [1, 3]);
    histTopRow.ColumnWidth = {50, 100, '1x'};
    histTopRow.Padding     = [0 0 0 0];
    uilabel(histTopRow, 'Text', 'Link:');
    handles.linkHistDD = uidropdown(histTopRow, ...
        'Items', {'DL','UL','Bidi'}, 'Value', 'DL');
    handles.axHist = uiaxes(histGrid);

    handles.axMap  = uiaxes(tabMap);
    handles.axMap.Position  = [10 10 880 580];

    ccdfGrid = uigridlayout(tabCcdf, [2, 1]);
    ccdfGrid.RowHeight = {30, '1x'};
    ccdfGrid.Padding   = [4 4 4 4];
    togRow = uigridlayout(ccdfGrid, [1, 6]);
    togRow.ColumnWidth = {110, 110, 90, 50, 100, '1x'};
    togRow.Padding     = [0 0 0 0];
    handles.cbInf = uicheckbox(togRow, 'Text', 'Infantry',  'Value', true);
    handles.cbVeh = uicheckbox(togRow, 'Text', 'Vehicular', 'Value', true);
    handles.cbTot = uicheckbox(togRow, 'Text', 'Total',     'Value', true);
    uilabel(togRow, 'Text', 'Link:');
    handles.linkCcdfDD = uidropdown(togRow, ...
        'Items', {'DL','UL','Bidi'}, 'Value', 'DL');
    handles.axCcdf = uiaxes(ccdfGrid);

    % --- Antennas tab (inspect/edit per-antenna patterns) ----------------
    handles.antennasDir = antennas_dir;
    % Stash early so antennaCsvPath (called from buildAntennasTab) can find
    % handles.antennasDir via getappdata. The final stash at end of function
    % overwrites this with the fully-populated handles struct.
    setappdata(fig, 'handles', handles);
    buildAntennasTab(tabAnt, ant_names);

    btnPanel = uipanel(g, 'BorderType', 'none');
    btnPanel.Layout.Row    = 3;
    btnPanel.Layout.Column = [1 3];
    bg = uigridlayout(btnPanel, [1, 5]);
    bg.ColumnWidth = {120, 140, 140, 140, '1x'};
    bg.Padding     = [4 4 4 4];

    handles.runBtn = uibutton(bg, 'Text', 'Run', ...
        'BackgroundColor', [0.20 0.65 0.30], ...
        'FontColor', [1 1 1], 'FontWeight', 'bold');
    handles.runBtn.Layout.Column = 1;
    handles.runBtn.ButtonPushedFcn = @(~,~) onRun(handles);

    handles.saveBtn = uibutton(bg, 'Text', 'Save config...');
    handles.saveBtn.Layout.Column = 2;
    handles.saveBtn.ButtonPushedFcn = @(~,~) onSaveCfg(handles);

    handles.loadBtn = uibutton(bg, 'Text', 'Load config...');
    handles.loadBtn.Layout.Column = 3;
    handles.loadBtn.ButtonPushedFcn = @(~,~) onLoadCfg(handles);

    handles.exportBtn = uibutton(bg, 'Text', 'Export HTML...');
    handles.exportBtn.Layout.Column = 4;
    handles.exportBtn.ButtonPushedFcn = @(~,~) onExportHtml(handles, fig);

    handles.statusLbl = uilabel(bg, 'Text', '', ...
        'HorizontalAlignment', 'right', 'FontColor', [0.4 0.4 0.4]);
    handles.statusLbl.Layout.Column = 5;

    handles.cbInf.ValueChangedFcn      = @(~,~) renderCcdf(fig);
    handles.cbVeh.ValueChangedFcn      = @(~,~) renderCcdf(fig);
    handles.cbTot.ValueChangedFcn      = @(~,~) renderCcdf(fig);
    handles.linkCcdfDD.ValueChangedFcn = @(~,~) renderCcdf(fig);
    handles.linkHistDD.ValueChangedFcn = @(~,~) renderHistogram(fig);

    setappdata(fig, 'handles', handles);
end


% =========================================================================
function v = resolveDD(name, items)
    if any(strcmp(name, items))
        v = char(name);
    else
        v = char(items(1));
    end
end


% =========================================================================
function ctrl = addRow(parent, row, labelText, ctrlFactory)
    lbl = uilabel(parent, 'Text', labelText);
    lbl.Layout.Row    = row;
    lbl.Layout.Column = 1;
    ctrl = ctrlFactory(parent);
    ctrl.Layout.Row    = row;
    ctrl.Layout.Column = 2;
end


function onRun(h)
    h.runBtn.Enable = 'off';
    h.statusLbl.Text = 'Running...';
    drawnow;
    cleaner = onCleanup(@() reEnable(h));   % unused-on-purpose; destructor re-enables Run button

    try
        s = readForm(h);
        ccdf_opts = struct('show_inf', logical(h.cbInf.Value), ...
                           'show_veh', logical(h.cbVeh.Value), ...
                           'show_tot', logical(h.cbTot.Value));
        [results, cfg] = runFromUI(s, h.html, h.axHist, h.axMap, h.axTraj, ...
                                   h.axCcdf, ccdf_opts);
        fig = ancestor(h.runBtn, 'figure');
        setappdata(fig, 'last_results', results);
        setappdata(fig, 'last_cfg',     cfg);
        h.statusLbl.Text = 'Done.';
    catch ME
        h.statusLbl.Text = sprintf('Error: %s', ME.message);
        h.html.Data = struct( ...
            'stage', 'error', 'region_idx', 0, 'n_regions', 0, ...
            'rot_idx', 0, 'n_rots', 0, ...
            'msg', sprintf('%s', ME.message), ...
            'tick', posixtime(datetime('now')));
        drawnow;
        rethrow(ME);
    end
end


function reEnable(h)
    h.runBtn.Enable = 'on';
end


function s = readForm(h)
    % regionDD is a multi-select uilistbox; Value is a cellstr (or char if
    % a single item). Normalise to a cellstr of region names.
    region_names = h.regionDD.Value;
    if ischar(region_names)
        region_names = {region_names};
    elseif isstring(region_names)
        region_names = cellstr(region_names);
    end
    if isempty(region_names)
        error('galuboySimApp:noRegion', ...
            'Select at least one region before running.');
    end

    s = struct( ...
        'region_names',            {region_names}, ...     % cell array
        'n_inf',                   h.nInf.Value, ...
        'inf_height_m',            h.infH.Value, ...
        'n_veh',                   h.nVeh.Value, ...
        'veh_height_m',            h.vehH.Value, ...
        'frequency_hz',            h.freqMHz.Value * 1e6, ...
        'power_air_dbm',           h.powerAirDbm.Value, ...
        'power_gnd_dbm',           h.powerGndDbm.Value, ...
        'altitude_m',              h.altM.Value, ...
        'tx_air_antenna_name',     char(h.txAirDD.Value), ...
        'tx_gnd_antenna_name',     char(h.txGndDD.Value), ...
        'rx_air_antenna_name_ul',  char(h.rxAirDD.Value), ...
        'inf_antenna_name_dl',     char(h.infAntDD.Value), ...
        'veh_antenna_name_dl',     char(h.vehAntDD.Value), ...
        'bandwidth_hz',            h.bwKHz.Value * 1e3, ...
        'use_terrain',             logical(h.useTerr.Value), ...
        'threshold_dbm',           h.threshDbm.Value, ...
        'percentile',              h.percent.Value, ...
        'fade_margin_db',          h.fadeDb.Value, ...
        'num_flight_steps',        h.flightSteps.Value, ...
        'num_rotations',           h.numRot.Value, ...
        'lemniscate_length_m',     h.lemLenM.Value, ...
        'max_tilt_deg',            h.maxTilt.Value, ...
        'link_hist',               lower(char(h.linkHistDD.Value)), ...
        'link_ccdf',               lower(char(h.linkCcdfDD.Value)));
end


function onSaveCfg(h)
    [file, path] = uiputfile('*.json', 'Save UI form state as JSON');
    if isequal(file, 0), return; end
    s = readForm(h);
    fid = fopen(fullfile(path, file), 'w');
    fwrite(fid, jsonencode(s, 'PrettyPrint', true));
    fclose(fid);
    h.statusLbl.Text = sprintf('Saved %s', file);
end


function onLoadCfg(h)
    [file, path] = uigetfile('*.json', 'Load UI form state');
    if isequal(file, 0), return; end
    fid = fopen(fullfile(path, file), 'r');
    raw = fread(fid, '*char')'; fclose(fid);
    s = jsondecode(raw);

    fp = fieldnames(s);
    % Backward compat: accept old 'region_name' (char) OR new 'region_names'
    % (cell of chars / string array). uilistbox.Value expects a cellstr.
    if ismember('region_names', fp)
        names = s.region_names;
        if isstring(names),  names = cellstr(names);  end
        if ischar(names),    names = {names};         end
        h.regionDD.Value = names;
    elseif ismember('region_name', fp)
        h.regionDD.Value = {char(s.region_name)};
    end
    if ismember('n_inf',                   fp), h.nInf.Value         = s.n_inf;                          end
    if ismember('inf_height_m',            fp), h.infH.Value         = s.inf_height_m;                   end
    if ismember('n_veh',                   fp), h.nVeh.Value         = s.n_veh;                          end
    if ismember('veh_height_m',            fp), h.vehH.Value         = s.veh_height_m;                   end
    if ismember('frequency_hz',            fp), h.freqMHz.Value      = s.frequency_hz / 1e6;             end
    if ismember('power_air_dbm',           fp), h.powerAirDbm.Value  = s.power_air_dbm;                  end
    if ismember('power_gnd_dbm',           fp), h.powerGndDbm.Value  = s.power_gnd_dbm;                  end
    if ismember('altitude_m',              fp), h.altM.Value         = s.altitude_m;                     end
    if ismember('tx_air_antenna_name',     fp), h.txAirDD.Value      = char(s.tx_air_antenna_name);      end
    if ismember('tx_gnd_antenna_name',     fp), h.txGndDD.Value      = char(s.tx_gnd_antenna_name);      end
    if ismember('rx_air_antenna_name_ul',  fp), h.rxAirDD.Value      = char(s.rx_air_antenna_name_ul);   end
    if ismember('inf_antenna_name_dl',     fp), h.infAntDD.Value     = char(s.inf_antenna_name_dl);      end
    if ismember('veh_antenna_name_dl',     fp), h.vehAntDD.Value     = char(s.veh_antenna_name_dl);      end
    if ismember('bandwidth_hz',            fp), h.bwKHz.Value        = s.bandwidth_hz / 1e3;             end
    if ismember('use_terrain',             fp), h.useTerr.Value      = logical(s.use_terrain);           end
    if ismember('threshold_dbm',           fp), h.threshDbm.Value    = s.threshold_dbm;                  end
    if ismember('percentile',              fp), h.percent.Value      = s.percentile;                     end
    if ismember('fade_margin_db',          fp), h.fadeDb.Value       = s.fade_margin_db;                 end
    if ismember('num_flight_steps',        fp), h.flightSteps.Value  = s.num_flight_steps;               end
    if ismember('num_rotations',           fp), h.numRot.Value       = s.num_rotations;                  end
    if ismember('lemniscate_length_m',     fp), h.lemLenM.Value      = s.lemniscate_length_m;            end
    % Legacy placement_diameter_km / placement_radius_km in saved JSON are
    % ignored - per-region diameters live on cfg.regions now.
    if ismember('max_tilt_deg',            fp), h.maxTilt.Value      = s.max_tilt_deg;                   end
    if ismember('link_hist',               fp), h.linkHistDD.Value   = upperLink(s.link_hist);           end
    if ismember('link_ccdf',               fp), h.linkCcdfDD.Value   = upperLink(s.link_ccdf);           end

    h.statusLbl.Text = sprintf('Loaded %s', file);
end


function v = upperLink(s)
    switch lower(char(s))
        case 'dl',   v = 'DL';
        case 'ul',   v = 'UL';
        case 'bidi', v = 'Bidi';
        otherwise,   v = 'DL';
    end
end


function renderCcdf(fig)
    h = getappdata(fig, 'handles');
    results = getappdata(fig, 'last_results');
    cfg     = getappdata(fig, 'last_cfg');
    if isempty(results) || isempty(cfg), return; end
    plotAvailabilityCCDF(results, cfg, h.axCcdf, ...
        'show_inf',   logical(h.cbInf.Value), ...
        'show_veh',   logical(h.cbVeh.Value), ...
        'show_total', logical(h.cbTot.Value), ...
        'link',       lower(char(h.linkCcdfDD.Value)));
end


function renderHistogram(fig)
    h = getappdata(fig, 'handles');
    results = getappdata(fig, 'last_results');
    cfg     = getappdata(fig, 'last_cfg');
    if isempty(results) || isempty(cfg), return; end
    plotRangeHistogram(results, cfg, h.axHist, ...
        'link', lower(char(h.linkHistDD.Value)));
end


function onExportHtml(h, fig)
    results = getappdata(fig, 'last_results');
    cfg     = getappdata(fig, 'last_cfg');
    if isempty(results) || isempty(cfg)
        h.statusLbl.Text = 'Run a scenario first.';
        return;
    end
    [file, path] = uiputfile('*.html', 'Export simulation summary');
    if isequal(file, 0), return; end
    out_path = fullfile(path, file);
    try
        exportSummaryHtml(cfg, results, ...
            struct('Trajectory',         h.axTraj, ...
                   'RangeHistogram',     h.axHist, ...
                   'AvailabilityMap',    h.axMap, ...
                   'AvailabilityCCDF',   h.axCcdf), ...
            out_path);
        web(out_path, '-browser');
        h.statusLbl.Text = sprintf('Exported %s', file);
    catch ME
        h.statusLbl.Text = sprintf('Export failed: %s', ME.message);
    end
end


% =========================================================================
% Antennas tab — inspect/edit per-antenna patterns
% =========================================================================
function buildAntennasTab(tabAnt, ant_names)
    grid = uigridlayout(tabAnt, [3, 1]);
    grid.RowHeight = {32, '1x', 40};
    grid.Padding   = [6 6 6 6];
    grid.RowSpacing = 6;

    topRow = uigridlayout(grid, [1, 2]);
    topRow.ColumnWidth = {130, '1x'};
    topRow.Padding     = [0 0 0 0];
    uilabel(topRow, 'Text', 'Antenna:');
    antDD = uidropdown(topRow, 'Items', cellstr(ant_names), 'Value', char(ant_names(1)));

    midRow = uigridlayout(grid, [1, 2]);
    midRow.ColumnWidth = {'1x', '1x'};
    midRow.Padding     = [0 0 0 0];
    midRow.ColumnSpacing = 8;

    leftPanel = uipanel(midRow, 'BorderType', 'none');
    rightPanel = uipanel(midRow, 'BorderType', 'none', 'Title', 'Pattern table');

    tbl = uitable(rightPanel);
    tbl.Position = [6 6 360 480];
    tbl.ColumnEditable = true;

    btnRow = uigridlayout(grid, [1, 4]);
    btnRow.ColumnWidth = {110, 130, 170, '1x'};
    btnRow.Padding     = [0 0 0 0];
    btnSave    = uibutton(btnRow, 'Text', 'Save');
    btnReload  = uibutton(btnRow, 'Text', 'Reload from disk');
    btnRegen   = uibutton(btnRow, 'Text', 'Regenerate patch shape');
    statusLbl  = uilabel(btnRow, 'Text', '', 'HorizontalAlignment', 'right', ...
                         'FontColor', [0.4 0.4 0.4]);

    state.antDD     = antDD;
    state.leftPanel = leftPanel;
    state.tbl       = tbl;
    state.statusLbl = statusLbl;
    state.btnRegen  = btnRegen;
    setappdata(tabAnt, 'antState', state);

    antDD.ValueChangedFcn = @(~,~) loadAntennaIntoTab(tabAnt);
    tbl.CellEditCallback  = @(~,~) refreshAntennaPlot(tabAnt);
    btnSave.ButtonPushedFcn   = @(~,~) saveAntennaTableToCsv(tabAnt);
    btnReload.ButtonPushedFcn = @(~,~) loadAntennaIntoTab(tabAnt);
    btnRegen.ButtonPushedFcn  = @(~,~) regeneratePatchCsv(tabAnt);

    loadAntennaIntoTab(tabAnt);
end


function loadAntennaIntoTab(tabAnt)
    state = getappdata(tabAnt, 'antState');
    name  = char(state.antDD.Value);
    csv_path = antennaCsvPath(tabAnt, name);
    if ~isfile(csv_path)
        state.statusLbl.Text = sprintf('Missing: %s', name);
        return;
    end

    raw = readlines(csv_path);
    [headerLine, body] = sliceCsv(raw);
    if startsWith(lower(strtrim(headerLine)), 'isotropic')
        toks = strsplit(strtrim(headerLine), ',');
        data = {str2double(strtrim(toks{2}))};
        cols = {'gain_dbi'};
    else
        cols = strsplit(strtrim(headerLine), ',');
        n = numel(body);
        data = cell(n, numel(cols));
        for k = 1:n
            tk = strsplit(strtrim(body{k}), ',');
            for j = 1:numel(cols)
                if j <= numel(tk)
                    data{k, j} = str2double(tk{j});
                end
            end
        end
    end
    state.tbl.ColumnName = cols;
    state.tbl.Data = data;

    % Patch-regen button only meaningful for the 2D patch CSVs.
    is_patch = any(strcmp(name, {'airborne_rx', 'ground1_rx', 'ground2_rx'}));
    if is_patch, state.btnRegen.Enable = 'on'; else, state.btnRegen.Enable = 'off'; end

    state.statusLbl.Text = sprintf('Loaded %s', name);
    refreshAntennaPlot(tabAnt);
end


function refreshAntennaPlot(tabAnt)
    state = getappdata(tabAnt, 'antState');
    parent = state.leftPanel;
    delete(allchild(parent));

    [kind, data, ~] = readAntennaTableData(state.tbl);
    switch kind
        case 'isotropic'
            uilabel(parent, 'Text', sprintf('Isotropic: %.2f dBi', data{1,1}), ...
                'Position', [10 10 400 30]);
        case '1d'
            ax = polaraxes(parent);
            el_deg = cellfun(@(x) double(x), data(:,1));
            g_dbi  = cellfun(@(x) double(x), data(:,2));
            theta  = deg2rad(el_deg(:));
            polarplot(ax, theta, g_dbi, '-o', 'LineWidth', 1.5);
            title(ax, 'Gain (dBi) vs elevation offset');
        case '2d'
            ax = uiaxes(parent);
            ax.Position = [10 10 480 480];
            az_deg = cellfun(@(x) double(x), data(:,1));
            el_deg = cellfun(@(x) double(x), data(:,2));
            g_dbi  = cellfun(@(x) double(x), data(:,3));
            [u_az, ~, ia] = unique(az_deg);
            [u_el, ~, ie] = unique(el_deg);
            G = nan(numel(u_az), numel(u_el));
            for k = 1:numel(g_dbi)
                G(ia(k), ie(k)) = g_dbi(k);
            end
            imagesc(ax, u_el, u_az, G);
            set(ax, 'YDir', 'normal');
            colormap(ax, 'turbo');
            cb = colorbar(ax);
            ylabel(cb, 'Gain (dBi)');
            xlabel(ax, 'El offset (deg)');
            ylabel(ax, 'Az offset (deg)');
            title(ax, 'Gain (dBi) — 2D patch');
        otherwise
            uilabel(parent, 'Text', '(no plot)', 'Position', [10 10 200 30]);
    end
    drawnow;
end


function [kind, data, cols] = readAntennaTableData(tbl)
    cols = tbl.ColumnName;
    data = tbl.Data;
    if isempty(cols)
        kind = ''; return;
    end
    if ischar(cols) || isstring(cols), cols = cellstr(cols); end
    switch numel(cols)
        case 1
            kind = 'isotropic';
        case 2
            kind = '1d';
        case 3
            kind = '2d';
        otherwise
            kind = '';
    end
end


function saveAntennaTableToCsv(tabAnt)
    state = getappdata(tabAnt, 'antState');
    name  = char(state.antDD.Value);
    csv_path = antennaCsvPath(tabAnt, name);
    [kind, data, ~] = readAntennaTableData(state.tbl);
    fid = fopen(csv_path, 'w');
    if fid < 0
        state.statusLbl.Text = sprintf('Cannot write %s', name);
        return;
    end
    cleanup = onCleanup(@() fclose(fid));   % unused-on-purpose; destructor closes fid
    fprintf(fid, '# galuboy antenna pattern — saved from UI\n');
    switch kind
        case 'isotropic'
            v = double(data{1,1});
            fprintf(fid, 'isotropic,%.6g\n', v);
        case '1d'
            fprintf(fid, 'el_deg,gain_dbi\n');
            for k = 1:size(data, 1)
                fprintf(fid, '%g,%.6g\n', double(data{k,1}), double(data{k,2}));
            end
        case '2d'
            fprintf(fid, 'az_deg,el_deg,gain_dbi\n');
            for k = 1:size(data, 1)
                fprintf(fid, '%g,%g,%.6g\n', double(data{k,1}), double(data{k,2}), double(data{k,3}));
            end
    end
    state.statusLbl.Text = sprintf('Saved %s', name);
end


function regeneratePatchCsv(tabAnt)
    state = getappdata(tabAnt, 'antState');
    name  = char(state.antDD.Value);
    csv_path = antennaCsvPath(tabAnt, name);

    cfg = buildConfig();
    N = 16; if any(strcmp(name, {'ground1_rx', 'ground2_rx'})), N = 8; end
    try
        generatePatchArrayCsv(csv_path, N, ...
            'fov_in_deg',  cfg.rx.patch.fov_in_deg, ...
            'fov_out_deg', cfg.rx.patch.fov_out_deg, ...
            'floor_db',    cfg.rx.patch.floor_db);
        state.statusLbl.Text = sprintf('Regenerated %s (N=%d)', name, N);
    catch ME
        state.statusLbl.Text = sprintf('Regen failed: %s', ME.message);
        return;
    end
    loadAntennaIntoTab(tabAnt);
end


function p = antennaCsvPath(tabAnt, name)
    fig = ancestor(tabAnt, 'figure');
    h = getappdata(fig, 'handles');
    p = fullfile(h.antennasDir, [name, '.csv']);
end


function [headerLine, body] = sliceCsv(raw_lines)
    body = {};
    headerLine = '';
    found = false;
    for k = 1:numel(raw_lines)
        s = strtrim(char(raw_lines(k)));
        if isempty(s) || startsWith(s, '#'), continue; end
        if ~found
            headerLine = s; found = true; continue;
        end
        body{end+1} = s; %#ok<AGROW>
    end
end
