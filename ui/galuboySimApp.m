function galuboySimApp()
%GALUBOYSIMAPP  uihtml-based UI for galuboy-sim.
%
%   galuboySimApp()  opens a uifigure with:
%     - left pane: native MATLAB form controls for the most-edited
%                  config fields (counts, freq, power, percentile,
%                  fade margin, antenna selection, region selection)
%     - center: uihtml progress panel (live region/rotation bars + log)
%     - right: tab group with Range histogram and Availability map
%              (uiaxes targets)
%     - bottom: Run + Save config + Load config buttons
%
%   Requires MATLAB R2021b+ for the uihtml component. The simulation
%   runs synchronously on the UI thread; drawnow inside the progress
%   callback keeps the app responsive without backgroundPool.

    % Make sure project paths are on the search path. Use the dedicated
    % helper (NOT addpath(genpath(repo_root))) so any git worktrees under
    % .claude/ don't shadow the production code with stale revisions.
    repo_root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(repo_root, 'utils'));
    addProjectPaths(repo_root);

    % --- Pull defaults from config --------------------------------------
    cfg0 = buildConfig();
    region_names = arrayfun(@(rg) string(rg.name), cfg0.regions);

    antennas_dir = fullfile(repo_root, 'resources', 'antennas');
    ant_files    = dir(fullfile(antennas_dir, '*.csv'));
    ant_names    = string({ant_files.name});
    ant_names    = arrayfun(@(s) erase(s, ".csv"), ant_names);
    if isempty(ant_names)
        ant_names = ["tx_omni", "rx_infantry_omni", "rx_vehicular_dipole"];
    end

    % --- Figure + grid layout -------------------------------------------
    fig = uifigure('Name', 'galuboy-sim', 'Position', [80, 80, 1200, 720]);
    g   = uigridlayout(fig, [3, 3]);
    g.RowHeight   = {'1x', '1x', 56};
    g.ColumnWidth = {290, 360, '1x'};
    g.Padding     = [10 10 10 10];
    g.RowSpacing    = 8;
    g.ColumnSpacing = 10;

    % --- Form (left, spans rows 1-2) ------------------------------------
    formPanel = uipanel(g, 'Title', 'Configuration', 'Scrollable', 'on');
    formPanel.Layout.Row    = [1 2];
    formPanel.Layout.Column = 1;
    nRows = 21;
    fg = uigridlayout(formPanel, [nRows, 2]);
    fg.RowHeight   = repmat({26}, 1, nRows);
    fg.ColumnWidth = {140, '1x'};
    fg.Padding     = [8 8 8 8];
    fg.RowSpacing  = 4;
    fg.Scrollable  = 'on';

    handles.regionDD = addRow(fg, 1,  'Region', @(p) ...
        uidropdown(p, 'Items', cellstr(region_names), 'Value', char(region_names(1))));
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
    handles.powerDbm = addRow(fg, 7,  'TX power (dBm)', @(p) ...
        uispinner(p, 'Limits', [0, 80], 'Value', cfg0.tx.power_dbm, 'Step', 1));
    handles.altM     = addRow(fg, 8,  'TX altitude (m AGL)', @(p) ...
        uieditfield(p, 'numeric', 'Limits', [0, 20000], 'Value', cfg0.tx.altitude_m));
    handles.txAntDD  = addRow(fg, 9,  'TX antenna', @(p) ...
        uidropdown(p, 'Items', cellstr(ant_names), 'Value', cfg0.tx.antenna_name));
    handles.infAntDD = addRow(fg, 10, 'Infantry antenna', @(p) ...
        uidropdown(p, 'Items', cellstr(ant_names), 'Value', cfg0.rx.infantry.antenna_name));
    handles.vehAntDD = addRow(fg, 11, 'Vehicular antenna', @(p) ...
        uidropdown(p, 'Items', cellstr(ant_names), 'Value', cfg0.rx.vehicular.antenna_name));
    handles.bwKHz    = addRow(fg, 12, 'Bandwidth (kHz)', @(p) ...
        uieditfield(p, 'numeric', 'Limits', [0.1, 1e6], 'Value', cfg0.propagation.bandwidth_hz / 1e3));
    handles.useTerr  = addRow(fg, 13, 'Use DT2 terrain', @(p) ...
        uicheckbox(p, 'Value', cfg0.propagation.use_terrain, 'Text', ''));
    handles.threshDbm = addRow(fg, 14, 'Threshold (dBm)', @(p) ...
        uispinner(p, 'Limits', [-200, 30], 'Value', cfg0.analysis.threshold_dbm, 'Step', 1));
    handles.percent  = addRow(fg, 15, 'Percentile', @(p) ...
        uispinner(p, 'Limits', [50, 100], 'Value', cfg0.analysis.percentile, 'Step', 1));
    handles.fadeDb   = addRow(fg, 16, 'Fade margin (dB)', @(p) ...
        uispinner(p, 'Limits', [0, 60], 'Value', cfg0.analysis.fade_margin_db, 'Step', 1));
    handles.flightSteps = addRow(fg, 17, 'Flight steps', @(p) ...
        uispinner(p, 'Limits', [4, 10000], 'Value', cfg0.flight.num_flight_steps, 'Step', 10));
    handles.numRot   = addRow(fg, 18, 'Rotations', @(p) ...
        uispinner(p, 'Limits', [1, 360], 'Value', cfg0.flight.num_rotations, 'Step', 1));
    handles.lemLenM  = addRow(fg, 19, 'Lemniscate length (m)', @(p) ...
        uieditfield(p, 'numeric', 'Limits', [10, 1e6], 'Value', cfg0.flight.lemniscate_length_m));
    handles.maxTilt  = addRow(fg, 20, 'Max tilt (deg)', @(p) ...
        uispinner(p, 'Limits', [0, 89], 'Value', cfg0.flight.max_tilt_deg, 'Step', 1));
    handles.placeKm  = addRow(fg, 21, 'RX placement radius (km)', @(p) ...
        uieditfield(p, 'numeric', 'Limits', [0.01, 500], 'Value', cfg0.rx.placement_radius_km));

    % --- Progress (center, spans rows 1-2) ------------------------------
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

    % --- Plots (right, spans rows 1-2) ----------------------------------
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
    handles.axTraj = uiaxes(tabTraj);
    handles.axTraj.Position = [10 10 880 580];
    handles.axHist = uiaxes(tabHist);
    handles.axHist.Position = [10 10 880 580];
    handles.axMap  = uiaxes(tabMap);
    handles.axMap.Position  = [10 10 880 580];

    % --- Availability CCDF tab: 3 toggles + axes ------------------------
    ccdfGrid = uigridlayout(tabCcdf, [2, 1]);
    ccdfGrid.RowHeight = {30, '1x'};
    ccdfGrid.Padding   = [4 4 4 4];
    togRow = uigridlayout(ccdfGrid, [1, 4]);
    togRow.ColumnWidth = {110, 110, 90, '1x'};
    togRow.Padding     = [0 0 0 0];
    handles.cbInf = uicheckbox(togRow, 'Text', 'Infantry',  'Value', true);
    handles.cbVeh = uicheckbox(togRow, 'Text', 'Vehicular', 'Value', true);
    handles.cbTot = uicheckbox(togRow, 'Text', 'Total',     'Value', true);
    handles.axCcdf = uiaxes(ccdfGrid);

    % --- Buttons (row 3, spanning columns) ------------------------------
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

    % CCDF checkbox callbacks: re-render from cached results
    handles.cbInf.ValueChangedFcn = @(~,~) renderCcdf(fig);
    handles.cbVeh.ValueChangedFcn = @(~,~) renderCcdf(fig);
    handles.cbTot.ValueChangedFcn = @(~,~) renderCcdf(fig);

    setappdata(fig, 'handles', handles);
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
    cleaner = onCleanup(@() reEnable(h));

    try
        s = readForm(h);
        ccdf_opts = struct('show_inf', logical(h.cbInf.Value), ...
                           'show_veh', logical(h.cbVeh.Value), ...
                           'show_tot', logical(h.cbTot.Value));
        [results, cfg] = runFromUI(s, h.html, h.axHist, h.axMap, h.axTraj, ...
                                   h.axCcdf, ccdf_opts);
        % Stash for the export button + CCDF re-renders
        fig = ancestor(h.runBtn, 'figure');
        setappdata(fig, 'last_results', results);
        setappdata(fig, 'last_cfg',     cfg);
        h.statusLbl.Text = 'Done.';
    catch ME
        h.statusLbl.Text = sprintf('Error: %s', ME.message);
        % Surface to the progress panel too
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
    s = struct( ...
        'region_name',           char(h.regionDD.Value), ...
        'n_inf',                 h.nInf.Value, ...
        'inf_height_m',          h.infH.Value, ...
        'n_veh',                 h.nVeh.Value, ...
        'veh_height_m',          h.vehH.Value, ...
        'frequency_hz',          h.freqMHz.Value * 1e6, ...
        'power_dbm',             h.powerDbm.Value, ...
        'altitude_m',            h.altM.Value, ...
        'tx_antenna_name',       char(h.txAntDD.Value), ...
        'inf_antenna_name',      char(h.infAntDD.Value), ...
        'veh_antenna_name',      char(h.vehAntDD.Value), ...
        'bandwidth_hz',          h.bwKHz.Value * 1e3, ...
        'use_terrain',           logical(h.useTerr.Value), ...
        'threshold_dbm',         h.threshDbm.Value, ...
        'percentile',            h.percent.Value, ...
        'fade_margin_db',        h.fadeDb.Value, ...
        'num_flight_steps',      h.flightSteps.Value, ...
        'num_rotations',         h.numRot.Value, ...
        'lemniscate_length_m',   h.lemLenM.Value, ...
        'max_tilt_deg',          h.maxTilt.Value, ...
        'placement_radius_km',   h.placeKm.Value);
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
    if ismember('region_name',           fp), h.regionDD.Value    = char(s.region_name);          end
    if ismember('n_inf',                 fp), h.nInf.Value        = s.n_inf;                      end
    if ismember('inf_height_m',          fp), h.infH.Value        = s.inf_height_m;               end
    if ismember('n_veh',                 fp), h.nVeh.Value        = s.n_veh;                      end
    if ismember('veh_height_m',          fp), h.vehH.Value        = s.veh_height_m;               end
    if ismember('frequency_hz',          fp), h.freqMHz.Value     = s.frequency_hz / 1e6;         end
    if ismember('power_dbm',             fp), h.powerDbm.Value    = s.power_dbm;                  end
    if ismember('altitude_m',            fp), h.altM.Value        = s.altitude_m;                 end
    if ismember('tx_antenna_name',       fp), h.txAntDD.Value     = char(s.tx_antenna_name);      end
    if ismember('inf_antenna_name',      fp), h.infAntDD.Value    = char(s.inf_antenna_name);     end
    if ismember('veh_antenna_name',      fp), h.vehAntDD.Value    = char(s.veh_antenna_name);     end
    if ismember('bandwidth_hz',          fp), h.bwKHz.Value       = s.bandwidth_hz / 1e3;         end
    if ismember('use_terrain',           fp), h.useTerr.Value     = logical(s.use_terrain);       end
    if ismember('threshold_dbm',         fp), h.threshDbm.Value   = s.threshold_dbm;              end
    if ismember('percentile',            fp), h.percent.Value     = s.percentile;                 end
    if ismember('fade_margin_db',        fp), h.fadeDb.Value      = s.fade_margin_db;             end
    if ismember('num_flight_steps',      fp), h.flightSteps.Value = s.num_flight_steps;           end
    if ismember('num_rotations',         fp), h.numRot.Value      = s.num_rotations;              end
    if ismember('lemniscate_length_m',   fp), h.lemLenM.Value     = s.lemniscate_length_m;        end
    if ismember('placement_radius_km',   fp), h.placeKm.Value     = s.placement_radius_km;        end
    if ismember('max_tilt_deg',          fp), h.maxTilt.Value     = s.max_tilt_deg;               end

    h.statusLbl.Text = sprintf('Loaded %s', file);
end


function renderCcdf(fig)
    h = getappdata(fig, 'handles');
    results = getappdata(fig, 'last_results');
    cfg     = getappdata(fig, 'last_cfg');
    if isempty(results) || isempty(cfg), return; end
    plotAvailabilityCCDF(results, cfg, h.axCcdf, ...
        'show_inf',   logical(h.cbInf.Value), ...
        'show_veh',   logical(h.cbVeh.Value), ...
        'show_total', logical(h.cbTot.Value));
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
