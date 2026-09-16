%% run_view_partB_ui.m -- open galuboyAnalysisApp (Part B) for manual QA
%
% Standalone script (no MCP round-trip / timeout) -- run this directly in
% MATLAB (F5). Sets up the path, points the app at real Part A output when
% available (falls back to a small synthetic workbook otherwise), and opens
% the interactive Part B analysis UI (MCS / fingers / antenna-name dropdowns
% added by the MCS-table feature -- see resources/README.md's `mcs/` section
% and config/buildConfigB.m).
%
% After it opens, walk through the checklist printed in the Command Window.

repo = '/Users/lev/galuboy-sim';
addpath(genpath(fullfile(repo,'config')));
addpath(genpath(fullfile(repo,'partA')));
addpath(genpath(fullfile(repo,'sim')));
addpath(genpath(fullfile(repo,'terrain')));
addpath(genpath(fullfile(repo,'antennas')));
addpath(genpath(fullfile(repo,'visualization')));
addpath(genpath(fullfile(repo,'ui')));
addpath(genpath(fullfile(repo,'utils')));
addpath(genpath(fullfile(repo,'partB')));   % last: partB/plotAvailability* must win
                                             % over visualization/'s same-named fn

% ---- Part A workbook: prefer real output, else a small synthetic one ------
realPartA = fullfile(repo, 'results', 'partA_candidate03_latest.xlsx');
if exist(realPartA, 'file') == 2
    partA_path = realPartA;
    fprintf('Using real Part A output: %s\n', partA_path);
else
    partA_path = fullfile(tempdir, 'synth_partA_ui_demo.xlsx');
    buildSyntheticPartA(partA_path);
    fprintf('No real Part A output found; using synthetic demo data: %s\n', partA_path);
end

galuboyAnalysisApp(partA_path);

fprintf([ ...
    '\n=== Part B UI manual checklist ===\n' ...
    ' 1. Graph dropdown: switch CCDF / histogram / margin -- plot redraws each time.\n' ...
    ' 2. Availability %% field: change 99 -> 90 -> Update plot -- curves shift.\n' ...
    ' 3. Series checkboxes: untick one -- its line disappears; retick -- it returns.\n' ...
    ' 4. Link parameters table, per row (link is read-only; MCS/fingers/antennas are\n' ...
    '    dropdowns -- click a cell to open the list):\n' ...
    '      - MCS column:    change e.g. DL_infantry to a LOW MCS (1-2, BPSK) and hit\n' ...
    '                       Update plot -- required-SNR drops, availability should rise\n' ...
    '                       (placeholder numbers currently give ~0%% availability at the\n' ...
    '                       committed default MCS choices, so this is the easy way to see\n' ...
    '                       the link budget respond).\n' ...
    '      - fingers column: raise 1 -> 2 -- legend rate (Mbps) for that series should\n' ...
    '                       double, and required BW (hence noise, hence SNR) changes too.\n' ...
    '      - tx/rx antenna columns: pick a different antenna name -- plot redraws using\n' ...
    '                       that pattern''s gain.\n' ...
    ' 5. Legend text: confirm each series shows "<link> / <terrain> (<rate> Mbps)" and the\n' ...
    '    rate matches finger count x that MCS''s rate_1finger_mbps for the link''s direction\n' ...
    '    (resources/mcs/mcs_tables.xlsx).\n' ...
    ' 6. "Load MCS workbook..." button: point it at a copy of resources/mcs/mcs_tables.xlsx\n' ...
    '    you''ve edited (e.g. changed req_snr_db) -- the MCS dropdown list and the plot both\n' ...
    '    update after Update plot.\n' ...
    ' 7. "Load antenna workbook..." button: same check, against resources/antennas/antenna_patterns.xlsx.\n' ...
    ' 8. "Load Part A workbook..." button: point it at a different Part A .xlsx if you have one.\n' ...
    '===================================\n\n']);

% -------------------------------------------------------------------------
function buildSyntheticPartA(xlsx)
%BUILDSYNTHETICPARTA  Small 2-terrain-category, 2-device-type demo workbook.
    if exist(xlsx, 'file'), delete(xlsx); end
    Nf = 8;
    P = table(["frequency_hz";"num_samples"], ["4e9";num2str(Nf)], ...
              'VariableNames', {'name','value'});
    Rg = table([1;2], ["r1";"r2"], ["urban_suburban";"other"], ...
              'VariableNames', {'region_id','name','category'});

    n = 80;   % 20 per (type x terrain) combo, so all 8 series have data
    rng(7);
    type      = repmat([repmat("infantry",n/4,1);  repmat("vehicular",n/4,1)], 2, 1);
    region_id = [ones(n/2,1); 2*ones(n/2,1)];
    region    = ["r1";"r2"]; region = region(region_id);
    height_m  = double(type=="infantry")*1.5 + double(type=="vehicular")*2.5;
    dist_m    = linspace(50, 3000, n)';
    lon       = 35 + 0.01*(1:n)';
    lat       = 32 + 0.01*(1:n)';

    pl_db     = 90 + 0.02*dist_m + 3*randn(n,Nf);
    steer_gnd = 10*rand(n,Nf);
    steer_air = 20*rand(n,Nf);

    D = table((1:n)', region, region_id, type, height_m, lon, lat, dist_m, ...
        rowjoin(pl_db), rowjoin(steer_gnd), rowjoin(steer_air), ...
        'VariableNames', {'device_id','region','region_id','type','height_m','lon','lat', ...
        'dist_m','pl_db','steer_gnd_deg','steer_air_deg'});

    writetable(P,  xlsx, 'Sheet', 'params');
    writetable(Rg, xlsx, 'Sheet', 'regions');
    writetable(D,  xlsx, 'Sheet', 'devices');
end

function s = rowjoin(M)
    s = strings(size(M,1),1);
    for i = 1:size(M,1)
        s(i) = strjoin(string(M(i,:)), ',');
    end
end
