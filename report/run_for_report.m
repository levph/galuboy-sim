%RUN_FOR_REPORT  Drive a galuboy-sim run for the report.
%   Runs two scenarios:
%     (A) built / forest / mountain regions at 5 km RX placement radius
%     (B) open region at 10 km RX placement radius
%   Saves a combined cfg+results to results/report_run.mat.

repo_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(repo_root));

% ---- Common base config -------------------------------------------------
base_cfg = buildConfig();
% Exclude yarka; we don't reference it in the report.
keep = ~strcmp({base_cfg.regions.name}, 'yarka');
base_cfg.regions = base_cfg.regions(keep);
base_cfg.flight.num_flight_steps = 50;

% Partition regions: 5 km radius for built (urban_suburban_*),
% 10 km radius for everything else (forest, mountain, rural_open, ...).
region_names = {base_cfg.regions.name};
is_built = startsWith(region_names, 'urban_suburban');
regions_built = base_cfg.regions(is_built);
regions_other = base_cfg.regions(~is_built);

% ---- Scenario A: built (urban_suburban), 5 km radius -------------------
cfg_a = base_cfg;
cfg_a.regions = regions_built;
cfg_a.rx.placement_radius_km = 5;

fprintf('[run_for_report] Scenario A (5 km radius, %d regions): %s\n', ...
        numel(cfg_a.regions), strjoin({cfg_a.regions.name}, ', '));
t0 = tic;
results_a = runScenario(cfg_a, @(varargin) fprintf('  %s\n', plotProgress(varargin{:})));
fprintf('[run_for_report] Scenario A elapsed: %.1f min\n', toc(t0) / 60);

% ---- Scenario B: forest + mountain + open, 10 km radius ----------------
cfg_b = base_cfg;
cfg_b.regions = regions_other;
cfg_b.rx.placement_radius_km = 10;

fprintf('[run_for_report] Scenario B (10 km radius, %d regions): %s\n', ...
        numel(cfg_b.regions), strjoin({cfg_b.regions.name}, ', '));
t0 = tic;
results_b = runScenario(cfg_b, @(varargin) fprintf('  %s\n', plotProgress(varargin{:})));
fprintf('[run_for_report] Scenario B elapsed: %.1f min\n', toc(t0) / 60);

% ---- Combine -----------------------------------------------------------
% Stash each region's placement radius onto the result struct so the
% renderer can label/scale appropriately.
for k = 1:numel(results_a)
    results_a(k).placement_radius_km = single(cfg_a.rx.placement_radius_km);
end
for k = 1:numel(results_b)
    results_b(k).placement_radius_km = single(cfg_b.rx.placement_radius_km);
end
results = [results_a, results_b];

% Combined cfg: keep scenario A as canonical, but record both radii
cfg = cfg_a;
cfg.regions = base_cfg.regions;
cfg.rx.placement_radius_km_built = 5;
cfg.rx.placement_radius_km_other = 10;

out_path = fullfile(repo_root, cfg.io.results_dir, 'report_run.mat');
save(out_path, 'cfg', 'results', '-v7.3');
fprintf('[run_for_report] Saved %s  (n_regions = %d)\n', out_path, numel(results));
