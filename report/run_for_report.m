%RUN_FOR_REPORT  Drive a galuboy-sim run for the report.
%   Runs two scenarios:
%     (A) urban_suburban regions  -- 5 km RX placement DIAMETER
%     (B) all other regions        -- 10 km RX placement DIAMETER
%   Saves a combined cfg+results to results/report_run.mat.

repo_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(repo_root));

% ---- Common base config -------------------------------------------------
base_cfg = buildConfig();
base_cfg.flight.num_flight_steps = 50;

% Partition: urban_suburban_* vs everything else.
region_names  = {base_cfg.regions.name};
is_built      = startsWith(region_names, 'urban_suburban');
regions_built = base_cfg.regions(is_built);
regions_other = base_cfg.regions(~is_built);

% ---- Scenario A: built (urban_suburban), 5 km DIAMETER -----------------
cfg_a = base_cfg;
cfg_a.regions = regions_built;
cfg_a.rx.placement_diameter_km = 5;

fprintf('[run_for_report] Scenario A (5 km diameter, %d regions): %s\n', ...
        numel(cfg_a.regions), strjoin({cfg_a.regions.name}, ', '));
t0 = tic;
results_a = runScenario(cfg_a, @(varargin) fprintf('  %s\n', plotProgress(varargin{:})));
fprintf('[run_for_report] Scenario A elapsed: %.1f min\n', toc(t0) / 60);

% ---- Scenario B: forest + mountain + open, 10 km DIAMETER --------------
cfg_b = base_cfg;
cfg_b.regions = regions_other;
cfg_b.rx.placement_diameter_km = 10;

fprintf('[run_for_report] Scenario B (10 km diameter, %d regions): %s\n', ...
        numel(cfg_b.regions), strjoin({cfg_b.regions.name}, ', '));
t0 = tic;
results_b = runScenario(cfg_b, @(varargin) fprintf('  %s\n', plotProgress(varargin{:})));
fprintf('[run_for_report] Scenario B elapsed: %.1f min\n', toc(t0) / 60);

% ---- Combine -----------------------------------------------------------
% Stash each region's placement diameter onto the result struct so the
% renderer can label/scale appropriately.
for k = 1:numel(results_a)
    results_a(k).placement_diameter_km = single(cfg_a.rx.placement_diameter_km);
end
for k = 1:numel(results_b)
    results_b(k).placement_diameter_km = single(cfg_b.rx.placement_diameter_km);
end
results = [results_a, results_b];

% Combined cfg: keep scenario A as canonical, but record both diameters.
cfg = cfg_a;
cfg.regions = base_cfg.regions;
cfg.rx.placement_diameter_km_built = 5;
cfg.rx.placement_diameter_km_other = 10;

out_path = fullfile(repo_root, cfg.io.results_dir, 'report_run.mat');
save(out_path, 'cfg', 'results', '-v7.3');
fprintf('[run_for_report] Saved %s  (n_regions = %d)\n', out_path, numel(results));
