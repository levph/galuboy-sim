%% run_galuboy.m -- Air-to-Ground Path Loss Simulation  v4.0
%
% Thin headless driver. Builds the default config, runs the scenario
% across all configured regions, and renders the two summary plots.
%
% PIPELINE
%   1. config = buildConfig()
%   2. results = runScenario(cfg, progress_cb)   % terrain register + per-rotation
%                                                % parfor + reduce; saves .mat
%   3. plotRangeHistogram(results, cfg)
%   4. plotAvailabilityMap(results, cfg)
%
% Override config fields between (1) and (2) for one-off experiments:
%   cfg = buildConfig();
%   cfg.flight.num_rotations = 4;
%   cfg.propagation.use_terrain = false;     % skip DT2 - default GMTED
%   results = runScenario(cfg, @defaultProgressPrinter);
%
% Authors: Tomer Antebi & Lev Panov
% Version: 4.0
% Date:    2026-05

repo_root = fileparts(mfilename('fullpath'));
addpath(fullfile(repo_root, 'utils'));   % bootstrap so addProjectPaths is visible
addProjectPaths(repo_root);

cfg = buildConfig();

t0 = tic;
results = runScenario(cfg, @defaultProgressPrinter);
fprintf('\nScenario complete in %.1f s\n', toc(t0));

% --- Summary printout -----------------------------------------------------
for r = 1:numel(results)
    avail = results(r).available;
    n_inf = cfg.rx.infantry.count;
    n_veh = cfg.rx.vehicular.count;
    if n_inf > 0 && n_veh > 0
        inf_avail = mean(reshape(avail(:, 1:n_inf), [], 1));
        veh_avail = mean(reshape(avail(:, n_inf + (1:n_veh)), [], 1));
        fprintf('  %-12s overall %.1f%%  (inf %.1f%%, veh %.1f%%)  M=%d, n_rx=%d\n', ...
            results(r).region.name, 100*mean(avail(:)), ...
            100*inf_avail, 100*veh_avail, ...
            size(avail, 1), size(avail, 2));
    else
        fprintf('  %-12s overall %.1f%%  M=%d, n_rx=%d\n', ...
            results(r).region.name, 100*mean(avail(:)), ...
            size(avail, 1), size(avail, 2));
    end
end

% --- Plots ----------------------------------------------------------------
plotRangeHistogram(results, cfg);
plotAvailabilityMap(results, cfg);


% =========================================================================
function defaultProgressPrinter(stage, region_idx, n_regions, rot_idx, n_rots, msg)
    fprintf('%s\n', plotProgress(stage, region_idx, n_regions, rot_idx, n_rots, msg));
end
