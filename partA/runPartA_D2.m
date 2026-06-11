function info = runPartA_D2(mode)
%RUNPARTA_D2  Convenience wrapper to run Part A with SBR R=2, D=2 (NLOS / diffraction).
%
%   runPartA_D2()          full run, all regions -> results/partA_latest.xlsx
%   runPartA_D2('probe')   time ONE region (region 2) into a temp dir (does NOT
%                          touch results/partA_latest.xlsx) so you can estimate
%                          the full-run cost before committing.
%
%   Run from the repo root in the MATLAB desktop with the path set:
%       cd <repo>; addpath(genpath(pwd)); runPartA_D2('probe')
%       runPartA_D2()
%
%   Forces R=2 / D=2 regardless of what buildConfigA currently holds, so the
%   diffraction (NLOS-rescue) run is explicit. Ctrl-C aborts cleanly.

    if nargin < 1 || isempty(mode), mode = 'full'; end
    repo = fileparts(fileparts(mfilename('fullpath')));   % partA/ -> repo

    close all force                                       % clear stray Site Viewers

    cfg = buildConfigA();
    cfg.propagation.max_reflections  = 2;
    cfg.propagation.max_diffractions = 2;
    fprintf('=== Part A  R=%d  D=%d  (%s)  %s ===\n', ...
        cfg.propagation.max_reflections, cfg.propagation.max_diffractions, mode, char(datetime('now')));

    switch lower(mode)
        case 'probe'
            cfg.io.results_dir = fullfile(tempdir, 'partA_probe');   % throwaway
            t = tic; info = runPartA(cfg, 2); el = toc(t);           % region 2 only
            fprintf('\nPROBE region 2: %.1f s (%.1f min)  ->  full 6-region run ~%.0f min\n', ...
                el, el/60, el*6/60);

        case 'full'
            % back up the existing latest before overwriting it
            lf = fullfile(repo, cfg.io.results_dir, 'partA_latest.xlsx');
            if exist(lf, 'file')
                copyfile(lf, fullfile(repo, cfg.io.results_dir, 'partA_prev_backup.xlsx'));
                fprintf('backed up current partA_latest.xlsx -> partA_prev_backup.xlsx\n');
            end
            info = runPartA(cfg);
            % how much did diffraction help vs the D=0 baseline (~0.34%% outage)?
            S = loadPartA(info.file);
            outage = mean(isinf(S.device.pl_db(:)));
            fprintf('\nOUTAGE (no-ray Inf PL): %.2f%%   [D=0 baseline was ~0.34%%]\n', 100*outage);

        otherwise
            error('runPartA_D2:mode', 'mode must be ''full'' or ''probe''');
    end
end
