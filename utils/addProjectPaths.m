function addProjectPaths(repo_root)
%ADDPROJECTPATHS  Add galuboy-sim subdirectories to MATLAB's path.
%
%   addProjectPaths()           uses the directory containing this file's
%                                parent (i.e. <repo>/utils -> <repo>).
%   addProjectPaths(repo_root)  uses an explicit repo root.
%
%   Recursively adds production code only. Skips:
%     .claude/          (Claude Code worktrees that may contain stale
%                        copies of the codebase, shadowing real files)
%     .git/
%     legacy/           (pre-v4 reference modules)
%     terrain_cache/    (huge - tens of thousands of tile files)
%     results/          (saved .mat outputs)
%     resources/        (data files, not MATLAB code)
%
%   Without these exclusions, addpath(genpath(pwd)) on a repo with any
%   git worktree under .claude/ ends up shadowing the production code
%   with whatever ref the worktree points at - the symptom looks like
%   "Unrecognized field name 'regions'" or other config-schema mismatches.

    if nargin < 1 || isempty(repo_root)
        this_dir  = fileparts(mfilename('fullpath'));   % <repo>/utils
        repo_root = fileparts(this_dir);
    end

    % Top-level directories that contain MATLAB code.
    code_dirs = {'config', 'sim', 'terrain', 'antennas', 'visualization', ...
                 'utils', 'ui', 'tests'};

    % Add the repo root itself (for run_galuboy.m local function handle).
    addpath(repo_root);

    for k = 1:numel(code_dirs)
        d = fullfile(repo_root, code_dirs{k});
        if exist(d, 'dir')
            addpath(genpath(d));
        end
    end
end
