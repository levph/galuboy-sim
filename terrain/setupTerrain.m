function setupTerrain(terrain_name, tile_files)
%SETUPTERRAIN Register DTED terrain tiles with MATLAB.
%
%   setupTerrain(terrain_name, tile_files) registers the given DTED
%   tile files under terrain_name using a PERSISTENT on-disk cache
%   under <repo>/terrain_cache/<terrain_name>/. When the named terrain
%   is already registered and its cache looks healthy, registration is
%   reused as-is, which avoids the ~60 s retile on every simulation run.
%
%   tile_files - cell array (or string array) of full paths to .dt2 files
%
%   Why the explicit WriteLocation:
%   addCustomTerrain's default WriteLocation is tempname, which resolves
%   under tempdir. When MATLAB is launched in -batch from a backend
%   subprocess, $TMPDIR is typically empty and tempdir falls back to
%   /private/tmp, which macOS periodically cleans. The custom-terrain
%   settings group persists across sessions as a PersonalValue in user
%   prefs - so the registration survives, but the tile data on disk
%   does not, and every subsequent query fails with
%       "Unable to access terrain ..."
%       "copyfile: No matching files named /private/tmp/tp.../Z/X/Y.terrain"
%   Writing to a persistent path under the project side-steps this.
%
%   Pattern adapted from /Users/lev/freq-planner/matlab/code/terrain/setupTerrain.m

    if isstring(tile_files)
        tile_files = cellstr(tile_files);
    end
    for k = 1:numel(tile_files)
        assert(isfile(tile_files{k}), 'Tile not found: %s', tile_files{k});
    end

    % Persistent cache location: <repo>/terrain_cache/<terrain_name>/
    this_dir   = fileparts(mfilename('fullpath'));   % <repo>/terrain
    repo_root  = fileparts(this_dir);                 % <repo>/
    cache_root = fullfile(repo_root, 'terrain_cache');
    write_location = fullfile(cache_root, terrain_name);

    % Fingerprint of the requested tile set - if this matches the manifest
    % left behind by a previous successful registration, the cache on disk
    % corresponds to the same tiles and can be reused.
    manifest_path = fullfile(write_location, 'tile_manifest.txt');
    manifest      = buildTileManifest(tile_files);

    % Fast path: reuse cache iff registration is healthy AND the cache was
    % built from this exact tile set.
    if isValidRegistration(terrain_name, write_location) && ...
            manifestMatches(manifest_path, manifest)
        fprintf('[Terrain] "%s" cache reused (%s)\n', terrain_name, write_location);
        return;
    end

    % Otherwise, clear any stale registration (settings group + any files
    % that happen to still live at the stale writeLocation).
    try
        removeCustomTerrain(terrain_name);
        fprintf('[Terrain] Removed stale "%s" registration\n', terrain_name);
    catch
        % Either the name wasn't registered, or removeCustomTerrain errored
        % trying to clean a vanished writeLocation. Fall back to removing
        % just the settings group so addCustomTerrain below won't fail with
        % TerrainNameExists.
        try
            s = settings;
            opts = properties(s.shared.terrain);
            [ismem, ind] = ismember(lower(terrain_name), lower(opts));
            if ismem
                s.shared.terrain.removeGroup(opts{ind});
                fprintf('[Terrain] Force-removed settings group for "%s"\n', terrain_name);
            end
        catch
            % Nothing else we can do; let addCustomTerrain surface the error.
        end
    end

    % Ensure the cache dir exists and is empty before retiling. A partial /
    % corrupt cache from an aborted run would otherwise confuse the Tiler
    % or the query layer.
    %
    % MATLAB's terrain Tiler adds every numeric zoom-level subfolder to
    % the MATLAB path. rmdir then auto-emits a MATLAB:RMDIR:RemovedFromPath
    % warning per folder - hundreds of lines of noise. Silence them.
    if exist(write_location, 'dir')
        warn_state = warning('off', 'MATLAB:RMDIR:RemovedFromPath');
        cleaner    = onCleanup(@() warning(warn_state));
        rmdir(write_location, 's');
        clear cleaner;
    end
    if ~exist(cache_root, 'dir')
        mkdir(cache_root);
    end
    mkdir(write_location);

    % FillMissing=true: addCustomTerrain otherwise requires a strictly
    % rectangular tile set. Real SRTM coverage on the project's edges is
    % often non-rectangular (ocean tiles missing, etc); fill those with 0.
    addCustomTerrain(terrain_name, string(tile_files), ...
        'WriteLocation', write_location, ...
        'FillMissing',   true);

    % Write the manifest so the next run's fast path can verify the cache
    % corresponds to the same set of source tiles.
    fid = fopen(manifest_path, 'w');
    if fid >= 0
        fprintf(fid, '%s', manifest);
        fclose(fid);
    end

    fprintf('[Terrain] "%s" registered at %s (%d tiles)\n', ...
        terrain_name, write_location, numel(tile_files));
end


% -------------------------------------------------------------------------
function m = buildTileManifest(tile_files)
%BUILDTILEMANIFEST Stable fingerprint of a tile set: sorted absolute paths,
%one per line. Matches iff the paths and their order (after sort) are
%identical.
    abs_paths = cell(1, numel(tile_files));
    for k = 1:numel(tile_files)
        f = tile_files{k};
        if ~isAbsolutePath(f)
            f = fullfile(pwd, f);
        end
        abs_paths{k} = f;
    end
    abs_paths = sort(abs_paths);
    m = strjoin(abs_paths, newline);
end


function tf = manifestMatches(manifest_path, expected)
    tf = false;
    if ~isfile(manifest_path)
        return;
    end
    try
        fid = fopen(manifest_path, 'r');
        raw = fread(fid, '*char')';
        fclose(fid);
        tf = strcmp(raw, expected);
    catch
        tf = false;
    end
end


function tf = isValidRegistration(name, expected_location)
%ISVALIDREGISTRATION True iff the named terrain is registered, points at
%our expected persistent cache, and that cache still contains at least the
%max-zoom tile folder generated by the Tiler.

    tf = false;
    try
        s = settings;
        opts = properties(s.shared.terrain);
        if ~ismember(lower(name), lower(opts))
            return;
        end
        g   = s.shared.terrain.(name);
        loc = g.Location.ActiveValue;

        % Stale if it points anywhere other than our persistent cache
        % (e.g. a previous tempdir path from before this fix landed).
        if ~strcmp(loc, expected_location)
            return;
        end
        if ~exist(loc, 'dir')
            return;
        end

        % The Tiler writes tiles into numeric zoom-level subfolders. Probe
        % the max zoom folder - cheap and specific.
        maxZoom = g.MaxZoomLevel.ActiveValue;
        if ~exist(fullfile(loc, num2str(maxZoom)), 'dir')
            return;
        end
        tf = true;
    catch
        tf = false;
    end
end


function tf = isAbsolutePath(p)
%ISABSOLUTEPATH True if p is an absolute filesystem path (cross-platform).
    tf = ~isempty(p) && (p(1) == '/' || p(1) == '\' || ...
         (numel(p) >= 2 && p(2) == ':'));
end
