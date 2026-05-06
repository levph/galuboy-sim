function tile_files = discoverTerrainTiles(terrain_dir, latlim, lonlim)
%DISCOVERTERRAINTILES  Find DT2 tiles intersecting a lat/lon bounding box.
%
%   tile_files = discoverTerrainTiles(terrain_dir, latlim, lonlim)
%
%   Globs *.dt2 (and *.dt1, *.tif) under terrain_dir. Parses standard SRTM
%   1-arc-second filenames of the form  n{LL}_{e|w}{LLL}_*.{dt1,dt2,tif}
%   where the leading lower-left corner is (LL, +/-LLL). Returns the
%   absolute paths of every tile whose 1-deg square intersects
%   [latlim, lonlim] expanded by a 0.05-deg buffer.
%
%   Errors with a clear message (and a list of tiles seen) when no tile
%   covers the requested box.
%
%   terrain_dir   absolute path to the directory holding the tile files
%   latlim        [lat_min, lat_max] in degrees
%   lonlim        [lon_min, lon_max] in degrees
%
%   tile_files    cell array of absolute paths (column)

    arguments
        terrain_dir (1,:) char
        latlim      (1,2) double
        lonlim      (1,2) double
    end

    if ~exist(terrain_dir, 'dir')
        error('discoverTerrainTiles:dirMissing', ...
              'Terrain directory not found: %s', terrain_dir);
    end

    BUFFER_DEG = 0.05;
    lat_lo = min(latlim) - BUFFER_DEG;
    lat_hi = max(latlim) + BUFFER_DEG;
    lon_lo = min(lonlim) - BUFFER_DEG;
    lon_hi = max(lonlim) + BUFFER_DEG;

    patterns = {'*.dt2', '*.dt1', '*.tif', '*.tiff'};
    all_files = {};
    for k = 1:numel(patterns)
        d = dir(fullfile(terrain_dir, patterns{k}));
        for j = 1:numel(d)
            all_files{end+1} = fullfile(d(j).folder, d(j).name); %#ok<AGROW>
        end
    end

    if isempty(all_files)
        error('discoverTerrainTiles:noTiles', ...
              ['No DT2/DT1/TIF tiles found in %s.\n' ...
               'Drop SRTM 1-arc tiles (e.g. n32_e035_1arc_v3.dt2) into resources/terrain/.'], ...
              terrain_dir);
    end

    keep = false(1, numel(all_files));
    parsed_any = false;
    for k = 1:numel(all_files)
        [tile_lat, tile_lon, ok] = parseSrtmName(all_files{k});
        if ~ok
            % Non-SRTM TIF: fall back to geotiffinfo for bbox
            [~, ~, ext] = fileparts(all_files{k});
            if ~ismember(lower(ext), {'.tif', '.tiff'})
                continue;
            end
            try
                info = geotiffinfo(all_files{k});
                if ~strcmpi(info.SpatialRef.Type, 'geographic')
                    continue;   % projected CRS — can't compare lat/lon directly
                end
                bb = info.BoundingBox;  % [lon_min lat_min; lon_max lat_max]
                tif_lonlim = [bb(1,1), bb(2,1)];
                tif_latlim = [bb(1,2), bb(2,2)];
                parsed_any = true;
                if (tif_latlim(2) >= lat_lo) && (tif_latlim(1) <= lat_hi) && ...
                   (tif_lonlim(2) >= lon_lo) && (tif_lonlim(1) <= lon_hi)
                    keep(k) = true;
                end
            catch
                % geotiffinfo unavailable or file unreadable — skip silently
            end
            continue;
        end
        parsed_any = true;
        % 1-deg square at tile lower-left corner -> [tile_lat, tile_lat+1] x
        % [tile_lon, tile_lon+1]. Intersect with the buffered query box.
        if (tile_lat + 1 >= lat_lo) && (tile_lat <= lat_hi) && ...
           (tile_lon + 1 >= lon_lo) && (tile_lon <= lon_hi)
            keep(k) = true;
        end
    end

    if ~parsed_any
        error('discoverTerrainTiles:unrecognisedNames', ...
              ['Found %d tile files in %s but none matched the SRTM naming\n' ...
               'convention (n{LL}_e{LLL}_..., or s{LL}_w{LLL}_...). Rename them\n' ...
               'or pass the file list explicitly.'], ...
              numel(all_files), terrain_dir);
    end

    tile_files = all_files(keep);

    if isempty(tile_files)
        listing = sprintf('  %s\n', all_files{:});
        error('discoverTerrainTiles:noCoverage', ...
              ['No tiles in %s cover lat=[%.3f, %.3f], lon=[%.3f, %.3f].\n' ...
               'Tiles seen:\n%s'], ...
              terrain_dir, latlim(1), latlim(2), lonlim(1), lonlim(2), listing);
    end

    tile_files = tile_files(:);   % column
end


% -------------------------------------------------------------------------
function [tile_lat, tile_lon, ok] = parseSrtmName(filepath)
%PARSESRTMNAME  Extract lower-left corner from a tile filename.
%
%   Recognises  n{LL}_e{LLL}_*  and  s{LL}_w{LLL}_*  (case-insensitive),
%   where LL is 1-2 digits of latitude and LLL is 1-3 digits of longitude.

    [~, name, ~] = fileparts(filepath);
    tok = regexpi(name, '^([ns])(\d{1,2})_([ew])(\d{1,3})', 'tokens', 'once');
    if isempty(tok)
        tile_lat = NaN; tile_lon = NaN; ok = false;
        return;
    end
    lat_sign = lower(tok{1}); lat_num = str2double(tok{2});
    lon_sign = lower(tok{3}); lon_num = str2double(tok{4});
    if lat_sign == 's', lat_num = -lat_num; end
    if lon_sign == 'w', lon_num = -lon_num; end
    tile_lat = lat_num;
    tile_lon = lon_num;
    ok = true;
end
