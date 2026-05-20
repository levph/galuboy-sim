function buildings = osmReadBuildings(osm_path)
%OSMREADBUILDINGS  Parse building footprints (lon, lat) from an OSM XML file.
%
%   buildings = osmReadBuildings(osm_path)
%
%   Returns a cell array of [N x 2] matrices (columns: lon, lat), one per
%   <way> tagged with k="building" (any value). Used by placeReceivers to
%   reject RX samples that land inside a building footprint.
%
%   - Returns {} if the file has no buildings, is missing, or is unreadable;
%     placement should continue gracefully for OSM-less regions.
%   - Result is cached per (path, mtime) in a persistent variable so repeat
%     calls (across rotations within a region) are free.

    persistent CACHE
    if isempty(CACHE), CACHE = containers.Map('KeyType','char','ValueType','any'); end

    if nargin < 1 || isempty(osm_path)
        buildings = {};
        return;
    end
    osm_path = char(osm_path);

    info = dir(osm_path);
    if isempty(info)
        buildings = {};
        return;
    end
    cache_key = sprintf('%s|%.6f|%d', osm_path, info.datenum, info.bytes);
    if isKey(CACHE, cache_key)
        buildings = CACHE(cache_key);
        return;
    end

    try
        text = fileread(osm_path);
    catch
        buildings = {};
        return;
    end

    % --- Nodes: id, lat, lon ---------------------------------------------
    node_pat = '<node\s+id="(\d+)"[^>]*lat="([\-0-9\.]+)"[^>]*lon="([\-0-9\.]+)"';
    toks = regexp(text, node_pat, 'tokens');
    if isempty(toks)
        buildings = {};
        CACHE(cache_key) = buildings;
        return;
    end
    M    = vertcat(toks{:});
    ids  = str2double(M(:,1));
    lats = str2double(M(:,2));
    lons = str2double(M(:,3));

    [sorted_ids, sort_idx] = sort(ids);
    sorted_lons = lons(sort_idx);
    sorted_lats = lats(sort_idx);

    % --- Ways with k="building" (any value) ------------------------------
    way_bodies = regexp(text, '(?s)<way\s[^>]*>(.*?)</way>', 'tokens');
    nways = numel(way_bodies);
    out = cell(nways, 1);
    n_out = 0;
    for i = 1:nways
        body = way_bodies{i}{1};
        if isempty(regexp(body, '<tag\s+k="building"', 'once'))
            continue;
        end
        refs = regexp(body, '<nd\s+ref="(\d+)"', 'tokens');
        if numel(refs) < 3, continue; end   % need >=3 points for a polygon
        ref_ids = str2double(vertcat(refs{:}));
        [tf, loc] = ismember(ref_ids, sorted_ids);
        if ~any(tf), continue; end
        wlon = sorted_lons(loc(tf));
        wlat = sorted_lats(loc(tf));
        n_out = n_out + 1;
        out{n_out} = [wlon, wlat];
    end
    buildings = out(1:n_out);
    CACHE(cache_key) = buildings;
end
