function highways = osmReadHighways(osm_path)
%OSMREADHIGHWAYS  Parse highway polylines (lon, lat) from an OSM XML file.
%
%   highways = osmReadHighways(osm_path)
%
%   Returns a cell array of [N x 2] matrices (columns: lon, lat) for every
%   <way> tagged with k="highway". Buildings, waterways, and other
%   features are ignored - this is purely a visual reference layer.
%
%   Result is cached per (path, mtime) in a persistent variable so repeat
%   calls are free. The first parse of a multi-MB file takes a few seconds.

    persistent CACHE
    if isempty(CACHE), CACHE = containers.Map('KeyType','char','ValueType','any'); end

    osm_path = char(osm_path);
    info = dir(osm_path);
    if isempty(info)
        error('osmReadHighways:fileMissing', 'OSM file not found: %s', osm_path);
    end
    cache_key = sprintf('%s|%.6f|%d', osm_path, info.datenum, info.bytes);
    if isKey(CACHE, cache_key)
        highways = CACHE(cache_key);
        return;
    end

    text = fileread(osm_path);

    % --- Nodes: id, lat, lon ---------------------------------------------
    node_pat = '<node\s+id="(\d+)"[^>]*lat="([\-0-9\.]+)"[^>]*lon="([\-0-9\.]+)"';
    toks = regexp(text, node_pat, 'tokens');
    if isempty(toks)
        highways = {};
        CACHE(cache_key) = highways;
        return;
    end
    M    = vertcat(toks{:});
    ids  = str2double(M(:,1));
    lats = str2double(M(:,2));
    lons = str2double(M(:,3));

    [sorted_ids, sort_idx] = sort(ids);
    sorted_lons = lons(sort_idx);
    sorted_lats = lats(sort_idx);

    % --- Ways with k="highway" -------------------------------------------
    way_bodies = regexp(text, '(?s)<way\s[^>]*>(.*?)</way>', 'tokens');
    nways = numel(way_bodies);
    out = cell(nways, 1);
    n_out = 0;
    for i = 1:nways
        body = way_bodies{i}{1};
        if isempty(regexp(body, '<tag\s+k="highway"', 'once'))
            continue;
        end
        refs = regexp(body, '<nd\s+ref="(\d+)"', 'tokens');
        if numel(refs) < 2, continue; end
        ref_ids = str2double(vertcat(refs{:}));
        [tf, loc] = ismember(ref_ids, sorted_ids);
        if ~any(tf), continue; end
        wlon = sorted_lons(loc(tf));
        wlat = sorted_lats(loc(tf));
        n_out = n_out + 1;
        out{n_out} = [wlon, wlat];
    end
    highways = out(1:n_out);
    CACHE(cache_key) = highways;
end
