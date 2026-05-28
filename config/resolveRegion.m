function region = resolveRegion(region_cfg, repo_root)
%RESOLVEREGION  Materialise a region entry into absolute paths and bounds.
%
%   region = resolveRegion(region_cfg, repo_root)
%
%   region_cfg  one element of config.regions (struct with fields:
%                 name, osm_path, latlim_override, lonlim_override)
%   repo_root   absolute path to the repo root, used to resolve relative
%               osm_path values
%
%   Returned region:
%     name                   char
%     osm_path               absolute path
%     latlim, lonlim         [lo, hi] in degrees
%     center_lat, center_lon
%     placement_diameter_km  scalar km (passed through; [] if absent)
%     category               char  (passed through; '' if absent)
%
%   Errors with a friendly message naming the expected drop path when
%   the OSM file is missing.

    % Resolve OSM path to absolute
    if isAbsolutePath(region_cfg.osm_path)
        osm_path = region_cfg.osm_path;
    else
        osm_path = fullfile(repo_root, region_cfg.osm_path);
    end

    has_overrides = ~isempty(region_cfg.latlim_override) && ~isempty(region_cfg.lonlim_override);

    if ~has_overrides && ~isfile(osm_path)
        error('resolveRegion:osmMissing', ...
              ['Region "%s" expects an OSM file at:\n  %s\n' ...
               'Drop your .osm into resources/osm/ and re-run.'], ...
              region_cfg.name, osm_path);
    end

    % Bounds: prefer overrides, otherwise read from OSM <bounds> tag
    if has_overrides
        latlim = region_cfg.latlim_override(:).';
        lonlim = region_cfg.lonlim_override(:).';
    else
        [lonlim, latlim, ~] = osmReadBounds(osm_path);
    end

    if isfield(region_cfg, 'placement_diameter_km')
        placement_diameter_km = region_cfg.placement_diameter_km;
    else
        placement_diameter_km = [];
    end
    if isfield(region_cfg, 'category')
        category = char(region_cfg.category);
    else
        category = '';
    end

    region = struct( ...
        'name',                  char(region_cfg.name), ...
        'osm_path',              char(region_cfg.osm_path), ...
        'latlim',                latlim, ...
        'lonlim',                lonlim, ...
        'center_lat',            mean(latlim), ...
        'center_lon',            mean(lonlim), ...
        'placement_diameter_km', placement_diameter_km, ...
        'category',              category);
end


% -------------------------------------------------------------------------
function tf = isAbsolutePath(p)
    tf = ~isempty(p) && (p(1) == '/' || p(1) == '\' || ...
         (numel(p) >= 2 && p(2) == ':'));
end
