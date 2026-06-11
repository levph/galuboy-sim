function T = loadBuildings(geojson_path)
%LOADBUILDINGS  Load a region's building footprints as a raytracing geotable.
%
%   T = loadBuildings(geojson_path)
%
%   Reads an Overture/nDSM building GeoJSON (geopolyshape footprints with a
%   derived per-building `height` in metres) and returns a geotable shaped for
%   siteviewer/raytrace: it adds the MinHeight / MaxHeight columns the ray
%   tracer extrudes each footprint between.
%
%     MinHeight = 0          (footprints sit on the terrain surface)
%     MaxHeight = height     (nDSM-derived, floored 3 m / capped 60 m)
%
%   Requires R2025a (readgeotable + siteviewer building geotables).
%
%   See memory project_mapping_raytrace.md: heights live in the `height`
%   property; OSM is NOT used (Overture/OSM height tags are empty for this AOI,
%   and urban_suburban_01.osm is node-less / unreadable).

    if exist(geojson_path, 'file') ~= 2
        error('loadBuildings:fileNotFound', ...
            'Building file not found: %s', geojson_path);
    end

    T = readgeotable(geojson_path);

    if ~ismember('height', T.Properties.VariableNames)
        error('loadBuildings:noHeight', ...
            '%s has no "height" column; expected nDSM-derived heights.', geojson_path);
    end

    h = double(T.height);
    h(~isfinite(h) | h <= 0) = 3;          % guard: floor anything missing/bad

    T.MinHeight = zeros(height(T), 1);
    T.MaxHeight = h;
end
