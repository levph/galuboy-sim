function tester = buildingRejectMask(buildings)
%BUILDINGREJECTMASK  Fast point-in-buildings tester for a region's footprints.
%
%   tester = buildingRejectMask(geojson_path)
%   tester = buildingRejectMask(geotable)        % from loadBuildings
%
%   Returns a function handle  tester(lon, lat) -> logical  that reports, for
%   each query point, whether it falls inside ANY building footprint of the
%   region. lon/lat may be scalars or matching arrays; the result has the same
%   shape. Designed to be built ONCE per region and reused across all RX
%   rejection-sampling attempts (placeReceivers accepts this handle directly).
%
%   Why not a single big polyshape + isinterior? At these extents (a few km)
%   lon/lat are treated as planar x/y, which is fine for point-in-polygon. But
%   isinterior() on a polyshape of thousands of disjoint regions is ~2 ms per
%   point - ~20 s for the worst-case 200 RX x 50 attempts. Instead we keep each
%   footprint's ring + axis-aligned bounding box and, per query point, prefilter
%   by bbox (vectorised) and run inpolygon only on the handful of footprints
%   whose bbox actually contains the point. That drops the worst case to a
%   fraction of a second.
%
%   Robust to either coordinate encoding jsondecode produces for a GeoJSON
%   Polygon: a numeric [1 x nVert x 2] array (single ring, the common Overture
%   case) or a cell of rings (holes / multipolygon). Only the exterior ring of
%   each polygon is used for rejection (interior holes are ignored - dropping a
%   receiver in a courtyard hole is acceptable and rare).
%
%   See loadBuildings.m (the geotable loader) and placeReceivers.m (consumer).

    [rings, bx] = parseFootprints(buildings);

    % Closure capturing the per-building rings + bounding boxes. n_b == 0 yields
    % an all-false tester (no rejection) so callers needn't special-case it.
    tester = @(lon, lat) insideAny(lon, lat, rings, bx);
end


% -------------------------------------------------------------------------
function tf = insideAny(lon, lat, rings, bx)
%INSIDEANY  Vectorised point-in-any-footprint via bbox prefilter + inpolygon.
    sz = size(lon);
    lon = lon(:);  lat = lat(:);
    tf  = false(numel(lon), 1);
    if isempty(rings)
        tf = reshape(tf, sz);
        return;
    end
    for k = 1:numel(lon)
        lo = lon(k);  la = lat(k);
        % Cheap vectorised bbox test narrows thousands of footprints to a few.
        cand = find(lo >= bx(:,1) & lo <= bx(:,2) & ...
                    la >= bx(:,3) & la <= bx(:,4));
        for j = cand.'
            r = rings{j};
            if inpolygon(lo, la, r(:,1), r(:,2))
                tf(k) = true;
                break;
            end
        end
    end
    tf = reshape(tf, sz);
end


% -------------------------------------------------------------------------
function [rings, bx] = parseFootprints(buildings)
%PARSEFOOTPRINTS  -> per-building exterior rings [Nx2 lon,lat] + bboxes.
    if ischar(buildings) || (isstring(buildings) && isscalar(buildings))
        % GeoJSON path: parse coordinates directly (most reliable in R2025a;
        % geopolyshape exposes no vertex accessor).
        if exist(buildings, 'file') ~= 2
            error('buildingRejectMask:fileNotFound', ...
                  'Building file not found: %s', buildings);
        end
        S = jsondecode(fileread(buildings));
        feats = S.features;
        nf = numel(feats);
        rings = cell(nf, 1);
        for i = 1:nf
            rings{i} = ringFromGeoJSON(feats(i).geometry.coordinates);
        end
    elseif istable(buildings) || isa(buildings, 'table')
        error('buildingRejectMask:geotableUnsupported', ...
              ['geopolyshape geotables expose no vertex accessor; pass the ' ...
               'GeoJSON path instead (see partA/setupRegionScene for the path).']);
    elseif iscell(buildings)
        % Legacy polygon-cell form (each cell [Nx2] lon,lat).
        rings = buildings(:);
    else
        error('buildingRejectMask:badInput', ...
              'buildings must be a GeoJSON path or a cell of [Nx2] rings.');
    end

    % Drop empties and build per-ring bounding boxes [minlon maxlon minlat maxlat].
    keep  = ~cellfun(@isempty, rings);
    rings = rings(keep);
    nb    = numel(rings);
    bx    = zeros(nb, 4);
    for i = 1:nb
        r = rings{i};
        bx(i,:) = [min(r(:,1)) max(r(:,1)) min(r(:,2)) max(r(:,2))];
    end
end


% -------------------------------------------------------------------------
function ring = ringFromGeoJSON(coords)
%RINGFROMGEOJSON  Exterior ring [Nx2 lon,lat] from a Polygon coordinates blob.
    if iscell(coords)
        % {ring1, ring2, ...} - exterior ring is the first.
        ring = reshape(coords{1}, [], 2);
    else
        % Numeric [1 x nVert x 2] (jsondecode of a single-ring polygon).
        ring = squeeze(coords(1, :, :));
        if size(ring, 2) ~= 2          % degenerate (single vertex) guard
            ring = reshape(ring, [], 2);
        end
    end
end
