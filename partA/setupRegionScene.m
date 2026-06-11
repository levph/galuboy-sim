function sv = setupRegionScene(region, cfg, visible)
%SETUPREGIONSCENE  Build a raytracing Site Viewer for one region (Part A).
%
%   sv = setupRegionScene(region, cfg)            % hidden (fast batch default)
%   sv = setupRegionScene(region, cfg, true)      % visible (interactive)
%
%   Combines the (already registered) shared FABDEM terrain with the region's
%   building footprints into a single Site Viewer that raytrace/pathloss can
%   use as their Map. Buildings require a Site Viewer (terrain-only raytrace is
%   headless, but buildings are not) - this is the single-instance scene the
%   per-rotation sweep reuses.
%
%   Inputs:
%     region   resolved region struct (needs .name); from resolveRegion
%     cfg      buildConfigA() config (uses propagation.terrain_name, io.buildings_dir)
%
%   The shared terrain must be registered first (once, union bbox) via
%   discoverTerrainTiles + setupTerrain - the Part A driver does that before
%   the region loop.
%
%   Requires R2025a (siteviewer building geotables).

    if nargin < 3 || isempty(visible)
        visible = false;   % batch default: hidden viewer tiles much faster
    end

    repo_root = fileparts(fileparts(mfilename('fullpath')));   % partA/ -> repo
    geojson   = fullfile(repo_root, cfg.io.buildings_dir, [region.name '.geojson']);

    B = loadBuildings(geojson);

    vis = 'off'; if visible, vis = 'on'; end
    sv = siteviewer('Terrain',   cfg.propagation.terrain_name, ...
                    'Buildings', B, ...
                    'Visible',   vis);
end
