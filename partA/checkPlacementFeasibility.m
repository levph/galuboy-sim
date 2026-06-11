function info = checkPlacementFeasibility(region_sel, cfg, n_mc)
%CHECKPLACEMENTFEASIBILITY  Can RX be placed off-buildings in a region's disc?
%
%   info = checkPlacementFeasibility(region_sel)
%   info = checkPlacementFeasibility(region_sel, cfg, n_mc)
%
%   Quantifies how much OPEN GROUND exists inside a region's RX-placement disc,
%   so we know rejection sampling (no-RX-on-building) can actually succeed - and
%   that dense urban clusters still leave gaps a receiver can land in.
%
%   Method (point-in-polygon is exact; this is purely a coverage question):
%     - Monte-Carlo n_mc uniform points in the disc, test against the footprint
%       rejecter (buildingRejectMask) -> building coverage fraction.
%     - open = 1 - coverage; expected rejection attempts per RX = 1/open.
%     - grid the disc and report the DENSEST cell's coverage (worst local case);
%       open>0 there means a receiver CAN land even in the densest cluster.
%     - empirically place the full RX set with rejection on and count how many
%       got forced onto a building (hit the 50-attempt cap).
%
%   region_sel : region index or name. cfg defaults to buildConfigA().
%   n_mc       : Monte-Carlo sample count (default 20000).
%
%   Prints a summary and returns a struct with the numbers.

    if nargin < 2 || isempty(cfg),  cfg = buildConfigA(); end
    if nargin < 3 || isempty(n_mc), n_mc = 20000; end
    repo = fileparts(fileparts(mfilename('fullpath')));

    names = string({cfg.regions.name});
    if isnumeric(region_sel), ridx = region_sel; else, ridx = find(names==string(region_sel),1); end
    region = resolveRegion(cfg.regions(ridx), repo);

    gj = fullfile(repo, cfg.io.buildings_dir, [region.name '.geojson']);
    tester = buildingRejectMask(gj);

    % --- disc geometry (same math as placeReceivers) ----------------------
    clat = region.center_lat; clon = region.center_lon;
    R_m  = (region.placement_diameter_km/2) * 1000;
    mlat = 111320; mlon = 111320*cosd(clat);

    % --- Monte-Carlo coverage --------------------------------------------
    u = rand(n_mc,1); v = rand(n_mc,1);
    r = R_m*sqrt(u); th = 2*pi*v;
    lon = clon + (r.*cos(th))/mlon;
    lat = clat + (r.*sin(th))/mlat;
    inside = tester(lon, lat);
    coverage = mean(inside);
    openfrac = 1 - coverage;

    % --- densest local cell ----------------------------------------------
    G = 12;
    xe = linspace(-R_m, R_m, G+1); ye = xe;
    xm = (r.*cos(th)); ym = (r.*sin(th));   % metres from centre
    ix = discretize(xm, xe); iy = discretize(ym, ye);
    good = ~isnan(ix) & ~isnan(iy);
    cellcov = nan(G,G); celln = zeros(G,G);
    for a=1:G, for b=1:G
        m = good & ix==a & iy==b;
        celln(a,b) = nnz(m);
        if celln(a,b) >= 25, cellcov(a,b) = mean(inside(m)); end
    end, end
    dense_cov = max(cellcov(:), [], 'omitnan');

    % --- empirical placement (full counts, rejection on) ------------------
    rc = cfg.rx; rc.placement_diameter_km = region.placement_diameter_km;
    rng(cfg.flight.rng_seed + ridx);
    ws = warning('off','placeReceivers:buildingReject');
    [~, rxt] = placeReceivers(region, rc, tester);
    warning(ws);
    forced = sum(tester(rxt.lon, rxt.lat));   % placed RX that ended up on a building
    n_rx = height(rxt);

    info = struct('region',region.name,'coverage',coverage,'open_fraction',openfrac, ...
        'expected_attempts',1/max(openfrac,eps),'densest_cell_coverage',dense_cov, ...
        'n_rx',n_rx,'forced_onto_building',forced);

    fprintf(['%-18s  building coverage %.1f%%  | open %.1f%%  | ~%.1f attempts/RX\n' ...
             '                    densest 1/144 cell coverage %.1f%% (open %.1f%%)\n' ...
             '                    placed %d RX, %d forced onto a building\n'], ...
        region.name, 100*coverage, 100*openfrac, 1/max(openfrac,eps), ...
        100*dense_cov, 100*(1-dense_cov), n_rx, forced);
end
