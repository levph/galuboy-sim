function S = buildSeries(R)
%BUILDSERIES  Split analyzeLinks output into plottable (link x terrain) series.
%
%   S = buildSeries(R)
%
%   R from analyzeLinks (one entry per link type). Produces one series per
%   (link type, terrain category), which is the unit the plots overlay and the
%   UI tick-boxes toggle.
%
%   S(i) fields: label, link, dir, ground, category, req_snr_db,
%                dist_m, available, margin_db, pSNR_db   (per device in the series)

    cats   = {'urban_suburban','other'};
    catlab = {'urban','open/mtn'};

    S = struct('label',{},'link',{},'dir',{},'ground',{},'category',{}, ...
               'req_snr_db',{},'dist_m',{},'available',{},'margin_db',{},'pSNR_db',{});
    for k = 1:numel(R)
        for c = 1:numel(cats)
            sel = R(k).category == cats{c};
            if ~any(sel), continue; end
            e.label      = sprintf('%s / %s', R(k).name, catlab{c});
            e.link       = R(k).name;
            e.dir        = R(k).dir;
            e.ground     = R(k).ground;
            e.category   = cats{c};
            e.req_snr_db = R(k).req_snr_db;
            e.dist_m     = R(k).dist_m(sel);
            e.available  = R(k).available(sel);
            e.margin_db  = R(k).margin_db(sel);
            e.pSNR_db    = R(k).pSNR_db(sel);
            S(end+1) = e; %#ok<AGROW>
        end
    end
end
