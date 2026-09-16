function S = buildSeries(R)
%BUILDSERIES  Split analyzeLinks output into plottable (link x terrain) series.
%
%   S = buildSeries(R)
%
%   R from analyzeLinks (one entry per link type). Produces one series per
%   (link type, terrain category), which is the unit the plots overlay and the
%   UI tick-boxes toggle.
%
%   S(i) fields: key, label, link, dir, ground, category, req_snr_db, mcs_index,
%                mcs_name, fingers, rate_bps,
%                dist_m, available, margin_db, pSNR_db   (per device in the series)
%
%   key   stable (link, terrain) identity - unaffected by MCS/finger choice - for
%         series selection (UI checkboxes, renderPartBAxes filtering).
%   label display text incl. rate (Mbps) - it can differ series to series once
%         an overlay mixes link types (e.g. the UI's free series selection), so
%         it's the legend DisplayName rather than a shared axes title.

    cats   = {'urban_suburban','other'};
    catlab = {'urban','open/mtn'};

    S = struct('key',{},'label',{},'link',{},'dir',{},'ground',{},'category',{}, ...
               'req_snr_db',{},'mcs_index',{},'mcs_name',{},'fingers',{},'rate_bps',{}, ...
               'dist_m',{},'available',{},'margin_db',{},'pSNR_db',{});
    for k = 1:numel(R)
        for c = 1:numel(cats)
            sel = R(k).category == cats{c};
            if ~any(sel), continue; end
            e.key        = sprintf('%s / %s', R(k).name, catlab{c});
            e.label      = sprintf('%s / %s (%.1f Mbps)', R(k).name, catlab{c}, R(k).rate_bps/1e6);
            e.link       = R(k).name;
            e.dir        = R(k).dir;
            e.ground     = R(k).ground;
            e.category   = cats{c};
            e.req_snr_db = R(k).req_snr_db;
            e.mcs_index  = R(k).mcs_index;
            e.mcs_name   = R(k).mcs_name;
            e.fingers    = R(k).fingers;
            e.rate_bps   = R(k).rate_bps;
            e.dist_m     = R(k).dist_m(sel);
            e.available  = R(k).available(sel);
            e.margin_db  = R(k).margin_db(sel);
            e.pSNR_db    = R(k).pSNR_db(sel);
            S(end+1) = e; %#ok<AGROW>
        end
    end
end
