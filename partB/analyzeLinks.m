function R = analyzeLinks(S, cfg, book)
%ANALYZELINKS  Per-link SNR + availability for all link types (Part B).
%
%   R = analyzeLinks(S, cfg, book)
%
%   For each link in cfg.links: select the devices of that link's ground type,
%   run the link budget (computeLinkSNR) and the p% availability
%   (computeLinkAvailability), and collect per-device results for plotting.
%
%   S     from loadPartA. cfg from buildConfigB. book from loadAntennaBook.
%
%   Returns a struct array R (one per link) with fields:
%     name, dir, ground, req_snr_db,
%     dist_m, category, pSNR_db, available, margin_db   (per selected device)

    R = repmat(struct('name','','dir','','ground','','req_snr_db',[], ...
        'dist_m',[],'category',strings(0,1),'pSNR_db',[],'available',[],'margin_db',[]), ...
        1, numel(cfg.links));

    for k = 1:numel(cfg.links)
        L   = cfg.links(k);
        sel = S.device.type == string(L.ground);

        dev.pl_db         = S.device.pl_db(sel, :);
        dev.steer_air_deg = S.device.steer_air_deg(sel, :);
        dev.steer_gnd_deg = S.device.steer_gnd_deg(sel, :);

        SNR = computeLinkSNR(dev, L, cfg, book);
        av  = computeLinkAvailability(SNR, L.req_snr_db, cfg.percentile);

        R(k).name       = L.name;
        R(k).dir        = L.dir;
        R(k).ground     = L.ground;
        R(k).req_snr_db = L.req_snr_db;
        R(k).dist_m     = S.device.dist_m(sel);
        R(k).category   = S.device.category(sel);
        R(k).pSNR_db    = av.pSNR_db;
        R(k).available  = av.available;
        R(k).margin_db  = av.margin_db;
    end
end
