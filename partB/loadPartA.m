function S = loadPartA(xlsx_path)
%LOADPARTA  Read a Part A workbook into per-device matrices (Part B entry).
%
%   S = loadPartA(xlsx_path)
%
%   Parses the 3-sheet Part A output (params / regions / devices) and expands
%   the comma-joined per-flight-point arrays back into [n_device x Nf] matrices.
%
%   Returns S with:
%     S.params   struct (name -> value string from the params sheet)
%     S.regions  table (the regions sheet)
%     S.Nf       number of flight points
%     S.device   struct of column vectors / matrices, all aligned by device:
%                  id, region (string), region_id, category (string),
%                  type (string: "infantry"|"vehicular"), height_m, lon, lat,
%                  dist_m  [n x 1]
%                  pl_db, steer_gnd_deg, steer_air_deg  [n x Nf]
%
%   R2021b-clean (readtable, split, str2double).

    if exist(xlsx_path, 'file') ~= 2
        error('loadPartA:notFound', 'Part A workbook not found: %s', xlsx_path);
    end

    P = readtable(xlsx_path, 'Sheet', 'params',  'TextType', 'string');
    R = readtable(xlsx_path, 'Sheet', 'regions', 'TextType', 'string');
    D = readtable(xlsx_path, 'Sheet', 'devices', 'TextType', 'string');

    params = struct();
    for i = 1:height(P)
        params.(matlab.lang.makeValidName(P.name(i))) = P.value(i);
    end

    pl   = parseArrays(D.pl_db);
    sgnd = parseArrays(D.steer_gnd_deg);
    sair = parseArrays(D.steer_air_deg);

    % map each device's region_id -> terrain category from the regions sheet
    [~, loc] = ismember(D.region_id, R.region_id);
    category = R.category(loc);

    dev = struct();
    dev.id        = D.device_id;
    dev.region    = D.region;
    dev.region_id = D.region_id;
    dev.category  = category;
    dev.type      = D.type;
    dev.height_m  = D.height_m;
    dev.lon       = D.lon;
    dev.lat       = D.lat;
    dev.dist_m    = D.dist_m;
    dev.pl_db        = pl;
    dev.steer_gnd_deg = sgnd;
    dev.steer_air_deg = sair;

    S = struct('params', params, 'regions', R, 'Nf', size(pl, 2), 'device', dev);
end

% -------------------------------------------------------------------------
function M = parseArrays(strs)
%PARSEARRAYS  n x 1 comma-joined strings -> [n x Nf] numeric (Inf preserved).
    n  = numel(strs);
    Nf = numel(split(strs(1), ","));
    M  = zeros(n, Nf);
    for i = 1:n
        v = str2double(split(strs(i), ","));
        M(i, :) = v(:).';
    end
end
