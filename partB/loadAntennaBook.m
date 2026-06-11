function book = loadAntennaBook(path)
%LOADANTENNABOOK  Load all antenna gain patterns from one workbook (Part B).
%
%   book = loadAntennaBook(path)
%
%   Reads a single .xlsx/.csv where column 1 is the off-boresight Angle (deg,
%   0 = boresight) and every other column is one antenna's gain (dBi), with the
%   antenna NAME as the column header:
%
%       Angle | air_tx | air_rx | infantry_rx | ...
%         0   |  10.0  |   8.0  |     2.0     | ...
%        ...
%       180   |  ...
%
%   All antennas are azimuth-independent, so the single angle axis (folded to
%   |angle|) is sufficient. A column may be shorter than others (blank/NaN cells
%   beyond its range, e.g. an air antenna defined only to 90 deg) - those rows
%   are dropped per column and the gain is held (clamped) beyond its last angle.
%
%   Returns a 1xN struct array `book` (N = number of antenna columns), indexed
%   by ANTENNA INDEX. Each element has: name (column header), ang_deg, gain_dbi,
%   max_deg, interp. Query with gainFromPattern(book(idx), theta_deg); the user
%   picks tx/rx antenna by index per link (cfg.links(*).tx_ant_idx / rx_ant_idx).
%
%   R2021b-clean (readtable VariableNamingRule preserve, griddedInterpolant).

    if exist(path, 'file') ~= 2
        error('loadAntennaBook:notFound', 'Antenna workbook not found: %s', path);
    end
    T = readtable(path, 'VariableNamingRule', 'preserve');
    vn = T.Properties.VariableNames;
    if numel(vn) < 2
        error('loadAntennaBook:noAntennas', 'Need an Angle column + >=1 antenna column.');
    end

    ang_all = T{:, 1};
    nant = numel(vn) - 1;
    book = repmat(emptyPat(), 1, nant);
    for c = 2:numel(vn)
        g  = T{:, c};
        ok = ~isnan(ang_all) & ~isnan(g);
        book(c-1) = makePat(vn{c}, abs(ang_all(ok)), g(ok));
    end
end

% -------------------------------------------------------------------------
function p = emptyPat()
    p = struct('name','', 'ang_deg',[], 'gain_dbi',[], 'max_deg',[], 'interp',[]);
end

function p = makePat(name, ang, g)
    if isempty(ang)
        error('loadAntennaBook:emptyColumn', 'Antenna "%s" has no data.', name);
    end
    [ua, ~, ic] = unique(ang);
    gg = accumarray(ic, g, [], @max);        % fold symmetric duplicates
    p = emptyPat();
    p.name     = char(name);
    p.ang_deg  = ua(:);
    p.gain_dbi = gg(:);
    if numel(ua) == 1
        gc = gg(1);
        p.max_deg = 180;
        p.interp  = @(t) gc * ones(size(t));
    else
        p.max_deg = ua(end);
        p.interp  = griddedInterpolant(ua, gg, 'linear', 'nearest');
    end
end
