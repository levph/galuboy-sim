function pat = loadAntennaPattern(antenna_name, antennas_dir)
%LOADANTENNAPATTERN  Load an az/el gain table from a CSV in resources/antennas.
%
%   pat = loadAntennaPattern(antenna_name, antennas_dir)
%
%   Returns a struct:
%     pat.name           char
%     pat.kind           char  'isotropic' | '1d' | '2d'
%     pat.is_isotropic   logical  (back-compat alias for pat.kind == 'isotropic')
%     pat.flat_gain_dbi  numeric  (only for isotropic patterns; NaN otherwise)
%     pat.gain_func      handle:  G_dbi = pat.gain_func(az_off_rad, el_off_rad)
%     pat.az_grid_rad    [] for isotropic, [] for 1D, [1 x n_az] vector for 2D
%     pat.el_grid_rad    [] for isotropic, [1 x n_el] vector for 1D and 2D
%     pat.gain_grid_dbi  [] for isotropic, [n_el x 1] for 1D, [n_az x n_el] for 2D
%
%   Conventions:
%     - Inputs to gain_func are RADIANS, offsets from boresight.
%     - CSV columns az_deg, el_deg are DEGREES on the same offset frame.
%     - 2D grids must be RECTANGULAR (every az repeats the same el list).
%     - Out-of-range queries clamp to the nearest grid point - no NaN.
%
%   Three CSV formats supported:
%
%   1. Isotropic sentinel (single non-comment line):
%
%        isotropic,7.0
%
%   2. 1D elevation-only table (azimuth-independent):
%
%        el_deg,gain_dbi
%        -90,-3
%        -60, 0
%        ...
%
%   3. 2D regular grid:
%
%        az_deg,el_deg,gain_dbi
%        0,-90,2.5
%        0,-60,3.5
%        ...
%
%   Comment lines start with '#'.

    arguments
        antenna_name (1,:) char
        antennas_dir (1,:) char
    end

    csv_path = fullfile(antennas_dir, [antenna_name, '.csv']);
    if ~isfile(csv_path)
        error('loadAntennaPattern:notFound', ...
              ['Antenna pattern file not found: %s\n' ...
               'Drop a CSV under resources/antennas/<antenna_name>.csv ' ...
               'or update config to use an existing one.'], csv_path);
    end

    raw_lines  = readlines(csv_path);
    data_lines = stripCommentsAndBlanks(raw_lines);
    if isempty(data_lines)
        error('loadAntennaPattern:emptyFile', ...
              'Antenna pattern CSV has no data lines: %s', csv_path);
    end

    pat.name           = antenna_name;
    pat.az_grid_rad    = [];
    pat.el_grid_rad    = [];
    pat.gain_grid_dbi  = [];

    % --- Isotropic sentinel ----------------------------------------------
    first = strtrim(data_lines{1});
    if startsWith(lower(first), 'isotropic')
        toks = strsplit(first, ',');
        if numel(toks) < 2
            error('loadAntennaPattern:badIsotropic', ...
                  'Isotropic line must be "isotropic,<gain_dbi>": %s', csv_path);
        end
        gain_dbi = str2double(strtrim(toks{2}));
        if ~isfinite(gain_dbi)
            error('loadAntennaPattern:badIsotropic', ...
                  'Isotropic gain not numeric in %s', csv_path);
        end
        pat.kind          = 'isotropic';
        pat.is_isotropic  = true;
        pat.flat_gain_dbi = gain_dbi;
        pat.gain_func     = @(az, el) gain_dbi * ones(size(az));
        return;
    end

    header = strsplit(lower(strtrim(data_lines{1})), ',');

    if isequal(header, {'el_deg', 'gain_dbi'})
        pat = readOneDim(pat, data_lines, csv_path);
        return;
    elseif isequal(header, {'az_deg', 'el_deg', 'gain_dbi'})
        pat = readTwoDim(pat, data_lines, csv_path);
        return;
    else
        error('loadAntennaPattern:badHeader', ...
              ['Expected one of:\n' ...
               '  "isotropic,<gain>"\n' ...
               '  "el_deg,gain_dbi"        (1D, azimuth-independent)\n' ...
               '  "az_deg,el_deg,gain_dbi" (2D regular grid)\n' ...
               'Got "%s" in %s.'], data_lines{1}, csv_path);
    end
end


% -------------------------------------------------------------------------
function pat = readOneDim(pat, data_lines, csv_path)
    body = data_lines(2:end);
    n    = numel(body);
    if n < 2
        error('loadAntennaPattern:tooFewRows', ...
              'Pattern %s has only %d data rows; need >=2.', csv_path, n);
    end

    el_deg   = zeros(n, 1);
    gain_dbi = zeros(n, 1);
    for k = 1:n
        toks = strsplit(strtrim(body{k}), ',');
        if numel(toks) < 2
            error('loadAntennaPattern:badRow', ...
                  'Row %d malformed in %s: "%s"', k+1, csv_path, body{k});
        end
        el_deg(k)   = str2double(toks{1});
        gain_dbi(k) = str2double(toks{2});
    end
    if any(~isfinite(el_deg)) || any(~isfinite(gain_dbi))
        error('loadAntennaPattern:nonNumeric', ...
              'Non-numeric value in %s', csv_path);
    end

    [el_sorted, idx] = sort(el_deg);
    g_sorted = gain_dbi(idx);
    if numel(unique(el_sorted)) ~= numel(el_sorted)
        error('loadAntennaPattern:duplicateEl', ...
              'Duplicate el_deg values in %s', csv_path);
    end

    el_grid_rad = deg2rad(el_sorted);
    F = griddedInterpolant(el_grid_rad, g_sorted, 'linear', 'nearest');

    pat.kind          = '1d';
    pat.is_isotropic  = false;
    pat.flat_gain_dbi = NaN;
    pat.el_grid_rad   = el_grid_rad(:).';
    pat.gain_grid_dbi = g_sorted(:);
    pat.gain_func     = @(az, el) F(el);
end


% -------------------------------------------------------------------------
function pat = readTwoDim(pat, data_lines, csv_path)
    body = data_lines(2:end);
    n    = numel(body);
    if n < 4
        error('loadAntennaPattern:tooFewRows', ...
              'Pattern %s has only %d data rows; need >=4 for a 2x2 grid.', ...
              csv_path, n);
    end

    az_deg   = zeros(n, 1);
    el_deg   = zeros(n, 1);
    gain_dbi = zeros(n, 1);
    for k = 1:n
        toks = strsplit(strtrim(body{k}), ',');
        if numel(toks) < 3
            error('loadAntennaPattern:badRow', ...
                  'Row %d malformed in %s: "%s"', k+1, csv_path, body{k});
        end
        az_deg(k)   = str2double(toks{1});
        el_deg(k)   = str2double(toks{2});
        gain_dbi(k) = str2double(toks{3});
    end
    if any(~isfinite(az_deg)) || any(~isfinite(el_deg)) || any(~isfinite(gain_dbi))
        error('loadAntennaPattern:nonNumeric', ...
              'Non-numeric value in %s', csv_path);
    end

    az_grid = unique(az_deg);
    el_grid = unique(el_deg);
    if numel(az_grid) * numel(el_grid) ~= n
        error('loadAntennaPattern:nonRectangular', ...
              ['Pattern grid in %s is not rectangular: %d unique az x %d unique el ' ...
               '!= %d rows.'], csv_path, numel(az_grid), numel(el_grid), n);
    end

    % Reshape into [n_az x n_el]. Sort rows by (az, el) lexicographic order
    % so reshape gives the right layout regardless of file ordering.
    [~, idx] = sortrows([az_deg, el_deg]);
    G = reshape(gain_dbi(idx), numel(el_grid), numel(az_grid)).';   % [n_az x n_el]

    az_grid_rad = deg2rad(az_grid);
    el_grid_rad = deg2rad(el_grid);

    % Make the az axis periodic by replicating the az=0 row at az=2*pi
    % so queries near the wrap point interpolate continuously instead of
    % falling outside the grid and getting 'nearest'-clamped.
    if abs(az_grid_rad(end) - 2*pi) > 1e-9
        az_grid_rad(end+1) = 2*pi;
        G(end+1, :)        = G(1, :);
    end

    F = griddedInterpolant({az_grid_rad, el_grid_rad}, G, 'linear', 'nearest');

    pat.kind          = '2d';
    pat.is_isotropic  = false;
    pat.flat_gain_dbi = NaN;
    pat.az_grid_rad   = az_grid_rad(:).';
    pat.el_grid_rad   = el_grid_rad(:).';
    pat.gain_grid_dbi = G;
    pat.gain_func     = @(az, el) F(az, el);
end


% -------------------------------------------------------------------------
function out = stripCommentsAndBlanks(lines)
    out = {};
    for k = 1:numel(lines)
        s = strtrim(char(lines(k)));
        if isempty(s) || startsWith(s, '#')
            continue;
        end
        out{end+1} = s; %#ok<AGROW>
    end
end
