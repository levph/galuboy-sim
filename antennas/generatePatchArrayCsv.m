function generatePatchArrayCsv(csv_path, N, opts)
%GENERATEPATCHARRAYCSV  Write a 2D (az, el) gain CSV for a steered patch array.
%
%   generatePatchArrayCsv(csv_path, N)
%   generatePatchArrayCsv(csv_path, N, opts)
%
%   Models the peak main-lobe gain of an N-element steered patch array as
%   a smoothed rectangular window over offset from boresight: flat at
%   G_max = 10*log10(N) inside the FOV, raised-cosine taper at the edge,
%   low floor outside. The same window function is applied independently
%   to both the azimuth and elevation offset from broadside, then combined
%   with a min() so that the lobe is contained in the |az|<=fov && |el|<=fov
%   cone with smoothly tapered edges.
%
%   csv_path  destination .csv path
%   N         element count (8 for ground patch, 16 for airborne patch)
%   opts      struct (all optional):
%     .fov_in_deg   half-width of the FLAT region (deg, default 50)
%     .fov_out_deg  half-width where the gain reaches the floor (deg, default 70)
%     .floor_db     out-of-FOV floor in dB (default -20)
%     .n_az         number of az samples (default 73, 5-deg step from -180..180)
%     .n_el         number of el samples (default 37, 5-deg step from  -90..90)
%
%   The CSV follows the loadAntennaPattern 2D grid format:
%
%       az_deg,el_deg,gain_dbi
%       -180,-90,-20
%       ...
%
%   The peak gain is referenced to a single element (i.e. 10*log10(N)
%   coherent array gain over an isotropic element). This is a coarse
%   model — the user will replace these CSVs with measured/EM-simulated
%   patterns later — but it gives the link budget the right shape.

    arguments
        csv_path (1,:) char
        N (1,1) double {mustBePositive, mustBeInteger}
        opts.fov_in_deg  (1,1) double = 50
        opts.fov_out_deg (1,1) double = 70
        opts.floor_db    (1,1) double = -20
        opts.n_az        (1,1) double {mustBePositive, mustBeInteger} = 73
        opts.n_el        (1,1) double {mustBePositive, mustBeInteger} = 37
    end

    if opts.fov_out_deg <= opts.fov_in_deg
        error('generatePatchArrayCsv:badFov', ...
              'fov_out_deg (%.1f) must be > fov_in_deg (%.1f).', ...
              opts.fov_out_deg, opts.fov_in_deg);
    end

    G_max = 10 * log10(N);

    az_deg = linspace(-180, 180, opts.n_az);
    el_deg = linspace( -90,  90, opts.n_el);

    [AZ, EL] = ndgrid(az_deg, el_deg);

    g_az = patchWindow(AZ, opts.fov_in_deg, opts.fov_out_deg, G_max, opts.floor_db);
    g_el = patchWindow(EL, opts.fov_in_deg, opts.fov_out_deg, G_max, opts.floor_db);
    % Combine: lobe is the AND of "in-FOV in az" AND "in-FOV in el".
    G = min(g_az, g_el);

    [out_dir, ~, ~] = fileparts(csv_path);
    if ~isempty(out_dir) && ~exist(out_dir, 'dir'), mkdir(out_dir); end

    fid = fopen(csv_path, 'w');
    if fid < 0
        error('generatePatchArrayCsv:openFailed', 'Cannot write %s', csv_path);
    end
    cleanup = onCleanup(@() fclose(fid));

    fprintf(fid, '# galuboy patch-array antenna pattern\n');
    fprintf(fid, '# N=%d  fov_in=%.1f  fov_out=%.1f  floor=%.1f dB  G_max=%.2f dBi\n', ...
            N, opts.fov_in_deg, opts.fov_out_deg, opts.floor_db, G_max);
    fprintf(fid, '# az/el are offsets from boresight in degrees.\n');
    fprintf(fid, 'az_deg,el_deg,gain_dbi\n');
    for i = 1:numel(az_deg)
        for j = 1:numel(el_deg)
            fprintf(fid, '%g,%g,%.3f\n', az_deg(i), el_deg(j), G(i, j));
        end
    end
end


% -------------------------------------------------------------------------
function g = patchWindow(theta_deg, fov_in_deg, fov_out_deg, G_max, floor_db)
%PATCHWINDOW  Smoothed rectangular window over offset from broadside.
    abs_th = abs(theta_deg);
    taper  = fov_out_deg - fov_in_deg;

    g = floor_db * ones(size(theta_deg));

    mask_flat = abs_th <= fov_in_deg;
    g(mask_flat) = G_max;

    mask_taper = (abs_th > fov_in_deg) & (abs_th <= fov_out_deg);
    t = (abs_th(mask_taper) - fov_in_deg) / taper;
    g(mask_taper) = floor_db + (G_max - floor_db) * 0.5 .* (1 + cos(pi * t));
end
