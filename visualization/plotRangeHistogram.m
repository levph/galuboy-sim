function plotRangeHistogram(all_distances, all_available, config)
%PLOTRANGEHISTOGRAM  Bar chart of link availability vs distance from centroid.
%
%   plotRangeHistogram(all_distances, all_available, config)
%
%   all_distances  [N x 1] ground distances from trajectory centroid (m)
%   all_available  [N x 1] logical — true if link available for that sample
%   config         struct from buildConfig() — uses hist.n_bins,
%                  hist.min_samples_per_bin, link.percentile,
%                  tx.power_dbm, sim.availability_threshold_dbm,
%                  prop.fade_margin_db, sim.num_rotations

    n_bins = config.hist.n_bins;

    dist_edges  = linspace(min(all_distances), max(all_distances), n_bins + 1);
    n_total     = histcounts(all_distances,                    dist_edges);
    n_available = histcounts(all_distances(all_available),     dist_edges);

    frac_available = n_available ./ n_total;
    insufficient   = n_total < config.hist.min_samples_per_bin;
    frac_available(insufficient) = NaN;

    bin_centers = 0.5 * (dist_edges(1:end-1) + dist_edges(2:end));

    fig = figure('Name', 'Range Analysis — Availability vs Distance', ...
                 'Position', [100, 100, 900, 550]);
    ax  = axes('Parent', fig);

    b = bar(ax, bin_centers, frac_available * 100, 1.0);
    b.FaceColor = [0.20, 0.45, 0.75];
    b.EdgeColor = 'none';
    b.FaceAlpha = 0.85;

    hold(ax, 'on');
    yline(ax, 50, '--r', 'LineWidth', 1.5, ...
          'Label', '50% availability', 'LabelHorizontalAlignment', 'left');

    for k = 1:numel(bin_centers)
        if ~isnan(frac_available(k))
            text(ax, bin_centers(k), frac_available(k) * 100 + 1.5, ...
                 sprintf('n=%d', n_total(k)), ...
                 'HorizontalAlignment', 'center', 'FontSize', 7, 'Color', [0.3 0.3 0.3]);
        end
    end
    hold(ax, 'off');

    xlabel(ax, 'Distance from Trajectory Centroid (m)', 'FontSize', 12);
    ylabel(ax, sprintf('Availability (%%)\n[P_{rx}(p%d) > MDS]', config.link.percentile), ...
           'FontSize', 12);
    title(ax, sprintf(['Aerial Tx Range Analysis\n' ...
           'P_{tx}=%.0f dBm | MDS=%.0f dBm | Fade margin=%.0f dB | ' ...
           '%d-th pct | M=%d rotations'], ...
           config.tx.power_dbm, config.sim.availability_threshold_dbm, ...
           config.prop.fade_margin_db, config.link.percentile, ...
           config.sim.num_rotations), 'FontSize', 11);

    ylim(ax, [0, 110]);
    xlim(ax, [dist_edges(1), dist_edges(end)]);
    grid(ax, 'on');
    ax.GridAlpha = 0.3;
    ax.Box       = 'off';
end
