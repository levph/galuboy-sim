function plotAvailabilityMap(availability_map, flight_lons, flight_lats, ...
                              lon_grid, lat_grid, config)
%PLOTAVAILABILITYMAP  Heatmap of signal availability over the simulation area.
%
%   plotAvailabilityMap(availability_map, flight_lons, flight_lats, ...
%                       lon_grid, lat_grid, config)
%
%   availability_map  [rows x cols] availability in percent, shaped to match
%                     lat_grid / lon_grid
%   flight_lons       [1 x Nf] flight path longitudes (reference rotation)
%   flight_lats       [1 x Nf] flight path latitudes  (reference rotation)
%   lon_grid          [rows x cols] longitude mesh (degrees)
%   lat_grid          [rows x cols] latitude mesh  (degrees)
%   config            struct from buildConfig()

    figure('Name', 'Signal Availability Map (mean over rotations)', ...
           'Position', [100, 100, 800, 600]);

    lon_space = lon_grid(1, :);
    lat_space = lat_grid(:, 1);

    imagesc(lon_space, lat_space, availability_map);
    set(gca, 'YDir', 'normal');
    colormap(parula);
    cb = colorbar;
    ylabel(cb, 'Mean Availability (%)');

    xlabel('Longitude (\circ)');
    ylabel('Latitude (\circ)');
    title(sprintf('Signal Availability — mean over %d trajectory rotations', ...
                  config.sim.num_rotations));

    hold on;
    plot(flight_lons, flight_lats, 'r-', 'LineWidth', 2, 'DisplayName', 'Flight Path (0\circ)');
    legend('Location', 'best');
    hold off;
    grid on;
end
