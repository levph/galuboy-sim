function slice = runFlightLoop(tx, rx_array, pm, flight_lons, flight_lats, config)
%RUNFLIGHTLOOP  Inner simulation loop — fly one trajectory and record raw data.
%
%   slice = runFlightLoop(tx, rx_array, pm, flight_lons, flight_lats, config)
%
%   Iterates over all flight steps for a single trajectory rotation,
%   computing path loss and geometry angles at each step.
%
%   tx            MATLAB txsite object (position updated each step)
%   rx_array      MATLAB rxsite array (n_rx receivers)
%   pm            propagationModel object
%   flight_lons   [1 x Nf] longitude waypoints (degrees)
%   flight_lats   [1 x Nf] latitude waypoints (degrees)
%   config        struct from buildConfig()
%
%   Returns slice — struct with fields [Nf x n_rx] (single precision):
%     path_loss_db   path loss in dB
%     azimuth_rad    TX→RX azimuth [0, 2π)
%     elevation_rad  TX→RX elevation (radians)

    Nf   = config.sim.num_flight_steps;
    n_rx = length(rx_array);

    slice.path_loss_db   = zeros(Nf, n_rx, 'single');
    slice.azimuth_rad    = zeros(Nf, n_rx, 'single');
    slice.elevation_rad  = zeros(Nf, n_rx, 'single');

    for i = 1:Nf
        tx.Longitude = flight_lons(i);
        tx.Latitude  = flight_lats(i);

        % Path loss via Longley-Rice
        slice.path_loss_db(i, :) = pathloss(pm, rx_array, tx);

        % Geometry angles (MATLAB returns degrees)
        [az_deg, el_deg] = angle(rx_array, tx);
        slice.azimuth_rad(i, :)   = mod(deg2rad(az_deg) + 2*pi, 2*pi);
        slice.elevation_rad(i, :) = deg2rad(el_deg);

        if mod(i, 10) == 0
            fprintf('    Step %d/%d\n', i, Nf);
        end
    end
end
