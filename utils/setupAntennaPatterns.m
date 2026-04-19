function [tx_gain_func, rx_gain_func, n_rx_types] = setupAntennaPatterns()
%SETUPANTENNAPATTERNS  Isotropic antenna gain interpolation functions.
%
%   Returns interpolation function handles for:
%     - TX: isotropic 7 dBi
%     - RX: two types — type 1 = 2.5 dBi, type 2 = 7 dBi
%
%   [tx_gain_func, rx_gain_func, n_rx_types] = setupAntennaPatterns()
%
%   tx_gain_func(az, el)        → scalar/array gain in dBi
%   rx_gain_func(type_idx, az, el) → scalar/array gain in dBi
%   n_rx_types                  → 2

    % TX: uniform 7 dBi over all az/el
    tx_gain_dbi  = repmat(7, [2, 2]);
    tx_az        = linspace(0,    2*pi, size(tx_gain_dbi, 1));
    tx_el        = linspace(-pi,    pi, size(tx_gain_dbi, 2));
    [tx_X, tx_Y] = meshgrid(tx_az, tx_el);

    % RX: two types stacked in 3rd dimension [2x2x2]
    rx_gain_dbi  = repmat(shiftdim([2.5; 7], -2), [2, 2]);
    n_rx_types   = size(rx_gain_dbi, 3);
    rx_az        = linspace(0,    2*pi, size(rx_gain_dbi, 1));
    rx_el        = linspace(-pi,    pi, size(rx_gain_dbi, 2));
    [rx_X, rx_Y] = meshgrid(rx_az, rx_el);

    tx_gain_func = @(az, el) interp2(tx_X, tx_Y, tx_gain_dbi,           az, el, 'linear');
    rx_gain_func = @(idx, az, el) interp2(rx_X, rx_Y, rx_gain_dbi(:,:,idx), az, el, 'linear');
end
