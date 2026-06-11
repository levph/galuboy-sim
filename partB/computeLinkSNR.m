function SNR = computeLinkSNR(dev, link, cfg, book)
%COMPUTELINKSNR  Per-(device, flight-point) SNR for one link type (Part B).
%
%   SNR = computeLinkSNR(dev, link, cfg, book)
%
%   dev   struct of [n x Nf] matrices for the devices on this link:
%           .pl_db, .steer_air_deg, .steer_gnd_deg
%         (caller filters devices to link.ground type first).
%   link  one element of cfg.links (buildConfigB).
%   cfg   buildConfigB() config (powers, losses, noise figures).
%   book  antenna pattern array from loadAntennaBook; the link selects antennas
%         by index: book(link.tx_ant_idx), book(link.rx_ant_idx).
%
%   Link budget (dB / dBm), elementwise over [n x Nf]:
%     EIRP = Ptx(txdev) - tx_rf_loss - papr(link) + Gtx(steer_tx)
%     Prx  = EIRP - PL + [Grx(steer_rx) - pol_loss - mmse_loss]
%     N    = -174 + NF(rxdev) + 10log10(BW)
%     SNR  = Prx - N
%   PL = Inf (outage) -> SNR = -Inf.

    tx_steer = dev.(steerField(link.tx_side));
    rx_steer = dev.(steerField(link.rx_side));

    Gtx = gainFromPattern(book(link.tx_ant_idx), tx_steer);
    Grx = gainFromPattern(book(link.rx_ant_idx), rx_steer);

    eirp = cfg.tx_power_dbm.(link.tx_dev) - cfg.const.tx_rf_loss_db ...
         - link.papr_db + Gtx;
    total_rx_gain = Grx - cfg.const.polarization_loss_db - cfg.const.mmse_loss_db;

    Prx = eirp - dev.pl_db + total_rx_gain;

    N = cfg.const.thermal_dbm_hz + cfg.noise_figure_db.(link.rx_dev) ...
      + 10*log10(link.bw_hz);

    SNR = Prx - N;
end

% -------------------------------------------------------------------------
function f = steerField(side)
    switch lower(side)
        case 'air', f = 'steer_air_deg';
        case 'gnd', f = 'steer_gnd_deg';
        otherwise,  error('computeLinkSNR:side', 'side must be air|gnd, got %s', side);
    end
end
