function config = buildConfigB()
%BUILDCONFIGB  Part B (offline analysis) configuration: full link-budget model.
%
%   config = buildConfigB()
%
%   Part B (R2021b+) turns Part A's raw path loss + steering angles into SNR and
%   p% availability per device, per LINK TYPE. There are 4 link types (direction
%   x ground device); each side's antenna gain is read from an azimuth-
%   independent pattern at the matching stored steering angle:
%     - air-side antenna  <- steer_air  (off the outward-tilted nadir boresight)
%     - ground-side antenna <- steer_gnd (off zenith)
%
%   Link budget (per device, per flight point), all dB / dBm:
%     EIRP          = Ptx_raw(txdev) - tx_rf_loss - papr_backoff(link) + Gtx(steer)
%     total_rx_gain = Grx(steer) - polarization_loss - mmse_loss
%     Prx           = EIRP - PL + total_rx_gain
%     N             = thermal(-174) + NF(rxdev) + 10*log10(BW(link))
%     SNR           = Prx - N
%   Then per device: pSNR = prctile(SNR, 100-percentile); available = pSNR >=
%   req_snr(link); margin = pSNR - req_snr.
%
%   MANY VALUES BELOW ARE PLACEHOLDERS ("given" inputs to be supplied / driven
%   by MCS): tx powers, losses, noise figures, per-link BW / papr_backoff /
%   req_snr, and the antenna pattern files. The structure / math is final.

    config = struct();

    % ---- Availability percentile (parameter; UI-overridable) -------------
    config.percentile = 99;          % pSNR = SNR exceeded p% of the trajectory

    % ---- Constants shared by all links (dB) -----------------------------
    config.const.thermal_dbm_hz      = -174;   % thermal noise density
    config.const.tx_rf_loss_db       = 2.0;    % GIVEN (same all links)
    config.const.polarization_loss_db= 3.0;    % GIVEN
    config.const.mmse_loss_db        = 2.0;    % GIVEN

    % ---- Per-device raw TX antenna power (dBm) ---------------------------
    config.tx_power_dbm.air       = 40;        % GIVEN (placeholder)
    config.tx_power_dbm.infantry  = 30;
    config.tx_power_dbm.vehicular = 33;

    % ---- Per-device receiver noise figure (dB) --------------------------
    config.noise_figure_db.air       = 3.0;    % GIVEN (placeholder)
    config.noise_figure_db.infantry  = 5.0;
    config.noise_figure_db.vehicular = 4.0;

    % ---- Antenna patterns: ONE workbook, one column per antenna ---------
    %   resources/antennas/antenna_patterns.xlsx
    %   col 1 = Angle (deg off boresight, 0=boresight); other cols = antennas
    %   (header = name). Links pick antennas by COLUMN INDEX (1-based among the
    %   antenna columns). Example column order:
    %     1 air_tx | 2 air_rx | 3 infantry_tx | 4 infantry_rx
    %     5 vehicular_tx | 6 vehicular_rx
    config.antenna_book = fullfile('resources','antennas','antenna_patterns.xlsx');

    % ---- Link definitions (4) -------------------------------------------
    % fields: name, dir, ground, tx_dev, rx_dev, tx_ant_idx, rx_ant_idx,
    %         tx_side('air'|'gnd'), rx_side, bw_hz, papr_db, req_snr_db
    % antenna indices reference columns of the workbook above; bw_hz / papr_db /
    % req_snr_db are GIVEN per link (MCS/BW driven) - placeholders.
    config.links = [ ...
        mkLink('DL_infantry','DL','infantry','air','infantry', 1, 4,'air','gnd', 10e6, 8, 5); ...
        mkLink('DL_vehicular','DL','vehicular','air','vehicular', 1, 6,'air','gnd', 20e6, 8, 5); ...
        mkLink('UL_infantry','UL','infantry','infantry','air', 3, 2,'gnd','air', 5e6, 6, 3); ...
        mkLink('UL_vehicular','UL','vehicular','vehicular','air', 5, 2,'gnd','air', 10e6, 6, 3) ];

    % ---- Plotting -------------------------------------------------------
    config.plot.dist_bin_m       = 100;  % availability / margin histogram bin (m)
    config.plot.hist_min_samples = 5;    % min devices/bin to plot that bin
    config.regions = defineRegions();  % for terrain-category aggregation

end

% -------------------------------------------------------------------------
function L = mkLink(name, dir, ground, tx_dev, rx_dev, tx_ant_idx, rx_ant_idx, tx_side, rx_side, bw_hz, papr_db, req_snr_db)
    L = struct('name',name,'dir',dir,'ground',ground, ...
               'tx_dev',tx_dev,'rx_dev',rx_dev,'tx_ant_idx',tx_ant_idx,'rx_ant_idx',rx_ant_idx, ...
               'tx_side',tx_side,'rx_side',rx_side, ...
               'bw_hz',bw_hz,'papr_db',papr_db,'req_snr_db',req_snr_db);
end
