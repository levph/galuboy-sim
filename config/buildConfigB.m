function config = buildConfigB()
%BUILDCONFIGB  Part B (offline analysis) configuration: full link-budget model.
%
%   config = buildConfigB()
%
%   Part B (R2021b+) turns Part A's raw path loss + steering angles into SNR and
%   p% availability per device, per LINK TYPE. There are 6 link types (direction
%   x ground device, plus a reduced-power infantry variant); each side's
%   antenna gain is read from an azimuth-
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
%   Each link picks an MCS (by index, 1-11) and a finger count instead of
%   entering BW / IBO(PAPR backoff) / required-SNR directly:
%     - IBO and required-SNR come from the MCS lookup table for the link's
%       DIRECTION (config.mcs_book, sheet 'DL' or 'UL') - same across ground
%       device types (infantry/vehicular), different across DL/UL.
%     - BW and rate scale with finger count off a per-direction organic
%       (1-finger) baseline: BW(link) = fingers * bw_per_finger_hz(dir);
%       rate(link) = fingers * rate_1finger(mcs, dir). See resolveLinkParams.
%
%   MANY VALUES BELOW ARE PLACEHOLDERS ("given" inputs, made up pending real
%   figures): tx powers, losses, noise figures, per-finger BW, the MCS table,
%   and the antenna pattern files. The structure / math is final.

    config = struct();

    % ---- Availability percentile (parameter; UI-overridable) -------------
    config.percentile = 99;          % pSNR = SNR exceeded p% of the trajectory

    % ---- Constants shared by all links (dB) -----------------------------
    config.const.thermal_dbm_hz      = -174;   % thermal noise density
    config.const.tx_rf_loss_db       = 2.0;    % GIVEN (same all links)
    config.const.polarization_loss_db= 3.0;    % GIVEN
    config.const.mmse_loss_db        = 2.0;    % GIVEN

    % ---- Per-device raw TX antenna power (dBm) ---------------------------
    config.tx_power_dbm.air              = 40;   % GIVEN (placeholder)
    config.tx_power_dbm.infantry         = 30;
    config.tx_power_dbm.vehicular        = 33;
    config.tx_power_dbm.infantry_reduced = 23;   % GIVEN (placeholder, -7 dB vs infantry)

    % ---- Per-device receiver noise figure (dB) --------------------------
    config.noise_figure_db.air       = 3.0;    % GIVEN (placeholder)
    config.noise_figure_db.infantry  = 5.0;
    config.noise_figure_db.vehicular = 4.0;

    % ---- Antenna patterns: ONE workbook, one column per antenna ---------
    %   resources/antennas/antenna_patterns.xlsx
    %   col 1 = Angle (deg off boresight, 0=boresight); other cols = antennas
    %   (header = name). Links pick antennas by NAME (the column header),
    %   resolved to a column index by resolveLinkParams. Committed columns:
    %     air_tx | air_rx | infantry_tx | infantry_rx | vehicular_tx | vehicular_rx
    config.antenna_book = fullfile('resources','antennas','antenna_patterns.xlsx');

    % ---- MCS lookup table: one workbook, sheet 'DL' + sheet 'UL' --------
    %   resources/mcs/mcs_tables.xlsx (see resources/README.md).
    %   Columns: mcs_index, mcs, ibo_db, req_snr_db, rate_1finger_mbps.
    %   Same MCS table across ground device types; separate sheet per direction.
    config.mcs_book = fullfile('resources','mcs','mcs_tables.xlsx');

    % ---- Organic (1-finger) bandwidth per direction (Hz) -----------------
    %   BW(link) = fingers(link) * bw_per_finger_hz.(link.dir). GIVEN - placeholder.
    config.finger.bw_per_finger_hz.DL = 10e6;
    config.finger.bw_per_finger_hz.UL = 5e6;

    % ---- Link definitions (6) -------------------------------------------
    % fields: name, dir, ground, tx_dev, rx_dev, tx_ant, rx_ant,
    %         tx_side('air'|'gnd'), rx_side, mcs_index(1-11), fingers
    % tx_ant/rx_ant are antenna NAMES (workbook column headers). mcs_index +
    % fingers select IBO/req_snr/BW/rate via resolveLinkParams (config.mcs_book,
    % config.finger). fingers are GIVEN per link (placeholder: infantry=1,
    % vehicular=2, i.e. vehicular gets double the organic BW/rate).
    %
    % The two *_infantry_reduced links model a weaker infantry radio: same
    % ground='infantry' (reuses the regular infantry RX samples from Part A),
    % same mcs_index/fingers as the corresponding infantry link, but UL
    % transmits at the reduced infantry_reduced TX power instead of infantry's.
    % Antennas default to the same names as the regular infantry links; pick
    % the actual reduced antenna per-link in the galuboyAnalysisApp UI.
    config.links = [ ...
        mkLink('DL_infantry','DL','infantry','air','infantry', 'air_tx','infantry_rx','air','gnd', 6, 1); ...
        mkLink('DL_vehicular','DL','vehicular','air','vehicular', 'air_tx','vehicular_rx','air','gnd', 8, 2); ...
        mkLink('DL_infantry_reduced','DL','infantry','air','infantry', 'air_tx','infantry_rx','air','gnd', 6, 1); ...
        mkLink('UL_infantry','UL','infantry','infantry','air', 'infantry_tx','air_rx','gnd','air', 4, 1); ...
        mkLink('UL_vehicular','UL','vehicular','vehicular','air', 'vehicular_tx','air_rx','gnd','air', 6, 2); ...
        mkLink('UL_infantry_reduced','UL','infantry','infantry_reduced','air', 'infantry_tx','air_rx','gnd','air', 4, 1) ];

    % ---- Plotting -------------------------------------------------------
    config.plot.dist_bin_m       = 100;  % availability / margin histogram bin (m)
    config.plot.hist_min_samples = 5;    % min devices/bin to plot that bin
    config.regions = defineRegions();  % for terrain-category aggregation

end

% -------------------------------------------------------------------------
function L = mkLink(name, dir, ground, tx_dev, rx_dev, tx_ant, rx_ant, tx_side, rx_side, mcs_index, fingers)
    L = struct('name',name,'dir',dir,'ground',ground, ...
               'tx_dev',tx_dev,'rx_dev',rx_dev,'tx_ant',tx_ant,'rx_ant',rx_ant, ...
               'tx_side',tx_side,'rx_side',rx_side, ...
               'mcs_index',mcs_index,'fingers',fingers);
end
