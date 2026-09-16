function L = resolveLinkParams(link, mcsbook, finger, book)
%RESOLVELINKPARAMS  Resolve one link's MCS/finger/antenna-name choices (Part B).
%
%   L = resolveLinkParams(link, mcsbook, finger, book)
%
%   link     one element of cfg.links (buildConfigB): carries mcs_index,
%            fingers, tx_ant/rx_ant (antenna NAMES), dir, plus the fields
%            computeLinkSNR/computeLinkAvailability need unchanged.
%   mcsbook  from loadMcsBook (fields .DL, .UL).
%   finger   cfg.finger (bw_per_finger_hz.DL/.UL).
%   book     antenna pattern array from loadAntennaBook (indexed by name via
%            book(i).name).
%
%   Returns link with tx_ant_idx/rx_ant_idx/bw_hz/papr_db/req_snr_db added (the
%   fields computeLinkSNR/computeLinkAvailability consume), plus mcs_name and
%   rate_bps for display.

    T   = mcsbook.(link.dir);
    row = T.mcs_index == link.mcs_index;
    if ~any(row)
        error('resolveLinkParams:mcs', 'MCS index %d not found in %s sheet.', ...
            link.mcs_index, link.dir);
    end

    L = link;
    L.papr_db     = T.ibo_db(row);
    L.req_snr_db  = T.req_snr_db(row);
    L.mcs_name    = char(T.mcs(row));
    L.bw_hz       = link.fingers * finger.bw_per_finger_hz.(link.dir);
    L.rate_bps    = link.fingers * T.rate_1finger_mbps(row) * 1e6;
    L.tx_ant_idx  = antIndexByName(book, link.tx_ant);
    L.rx_ant_idx  = antIndexByName(book, link.rx_ant);
end

% -------------------------------------------------------------------------
function idx = antIndexByName(book, name)
    idx = find(strcmp({book.name}, name), 1);
    if isempty(idx)
        error('resolveLinkParams:antenna', 'Antenna "%s" not found in antenna workbook.', name);
    end
end
