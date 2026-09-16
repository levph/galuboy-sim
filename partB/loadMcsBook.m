function mcsbook = loadMcsBook(path)
%LOADMCSBOOK  Load the MCS -> IBO/required-SNR/rate lookup tables (Part B).
%
%   mcsbook = loadMcsBook(path)
%
%   Reads a workbook with one sheet per direction, 'DL' and 'UL' (same MCS set
%   across ground device types; IBO/required-SNR/rate differ by direction):
%
%       mcs_index | mcs        | ibo_db | req_snr_db | rate_1finger_mbps
%           1      | BPSK 1/6  |  1.0   |    -2      |       1.67
%          ...
%
%   rate_1finger_mbps is the organic rate at 1 finger on that direction's
%   organic bandwidth; resolveLinkParams multiplies both by finger count.
%
%   Returns mcsbook with fields .DL, .UL, each the sheet's table indexed by
%   mcs_index (resolveLinkParams looks up a link's mcs_index/dir here).
%
%   R2021b-clean (readtable, 'Sheet').

    if exist(path, 'file') ~= 2
        error('loadMcsBook:notFound', 'MCS workbook not found: %s', path);
    end

    mcsbook = struct();
    for d = ["DL","UL"]
        T = readtable(path, 'Sheet', d, 'VariableNamingRule', 'preserve', 'TextType', 'string');
        req = {'mcs_index','mcs','ibo_db','req_snr_db','rate_1finger_mbps'};
        if ~all(ismember(req, T.Properties.VariableNames))
            error('loadMcsBook:badSheet', 'Sheet "%s" missing required columns.', d);
        end
        mcsbook.(d) = T;
    end
end
