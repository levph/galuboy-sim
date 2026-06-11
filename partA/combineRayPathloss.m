function pl_db = combineRayPathloss(rays)
%COMBINERAYPATHLOSS  Coherently combine raytrace rays into one path loss per link.
%
%   pl_db = combineRayPathloss(rays)
%
%   raytrace() returns a cell array - one cell per TX-RX link, each holding a
%   comm.Ray array (the rays found for that link). This collapses each cell to a
%   single effective path loss by COHERENT combination of the rays' complex
%   field amplitudes (see coherentCombineDb): per-ray PathLoss (dB) and
%   PhaseShift (rad) are summed as voltages, so multipath interference -
%   including destructive cancellation - is captured.
%
%   A link with no rays (full blockage) collapses to Inf (outage), which Part B
%   maps to SNR -Inf (unavailable).
%
%   NOTE: requires raytrace() output (comm.Ray, which carries PhaseShift), NOT
%   pathloss() output (per-ray dB only, no phase).
%
%   Input  rays  : cell array (any shape) of comm.Ray arrays (or struct arrays
%                  exposing PathLoss / PhaseShift fields).
%   Output pl_db : double array, same shape, one combined dB per link.

    pl_db = inf(size(rays));
    for k = 1:numel(rays)
        rk = rays{k};
        if isempty(rk)
            continue;                       % no ray -> Inf (outage)
        end
        pl_db(k) = coherentCombineDb([rk.PathLoss], [rk.PhaseShift]);
    end
end
