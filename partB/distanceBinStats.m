function [centers, out, counts] = distanceBinStats(dist, vals, edges, reducer)
%DISTANCEBINSTATS  Reduce per-device values into distance bins (pure helper).
%
%   [centers, out, counts] = distanceBinStats(dist, vals, edges, reducer)
%
%   dist     [n x 1] distance per device (m)
%   vals     [n x 1] value per device (e.g. availability 0/1, or margin dB)
%   edges    [1 x nb+1] bin edges (m)
%   reducer  function vec -> row, applied to the values in each bin
%            (e.g. @mean, or @(v) prctile(v,[10 50 90]))
%
%   Returns:
%     centers [1 x nb]      bin centres
%     out     [nb x k]      reducer output per bin (k = width of reducer row);
%                           empty bins are NaN
%     counts  [nb x 1]      device count per bin
%
%   R2021b-clean (discretize). No graphics.

    nb      = numel(edges) - 1;
    centers = (edges(1:end-1) + edges(2:end)) / 2;
    bin     = discretize(dist(:), edges);
    vals    = vals(:);

    counts = zeros(nb, 1);
    out    = [];
    for b = 1:nb
        m = (bin == b);
        counts(b) = nnz(m);
        if counts(b) > 0
            r = reducer(vals(m));
            if isempty(out)
                out = nan(nb, numel(r));
            end
            out(b, :) = r(:).';
        end
    end
    if isempty(out)
        out = nan(nb, 1);
    end
end
