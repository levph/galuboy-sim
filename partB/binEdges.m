function edges = binEdges(series, bin_m)
%BINEDGES  Distance bin edges 0..max spanning all series (Part B plot helper).
%   edges = binEdges(series, bin_m)
    maxd = 0;
    for s = series
        if ~isempty(s.dist_m)
            maxd = max(maxd, max(s.dist_m));
        end
    end
    if maxd <= 0, maxd = bin_m; end
    edges = 0:bin_m:(ceil(maxd / bin_m) * bin_m);
    if numel(edges) < 2, edges = [0 bin_m]; end
end
