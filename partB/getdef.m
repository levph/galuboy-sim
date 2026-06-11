function v = getdef(s, f, d)
%GETDEF  Struct field value with a default (small Part B plotting helper).
%   v = getdef(s, f, d)  returns s.(f) if present & non-empty, else d.
    if isstruct(s) && isfield(s, f) && ~isempty(s.(f))
        v = s.(f);
    else
        v = d;
    end
end
