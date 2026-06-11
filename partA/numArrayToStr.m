function s = numArrayToStr(v)
%NUMARRAYTOSTR  Pack a numeric vector into a comma-joined string (for xlsx cells).
%
%   s = numArrayToStr(v)
%
%   Used to store a per-flight-point array (path loss / steering / trajectory)
%   in a single spreadsheet cell. Inf (outage) is preserved as "Inf"; Part B
%   recovers the vector with  str2double(split(s, ",")).
%
%   Fixed 6-decimal formatting keeps ~0.1 m coordinate precision and is exact
%   enough for dB / degrees.

    if isempty(v)
        s = "";
        return;
    end
    s = join(compose("%.6f", double(v(:).')), ",");
end
