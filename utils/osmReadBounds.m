function [lonlim, latlim, bounds] = osmReadBounds(osmPath)
%OSMREADBOUNDS Extract lat/lon bounds from an .osm XML file.
%
%   [lonlim, latlim, bounds] = osmReadBounds(osmPath)
%     lonlim = [minlon maxlon]
%     latlim = [minlat maxlat]
%     bounds = struct with fields minlat, minlon, maxlat, maxlon
%
% Works by scanning the first ~200 lines for a <bounds .../> tag.

    arguments
        osmPath (1,1) string
    end

    fid = fopen(osmPath, 'r');
    if fid < 0
        error("Could not open file: %s", osmPath);
    end
    c = onCleanup(@() fclose(fid));

    maxLinesToScan = 200;
    boundsLine = "";

    for k = 1:maxLinesToScan
        tline = fgetl(fid);
        if ~ischar(tline); break; end

        if contains(tline, "<bounds") %#ok<*STREMP>
            boundsLine = string(tline);
            break;
        end
    end

    if boundsLine == ""
        error("No <bounds .../> tag found in the first %d lines of %s", ...
              maxLinesToScan, osmPath);
    end

    % Extract attributes robustly (handles spacing/order differences)
    bounds.minlat = extractAttr(boundsLine, "minlat");
    bounds.minlon = extractAttr(boundsLine, "minlon");
    bounds.maxlat = extractAttr(boundsLine, "maxlat");
    bounds.maxlon = extractAttr(boundsLine, "maxlon");

    latlim = [bounds.minlat bounds.maxlat];
    lonlim = [bounds.minlon bounds.maxlon];
end

function val = extractAttr(line, attrName)
% Extract numeric value from attrName="..."
    pat = attrName + "=""([^""]+)""";
    tok = regexp(line, pat, "tokens", "once");
    if isempty(tok)
        error("Missing attribute %s in bounds line: %s", attrName, line);
    end
    val = str2double(tok{1});
    if isnan(val)
        error("Attribute %s is not numeric: %s", attrName, tok{1});
    end
end