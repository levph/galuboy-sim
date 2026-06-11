classdef test_loadAntennaBook < matlab.unittest.TestCase
%TEST_LOADANTENNABOOK  Verify the one-workbook antenna loader + indexing.

    properties
        f
    end

    methods (TestClassSetup)
        function makeBook(tc)
            tc.f = [tempname '.xlsx'];
            ang = (0:10:180)';
            dir90 = 10 - 0.01*ang.^2;  dir90(ang > 90) = NaN;   % defined only 0..90
            wide  = 3  - 0.005*ang.^2;                           % full 0..180
            T = table(ang, dir90, wide, 'VariableNames', {'Angle','dir90','wide'});
            writetable(T, tc.f);
        end
    end
    methods (TestClassTeardown)
        function cleanup(tc), if exist(tc.f,'file'), delete(tc.f); end, end
    end

    methods (Test)
        function loadsColumnsAsAntennas(tc)
            book = loadAntennaBook(tc.f);
            tc.verifyEqual(numel(book), 2);
            tc.verifyEqual(book(1).name, 'dir90');
            tc.verifyEqual(book(2).name, 'wide');
        end
        function perColumnRangeAndClamp(tc)
            book = loadAntennaBook(tc.f);
            tc.verifyEqual(book(1).max_deg, 90);    % NaN rows >90 dropped
            tc.verifyEqual(book(2).max_deg, 180);
            g90 = 10 - 0.01*90^2;                   % value at the last valid angle
            tc.verifyEqual(gainFromPattern(book(1), 0),   10,  'AbsTol', 1e-9);
            tc.verifyEqual(gainFromPattern(book(1), 130), g90, 'AbsTol', 1e-9); % clamp beyond 90
        end
        function symmetricFold(tc)
            book = loadAntennaBook(tc.f);
            tc.verifyEqual(gainFromPattern(book(2), -40), gainFromPattern(book(2), 40), 'AbsTol', 1e-9);
        end
        function indexSelectsAntenna(tc)
            book = loadAntennaBook(tc.f);
            tc.verifyEqual(gainFromPattern(book(2), 0), 3, 'AbsTol', 1e-9);  % "wide" boresight
        end
    end
end
