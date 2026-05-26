classdef test_loadAntennaPattern_1d < matlab.unittest.TestCase
%TEST_LOADANTENNAPATTERN_1D  1D el_deg,gain_dbi CSV loader path.
%
%   Verifies the new 1D header form loads correctly, builds a 1D
%   griddedInterpolant keyed only on elevation (radians), and returns
%   the table values at the sample points (azimuth is ignored).

    properties
        AntennasDir
    end

    methods (TestClassSetup)
        function findDir(tc)
            this_dir = fileparts(mfilename('fullpath'));
            tc.AntennasDir = fullfile(fileparts(this_dir), 'resources', 'antennas');
            tc.assertTrue(exist(tc.AntennasDir, 'dir') > 0, ...
                'resources/antennas/ must exist for these tests');
        end
    end

    methods (Test)

        function airborneTxIs1d(tc)
            pat = loadAntennaPattern('airborne_tx', tc.AntennasDir);
            tc.verifyEqual(pat.kind, '1d');
            tc.verifyFalse(pat.is_isotropic);

            % Sample at the exact table points (peak at el=0 -> 2 dBi,
            % floor at el=+-pi/2 -> -3 dBi).
            tc.verifyEqual(pat.gain_func(0, 0),        2.0,  'AbsTol', 1e-6);
            tc.verifyEqual(pat.gain_func(0,  pi/2),   -3.0,  'AbsTol', 1e-6);
            tc.verifyEqual(pat.gain_func(0, -pi/2),   -3.0,  'AbsTol', 1e-6);
        end

        function azIsIgnoredFor1d(tc)
            pat = loadAntennaPattern('ground_tx', tc.AntennasDir);
            G0   = pat.gain_func( 0,    0);
            Gpi  = pat.gain_func( pi,   0);
            Gpi2 = pat.gain_func(-pi/2, 0);
            tc.verifyEqual(G0, Gpi,  'AbsTol', 1e-6, 'az should be ignored for 1D pattern');
            tc.verifyEqual(G0, Gpi2, 'AbsTol', 1e-6, 'az should be ignored for 1D pattern');
        end

        function applyAntennaGainBranchesOn1d(tc)
            % applyAntennaGain should use only the elevation offset for 1D.
            pat = loadAntennaPattern('airborne_tx', tc.AntennasDir);
            G_peak = applyAntennaGain(pat, 0, 0, 0, 0);            % on-boresight
            G_neg  = applyAntennaGain(pat, 0, pi/2, 0, 0);         % off-boresight in el
            tc.verifyGreaterThan(G_peak, G_neg, ...
                '1D pattern: gain at boresight el should exceed gain at +pi/2 offset');
        end

        function outOfRangeClamps(tc)
            pat = loadAntennaPattern('airborne_tx', tc.AntennasDir);
            G_far = pat.gain_func(0, 5);   % beyond +pi/2 -> clamps
            tc.verifyTrue(isfinite(G_far));
        end

        function badHeaderErrors(tc)
            % Write a temp CSV with an unrecognized header.
            bad_name = sprintf('test_bad_%d', feature('getpid'));
            bad_path = fullfile(tc.AntennasDir, [bad_name, '.csv']);
            fid = fopen(bad_path, 'w'); fwrite(fid, sprintf('foo,bar\n0,1')); fclose(fid);
            cleanup = onCleanup(@() delete(bad_path));
            tc.verifyError(@() loadAntennaPattern(bad_name, tc.AntennasDir), ...
                'loadAntennaPattern:badHeader');
        end

        function patchCsvLoadsAs2d(tc)
            pat = loadAntennaPattern('airborne_rx', tc.AntennasDir);
            tc.verifyEqual(pat.kind, '2d');
            % N=16 patch -> peak G_max = 10*log10(16) ~ 12.04 dBi inside FOV.
            tc.verifyEqual(pat.gain_func(0, 0), 10*log10(16), 'AbsTol', 1e-3);
        end
    end
end
