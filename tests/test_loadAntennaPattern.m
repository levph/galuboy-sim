classdef test_loadAntennaPattern < matlab.unittest.TestCase
%TEST_LOADANTENNAPATTERN  CSV -> griddedInterpolant gain_func.

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

        function isotropicSentinel(tc)
            pat = loadAntennaPattern('tx_omni', tc.AntennasDir);
            tc.verifyTrue(pat.is_isotropic);
            tc.verifyEqual(pat.flat_gain_dbi, 7.0);
            % Flat handle: any input -> 7.0
            G = pat.gain_func([0, pi/4; -pi, 0.7], [0, 0.1; 0.2, 0.3]);
            tc.verifyEqual(G, 7.0 * ones(2, 2));
        end

        function griddedInfantryPattern(tc)
            pat = loadAntennaPattern('rx_infantry_omni', tc.AntennasDir);
            tc.verifyFalse(pat.is_isotropic);

            % Boresight peak: az=0, el=0 should be the maximum (6.5 dBi)
            G_boresight = pat.gain_func(0, 0);
            tc.verifyEqual(G_boresight, 6.5, 'AbsTol', 1e-6);

            % El=±pi/2 should be the minimum (2.5 dBi)
            G_zenith = pat.gain_func(0,  pi/2);
            G_nadir  = pat.gain_func(0, -pi/2);
            tc.verifyEqual(G_zenith, 2.5, 'AbsTol', 1e-6);
            tc.verifyEqual(G_nadir,  2.5, 'AbsTol', 1e-6);

            % Az is irrelevant for this pattern. Pass column vectors so
            % griddedInterpolant treats them as scattered queries (not mesh).
            G_az_sweep = pat.gain_func([0; pi/2; pi; 3*pi/2], zeros(4, 1));
            tc.verifyEqual(max(G_az_sweep) - min(G_az_sweep), 0, 'AbsTol', 1e-6);
        end

        function vehicularDeepNotch(tc)
            pat = loadAntennaPattern('rx_vehicular_dipole', tc.AntennasDir);
            % Horizon peak ~7 dBi
            tc.verifyEqual(pat.gain_func(0, 0), 7.0, 'AbsTol', 1e-6);
            % Zenith / nadir notch ~-10 dBi
            tc.verifyEqual(pat.gain_func(0,  pi/2), -10.0, 'AbsTol', 1e-6);
            tc.verifyEqual(pat.gain_func(0, -pi/2), -10.0, 'AbsTol', 1e-6);
        end

        function outOfGridClamps(tc)
            pat = loadAntennaPattern('rx_infantry_omni', tc.AntennasDir);
            % Beyond +pi/2 elevation -> clamps to +pi/2 grid edge (2.5 dBi)
            G_far = pat.gain_func(0, pi);   % off-grid
            tc.verifyTrue(isfinite(G_far), 'must not return NaN out of range');
            tc.verifyEqual(G_far, 2.5, 'AbsTol', 1e-6);
        end

        function vectorizedShape(tc)
            pat = loadAntennaPattern('rx_vehicular_dipole', tc.AntennasDir);
            az = linspace(-pi, pi, 7).';
            el = linspace(-pi/2, pi/2, 7).';
            [AZ, EL] = ndgrid(az, el);
            G = pat.gain_func(AZ, EL);
            tc.verifyEqual(size(G), size(AZ));
            tc.verifyTrue(all(isfinite(G(:))));
        end

        function missingFileErrors(tc)
            tc.verifyError(@() loadAntennaPattern('not_a_real_pattern', tc.AntennasDir), ...
                'loadAntennaPattern:notFound');
        end
    end
end
