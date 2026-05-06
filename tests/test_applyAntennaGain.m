classdef test_applyAntennaGain < matlab.unittest.TestCase
%TEST_APPLYANTENNAGAIN  Boresight subtraction, wrapping, vector shape.

    properties
        AntennasDir
        VehPat
        IsoPat
    end

    methods (TestClassSetup)
        function loadPats(tc)
            this_dir = fileparts(mfilename('fullpath'));
            tc.AntennasDir = fullfile(fileparts(this_dir), 'resources', 'antennas');
            tc.VehPat = loadAntennaPattern('rx_vehicular_dipole', tc.AntennasDir);
            tc.IsoPat = loadAntennaPattern('tx_omni',             tc.AntennasDir);
        end
    end

    methods (Test)

        function isotropicAlwaysFlat(tc)
            G = applyAntennaGain(tc.IsoPat, ...
                [0, pi/2, pi], [0, 0.5, -0.3], pi/4, -pi/3);
            tc.verifyEqual(G, [7.0, 7.0, 7.0]);
        end

        function boresightAtZenithPeaksAtZenith(tc)
            % RX boresight at zenith (+pi/2). Link arriving from straight
            % above (el = pi/2) is "boresight on" -> max gain.
            G_zenith = applyAntennaGain(tc.VehPat, 0,  pi/2, 0, pi/2);
            G_horiz  = applyAntennaGain(tc.VehPat, 0,  0,    0, pi/2);
            tc.verifyTrue(G_zenith > G_horiz, ...
                'Zenith arrival should beat horizon when boresight is zenith');
        end

        function azimuthSubtraction(tc)
            % VehPat is azimuth-independent so changing boresight_az should
            % not change the output for the same input az.
            G1 = applyAntennaGain(tc.VehPat, pi/3, 0, 0,    pi/2);
            G2 = applyAntennaGain(tc.VehPat, pi/3, 0, pi/2, pi/2);
            tc.verifyEqual(G1, G2, 'AbsTol', 1e-6);
        end

        function elevationClamps(tc)
            % Inputs beyond +-pi/2 are clamped, must not produce NaN.
            G = applyAntennaGain(tc.VehPat, 0, 5, 0, pi/2);
            tc.verifyTrue(isfinite(G));
        end

        function vectorisedPreservesShape(tc)
            sz = [4, 5];
            az = single(rand(sz) * 2*pi - pi);
            el = single(rand(sz) * pi - pi/2);
            G  = applyAntennaGain(tc.VehPat, az, el, 0, pi/2);
            tc.verifyEqual(size(G), sz);
            tc.verifyTrue(all(isfinite(G(:))));
            tc.verifyClass(G, 'single', 'Output should preserve single precision');
        end
    end
end
