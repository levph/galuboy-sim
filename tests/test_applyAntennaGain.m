classdef test_applyAntennaGain < matlab.unittest.TestCase
%TEST_APPLYANTENNAGAIN  Boresight subtraction, wrapping, vector shape.
%
%   Also locks the fix for the nadir/zenith singularity in 2D patterns:
%   a patch antenna with boresight at nadir (el=-pi/2) or zenith (el=+pi/2)
%   must give max gain when queried from the boresight direction regardless
%   of the azimuth component of that query.

    properties
        AntennasDir
        VehPat
        IsoPat
        AirRxPat    % airborne_rx: 2D patch, nadir boresight
        Gnd1Pat     % ground1_rx:  2D patch, zenith boresight
    end

    methods (TestClassSetup)
        function loadPats(tc)
            this_dir = fileparts(mfilename('fullpath'));
            tc.AntennasDir = fullfile(fileparts(this_dir), 'resources', 'antennas');
            tc.VehPat   = loadAntennaPattern('rx_vehicular_dipole', tc.AntennasDir);
            tc.IsoPat   = loadAntennaPattern('tx_omni',             tc.AntennasDir);
            tc.AirRxPat = loadAntennaPattern('airborne_rx',         tc.AntennasDir);
            tc.Gnd1Pat  = loadAntennaPattern('ground1_rx',          tc.AntennasDir);
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

        % --- Nadir/zenith singularity fix (2D patch) -----------------------
        % Before the fix, mod(AZ_tx - bs_az, 2*pi) = pi for many geometries
        % when boresight is at nadir/zenith, causing floor gain (-20 dBi)
        % instead of peak gain (~12 dBi).

        function nadirPatch_boresightDirectionGivesMaxGain(tc)
            % airborne_rx boresight = nadir (el=-pi/2).
            % Query at (AZ=pi, EL=-pi/2): same physical direction as boresight
            % (nadir), just with az=pi. The old code gave floor; correct is max.
            bs_az = 0; bs_el = -pi/2;
            G_any_az = applyAntennaGain(tc.AirRxPat, pi,   -pi/2, bs_az, bs_el);
            G_zero_az= applyAntennaGain(tc.AirRxPat, 0,    -pi/2, bs_az, bs_el);
            tc.verifyEqual(double(G_any_az), double(G_zero_az), 'AbsTol', 0.1, ...
                '(AZ=pi,EL=-pi/2) is the same nadir direction as (AZ=0,EL=-pi/2); gains must match');
            tc.verifyGreaterThan(double(G_any_az), 10, ...
                'Nadir-pointing patch at exact boresight must give near-peak gain (>10 dBi)');
        end

        function zenithPatch_boresightDirectionGivesMaxGain(tc)
            % ground1_rx boresight = zenith (el=+pi/2).
            % Query at (AZ=pi, EL=+pi/2): same physical direction as boresight.
            bs_az = 0; bs_el = pi/2;
            G_any_az  = applyAntennaGain(tc.Gnd1Pat, pi, pi/2, bs_az, bs_el);
            G_zero_az = applyAntennaGain(tc.Gnd1Pat, 0,  pi/2, bs_az, bs_el);
            tc.verifyEqual(double(G_any_az), double(G_zero_az), 'AbsTol', 0.1, ...
                '(AZ=pi,EL=+pi/2) is the same zenith direction as (AZ=0,EL=+pi/2); gains must match');
            tc.verifyGreaterThan(double(G_any_az), 7, ...
                'Zenith-pointing patch at exact boresight must give near-peak gain (>7 dBi)');
        end

        function nadirPatch_symmetricAroundBoresightAxis(tc)
            % At equal off-boresight angles, gain must be equal regardless of
            % the azimuth direction around the nadir axis.
            bs_az = 0; bs_el = -pi/2;
            theta = pi/6;   % 30-deg off-boresight
            az_vals = single([0, pi/3, 2*pi/3, pi, 4*pi/3, 5*pi/3]);
            el_val  = single(-pi/2 + theta);   % 30 deg above nadir in world frame
            G = applyAntennaGain(tc.AirRxPat, az_vals, repmat(el_val,1,6), bs_az, bs_el);
            tc.verifyEqual(double(G), repmat(double(G(1)),1,6), 'AbsTol', 0.1, ...
                'Gain must be equal for all azimuths at the same off-boresight angle');
        end
    end
end
