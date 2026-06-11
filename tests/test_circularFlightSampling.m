classdef test_circularFlightSampling < matlab.unittest.TestCase
%TEST_CIRCULARFLIGHTSAMPLING  Verify coordinated-turn speed + derived sampling.

    methods (Static)
        function fc = cfg(varargin)
            fc = struct('max_rotation_deg', 360, 'delta_t_s', 1.0, ...
                        'circle_radius_m', 1000, 'max_tilt_deg', 23);
            for k = 1:2:numel(varargin)
                fc.(varargin{k}) = varargin{k+1};
            end
        end
    end

    methods (Test)

        function speedFromCoordinatedTurn(tc)
            s = circularFlightSampling(test_circularFlightSampling.cfg());
            v_expected = sqrt(9.81 * 1000 * tand(23));
            tc.verifyEqual(s.v_mps, v_expected, 'RelTol', 1e-9);
            tc.verifyEqual(s.omega_radps, v_expected/1000, 'RelTol', 1e-9);
            tc.verifyEqual(s.v_mps, 64.53, 'AbsTol', 0.1);   % ~64.5 m/s
        end

        function sampleCountDerived(tc)
            s = circularFlightSampling(test_circularFlightSampling.cfg());
            % round(2*pi / (omega*delta_t)); ~97 for the defaults
            expected = round(2*pi / (s.omega_radps * 1.0));
            tc.verifyEqual(s.num_samples, expected);
            tc.verifyEqual(s.num_samples, 97);
        end

        function effectiveDtMatchesRequested(tc)
            s = circularFlightSampling(test_circularFlightSampling.cfg());
            tc.verifyEqual(s.dt_effective_s, 1.0, 'AbsTol', 0.02);
        end

        function anglesSpanArcNoDuplicateClose(tc)
            s = circularFlightSampling(test_circularFlightSampling.cfg());
            tc.verifySize(s.sample_angles_rad, [1 s.num_samples]);
            tc.verifyEqual(s.sample_angles_rad(1), 0);
            tc.verifyLessThan(s.sample_angles_rad(end), 2*pi);   % [0, 2*pi)
        end

        function finerDtGivesMoreSamples(tc)
            s1 = circularFlightSampling(test_circularFlightSampling.cfg('delta_t_s', 1.0));
            s2 = circularFlightSampling(test_circularFlightSampling.cfg('delta_t_s', 0.5));
            tc.verifyGreaterThan(s2.num_samples, s1.num_samples);
        end

        function biggerRadiusIsFaster(tc)
            s1 = circularFlightSampling(test_circularFlightSampling.cfg('circle_radius_m', 1000));
            s2 = circularFlightSampling(test_circularFlightSampling.cfg('circle_radius_m', 2000));
            tc.verifyGreaterThan(s2.v_mps, s1.v_mps);     % v ~ sqrt(R)
        end

    end
end
