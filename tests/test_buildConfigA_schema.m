classdef test_buildConfigA_schema < matlab.unittest.TestCase
%TEST_BUILDCONFIGA_SCHEMA  Validate the Part A (raytracing) config overrides.
%
%   buildConfigA() starts from buildConfig() and overrides only the Part A
%   fields. This checks those overrides took, the base schema is intact, and
%   the placeholder MCS table is well formed.

    methods (Test)

        function partAFieldsPresent(tc)
            cfg = buildConfigA();
            for f = ["tx","rx","propagation","flight","regions","io"]
                tc.verifyTrue(isfield(cfg, f), sprintf('missing field %s', f));
            end
            tc.verifyEqual(numel(cfg.regions), 6);
        end

        function partBKnobsAbsentFromA(tc)
            % Part A must not carry Part B concerns (powers, antenna names,
            % noise/BW, availability thresholds, LR variability).
            cfg = buildConfigA();
            tc.verifyFalse(isfield(cfg, 'analysis'), 'analysis is Part B');
            tc.verifyFalse(isfield(cfg.tx, 'power_air_dbm'), 'powers are Part B');
            tc.verifyFalse(isfield(cfg.tx, 'power_gnd_dbm'), 'powers are Part B');
            tc.verifyFalse(isfield(cfg.tx, 'airborne'), 'tx antenna names are Part B');
            tc.verifyFalse(isfield(cfg.rx.infantry, 'antenna_name_dl'), 'rx patterns are Part B');
            tc.verifyFalse(isfield(cfg.propagation, 'bandwidth_hz'), 'BW is Part B');
            tc.verifyFalse(isfield(cfg.propagation, 'time_variability'), 'LR-only field');
            tc.verifyFalse(isfield(cfg.flight, 'step_degrees'), 'dead with random rotations');
            % Boresights DO stay in A (platform geometry for steering angle).
            tc.verifyTrue(isfield(cfg.tx, 'boresight_el_rad'));
            tc.verifyTrue(isfield(cfg.rx.infantry, 'boresight_el_rad'));
            tc.verifyTrue(isfield(cfg.rx.airborne, 'boresight_el_rad'));
        end

        function aircraftGeometryOverridden(tc)
            cfg = buildConfigA();
            tc.verifyEqual(cfg.tx.frequency_hz, 4e9);
            tc.verifyEqual(cfg.tx.altitude_m, 4000);
            tc.verifyEmpty(cfg.tx.altitude_msl_m);
        end

        function circularTrajectory(tc)
            cfg = buildConfigA();
            % user-set trajectory params (sample count is derived, not stored)
            tc.verifyTrue(all(isfield(cfg.flight, ...
                {'max_rotation_deg','delta_t_s','circle_radius_m','max_tilt_deg'})));
            tc.verifyGreaterThan(cfg.flight.circle_radius_m, 0);
            tc.verifyGreaterThan(cfg.flight.delta_t_s, 0);
            tc.verifyEqual(cfg.flight.max_tilt_deg, 23);
            tc.verifyEqual(cfg.flight.tilt_direction, 'outward');
            % retired: lemniscate, rotation sweep, and the hard-coded sample count
            tc.verifyFalse(isfield(cfg.flight, 'lemniscate_length_m'));
            tc.verifyFalse(isfield(cfg.flight, 'num_rotations'));
            tc.verifyFalse(isfield(cfg.flight, 'num_flight_steps'));
        end

        function receiverCounts(tc)
            cfg = buildConfigA();
            tc.verifyEqual(cfg.rx.infantry.count, 100);
            tc.verifyEqual(cfg.rx.vehicular.count, 100);
        end

        function rejectInBuildingsDefaultOn(tc)
            cfg = buildConfigA();
            tc.verifyTrue(isfield(cfg.rx, 'reject_in_buildings'));
            tc.verifyTrue(cfg.rx.reject_in_buildings);
        end

        function raytracingParams(tc)
            cfg = buildConfigA();
            tc.verifyEqual(cfg.propagation.model, 'raytracing');
            tc.verifyEqual(cfg.propagation.rt_method, 'sbr');
            tc.verifyEqual(cfg.propagation.max_reflections, 2);
            tc.verifyEqual(cfg.propagation.max_diffractions, 0);
            tc.verifyTrue(cfg.propagation.use_terrain);
        end

        function resourceDirs(tc)
            cfg = buildConfigA();
            tc.verifyTrue(isfield(cfg.io, 'buildings_dir'));
            tc.verifyTrue(isfield(cfg.io, 'terrain_dir'));
        end

        function mcsTableWellFormed(tc)
            T = mcsTable();
            tc.verifyTrue(istable(T));
            tc.verifyTrue(all(ismember({'mcs','name','snr_min_db'}, T.Properties.VariableNames)));
            tc.verifyGreaterThan(height(T), 0);
            % thresholds strictly increasing with MCS
            tc.verifyTrue(all(diff(T.snr_min_db) > 0));
        end

    end
end
