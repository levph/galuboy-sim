classdef test_buildConfigB_schema < matlab.unittest.TestCase
%TEST_BUILDCONFIGB_SCHEMA  Validate the Part B link-budget config model.

    methods (Test)

        function topLevelFields(tc)
            cfg = buildConfigB();
            tc.verifyTrue(all(isfield(cfg, ...
                {'percentile','const','tx_power_dbm','noise_figure_db','antenna_book', ...
                 'mcs_book','finger','links','plot'})));
            tc.verifyEqual(cfg.percentile, 99);
        end

        function fingerConfigWellFormed(tc)
            cfg = buildConfigB();
            tc.verifyTrue(all(isfield(cfg.finger.bw_per_finger_hz, {'DL','UL'})));
            tc.verifyGreaterThan(cfg.finger.bw_per_finger_hz.DL, 0);
            tc.verifyGreaterThan(cfg.finger.bw_per_finger_hz.UL, 0);
        end

        function constantsAndDevices(tc)
            cfg = buildConfigB();
            tc.verifyTrue(all(isfield(cfg.const, ...
                {'thermal_dbm_hz','tx_rf_loss_db','polarization_loss_db','mmse_loss_db'})));
            tc.verifyEqual(cfg.const.thermal_dbm_hz, -174);
            for d = ["air","infantry","vehicular"]
                tc.verifyTrue(isfield(cfg.tx_power_dbm, d));
                tc.verifyTrue(isfield(cfg.noise_figure_db, d));
            end
        end

        function sixLinksWellFormed(tc)
            cfg = buildConfigB();
            tc.verifyEqual(numel(cfg.links), 6);
            tc.verifyEqual(string({cfg.links.name}), ...
                ["DL_infantry","DL_vehicular","DL_infantry_reduced", ...
                 "UL_infantry","UL_vehicular","UL_infantry_reduced"]);
            req = {'dir','ground','tx_dev','rx_dev','tx_ant','rx_ant', ...
                   'tx_side','rx_side','mcs_index','fingers'};
            for k = 1:numel(cfg.links)
                tc.verifyTrue(all(isfield(cfg.links(k), req)));
                tc.verifyTrue(ismember(cfg.links(k).tx_side, {'air','gnd'}));
                tc.verifyTrue(ismember(cfg.links(k).rx_side, {'air','gnd'}));
                tc.verifyTrue(cfg.links(k).mcs_index >= 1 && cfg.links(k).mcs_index <= 11);
                tc.verifyGreaterThanOrEqual(cfg.links(k).fingers, 1);
            end
        end

        function linkSidesConsistent(tc)
            % air-side antenna must be the aircraft; ground-side the ground unit
            % (tx_dev/rx_dev may be a power variant of L.ground, e.g.
            % 'infantry_reduced', so check the variant's device family matches
            % rather than requiring exact equality).
            cfg = buildConfigB();
            for k = 1:numel(cfg.links)
                L = cfg.links(k);
                if strcmp(L.dir,'DL')   % air transmits
                    tc.verifyEqual(L.tx_dev,'air'); tc.verifyEqual(L.tx_side,'air');
                    tc.verifyTrue(startsWith(L.rx_dev,L.ground)); tc.verifyEqual(L.rx_side,'gnd');
                else                    % UL: ground transmits
                    tc.verifyTrue(startsWith(L.tx_dev,L.ground)); tc.verifyEqual(L.tx_side,'gnd');
                    tc.verifyEqual(L.rx_dev,'air'); tc.verifyEqual(L.rx_side,'air');
                end
            end
        end

        function oldSchemaGone(tc)
            cfg = buildConfigB();
            tc.verifyFalse(isfield(cfg,'link'),  'old cfg.link replaced by tx_power_dbm/links');
            tc.verifyFalse(isfield(cfg,'mcs'),   'no top-level mcs field (mcs is per-link)');
            for k = 1:numel(cfg.links)
                tc.verifyFalse(isfield(cfg.links(k),'bw_hz'),   'bw_hz now resolved, not GIVEN');
                tc.verifyFalse(isfield(cfg.links(k),'papr_db'), 'papr_db now resolved, not GIVEN');
                tc.verifyFalse(isfield(cfg.links(k),'tx_ant_idx'), 'antennas now selected by name');
            end
        end

    end
end
