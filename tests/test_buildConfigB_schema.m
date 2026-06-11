classdef test_buildConfigB_schema < matlab.unittest.TestCase
%TEST_BUILDCONFIGB_SCHEMA  Validate the Part B link-budget config model.

    methods (Test)

        function topLevelFields(tc)
            cfg = buildConfigB();
            tc.verifyTrue(all(isfield(cfg, ...
                {'percentile','const','tx_power_dbm','noise_figure_db','antenna_book','links','plot'})));
            tc.verifyEqual(cfg.percentile, 99);
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

        function fourLinksWellFormed(tc)
            cfg = buildConfigB();
            tc.verifyEqual(numel(cfg.links), 4);
            tc.verifyEqual(string({cfg.links.name}), ...
                ["DL_infantry","DL_vehicular","UL_infantry","UL_vehicular"]);
            req = {'dir','ground','tx_dev','rx_dev','tx_ant_idx','rx_ant_idx', ...
                   'tx_side','rx_side','bw_hz','papr_db','req_snr_db'};
            for k = 1:numel(cfg.links)
                tc.verifyTrue(all(isfield(cfg.links(k), req)));
                tc.verifyTrue(ismember(cfg.links(k).tx_side, {'air','gnd'}));
                tc.verifyTrue(ismember(cfg.links(k).rx_side, {'air','gnd'}));
            end
        end

        function linkSidesConsistent(tc)
            % air-side antenna must be the aircraft; ground-side the ground unit.
            cfg = buildConfigB();
            for k = 1:numel(cfg.links)
                L = cfg.links(k);
                if strcmp(L.dir,'DL')   % air transmits
                    tc.verifyEqual(L.tx_dev,'air'); tc.verifyEqual(L.tx_side,'air');
                    tc.verifyEqual(L.rx_dev,L.ground); tc.verifyEqual(L.rx_side,'gnd');
                else                    % UL: ground transmits
                    tc.verifyEqual(L.tx_dev,L.ground); tc.verifyEqual(L.tx_side,'gnd');
                    tc.verifyEqual(L.rx_dev,'air'); tc.verifyEqual(L.rx_side,'air');
                end
            end
        end

        function oldSchemaGone(tc)
            cfg = buildConfigB();
            tc.verifyFalse(isfield(cfg,'link'),  'old cfg.link replaced by tx_power_dbm/links');
            tc.verifyFalse(isfield(cfg,'mcs'),   'mcs selection not in this config');
        end

    end
end
