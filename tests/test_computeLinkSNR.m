classdef test_computeLinkSNR < matlab.unittest.TestCase
%TEST_COMPUTELINKSNR  Verify the per-link SNR link budget against a hand calc.

    properties
        book; cfg; link;
    end

    methods (TestClassSetup)
        function setup(tc)
            mk = @(g) struct('name','x','ang_deg',[0;180],'gain_dbi',[g;g], ...
                'max_deg',180,'interp',griddedInterpolant([0 180],[g g],'linear','nearest'));
            tc.book = [mk(10) mk(6)];   % idx 1 = +10 dBi (tx), idx 2 = +6 dBi (rx)
            c.tx_power_dbm.air = 40;
            c.noise_figure_db.infantry = 5;
            c.const = struct('thermal_dbm_hz',-174,'tx_rf_loss_db',2, ...
                             'polarization_loss_db',3,'mmse_loss_db',2);
            tc.cfg = c;
            tc.link = struct('tx_side','air','rx_side','gnd','tx_dev','air', ...
                'rx_dev','infantry','tx_ant_idx',1,'rx_ant_idx',2,'bw_hz',1e6,'papr_db',8);
        end
    end

    methods (Test)
        function handCalc(tc)
            dev.pl_db        = [100 110];
            dev.steer_air_deg= [5 5];
            dev.steer_gnd_deg= [10 10];
            % EIRP=40-2-8+10=40; trg=6-3-2=1; Prx=41-pl=[-59 -69]
            % N=-174+5+60=-109; SNR=Prx-N=[50 40]
            SNR = computeLinkSNR(dev, tc.link, tc.cfg, tc.book);
            tc.verifyEqual(SNR, [50 40], 'AbsTol', 1e-9);
        end
        function outagePropagates(tc)
            dev.pl_db        = [Inf 110];
            dev.steer_air_deg= [5 5];
            dev.steer_gnd_deg= [10 10];
            SNR = computeLinkSNR(dev, tc.link, tc.cfg, tc.book);
            tc.verifyEqual(SNR(1), -Inf);
            tc.verifyEqual(SNR(2), 40, 'AbsTol', 1e-9);
        end
    end
end
