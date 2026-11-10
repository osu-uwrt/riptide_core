function cfg = talos_ekf_init
%TALOS_EKF_INIT Parameters for the Talos quaternion kinematic EKF.

cfg.sampleTime = 1 / 500;
cfg.initialState = [zeros(3,1); 1; zeros(12,1)];
cfg.initialCovariance = diag([1 1 0.25, 1e-3 1e-3 1e-3 1e-3, ...
    0.25 0.25 0.25, 0.05 0.05 0.05, 0.5 0.5 0.5]);

% Continuous-time covariance densities based on robot_localization. Runtime
% maps the three angular-error entries into quaternion tangent space and
% discretizes all entries using elapsed seconds.
cfg.processNoise = diag([5e-5 5e-5 6e-5, 3e-5 3e-5 6e-5 1e-6, ...
    2.5e-5 2.5e-5 4e-5, 1e-5 1e-5 2e-5, 1e-5 1e-5 1.5e-5]);

cfg.imuNoise = diag([2e-4 2e-4 2e-4, 2e-5 2e-5, 2e-3 2e-3 2e-3]);
cfg.fogNoise = 1e-6;
cfg.dvlNoise = diag([2.5e-4 2.5e-4 4e-4]);
cfg.depthNoise = 2.5e-3;
cfg.resetNoise = cfg.initialCovariance;

assignin('base', 'talosEkfCfg', cfg);
end
