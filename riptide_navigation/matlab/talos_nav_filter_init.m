function F = talos_nav_filter_init(p, cnt)
%TALOS_NAV_FILTER_INIT Replayed filter state at the configured initial condition.
% cnt carries the counters over a reset. Everything else restarts.
%#codegen
k = talos_nav_counters();
F.x = [zeros(3, 1); 1; zeros(12, 1); p.estimator.imu_acceleration_bias(:)];
F.P = diag([p.estimator.reset_covariance_diag(:); ...
    p.estimator.imu_acceleration_bias_variance * ones(3, 1)]);
F.tf = int64(0);
F.last_proc = zeros(4, 1, 'int64');
F.last_acc = zeros(4, 1, 'int64');
F.imu_motion_ns = int64(0);
F.imu_omega = zeros(3, 1);
F.imu_alpha = zeros(3, 1);
F.tilt_ns = int64(0);
F.has_tilt = false;
F.max_rate_var = 0;
F.tilt_pending = p.estimator.initialize_tilt_from_imu ~= 0;
g = talos_nav_gate_init();
F.gates = [g, g, g, g];
F.vn_ns = int64(0);
F.vn_rate = zeros(3, 1);
F.vn_rate_var = zeros(3, 1);
F.initialized = false;
F.cnt = zeros(k.c_count, 1);
if ~isempty(cnt)
    F.cnt = cnt;
end
F.proc_age = zeros(4, 1);
F.residence = zeros(4, 1);
F.queue_age = 0;
F.innov_norm = zeros(4, 1);
F.corr_norm = zeros(4, 1);
end
