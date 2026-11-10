function S = talos_nav_reset(S, p, x, stamp, pose)
%TALOS_NAV_RESET Restart the filter at state x and time stamp, keeping the counters.
% pose is the requested 6x6 pose covariance (zeros for none). Queued data is dropped.
%#codegen
S.snap = true;
S.qhead = 1;
S.qn = 0;
S.qcount = zeros(4, 1);
S.late_stamp = zeros(4, 1, 'int64');
S = talos_nav_history(S, p, 'clear', false);
F = talos_nav_filter_init(p, S.F.cnt);
F.proc_age = S.F.proc_age;
F.residence = S.F.residence;
F.queue_age = S.F.queue_age;
F.innov_norm = S.F.innov_norm;
F.corr_norm = S.F.corr_norm;
F.tilt_pending = false;
if S.has_pub
    x = talos_nav_normalize(x, S.pub_x);
else
    x = talos_nav_normalize(x, zeros(0, 1));
end
P = diag([p.estimator.reset_covariance_diag(:); ...
    p.estimator.imu_acceleration_bias_variance * ones(3, 1)]);
if any(pose(:) ~= 0)
    spec = talos_nav_pose_covariance('fill', x, pose, talos_nav_pose_covariance('ros', x, P));
    P = talos_nav_pose_covariance('apply', x, spec, P);
end
F = talos_nav_install(S, F, x, P);
F.tf = stamp;
F.initialized = true;
S.F = F;
S.reset_stamp = stamp;
S = talos_nav_history(S, p, 'checkpoint', true);
k = talos_nav_counters();
S.n(k.n_resets) = S.n(k.n_resets) + 1;
end
