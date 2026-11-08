function S = talos_nav_process(S, p, in, ekf)
%TALOS_NAV_PROCESS Fuse one measurement event: predict to its stamp, gate, correct, account.
% The log records the event as it arrived, before
% the IMU lever-arm compensation.
%#codegen
k = talos_nav_counters();
F = S.F;
E = in;
s = E.sensor;
imuCfg = double(p.sensor_axes.vectornav_imu_config(:));
fogCfg = double(p.sensor_axes.fog_twist_config(:));
dvlCfg = double(p.sensor_axes.dvl_twist_config(:));
depthCfg = double(p.sensor_axes.depth_pose_config(:));
if S.has_pub
    ref = S.pub_x;
else
    ref = zeros(0, 1);
end

% /clock can start late, so adopt the first stamp's epoch instead of stepping across it
if F.tf == 0
    F.tf = E.stamp;
    F.cnt(k.c_startup_epoch) = F.cnt(k.c_startup_epoch) + 1;
    if S.recording
        S.F = F;
        S = talos_nav_history(S, p, 'clear', false);
    end
end
if E.stamp <= F.last_proc(s)
    F.cnt(k.c_duplicate) = F.cnt(k.c_duplicate) + 1;
    F.cnt(k.c_duplicate_by + s - 1) = F.cnt(k.c_duplicate_by + s - 1) + 1;
    S.F = F;
    return;
end
if E.stamp < F.tf
    F.cnt(k.c_stale) = F.cnt(k.c_stale) + 1;
    F.cnt(k.c_stale_by + s - 1) = F.cnt(k.c_stale_by + s - 1) + 1;
    S.F = F;
    return;
end
if S.recording
    S = talos_nav_history(S, p, 'input', in);
end

if s == 1
    if E.groups(2) ~= 0
        [F, E] = compensate(F, p, E);
        F.vn_ns = E.stamp;
        F.vn_rate = E.value(4:6);
        F.vn_rate_var = diag(E.cov(4:6, 4:6));
    else
        F.imu_motion_ns = int64(0);
        F.imu_omega = zeros(3, 1);
        F.imu_alpha = zeros(3, 1);
    end
end
F.last_proc(s) = E.stamp;
F.proc_age(s) = double(S.now - E.stamp) * 1e-6;
F.residence(s) = 0;
if E.arrival ~= 0
    F.residence(s) = double(S.now - E.arrival) * 1e-6;
end
F.queue_age = max(0, double(S.now - E.stamp) * 1e-6);

F = predict_to(F, p, E.stamp, ekf);
F = talos_nav_install(S, F, F.x, F.P);

if F.tilt_pending && s == 1 && E.groups(1) ~= 0 && imuCfg(4) ~= 0 && imuCfg(5) ~= 0
    aligned = talos_nav_gravity_attitude(F.x, E.value(1:3));
    Pa = talos_nav_transport(F.x, aligned, F.P);
    F = talos_nav_install(S, F, aligned, Pa);
    F.tilt_pending = false;
    F.cnt(k.c_tilt_init) = F.cnt(k.c_tilt_init) + 1;
end
before = F.x;

applyTilt = false;
if s == 1 && E.groups(1) ~= 0 && any(imuCfg(4:6) ~= 0)
    period = int64(0);
    if p.estimator.vectornav_tilt_correction_frequency > 0
        period = int64(round(1e9 / p.estimator.vectornav_tilt_correction_frequency));
    end
    applyTilt = ~F.has_tilt || E.stamp - F.tilt_ns >= period;
end
F.innov_norm(s) = talos_nav_innovation_norm(F, p, E, double(applyTilt));

if s ~= 1 || applyTilt
    % the IMU gate covers tilt only, rates and acceleration still fuse
    [F, decision] = gate(F, p, E, depthCfg, fogCfg, dvlCfg);
    if decision == 2
        F.cnt(k.c_recoveries) = F.cnt(k.c_recoveries) + 1;
        F.cnt(k.c_recoveries_by + s - 1) = F.cnt(k.c_recoveries_by + s - 1) + 1;
    elseif decision == 3
        F.cnt(k.c_frozen_by + s - 1) = F.cnt(k.c_frozen_by + s - 1) + 1;
        S.F = F;
        return;
    elseif decision == 4
        F.cnt(k.c_holds_by + s - 1) = F.cnt(k.c_holds_by + s - 1) + 1;
        if s ~= 1
            S.F = F;
            return;
        end
        applyTilt = false;
    elseif decision == 1
        F.cnt(k.c_innovation) = F.cnt(k.c_innovation) + 1;
        F.cnt(k.c_innovation_by + s - 1) = F.cnt(k.c_innovation_by + s - 1) + 1;
        if s ~= 1
            S.F = F;
            return;
        end
        applyTilt = false;
    end
end
if applyTilt
    F.tilt_ns = E.stamp;
    F.has_tilt = true;
    F.cnt(k.c_tilt_corrections) = F.cnt(k.c_tilt_corrections) + 1;
end

y = zeros(9, 1);
ctx = zeros(12, 1);
R = eye(9);
consider = p.estimator.imu_acceleration_update == 2;
switch s
    case 1
        tiltMask = talos_nav_tilt_mask(p);
        S.fog_fallback = p.estimator.imu_yaw_rate_fallback ~= 0 && imuCfg(12) == 0 && fogCfg(12) ~= 0 && ...
            E.groups(2) ~= 0 && (F.last_acc(2) == 0 || ...
            double(E.stamp - F.last_acc(2)) * 1e-9 > p.timing.sensor_timeouts(2));
        rateMask = [imuCfg(10); imuCfg(11); imuCfg(12)];
        if S.fog_fallback
            rateMask(3) = 1;
            E.cov(6, 6) = E.cov(6, 6) + p.estimator.imu_yaw_rate_fallback_variance;
        end
        accelMask = imuCfg(13:15) * E.groups(3);
        y(4:6) = E.value(4:6) .* rateMask * E.groups(2);
        y(7:9) = E.value(7:9) .* accelMask;
        ctx(1:3) = p.estimator.gravitational_acceleration * E.value(1:3);
        if applyTilt
            ctx(4:6) = tiltMask;
        end
        ctx(7:9) = rateMask * E.groups(2);
        ctx(10:12) = accelMask;
        R = masked(E.cov, ctx(4:12));
    case 2
        mask = fogCfg(10:12);
        y(1:3) = E.value(1:3) .* mask;
        ctx(1:3) = mask;
        R(1:3, 1:3) = masked(E.cov(1:3, 1:3), mask);
    case 3
        mask = dvlCfg(7:9);
        y(1:3) = E.value(1:3) .* mask;
        ctx(1:3) = E.offset;
        ctx(4:6) = mask;
        R(1:3, 1:3) = masked(E.cov(1:3, 1:3), mask);
    otherwise
        y(1) = E.value(1) * depthCfg(3);
        ctx(1) = depthCfg(3);
        if depthCfg(3) ~= 0
            R(1, 1) = E.cov(1, 1);
        end
end

ok = true;
if s == 1 && consider
    [x1, P1, ok] = talos_nav_consider(F.x, F.P, y, ctx, R);
    if ok
        F.x = x1;
        F.P = P1;
    end
elseif s == 2 && p.estimator.scalar_fog_correction ~= 0 && sum(fogCfg(10:12) == 1) == 1
    [~, axis] = max(fogCfg(10:12) == 1);
    [x1, P1, ok] = talos_nav_scalar(F.x, F.P, 10 + axis, E.value(axis), E.cov(axis, axis));
    if ok
        F = talos_nav_install(S, F, x1, P1);
    end
else
    [x1, P1] = ekf(F.x, F.P, 0, zeros(19), s, y, R, ctx);
    F = talos_nav_install(S, F, x1, P1);
end
if ~ok
    F.cnt(k.c_invalid) = F.cnt(k.c_invalid) + 1;
    F.cnt(k.c_invalid_by + s - 1) = F.cnt(k.c_invalid_by + s - 1) + 1;
    S.F = F;
    return;
end
F.tf = E.stamp;

if s == 1 && applyTilt
    proj = talos_nav_yaw_gauge(before, F.x);
    F = talos_nav_install(S, F, proj, talos_nav_transport(F.x, proj, F.P));
elseif s == 1 && consider
    F = talos_nav_install(S, F, F.x, F.P);
end
F.corr_norm(s) = sqrt(sum((F.x - before) .^ 2));
F.cnt(k.c_accepted + s - 1) = F.cnt(k.c_accepted + s - 1) + 1;
F.last_acc(s) = E.stamp;
F.initialized = true;
S.F = F;
end

function F = predict_to(F, p, stamp, ekf)
% Bounded prediction steps. The bias decorrelation looks at the covariance from before the steps.
remaining = double(stamp - F.tf) * 1e-9;
dens = [p.estimator.process_noise_diag(:); p.estimator.imu_acceleration_bias_density * ones(3, 1)];
resetd = [p.estimator.reset_covariance_diag(:); p.estimator.imu_acceleration_bias_variance * ones(3, 1)];
P0 = F.P;
while remaining > 1e-9
    dt = min(remaining, p.timing.max_prediction_dt);
    [F.x, F.P] = talos_nav_coast(F, p, F.x, F.P, F.tf, dt);
    Sq = talos_process_root(F.x, P0, dt, dens, p.estimator.imu_acceleration_bias_rotation_variance, resetd);
    [F.x, F.P] = ekf(F.x, F.P, dt, Sq, 0, zeros(9, 1), eye(9), zeros(12, 1));
    if dt >= remaining
        F.tf = stamp;
    else
        F.tf = F.tf + int64(round(dt * 1e9));
    end
    remaining = double(stamp - F.tf) * 1e-9;
end
end

function [F, E] = compensate(F, p, E)
% Remove lever-arm acceleration from the IMU sample, with its covariance and rate cross terms.
omega = E.value(4:6);
r = E.offset;
tau = p.estimator.imu_angular_acceleration_time_constant;
dt = 0;
if F.imu_motion_ns ~= 0
    dt = double(E.stamp - F.imu_motion_ns) * 1e-9;
end
den = tau + dt;
decay = tau / den;
avail = dt > 0 && dt <= p.timing.max_prediction_dt;
alpha = zeros(3, 1);
if avail
    alpha = decay * F.imu_alpha + (omega - F.imu_omega) / den;
end
E.value(7:9) = E.value(7:9) - (cross(alpha, r) + cross(omega, cross(omega, r)));

skewR = [0 -r(3) r(2); r(3) 0 -r(1); -r(2) r(1) 0];
b = -(omega * r' + (omega' * r) * eye(3) - 2 * r * omega');
if avail
    b = b + skewR / den;
end
rc = E.cov(4:6, 4:6);
F.max_rate_var = max(trace(rc), F.max_rate_var * decay);
% a filtered derivative has total noise gain <= 2/tau, and noise above the acceleration
% state's bandwidth averages out, so the bound stops there
gain = min(2 / tau, p.estimator.imu_angular_acceleration_noise_bandwidth);
extra = F.max_rate_var * gain * gain * eye(3);
cr = rc * b';
E.cov(4:6, 7:9) = cr;
E.cov(7:9, 4:6) = cr';
E.cov(7:9, 7:9) = E.cov(7:9, 7:9) + b * rc * b' + skewR * extra * skewR';
F.imu_omega = omega;
F.imu_alpha = alpha;
F.imu_motion_ns = E.stamp;
end

function [F, decision] = gate(F, p, E, depthCfg, fogCfg, dvlCfg)
% Innovation gate decision for the event, with the sensor's threshold and agreement floor.
s = E.sensor;
decision = 0;
switch s
    case 1
        thr = p.gates.tilt_rejection_threshold;
    case 2
        thr = p.gates.fog_rejection_threshold;
    case 3
        thr = p.gates.dvl_rejection_threshold;
    otherwise
        thr = 0;
        if depthCfg(3) ~= 0
            thr = p.gates.depth_rejection_threshold;
        end
end
if thr == 0
    return;
end
o.threshold = thr;
o.recovery_time = p.gates.gate_recovery_time;
o.gap_timeout = p.timing.sensor_timeouts(s);
o.consistency_stddev = p.gates.gate_consistency_stddev(s);
o.frozen_limit = 0;
if s == 3
    o.frozen_limit = double(p.gates.dvl_frozen_sample_limit);
end
o.max_reacquire_scale = p.gates.gate_reacquire_max_scale;
o.consistency_threshold = p.gates.gate_consistency_threshold;
o.reacquire_samples = double(p.gates.gate_reacquire_samples);
o.recovery_samples = double(p.gates.gate_recovery_samples);
o.min_recovery_window = p.gates.gate_min_recovery_window;
o.flap_factor = p.gates.gate_flap_factor;
value = zeros(3, 1);
variance = zeros(3, 1);
n = 3;
switch s
    case 1
        value = E.value(1:3);
    case 2
        value = E.value(1:3) .* fogCfg(10:12);
        variance = diag(E.cov(1:3, 1:3));
    case 3
        value = E.value(1:3) .* dvlCfg(7:9);
        variance = diag(E.cov(1:3, 1:3));
    otherwise
        value(1) = E.value(1);
        variance(1) = E.cov(1, 1);
        n = 1;
end
sq = talos_nav_innovation_squared(F, p, E);
[g, decision] = talos_nav_gate(F.gates(s), E.stamp, value, variance, n, sq, o);
F.gates(s) = g;
end

function R = masked(C, mask)
% Rows and columns of unused axes become unit variance, decoupled.
m = mask(:) ~= 0;
R = zeros(numel(m));
for i = 1:numel(m)
    for j = 1:numel(m)
        if m(i) && m(j)
            R(i, j) = C(i, j);
        elseif i == j
            R(i, j) = 1;
        end
    end
end
end
