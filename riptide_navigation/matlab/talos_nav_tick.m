function [S, out] = talos_nav_tick(S, p, now, doPublish, ekf)
%TALOS_NAV_TICK Drain the reorder queue up to now, and publish if asked.
% ekf is the filter, see talos_nav_ekf and talos_filter_apply.
% Samples older than the filter replay from the checkpoint log, DVL and depth beyond the log are
% fused now with the lag added to R, and a clock jump back restarts the filter.
%#codegen
k = talos_nav_counters();
S.now = now;
out = talos_nav_output();

% a /clock that starts late leaves the filter at zero until its first update
if S.F.tf == 0 && now > 0
    if S.qn == 0
        S.F.tf = now;
    else
        S.F.tf = min(now, S.q(slot(S, 1)).stamp);
    end
    S.F.cnt(k.c_startup_epoch) = S.F.cnt(k.c_startup_epoch) + 1;
    S = talos_nav_history(S, p, 'clear', false);
    S = talos_nav_history(S, p, 'checkpoint', true);
end
if S.last_clock ~= 0 && now < S.last_clock
    was = S.F.initialized;
    S = talos_nav_reset(S, p, S.F.x, now, zeros(6));
    S.F.initialized = was;
    S.last_pub = int64(0);
    S.last_diag = int64(0);
end
S.last_clock = now;

watermark = now - int64(round(max(0, p.timing.reorder_delay) * 1e9));
limit = double(p.timing.max_measurements_per_cycle);
processed = 0;
while S.qn > 0 && S.q(slot(S, 1)).stamp <= watermark
    if limit > 0 && processed >= limit
        S.n(k.n_overload) = S.n(k.n_overload) + 1;
        break;
    end
    E = S.q(slot(S, 1));
    S = pop(S);
    if E.stamp < S.F.tf
        S = process_late(S, p, E, ekf);
    else
        S = talos_nav_process(S, p, E, ekf);
        S = talos_nav_history(S, p, 'checkpoint', false);
    end
    processed = processed + 1;
end
S = talos_nav_history(S, p, 'trim', S.F.tf - int64(round(p.timing.history_length * 1e9)));

if doPublish
    queued = p.timing.publication_mode == 2;
    ready = queued && S.qn > 0 && S.q(slot(S, 1)).stamp <= now;
    if (S.F.initialized || ready) && (S.last_pub == 0 || now > S.last_pub)
        [S, out] = publish(S, p, now, ekf);
        if out.valid
            S.last_pub = now;
        end
    end
end
end
function i = slot(S, n)
% Ring slot of the n-th queued event, counting from the front.
i = mod(S.qhead + n - 2, numel(S.q)) + 1;
end

function S = pop(S)
s = S.q(slot(S, 1)).sensor;
S.qhead = mod(S.qhead, numel(S.q)) + 1;
S.qn = S.qn - 1;
S.qcount(s) = S.qcount(s) - 1;
end

function t = earlier(a, b)
if a.stamp == b.stamp
    t = a.seq < b.seq;
else
    t = a.stamp < b.stamp;
end
end

function S = process_late(S, p, E, ekf)
% A sample older than the filter: replay it from the log, or fuse it now with the lag in R.
k = talos_nav_counters();
z = talos_nav_sizes();
s = E.sensor;
F = S.F;
lag = double(F.tf - E.stamp) * 1e-9;
S.late_lag(s) = lag * 1000;
S.late_lag_max(s) = max(S.late_lag_max(s), S.late_lag(s));
if E.stamp < S.reset_stamp || lag > p.timing.max_measurement_lag
    S.F.cnt(k.c_stale) = S.F.cnt(k.c_stale) + 1;
    S.F.cnt(k.c_stale_by + s - 1) = S.F.cnt(k.c_stale_by + s - 1) + 1;
    return;
end
dup = E.stamp <= S.late_stamp(s);
for i = max(S.hfirst, S.hn - z.inputs) + 1:S.hn
    h = S.hin(mod(i - 1, z.inputs) + 1);
    dup = dup || (h.sensor == s && h.stamp == E.stamp);
end
if dup
    S.F.cnt(k.c_duplicate) = S.F.cnt(k.c_duplicate) + 1;
    S.F.cnt(k.c_duplicate_by + s - 1) = S.F.cnt(k.c_duplicate_by + s - 1) + 1;
    return;
end
c = 0;
for i = S.ck_total:-1:S.ck_first
    if S.hck(mod(i - 1, z.checkpoints) + 1).t <= E.stamp
        c = i;
        break;
    end
end
if c > 0 && S.hck(mod(c - 1, z.checkpoints) + 1).idx >= S.hn - z.inputs
    S = replay_late(S, p, c, E, ekf);
    return;
end
if s ~= 3 && s ~= 4
    S.F.cnt(k.c_stale) = S.F.cnt(k.c_stale) + 1;
    S.F.cnt(k.c_stale_by + s - 1) = S.F.cnt(k.c_stale_by + s - 1) + 1;
    return;
end

% whatever changed since the stamp is unknown, so widen R by what it plausibly was
x = F.x;
P = F.P;
dens = p.estimator.process_noise_diag;
if s == 3
    for i = 1:3
        E.cov(i, i) = E.cov(i, i) + lag * lag * (x(13 + i)^2 + P(13 + i, 13 + i)) + dens(7 + i) * lag;
    end
else
    R = talos_nav_rotation(x(4:7));
    vz = R(3, :) * x(8:10);
    pv = max([P(8, 8), P(9, 9), P(10, 10)]);
    E.cov(1, 1) = E.cov(1, 1) + lag * lag * (vz * vz + pv) + dens(3) * lag;
end
S.late_stamp(s) = E.stamp;
S.n(k.n_late_fallbacks + s - 1) = S.n(k.n_late_fallbacks + s - 1) + 1;
E.stamp = F.tf;
S = talos_nav_process(S, p, E, ekf);
S = talos_nav_history(S, p, 'checkpoint', false);
end

function S = replay_late(S, p, c, late, ekf)
% Restore checkpoint c, merge the late event into the logged inputs after it and rerun them.
% Counters keep their old values plus whatever the late event did.
k = talos_nav_counters();
z = talos_nav_sizes();
s = late.sensor;
ck = S.hck(mod(c - 1, z.checkpoints) + 1);
m0 = S.hn - ck.idx;
pos = m0 + 1;
for i = 1:m0
    h = S.hin(mod(ck.idx + i - 1, z.inputs) + 1);
    if earlier(late, h)
        pos = i;
        break;
    end
end
for i = 1:pos - 1
    S.scratch(i) = S.hin(mod(ck.idx + i - 1, z.inputs) + 1);
end
S.scratch(pos) = late;
for i = pos:m0
    S.scratch(i + 1) = S.hin(mod(ck.idx + i - 1, z.inputs) + 1);
end
m = m0 + 1;

base = S.F.cnt;
from = base;
to = base;
S.F = ck.F;
S.hn = ck.idx;
S.ck_total = c;
for i = 1:m
    if i == pos
        from = S.F.cnt;
    end
    S = talos_nav_process(S, p, S.scratch(i), ekf);
    if i == pos
        to = S.F.cnt;
    end
    S = talos_nav_history(S, p, 'checkpoint', false);
end
S.F.cnt = base + (to - from);
S.n(k.n_lagged_replays + s - 1) = S.n(k.n_lagged_replays + s - 1) + 1;
S.n(k.n_replayed) = S.n(k.n_replayed) + m;
end

function [S, out] = publish(S, p, stamp, ekf)
% Propagate the estimate to stamp, rate-limit its position correction and fill the outputs.
out = talos_nav_output();
S.F = settle_outage(S, p, stamp, ekf);
F = S.F;
if p.timing.publication_mode == 2
    [S, F, ok] = preview(S, p, stamp, ekf);
    if ~ok
        return;
    end
end
x = F.x;
P = F.P;
remaining = max(0, double(stamp - F.tf) * 1e-9);
if p.timing.publication_mode ~= 2
    S.pub_prediction = remaining * 1000;
end
dens = [p.estimator.process_noise_diag(:); p.estimator.imu_acceleration_bias_density * ones(3, 1)];
resetd = [p.estimator.reset_covariance_diag(:); p.estimator.imu_acceleration_bias_variance * ones(3, 1)];
while remaining > 1e-9
    dt = min(remaining, p.timing.max_prediction_dt);
    [x, P] = talos_nav_coast(F, p, x, P, stamp - int64(round(remaining * 1e9)), dt);
    Sq = talos_process_root(x, P, dt, dens, p.estimator.imu_acceleration_bias_rotation_variance, resetd);
    [x, P] = ekf(x, P, dt, Sq, 0, zeros(9, 1), eye(9), zeros(12, 1));
    remaining = remaining - dt;
end
[x, P] = talos_nav_normalize_state(x, P, ref_state(S));
[S, x, P] = limit_position(S, p, stamp, x, P);
S.pub_x = x;
S.has_pub = true;

out.valid = true;
out.stamp = stamp;
out.x = x;
out.P = P;
out.pose_cov = talos_nav_pose_covariance('ros', x, P);
end

function F = settle_outage(S, p, stamp, ekf)
% With every sensor lost nothing moves the filter's time, and each publication would predict the
% whole outage again. Commit up to the lag behind stamp, so late samples can still fuse.
F = S.F;
lag = p.timing.outage_commit_lag;
if lag <= 0 || ~F.initialized || S.qn > 0 || F.tf == 0
    return;
end
to = stamp - int64(round(lag * 1e9));
ages = double(to) - double(F.last_proc(1:4));
if to <= F.tf || any(F.last_proc(1:4) == 0) || any(ages * 1e-9 <= p.timing.sensor_timeouts(:))
    return;
end
dens = [p.estimator.process_noise_diag(:); p.estimator.imu_acceleration_bias_density * ones(3, 1)];
resetd = [p.estimator.reset_covariance_diag(:); p.estimator.imu_acceleration_bias_variance * ones(3, 1)];
remaining = double(to - F.tf) * 1e-9;
P0 = F.P;
while remaining > 1e-9
    dt = min(remaining, p.timing.max_prediction_dt);
    [F.x, F.P] = talos_nav_coast(F, p, F.x, F.P, F.tf, dt);
    Sq = talos_process_root(F.x, P0, dt, dens, p.estimator.imu_acceleration_bias_rotation_variance, resetd);
    [F.x, F.P] = ekf(F.x, F.P, dt, Sq, 0, zeros(9, 1), eye(9), zeros(12, 1));
    F.tf = F.tf + int64(round(dt * 1e9));
    remaining = double(to - F.tf) * 1e-9;
end
F = talos_nav_install(S, F, F.x, F.P);
end

function ref = ref_state(S)
if S.has_pub
    ref = S.pub_x;
else
    ref = zeros(0, 1);
end
end

function [S, F, ok] = preview(S, p, stamp, ekf)
% Run the queued observations up to stamp on a copy of the filter, then drop the copy.
saved = S.F;
S.recording = false;
limit = double(p.timing.max_measurements_per_cycle);
if limit == 0
    limit = inf;
end
i = 1;
count = 0;
while i <= S.qn && S.q(slot(S, i)).stamp <= stamp && count < limit
    S = talos_nav_process(S, p, S.q(slot(S, i)), ekf);
    i = i + 1;
    count = count + 1;
end
S.preview_events = count;
S.preview_limited = i <= S.qn && S.q(slot(S, i)).stamp <= stamp;
F = S.F;
ok = F.initialized;
S.pub_prediction = max(0, double(stamp - F.tf) * 1e-9) * 1000;
if ok
    dens = [p.estimator.process_noise_diag(:); p.estimator.imu_acceleration_bias_density * ones(3, 1)];
    resetd = [p.estimator.reset_covariance_diag(:); p.estimator.imu_acceleration_bias_variance * ones(3, 1)];
    remaining = double(stamp - F.tf) * 1e-9;
    P0 = F.P;
    while remaining > 1e-9
        dt = min(remaining, p.timing.max_prediction_dt);
        [F.x, F.P] = talos_nav_coast(F, p, F.x, F.P, F.tf, dt);
        Sq = talos_process_root(F.x, P0, dt, dens, p.estimator.imu_acceleration_bias_rotation_variance, resetd);
        [F.x, F.P] = ekf(F.x, F.P, dt, Sq, 0, zeros(9, 1), eye(9), zeros(12, 1));
        F.tf = F.tf + int64(round(dt * 1e9));
        remaining = double(stamp - F.tf) * 1e-9;
    end
    F = talos_nav_install(S, F, F.x, F.P);
end
S.F = saved;
S.recording = true;
end

function [S, x, P] = limit_position(S, p, stamp, x, P)
% A large horizontal correction, such as DVL reacquisition after dead reckoning, reaches a
% position controller as a setpoint step. Publish it at a bounded rate and carry the rest in
% the covariance. Depth is measured directly and stays as estimated.
k = talos_nav_counters();
dt = double(stamp - S.last_pub) * 1e-9;
rate = p.timing.max_position_correction_rate;
if rate == 0 || ~S.has_pub || S.snap || S.last_pub == 0 || dt <= 0 || dt > p.timing.position_correction_max_gap
    S.pub_offset = zeros(2, 1);
    S.snap = false;
    return;
end
R = talos_nav_rotation(x(4:7));
predicted = S.pub_x(1:2) + R(1:2, :) * x(8:10) * dt;
correction = x(1:2) - predicted;
nrm = sqrt(sum(correction .^ 2));
limit = rate * dt;
scale = 1;
if nrm > limit
    scale = limit / nrm;
    S.n(k.n_limited) = S.n(k.n_limited) + 1;
end
published = predicted + scale * correction;
S.pub_offset = x(1:2) - published;
x(1:2) = published;
% the published position is off the estimate by the offset
P(1:2, 1:2) = P(1:2, 1:2) + S.pub_offset * S.pub_offset';
end
