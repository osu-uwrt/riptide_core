function S = talos_nav_arrive(S, p, raw, now)
%TALOS_NAV_ARRIVE Take the samples that arrived at time now (ns) into the reorder queue.
% Each sample becomes an event in the base frame, with its arrival checks. A pose reset request
% restarts the filter at once.
%#codegen
S.now = now;

if raw.imu.valid
    S = take(S, p, 1, raw.imu);
end
if raw.fog.valid
    S = take(S, p, 2, raw.fog);
end
if raw.dvl.valid
    S = take(S, p, 3, raw.dvl);
end
if raw.depth.valid
    S = take(S, p, 4, raw.depth);
end
if raw.set_pose.valid
    S = set_pose(S, p, raw.set_pose);
end
end

function S = take(S, p, sensor, r)
[S, E, ok] = talos_nav_ingest(S, p, sensor, r);
if ok
    S = enqueue(S, p, E);
end
end

function i = slot(S, n)
% Ring slot of the n-th queued event, counting from the front.
i = mod(S.qhead + n - 2, numel(S.q)) + 1;
end

function S = enqueue(S, p, E)
% Arrival checks, then an ordered insert by (stamp, sequence). A full sensor drops its own oldest.
k = talos_nav_counters();
s = E.sensor;
E.arrival = S.now;
age = double(S.now - E.stamp) * 1e-6;
S.arrival_age(s) = age;
if S.arrival_count(s) == 0
    S.arrival_min(s) = age;
    S.arrival_max(s) = age;
end
S.arrival_count(s) = S.arrival_count(s) + 1;
S.arrival_min(s) = min(S.arrival_min(s), age);
S.arrival_max(s) = max(S.arrival_max(s), age);
S.arrival_valid(s) = E.stamp > 0;
if E.stamp <= 0 || ~(all(isfinite(E.value)) && all(isfinite(E.cov(:))) && all(isfinite(E.offset)))
    S.F.cnt(k.c_invalid) = S.F.cnt(k.c_invalid) + 1;
    S.F.cnt(k.c_invalid_by + s - 1) = S.F.cnt(k.c_invalid_by + s - 1) + 1;
    return;
end
S.n(k.n_arrivals + s - 1) = S.n(k.n_arrivals + s - 1) + 1;
if E.arrival == 0
    % no /clock yet, so nothing can be ordered against a zero watermark
    S.n(k.n_pre_clock + s - 1) = S.n(k.n_pre_clock + s - 1) + 1;
    return;
end
if double(E.stamp - E.arrival) * 1e-9 > p.timing.max_stamp_lead
    S.n(k.n_future + s - 1) = S.n(k.n_future + s - 1) + 1;
    return;
end
S.seq = S.seq + 1;
E.seq = S.seq;

if S.qcount(s) >= p.timing.max_queue_size
    for i = 1:S.qn
        if S.q(slot(S, i)).sensor == s
            for j = i:S.qn - 1
                S.q(slot(S, j)) = S.q(slot(S, j + 1));
            end
            S.qn = S.qn - 1;
            S.qcount(s) = S.qcount(s) - 1;
            break;
        end
    end
    S.n(k.n_queue) = S.n(k.n_queue) + 1;
    S.n(k.n_queue_by + s - 1) = S.n(k.n_queue_by + s - 1) + 1;
end
pos = S.qn + 1;
while pos > 1 && earlier(E, S.q(slot(S, pos - 1)))
    pos = pos - 1;
end
for j = S.qn:-1:pos
    S.q(slot(S, j + 1)) = S.q(slot(S, j));
end
S.q(slot(S, pos)) = E;
S.qn = S.qn + 1;
S.qcount(s) = S.qcount(s) + 1;
end

function t = earlier(a, b)
if a.stamp == b.stamp
    t = a.seq < b.seq;
else
    t = a.stamp < b.stamp;
end
end

function S = set_pose(S, p, r)
% Validate a pose reset request, then restart the filter there with the learned bias kept.
k = talos_nav_counters();
if r.frame == 1 || r.frame == 3
    S.n(k.n_transform) = S.n(k.n_transform) + 1;
    S = reject(S, 5);
    return;
end
C = r.cov;
if ~all(isfinite(C(:))) || any(any(abs(C - C') > 1e-9))
    S.F.cnt(k.c_invalid) = S.F.cnt(k.c_invalid) + 1;
    S = reject(S, 1);
    return;
end
if ~psd(C)
    S.F.cnt(k.c_invalid) = S.F.cnt(k.c_invalid) + 1;
    S = reject(S, 2);
    return;
end
% the motion restarts from rest; only the learned bias carries over
x = [r.pos(:); r.quat(:); zeros(9, 1); S.F.x(17:19)];
n = sqrt(sum(r.quat(:) .^ 2));
if ~all(isfinite(x))
    S.F.cnt(k.c_invalid) = S.F.cnt(k.c_invalid) + 1;
    S = reject(S, 3);
    return;
end
if abs(n - 1) > p.estimator.initial_quaternion_tolerance || n < 1e-12
    S.F.cnt(k.c_invalid) = S.F.cnt(k.c_invalid) + 1;
    S = reject(S, 4);
    return;
end
S = talos_nav_reset(S, p, x, S.now, C);
end

function S = reject(S, code)
k = talos_nav_counters();
S.n(k.n_set_pose) = S.n(k.n_set_pose) + 1;
S.set_pose_error = code;
end

function ok = psd(C)
% Positive semidefinite after scaling by the standard deviations. Zero variance needs a zero row.
d = diag(C);
ok = all(d >= 0);
if ~ok
    return;
end
sc = sqrt(d);
live = sc > 0;
if any(any(C(~live, :) ~= 0))
    ok = false;
    return;
end
M = C(live, live) ./ (sc(live) * sc(live)');
ok = all(eig(0.5 * (M + M')) >= -256 * eps * numel(sc));
end
