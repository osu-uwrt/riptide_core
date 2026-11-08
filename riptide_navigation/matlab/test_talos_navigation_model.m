% Model assertions: the estimator functions the generated model runs, on synthetic
% inputs, then a compile of models/talos_navigation. Plain asserts.
here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here, '..', 'models'));
p = talos_nav_params(fullfile(here, '..', 'config', 'talos_navigation.yaml'));
p.frames = rmfield(p.frames, {'world_frame', 'base_link_frame', 'imu_frame_fallback', ...
    'fog_frame_fallback', 'dvl_frame_fallback'});
for s = {'imu', 'fog', 'dvl'}
    p.mounts.([s{1} '_mount_valid']) = 1;
    p.mounts.([s{1} '_translation']) = zeros(3, 1);
    p.mounts.([s{1} '_rotation']) = [1; 0; 0; 0];
end
k = talos_nav_counters();
dvlSlot = 3;

% 0-10 s at rest, 10-20 s DVL dropout, 20-30 s DVL back, 30-40 s DVL at 0.2 m/s forward,
% 40-45 s at rest with NaN DVL samples
% DVL and depth carry noise; identical DVL samples are rejected as a frozen sensor
rng(20261028);
t0 = int64(1e18);
sec = @(t) t0 + int64(round(t * 1e9));
S = talos_nav_init(p);
ticks = 0:1/30:45;
log = struct('t', {}, 'x', {}, 'P', {}, 'cnt', {});
next = struct('imu', 0, 'fog', 0, 'dvl', 0, 'depth', 0);
rates = struct('imu', 200, 'fog', 200, 'dvl', 8, 'depth', 10);
names = fieldnames(rates);
for tick = ticks
    % deliver every sample stamped up to this tick, in stamp order
    while true
        due = cellfun(@(n) next.(n), names);
        [first, which] = min(due);
        if first > tick
            break;
        end
        name = names{which};
        next.(name) = first + 1 / rates.(name);
        r = sample(name, first, sec);
        if ~isempty(r)
            S = talos_nav_arrive(S, p, r, sec(first) + int64(1e6));
        end
    end
    [S, out] = talos_nav_tick(S, p, sec(tick) + int64(2e6), true, @talos_filter_apply);
    if out.valid
        log(end + 1) = struct('t', tick, 'x', out.x, 'P', out.pose_cov, 'cnt', S.F.cnt); %#ok<SAGROW>
    end
end

t = [log.t];
x = [log.x];
assert(numel(t) > 0.9 * numel(ticks), 'too few publications: %d', numel(t));
assert(all(isfinite(x), 'all'), 'nonfinite estimate');
maxQuaternionNormError = max(abs(vecnorm(x(4:7, :)) - 1));
assert(maxQuaternionNormError < 1e-9);
minEig = inf;
maxAsymmetry = 0;
for i = 1:numel(log)
    minEig = min(minEig, min(eig((log(i).P + log(i).P') / 2)));
    maxAsymmetry = max(maxAsymmetry, max(abs(log(i).P - log(i).P'), [], 'all'));
end
assert(minEig > -1e-12 && maxAsymmetry < 1e-12);

at = @(s) find(t <= s, 1, 'last');
restDrift = norm(x(1:3, at(10)));
assert(restDrift < 0.02, 'rest drift %.4f m', restDrift);
tilt = rad2deg(2 * acos(min(1, abs(x(4, at(10))))));
assert(tilt < 0.5, 'rest attitude %.3f deg', tilt);

% dropout: covariance grows, no DVL accepted, position held near rest
sigmaBefore = sqrt(log(at(10)).P(1, 1));
sigmaDuring = sqrt(log(at(19.9)).P(1, 1));
assert(sigmaDuring > 1.5 * sigmaBefore, 'sigma %.4f -> %.4f', sigmaBefore, sigmaDuring);
dvlAcceptedDuring = log(at(19.9)).cnt(k.c_accepted + dvlSlot - 1) - log(at(10.2)).cnt(k.c_accepted + dvlSlot - 1);
assert(dvlAcceptedDuring == 0);
dropoutDrift = norm(x(1:3, at(20)) - x(1:3, at(10)));
assert(dropoutDrift < 0.5, 'dropout drift %.3f m', dropoutDrift);

% reacquire: DVL accepted again and velocity back at rest
dvlAcceptedAfter = log(at(29.9)).cnt(k.c_accepted + dvlSlot - 1) - log(at(20.2)).cnt(k.c_accepted + dvlSlot - 1);
assert(dvlAcceptedAfter > 60, 'dvl accepted after the gap: %d', dvlAcceptedAfter);
restSpeed = norm(x(8:10, at(29.9)));
assert(restSpeed < 0.02, 'speed after reacquire %.4f m/s', restSpeed);

% forward motion integrates to about 2 m along x
travel = x(1:3, at(40)) - x(1:3, at(30));
assert(abs(travel(1) - 2) < 0.2 && norm(travel(2:3)) < 0.1, ...
    'travel [%.3f %.3f %.3f]', travel);

% NaN DVL samples are counted invalid, not fused
invalid = log(end).cnt(k.c_invalid_by + dvlSlot - 1) - log(at(40)).cnt(k.c_invalid_by + dvlSlot - 1);
assert(invalid >= 30, 'invalid dvl samples %d', invalid);
assert(all(isfinite(x(:, end))));

% a pose reset restarts the motion from rest and keeps the learned bias
S.F.x(8:10) = [0.3; -0.2; 0.1];
bias = S.F.x(17:19);
r = talos_nav_raw();
r.set_pose.valid = true;
r.set_pose.stamp = S.now;
r.set_pose.pos = [1; 2; -1];
r.set_pose.quat = [1; 0; 0; 0];
S = talos_nav_arrive(S, p, r, S.now);
assert(isequal(S.F.x(1:3), [1; 2; -1]) && all(S.F.x(8:16) == 0) && isequal(S.F.x(17:19), bias));

% a lost IMU decays the held motion, and a live one changes nothing
F = S.F;
F.last_proc(1:4) = S.now;
F.x(8:16) = 1;
[xc, Pc] = talos_nav_coast(F, p, F.x, F.P, S.now, 0.1);
assert(isequal(xc, F.x) && isequal(Pc, F.P));
F.last_proc(1) = S.now - int64(1e9);
[xc, Pc] = talos_nav_coast(F, p, F.x, F.P, S.now, 0.1);
assert(all(xc([11, 12, 14, 15, 16]) < 1) && all(xc([8:10, 13]) == 1) && all(eig(0.5 * (Pc + Pc')) > -1e-12));
F.last_proc(2:3) = S.now - int64(1e9);
xc = talos_nav_coast(F, p, F.x, F.P, S.now, 0.1);
assert(all(xc(8:16) < 1));

% samples survive the flat ring between the two tasks, stamps bit for bit
packed = talos_nav_raw();
packed.imu.valid = true;
packed.imu.stamp = int64(1793773804040807778);
packed.imu.cov_a = [1 2 3; 4 5 6; 7 8 9];
packed.set_pose.cov = reshape(1:36, 6, 6);
assert(isequaln(talos_nav_raw_unpack(talos_nav_raw_pack(packed)), packed), 'ring slot round trip');

% the generated model compiles with its buses and the same functions
talos_navigation_setup;
model = 'talos_navigation';
load_system(fullfile(here, '..', 'models', model, [model '.slx']));
cleanup = onCleanup(@() close_system(model, 0));
set_param(model, 'SimulationCommand', 'update');

% diagram rules: right-angle lines, none over blocks, no overlaps or loose ports
issues = check_talos_navigation_layout(model);
assert(isempty(issues), 'layout: %s', strjoin(issues, '; '));

% the model filters with the library Extended Kalman Filter block
filter = [model '/Estimator/Extended_Kalman_Filter'];
assert(strcmp(get_param([filter '/EKF'], 'ReferenceBlock'), 'sharedTrackingLibrary/Extended Kalman Filter'));

% the block, driven as the estimator drives it, against the reference filter
harness = 'talos_nav_ekf_harness';
new_system(harness);
closeHarness = onCleanup(@() bdclose(harness));
set_param(harness, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', 'FixedStep', '1', 'StopTime', '599');
add_block(filter, [harness '/Extended_Kalman_Filter']);
add_block('simulink/Sources/Counter Free-Running', [harness '/k'], 'NumBits', '16', 'tsamp', '1');
add_block('simulink/Signal Attributes/Data Type Conversion', [harness '/step'], 'OutDataTypeStr', 'double');
add_block('simulink/User-Defined Functions/MATLAB Function', [harness '/drive']);
chart = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', [harness '/drive']);
chart.Script = ['function err = drive(k)' newline 'err = ekf_sequence(@ekf, k);' newline 'end' newline ...
    'function [x, P] = ekf(x, P, dt, Sq, s, y, R, c)' newline ...
    '[x, P] = talos_nav_ekf(@step, x, P, dt, Sq, s, y, R, c);' newline 'end' newline ...
    'function [x, P] = step(m, dt, Sq, x0, s, y, R, c)' newline ...
    '[x, P] = talos_nav_ekf_step(m, dt, Sq, x0, s, y, R, c);' newline 'end' newline];
add_block('simulink/Sinks/To Workspace', [harness '/log'], 'VariableName', 'err', 'SaveFormat', 'Array');
add_line(harness, 'k/1', 'step/1');
add_line(harness, 'step/1', 'drive/1');
add_line(harness, 'drive/1', 'log/1');
run = sim(harness);
blockError = run.err(end);
assert(blockError < 1e-9, 'EKF block vs reference filter %.3g', blockError);

fprintf('publications=%d\n', numel(t));
fprintf('max_quaternion_norm_error=%.12g\n', maxQuaternionNormError);
fprintf('minimum_covariance_eigenvalue=%.12g\n', minEig);
fprintf('rest_drift_m=%.6f rest_tilt_deg=%.4f\n', restDrift, tilt);
fprintf('dropout_sigma_m=%.4f->%.4f dropout_drift_m=%.4f\n', sigmaBefore, sigmaDuring, dropoutDrift);
fprintf('dvl_accepted_after=%d rest_speed_mps=%.5f\n', dvlAcceptedAfter, restSpeed);
fprintf('travel_m=[%.4f %.4f %.4f] invalid_dvl=%d\n', travel, invalid);
fprintf('ekf_block_vs_reference=%.3g\n', blockError);

function r = sample(name, t, sec)
% one synthetic message; empty during the DVL dropout
r = talos_nav_raw();
stamp = sec(t);
switch name
    case 'imu'
        r.imu.valid = true;
        r.imu.stamp = stamp;
        r.imu.frame = 1;
        r.imu.orientation = [0; 0; 0; 1];
        r.imu.accel = [0; 0; 9.755455];
        r.imu.cov_o = 1e-4 * eye(3);
        r.imu.cov_w = 2e-5 * eye(3);
        r.imu.cov_a = 2e-3 * eye(3);
    case 'fog'
        r.fog.valid = true;
        r.fog.stamp = stamp;
        r.fog.frame = 1;
        r.fog.cov = diag([1e-6, 1e-6, 1e-6]);
    case 'dvl'
        if t >= 10 && t < 20
            r = [];
            return;
        end
        r.dvl.valid = true;
        r.dvl.stamp = stamp;
        r.dvl.frame = 1;
        r.dvl.cov = 2.5e-5 * eye(3);
        r.dvl.vel = 0.005 * randn(3, 1);
        if t >= 30 && t < 40
            r.dvl.vel(1) = r.dvl.vel(1) + 0.2;
        elseif t >= 40
            r.dvl.vel = [nan; 0; 0];
        end
    otherwise
        r.depth.valid = true;
        r.depth.stamp = stamp;
        r.depth.frame = 2;
        r.depth.z = 0.01 * randn;
        r.depth.var = 1e-4;
end
end
