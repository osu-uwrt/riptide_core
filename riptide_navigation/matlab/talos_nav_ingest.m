function [S, E, ok] = talos_nav_ingest(S, p, sensor, raw)
%TALOS_NAV_INGEST Turn a raw sample into a measurement event in the base frame.
% frame: 0 empty with no fallback, 1 base link, 2 the sensor's mount frame, 3 any other,
% 4 empty with a fallback frame. Mounts come from p.mounts, which the overseer fills from TF.
%#codegen
E = talos_nav_event();
E.stamp = raw.stamp;
E.sensor = sensor;
ok = false;
k = talos_nav_counters();
c = p.sensor_axes;
switch sensor
    case 1
        [S, good, off, qbs] = mount(S, p, 1, raw.frame);
        if ~good
            return;
        end
        qbs = qbs / sqrt(sum(qbs .^ 2));
        rbs = talos_nav_rotation(qbs);
        qws = [raw.orientation(4); raw.orientation(1:3)];
        len2 = sum(qws .^ 2);
        unit = isfinite(len2) && abs(sqrt(len2) - 1) <= p.estimator.imu_quaternion_norm_tolerance;
        if raw.cov_o(1, 1) ~= -1 && ~unit
            S.n(k.n_imu_quaternions) = S.n(k.n_imu_quaternions) + 1;
        end
        groups = [raw.cov_o(1, 1) ~= -1 && unit; ...
            raw.cov_w(1, 1) ~= -1 && all(isfinite(raw.omega)); ...
            raw.cov_a(1, 1) ~= -1 && all(isfinite(raw.accel))];
        if ~groups(2) && sum(off .^ 2) > 1e-24
            % a mounted accelerometer needs rates to remove its rotational acceleration
            groups(3) = false;
        end
        S.imu_groups = double(groups);
        for i = 1:3
            if ~groups(i)
                S.n(k.n_imu_unavailable + i - 1) = S.n(k.n_imu_unavailable + i - 1) + 1;
            end
        end
        cfg = double(c.vectornav_imu_config(:));
        if ~any([groups(1) * cfg(4:6); groups(2) * cfg(10:12); groups(3) * cfg(13:15)] ~= 0)
            return;
        end
        if ~groups(1)
            qws = [1; 0; 0; 0];
        end
        qws = qws / sqrt(sum(qws .^ 2));
        qwb = quat_mul(qws, [qbs(1); -qbs(2:4)]);
        qwb = qwb / sqrt(sum(qwb .^ 2));
        gb = talos_nav_rotation(qwb)' * [0; 0; 1];
        omega = zeros(3, 1);
        accel = zeros(3, 1);
        if groups(2)
            omega = rbs * raw.omega(:);
        end
        if groups(3)
            accel = rbs * raw.accel(:);
        end
        E.groups = double(groups);
        E.offset = off;
        E.value = [gb; omega; accel];

        co = cov_value(raw.cov_o, c.default_orientation_variance);
        cw = cov_value(raw.cov_w, c.default_angular_velocity_variance);
        ca = cov_value(raw.cov_a, c.default_acceleration_variance);
        im = c.imu_covariance_multiplier;
        co = condition(co, im * c.imu_orientation_covariance_multiplier, c.covariance_floor);
        cw = condition(cw, im * c.imu_rate_covariance_multiplier, c.covariance_floor);
        ca = condition(ca, im * c.imu_acceleration_covariance_multiplier, c.covariance_floor);
        if p.sensor_axes.imu_orientation_covariance_coordinates == 2
            Rw = talos_nav_rotation(qws);
            roll = atan2(Rw(3, 2), Rw(3, 3));
            pitch = atan2(-Rw(3, 1), sqrt(Rw(3, 2)^2 + Rw(3, 3)^2));
            e = [1 0 -sin(pitch); 0 cos(roll) sin(roll) * cos(pitch); 0 -sin(roll) cos(roll) * cos(pitch)];
            co = e * co * e';
        end
        co = rbs * co * rbs';
        cw = rbs * cw * rbs';
        ca = rbs * ca * rbs';
        % J = -skew(g): gravity covariance drops yaw about gravity on its own
        J = [0 gb(3) -gb(2); -gb(3) 0 gb(1); gb(2) -gb(1) 0];
        gc = J * co * J' + c.gravity_direction_variance_floor * eye(3);
        tc = J * gc * J' + c.covariance_floor * eye(3);
        E.cov(1:3, 1:3) = axis_scale(tc, c.imu_orientation_axis_multiplier);
        E.cov(4:6, 4:6) = axis_scale(cw, c.imu_angular_velocity_axis_multiplier);
        E.cov(7:9, 7:9) = axis_scale(ca, c.imu_acceleration_axis_multiplier);
    case 2
        [S, good, ~, q] = mount(S, p, 2, raw.frame);
        if ~good
            return;
        end
        R = talos_nav_rotation(q);
        E.value(1:3) = R * raw.omega(:);
        cf = raw.cov;
        d = c.default_fog_variance * eye(3);
        m = ~isfinite(cf);
        cf(m) = d(m);
        % a negative variance is an unavailable sentinel, not a near-zero variance
        if all(cf(:) == 0) || cf(1, 1) < 0 || cf(2, 2) < 0 || cf(3, 3) < 0
            cf = d;
        end
        cf = condition(cf, c.fog_covariance_multiplier, c.covariance_floor);
        E.cov(1:3, 1:3) = axis_scale(R * cf * R', c.fog_axis_multiplier);
    case 3
        [S, good, off, q] = mount(S, p, 3, raw.frame);
        if ~good
            return;
        end
        R = talos_nav_rotation(q);
        E.value(1:3) = R * raw.vel(:);
        E.offset = off;
        cd = raw.cov;
        d = c.default_dvl_variance * eye(3);
        m = ~isfinite(cd);
        cd(m) = d(m);
        if all(cd(:) == 0) || cd(1, 1) < 0 || cd(2, 2) < 0 || cd(3, 3) < 0
            cd = d;
        else
            % losing bottom lock shows as a sigma jump a few samples early
            [S, ratio] = sigma_ratio(S, p, sqrt((cd(1, 1) + cd(2, 2) + cd(3, 3)) / 3));
            if p.gates.dvl_sigma_jump_ratio > 0 && ratio > p.gates.dvl_sigma_jump_ratio
                cd = cd * ratio * ratio;
                S.n(k.n_dvl_degraded) = S.n(k.n_dvl_degraded) + 1;
            end
        end
        cd = condition(cd, c.dvl_covariance_multiplier, c.covariance_floor);
        cd = condition(R * cd * R', 1, c.covariance_floor);
        E.cov(1:3, 1:3) = axis_scale(cd, c.dvl_axis_multiplier);
    otherwise
        if raw.frame == 1 || raw.frame == 3
            % absolute depth can't be a body-frame value, and R2023a has no Apply Transform
            S.n(k.n_transform) = S.n(k.n_transform) + 1;
            S.n(k.n_transform_by + 3) = S.n(k.n_transform_by + 3) + 1;
            return;
        end
        E.value(1) = raw.z;
        v = raw.var;
        if ~(isfinite(v) && v > 0)
            v = c.default_depth_variance;
        end
        E.cov(1, 1) = max(v * c.depth_covariance_multiplier, c.covariance_floor);
end
ok = true;
end

function [S, ok, off, q] = mount(S, p, s, frame)
% Latest static mount of sensor s, or a counted rejection.
k = talos_nav_counters();
off = zeros(3, 1);
q = [1; 0; 0; 0];
ok = false;
switch s
    case 1
        valid = p.mounts.imu_mount_valid ~= 0;
        off = p.mounts.imu_translation(:);
        q = p.mounts.imu_rotation(:);
    case 2
        valid = p.mounts.fog_mount_valid ~= 0;
        off = p.mounts.fog_translation(:);
        q = p.mounts.fog_rotation(:);
    otherwise
        valid = p.mounts.dvl_mount_valid ~= 0;
        off = p.mounts.dvl_translation(:);
        q = p.mounts.dvl_rotation(:);
end
if frame == 0
    S.n(k.n_empty_frame + s - 1) = S.n(k.n_empty_frame + s - 1) + 1;
    return;
elseif frame == 4
    S.n(k.n_empty_fallback + s - 1) = S.n(k.n_empty_fallback + s - 1) + 1;
end
if frame == 1
    off = zeros(3, 1);
    q = [1; 0; 0; 0];
    ok = true;
elseif frame == 3 || ~valid
    S.n(k.n_transform) = S.n(k.n_transform) + 1;
    S.n(k.n_transform_by + s - 1) = S.n(k.n_transform_by + s - 1) + 1;
else
    ok = true;
end
end

function c = cov_value(m, fallback)
% ROS marks an unavailable estimate with covariance[0] == -1, and zeros mean unspecified.
d = fallback * eye(3);
if m(1, 1) < 0 || all(m(:) == 0)
    c = d;
    return;
end
c = m;
bad = ~isfinite(m);
c(bad) = d(bad);
end

function c = condition(c, multiplier, floorVar)
c = 0.5 * (c + c') * multiplier;
for i = 1:3
    c(i, i) = max(c(i, i), floorVar);
end
for i = 1:3
    off = sum(abs(c(i, :))) - abs(c(i, i));
    c(i, i) = max(c(i, i), off + floorVar);
end
end

function c = axis_scale(c, m)
d = sqrt(m(:));
c = (d * d') .* c;
end

function q = quat_mul(a, b)
q = [a(1)*b(1) - sum(a(2:4) .* b(2:4)); ...
    a(1)*b(2:4) + b(1)*a(2:4) + cross(a(2:4), b(2:4))];
end

function [S, ratio] = sigma_ratio(S, p, sigma)
% Reported sigma over the median of the recent ones, zero until there is enough history.
n = numel(S.dvl_sigma_hist);
count = min(S.dvl_sigma_count, n);
ratio = 0;
if count >= p.gates.dvl_sigma_min_history
    s = sort(S.dvl_sigma_hist(1:count));
    med = s(floor(count / 2) + 1);
    if med > 0
        ratio = sigma / med;
    end
end
S.dvl_sigma_hist(mod(S.dvl_sigma_count, n) + 1) = sigma;
S.dvl_sigma_count = S.dvl_sigma_count + 1;
end
