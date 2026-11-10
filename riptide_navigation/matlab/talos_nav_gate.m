function [g, decision] = talos_nav_gate(g, stamp, value, variance, n, squared, o)
%TALOS_NAV_GATE Innovation gate with outage reacquisition and consistency recovery.
% decision: 0 accept, 1 reject, 2 recover, 3 frozen, 4 hold. g.mode: 1 reacquire, 2 tracking,
% 3 rejecting, 4 recovering. A sample failing the state gate fuses only once the rejected
% samples agree with each other, so a real step is followed and garbage stays out.
%#codegen
decision = 0;
if o.frozen_limit > 0
    if g.has_last_value && all(value == g.last_value)
        g.frozen = g.frozen + 1;
    else
        g.frozen = 1;
    end
    g.last_value = value;
    g.has_last_value = true;
    if g.frozen >= o.frozen_limit
        decision = 3;
        return;
    end
end

fin = isfinite(squared);
limit = o.threshold * o.threshold;
if g.mode ~= 1 && g.mode ~= 3 && g.last_accept ~= 0 && o.gap_timeout > 0 && ...
        double(stamp - g.last_accept) * 1e-9 > o.gap_timeout
    g.mode = 1;
    g.has_prior = true;
    g.count = 0;
end

switch g.mode
    case 1
        gap = 0;
        if g.last_accept ~= 0
            gap = double(stamp - g.last_accept) * 1e-9;
        end
        scale = min(o.max_reacquire_scale, 1 + gap / max(o.gap_timeout, 1e-9));
        if g.has_prior && fin && squared <= limit * scale * scale
            g = accept(g, stamp, 2);
            return;
        end
        needed = max(o.reacquire_samples, 3);
        g = push(g, stamp, value, variance, n, 0, needed);
        if g.count >= needed && consistent(g, o)
            g = accept(g, stamp, 4);
            return;
        end
        decision = 4;
    case 2
        if fin && squared <= limit
            g = accept(g, stamp, 2);
            return;
        end
        g = start_rejecting(g, stamp, value, variance, n);
        decision = 1;
    case 4
        if fin && squared <= limit
            g = accept(g, stamp, 2);
            return;
        end
        g = push(g, stamp, value, variance, n, max(o.recovery_time, o.min_recovery_window), 64);
        if consistent(g, o)
            g = accept(g, stamp, 4);
            return;
        end
        g = start_rejecting(g, stamp, value, variance, n);
        decision = 1;
    otherwise
        if fin && squared <= limit
            % short rejections between good samples are intermittent garbage, not a step
            if double(stamp - g.rejection_start) * 1e-9 < o.recovery_time
                g.last_flap = stamp;
            end
            g = accept(g, stamp, 2);
            return;
        end
        flapping = g.last_flap ~= 0 && double(stamp - g.last_flap) * 1e-9 < o.flap_factor * o.recovery_time;
        required = o.recovery_time;
        if flapping
            required = o.recovery_time * o.flap_factor;
        end
        g = push(g, stamp, value, variance, n, required, 64);
        decision = 1;
        if o.recovery_time <= 0 || double(stamp - g.rejection_start) * 1e-9 < required
            return;
        end
        if g.count >= max(o.recovery_samples, 3) && consistent(g, o)
            g = accept(g, stamp, 4);
            decision = 2;
            return;
        end
        if double(stamp - g.garbage_mark) * 1e-9 >= o.recovery_time
            g.garbage_windows = g.garbage_windows + 1;
            g.garbage_mark = stamp;
        end
end
end

function g = accept(g, stamp, mode)
if mode ~= 4
    g.count = 0;
end
g.mode = mode;
g.last_accept = stamp;
end

function g = start_rejecting(g, stamp, value, variance, n)
g.mode = 3;
g.rejection_start = stamp;
g.garbage_mark = stamp;
g.count = 0;
g = push(g, stamp, value, variance, n, 0, 64);
end

function g = push(g, stamp, value, variance, n, horizon, cap)
% Append a sample, then drop the oldest beyond cap or older than the horizon seconds.
if g.count >= size(g.stamp, 2)
    g = drop(g, 1);
end
g.count = g.count + 1;
g.n = n;
g.stamp(g.count) = stamp;
g.value(:, g.count) = value;
g.variance(:, g.count) = variance;
while g.count > cap || (horizon > 0 && g.count > 1 && double(stamp - g.stamp(1)) * 1e-9 > horizon)
    g = drop(g, 1);
end
end

function g = drop(g, k)
m = g.count;
g.stamp(1:m-k) = g.stamp(1+k:m);
g.value(:, 1:m-k) = g.value(:, 1+k:m);
g.variance(:, 1:m-k) = g.variance(:, 1+k:m);
g.count = m - k;
end

function ok = consistent(g, o)
% Weighted line fit per axis against the sample's own variance, in seconds from the newest.
m = g.count;
ok = false;
if m < 3
    return;
end
floorVar = o.consistency_stddev * o.consistency_stddev;
t = double(g.stamp(1:m) - g.stamp(m)) * 1e-9;
total = 0;
for a = 1:g.n
    w = 1 ./ max(max(g.variance(a, 1:m), floorVar), 1e-12);
    sw = sum(w); st = sum(w .* t); stt = sum(w .* t .* t);
    y = g.value(a, 1:m);
    sy = sum(w .* y); sty = sum(w .* t .* y);
    det = sw * stt - st * st;
    slope = 0;
    if det > 1e-12 * sw * sw
        slope = (sw * sty - st * sy) / det;
    end
    offset = (sy - slope * st) / sw;
    r = y - offset - slope * t;
    total = total + sum(r .* r .* w);
end
dof = (m - 2) * g.n;
ok = isfinite(total) && total <= o.consistency_threshold * o.consistency_threshold * dof;
end
