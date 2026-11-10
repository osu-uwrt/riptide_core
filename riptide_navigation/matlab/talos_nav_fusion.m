function [pub, x, P, poseCov, d, taken] = talos_nav_fusion(p, ring, now, started, doPublish, doDiag, ekf)
%TALOS_NAV_FUSION Body of the model's Fusion block: queue the new samples, fuse them, publish.
% ring comes from the input task (see talos_nav_ring_write), which taken acknowledges, and the
% cadence flags from the schedule. The
% estimator restarts once the overseer marks the parameters loaded (started), and ekf is the
% Extended Kalman Filter block, see talos_nav_ekf.
%#codegen
persistent S running seen
if isempty(S)
    S = talos_nav_init(p);
    running = false;
    seen = 0;
end
pub = false;
x = zeros(19, 1);
P = zeros(19);
poseCov = zeros(6);
d = zeros(numel(talos_nav_diag(S, p)), 1);
taken = seen;
newest = ring(1) + ring(2);
if ~started
    seen = newest;
    taken = seen;
    return;
end
if ~running
    S = talos_nav_init(p);
    running = true;
end

S.step = S.step + 1;
k = talos_nav_counters();
w = talos_nav_ring_width();
S.n(k.n_ring_lost) = ring(3);
seen = max(seen, ring(1));
while seen < newest
    first = 3 + (seen - ring(1)) * w;
    S = talos_nav_arrive(S, p, talos_nav_raw_unpack(ring(first + 2:first + w)), ...
        typecast(ring(first + 1), 'int64'));
    seen = seen + 1;
end
taken = seen;
[S, out] = talos_nav_tick(S, p, now, doPublish, ekf);
if out.valid
    pub = true;
    x = out.x;
    P = out.P;
    poseCov = out.pose_cov;
end
if doDiag
    d = talos_nav_diag(S, p);
end
end
