function ring = talos_nav_ring_write(raw, now, seen)
%TALOS_NAV_RING_WRITE Body of the input task: keep every sample set, stamped with its arrival.
% A Subscribe block only holds its latest message, so this task must stay short. The estimator task
% can then take as long as it needs and catch up. Each step offers it the oldest sets it has not
% acknowledged (seen), a few at a time, so a long ring costs nothing to carry across.
%#codegen
persistent R writes out lost
z = talos_nav_sizes();
w = talos_nav_ring_width();
if isempty(R)
    R = zeros(z.ring * w, 1);
    writes = 0;
    out = talos_nav_ring();
    lost = 0;
end
if raw.imu.valid || raw.fog.valid || raw.dvl.valid || raw.depth.valid || raw.set_pose.valid
    first = mod(writes, z.ring) * w;
    R(first + 1) = typecast(now, 'double');
    R(first + 2:first + w) = talos_nav_raw_pack(raw);
    writes = writes + 1;
end
from = max(seen, writes - z.ring);
count = min(writes - from, z.ring_window);
if seen < from
    lost = from - seen;
end
out(1) = from;
out(2) = count;
out(3) = lost;
for k = 1:count
    src = mod(from + k - 1, z.ring) * w;
    dst = 3 + (k - 1) * w;
    out(dst + 1:dst + w) = R(src + 1:src + w);
end
ring = out;
end
