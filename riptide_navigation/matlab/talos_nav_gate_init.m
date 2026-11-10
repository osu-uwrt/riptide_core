function g = talos_nav_gate_init
%TALOS_NAV_GATE_INIT Fresh innovation gate, reacquiring.

w = talos_nav_sizes().window;
g.mode = 1;
g.has_prior = false;
g.has_last_value = false;
g.last_accept = int64(0);
g.rejection_start = int64(0);
g.garbage_mark = int64(0);
g.last_flap = int64(0);
g.count = 0;
g.n = 1;
g.stamp = zeros(1, w, 'int64');
g.value = zeros(3, w);
g.variance = zeros(3, w);
g.last_value = zeros(3, 1);
g.frozen = 0;
g.garbage_windows = 0;
end
