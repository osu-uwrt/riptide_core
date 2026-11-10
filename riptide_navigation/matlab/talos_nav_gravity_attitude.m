function x = talos_nav_gravity_attitude(prior, g)
%TALOS_NAV_GRAVITY_ATTITUDE Set the two gravity-observable attitude angles, keeping the prior heading.
%#codegen
n = sqrt(sum(g .^ 2));
if ~isfinite(n) || n < 1e-12
    x = prior;
    return;
end
g = g / n;
a = prior;
a(4:7) = [1 + g(3); g(2); -g(1); 0];
[a, ok] = talos_nav_normalize(a, []);
if ~ok
    a(4:7) = [0; 1; 0; 0];
end
[ref, ok] = talos_nav_normalize(prior, []);
if ~ok
    x = prior;
    return;
end
r = ref(4:7); c = a(4:7);
bx = 1 - 2*(r(3)^2 + r(4)^2);
by = 2*(r(2)*r(3) + r(1)*r(4));
ax = 1 - 2*(c(3)^2 + c(4)^2);
ay = 2*(c(2)*c(3) + c(1)*c(4));
if hypot(bx, by) < 1e-8 || hypot(ax, ay) < 1e-8
    x = talos_nav_yaw_gauge(prior, a);
    return;
end
half = 0.5 * atan2(ax*by - ay*bx, ax*bx + ay*by);
cs = cos(half); sn = sin(half);
a(4:7) = [cs*c(1) - sn*c(4); cs*c(2) - sn*c(3); cs*c(3) + sn*c(2); cs*c(4) + sn*c(1)];
x = talos_nav_normalize(a, ref);
end
