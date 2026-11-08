function x = talos_nav_yaw_gauge(before, x)
%TALOS_NAV_YAW_GAUGE Remove the world yaw a gravity-only correction cannot observe.
% Picks the attitude on the world Z orbit of x nearest to before.
%#codegen
[ref, ok] = talos_nav_normalize(before, []);
[a, ok2] = talos_nav_normalize(x, ref);
if ~ok || ~ok2
    x = before;
    return;
end
b = ref(4:7); c = a(4:7);
d = sum(b .* c);
cr = -b(1)*c(4) - b(2)*c(3) + b(3)*c(2) + b(4)*c(1);
h = atan2(cr, d);
dw = cos(h); dz = sin(h);
a(4:7) = [dw*c(1) - dz*c(4); dw*c(2) - dz*c(3); dw*c(3) + dz*c(2); dw*c(4) + dz*c(1)];
x = talos_nav_normalize(a, ref);
end
