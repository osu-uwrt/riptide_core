function P = talos_nav_transport(from, to, P)
%TALOS_NAV_TRANSPORT Carry the covariance across a world-yaw gauge rotation of the quaternion.
%#codegen
from = talos_nav_normalize(from, []);
to = talos_nav_normalize(to, from);
f = from(4:7); t = to(4:7);
w = sum(t .* f);
x = -t(1)*f(2) + t(2)*f(1) - t(3)*f(4) + t(4)*f(3);
y = -t(1)*f(3) + t(2)*f(4) + t(3)*f(1) - t(4)*f(2);
z = -t(1)*f(4) - t(2)*f(3) + t(3)*f(2) + t(4)*f(1);
L = [w -x -y -z; x w -z y; y z w -x; z -y x w];
P(4:7, :) = L * P(4:7, :);
P(:, 4:7) = P(:, 4:7) * L';
end
