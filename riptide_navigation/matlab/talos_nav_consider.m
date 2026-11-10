function [x, P, ok] = talos_nav_consider(x, P, y, ctx, R)
%TALOS_NAV_CONSIDER IMU update where position stays a consider state and the bias sees acceleration only.
% y, ctx and R are the 9 measurements, 12 context values and 9x9 covariance of the joint update.
% The gain is deliberately constrained, so the covariance uses the Joseph form.
%#codegen
ok = false;
[xn, good] = talos_nav_normalize(x, []);
if ~good
    return;
end
nrm = sqrt(sum(x(4:7) .^ 2));
w = xn(4); qx = xn(5); qy = xn(6); qz = xn(7);
mag = max(sqrt(sum(ctx(1:3) .^ 2)), 1e-12);
g = [2*(qx*qz - w*qy); 2*(qy*qz + w*qx); 1 - 2*(qx*qx + qy*qy)];
m = ctx(1:3) / mag;
cr = [0 -m(3) m(2); m(3) 0 -m(1); -m(2) m(1) 0];
rawJ = [-2*qy 2*qz -2*w 2*qx; 2*qx 2*w 2*qz 2*qy; 0 -4*qx -4*qy 0];
q = xn(4:7);
J = rawJ * (eye(4) - q * q') / nrm;

H = zeros(9, 19);
res = zeros(9, 1);
for i = 1:3
    res(i) = y(i) - ctx(3 + i) * (cr(i, :) * g);
    res(3 + i) = y(3 + i) - ctx(6 + i) * x(10 + i);
    res(6 + i) = y(6 + i) - ctx(9 + i) * (x(13 + i) + x(16 + i) + mag * g(i));
    H(3 + i, 10 + i) = ctx(6 + i);
    H(6 + i, 13 + i) = ctx(9 + i);
    H(6 + i, 16 + i) = ctx(9 + i);
    H(i, 4:7) = ctx(3 + i) * cr(i, :) * J;
    H(6 + i, 4:7) = ctx(9 + i) * mag * J(i, :);
end

PH = P * H';
S = R + H * PH;
[~, bad] = chol(S, 'lower');
if bad ~= 0
    return;
end
Sa = S(7:9, 7:9);
[~, bad] = chol(Sa, 'lower');
if bad ~= 0
    return;
end

% bias sees only acceleration rows, angular states only tilt and rate rows
K = zeros(19, 9);
for i = 4:19
    if i >= 17
        K(i, 7:9) = (Sa \ PH(i, 7:9)')';
    else
        angular = (i >= 4 && i <= 7) || (i >= 11 && i <= 13);
        n = 9;
        if angular
            n = 6;
        end
        K(i, 1:n) = (S(1:n, 1:n) \ PH(i, 1:n)')';
    end
end

xc = x + K * res;
A = eye(19) - K * H;
P2 = A * P * A' + K * R * K';
P2 = tril(P2) + tril(P2, -1)';
if ~all(isfinite(xc))
    return;
end
[x, P] = talos_nav_normalize_state(xc, P2, x);
ok = true;
end
