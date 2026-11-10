function sq = talos_nav_innovation_squared(F, p, E)
%TALOS_NAV_INNOVATION_SQUARED Squared whitened innovation of a gated sensor, inf if S is not SPD.
%#codegen
sq = inf;
x = F.x; P = F.P;
switch E.sensor
    case 4
        S = P(3, 3) + E.cov(1, 1);
        if S > 0
            sq = (E.value(1) - x(3))^2 / S;
        end
    case 2
        % check against the VN gyro: independent, and it sees real impacts too
        sq = 0;
        if F.vn_ns == 0 || abs(double(E.stamp - F.vn_ns)) * 1e-9 > p.gates.vn_rate_pairing_window
            return;
        end
        cfg = double(p.sensor_axes.fog_twist_config(:));
        sq = 0;
        for i = 1:3
            if cfg(9 + i) == 0
                continue;
            end
            r = E.value(i) - F.vn_rate(i);
            sq = sq + r * r / (E.cov(i, i) + F.vn_rate_var(i) + p.gates.fog_vn_rate_tolerance^2);
        end
    case 1
        sq = tilt(x, P, E.value(1:3), E.cov(1:3, 1:3), talos_nav_tilt_mask(p));
    otherwise
        cfg = double(p.sensor_axes.dvl_twist_config(:));
        r = E.offset;
        omap = [0 r(3) -r(2); -r(3) 0 r(1); r(2) -r(1) 0];
        axes = find(cfg(7:9) ~= 0);
        n = numel(axes);
        H = zeros(n, 19);
        res = zeros(n, 1);
        for k = 1:n
            i = axes(k);
            H(k, 7 + i) = 1;
            H(k, 11:13) = omap(i, :);
            res(k) = E.value(i) - x(7 + i) - omap(i, :) * x(11:13);
        end
        S = E.cov(axes, axes) + H * P * H';
        [L, flag] = chol(S, 'lower');
        if flag == 0 && n > 0
            w = L \ res;
            sq = sum(w .* w);
            if ~isfinite(sq)
                sq = inf;
            end
        elseif n == 0
            sq = 0;
        end
end
end

function sq = tilt(x, P, grav, cov, mask)
sq = inf;
[xn, ok] = talos_nav_normalize(x, []);
if ~ok
    return;
end
nrm = sqrt(sum(x(4:7) .^ 2));
w = xn(4); qx = xn(5); qy = xn(6); qz = xn(7);
mag = max(sqrt(sum(grav .^ 2)), 1e-12);
g = [2*(qx*qz - w*qy); 2*(qy*qz + w*qx); 1 - 2*(qx*qx + qy*qy)];
m = grav / mag;
cr = [0 -m(3) m(2); m(3) 0 -m(1); -m(2) m(1) 0];
rawJ = [-2*qy 2*qz -2*w 2*qx; 2*qx 2*w 2*qz 2*qy; 0 -4*qx -4*qy 0];
q = xn(4:7);
J = rawJ * (eye(4) - q * q') / nrm;
D = diag(mask);
res = -D * cr * g;
H = D * cr * J;
S = D * cov * D + 1e-12 * eye(3) + H * P(4:7, 4:7) * H';
[L, flag] = chol(S, 'lower');
if flag ~= 0
    return;
end
wh = L \ res;
sq = sum(wh .* wh);
end
