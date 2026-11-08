function [x, P, ok] = talos_nav_scalar(x, P, c, z, r)
%TALOS_NAV_SCALAR Exact linear Kalman update of state c from a scalar measurement z with variance r.
%#codegen
ok = false;
if ~isfinite(z) || ~isfinite(r) || r < 0
    return;
end
pc = P(c, c);
s = pc + r;
res = z - x(c);
if pc < 0 || ~isfinite(s) || s <= 0 || ~isfinite(res)
    return;
end
cross = P(:, c);
K = cross / s;
if ~all(isfinite(K))
    return;
end
xc = x + K * res;
if ~all(isfinite(xc))
    return;
end
% expanded Joseph form, with the observed row and column from the exact cross terms
P2 = P - K * cross' - cross * K' + K * pc * K' + K * r * K';
P2(c, :) = cross' * (r / s);
P2(:, c) = cross * (r / s);
P2 = tril(P2) + tril(P2, -1)';
if ~all(isfinite(P2(:)))
    return;
end
x = xc;
P = P2;
ok = true;
end
