function [x, P] = talos_nav_normalize_state(x, P, ref)
%TALOS_NAV_NORMALIZE_STATE Normalize the quaternion and project its covariance rows and columns.
% A sign flip toward ref(4:7), which may be empty, flips the quaternion cross terms.
%#codegen
q = x(4:7);
n = sqrt(sum(q .^ 2));
if n < 1e-12 || ~isfinite(n)
    x(4:7) = [1; 0; 0; 0];
    return;
end
u = q / n;
J = (eye(4) - u * u') / n;
P(4:7, :) = J * P(4:7, :);
P(:, 4:7) = P(:, 4:7) * J';
flip = ~isempty(ref) && sum(u .* ref(4:7)) < 0;
x = talos_nav_normalize(x, ref);
if flip
    other = [1:3, 8:19];
    P(4:7, other) = -P(4:7, other);
    P(other, 4:7) = -P(other, 4:7);
end
end
