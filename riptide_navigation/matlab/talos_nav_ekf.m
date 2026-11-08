function [x, P] = talos_nav_ekf(step, x, P, dt, Sq, sensor, y, R, ctx)
%TALOS_NAV_EKF One predict (sensor 0) or correct through the model's Extended Kalman Filter block.
% step calls the talos_nav_ekf_step Simulink Function, which runs the block once: correct the
% enabled measurement, output, then the transition. The block keeps its own state, so x, P are
% loaded into it first unless it already holds them, and a predict needs one more run to read
% its result. Plain MATLAB has no block and uses talos_filter_apply instead.
%#codegen
persistent xb Pb
if isempty(xb)
    xb = nan(19, 1);
    Pb = nan(19);
end
none = zeros(19);
if ~isequal(x, xb) || ~isequal(P, Pb)
    step(2, 0, covariance_root(P), x, 0, y, R, ctx);
end
if sensor == 0
    step(1, dt, Sq, x, 0, y, R, ctx);
    [x, P] = step(0, 0, none, x, 0, y, R, ctx);
else
    [x, P] = step(0, 0, none, x, sensor, y, R, ctx);
end
P = 0.5 * (P + P');
xb = x;
Pb = P;
end

function S = covariance_root(P)
% Lower Cholesky factor, or through the eigenvalues for a covariance that is only semidefinite.
C = 0.5 * (P + P');
S = zeros(19);
[L, failed] = chol(C, 'lower');
if failed == 0
    S(:) = L(:);
else
    [V, D] = eig(C);
    S(:) = real(V * diag(sqrt(max(real(diag(D)), 0))));
end
end
