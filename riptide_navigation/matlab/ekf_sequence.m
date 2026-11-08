function err = ekf_sequence(ekf, k)
%EKF_SEQUENCE Step k of a predict/correct sequence through ekf and through talos_filter_apply.
% Every sensor is corrected in turn, and every sixth step normalizes the quaternion outside the
% filter, so the block also has to load the edited state. err is the largest state or
% covariance difference so far.
%#codegen
persistent x P xr Pr worst
if isempty(x)
    x = [0; 0; 0; 1; zeros(15, 1)];
    P = diag(0.1 + (1:19)' / 100);
    xr = x;
    Pr = P;
    worst = 0;
end
s = mod(k, 6);
y = 0.01 * sin((1:9)' + k);
R = 1e-3 * eye(9);
ctx = [0; 0; 9.8; 1; 1; 0; 1; 1; 1; 1; 1; 1];
if s == 0 || s == 3
    dt = 0.004;
    Sq = talos_process_root(x, P, dt, 1e-3 * ones(19, 1), 0, ones(19, 1));
    [x, P] = ekf(x, P, dt, Sq, 0, y, R, ctx);
    [xr, Pr] = talos_filter_apply(xr, Pr, dt, Sq, 0, y, R, ctx);
else
    sensor = min(s, 4);
    if sensor == 2 || sensor == 4
        ctx(1:3) = 1;
    elseif sensor == 3
        ctx(1:6) = [0.1; 0; 0.2; 1; 1; 1];
    end
    [x, P] = ekf(x, P, 0, zeros(19), sensor, y, R, ctx);
    [xr, Pr] = talos_filter_apply(xr, Pr, 0, zeros(19), sensor, y, R, ctx);
end
if s == 5
    x(4:7) = x(4:7) / norm(x(4:7));
    xr(4:7) = xr(4:7) / norm(xr(4:7));
end
worst = max([worst; abs(x - xr); abs(P(:) - Pr(:))]);
err = worst;
end
