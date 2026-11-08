function [x, P] = talos_nav_coast(F, p, x, P, t, dt)
%TALOS_NAV_COAST Stop integrating held motion once the sensors behind it are lost.
% Without the IMU the last rate and acceleration would otherwise run on, so a small held pitch
% rate tumbles the attitude and turns body velocity into depth, and the correlations this builds
% throw the state on reacquisition. Each one decays toward zero at its reset variance instead:
% a first-order Gauss-Markov step, which fades those correlations too. Yaw rate and velocity
% only decay when the FOG and DVL are also lost. t is the step's start, nanoseconds.
%#codegen
tau = p.timing.outage_motion_time_constant;
if tau <= 0 || ~lost(F, p, t, 1)
    return;
end
s = ones(19, 1);
s([11, 12, 14, 15, 16]) = exp(-dt / tau);
if lost(F, p, t, 2)
    s(13) = s(14);
end
tauV = p.timing.outage_velocity_time_constant;
if tauV > 0 && lost(F, p, t, 3)
    s(8:10) = exp(-dt / tauV);
end
x = s .* x;
P = (s * s') .* P;
resetd = p.estimator.reset_covariance_diag(:);
for i = 8:16
    P(i, i) = P(i, i) + (1 - s(i) ^ 2) * resetd(i);
end
end

function out = lost(F, p, t, sensor)
out = F.last_proc(sensor) ~= 0 && ...
    double(t - F.last_proc(sensor)) * 1e-9 > p.timing.sensor_timeouts(sensor);
end
