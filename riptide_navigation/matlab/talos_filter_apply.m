function [x, P] = talos_filter_apply(x, P, dt, Sq, sensor, y, R, ctx)
%TALOS_FILTER_APPLY Reference predict (sensor 0) or correct, with the same arguments as the model's filter.
% The estimator takes its filter as a handle: the model passes the Extended Kalman Filter block,
% and MATLAB runs (unit tests, replays) pass this.
%#codegen
if sensor == 0
    [x, P] = talos_filter_predict(x, P, dt, Sq);
else
    [x, P] = talos_filter_correct(x, P, sensor, y, R, ctx);
end
end
