function [x, P] = talos_filter_correct(x, P, sensor, y, R, context)
%TALOS_FILTER_CORRECT Joseph-form measurement update through the configured models.
% sensor selects the measurement: 1 VN (9 rows), 2 FOG (3), 3 DVL (3), 4 depth (1).
% y, R and context are padded to 9, 9x9 and 12 elements.
%#codegen
switch sensor
    case 1
        n = 9;
        h = talos_measure_imu_configured(x, context);
        H = talos_measure_imu_configured_jacobian(x, context);
    case 2
        n = 3;
        h = talos_measure_fog_configured(x, context(1:3));
        H = talos_measure_fog_configured_jacobian(x, context(1:3));
    case 3
        n = 3;
        h = talos_measure_dvl_configured(x, context(1:6));
        H = talos_measure_dvl_configured_jacobian(x, context(1:6));
    otherwise
        n = 1;
        h = talos_measure_depth_configured(x, context(1));
        H = talos_measure_depth_configured_jacobian(x, context(1));
end

R = R(1:n, 1:n);
K = (P * H') / (H * P * H' + R);
x = x + K * (y(1:n) - h);
A = eye(19) - K * H;
P = A * P * A' + K * R * K';
P = 0.5 * (P + P');
end
