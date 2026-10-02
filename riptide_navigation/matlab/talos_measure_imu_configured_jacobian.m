function H = talos_measure_imu_configured_jacobian(x, context)
H = talos_numeric_jacobian(x, 1, context);
end
