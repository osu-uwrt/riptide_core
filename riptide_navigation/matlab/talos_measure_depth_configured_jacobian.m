function H = talos_measure_depth_configured_jacobian(x, mask)
H = talos_numeric_jacobian(x, 4, mask);
end
