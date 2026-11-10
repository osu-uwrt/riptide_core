function H = talos_measure_fog_configured_jacobian(x, mask)
H = talos_numeric_jacobian(x, 2, mask);
end
