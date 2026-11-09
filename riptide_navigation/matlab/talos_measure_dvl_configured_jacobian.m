function H = talos_measure_dvl_configured_jacobian(~, context)
%TALOS_MEASURE_DVL_JACOBIAN Includes sensitivity to angular velocity.
offset = context(1:3);
mask = context(4:6);
H = zeros(3,16);
H(:,8:10) = diag(mask);
x = offset(1); y = offset(2); z = offset(3);
H(:,11:13) = diag(mask)*[0 z -y; -z 0 x; y -x 0];
end
