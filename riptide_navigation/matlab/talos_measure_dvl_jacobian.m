function H = talos_measure_dvl_jacobian(~, offset)
%TALOS_MEASURE_DVL_JACOBIAN Includes sensitivity to angular velocity.
H = zeros(3,16);
H(:,8:10) = eye(3);
x = offset(1); y = offset(2); z = offset(3);
H(:,11:13) = [0 z -y; -z 0 x; y -x 0];
end
