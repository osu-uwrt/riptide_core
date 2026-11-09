function H = talos_measure_imu_configured_jacobian(x, context)
% Analytic form preserves the exact global-yaw nullspace of gravity. A
% finite-difference quaternion Jacobian leaks that unobservable direction as
% yaw covariance grows.
gravityMeasurement = context(1:3);
mask = context(4:12);
qRaw = x(4:7);
qNorm = max(sqrt(sum(qRaw.*qRaw)), 1e-12);
q = qRaw/qNorm;
w=q(1); qx=q(2); qy=q(3); qz=q(4);

gravityQuaternionJacobian = [ ...
    -2*qy,  2*qz, -2*w,  2*qx; ...
     2*qx,  2*w,   2*qz, 2*qy; ...
     0,    -4*qx, -4*qy, 0];
normalizationJacobian = (eye(4) - q*q.')/qNorm;
gravityCrossJacobian = [ ...
    0, -gravityMeasurement(3), gravityMeasurement(2); ...
    gravityMeasurement(3), 0, -gravityMeasurement(1); ...
    -gravityMeasurement(2), gravityMeasurement(1), 0];

H = zeros(9,16);
H(1:3,4:7) = diag(mask(1:3))*gravityCrossJacobian* ...
    gravityQuaternionJacobian*normalizationJacobian;
H(4,11) = mask(4);
H(5,12) = mask(5);
H(6,13) = mask(6);
H(7,14) = mask(7);
H(8,15) = mask(8);
H(9,16) = mask(9);
end
