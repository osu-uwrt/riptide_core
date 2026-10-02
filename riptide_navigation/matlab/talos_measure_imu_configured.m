function y = talos_measure_imu_configured(x, context)
% Gravity-direction innovation is expressed as a quaternion-safe tangent error.
gravityMeasurement = context(1:3);
mask = context(4:12);
q = x(4:7); q = q / max(sqrt(sum(q.*q)), 1e-12);
w=q(1); qx=q(2); qy=q(3); qz=q(4);
gravityBody = [2*(qx*qz-w*qy); 2*(qy*qz+w*qx); 1-2*(qx*qx+qy*qy)];
y = [mask(1:3).*cross(gravityMeasurement, gravityBody); ...
     mask(4:6).*x(11:13); ...
     mask(7:9).*x(14:16)];
end
