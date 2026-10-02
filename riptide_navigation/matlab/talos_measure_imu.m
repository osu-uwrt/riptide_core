function y = talos_measure_imu(x)
% Gravity direction removes yaw while retaining nonsingular tilt information.
q = x(4:7); q = q / max(sqrt(sum(q.*q)), 1e-12);
w=q(1); qx=q(2); qy=q(3); qz=q(4);
gravityBody = [2*(qx*qz-w*qy); 2*(qy*qz+w*qx); 1-2*(qx*qx+qy*qy)];
y = [gravityBody; x(11:12); x(14:16)];
end
