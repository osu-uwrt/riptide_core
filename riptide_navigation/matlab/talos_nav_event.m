function E = talos_nav_event
%TALOS_NAV_EVENT Empty measurement event. sensor: 1 VN IMU, 2 FOG, 3 DVL, 4 depth.
% cov holds the IMU blocks [tilt; rate; acceleration] or the leading block of the other sensors.

E.stamp = int64(0);
E.arrival = int64(0);
E.seq = 0;
E.sensor = 0;
E.value = zeros(9, 1);
E.cov = zeros(9, 9);
E.offset = zeros(3, 1);
E.groups = ones(3, 1);
end
