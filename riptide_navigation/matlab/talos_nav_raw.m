function raw = talos_nav_raw
%TALOS_NAV_RAW Empty set of samples for one step. valid marks a sample that arrived this step.
% frame: see talos_nav_ingest; depth and pose use 0 empty, 1 base link, 2 world, 3 other.
% Covariances are 3x3 matrices in row-major ROS order, orientation is [x; y; z; w].
% The field order is the model's raw sample bus order, see talos_navigation_buses.

stamp = int64(0);
raw.imu = struct('valid', false, 'stamp', stamp, 'orientation', [0; 0; 0; 1], 'omega', zeros(3, 1), ...
    'accel', zeros(3, 1), 'cov_o', zeros(3), 'cov_w', zeros(3), 'cov_a', zeros(3), 'frame', 0);
raw.fog = struct('valid', false, 'stamp', stamp, 'omega', zeros(3, 1), 'cov', zeros(3), 'frame', 0);
raw.dvl = struct('valid', false, 'stamp', stamp, 'vel', zeros(3, 1), 'cov', zeros(3), 'frame', 0);
raw.depth = struct('valid', false, 'stamp', stamp, 'z', 0, 'var', 0, 'frame', 2);
raw.set_pose = struct('valid', false, 'stamp', stamp, 'pos', zeros(3, 1), 'quat', [1; 0; 0; 0], ...
    'cov', zeros(6), 'frame', 2);
end
