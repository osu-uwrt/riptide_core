function raw = talos_nav_raw_unpack(v)
%TALOS_NAV_RAW_UNPACK Rebuild one set of samples from talos_nav_raw_pack.
%#codegen
raw = talos_nav_raw();
i = 0;
[raw.imu.valid, i] = flag(v, i);
[raw.imu.stamp, i] = stamp(v, i);
[raw.imu.orientation, i] = take(v, i, 4, 1);
[raw.imu.omega, i] = take(v, i, 3, 1);
[raw.imu.accel, i] = take(v, i, 3, 1);
[raw.imu.cov_o, i] = take(v, i, 3, 3);
[raw.imu.cov_w, i] = take(v, i, 3, 3);
[raw.imu.cov_a, i] = take(v, i, 3, 3);
[raw.imu.frame, i] = take(v, i, 1, 1);
[raw.fog.valid, i] = flag(v, i);
[raw.fog.stamp, i] = stamp(v, i);
[raw.fog.omega, i] = take(v, i, 3, 1);
[raw.fog.cov, i] = take(v, i, 3, 3);
[raw.fog.frame, i] = take(v, i, 1, 1);
[raw.dvl.valid, i] = flag(v, i);
[raw.dvl.stamp, i] = stamp(v, i);
[raw.dvl.vel, i] = take(v, i, 3, 1);
[raw.dvl.cov, i] = take(v, i, 3, 3);
[raw.dvl.frame, i] = take(v, i, 1, 1);
[raw.depth.valid, i] = flag(v, i);
[raw.depth.stamp, i] = stamp(v, i);
[raw.depth.z, i] = take(v, i, 1, 1);
[raw.depth.var, i] = take(v, i, 1, 1);
[raw.depth.frame, i] = take(v, i, 1, 1);
[raw.set_pose.valid, i] = flag(v, i);
[raw.set_pose.stamp, i] = stamp(v, i);
[raw.set_pose.pos, i] = take(v, i, 3, 1);
[raw.set_pose.quat, i] = take(v, i, 4, 1);
[raw.set_pose.cov, i] = take(v, i, 6, 6);
[raw.set_pose.frame, ~] = take(v, i, 1, 1);
end

function [x, i] = take(v, i, rows, cols)
x = reshape(v(i + 1:i + rows * cols), rows, cols);
i = i + rows * cols;
end

function [x, i] = flag(v, i)
x = v(i + 1) ~= 0;
i = i + 1;
end

function [x, i] = stamp(v, i)
x = typecast(v(i + 1), 'int64');
i = i + 1;
end
