function n = talos_nav_innovation_norm(F, p, E, applyTilt)
%TALOS_NAV_INNOVATION_NORM Raw residual norm of the masked measurement, for diagnostics.
%#codegen
x = F.x;
imu = double(p.sensor_axes.vectornav_imu_config(:));
fog = double(p.sensor_axes.fog_twist_config(:));
dvl = double(p.sensor_axes.dvl_twist_config(:));
depth = double(p.sensor_axes.depth_pose_config(:));
switch E.sensor
    case 1
        R = talos_nav_rotation(x(4:7));
        tilt = cross(E.value(1:3), R(3, :)');
        res = [-tilt .* talos_nav_tilt_mask(p) * applyTilt; ...
            (E.value(4:6) - x(11:13)) .* imu(10:12) * E.groups(2); ...
            (E.value(7:9) - x(14:16) - p.estimator.gravitational_acceleration * R(3, :)') .* imu(13:15) * E.groups(3)];
    case 2
        res = (E.value(1:3) - x(11:13)) .* fog(10:12);
    case 3
        v = x(8:10) + cross(x(11:13), E.offset);
        res = (E.value(1:3) - v) .* dvl(7:9);
    otherwise
        res = (E.value(1) - x(3)) * depth(3);
end
n = sqrt(sum(res .^ 2));
end
