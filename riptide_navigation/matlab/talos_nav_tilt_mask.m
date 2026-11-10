function mask = talos_nav_tilt_mask(p)
%TALOS_NAV_TILT_MASK VN tilt rows in use. Roll and pitch together select all three cross-product rows.

cfg = double(p.sensor_axes.vectornav_imu_config(:));
mask = cfg(4:6);
if mask(1) ~= 0 && mask(2) ~= 0
    mask = ones(3, 1);
end
end
