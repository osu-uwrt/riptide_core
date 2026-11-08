function out = talos_nav_output
%TALOS_NAV_OUTPUT Publication result of a step. valid is false when nothing was published.
% P is the published covariance; pose_cov is its 6x6 pose block in ROS angle coordinates.

out.valid = false;
out.stamp = int64(0);
out.x = zeros(19, 1);
out.P = zeros(19);
out.pose_cov = zeros(6, 6);
end
