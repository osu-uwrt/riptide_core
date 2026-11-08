function r = talos_nav_rotation(q)
%TALOS_NAV_ROTATION Body to world matrix of a quaternion [w; x; y; z].

w = q(1); x = q(2); y = q(3); z = q(4);
r = [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w); ...
     2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w); ...
     2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)];
end
