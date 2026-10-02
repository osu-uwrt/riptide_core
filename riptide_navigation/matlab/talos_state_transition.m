function next = talos_state_transition(x, dt)
%TALOS_STATE_TRANSITION Quaternion kinematics with body-frame velocity.
% State: [p_odom(3), q_body_to_odom(wxyz)(4), v_body(3), omega_body(3), a_body(3)].

dt = min(max(dt, 0), 0.1);
q = normalize_quaternion(x(4:7));
v = x(8:10);
w = x(11:13);
a = x(14:16);
R = quaternion_rotation(q);

next = x;
next(1:3) = x(1:3) + R * v * dt + 0.5 * R * a * dt * dt;
next(4:7) = normalize_quaternion(quaternion_multiply(q, rotation_vector_quaternion(w * dt)));
next(8:10) = v + (a - cross(w, v)) * dt;
end

function q = normalize_quaternion(q)
n = sqrt(sum(q .* q));
if n < 1e-12
    q = [1; 0; 0; 0];
else
    q = q / n;
end
end

function q = rotation_vector_quaternion(r)
a = sqrt(sum(r .* r));
if a < 1e-9
    q = normalize_quaternion([1; 0.5 * r]);
else
    q = [cos(0.5 * a); sin(0.5 * a) * r / a];
end
end

function out = quaternion_multiply(a, b)
out = [a(1)*b(1)-dot(a(2:4),b(2:4)); ...
       a(1)*b(2:4)+b(1)*a(2:4)+cross(a(2:4),b(2:4))];
end

function R = quaternion_rotation(q)
w=q(1); x=q(2); y=q(3); z=q(4);
R = [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w); ...
     2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w); ...
     2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)];
end
