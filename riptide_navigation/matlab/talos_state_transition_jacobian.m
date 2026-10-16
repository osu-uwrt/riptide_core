function F = talos_state_transition_jacobian(x, dt)
%TALOS_STATE_TRANSITION_JACOBIAN Exact normalized-quaternion kinematic Jacobian.
% Includes both normalization Jacobians, body-frame Coriolis velocity and
% the derivative of the exponential-map increment at zero angular velocity.

dt = min(max(dt, 0), 0.1);
raw = x(4:7);
n = norm(raw);
if n < 1e-12
    q = [1; 0; 0; 0];
    N = zeros(4);
else
    q = raw / n;
    N = (eye(4) - q*q') / n;
end
w = q(1); a = q(2); b = q(3); c = q(4);
R = [1-2*(b*b+c*c), 2*(a*b-c*w), 2*(a*c+b*w); ...
     2*(a*b+c*w), 1-2*(a*a+c*c), 2*(b*c-a*w); ...
     2*(a*c-b*w), 2*(b*c+a*w), 1-2*(a*a+b*b)];
u = x(8:10)*dt + x(14:16)*(0.5*dt*dt);
ux = u(1); uy = u(2); uz = u(3);
D = 2*[-c*uy+b*uz, b*uy+c*uz, -2*b*ux+a*uy+w*uz, -2*c*ux-w*uy+a*uz; ...
        c*ux-a*uz, b*ux-2*a*uy-w*uz, a*ux+c*uz, w*ux-2*c*uy+b*uz; ...
       -b*ux+a*uy, c*ux+w*uy-2*a*uz, -w*ux+c*uy-2*b*uz, a*ux+b*uy];

r = x(11:13)*dt;
angle = norm(r);
if angle < 1e-9
    inc = [1; 0.5*r];
    incNorm = norm(inc);
    inc = inc/incNorm;
    Jinc = (eye(4)-inc*inc')/incNorm * [zeros(1,3); 0.5*eye(3)];
else
    s = sin(0.5*angle)/angle;
    inc = [cos(0.5*angle); s*r];
    % The direct expression loses precision when the two terms cancel.
    if angle < 1e-4
        coefficient = -1/24 + angle^2/960 - angle^4/107520;
    else
        coefficient = (0.5*cos(0.5*angle)-s)/(angle*angle);
    end
    Jinc = [-0.5*s*r'; s*eye(3)+coefficient*(r*r')];
end
L = [w, -a, -b, -c; a, w, -c, b; b, c, w, -a; c, -b, a, w];
T = [inc(1), -inc(2), -inc(3), -inc(4); ...
     inc(2), inc(1), inc(4), -inc(3); ...
     inc(3), -inc(4), inc(1), inc(2); ...
     inc(4), inc(3), -inc(2), inc(1)];
nextQ = L*inc;
nextNorm = norm(nextQ);
nextQ = nextQ/nextNorm;
Nnext = (eye(4)-nextQ*nextQ')/nextNorm;

F = eye(16);
F(1:3,4:7) = D*N;
F(1:3,8:10) = R*dt;
F(1:3,14:16) = R*(0.5*dt*dt);
F(4:7,4:7) = Nnext*T*N;
F(4:7,11:13) = Nnext*L*Jinc*dt;
F(8:10,8:10) = eye(3)-skew(x(11:13))*dt;
F(8:10,11:13) = skew(x(8:10))*dt;
F(8:10,14:16) = eye(3)*dt;
end

function S = skew(v)
S = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
end
