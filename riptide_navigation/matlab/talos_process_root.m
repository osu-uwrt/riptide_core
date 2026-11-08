function S = talos_process_root(x, P, dt, density, biasDecorrelation, resetDiag)
%TALOS_PROCESS_ROOT Square root of the discretized process covariance, Q = S*S'.
% density is the 19 continuous-time densities (position, 3 body angular error
% densities in rows 4:6 mapped into the quaternion tangent, velocity, rate,
% acceleration, accelerometer bias). A bias density grows with tilt rate while its
% variance is below its resetDiag entry, so it can decorrelate as cos(tilt) changes.
%#codegen
dt = max(dt, 0);
q = x(4:7);
n = sqrt(sum(q .* q));
if n < 1e-12
    q = [1; 0; 0; 0];
else
    q = q / n;
end
w = q(1); qx = q(2); qy = q(3); qz = q(4);

rate = abs(x(11) * 2 * (qy*qz + w*qx) - x(12) * 2 * (qx*qz - w*qy));
for i = 17:19
    if P(i,i) < resetDiag(i)
        density(i) = density(i) + biasDecorrelation * rate;
    end
end

S = zeros(19, 19);
for i = [1:3, 8:19]
    S(i,i) = sqrt(density(i) * dt);
end
G = [-qx -qy -qz; w -qz qy; qz w -qx; -qy qx w];
for axis = 1:3
    S(4:7, 3+axis) = 0.5 * sqrt(dt * density(3+axis)) * G(:, axis);
end
end
