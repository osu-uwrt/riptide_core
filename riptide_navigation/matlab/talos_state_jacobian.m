function F = talos_state_jacobian(x, dt)
%TALOS_STATE_JACOBIAN Central-difference Jacobian including q normalization.
F = zeros(16,16);
for k = 1:16
    h = 1e-6 * max(1, abs(x(k)));
    xp = x; xm = x;
    xp(k) = xp(k) + h;
    xm(k) = xm(k) - h;
    F(:,k) = (talos_state_transition(xp,dt)-talos_state_transition(xm,dt))/(2*h);
end
end
