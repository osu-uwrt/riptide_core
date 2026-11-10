function [stateJacobian, noiseJacobian] = talos_nav_transition_jacobian(x, noise, context)
%TALOS_NAV_TRANSITION_JACOBIAN Jacobians of talos_nav_transition for each mode.
% A load drops the old covariance and takes the context's root, a hold keeps it. The block's Q is
% a constant identity, so it never factors one again.
%#codegen
switch context(1)
    case 1
        [stateJacobian, noiseJacobian] = talos_state_transition_with_noise_jacobian(x, noise, context(2:363));
    case 2
        stateJacobian = zeros(19);
        noiseJacobian = reshape(context(3:363), 19, 19);
    otherwise
        stateJacobian = eye(19);
        noiseJacobian = zeros(19);
end
end
