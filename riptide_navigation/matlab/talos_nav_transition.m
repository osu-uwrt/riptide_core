function next = talos_nav_transition(x, noise, context)
%TALOS_NAV_TRANSITION State transition of the model's Extended Kalman Filter block.
% context is [mode; dt; process-noise root, column major; loaded state]. Mode 1 predicts with
% talos_state_transition_with_noise, 2 loads context(364:382) with the root as its covariance
% root, and anything else holds the state.
%#codegen
switch context(1)
    case 1
        next = talos_state_transition_with_noise(x, noise, context(2:363));
    case 2
        next = context(364:382) + reshape(context(3:363), 19, 19) * noise;
    otherwise
        next = x;
end
end
