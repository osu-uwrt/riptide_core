function [x, P] = talos_filter_predict(x, P, dt, S)
%TALOS_FILTER_PREDICT Time update with the process root S from talos_process_root.
%#codegen
F = talos_state_transition_jacobian(x, dt);
x = talos_state_transition(x, dt);
P = F * P * F' + S * S';
P = 0.5 * (P + P');
end
