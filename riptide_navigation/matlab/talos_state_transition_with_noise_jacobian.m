function [stateJacobian, noiseJacobian] = talos_state_transition_with_noise_jacobian(x, noise, context)
%#codegen
coder.inline('always');
%The supplied root is an input, independent of x and the zero-mean noise.
stateJacobian = talos_state_transition_jacobian(x,context(1));
noiseJacobian = reshape(context(2:257),16,16);
end
