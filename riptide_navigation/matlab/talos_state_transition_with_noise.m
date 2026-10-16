function next = talos_state_transition_with_noise(x, noise, context)
%#codegen
coder.inline('always');
%CONTEXT is [dt; process-noise square root, column major].
root = reshape(context(2:257),16,16);
next = talos_state_transition(x,context(1)) + root*noise;
end
