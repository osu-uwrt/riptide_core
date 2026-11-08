function s = talos_nav_sizes
%TALOS_NAV_SIZES Array sizes of the estimator. The first four equal the build group of the config.

s.queue = 512;
s.inputs = 512;
s.checkpoints = 128;
s.ring = 256;
s.ring_window = 16;
s.window = 64;
s.sigma = 16;
end
