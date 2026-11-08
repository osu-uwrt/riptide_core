function ring = talos_nav_ring
%TALOS_NAV_RING Empty message from the input task to the estimator task.
% A flat vector, since only numeric arrays cross a rate transition without blocking either task:
% [first; count; lost; slots], and each slot is [arrival; packed samples]. first numbers the first
% slot among all sets ever written, so the reader can skip what it already took.
z = talos_nav_sizes();
ring = zeros(3 + z.ring_window * talos_nav_ring_width(), 1);
end
