function w = talos_nav_ring_width
%TALOS_NAV_RING_WIDTH Elements of one ring slot: the arrival time and the packed samples.
%#codegen
w = numel(talos_nav_raw_pack(talos_nav_raw())) + 1;
end
