function k = talos_nav_world_kind(bytes, n, base, world)
%TALOS_NAV_WORLD_KIND Frame of a depth or pose sample: 0 empty, 1 base link, 2 world, 3 other.
%#codegen
if n == 0
    k = 0;
elseif talos_nav_frame_match(bytes, n, base)
    k = 1;
elseif talos_nav_frame_match(bytes, n, world)
    k = 2;
else
    k = 3;
end
end
