function k = talos_nav_frame_kind(bytes, n, base, mount)
%TALOS_NAV_FRAME_KIND Frame of a sensor sample, see talos_nav_ingest.
% 0 empty without a fallback, 1 base link, 2 mount frame, 3 other, 4 empty with a fallback.
%#codegen
if n == 0
    k = 0;
    if mount(1) ~= 0
        k = 4;
    end
elseif talos_nav_frame_match(bytes, n, base)
    k = 1;
elseif talos_nav_frame_match(bytes, n, mount)
    k = 2;
else
    k = 3;
end
end
