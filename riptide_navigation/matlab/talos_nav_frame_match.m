function e = talos_nav_frame_match(bytes, n, s)
%TALOS_NAV_FRAME_MATCH True when the first n bytes of a message frame_id spell the frame name s.
% Names arrive as zero-padded uint8 parameters.
%#codegen
len = numel(s);
for i = 1:numel(s)
    if s(i) == 0
        len = i - 1;
        break;
    end
end
n = double(n);
e = n == len && n <= numel(bytes) && all(uint8(bytes(1:n)) == uint8(s(1:n)));
end
