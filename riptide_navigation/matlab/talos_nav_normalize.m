function [x, ok] = talos_nav_normalize(x, ref)
%TALOS_NAV_NORMALIZE Unit quaternion in rows 4:7, sign matched to ref(4:7) when ref is given.
% ref may be empty. ok is false for a zero or nonfinite norm, and x is left alone.
%#codegen
n = sqrt(sum(x(4:7) .^ 2));
ok = isfinite(n) && n >= 1e-12;
if ~ok
    return;
end
x(4:7) = x(4:7) / n;
if ~isempty(ref) && sum(x(4:7) .* ref(4:7)) < 0
    x(4:7) = -x(4:7);
end
end
