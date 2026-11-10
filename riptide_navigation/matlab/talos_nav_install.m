function F = talos_nav_install(S, F, x, P)
%TALOS_NAV_INSTALL Make x, P the filter state, normalized with the published attitude as sign reference.
%#codegen
if S.has_pub
    ref = S.pub_x;
else
    ref = zeros(0, 1);
end
[F.x, F.P] = talos_nav_normalize_state(x, P, ref);
end
