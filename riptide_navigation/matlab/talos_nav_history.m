function S = talos_nav_history(S, p, op, a)
%TALOS_NAV_HISTORY Bounded log of processed inputs and filter checkpoints, for late-sample replay.
% op: 'clear', 'input' (a is the event), 'checkpoint' (a is true to force one) or 'trim'
% (a is the horizon stamp, keeping the newest checkpoint at or before it). Rings are indexed by
% global counts: input i sits in slot mod(i-1, inputs)+1, checkpoint c in slot mod(c-1, checkpoints)+1.
%#codegen
z = talos_nav_sizes();
switch op
    case 'clear'
        S.hn = 0;
        S.hfirst = 0;
        S.ck_first = 1;
        S.ck_total = 0;
    case 'input'
        S.hn = S.hn + 1;
        S.hin(mod(S.hn - 1, z.inputs) + 1) = a;
    case 'checkpoint'
        if ~S.recording
            return;
        end
        t = S.F.tf;
        if ~a && S.ck_total >= S.ck_first
            newest = S.hck(mod(S.ck_total - 1, z.checkpoints) + 1).t;
            if double(t - newest) < p.timing.checkpoint_stride * 1e9
                return;
            end
        end
        S.ck_total = S.ck_total + 1;
        slot = mod(S.ck_total - 1, z.checkpoints) + 1;
        S.hck(slot).F = S.F;
        S.hck(slot).t = t;
        S.hck(slot).idx = S.hn;
        if S.ck_total - S.ck_first + 1 > z.checkpoints
            S.ck_first = S.ck_first + 1;
        end
    otherwise
        if S.ck_total < S.ck_first
            return;
        end
        while S.ck_total - S.ck_first + 1 > 1 && ...
                S.hck(mod(S.ck_first, z.checkpoints) + 1).t <= a
            S.ck_first = S.ck_first + 1;
        end
        front = S.hck(mod(S.ck_first - 1, z.checkpoints) + 1).idx;
        S.hfirst = max(S.hfirst, front);
end
end
