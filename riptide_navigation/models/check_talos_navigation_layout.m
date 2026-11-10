function issues = check_talos_navigation_layout(model)
%CHECK_TALOS_NAVIGATION_LAYOUT Diagram rules of the generated model, as a list of problems.
% Every line segment is horizontal or vertical, no line crosses a block, blocks do not
% overlap and every port is connected. Library internals (ROS blocks, the EKF block) are skipped.
issues = {};
systems = [{model}; find_system(model, 'LookUnderMasks', 'all', 'FollowLinks', 'off', 'BlockType', 'SubSystem')];
for i = 1:numel(systems)
    sys = systems{i};
    if i > 1 && (~strcmp(get_param(sys, 'LinkStatus'), 'none') || ~isempty(get_param(sys, 'ReferenceBlock')) ...
            || strcmp(get_param(sys, 'Mask'), 'on') || ~strcmp(get_param(sys, 'SFBlockType'), 'NONE'))
        continue;
    end
    if inside_library(sys)
        continue;
    end
    blocks = find_system(sys, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'block');
    blocks = blocks(blocks ~= get_param(sys, 'Handle'));
    rects = cell2mat(get_param(blocks, 'Position'));
    if isempty(rects)
        continue;
    end
    names = get_param(blocks, 'Name');
    if ~iscell(names), names = {names}; end

    for a = 1:numel(blocks)
        for b = a + 1:numel(blocks)
            if overlap(rects(a, :), rects(b, :))
                issues{end + 1} = sprintf('%s: %s overlaps %s', sys, names{a}, names{b}); %#ok<AGROW>
            end
        end
        ports = get_param(blocks(a), 'PortConnectivity');
        if strcmp(get_param(blocks(a), 'BlockType'), 'SubSystem') && strcmp(get_param(blocks(a), 'IsSimulinkFunction'), 'on')
            % a Simulink Function is called, not wired
            ports = ports(~strcmp({ports.Type}, 'trigger'));
        end
        for k = 1:numel(ports)
            if isempty(ports(k).SrcBlock) && isempty(ports(k).DstBlock)
                issues{end + 1} = sprintf('%s: %s port %s unconnected', sys, names{a}, ports(k).Type); %#ok<AGROW>
            elseif ~isempty(ports(k).SrcBlock) && all(ports(k).SrcBlock < 0)
                issues{end + 1} = sprintf('%s: %s input %s unconnected', sys, names{a}, ports(k).Type); %#ok<AGROW>
            end
        end
    end

    lines = find_system(sys, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
    for l = 1:numel(lines)
        pts = get_param(lines(l), 'Points');
        for s = 1:size(pts, 1) - 1
            p = pts(s, :);
            q = pts(s + 1, :);
            if p(1) ~= q(1) && p(2) ~= q(2)
                issues{end + 1} = sprintf('%s: diagonal segment [%d %d]-[%d %d]', sys, p, q); %#ok<AGROW>
                continue;
            end
            for b = 1:numel(blocks)
                if crosses(p, q, rects(b, :))
                    issues{end + 1} = sprintf('%s: line [%d %d]-[%d %d] crosses %s', sys, p, q, names{b}); %#ok<AGROW>
                end
            end
        end
    end
end
issues = issues(:);
end

function e = inside_library(sys)
% True below a linked or masked library block, such as a ROS block or the EKF.
e = false;
parent = get_param(sys, 'Parent');
while ~isempty(parent) && contains(parent, '/')
    if ~isempty(get_param(parent, 'ReferenceBlock')) || strcmp(get_param(parent, 'Mask'), 'on')
        e = true;
        return;
    end
    parent = get_param(parent, 'Parent');
end
end

function e = overlap(a, b)
e = a(1) < b(3) && b(1) < a(3) && a(2) < b(4) && b(2) < a(4);
end

function e = crosses(p, q, r)
% A segment that enters the block's interior; touching its edge at a port is fine.
lo = min(p, q);
hi = max(p, q);
if lo(2) == hi(2)
    e = lo(2) > r(2) && lo(2) < r(4) && lo(1) < r(3) && hi(1) > r(1);
else
    e = lo(1) > r(1) && lo(1) < r(3) && lo(2) < r(4) && hi(2) > r(2);
end
end
