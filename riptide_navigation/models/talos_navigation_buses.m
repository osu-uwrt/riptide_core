function talos_navigation_buses
%TALOS_NAVIGATION_BUSES Define the param and raw sample bus objects in the base workspace.
% The parameter buses follow config/parameter_schema.json, so they cannot drift from the config,
% and the raw sample buses follow talos_nav_raw, which the estimator functions use.

root = fileparts(fileparts(mfilename('fullpath')));
schema = jsondecode(fileread(fullfile(root, 'config', 'parameter_schema.json'))).parameters;
if ~iscell(schema), schema = num2cell(schema); end
entries = schema(cellfun(@(e) any(strcmp(e.consumer, {'model', 'both'})), schema));
groups = unique(cellfun(@(e) e.group, entries, 'UniformOutput', false), 'stable');
top = [];
for g = 1:numel(groups)
    members = entries(cellfun(@(e) strcmp(e.group, groups{g}), entries));
    elements = [];
    for k = 1:numel(members)
        [type, count] = elementType(members{k});
        elements = [elements, element(members{k}.name, type, count)]; %#ok<AGROW>
    end
    define(['talos_nav_params_' groups{g}], elements);
    top = [top, element(groups{g}, ['Bus: talos_nav_params_' groups{g}], 1)]; %#ok<AGROW>
end
define('talos_nav_params', top);

raw = talos_nav_raw();
sensors = fieldnames(raw);
top = [];
for i = 1:numel(sensors)
    sample = raw.(sensors{i});
    fields = fieldnames(sample);
    elements = [];
    for k = 1:numel(fields)
        v = sample.(fields{k});
        type = class(v);
        if islogical(v), type = 'boolean'; end
        count = size(v);
        if count(2) == 1, count = count(1); end
        elements = [elements, element(fields{k}, type, count)]; %#ok<AGROW>
    end
    define(['talos_nav_raw_' sensors{i}], elements);
    top = [top, element(sensors{i}, ['Bus: talos_nav_raw_' sensors{i}], 1)]; %#ok<AGROW>
end
define('talos_nav_raw', top);

end

function e = element(name, type, count)
e = Simulink.BusElement;
e.Name = name;
e.DataType = type;
e.Dimensions = count;
end

function define(name, elements)
b = Simulink.Bus;
b.Elements = elements;
assignin('base', name, b);
end

function [type, count] = elementType(entry)
% Get Parameter has no string or enum type: strings travel as uint8[64], enums as int64.
count = 1;
kind = regexp(entry.type, '^\w+', 'match', 'once');
tokens = regexp(entry.type, '\[(\d+)\]', 'tokens', 'once');
if ~isempty(tokens), count = str2double(tokens{1}); end
switch kind
    case 'double', type = 'double';
    case {'int', 'enum'}, type = 'int64';
    case 'bool', type = 'boolean';
    case 'string', type = 'uint8'; count = 64;
end
end
