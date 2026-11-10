function make_talos_navigation
%MAKE_TALOS_NAVIGATION Create models/talos_navigation/talos_navigation.slx.
% Laid out like complete_controller, left to right in four stages: Inputs (every ROS parameter on
% one param bus, every subscribed message on one message bus, the ROS time), Measurement Intake
% (messages unpacked into raw samples, kept on a ring), Estimator (the fusion logic around the
% library Extended Kalman Filter block) and ROS Output. The parameter blocks come from
% config/parameter_schema.json, so the model and the config file cannot drift apart.
% It runs three tasks. Subscribe only holds its latest message, so the subscribers, intake and ring
% run in a short fast task and the estimator in a slower one that can overrun and catch up. The
% parameters only change at load, so they get a task of their own at a low rate.

root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'matlab'));
talos_navigation_setup;
schema = jsondecode(fileread(fullfile(root, 'config', 'parameter_schema.json'))).parameters;
if ~iscell(schema), schema = num2cell(schema); end
config = fullfile(root, 'config', 'talos_navigation.yaml');

mdl = 'talos_navigation';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', 'FixedStep', fastStep(), ...
    'EnableMultiTasking', 'on');
% ERT here so the generator keeps it; the estimator uses bounded variable sizes
set_param(mdl, 'SystemTargetFile', 'ert.tlc', 'SupportVariableSizeSignals', 'on');

% the diagnostics vector length is fixed by the estimator functions
p = talos_nav_params(config);
nDiag = numel(talos_nav_diag(talos_nav_init(p), p));

buildInputs(mdl, schema);
buildIntake(mdl);
buildEstimator(mdl, nDiag);
buildOutput(mdl, nDiag);
releaseRosEntities(mdl);
parkFunctionTask(mdl);

save_system(mdl, fullfile(root, 'models', 'talos_navigation', [mdl '.slx']));
close_system(mdl, 0);
end

%% Top level

function t = fastStep
t = ['1/' buildRate('base_rate')];
end

function t = slowStep
t = ['1/' buildRate('estimator_rate')];
end

function t = parameterStep
t = ['1/' buildRate('parameter_rate')];
end

function rate = buildRate(name)
config = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'config', 'talos_navigation.yaml');
[~, rate] = system(['python3 -c "import yaml,sys; print(yaml.safe_load(open(sys.argv[1]))[''navigation''][''build''][''' ...
    name '''])" "' config '"']);
rate = strtrim(rate);
end

function transition(sys, name, x, y, step, deterministic)
% Integrity-only Rate Transitions keep each array whole and never block either task, but cannot
% carry a bus. The deterministic one for the parameters only sees values that have stopped changing.
put(sys, name, 'simulink/Signal Attributes/Rate Transition', x, y, 60, 40, 'Integrity', 'on', ...
    'Deterministic', deterministic, 'OutPortSampleTime', step);
end

function buildInputs(mdl, schema)
% Parameters, subscriptions and the ROS clock, shared with the later stages through Goto tags.
put(mdl, 'Parameters', 'built-in/SubSystem', 80, 120, 160, 60);
buildParameters([mdl '/Parameters'], schema);
put(mdl, 'Start_Gate', 'built-in/SubSystem', 320, 220, 140, 40);
buildStartGate([mdl '/Start_Gate']);
put(mdl, 'Subscribers', 'built-in/SubSystem', 80, 340, 160, 60);
buildSubscribers([mdl '/Subscribers']);
put(mdl, 'ROS_Time', 'ros2lib/Current Time', 80, 480, 160, 50, 'SampleTime', slowStep());
put(mdl, 'Time_To_Nanoseconds', 'built-in/SubSystem', 320, 560, 140, 40);
buildStamp([mdl '/Time_To_Nanoseconds'], 'time', 'now');
put(mdl, 'Arrival_Time', 'ros2lib/Current Time', 80, 660, 160, 50, 'SampleTime', fastStep());
put(mdl, 'Arrival_To_Nanoseconds', 'built-in/SubSystem', 320, 740, 140, 40);
buildStamp([mdl '/Arrival_To_Nanoseconds'], 'time', 'now');

tagOut(mdl, 'Parameters', 1, 'ParamBusSlow', 270);
from(mdl, 'ParamBusSlow', 'Start_Gate', 1, 225);
tagOut(mdl, 'Start_Gate', 1, 'StartedSlow', 470);
transition(mdl, 'Parameters_To_Estimator', 500, 120, slowStep(), 'on');
from(mdl, 'ParamBusSlow', 'Parameters_To_Estimator', 1, 395);
tagOut(mdl, 'Parameters_To_Estimator', 1, 'ParamBus', 590);
transition(mdl, 'Start_To_Estimator', 500, 280, slowStep(), 'off');
from(mdl, 'StartedSlow', 'Start_To_Estimator', 1, 395);
tagOut(mdl, 'Start_To_Estimator', 1, 'Started', 590);
tagOut(mdl, 'Subscribers', 1, 'MessageBus', 590);
tagOut(mdl, 'ROS_Time', 1, 'ROSTime', 590);
wire(mdl, 'ROS_Time/1', 'Time_To_Nanoseconds/1');
tagOut(mdl, 'Time_To_Nanoseconds', 1, 'Now', 590);
wire(mdl, 'Arrival_Time/1', 'Arrival_To_Nanoseconds/1');
tagOut(mdl, 'Arrival_To_Nanoseconds', 1, 'ArrivalNow', 590);
area(mdl, 'Inputs', [40 40 695 820], 'lavender');
end

function buildIntake(mdl)
% the slow parameters and start gate cross into the input task here
transition(mdl, 'Parameters_To_Inputs', 860, 80, fastStep(), 'on');
from(mdl, 'ParamBusSlow', 'Parameters_To_Inputs', 1, 740);
tagOut(mdl, 'Parameters_To_Inputs', 1, 'ParamBusFast', 980);
transition(mdl, 'Start_To_Inputs', 860, 180, fastStep(), 'off');
from(mdl, 'StartedSlow', 'Start_To_Inputs', 1, 740);
tagOut(mdl, 'Start_To_Inputs', 1, 'StartedFast', 980);

put(mdl, 'Measurement_Intake', 'built-in/SubSystem', 860, 460, 220, 260);
intake([mdl '/Measurement_Intake']);
tagIn(mdl, 'Measurement_Intake', {'ParamBusFast', 'MessageBus', 'StartedFast', 'ArrivalNow', 'TakenFast'}, 740);
transition(mdl, 'Taken_To_Inputs', 860, 280, fastStep(), 'off');
from(mdl, 'Taken', 'Taken_To_Inputs', 1, 740);
tagOut(mdl, 'Taken_To_Inputs', 1, 'TakenFast', 980);
transition(mdl, 'Ring_To_Estimator', 1140, portY(mdl, 'Measurement_Intake', 'Outport', 1), slowStep(), 'off');
wire(mdl, 'Measurement_Intake/1', 'Ring_To_Estimator/1', 'ring');
tagOut(mdl, 'Ring_To_Estimator', 1, 'Ring', 1225);
area(mdl, 'Measurement Intake', [700 40 1300 800], 'cream');
end

function buildEstimator(mdl, nDiag)
put(mdl, 'Estimator', 'built-in/SubSystem', 1460, 340, 240, 260);
estimator([mdl '/Estimator'], nDiag);
tagIn(mdl, 'Estimator', {'ParamBus', 'Ring', 'Now', 'Started'}, 1360);
tagOut(mdl, 'Estimator', 2, 'Taken', 1720);
area(mdl, 'Estimator', [1320 40 1820 640], 'blue');
end

function buildOutput(mdl, nDiag)
put(mdl, 'Publish_System', 'built-in/SubSystem', 2040, 340, 240, 260);
publishSystem([mdl '/Publish_System'], nDiag);
tagIn(mdl, 'Publish_System', {'ParamBus', 'ROSTime'}, 1930);
wire(mdl, 'Estimator/1', 'Publish_System/3', 'estimate');
area(mdl, 'ROS Output', [1880 40 2340 640], 'lavender');
end

%% Inputs

function buildParameters(sub, schema)
% One Get Parameter per model parameter, one subsystem per group, all on the param bus.
entries = schema(cellfun(@(e) any(strcmp(e.consumer, {'model', 'both'})), schema));
groups = unique(cellfun(@(e) e.group, entries, 'UniformOutput', false), 'stable');
pitch = 80;
top = 40;
add_block('simulink/Signal Routing/Bus Creator', [sub '/Param_Bus'], 'Inputs', num2str(numel(groups)), ...
    'Position', [420 top 425 top + pitch * numel(groups)], 'OutDataTypeStr', 'Bus: talos_nav_params', ...
    'NonVirtualBus', 'on', 'ShowName', 'off');
for g = 1:numel(groups)
    members = entries(cellfun(@(e) strcmp(e.group, groups{g}), entries));
    put(sub, groups{g}, 'built-in/SubSystem', 160, top + (g - 0.5) * pitch, 160, 50);
    parameterGroup([sub '/' groups{g}], groups{g}, members);
    wire(sub, [groups{g} '/1'], ['Param_Bus/' num2str(g)], groups{g});
end
put(sub, 'param_bus', 'simulink/Sinks/Out1', 500, portY(sub, 'Param_Bus', 'Outport', 1), 30, 14);
wire(sub, 'Param_Bus/1', 'param_bus/1');
end

function parameterGroup(sub, group, members)
pitch = 40;
top = 40;
add_block('simulink/Signal Routing/Bus Creator', [sub '/' group '_bus'], 'Inputs', num2str(numel(members)), ...
    'Position', [460 top 465 top + pitch * numel(members)], ...
    'OutDataTypeStr', ['Bus: talos_nav_params_' group], 'NonVirtualBus', 'on', 'ShowName', 'off');
for k = 1:numel(members)
    entry = members{k};
    [type, count] = parameterType(entry);
    name = ['navigation__' entry.group '__' entry.name];
    y = top + (k - 0.5) * pitch;
    if strcmp(type, 'double[]')
        % R2023a generates double[] parameters as int64 arrays, so each element is its own double
        put(sub, entry.name, 'built-in/SubSystem', 160, y, 220, 30);
        arrayParameter([sub '/' entry.name], name, count, initialValue(entry));
    else
        block = put(sub, entry.name, 'ros2lib/Get Parameter', 160, y, 220, 30);
        set_param(block, 'ParameterName', name, 'ParameterType', type, 'SampleTime', parameterStep());
        if count > 1, set_param(block, 'ParameterMaxArrayLength', num2str(count)); end
        if ~strcmp(entry.type, 'string')
            set_param(block, 'ParameterInitialValue', mat2str(initialValue(entry)));
        end
        % arrays also report their received length, which the param bus does not carry
        ports = get_param(block, 'PortHandles');
        if numel(ports.Outport) > 1
            put(sub, [entry.name '_length'], 'simulink/Sinks/Terminator', 410, portY(sub, entry.name, 'Outport', 2), ...
                20, 20, 'ShowName', 'off');
            wire(sub, [entry.name '/2'], [entry.name '_length/1']);
        end
    end
    wire(sub, [entry.name '/1'], [group '_bus/' num2str(k)], entry.name);
end
put(sub, group, 'simulink/Sinks/Out1', 540, portY(sub, [group '_bus'], 'Outport', 1), 30, 14);
wire(sub, [group '_bus/1'], [group '/1']);
end

function arrayParameter(sub, name, count, value)
pitch = 40;
top = 40;
add_block('simulink/Signal Routing/Vector Concatenate', [sub '/elements'], 'NumInputs', num2str(count), ...
    'Mode', 'Vector', 'Position', [340 top 345 top + pitch * count], 'ShowName', 'off');
for i = 1:count
    block = put(sub, sprintf('element_%d', i - 1), 'ros2lib/Get Parameter', 60, top + (i - 0.5) * pitch, 220, 30);
    set_param(block, 'ParameterName', sprintf('%s_%d', name, i - 1), 'ParameterType', 'double', ...
        'SampleTime', parameterStep(), 'ParameterInitialValue', mat2str(value(i)));
    wire(sub, sprintf('element_%d/1', i - 1), sprintf('elements/%d', i));
end
put(sub, 'value', 'simulink/Sinks/Out1', 420, portY(sub, 'elements', 'Outport', 1), 30, 14);
wire(sub, 'elements/1', 'value/1');
end

function buildStartGate(sub)
% The estimator starts once the overseer marks the parameters loaded, and stays started. The mark
% has to hold for two reads, so a read that landed during the load cannot start it on old values.
put(sub, 'param_bus', 'simulink/Sources/In1', 40, 80, 30, 14);
put(sub, 'loaded', 'simulink/Signal Routing/Bus Selector', 120, 80, 5, 30, ...
    'OutputSignals', 'mounts.parameters_loaded', 'ShowName', 'off');
put(sub, 'latch', 'simulink/Logic and Bit Operations/Logical Operator', 340, 95, 40, 60, 'Operator', 'OR');
put(sub, 'twice', 'simulink/Logic and Bit Operations/Logical Operator', 240, 60, 40, 60, 'Operator', 'AND');
put(sub, 'loaded_before', 'simulink/Discrete/Unit Delay', 160, 150, 40, 30, ...
    'InitialCondition', 'false', 'SampleTime', '-1');
put(sub, 'started_before', 'simulink/Discrete/Unit Delay', 340, 190, 40, 30, ...
    'InitialCondition', 'false', 'SampleTime', '-1', 'Orientation', 'left');
snap(sub, 'twice', 'Outport', 1, portY(sub, 'latch', 'Inport', 1));
snap(sub, 'loaded', 'Outport', 1, portY(sub, 'twice', 'Inport', 1));
snap(sub, 'param_bus', 'Outport', 1, portY(sub, 'loaded', 'Inport', 1));
snap(sub, 'loaded_before', 'Outport', 1, portY(sub, 'twice', 'Inport', 2));
put(sub, 'started', 'simulink/Sinks/Out1', 460, portY(sub, 'latch', 'Outport', 1), 30, 14);
wire(sub, 'param_bus/1', 'loaded/1');
wire(sub, 'loaded/1', 'twice/1', 'parameters_loaded');
wire(sub, 'loaded/1', 'loaded_before/1');
wire(sub, 'loaded_before/1', 'twice/2');
wire(sub, 'twice/1', 'latch/1');
wire(sub, 'latch/1', 'started/1', 'started');
wire(sub, 'latch/1', 'started_before/1');
wire(sub, 'started_before/1', 'latch/2');
end

function buildSubscribers(sub)
% Every subscribed message on one Message Bus, each with its IsNew flag.
sensors = {'imu', 'vectornav/imu', 'sensor_msgs/Imu'; ...
    'fog', 'gyro/twist', 'geometry_msgs/TwistWithCovarianceStamped'; ...
    'dvl', 'dvl_twist', 'geometry_msgs/TwistWithCovarianceStamped'; ...
    'depth', 'depth/pose', 'geometry_msgs/PoseWithCovarianceStamped'; ...
    'set_pose', 'set_pose', 'geometry_msgs/PoseWithCovarianceStamped'};
n = size(sensors, 1);
pitch = 100;
top = 40;
add_block('simulink/Signal Routing/Bus Creator', [sub '/Message_Bus'], 'Inputs', num2str(n), ...
    'Position', [520 top 525 top + pitch * n], 'ShowName', 'off');
for i = 1:n
    name = sensors{i, 1};
    y = portY(sub, 'Message_Bus', 'Inport', i);
    put(sub, [name '_bus'], 'simulink/Signal Routing/Bus Creator', 360, y, 5, 60, 'Inputs', '2', 'ShowName', 'off');
    block = put(sub, ['sub_' name], 'ros2lib/Subscribe', 80, y, 160, 60);
    set_param(block, 'topic', sensors{i, 2}, 'messageType', sensors{i, 3}, ...
        'QOSReliability', 'Best effort', 'QOSDepth', '100', 'SampleTime', fastStep());
    snap(sub, ['sub_' name], 'Outport', 1, portY(sub, [name '_bus'], 'Inport', 1));
    wire(sub, ['sub_' name '/1'], [name '_bus/1'], 'IsNew');
    wire(sub, ['sub_' name '/2'], [name '_bus/2'], 'Msg');
    wire(sub, [name '_bus/1'], ['Message_Bus/' num2str(i)], name);
end
put(sub, 'message_bus', 'simulink/Sinks/Out1', 600, portY(sub, 'Message_Bus', 'Outport', 1), 30, 14);
wire(sub, 'Message_Bus/1', 'message_bus/1');
end

function buildStamp(sub, in, out)
% ROS time to int64 nanoseconds: sec * 1e9 + nanosec.
put(sub, in, 'simulink/Sources/In1', 40, 70, 30, 14);
put(sub, 'parts', 'simulink/Signal Routing/Bus Selector', 120, 70, 5, 60, ...
    'OutputSignals', 'sec,nanosec', 'ShowName', 'off');
y1 = portY(sub, 'parts', 'Outport', 1);
y2 = portY(sub, 'parts', 'Outport', 2);
put(sub, 'sec_int64', 'simulink/Signal Attributes/Data Type Conversion', 180, y1, 60, 24, 'OutDataTypeStr', 'int64');
put(sub, 'nanosec_int64', 'simulink/Signal Attributes/Data Type Conversion', 180, y2, 60, 24, 'OutDataTypeStr', 'int64');
put(sub, 'per_second', 'simulink/Math Operations/Gain', 290, y1, 70, 24, 'Gain', '1000000000', ...
    'ParamDataTypeStr', 'int64', 'OutDataTypeStr', 'int64');
put(sub, 'total', 'simulink/Math Operations/Sum', 410, (y1 + y2) / 2, 30, 60, 'Inputs', '++', ...
    'IconShape', 'rectangular', 'OutDataTypeStr', 'int64');
put(sub, out, 'simulink/Sinks/Out1', 490, (y1 + y2) / 2, 30, 14);
wire(sub, [in '/1'], 'parts/1');
wire(sub, 'parts/1', 'sec_int64/1', 'sec');
wire(sub, 'parts/2', 'nanosec_int64/1', 'nanosec');
wire(sub, 'sec_int64/1', 'per_second/1');
wire(sub, 'per_second/1', 'total/1');
wire(sub, 'nanosec_int64/1', 'total/2');
wire(sub, 'total/1', [out '/1'], out);
end

%% Measurement intake

function intake(sub)
% Each subscribed message becomes a raw sample: stamp in ns, frame kind, vectors and row-major
% covariances. They all go on one raw sample bus and from there onto the ring for the estimator.
put(sub, 'param_bus', 'simulink/Sources/In1', 40, 60, 30, 14);
put(sub, 'message_bus', 'simulink/Sources/In1', 40, 100, 30, 14);
put(sub, 'started', 'simulink/Sources/In1', 40, 140, 30, 14);
put(sub, 'arrival', 'simulink/Sources/In1', 800, 140, 30, 14);
put(sub, 'taken', 'simulink/Sources/In1', 800, 180, 30, 14);
tagOut(sub, 'param_bus', 1, 'ParamBus', 110);
tagOut(sub, 'message_bus', 1, 'MessageBus', 110);
tagOut(sub, 'started', 1, 'Started', 110);

names = {'IMU', 'FOG', 'DVL', 'Depth', 'Set_Pose'};
builders = {@(s) buildSensor(s, 'imu', 'imu_frame_fallback', { ...
    'vector', 'orientation', {'orientation.x', 'orientation.y', 'orientation.z', 'orientation.w'}; ...
    'vector', 'omega', {'angular_velocity.x', 'angular_velocity.y', 'angular_velocity.z'}; ...
    'vector', 'accel', {'linear_acceleration.x', 'linear_acceleration.y', 'linear_acceleration.z'}; ...
    'cov', 'cov_o', {'orientation_covariance', 3, []}; ...
    'cov', 'cov_w', {'angular_velocity_covariance', 3, []}; ...
    'cov', 'cov_a', {'linear_acceleration_covariance', 3, []}}), ...
    @(s) buildSensor(s, 'fog', 'fog_frame_fallback', { ...
    'vector', 'omega', {'twist.twist.angular.x', 'twist.twist.angular.y', 'twist.twist.angular.z'}; ...
    'cov', 'cov', {'twist.covariance', 6, 4:6}}), ...
    @(s) buildSensor(s, 'dvl', 'dvl_frame_fallback', { ...
    'vector', 'vel', {'twist.twist.linear.x', 'twist.twist.linear.y', 'twist.twist.linear.z'}; ...
    'cov', 'cov', {'twist.covariance', 6, 1:3}}), ...
    @buildDepth, @buildSetPose};
topics = {'imu', 'fog', 'dvl', 'depth', 'set_pose'};
pitch = 200;
top = 200;
add_block('simulink/Signal Routing/Bus Creator', [sub '/Raw_Bus'], 'Inputs', num2str(numel(names)), ...
    'Position', [720 top 725 top + pitch * numel(names)], 'OutDataTypeStr', 'Bus: talos_nav_raw', ...
    'NonVirtualBus', 'on', 'ShowName', 'off');
for i = 1:numel(names)
    nIn = 3 + strcmp(names{i}, 'Set_Pose');
    put(sub, names{i}, 'built-in/SubSystem', 440, 0, 160, 40 * nIn);
    builders{i}([sub '/' names{i}]);
    snap(sub, names{i}, 'Outport', 1, portY(sub, 'Raw_Bus', 'Inport', i));
    put(sub, [topics{i} '_message'], 'simulink/Signal Routing/Bus Selector', 340, 0, 5, 80, ...
        'OutputSignals', [topics{i} '.IsNew,' topics{i} '.Msg'], 'ShowName', 'off');
    snap(sub, [topics{i} '_message'], 'Outport', 1, portY(sub, names{i}, 'Inport', 1));
    wire(sub, [topics{i} '_message/1'], [names{i} '/1'], 'IsNew');
    wire(sub, [topics{i} '_message/2'], [names{i} '/2'], 'Msg');
    from(sub, 'MessageBus', [topics{i} '_message'], 1, 220);
    from(sub, 'ParamBus', names{i}, 3, 340);
    if nIn == 4
        from(sub, 'Started', names{i}, 4, 340);
    end
    wire(sub, [names{i} '/1'], ['Raw_Bus/' num2str(i)], topics{i});
end

fcn = put(sub, 'Input_Ring', 'simulink/User-Defined Functions/MATLAB Function', 880, ...
    portY(sub, 'Raw_Bus', 'Outport', 1) + 20, 120, 80);
chart = script(fcn, ['function ring = input_ring(raw, arrival, taken)' newline '% see talos_nav_ring_write' newline ...
    'ring = talos_nav_ring_write(raw, arrival, taken);' newline 'end' newline]);
data = chart.find('-isa', 'Stateflow.Data', 'Name', 'raw', 'Scope', 'Input');
data.DataType = 'Bus: talos_nav_raw';
data = chart.find('-isa', 'Stateflow.Data', 'Name', 'ring', 'Scope', 'Output');
data.Props.Array.Size = num2str(numel(talos_nav_ring()));
snap(sub, 'arrival', 'Outport', 1, portY(sub, 'Input_Ring', 'Inport', 2));
wire(sub, 'Raw_Bus/1', 'Input_Ring/1', 'raw');
wire(sub, 'arrival/1', 'Input_Ring/2');
snap(sub, 'taken', 'Outport', 1, portY(sub, 'Input_Ring', 'Inport', 3));
wire(sub, 'taken/1', 'Input_Ring/3');
put(sub, 'ring', 'simulink/Sinks/Out1', 1040, portY(sub, 'Input_Ring', 'Outport', 1), 30, 14);
wire(sub, 'Input_Ring/1', 'ring/1', 'ring');
end

function buildSensor(sub, sensor, fallback, groups)
% Sensor message -> raw sample: rows of message fields feed the stamp, vector and covariance
% blocks, and the frame_id is classified against base_link_frame and the sensor's fallback.
fields = {'header.stamp'};
for g = 1:size(groups, 1)
    if strcmp(groups{g, 1}, 'vector')
        fields = [fields, groups{g, 3}]; %#ok<AGROW>
    else
        fields = [fields, groups{g, 3}(1)]; %#ok<AGROW>
    end
end
[rows, layout] = unpack(sub, fields, {'frames.base_link_frame', ['frames.' fallback]});

outputs = {'stamp'};
r = 2;
for g = 1:size(groups, 1)
    name = groups{g, 2};
    if strcmp(groups{g, 1}, 'vector')
        k = numel(groups{g, 3});
        vector(sub, name, rows, r, k, layout);
        r = r + k;
    else
        covariance(sub, name, rows(r), groups{g, 3}{2}, groups{g, 3}{3}, layout);
        r = r + 1;
    end
    outputs{end + 1} = name; %#ok<AGROW>
end
frameKind(sub, 'Frame_Kind', layout, ...
    ['function frame = kind(frame_id, n, base, mount)' newline ...
    'frame = talos_nav_frame_kind(frame_id, n, base, mount);' newline]);
outputs{end + 1} = 'Frame_Kind';
pack(sub, ['talos_nav_raw_' sensor], outputs, layout);
end

function buildDepth(sub)
% Depth sample: z in world_frame, moved there through the overseer's depth mount when it
% arrives in depth_frame.
fields = {'header.stamp', 'pose.pose.position.x', 'pose.pose.position.y', 'pose.pose.position.z', ...
    'pose.covariance'};
params = {'frames.base_link_frame', 'frames.world_frame', 'mounts.depth_frame', 'mounts.depth_mount_valid', ...
    'mounts.depth_rotation', 'mounts.depth_translation'};
[rows, layout] = unpack(sub, fields, params);
vector(sub, 'position', rows, 2, 3, layout);
put(sub, 'variance', 'simulink/Signal Routing/Selector', layout.process, rows(5), 70, 24, ...
    'NumberOfDimensions', '1', 'IndexOptions', 'Index vector (dialog)', 'Indices', '15', 'InputPortWidth', '36');
wire(sub, 'fields/5', 'variance/1', 'covariance');
frameKind(sub, 'Frame_Kind', layout, ...
    ['function [frame, mount] = kind(frame_id, n, base, world, depth_frame)' newline ...
    '% world kind, and whether the sample is in the overseer''s depth_frame' newline ...
    'frame = talos_nav_world_kind(frame_id, n, base, world);' newline ...
    'mount = talos_nav_frame_match(frame_id, n, depth_frame);' newline]);

% the mount stage sits right of the unpacking, fed by the vector, kind and mount parameters
x = layout.pack - 220;
put(sub, 'Depth_Mount', 'built-in/SubSystem', x, 0, 140, 6 * 30);
depthMount([sub '/Depth_Mount']);
snap(sub, 'Depth_Mount', 'Inport', 1, portY(sub, 'position', 'Outport', 1));
wire(sub, 'position/1', 'Depth_Mount/1', 'position');
wire(sub, 'Frame_Kind/1', 'Depth_Mount/2', 'kind');
wire(sub, 'Frame_Kind/2', 'Depth_Mount/3', 'in_depth_frame');
wire(sub, 'frame_params/4', 'Depth_Mount/4', 'depth_mount_valid');
wire(sub, 'frame_params/5', 'Depth_Mount/5', 'depth_rotation');
wire(sub, 'frame_params/6', 'Depth_Mount/6', 'depth_translation');
pack(sub, 'talos_nav_raw_depth', {'stamp', {'Depth_Mount', 1}, 'variance', {'Depth_Mount', 2}}, layout);
end

function depthMount(sub)
% A sample in depth_frame is rotated and translated into world_frame once the mount is valid.
names = {'position', 'kind', 'in_depth_frame', 'mount_valid', 'rotation', 'translation'};
ys = [85 230 260 290 55 180];
for i = 1:numel(names)
    put(sub, names{i}, 'simulink/Sources/In1', 40, ys(i), 30, 14);
end
put(sub, 'Rotation_Matrix', 'simulink/User-Defined Functions/MATLAB Function', 110, 55, 100, 40, 'ShowName', 'off');
script([sub '/Rotation_Matrix'], ['function R = rotation(q)' newline ...
    '% body to world matrix of the quaternion [w; x; y; z]' newline 'R = talos_nav_rotation(q);' newline]);
put(sub, 'rotate', 'simulink/Math Operations/Product', 260, 70, 40, 60, 'Multiplication', 'Matrix(*)');
put(sub, 'translate', 'simulink/Math Operations/Sum', 360, 0, 30, 60, 'Inputs', '++', 'IconShape', 'rectangular');
snap(sub, 'translate', 'Inport', 1, portY(sub, 'rotate', 'Outport', 1));
put(sub, 'moved_z', 'simulink/Signal Routing/Selector', 440, portY(sub, 'translate', 'Outport', 1), 50, 24, ...
    'NumberOfDimensions', '1', 'IndexOptions', 'Index vector (dialog)', 'Indices', '3', 'InputPortWidth', '3');
put(sub, 'z', 'simulink/Signal Routing/Switch', 640, 0, 40, 90, 'Criteria', 'u2 ~= 0');
snap(sub, 'z', 'Inport', 1, portY(sub, 'moved_z', 'Outport', 1));
put(sub, 'raw_z', 'simulink/Signal Routing/Selector', 440, portY(sub, 'z', 'Inport', 3), 50, 24, ...
    'NumberOfDimensions', '1', 'IndexOptions', 'Index vector (dialog)', 'Indices', '3', 'InputPortWidth', '3');
wire(sub, 'rotation/1', 'Rotation_Matrix/1');
wire(sub, 'Rotation_Matrix/1', 'rotate/1', 'R');
wire(sub, 'position/1', 'rotate/2');
wire(sub, 'rotate/1', 'translate/1');
wire(sub, 'translation/1', 'translate/2');
wire(sub, 'translate/1', 'moved_z/1', 'moved');
wire(sub, 'position/1', 'raw_z/1');
wire(sub, 'moved_z/1', 'z/1');
wire(sub, 'raw_z/1', 'z/3');

% the mount applies to a frame outside base link and world that is depth_frame, once resolved
tagOut(sub, 'kind', 1, 'Kind', 110);
put(sub, 'other_frame', 'simulink/Logic and Bit Operations/Compare To Constant', 320, 230, 60, 24, ...
    'relop', '==', 'const', '3');
from(sub, 'Kind', 'other_frame', 1, 220);
put(sub, 'use_mount', 'simulink/Logic and Bit Operations/Logical Operator', 440, 260, 40, 90, ...
    'Operator', 'AND', 'Inputs', '3');
snap(sub, 'other_frame', 'Outport', 1, portY(sub, 'use_mount', 'Inport', 1));
snap(sub, 'in_depth_frame', 'Outport', 1, portY(sub, 'use_mount', 'Inport', 2));
snap(sub, 'mount_valid', 'Outport', 1, portY(sub, 'use_mount', 'Inport', 3));
wire(sub, 'other_frame/1', 'use_mount/1');
wire(sub, 'in_depth_frame/1', 'use_mount/2');
wire(sub, 'mount_valid/1', 'use_mount/3');
tagOut(sub, 'use_mount', 1, 'UseMount', 510);
from(sub, 'UseMount', 'z', 2, 530);
put(sub, 'frame', 'simulink/Signal Routing/Switch', 640, 330, 40, 90, 'Criteria', 'u2 ~= 0');
put(sub, 'world_frame', 'simulink/Sources/Constant', 560, portY(sub, 'frame', 'Inport', 1), 40, 24, 'Value', '2');
from(sub, 'UseMount', 'frame', 2, 530);
from(sub, 'Kind', 'frame', 3, 530);
wire(sub, 'world_frame/1', 'frame/1');
put(sub, 'z_world', 'simulink/Sinks/Out1', 740, portY(sub, 'z', 'Outport', 1), 30, 14);
put(sub, 'frame_world', 'simulink/Sinks/Out1', 740, portY(sub, 'frame', 'Outport', 1), 30, 14);
wire(sub, 'z/1', 'z_world/1');
wire(sub, 'frame/1', 'frame_world/1');
end

function buildSetPose(sub)
% A reset that arrives before the estimator starts is held for its first step.
put(sub, 'IsNew', 'simulink/Sources/In1', 40, 60, 30, 14);
put(sub, 'Msg', 'simulink/Sources/In1', 40, 100, 30, 14);
put(sub, 'param_bus', 'simulink/Sources/In1', 40, 140, 30, 14);
put(sub, 'started', 'simulink/Sources/In1', 40, 180, 30, 14);
put(sub, 'Hold_Until_Start', 'built-in/SubSystem', 140, 120, 140, 120);
holdUntilStart([sub '/Hold_Until_Start']);
wire(sub, 'IsNew/1', 'Hold_Until_Start/1');
wire(sub, 'Msg/1', 'Hold_Until_Start/2');
wire(sub, 'started/1', 'Hold_Until_Start/3');

fields = {'header.stamp', 'pose.pose.position.x', 'pose.pose.position.y', 'pose.pose.position.z', ...
    'pose.pose.orientation.w', 'pose.pose.orientation.x', 'pose.pose.orientation.y', 'pose.pose.orientation.z', ...
    'pose.covariance'};
[rows, layout] = unpack(sub, fields, {'frames.base_link_frame', 'frames.world_frame'}, ...
    struct('msg', 'Hold_Until_Start/2', 'valid', 'Hold_Until_Start/1', 'x', 360));
vector(sub, 'pos', rows, 2, 3, layout);
vector(sub, 'quat', rows, 5, 4, layout);
covariance(sub, 'cov', rows(9), 6, [], layout);
frameKind(sub, 'Frame_Kind', layout, ...
    ['function frame = kind(frame_id, n, base, world)' newline ...
    'frame = talos_nav_world_kind(frame_id, n, base, world);' newline]);
pack(sub, 'talos_nav_raw_set_pose', {'stamp', 'pos', 'quat', 'cov', 'Frame_Kind'}, layout);
end

function holdUntilStart(sub)
% valid while a reset is new or held; held until the first started step.
put(sub, 'IsNew', 'simulink/Sources/In1', 40, 60, 30, 14);
put(sub, 'Msg', 'simulink/Sources/In1', 40, 180, 30, 14);
put(sub, 'started', 'simulink/Sources/In1', 40, 330, 30, 14);
tagOut(sub, 'IsNew', 1, 'New', 100);

put(sub, 'arrived', 'simulink/Logic and Bit Operations/Logical Operator', 340, 75, 40, 60, 'Operator', 'OR');
from(sub, 'New', 'arrived', 1, 220);
from(sub, 'Pending', 'arrived', 2, 220);
put(sub, 'valid', 'simulink/Sinks/Out1', 560, portY(sub, 'arrived', 'Outport', 1), 30, 14);
wire(sub, 'arrived/1', 'valid/1', 'valid');
tagOut(sub, 'arrived', 1, 'Arrived', 440);
moveBlock([sub '/Arrived_goto'], [0 40]);

put(sub, 'latest', 'simulink/Signal Routing/Switch', 340, 0, 40, 90, 'Criteria', 'u2 ~= 0');
snap(sub, 'latest', 'Inport', 1, 180);
from(sub, 'New', 'latest', 2, 220);
from(sub, 'Held', 'latest', 3, 220);
wire(sub, 'Msg/1', 'latest/1');
put(sub, 'msg', 'simulink/Sinks/Out1', 560, portY(sub, 'latest', 'Outport', 1), 30, 14);
wire(sub, 'latest/1', 'msg/1', 'msg');
put(sub, 'held', 'simulink/Discrete/Unit Delay', 440, portY(sub, 'latest', 'Outport', 1) + 60, 40, 30, ...
    'InitialCondition', '0', 'SampleTime', '-1');
wire(sub, 'latest/1', 'held/1');
tagOut(sub, 'held', 1, 'Held', 520);

put(sub, 'waiting', 'simulink/Logic and Bit Operations/Logical Operator', 120, 330, 30, 30, 'Operator', 'NOT');
put(sub, 'keep', 'simulink/Logic and Bit Operations/Logical Operator', 340, 0, 40, 60, 'Operator', 'AND');
snap(sub, 'keep', 'Inport', 1, 330);
from(sub, 'Arrived', 'keep', 2, 220);
wire(sub, 'started/1', 'waiting/1');
wire(sub, 'waiting/1', 'keep/1');
put(sub, 'pending', 'simulink/Discrete/Unit Delay', 440, portY(sub, 'keep', 'Outport', 1), 40, 30, ...
    'InitialCondition', 'false', 'SampleTime', '-1');
wire(sub, 'keep/1', 'pending/1');
tagOut(sub, 'pending', 1, 'Pending', 520);
end

function [rows, layout] = unpack(sub, fields, params, opts)
% Inports and the field selectors: one row per message field, then the frame name parameters.
% Returns the row centers and the column positions shared by the sensor builders.
P = 30;
top = 100;
layout.select = 160;
if nargin > 3
    layout.select = opts.x;
end
layout.process = layout.select + 140;
layout.transpose = layout.process + 170;
layout.block = layout.transpose + 90;
layout.pack = layout.block + 330;
layout.top = top;
% the frame_id and its length get their own selector a row below, so the frame
% classification has room above its ports
n = numel(fields);
add_block('simulink/Signal Routing/Bus Selector', [sub '/fields'], 'OutputSignals', strjoin(fields, ','), ...
    'Position', [layout.select top layout.select + 5 top + n * P], 'ShowName', 'off');
add_block('simulink/Signal Routing/Bus Selector', [sub '/frame_fields'], ...
    'OutputSignals', 'header.frame_id,header.frame_id_SL_Info.CurrentLength', ...
    'Position', [layout.select top + (n + 1) * P layout.select + 5 top + (n + 3) * P], 'ShowName', 'off');
add_block('simulink/Signal Routing/Bus Selector', [sub '/frame_params'], 'OutputSignals', strjoin(params, ','), ...
    'Position', [layout.select top + (n + 3) * P layout.select + 5 top + (n + 3 + numel(params)) * P], 'ShowName', 'off');
rows = top + ((1:n) - 0.5) * P;
layout.frame = top + (n + 1.5) * P;
if nargin > 3
    wire(sub, opts.msg, 'fields/1', 'msg');
    wire(sub, opts.msg, 'frame_fields/1', 'msg');
    layout.valid = opts.valid;
else
    put(sub, 'IsNew', 'simulink/Sources/In1', 40, top - 40, 30, 14);
    put(sub, 'Msg', 'simulink/Sources/In1', 40, portY(sub, 'fields', 'Inport', 1), 30, 14);
    put(sub, 'param_bus', 'simulink/Sources/In1', 40, portY(sub, 'frame_params', 'Inport', 1), 30, 14);
    wire(sub, 'Msg/1', 'fields/1');
    wire(sub, 'Msg/1', 'frame_fields/1');
    layout.valid = 'IsNew/1';
end
wire(sub, 'param_bus/1', 'frame_params/1');
put(sub, 'stamp', 'built-in/SubSystem', layout.process, rows(1), 110, 24);
buildStamp([sub '/stamp'], 'stamp', 'ns');
wire(sub, 'fields/1', 'stamp/1', 'header.stamp');
end

function vector(sub, name, rows, first, k, layout)
% Consecutive scalar fields -> one vector.
P = 30;
y = rows(first) - P / 2;
add_block('simulink/Signal Routing/Vector Concatenate', [sub '/' name], 'NumInputs', num2str(k), ...
    'Mode', 'Vector', 'Position', [layout.process y layout.process + 5 y + k * P], 'ShowName', 'off');
for i = 1:k
    wire(sub, sprintf('fields/%d', first + i - 1), sprintf('%s/%d', name, i));
end
end

function covariance(sub, name, y, n, block, layout)
% ROS covariance array -> row-major n x n matrix, optionally one 3x3 block of it.
put(sub, [name '_matrix'], 'simulink/Math Operations/Reshape', layout.process, y, 60, 24, ...
    'OutputDimensionality', 'Customize', 'OutputDimensions', sprintf('[%d %d]', n, n));
put(sub, [name '_rows'], 'simulink/Math Operations/Math Function', layout.transpose, y, 50, 24, 'Operator', 'transpose');
port = round((y - layout.top) / 30 + 0.5);
wire(sub, sprintf('fields/%d', port), [name '_matrix/1']);
wire(sub, [name '_matrix/1'], [name '_rows/1']);
if ~isempty(block)
    idx = sprintf('%d:%d', block(1), block(end));
    put(sub, name, 'simulink/Signal Routing/Selector', layout.block, y, 70, 24, 'NumberOfDimensions', '2', ...
        'IndexOptions', 'Index vector (dialog),Index vector (dialog)', 'Indices', [idx ',' idx]);
    wire(sub, [name '_rows/1'], [name '/1']);
else
    set_param([sub '/' name '_rows'], 'Name', name);
end
end

function frameKind(sub, name, layout, code)
% Frame classification: string comparison, so a MATLAB function. Its ports sit on the
% frame_id, length and frame parameter rows.
args = regexp(code, '\(([^)]*)\)', 'tokens', 'once');
nIn = numel(strsplit(args{1}, ','));
put(sub, name, 'simulink/User-Defined Functions/MATLAB Function', layout.process, 0, 120, (nIn + 1) * 30);
script([sub '/' name], code);
snap(sub, name, 'Inport', 1, layout.frame);
wire(sub, 'frame_fields/1', [name '/1']);
wire(sub, 'frame_fields/2', [name '/2']);
for i = 3:nIn
    wire(sub, sprintf('frame_params/%d', i - 2), sprintf('%s/%d', name, i));
end
end

function pack(sub, bus, sources, layout)
% Raw sample bus: valid first, then each source in the bus element order.
n = numel(sources) + 1;
srcY = zeros(1, numel(sources));
for i = 1:numel(sources)
    [b, port] = source(sources{i});
    srcY(i) = portY(sub, b, 'Outport', port);
end
top = layout.top - 40;
bottom = max(srcY) + 30;
add_block('simulink/Signal Routing/Bus Creator', [sub '/sample'], 'Inputs', num2str(n), ...
    'Position', [layout.pack top layout.pack + 5 bottom], 'OutDataTypeStr', ['Bus: ' bus], ...
    'NonVirtualBus', 'on', 'ShowName', 'off');
object = evalin('base', bus);
names = {object.Elements.Name};
wire(sub, layout.valid, 'sample/1', names{1});
for i = 1:numel(sources)
    [b, port] = source(sources{i});
    wire(sub, sprintf('%s/%d', b, port), sprintf('sample/%d', i + 1), names{i + 1});
end
put(sub, 'raw_sample', 'simulink/Sinks/Out1', layout.pack + 70, portY(sub, 'sample', 'Outport', 1), 30, 14);
wire(sub, 'sample/1', 'raw_sample/1');
end

function [b, port] = source(s)
if iscell(s)
    b = s{1};
    port = s{2};
else
    b = s;
    port = 1;
end
end

%% Estimator

function estimator(sub, nDiag)
% Schedule -> Fusion (queue, gating, replay around the EKF block) -> published covariances.
put(sub, 'param_bus', 'simulink/Sources/In1', 40, 100, 30, 14);
put(sub, 'ring', 'simulink/Sources/In1', 40, 200, 30, 14);
put(sub, 'now', 'simulink/Sources/In1', 40, 300, 30, 14);
put(sub, 'started', 'simulink/Sources/In1', 40, 400, 30, 14);
tagOut(sub, 'param_bus', 1, 'ParamBus', 110);
tagOut(sub, 'ring', 1, 'Ring', 110);
tagOut(sub, 'now', 1, 'Now', 110);
tagOut(sub, 'started', 1, 'Started', 110);

% cadence of publications and diagnostics
put(sub, 'cadence', 'simulink/Signal Routing/Bus Selector', 360, 0, 5, 60, ...
    'OutputSignals', 'timing.frequency,diagnostics.diagnostics_period', 'ShowName', 'off');
put(sub, 'Schedule', 'built-in/SubSystem', 460, 420, 150, 120);
schedule([sub '/Schedule']);
snap(sub, 'cadence', 'Outport', 1, portY(sub, 'Schedule', 'Inport', 2));
from(sub, 'Now', 'Schedule', 1, 340);
from(sub, 'ParamBus', 'cadence', 1, 240);
from(sub, 'Started', 'Schedule', 'Enable', 360);
wire(sub, 'cadence/1', 'Schedule/2', 'frequency');
wire(sub, 'cadence/2', 'Schedule/3', 'diagnostics_period');
tagOut(sub, 'Schedule', 1, 'Publish', 640);
tagOut(sub, 'Schedule', 2, 'Diagnostics', 640);

% the fusion logic, which runs the EKF block through talos_nav_ekf_step
fcn = put(sub, 'Fusion', 'simulink/User-Defined Functions/MATLAB Function', 880, 260, 220, 6 * 60);
chart = script(fcn, ['function [pub, x, P, pose_cov, d, taken] = fusion(param_bus, ring, now, started, publish, diagnostics)' newline ...
    '% see talos_nav_fusion; the filter is the Extended Kalman Filter block' newline ...
    '[pub, x, P, pose_cov, d, taken] = talos_nav_fusion(param_bus, ring, now, started, publish, diagnostics, @ekf);' newline ...
    'end' newline newline ...
    'function [x, P] = ekf(x, P, dt, Sq, sensor, y, R, ctx)' newline ...
    '[x, P] = talos_nav_ekf(@step, x, P, dt, Sq, sensor, y, R, ctx);' newline ...
    'end' newline newline ...
    'function [x, P] = step(mode, dt, Sq, x0, sensor, y, R, ctx)' newline ...
    '[x, P] = talos_nav_ekf_step(mode, dt, Sq, x0, sensor, y, R, ctx);' newline ...
    'end' newline]);
types = {'param_bus', 'Input', 'Bus: talos_nav_params', '-1'; 'pub', 'Output', 'boolean', '1'; 'x', 'Output', 'double', '19'; 'P', 'Output', 'double', '[19 19]'; ...
    'pose_cov', 'Output', 'double', '[6 6]'; 'd', 'Output', 'double', num2str(nDiag); ...
    'taken', 'Output', 'double', '1'};
for i = 1:size(types, 1)
    data = chart.find('-isa', 'Stateflow.Data', 'Name', types{i, 1}, 'Scope', types{i, 2});
    data.DataType = types{i, 3};
    data.Props.Array.Size = types{i, 4};
end
tagIn(sub, 'Fusion', {'ParamBus', 'Ring', 'Now', 'Started', 'Publish', 'Diagnostics'}, 760);

put(sub, 'Extended_Kalman_Filter', 'simulink/User-Defined Functions/Simulink Function', 880, 620, 220, 120);
ekfFunction([sub '/Extended_Kalman_Filter']);

% published covariances in ROS row-major order
put(sub, 'Covariances', 'built-in/SubSystem', 1180, 0, 150, 90);
covariances([sub '/Covariances']);
snap(sub, 'Covariances', 'Inport', 1, portY(sub, 'Fusion', 'Outport', 3));

add_block('simulink/Signal Routing/Bus Creator', [sub '/Estimate_Bus'], 'Inputs', '7', ...
    'Position', [1420 80 1425 500], 'ShowName', 'off');
wire(sub, 'Fusion/1', 'Estimate_Bus/1', 'pub');
wire(sub, 'Fusion/2', 'Estimate_Bus/2', 'x');
wire(sub, 'Fusion/3', 'Covariances/1', 'P');
wire(sub, 'Fusion/4', 'Covariances/2', 'pose_cov');
wire(sub, 'Covariances/1', 'Estimate_Bus/3', 'pose_cov');
wire(sub, 'Covariances/2', 'Estimate_Bus/4', 'twist_cov');
wire(sub, 'Covariances/3', 'Estimate_Bus/5', 'accel_cov');
from(sub, 'Diagnostics', 'Estimate_Bus', 6, 1280);
set_param(get_param([sub '/Estimate_Bus'], 'LineHandles').Inport(6), 'Name', 'diag_pub');
wire(sub, 'Fusion/5', 'Estimate_Bus/7', 'diag');
put(sub, 'estimate', 'simulink/Sinks/Out1', 1500, portY(sub, 'Estimate_Bus', 'Outport', 1), 30, 14);
wire(sub, 'Estimate_Bus/1', 'estimate/1', 'estimate');
put(sub, 'taken', 'simulink/Sinks/Out1', 1200, portY(sub, 'Fusion', 'Outport', 6), 30, 14);
wire(sub, 'Fusion/6', 'taken/1', 'taken');

area(sub, 'Publication and diagnostics cadence', [200 300 745 520], 'cream');
area(sub, 'Fusion around the Extended Kalman Filter block', [750 30 1150 720], 'blue');
end

function schedule(sub)
% Runs once started: the publication clock keeps its phase, the diagnostics clock restarts.
put(sub, 'now', 'simulink/Sources/In1', 40, 80, 30, 14);
put(sub, 'frequency', 'simulink/Sources/In1', 40, 200, 30, 14);
put(sub, 'diagnostics_period', 'simulink/Sources/In1', 40, 360, 30, 14);
add_block('simulink/Ports & Subsystems/Enable', [sub '/started'], 'Position', [300 20 320 40], ...
    'StatesWhenEnabling', 'held');
put(sub, 'Publication_Clock', 'built-in/SubSystem', 300, 140, 160, 120);
cadenceClock([sub '/Publication_Clock'], true);
put(sub, 'Diagnostics_Clock', 'built-in/SubSystem', 300, 330, 160, 120);
cadenceClock([sub '/Diagnostics_Clock'], false);
tagOut(sub, 'now', 1, 'Now', 100);
from(sub, 'Now', 'Publication_Clock', 1, 180);
from(sub, 'Now', 'Diagnostics_Clock', 1, 180);
snap(sub, 'frequency', 'Outport', 1, portY(sub, 'Publication_Clock', 'Inport', 2));
snap(sub, 'diagnostics_period', 'Outport', 1, portY(sub, 'Diagnostics_Clock', 'Inport', 2));
wire(sub, 'frequency/1', 'Publication_Clock/2', 'frequency');
wire(sub, 'diagnostics_period/1', 'Diagnostics_Clock/2', 'diagnostics_period');
put(sub, 'publish', 'simulink/Sinks/Out1', 540, portY(sub, 'Publication_Clock', 'Outport', 1), 30, 14, ...
    'OutputWhenDisabled', 'reset', 'InitialOutput', 'false');
put(sub, 'diagnostics', 'simulink/Sinks/Out1', 540, portY(sub, 'Diagnostics_Clock', 'Outport', 1), 30, 14, ...
    'OutputWhenDisabled', 'reset', 'InitialOutput', 'false');
wire(sub, 'Publication_Clock/1', 'publish/1', 'publish');
wire(sub, 'Diagnostics_Clock/1', 'diagnostics/1', 'diagnostics');
end

function cadenceClock(sub, keepPhase)
% due when now reaches the next deadline. A publication deadline advances by one period, or to
% a period after now if it fell behind; a diagnostics deadline restarts a period after now.
c = [40 200 330 450 570 690 810 930 1050 1170];
put(sub, 'now', 'simulink/Sources/In1', c(1), 180, 30, 14);

% period in int64 nanoseconds
if keepPhase
    put(sub, 'frequency', 'simulink/Sources/In1', c(1), 90, 30, 14);
    put(sub, 'one_second', 'simulink/Sources/Constant', c(2), 60, 60, 24, 'Value', '1e9');
    put(sub, 'seconds', 'simulink/Math Operations/Divide', c(3), 75, 40, 60);
    snap(sub, 'one_second', 'Outport', 1, portY(sub, 'seconds', 'Inport', 1));
    snap(sub, 'frequency', 'Outport', 1, portY(sub, 'seconds', 'Inport', 2));
    wire(sub, 'one_second/1', 'seconds/1');
    wire(sub, 'frequency/1', 'seconds/2');
else
    put(sub, 'period_s', 'simulink/Sources/In1', c(1), 75, 30, 14);
    put(sub, 'seconds', 'simulink/Math Operations/Gain', c(3), 75, 60, 24, 'Gain', '1e9');
    wire(sub, 'period_s/1', 'seconds/1');
end
put(sub, 'nearest', 'simulink/Math Operations/Rounding Function', c(4), 75, 60, 24, 'Operator', 'round');
put(sub, 'period', 'simulink/Signal Attributes/Data Type Conversion', c(5), 75, 60, 24, 'OutDataTypeStr', 'int64');
wire(sub, 'seconds/1', 'nearest/1');
wire(sub, 'nearest/1', 'period/1');
tagOut(sub, 'period', 1, 'Period', c(6));

% due: now > 0 and now >= deadline
tagOut(sub, 'now', 1, 'Now', 90);
put(sub, 'positive', 'simulink/Logic and Bit Operations/Compare To Zero', c(3), 180, 50, 30, 'relop', '>');
put(sub, 'reached', 'simulink/Logic and Bit Operations/Relational Operator', c(3), 250, 50, 60, 'Operator', '>=');
put(sub, 'due', 'simulink/Logic and Bit Operations/Logical Operator', c(4), 215, 40, 70, 'Operator', 'AND');
from(sub, 'Now', 'positive', 1, c(2));
from(sub, 'Now', 'reached', 1, c(2));
from(sub, 'Deadline', 'reached', 2, c(2));
wire(sub, 'positive/1', 'due/1');
wire(sub, 'reached/1', 'due/2');
tagOut(sub, 'due', 1, 'Due', c(5));
put(sub, 'is_due', 'simulink/Sinks/Out1', c(10), 215, 30, 14);
from(sub, 'Due', 'is_due', 1, c(9));

% next deadline, held between due steps
put(sub, 'restart', 'simulink/Math Operations/Sum', c(3), 360, 30, 60, 'Inputs', '++', 'IconShape', 'rectangular');
from(sub, 'Now', 'restart', 1, c(2));
from(sub, 'Period', 'restart', 2, c(2));
put(sub, 'next', 'simulink/Signal Routing/Switch', c(8), 0, 40, 90, 'Criteria', 'u2 ~= 0');
if keepPhase
    put(sub, 'advance', 'simulink/Math Operations/Sum', c(3), 460, 30, 60, 'Inputs', '++', 'IconShape', 'rectangular');
    from(sub, 'Deadline', 'advance', 1, c(2));
    from(sub, 'Period', 'advance', 2, c(2));
    put(sub, 'behind', 'simulink/Logic and Bit Operations/Relational Operator', c(5), 0, 50, 60, 'Operator', '<=');
    snap(sub, 'behind', 'Inport', 1, portY(sub, 'advance', 'Outport', 1));
    from(sub, 'Now', 'behind', 2, c(4));
    put(sub, 'catch_up', 'simulink/Signal Routing/Switch', c(6), 0, 40, 90, 'Criteria', 'u2 ~= 0');
    snap(sub, 'catch_up', 'Inport', 2, portY(sub, 'behind', 'Outport', 1));
    wire(sub, 'advance/1', 'behind/1');
    wire(sub, 'restart/1', 'catch_up/1');
    wire(sub, 'behind/1', 'catch_up/2');
    wire(sub, 'advance/1', 'catch_up/3');
    snap(sub, 'next', 'Inport', 1, portY(sub, 'catch_up', 'Outport', 1));
    wire(sub, 'catch_up/1', 'next/1');
else
    snap(sub, 'next', 'Inport', 1, portY(sub, 'restart', 'Outport', 1));
    wire(sub, 'restart/1', 'next/1');
end
from(sub, 'Due', 'next', 2, c(7));
from(sub, 'Deadline', 'next', 3, c(7));
put(sub, 'deadline', 'simulink/Discrete/Unit Delay', c(9), portY(sub, 'next', 'Outport', 1), 40, 30, ...
    'InitialCondition', 'int64(0)', 'SampleTime', '-1');
wire(sub, 'next/1', 'deadline/1');
tagOut(sub, 'deadline', 1, 'Deadline', c(10));
end

function covariances(sub)
% Row-major 36-element arrays: the pose block as published, the twist block and the
% acceleration block padded to 6x6.
put(sub, 'P', 'simulink/Sources/In1', 40, 120, 30, 14);
put(sub, 'pose_block', 'simulink/Sources/In1', 40, 50, 30, 14);
put(sub, 'twist_block', 'simulink/Signal Routing/Selector', 140, 120, 80, 24, 'NumberOfDimensions', '2', ...
    'IndexOptions', 'Index vector (dialog),Index vector (dialog)', 'Indices', '8:13,8:13');
put(sub, 'pad_columns', 'simulink/Math Operations/Matrix Concatenate', 260, 220, 5, 60, ...
    'NumInputs', '2', 'ConcatenateDimension', '2', 'ShowName', 'off');
put(sub, 'accel_block', 'simulink/Signal Routing/Selector', 140, portY(sub, 'pad_columns', 'Inport', 1), 80, 24, ...
    'NumberOfDimensions', '2', 'IndexOptions', 'Index vector (dialog),Index vector (dialog)', 'Indices', '14:16,14:16');
put(sub, 'zeros_3x3', 'simulink/Sources/Constant', 140, portY(sub, 'pad_columns', 'Inport', 2), 80, 24, 'Value', 'zeros(3)');
put(sub, 'pad_rows', 'simulink/Math Operations/Matrix Concatenate', 380, 0, 5, 60, ...
    'NumInputs', '2', 'ConcatenateDimension', '1', 'ShowName', 'off');
snap(sub, 'pad_rows', 'Inport', 1, portY(sub, 'pad_columns', 'Outport', 1));
put(sub, 'zeros_3x6', 'simulink/Sources/Constant', 280, portY(sub, 'pad_rows', 'Inport', 2) + 20, 80, 24, ...
    'Value', 'zeros(3, 6)');
snap(sub, 'zeros_3x6', 'Outport', 1, portY(sub, 'pad_rows', 'Inport', 2));
wire(sub, 'P/1', 'twist_block/1');
wire(sub, 'P/1', 'accel_block/1');
wire(sub, 'accel_block/1', 'pad_columns/1');
wire(sub, 'zeros_3x3/1', 'pad_columns/2');
wire(sub, 'pad_columns/1', 'pad_rows/1');
wire(sub, 'zeros_3x6/1', 'pad_rows/2');
sources = {'pose_block', 'twist_block', 'pad_rows'};
names = {'pose', 'twist', 'accel'};
for i = 1:3
    y = portY(sub, sources{i}, 'Outport', 1);
    put(sub, [names{i} '_rows'], 'simulink/Math Operations/Math Function', 480, y, 50, 24, 'Operator', 'transpose');
    put(sub, [names{i} '_row_major'], 'simulink/Math Operations/Reshape', 570, y, 50, 24, ...
        'OutputDimensionality', '1-D array');
    put(sub, [names{i} '_out'], 'simulink/Sinks/Out1', 670, y, 30, 14);
    wire(sub, [sources{i} '/1'], [names{i} '_rows/1']);
    wire(sub, [names{i} '_rows/1'], [names{i} '_row_major/1']);
    wire(sub, [names{i} '_row_major/1'], [names{i} '_out/1'], [names{i} '_cov']);
end
set_param([sub '/pose_out'], 'Name', 'pose_cov');
set_param([sub '/twist_out'], 'Name', 'twist_cov');
set_param([sub '/accel_out'], 'Name', 'accel_cov');
end

function ekfFunction(fcn)
% talos_nav_ekf_step runs the library Extended Kalman Filter block once per call, with the
% transition mode in its context: 0 hold, 1 predict with root Sq, 2 load x0 with root Sq.
% It is scoped to the Estimator, where Fusion calls it; C++ class code allows no global one.
P = 60;
trig = find_system(fcn, 'SearchDepth', 1, 'BlockType', 'TriggerPort');
set_param(trig{1}, 'FunctionName', 'talos_nav_ekf_step', 'FunctionVisibility', 'scoped', ...
    'FunctionPrototype', '[x, P] = talos_nav_ekf_step(mode, dt, Sq, x0, sensor, y, R, ctx)', ...
    'Position', [600 20 620 40]);
delete_line(find_system(fcn, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'line'));
dims = struct('mode', '1', 'dt', '1', 'Sq', '[19 19]', 'x0', '19', 'sensor', '1', ...
    'y', '9', 'R', '[9 9]', 'ctx', '12', 'x', '19', 'P', '[19 19]');
args = [find_system(fcn, 'SearchDepth', 1, 'BlockType', 'ArgIn'); find_system(fcn, 'SearchDepth', 1, 'BlockType', 'ArgOut')];
h = get_param(args, 'Handle');
for i = 1:numel(h), set_param(h{i}, 'Name', sprintf('arg_%d', i)); end
for i = 1:numel(h)
    n = get_param(h{i}, 'ArgumentName');
    set_param(h{i}, 'Name', n, 'PortDimensions', dims.(n), 'OutDataTypeStr', 'double');
end

ekf = [fcn '/EKF'];
top = 80;
add_block('sharedTrackingLibrary/Extended Kalman Filter', ekf, 'Position', [900 top 1100 top + 18 * P]);
set_param(ekf, 'StateTransitionFcn', 'talos_nav_transition', ...
    'HasStateTransitionJacobianFcn', 'on', 'StateTransitionJacobianFcn', 'talos_nav_transition_jacobian', ...
    'HasAdditiveProcessNoise', 'Nonadditive', 'ProcessNoise', 'eye(19)', 'HasTimeVaryingProcessNoise', 'off', ...
    'HasStateTransitionFcnExtraArgument', '1', 'InitialState', '[zeros(3, 1); 1; zeros(15, 1)]', ...
    'InitialStateCovariance', 'eye(19)', 'NumberOfMeasurements', '4', 'OutputStateCovariance', 'on', ...
    'UseCurrentEstimator', 'on', 'SampleTime', '-1');
sensors = {'imu', 'fog', 'dvl', 'depth'};
for i = 1:4
    n = num2str(i);
    set_param(ekf, ['MeasurementFcn' n], ['talos_measure_' sensors{i} '_configured'], ...
        ['HasMeasurementJacobianFcn' n], 'on', ...
        ['MeasurementJacobianFcn' n], ['talos_measure_' sensors{i} '_configured_jacobian'], ...
        ['HasMeasurementEnablePort' n], 'on', ['HasAdditiveMeasurementNoise' n], 'Additive', ...
        ['HasTimeVaryingMeasurementNoise' n], 'on', ['HasMeasurementFcnExtraArgument' n], '1');
end

% arguments on Goto tags, so each block below reads only what it needs
names = {'mode', 'dt', 'Sq', 'x0', 'sensor', 'y', 'R', 'ctx'};
for i = 1:numel(names)
    y = top + 20 + 50 * (i - 1);
    put(fcn, names{i}, [], 40, y, 30, 14);
    tagOut(fcn, names{i}, 1, ['arg_' names{i}], 110);
end

% process model: the transition context
yC = portY(fcn, 'EKF', 'Inport', 1);
add_block('simulink/Signal Routing/Vector Concatenate', [fcn '/context'], 'NumInputs', '4', ...
    'Position', [540 yC - 60 545 yC + 60], 'ShowName', 'off');
put(fcn, 'Sq_columns', 'simulink/Math Operations/Reshape', 450, portY(fcn, 'context', 'Inport', 3), 60, 24, ...
    'OutputDimensionality', '1-D array', 'ShowName', 'off');
from(fcn, 'arg_mode', 'context', 1, 420);
from(fcn, 'arg_dt', 'context', 2, 420);
from(fcn, 'arg_Sq', 'Sq_columns', 1, 330);
from(fcn, 'arg_x0', 'context', 4, 420);
wire(fcn, 'Sq_columns/1', 'context/3');
wire(fcn, 'context/1', 'EKF/1', 'context');

% measurements: each sensor's enable, y, R and context, cut to its size
rows = [9 3 3 1];
widths = [12 3 6 1];
for i = 1:4
    base = 4 * (i - 1) + 1;
    s = sensors{i};
    put(fcn, ['is_' s], 'simulink/Logic and Bit Operations/Compare To Constant', 760, ...
        portY(fcn, 'EKF', 'Inport', base + 1), 60, 24, 'relop', '==', 'const', num2str(i));
    from(fcn, 'arg_sensor', ['is_' s], 1, 640);
    wire(fcn, ['is_' s '/1'], sprintf('EKF/%d', base + 1), ['enable_' s]);
    if i == 1
        from(fcn, 'arg_y', 'EKF', base + 2, 760);
        from(fcn, 'arg_R', 'EKF', base + 3, 760);
        from(fcn, 'arg_ctx', 'EKF', base + 4, 760);
        continue;
    end
    put(fcn, ['y_' s], 'simulink/Signal Routing/Selector', 760, portY(fcn, 'EKF', 'Inport', base + 2), 60, 24, ...
        'NumberOfDimensions', '1', 'IndexOptions', 'Index vector (dialog)', ...
        'Indices', sprintf('1:%d', rows(i)), 'InputPortWidth', '9');
    put(fcn, ['R_' s], 'simulink/Signal Routing/Selector', 760, portY(fcn, 'EKF', 'Inport', base + 3), 60, 24, ...
        'NumberOfDimensions', '2', 'IndexOptions', 'Index vector (dialog),Index vector (dialog)', ...
        'Indices', sprintf('1:%d,1:%d', rows(i), rows(i)));
    put(fcn, ['ctx_' s], 'simulink/Signal Routing/Selector', 760, portY(fcn, 'EKF', 'Inport', base + 4), 60, 24, ...
        'NumberOfDimensions', '1', 'IndexOptions', 'Index vector (dialog)', ...
        'Indices', sprintf('1:%d', widths(i)), 'InputPortWidth', '12');
    from(fcn, 'arg_y', ['y_' s], 1, 640);
    from(fcn, 'arg_R', ['R_' s], 1, 640);
    from(fcn, 'arg_ctx', ['ctx_' s], 1, 640);
    wire(fcn, ['y_' s '/1'], sprintf('EKF/%d', base + 2), ['y_' s]);
    wire(fcn, ['R_' s '/1'], sprintf('EKF/%d', base + 3), ['R_' s]);
    wire(fcn, ['ctx_' s '/1'], sprintf('EKF/%d', base + 4), ['ctx_' s]);
end

% the estimate after this run's correction, before its transition
put(fcn, 'x', [], 1200, portY(fcn, 'EKF', 'Outport', 1), 30, 14);
put(fcn, 'P', [], 1200, portY(fcn, 'EKF', 'Outport', 2), 30, 14);
wire(fcn, 'EKF/1', 'x/1', 'x');
wire(fcn, 'EKF/2', 'P/1', 'P');

note = add_block('built-in/Note', [fcn '/modes']);
set_param(note, 'Text', ['mode 0: hold the state' newline 'mode 1: predict over dt with process root Sq' newline ...
    'mode 2: load x0 with covariance root Sq' newline 'sensor 1-4: correct with the IMU, FOG,' newline 'DVL or depth measurement' newline ...
    'y, R and ctx are padded to 9, 9x9 and 12'], 'Position', [40 top + 520 280 top + 640]);
area(fcn, 'Process model', [310 top - 50 880 yC + 75], 'cream');
area(fcn, 'Measurements', [310 yC + 85 880 top + 18 * P + 20], 'blue');
end

%% ROS output

function publishSystem(sub, nDiag)
% Fill each message with Bus Assignment blocks and publish it on the steps the estimator says so.
put(sub, 'param_bus', 'simulink/Sources/In1', 40, 80, 30, 14);
put(sub, 'time', 'simulink/Sources/In1', 40, 360, 30, 14);
put(sub, 'estimate', 'simulink/Sources/In1', 40, 520, 30, 14);
put(sub, 'frames', 'simulink/Signal Routing/Bus Selector', 140, 80, 5, 90, ...
    'OutputSignals', 'frames.world_frame,frames.base_link_frame,frames.publish_tf', 'ShowName', 'off');
snap(sub, 'param_bus', 'Outport', 1, portY(sub, 'frames', 'Inport', 1));
signals = {'pub', 'x', 'pose_cov', 'twist_cov', 'accel_cov', 'diag_pub', 'diag'};
put(sub, 'estimate_signals', 'simulink/Signal Routing/Bus Selector', 140, 560, 5, 30 * numel(signals), ...
    'OutputSignals', strjoin(signals, ','), 'ShowName', 'off');
snap(sub, 'estimate', 'Outport', 1, portY(sub, 'estimate_signals', 'Inport', 1));
wire(sub, 'param_bus/1', 'frames/1');
wire(sub, 'estimate/1', 'estimate_signals/1');
put(sub, 'World_Frame_ID', 'built-in/SubSystem', 260, portY(sub, 'frames', 'Outport', 1), 130, 50);
put(sub, 'Base_Frame_ID', 'built-in/SubSystem', 260, portY(sub, 'frames', 'Outport', 2) + 70, 130, 50);
frameId([sub '/World_Frame_ID']);
frameId([sub '/Base_Frame_ID']);
wire(sub, 'frames/1', 'World_Frame_ID/1', 'world_frame');
wire(sub, 'frames/2', 'Base_Frame_ID/1', 'base_link_frame');
tagOut(sub, 'World_Frame_ID', 1, 'WorldId', 430);
tagOut(sub, 'World_Frame_ID', 2, 'WorldLength', 430);
tagOut(sub, 'Base_Frame_ID', 1, 'BaseId', 430);
tagOut(sub, 'Base_Frame_ID', 2, 'BaseLength', 430);
tagOut(sub, 'time', 1, 'Time', 140);
for i = 1:numel(signals)
    tagOut(sub, 'estimate_signals', i, signals{i}, 260);
end
put(sub, 'tf_enable', 'simulink/Logic and Bit Operations/Logical Operator', 340, 0, 40, 60, 'Operator', 'AND');
snap(sub, 'tf_enable', 'Inport', 2, portY(sub, 'frames', 'Outport', 3) + 160);
from(sub, 'pub', 'tf_enable', 1, 220);
wire(sub, 'frames/3', 'tf_enable/2', 'publish_tf');
tagOut(sub, 'tf_enable', 1, 'TfEnable', 430);

publishers = { ...
    'Odometry', {'pub', 'Time', 'WorldId', 'WorldLength', 'BaseId', 'BaseLength', 'x', 'pose_cov', 'twist_cov'}, @odometry; ...
    'Acceleration', {'pub', 'Time', 'BaseId', 'BaseLength', 'x', 'accel_cov'}, @acceleration; ...
    'TF', {'TfEnable', 'Time', 'WorldId', 'WorldLength', 'BaseId', 'BaseLength', 'x'}, @transform; ...
    'Diagnostics', {'diag_pub', 'diag'}, @(s) diagnostics(s, nDiag)};
y = 60;
for i = 1:size(publishers, 1)
    n = numel(publishers{i, 2});
    put(sub, publishers{i, 1}, 'built-in/SubSystem', 760, y + 20 * n, 180, 40 * n);
    publishers{i, 3}([sub '/' publishers{i, 1}]);
    tagIn(sub, publishers{i, 1}, publishers{i, 2}, 640);
    y = y + 40 * n + 60;
end
area(sub, 'Frames and estimate', [100 20 520 y], 'cream');
area(sub, 'Publishers', [600 20 1000 y], 'lavender');
end

function frameId(sub)
% Zero-padded frame name parameter -> message frame_id and its length.
put(sub, 'name', 'simulink/Sources/In1', 40, 100, 30, 14);
put(sub, 'padding', 'simulink/Sources/Constant', 120, 140, 110, 24, 'Value', 'zeros(64, 1, ''uint8'')');
add_block('simulink/Signal Routing/Vector Concatenate', [sub '/padded'], 'NumInputs', '2', 'Mode', 'Vector', ...
    'Position', [280 85 285 155], 'ShowName', 'off');
snap(sub, 'name', 'Outport', 1, portY(sub, 'padded', 'Inport', 1));
snap(sub, 'padding', 'Outport', 1, portY(sub, 'padded', 'Inport', 2));
put(sub, 'is_char', 'simulink/Logic and Bit Operations/Compare To Zero', 160, 220, 60, 30, 'relop', '~=');
put(sub, 'count', 'simulink/Signal Attributes/Data Type Conversion', 260, 220, 60, 24, 'OutDataTypeStr', 'uint32');
put(sub, 'chars', 'simulink/Math Operations/Sum', 360, 220, 40, 30, 'Inputs', '+', 'IconShape', 'rectangular', ...
    'OutDataTypeStr', 'uint32');
put(sub, 'frame_id', 'simulink/Sinks/Out1', 440, portY(sub, 'padded', 'Outport', 1), 30, 14);
put(sub, 'length', 'simulink/Sinks/Out1', 440, 220, 30, 14);
wire(sub, 'name/1', 'padded/1');
wire(sub, 'padding/1', 'padded/2');
wire(sub, 'padded/1', 'frame_id/1', 'frame_id');
wire(sub, 'name/1', 'is_char/1');
wire(sub, 'is_char/1', 'count/1');
wire(sub, 'count/1', 'chars/1');
wire(sub, 'chars/1', 'length/1', 'length');
end

function odometry(sub)
assignments = {'header.stamp', 'header.frame_id', 'header.frame_id_SL_Info.CurrentLength', ...
    'child_frame_id', 'child_frame_id_SL_Info.CurrentLength', ...
    'pose.pose.position.x', 'pose.pose.position.y', 'pose.pose.position.z', ...
    'pose.pose.orientation.w', 'pose.pose.orientation.x', 'pose.pose.orientation.y', 'pose.pose.orientation.z', ...
    'twist.twist.linear.x', 'twist.twist.linear.y', 'twist.twist.linear.z', ...
    'twist.twist.angular.x', 'twist.twist.angular.y', 'twist.twist.angular.z', ...
    'pose.covariance', 'twist.covariance'};
inputs = {'time', 'world_id', 'world_length', 'base_id', 'base_length', {'x', 13, 1}, 'pose_cov', 'twist_cov'};
message(sub, 'nav_msgs/Odometry', 'odometry/filtered', assignments, inputs);
end

function acceleration(sub)
assignments = {'header.stamp', 'header.frame_id', 'header.frame_id_SL_Info.CurrentLength', ...
    'accel.accel.linear.x', 'accel.accel.linear.y', 'accel.accel.linear.z', 'accel.covariance'};
inputs = {'time', 'base_id', 'base_length', {'x', 3, 14}, 'accel_cov'};
message(sub, 'geometry_msgs/AccelWithCovarianceStamped', 'accel/filtered', assignments, inputs);
end

function transform(sub)
% world_frame -> base_link_frame as the first of the message's transforms.
assignments = {'header.stamp', 'header.frame_id', 'header.frame_id_SL_Info.CurrentLength', ...
    'child_frame_id', 'child_frame_id_SL_Info.CurrentLength', ...
    'transform.translation.x', 'transform.translation.y', 'transform.translation.z', ...
    'transform.rotation.w', 'transform.rotation.x', 'transform.rotation.y', 'transform.rotation.z'};
inputs = {'time', 'world_id', 'world_length', 'base_id', 'base_length', {'x', 7, 1}};
last = message(sub, 'geometry_msgs/TransformStamped', '', assignments, inputs);

x = last + 60;
y = portY(sub, 'fill', 'Outport', 1);
put(sub, 'first', 'simulink/Math Operations/Assignment', x + 240, y, 60, 60, 'NumberOfDimensions', '1', ...
    'IndexOptions', 'Index vector (dialog)', 'Indices', '1');
snap(sub, 'first', 'Inport', 2, y);
put(sub, 'tf_message', 'simulink/Signal Routing/Bus Assignment', x + 380, 0, 10, 150, ...
    'AssignedSignals', 'transforms,transforms_SL_Info.CurrentLength', 'ShowName', 'off');
snap(sub, 'tf_message', 'Inport', 2, portY(sub, 'first', 'Outport', 1));
put(sub, 'transforms', 'simulink/Signal Routing/Bus Selector', x + 170, portY(sub, 'first', 'Inport', 1), 5, 30, ...
    'OutputSignals', 'transforms', 'ShowName', 'off');
put(sub, 'tf_blank', 'ros2lib/Blank Message', x, portY(sub, 'tf_message', 'Inport', 1), 130, 40, 'SampleTime', slowStep());
set_param([sub '/tf_blank'], 'messageType', 'tf2_msgs/TFMessage', 'entityType', 'tf2_msgs/TFMessage');
put(sub, 'one', 'simulink/Sources/Constant', x + 140, portY(sub, 'tf_message', 'Inport', 3), 80, 24, ...
    'Value', 'uint32(1)', 'OutDataTypeStr', 'uint32');
wire(sub, 'tf_blank/1', 'tf_message/1');
wire(sub, 'tf_blank/1', 'transforms/1');
wire(sub, 'transforms/1', 'first/1');
wire(sub, 'fill/1', 'first/2', 'transform');
wire(sub, 'first/1', 'tf_message/2', 'transforms');
wire(sub, 'one/1', 'tf_message/3', 'count');
publishGate(sub, 'tf_message', 'tf2_msgs/TFMessage', '/tf', x + 460);
end

function diagnostics(sub, nDiag)
% The diagnostics vector over Float64MultiArray messages of up to 128 values, each with its offset.
put(sub, 'enable', 'simulink/Sources/In1', 40, 40, 30, 14);
put(sub, 'diag', 'simulink/Sources/In1', 40, 200, 30, 14);
for k = 0:1
    first = 100 * k;
    n = max(0, min(128, nDiag - first));
    tag = sprintf('part_%d', k);
    y = 120 + 340 * k;
    put(sub, [tag '_blank'], 'ros2lib/Blank Message', 160, y - 60, 140, 40, 'SampleTime', slowStep());
    set_param([sub '/' tag '_blank'], 'messageType', 'std_msgs/Float64MultiArray', ...
        'entityType', 'std_msgs/Float64MultiArray');
    put(sub, [tag '_length'], 'simulink/Sources/Constant', 300, y + 90, 90, 24, 'Value', sprintf('uint32(%d)', n), ...
        'OutDataTypeStr', 'uint32');
    put(sub, [tag '_offset'], 'simulink/Sources/Constant', 300, y + 130, 90, 24, 'Value', sprintf('uint32(%d)', first), ...
        'OutDataTypeStr', 'uint32');
    put(sub, [tag '_fill'], 'simulink/Signal Routing/Bus Assignment', 460, y + 60, 10, 240, ...
        'AssignedSignals', 'data,data_SL_Info.CurrentLength,layout.data_offset', 'ShowName', 'off');
    snap(sub, [tag '_blank'], 'Outport', 1, portY(sub, [tag '_fill'], 'Inport', 1));
    snap(sub, [tag '_length'], 'Outport', 1, portY(sub, [tag '_fill'], 'Inport', 3));
    snap(sub, [tag '_offset'], 'Outport', 1, portY(sub, [tag '_fill'], 'Inport', 4));
    wire(sub, [tag '_blank/1'], [tag '_fill/1']);
    wire(sub, [tag '_length/1'], [tag '_fill/3'], 'length');
    wire(sub, [tag '_offset/1'], [tag '_fill/4'], 'offset');

    % data: this part's values, zero-padded to the message's 128
    yd = portY(sub, [tag '_fill'], 'Inport', 2);
    parts = {};
    if n > 0
        put(sub, [tag '_values'], 'simulink/Signal Routing/Selector', 160, yd, 90, 24, 'NumberOfDimensions', '1', ...
            'IndexOptions', 'Starting index (dialog)', 'Indices', num2str(first + 1), ...
            'OutputSizes', num2str(n), 'InputPortWidth', num2str(nDiag));
        from(sub, 'Diag', [tag '_values'], 1, 50);
        parts{end + 1} = [tag '_values']; %#ok<AGROW>
    end
    if n < 128
        put(sub, [tag '_padding'], 'simulink/Sources/Constant', 160, yd + 40, 90, 24, ...
            'Value', sprintf('zeros(%d, 1)', 128 - n));
        parts{end + 1} = [tag '_padding']; %#ok<AGROW>
    end
    if numel(parts) == 1
        moveBlock([sub '/' parts{1}], [0, yd - portY(sub, parts{1}, 'Outport', 1)]);
        wire(sub, [parts{1} '/1'], [tag '_fill/2'], 'data');
    else
        add_block('simulink/Signal Routing/Vector Concatenate', [sub '/' tag '_data'], 'NumInputs', '2', ...
            'Mode', 'Vector', 'Position', [320 yd - 30 325 yd + 30], 'ShowName', 'off');
        snap(sub, parts{1}, 'Outport', 1, portY(sub, [tag '_data'], 'Inport', 1));
        snap(sub, parts{2}, 'Outport', 1, portY(sub, [tag '_data'], 'Inport', 2));
        wire(sub, [parts{1} '/1'], [tag '_data/1']);
        wire(sub, [parts{2} '/1'], [tag '_data/2']);
        wire(sub, [tag '_data/1'], [tag '_fill/2'], 'data');
    end
    publishGate(sub, [tag '_fill'], 'std_msgs/Float64MultiArray', 'navigation/diagnostics', 540);
    set_param([sub '/' tag '_fill_publish'], 'Name', [tag '_publish']);
end
tagOut(sub, 'diag', 1, 'Diag', 90);
end

function last = message(sub, type, topic, assignments, inputs)
% Blank message -> Bus Assignment of every field, vectors split element by element.
% Returns the right edge of the fill block.
n = numel(assignments) + 1;
P = 30;
top = 60;
x0 = 360;
add_block('simulink/Signal Routing/Bus Assignment', [sub '/fill'], 'AssignedSignals', strjoin(assignments, ','), ...
    'Position', [x0 top x0 + 10 top + n * P], 'ShowName', 'off');
put(sub, 'blank', 'ros2lib/Blank Message', 160, portY(sub, 'fill', 'Inport', 1), 140, 40, 'SampleTime', slowStep());
set_param([sub '/blank'], 'messageType', type, 'entityType', type);
wire(sub, 'blank/1', 'fill/1');
put(sub, 'enable', 'simulink/Sources/In1', 40, top - 30, 30, 14);
port = 2;
for i = 1:numel(inputs)
    if iscell(inputs{i})
        [name, k, first] = inputs{i}{:};
        ys = portY(sub, 'fill', 'Inport', port);
        ye = portY(sub, 'fill', 'Inport', port + k - 1);
        add_block('simulink/Signal Routing/Demux', [sub '/' name '_elements'], 'Outputs', num2str(k), ...
            'Position', [x0 - 60 ys - P / 2 x0 - 55 ye + P / 2], 'ShowName', 'off');
        put(sub, [name '_fields'], 'simulink/Signal Routing/Selector', x0 - 180, (ys + ye) / 2, 90, 24, ...
            'NumberOfDimensions', '1', 'IndexOptions', 'Index vector (dialog)', ...
            'Indices', sprintf('%d:%d', first, first + k - 1), 'InputPortWidth', '19');
        put(sub, name, 'simulink/Sources/In1', 40, (ys + ye) / 2, 30, 14);
        wire(sub, [name '/1'], [name '_fields/1'], name);
        wire(sub, [name '_fields/1'], [name '_elements/1']);
        for j = 1:k
            wire(sub, sprintf('%s_elements/%d', name, j), sprintf('fill/%d', port + j - 1), field(assignments{port + j - 2}));
        end
        port = port + k;
    else
        put(sub, inputs{i}, 'simulink/Sources/In1', 40, portY(sub, 'fill', 'Inport', port), 30, 14);
        wire(sub, [inputs{i} '/1'], sprintf('fill/%d', port), field(assignments{port - 1}));
        port = port + 1;
    end
end
last = x0 + 10;
if ~isempty(topic)
    publishGate(sub, 'fill', type, topic, x0 + 80);
end
end

function s = field(path)
% Last two parts of a message field path, as a line label on its Bus Assignment port.
parts = strsplit(path, '.');
s = strjoin(parts(max(1, end - 1):end), '.');
end

function publishGate(sub, source, type, topic, x)
% Publish only on enabled steps.
gate = [sub '/' source '_publish'];
add_block('simulink/Ports & Subsystems/Enabled Subsystem', gate);
y = portY(sub, source, 'Outport', 1);
set_param(gate, 'Position', [x y - 30 x + 120 y + 30]);
delete_line(gate, 'In1/1', 'Out1/1');
delete_block([gate '/Out1']);
set_param([gate '/In1'], 'Name', 'msg', 'Position', [40 63 70 77]);
set_param([gate '/Enable'], 'Position', [175 10 195 30]);
add_block('ros2lib/Publish', [gate '/Publish'], 'Position', [140 50 260 90]);
set_param([gate '/Publish'], 'topic', topic, 'messageType', type);
snap(gate, 'msg', 'Outport', 1, portY(gate, 'Publish', 'Inport', 1));
wire(gate, 'msg/1', 'Publish/1');
wire(sub, [source '/1'], [source '_publish/1'], 'msg');
wire(sub, 'enable/1', [source '_publish/Enable']);
end

%% Layout helpers

function h = put(sys, name, src, x, yc, w, ht, varargin)
% Block with its left edge at x, centered on yc. An empty src moves an existing block.
pos = [x, round(yc - ht / 2), x + w, round(yc - ht / 2) + ht];
if isempty(src)
    h = get_param([sys '/' name], 'Handle');
    set_param(h, 'Position', pos);
else
    h = add_block(src, [sys '/' name], 'Position', pos, varargin{:});
end
end

function y = portY(sys, name, kind, i)
ph = get_param([sys '/' name], 'PortHandles');
p = get_param(ph.(kind)(i), 'Position');
y = p(2);
end

function snap(sys, name, kind, i, y)
% Move a block vertically so its port sits at y, for a straight line.
moveBlock([sys '/' name], [0, y - portY(sys, name, kind, i)]);
end

function moveBlock(block, d)
set_param(block, 'Position', get_param(block, 'Position') + [d d]);
end

function wire(sys, src, dst, label)
line = add_line(sys, src, dst, 'autorouting', 'smart');
% a bus creator takes its element names from its input lines
target = dst(1:find(dst == '/', 1, 'last') - 1);
named = strcmp(get_param([sys '/' target], 'BlockType'), 'BusCreator');
if nargin < 4 || (~named && strcmp(label, portLabel(sys, src)))
    return;
end
% Bus Selector outputs carry their field names already
source = src(1:find(src == '/', 1, 'last') - 1);
if ~strcmp(get_param([sys '/' source], 'BlockType'), 'BusSelector')
    set_param(line, 'Name', label);
end
end

function label = portLabel(sys, src)
% Name the source already shows: an inport's name, or a subsystem's output port name.
slash = find(src == '/', 1, 'last');
block = [sys '/' src(1:slash - 1)];
label = '';
switch get_param(block, 'BlockType')
    case 'Inport'
        label = get_param(block, 'Name');
    case 'SubSystem'
        out = find_system(block, 'SearchDepth', 1, 'BlockType', 'Outport', 'Port', src(slash + 1:end));
        if ~isempty(out)
            label = get_param(out{1}, 'Name');
        end
end
end

function tagOut(sys, name, port, tag, x)
% Goto at x level with an output port.
goto = uniqueName(sys, [tag '_goto']);
put(sys, goto, 'simulink/Signal Routing/Goto', x, portY(sys, name, 'Outport', port), 90, 24, ...
    'GotoTag', tag, 'TagVisibility', 'local', 'ShowName', 'off');
wire(sys, sprintf('%s/%d', name, port), [goto '/1']);
end

function from(sys, tag, name, port, x)
% From at x level with an input port; port may be 'Enable'.
ph = get_param([sys '/' name], 'PortHandles');
if ischar(port)
    p = get_param(ph.Enable, 'Position');
    blk = put(sys, uniqueName(sys, [tag '_from']), 'simulink/Signal Routing/From', p(1) - 45, p(2) - 50, 90, 24, ...
        'GotoTag', tag, 'ShowName', 'off');
    wire(sys, [get_param(blk, 'Name') '/1'], [name '/Enable']);
    return;
end
blk = put(sys, uniqueName(sys, [tag '_from']), 'simulink/Signal Routing/From', x, portY(sys, name, 'Inport', port), ...
    90, 24, 'GotoTag', tag, 'ShowName', 'off');
wire(sys, [get_param(blk, 'Name') '/1'], sprintf('%s/%d', name, port));
end

function tagIn(sys, name, tags, x)
for i = 1:numel(tags)
    from(sys, tags{i}, name, i, x);
end
end

function n = uniqueName(sys, base)
n = base;
k = 1;
while ~isempty(find_system(sys, 'SearchDepth', 1, 'Name', n))
    k = k + 1;
    n = sprintf('%s_%d', base, k);
end
end

function area(sys, title, rect, color)
% Colored area behind a stage, as in complete_controller.
colors = struct('lavender', '[0.901961, 0.901961, 1.000000]', 'cream', '[0.996078, 0.968627, 0.909804]', ...
    'blue', '[0.901961, 0.960784, 1.000000]');
note = add_block('built-in/Area', [sys '/' title ' area'], 'Position', rect);
set_param(note, 'Text', title, 'BackgroundColor', colors.(color), 'ForegroundColor', colors.(color), ...
    'DropShadow', 'off');
end

%% Shared helpers

function releaseRosEntities(mdl)
% R2023a keeps every publisher, subscriber and parameter getter in a global that is
% destroyed after main returns, once the middleware is gone, so the node crashes on exit.
% The model's terminate() runs before rclcpp::shutdown(), so free them there instead.
% Generated names are <prefix>_<model>_<SID>; a wrong name fails the compile.
kinds = {'ros2lib/Publish', 'Pub'; 'ros2lib/Subscribe', 'Sub'; 'ros2lib/Get Parameter', 'ParamGet'};
calls = {};
for k = 1:size(kinds, 1)
    blocks = find_system(mdl, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'ReferenceBlock', kinds{k, 1});
    for i = 1:numel(blocks)
        calls{end + 1} = sprintf('  talos_nav_release(%s_%s_%s);\n', kinds{k, 2}, mdl, get_param(blocks{i}, 'SID')); %#ok<AGROW>
    end
end
source = ['#include <chrono>' newline '#include <new>' newline newline ...
    'template<class T>' newline 'static void talos_nav_release(T & entity)' newline '{' newline ...
    '  entity.~T();' newline '  new (&entity) T();' newline '}' newline newline ...
    wakeTimer() ...
    'static void talos_nav_release_ros()' newline '{' newline ...
    '  talos_nav_wake_timer.reset();' newline '  talos_nav_wake_group.reset();' newline [calls{:}] '}' newline];
set_param(mdl, 'CustomSourceCode', source, 'CustomInitializer', 'talos_nav_start_wake_timer();', ...
    'CustomTerminator', 'talos_nav_release_ros();');
end

function source = wakeTimer
% Everything generated shares one mutually exclusive callback group, so while a callback runs the
% multithreaded executor waits on no timers at all, with no timeout, until a guard condition wakes
% it. Under rmw_zenoh that wake-up can get lost and the node, base rate included, then sleeps for
% good. An empty timer in a group of its own keeps a timeout in every wait.
source = ['static rclcpp::CallbackGroup::SharedPtr talos_nav_wake_group;' newline ...
    'static rclcpp::TimerBase::SharedPtr talos_nav_wake_timer;' newline newline ...
    'static void talos_nav_start_wake_timer()' newline '{' newline ...
    '  talos_nav_wake_group = SLROSNodePtr->create_callback_group(rclcpp::CallbackGroupType::Reentrant);' newline ...
    '  talos_nav_wake_timer = SLROSNodePtr->create_wall_timer(' newline ...
    '    std::chrono::milliseconds(10), []() {}, talos_nav_wake_group);' newline '}' newline newline];
end

function parkFunctionTask(mdl)
% With a Simulink Function in a multitasking model, the R2023a ROS 2 target takes it for one more
% rate: its node interface starts a thread that calls a step3() the model does not have, when
% TID[3] says so. The function really runs in the caller's task, so that rate must never fire
% and the thread just waits. A generated name changing here fails the compile.
set_param(mdl, 'CustomHeaderCode', ['#undef rtmStepTask' newline ...
    '#define rtmStepTask(rtm, idx) ((idx) < 3 && (rtm)->Timing.TaskCounters.TID[(idx)] == 0)' newline ...
    '#define step3 step2' newline]);
end

function chart = script(block, code)
chart = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', getfullname(block));
chart.Script = code;
% bounded variable sizes only, no dynamic allocation
chart.SupportVariableSizing = true;
end

function [type, count] = parameterType(entry)
% Get Parameter has no string or enum type: strings travel as uint8[], enums as int64.
count = 1;
kind = regexp(entry.type, '^\w+', 'match', 'once');
tokens = regexp(entry.type, '\[(\d+)\]', 'tokens', 'once');
if ~isempty(tokens), count = str2double(tokens{1}); end
switch kind
    case 'double', base = 'double';
    case {'int', 'enum'}, base = 'int64';
    case 'bool', base = 'boolean';
    case 'string', base = 'uint8'; count = 64;
end
if count > 1, base = [base '[]']; end
type = base;
end

function value = initialValue(entry)
value = entry.default;
if strcmp(entry.type, 'enum'), value = find(strcmp(entry.enum, entry.default)); end
value = double(value);
end
