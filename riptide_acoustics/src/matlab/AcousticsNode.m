clear;

%read from config
speed_of_sound = 1500;
pinger_depth = -1.0;

%initialize node
global node;
node = ros2node("/acoustics_node");

%intialize tf2
global tftree;
tftree = ros2tf(node);

%initialize transform array
global measurements;

%get acoustic pod transform
global port_to_origin_transform
global port_to_startboard_transform

transforms_recieved = false;
while(transforms_recieved == false)
    %loop until transforms become available

    try
        port_to_origin_transform = getTransform(tftree, "talos/origin", "talos/acoustics_port_link");
        port_to_startboard_transform = getTransform(tftree, "talos/acoustics_starboard_link", "talos/acoustics_port_link");

        transforms_recieved = true;
    catch
        pause(1);
        disp("Waiting for acoustic transforms to become available!");
    end
end


%initialize publishers and subscribers
pub = ros2publisher(node, "/test_matlab", "std_msgs/UInt8");

delta_t_sub = ros2subscriber(node, "/talos/acoustics/delta_t", @delta_t_callback);

x = 0;
while(x < 1)

    msg = ros2message("std_msgs/UInt8");
    msg.data = uint8(100);

    send(pub, msg);
    pause(5)

    if(length(measurements) > 2)
        %select the two measurements to calculate with
        m1 = measurements(end - 1);
        m2 = measurements(end);

        %formatting...
        p2o_translation = [port_to_origin_transform.transform.translation.x; port_to_origin_transform.transform.translation.y; port_to_origin_transform.transform.translation.z];
        p2o_quat = [port_to_origin_transform.transform.rotation.w; port_to_origin_transform.transform.rotation.x; port_to_origin_transform.transform.rotation.y; port_to_origin_transform.transform.rotation.z];
        p2o_rotation = quat2rotm(p2o_quat');

        o2m1_translation = [m1.auv_origin.transform.translation.x; m1.auv_origin.transform.translation.y; m1.auv_origin.transform.translation.z];
        o2m1_quat = [m1.auv_origin.transform.rotation.w; m1.auv_origin.transform.rotation.x; m1.auv_origin.transform.rotation.y; m1.auv_origin.transform.rotation.z];
        o2m1_rotation = quat2rotm(o2m1_quat');

        o2m2_translation = [m2.auv_origin.transform.translation.x; m2.auv_origin.transform.translation.y; m2.auv_origin.transform.translation.z];
        o2m2_quat = [m2.auv_origin.transform.rotation.w; m2.auv_origin.transform.rotation.x; m2.auv_origin.transform.rotation.y; m2.auv_origin.transform.rotation.z];
        o2m2_rotation = quat2rotm(o2m1_quat');

        p2s_translation = [port_to_startboard_transform.transform.translation.x; port_to_startboard_transform.transform.translation.y; port_to_startboard_transform.transform.translation.z];
            
        %transform each hydrophone into origin frame for each measurement
        m1t = o2m1_rotation * p2o_translation + o2m1_translation;
        m1r = transpose(quatmultiply(p2o_quat', o2m1_quat'));
        m2t = o2m2_rotation * p2o_translation + o2m2_translation;
        m2r = transpose(quatmultiply(p2o_quat', o2m2_quat'));

        [solve_x, solve_y] = solve_two_pulse_system_depth(m1.delta_t/speed_of_sound, m2.delta_t/speed_of_sound,[m1t;m1r],[m2t;m2r], p2s_translation, 3000, pinger_depth);
        
        pinger_location = o2m1_rotation * ([solve_x; solve_y; pinger_depth] + p2s_translation) + o2m1_translation
    end

end


function delta_t_callback(message)
    %introduce global variables
    global tftree;
    global measurements;

    try
        %get the transform at the time of the measurement
        port_transform = getTransform(tftree, "map", 'talos/origin');
    
        %fill out measurement data struct
        measurement = struct();
        measurement.delta_t = message.vector.x;
        measurement.frequency = message.vector.y;
        measurement.stamp = message.header.stamp;
        measurement.auv_origin = port_transform;

        if(length(measurements) == 0)
            measurements = [measurement]

        else
            measurements(end+1) = measurement;
        end

    catch
        disp("Failed to get transform")

        tftree.AvailableFrames
    end

end 