clear;

%initialize node
global node
node = ros2node("/acoustics_node");

%initialize publishers and subscribers
pub = ros2publisher(node, "/test_matlab", "std_msgs/UInt8");

delta_t_sub = ros2subscriber(node, "/talos/acoustics/delta_t", @delta_t_callback);

%intialize tf2
tftree = ros2tf(node);





x = 0;
while(x < 1)

    msg = ros2message("std_msgs/UInt8");
    msg.data = uint8(100);

    send(pub, msg);
    pause(5)

    tf_update = tftree.LastUpdateTime;
    disp(tf_update)

end


function delta_t_callback(message)
    disp(message.data)
end