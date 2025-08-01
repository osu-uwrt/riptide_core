%run the node to use its initial parameters

%load in csv
filename = "data/test3.csv";

data = table2array(readtable(filename, "NumHeaderLines", 1));

%load into ros2 messages
transform = ros2message("geometry_msgs/TransformStamped");

%pinger pose hardcoded
pinger_pose = [3.0,2.0,1.0,0.0,0.0,0.0];

line_1 = 7;
m1 = struct();
m1.delta_t = data(line_1, 3);

transform.transform.translation.x = data(line_1, 4);
transform.transform.translation.y = data(line_1, 5);
transform.transform.translation.z = data(line_1, 6);
transform.transform.rotation.w = data(line_1,7);
transform.transform.rotation.x = data(line_1,8);
transform.transform.rotation.y = data(line_1,9);
transform.transform.rotation.z = data(line_1,10);

m1.auv_origin = transform;

line_2 = 16;
m2 = struct();
m2.delta_t = data(line_2, 3);

transform.transform.translation.x = data(line_2, 4);
transform.transform.translation.y = data(line_2, 5);
transform.transform.translation.z = data(line_2, 6);
transform.transform.rotation.w = data(line_2,7);
transform.transform.rotation.x = data(line_2,8);
transform.transform.rotation.y = data(line_2,9);
transform.transform.rotation.z = data(line_2,10);

m2.auv_origin = transform;

parse_measurements(m1,m2,port_to_origin_transform, port_to_startboard_transform, speed_of_sound, pinger_depth)
