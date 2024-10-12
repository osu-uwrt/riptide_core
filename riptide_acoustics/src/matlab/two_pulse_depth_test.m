%should yeild (52,116.92)
distances = [-20, 41.465];
pose1 = [0,0,0,1,0,0,0];
pose2 = [141.89, 22.89,0, .9915,0,0,-.1300];
pinger_depth = 0;


[slv_x,slv_y] = solve_two_pulse_system_depth(distances(1),distances(2),pose1, pose2, [50;0;0], 3000, pinger_depth)

%should yeild (52,116.92)
distances = [-10.893, 23.1377];
pose1 = [0,0,0,1,0,0,0];
pose2 = [141.89, 22.89,0, .9915,0,0,-.1300];
pinger_depth = -200;


[slv_x,slv_y] = solve_two_pulse_system_depth(distances(1),distances(2),pose1, pose2, [50;0;0], 100000, pinger_depth)


%should yeild (152,216.92)
distances = [-10.893, 23.1377];
pose1 = [100,100,0,1,0,0,0];
pose2 = [241.89, 122.89,0, .9915,0,0,-.1300];
pinger_depth = -200;


[slv_x,slv_y] = solve_two_pulse_system_depth(distances(1),distances(2),pose1, pose2, [50;0;0], 100000, pinger_depth)

%should yeild (2998.5,95.04)
% distances = [2,19];
% transform = [-95.04, 8.49, -.38207];
