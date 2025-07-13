function [delta_t] = GetTimingDifference(pose_1, hydrophone_base_vector, pinger_pos, speed_of_sound)

%rotation matrix
rotm = quat2rotm([pose_1(4), pose_1(5), pose_1(6), pose_1(7)]);
hydrophone2_pos = pose_1(1:3) + rotm * hydrophone_base_vector;

time_1 = norm(pose_1(1:3) - pinger_pos) / speed_of_sound;
time_2 = norm(hydrophone2_pos - pinger_pos) / speed_of_sound;

delta_t = time_2 - time_1;

end