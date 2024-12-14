function pinger_location = parse_measurements(m1, m2, port_to_origin_transform, port_to_startboard_transform, speed_of_sound, pinger_depth)        

        p2o_translation = [port_to_origin_transform.transform.translation.x; port_to_origin_transform.transform.translation.y; port_to_origin_transform.transform.translation.z];
        p2o_quat = [port_to_origin_transform.transform.rotation.w; port_to_origin_transform.transform.rotation.x; port_to_origin_transform.transform.rotation.y; port_to_origin_transform.transform.rotation.z];
        p2o_rotation = quat2rotm(p2o_quat');

        %origin to world frame
        o2m1_translation = [m1.auv_origin.transform.translation.x; m1.auv_origin.transform.translation.y; m1.auv_origin.transform.translation.z];
        o2m1_quat = [m1.auv_origin.transform.rotation.w; m1.auv_origin.transform.rotation.x; m1.auv_origin.transform.rotation.y; m1.auv_origin.transform.rotation.z];
        o2m1_rotation = quat2rotm(o2m1_quat');


        o2m2_translation = [m2.auv_origin.transform.translation.x; m2.auv_origin.transform.translation.y; m2.auv_origin.transform.translation.z];
        o2m2_quat = [m2.auv_origin.transform.rotation.w; m2.auv_origin.transform.rotation.x; m2.auv_origin.transform.rotation.y; m2.auv_origin.transform.rotation.z];
        o2m2_rotation = quat2rotm(o2m1_quat');

        p2s_translation = [port_to_startboard_transform.transform.translation.x; port_to_startboard_transform.transform.translation.y; port_to_startboard_transform.transform.translation.z];

        %transform each hydrophone into world frame for each measurement
        m1t = o2m1_rotation * p2o_translation + o2m1_translation;
        m1r = transpose(quatmultiply(p2o_quat', o2m1_quat'));
        m2t = o2m2_rotation * p2o_translation + o2m2_translation;
        m2r = transpose(quatmultiply(p2o_quat', o2m2_quat'));

        %in world frame
        [solve_x, solve_y, solve_depth] = solve_two_pulse_system_depth(m1.delta_t*speed_of_sound, m2.delta_t*speed_of_sound,[m1t;m1r],[m2t;m2r], p2s_translation, 3000, pinger_depth);
        
        pinger_location = ([solve_x; solve_y; pinger_depth]);

end