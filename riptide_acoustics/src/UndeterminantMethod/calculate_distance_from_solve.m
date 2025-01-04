function [test_point, current_distance] = calculate_distance_from_solve(delta_d, characteristic_distance, transformed_point)
    %delta_d  - distance from hydrophone 2 to pinger - distance from
    %hydrophone 1 to pinger
    %characteristic distance - (+) distance between the two hydrophones
    %hydrophone 1 is assumed to be at (0,0,0)
    %transformed point - the point to be evalulated transformed into the
    %frame in which h1 is (0,0,0) and h2 is (0, cd, 0)

    convergence_rate = 0.001;

    %calculate function based on delta d and cd
    if(delta_d > 0)
        y_xz = @(x, z, d, s) (d^3 + sqrt(d^4*s^2 - 2*d^2*s^4 + 4*d^2*s^2*x^2 + 4*d^2*s^2*z^2 + s^6 - 4*s^4*x^2 - 4*s^4*z^2) - d*s^2)/(2*(d^2 - s^2));
    else
        y_xz = @(x, z, d, s) (d^3 - sqrt(d^4*s^2 - 2*d^2*s^4 + 4*d^2*s^2*x^2 + 4*d^2*s^2*z^2 + s^6 - 4*s^4*x^2 - 4*s^4*z^2) - d*s^2)/(2*(d^2 - s^2));
    end

    %dz_dx = @(x,y,d,s) (2*s*x) / sqrt(abs(d^4 - 2*d^2*s^2 + s^4 - 4*s^2*x^2 - 4*d^3*y + 4*d*s^2*y + 4*d^2*y^2 - 4*s^2*y^2)) * sign(d^4 - 2*d^2*s^2 + s^4 - 4*s^2*x^2 - 4*d^3*y + 4*d*s^2*y + 4*d^2*y^2 - 4*s^2*y^2); 
    %dz_dy = @(x,y,d,s) ((d^2 - s^2)*(d - 2*y))/(s*sqrt(abs(d^4 + s^4 - 4*d^3*y + 4*d*s^2*y - 2*d^2*(s^2 - 2*y^2) - 4*s^2*(x^2 + y^2)))) * sign(d^4 + s^4 - 4*d^3*y + 4*d*s^2*y - 2*d^2*(s^2 - 2*y^2) - 4*s^2*(x^2 + y^2));
    distance = @(x,y,z,point) norm(point - [x;y;z]);
    %ddist_dx = @(x,y,d,s,point) -((point(1) - x) + (point(3) - z_xy(x,y,d,s)) * dz_dx(x,y,d,s)) / norm(point - [x,y,z_xy(x,y,d,s)]);
    %ddist_dy = @(x,y,d,s,point) -((point(2) - y) + (point(3) - z_xy(x,y,d,s)) * dz_dy(x,y,d,s)) / norm(point - [x,y,z_xy(x,y,d,s)]);

    %guess where the cloest point is -- directly up
    test_point = [transformed_point(1), y_xz(transformed_point(1), transformed_point(3), characteristic_distance, delta_d), transformed_point(3)];
    
    %find the clostest point
    counter = 5;
    relative_error_threshold = 1e-8;
    previous_distance = -1;
    convergence_multiplier = .1;
    while counter > 0
       %calculate the gradient

        temp_test_1 = test_point(1) - convergence_multiplier * (distance(test_point(1) + convergence_rate, y_xz(convergence_rate + test_point(1), test_point(3), characteristic_distance, delta_d), test_point(3), transformed_point) - distance(test_point(1) - convergence_rate, y_xz(test_point(1) - convergence_rate, test_point(3), characteristic_distance, delta_d), test_point(3), transformed_point)) / (2);
        test_point(3) = test_point(3) - convergence_multiplier * (distance(test_point(1), y_xz(test_point(1), test_point(3) + convergence_rate, characteristic_distance, delta_d), test_point(3) + convergence_rate,  transformed_point) - distance(test_point(1), y_xz(test_point(1), test_point(3) - convergence_rate, characteristic_distance, delta_d), test_point(3) - convergence_rate, transformed_point)) / (2); 
        test_point(1) = temp_test_1;
            
        test_point(2) = y_xz(test_point(1), test_point(3), characteristic_distance, delta_d);

        current_distance = distance(test_point(1), y_xz(test_point(1), test_point(3), characteristic_distance, delta_d), test_point(3), transformed_point);

        if(previous_distance - relative_error_threshold < current_distance)
            counter = counter - 1;
        else
            counter = 5;
        end

        if(previous_distance == -1 || previous_distance > current_distance)
            previous_distance = current_distance;  
        else
            convergence_multiplier = convergence_multiplier / 2;
        end
    
    end

    %calculate the vector between the closest point and the guessed
    %vector = test_point - transformed_point;
    % 
    % gradient = [1 / ddist_dx(test_point(1), test_point(1), characteristic_distance, delta_d, transformed_point), 1 / ddist_dy(test_point(1), test_point(1), characteristic_distance, delta_d, transformed_point), 1]
    % 
    % ddist_dx(-1, 1, characteristic_distance, delta_d, transformed_point)
    % ddist_dy(-1, -1, characteristic_distance, delta_d, transformed_point)
    % 
    % plotting vals for surface

    %UNCOMMENT FOR DEBUG
    % num_x = 200;
    % num_z = 100;
    % x_vals = linspace(-abs(test_point(1)) - .5, abs(test_point(1)) + .5, num_x);
    % z_vals = linspace(-abs(test_point(2) - .5), abs(test_point(2)) + .5, 100);
    % x_mat = zeros(1, num_x * 100);
    % z_mat = zeros(1, num_x * 100);
    % y_mat = zeros(1, num_x * 100);
    % dist_mat = zeros(1, num_x*100);
    % 
    % for xi = 0:(num_x - 1)
    %     for zi = 1:num_z
    %         x_mat(1, xi * num_z + zi) = x_vals(xi + 1);
    %         z_mat(1, xi * num_z + zi) = z_vals(zi);
    % 
    %         y_mat(1, xi * num_z + zi) = y_xz(x_vals(xi + 1), z_vals(zi), characteristic_distance, delta_d);
    %         dist_mat(1, xi * num_z + zi) = norm(transformed_point - [x_mat(1, xi * num_z + zi); y_mat(1, xi * num_z + zi); z_mat(1, xi * num_z + zi);]);
    %     end
    % end
    % 
    % figure;
    % hold on
    % plot3([0,0], [0,characteristic_distance], [0,0], "*r")
    % plot3(x_mat, y_mat, z_mat, ".b")
    % plot3(test_point(1), test_point(2), test_point(3), "*g")
    % plot3(transformed_point(1), transformed_point(2), transformed_point(3), "*g")
    % title("Calculate Distance Visualization")
    % subtitle_string = "Delta d: " + num2str(delta_d);
    % subtitle(subtitle_string);
    % xlabel("X")
    % ylabel("Y")
    % zlabel("Z")
    % hold off
    % 
    % figure;
    % hold on
    % plot3(x_mat, z_mat, dist_mat, ".r")
    % plot3(test_point(1), test_point(3), distance(test_point(1), test_point(2), test_point(3), transformed_point), "*b")
    % title("Distance from pinger point")
    % subtitle_string = "Norm Distance: " + num2str(distance(test_point(1), test_point(2), test_point(3), transformed_point));
    % subtitle(subtitle_string);
    % xlabel("X")
    % ylabel("Z")
    % zlabel("Dist")
    % hold off

end
