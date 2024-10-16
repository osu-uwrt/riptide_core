function [intersection_x, intersection_y] = solve_two_pulse_system(k1, k2, x1, y1, heading_change, hydrophone_base_width, estimated_distance)
%k1: hydrophone 1 pulse time - hydrophone 2 pulse time for first pulse
%k2: hydrophone 1 pulse time - hydrophone 2 pulse time for second pulse
%x1: x distance traveled between pulses in the coordinate frame of first
%pulse
%y1: y distance traveled between pulses in the coordinate frame of second
%pulse
%heading change: the change in heading of vehicle between the two pulses:
%ccw is positive
%solves: the number of successful solves to use in estimatation process
%hydrophone_base_width: distance between the two hydrophones

%transform location of other hydrophone
x2 = x1 + hydrophone_base_width * (cos(heading_change));
y2 = y1 + hydrophone_base_width * sin(heading_change);

%define core functions
Y = @(a,k) sqrt(a^2 + 2*a*k + k^2 - ((hydrophone_base_width^2 + 2*a*k +k^2)/(2*hydrophone_base_width))^2);
X = @(a,y,k) sqrt(a^2 - y^2) * -sign(k) - hydrophone_base_width / 2;


norm1 = 5;
answer = [0;0];
while(norm1 > .001)

    x0 = [rand() * estimated_distance;rand() * estimated_distance];% [estimated_distance; estimated_distance; estimated_distance; estimated_distance; k1 / 2 ; k2 / 2] * (.2 - rand()*.2);

    %equations of localization
%         F = @(x) [sqrt(abs((x(1)+x(3))^2-x(5)^2))*sign((x(1)+x(3))^2-x(5)^2) + sqrt(abs((x(2)+x(4))^2-x(5)^2))*sign((x(1)+x(3))^2-x(5)^2) - sqrt(x1^2+y1^2);
%                 sqrt(abs((x(1)+x(3)+k1)^2-x(6)^2))*sign((x(1)+x(3))^2-x(5)^2) + sqrt(abs((x(2)+x(4)+k2)^2-x(6)^2))*sign((x(1)+x(3))^2-x(5)^2) - sqrt(x2^2+y2^2);
%                 (x(2)+x(4))^2 - (((x(1)+x(3))^2-(x(2)+x(4))^2-(x1^2+y1^2))/(-2*sqrt(x1^2+y1^2)))^2 - x(5)^2;
%                 (x(2)+x(4)+k2)^2 - (((x(1)+x(3)+k1)^2-(x(2)+x(4)+k2)^2-(x2^2+y2^2))/(-2*sqrt(x2^2+y2^2)))^2 - x(6)^2;
%                 .5*hydrophone_base_width^2 - k1^2 - 2*(k1+x(3))*x(3) - 2*k1*x(1) - 4*x(1)*x(3);
%                 .5*hydrophone_base_width^2 - k2^2 - 2*(k2+x(4))*x(4) - 2*k2*x(2) - 4*x(2)*x(4)];


    

    %cramers rule & determinant of rot matrix is one
    F = @(x) [cos(-heading_change)*(Y(x(1), k1) - (y1 + y2) / 2) + (sin(-heading_change) * (X(x(1), Y(x(1), k1), k1) - (x1 + x2 - hydrophone_base_width) / 2)) - Y(x(2),k2);
              (X(x(1), Y(x(1), k1), k1) - (x1 + x2 - hydrophone_base_width) / 2) * cos(-heading_change) - (Y(x(1), k1) - (y1 + y2) / 2) * sin(-heading_change) - (X(x(2),Y(x(2),k2), k2))];
    
    %options = optimoptions('fsolve','Display','iter', "OptimalityTolerance", 10e-12);
    options = optimoptions('fsolve', 'Display', 'off', "OptimalityTolerance", 10e-6);

    solve1 = fsolve(F, x0, options);
 
    norm1 = norm(F(solve1));
    answer = solve1;
end

%calculate position

intersection_x = X(answer(1),Y(answer(1), k1),k1);
intersection_y = Y(answer(1), k1);

%for quadratic fit
% intersection_x = (-(b2(2) - b1(2)) - sqrt(abs((b2(2) - b1(2))^2 - 4 * (b2(1) - b1(1))*(b2(3) - b1(3))))) / (2 * (b2(3) - b1(3)));
% intersection_y = b1(1) + b1(2)* intersection_x + b1(3) * intersection_x^2;


%plots for debug
% figure(1);
% plot(fx1(2,:),fy1, "or", rot_points(1,:), rot_points(2,:),"xg")

% figure(2)
% w_vals = linspace(-1000,1000, 1000);
% h1_vals = b1(1) + b1(2) * w_vals + b1(3)*w_vals.^2;
% h2_vals = b2(2) * w_vals + b2(1) + b2(3) * w_vals.^2;%b2_intercept ;
% 
% plot(w_vals, h1_vals,"r", w_vals, h2_vals, "g", [-1000], [1000], "bo")


