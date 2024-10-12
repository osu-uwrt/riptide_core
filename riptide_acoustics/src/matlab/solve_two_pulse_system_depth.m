function [intersection_x, intersection_y] = solve_two_pulse_system_depth(k1, k2, pose_1, pose_2, hydrophone_base_vector, estimated_distance, pinger_depth)
%k1: hydrophone 1 pulse time - hydrophone 2 pulse time for first pulse
%k2: hydrophone 1 pulse time - hydrophone 2 pulse time for second pulse
%pose_1: the pose of the front hydrophone at the time of the first pulse
%pose_2: the pose of the front hydrophone at the time of the second pulse
%hydrophone_base_vector: the position vector between hydrophone 1 and 2
%estimated disance: the estimated distance from the front hydrophone and
%pinger
%pinger_depth: the estimated depth of the pinger

%pose format = [x,y,z,rw,rx,ry,rz]

%base frame is hydrophone 1 pulse 1 = [0,0,0,0,0,0]
x1_1 = 0;
y1_1 = 0;
z1_1 = 0;

pinger_depth = pinger_depth - pose_1(3);

%calculate position of second hydrophone
%order [Z,Y,X]
eul_1 = quat2eul([pose_1(4), pose_1(5), pose_1(6), pose_1(7)]);
norm_matrix = eul2rotm([-eul_1(1), 0, 0]);
rot_1 = norm_matrix * quat2rotm([pose_1(4), pose_1(5), pose_1(6), pose_1(7)]);
pos2_1 = rot_1 * hydrophone_base_vector;

x1_2 = pose_2(1) - pose_1(1);
y1_2 = pose_2(2) - pose_1(2);
z1_2 = pose_2(3) - pose_1(3);

%calculate position of second hydrophone in second pulse
rot_2 = norm_matrix * quat2rotm([pose_2(4), pose_2(5), pose_2(6), pose_2(7)]);
pos2_2 = rot_2 * hydrophone_base_vector + [x1_2;y1_2;z1_2];

%base widths in x,y plane
h1 = pos2_1(1);
h2 = norm([pos2_2(1) - x1_2 , pos2_2(2) - y1_2]);

%calculate delta positions
x1 = x1_2 - x1_1;
x2 = pos2_2(1) - x1_1;

y1 = y1_2 - y1_1;
y2 = pos2_2(2) - pos2_1(2);

%changes in depth
dz1_1 = pinger_depth;
dz2_1 = pos2_1(3) + pinger_depth;
dz1_2 = pose_2(3) + pinger_depth;
dz2_2 = pos2_2(3) + pinger_depth;

%calculate heading change
eul_2 = rotm2eul(rot_2);
heading_change = eul_2(1);

%define core functions
K = @(a,k_bar,z1,z2) (-(2*a) + sqrt(4*a^2 + 4 * (k_bar^2 + 2 * k_bar * sqrt(a^2 + z1^2) + z1^2 - z2^2))) / 2;
Y = @(a,k,h) sqrt(a^2 + 2*a*k + k^2 - ((h^2 + 2*a*k +k^2)/(2*h))^2);
X = @(a,y,k,h) sqrt(a^2 - y^2) * -sign(k) - h / 2;

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
%     F = @(x) [cos(-heading_change)*(Y(x(1), k1, h1) - (y1 + y2) / 2) + (sin(-heading_change) * (X(x(1), Y(x(1), k1, h1), k1, h1) - (x1 + x2 - hydrophone_base_vector(1)) / 2)) - Y(x(2),k2, h2);
%               (X(x(1), Y(x(1), k1, h1), k1, h1) - (x1 + x2 - hydrophone_base_vector(1)) / 2) * cos(-heading_change) - (Y(x(1), k1, h1) - (y1 + y2) / 2) * sin(-heading_change) - (X(x(2),Y(x(2),k2,h2), k2, h2))];
    F = @(x) [cos(-heading_change)*(Y(x(1), K(x(1), k1, dz1_1, dz2_1), h1) - (y1 + y2) / 2) + (sin(-heading_change) * (X(x(1), Y(x(1), K(x(1), k1, dz1_1, dz2_1), h1), K(x(1), k1, dz1_1, dz2_1), h1) - (x1 + x2 - hydrophone_base_vector(1)) / 2)) - Y(x(2),K(x(2), k2, dz1_2, dz2_2), h2);
              (X(x(1), Y(x(1), K(x(1), k1, dz1_1, dz2_1), h1), K(x(1), k1, dz1_1, dz2_1), h1) - (x1 + x2 - hydrophone_base_vector(1)) / 2) * cos(-heading_change) - (Y(x(1), K(x(1), k1, dz1_1, dz2_1), h1) - (y1 + y2) / 2) * sin(-heading_change) - (X(x(2),Y(x(2),K(x(2), k2, dz1_2, dz2_2),h2), K(x(2), k2, dz1_2, dz2_2), h2))];

    %options = optimoptions('fsolve','Display','iter', "OptimalityTolerance", 10e-12);
    options = optimoptions("fsolve", 'Display', 'off', "OptimalityTolerance", 10e-6);

    %solve non-linear system
    solve1 = fsolve(F, x0, options);

    norm1 = norm(F(solve1));
    answer = solve1;

end

 
%calculate position

intersection_x = X(answer(1),Y(answer(1), K(answer(1), k1, dz1_1, dz2_1), h1), K(answer(1), k1, dz1_1, dz2_1), h1) + pose_1(1);
intersection_y = Y(answer(1), K(answer(1), k1, dz1_1, dz2_1), h1) + pose_1(2);

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


