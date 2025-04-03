%Script to Calculate the resolution of 2 hydrophone acoustics system
clear;

%All units in meters
hydrophone_base_width = 50;%0.05372;
water_speed = 1500;
adc_sampling_frequency = 5*10^5;

%define transform between two pulses
x1 = -95.04;
y1 = 8.49;
heading_change = -21.8912/360 *2*pi(); %radians

x2 = x1 - hydrophone_base_width * (1 - cos(heading_change));
y2 = y1 + hydrophone_base_width * sin(heading_change);


%data gathered from acoustics
k1 = 2;
k2 = 19;

% %equations to solve acoustics systems
% syms a1 a2 c1 c2 B1 B2
% 
% %equations of localization
% eq1 = sqrt(x1^2+y1^2) == sqrt((a1+c1)^2-B1^2) + sqrt((a2+c2^2)-B1^2);
% eq2 = sqrt(x2^2+y2^2) == sqrt((a1+c1+k1)^2-B2^2) + sqrt((a2+c2+k2)^2-B2^2);
% eq3 = B1^2 == (a2+c2)^2 -(((a1+c1)^2-(a2+c2)^2-(x1^2+y1^2))/(-2*sqrt(x1^2+y1^2)))^2;
% eq4 = B2^2 == (a2+c2+k2)^2 -(((a1+c1+k1)^2-(a2+c2+k2)^2-(x2^2+y2^2))/(-2*sqrt(x2^2+y2^2)))^2;
% eq5 = 2*(k1+1)*c1 + 2*k1*a1 + 4*a1*c1 == .5*hydrophone_base_width + k1^2;
% eq6 = 2*(k2+1)*c2 + 2*k2*a2 + 4*a2*c2 == .5*hydrophone_base_width + k2^2;
% 
% solve(eq1,eq2,eq3,eq4,eq5,eq6,a1,a2,c1,c2,B1,B2)

%equations to solve acoustics systems
%x0 = [64.7495,44,2.5505,12.30,46.2,41.04]';

answer_count = 20;
answer = zeros(6, answer_count);
counter = 0;

while(counter < answer_count)
    
    norm1 = 5;
    while(norm1 > .001)

        x0 = rand([6,1])*5000;

        %equations of localization
%         F = @(x) [-sqrt(abs((x(1)+x(3))^2-x(5)^2)) + sqrt(abs((x(2)+x(4))^2-x(5)^2)) - sqrt(x1^2+y1^2);
%                 sqrt(abs((x(1)+x(3)+k1)^2-x(6)^2)) - sqrt(abs((x(2)+x(4)+k2)^2-x(6)^2))-sqrt(x2^2+y2^2);
%                 (x(2)+x(4))^2 - (((x(1)+x(3))^2-(x(2)+x(4))^2-(x1^2+y1^2))/(-2*sqrt(x1^2+y1^2)))^2 - x(5)^2;
%                 (x(2)+x(4)+k2)^2 - (((x(1)+x(3)+k1)^2-(x(2)+x(4)+k2)^2-(x2^2+y2^2))/(-2*sqrt(x2^2+y2^2)))^2 - x(6)^2;
%                 .5*hydrophone_base_width^2 - k1^2 - 2*(k1+x(3))*x(3) - 2*k1*x(1) - 4*x(1)*x(3);
%                 .5*hydrophone_base_width^2 - k2^2 - 2*(k2+x(4))*x(4) - 2*k2*x(2) - 4*x(2)*x(4)];


        
        options = optimoptions('fsolve','Display','iter', "OptimalityTolerance", 10e-12);
    
        solve1 = fsolve(F, x0, options);
    
        norm1 = norm(F(solve1));
    end

    counter = counter + 1;
    answer(:, counter) = solve1;
    
end



w = (2*answer(3,:)*k1 + answer(3,:).^2 + k1^2 + 2 .* answer(1,:) .* answer(3,:) + 2 * answer(1,:) * k1 - hydrophone_base_width^2 / 4) / -hydrophone_base_width;
h = sqrt(answer(1,:).^2 - w.^2);

%rot = [cos(heading_change),-sin(heading_change);sin(heading_change),cos(heading_change)];

fx1 = [0;0];
length = 1;
for counter = 1:answer_count
    if(w(counter) < 0)
        fx1(:,length) = [1;w(counter)];
        fy1(length)= h(counter);

       length = length + 1;
    end
end

b1 = inv(fx1*fx1') * fx1 * fy1';

w2t = (2*answer(4,:)*k2 + answer(4,:).^2 + k2^2 + 2 .* answer(2,:) .* answer(4,:) + 2 * answer(2,:) * k2 - hydrophone_base_width^2 / 4) / -hydrophone_base_width;
h2t = sqrt(answer(2,:).^2 - w2t.^2);


figure(1)

fx2 = [ones(size(w2t(1,:)));w2t];
fy2= h2t;

b2 = inv(fx2*fx2') * fx2 * fy2';
b2 = -tan(atan(b2) - heading_change);
b2_origin = [0; b2(1)];
rot = [cos(heading_change), -sin(heading_change); sin(heading_change), cos(heading_change)];
b2_origin = rot*b2_origin + [(x1+x2)/2; (y1+y2)/2];

plot(fx1(2,:),fy1, "or", w2t, h2t,"xg")

figure(2)
w_vals = linspace(-200,200, 1000);
h1_vals = b1(1) + b1(2) * w_vals;
h2_vals = b2(2) * w_vals;

plot(w_vals, h1_vals, w_vals + b2_origin(1), h2_vals + b2_origin(2), [-120.04], [2998], "go")

b2_intercept = b2_origin(2) - b2_origin(1) * b2(2);

intersection_x = -(b2_intercept - b1(1))/(b2(2) - b1(2));
intersection_y = b1(2) * intersection_x + b1(1);
