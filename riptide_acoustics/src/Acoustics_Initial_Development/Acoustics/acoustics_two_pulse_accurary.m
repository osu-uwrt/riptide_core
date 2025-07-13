%do some accurary verification
%units = mm

clear;

%cases that need evaluated for
    % - negative sqrts in eq1 and eq2
    % - sign of slope 2


%using little boat base width

hydrophone_base_width = 50;
water_speed = 1500000;
rp2040_adc_sampling_frequency = 5*10^5;
MHz1_adc_sampling_frequency = 5*10^6;

adc_distance_accurarcy_rp2040 = 1 / (rp2040_adc_sampling_frequency) * water_speed;
adc_distance_accurarcy_1MHZ = 1 / (MHz1_adc_sampling_frequency) * water_speed;

samples = 10;

%should yeild (2998.5,95.04)
% distances = [2,19];
% transform = [-95.04, 8.49, -.38207];

%should yeild (-44 ,98.17)
% distances = [20, -7.91];
% transform = [-150, 10, -.6604];

%should yeild (52,116.92)
% distances = [-20, -18.3411];
% transform = [-175, 10, -.6604];

%should yeild (52,116.92)
distances = [-20, 41.465];
transform = [141.89, 22.89, -.26074];

%should yield [1,1]
%distances = [35.36;29.7];


[slv_x,slv_y] = solve_two_pulse_system(distances(1),distances(2),transform(1),transform(2),transform(3),50, 3000)

positions_5 = zeros(2,samples);
count = 1;
while count <= samples  

    [x,y] = solve_two_pulse_system(distances(1) + (rand() - .5) *adc_distance_accurarcy_rp2040,distances(2) + (rand() - .5) *adc_distance_accurarcy_rp2040,transform(1),transform(2),transform(3),50, 1000);
    positions_5(1, count) = x;
    positions_5(2, count) = y;

    count = count + 1;
end


positions_6 = zeros(2,samples);
count = 1;
while count <= samples  

    [x,y] = solve_two_pulse_system(distances(1) + (rand() - .5) *adc_distance_accurarcy_1MHZ,distances(2) + (rand() - .5) *adc_distance_accurarcy_1MHZ,transform(1),transform(2),transform(3),50, 1000);
    positions_6(1, count) = x;
    positions_6(2, count) = y;

    count = count + 1;
end

figure(1)
plot([slv_x], [slv_y], "xg", positions_5(1,:), positions_5(2,:), "or", positions_6(1,:), positions_6(2,:), "ob")
title("Solve Results for Acoustics Target at -1m,1m for LB Baseline")
xlabel("Solved Distance (mm)")
ylabel("Solved Distance (mm)")
legend("Solve With No Error", "RP2040 ADC", "1MHz ADC")

% 
% %talos baseline
% 
% %should yield[-1,1]
% distances = [141.2444;118.367];
% 
% %should yield[1,1]
% distances = [-141.2444; -167.7482];
% 
% [slv_x,slv_y] = solve_two_pulse_system(distances(1),distances(2),-400,100,.05,200, 10);
% 
% positions_5 = zeros(2,samples);
% count = 1;
% while count <= samples  
% 
%     [x,y] = solve_two_pulse_system(distances(1) + (rand() - .5) *adc_distance_accurarcy_rp2040,distances(2) + (rand() - .5) *adc_distance_accurarcy_rp2040,-400,100,.05,200, 10);
%     positions_5(1, count) = x;
%     positions_5(2, count) = y;
% 
%     count = count + 1
% end
% 
% 
% positions_6 = zeros(2,samples);
% count = 1;
% while count <= samples  
% 
%     [x,y] = solve_two_pulse_system(distances(1) + (rand() - .5) * adc_distance_accurarcy_1MHZ,distances(2) + (rand() - .5) *adc_distance_accurarcy_1MHZ,-400,100,.05,200, 10);
%     positions_6(1, count) = x;
%     positions_6(2, count) = y;
% 
%     count = count + 1
% end
% 
% figure(2)
% plot([slv_x], [slv_y], "xg", positions_5(1,:), positions_5(2,:), "or", positions_6(1,:), positions_6(2,:), "ob")
% title("Solve Results for Acoustics Target at -1m,1m for Talos Baseline")
% xlabel("Solved Distance (mm)")
% ylabel("Solved Distance (mm)")
% legend("Solve With No Error", "RP2040 ADC", "1MHz ADC")