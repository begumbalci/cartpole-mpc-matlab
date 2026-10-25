% Evaluate the system's behavior

clc;
close all;

% From day5 x_ref=[0.5;0;0;0]

target_position = 0.5;

error_position = target_position - x_history(1,:);

% Draw the graph
% Position Error graph
figure;
plot(t_history, error_position);

xlabel('Time(s)');
ylabel('Position Error (m)');
title('Cart-Pole Error Tracking(m)');
grid on;
yline(0,'--');

% Position tracking graph
figure;

plot(t_history, x_history(1,:), 'LineWidth', 1.5);
hold on;

yline(0.5, '--', 'Target');

xlabel('Time (s)');
ylabel('Cart position (m)');
title('Cart Position Tracking');

grid on;

% Graph Angle of cart-pole (x_history(3,:))

figure;

plot(t_history, x_history(3,:), 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Pole angle (rad)');
title('Pole Angle');
load('mpc_results.mat');

grid on;

yline(0, '--', 'Upright');

%% Finding the constraing Day6 
% Find the max angle 
max_angle = max(abs(x_history(3,:)));

fprintf('Maximum pole angle: %.4f rad\n', max_angle);

max_force = max(abs(u_history));

fprintf('Maximum control force: %.4f N\n', max_force);
fprintf('Input limits: %.2f N to %.2f N\n', u_min, u_max);
