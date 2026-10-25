%% Constrained vs. Unconstrained MPC

% MPC cost, prediction model, target Initial condition
% Stays the same just adding lb ub in quadprog

clc;
close all;
load('cart_pole_system.mat');

x_current = [0;0;0.2;0];

ssteps = 100;

x_history_unconstrained = zeros(n,ssteps+1);
u_history_unconstrained = zeros(m_in,ssteps);

t_history = (0:ssteps)*Ts;

x_history_unconstrained(:,1) = x_current;

%Unconstrained mpc loop (structure same as day 4)
for step = 1:ssteps

    f = 2 * Phi' * Qbar * (M*x_current - X_ref);

    [U_opt, ~, exitflag] = quadprog(H, f, [], [], [], [], [], [], [], options);

    if exitflag ~= 1
        warning('QP failed at step %d, stopping simulation', step);
        break;
    end

    u0 = U_opt(1:m_in);

    x_next = A*x_current + B*u0;

    u_history_unconstrained(:,step) = u0;
    x_history_unconstrained(:,step+1) = x_next;

    x_current = x_next;
end

max_force_unconstrained = max(abs(u_history_unconstrained));

fprintf('Maximum unconstrained force: %.4f N\n', max_force_unconstrained);

% new constrained simulation

u_max = 10; %chosen so constraint is active
u_min = -u_max;
% create the limits for all 10 predicted control inputs.
lb = u_min * ones(N*m_in,1);
ub = u_max * ones(N*m_in,1);

x_current=[0;0;0.2;0]; %using same initials
x_history_constrained = zeros(n,ssteps+1);
u_history_constrained = zeros(m_in,ssteps);

x_history_constrained(:,1) = x_current;

% Constrained MPC loop



for step = 1:ssteps

    f = 2 * Phi' * Qbar * (M*x_current - X_ref);

    [U_opt, ~, exitflag] = quadprog(H, f, [], [], [], [], lb, ub, [], options);

    if exitflag ~= 1
        warning('QP failed at step %d, stopping simulation', step);
        break;
    end

    u0 = U_opt(1:m_in);

    x_next = A*x_current + B*u0;

    u_history_constrained(:,step) = u0;
    x_history_constrained(:,step+1) = x_next;

    x_current = x_next;
end

max_force_constrained = max(abs(u_history_constrained));

fprintf('Maximum constrained force: %.4f N\n', max_force_constrained);
fprintf('Constraint: %.1f N to %.1f N\n',u_min,u_max);

figure;


stairs(t_history(1:end-1),u_history_unconstrained(1,:),'LineWidth',1.5);
hold on;

stairs(t_history(1:end-1),u_history_constrained(1,:),'--','LineWidth',1.5);

yline(10,'--','Maximum force');
yline(-10,'--','Minimum force');

xlabel('Time (s)');
ylabel('Control force (N)');
title('Constrained vs. Unconstrained Control Force');

legend('Unconstrained','Constrained','Location','best');

grid on;

% Compare position error

target_position = 0.5;
error_unconstrained = target_position - x_history_unconstrained(1,:);
error_constrained = target_position - x_history_constrained(1,:);

max_error_unconstrained = max(abs(error_unconstrained));
max_error_constrained = max(abs(error_constrained));

fprintf('Maximum unconstrained position error: %.4f m\n', max_error_unconstrained);
fprintf('Maximum constrained position error: %.4f m\n', max_error_constrained);


figure;

plot(t_history,error_unconstrained,'LineWidth',1.5);
hold on;

plot(t_history,error_constrained,'--','LineWidth',1.5);

yline(0,'--');

xlabel('Time (s)');
ylabel('Position error (m)');
title('Tracking Error: Constrained vs. Unconstrained MPC');

legend('Unconstrained','Constrained','Location','best');

grid on;

max_angle_unconstrained = max(abs(x_history_unconstrained(3,:)));
max_angle_constrained = max(abs(x_history_constrained(3,:)));

fprintf('Maximum unconstrained pole angle: %.4f rad\n', max_angle_unconstrained);
fprintf('Maximum constrained pole angle: %.4f rad\n', max_angle_constrained);
figure;

plot(t_history,x_history_unconstrained(3,:),'LineWidth',1.5);
hold on;

plot(t_history,x_history_constrained(3,:),'--','LineWidth',1.5);

yline(0,'--');

xlabel('Time (s)');
ylabel('Pole angle (rad)');
title('Pole Angle: Constrained vs. Unconstrained MPC');

legend('Unconstrained','Constrained','Location','best');

grid on;
