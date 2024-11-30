%Anshul Jain, CU Boulder
%November 30, 2024

%Implementation of a PID Controller on a Mass-Spring-Damper system
%The goal of the implementation is to control the position of the mass (x)
%such that it follows a setpoint x_eq.

%Differential equation for the considered system:
% m*x_dd + c*x_d + k*x = F

%Hence, the input for this system is F (force) and the desired output is x
%(position).
%%
%Initialize system parameters
m = 1;          %Mass (kg)
c = 2;          %Damping Coefficient (N-s/m)
k = 20;         %Spring Constant (N/m)

%Initial Conditions
x = 0;          %Initial position (m)
v = 0;          %Initial velocity (m/s)

%Desired setpoint
x_des = 1;      %Desired position (m)

%PID Controller Gain and Errors
Kp = 100;       %Proportional Gain
Ki = 50;        %Integral Gain
Kd = 20;        %Derivative Gain
e = 0;          %Current Error
e_prev = 0;     %Error at previous time step
e_sum = 0;      %Integral of error until current time step
e_dot = 0;      %Derivative of error at current time step
%%
%Simulation time step
dt = 0.01;                      %Time step (s)
t_end = 100;                    %Total simulation time (s)
t = 0:dt:t_end;                 %Time vector

%Data storage
x_history = zeros(size(t));
F_history = zeros(size(t));

%Simulation Loop
for i = 1:length(t)
    %Error calculation
    e = x_des - x;              %Current Error
    e_sum = e_sum + (e*dt);     %Cumulative Error
    e_dot = (e - e_prev)/dt;    %Derivative of Current Error

    %PID Control Law
    F = (Kp*e) + (Ki*e_sum) + (Kd*e_dot);

    %Update system dynamics
    a = (F - (c*v) - (k*x))/m;  %Acceleration
    v = v + (a*dt);             %Velocity
    x = x + (v*dt);             %Position

    %Store updates states
    x_history(i) = x;
    F_history(i) = F;

    %Update error
    e_prev = e;
end
%%
%Plot the results
x_min = min(x_history);
x_max = max(x_history);
disp('Position variation:')
disp([x_min, x_max]);
F_min = min(F_history);
F_max = max(F_history);
disp('Force variation:')
disp([F_min, F_max]);

figure;
subplot(2,1,1);
plot(t, x_history, 'b', 'LineWidth', 1.25);
yline(x_des, 'r--', 'LineWidth', 2);
ylim([0 1.05])
xlim([0 8])
xlabel('Time (s)');
ylabel('Position (m)');
title('Position vs Time')
legend('x', 'Setpoint');
grid on;
hold on;

subplot(2,1,2);
plot(t, F_history, 'k', 'LineWidth', 1.25);
ylim([-400 50])
xlim([0 2])
xlabel('Time (s)');
ylabel('Force (N)');
title('Force vs Time');
grid on;