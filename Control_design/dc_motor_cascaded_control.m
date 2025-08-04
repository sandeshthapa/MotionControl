% High-Precision DC Motor Position Control with Cascaded Loops
s = tf('s');

%% Motor Parameters
K = 10;          % DC motor gain
tau = 0.05;      % Motor time constant (s)
G_motor = tf(K, [tau 1 0]);  % Position from voltage

%% Velocity Loop Design (Inner Loop)
G_vel = tf(K, [tau 1]);      % Velocity transfer function
Kp_v = 0.5;                  % Velocity loop proportional gain
Ki_v = 10;                   % Velocity loop integral gain
C_vel = Kp_v + Ki_v / s;     % Velocity PI controller

L_vel = C_vel * G_vel;
figure; margin(L_vel); grid on; title('Velocity Loop Open-Loop Bode');

G_inner = feedback(L_vel, 1);    % Inner closed-loop system (velocity)
G_pos = G_inner / s;             % Integrate velocity to get position

%% Position Loop Design (Outer Loop)
Kp_p = 2; Ki_p = 5; Kd_p = 0.01;
C_pos = Kp_p + Ki_p/s + Kd_p*s;  % PID controller for position

L_pos = C_pos * G_pos;
figure; margin(L_pos); grid on; title('Position Loop Open-Loop Bode');

%% Closed-loop simulation
T_closed = feedback(L_pos, 1);
t = 0:0.01:5;
figure; step(T_closed, t); grid on;
title('Closed-Loop Step Response (Position)');
ylabel('Position (rad)'); xlabel('Time (s)');
