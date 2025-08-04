close all
% wn = 1;
Kp = 4.8; % Ku = 8, Ts = 3.6 => P: 0.5Kp, PI: 0.45Kp 1.2Kp/Ts PID: 0.6Kp 2Kp/Ts KpTs/8
Ki = 2*8/3.6;
Kd = 8*3.6/8;
num = [1];
den = [1 3 3 1];
Dt = 0.1;
sim('DC_Motor_PID_Control')

figure 
plot(y)
hold on 
ylabel('Amplitude')