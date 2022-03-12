% Use sys ID to build a controller

close all
clear

% Plant constants
T = 0.4174;     % Time constant, sec
K = 0.7756;     % Static gain, m/s
t_di = 0.15;    % input delay, sec
t_do = 0.15;    % output delay, sec

% System contants
ts = 0.1;       % Sample time, sec

% Build plant
G_cont = tf(K, [T 1], 'InputDelay', t_di, 'OutputDelay', t_do);
G_zoh = c2d(G_cont, ts, 'zoh');

% various plots
% figure, step(G_zoh)
% figure, rlocus(G_zoh)
% figure, bode(G_zoh), grid

% frequency domain, direct discrete, lead lag controller design
gain = 1.0;
PM_goal = 30;
wgc_goal = 0;

PM_now = angle(freqresp(G_zoh, wgc_goal)) + pi;

phi_m = PM_goal - PM_now + 5;
gamma = (1+sin(phi_m)) / (1-sin(phi_m));


% simulate
C = 1.0;
step(feedback(C*G_zoh, 1))