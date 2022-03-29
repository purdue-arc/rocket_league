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
G = tf(K, [T 1], 'InputDelay', t_di, 'OutputDelay', t_do);
G_zoh = c2d(G, ts, 'zoh');

% various plots
% figure, step(G_zoh)
% figure, rlocus(G_zoh)
% figure, bode(G_zoh), grid

% frequency domain, lead lag controller design
% Inputs
sse = 0.01; % steady state error. Percent error wrt velocity input
PM = 50;    % desired phase margin. Degrees. Higher increases stability. 30 is suggested minimum
w0 = 10.0;   % desired gain crossover frequency. Rad/s. Higher increases speed

% Calculate required controller gain
Kc = (1/sse - 1)/K;

% Find existing phase and gain at w0
phi_0 = angle(evalfr(G, w0*1i)) + pi;       % radians
if phi_0 > pi
    % only valid until phase is below -360 (w0 a bit over 10 rad/s)
    % after that, need to subtract another 2*pi
    phi_0 = phi_0 - 2*pi;
end
M0 = 20*log10(Kc * abs(evalfr(G, w0*1i)));  % dB

% Lead controller
phi_m = PM - phi_0 + 5;
gamma = (1+sin(phi_m)) / (1-sin(phi_m));
wz = w0 / sqrt(gamma);
wp = w0 * sqrt(gamma);
G_lead = gamma * tf([1 wz], [1 wp]);

% Lag controller
M_lead = 10*log10(gamma);
M_lag = -(M0 + M_lead);
Gamma = 10^(M_lag/-20);
wz = w0 / 10;
wp = w0 / Gamma;
G_lag = 1/Gamma * tf([1 wz], [1 wp]);

% Full lead-lag controller
C = Kc * G_lead * G_lag;
figure, margin(C*G)

% simulate
figure, step(feedback(C*G, 1))