% Use sys ID to build a controller
close all
clear

% Plant constants
T = 0.4174;     % Time constant, sec
K = 0.7756;     % Static gain, m/s

t_di = 0.15;    % input delay, sec
t_do = 0.05;    % input delay, sec

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
sse = 0.05; % steady state error. Percent error wrt velocity input
PM = 60;    % desired phase margin. Degrees. Higher increases stability. 30 is suggested minimum
w0 = 4.0;   % desired gain crossover frequency. Rad/s. Higher increases speed

% Calculate required controller gain
Kc = (1/sse - 1)/K;

% Find existing phase and gain at w0
[m, p] = bode(G, w0);
phi_0 = p + 180;        % deg
M0 = 20*log10(m * Kc);  % dB

% Lead controller
phi_m = PM - phi_0 + 5;     % can tweak the 5 degree fudge factor if desired
gamma = (1+sind(phi_m)) / (1-sind(phi_m));
wz_lead = w0 / sqrt(gamma);
wp_lead = w0 * sqrt(gamma);
G_lead = gamma * tf([1 wz_lead], [1 wp_lead]);

% Lag controller
M_lead = 10*log10(gamma);
M_lag = -(M0 + M_lead);
Gamma = 10^(M_lag/-20);
wz_lag = w0 / 10;
wp_lag = wz_lag / Gamma;
G_lag = 1/Gamma * tf([1 wz_lag], [1 wp_lag]);

% Full lead-lag controller
C = Kc * G_lead * G_lag;
% figure, margin(C*G)

% simulate
figure, step(feedback(C*G, 1)), grid, hold

% discretize
Cd_rect = c2d(C, ts, 'zoh');
step(feedback(Cd_rect*G_zoh, 1))

Cd_trap = c2d(C, ts, 'tustin');
step(feedback(Cd_trap*G_zoh, 1))

legend(["Analog Controller", "Rectangular Integration", "Trapezoidal Integration"])

% output
zpk(Cd_trap)
