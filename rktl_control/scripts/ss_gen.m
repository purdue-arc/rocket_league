% Generate desired state space model from overshoot and settling time

close all

OS = 0.05;  % overshoot
ts = 1;     % settling time (s)

zeta = -log(OS)/sqrt(pi^2 + log(OS)^2);
sigma = 4/ts;
wn = sigma/zeta;

G = tf(wn^2, [1 2*zeta*wn wn^2]);

step(G), grid, hold
% step(c2d(G, 1/35, 'zoh'))
% step(c2d(G, 1/35, 'tustin'))

H = ss(G)
T = 1/35;
I = c2d(H, T, 'tustin');

t = 0:T:1.8;
x = [0; 0];
y = [];

for i = t
    y = [y H.C * x];
    xdot = H.A * x + H.B;
    x = x + xdot * 1/35;
end
plot(t, y, ':');

x = [0; 0];
y = [];
for i = t
    y = [y I.C * x];
    x = I.A * x + I.B;
end
plot(t, y, ':');