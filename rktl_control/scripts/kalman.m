% Experiment with extended Kalman filter for car's bicycle model
clear
close all

%% Constants
% timing
DURATION = 30;                  % duration of data, sec
FILTER_DELTA_T = 0.1;           % step size of filter, sec
TRUTH_DELTA_T = 0.01;           % step size of ground truth simulation, sec
% ^ THESE NEED TO BE INTEGER MULTIPLES

% uncertainties
ORIGIN_LOC_STD_DEV = 0.1;           % uncertainty of initial location, m
ORIGIN_DIR_STD_DEV = deg2rad(10);   % uncertainty of initial heading, rad

MEAS_LOC_STD_DEV = 0.05;            % uncertainty of location measurement, m
MEAS_DIR_STD_DEV = deg2rad(5);      % uncertainty of heading measurment, rad

PROC_VEL_STD_DEV = 0.025;           % process noise for velocity, m/s
PROC_BETA_STD_DEV = deg2rad(0.75);  % process noise for beta, rad

% physical properties of car
global CAR_LENGTH MAX_STEERING STEERING_RATE MAX_SPEED SPEED_RATE;
CAR_LENGTH = .01;               % wheel to wheel dist, m
MAX_STEERING = deg2rad(15);     % center-side throw, rad
STEERING_RATE = deg2rad(30);    % ROC steering, rad per sec
MAX_SPEED = 1;                  % max speed, m/s
SPEED_RATE = 1;                 % ROC speed, m/s2

%% Generate randomized ground truth data
% pre-allocate matrix to hold output
% each column is full state (x, y, theta, v, beta)
% new column for each time step
ground_truth = zeros(5, DURATION/TRUTH_DELTA_T);

% generate initial state
% random position, zero velocity, centered steering
state = randn(5, 1).*[ORIGIN_LOC_STD_DEV; ORIGIN_LOC_STD_DEV; ORIGIN_DIR_STD_DEV; 0; 0];

% generate an initial control (throttle; steering)
% throttle uniform between 0.5 and 1.0
% steering uniform between -1.0 and 1.0
control = rand(2, 1).*[0.5; 2] + [0.5; -1];

for i = 1:size(ground_truth,2)
    % add to raw_data
    ground_truth(:,i) = state;

    % update random control with same frequency as filter
    if (mod(i, FILTER_DELTA_T/TRUTH_DELTA_T) == 0)
        control = rand(2, 1).*[0.5; 2] + [0.5; -1];
    end

    % step
    state = simulate(state, control, TRUTH_DELTA_T);
end

%% Calculate resulting measurement data
% each column is measurement (x, y, theta)
% new column for each time step
measurements = ground_truth(1:3, 1:(FILTER_DELTA_T/TRUTH_DELTA_T):end);

% add artificial noise
measurements = measurements + randn(size(measurements)).*[MEAS_LOC_STD_DEV; MEAS_LOC_STD_DEV; MEAS_DIR_STD_DEV];

%% Paramters for EKF
% process noise
% project v, beta error onto model later
Q0 = eye(5).*[0; 0; 0; PROC_VEL_STD_DEV; PROC_BETA_STD_DEV].^2;

% observation matrix (can only see position, not internal state)
H = [eye(3), zeros(3,2)];

% measurement uncertainty
% assume variances along diagonal
R = eye(3).*[MEAS_LOC_STD_DEV; MEAS_LOC_STD_DEV; MEAS_DIR_STD_DEV].^2;

%% Initial guess
state = [0; 0; 0; 0; 0];

% covariance
% assume variances along diagonal
% pick something small for v, beta
cov = eye(5).*[ORIGIN_LOC_STD_DEV; ORIGIN_LOC_STD_DEV; ORIGIN_DIR_STD_DEV; 0.01; deg2rad(1)].^2;

%% Process via Kalman filter
% pre-allocate matrices to hold output
estimates = zeros(5, size(measurements,2));

% each page corresponds to time step
covariances = zeros(5,5,size(measurements,2));

for i = 1:size(measurements,2)
    % Add to outputs
    estimates(:,i) = state;
    covariances(:,:,i) = cov;

    % Extrapolate
    [next_state, F] = extrapolate(state, FILTER_DELTA_T);
    Q = F*Q0*F';
    next_cov = F*cov*F'+ Q;

    % Kalman gain
    gain = next_cov*H'/(H*next_cov*H'+R);

    % Update
    state = next_state + gain*(measurements(:,i) - H*next_state);
    cov = (eye(5)-gain*H)*next_cov*(eye(5)-gain*H)' + gain*R*gain';
end

%% Graphical output
figure
subplot(2,2,1)
plot(ground_truth(1,:), ground_truth(2,:))
hold, grid on
plot(measurements(1,:), measurements(2,:), 'x')
plot(estimates(1,:), estimates(2,:), '--')
plot(ground_truth(1,1), ground_truth(2,1), '*b')
legend(["Ground Truth", "Measurements", "Estimates"])
title("XY tracking")

subplot(2,2,2)
i = 1;
plot(ground_truth(i, 1:(FILTER_DELTA_T/TRUTH_DELTA_T):end))
hold, grid on
plot(measurements(i,:), 'x')
plot(estimates(i,:), '--')
legend(["Ground Truth", "Measurements", "Estimates"])
title("X tracking")

subplot(2,2,3)
i = 2;
plot(ground_truth(i, 1:(FILTER_DELTA_T/TRUTH_DELTA_T):end))
hold, grid on
plot(measurements(i,:), 'x')
plot(estimates(i,:), '--')
legend(["Ground Truth", "Measurements", "Estimates"])
title("Y tracking")

subplot(2,2,4)
i = 3;
plot(ground_truth(i, 1:(FILTER_DELTA_T/TRUTH_DELTA_T):end))
hold, grid on
plot(measurements(i,:), 'x')
plot(estimates(i,:), '--')
legend(["Ground Truth", "Measurements", "Estimates"])
title("Theta tracking")


%% Helper functions
function [next_state, F] = extrapolate(state, DELTA_T)
    % equations were derived in bicyle_model.m
    global CAR_LENGTH;

    % disect state
    theta = state(3);
    v = state(4);
    beta = state(5);

    % using bicycle model, extrapolate future state
    next_state = state + DELTA_T * [
        v*cos(beta + theta);
        v*sin(beta + theta);
        2*v*sin(beta)/CAR_LENGTH;
        0; 0];

    % calculate jacobian (linearization of this function about this point)
    F = [1, 0, -DELTA_T*v*sin(beta + theta),        DELTA_T*cos(beta + theta),       -DELTA_T*v*sin(beta + theta);
         0, 1,  DELTA_T*v*cos(beta + theta),        DELTA_T*sin(beta + theta),        DELTA_T*v*cos(beta + theta);
         0, 0,                            1, (2*DELTA_T*sin(beta))/CAR_LENGTH, (2*DELTA_T*v*cos(beta))/CAR_LENGTH;
         0, 0,                            0,                                1,                                  0;
         0, 0,                            0,                                0,                                  1];
end

function next_state = simulate(state, control, DELTA_T)
    % simple bicycle model with control
    global CAR_LENGTH MAX_STEERING STEERING_RATE MAX_SPEED SPEED_RATE;

    % unpack input
    x = state(1);
    y = state(2);
    theta = state(3);
    v = state(4);
    beta = state(5);

    v_rear_ref = MAX_SPEED*control(1);
    psi_ref = MAX_STEERING*control(2);

    % convert v, beta to v_rear, psi
    v_rear = v*cos(beta);
    psi = atan(2*tan(beta));

    % update v_rear, psi
    if abs(v_rear_ref - v_rear) < SPEED_RATE*DELTA_T
        v_rear = v_rear_ref;
    elseif v_rear_ref > v_rear
        v_rear = v_rear + SPEED_RATE*DELTA_T;
    else
        v_rear = v_rear - SPEED_RATE*DELTA_T;
    end

    if abs(psi_ref - psi) < STEERING_RATE*DELTA_T
        psi = psi_ref;
    elseif psi_ref > psi
        psi = psi + STEERING_RATE*DELTA_T;
    else
        psi = psi - STEERING_RATE*DELTA_T;
    end

    % convert v_rear, psi to v, beta
    v = v_rear/cos(beta);
    beta = atan(tan(psi)/2);

    % using bicycle model, extrapolate future state
    next_state = [
        x + v*cos(beta + theta)*DELTA_T;
        y + v*sin(beta + theta)*DELTA_T;
        theta + 2*v*sin(beta)/CAR_LENGTH*DELTA_T;
        v;
        beta];
end