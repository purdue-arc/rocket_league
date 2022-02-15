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

PROC_VEL_STD_DEV = 0.05;           % process noise for rear wheel velocity, m/s
PROC_PSI_STD_DEV = deg2rad(0.75);  % process noise for steering angle, rad

% physical properties of car
global CAR_LENGTH MAX_STEERING STEERING_RATE MAX_SPEED SPEED_RATE;
CAR_LENGTH = .01;               % wheel to wheel dist, m
MAX_STEERING = deg2rad(15);     % center-side throw, rad
STEERING_RATE = deg2rad(30);    % ROC steering, rad per sec
MAX_SPEED = 1;                  % max speed, m/s
SPEED_RATE = 1;                 % ROC speed, m/s2

%% Generate randomized ground truth data
% pre-allocate matrix to hold output
% each column is full state (x, y, theta, v_rear, psi)
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
% assume steering and rear wheel velocity are independent
% projecting v_rear, psi noise onto model requires linearization done at each time step
Q0 = eye(5).*[0; 0; 0; PROC_VEL_STD_DEV; PROC_PSI_STD_DEV].^2;

% observation matrix (can only see position, not internal state)
H = [eye(3), zeros(3,2)];

% measurement uncertainty
% assume independence between x, y, theta
R = eye(3).*[MEAS_LOC_STD_DEV; MEAS_LOC_STD_DEV; MEAS_DIR_STD_DEV].^2;

%% Initial guess
state = [0; 0; 0; 0; 0];

% covariance
% assume independence between all
% pick something small for v_rear, psi
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
    next_cov = F*cov*F'+ F*Q0*F';

    % Kalman gain
    gain = next_cov*H'/(H*next_cov*H'+R);

    % Update
    state = next_state + gain*(measurements(:,i) - H*next_state);
    cov = (eye(5)-gain*H)*next_cov*(eye(5)-gain*H)' + gain*R*gain';
end

%% Graphical output
figure
subplot(2,3,1)
plot(ground_truth(1,:), ground_truth(2,:))
hold, grid on
plot(estimates(1,:), estimates(2,:), '--')
plot(measurements(1,:), measurements(2,:), '.')
plot(ground_truth(1,1), ground_truth(2,1), '*b')
legend(["Ground Truth", "Estimates", "Measurements"])
title("XY tracking")

for i = 1:5
    subplot(2,3,i+1)
    plot(ground_truth(i, 1:(FILTER_DELTA_T/TRUTH_DELTA_T):end))
    hold, grid on
    plot(estimates(i,:), '--')
    if (i <= 3)
        plot(measurements(i,:), '.')
        legend(["Ground Truth", "Estimates", "Measurements"])
    else
        legend(["Ground Truth", "Estimates"])
    end
    switch i
        case 1; title("X tracking")
        case 2; title("Y tracking")
        case 3; title("\theta tracking")
        case 4; title("rear wheel velocity tracking")
        case 5; title("steering angle tracking")
    end
end

%% Helper functions
function [next_state, F] = extrapolate(state, DELTA_T)
    % using bicycle model, extrapolate future state
    global CAR_LENGTH;

    % unpack input
    x = state(1);
    y = state(2);
    theta = state(3);
    v_rear = state(4);
    psi = state(5);

    % equations were derived in bicyle_model.m
    % calculate predicted next state
    next_state = [
        x + DELTA_T*v_rear*cos(theta + atan(tan(psi)/2))*(tan(psi)^2/4 + 1)^(1/2);
        y + DELTA_T*v_rear*sin(theta + atan(tan(psi)/2))*(tan(psi)^2/4 + 1)^(1/2);
        theta + (DELTA_T*v_rear*tan(psi))/CAR_LENGTH;
        v_rear;
        psi
    ];

    % calculate jacobian (linearization of this function about this point)
    F = [
        1, 0, -DELTA_T*v_rear*sin(theta + atan(tan(psi)/2))*(tan(psi)^2/4 + 1)^(1/2), DELTA_T*cos(theta + atan(tan(psi)/2))*(tan(psi)^2/4 + 1)^(1/2), (DELTA_T*v_rear*cos(theta + atan(tan(psi)/2))*tan(psi)*(tan(psi)^2 + 1))/(4*(tan(psi)^2/4 + 1)^(1/2)) - (DELTA_T*v_rear*sin(theta + atan(tan(psi)/2))*(tan(psi)^2/2 + 1/2))/(tan(psi)^2/4 + 1)^(1/2);
        0, 1,  DELTA_T*v_rear*cos(theta + atan(tan(psi)/2))*(tan(psi)^2/4 + 1)^(1/2), DELTA_T*sin(theta + atan(tan(psi)/2))*(tan(psi)^2/4 + 1)^(1/2), (DELTA_T*v_rear*cos(theta + atan(tan(psi)/2))*(tan(psi)^2/2 + 1/2))/(tan(psi)^2/4 + 1)^(1/2) + (DELTA_T*v_rear*tan(psi)*sin(theta + atan(tan(psi)/2))*(tan(psi)^2 + 1))/(4*(tan(psi)^2/4 + 1)^(1/2));
        0, 0,                                                                      1,                                  (DELTA_T*tan(psi))/CAR_LENGTH,                                                                                                                                                         (DELTA_T*v_rear*(tan(psi)^2 + 1))/CAR_LENGTH;
        0, 0,                                                                      0,                                                              1,                                                                                                                                                                                                    0;
        0, 0,                                                                      0,                                                              0,                                                                                                                                                                                                    1
    ];
end

function next_state = simulate(state, control, DELTA_T)
    % simple bicycle model with control
    global CAR_LENGTH MAX_STEERING STEERING_RATE MAX_SPEED SPEED_RATE;

    % unpack input
    x = state(1);
    y = state(2);
    theta = state(3);
    v_rear = state(4);
    psi = state(5);

    v_rear_ref = MAX_SPEED*control(1);
    psi_ref = MAX_STEERING*control(2);

    % update v_rear, psi using massless, constant acceleration
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

    % using bicycle model, extrapolate future state
    next_state = [
        x + DELTA_T*v_rear*cos(theta + atan(tan(psi)/2))*(tan(psi)^2/4 + 1)^(1/2);
        y + DELTA_T*v_rear*sin(theta + atan(tan(psi)/2))*(tan(psi)^2/4 + 1)^(1/2);
        theta + (DELTA_T*v_rear*tan(psi))/CAR_LENGTH;
        v_rear;
        psi
    ];
end