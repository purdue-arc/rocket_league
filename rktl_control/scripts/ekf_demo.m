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

PROC_PSI_STD_DEV = deg2rad(0.75);   % process noise for steering angle, rad
PROC_ACL_STD_DEV = 0.1;             % process noise for rear wheel acceleration, m/s2

% physical properties of car
global CAR_LENGTH MAX_SPEED THROTTLE_TAU STEERING_THROW STEERING_RATE
CAR_LENGTH = .01;               % wheel to wheel dist, m
MAX_SPEED = 1;                  % max speed, m/s
THROTTLE_TAU = 0.25;            % time constant for changing speed, sec
STEERING_THROW = deg2rad(15);   % center-side steering throw, rad
STEERING_RATE = deg2rad(30);    % ROC steering, rad per sec

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
Q0 = eye(6).*[0; 0; 0; 0; PROC_PSI_STD_DEV; PROC_ACL_STD_DEV].^2;

% observation matrix (can only see position, not internal state)
H = [eye(3), zeros(3, 3)];

% measurement uncertainty
% assume independence between x, y, theta
R = eye(3).*[MEAS_LOC_STD_DEV; MEAS_LOC_STD_DEV; MEAS_DIR_STD_DEV].^2;

%% Initial guess
state = zeros(6, 1);

% covariance
% assume independence between all
% pick something small for v_rear, psi, a_rear
cov = eye(6).*[ORIGIN_LOC_STD_DEV; ORIGIN_LOC_STD_DEV; ORIGIN_DIR_STD_DEV; 0.01; deg2rad(1); 0.01].^2;

%% Process via Kalman filter
% pre-allocate matrices to hold output
estimates = zeros(6, size(measurements,2));

% each page corresponds to time step
covariances = zeros(6, 6, size(measurements,2));

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
    cov = (eye(6)-gain*H)*next_cov*(eye(6)-gain*H)' + gain*R*gain';
end

%% Calculate odometry output
beta = atan(tan(ground_truth(5,:))/2);
v_body = ground_truth(4,:)/cos(beta);
curvature = 2*sin(beta)/CAR_LENGTH;
odom_true = [
    ground_truth(1:3,:);
    v_body*cos(beta);
    v_body*sin(beta);
    v_body*curvature
];
truth_time = 0:TRUTH_DELTA_T:(DURATION-TRUTH_DELTA_T);

beta = atan(tan(estimates(5,:))/2);
v_body = estimates(4,:)/cos(beta);
curvature = 2*sin(beta)/CAR_LENGTH;
odom_est = [
    estimates(1:3,:);
    v_body*cos(beta);
    v_body*sin(beta);
    v_body*curvature
];
filter_time = 0:FILTER_DELTA_T:(DURATION-FILTER_DELTA_T);

%% Graphical output
figure
plot(ground_truth(1,:), ground_truth(2,:))
hold, grid on
plot(estimates(1,:), estimates(2,:), '--')
plot(measurements(1,:), measurements(2,:), '.')
plot(ground_truth(1,1), ground_truth(2,1), '*b')
legend(["Ground Truth", "Estimates", "Measurements"], 'Location', 'Best')
title("XY tracking")
xlabel("x, m")
ylabel("y, m")

figure
sgtitle("Internal state tracking")
for i = 1:5
    subplot(2,3,i)
    plot(truth_time, ground_truth(i, :))
    hold, grid on
    plot(filter_time, estimates(i,:), '--')
    if (i <= 3)
        plot(filter_time, measurements(i,:), '.')
        legend(["Ground Truth", "Estimates", "Measurements"], 'Location', 'Best')
    else
        legend(["Ground Truth", "Estimates"], 'Location', 'Best')
    end
    xlabel("Time, sec")
    switch i
        case 1; title("X tracking"); ylabel("x, m")
        case 2; title("Y tracking"); ylabel("y, m")
        case 3; title("Heading tracking"); ylabel("\theta, rad")
        case 4; title("Rear wheel velocity tracking"); ylabel("v_{rear}, m/s")
        case 5; title("Steering angle tracking"); ylabel("\psi, rad")
    end
end

figure
sgtitle("External state tracking")
for i = 1:6
    subplot(2,3,i)
    plot(truth_time, odom_true(i, :))
    hold, grid on
    plot(filter_time, odom_est(i,:), '--')
    if (i <= 3)
        plot(filter_time, measurements(i,:), '.')
        legend(["Ground Truth", "Estimates", "Measurements"], 'Location', 'Best')
    else
        legend(["Ground Truth", "Estimates"], 'Location', 'Best')
    end
    xlabel("Time, sec")
    switch i
        case 1; title("X tracking"); ylabel("x, m")
        case 2; title("Y tracking"); ylabel("y, m")
        case 3; title("Heading tracking"); ylabel("\theta, rad")
        case 4; title("Body forward velocity tracking"); ylabel("x', m/s")
        case 5; title("Body lateral velocity tracking"); ylabel("y', m/s")
        case 6; title("Angular velocity tracking"); ylabel("\omega, rad/sec")
    end
end

%% Helper functions
function [next_state, F] = extrapolate(state, DELTA_T)
    % using bicycle model, extrapolate future state
    global CAR_LENGTH

    % unpack input
    x = state(1);
    y = state(2);
    theta = state(3);
    v_rear = state(4);
    psi = state(5);
    a_rear = state(6);

    % equations were derived in bicyle_model.m
    % calculate predicted next state
    next_state = [
        x + DELTA_T*v_rear*cos(theta + atan(tan(psi)/2))*(tan(psi)^2/4 + 1)^(1/2);
        y + DELTA_T*v_rear*sin(theta + atan(tan(psi)/2))*(tan(psi)^2/4 + 1)^(1/2);
        theta + (DELTA_T*v_rear*tan(psi))/CAR_LENGTH;
        v_rear + DELTA_T*a_rear;
        psi;
        a_rear
    ];

    % calculate jacobian (linearization of this function about this point)
    F = [
        1, 0, -DELTA_T*v_rear*sin(theta + atan(tan(psi)/2))*(tan(psi)^2/4 + 1)^(1/2), DELTA_T*cos(theta + atan(tan(psi)/2))*(tan(psi)^2/4 + 1)^(1/2), (DELTA_T*v_rear*cos(theta + atan(tan(psi)/2))*tan(psi)*(tan(psi)^2 + 1))/(4*(tan(psi)^2/4 + 1)^(1/2)) - (DELTA_T*v_rear*sin(theta + atan(tan(psi)/2))*(tan(psi)^2/2 + 1/2))/(tan(psi)^2/4 + 1)^(1/2),       0;
        0, 1,  DELTA_T*v_rear*cos(theta + atan(tan(psi)/2))*(tan(psi)^2/4 + 1)^(1/2), DELTA_T*sin(theta + atan(tan(psi)/2))*(tan(psi)^2/4 + 1)^(1/2), (DELTA_T*v_rear*cos(theta + atan(tan(psi)/2))*(tan(psi)^2/2 + 1/2))/(tan(psi)^2/4 + 1)^(1/2) + (DELTA_T*v_rear*tan(psi)*sin(theta + atan(tan(psi)/2))*(tan(psi)^2 + 1))/(4*(tan(psi)^2/4 + 1)^(1/2)),       0;
        0, 0,                                                                      1,                                  (DELTA_T*tan(psi))/CAR_LENGTH,                                                                                                                                                         (DELTA_T*v_rear*(tan(psi)^2 + 1))/CAR_LENGTH,       0;
        0, 0,                                                                      0,                                                              1,                                                                                                                                                                                                    0, DELTA_T;
        0, 0,                                                                      0,                                                              0,                                                                                                                                                                                                    1,       0;
        0, 0,                                                                      0,                                                              0,                                                                                                                                                                                                    0,       1
    ];
end

function next_state = simulate(state, control, DELTA_T)
    % simple bicycle model with control
    global CAR_LENGTH MAX_SPEED THROTTLE_TAU STEERING_THROW STEERING_RATE

    % unpack input
    x = state(1);
    y = state(2);
    theta = state(3);
    v_rear = state(4);
    psi = state(5);

    v_rear_ref = MAX_SPEED*control(1);
    psi_ref = STEERING_THROW*control(2);

    % update rear wheel velocity using 1st order model
    v_rear = (v_rear-v_rear_ref)*exp(-DELTA_T/THROTTLE_TAU) + v_rear_ref;

    % update steering angle using massless acceleration to a fixed rate
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