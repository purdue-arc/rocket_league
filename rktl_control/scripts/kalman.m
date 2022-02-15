% Experiment with extended Kalman filter for car's bicycle model
clear
close all

%% Constants
global DELTA_T CAR_LENGTH MAX_STEERING STEERING_RATE MAX_SPEED SPEED_RATE;

ORIGIN_STD_DEV = 0.1;
HEADING_STD_DEV = deg2rad(10);
DELTA_T = 0.1;
DURATION = 10;


CAR_LENGTH = .01;               % wheel to wheel dist, m
MAX_STEERING = deg2rad(15);     % center-side throw, rad
STEERING_RATE = deg2rad(5);    % roc steering, rad per sec
MAX_SPEED = 1;                  % max speed, m/s
SPEED_RATE = 0.1;                 % roc speed, m/s2

%% Generate random measurement data
% pre-allocate matrix to hold output
% each column is x, y, theta
% new column for each time step
raw_data = zeros([3,DURATION/DELTA_T]);

% generate initial state
% random position, zero velocity, centered steering
state = randn([5,1]).*[ORIGIN_STD_DEV; ORIGIN_STD_DEV; HEADING_STD_DEV; 0; 0];
for i = 1:size(raw_data,2)
    % add to raw_data
    raw_data(:,i) = state(1:3);

    % generate random controls
    control = rand([2,1]).*[0.5; 2] + [0.5; -1];

    % step
    state = simulate(state, control);
end


%% Initial guess
% state: [x, y, yaw, v, beta]^T
state = [0;0;0;0;0];

% covariance
% assume variances along diagonal
cov = eye(5).*[ORIGIN_STD_DEV; ORIGIN_STD_DEV; HEADING_STD_DEV; 0.01; deg2rad(1)].^2;

%% PARAMETERS
% process noise
% this is wrong, but maybe an OK guess
Q = eye(5).*[0.001; 0.001; deg2rad(1); 0.1; deg2rad(3)].^2;

% observation matrix (can only see position, not internal state)
H = [eye(3), zeros([3,2])];

% measurement uncertainty
R = eye(3).*[0.05; 0.05; deg2rad(5)].^2;


%% Process via Kalman filter
% pre-allocate matrix to hold output
% first column is state
% others are covariance
% each page corresponds to time step
filtered_data = zeros([5,6,size(raw_data,1)]);

for i = 1:size(raw_data,2)
    % Add to filtered_data
    filtered_data(:,:,i) = [state, cov];

    % Extrapolate
    [next_state, F] = extrapolate(state);
    next_cov = F*cov*F'+ Q;

    % Kalman gain
    gain = next_cov*H'/(H*next_cov*H'+R);

    % Update
    meas = raw_data(:,i);
    state = next_state + gain*(meas - H*next_state);
    cov = (eye(5)-gain*H)*next_cov*(eye(5)-gain*H)' + gain*R*gain';
end

%% Graphical output
figure
plot(raw_data(1,:), raw_data(2,:)), hold
no_cov = reshape(filtered_data(:,1,:), [5,size(filtered_data,3)]);
plot(no_cov(1,:), no_cov(2,:))
plot(raw_data(1,1), raw_data(2,1), '*')


%% Helper functions
function [next_state, F] = extrapolate(state)
    % equations were derived in bicyle_model.m
    global DELTA_T CAR_LENGTH;

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

function next_state = simulate(state, control)
    % simple bicycle model with control
    global DELTA_T CAR_LENGTH MAX_STEERING STEERING_RATE MAX_SPEED SPEED_RATE;

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