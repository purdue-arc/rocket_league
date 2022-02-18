% Experiment with different types of filters (ekf, ukf, particle)
clear
close all

%% Constants
% timing
global DELTA_T
DURATION = 15;          % duration of data, sec
DELTA_T = 0.1;          % step size of filter, sec
TRUTH_DELTA_T = 0.001;  % step size of ground truth simulation, sec

STEPS = DURATION/DELTA_T;
TRUTH_STEPS = DURATION/TRUTH_DELTA_T;

% uncertainties
ORIGIN_LOC_STD_DEV = 0.1;           % uncertainty of initial location, m
ORIGIN_DIR_STD_DEV = deg2rad(10);   % uncertainty of initial heading, rad

global MEAS_LOC_STD_DEV MEAS_DIR_STD_DEV
MEAS_LOC_STD_DEV = 0.05;            % uncertainty of location measurement, m
MEAS_DIR_STD_DEV = deg2rad(5);      % uncertainty of heading measurment, rad

PROC_PSI_STD_DEV = deg2rad(1.5);    % process noise for steering angle, rad
PROC_ACL_STD_DEV = 0.2;             % process noise for rear wheel acceleration, m/s2
NUM_PARTICLES = 1000;               % particles in filter

% physical properties of car
global CAR_LENGTH MAX_SPEED THROTTLE_TAU STEERING_THROW STEERING_RATE
CAR_LENGTH = .01;               % wheel to wheel dist, m
MAX_SPEED = 1;                  % max speed, m/s
THROTTLE_TAU = 0.25;            % time constant for changing speed, sec
STEERING_THROW = deg2rad(15);   % center-side steering throw, rad
STEERING_RATE = deg2rad(30);    % ROC steering, rad per sec

%% Generate randomized control vectors
% generate an initial control (throttle; steering)
% throttle uniform between 0.5 and 1.0
% steering uniform between -1.0 and 1.0
control = rand(2, STEPS).*[0.5; 2] + [0.5; -1];

%% Generate resulting ground truth data
% pre-allocate matrix to hold output
% each column is full state (x, y, theta, v_rear, psi)
% new column for each time step
ground_truth = zeros(5, TRUTH_STEPS);

% generate initial state
% random position, zero velocity, centered steering
state = randn(5, 1).*[ORIGIN_LOC_STD_DEV; ORIGIN_LOC_STD_DEV; ORIGIN_DIR_STD_DEV; 0; 0];

j = 1;
for i = 1:TRUTH_STEPS
    % add to raw_data
    ground_truth(:,i) = state;

    % step
    state = simulate(state, control(:,j), TRUTH_DELTA_T);

    % update index to control vector
    if (mod(i, TRUTH_STEPS/STEPS) == 0)
        j = j + 1;
    end
end

%% Calculate resulting measurement data
% each column is measurement (x, y, theta)
% new column for each time step
measurements = ground_truth(1:3, 1:(TRUTH_STEPS/STEPS):end);

% add artificial noise
measurements = measurements + randn(size(measurements)).*[MEAS_LOC_STD_DEV; MEAS_LOC_STD_DEV; MEAS_DIR_STD_DEV];

%% Create inputs to filters
% initial guesses
state = zeros(6, 1);
cov = diag([ORIGIN_LOC_STD_DEV, ORIGIN_LOC_STD_DEV, ORIGIN_DIR_STD_DEV, 0.01, deg2rad(1), 0.01]).^2;

% measurement noise
R = diag([MEAS_LOC_STD_DEV; MEAS_LOC_STD_DEV; MEAS_DIR_STD_DEV]);

% process noise
Q0 = diag([0, 0, 0, 0, PROC_PSI_STD_DEV, PROC_ACL_STD_DEV]).^2;
F = state_jacobian_func(zeros(6,1));
Q = F*Q0*F';

%% Create filters
% EKF (additive)
ekf = extendedKalmanFilter(@state_func, @meas_func, state);
ekf.StateTransitionJacobianFcn = @state_jacobian_func;
ekf.MeasurementJacobianFcn = @meas_jacobian_func;
ekf.StateCovariance = cov;
ekf.MeasurementNoise = R;
ekf.ProcessNoise = Q;

% UKF (additive)
ukf = unscentedKalmanFilter(@state_func, @meas_func, state);
ukf.StateCovariance = cov;
ukf.MeasurementNoise = R;
ukf.ProcessNoise = Q;

% Particle filter
pf = particleFilter(@particle_state_func, @particle_meas_func);
initialize(pf, NUM_PARTICLES, zeros(5,1), cov(1:5,1:5));

%% Pre-allocate output
ekf_state = zeros(6, STEPS);
ukf_state = zeros(6, STEPS);
pf_state = zeros(5, STEPS);

%% Run filters
for i = 1:DURATION/DELTA_T
    %EKF
    [ekf_state(:,i), ~] = correct(ekf, measurements(:,i));
    F = state_jacobian_func(ekf_state(:,i));
    ekf.ProcessNoise = F*Q0*F';
    predict(ekf);

    % UKF
    [ukf_state(:,i), ~] = correct(ukf, measurements(:,i));
    F = state_jacobian_func(ukf_state(:,i));
    ukf.ProcessNoise = F*Q0*F';
    predict(ukf);

    % Particle
    pf_state(:,i) = correct(pf, measurements(:,i));
    predict(pf);
end

%% Plotting
% time vectors
truth_time = 0:TRUTH_DELTA_T:(DURATION-TRUTH_DELTA_T);
time = 0:DELTA_T:(DURATION-DELTA_T);

% convert to odom
truth_odom = state_to_odom(ground_truth);
ekf_odom = state_to_odom(ekf_state);
ukf_odom = state_to_odom(ukf_state);
pf_odom = state_to_odom(pf_state);

% XY plot
figure
plot(truth_odom(1,:), truth_odom(2,:))
hold, grid on
plot(ekf_odom(1,:), ekf_odom(2,:), '--')
plot(ukf_odom(1,:), ukf_odom(2,:), '--')
plot(pf_odom(1,:), pf_odom(2,:), '--')
plot(measurements(1,:), measurements(2,:), 'x:')
plot(truth_odom(1,1), truth_odom(2,1), '*b')
legend(["Ground Truth", "EKF", "UKF", "PF", "Measurements"], 'Location', 'Best')
title("XY tracking")
xlabel("x, m")
ylabel("y, m")

% Odom tracking
figure
sgtitle("External state tracking")
for i = 1:6
    subplot(2,3,i)
    plot(truth_time, truth_odom(i, :))
    hold, grid on
    plot(time, ekf_odom(i,:), '--')
    plot(time, ukf_odom(i,:), '--')
    plot(time, pf_odom(i,:), '--')
    if (i <= 3)
        plot(time, measurements(i,:), 'x:')
        legend(["Ground Truth", "EKF", "UKF", "PF", "Measurements"], 'Location', 'Best')
    else
        legend(["Ground Truth", "EKF", "UKF", "PF"], 'Location', 'Best')
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
% transition functions (additive noise) for EKF, UFK
function next_state = state_func(state)
    % using bicycle model, extrapolate future state
    global CAR_LENGTH DELTA_T

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
end

function measurement = meas_func(state)
    % basically just the H matrix as a function
    measurement = state(1:3);
end

% jacobian functions for EKF
function jacobian = state_jacobian_func(state)
    global CAR_LENGTH DELTA_T

    % unpack input
    x = state(1);
    y = state(2);
    theta = state(3);
    v_rear = state(4);
    psi = state(5);
    a_rear = state(6);

    % equations were derived in bicyle_model.m
    jacobian = [
        1, 0, -DELTA_T*v_rear*sin(theta + atan(tan(psi)/2))*(tan(psi)^2/4 + 1)^(1/2), DELTA_T*cos(theta + atan(tan(psi)/2))*(tan(psi)^2/4 + 1)^(1/2), (DELTA_T*v_rear*cos(theta + atan(tan(psi)/2))*tan(psi)*(tan(psi)^2 + 1))/(4*(tan(psi)^2/4 + 1)^(1/2)) - (DELTA_T*v_rear*sin(theta + atan(tan(psi)/2))*(tan(psi)^2/2 + 1/2))/(tan(psi)^2/4 + 1)^(1/2),       0;
        0, 1,  DELTA_T*v_rear*cos(theta + atan(tan(psi)/2))*(tan(psi)^2/4 + 1)^(1/2), DELTA_T*sin(theta + atan(tan(psi)/2))*(tan(psi)^2/4 + 1)^(1/2), (DELTA_T*v_rear*cos(theta + atan(tan(psi)/2))*(tan(psi)^2/2 + 1/2))/(tan(psi)^2/4 + 1)^(1/2) + (DELTA_T*v_rear*tan(psi)*sin(theta + atan(tan(psi)/2))*(tan(psi)^2 + 1))/(4*(tan(psi)^2/4 + 1)^(1/2)),       0;
        0, 0,                                                                      1,                                  (DELTA_T*tan(psi))/CAR_LENGTH,                                                                                                                                                         (DELTA_T*v_rear*(tan(psi)^2 + 1))/CAR_LENGTH,       0;
        0, 0,                                                                      0,                                                              1,                                                                                                                                                                                                    0, DELTA_T;
        0, 0,                                                                      0,                                                              0,                                                                                                                                                                                                    1,       0;
        0, 0,                                                                      0,                                                              0,                                                                                                                                                                                                    0,       1
    ];
end

function jacobian = meas_jacobian_func(~)
    jacobian = [eye(3), zeros(3)];
end

% Functions for particle filter
function particles = particle_state_func(particles)
    for i = 1:size(particles, 2)
        % create random control noise
        control = rand(2,1).*[0.5; 2] + [0.5; -1];
        % propagate
        global DELTA_T
        particles(:,i) = simulate(particles(:,i), control, DELTA_T);
    end
    % add a little more modeling noise
%     particles = particles + diag([.001, .001, deg2rad(1), .01, deg2rad(5)])*randn(size(particles));
end

function probability = particle_meas_func(particles, measurement)
    pred_meas = particles(1:3,:);
    error = pred_meas - ones(size(pred_meas)).*measurement;

    global MEAS_DIR_STD_DEV MEAS_LOC_STD_DEV
    sigma = [MEAS_DIR_STD_DEV; MEAS_DIR_STD_DEV; MEAS_LOC_STD_DEV];
    base = (sigma*sqrt(2*pi)).^2;
    probs = base .* exp(-.5*(error./sigma).^2);
   
    probability = probs(1,:).*probs(2,:).*probs(3,:);
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

function odom = state_to_odom(state)
    global CAR_LENGTH

    beta = atan(tan(state(5,:))/2);
    v_body = state(4,:)./cos(beta);
    curvature = 2*sin(beta)/CAR_LENGTH;
    odom = [
        state(1:3,:);
        v_body.*cos(beta);
        v_body.*sin(beta);
        v_body.*curvature
    ];
end