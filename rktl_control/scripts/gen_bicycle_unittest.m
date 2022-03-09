% Generate several bicycle model paths to verify accuracy of Python implementations

% physical properties of car
global CAR_LENGTH MAX_SPEED THROTTLE_TAU STEERING_THROW STEERING_RATE
CAR_LENGTH = .01;               % wheel to wheel dist, m
MAX_SPEED = 1;                  % max speed, m/s
THROTTLE_TAU = 0.25;            % time constant for changing speed, sec
STEERING_THROW = deg2rad(15);   % center-side steering throw, rad
STEERING_RATE = deg2rad(30);    % ROC steering, rad per sec

N_PARTICLES = 5;
DELTA_T = 0.1;

control = rand(N_PARTICLES, 2)*2 + -1
states = randn(N_PARTICLES, 5)
next_states = states;
for i = 1:N_PARTICLES
    next_states(i,:) = simulate(states(i,:), control(i,:), DELTA_T);
end
next_states

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